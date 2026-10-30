"""Interactive flight session: one chat message = one step from the live pose.

Threads:
  * the caller's thread (HTTP request) runs handle_message() / command() /
    pilot_arm() and NEVER waits on the LLM - fast commands act on the vehicle
    right there, step requests are handed to a planning thread;
  * one planning thread per step request (LLM -> validate_step -> compile_step
    -> battery check);
  * one execution thread per step (session.runner.run_step);
  * the owner calls tick() a few times per second: RC override, landing
    completion, the pilot-arm gate and the 2-minute inactivity watchdog.

Every action that changes what the vehicle is doing bumps `_gen`. Planning and
execution threads capture `_gen` when they start and drop their result if it
has moved on, so a "stop" can never be overtaken by a plan that was still being
generated when it arrived.

Invariants:
  * the session never arms - a take-off step waits in AWAITING_ARM for the pilot;
  * while the pilot has RC control, the session sends the vehicle nothing;
  * a step is only executed after validate_step() accepted it and the battery
    check passed.
"""
from __future__ import annotations

import json
import math
import threading
import time
from enum import Enum
from typing import Callable, Dict, List, Optional, Sequence

from executor import (Disarm, ExecutableCommand, Goto, Land, ReturnToLaunch, Takeoff,
                      compile_step)
from llm_layer import LLMError
from sim_bridge.vehicle_link import VehicleLink, VehicleState
from validator import (DRONE_LIMITS, GridCmd, MissionValidationError, OrbitCmd, SafetyLimits,
                       SpiralCmd, VehiclePose, validate_step)

from validator.terrain import TerrainModel, check_clearance

from .battery import BatteryPolicy, check_step, estimate_seconds
from .commands import (FastCommand, parse_altitude_change, parse_fast_command,
                       parse_waypoint_command)
from .runner import StepOutcome, run_step

INACTIVITY_TIMEOUT_S = 120.0
ARM_TIMEOUT_S = 60.0
LLM_ATTEMPTS = 3
HISTORY_TURNS = 10
TRANSCRIPT_MAX = 200

Planner = Callable[[str, dict, list], str]   # (request, live state, history) -> raw JSON


class Phase(str, Enum):
    ON_GROUND = "ON_GROUND"
    PLANNING = "PLANNING"
    AWAITING_ARM = "AWAITING_ARM"
    EXECUTING = "EXECUTING"
    HOVERING = "HOVERING"
    RETURNING = "RETURNING"
    LANDING = "LANDING"
    RC_OVERRIDE = "RC_OVERRIDE"
    FAULT = "FAULT"


def _where(st: VehicleState) -> str:
    return f"({st.x:.1f}, {st.y:.1f}) at {st.alt:.1f} m"


def _batt(st: VehicleState) -> str:
    return "battery unknown" if st.battery_pct is None else f"battery {st.battery_pct:.0f}%"


class Session:
    def __init__(self, link: VehicleLink, planner: Planner, *,
                 fallback_planner: Optional[Planner] = None,
                 planner_label: str = "llm",
                 inactivity_timeout_s: float = INACTIVITY_TIMEOUT_S,
                 arm_timeout_s: float = ARM_TIMEOUT_S,
                 battery: BatteryPolicy = BatteryPolicy(),
                 limits: Optional[SafetyLimits] = None,
                 terrain: Optional[TerrainModel] = None,
                 min_clearance_m: float = 10.0,
                 rtl_alt_m: float = 30.0,
                 clock: Callable[[], float] = time.monotonic):
        self.link = link
        self._planner = planner
        self._fallback = fallback_planner
        self.planner_label = planner_label
        self.inactivity_timeout_s = inactivity_timeout_s
        self.arm_timeout_s = arm_timeout_s
        self.battery = battery
        self.limits = limits or DRONE_LIMITS      # site geofence etc. (validate_step)
        # Altitudes are above HOME; the ground is not flat. With a terrain model
        # every path is checked for clearance above the actual ground, including
        # the straight return home at the autopilot's RTL altitude.
        self.terrain = terrain
        self.min_clearance_m = min_clearance_m
        self.rtl_alt_m = rtl_alt_m
        self._clock = clock

        self._lock = threading.RLock()
        self._gen = 0
        self._cancel = threading.Event()
        self._interrupt = ""                 # what bumped _gen: hold | land | rtl | rc
        st = link.state()
        self.phase = Phase.HOVERING if st.in_air else Phase.ON_GROUND
        self._idle_since = clock()
        self._transcript: List[dict] = []
        self._history: List[dict] = []
        self._msg_id = 0
        self._step: Optional[dict] = None    # current / last step for the UI
        self._resume: Optional[dict] = None  # remainder of a step interrupted by stop
        self._pending: Optional[dict] = None # validated take-off step awaiting the pilot
        self._still = {"phase": None, "pos": None, "since": 0.0, "warned": False}
        self._fault_action = ""              # land / rtl commanded while in FAULT
        self._override_from: Optional[tuple] = None   # where we last had control
        self._say("system", f"Session started on '{link.name}' ({planner_label} planner). "
                            f"Vehicle {'airborne' if st.in_air else 'on the ground'}, {_batt(st)}. "
                            "Tell me what to do.")

    # ------------------------------------------------------------------ #
    # operator input
    # ------------------------------------------------------------------ #

    def handle_message(self, text: str) -> dict:
        text = (text or "").strip()
        if not text:
            return {"ok": False, "reply": "empty message"}
        with self._lock:
            self._say("operator", text)
            self._idle_since = self._clock()
        fast = parse_fast_command(text)
        if fast is not None:
            return self.command(fast, echo=False)
        wp = parse_waypoint_command(text)
        if wp is not None:
            (x, y), where = wp
            return self.add_waypoints([(x, y)], where, echo=False)
        alt = parse_altitude_change(text)
        if alt is not None and self.phase == Phase.EXECUTING:
            return self._change_altitude(*alt)
        if alt is not None:
            # hovering: same meaning, phrased the way every planner understands
            mode, v = alt
            text = (f"climb to {v:g} m" if mode == "to"
                    else f"{'climb' if v > 0 else 'descend'} {abs(v):g} m")
        return self._request_step(text)

    def _change_altitude(self, mode: str, value: float) -> dict:
        """Re-target the altitude of the step that is flying, without stopping
        it: the remaining legs fly at the new height. Deterministic, no LLM.
        Every resulting altitude is checked against the same limits as any
        validated step, and the battery check is re-run for the remainder."""
        with self._lock:
            s = self._step
            if self.phase != Phase.EXECUTING or not s or s["index"] < 0:
                return self._request_step(f"{'climb to' if mode == 'to' else 'change altitude by'} {value:g} m")
            st = self.link.state()
            lim = self.limits
            out, alts = [], []
            for c in s["commands"][s["index"]:]:
                if isinstance(c, (Goto, Takeoff)):
                    alt = value if mode == "to" else c.alt + value
                    alts.append(alt)
                    c = (Goto(x=c.x, y=c.y, alt=alt, speed_mps=c.speed_mps) if isinstance(c, Goto)
                         else Takeoff(alt=alt))
                out.append(c)
            if not alts:
                return self._reply("The rest of this step has no legs to change the height of.", ok=False)
            bad = [a for a in alts if not lim.min_alt_m <= a <= lim.max_alt_m]
            if bad:
                return self._reply(f"Can't: that puts the drone at {bad[0]:.1f} m, outside the "
                                   f"{lim.min_alt_m:g}-{lim.max_alt_m:g} m limit. Still flying "
                                   f"'{s['text']}' at its current height.", ok=False)
            verdict = check_step(out, (st.x, st.y, st.alt), st.battery_pct, self.battery)
            if not verdict.ok:
                return self._reply(f"Can't change height: {verdict.message}.", ok=False)
            ground = self._clearance(out, st)
            if not ground.ok:
                return self._reply(f"Can't change height: {ground.message}. Still flying "
                                   f"'{s['text']}' at its current height.", ok=False)
            what = f"to {value:g} m" if mode == "to" else f"by {value:+g} m (now {alts[0]:g} m)"
            return self._replace_rest(s, out, f"altitude change: rest of the step flies {what}",
                                      f"Changing altitude {what} for the rest of '{s['text']}' - "
                                      "continuing the mission.")

    def _replace_rest(self, s: dict, out: list, memo: str, reply: str) -> dict:
        """Caller holds the lock. Swap the remaining commands of the flying
        step for `out` (already validated + battery-checked) without a hover
        in between: the runner is cancelled and restarted on the new list."""
        text = s["text"]
        self._bump("edit")
        self._start_execution(text, out, self._gen)
        self._step["about"] = s.get("about", "")
        self._remember(f"(during '{text}')", memo)
        return self._reply(reply)

    # ------------------------------------------------------------------ #
    # waypoints drawn on the map (or "add waypoint at x, y" in chat)
    # ------------------------------------------------------------------ #

    def fly_path(self, points: Sequence[tuple], alt: Optional[float] = None,
                 circuit: bool = False, echo: bool = True) -> dict:
        """Fly a path drawn on the map. Goes through the SAME pipeline as a
        chat message - validate_step, battery check, pilot-arm gate, mid-flight
        redirect - with the drawn path standing in for the planner's answer."""
        pts = [(round(float(x), 2), round(float(y), 2)) for x, y in points]
        if not pts:
            return {"ok": False, "reply": "no waypoints"}
        st = self.link.state()
        a = float(alt) if alt is not None else (round(st.alt, 1) if st.in_air else 10.0)
        cmds: list = [] if st.in_air else [{"type": "TAKEOFF", "alt": a}]
        wps = [{"x": x, "y": y, "alt": a} for x, y in pts]
        if circuit and len(pts) >= 2:
            cmds.append({"type": "LOOP", "count": 1, "waypoints": wps + [wps[0]]})
        else:
            cmds += [{"type": "GOTO", **w} for w in wps]
        kind = "closed circuit" if circuit and len(pts) >= 2 else "path"
        label = f"map {kind} {len(pts)} points"
        raw = json.dumps({
            "schema_version": "1.0", "mission_name": label, "vehicle": "drone",
            "frame": "LOCAL_ENU_METERS", "cruise_speed_mps": 5.0,
            "notes": f"Flying the {kind} drawn on the map: {len(pts)} waypoint(s) at {a:g} m.",
            "commands": cmds})
        if echo:
            with self._lock:
                self._say("operator", f"[MAP] fly {kind} through {len(pts)} point(s) at {a:g} m")
                self._idle_since = self._clock()
        return self._request_step(label, raw_json=raw)

    def add_waypoints(self, points: Sequence[tuple], where: str = "end",
                      echo: bool = True) -> dict:
        """Add waypoints to the step that is flying: where="next" flies to them
        now and then carries on; where="end" appends them after the last leg.
        Hovering: just fly there. Every point is validated like any step."""
        with self._lock:
            if echo:
                self._say("operator", f"[MAP] add {len(points)} waypoint(s) "
                                      f"{'next' if where == 'next' else 'at the end'}")
            self._idle_since = self._clock()
            s = self._step
            executing = self.phase == Phase.EXECUTING and s and s["index"] >= 0
        if not executing:
            return self.fly_path(points, echo=False)
        with self._lock:
            st = self.link.state()
            rest = list(s["commands"][s["index"]:])
            leg = next((c for c in rest if isinstance(c, (Goto, Takeoff))), None)
            alt = leg.alt if leg is not None else st.alt
            speed = leg.speed_mps if isinstance(leg, Goto) else 5.0
            raw = json.dumps({
                "schema_version": "1.0", "mission_name": "added waypoints", "vehicle": "drone",
                "frame": "LOCAL_ENU_METERS", "cruise_speed_mps": speed,
                "commands": [{"type": "GOTO", "x": float(x), "y": float(y), "alt": alt}
                             for x, y in points]})
            try:                                       # same trust boundary as any step
                new = list(compile_step(validate_step(
                    raw, VehiclePose(st.x, st.y, st.alt, st.heading_deg, True), self.limits)))
            except MissionValidationError as e:
                return self._reply("Can't add that waypoint: " + "; ".join(e.errors[:2]), ok=False)
            if where == "next":
                out = new + rest
            else:                                      # before any landing at the end
                k = len(rest)
                while k > 0 and isinstance(rest[k - 1], (Land, ReturnToLaunch, Disarm)):
                    k -= 1
                out = rest[:k] + new + rest[k:]
            verdict = check_step(out, (st.x, st.y, st.alt), st.battery_pct, self.battery)
            if not verdict.ok:
                return self._reply(f"Can't add it: {verdict.message}. Still flying "
                                   f"'{s['text']}'.", ok=False)
            ground = self._clearance(out, st)
            if not ground.ok:
                return self._reply(f"Can't add it: {ground.message}. Still flying "
                                   f"'{s['text']}'.", ok=False)
            pts = ", ".join(f"({c.x:.0f}, {c.y:.0f})" for c in new)
            when = "next, then continuing" if where == "next" else "after the last leg of"
            return self._replace_rest(s, out, f"added waypoint(s) {pts} {when}",
                                      f"Added waypoint(s) {pts} at {alt:g} m - flying them {when} "
                                      f"'{s['text']}'. {verdict.message.capitalize()}.")

    def command(self, cmd: FastCommand, echo: bool = True) -> dict:
        """Instant command - never touches the LLM. Called for chat messages
        the fast parser recognised and for the dashboard buttons."""
        with self._lock:
            if echo:
                self._say("operator", f"[{cmd.value.upper()}]")
            self._idle_since = self._clock()
            st = self.link.state()

            if cmd == FastCommand.STATUS:
                return self._reply(self._status_line(st))

            if self.phase == Phase.RC_OVERRIDE:
                if cmd == FastCommand.RESUME:
                    self.link.release_override()
                    self.phase = Phase.HOVERING if st.in_air else Phase.ON_GROUND
                    self._idle_since = self._clock()
                    msg = (f"Control handed back. {self._moved_since_override(st)}Hovering at "
                           f"{_where(st)}, {_batt(st)}.")
                    self._override_from = None
                    if self._resume:
                        msg += " Say 'resume' again to continue the interrupted step."
                    return self._reply(msg + " What next?")
                return self._reply("The pilot has RC control - I am not sending any commands. "
                                   "Switch back to OFFBOARD or say 'resume' to hand control back.",
                                   ok=False)

            if self.phase == Phase.FAULT and cmd in (FastCommand.LAND, FastCommand.RTL,
                                                      FastCommand.HOLD):
                {FastCommand.LAND: self.link.land, FastCommand.RTL: self.link.rtl,
                 FastCommand.HOLD: self.link.hold}[cmd]()
                self._fault_action = cmd.value
                what = {"land": "Landing", "rtl": "Return home", "hold": "Hold"}[cmd.value]
                return self._reply(f"{what} commanded despite the fault - the vehicle may not "
                                   "respond; I'll warn you if it doesn't move.")
            if cmd == FastCommand.RESUME and self.phase == Phase.FAULT:
                return self._reply("Can't resume during a vehicle fault.", ok=False)
            if cmd == FastCommand.RESUME:
                if self._resume and st.in_air and self.phase == Phase.HOVERING:
                    rem = self._resume
                    verdict = check_step(rem["commands"], (st.x, st.y, st.alt),
                                         st.battery_pct, self.battery)
                    if not verdict.ok:
                        return self._reply(f"Can't resume: {verdict.message}.", ok=False)
                    ground = self._clearance(rem["commands"], st)
                    if not ground.ok:
                        return self._reply(f"Can't resume: {ground.message}.", ok=False)
                    self._resume = None
                    self._start_execution(rem["text"], rem["commands"], self._gen)
                    return self._reply(f"Resuming '{rem['text']}' ({len(rem['commands'])} "
                                       f"commands left). {verdict.message}.")
                return self._reply("Nothing to resume.", ok=False)

            # HOLD / LAND / RTL
            if not st.in_air:
                cancelled = self.phase in (Phase.PLANNING, Phase.AWAITING_ARM)
                self._bump("hold")
                self._pending = None
                self.phase = Phase.ON_GROUND
                return self._reply("Cancelled the pending step. On the ground." if cancelled
                                   else "Already on the ground.")

            if cmd == FastCommand.HOLD and self.phase == Phase.EXECUTING and self._step \
                    and self._step["index"] >= 0:
                # record what is left NOW, so "resume" works the instant we reply
                s = self._step
                self._resume = {"text": s["text"], "commands": s["commands"][s["index"]:]}
            self._bump(cmd.value)
            self._pending = None
            if cmd == FastCommand.HOLD:
                self.link.hold()
                self.phase = Phase.HOVERING
                self._idle_since = self._clock()
                return self._reply(f"Holding position at {_where(st)}, {_batt(st)}. What next?")
            self._resume = None
            if cmd == FastCommand.LAND:
                self.link.land()
                self.phase = Phase.LANDING
                return self._reply(f"Landing now at ({st.x:.1f}, {st.y:.1f}).")
            self.link.rtl()
            self.phase = Phase.RETURNING
            return self._reply(f"Returning home from {_where(st)}.")

    def pilot_arm(self) -> dict:
        """The pilot's ARM action (dashboard button). The only place arming
        is ever requested, and it is a human doing it."""
        with self._lock:
            if self.phase == Phase.RC_OVERRIDE:
                return self._reply("Pilot has RC control - arm from the transmitter.", ok=False)
            self._say("operator", "[PILOT ARM]")
            self._idle_since = self._clock()
            # decided BEFORE arming: tick() may launch the step (and leave
            # AWAITING_ARM) the instant PX4 reports armed
            launching = self.phase == Phase.AWAITING_ARM
        try:
            self.link.arm()
        except RuntimeError as e:
            with self._lock:
                return self._reply(f"Arming failed: {e}", ok=False)
        with self._lock:
            if launching:
                return {"ok": True, "reply": "Armed - launching the planned step."}
            return self._reply("Armed. PX4 disarms again after ~10 s on the ground "
                               "without a take-off - tell me what to do.")

    # ------------------------------------------------------------------ #
    # periodic supervision
    # ------------------------------------------------------------------ #

    def tick(self) -> None:
        st = self.link.state()
        now = self._clock()
        with self._lock:
            if st.fault and (st.armed or st.in_air) and self.phase != Phase.FAULT:
                if self._override_from is None:        # where we last had control
                    self._override_from = (st.x, st.y, st.alt)
                self._fault_action = ""
                self._bump("fault")
                self._pending = None
                self.phase = Phase.FAULT
                self._say("drone", f"VEHICLE FAULT reported by the autopilot: {st.fault}, at "
                                   f"{_where(st)}. I've stopped flying steps. If it is still "
                                   "flying, take RC control, or say 'land' / 'return home'.")
                return
            if self.phase == Phase.FAULT:
                self._watch_progress(st, now)
                if not st.fault:
                    if st.rc_override:                 # e.g. PX4's failsafe still flying it
                        self.phase = Phase.RC_OVERRIDE
                        self._say("drone", f"Fault cleared, but the autopilot (mode {st.mode}) "
                                           "still has control - waiting for it to hand back.")
                        return
                    self.phase = Phase.HOVERING if st.in_air else Phase.ON_GROUND
                    self._idle_since = now
                    self._say("drone", f"Fault cleared. {self._moved_since_override(st)}"
                                       f"{self._status_line(st)}")
                    self._override_from = None
                return
            self._watch_progress(st, now)
            if st.rc_override and self.phase != Phase.RC_OVERRIDE:
                if self._override_from is None:
                    self._override_from = (st.x, st.y, st.alt)
                self._bump("rc")
                self._pending = None
                self.phase = Phase.RC_OVERRIDE
                if st.mode == "FAILSAFE":
                    self._say("drone", "AUTOPILOT FAILSAFE - PX4 took control itself (sensor, "
                                       "battery or link failsafe). AI commands are paused; watch "
                                       "the vehicle and take RC control if needed. Say 'resume' "
                                       "only once the cause is cleared.")
                else:
                    self._say("drone", f"RC OVERRIDE - the pilot has control (mode {st.mode}). "
                                       "AI commands are paused and I won't send the vehicle "
                                       "anything. Switch back to OFFBOARD or say 'resume' to "
                                       "hand control back.")
                return
            if self.phase == Phase.RC_OVERRIDE:
                if not st.rc_override:
                    self.phase = Phase.HOVERING if st.in_air else Phase.ON_GROUND
                    self._idle_since = now
                    self._say("drone", f"Control is back with me. {self._moved_since_override(st)}"
                                       f"{self._status_line(st)} What next?" if st.in_air else
                                       "The vehicle landed while out of my control. On the ground.")
                    self._override_from = None
                return

            if self.phase in (Phase.LANDING, Phase.RETURNING) and not st.in_air and not st.armed:
                self.phase = Phase.ON_GROUND
                self._say("drone", f"Landed at ({st.x:.1f}, {st.y:.1f}) and disarmed. "
                                   f"{_batt(st).capitalize()}. What next?")
                return

            if self.phase == Phase.AWAITING_ARM and self._pending:
                if st.armed:
                    p, self._pending = self._pending, None
                    self._say("drone", "Pilot armed - launching.")
                    self._start_execution(p["text"], p["commands"], self._gen)
                elif now > p_deadline(self._pending):
                    self._pending = None
                    self.phase = Phase.ON_GROUND
                    self._say("drone", f"The pilot did not arm within {self.arm_timeout_s:.0f} s "
                                       "- take-off cancelled. Tell me again when ready.")
                return

            if self.phase == Phase.HOVERING:
                if not st.in_air and not st.armed:
                    self.phase = Phase.ON_GROUND
                    self._say("drone", "The vehicle is on the ground (landed or disarmed "
                                       "outside my control).")
                elif st.in_air and now - self._idle_since > self.inactivity_timeout_s:
                    self._bump("rtl")
                    self._resume = None
                    self.link.rtl()
                    self.phase = Phase.RETURNING
                    t = self.inactivity_timeout_s
                    span = f"{t / 60:.0f} minutes" if t >= 120 else f"{t:.0f} s"
                    self._say("drone", f"No reply for {span} - returning home from "
                                       f"{_where(st)}.")
            elif self.phase == Phase.ON_GROUND and st.in_air:
                self.phase = Phase.HOVERING
                self._idle_since = now

    # ------------------------------------------------------------------ #
    # planning
    # ------------------------------------------------------------------ #

    def _request_step(self, text: str, raw_json: Optional[str] = None) -> dict:
        with self._lock:
            st = self.link.state()
            # A new instruction while flying REPLACES what the drone is doing:
            # break off, hover where it is, and plan from that point.
            redirect = self.phase in (Phase.EXECUTING, Phase.RETURNING) or (
                self.phase == Phase.LANDING and st.in_air)
            busy = {
                Phase.RC_OVERRIDE: "The pilot has RC control - I can't fly steps until "
                                   "control is handed back ('resume').",
                Phase.PLANNING: "Still planning your previous message - one moment.",
                Phase.FAULT: "The autopilot reports a vehicle fault - no new steps. Say "
                             "'land' or 'return home', or take RC control.",
                Phase.AWAITING_ARM: "Waiting for the pilot to arm for the planned take-off. "
                                    "Say 'cancel' to drop it.",
            }.get(self.phase)
            if busy:
                return self._reply(busy, ok=False)
            if not st.connected:
                return self._reply("No telemetry from the vehicle - can't plan from an unknown "
                                   "position.", ok=False)
            if st.battery_pct is None or st.battery_pct < self.battery.min_start_pct:
                return self._reply(f"Refused: {_batt(st)} - below the "
                                   f"{self.battery.min_start_pct:.0f}% needed to start a step."
                                   + (" Say 'return home' or 'land'." if st.in_air else ""),
                                   ok=False)
            if redirect:
                if self.phase == Phase.EXECUTING and self._step and self._step["index"] >= 0:
                    cur = self._step
                    self._resume = {"text": cur["text"],
                                    "commands": cur["commands"][cur["index"]:]}
                was = {Phase.EXECUTING: "the current step", Phase.RETURNING: "returning home",
                       Phase.LANDING: "landing"}[self.phase]
                self._bump("redirect")
                self._pending = None
                self.link.hold()
                self._say("drone", f"New instruction - breaking off {was} to hover at "
                                   f"{_where(st)} and re-plan from there.")
            self.phase = Phase.PLANNING
            gen = self._gen
        threading.Thread(target=self._plan, args=(text, gen, redirect, raw_json), daemon=True,
                         name="session-plan").start()
        return {"ok": True, "reply": "Planning...", "planning": True}

    def _plan(self, text: str, gen: int, settle: bool = False,
              raw_json: Optional[str] = None) -> None:
        """raw_json: a step built without the planner (a path drawn on the map);
        it is validated exactly like a planner's answer."""
        if settle:
            self._wait_until_hovering(gen)
        st = self.link.state()
        pose = VehiclePose(st.x, st.y, st.alt, st.heading_deg, st.in_air)
        live = {
            "position": {"x": st.x, "y": st.y, "alt": st.alt},
            "heading_deg": st.heading_deg, "armed": st.armed, "in_air": st.in_air,
            "battery_pct": st.battery_pct, "home": {"x": 0.0, "y": 0.0, "alt": 0.0},
            **self._terrain_brief(st),
            "limits": {"geofence_half_side_m": self.limits.geofence_half_side_m,
                       "min_alt_m": self.limits.min_alt_m, "max_alt_m": self.limits.max_alt_m,
                       "max_speed_mps": self.limits.max_speed_mps},
        }
        with self._lock:
            history = list(self._history[-HISTORY_TURNS:])

        note = ""
        planner = self._planner if raw_json is None else (lambda *_: raw_json)
        error = ""
        plan = raw = None
        for attempt in range(LLM_ATTEMPTS if raw_json is None else 1):
            try:
                raw = planner(text, live, history)
            except LLMError as e:
                if self._fallback is not None and planner is not self._fallback:
                    note = f" (LLM unavailable: {str(e)[:80]}; used the offline parser)"
                    planner = self._fallback
                    continue
                error = f"Couldn't plan that: {e}"
                break
            try:
                plan = validate_step(raw, pose, self.limits)
                break
            except MissionValidationError as e:
                if e.stage in ("json", "schema") and attempt < LLM_ATTEMPTS - 1:
                    continue           # formatting slip: re-ask
                error = ("Refused by the safety validator: " + "; ".join(e.errors[:3])
                         if e.stage == "safety" else
                         "The planner kept producing invalid output: " + e.errors[0])
                break

        with self._lock:
            if gen != self._gen:
                return                 # a stop/land/home arrived meanwhile - drop it
            if plan is None:
                self.phase = Phase.HOVERING if st.in_air else Phase.ON_GROUND
                self._idle_since = self._clock()
                self._say("drone", f"{error or 'Planning failed.'} Still "
                                   f"{'hovering at ' + _where(st) if st.in_air else 'on the ground'}"
                                   ". What next?")
                return
            cmds = compile_step(plan)
            notes = ""
            try:
                notes = str(json.loads(raw).get("notes", ""))[:400]
            except (ValueError, AttributeError):
                pass
            summary = (notes or f"{len(cmds)} commands") + note
            verdict = check_step(cmds, (st.x, st.y, st.alt), st.battery_pct, self.battery)
            if not verdict.ok:
                self.phase = Phase.HOVERING if st.in_air else Phase.ON_GROUND
                self._idle_since = self._clock()
                # say what was planned too, so a reduced request is never hidden
                self._say("drone", f"Planned: {summary} - REFUSED: {verdict.message}. "
                                   + ("Ask for something shorter, or say 'return home' or "
                                      "'land'." if st.in_air else "Ask for something shorter."))
                return
            ground = self._clearance(cmds, st)
            if not ground.ok:
                self.phase = Phase.HOVERING if st.in_air else Phase.ON_GROUND
                self._idle_since = self._clock()
                self._say("drone", f"Planned: {summary} - REFUSED (terrain): {ground.message}.")
                return
            self._step = {"text": text, "commands": list(cmds), "index": -1,
                          "start": (st.x, st.y, st.alt), "battery": st.battery_pct,
                          "about": _describe_patterns(plan.commands)}
            now_st = self.link.state()
            if not now_st.in_air and not now_st.armed:
                self._pending = {"text": text, "commands": cmds,
                                 "deadline": self._clock() + self.arm_timeout_s}
                self.phase = Phase.AWAITING_ARM
                self._say("drone", f"{summary} {verdict.message.capitalize()}. Ready to take "
                                   "off - PILOT: arm the vehicle to launch (I never arm it "
                                   f"myself). Waiting {self.arm_timeout_s:.0f} s.")
                return
            self._say("drone", f"{summary} {verdict.message.capitalize()}.")
            self._start_execution(text, cmds, gen)

    def _wait_until_hovering(self, gen: int, timeout_s: float = 5.0) -> None:
        """After breaking off mid-leg the vehicle is still braking. Plan from
        where it actually stops, so relative moves resolve from the hover point."""
        prev = None
        end = time.monotonic() + timeout_s
        while time.monotonic() < end and gen == self._gen:
            st = self.link.state()
            cur = (st.x, st.y, st.alt)
            if prev is not None and math.dist(prev, cur) < 0.15:
                return
            prev = cur
            time.sleep(0.4)

    # ------------------------------------------------------------------ #
    # execution
    # ------------------------------------------------------------------ #

    def _start_execution(self, text: str, cmds: Sequence[ExecutableCommand], gen: int) -> None:
        """Caller holds the lock."""
        st = self.link.state()
        self._cancel = threading.Event()
        self.phase = Phase.EXECUTING
        if not self._step or self._step["commands"] != list(cmds):
            self._step = {"text": text, "commands": list(cmds), "index": -1,
                          "start": (st.x, st.y, st.alt), "battery": st.battery_pct}
        threading.Thread(target=self._execute, args=(text, list(cmds), gen, self._cancel),
                         daemon=True, name="session-exec").start()

    def _execute(self, text: str, cmds: List[ExecutableCommand], gen: int,
                 cancel: threading.Event) -> None:
        def progress(i: int, _cmd) -> None:
            with self._lock:
                if self._step is not None and gen == self._gen:
                    self._step["index"] = i

        def log(line: str) -> None:
            with self._lock:
                self._say("system", line)

        outcome: StepOutcome = run_step(self.link, cmds, cancel, progress, log)
        st = self.link.state()
        with self._lock:
            # only this execution's own step: a newer one (e.g. after an
            # altitude change) may already be flying
            if self._step is not None and gen == self._gen:
                self._step["index"] = -1
            remaining = cmds[outcome.next_index:]
            if gen != self._gen or outcome.reason in ("cancelled", "rc_override"):
                # the runner can notice an RC override before tick() does
                why = self._interrupt if gen != self._gen else outcome.reason
                if why in ("rc", "rc_override") and remaining:
                    self._resume = {"text": text, "commands": remaining}
                if why != "altitude":         # the step carries on at the new height
                    self._remember(text, f"interrupted ({why}){self._about()} at {_where(st)}")
                return
            if outcome.completed:
                ended_down = any(isinstance(c, (Land, ReturnToLaunch)) for c in cmds)
                if ended_down:
                    self.phase = Phase.ON_GROUND
                    self._say("drone", f"Done - landed at ({st.x:.1f}, {st.y:.1f}). "
                                       f"{_batt(st).capitalize()}. What next?")
                    self._remember(text, f"completed{self._about()}; landed at "
                                         f"({st.x:.1f}, {st.y:.1f})")
                else:
                    self.phase = Phase.HOVERING
                    self._idle_since = self._clock()
                    self._say("drone", f"Done. Hovering at {_where(st)}, {_batt(st)}. What next?")
                    self._remember(text, f"completed{self._about()}; now hovering at "
                                         f"{_where(st)}")
                return
            self.phase = Phase.HOVERING if st.in_air else Phase.ON_GROUND
            self._idle_since = self._clock()
            self._say("drone", f"Step failed: {outcome.detail}. "
                               + (f"Holding at {_where(st)}. What next?" if st.in_air else ""))
            self._remember(text, f"failed: {outcome.detail}")

    # ------------------------------------------------------------------ #
    # helpers / state for the UI
    # ------------------------------------------------------------------ #

    def _bump(self, why: str) -> None:
        """Caller holds the lock. Invalidate any plan in flight and stop any step."""
        self._gen += 1
        self._interrupt = why
        self._cancel.set()

    def _say(self, role: str, text: str) -> None:
        self._msg_id += 1
        self._transcript.append({"id": self._msg_id, "role": role, "text": text,
                                 "t": round(time.time(), 2)})
        del self._transcript[:-TRANSCRIPT_MAX]

    def _reply(self, text: str, ok: bool = True) -> dict:
        self._say("drone", text)
        return {"ok": ok, "reply": text}

    def _remember(self, operator: str, result: str) -> None:
        self._history.append({"operator": operator, "result": result})
        del self._history[:-HISTORY_TURNS]

    STILL_S = 20.0

    def _watch_progress(self, st: VehicleState, now: float) -> None:
        """Caller holds the lock. A return home / landing that does not move
        is reported once, instead of the chat saying "Returning home" forever."""
        w = self._still
        moving_phase = st.in_air and (
            self.phase in (Phase.RETURNING, Phase.LANDING)
            or (self.phase == Phase.FAULT and self._fault_action in ("rtl", "land")))
        pos = (st.x, st.y, st.alt)
        if not moving_phase or w["phase"] != self.phase or w["pos"] is None \
                or math.dist(pos, w["pos"]) > 0.5:
            w.update(phase=self.phase, pos=pos, since=now, warned=False)
            return
        if not w["warned"] and now - w["since"] > self.STILL_S:
            w["warned"] = True
            what = ("Return home" if Phase.RETURNING == self.phase or self._fault_action == "rtl"
                    else "Landing")
            self._say("drone", f"WARNING: {what} is not progressing - still at {_where(st)} after "
                               f"{self.STILL_S:.0f} s (mode {st.mode}). The vehicle may be stuck "
                               "or damaged: take RC control if you can.")

    def _moved_since_override(self, st: VehicleState) -> str:
        """While someone else had the vehicle it may have moved a lot (a GPS
        failsafe descends it); say so before asking what next."""
        o = self._override_from
        if not o:
            return ""
        d, dz = math.hypot(st.x - o[0], st.y - o[1]), st.alt - o[2]
        if d < 2.0 and abs(dz) < 2.0:
            return ""
        return (f"WHILE OUT OF MY CONTROL it moved {d:.0f} m and "
                f"{'climbed' if dz > 0 else 'descended'} {abs(dz):.0f} m. ")

    def note(self, text: str) -> dict:
        with self._lock:
            self._say("system", text[:300])
        return {"ok": True}

    def _clearance(self, cmds, st: VehicleState):
        return check_clearance(cmds, (st.x, st.y, st.alt), self.terrain,
                               self.min_clearance_m, self.rtl_alt_m)

    def _terrain_brief(self, st: VehicleState) -> dict:
        if self.terrain is None:
            return {}
        here = self.terrain.height(st.x, st.y)
        return {"terrain": {
            "ground_here_m": round(here, 1),
            "min_clearance_m": self.min_clearance_m,
            "note": "ground heights are relative to HOME, like every altitude; the ground is not "
                    "flat. Plan the altitude the operator asks for; the validator samples every "
                    "leg against the real ground and, if one is too low, tells the operator the "
                    "exact minimum safe altitude"}}

    def _about(self) -> str:
        a = (self._step or {}).get("about", "")
        return f" ({a})" if a else ""

    def _status_line(self, st: VehicleState) -> str:
        where = _where(st) if st.in_air else f"on the ground at ({st.x:.1f}, {st.y:.1f})"
        return (f"Status: {self.phase.value.lower().replace('_', ' ')}, {where}, heading "
                f"{st.heading_deg:.0f} deg, {_batt(st)}, {'armed' if st.armed else 'disarmed'}, "
                f"mode {st.mode}.")

    def snapshot(self, since_id: int = 0) -> dict:
        st = self.link.state()
        with self._lock:
            idle_left = None
            if self.phase == Phase.HOVERING and st.in_air:
                idle_left = max(0.0, self.inactivity_timeout_s - (self._clock() - self._idle_since))
            arm_left = None
            if self.phase == Phase.AWAITING_ARM and self._pending:
                arm_left = max(0.0, p_deadline(self._pending) - self._clock())
            return {
                "active": True,
                "link": self.link.name,
                "planner": self.planner_label,
                "phase": self.phase.value,
                "vehicle": st.__dict__,
                "watchdog_s": None if idle_left is None else round(idle_left, 1),
                "arm_wait_s": None if arm_left is None else round(arm_left, 1),
                "can_resume": bool(self._resume),
                "step": self._step_view(),
                "transcript": [m for m in self._transcript if m["id"] > since_id],
                "history": list(self._history),
            }

    def _step_view(self) -> Optional[dict]:
        s = self._step
        if not s:
            return None
        drain = self.battery.drain_pct_per_min
        pos = s["start"]
        batt = s["battery"]
        cum = 0.0
        points = []
        kinds = {"Takeoff": "Take off", "Goto": "Fly to"}
        for i, c in enumerate(s["commands"]):
            if not isinstance(c, (Takeoff, Goto)):
                continue
            secs, pos, _ = estimate_seconds([c], pos, self.battery)
            cum += secs
            points.append({
                "cmd_index": i, "x": pos[0], "y": pos[1], "alt": pos[2],
                "speed": round((c.speed_mps if isinstance(c, Goto) else 2.0) * 3.6, 1),
                "action": kinds[type(c).__name__],
                "energy": None if batt is None else int(round(batt - cum / 60.0 * drain)),
                "active": i == s["index"],
            })
        for n, p in enumerate(points, 1):
            p["idx"], p["total"] = n, len(points)
        return {"text": s["text"], "index": s["index"], "total": len(s["commands"]),
                "waypoints": points}


def _describe_patterns(commands) -> str:
    """Where the step's patterns are, in words the planner can reuse later
    ("patrol that area", "orbit the field again")."""
    out = []
    for c in commands:
        if isinstance(c, GridCmd):
            out.append(f"survey area centred ({c.center_x:.1f}, {c.center_y:.1f}), "
                       f"{c.width:g} x {c.height:g} m at {c.alt:g} m")
        elif isinstance(c, OrbitCmd):
            out.append(f"orbit centred ({c.x:.1f}, {c.y:.1f}), radius {c.radius:g} m")
        elif isinstance(c, SpiralCmd):
            out.append(f"spiral centred ({c.x:.1f}, {c.y:.1f})")
    return "; ".join(out)


def p_deadline(pending: Dict) -> float:
    return pending["deadline"]
