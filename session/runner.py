"""Walks one compiled interactive step on a VehicleLink.

Shared by every link, so the sim and PX4 behave identically from the session's
point of view. Cancellation is checked every poll, so a stop / land / return
home from the operator (or an RC override) takes effect within one poll period
- the runner never sits inside a blocking call.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional, Sequence

from executor import (
    Arm,
    Capture,
    Disarm,
    ExecutableCommand,
    Gimbal,
    Goto,
    Hold,
    Land,
    Record,
    ReturnToLaunch,
    Takeoff,
)
from sim_bridge.base_backend import MissionBackend
from sim_bridge.vehicle_link import VehicleLink

POLL_S = 0.05
# A waypoint nobody gets closer to is either "close enough" (GPS noise / wind
# holding the vehicle a few metres off - common outdoors) or genuinely blocked
# (terrain, obstacle, a mode that ignores our target). Either way the operator
# hears about it within seconds instead of the vehicle hovering silently until
# the 180 s command timeout.
NO_PROGRESS_S = 12.0          # vehicle-clock seconds without getting closer...
PROGRESS_M = 0.3              # ...by at least this much
CLOSE_ENOUGH_FACTOR = 3.0     # within this x the acceptance radius: accept and move on
COMMAND_TIMEOUT_S = 180.0      # vehicle-clock seconds per command
DISARM_WAIT_S = 30.0
TAKEOFF_CLIMB_MPS = 2.0


SETTLE_WINDOW_S = 0.6         # "hovering" = moved less than SETTLE_M over this window
SETTLE_M = 0.15
SETTLE_TIMEOUT_S = 8.0


def _settle(link: VehicleLink, wait) -> None:
    """A Goto is "reached" inside the acceptance radius, often while still
    moving at cruise speed. Before reporting "Done" (and before the next step
    resolves relative moves from this position) wait until the vehicle has
    actually braked to a hover, so it does not overshoot past the reported spot."""
    track: list = []

    def still() -> bool:
        s = link.state()
        now = link.now()
        track.append((now, s.x, s.y, s.alt))
        while track and now - track[0][0] > SETTLE_WINDOW_S:
            track.pop(0)
        if not track or now - track[0][0] < SETTLE_WINDOW_S * 0.8:
            return False
        _, x0, y0, z0 = track[0]
        return math.dist((x0, y0, z0), (s.x, s.y, s.alt)) < SETTLE_M
    try:
        wait(still, "settling into a hover", SETTLE_TIMEOUT_S)
    except _Stop as e:
        if e.reason != "failed":
            raise                 # stop / override still win; a slow settle does not


@dataclass
class StepOutcome:
    completed: bool
    detail: str
    next_index: int            # first command NOT finished (for "resume")
    reason: str = ""           # "", "cancelled", "rc_override", "failed"


class _Stop(Exception):
    def __init__(self, reason: str, detail: str):
        super().__init__(detail)
        self.reason = reason


def run_step(link: VehicleLink, commands: Sequence[ExecutableCommand],
             cancel: threading.Event,
             on_progress: Optional[Callable[[int, ExecutableCommand], None]] = None,
             log: Optional[Callable[[str], None]] = None) -> StepOutcome:
    """Execute `commands` in order. Never arms: an Arm in the sequence is a
    programming error (compile_step() does not emit one) and is refused."""
    MissionBackend._type_gate(commands)      # compiled commands only
    if any(isinstance(c, Arm) for c in commands):
        raise ValueError("interactive steps never arm - arming is the pilot's job")

    def wait(cond: Callable[[], bool], what: str, timeout: float = COMMAND_TIMEOUT_S) -> None:
        start = link.now()
        while True:
            if cancel.is_set():
                raise _Stop("cancelled", "interrupted by operator")
            st = link.state()
            if st.rc_override:
                raise _Stop("rc_override", "pilot took control (RC override)")
            if not st.connected:
                raise _Stop("failed", "lost vehicle telemetry")
            if cond():
                return
            if link.now() - start > timeout:
                raise _Stop("failed", f"timed out after {timeout:.0f} s on {what}")
            time.sleep(POLL_S)

    i = 0
    try:
        for i, cmd in enumerate(commands):
            if on_progress:
                on_progress(i, cmd)
            if isinstance(cmd, Takeoff):
                st = link.state()
                if not st.armed:
                    raise _Stop("failed", "vehicle is not armed (the pilot must arm it)")
                link.goto(st.x, st.y, cmd.alt, TAKEOFF_CLIMB_MPS)
                wait(lambda: abs(link.state().alt - cmd.alt) < link.alt_tolerance_m,
                     f"take-off to {cmd.alt:g} m")
            elif isinstance(cmd, Goto):
                link.goto(cmd.x, cmd.y, cmd.alt, cmd.speed_mps)
                track = {"best": float("inf"), "since": link.now(), "close": False}

                def reached(c=cmd, tr=track) -> bool:
                    s = link.state()
                    horiz = math.hypot(s.x - c.x, s.y - c.y)
                    vert = abs(s.alt - c.alt)
                    if horiz < link.accept_radius_m and vert < link.alt_tolerance_m:
                        return True
                    d = math.hypot(horiz, vert)
                    if d < tr["best"] - PROGRESS_M:
                        tr["best"], tr["since"] = d, link.now()
                    elif link.now() - tr["since"] > NO_PROGRESS_S:
                        if d < CLOSE_ENOUGH_FACTOR * link.accept_radius_m:
                            tr["close"] = True
                            return True
                        raise _Stop("failed",
                                    f"not getting any closer to waypoint ({c.x:g}, {c.y:g}, "
                                    f"{c.alt:g} m): stuck {horiz:.1f} m away horizontally and "
                                    f"{s.alt - c.alt:+.1f} m in height for {NO_PROGRESS_S:g} s "
                                    f"(mode {s.mode}) - blocked by terrain/obstacle, wind, or "
                                    "the autopilot not following the target")
                    return False
                wait(reached, f"waypoint ({cmd.x:g}, {cmd.y:g}, {cmd.alt:g})")
                if track["close"] and log:
                    log(f"waypoint ({cmd.x:g}, {cmd.y:g}) accepted within "
                        f"{CLOSE_ENOUGH_FACTOR * link.accept_radius_m:g} m after "
                        f"{NO_PROGRESS_S:g} s without progress (position noise/wind)")
            elif isinstance(cmd, Hold):
                link.hold()
                end = link.now() + cmd.seconds
                wait(lambda: link.now() >= end, f"hover {cmd.seconds:g} s", cmd.seconds + 5)
            elif isinstance(cmd, (Land, ReturnToLaunch)):
                link.land() if isinstance(cmd, Land) else link.rtl()
                wait(lambda: not link.state().in_air,
                     "landing" if isinstance(cmd, Land) else "return home")
            elif isinstance(cmd, Disarm):
                # PX4 disarms by itself after touchdown; just wait for it
                try:
                    wait(lambda: not link.state().armed, "auto-disarm", DISARM_WAIT_S)
                except _Stop as e:
                    if e.reason != "failed":
                        raise
            elif isinstance(cmd, (Capture, Record, Gimbal)):
                line = link.payload(cmd)
                if log:
                    log(line)
        if link.state().in_air and any(isinstance(c, (Goto, Takeoff)) for c in commands):
            _settle(link, wait)
    except _Stop as e:
        if e.reason == "failed":
            link.hold()          # a failed step leaves the vehicle hovering
        return StepOutcome(False, str(e), i, e.reason)
    return StepOutcome(True, "step complete", len(commands))
