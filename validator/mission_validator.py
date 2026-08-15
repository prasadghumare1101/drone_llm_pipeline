"""Mission validator: the ONLY trust boundary between the LLM and the vehicle.

Input : untrusted raw string (or dict) — typically whatever the LLM produced.
Output: a frozen, typed `MissionPlan` — the only object the executor accepts.

Design rules:
  * Fail loudly and specifically: ALL problems are collected and reported in a
    single MissionValidationError, each with a JSON path.
  * Two enforcement layers:
      1. JSON Schema (schema/mission_schema.json): structure, types, command
         whitelist, additionalProperties=false everywhere.
      2. Semantic safety rules (this file + safety_limits.py): altitude/speed
         bounds, geofence, loop counts, unrolled waypoint budget, mission
         shape (drone must TAKEOFF first and end with LAND/RTL, etc.).
  * No side effects, no network, no vehicle I/O. Pure data in, pure data out.
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

from jsonschema import Draft7Validator
from jsonschema.exceptions import best_match

from .safety_limits import LIMITS_BY_VEHICLE, SafetyLimits

_SCHEMA_PATH = Path(__file__).resolve().parents[1] / "schema" / "mission_schema.json"
with _SCHEMA_PATH.open("r", encoding="utf-8") as _f:
    MISSION_SCHEMA: Dict[str, Any] = json.load(_f)
_SCHEMA_VALIDATOR = Draft7Validator(MISSION_SCHEMA)


# --------------------------------------------------------------------------- #
# Typed, immutable mission representation (the validated artifact)
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    alt: float


@dataclass(frozen=True)
class TakeoffCmd:
    alt: float


@dataclass(frozen=True)
class GotoCmd:
    x: float
    y: float
    alt: float
    speed_mps: Optional[float] = None  # None -> use mission cruise speed


@dataclass(frozen=True)
class LoopCmd:
    count: int
    waypoints: Tuple[Waypoint, ...]


@dataclass(frozen=True)
class HoldCmd:
    seconds: float


@dataclass(frozen=True)
class LandCmd:
    pass


@dataclass(frozen=True)
class RtlCmd:
    pass


# --- declarative patterns: geometry is computed by the executor ------------ #

@dataclass(frozen=True)
class GridCmd:
    center_x: float
    center_y: float
    width: float
    height: float
    spacing: float
    alt: float
    heading_deg: float = 0.0
    speed_mps: Optional[float] = None
    capture: bool = False

    @property
    def lanes(self) -> int:
        import math
        return max(1, int(math.floor(self.height / self.spacing)) + 1)

    @property
    def waypoint_count(self) -> int:
        return self.lanes * 2


@dataclass(frozen=True)
class OrbitCmd:
    x: float
    y: float
    alt: float
    radius: float
    turns: int = 1
    points_per_turn: int = 12
    speed_mps: Optional[float] = None
    capture: bool = False

    @property
    def waypoint_count(self) -> int:
        return self.turns * self.points_per_turn + 1


@dataclass(frozen=True)
class SpiralCmd:
    x: float
    y: float
    alt: float
    start_radius: float
    growth: float
    turns: int
    points_per_turn: int = 12
    speed_mps: Optional[float] = None
    capture: bool = False

    @property
    def waypoint_count(self) -> int:
        return self.turns * self.points_per_turn + 1

    @property
    def max_radius(self) -> float:
        return self.start_radius + self.growth * self.turns


# --- payload actions ------------------------------------------------------- #

@dataclass(frozen=True)
class CaptureCmd:
    label: str = ""


@dataclass(frozen=True)
class RecordCmd:
    start: bool


@dataclass(frozen=True)
class GimbalCmd:
    pitch_deg: float
    yaw_deg: float = 0.0


MissionCommand = Union[TakeoffCmd, GotoCmd, LoopCmd, HoldCmd, LandCmd, RtlCmd,
                       GridCmd, OrbitCmd, SpiralCmd,
                       CaptureCmd, RecordCmd, GimbalCmd]


@dataclass(frozen=True)
class MissionPlan:
    """Frozen validated mission. Instances are only created by validate_mission()."""
    schema_version: str
    mission_name: str
    vehicle: str
    frame: str
    cruise_speed_mps: float
    commands: Tuple[MissionCommand, ...]
    source_sha256: str  # hash of the canonicalised source JSON (traceability)


class MissionValidationError(ValueError):
    """Raised with a full, specific list of everything wrong with a mission.

    `stage` records WHICH gate refused, so callers can tell a recoverable
    formatting slip from a genuine safety refusal:
      "json"   - the model emitted malformed JSON (retrying usually fixes it)
      "schema" - well-formed JSON, wrong shape (retrying may fix it)
      "safety" - the plan is structurally valid but UNSAFE. Never retry this:
                 re-rolling until a limit check passes would launder exactly
                 the request the operator must be told was refused.
    """

    def __init__(self, errors: List[str], stage: str = "safety"):
        self.errors = list(errors)
        self.stage = stage
        bullet = "\n  - ".join(self.errors)
        super().__init__(f"Mission REJECTED ({len(self.errors)} problem(s)):\n  - {bullet}")


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #

def _json_path(err) -> str:
    parts = ["$"]
    for p in err.absolute_path:
        parts.append(f"[{p}]" if isinstance(p, int) else f".{p}")
    return "".join(parts)


def _schema_errors(data: Any) -> List[str]:
    msgs: List[str] = []
    for err in sorted(_SCHEMA_VALIDATOR.iter_errors(data), key=lambda e: list(e.absolute_path)):
        if err.context:  # oneOf failures are noisy; pick the closest sub-error
            bm = best_match(err.context)
            msgs.append(f"{_json_path(err)}: {bm.message}")
        else:
            msgs.append(f"{_json_path(err)}: {err.message}")
    return msgs


def _canonical_sha256(data: Dict[str, Any]) -> str:
    canon = json.dumps(data, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(canon).hexdigest()


# --------------------------------------------------------------------------- #
# Semantic safety rules (run only after schema validation passes)
# --------------------------------------------------------------------------- #

def _check_alt(alt: float, lim: SafetyLimits, where: str, errors: List[str], is_flight_alt: bool) -> None:
    if lim.max_alt_m == 0.0:  # ground robot
        if alt != 0.0:
            errors.append(f"{where}: ground_robot waypoints must have alt == 0 (got {alt})")
        return
    if is_flight_alt and alt < lim.min_alt_m:
        errors.append(f"{where}: altitude {alt} m below minimum {lim.min_alt_m} m")
    if alt > lim.max_alt_m:
        errors.append(f"{where}: altitude {alt} m exceeds hard ceiling {lim.max_alt_m} m")


def _check_xy(x: float, y: float, lim: SafetyLimits, where: str, errors: List[str]) -> None:
    g = lim.geofence_half_side_m
    if abs(x) > g or abs(y) > g:
        errors.append(f"{where}: waypoint ({x}, {y}) outside geofence (|x|,|y| <= {g} m)")


def _check_speed(spd: Optional[float], lim: SafetyLimits, where: str,
                 errors: List[str]) -> None:
    if spd is not None and spd > lim.max_speed_mps:
        errors.append(f"{where}: speed_mps {spd} exceeds max {lim.max_speed_mps} m/s")


def _semantic_errors(data: Dict[str, Any]) -> List[str]:
    errors: List[str] = []
    vehicle = data["vehicle"]
    lim = LIMITS_BY_VEHICLE[vehicle]
    total_caps = 0

    if data["cruise_speed_mps"] > lim.max_speed_mps:
        errors.append(
            f"$.cruise_speed_mps: {data['cruise_speed_mps']} m/s exceeds max "
            f"{lim.max_speed_mps} m/s for vehicle '{vehicle}'"
        )

    cmds = data["commands"]
    total_wps = 0

    for i, cmd in enumerate(cmds):
        where = f"$.commands[{i}] ({cmd['type']})"
        ctype = cmd["type"]

        if ctype not in lim.allowed_commands:
            errors.append(
                f"{where}: command not allowed for vehicle '{vehicle}' "
                f"(allowed: {sorted(lim.allowed_commands)})"
            )
            continue

        if ctype == "TAKEOFF":
            _check_alt(cmd["alt"], lim, where, errors, is_flight_alt=True)

        elif ctype == "GOTO":
            total_wps += 1
            _check_alt(cmd["alt"], lim, where, errors, is_flight_alt=(vehicle == "drone"))
            _check_xy(cmd["x"], cmd["y"], lim, where, errors)
            spd = cmd.get("speed_mps")
            if spd is not None and spd > lim.max_speed_mps:
                errors.append(f"{where}: speed_mps {spd} exceeds max {lim.max_speed_mps} m/s")

        elif ctype == "LOOP":
            if cmd["count"] > lim.max_loop_count:
                errors.append(f"{where}: loop count {cmd['count']} exceeds max {lim.max_loop_count}")
            total_wps += cmd["count"] * len(cmd["waypoints"])
            for j, wp in enumerate(cmd["waypoints"]):
                wwhere = f"{where}.waypoints[{j}]"
                _check_alt(wp["alt"], lim, wwhere, errors, is_flight_alt=(vehicle == "drone"))
                _check_xy(wp["x"], wp["y"], lim, wwhere, errors)

        elif ctype == "HOLD":
            if cmd["seconds"] > lim.max_hold_seconds:
                errors.append(f"{where}: hold {cmd['seconds']} s exceeds max {lim.max_hold_seconds} s")

        elif ctype == "GRID":
            _check_alt(cmd["alt"], lim, where, errors, is_flight_alt=(vehicle == "drone"))
            if cmd["spacing"] < lim.min_pattern_spacing_m:
                errors.append(f"{where}: spacing {cmd['spacing']} m below minimum "
                              f"{lim.min_pattern_spacing_m} m (would explode the waypoint count)")
            g = GridCmd(cmd["center_x"], cmd["center_y"], cmd["width"], cmd["height"],
                        max(cmd["spacing"], 1e-6), cmd["alt"])
            total_wps += g.waypoint_count
            if cmd.get("capture"):
                total_caps += g.waypoint_count
            # every corner of the (unrotated) footprint must sit inside the fence
            hw, hh = cmd["width"] / 2.0, cmd["height"] / 2.0
            for cx, cy in ((-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)):
                _check_xy(cmd["center_x"] + cx, cmd["center_y"] + cy, lim, where, errors)
            _check_speed(cmd.get("speed_mps"), lim, where, errors)

        elif ctype == "ORBIT":
            _check_alt(cmd["alt"], lim, where, errors, is_flight_alt=(vehicle == "drone"))
            if cmd["radius"] > lim.max_orbit_radius_m:
                errors.append(f"{where}: radius {cmd['radius']} m exceeds max "
                              f"{lim.max_orbit_radius_m} m")
            turns = cmd.get("turns", 1)
            if turns > lim.max_pattern_turns:
                errors.append(f"{where}: {turns} turns exceeds max {lim.max_pattern_turns}")
            o = OrbitCmd(cmd["x"], cmd["y"], cmd["alt"], cmd["radius"], turns,
                         cmd.get("points_per_turn", 12))
            total_wps += o.waypoint_count
            if cmd.get("capture"):
                total_caps += o.waypoint_count
            for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):   # ring extremes
                _check_xy(cmd["x"] + dx * cmd["radius"],
                          cmd["y"] + dy * cmd["radius"], lim, where, errors)
            _check_speed(cmd.get("speed_mps"), lim, where, errors)

        elif ctype == "SPIRAL":
            _check_alt(cmd["alt"], lim, where, errors, is_flight_alt=(vehicle == "drone"))
            turns = cmd["turns"]
            if turns > lim.max_pattern_turns:
                errors.append(f"{where}: {turns} turns exceeds max {lim.max_pattern_turns}")
            s = SpiralCmd(cmd["x"], cmd["y"], cmd["alt"], cmd["start_radius"],
                          cmd["growth"], turns, cmd.get("points_per_turn", 12))
            total_wps += s.waypoint_count
            if cmd.get("capture"):
                total_caps += s.waypoint_count
            if s.max_radius > lim.max_orbit_radius_m:
                errors.append(f"{where}: spiral grows to {s.max_radius:.1f} m radius, "
                              f"exceeds max {lim.max_orbit_radius_m} m")
            for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                _check_xy(cmd["x"] + dx * s.max_radius,
                          cmd["y"] + dy * s.max_radius, lim, where, errors)
            _check_speed(cmd.get("speed_mps"), lim, where, errors)

        elif ctype == "CAPTURE":
            total_caps += 1

        elif ctype == "GIMBAL":
            p = cmd["pitch_deg"]
            if not (lim.min_gimbal_pitch_deg <= p <= lim.max_gimbal_pitch_deg):
                errors.append(f"{where}: gimbal pitch {p}° outside allowed range "
                              f"[{lim.min_gimbal_pitch_deg}, {lim.max_gimbal_pitch_deg}]")

    if total_caps > lim.max_captures:
        errors.append(f"$.commands: mission would take {total_caps} captures, "
                      f"exceeds budget of {lim.max_captures}")

    if total_wps > lim.max_total_waypoints:
        errors.append(
            f"$.commands: mission unrolls to {total_wps} waypoints, "
            f"exceeds budget of {lim.max_total_waypoints}"
        )

    # Mission-shape rules. Payload actions (CAPTURE/RECORD/GIMBAL) are not
    # movement, so they are ignored when checking that a flight starts with a
    # climb and ends on the ground - otherwise a trailing CAPTURE would look
    # like an unterminated mission.
    PAYLOAD = {"CAPTURE", "RECORD", "GIMBAL"}
    types = [c["type"] for c in cmds]
    flight = [t for t in types if t not in PAYLOAD]
    if vehicle == "drone":
        if not flight:
            errors.append("$.commands: mission contains no flight commands")
        else:
            if flight[0] != "TAKEOFF":
                errors.append("$.commands[0]: drone mission must start with TAKEOFF")
            if flight.count("TAKEOFF") > 1:
                errors.append("$.commands: at most one TAKEOFF per mission")
            if flight[-1] not in ("LAND", "RTL"):
                errors.append("$.commands[-1]: drone mission must end with LAND or RTL")
            for i, t in enumerate(flight[:-1]):
                if t in ("LAND", "RTL"):
                    errors.append(f"$.commands: {t} only permitted as the final "
                                  "flight command")

    return errors


# --------------------------------------------------------------------------- #
# Structuring (dict -> frozen dataclasses); only reached when fully valid
# --------------------------------------------------------------------------- #

def _to_plan(data: Dict[str, Any]) -> MissionPlan:
    cmds: List[MissionCommand] = []
    for c in data["commands"]:
        t = c["type"]
        if t == "TAKEOFF":
            cmds.append(TakeoffCmd(alt=float(c["alt"])))
        elif t == "GOTO":
            spd = c.get("speed_mps")
            cmds.append(GotoCmd(x=float(c["x"]), y=float(c["y"]), alt=float(c["alt"]),
                                speed_mps=float(spd) if spd is not None else None))
        elif t == "LOOP":
            wps = tuple(Waypoint(float(w["x"]), float(w["y"]), float(w["alt"]))
                        for w in c["waypoints"])
            cmds.append(LoopCmd(count=int(c["count"]), waypoints=wps))
        elif t == "HOLD":
            cmds.append(HoldCmd(seconds=float(c["seconds"])))
        elif t == "GRID":
            cmds.append(GridCmd(
                center_x=float(c["center_x"]), center_y=float(c["center_y"]),
                width=float(c["width"]), height=float(c["height"]),
                spacing=float(c["spacing"]), alt=float(c["alt"]),
                heading_deg=float(c.get("heading_deg", 0.0)),
                speed_mps=float(c["speed_mps"]) if c.get("speed_mps") is not None else None,
                capture=bool(c.get("capture", False))))
        elif t == "ORBIT":
            cmds.append(OrbitCmd(
                x=float(c["x"]), y=float(c["y"]), alt=float(c["alt"]),
                radius=float(c["radius"]), turns=int(c.get("turns", 1)),
                points_per_turn=int(c.get("points_per_turn", 12)),
                speed_mps=float(c["speed_mps"]) if c.get("speed_mps") is not None else None,
                capture=bool(c.get("capture", False))))
        elif t == "SPIRAL":
            cmds.append(SpiralCmd(
                x=float(c["x"]), y=float(c["y"]), alt=float(c["alt"]),
                start_radius=float(c["start_radius"]), growth=float(c["growth"]),
                turns=int(c["turns"]),
                points_per_turn=int(c.get("points_per_turn", 12)),
                speed_mps=float(c["speed_mps"]) if c.get("speed_mps") is not None else None,
                capture=bool(c.get("capture", False))))
        elif t == "CAPTURE":
            cmds.append(CaptureCmd(label=str(c.get("label", ""))))
        elif t == "RECORD":
            cmds.append(RecordCmd(start=(c["action"] == "start")))
        elif t == "GIMBAL":
            cmds.append(GimbalCmd(pitch_deg=float(c["pitch_deg"]),
                                  yaw_deg=float(c.get("yaw_deg", 0.0))))
        elif t == "LAND":
            cmds.append(LandCmd())
        elif t == "RTL":
            cmds.append(RtlCmd())
        else:  # pragma: no cover — schema whitelist makes this unreachable
            raise MissionValidationError([f"unknown command type '{t}'"])
    return MissionPlan(
        schema_version=data["schema_version"],
        mission_name=data["mission_name"],
        vehicle=data["vehicle"],
        frame=data["frame"],
        cruise_speed_mps=float(data["cruise_speed_mps"]),
        commands=tuple(cmds),
        source_sha256=_canonical_sha256(data),
    )


# --------------------------------------------------------------------------- #
# Public API
# --------------------------------------------------------------------------- #

def validate_mission(raw: Union[str, bytes, Dict[str, Any]]) -> MissionPlan:
    """Validate untrusted mission JSON. Returns MissionPlan or raises
    MissionValidationError listing every specific problem found."""
    if isinstance(raw, (str, bytes)):
        try:
            data = json.loads(raw)
        except json.JSONDecodeError as e:
            raise MissionValidationError(
                [f"input is not valid JSON: {e.msg} at line {e.lineno} column {e.colno}"],
                stage="json",
            ) from e
    elif isinstance(raw, dict):
        data = raw
    else:
        raise MissionValidationError(
            [f"input must be a JSON string or dict, got {type(raw).__name__}"],
            stage="json",
        )

    if not isinstance(data, dict):
        raise MissionValidationError(
            [f"top-level JSON value must be an object, got {type(data).__name__}"],
            stage="json",
        )

    errors = _schema_errors(data)
    if errors:  # semantic checks assume schema-valid shape
        raise MissionValidationError(errors, stage="schema")

    errors = _semantic_errors(data)
    if errors:
        raise MissionValidationError(errors, stage="safety")

    return _to_plan(data)


def validate_mission_file(path: Union[str, Path]) -> MissionPlan:
    return validate_mission(Path(path).read_text(encoding="utf-8"))
