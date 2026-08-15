"""Deterministic mission executor (compiler).

    compile_mission(plan: MissionPlan) -> Tuple[ExecutableCommand, ...]

Guarantees:
  * PURE + DETERMINISTIC: no I/O, no clock, no randomness, no network, and —
    by construction — no LLM calls. Same MissionPlan in => byte-identical
    command sequence out, every time (see commands_digest()).
  * TYPE-GATED: accepts ONLY a `validator.MissionPlan`. Raw strings, dicts, or
    anything an LLM produced directly are rejected with TypeError. There is no
    JSON parsing anywhere in this layer.
  * Loop unrolling happens here: a LOOP(count=N, waypoints=[...]) becomes N
    explicit Goto sequences, so backends only ever see a flat, fixed path.
"""
from __future__ import annotations

import math
from typing import List, Tuple

from validator import (
    CaptureCmd,
    GimbalCmd,
    GotoCmd,
    GridCmd,
    HoldCmd,
    LandCmd,
    LoopCmd,
    MissionPlan,
    OrbitCmd,
    RecordCmd,
    RtlCmd,
    SpiralCmd,
    TakeoffCmd,
)

from .commands import (
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

# --------------------------------------------------------------------------- #
# Pattern expansion
#
# GRID / ORBIT / SPIRAL are declarative in the schema: the model states intent
# (area, spacing, radius) and the geometry is computed HERE. Two reasons:
#   * determinism - the same intent always yields byte-identical waypoints,
#     which a model hand-computing trigonometry can never guarantee,
#   * cost - emitting 40 waypoints of JSON burns tokens and invites the exact
#     malformed-JSON slips we kept hitting.
# All coordinates are LOCAL_ENU_METERS: x = East, y = North, alt = up.
# Rounded to 1 mm so float noise can never change a digest.
# --------------------------------------------------------------------------- #

_MM = 3          # round to millimetre


def _r(v: float) -> float:
    return round(v + 0.0, _MM)


def expand_grid(c: "GridCmd", speed: float) -> List[ExecutableCommand]:
    """Boustrophedon ("lawnmower") sweep of an axis-aligned area, optionally
    rotated by heading_deg. Lanes run along the width axis."""
    out: List[ExecutableCommand] = []
    lanes = max(1, int(math.floor(c.height / c.spacing)) + 1)
    half_w, half_h = c.width / 2.0, c.height / 2.0
    th = math.radians(c.heading_deg)
    cos_t, sin_t = math.cos(th), math.sin(th)

    for i in range(lanes):
        # lane offset from the south edge, clamped to the area
        oy = -half_h + min(i * c.spacing, c.height)
        ends = [(-half_w, oy), (half_w, oy)]
        if i % 2:                      # alternate direction => lawnmower
            ends.reverse()
        for ex, ey in ends:
            gx = c.center_x + ex * cos_t - ey * sin_t
            gy = c.center_y + ex * sin_t + ey * cos_t
            out.append(Goto(x=_r(gx), y=_r(gy), alt=c.alt, speed_mps=speed))
            if c.capture:
                out.append(Capture(label="grid"))
    return out


def expand_orbit(c: "OrbitCmd", speed: float) -> List[ExecutableCommand]:
    """Ring of waypoints around (x, y). Starts at due east and goes anticlockwise."""
    out: List[ExecutableCommand] = []
    n = c.points_per_turn
    for turn in range(c.turns):
        for k in range(n):
            a = 2.0 * math.pi * k / n
            out.append(Goto(x=_r(c.x + c.radius * math.cos(a)),
                            y=_r(c.y + c.radius * math.sin(a)),
                            alt=c.alt, speed_mps=speed))
            if c.capture:
                out.append(Capture(label="orbit"))
    # close the ring so the path ends where it began
    out.append(Goto(x=_r(c.x + c.radius), y=_r(c.y), alt=c.alt, speed_mps=speed))
    return out


def expand_spiral(c: "SpiralCmd", speed: float) -> List[ExecutableCommand]:
    """Archimedean spiral: radius grows linearly by `growth` per revolution."""
    out: List[ExecutableCommand] = []
    n = c.points_per_turn
    total = c.turns * n
    for k in range(total + 1):
        frac = k / n                                  # revolutions completed
        radius = c.start_radius + c.growth * frac
        if radius <= 0:
            continue
        a = 2.0 * math.pi * frac
        out.append(Goto(x=_r(c.x + radius * math.cos(a)),
                        y=_r(c.y + radius * math.sin(a)),
                        alt=c.alt, speed_mps=speed))
        if c.capture:
            out.append(Capture(label="spiral"))
    return out


def compile_mission(plan: MissionPlan) -> Tuple[ExecutableCommand, ...]:
    """Compile a validated MissionPlan into a flat executable command sequence."""
    if not isinstance(plan, MissionPlan):
        raise TypeError(
            "compile_mission() accepts only a validated validator.MissionPlan "
            f"(got {type(plan).__name__}). Raw LLM output / strings / dicts must go "
            "through validator.validate_mission() first — this is the safety boundary."
        )

    cruise = plan.cruise_speed_mps
    seq: List[ExecutableCommand] = []

    if plan.vehicle == "drone":
        seq.append(Arm())

    for cmd in plan.commands:
        if isinstance(cmd, TakeoffCmd):
            seq.append(Takeoff(alt=cmd.alt))
        elif isinstance(cmd, GotoCmd):
            seq.append(Goto(x=cmd.x, y=cmd.y, alt=cmd.alt,
                            speed_mps=cmd.speed_mps if cmd.speed_mps is not None else cruise))
        elif isinstance(cmd, LoopCmd):
            for _ in range(cmd.count):           # deterministic unroll
                for wp in cmd.waypoints:
                    seq.append(Goto(x=wp.x, y=wp.y, alt=wp.alt, speed_mps=cruise))
        elif isinstance(cmd, GridCmd):
            seq.extend(expand_grid(cmd, cmd.speed_mps or cruise))
        elif isinstance(cmd, OrbitCmd):
            seq.extend(expand_orbit(cmd, cmd.speed_mps or cruise))
        elif isinstance(cmd, SpiralCmd):
            seq.extend(expand_spiral(cmd, cmd.speed_mps or cruise))
        elif isinstance(cmd, CaptureCmd):
            seq.append(Capture(label=cmd.label))
        elif isinstance(cmd, RecordCmd):
            seq.append(Record(start=cmd.start))
        elif isinstance(cmd, GimbalCmd):
            seq.append(Gimbal(pitch_deg=cmd.pitch_deg, yaw_deg=cmd.yaw_deg))
        elif isinstance(cmd, HoldCmd):
            seq.append(Hold(seconds=cmd.seconds))
        elif isinstance(cmd, LandCmd):
            seq.append(Land())
            seq.append(Disarm())
        elif isinstance(cmd, RtlCmd):
            seq.append(ReturnToLaunch())
            seq.append(Disarm())
        else:  # pragma: no cover — MissionPlan can't contain anything else
            raise TypeError(f"unhandled mission command {type(cmd).__name__}")

    return tuple(seq)
