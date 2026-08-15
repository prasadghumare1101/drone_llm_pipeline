"""Numeric safety bounds enforced by the validator.

Assumptions (documented defaults, tune per site/regulations):
  - Drone limits sized for a small quad under Indian DGCA micro-class style
    constraints: hard ceiling 60 m AGL (raised from 30 m so SuperPoint +
    LightGlue geo-localizer missions — which acquire lock only above ~50 m —
    are flyable; still well under the 120 m legal cap), 12 m/s max horizontal
    speed, 200 m square geofence around home.
  - Ground robot limits sized for a TurtleBot3-class platform.
All units: metres, metres/second, seconds.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import FrozenSet, Mapping


@dataclass(frozen=True)
class SafetyLimits:
    min_alt_m: float          # minimum commanded flight altitude (drone only)
    max_alt_m: float          # hard altitude ceiling
    max_speed_mps: float      # cap on cruise_speed_mps and per-GOTO speed_mps
    geofence_half_side_m: float   # |x| and |y| of every waypoint must be <= this
    max_loop_count: int
    max_total_waypoints: int  # after unrolling loops AND expanding patterns
    max_hold_seconds: float
    allowed_commands: FrozenSet[str]
    # --- pattern + payload bounds ----------------------------------------- #
    # Patterns are expanded by the executor, so their expanded size is checked
    # BEFORE anything runs: an unbounded GRID/SPIRAL is the easiest way to blow
    # the waypoint budget or fill the disk with captures.
    min_pattern_spacing_m: float = 2.0    # floor on GRID spacing / SPIRAL growth
    max_pattern_turns: int = 20           # ORBIT / SPIRAL revolutions
    max_orbit_radius_m: float = 200.0
    max_captures: int = 500               # total stills per mission
    min_gimbal_pitch_deg: float = -90.0   # straight down
    max_gimbal_pitch_deg: float = 30.0    # slightly above horizon


DRONE_LIMITS = SafetyLimits(
    min_alt_m=2.0,
    max_alt_m=60.0,
    max_speed_mps=12.0,
    geofence_half_side_m=200.0,
    max_loop_count=10,
    max_total_waypoints=500,
    max_hold_seconds=300.0,
    allowed_commands=frozenset({
        "TAKEOFF", "GOTO", "LOOP", "HOLD", "LAND", "RTL",
        "GRID", "ORBIT", "SPIRAL", "CAPTURE", "RECORD", "GIMBAL",
    }),
)

GROUND_ROBOT_LIMITS = SafetyLimits(
    min_alt_m=0.0,
    max_alt_m=0.0,          # ground robot waypoints must have alt == 0
    max_speed_mps=1.5,
    geofence_half_side_m=100.0,
    max_loop_count=10,
    max_total_waypoints=500,
    max_hold_seconds=300.0,
    allowed_commands=frozenset({"GOTO", "LOOP", "HOLD"}),
)

LIMITS_BY_VEHICLE: Mapping[str, SafetyLimits] = {
    "drone": DRONE_LIMITS,
    "ground_robot": GROUND_ROBOT_LIMITS,
}
