"""Pre-step battery check.

Before every step the session estimates the energy the step needs PLUS the
energy to get home and land from where the step ends, and refuses the step if
the vehicle would finish below the reserve. Battery unknown counts as a
refusal: an unread gauge is not a full one.

Model (documented assumption - calibrate per airframe): constant drain while
airborne, flight time from straight-line legs at the commanded speeds.
PX4's own low-battery failsafe (BAT_* / COM_LOW_BAT_ACT) remains the backstop.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from executor import ExecutableCommand, Goto, Hold, Land, ReturnToLaunch, Takeoff


@dataclass(frozen=True)
class BatteryPolicy:
    drain_pct_per_min: float = 2.5   # airborne consumption
    reserve_pct: float = 20.0        # must remain after the step AND the trip home
    min_start_pct: float = 30.0      # never start a step below this
    climb_mps: float = 2.0
    descent_mps: float = 1.0
    home_speed_mps: float = 5.0


@dataclass(frozen=True)
class BatteryVerdict:
    ok: bool
    needed_pct: float
    message: str


def estimate_seconds(commands: Sequence[ExecutableCommand],
                     start: Tuple[float, float, float],
                     policy: BatteryPolicy) -> Tuple[float, Tuple[float, float, float], bool]:
    """(seconds airborne for the step, end position, ends on the ground)."""
    x, y, alt = start
    t = 0.0
    on_ground = False
    for c in commands:
        if isinstance(c, Takeoff):
            t += abs(c.alt - alt) / policy.climb_mps
            alt = c.alt
        elif isinstance(c, Goto):
            horiz = math.hypot(c.x - x, c.y - y)
            vert = abs(c.alt - alt)
            t += max(horiz / c.speed_mps,
                     vert / (policy.climb_mps if c.alt > alt else policy.descent_mps))
            x, y, alt = c.x, c.y, c.alt
        elif isinstance(c, Hold):
            t += c.seconds
        elif isinstance(c, ReturnToLaunch):
            t += math.hypot(x, y) / policy.home_speed_mps + alt / policy.descent_mps
            x, y, alt, on_ground = 0.0, 0.0, 0.0, True
        elif isinstance(c, Land):
            t += alt / policy.descent_mps
            alt, on_ground = 0.0, True
    return t, (x, y, alt), on_ground


def home_seconds(pos: Tuple[float, float, float], policy: BatteryPolicy) -> float:
    x, y, alt = pos
    return math.hypot(x, y) / policy.home_speed_mps + alt / policy.descent_mps


def check_step(commands: Sequence[ExecutableCommand], start: Tuple[float, float, float],
               battery_pct: Optional[float], policy: BatteryPolicy = BatteryPolicy()
               ) -> BatteryVerdict:
    step_s, end, on_ground = estimate_seconds(commands, start, policy)
    total_s = step_s + (0.0 if on_ground else home_seconds(end, policy))
    needed = total_s / 60.0 * policy.drain_pct_per_min
    if battery_pct is None:
        return BatteryVerdict(False, needed, "battery level unknown - refusing to start a step")
    if battery_pct < policy.min_start_pct:
        return BatteryVerdict(False, needed,
                              f"battery {battery_pct:.0f}% is below the {policy.min_start_pct:.0f}% "
                              "minimum to start a step")
    left = battery_pct - needed
    if left < policy.reserve_pct:
        return BatteryVerdict(False, needed,
                              f"this step plus the trip home needs ~{needed:.0f}% and would leave "
                              f"{left:.0f}%, below the {policy.reserve_pct:.0f}% reserve")
    return BatteryVerdict(True, needed,
                          f"battery ok: ~{needed:.0f}% for this step and the trip home, "
                          f"~{left:.0f}% left")
