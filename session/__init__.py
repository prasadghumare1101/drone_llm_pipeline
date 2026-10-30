"""Interactive flight: one chat message = one step from the live position.

See docs/INTERACTIVE_MODE.md. The safety boundary is unchanged: every step
goes through validator.validate_step() and executor.compile_step(); this
package only decides WHEN a validated step runs and stops it instantly.
"""
from .battery import BatteryPolicy, BatteryVerdict, check_step
from .commands import FastCommand, parse_fast_command
from .runner import StepOutcome, run_step
from .session import Phase, Session

__all__ = ["Session", "Phase", "FastCommand", "parse_fast_command", "run_step",
           "StepOutcome", "BatteryPolicy", "BatteryVerdict", "check_step"]
