"""Persistent vehicle link for interactive (step-by-step) flight.

The one-shot backends (MissionBackend) own the vehicle for exactly one mission:
connect, arm, fly, land, disconnect. Interactive mode needs the opposite: one
long-lived link that keeps the vehicle hovering between chat messages and can
be told to stop / land / return home at any moment, from any thread.

A link exposes setpoint-level primitives only. Walking a compiled step is the
job of session.runner, which is shared by every link.

Thread-safety contract: every method may be called from any thread, at any
time, including while another thread is inside a wait loop on state().
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

from executor import Capture, Gimbal, Record


@dataclass(frozen=True)
class VehicleState:
    """Snapshot of the vehicle. LOCAL_ENU_METERS; heading is compass degrees."""
    x: float
    y: float
    alt: float
    heading_deg: float
    armed: bool
    in_air: bool
    battery_pct: Optional[float]   # None = unknown (never assume it is fine)
    mode: str                      # human-readable flight mode
    rc_override: bool              # pilot has taken control away from us
    connected: bool                # fresh position data is arriving
    fault: str = ""                # autopilot failure detector, e.g. "attitude (roll)"


class VehicleLink(ABC):
    name: str = "abstract"
    accept_radius_m: float = 0.7    # Goto counts as reached inside this radius
    alt_tolerance_m: float = 0.5    # Takeoff counts as reached inside this

    @abstractmethod
    def state(self) -> VehicleState: ...

    @abstractmethod
    def now(self) -> float:
        """Vehicle clock in seconds (sim time for simulators)."""

    @abstractmethod
    def goto(self, x: float, y: float, alt: float, speed_mps: float) -> None:
        """Fly to an absolute position under our (offboard) control."""

    @abstractmethod
    def hold(self) -> None:
        """Hover where the vehicle is now. Also cancels a LAND/RTL in progress."""

    @abstractmethod
    def land(self) -> None: ...

    @abstractmethod
    def rtl(self) -> None: ...

    @abstractmethod
    def arm(self) -> None:
        """PILOT ACTION ONLY. Nothing in the session or step runner calls this;
        it exists for the dashboard's ARM button (the pilot's consent)."""

    @abstractmethod
    def release_override(self) -> None:
        """Pilot hands control back after an RC override: resume holding here."""

    def payload(self, cmd) -> str:
        """Camera / gimbal side effects. Returns a short log line."""
        if isinstance(cmd, Capture):
            return f"photo '{cmd.label or 'photo'}'"
        if isinstance(cmd, Record):
            return f"recording {'started' if cmd.start else 'stopped'}"
        if isinstance(cmd, Gimbal):
            return f"gimbal pitch {cmd.pitch_deg:g} deg"
        return type(cmd).__name__

    def close(self) -> None:
        """Release the vehicle. Implementations must leave it safe."""
