"""Real-time kinematic simulator behind the VehicleLink interface.

The one-shot KinematicSimBackend integrates a whole mission as fast as it can.
Interactive mode needs a vehicle that *lives*: it hovers between messages, can
be interrupted mid-leg, and drains its battery while it waits. This link runs
the same point-mass model on a background thread against a clock that can be
sped up (time_scale) so tests fly minutes of flight in a second or two.

PX4 behaviours mirrored on purpose, so the session is exercised against the
same edge cases it meets on the real autopilot:
  * auto-disarm 2 s after touchdown and 10 s after arming without taking off,
  * RTL climbs to a minimum return altitude, flies home, then lands,
  * an RC takeover (pilot_takeover) switches to a pilot mode that we do not
    control; only pilot_handback / release_override returns control,
  * a horizontal leg longer than 1 m first turns the vehicle in place to face
    the direction of travel, then flies forward (same rule as Px4Link).
"""
from __future__ import annotations

import math
import threading
import time
from typing import Optional

from .vehicle_link import VehicleLink, VehicleState

DT = 0.05                      # s of sim time per integration step
CLIMB_MPS = 2.0
DESCENT_MPS = 1.0
RTL_MIN_ALT_M = 10.0
DISARM_AFTER_LAND_S = 2.0      # PX4 COM_DISARM_LAND
DISARM_PREFLIGHT_S = 10.0      # PX4 COM_DISARM_PRFLT
GROUND_M = 0.05
YAW_RATE_DPS = 60.0            # turn rate while aligning with a new leg
TURN_MIN_LEG_M = 1.0
ALIGN_TOL_DEG = 8.0


def _wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


class KinematicLink(VehicleLink):
    name = "sim"
    accept_radius_m = 0.5
    alt_tolerance_m = 0.3

    def __init__(self, time_scale: float = 1.0, heading_deg: float = 90.0,
                 battery_pct: float = 100.0, drain_pct_per_min: float = 2.5):
        self.time_scale = float(time_scale)
        self._lock = threading.RLock()
        self._x = self._y = self._alt = 0.0
        self._heading = heading_deg
        self._armed = False
        self._mode = "IDLE"               # IDLE | OFFBOARD | LAND | RTL | POSCTL
        self._target = (0.0, 0.0, 0.0)
        self._speed = 5.0
        self._battery = battery_pct
        self._drain = drain_pct_per_min
        self._t = 0.0
        self._ground_since: Optional[float] = 0.0
        self._rtl_phase = ""
        self._yaw_target = heading_deg
        self._fault = ""
        self._stuck = False
        self._aligning = False
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="kinematic-link", daemon=True)
        self._thread.start()

    # ---------------------------- interface ---------------------------- #

    def state(self) -> VehicleState:
        with self._lock:
            return VehicleState(
                x=round(self._x, 3), y=round(self._y, 3), alt=round(self._alt, 3),
                heading_deg=self._heading, armed=self._armed,
                in_air=self._alt > GROUND_M, battery_pct=round(self._battery, 1),
                mode=self._mode, rc_override=self._mode == "POSCTL", connected=True,
                fault=self._fault)

    def now(self) -> float:
        with self._lock:
            return self._t

    def goto(self, x: float, y: float, alt: float, speed_mps: float) -> None:
        with self._lock:
            if self._mode == "POSCTL":
                return                    # the pilot has the vehicle
            self._mode = "OFFBOARD"
            self._target = (x, y, alt)
            self._speed = speed_mps
            dx, dy = x - self._x, y - self._y
            self._aligning = False
            if math.hypot(dx, dy) >= TURN_MIN_LEG_M:
                self._yaw_target = math.degrees(math.atan2(dx, dy)) % 360.0
                self._aligning = abs(_wrap_deg(self._yaw_target - self._heading)) >= ALIGN_TOL_DEG

    def hold(self) -> None:
        with self._lock:
            if self._mode == "POSCTL":
                return
            self._mode = "OFFBOARD" if self._armed else "IDLE"
            self._target = (self._x, self._y, self._alt)
            self._aligning = False

    def land(self) -> None:
        with self._lock:
            if self._mode != "POSCTL" and self._armed:
                self._mode = "LAND"

    def rtl(self) -> None:
        with self._lock:
            if self._mode != "POSCTL" and self._armed:
                self._mode, self._rtl_phase = "RTL", "climb"

    def arm(self) -> None:
        with self._lock:
            if not self._armed:
                self._armed = True
                self._mode = "OFFBOARD"
                self._target = (self._x, self._y, self._alt)
                self._ground_since = self._t     # the 10 s pre-flight timer starts at arming

    def release_override(self) -> None:
        self.pilot_handback()

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    # ------------------------ test / sim hooks ------------------------- #

    def pilot_takeover(self) -> None:
        """Simulate the pilot flipping the RC mode switch (position mode)."""
        with self._lock:
            self._mode = "POSCTL"
            self._target = (self._x, self._y, self._alt)

    def pilot_handback(self) -> None:
        with self._lock:
            if self._mode == "POSCTL":
                self._mode = "OFFBOARD" if self._armed else "IDLE"
                self._target = (self._x, self._y, self._alt)

    def inject_fault(self, fault: str = "attitude (roll)", stuck: bool = True) -> None:
        """Simulate an autopilot fault (e.g. a crash into terrain): the vehicle
        reports `fault` and, if `stuck`, stops moving whatever it is told."""
        with self._lock:
            self._fault, self._stuck = fault, stuck

    def set_battery(self, pct: float) -> None:
        with self._lock:
            self._battery = pct

    # ----------------------------- physics ----------------------------- #

    def _run(self) -> None:
        wall0 = time.monotonic()
        while not self._stop.is_set():
            target_t = (time.monotonic() - wall0) * self.time_scale
            with self._lock:
                while self._t + DT <= target_t:
                    self._step()
            self._stop.wait(0.01)

    def _move_toward(self, tx: float, ty: float, tz: float, speed: float) -> float:
        dx, dy, dz = tx - self._x, ty - self._y, tz - self._alt
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist < 1e-9:
            return 0.0
        horiz = math.hypot(dx, dy)
        # vertical-only legs use the climb / descent rate, like a multicopter
        if horiz < 1e-6:
            speed = min(speed, CLIMB_MPS if dz > 0 else DESCENT_MPS)
        move = min(speed * DT, dist)
        self._x += dx / dist * move
        self._y += dy / dist * move
        self._alt = max(0.0, self._alt + dz / dist * move)
        return dist - move

    def _step(self) -> None:
        self._t += DT
        if self._alt > GROUND_M:
            self._battery = max(0.0, self._battery - self._drain * DT / 60.0)
        if not self._armed or self._stuck:
            return

        if self._mode == "OFFBOARD":
            err = _wrap_deg(self._yaw_target - self._heading)
            turn = max(-YAW_RATE_DPS * DT, min(YAW_RATE_DPS * DT, err))
            self._heading = (self._heading + turn) % 360.0
            if self._aligning and abs(err - turn) < ALIGN_TOL_DEG:
                self._aligning = False
            if not self._aligning:        # turn in place first, then fly forward
                self._move_toward(*self._target, self._speed)
        elif self._mode == "LAND":
            self._move_toward(self._x, self._y, 0.0, DESCENT_MPS)
        elif self._mode == "RTL":
            if self._rtl_phase == "climb":
                if self._move_toward(self._x, self._y, max(self._alt, RTL_MIN_ALT_M),
                                     CLIMB_MPS) <= 0.01:
                    self._rtl_phase = "home"
            elif self._rtl_phase == "home":
                if self._move_toward(0.0, 0.0, self._alt, 5.0) <= 0.01:
                    self._rtl_phase = "descend"
            else:
                self._move_toward(0.0, 0.0, 0.0, DESCENT_MPS)
        # POSCTL: the pilot holds the vehicle where it is

        on_ground = self._alt <= GROUND_M
        if on_ground and self._ground_since is None:
            self._ground_since = self._t
        elif not on_ground:
            self._ground_since = None
        if on_ground and self._ground_since is not None:
            landing = self._mode in ("LAND", "RTL")
            limit = DISARM_AFTER_LAND_S if landing else DISARM_PREFLIGHT_S
            # an OFFBOARD target above ground means "about to take off"
            climbing = self._mode == "OFFBOARD" and self._target[2] > GROUND_M
            if not climbing and self._t - self._ground_since >= limit:
                self._armed = False
                self._mode = "IDLE"
