"""PX4 (uXRCE-DDS, ROS 2 Humble) behind the VehicleLink interface.

One long-lived node that keeps the vehicle hovering under OFFBOARD control
between interactive steps. Same QoS / topic-variant rules as px4_dds_bridge.

Lessons this file is built on (each one was a real failure in SITL):
  * /fmu/in/trajectory_setpoint feeds the SAME uORB topic PX4's own auto modes
    (LAND, RTL, LOITER, POSCTL) write. Position setpoints are therefore only
    published while PX4 is in OFFBOARD, or while we are switching into it. The
    OffboardControlMode heartbeat streams always: it only advertises that an
    offboard source exists, so PX4 accepts a switch into OFFBOARD at any time.
  * px4_msgs must match the PX4 checkout, or VehicleStatus never decodes and
    the link is blind to armed / mode / RC override.

RC override: if the vehicle leaves the mode we asked for while armed (pilot
flips the mode switch or moves the sticks with COM_RC_OVERRIDE, or a PX4
failsafe takes over), the link latches rc_override and stops publishing
setpoints. It clears when the pilot switches back to OFFBOARD, or on
release_override() (the operator explicitly hands control back).

Heading: like a pilot, the vehicle turns to face where it is going and then
flies forward. A horizontal leg longer than TURN_MIN_LEG_M first yaws in place
(position held) until the heading is within ALIGN_TOL_DEG, then the position
target is released. Climbs, descents and short nudges keep the heading.
"""
from __future__ import annotations

import math
import threading
import time
from functools import partial
from typing import Optional

from .vehicle_link import VehicleLink, VehicleState

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from px4_msgs.msg import (
        BatteryStatus,
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleCommand,
        VehicleLandDetected,
        VehicleLocalPosition,
        VehicleStatus,
    )
    _ROS_OK, _ROS_ERR = True, ""
except ImportError as _e:  # pragma: no cover - exercised only off-sandbox
    _ROS_OK, _ROS_ERR = False, str(_e)

TICK_HZ = 20.0
STALE_S = 1.0                     # no local position for this long -> disconnected
MODE_SWITCH_TIMEOUT_S = 3.0
TURN_MIN_LEG_M = 1.0             # shorter horizontal moves do not turn the vehicle
ALIGN_TOL_DEG = 8.0              # heading error at which a turned leg starts moving
ARM_TIMEOUT_S = 5.0

if _ROS_OK:
    NAV = VehicleStatus
    _NAV_NAMES = {
        NAV.NAVIGATION_STATE_MANUAL: "MANUAL", NAV.NAVIGATION_STATE_ALTCTL: "ALTITUDE",
        NAV.NAVIGATION_STATE_POSCTL: "POSITION", NAV.NAVIGATION_STATE_AUTO_MISSION: "MISSION",
        NAV.NAVIGATION_STATE_AUTO_LOITER: "LOITER", NAV.NAVIGATION_STATE_AUTO_RTL: "RTL",
        NAV.NAVIGATION_STATE_ACRO: "ACRO", NAV.NAVIGATION_STATE_DESCEND: "DESCEND",
        NAV.NAVIGATION_STATE_TERMINATION: "TERMINATION", NAV.NAVIGATION_STATE_OFFBOARD: "OFFBOARD",
        NAV.NAVIGATION_STATE_STAB: "STABILIZED", NAV.NAVIGATION_STATE_AUTO_TAKEOFF: "TAKEOFF",
        NAV.NAVIGATION_STATE_AUTO_LAND: "LAND",
    }
    # PX4 failure detector (VehicleStatus.failure_detector_status)
    _FAILURES = [(getattr(NAV, f"FAILURE_{k}"), v) for k, v in (
        ("ROLL", "attitude (roll) - likely crashed or tipped over"),
        ("PITCH", "attitude (pitch) - likely crashed or tipped over"),
        ("ALT", "altitude"), ("EXT", "external"), ("ARM_ESC", "ESC at arming"),
        ("BATTERY", "battery"), ("IMBALANCED_PROP", "imbalanced propeller"), ("MOTOR", "motor"))
        if hasattr(NAV, f"FAILURE_{k}")]
    # modes PX4 may legitimately be in after each of our requests
    _EXPECTED = {
        "OFFBOARD": {NAV.NAVIGATION_STATE_OFFBOARD},
        "LAND": {NAV.NAVIGATION_STATE_AUTO_LAND},
        "RTL": {NAV.NAVIGATION_STATE_AUTO_RTL, NAV.NAVIGATION_STATE_AUTO_LAND},
    }


def _wrap(a: float) -> float:
    """Angle difference wrapped to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Px4Link(VehicleLink):
    name = "px4"
    accept_radius_m = 1.0            # horizontal; outdoors GPS alone wanders ~1 m
    alt_tolerance_m = 0.7

    def __init__(self, out_prefix: str = "/fmu/out", in_prefix: str = "/fmu/in"):
        if not _ROS_OK:
            raise RuntimeError(
                "Px4Link needs ROS 2 Humble + px4_msgs, which are not importable "
                f"here ({_ROS_ERR}). Source /opt/ros/humble/setup.bash and the workspace.")
        if not rclpy.ok():
            rclpy.init()
        self._lock = threading.RLock()
        self._node = rclpy.create_node("aeromast_session_link")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        n = self._node
        self._ocm_pub = n.create_publisher(OffboardControlMode, f"{in_prefix}/offboard_control_mode", qos)
        self._sp_pub = n.create_publisher(TrajectorySetpoint, f"{in_prefix}/trajectory_setpoint", qos)
        self._cmd_pub = n.create_publisher(VehicleCommand, f"{in_prefix}/vehicle_command", qos)
        for base, typ, cb in (("vehicle_local_position", VehicleLocalPosition, self._on_pos),
                              ("vehicle_status", VehicleStatus, self._on_status),
                              ("battery_status", BatteryStatus, self._on_battery),
                              ("vehicle_land_detected", VehicleLandDetected, self._on_land)):
            for t in (f"{out_prefix}/{base}", f"{out_prefix}/{base}_v1"):
                n.create_subscription(typ, t, cb, qos)

        # vehicle state (NED from PX4, converted on read)
        self._ned: Optional[tuple] = None
        self._heading_rad = 0.0
        self._pos_stamp = 0.0
        self._armed = False
        self._nav = -1
        self._failsafe = False
        self._battery: Optional[float] = None
        self._fault = ""
        self._invalid_since: Optional[float] = None   # EKF position estimate lost since
        self._landed: Optional[bool] = None

        # control state
        self._want: Optional[str] = None      # OFFBOARD | LAND | RTL | None
        self._confirmed = False               # PX4 reached the mode we asked for
        self._override = False
        self._target_ned = (0.0, 0.0, 0.0)
        self._yaw = float("nan")
        self._goal_ned: Optional[tuple] = None  # leg waiting for the turn to finish

        n.create_timer(1.0 / TICK_HZ, self._tick)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(n)
        self._spin = threading.Thread(target=self._executor.spin, name="px4-link", daemon=True)
        self._spin.start()

    # --------------------------- subscriptions -------------------------- #

    def _on_pos(self, m) -> None:
        with self._lock:
            valid = bool(m.xy_valid and m.z_valid)
            if valid:
                self._invalid_since = None
            elif self._invalid_since is None:
                self._invalid_since = time.monotonic()
        if m.xy_valid and m.z_valid:
            with self._lock:
                self._ned = (m.x, m.y, m.z)
                self._heading_rad = m.heading
                self._pos_stamp = time.monotonic()

    def _on_status(self, m) -> None:
        with self._lock:
            self._armed = m.arming_state == VehicleStatus.ARMING_STATE_ARMED
            self._nav = m.nav_state
            self._failsafe = bool(m.failsafe)
            bits = int(getattr(m, "failure_detector_status", 0))
            self._fault = ", ".join(name for bit, name in _FAILURES if bits & bit)
            if not self._armed:
                self._want, self._confirmed, self._override = None, False, False
                return
            if self._override:
                if self._nav == NAV.NAVIGATION_STATE_OFFBOARD:   # pilot switched back
                    self._override = False
                    self._take_control_here("OFFBOARD")
                    self._confirmed = True
                return
            if self._want is None:
                return
            expected = _EXPECTED[self._want]
            if self._nav in expected:
                self._confirmed = True
            elif self._confirmed:
                # we had the mode we asked for and something else took it
                self._override = True
                self._want = None

    def _on_battery(self, m) -> None:
        with self._lock:
            self._battery = float(m.remaining) * 100.0 if m.remaining >= 0.0 else None

    def _on_land(self, m) -> None:
        with self._lock:
            self._landed = bool(m.landed)

    # ------------------------------ helpers ----------------------------- #

    def _now_us(self) -> int:
        return int(self._node.get_clock().now().nanoseconds / 1000)

    def _command(self, command: int, p1: float = 0.0, p2: float = 0.0) -> None:
        m = VehicleCommand()
        m.timestamp = self._now_us()
        m.command = command
        m.param1, m.param2 = float(p1), float(p2)
        m.target_system = m.target_component = 1
        m.source_system = m.source_component = 1
        m.from_external = True
        self._cmd_pub.publish(m)

    def _take_control_here(self, want: str) -> None:
        """Caller holds the lock. Hold the current position at the current heading."""
        if self._ned is not None:
            self._target_ned = self._ned
        self._yaw = self._heading_rad
        self._goal_ned = None
        self._want = want

    def _request_offboard(self) -> None:
        """Caller holds the lock."""
        if self._nav != NAV.NAVIGATION_STATE_OFFBOARD:
            self._confirmed = False
            self._command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self._want = "OFFBOARD"

    def _tick(self) -> None:
        with self._lock:
            ocm = OffboardControlMode()
            ocm.timestamp = self._now_us()
            ocm.position = True
            ocm.velocity = ocm.acceleration = ocm.attitude = ocm.body_rate = False
            self._ocm_pub.publish(ocm)
            if self._override or self._want != "OFFBOARD":
                return            # never write setpoints over PX4's own modes
            if self._goal_ned is not None and \
                    abs(_wrap(self._yaw - self._heading_rad)) < math.radians(ALIGN_TOL_DEG):
                self._target_ned, self._goal_ned = self._goal_ned, None   # turned: go
            sp = TrajectorySetpoint()
            sp.timestamp = self._now_us()
            sp.position = [float(v) for v in self._target_ned]
            sp.yaw = float(self._yaw)
            self._sp_pub.publish(sp)

    # ----------------------------- interface ---------------------------- #

    def state(self) -> VehicleState:
        with self._lock:
            n, e, d = self._ned or (0.0, 0.0, 0.0)
            in_air = (not self._landed) if self._landed is not None else -d > 0.3
            if self._override:
                mode = "FAILSAFE" if self._failsafe else _NAV_NAMES.get(self._nav, f"MODE {self._nav}")
            else:
                mode = _NAV_NAMES.get(self._nav, "UNKNOWN")
            return VehicleState(
                x=round(e, 3), y=round(n, 3), alt=round(-d, 3),
                heading_deg=round(math.degrees(self._heading_rad) % 360.0, 1),
                armed=self._armed, in_air=bool(in_air and self._armed),
                battery_pct=None if self._battery is None else round(self._battery, 1),
                mode=mode, rc_override=self._override,
                connected=self._ned is not None and time.monotonic() - self._pos_stamp < STALE_S,
                fault=self._faults())

    def _faults(self) -> str:
        """Caller holds the lock. Failure detector + a lost position estimate:
        without it the last good position would be shown as if it were current
        while the autopilot's failsafe is actually descending the vehicle."""
        out = [self._fault] if self._fault else []
        if self._armed and self._invalid_since is not None \
                and time.monotonic() - self._invalid_since > 1.0:
            out.append("position estimate lost (GPS / EKF) - the position shown is the LAST "
                       "KNOWN one, the vehicle may be descending or drifting")
        return "; ".join(out)

    def now(self) -> float:
        return time.monotonic()

    def goto(self, x: float, y: float, alt: float, speed_mps: float) -> None:
        # Position-only setpoints: PX4 flies them at MPC_XY_CRUISE, the
        # validator has already capped any requested speed.
        with self._lock:
            if self._override:
                return
            if self._want != "OFFBOARD":
                self._take_control_here("OFFBOARD")
            goal = (y, x, -alt)
            here = self._ned or goal
            dn, de = goal[0] - here[0], goal[1] - here[1]
            self._goal_ned = None
            if math.hypot(dn, de) >= TURN_MIN_LEG_M:
                self._yaw = math.atan2(de, dn)          # face the direction of travel
                if abs(_wrap(self._yaw - self._heading_rad)) >= math.radians(ALIGN_TOL_DEG):
                    # turn in place first: hold here, release the leg once aligned
                    self._target_ned, self._goal_ned = here, goal
                    self._request_offboard()
                    return
            self._target_ned = goal
            self._request_offboard()

    def hold(self) -> None:
        with self._lock:
            if self._override or not self._armed:
                return
            self._take_control_here("OFFBOARD")
            self._request_offboard()

    def land(self) -> None:
        with self._lock:
            if self._override or not self._armed:
                return
            self._want, self._confirmed = "LAND", False
            self._command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def rtl(self) -> None:
        with self._lock:
            if self._override or not self._armed:
                return
            self._want, self._confirmed = "RTL", False
            self._command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)

    def arm(self) -> None:
        """PILOT ACTION ONLY (dashboard ARM button). OFFBOARD first, then arm,
        the order PX4 requires for an offboard take-off."""
        with self._lock:
            if self._armed:
                return
            if self._ned is None:
                raise RuntimeError("no position estimate from PX4 yet")
            self._take_control_here("OFFBOARD")
            self._request_offboard()
        self._wait(lambda: self._nav == NAV.NAVIGATION_STATE_OFFBOARD,
                   MODE_SWITCH_TIMEOUT_S, "PX4 did not enter OFFBOARD")
        with self._lock:
            self._command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self._wait(lambda: self._armed, ARM_TIMEOUT_S,
                   "PX4 refused to arm (check the pxh / sitl log for the reason)")

    def release_override(self) -> None:
        with self._lock:
            if not self._override or not self._armed:
                return
            self._override = False
            self._take_control_here("OFFBOARD")
            self._request_offboard()

    def close(self) -> None:
        with self._lock:
            flying = self._armed and not self._landed and not self._override
        if flying:
            self.rtl()           # never abandon a hovering vehicle mid-air
            time.sleep(0.3)
        self._executor.shutdown(timeout_sec=1.0)
        self._node.destroy_node()

    def _wait(self, cond, timeout_s: float, err: str) -> None:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            with self._lock:
                if cond():
                    return
            time.sleep(0.05)
        raise RuntimeError(err)
