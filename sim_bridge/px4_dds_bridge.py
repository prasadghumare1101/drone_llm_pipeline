"""PX4 SITL bridge over uXRCE-DDS (ROS 2 Humble + px4_msgs).

Drives PX4 in OFFBOARD mode from the compiled ExecutableCommand sequence.

Requirements on the target machine (see README for exact install commands):
  * ROS 2 Humble sourced, `px4_msgs` built in the workspace.
    CRITICAL: the px4_msgs branch/commit must match the PX4 checkout
    (release/1.14 <-> v1.14.x, release/1.15 <-> v1.15.x, main <-> main),
    otherwise DDS type hashes differ and readers/writers silently never match.
  * MicroXRCEAgent running:      MicroXRCEAgent udp4 -p 8888
  * PX4 SITL running:            make px4_sitl gazebo-classic[_<model>]

Conventions / documented assumptions:
  * Mission frame LOCAL_ENU_METERS -> PX4 NED: north = y, east = x, down = -alt.
  * Offboard rule respected: setpoints are streamed at 20 Hz and for >= 10 s
    BEFORE requesting OFFBOARD + arming (PX4 rejects the switch otherwise).
  * PX4 >= v1.16 / main renames some /fmu/out topics with a _v1 suffix.
  * Per-Goto speed: PX4 position setpoints are tracked under MPC_XY_CRUISE /
    MPC_XY_VEL_MAX; the validator has already capped requested speeds and this
    bridge stays position-only for robustness.
"""
from __future__ import annotations

import math
import time
from functools import partial
from typing import Optional, Sequence

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
from .capture_sink import CaptureSink

from .base_backend import BackendResult, MissionBackend

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        QoSProfile,
        ReliabilityPolicy,
        HistoryPolicy,
        DurabilityPolicy,
    )
    from px4_msgs.msg import (
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleCommand,
        VehicleLocalPosition,
        VehicleStatus,
    )
    _ROS_OK = True
    _ROS_ERR = ""
except ImportError as _e:  # pragma: no cover - exercised only off-sandbox
    _ROS_OK = False
    _ROS_ERR = str(_e)


ACCEPT_RADIUS_M = 0.7
TAKEOFF_Z_TOL_M = 0.5
TICK_HZ = 20.0
# Stream setpoints for this long before requesting OFFBOARD + arm.
WARMUP_SECONDS = 10.0
WARMUP_TICKS = int(WARMUP_SECONDS * TICK_HZ)
# Give SITL time to get a GPS lock / EKF convergence before giving up.
EKF_HEALTH_TIMEOUT_S = 60.0
# If VehicleStatus never delivers (px4_msgs<->PX4 type-hash mismatch), fall back
# to open-loop offboard/arm after this many seconds inside the Arm command.
STATUS_FALLBACK_S = 3.0
COMMAND_TIMEOUT_S = 180.0           # per-command watchdog
LANDED_Z_NED = -0.3                 # NED z > this  ==> effectively on ground


def _require_ros() -> None:
    if not _ROS_OK:
        raise RuntimeError(
            "PX4 DDS bridge needs ROS 2 Humble + px4_msgs, which are not "
            f"importable here ({_ROS_ERR}). Source /opt/ros/humble/setup.bash "
            "and your px4_msgs workspace (see README 'Real-rig setup')."
        )


if _ROS_OK:

    class _Px4MissionNode(Node):
        """20 Hz state machine executing the compiled command sequence."""

        def __init__(self, commands: Sequence[ExecutableCommand],
                     out_prefix: str = "/fmu/out", in_prefix: str = "/fmu/in",
                     status_topic: Optional[str] = None,
                     local_position_topic: Optional[str] = None,
                     out_dir=None):
            super().__init__("llm_pipeline_px4_bridge")
            self._captures = CaptureSink(out_dir)
            self._recording = False
            self._lat = 0.0
            self._lon = 0.0
            # PX4 uXRCE-DDS /fmu/in readers are BEST_EFFORT; match them exactly
            # (mirrors the working px4_ros_com offboard_control reference).
            qos_pub = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            qos_sub = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )
            self._ocm_pub = self.create_publisher(
                OffboardControlMode, f"{in_prefix}/offboard_control_mode", qos_pub)
            self._sp_pub = self.create_publisher(
                TrajectorySetpoint, f"{in_prefix}/trajectory_setpoint", qos_pub)
            self._cmd_pub = self.create_publisher(
                VehicleCommand, f"{in_prefix}/vehicle_command", qos_pub)

            # Subscribe to BOTH topic-name variants (unversioned + _v1);
            # only the one PX4 actually publishes will ever deliver.
            lp_candidates = [local_position_topic] if local_position_topic else [
                f"{out_prefix}/vehicle_local_position",
                f"{out_prefix}/vehicle_local_position_v1",
            ]
            st_candidates = [status_topic] if status_topic else [
                f"{out_prefix}/vehicle_status",
                f"{out_prefix}/vehicle_status_v1",
            ]
            for t in lp_candidates:
                self.create_subscription(
                    VehicleLocalPosition, t, partial(self._on_local_pos, t), qos_sub)
            for t in st_candidates:
                self.create_subscription(
                    VehicleStatus, t, partial(self._on_status, t), qos_sub)
            self._lp_src: Optional[str] = None
            self._st_src: Optional[str] = None

            self._commands = list(commands)
            self._idx = -1                      # -1 == warmup phase
            self._tick = 0
            self._cmd_started_at = time.monotonic()
            self._hold_until: Optional[float] = None

            self._pos_ned: Optional[tuple] = None   # (north, east, down)
            self._armed = False
            self._nav_state = -1
            # New state variables for EKF health and offboard/arming sequence
            self._local_pos_ok = False          # EKF local position valid
            self._offboard_confirmed = False    # OFFBOARD mode active
            self._arm_commanded = False         # arm command has been sent
            self._waiting_for_offboard = False  # waiting for mode switch ack
            self._stable_armed_count = 0        # ticks armed stable before advancing

            # target setpoint in NED; hold current position until told otherwise
            self._target_ned = (0.0, 0.0, 0.0)
            self._target_yaw = float("nan")

            self.done = False
            self.success = False
            self.detail = ""
            self.reached_gotos = 0
            self.total_gotos = sum(isinstance(c, Goto) for c in self._commands)

            self._timer = self.create_timer(1.0 / TICK_HZ, self._on_tick)
            self.get_logger().info(
                f"PX4 bridge up: {len(self._commands)} commands, "
                f"{self.total_gotos} waypoints. Warming up setpoint stream for "
                f"{WARMUP_SECONDS} s and waiting for EKF health…")

        # ------------------------- subscriptions ------------------------- #

        def _on_local_pos(self, topic: str, msg: "VehicleLocalPosition") -> None:
            if self._lp_src is None:
                self._lp_src = topic
                self.get_logger().info(f"position feedback live on {topic}")
            # EKF health for offboard position control == a valid local estimate.
            # This is the authoritative signal and does not depend on VehicleStatus.
            self._local_pos_ok = bool(msg.xy_valid and msg.z_valid)
            if self._local_pos_ok:
                self._pos_ned = (msg.x, msg.y, msg.z)

        def _on_status(self, topic: str, msg: "VehicleStatus") -> None:
            if self._st_src is None:
                self._st_src = topic
                self.get_logger().info(f"status feedback live on {topic}")
            self._armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
            self._nav_state = msg.nav_state

        # --------------------------- publishing -------------------------- #

        def _now_us(self) -> int:
            return int(self.get_clock().now().nanoseconds / 1000)

        def _publish_stream(self) -> None:
            ocm = OffboardControlMode()
            ocm.timestamp = self._now_us()
            ocm.position = True
            ocm.velocity = False
            ocm.acceleration = False
            ocm.attitude = False
            ocm.body_rate = False
            self._ocm_pub.publish(ocm)

            sp = TrajectorySetpoint()
            sp.timestamp = self._now_us()
            sp.position = [float(self._target_ned[0]),
                           float(self._target_ned[1]),
                           float(self._target_ned[2])]
            sp.yaw = float(self._target_yaw)
            self._sp_pub.publish(sp)

        def _vehicle_command(self, command: int, p1: float = 0.0,
                             p2: float = 0.0) -> None:
            m = VehicleCommand()
            m.timestamp = self._now_us()
            m.command = command
            m.param1 = float(p1)
            m.param2 = float(p2)
            m.target_system = 1
            m.target_component = 1
            m.source_system = 1
            m.source_component = 1
            m.from_external = True
            self._cmd_pub.publish(m)

        # -------------------------- helpers ------------------------------ #

        @staticmethod
        def _enu_to_ned(x_e: float, y_n: float, alt_up: float) -> tuple:
            return (y_n, x_e, -alt_up)

        def _dist_to_target(self) -> float:
            if self._pos_ned is None:
                return float("inf")
            dn = self._target_ned[0] - self._pos_ned[0]
            de = self._target_ned[1] - self._pos_ned[1]
            dd = self._target_ned[2] - self._pos_ned[2]
            return math.sqrt(dn * dn + de * de + dd * dd)

        def _advance(self) -> None:
            self._idx += 1
            self._cmd_started_at = time.monotonic()
            self._hold_until = None
            # Reset arm‑related flags for the new command (if it's not Arm we don't care)
            self._arm_commanded = False
            self._waiting_for_offboard = False
            self._stable_armed_count = 0

            if self._idx >= len(self._commands):
                self._finish(True, "all commands executed")
                return
            cmd = self._commands[self._idx]
            self.get_logger().info(f"[{self._idx + 1}/{len(self._commands)}] "
                                   f"{type(cmd).__name__} {cmd}")
            if isinstance(cmd, Goto):
                n, e, d = self._enu_to_ned(cmd.x, cmd.y, cmd.alt)
                if self._pos_ned is not None:
                    self._target_yaw = math.atan2(e - self._pos_ned[1],
                                                  n - self._pos_ned[0])
                self._target_ned = (n, e, d)
            elif isinstance(cmd, Takeoff):
                base = self._pos_ned or (0.0, 0.0, 0.0)
                self._target_ned = (base[0], base[1], -cmd.alt)
                self._target_yaw = float("nan")
            elif isinstance(cmd, Hold):
                self._hold_until = time.monotonic() + cmd.seconds
            elif isinstance(cmd, Capture):
                # Non-blocking: queued to a worker so the 20 Hz offboard
                # heartbeat is never interrupted by an HTTP fetch.
                self._captures.request(cmd.label, self._pos_ned,
                                       self._lat, self._lon)
            elif isinstance(cmd, Record):
                self._recording = cmd.start
                self.get_logger().info(
                    f"recording {'STARTED' if cmd.start else 'STOPPED'}")
            elif isinstance(cmd, Gimbal):
                # MAV_CMD_DO_MOUNT_CONTROL: param1=pitch, param2=roll, param3=yaw
                self._vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONTROL,
                    cmd.pitch_deg, 0.0)
                self.get_logger().info(f"gimbal pitch -> {cmd.pitch_deg}°")
            elif isinstance(cmd, Land):
                self._vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            elif isinstance(cmd, ReturnToLaunch):
                self._vehicle_command(
                    VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
            elif isinstance(cmd, Arm):
                # Do NOT send mode+arm immediately. We'll do it in the tick loop.
                # Just set the waiting flag and let the tick loop handle it.
                if not self._offboard_confirmed:
                    self._vehicle_command(
                        VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # OFFBOARD
                    self._waiting_for_offboard = True
                    self.get_logger().info("Requested OFFBOARD mode, waiting for confirmation…")
                else:
                    # Already in OFFBOARD – arm now
                    self._vehicle_command(
                        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                    self._arm_commanded = True
                    self.get_logger().info("Arm command sent")
            # Disarm: passive — wait for PX4 auto-disarm after land/RTL

        def _finish(self, ok: bool, detail: str) -> None:
            self.done = True
            self.success = ok
            self.detail = detail
            self.get_logger().info(f"mission finished: ok={ok} ({detail})")

        # --------------------------- main tick --------------------------- #

        def _on_tick(self) -> None:
            if self.done:
                return
            self._tick += 1
            self._publish_stream()          # never stops: offboard heartbeat

            # ---------- WARMUP PHASE ----------
            if self._idx == -1:
                if self._pos_ned is not None:
                    n, e, d = self._pos_ned
                    self._target_ned = (n, e, d)   # hold current position
                # Wait until EKF is healthy AND we have position AND enough time
                if (self._pos_ned is not None and self._local_pos_ok and
                        self._tick >= WARMUP_TICKS):
                    self.get_logger().info("Warmup complete – EKF healthy and position valid.")
                    self._advance()
                elif self._tick > int(EKF_HEALTH_TIMEOUT_S * TICK_HZ):
                    if self._pos_ned is None:
                        self.get_logger().error(
                            f"No position feedback after {EKF_HEALTH_TIMEOUT_S:.0f} s – "
                            "check px4_msgs<->PX4 version match and the uXRCE-DDS agent"
                        )
                        self._finish(False, "no position feedback")
                    else:
                        self.get_logger().error(
                            f"EKF never became healthy after {EKF_HEALTH_TIMEOUT_S:.0f} s "
                            "– check PX4 startup / GPS lock"
                        )
                        self._finish(False, "EKF health timeout")
                return   # stay in warmup

            # ---------- HANDLE OFFBOARD / ARM SEQUENCE ----------
            cmd = self._commands[self._idx]
            elapsed = time.monotonic() - self._cmd_started_at
            if elapsed > COMMAND_TIMEOUT_S:
                self._finish(False, f"timeout on {type(cmd).__name__} "
                                    f"after {COMMAND_TIMEOUT_S:.0f} s")
                return

            if isinstance(cmd, Arm):
                have_status = self._st_src is not None
                # If waiting for OFFBOARD confirmation
                if self._waiting_for_offboard:
                    # Confirm via feedback when we have it; otherwise (no VehicleStatus
                    # delivering) proceed open-loop after a short settle time. PX4 will
                    # accept the switch as long as setpoints keep streaming, which they do.
                    offboard_ok = (self._nav_state ==
                                   VehicleStatus.NAVIGATION_STATE_OFFBOARD)
                    if offboard_ok or (not have_status and elapsed > STATUS_FALLBACK_S):
                        self._offboard_confirmed = True
                        self._waiting_for_offboard = False
                        self.get_logger().info(
                            "OFFBOARD %s – now arming" %
                            ("confirmed" if offboard_ok else "assumed (no status feedback)"))
                        self._vehicle_command(
                            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                        self._arm_commanded = True
                    return   # stay in this command until mode is confirmed/assumed

                # If we already sent the arm command, wait for arm to become true
                if self._arm_commanded:
                    if self._armed:
                        # Ensure it stays armed for a few ticks (defence against auto‑disarm)
                        self._stable_armed_count += 1
                        if self._stable_armed_count >= 3:   # ~0.15s
                            self._advance()
                    elif not have_status and elapsed > STATUS_FALLBACK_S + 2.0:
                        # No armed feedback available – assume the command took and
                        # continue; the Takeoff altitude check will catch a real failure.
                        self.get_logger().info("arm assumed (no status feedback) – continuing")
                        self._advance()
                    else:
                        # Arm not yet true; if too much time passes, abort
                        if have_status and elapsed > 10.0:
                            self._finish(False, "Arm command timed out")
                    return   # stay in this command until armed stable

                # Should not get here – but just in case, advance?
                self._advance()
                return

            # ---------- OTHER COMMANDS ----------
            if isinstance(cmd, Takeoff):
                if self._pos_ned is not None and \
                        abs(self._pos_ned[2] - self._target_ned[2]) < TAKEOFF_Z_TOL_M:
                    self._advance()
            elif isinstance(cmd, Goto):
                if self._dist_to_target() < ACCEPT_RADIUS_M:
                    self.reached_gotos += 1
                    self._advance()
            elif isinstance(cmd, Hold):
                if self._hold_until is not None and \
                        time.monotonic() >= self._hold_until:
                    self._advance()
            elif isinstance(cmd, (Land, ReturnToLaunch)):
                on_ground = (self._pos_ned is not None
                             and self._pos_ned[2] > LANDED_Z_NED)
                if not self._armed and (on_ground or elapsed > 5.0):
                    self._advance()
            elif isinstance(cmd, (Capture, Record, Gimbal)):
                # Fire-and-forget: dispatched in _advance(), nothing to await.
                self._advance()
            elif isinstance(cmd, Disarm):
                if not self._armed:
                    self._advance()
                elif elapsed > 20.0:
                    self._vehicle_command(
                        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)


class Px4DdsBackend(MissionBackend):
    """MissionBackend wrapper: spins the ROS 2 node until the mission ends."""
    name = "px4"

    def __init__(self, mission_timeout_s: float = 900.0,
                 status_topic: Optional[str] = None,
                 local_position_topic: Optional[str] = None,
                 out_dir=None):
        _require_ros()
        self.mission_timeout_s = mission_timeout_s
        self.status_topic = status_topic
        self.local_position_topic = local_position_topic
        self.out_dir = out_dir

    def _run_checked(self, commands: Sequence[ExecutableCommand]) -> BackendResult:
        rclpy.init()
        node = _Px4MissionNode(commands,
                               status_topic=self.status_topic,
                               local_position_topic=self.local_position_topic,
                               out_dir=self.out_dir)
        deadline = time.monotonic() + self.mission_timeout_s
        try:
            while rclpy.ok() and not node.done and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if not node.done:
                node.detail = f"mission timeout after {self.mission_timeout_s:.0f} s"
        finally:
            node._captures.close()          # drains, then writes the geotag index
            node.destroy_node()
            rclpy.shutdown()
        caps = node._captures
        artifacts = {}
        if caps.dir:
            artifacts["captures_dir"] = str(caps.dir)
            artifacts["capture_index"] = str(caps.dir / "index.json")
        return BackendResult(
            backend=self.name,
            completed=node.done and node.success,
            detail=node.detail,
            metrics={"waypoints_reached": float(node.reached_gotos),
                     "waypoints_commanded": float(node.total_gotos),
                     "captures_saved": float(caps.saved),
                     "captures_dropped": float(caps.dropped),
                     "captures_failed": float(caps.failed)},
            artifacts=artifacts,
        )