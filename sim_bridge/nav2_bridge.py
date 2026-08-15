"""Nav2 bridge (ground-robot case): ROS 2 Humble + Gazebo Classic + Nav2.

Feeds the compiled Goto sequence to Nav2 via nav2_simple_commander's
BasicNavigator (NavigateToPose under the hood). Loops are already unrolled by
the executor, so this bridge just walks a flat waypoint list.

Target-machine bring-up (see also launch/nav2_sim.launch.py and README):
  export TURTLEBOT3_MODEL=waffle
  ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False

Conventions:
  * Mission LOCAL_ENU_METERS maps 1:1 onto the ROS 'map' frame (x=E, y=N),
    with the robot's start pose as the origin — set the initial pose in RViz
    (or via BasicNavigator.setInitialPose) accordingly.
  * Drone-only commands (Arm/Takeoff/Land/RTL/Disarm) are rejected — the
    validator already forbids them for ground_robot; the check here is
    defence-in-depth.

Reviewed but not CI-tested in this sandbox (no ROS 2 here).
"""
from __future__ import annotations

import math
import time
from typing import Sequence

from executor import ExecutableCommand, Goto, Hold

from .base_backend import BackendResult, MissionBackend

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    _NAV2_OK = True
    _NAV2_ERR = ""
except ImportError as _e:  # pragma: no cover
    _NAV2_OK = False
    _NAV2_ERR = str(_e)


class Nav2Backend(MissionBackend):
    name = "nav2"

    def __init__(self, frame_id: str = "map"):
        if not _NAV2_OK:
            raise RuntimeError(
                "Nav2 backend needs ROS 2 Humble + nav2_simple_commander "
                f"(import failed: {_NAV2_ERR}). Install: sudo apt install "
                "ros-humble-navigation2 ros-humble-nav2-bringup "
                "ros-humble-nav2-simple-commander — see README."
            )
        self.frame_id = frame_id

    def _pose(self, nav: "BasicNavigator", x: float, y: float,
              yaw: float) -> "PoseStamped":
        p = PoseStamped()
        p.header.frame_id = self.frame_id
        p.header.stamp = nav.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    def _run_checked(self, commands: Sequence[ExecutableCommand]) -> BackendResult:
        bad = [type(c).__name__ for c in commands
               if not isinstance(c, (Goto, Hold))]
        if bad:
            raise ValueError(
                f"Nav2 (ground robot) backend cannot execute: {sorted(set(bad))}. "
                "Use vehicle='ground_robot' in the mission so the validator "
                "restricts commands to GOTO/LOOP/HOLD."
            )

        rclpy.init()
        nav = BasicNavigator()
        reached = 0
        total = sum(isinstance(c, Goto) for c in commands)
        ok = True
        detail = "all waypoints reached"
        try:
            nav.waitUntilNav2Active()
            prev = (0.0, 0.0)
            for i, cmd in enumerate(commands):
                if isinstance(cmd, Hold):
                    print(f"[nav2 {i + 1}/{len(commands)}] HOLD {cmd.seconds:.1f}s")
                    time.sleep(cmd.seconds)
                    continue
                yaw = math.atan2(cmd.y - prev[1], cmd.x - prev[0])
                print(f"[nav2 {i + 1}/{len(commands)}] GOTO "
                      f"({cmd.x:.1f}, {cmd.y:.1f})")
                nav.goToPose(self._pose(nav, cmd.x, cmd.y, yaw))
                while not nav.isTaskComplete():
                    time.sleep(0.2)
                if nav.getResult() == TaskResult.SUCCEEDED:
                    reached += 1
                    prev = (cmd.x, cmd.y)
                else:
                    ok = False
                    detail = f"Nav2 failed at waypoint {reached + 1}/{total}"
                    break
        finally:
            nav.lifecycleShutdown()
            rclpy.shutdown()

        return BackendResult(
            backend=self.name, completed=ok, detail=detail,
            metrics={"waypoints_reached": float(reached),
                     "waypoints_commanded": float(total)},
        )
