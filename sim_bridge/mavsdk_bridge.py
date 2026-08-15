"""MAVSDK-Python bridge into PX4 SITL (ROS-free alternative to the DDS bridge).

Connects over MAVLink UDP (SITL default udp://:14540) and drives the compiled
command sequence in OFFBOARD mode using PositionNedYaw setpoints. MAVSDK keeps
re-sending the last setpoint in the background, which satisfies PX4's offboard
heartbeat requirement.

Install on the target machine:  pip3 install mavsdk
Reviewed but not CI-tested in this sandbox (no PX4 here).

Note: ArduPilot targets (e.g. a SpeedyBee F405 stack) would use pymavlink /
GUIDED mode instead — out of scope here, hook point documented in README.
"""
from __future__ import annotations

import asyncio
import math
from typing import Sequence

from executor import (
    Arm,
    Disarm,
    ExecutableCommand,
    Goto,
    Hold,
    Land,
    ReturnToLaunch,
    Takeoff,
)

from .base_backend import BackendResult, MissionBackend

ACCEPT_RADIUS_M = 0.8
SETTLE_POLL_S = 0.2
CMD_TIMEOUT_S = 180.0


class MavsdkBackend(MissionBackend):
    name = "mavsdk"

    def __init__(self, system_address: str = "udp://:14540"):
        try:
            import mavsdk  # noqa: F401
        except ImportError as e:
            raise RuntimeError(
                "MAVSDK backend requires the 'mavsdk' package "
                "(pip3 install mavsdk) and a running PX4 SITL."
            ) from e
        self.system_address = system_address

    def _run_checked(self, commands: Sequence[ExecutableCommand]) -> BackendResult:
        return asyncio.run(self._fly(commands))

    # ------------------------------------------------------------------ #

    async def _fly(self, commands: Sequence[ExecutableCommand]) -> BackendResult:
        from mavsdk import System
        from mavsdk.offboard import OffboardError, PositionNedYaw

        drone = System()
        await drone.connect(system_address=self.system_address)

        print(f"[mavsdk] waiting for connection on {self.system_address} …")
        async for state in drone.core.connection_state():
            if state.is_connected:
                break
        print("[mavsdk] connected; waiting for position health …")
        async for health in drone.telemetry.health():
            if health.is_local_position_ok and health.is_home_position_ok:
                break

        async def pos_ned():
            async for pv in drone.telemetry.position_velocity_ned():
                return (pv.position.north_m, pv.position.east_m, pv.position.down_m)

        async def wait_reach(target, timeout=CMD_TIMEOUT_S) -> bool:
            waited = 0.0
            while waited < timeout:
                n, e, d = await pos_ned()
                if math.dist((n, e, d), target) < ACCEPT_RADIUS_M:
                    return True
                await asyncio.sleep(SETTLE_POLL_S)
                waited += SETTLE_POLL_S
            return False

        reached = 0
        total = sum(isinstance(c, Goto) for c in commands)
        offboard_started = False
        ok = True
        detail = "all commands executed"

        try:
            for i, cmd in enumerate(commands):
                tag = f"[mavsdk {i + 1}/{len(commands)}] {type(cmd).__name__}"
                print(tag)

                if isinstance(cmd, Arm):
                    await drone.action.arm()
                    n, e, d = await pos_ned()
                    await drone.offboard.set_position_ned(
                        PositionNedYaw(n, e, d, 0.0))
                    try:
                        await drone.offboard.start()
                        offboard_started = True
                    except OffboardError as err:
                        ok, detail = False, f"offboard start failed: {err._result.result}"
                        break

                elif isinstance(cmd, Takeoff):
                    n, e, _ = await pos_ned()
                    target = (n, e, -cmd.alt)
                    await drone.offboard.set_position_ned(
                        PositionNedYaw(*target, 0.0))
                    if not await wait_reach(target):
                        ok, detail = False, "takeoff altitude not reached"
                        break

                elif isinstance(cmd, Goto):
                    # ENU (x=E, y=N, alt up) -> NED
                    target = (cmd.y, cmd.x, -cmd.alt)
                    n, e, _ = await pos_ned()
                    yaw_deg = math.degrees(
                        math.atan2(target[1] - e, target[0] - n))
                    await drone.offboard.set_position_ned(
                        PositionNedYaw(*target, yaw_deg))
                    if await wait_reach(target):
                        reached += 1
                    else:
                        ok, detail = False, f"waypoint {reached + 1}/{total} not reached"
                        break

                elif isinstance(cmd, Hold):
                    await asyncio.sleep(cmd.seconds)

                elif isinstance(cmd, Land):
                    if offboard_started:
                        await drone.offboard.stop()
                        offboard_started = False
                    await drone.action.land()
                    async for in_air in drone.telemetry.in_air():
                        if not in_air:
                            break

                elif isinstance(cmd, ReturnToLaunch):
                    if offboard_started:
                        await drone.offboard.stop()
                        offboard_started = False
                    await drone.action.return_to_launch()
                    async for in_air in drone.telemetry.in_air():
                        if not in_air:
                            break

                elif isinstance(cmd, Disarm):
                    waited = 0.0
                    async for armed in drone.telemetry.armed():
                        if not armed or waited > 30.0:
                            break
                        await asyncio.sleep(0.5)
                        waited += 0.5
        finally:
            if offboard_started:
                try:
                    await drone.offboard.stop()
                except Exception:
                    pass

        return BackendResult(
            backend=self.name, completed=ok,
            detail=detail if ok else f"FAILED: {detail}",
            metrics={"waypoints_reached": float(reached),
                     "waypoints_commanded": float(total)},
        )
