#!/usr/bin/env python3
"""Consolidated telemetry publisher for the AEROMAST GCS.

Subscribes to the PX4 uXRCE-DDS output topics and republishes ONE compact JSON
document on /gcs/consolidated_telemetry (std_msgs/String) at 10 Hz.

Why one consolidated topic: rosbridge serialises every topic separately over the
websocket. Five topics at 50 Hz would flood the browser and leak memory in the
React layer. One 10 Hz topic = one setState per tick in the UI.

QoS + topic naming follow the same rules the px4_dds_bridge learned the hard way:
  * subscribers MUST be BEST_EFFORT (PX4 publishes best-effort; a RELIABLE
    reader silently never matches),
  * PX4 >= v1.15 renames /fmu/out topics with a _v1 suffix, so subscribe to both
    variants and let the one PX4 actually publishes win.

Run:  python3 telemetry_node.py
"""
from __future__ import annotations

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String

from px4_msgs.msg import (
    BatteryStatus,
    SensorGps,
    VehicleGlobalPosition,
    VehicleLocalPosition,
    VehicleStatus,
)

PUBLISH_HZ = 10.0

NAV_STATE_NAMES = {
    0: "MANUAL", 1: "ALTITUDE", 2: "POSITION", 3: "AUTO-MISSION",
    4: "AUTO-LOITER", 5: "AUTO-RTL", 6: "POSITION-SLOW", 10: "ACRO",
    12: "DESCEND", 13: "TERMINATION", 14: "OFFBOARD", 15: "STABILIZED",
    17: "AUTO-TAKEOFF", 18: "AUTO-LAND", 19: "AUTO-FOLLOW",
    20: "AUTO-PRECLAND", 21: "ORBIT", 22: "AUTO-VTOL-TAKEOFF",
}


def _sub_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )


class TelemetryNode(Node):
    def __init__(self) -> None:
        super().__init__("gcs_telemetry_node")
        qos = _sub_qos()

        # state
        self.x = self.y = self.z = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.heading = 0.0
        self.pos_valid = False
        self.armed = False
        self.nav_state = -1
        self.battery = 0.0
        self.voltage = 0.0
        self.temp = 0.0
        self.minutes = 0
        self.lat = 0.0
        self.lon = 0.0
        self.sats = 0

        for base, msg_type, cb in (
            ("vehicle_local_position", VehicleLocalPosition, self._on_local),
            ("vehicle_status", VehicleStatus, self._on_status),
            ("battery_status", BatteryStatus, self._on_battery),
            ("vehicle_global_position", VehicleGlobalPosition, self._on_global),
            ("vehicle_gps_position", SensorGps, self._on_gps),
        ):
            for topic in (f"/fmu/out/{base}", f"/fmu/out/{base}_v1"):
                self.create_subscription(msg_type, topic, cb, qos)

        self.pub = self.create_publisher(String, "/gcs/consolidated_telemetry", 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._tick)
        self.get_logger().info(
            "GCS telemetry node up -> /gcs/consolidated_telemetry @ %.0f Hz" % PUBLISH_HZ)

    # ------------------------------ callbacks ------------------------------ #

    def _on_local(self, m: VehicleLocalPosition) -> None:
        self.pos_valid = bool(m.xy_valid and m.z_valid)
        if self.pos_valid:
            self.x, self.y, self.z = m.x, m.y, m.z
            self.vx, self.vy, self.vz = m.vx, m.vy, m.vz
            self.heading = math.degrees(m.heading) % 360.0

    def _on_status(self, m: VehicleStatus) -> None:
        self.armed = (m.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.nav_state = int(m.nav_state)

    def _on_battery(self, m: BatteryStatus) -> None:
        if m.remaining >= 0.0:
            self.battery = float(m.remaining) * 100.0
        self.voltage = float(m.voltage_v)
        if not math.isnan(m.temperature):
            self.temp = float(m.temperature)
        if not math.isnan(m.time_remaining_s) and m.time_remaining_s > 0:
            self.minutes = int(m.time_remaining_s / 60.0)

    def _on_global(self, m: VehicleGlobalPosition) -> None:
        self.lat, self.lon = float(m.lat), float(m.lon)

    def _on_gps(self, m: SensorGps) -> None:
        self.sats = int(m.satellites_used)

    # -------------------------------- tick --------------------------------- #

    def _tick(self) -> None:
        # NED -> ENU for the map: east = y, north = x, up = -z
        speed_kmh = math.sqrt(self.vx ** 2 + self.vy ** 2) * 3.6
        mode = NAV_STATE_NAMES.get(self.nav_state, "UNKNOWN")
        if not self.armed:
            mode = f"{mode} (DISARMED)"

        payload = {
            "altitude": round(-self.z, 2),
            "heading": round(self.heading, 1),
            "speed": round(speed_kmh, 1),
            "climb_rate": round(-self.vz, 2),
            "battery": round(self.battery, 1),
            "battery_voltage": round(self.voltage, 2),
            "battery_temp": round(self.temp, 1),
            "battery_minutes": self.minutes,
            # SITL has no real radio; report link health as full while data flows.
            "signal_rc": 95 if self.pos_valid else 0,
            "signal_fpv": 92 if self.pos_valid else 0,
            "lat": round(self.lat, 6),
            "lon": round(self.lon, 6),
            "satellites": self.sats,
            "gps_ok": self.sats >= 6,
            "armed": self.armed,
            "flight_mode": mode,
            "x": round(self.y, 2),   # ENU east
            "y": round(self.x, 2),   # ENU north
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
