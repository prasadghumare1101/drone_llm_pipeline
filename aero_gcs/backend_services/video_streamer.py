#!/usr/bin/env python3
"""Lightweight MJPEG streamer for the FPV panel.

Source is a ROS 2 camera topic from Gazebo (there is no webcam on a SITL rig).
Serves multipart/x-mixed-replace on :8080/video_feed so the browser can bind it
straight to an <img src> - frames never enter React state, which is what keeps
the dashboard's memory flat.

Zero-transcode path: image_transport already publishes JPEG on
<topic>/compressed, so those bytes are forwarded to the browser untouched. No
decode, no re-encode, near-zero CPU. The raw rgb8 topic is only used as a
fallback when the compressed topic is unavailable.

Config (env):
  AERO_CAMERA_TOPIC   base topic (default /front_camera/image_raw)
  AERO_FPS            max frames/sec pushed to the browser (default 15)
  AERO_JPEG_QUALITY   quality for the raw-topic fallback only (default 50)

Must run with ROS 2 sourced. Run:  python3 video_streamer.py
"""
from __future__ import annotations

import os
import threading
import time

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, Response
from fastapi.responses import StreamingResponse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

# AERO_CAMERA_TOPIC is only a PREFERENCE now. If it isn't present on the graph,
# the streamer auto-discovers a camera topic (so it works no matter what the
# Gazebo model actually names it). Set it to force a specific topic.
BASE_TOPIC = os.environ.get("AERO_CAMERA_TOPIC", "/front_camera/image_raw")
TARGET_FPS = float(os.environ.get("AERO_FPS", 15))
JPEG_QUALITY = int(os.environ.get("AERO_JPEG_QUALITY", 50))
STALE_AFTER_S = 2.0
WIDTH, HEIGHT = 640, 360

# Never pick these as the FPV feed (depth maps, point clouds, metadata, …).
_EXCLUDE = ("depth", "points", "camera_info", "theora", "disparity", "/info")


class LatestFrame:
    """Single-slot buffer. Old frames are dropped, never queued, so a slow
    browser can never make this grow."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._jpeg: bytes | None = None
        self._stamp = 0.0

    def set(self, jpeg: bytes) -> None:
        with self._lock:
            self._jpeg = jpeg
            self._stamp = time.monotonic()

    def get(self) -> bytes | None:
        with self._lock:
            if self._jpeg is None or time.monotonic() - self._stamp > STALE_AFTER_S:
                return None
            return self._jpeg


FRAME = LatestFrame()
NODE = None          # set once the ROS node is created (for /health reporting)


def _placeholder(msg: str = "WAITING FOR CAMERA") -> bytes:
    img = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
    img[:] = (28, 28, 32)
    cv2.putText(img, msg, (28, HEIGHT // 2 - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (120, 120, 130), 2)
    cv2.putText(img, BASE_TOPIC, (28, HEIGHT // 2 + 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (90, 90, 100), 1)
    ok, buf = cv2.imencode(".jpg", img)
    return buf.tobytes() if ok else b""


def _rank_topic(name: str) -> int:
    """Higher = more likely to be the FPV camera. Negative = never use."""
    n = name.lower()
    if any(x in n for x in _EXCLUDE):
        return -1
    score = 0
    if name == BASE_TOPIC or name == f"{BASE_TOPIC}/compressed":
        score += 100                      # honour the configured preference
    if "front" in n:
        score += 50
    if "camera" in n or "image" in n:
        score += 10
    if "downward" in n:
        score -= 20                       # prefer the forward camera
    return score


class CameraNode(Node):
    """Discovers a camera topic from the live ROS graph and streams it.

    The exact topic a Gazebo model publishes varies (namespaces, remappings),
    and it only appears AFTER the sim starts — so we keep scanning until we find
    one, subscribe to the best-ranked image topic (+ its /compressed sibling),
    and forward frames. No hardcoded topic name required.
    """

    def __init__(self) -> None:
        super().__init__("gcs_video_streamer")
        # Gazebo image_transport publishers are best-effort; a RELIABLE reader
        # would silently never match them.
        self._qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._subs: dict = {}             # topic -> subscription
        self._got_compressed = False
        self.active_topic: str | None = None
        self._discover()                  # try immediately
        self.create_timer(3.0, self._discover)   # then keep re-scanning

    # ---- topic discovery ------------------------------------------------- #
    def _discover(self) -> None:
        # Once a source is delivering fresh frames, stop hunting.
        if self.active_topic and FRAME.get() is not None:
            return
        raw_type = "sensor_msgs/msg/Image"
        cmp_type = "sensor_msgs/msg/CompressedImage"
        best_raw, best_raw_score = None, 0
        try:
            topics = self.get_topic_names_and_types()
        except Exception:
            return
        for name, types in topics:
            score = _rank_topic(name)
            if score <= 0:
                continue
            if raw_type in types and score > best_raw_score:
                best_raw, best_raw_score = name, score
            # a compressed topic already carries JPEG -> subscribe directly
            if cmp_type in types and name not in self._subs:
                self._sub_compressed(name)
        if best_raw and best_raw not in self._subs:
            self._sub_raw(best_raw)
            # its image_transport sibling, if any
            comp = f"{best_raw}/compressed"
            if comp not in self._subs:
                self._sub_compressed(comp)

    def _sub_raw(self, topic: str) -> None:
        self._subs[topic] = self.create_subscription(
            Image, topic, lambda m, t=topic: self._on_raw(t, m), self._qos)
        self.get_logger().info(f"video: subscribed raw {topic}")

    def _sub_compressed(self, topic: str) -> None:
        self._subs[topic] = self.create_subscription(
            CompressedImage, topic, lambda m, t=topic: self._on_compressed(t, m),
            self._qos)
        self.get_logger().info(f"video: subscribed compressed {topic}")

    # ---- frame callbacks ------------------------------------------------- #
    def _on_compressed(self, topic: str, msg: CompressedImage) -> None:
        if "jpeg" in msg.format.lower() or "jpg" in msg.format.lower():
            self._got_compressed = True
            self.active_topic = topic
            FRAME.set(bytes(msg.data))

    def _on_raw(self, topic: str, msg: Image) -> None:
        if self._got_compressed:          # prefer the zero-transcode path
            return
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, -1)
            if msg.encoding == "rgb8":
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            ok, buf = cv2.imencode(
                ".jpg", arr, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            if ok:
                self.active_topic = topic
                FRAME.set(buf.tobytes())
        except (ValueError, cv2.error):
            pass


app = FastAPI(title="AEROMAST Video Streamer")


def generate_frames():
    period = 1.0 / TARGET_FPS
    blank = _placeholder()
    while True:
        start = time.monotonic()
        jpeg = FRAME.get() or blank
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
        time.sleep(max(0.0, period - (time.monotonic() - start)))


@app.get("/video_feed")
def video_feed() -> StreamingResponse:
    return StreamingResponse(
        generate_frames(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/snapshot")
def snapshot() -> Response:
    """Single current frame. Used by the CAPTURE mission command - one cheap
    GET beats holding a second image subscription open in the bridge."""
    jpeg = FRAME.get()
    if jpeg is None:
        return Response(content=_placeholder("NO FRAME"), media_type="image/jpeg",
                        status_code=503)
    return Response(content=jpeg, media_type="image/jpeg")


@app.get("/health")
def health() -> dict:
    active = getattr(NODE, "active_topic", None)
    subs = list(getattr(NODE, "_subs", {}).keys())
    return {"ok": True, "preferred": BASE_TOPIC, "active_topic": active,
            "subscribed": subs, "receiving": FRAME.get() is not None}


def _spin_ros() -> None:
    global NODE
    rclpy.init()
    node = CameraNode()
    NODE = node
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    threading.Thread(target=_spin_ros, daemon=True).start()
    print(f"[INFO] Video streamer on :8080  (source {BASE_TOPIC})")
    uvicorn.run(app, host="0.0.0.0", port=8080, log_level="warning")
