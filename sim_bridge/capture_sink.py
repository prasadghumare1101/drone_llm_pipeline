"""Geotagged still capture for the CAPTURE mission command.

Pulls one JPEG from the video streamer's /snapshot endpoint and writes it next
to the run's other artifacts, together with an index recording where each frame
was taken. That index is what turns a survey flight into usable field data.

Design notes:
  * Fetches happen on a worker thread. The PX4 bridge must keep publishing
    setpoints at 20 Hz - a blocking HTTP GET inside the tick loop would break
    the offboard heartbeat and drop the vehicle out of OFFBOARD.
  * The queue is bounded. If the streamer stalls, captures are dropped and
    counted rather than accumulating into unbounded memory.
  * No ROS image subscription here: one cheap GET reuses the stream the
    dashboard is already decoding.
"""
from __future__ import annotations

import json
import queue
import threading
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional, Tuple

SNAPSHOT_URL = "http://localhost:8080/snapshot"
QUEUE_MAX = 32
FETCH_TIMEOUT_S = 2.0


class CaptureSink:
    """Threaded, bounded, best-effort still writer."""

    def __init__(self, out_dir: Optional[Path], url: str = SNAPSHOT_URL):
        self.dir = Path(out_dir) / "captures" if out_dir else None
        self.url = url
        self.saved = 0
        self.dropped = 0
        self.failed = 0
        self._index: list = []
        self._q: "queue.Queue[tuple]" = queue.Queue(maxsize=QUEUE_MAX)
        self._stop = threading.Event()
        self._worker: Optional[threading.Thread] = None
        if self.dir:
            self.dir.mkdir(parents=True, exist_ok=True)
            self._worker = threading.Thread(target=self._run, daemon=True)
            self._worker.start()

    # ------------------------------------------------------------------ #

    def request(self, label: str, pos_ned: Optional[Tuple[float, float, float]],
                lat: float = 0.0, lon: float = 0.0) -> None:
        """Queue a capture. Never blocks the caller's control loop."""
        if not self.dir:
            return
        try:
            self._q.put_nowait((label, pos_ned, lat, lon,
                                datetime.now(timezone.utc)))
        except queue.Full:
            self.dropped += 1

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                label, pos, lat, lon, stamp = self._q.get(timeout=0.25)
            except queue.Empty:
                continue
            try:
                with urllib.request.urlopen(self.url, timeout=FETCH_TIMEOUT_S) as r:
                    if r.status != 200:
                        self.failed += 1
                        continue
                    data = r.read()
            except (urllib.error.URLError, OSError, TimeoutError):
                self.failed += 1
                continue

            self.saved += 1
            n, e, d = pos if pos else (0.0, 0.0, 0.0)
            name = (f"{self.saved:04d}_{label or 'wp'}"
                    f"_n{n:+.1f}_e{e:+.1f}_alt{-d:.1f}.jpg")
            try:
                (self.dir / name).write_bytes(data)
            except OSError:
                self.failed += 1
                self.saved -= 1
                continue
            self._index.append({
                "file": name, "label": label,
                "utc": stamp.isoformat(),
                "north_m": round(n, 2), "east_m": round(e, 2),
                "alt_m": round(-d, 2), "lat": lat, "lon": lon,
            })

    def close(self) -> None:
        """Drain briefly, then write the geotag index."""
        if not self.dir:
            return
        self._q.join() if False else None      # bounded wait below instead
        self._stop.wait(0.1)
        deadline = 5.0
        while not self._q.empty() and deadline > 0:
            self._stop.wait(0.2)
            deadline -= 0.2
        self._stop.set()
        if self._worker:
            self._worker.join(timeout=3.0)
        try:
            (self.dir / "index.json").write_text(
                json.dumps({"captures": self._index,
                            "saved": self.saved,
                            "dropped": self.dropped,
                            "failed": self.failed}, indent=2),
                encoding="utf-8")
        except OSError:
            pass
