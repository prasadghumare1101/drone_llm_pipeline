#!/usr/bin/env python3
"""AEROMAST control API - the service that replaces the five terminals.

Owns two things and nothing else:
  1. process lifecycle for the simulation stack (uXRCE-DDS agent, PX4 SITL,
     rosbridge, telemetry node, video streamer),
  2. mission dispatch - takes a natural-language prompt from the dashboard and
     runs run_pipeline.py (LLM -> validator -> executor -> PX4) as a subprocess.

It deliberately does NOT import rclpy or the pipeline packages: keeping this a
pure supervisor means a crash in a child process can never take the API down.

Run:  python3 control_api.py          (listens on :8000)
"""
from __future__ import annotations

import glob
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Dict, List, Optional

import uvicorn
from fastapi import FastAPI, Request
from fastapi.responses import FileResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# --------------------------------------------------------------------------- #
# Paths / configuration
# --------------------------------------------------------------------------- #

PIPELINE_ROOT = Path(__file__).resolve().parents[2]      # ~/drone_llm_pipeline
# Linux gives us /proc (process environments) and `ss`; macOS does not. The few
# features that need them have a fallback or are Linux-only (see GUI service).
LINUX = Path("/proc/self/environ").exists()


def _load_key_files() -> None:
    """LLM keys (GROQ_API_KEY, HF_TOKEN, ...) from ~/.config/drone_llm/*.env, so
    the API needs no special launch command. Never overrides the environment."""
    d = Path.home() / ".config" / "drone_llm"
    for f in sorted(d.glob("*.env")) if d.is_dir() else []:
        for line in f.read_text(encoding="utf-8", errors="replace").splitlines():
            k, sep, v = line.strip().partition("=")
            if sep and k and not k.startswith("#"):
                os.environ.setdefault(k.strip(), v.strip().strip("'\""))


_load_key_files()
SERVICES_DIR = Path(__file__).resolve().parent
PX4_DIR = Path(os.environ.get("PX4_DIR", Path.home() / "PX4-Autopilot"))
ROS_SETUP = os.environ.get("ROS_SETUP", "/opt/ros/humble/setup.bash")
# colcon workspace with px4_msgs (built from YOUR PX4 checkout's msg/ folder)
WS_SETUP = Path(os.environ.get("AERO_WS_SETUP", PIPELINE_ROOT / "install" / "setup.bash"))

# PX4 gazebo-classic default home (Zurich). Override with PX4_HOME_LAT/LON.
HOME_LAT = float(os.environ.get("PX4_HOME_LAT", 47.397742))
HOME_LON = float(os.environ.get("PX4_HOME_LON", 8.545594))

LOG_LINES = 400
LOG_DIR = Path(__file__).resolve().parent / "logs"

WORLDS_DIR = (PX4_DIR / "Tools" / "simulation" / "gazebo-classic"
              / "sitl_gazebo-classic" / "worlds")
MODELS_DIR = (PX4_DIR / "Tools" / "simulation" / "gazebo-classic"
              / "sitl_gazebo-classic" / "models")

# Selected airframe + world. "empty" is the fast default.
SIM_CONFIG: Dict[str, str] = {
    "model": os.environ.get("PX4_MODEL", "iris"),
    "world": os.environ.get("PX4_WORLD", "empty"),
}

# Gazebo hangs for minutes (or forever) trying to pull meshes from its online
# model database when a heavy world - baylands, ksql_airport, sonoma_raceway -
# references a model that is not cached locally. That fetch is the usual cause
# of a "stuck" launch. Everything PX4 needs already ships in the repo, so the
# database is disabled and the local paths are pinned instead.
SITL_ENV: Dict[str, str] = {
    "GAZEBO_MODEL_DATABASE_URI": "",
    "GAZEBO_MODEL_PATH": f"{MODELS_DIR}:{Path.home() / '.gazebo' / 'models'}",
    # CRITICAL: run px4 as `px4 -d` (no interactive pxh shell). Without this the
    # pxh prompt redraws itself forever when stdin is not a TTY, writing ANSI
    # escape codes at hundreds of MB/min. That burns CPU and fills the disk,
    # which is what actually starves Gazebo into "not responding".
    "NO_PXH": "1",
    # Drop the gzclient follow-camera plugin: less GUI load, fewer stalls.
    "PX4_NO_FOLLOW_MODE": "1",
    # Do NOT let sitl_run.sh start the Gazebo window: it launches it as
    # `nice -n 20 gzclient`, and at that priority the window cannot drain its
    # updates fast enough - Gazebo's send queue overflows ("Queue limit reached
    # for topic .../user_camera/pose"), the window loses the master and freezes
    # while the sim keeps running. The "gui" service below starts it instead,
    # at normal priority, and _gui_watchdog() restarts it if it ever freezes.
    "HEADLESS": "1",
}
if not LINUX:
    # the managed window copies gzserver's environment from /proc; elsewhere
    # PX4's own launcher opens the Gazebo window as usual
    SITL_ENV.pop("HEADLESS")

# Hard ceiling on any captured service log. Defence in depth for the runaway
# case above - a log can never be allowed to consume the disk.
MAX_LOG_BYTES = 20 * 1024 * 1024

# PX4 SITL parameters the project depends on. Applied by code at every START SIM
# so every machine's sim behaves the same (they used to be set by hand once).
SITL_PARAMS = {
    "COM_RCL_EXCEPT": 4,      # SITL has no RC: don't treat that as RC loss in OFFBOARD
    "SYS_FAILURE_EN": 1,      # allow the dashboard's FAULTS panel
    "SIM_BAT_DRAIN": 2400,    # realistic battery: 100 -> 0 % in 40 min (2.5 %/min) ...
    "SIM_BAT_MIN_PCT": 0,     # ... draining continuously, like a real pack
    "SDLOG_MODE": 0,          # log only while armed (default logs from boot)
    "SDLOG_PROFILE": 1,       # default topics; SITL's high-rate profile wrote ~1 GB/h
    "SDLOG_DIRS_MAX": 3,      # keep 3 days of sim logs
}

# Heavy worlds need far longer than `empty` before the DDS link appears.
SITL_READY_TIMEOUT_S = float(os.environ.get("PX4_READY_TIMEOUT", 180))


def _sitl_target() -> str:
    """PX4 make target. `model__world` (double underscore) selects a world."""
    model = SIM_CONFIG["model"]
    world = SIM_CONFIG["world"]
    if not world or world in ("default", "empty"):
        return f"gazebo-classic_{model}"
    if not (WORLDS_DIR / f"{world}.world").is_file():
        # PX4's CMake only generates targets for its own worlds; others (e.g.
        # built by world_builder.py) launch the bare target + PX4_SITL_WORLD.
        return f"gazebo-classic_{model}"
    return f"gazebo-classic_{model}__{world}"


# Worlds built from real locations by world_builder.py (launched by path).
GENERATED_WORLDS = SERVICES_DIR.parent / "sim_worlds"
WORLD_OVERRIDES = Path.home() / "ws_offboard_control" / "sitl_worlds"


def _list_worlds() -> List[str]:
    names = set()
    for d in (WORLDS_DIR, WORLD_OVERRIDES, GENERATED_WORLDS):
        if d.is_dir():
            names.update(p.stem for p in d.glob("*.world") if not p.stem.endswith("__env"))
    return sorted(names) or ["empty"]


def _list_models() -> List[str]:
    if not MODELS_DIR.is_dir():
        return ["iris"]
    return sorted(p.name for p in MODELS_DIR.iterdir()
                  if p.is_dir() and (p / "model.config").is_file())


def _sourced(cmd: str, with_ws: bool = False) -> List[str]:
    """Wrap a command so it runs inside a properly sourced ROS 2 shell."""
    parts = [f"source {ROS_SETUP}"]
    if with_ws and WS_SETUP.exists():
        parts.append(f"source {WS_SETUP}")
    parts.append(f"exec {cmd}")
    return ["bash", "-c", " && ".join(parts)]


# --------------------------------------------------------------------------- #
# Managed service definitions
# --------------------------------------------------------------------------- #

class Service:
    def __init__(self, name: str, argv: List[str], cwd: Path, detail: str,
                 env: Optional[Dict[str, str]] = None):
        self.name = name
        self.argv = argv
        self.cwd = cwd
        self.detail = detail
        self.env = env
        self.proc: Optional[subprocess.Popen] = None
        self.log_path: Optional[Path] = None
        self._log_fh = None

    @property
    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def start(self) -> None:
        if self.running:
            return
        merged = None
        if self.env:
            merged = {**os.environ, **self.env}
        # Capture output to a file. Discarding it (DEVNULL) makes a failed or
        # hung launch completely undiagnosable from the dashboard.
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        self.log_path = LOG_DIR / f"{self.name}.log"
        if self.log_path.is_file() and self.log_path.stat().st_size:
            # keep the previous run's log: after a crash + automatic restart it
            # is the only evidence of what went wrong
            self.log_path.replace(self.log_path.with_suffix(".log.prev"))
        self._log_fh = self.log_path.open("w", buffering=1, encoding="utf-8",
                                          errors="replace")
        try:
            self.proc = subprocess.Popen(
                self.argv,
                cwd=str(self.cwd),
                env=merged,
                stdout=self._log_fh,
                stderr=subprocess.STDOUT,
                start_new_session=True,      # own process group -> killable as a tree
            )
        except (FileNotFoundError, NotADirectoryError) as e:
            self._log_fh.write(f"[control_api] cannot start {self.name}: {e}\n")
            self._log_fh.close()
            self._log_fh, self.proc = None, None
            if "not installed" not in self.detail:
                self.detail += f" (not installed: {self.argv[0]})"

    def tail(self, lines: int = 40) -> str:
        if not self.log_path or not self.log_path.is_file():
            return "(no log yet)"
        try:
            return "".join(self.log_path.read_text(
                encoding="utf-8", errors="replace").splitlines(keepends=True)[-lines:])
        except OSError as e:
            return f"(log unreadable: {e})"

    def stop(self) -> None:
        if not self.running or self.proc is None:
            self.proc = None
            return
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            try:
                self.proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                self.proc.wait(timeout=5)
        except (ProcessLookupError, PermissionError):
            pass
        finally:
            self.proc = None
            if self._log_fh:
                try:
                    self._log_fh.close()
                except OSError:
                    pass
                self._log_fh = None


SERVICES: Dict[str, Service] = {
    "agent": Service(
        "agent", ["MicroXRCEAgent", "udp4", "-p", "8888"], PIPELINE_ROOT,
        "udp4 :8888"),
    # argv/detail are rebuilt from SIM_CONFIG on every start (see _apply_sim_config)
    "sitl": Service(
        "sitl", ["bash", "-c", "exec make px4_sitl gazebo-classic_iris"], PX4_DIR,
        "iris / empty", env=SITL_ENV),
    "rosbridge": Service(
        "rosbridge",
        _sourced("ros2 launch rosbridge_server rosbridge_websocket_launch.xml"),
        PIPELINE_ROOT, "websocket :9090"),
    "telemetry": Service(
        "telemetry",
        _sourced(f"python3 {SERVICES_DIR / 'telemetry_node.py'}", with_ws=True),
        PIPELINE_ROOT, "/gcs/consolidated_telemetry"),
    # Gazebo window. argv is fixed; env is copied from the running gzserver at
    # start (see _prepare_gui) so it sees exactly the same models and plugins.
    "gui": Service(
        "gui", ["gzclient", "--verbose"], Path.home(), "Gazebo window"),
    "video": Service(
        "video",
        # needs rclpy: the FPV feed comes from a Gazebo ROS camera topic
        _sourced(f"python3 {SERVICES_DIR / 'video_streamer.py'}"),
        SERVICES_DIR, "MJPEG :8080"),
}

# Order matters: agent before SITL so the DDS link comes up cleanly.
START_ORDER = ["agent", "sitl", "gui", "rosbridge", "telemetry", "video"]
if not LINUX:
    START_ORDER.remove("gui")
    SERVICES["gui"].detail = "Gazebo window (opened by PX4 on this OS)"


# --------------------------------------------------------------------------- #
# Pre-flight cleanup
#
# A stale gzserver still holding the old world is the usual cause of the
# "simulation is not responding / force quit" dialog, and of the laggy second
# run. Every start therefore reaps orphans and clears ONLY Gazebo scratch.
#
# NEVER TOUCHED:
#   ~/.gazebo/models            - contains the hand-modified `iris` model
#   PX4 .../sitl_gazebo-classic/models
#   PX4 build output
#   MicroXRCEAgent on a SERIAL port - that is the real drone hardware link
#
# Log purging destroys history, so it is a separate endpoint the operator must
# confirm explicitly. It never runs as part of starting the simulation.
# --------------------------------------------------------------------------- #

PROTECTED_PATHS = (
    Path.home() / ".gazebo" / "models",
    PX4_DIR / "Tools" / "simulation" / "gazebo-classic" / "sitl_gazebo-classic" / "models",
    PX4_DIR / "build",
)

# Reaped on start. "MicroXRCEAgent udp4" is matched in full ON PURPOSE: the bare
# binary name would also match `MicroXRCEAgent serial --dev /dev/ttyUSB0`, which
# is the physical drone link and must survive.
ORPHAN_PATTERNS = ("gzserver", "gzclient", "px4", "MicroXRCEAgent udp4")

SCRATCH_GLOBS = ("/tmp/gazebo_*", f"/tmp/hsperfdata_{os.environ.get('USER', 'nobody')}")


def _is_protected(path: Path) -> bool:
    """True if the path is, contains, or lives inside anything protected."""
    try:
        rp = path.resolve()
    except OSError:
        return True                       # unresolvable -> refuse to touch it
    for guard in PROTECTED_PATHS:
        try:
            g = guard.resolve()
        except OSError:
            continue
        if rp == g or g in rp.parents or rp in g.parents:
            return True
    return False


def _reap_orphans() -> List[str]:
    """SIGTERM, then SIGKILL, leftover simulator processes. Returns what died."""
    killed: List[str] = []
    for pattern in ORPHAN_PATTERNS:
        try:
            out = subprocess.run(["pgrep", "-f", pattern],
                                 capture_output=True, text=True, timeout=5)
        except (subprocess.SubprocessError, FileNotFoundError):
            continue
        for pid_s in out.stdout.split():
            try:
                pid = int(pid_s)
            except ValueError:
                continue
            if pid == os.getpid():
                continue
            for sig in (signal.SIGTERM, signal.SIGKILL):
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    break                 # already gone
                except PermissionError:
                    break                 # not ours -> leave it alone
                time.sleep(0.4)
                try:
                    os.kill(pid, 0)
                except ProcessLookupError:
                    break
            killed.append(f"{pattern}:{pid}")
    return killed


def _clean_scratch() -> List[str]:
    """Remove Gazebo/JVM scratch only. Refuses symlinks, files owned by someone
    else, and anything inside a protected path."""
    removed: List[str] = []
    for pattern in SCRATCH_GLOBS:
        for hit in glob.glob(pattern):
            p = Path(hit)
            if p.is_symlink() or _is_protected(p):
                continue
            try:
                if p.stat().st_uid != os.getuid():
                    continue              # never force-remove another user's files
                if p.is_dir():
                    shutil.rmtree(p, ignore_errors=True)
                else:
                    p.unlink()
                removed.append(str(p))
            except OSError:
                continue
    return removed


# --------------------------------------------------------------------------- #
# Mission state
# --------------------------------------------------------------------------- #

class MissionState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.proc: Optional[subprocess.Popen] = None
        self.log: List[str] = []
        self.waypoints: List[dict] = []
        self.llm_status = "NO MISSION PLAN LOADED"
        self.active_cmd = -1
        self.total_cmd = 0

    @property
    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def append(self, line: str) -> None:
        with self.lock:
            self.log.append(line.rstrip())
            if len(self.log) > LOG_LINES:
                self.log = self.log[-LOG_LINES:]


MISSION = MissionState()


def _enu_to_latlon(east: float, north: float) -> tuple:
    home_lat, home_lon = _home()
    lat = home_lat + (north / 111_320.0)
    lon = home_lon + (east / (111_320.0 * math.cos(math.radians(home_lat))))
    return round(lat, 6), round(lon, 6)


def _latest_run_dir() -> Optional[Path]:
    runs = PIPELINE_ROOT / "runs"
    if not runs.is_dir():
        return None
    dirs = sorted((d for d in runs.iterdir() if d.is_dir()), key=lambda d: d.name)
    return dirs[-1] if dirs else None


def _load_waypoints() -> List[dict]:
    """Turn the newest compiled_commands.json into UI/map waypoints."""
    run_dir = _latest_run_dir()
    if not run_dir:
        return []
    f = run_dir / "compiled_commands.json"
    if not f.is_file():
        return []
    try:
        cmds = json.loads(f.read_text(encoding="utf-8")).get("commands", [])
    except (json.JSONDecodeError, OSError):
        return []

    ACTIONS = {
        "Takeoff": "Climb / Hold", "Goto": "Transit, Record",
        "Hold": "Hold, Pan 360", "Land": "Descend / Land",
        "ReturnToLaunch": "Return Home", "Arm": "Arm", "Disarm": "Disarm",
    }

    positional: List[dict] = []
    x = y = 0.0
    for ci, c in enumerate(cmds):
        kind = c.get("cmd")
        if kind == "Takeoff":
            alt, speed = float(c.get("alt", 0.0)), 0.0
        elif kind == "Goto":
            x, y = float(c.get("x", 0.0)), float(c.get("y", 0.0))
            alt, speed = float(c.get("alt", 0.0)), float(c.get("speed_mps", 0.0))
        else:
            continue
        lat, lon = _enu_to_latlon(x, y)
        positional.append({
            "cmd_index": ci, "x": x, "y": y, "alt": alt,
            "lat": lat, "lon": lon, "speed": round(speed * 3.6, 1),
            "action": ACTIONS.get(kind, kind),
        })

    # Energy remaining is a PROJECTION along path length, not a measurement.
    total = 0.0
    prev = (0.0, 0.0)
    for p in positional:
        total += math.dist(prev, (p["x"], p["y"]))
        prev = (p["x"], p["y"])
    cum = 0.0
    prev = (0.0, 0.0)
    n = len(positional)
    for i, p in enumerate(positional):
        cum += math.dist(prev, (p["x"], p["y"]))
        prev = (p["x"], p["y"])
        p["idx"] = i + 1
        p["total"] = n
        p["energy"] = int(round(100 - 40 * (cum / total if total else 0)))
        p["active"] = False
    return positional


def _mark_active() -> None:
    """Highlight the waypoint matching the executor's current command index."""
    for w in MISSION.waypoints:
        w["active"] = (w["cmd_index"] == MISSION.active_cmd)


_PROGRESS = re.compile(r"\[(\d+)/(\d+)\]")


def _run_mission(prompt: str) -> None:
    """Run run_pipeline.py and stream its output into the mission log."""
    argv = _sourced(
        "python3 run_pipeline.py "
        f"--prompt {json.dumps(prompt)} --backend px4 "
        f"--llm {os.environ.get('AERO_LLM', 'auto')}",
        with_ws=True,
    )
    MISSION.append(f"$ mission: {prompt}")
    proc = subprocess.Popen(
        argv, cwd=str(PIPELINE_ROOT),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1, start_new_session=True,
    )
    MISSION.proc = proc

    assert proc.stdout is not None
    for line in proc.stdout:
        MISSION.append(line)
        if "compiling to executable" in line or "commands, digest" in line:
            MISSION.waypoints = _load_waypoints()
            MISSION.llm_status = "LLM-UPLOADED MISSION PLAN CONFIRMED (REV 3.1)"
        m = _PROGRESS.search(line)
        if m:
            MISSION.active_cmd = int(m.group(1)) - 1
            MISSION.total_cmd = int(m.group(2))
            _mark_active()
        if "MISSION COMPLETED" in line:
            MISSION.llm_status = "MISSION COMPLETED"
        elif "MISSION FAILED" in line:
            MISSION.llm_status = "MISSION FAILED - see log"
        elif "REJECTED" in line:
            MISSION.llm_status = "PLAN REJECTED BY VALIDATOR"

    proc.wait()
    MISSION.active_cmd = -1
    _mark_active()
    MISSION.append(f"[exit {proc.returncode}]")


# --------------------------------------------------------------------------- #
# API
# --------------------------------------------------------------------------- #

@asynccontextmanager
async def _lifespan(_app: FastAPI):
    threading.Thread(target=_gui_watchdog, daemon=True, name="gui-watchdog").start()
    yield
    # never leave orphaned SITL/gazebo processes behind on API shutdown
    SESSION_SVC.stop()
    for name in reversed(START_ORDER):
        SERVICES[name].stop()


app = FastAPI(title="AEROMAST Control API", lifespan=_lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],          # LAN-local GCS; tighten if exposed
    allow_methods=["*"],
    allow_headers=["*"],
)


class MissionRequest(BaseModel):
    prompt: str


def _cap_logs() -> None:
    """Truncate any service log that has run away. Cheap, and it guarantees a
    misbehaving child can never fill the disk."""
    for s in SERVICES.values():
        p = s.log_path
        if not p or not p.is_file():
            continue
        try:
            if p.stat().st_size > MAX_LOG_BYTES:
                with p.open("r+", encoding="utf-8", errors="replace") as fh:
                    fh.seek(0)
                    fh.truncate()
                    fh.write(f"[control_api] log exceeded "
                             f"{MAX_LOG_BYTES // 1048576} MB and was truncated\n")
        except OSError:
            continue


@app.get("/api/stack/status")
def stack_status() -> dict:
    _cap_logs()
    return {
        n: {"running": s.running,
            "pid": s.proc.pid if s.running and s.proc else None,
            "detail": s.detail,
            "log": str(s.log_path) if s.log_path else None}
        for n, s in SERVICES.items()
    }


@app.get("/api/stack/log/{name}")
def stack_log(name: str, lines: int = 60) -> dict:
    """Tail a service log so launch failures are visible from the dashboard."""
    if name not in SERVICES:
        return {"ok": False, "detail": f"unknown service '{name}'"}
    return {"ok": True, "name": name, "tail": SERVICES[name].tail(lines)}


@app.post("/api/stack/cleanup")
def stack_cleanup() -> dict:
    """Reap orphaned sim processes + clear Gazebo scratch. Non-destructive:
    never touches models, build output, or the serial hardware agent."""
    killed = _reap_orphans()
    removed = _clean_scratch()
    return {"ok": True, "killed": killed, "removed": removed}


def _apply_sim_config() -> None:
    """Rebuild the SITL command from the currently selected model + world."""
    target = _sitl_target()
    SERVICES["sitl"].argv = ["bash", "-c", f"exec make px4_sitl {target}"]
    SERVICES["sitl"].detail = f"{SIM_CONFIG['model']} / {SIM_CONFIG['world']}"


def _apply_sitl_params(timeout_s: float = 30.0) -> str:
    exe = PX4_DIR / "build" / "px4_sitl_default" / "bin" / "px4-param"
    if not exe.is_file():
        return " | params: px4-param not found"
    pending = dict(SITL_PARAMS)
    deadline = time.monotonic() + timeout_s
    while pending and time.monotonic() < deadline:     # PX4 may still be booting
        for k, v in list(pending.items()):
            try:
                r = subprocess.run([str(exe), "set", k, str(v)], cwd=str(exe.parent.parent),
                                   capture_output=True, text=True, timeout=5)
            except subprocess.SubprocessError:
                continue
            if r.returncode == 0:
                pending.pop(k)
        if pending:
            time.sleep(1.0)
    return " | params ok" if not pending else f" | params NOT set: {', '.join(pending)}"


def _wait_for_sitl(timeout_s: float) -> bool:
    """Block until Gazebo has actually finished loading the world.

    Heavy worlds take far longer than `empty`, so a fixed sleep either wastes
    time or starts the downstream services before the DDS link exists. Poll for
    a live gzserver instead, and bail out early if SITL dies.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not SERVICES["sitl"].running:
            return False                      # SITL exited -> build/world error
        # exact process name + a listening master: `pgrep -f gzserver` also
        # matched the `pgrep -x gzserver` line inside setup_gazebo.bash and
        # declared the world ready before the server even existed
        if _pid_of("gzserver") is not None and _master_listening():
            time.sleep(2.0)                   # let the world settle
            return True
        time.sleep(1.0)
    return False


@app.get("/api/sim/config")
def sim_config() -> dict:
    return {**SIM_CONFIG, "target": _sitl_target(),
            "worlds": _list_worlds(), "models": _list_models()}


# --------------------------------------------------------------------------- #
# Aerial basemap for the MISSION MAP (worlds listed in basemaps.json)
#
# The image is the world's own Gazebo terrain texture, so the map shows exactly
# the ground the simulated drone flies over. It is read from the local Gazebo
# model (never committed - it is ~60 MB) and cached as a JPEG.
# --------------------------------------------------------------------------- #

BASEMAPS_FILE = SERVICES_DIR / "basemaps.json"          # committed (McMillan calibration)
SITES_FILE = SERVICES_DIR / "cache" / "sites.json"      # local, built by world_builder.py
BASEMAP_CACHE = SERVICES_DIR / "cache"


def _basemap_entry(world: str) -> Optional[dict]:
    for f in (SITES_FILE, BASEMAPS_FILE):
        try:
            entry = json.loads(f.read_text(encoding="utf-8")).get(world)
        except (OSError, ValueError):
            continue
        if entry:
            return entry
    return None


def _find_texture(rel: str) -> Optional[Path]:
    dirs = [MODELS_DIR, Path.home() / ".gazebo" / "models"]
    dirs += [Path(d) for d in os.environ.get("GAZEBO_MODEL_PATH", "").split(":") if d]
    for d in dirs:
        if (d / rel).is_file():
            return d / rel
    return None


@app.get("/api/sim/basemap")
def sim_basemap() -> dict:
    """Aerial image + its placement in the local ENU frame for the selected world."""
    world = SIM_CONFIG["world"]
    entry = _basemap_entry(world)
    if not entry or not _find_texture(entry["texture"]):
        return {"available": False, "world": world}
    (a, _, c), (_, e, f) = entry["affine"]
    w, h = entry["texture_size_px"]
    # u = a*x + c, v = e*y + f  (the cross terms are < 0.05 deg of rotation)
    return {"available": True, "world": world, "url": f"/api/sim/basemap/{world}.jpg",
            "bounds": {"west": -c / a, "east": (w - c) / a, "north": -f / e, "south": (h - f) / e},
            "metres_per_px": round(1.0 / a, 3),
            "validation_error_m": entry.get("validation_error_m")}


@app.get("/api/sim/basemap/{world}.jpg")
def sim_basemap_image(world: str):
    entry = _basemap_entry(world)
    src = _find_texture(entry["texture"]) if entry else None
    if src is None:
        return JSONResponse({"detail": f"no basemap for '{world}'"}, status_code=404)
    out = BASEMAP_CACHE / f"{world}.jpg"
    if not out.is_file() or out.stat().st_mtime < src.stat().st_mtime:
        from PIL import Image                      # only needed the first time
        BASEMAP_CACHE.mkdir(parents=True, exist_ok=True)
        Image.open(src).convert("RGB").save(out, "JPEG", quality=85)
    return FileResponse(out, media_type="image/jpeg",
                        headers={"Cache-Control": "max-age=3600"})


@app.post("/api/sim/config")
def set_sim_config(model: Optional[str] = None,
                   world: Optional[str] = None) -> dict:
    """Select airframe/world. Rejects unknown names so a typo cannot produce a
    make target that hangs. Takes effect on the next start."""
    if model is not None:
        if model not in _list_models():
            return {"ok": False, "detail": f"unknown model '{model}'"}
        SIM_CONFIG["model"] = model
    if world is not None:
        if world not in _list_worlds():
            return {"ok": False, "detail": f"unknown world '{world}'"}
        SIM_CONFIG["world"] = world
    _apply_sim_config()
    return {"ok": True, **sim_config()}


# --------------------------------------------------------------------------- #
# Gazebo window (gzclient): normal priority + self-healing
# --------------------------------------------------------------------------- #

GUI_STATE = {"wanted": False, "misses": 0, "started": 0.0, "restarts": 0, "server_seen": False}
GUI_CHECK_S = 5.0
GUI_GRACE_S = 20.0           # a fresh window needs a moment to reach the master
GUI_MAX_MISSES = 3           # ~15 s without a master connection = frozen


def _pid_of(name: str) -> Optional[int]:
    try:
        out = subprocess.run(["pgrep", "-x", name], capture_output=True, text=True,
                             timeout=5).stdout.split()
        return int(out[0]) if out else None
    except (subprocess.SubprocessError, FileNotFoundError, ValueError):
        return None


def _prepare_gui() -> bool:
    """Give the window gzserver's environment (models, plugins, libraries).
    PX4's setup_gazebo.bash is deliberately NOT sourced: on this machine it
    kills any running gzserver/gzclient/px4 first."""
    pid = _pid_of("gzserver")
    if pid is None:
        return False
    try:
        raw = Path(f"/proc/{pid}/environ").read_bytes().split(b"\0")
    except OSError:
        return False
    env = dict(kv.decode("utf-8", "replace").split("=", 1) for kv in raw if b"=" in kv)
    SERVICES["gui"].env = env
    return True


def _master_listening() -> bool:
    return _port_owner(11345) is not None or not _can_inspect_ports()


def _start_gui(wait_s: float = 30.0) -> None:
    # Wait for the Gazebo master to listen: by then PX4's launch script has
    # sourced setup_gazebo.bash, which on this machine kills any gzclient it
    # finds - a window started earlier would be shot down immediately.
    deadline = time.monotonic() + wait_s
    while not _master_listening() and time.monotonic() < deadline:
        time.sleep(0.5)
    if _prepare_gui():
        SERVICES["gui"].start()
        GUI_STATE.update(started=time.monotonic(), misses=0)


def _gui_has_master(pid: int) -> bool:
    """True while the window holds its connection to the Gazebo master (a
    window that lost it is frozen). NOTE: the master's unread backlog on this
    connection is NOT a usable signal here - on this setup every window shows
    one ~2.9 KB backlog seconds after start while still rendering normally."""
    try:
        out = subprocess.run(["ss", "-tnp"], capture_output=True, text=True,
                             timeout=5).stdout
    except (subprocess.SubprocessError, FileNotFoundError):
        return True               # can't tell -> don't restart on a guess
    return any(f"pid={pid}," in line and ":11345" in line.split()[4]
               for line in out.splitlines() if len(line.split()) > 4)


def _gui_watchdog() -> None:
    while True:
        time.sleep(GUI_CHECK_S)
        gui = SERVICES["gui"]
        sitl = SERVICES["sitl"]
        server_up = _pid_of("gzserver") is not None
        GUI_STATE["server_seen"] = GUI_STATE["server_seen"] or server_up
        # only a server that WAS up and is now gone is a crash (not a slow start)
        if sitl.running and not server_up and GUI_STATE["server_seen"]:
            # PX4 is still up waiting for a simulator that died: say so, loudly
            if "CRASHED" not in sitl.detail:
                sitl.detail += " - Gazebo server CRASHED: STOP SIM, then START SIM"
                print("[control_api] gzserver is gone while SITL runs", flush=True)
        if not GUI_STATE["wanted"] or not sitl.running or not server_up:
            continue
        reason = ""
        if gui.proc is None:
            continue                             # not launched yet (stack still starting)
        if not gui.running:
            if gui.proc.returncode == 0:
                GUI_STATE["wanted"] = False      # the operator closed the window
                continue
            reason = "exited"
        elif time.monotonic() - GUI_STATE["started"] > GUI_GRACE_S:
            if _gui_has_master(gui.proc.pid):
                GUI_STATE["misses"] = 0
            else:
                GUI_STATE["misses"] += 1
                if GUI_STATE["misses"] >= GUI_MAX_MISSES:
                    reason = "lost the Gazebo master (frozen)"
        if reason:
            GUI_STATE["restarts"] += 1
            gui.stop()
            _start_gui()
            gui.detail = f"Gazebo window (auto-restarted {GUI_STATE['restarts']}x)"
            print(f"[control_api] Gazebo window {reason} - restarted", flush=True)


# --------------------------------------------------------------------------- #
# Simulated environment for a real location (see environment.py): GPS / magnetic
# field / air pressure of that place + its weather as Gazebo wind. Saved to a
# file so it survives restarts; applied on the next START SIM.
# --------------------------------------------------------------------------- #

ENV_FILE = SERVICES_DIR / "cache" / "environment.json"
ENV_ACTIVE: Dict[str, object] = {}          # what the running sim was started with


def _env_load() -> dict:
    try:
        return json.loads(ENV_FILE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {"enabled": False}


def _home() -> tuple:
    if ENV_ACTIVE.get("enabled"):
        return float(ENV_ACTIVE["lat"]), float(ENV_ACTIVE["lon"])
    return HOME_LAT, HOME_LON


def _world_file(world: str) -> Optional[Path]:
    """Where sitl_run.sh would load `world` from (project override dir first)."""
    for d in (GENERATED_WORLDS, WORLD_OVERRIDES, WORLDS_DIR):
        if (d / f"{world}.world").is_file():
            return d / f"{world}.world"
    return None


def _apply_environment() -> str:
    """Set the SITL launch environment for the configured location / weather."""
    import environment as envmod
    env = _env_load()
    SERVICES["sitl"].env = dict(SITL_ENV)
    ENV_ACTIVE.clear()
    src = _world_file(SIM_CONFIG["world"] if SIM_CONFIG["world"] not in ("", "default") else "empty")
    site = (_basemap_entry(src.stem) or {}).get("location") if src else None
    if site:
        # A world built from a real place IS that place: GPS home must match its
        # terrain, whatever location the ENVIRONMENT panel holds. Only the
        # weather is taken from there.
        env = {**env, "lat": site["lat"], "lon": site["lon"], "alt_msl": site["alt_msl"],
               "name": src.stem}
        if not env.get("enabled"):
            env.update(enabled=True, wind_mps=0.0, gust_mps=0.0)
    if not env.get("enabled"):
        if src is not None and src.parent == GENERATED_WORLDS:
            SERVICES["sitl"].env["PX4_SITL_WORLD"] = str(src)
        return envmod.summary(None)
    if src is None:
        return "environment skipped: world file not found"
    world = envmod.build_world(src, SERVICES_DIR / "cache" / "worlds" / f"{src.stem}__env.world", env)
    SERVICES["sitl"].env.update({
        "PX4_HOME_LAT": f"{float(env['lat']):.7f}", "PX4_HOME_LON": f"{float(env['lon']):.7f}",
        "PX4_HOME_ALT": f"{float(env.get('alt_msl', 0.0)):.1f}", "PX4_SITL_WORLD": str(world)})
    ENV_ACTIVE.update(env)
    return envmod.summary(env)


class EnvConfig(BaseModel):
    enabled: bool = True
    name: str = "location"
    lat: float
    lon: float
    alt_msl: float = 0.0
    wind_mps: float = 0.0
    wind_from_deg: float = 0.0
    gust_mps: float = 0.0
    temp_c: Optional[float] = None
    weather_time: Optional[str] = None


@app.get("/api/sim/environment")
def sim_environment() -> dict:
    import environment as envmod
    return {"config": _env_load(), "active": dict(ENV_ACTIVE), "active_summary": envmod.summary(ENV_ACTIVE)}


@app.post("/api/sim/environment/weather")
def sim_environment_weather(lat: float, lon: float) -> dict:
    """Live weather at a location (Open-Meteo; only lat/lon are sent)."""
    import environment as envmod
    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
        return {"ok": False, "detail": "latitude / longitude out of range"}
    try:
        return {"ok": True, "weather": envmod.fetch_weather(lat, lon)}
    except Exception as e:                         # offline, rate-limited, ...
        return {"ok": False, "detail": f"weather service unavailable: {e}"}


@app.post("/api/sim/environment")
def set_sim_environment(cfg: EnvConfig) -> dict:
    if not (-90 <= cfg.lat <= 90 and -180 <= cfg.lon <= 180):
        return {"ok": False, "detail": "latitude / longitude out of range"}
    if not (0 <= cfg.wind_mps <= 30 and 0 <= cfg.gust_mps <= 40):
        return {"ok": False, "detail": "wind outside 0-30 m/s (gusts 0-40)"}
    ENV_FILE.parent.mkdir(parents=True, exist_ok=True)
    ENV_FILE.write_text(json.dumps(cfg.model_dump(), indent=2), encoding="utf-8")
    return {"ok": True, "detail": "saved - applies on the next START SIM", "config": cfg.model_dump()}


BUILD = {"state": "idle", "log": [], "result": None, "error": None}


class WorldBuildRequest(BaseModel):
    name: str
    lat: float
    lon: float
    size_m: float = 2000.0


def _build_world(req: WorldBuildRequest) -> None:
    import world_builder
    try:
        BUILD["result"] = world_builder.build(req.name, req.lat, req.lon, req.size_m,
                                              progress=lambda t: BUILD["log"].append(t))
        BUILD["state"] = "done"
    except Exception as e:
        BUILD.update(state="error", error=f"{type(e).__name__}: {e}")


@app.post("/api/sim/worlds/build")
def sim_world_build(req: WorldBuildRequest) -> dict:
    """Build a Gazebo world of a real location (SRTM terrain + aerial image)."""
    if BUILD["state"] == "running":
        return {"ok": False, "detail": "a world is already being built"}
    if not (-60 <= req.lat <= 60 and -180 <= req.lon <= 180):
        return {"ok": False, "detail": "latitude must be within ±60 (SRTM coverage)"}
    if not 500 <= req.size_m <= 3000:
        return {"ok": False, "detail": "size must be 500-3000 m"}
    BUILD.update(state="running", log=[], result=None, error=None)
    threading.Thread(target=_build_world, args=(req,), daemon=True).start()
    return {"ok": True}


@app.get("/api/sim/worlds/build")
def sim_world_build_status() -> dict:
    return {**BUILD, "log": BUILD["log"][-8:]}


@app.post("/api/stack/start")
def stack_start() -> dict:
    # Always start from a clean slate: a surviving gzserver from the previous
    # run is what produces the "not responding" dialog and the lag.
    _reap_orphans()
    _clean_scratch()
    _apply_sim_config()
    SERVICES["sitl"].detail += f" | env: {_apply_environment()}"
    time.sleep(1.0)
    GUI_STATE.update(wanted=LINUX, restarts=0, server_seen=False)
    ACTIVE_FAULTS.clear()
    SERVICES["gui"].detail = "Gazebo window"
    for name in START_ORDER:
        if name == "gui":
            _start_gui()
            time.sleep(1.0)
            continue
        SERVICES[name].start()
        if name == "sitl":
            # Wait for the world to finish loading rather than guessing. This is
            # what makes heavy worlds work without racing the DDS link.
            if _wait_for_sitl(SITL_READY_TIMEOUT_S):
                SERVICES["sitl"].detail += _apply_sitl_params()
        else:
            time.sleep(2.0 if name == "agent" else 1.0)
    return stack_status()


@app.post("/api/stack/stop")
def stack_stop() -> dict:
    GUI_STATE["wanted"] = False
    ACTIVE_FAULTS.clear()              # a fresh PX4 starts with every sensor healthy
    SESSION_SVC.stop()          # it holds the vehicle link; stop it first
    for name in reversed(START_ORDER):
        SERVICES[name].stop()
    # `make px4_sitl` spawns gzserver/gzclient outside our process group, so
    # killing the group alone reliably leaves them behind.
    _reap_orphans()
    return stack_status()


@app.get("/api/logs/size")
def logs_size() -> dict:
    """Report ROS log usage so the operator can decide before purging."""
    log_dir = Path.home() / ".ros" / "log"
    total = 0
    count = 0
    if log_dir.is_dir():
        for f in log_dir.rglob("*"):
            if f.is_file() and not f.is_symlink():
                try:
                    total += f.stat().st_size
                    count += 1
                except OSError:
                    pass
    return {"path": str(log_dir), "bytes": total,
            "human": f"{total / 1048576:.1f} MB", "files": count}


@app.post("/api/logs/purge")
def logs_purge(confirm: bool = False) -> dict:
    """Delete ROS logs. Requires ?confirm=true - destructive to history, so it
    never runs automatically and never as part of starting the simulation."""
    if not confirm:
        return {"ok": False, "detail": "refused: pass confirm=true",
                **logs_size()}
    log_dir = (Path.home() / ".ros" / "log").resolve()
    if _is_protected(log_dir) or log_dir == Path.home():
        return {"ok": False, "detail": "refused: protected path"}
    removed = 0
    for child in log_dir.glob("*"):
        if child.is_symlink():
            continue
        try:
            shutil.rmtree(child, ignore_errors=True) if child.is_dir() else child.unlink()
            removed += 1
        except OSError:
            continue
    return {"ok": True, "removed": removed}


@app.post("/api/stack/start/{name}")
def start_one(name: str) -> dict:
    if name == "gui":
        GUI_STATE["wanted"] = True
        _start_gui()
    elif name in SERVICES:
        SERVICES[name].start()
    return stack_status()


@app.post("/api/stack/stop/{name}")
def stop_one(name: str) -> dict:
    if name == "gui":
        GUI_STATE["wanted"] = False
    if name in SERVICES:
        SERVICES[name].stop()
    return stack_status()


@app.get("/api/mission/status")
def mission_status() -> dict:
    return {
        "llm_status": MISSION.llm_status,
        "waypoints": MISSION.waypoints,
        "running": MISSION.running,
        "log": MISSION.log[-60:],
    }


@app.post("/api/mission/run")
def mission_run(req: MissionRequest) -> dict:
    if MISSION.running:
        return {"ok": False, "detail": "a mission is already running"}
    if SESSION_SVC.running:     # exactly one offboard commander
        return {"ok": False, "detail": "an interactive session owns the vehicle - stop it first"}
    MISSION.log = []
    MISSION.llm_status = "LLM PLANNING MISSION…"
    threading.Thread(target=_run_mission, args=(req.prompt,), daemon=True).start()
    return {"ok": True}


@app.post("/api/mission/abort")
def mission_abort() -> dict:
    p = MISSION.proc
    if p and p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except (ProcessLookupError, PermissionError):
            pass
        MISSION.llm_status = "MISSION ABORTED BY OPERATOR"
    return {"ok": True}


# --------------------------------------------------------------------------- #
# Interactive session (one chat message = one step) - see docs/INTERACTIVE_MODE.md
#
# The session runs in its own process (session_service.py) because it holds a
# live rclpy vehicle link; this API only starts/stops it and proxies requests.
# --------------------------------------------------------------------------- #

SESSION_PORT = 8001
SESSION_URL = f"http://127.0.0.1:{SESSION_PORT}"
SESSION_SVC = Service("session", [], PIPELINE_ROOT, "interactive session :8001")


def _session_call(method: str, path: str, body: Optional[dict] = None,
                  timeout: float = 10.0) -> dict:
    data = json.dumps(body).encode("utf-8") if body is not None else None
    req = urllib.request.Request(f"{SESSION_URL}{path}", data=data, method=method,
                                 headers={"content-type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode("utf-8"))
    except (urllib.error.URLError, OSError, ValueError) as e:
        return {"ok": False, "reply": f"session service unreachable ({e})"}


@app.post("/api/session/start")
def session_start(backend: str = "px4", llm: Optional[str] = None) -> dict:
    if backend not in ("px4", "sim"):
        return {"ok": False, "detail": f"unknown backend '{backend}'"}
    if SESSION_SVC.running:
        return {"ok": True, "detail": "already running"}
    if MISSION.running:
        return {"ok": False, "detail": "a one-shot mission is running - wait or abort it"}
    if backend == "px4" and not SERVICES["sitl"].running:
        return {"ok": False, "detail": "start the simulation first (START SIM)"}
    llm = llm or os.environ.get("AERO_LLM", "auto")
    py = "python3" if backend == "px4" else sys.executable   # px4: ROS-sourced python
    cmd = (f"{py} {SERVICES_DIR / 'session_service.py'} --backend {backend} "
           f"--llm {llm} --port {SESSION_PORT}")
    site = _basemap_entry(SIM_CONFIG["world"]) or {}
    fence = site.get("geofence_half_side_m")
    if backend == "px4" and fence:          # the site is only known for the Gazebo world
        cmd += f" --geofence {float(fence):g}"
    terrain = site.get("terrain") or {}
    dem = _find_texture(terrain["dem"]) if backend == "px4" and terrain.get("dem") else None
    if dem:                                 # ground heights for the clearance check
        w, h = terrain["size_m"]
        cmd += (f" --terrain-dem {dem} --terrain-size {float(w):g} {float(h):g}"
                f" --clearance {float(site.get('min_terrain_clearance_m', 10)):g}"
                f" --rtl-alt {float(site.get('rtl_alt_m', 30)):g}")
    SESSION_SVC.argv = _sourced(cmd, with_ws=True) if backend == "px4" else ["bash", "-c", f"exec {cmd}"]
    SESSION_SVC.detail = (f"interactive session ({backend}, llm={llm}"
                          + (f", geofence ±{float(fence):g} m" if backend == "px4" and fence else "")
                          + (", terrain-aware)" if dem else ")"))
    SESSION_SVC.start()
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        if not SESSION_SVC.running:
            return {"ok": False, "detail": "session failed to start:\n" + SESSION_SVC.tail(15)}
        st = _session_call("GET", "/status", timeout=1.0)
        if st.get("active"):
            return {"ok": True, "detail": SESSION_SVC.detail}
        time.sleep(0.3)
    return {"ok": False, "detail": "session did not come up:\n" + SESSION_SVC.tail(15)}


@app.post("/api/session/stop")
def session_stop() -> dict:
    SESSION_SVC.stop()
    return {"ok": True}


@app.get("/api/session/status")
def session_status(since: int = 0) -> dict:
    if not SESSION_SVC.running:
        return {"active": False}
    st = _session_call("GET", f"/status?since={since}", timeout=2.0)
    for w in (st.get("step") or {}).get("waypoints", []):
        w["lat"], w["lon"] = _enu_to_latlon(w["x"], w["y"])
    return st


@app.post("/api/session/message")
def session_message(req: MissionRequest) -> dict:
    if not SESSION_SVC.running:
        return {"ok": False, "reply": "no interactive session - start one first"}
    return _session_call("POST", "/message", {"text": req.prompt})


@app.post("/api/session/command/{name}")
def session_command(name: str) -> dict:
    if not SESSION_SVC.running:
        return {"ok": False, "reply": "no interactive session"}
    return _session_call("POST", f"/command/{name}")


class PathRequest(BaseModel):
    points: List[List[float]]
    alt: Optional[float] = None
    circuit: bool = False
    where: str = "end"


@app.post("/api/session/path")
def session_path(req: PathRequest) -> dict:
    if not SESSION_SVC.running:
        return {"ok": False, "reply": "no interactive session"}
    return _session_call("POST", "/path", req.model_dump())


@app.post("/api/session/insert")
def session_insert(req: PathRequest) -> dict:
    if not SESSION_SVC.running:
        return {"ok": False, "reply": "no interactive session"}
    return _session_call("POST", "/insert", req.model_dump())


@app.post("/api/session/arm")
def session_arm() -> dict:
    if not SESSION_SVC.running:
        return {"ok": False, "reply": "no interactive session"}
    return _session_call("POST", "/arm", timeout=15.0)


@app.post("/api/session/sim/rc")
def session_sim_rc(on: bool = True) -> dict:
    return _session_call("POST", f"/sim/rc?on={'true' if on else 'false'}")


# --------------------------------------------------------------------------- #
# Sensor / motor fault injection into PX4 SITL (`failure` command; needs
# SYS_FAILURE_EN=1). Only what this PX4's simulator really implements is
# offered. Note: GPS "wrong" is a ~111 km jump (spoof / gross glitch), not
# gradual drift - PX4 SITL has no drift or wind injection.
# --------------------------------------------------------------------------- #

FAULT_CATALOG = {
    "gps": {"off": "GPS lost", "stuck": "GPS position frozen", "wrong": "GPS jump/spoof (~111 km)"},
    "mag": {"off": "compass dead", "stuck": "compass frozen"},
    "baro": {"off": "barometer dead", "stuck": "barometer frozen"},
    "accel": {"off": "accelerometer dead", "stuck": "accelerometer frozen"},
    "gyro": {"off": "gyro dead", "stuck": "gyro frozen"},
    "motor": {"off": "motor stopped"},
}
ACTIVE_FAULTS: Dict[str, str] = {}


def _px4_failure(component: str, ftype: str, instance: int = 0) -> tuple:
    exe = PX4_DIR / "build" / "px4_sitl_default" / "bin" / "px4-failure"
    try:
        out = subprocess.run([str(exe), component, ftype, "-i", str(instance)],
                             cwd=str(exe.parent.parent), capture_output=True, text=True, timeout=10)
    except (subprocess.SubprocessError, FileNotFoundError) as e:
        return False, str(e)
    text = re.sub(r"\x1b\[[0-9;]*m", "", out.stdout + out.stderr).strip()
    return out.returncode == 0, text


def _session_note(text: str) -> None:
    if SESSION_SVC.running:
        _session_call("POST", "/note", {"text": text}, timeout=2.0)


@app.get("/api/sim/faults")
def sim_faults() -> dict:
    return {"catalog": FAULT_CATALOG, "active": ACTIVE_FAULTS}


@app.post("/api/sim/fault")
def sim_fault(component: str, type: str, instance: int = 0) -> dict:
    if not SERVICES["sitl"].running:
        return {"ok": False, "detail": "the simulation is not running"}
    if type != "ok" and type not in FAULT_CATALOG.get(component, {}):
        return {"ok": False, "detail": f"unsupported fault {component}/{type}"}
    ok, text = _px4_failure(component, type, instance)
    if ok:
        if type == "ok":
            ACTIVE_FAULTS.pop(component, None)
            _session_note(f"[FAULT TEST] {component} restored")
        else:
            ACTIVE_FAULTS[component] = type
            _session_note(f"[FAULT TEST] injected: {FAULT_CATALOG[component][type]}")
    return {"ok": ok, "output": text, "active": ACTIVE_FAULTS}


@app.post("/api/sim/fault/clear")
def sim_fault_clear() -> dict:
    for comp in list(ACTIVE_FAULTS):
        sim_fault(comp, "ok")
    return {"ok": True, "active": ACTIVE_FAULTS}


# --------------------------------------------------------------------------- #
# Real-flight logs: upload from the dashboard, analyse, keep the report
#
# Files live in <pipeline>/realFlight_logs/<flight>/ (git-ignored). Names are
# sanitised to one path component each, so an upload can never write outside it.
# --------------------------------------------------------------------------- #

FLIGHT_LOGS = PIPELINE_ROOT / "realFlight_logs"
FLIGHT_LOG_MAX_BYTES = 400 * 1024 * 1024
LOG_EXTS = (".bin", ".tlog", ".json", ".txt", ".log", ".csv")


def _safe(name: str) -> str:
    s = re.sub(r"[^A-Za-z0-9._-]", "_", name or "").strip("._")[:120]
    return s or "unnamed"


def _flight_dir(flight: str) -> Path:
    return FLIGHT_LOGS / _safe(flight)


@app.get("/api/flightlogs")
def flightlogs_list() -> dict:
    flights = []
    if FLIGHT_LOGS.is_dir():
        for d in sorted((p for p in FLIGHT_LOGS.iterdir() if p.is_dir()), reverse=True):
            files = [{"name": f.name, "bytes": f.stat().st_size}
                     for f in sorted(d.iterdir()) if f.is_file() and f.name != "report.json"]
            flights.append({"flight": d.name, "files": files,
                            "analyzed": (d / "report.json").is_file()})
    return {"dir": str(FLIGHT_LOGS), "flights": flights}


@app.post("/api/flightlogs/upload")
async def flightlogs_upload(request: Request, flight: str, name: str) -> dict:
    """Raw body = the file (no multipart dependency). One call per file."""
    fname = _safe(name)
    if not fname.lower().endswith(LOG_EXTS):
        return {"ok": False, "detail": f"unsupported file type (use {', '.join(LOG_EXTS)})"}
    d = _flight_dir(flight)
    d.mkdir(parents=True, exist_ok=True)
    dest, size = d / fname, 0
    with dest.open("wb") as fh:
        async for chunk in request.stream():
            size += len(chunk)
            if size > FLIGHT_LOG_MAX_BYTES:
                fh.close()
                dest.unlink(missing_ok=True)
                return {"ok": False, "detail": "file larger than 400 MB"}
            fh.write(chunk)
    return {"ok": True, "flight": d.name, "file": fname, "bytes": size}


@app.post("/api/flightlogs/{flight}/analyze")
def flightlogs_analyze(flight: str, file: Optional[str] = None) -> dict:
    """Analyse one log of the flight (default: the dataflash .BIN, else the
    largest .tlog) and store the report next to it."""
    d = _flight_dir(flight)
    if not d.is_dir():
        return {"ok": False, "detail": f"no flight '{flight}'"}
    if file:
        target = d / _safe(file)
    else:
        logs = sorted((f for f in d.iterdir() if f.suffix.lower() in (".bin", ".tlog")),
                      key=lambda f: (f.suffix.lower() != ".bin", -f.stat().st_size))
        target = logs[0] if logs else None
    if target is None or not target.is_file():
        return {"ok": False, "detail": "no .BIN or .tlog in this flight"}
    from flight_log_analyzer import analyze          # pymavlink, only needed here
    try:
        report = analyze(str(target))
    except Exception as e:                            # a corrupt log must not kill the API
        return {"ok": False, "detail": f"could not parse {target.name}: {type(e).__name__}: {e}"}
    (d / "report.json").write_text(json.dumps(report), encoding="utf-8")
    return {"ok": True, "flight": d.name, "report": report}


@app.get("/api/flightlogs/{flight}/report")
def flightlogs_report(flight: str) -> dict:
    f = _flight_dir(flight) / "report.json"
    if not f.is_file():
        return {"ok": False, "detail": "not analysed yet"}
    return {"ok": True, "flight": _safe(flight), "report": json.loads(f.read_text(encoding="utf-8"))}


def _can_inspect_ports() -> bool:
    return bool(shutil.which("ss") or shutil.which("lsof"))


def _port_owner(port: int) -> Optional[int]:
    """PID currently listening on `port`, if any (`ss` on Linux, `lsof` on macOS)."""
    try:
        if shutil.which("ss"):
            out = subprocess.run(["ss", "-ltnp"], capture_output=True, text=True, timeout=5).stdout
            for line in out.splitlines():
                if f":{port} " in line:
                    m = re.search(r"pid=(\d+)", line)
                    if m:
                        return int(m.group(1))
            return None
        if shutil.which("lsof"):
            out = subprocess.run(["lsof", "-nP", f"-iTCP:{port}", "-sTCP:LISTEN", "-t"],
                                 capture_output=True, text=True, timeout=5).stdout.split()
            return int(out[0]) if out else None
    except (subprocess.SubprocessError, ValueError):
        pass
    return None


def _free_port(port: int) -> bool:
    """Stop whatever owns `port`. Only used with --force."""
    pid = _port_owner(port)
    if pid is None:
        return True
    for sig in (signal.SIGINT, signal.SIGKILL):
        try:
            os.kill(pid, sig)
        except (ProcessLookupError, PermissionError):
            return _port_owner(port) is None
        time.sleep(2)
        if _port_owner(port) is None:
            return True
    return _port_owner(port) is None


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="AEROMAST control API")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--force", action="store_true",
                    help="stop whatever already holds the port, then start")
    cli = ap.parse_args()

    owner = _port_owner(cli.port)
    if owner is not None:
        if cli.force:
            print(f"[INFO] port {cli.port} held by pid {owner} — stopping it")
            if not _free_port(cli.port):
                print(f"[ERROR] could not free port {cli.port} (pid {owner})")
                raise SystemExit(1)
        else:
            print(
                f"[ERROR] port {cli.port} is already in use by pid {owner}.\n"
                f"        An older control_api is probably still running.\n"
                f"        Restart it with:   python3 control_api.py --force\n"
                f"        Or stop it with:   kill {owner}"
            )
            raise SystemExit(1)

    print(f"[INFO] AEROMAST control API :{cli.port}  (pipeline root: {PIPELINE_ROOT})")
    uvicorn.run(app, host="0.0.0.0", port=cli.port, log_level="warning")
