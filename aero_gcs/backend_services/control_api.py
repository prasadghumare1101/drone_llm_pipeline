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
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Dict, List, Optional

import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# --------------------------------------------------------------------------- #
# Paths / configuration
# --------------------------------------------------------------------------- #

PIPELINE_ROOT = Path(__file__).resolve().parents[2]      # ~/drone_llm_pipeline
SERVICES_DIR = Path(__file__).resolve().parent
PX4_DIR = Path(os.environ.get("PX4_DIR", Path.home() / "PX4-Autopilot"))
ROS_SETUP = os.environ.get("ROS_SETUP", "/opt/ros/humble/setup.bash")
WS_SETUP = PIPELINE_ROOT / "install" / "setup.bash"

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
}

# Hard ceiling on any captured service log. Defence in depth for the runaway
# case above - a log can never be allowed to consume the disk.
MAX_LOG_BYTES = 20 * 1024 * 1024

# Heavy worlds need far longer than `empty` before the DDS link appears.
SITL_READY_TIMEOUT_S = float(os.environ.get("PX4_READY_TIMEOUT", 180))


def _sitl_target() -> str:
    """PX4 make target. `model__world` (double underscore) selects a world."""
    model = SIM_CONFIG["model"]
    world = SIM_CONFIG["world"]
    if not world or world in ("default", "empty"):
        return f"gazebo-classic_{model}"
    return f"gazebo-classic_{model}__{world}"


def _list_worlds() -> List[str]:
    if not WORLDS_DIR.is_dir():
        return ["empty"]
    return sorted(p.stem for p in WORLDS_DIR.glob("*.world"))


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
        self._log_fh = self.log_path.open("w", buffering=1, encoding="utf-8",
                                          errors="replace")
        self.proc = subprocess.Popen(
            self.argv,
            cwd=str(self.cwd),
            env=merged,
            stdout=self._log_fh,
            stderr=subprocess.STDOUT,
            start_new_session=True,      # own process group -> killable as a tree
        )

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
    "video": Service(
        "video",
        # needs rclpy: the FPV feed comes from a Gazebo ROS camera topic
        _sourced(f"python3 {SERVICES_DIR / 'video_streamer.py'}"),
        SERVICES_DIR, "MJPEG :8080"),
}

# Order matters: agent before SITL so the DDS link comes up cleanly.
START_ORDER = ["agent", "sitl", "rosbridge", "telemetry", "video"]


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
    lat = HOME_LAT + (north / 111_320.0)
    lon = HOME_LON + (east / (111_320.0 * math.cos(math.radians(HOME_LAT))))
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
    yield
    # never leave orphaned SITL/gazebo processes behind on API shutdown
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
        try:
            out = subprocess.run(["pgrep", "-f", "gzserver"],
                                 capture_output=True, text=True, timeout=5)
            if out.stdout.strip():
                time.sleep(2.0)               # let the world settle
                return True
        except (subprocess.SubprocessError, FileNotFoundError):
            pass
        time.sleep(1.0)
    return False


@app.get("/api/sim/config")
def sim_config() -> dict:
    return {**SIM_CONFIG, "target": _sitl_target(),
            "worlds": _list_worlds(), "models": _list_models()}


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


@app.post("/api/stack/start")
def stack_start() -> dict:
    # Always start from a clean slate: a surviving gzserver from the previous
    # run is what produces the "not responding" dialog and the lag.
    _reap_orphans()
    _clean_scratch()
    _apply_sim_config()
    time.sleep(1.0)
    for name in START_ORDER:
        SERVICES[name].start()
        if name == "sitl":
            # Wait for the world to finish loading rather than guessing. This is
            # what makes heavy worlds work without racing the DDS link.
            _wait_for_sitl(SITL_READY_TIMEOUT_S)
        else:
            time.sleep(2.0 if name == "agent" else 1.0)
    return stack_status()


@app.post("/api/stack/stop")
def stack_stop() -> dict:
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
    if name in SERVICES:
        SERVICES[name].start()
    return stack_status()


@app.post("/api/stack/stop/{name}")
def stop_one(name: str) -> dict:
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


def _port_owner(port: int) -> Optional[int]:
    """PID currently listening on `port`, if any."""
    try:
        out = subprocess.run(["ss", "-ltnp"], capture_output=True,
                             text=True, timeout=5).stdout
    except (subprocess.SubprocessError, FileNotFoundError):
        return None
    for line in out.splitlines():
        if f":{port} " in line:
            m = re.search(r"pid=(\d+)", line)
            if m:
                return int(m.group(1))
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
