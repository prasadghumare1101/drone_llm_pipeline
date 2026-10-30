#!/usr/bin/env python3
"""Interactive flight session service (one chat message = one step).

A long-lived process that owns the vehicle link and the session.Session. It
runs separately from control_api.py on purpose: control_api stays a pure
supervisor that never imports rclpy, and a crash here cannot take the
dashboard down. control_api starts / stops this process and proxies to it.

Listens on 127.0.0.1 only - the browser reaches it through control_api.

Run:  python3 session_service.py --backend px4     (PX4 SITL, ROS sourced)
      python3 session_service.py --backend sim     (built-in kinematic sim)
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from functools import partial
from pathlib import Path
from typing import List, Optional

import uvicorn
from fastapi import FastAPI
from pydantic import BaseModel

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from llm_layer import propose_step_json        # noqa: E402
from session import FastCommand, Session       # noqa: E402
from validator import drone_limits_for_site     # noqa: E402
from validator.terrain import TerrainModel      # noqa: E402

TICK_S = 0.2
SESSION: Session = None                         # set in main()

app = FastAPI(title="AEROMAST interactive session")


class Message(BaseModel):
    text: str


class PathRequest(BaseModel):
    points: List[List[float]]            # [[x_east, y_north], ...] metres from home
    alt: Optional[float] = None          # default: current altitude (10 m from the ground)
    circuit: bool = False                # close the loop back to the first point
    where: str = "end"                   # /insert only: "next" | "end"


@app.get("/status")
def status(since: int = 0) -> dict:
    return SESSION.snapshot(since)


@app.post("/message")
def message(msg: Message) -> dict:
    return SESSION.handle_message(msg.text)


@app.post("/command/{name}")
def command(name: str) -> dict:
    try:
        cmd = FastCommand(name)
    except ValueError:
        return {"ok": False, "reply": f"unknown command '{name}'"}
    return SESSION.command(cmd)


@app.post("/path")
def path(req: PathRequest) -> dict:
    """Fly a path drawn on the map (validated like any step)."""
    return SESSION.fly_path([tuple(p[:2]) for p in req.points], req.alt, req.circuit)


@app.post("/insert")
def insert(req: PathRequest) -> dict:
    """Add drawn waypoints to the step that is flying (next / at the end)."""
    return SESSION.add_waypoints([tuple(p[:2]) for p in req.points],
                                 "next" if req.where == "next" else "end")


@app.post("/note")
def note(msg: Message) -> dict:
    """A line from the operator console (e.g. a fault injected for a test)."""
    return SESSION.note(msg.text)


@app.post("/arm")
def arm() -> dict:
    return SESSION.pilot_arm()


@app.post("/sim/rc")
def sim_rc(on: bool = True) -> dict:
    """Kinematic sim only: simulate the pilot taking / handing back RC control."""
    link = SESSION.link
    if not hasattr(link, "pilot_takeover"):
        return {"ok": False, "reply": "RC simulation is only available on the sim backend"}
    link.pilot_takeover() if on else link.pilot_handback()
    return {"ok": True}


def main() -> None:
    global SESSION
    ap = argparse.ArgumentParser(description="AEROMAST interactive session service")
    ap.add_argument("--backend", choices=["px4", "sim"], default="px4")
    ap.add_argument("--llm", choices=["auto", "groq", "huggingface", "offline"], default="auto")
    ap.add_argument("--port", type=int, default=8001)
    ap.add_argument("--time-scale", type=float, default=1.0, help="sim backend only")
    ap.add_argument("--geofence", type=float, default=None,
                    help="site geofence half-side in metres (default: the 200 m in safety_limits)")
    ap.add_argument("--terrain-dem", default=None, help="DEM GeoTIFF of the site (ground heights)")
    ap.add_argument("--terrain-size", type=float, nargs=2, metavar=("EW_M", "NS_M"),
                    help="metres the DEM spans east-west and north-south, centred on home")
    ap.add_argument("--clearance", type=float, default=10.0,
                    help="minimum height above the ground anywhere on a path (m)")
    ap.add_argument("--rtl-alt", type=float, default=30.0,
                    help="autopilot's return-home altitude above home (PX4 RTL_RETURN_ALT)")
    ap.add_argument("--watchdog", type=float, default=120.0,
                    help="seconds without an operator reply before returning home")
    args = ap.parse_args()

    if args.backend == "px4":
        from sim_bridge.px4_link import Px4Link
        link = Px4Link()
        for _ in range(100):            # wait for position AND status/battery, so the
            st = link.state()           # session starts from the vehicle's real state
            if st.connected and st.battery_pct is not None:
                break
            time.sleep(0.1)
    else:
        from sim_bridge.kinematic_link import KinematicLink
        link = KinematicLink(time_scale=args.time_scale)

    planner = partial(propose_step_json, backend=args.llm)
    fallback = None if args.llm == "offline" else partial(propose_step_json, backend="offline")
    limits = drone_limits_for_site(args.geofence) if args.geofence else None
    terrain = (TerrainModel(args.terrain_dem, tuple(args.terrain_size))
               if args.terrain_dem and args.terrain_size else None)
    SESSION = Session(link, planner, fallback_planner=fallback, planner_label=args.llm,
                      inactivity_timeout_s=args.watchdog, limits=limits, terrain=terrain,
                      min_clearance_m=args.clearance, rtl_alt_m=args.rtl_alt)

    stop = threading.Event()

    def supervise() -> None:
        while not stop.is_set():
            try:
                SESSION.tick()
            except Exception as e:      # never let the watchdog thread die silently
                print(f"[session] tick error: {e!r}", flush=True)
            stop.wait(TICK_S)

    threading.Thread(target=supervise, daemon=True, name="session-tick").start()
    print(f"[INFO] session service :{args.port}  backend={args.backend} llm={args.llm}",
          flush=True)
    try:
        uvicorn.run(app, host="127.0.0.1", port=args.port, log_level="warning")
    finally:
        stop.set()
        link.close()                    # Px4Link: RTL if still flying


if __name__ == "__main__":
    main()
