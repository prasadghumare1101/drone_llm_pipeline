"""Built-in kinematic simulator backend.

Purpose: run the FULL pipeline (prompt -> LLM -> validated JSON -> executor ->
vehicle motion) on any machine with zero ROS 2 / PX4 / Gazebo installed —
used for CI, tests, and the reference end-to-end demo. Pure-Python, fixed-step
(dt = 0.05 s), fully deterministic.

Motion model (documented assumptions):
  * Point-mass, constant-speed straight-line segments (no dynamics/wind).
  * Climb 2.0 m/s, descent 1.0 m/s, waypoint acceptance radius 0.5 m.
  * RTL = fly home (0,0) at current altitude at cruise of the last Goto
    (fallback 5 m/s), then land.

Outputs: trajectory CSV, JSON run report, optional top-down PNG plot
(matplotlib, headless Agg; skipped gracefully if matplotlib is absent).
Completion is VERIFIED: every Goto target must actually be reached within
tolerance or the result reports completed=False.
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

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

from .base_backend import BackendResult, MissionBackend

DT = 0.05                 # s, integration step
CLIMB_SPEED = 2.0         # m/s
DESCENT_SPEED = 1.0       # m/s
ACCEPT_RADIUS = 0.5       # m
MAX_SIM_TIME = 3600.0     # s, hard runaway guard


class KinematicSimBackend(MissionBackend):
    name = "sim"

    def __init__(self, out_dir: Optional[Path] = None, make_plot: bool = True,
                 verbose: bool = True):
        self.out_dir = Path(out_dir) if out_dir else None
        self.make_plot = make_plot
        self.verbose = verbose

    # ------------------------------------------------------------------ #

    def _run_checked(self, commands: Sequence[ExecutableCommand]) -> BackendResult:
        t = 0.0
        x, y, z = 0.0, 0.0, 0.0
        armed = False
        distance = 0.0
        traj: List[Tuple[float, float, float, float, str]] = [(0.0, x, y, z, "INIT")]
        events: List[str] = []
        goto_targets: List[Tuple[float, float, float]] = []
        goto_reached: List[bool] = []
        captures: List[Tuple[float, float, float, str]] = []
        last_speed = 5.0

        def log(msg: str) -> None:
            events.append(f"t={t:7.2f}s  {msg}")
            if self.verbose:
                print(f"[sim] t={t:7.2f}s  {msg}")

        def step_toward(tx: float, ty: float, tz: float, speed: float,
                        label: str) -> bool:
            """Advance one dt toward target. Returns True when reached."""
            nonlocal t, x, y, z, distance
            dx, dy, dz = tx - x, ty - y, tz - z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist <= ACCEPT_RADIUS:
                return True
            move = min(speed * DT, dist)
            x += dx / dist * move
            y += dy / dist * move
            z += dz / dist * move
            distance += move
            t += DT
            traj.append((t, x, y, z, label))
            return False

        def fly_to(tx: float, ty: float, tz: float, speed: float, label: str) -> bool:
            while t < MAX_SIM_TIME:
                if step_toward(tx, ty, tz, speed, label):
                    return True
            return False

        # ----------------------------- command loop ----------------------- #
        for i, cmd in enumerate(commands):
            tag = f"{type(cmd).__name__}#{i}"
            if t >= MAX_SIM_TIME:
                break

            if isinstance(cmd, Arm):
                armed = True
                log("ARMED")

            elif isinstance(cmd, Disarm):
                armed = False
                log("DISARMED")

            elif isinstance(cmd, Takeoff):
                log(f"TAKEOFF -> {cmd.alt:.1f} m")
                fly_to(x, y, cmd.alt, CLIMB_SPEED, tag)
                log(f"takeoff complete at z={z:.2f} m")

            elif isinstance(cmd, Goto):
                goto_targets.append((cmd.x, cmd.y, cmd.alt))
                last_speed = cmd.speed_mps
                ok = fly_to(cmd.x, cmd.y, cmd.alt, cmd.speed_mps, tag)
                goto_reached.append(ok)
                log(f"GOTO ({cmd.x:.1f},{cmd.y:.1f},{cmd.alt:.1f}) @ "
                    f"{cmd.speed_mps:.1f} m/s -> {'reached' if ok else 'FAILED'}")

            elif isinstance(cmd, Hold):
                log(f"HOLD {cmd.seconds:.1f} s")
                end = t + cmd.seconds
                while t < end and t < MAX_SIM_TIME:
                    t += DT
                    traj.append((t, x, y, z, tag))

            elif isinstance(cmd, ReturnToLaunch):
                log("RTL: heading home")
                fly_to(0.0, 0.0, z, last_speed, tag)
                fly_to(0.0, 0.0, 0.0, DESCENT_SPEED, tag)
                log("RTL complete: landed at home")

            elif isinstance(cmd, Capture):
                captures.append((round(x, 2), round(y, 2), round(z, 2), cmd.label))
                log(f"CAPTURE '{cmd.label}' at ({x:.1f},{y:.1f},{z:.1f})")

            elif isinstance(cmd, Record):
                log(f"RECORD {'start' if cmd.start else 'stop'}")

            elif isinstance(cmd, Gimbal):
                log(f"GIMBAL pitch={cmd.pitch_deg}° yaw={cmd.yaw_deg}°")

            elif isinstance(cmd, Land):
                log("LAND: descending")
                fly_to(x, y, 0.0, DESCENT_SPEED, tag)
                log(f"landed at ({x:.2f},{y:.2f})")

        # ----------------------------- verdict ---------------------------- #
        all_reached = all(goto_reached) if goto_reached else True
        landed = z <= ACCEPT_RADIUS
        ran_out = t >= MAX_SIM_TIME
        completed = all_reached and landed and not armed and not ran_out

        metrics = {
            "sim_time_s": round(t, 2),
            "distance_m": round(distance, 2),
            "waypoints_commanded": float(len(goto_targets)),
            "waypoints_reached": float(sum(goto_reached)),
            "captures": float(len(captures)),
            "final_x": round(x, 3), "final_y": round(y, 3), "final_z": round(z, 3),
        }
        artifacts = {}
        if self.out_dir:
            artifacts = self._write_artifacts(traj, goto_targets, events, metrics, completed)

        detail = (f"{sum(goto_reached)}/{len(goto_targets)} waypoints reached, "
                  f"{distance:.1f} m in {t:.1f} s (sim), "
                  f"final pos ({x:.2f},{y:.2f},{z:.2f}), "
                  f"{'landed+disarmed' if landed and not armed else 'NOT safely down'}")
        return BackendResult(backend=self.name, completed=completed,
                             detail=detail, metrics=metrics, artifacts=artifacts)

    # ------------------------------------------------------------------ #

    def _write_artifacts(self, traj, goto_targets, events, metrics, completed):
        self.out_dir.mkdir(parents=True, exist_ok=True)
        artifacts = {}

        csv_path = self.out_dir / "trajectory.csv"
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "x_east_m", "y_north_m", "z_up_m", "segment"])
            w.writerows(traj)
        artifacts["trajectory_csv"] = str(csv_path)

        report_path = self.out_dir / "sim_report.json"
        report_path.write_text(json.dumps(
            {"completed": completed, "metrics": metrics, "events": events},
            indent=2), encoding="utf-8")
        artifacts["report_json"] = str(report_path)

        if self.make_plot:
            png = self._plot(traj, goto_targets)
            if png:
                artifacts["plot_png"] = str(png)
        return artifacts

    def _plot(self, traj, goto_targets) -> Optional[Path]:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("[sim] matplotlib not installed — skipping plot")
            return None

        xs = [p[1] for p in traj]
        ys = [p[2] for p in traj]
        zs = [p[3] for p in traj]

        fig, (ax1, ax2) = plt.subplots(
            1, 2, figsize=(11, 5), gridspec_kw={"width_ratios": [1.15, 1]})

        ax1.plot(xs, ys, lw=1.6, color="#1565c0", label="flight path")
        if goto_targets:
            ax1.scatter([w[0] for w in goto_targets], [w[1] for w in goto_targets],
                        s=60, marker="s", facecolors="none", edgecolors="#e65100",
                        linewidths=1.6, label="waypoints", zorder=5)
        ax1.scatter([0], [0], s=90, marker="^", color="#2e7d32",
                    label="home", zorder=6)
        ax1.set_xlabel("x East (m)")
        ax1.set_ylabel("y North (m)")
        ax1.set_title("Top-down path (LOCAL_ENU_METERS)")
        ax1.set_aspect("equal", adjustable="datalim")
        ax1.grid(alpha=0.3)
        ax1.legend(loc="upper left", fontsize=8)

        ts = [p[0] for p in traj]
        ax2.plot(ts, zs, lw=1.6, color="#6a1b9a")
        ax2.set_xlabel("sim time (s)")
        ax2.set_ylabel("altitude z (m)")
        ax2.set_title("Altitude profile")
        ax2.grid(alpha=0.3)

        fig.suptitle("Kinematic simulation — validated mission execution", fontsize=11)
        fig.tight_layout()
        png_path = self.out_dir / "trajectory_plot.png"
        fig.savefig(png_path, dpi=130)
        plt.close(fig)
        return png_path
