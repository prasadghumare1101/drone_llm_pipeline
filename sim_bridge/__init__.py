"""sim_bridge public interface.

Backends are imported lazily via get_backend() so a machine without ROS 2 /
MAVSDK can still run the kinematic simulator (and vice versa).
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional

from .base_backend import BackendResult, MissionBackend
from .kinematic_sim import KinematicSimBackend

__all__ = ["BackendResult", "MissionBackend", "KinematicSimBackend", "get_backend"]


def get_backend(name: str, out_dir: Optional[Path] = None) -> MissionBackend:
    name = name.lower()
    if name == "sim":
        return KinematicSimBackend(out_dir=out_dir)
    if name == "px4":
        from .px4_dds_bridge import Px4DdsBackend
        return Px4DdsBackend(out_dir=out_dir)   # enables geotagged CAPTURE
    if name == "mavsdk":
        from .mavsdk_bridge import MavsdkBackend
        return MavsdkBackend()
    if name == "nav2":
        from .nav2_bridge import Nav2Backend
        return Nav2Backend()
    raise ValueError(f"unknown backend '{name}' (use sim|px4|mavsdk|nav2)")
