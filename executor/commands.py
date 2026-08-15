"""Low-level executable command primitives.

This is the ONLY vocabulary backends (sim_bridge/*) understand. Frozen,
fully-resolved (no optional speeds — the executor resolves cruise speed),
frame = LOCAL_ENU_METERS as defined in the schema.
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass
from typing import Sequence, Tuple, Union


@dataclass(frozen=True)
class Arm:
    pass


@dataclass(frozen=True)
class Disarm:
    pass


@dataclass(frozen=True)
class Takeoff:
    alt: float  # m above home, positive up


@dataclass(frozen=True)
class Goto:
    x: float          # East, m
    y: float          # North, m
    alt: float        # Up, m above home
    speed_mps: float  # fully resolved — never None


@dataclass(frozen=True)
class Hold:
    seconds: float


@dataclass(frozen=True)
class Land:
    pass


@dataclass(frozen=True)
class ReturnToLaunch:
    pass


@dataclass(frozen=True)
class Capture:
    """Take one geotagged still. Patterns (GRID/ORBIT/SPIRAL) emit these between
    legs when capture is requested; the backend records the position it fired."""
    label: str = ""


@dataclass(frozen=True)
class Record:
    start: bool


@dataclass(frozen=True)
class Gimbal:
    pitch_deg: float
    yaw_deg: float = 0.0


ExecutableCommand = Union[Arm, Disarm, Takeoff, Goto, Hold, Land, ReturnToLaunch,
                          Capture, Record, Gimbal]

EXECUTABLE_TYPES: Tuple[type, ...] = (Arm, Disarm, Takeoff, Goto, Hold, Land,
                                      ReturnToLaunch, Capture, Record, Gimbal)


def command_to_dict(cmd: ExecutableCommand) -> dict:
    d = {"cmd": type(cmd).__name__}
    d.update(asdict(cmd))
    return d


def commands_digest(cmds: Sequence[ExecutableCommand]) -> str:
    """Canonical SHA-256 over a command sequence. Used to prove determinism:
    same validated JSON in -> same digest out, every time."""
    canon = json.dumps([command_to_dict(c) for c in cmds],
                       sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(canon).hexdigest()
