"""Abstract backend interface.

Every backend (kinematic sim, PX4 uXRCE-DDS, MAVSDK, Nav2) consumes ONLY the
flat `ExecutableCommand` sequence produced by executor.compile_mission().
The type gate below makes it structurally impossible to feed raw LLM output,
JSON strings, or dicts into a vehicle.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, Sequence

from executor import EXECUTABLE_TYPES, ExecutableCommand


@dataclass
class BackendResult:
    backend: str
    completed: bool
    detail: str = ""
    metrics: Dict[str, float] = field(default_factory=dict)
    artifacts: Dict[str, str] = field(default_factory=dict)  # name -> file path


class MissionBackend(ABC):
    name: str = "abstract"

    def run(self, commands: Sequence[ExecutableCommand]) -> BackendResult:
        self._type_gate(commands)
        return self._run_checked(tuple(commands))

    @staticmethod
    def _type_gate(commands: Sequence[ExecutableCommand]) -> None:
        if isinstance(commands, (str, bytes, dict)):
            raise TypeError(
                "Backend received raw data instead of compiled commands. "
                "Pipeline order is: LLM -> validate_mission() -> compile_mission() -> backend."
            )
        for i, c in enumerate(commands):
            if not isinstance(c, EXECUTABLE_TYPES):
                raise TypeError(
                    f"commands[{i}] is {type(c).__name__}, not an ExecutableCommand. "
                    "Only executor.compile_mission() output may reach a backend."
                )

    @abstractmethod
    def _run_checked(self, commands: Sequence[ExecutableCommand]) -> BackendResult:
        ...
