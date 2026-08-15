"""Executor tests: purity/determinism, loop unrolling, and the type gate."""
import json
from pathlib import Path

import pytest

from executor import (
    Arm,
    Disarm,
    Goto,
    ReturnToLaunch,
    Takeoff,
    commands_digest,
    compile_mission,
)
from validator import validate_mission

SAMPLES = Path(__file__).parent / "sample_missions"
GOOD = (SAMPLES / "valid_perimeter_loop.json").read_text()


def test_deterministic_same_json_same_commands_same_digest():
    p1 = validate_mission(GOOD)
    p2 = validate_mission(GOOD)
    c1, c2 = compile_mission(p1), compile_mission(p2)
    assert c1 == c2
    assert commands_digest(c1) == commands_digest(c2)
    # and stable across repeated compilation of the same object
    assert commands_digest(compile_mission(p1)) == commands_digest(c1)


def test_loop_unrolled_into_flat_fixed_path():
    plan = validate_mission(GOOD)
    loop = plan.commands[1]
    cmds = compile_mission(plan)
    gotos = [c for c in cmds if isinstance(c, Goto)]
    assert len(gotos) == loop.count * len(loop.waypoints)
    # order preserved & repeated exactly
    first_lap = [(g.x, g.y) for g in gotos[: len(loop.waypoints)]]
    second_lap = [(g.x, g.y) for g in gotos[len(loop.waypoints):]]
    assert first_lap == second_lap == [(w.x, w.y) for w in loop.waypoints]


def test_drone_sequence_shape():
    cmds = compile_mission(validate_mission(GOOD))
    assert isinstance(cmds[0], Arm)
    assert isinstance(cmds[1], Takeoff)
    assert isinstance(cmds[-2], ReturnToLaunch)
    assert isinstance(cmds[-1], Disarm)
    assert isinstance(cmds, tuple)  # immutable output


def test_cruise_speed_resolved_never_none():
    plan = validate_mission(GOOD)
    for c in compile_mission(plan):
        if isinstance(c, Goto):
            assert c.speed_mps == plan.cruise_speed_mps


def test_per_goto_speed_override_wins():
    m = json.loads(GOOD)
    m["commands"] = [
        {"type": "TAKEOFF", "alt": 10.0},
        {"type": "GOTO", "x": 5.0, "y": 5.0, "alt": 10.0, "speed_mps": 3.0},
        {"type": "RTL"},
    ]
    cmds = compile_mission(validate_mission(m))
    goto = next(c for c in cmds if isinstance(c, Goto))
    assert goto.speed_mps == 3.0


def test_type_gate_blocks_raw_llm_output():
    """Structural guarantee: strings/dicts (i.e. anything an LLM emits) can
    never reach the executor."""
    with pytest.raises(TypeError, match="validated validator.MissionPlan"):
        compile_mission(GOOD)               # raw JSON string
    with pytest.raises(TypeError, match="validated validator.MissionPlan"):
        compile_mission(json.loads(GOOD))   # parsed but unvalidated dict
