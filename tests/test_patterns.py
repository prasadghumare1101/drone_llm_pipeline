"""Pattern expansion + payload-command tests.

The point of declarative GRID/ORBIT/SPIRAL is that geometry is computed
deterministically by the executor, and that its EXPANDED size is bounded by the
validator before anything flies.
"""
import json
import math

import pytest

from executor import compile_mission, commands_digest, Capture, Gimbal, Goto, Record
from validator import MissionValidationError, validate_mission


def mission(*cmds, speed=8.0):
    return json.dumps({
        "schema_version": "1.0", "mission_name": "pattern test",
        "vehicle": "drone", "frame": "LOCAL_ENU_METERS",
        "cruise_speed_mps": speed,
        "commands": [{"type": "TAKEOFF", "alt": 40.0}, *cmds, {"type": "RTL"}],
    })


GRID = {"type": "GRID", "center_x": 0.0, "center_y": 0.0, "width": 100.0,
        "height": 100.0, "spacing": 25.0, "alt": 40.0}
ORBIT = {"type": "ORBIT", "x": 0.0, "y": 0.0, "alt": 40.0, "radius": 30.0,
         "turns": 2, "points_per_turn": 8}
SPIRAL = {"type": "SPIRAL", "x": 0.0, "y": 0.0, "alt": 40.0,
          "start_radius": 20.0, "growth": 20.0, "turns": 3}


def gotos(cmds):
    return [c for c in cmds if isinstance(c, Goto)]


# ----------------------------- geometry ---------------------------------- #

def test_grid_is_boustrophedon_and_covers_the_area():
    g = gotos(compile_mission(validate_mission(mission(GRID))))
    lanes = [g[i:i + 2] for i in range(0, len(g), 2)]
    assert len(lanes) == 5                       # 100 m / 25 m + 1
    # consecutive lanes must alternate direction (that is what makes it a sweep)
    for i in range(len(lanes) - 1):
        assert (lanes[i][1].x > lanes[i][0].x) != (lanes[i + 1][1].x > lanes[i + 1][0].x)
    assert min(p.y for p in g) == -50.0 and max(p.y for p in g) == 50.0
    assert min(p.x for p in g) == -50.0 and max(p.x for p in g) == 50.0


def test_orbit_points_are_all_on_the_circle():
    g = gotos(compile_mission(validate_mission(mission(ORBIT))))
    assert len(g) == 2 * 8 + 1                   # turns*points + closing point
    for p in g:
        assert math.isclose(math.hypot(p.x, p.y), 30.0, abs_tol=1e-3)


def test_spiral_radius_grows_monotonically():
    g = gotos(compile_mission(validate_mission(mission(SPIRAL))))
    radii = [math.hypot(p.x, p.y) for p in g]
    assert all(b >= a - 1e-6 for a, b in zip(radii, radii[1:]))
    assert math.isclose(radii[0], 20.0, abs_tol=1e-3)
    assert math.isclose(radii[-1], 20.0 + 20.0 * 3, abs_tol=1e-3)


def test_pattern_expansion_is_deterministic():
    d = {commands_digest(compile_mission(validate_mission(mission(GRID, ORBIT, SPIRAL))))
         for _ in range(5)}
    assert len(d) == 1


def test_capture_flag_emits_one_capture_per_waypoint():
    cmds = compile_mission(validate_mission(mission({**GRID, "capture": True})))
    assert len(gotos(cmds)) == sum(isinstance(c, Capture) for c in cmds)


# ------------------------------ limits ------------------------------------ #

def test_grid_spacing_floor_is_enforced():
    with pytest.raises(MissionValidationError, match="below minimum"):
        validate_mission(mission({**GRID, "spacing": 0.5}))


def test_pattern_outside_geofence_is_rejected():
    with pytest.raises(MissionValidationError, match="geofence"):
        validate_mission(mission({**ORBIT, "radius": 190.0, "x": 150.0}))


def test_spiral_growing_past_the_radius_cap_is_rejected():
    with pytest.raises(MissionValidationError, match="exceeds max"):
        validate_mission(mission({**SPIRAL, "growth": 80.0, "turns": 5}))


def test_expanded_waypoint_budget_is_enforced_before_flight():
    """20 turns x 72 points = 1441 waypoints: each field is individually legal,
    so only counting the EXPANSION catches it."""
    with pytest.raises(MissionValidationError, match="unrolls to|budget"):
        validate_mission(mission({**ORBIT, "turns": 20, "points_per_turn": 72}))


def test_capture_budget_is_enforced_before_flight():
    with pytest.raises(MissionValidationError, match="captures"):
        validate_mission(mission(
            {**GRID, "spacing": 2.0, "height": 400.0, "width": 100.0,
             "capture": True},
            {**GRID, "spacing": 2.0, "height": 400.0, "width": 100.0,
             "capture": True}))


def test_gimbal_pitch_range_is_enforced():
    with pytest.raises(MissionValidationError, match="gimbal pitch"):
        validate_mission(mission({"type": "GIMBAL", "pitch_deg": 75.0}))


# --------------------------- mission shape -------------------------------- #

def test_payload_commands_do_not_break_shape_rules():
    """A trailing CAPTURE must not look like an unterminated mission."""
    cmds = compile_mission(validate_mission(json.dumps({
        "schema_version": "1.0", "mission_name": "payload shape",
        "vehicle": "drone", "frame": "LOCAL_ENU_METERS", "cruise_speed_mps": 5.0,
        "commands": [
            {"type": "GIMBAL", "pitch_deg": -90.0},
            {"type": "TAKEOFF", "alt": 20.0},
            {"type": "RECORD", "action": "start"},
            {"type": "RTL"},
            {"type": "CAPTURE", "label": "final"},
        ]})))
    assert any(isinstance(c, Gimbal) for c in cmds)
    assert any(isinstance(c, Record) for c in cmds)
    assert any(isinstance(c, Capture) for c in cmds)
