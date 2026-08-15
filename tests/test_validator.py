"""Validator tests: schema conformance + safety rules, loud specific failures."""
import copy
import json
from pathlib import Path

import pytest

from validator import (
    MissionPlan,
    MissionValidationError,
    validate_mission,
    validate_mission_file,
)

SAMPLES = Path(__file__).parent / "sample_missions"


def _good() -> dict:
    return json.loads((SAMPLES / "valid_perimeter_loop.json").read_text())


# ------------------------------ happy paths ------------------------------ #

def test_valid_drone_mission_returns_frozen_plan():
    plan = validate_mission(json.dumps(_good()))
    assert isinstance(plan, MissionPlan)
    assert plan.vehicle == "drone"
    assert len(plan.source_sha256) == 64
    with pytest.raises(Exception):        # frozen dataclass
        plan.cruise_speed_mps = 99.0


def test_valid_ground_mission_file():
    plan = validate_mission_file(SAMPLES / "valid_ground_square.json")
    assert plan.vehicle == "ground_robot"


def test_dict_input_accepted():
    assert validate_mission(_good()).mission_name


# ------------------------------ loud failures ---------------------------- #

def test_malformed_json_is_loud():
    with pytest.raises(MissionValidationError, match="not valid JSON"):
        validate_mission("{ this is not json ")


def test_non_object_top_level_rejected():
    with pytest.raises(MissionValidationError, match="top-level"):
        validate_mission("[1, 2, 3]")


def test_unknown_command_rejected_by_schema():
    m = _good()
    m["commands"].insert(1, {"type": "SELF_DESTRUCT"})
    with pytest.raises(MissionValidationError, match=r"commands\[1\]"):
        validate_mission(m)


def test_extra_fields_rejected():
    m = _good()
    m["backdoor"] = "run arbitrary stuff"
    with pytest.raises(MissionValidationError, match="backdoor"):
        validate_mission(m)


def test_overspeed_rejected_with_specific_message():
    m = _good()
    m["cruise_speed_mps"] = 50.0
    with pytest.raises(MissionValidationError, match=r"50\.0 m/s exceeds max 12\.0"):
        validate_mission(m)


def test_altitude_ceiling_enforced():
    m = _good()
    m["commands"][0]["alt"] = 500.0
    with pytest.raises(MissionValidationError, match="exceeds hard ceiling"):
        validate_mission(m)


def test_geofence_enforced_inside_loops():
    m = _good()
    m["commands"][1]["waypoints"][2]["x"] = 5000.0
    with pytest.raises(MissionValidationError, match="geofence"):
        validate_mission(m)


def test_loop_count_capped():
    m = _good()
    m["commands"][1]["count"] = 99
    with pytest.raises(MissionValidationError, match="loop count 99 exceeds"):
        validate_mission(m)


def test_drone_must_start_with_takeoff_and_end_with_land_or_rtl():
    m = _good()
    m["commands"] = m["commands"][1:]          # drop TAKEOFF
    with pytest.raises(MissionValidationError, match="must start with TAKEOFF"):
        validate_mission(m)
    m2 = _good()
    m2["commands"] = m2["commands"][:-1]       # drop RTL
    with pytest.raises(MissionValidationError, match="must end with LAND or RTL"):
        validate_mission(m2)


def test_ground_robot_cannot_takeoff_or_fly():
    m = json.loads((SAMPLES / "valid_ground_square.json").read_text())
    m["commands"].insert(0, {"type": "TAKEOFF", "alt": 5.0})
    with pytest.raises(MissionValidationError, match="not allowed for vehicle"):
        validate_mission(m)
    m2 = json.loads((SAMPLES / "valid_ground_square.json").read_text())
    m2["commands"][0]["alt"] = 3.0
    with pytest.raises(MissionValidationError, match="alt == 0"):
        validate_mission(m2)


def test_multiple_errors_all_reported():
    m = _good()
    m["cruise_speed_mps"] = 40.0
    m["commands"][0]["alt"] = 400.0
    m["commands"][1]["count"] = 50
    with pytest.raises(MissionValidationError) as ei:
        validate_mission(m)
    assert len(ei.value.errors) >= 3


def test_known_bad_sample_rejected():
    with pytest.raises(MissionValidationError) as ei:
        validate_mission_file(SAMPLES / "invalid_overspeed_and_fence.json")
    joined = " ".join(ei.value.errors)
    assert "exceeds max" in joined and "geofence" in joined


def test_same_json_same_hash_different_json_different_hash():
    a = validate_mission(_good()).source_sha256
    b = validate_mission(_good()).source_sha256
    m = _good()
    m["cruise_speed_mps"] = 4.0
    c = validate_mission(m).source_sha256
    assert a == b and a != c
