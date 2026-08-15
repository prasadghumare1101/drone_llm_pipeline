"""End-to-end tests: prompt -> LLM -> validator -> executor -> kinematic sim."""
import pytest

from executor import compile_mission
from llm_layer import OfflineHeuristicBackend, propose_mission_json
from sim_bridge import KinematicSimBackend
from validator import MissionValidationError, validate_mission

PROMPT = "Patrol the perimeter loop twice at 15 metres"


def test_offline_llm_is_deterministic():
    a = OfflineHeuristicBackend().propose(PROMPT)
    b = OfflineHeuristicBackend().propose(PROMPT)
    assert a == b


def test_full_pipeline_drone_loop_completes(tmp_path):
    proposed = propose_mission_json(PROMPT, backend="offline")   # untrusted str
    plan = validate_mission(proposed)                            # trust boundary
    assert plan.vehicle == "drone"
    cmds = compile_mission(plan)                                 # deterministic
    sim = KinematicSimBackend(out_dir=tmp_path, make_plot=False, verbose=False)
    result = sim.run(cmds)
    assert result.completed, result.detail
    assert result.metrics["waypoints_reached"] == result.metrics["waypoints_commanded"] == 8.0
    assert result.metrics["final_z"] <= 0.5                      # landed
    assert (tmp_path / "trajectory.csv").exists()
    assert (tmp_path / "sim_report.json").exists()


def test_full_pipeline_ground_robot(tmp_path):
    proposed = propose_mission_json(
        "Drive the ground robot around a 10 metre square box three laps",
        backend="offline")
    plan = validate_mission(proposed)
    assert plan.vehicle == "ground_robot"
    result = KinematicSimBackend(out_dir=tmp_path, make_plot=False,
                                 verbose=False).run(compile_mission(plan))
    assert result.completed, result.detail
    assert result.metrics["final_z"] == 0.0


def test_pipeline_rejects_unsafe_llm_proposal():
    """If the 'LLM' proposes something outside safety bounds, the vehicle
    never sees it — validation fails loudly before the executor."""
    proposed = propose_mission_json(
        "Patrol the perimeter once at 500 metres", backend="offline")
    with pytest.raises(MissionValidationError, match="hard ceiling"):
        validate_mission(proposed)


def test_backend_type_gate_blocks_non_commands(tmp_path):
    sim = KinematicSimBackend(out_dir=tmp_path, make_plot=False, verbose=False)
    with pytest.raises(TypeError):
        sim.run('[{"cmd": "Goto"}]')          # raw string
    with pytest.raises(TypeError):
        sim.run([{"cmd": "Goto"}])            # list of dicts


# --------------------------------------------------------------------------- #
# LLM retry policy: recoverable formatting slips are re-asked, safety refusals
# are final. The distinction is what stops a retry loop laundering an unsafe
# request into an accepted one.
# --------------------------------------------------------------------------- #

VALID = (
    '{"schema_version":"1.0","mission_name":"retry ok","vehicle":"drone",'
    '"frame":"LOCAL_ENU_METERS","cruise_speed_mps":5.0,"commands":['
    '{"type":"TAKEOFF","alt":10.0},{"type":"RTL"}]}'
)
UNSAFE = (
    '{"schema_version":"1.0","mission_name":"too fast","vehicle":"drone",'
    '"frame":"LOCAL_ENU_METERS","cruise_speed_mps":45.0,"commands":['
    '{"type":"TAKEOFF","alt":10.0},{"type":"RTL"}]}'
)


def test_validation_error_reports_its_stage():
    for raw, stage in [('{"broken": ', "json"), ('{"nope": 1}', "schema"),
                       (UNSAFE, "safety")]:
        with pytest.raises(MissionValidationError) as ei:
            validate_mission(raw)
        assert ei.value.stage == stage


def test_malformed_json_is_retried_then_succeeds(monkeypatch, tmp_path):
    """A dropped quote on one attempt must not fail the whole mission."""
    import run_pipeline as rp
    calls = []

    def flaky(prompt, backend="auto"):
        calls.append(backend)
        return '{"schema_version":"1.0", "vehicle":' if len(calls) == 1 else VALID

    monkeypatch.setattr(rp, "propose_mission_json", flaky)
    rc = rp.main(["--prompt", "x", "--dry-run", "--out-dir", str(tmp_path)])
    assert rc == 0 and len(calls) == 2      # retried once, then compiled


def test_unsafe_plan_is_never_retried(monkeypatch, tmp_path):
    """Safety refusals are final - no re-rolling until the limits pass."""
    import run_pipeline as rp
    calls = []

    def always_unsafe(prompt, backend="auto"):
        calls.append(backend)
        return UNSAFE

    monkeypatch.setattr(rp, "propose_mission_json", always_unsafe)
    rc = rp.main(["--prompt", "x", "--dry-run", "--out-dir", str(tmp_path),
                  "--llm-attempts", "5"])
    assert rc == 2 and len(calls) == 1      # asked once, refused, stopped
