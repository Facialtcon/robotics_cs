"""Termination adapters preserve control outcomes and original diagnostic evidence."""

import json
from pathlib import Path
import runpy

import numpy as np
import pytest

from app import real_validation
from core.models import Wrench
from experiment_logging import termination
from policy.rule_policy import State
from robot.rtde_controller import RobotError
from simulation.simulator import ContourSimulator, load_simulation_config
from test_real_validation import make_runner


ROOT = Path(__file__).resolve().parents[1]


def make_simulator(tmp_path):
    config = load_simulation_config(ROOT / "simulation/scene_square.yaml")
    config["simulation"]["output_root"] = str(tmp_path / "results")
    simulator = ContourSimulator(config)
    simulator.termination.bind(tmp_path / "before_finalization")
    return simulator


def read_termination(directory):
    return json.loads((Path(directory) / "termination.json").read_text(encoding="utf-8"))


def test_simulator_step_budget_has_shared_reason_without_changing_legacy_stop(tmp_path):
    simulator = make_simulator(tmp_path)
    simulator.run_headless(2)
    assert simulator.termination is simulator.policy.termination
    assert simulator.stop_reason == "headless step limit reached (2)"
    assert simulator.final_policy_state == "TARGET_SEARCH"
    assert simulator.robot.time == pytest.approx(.04)
    assert np.all(simulator.robot.tcp_speed == 0)
    record = simulator.termination.record
    assert record["reason"] == "STOP_TIME_LIMIT"
    assert record["source"] == "simulation._finish"
    assert record["state"] == "TARGET_SEARCH"
    assert record["tcp_pose"] == simulator.robot.pose.tolist()
    assert record["anchor_pose"] == simulator.policy.probe_episodes[0].anchor_pose.tolist()
    output = simulator.finalize()
    assert read_termination(output)["reason"] == "STOP_TIME_LIMIT"
    summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    assert summary["stop_reason"] == simulator.stop_reason
    assert summary["termination_reason"] == "STOP_TIME_LIMIT"
    assert summary["termination_context"]["state"] == "TARGET_SEARCH"


def test_simulator_finalization_keeps_the_original_policy_failure(tmp_path):
    simulator = make_simulator(tmp_path)
    simulator.step()
    simulator.policy.state = State.BOUNDARY_RECOVERY
    simulator.policy.request_stop("boundary recovery exhausted expanded local sectors")
    original = simulator.termination.record
    simulator.normal_stop("finalized by operator")
    assert simulator.stop_reason == "finalized by operator"
    assert simulator.termination.record == original
    assert original["reason"] == "STOP_RECOVERY_EXHAUSTED"
    assert original["state"] == "BOUNDARY_RECOVERY"
    assert simulator.final_policy_state == "STOP"


def test_simulator_plot_failure_keeps_termination_artifacts_and_traceback(tmp_path, monkeypatch):
    simulator = make_simulator(tmp_path)
    simulator.run_headless(1)

    def failing_plot(*args, **kwargs):
        raise RuntimeError("diagnostic plot failure")

    monkeypatch.setattr("simulation.top_view.save_result_figure", failing_plot)
    with pytest.raises(RuntimeError, match="diagnostic plot failure"):
        simulator.finalize()
    record = read_termination(simulator.output_dir)
    assert record["reason"] == "STOP_TIME_LIMIT"
    assert any("failing_plot" in event.get("traceback", "") for event in record["secondary_errors"])
    assert simulator.stopped


def test_air_real_force_failure_keeps_fault_phase_actual_wrench_and_command(tmp_path):
    runner, controller, _, _, _ = make_runner(tmp_path, after_motion=Wrench(1.1, 0, 0, 0, 0, 0))
    summary = runner.run(confirm=lambda: True, poll_key=lambda: None)
    record = read_termination(tmp_path)
    assert summary["status"] == "failed"
    assert summary["termination_reason"] == record["reason"] == "STOP_FORCE_LIMIT"
    assert record["source"] == "air.run"
    assert record["state"] == "TARGET_SEARCH"
    assert record["policy_sub_state"] == "PROBE"
    assert record["raw_wrench"][0] == pytest.approx(1.1)
    assert record["processed_wrench"][0] == pytest.approx(.22)
    assert record["actual_force"][0] == pytest.approx(.22)
    assert record["command"]["move"] is True
    assert record["anchor_pose"] == controller.origin.tolist()
    assert "_sample" in record["traceback"]
    assert "real unfiltered force increment" in record["detail"]
    assert len(controller.commands) == 1


def test_air_nonfinite_trigger_frame_is_strict_json_diagnostic(tmp_path):
    runner, _, _, _, _ = make_runner(tmp_path, after_motion=Wrench(float("nan"), 0, 0, 0, 0, 0))
    summary = runner.run(confirm=lambda: True, poll_key=lambda: None)
    record = read_termination(tmp_path)
    assert summary["status"] == "failed"
    assert record["reason"] == "STOP_INVALID_WRENCH"
    assert record["raw_wrench"][0] == "NaN"
    assert "nonfinite real sensor/robot sample" in record["traceback"]


def test_air_command_exception_retains_chained_traceback_and_cleanup_is_secondary(tmp_path, monkeypatch):
    runner, controller, _, _, _ = make_runner(tmp_path)

    def failed_command(*args, **kwargs):
        try:
            raise OSError("test network reset")
        except OSError as exc:
            raise RobotError("speedL transport unavailable") from exc

    def failed_close():
        raise RuntimeError("secondary device close failure")

    monkeypatch.setattr(controller, "command_planar_velocity", failed_command)
    monkeypatch.setattr(controller, "close", failed_close)
    summary = runner.run(confirm=lambda: True, poll_key=lambda: None)
    record = read_termination(tmp_path)
    assert summary["status"] == "failed"
    assert record["reason"] == "STOP_MOTION_ERROR"
    assert record["operation"] == "RTDE_SPEED_COMMAND"
    assert "test network reset" in record["traceback"]
    assert "speedL transport unavailable" in record["traceback"]
    assert "failed_command" in record["traceback"]
    assert any("failed_close" in event["traceback"] for event in record["secondary_errors"])
    assert record["state"] == "TARGET_SEARCH"


@pytest.mark.parametrize("failed_acceptance", ["stop", "coverage"])
def test_air_failed_acceptance_revokes_provisional_success(tmp_path, monkeypatch, failed_acceptance):
    runner, controller, _, _, _ = make_runner(tmp_path)
    # Report fixture completion before coverage to exercise the existing final
    # acceptance checks without adding a new trajectory or a physical device.
    monkeypatch.setattr(real_validation.ScriptedAirContact, "complete", lambda self, policy: True)
    if failed_acceptance == "stop":
        original_stop = controller.stop

        def failed_completion_stop():
            if runner.termination.record and runner.termination.record["reason"] == "SUCCESS":
                raise RobotError("completion stop rejected")
            return original_stop()

        monkeypatch.setattr(controller, "stop", failed_completion_stop)
    summary = runner.run(confirm=lambda: True, poll_key=lambda: None)
    record = read_termination(tmp_path)
    assert summary["status"] == "failed"
    assert summary["termination_reason"] == record["reason"] != "SUCCESS"
    assert any(event["reason"] == "SUCCESS" for event in record["events"])
    if failed_acceptance == "stop":
        assert record["reason"] == "STOP_MOTION_ERROR"
        assert "completion stop rejected" in record["traceback"]
    else:
        assert "required air coverage missing" in record["detail"]
    assert not controller.commands


def test_air_entry_failure_creates_fallback_diagnostics_without_devices(tmp_path, monkeypatch):
    recorder = termination.TerminationRecorder(print_fn=lambda *args: None)
    recorder.bind(tmp_path)
    monkeypatch.setattr(termination, "TerminationRecorder", lambda **kwargs: recorder)

    def invalid_entry():
        raise ValueError("unknown early validation error")

    monkeypatch.setattr(real_validation, "main", invalid_entry)
    with pytest.raises(SystemExit) as raised:
        runpy.run_path(str(ROOT / "run_real_validation.py"), run_name="__main__")
    assert raised.value.code == 1
    record = read_termination(tmp_path)
    assert record["reason"] == "STOP_UNKNOWN_REASON"
    assert "invalid_entry" in record["traceback"]
    assert record["exception_type"] == "ValueError"


def test_air_existing_output_refusal_does_not_rewrite_the_existing_run(tmp_path):
    runner, controller, reader, _, _ = make_runner(tmp_path)
    original = '{"status":"previous-run","reason":"preserve this result"}\n'
    (tmp_path / "summary.json").write_text(original, encoding="utf-8")
    with pytest.raises(FileExistsError):
        runner.run(confirm=lambda: True, poll_key=lambda: None)
    assert (tmp_path / "summary.json").read_text(encoding="utf-8") == original
    assert not (tmp_path / "termination.json").exists()
    assert runner.termination.run_dir != tmp_path
    assert read_termination(runner.termination.run_dir)["exception_type"] == "FileExistsError"
    assert not controller.connected and not reader.connected


def test_air_unhandled_exit_is_not_mislabeled_as_unconfirmed_start(tmp_path):
    runner, controller, _, _, _ = make_runner(tmp_path)

    def poll():
        if controller.commands:
            raise SystemExit("opaque air process exit")
        return None

    with pytest.raises(SystemExit, match="opaque air process exit"):
        runner.run(confirm=lambda: True, poll_key=poll)
    record = read_termination(tmp_path)
    assert record["reason"] == "STOP_UNKNOWN_REASON"
    assert "opaque air process exit" in record["detail"]
    assert "SystemExit" in record["traceback"]
    assert record["state"] == "TARGET_SEARCH"
    assert record["command"]["move"] is True
    assert controller.closed
