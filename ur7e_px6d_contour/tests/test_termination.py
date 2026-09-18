import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from core.models import PolicyCommand, RobotState, Wrench
from experiment_logging.data_logger import ExperimentLogger
from experiment_logging.termination import TerminationReason as Reason, TerminationRecorder, classify_stop_reason
from robot.rtde_controller import RobotError
from sensor.px6d_reader import PX6DError, ProtocolError


@pytest.mark.parametrize("detail,expected", [
    ("processed force safety threshold exceeded", Reason.STOP_FORCE_LIMIT),
    ("unexpected contact during anchor transfer", Reason.STOP_UNEXPECTED_CONTACT),
    ("boundary recovery exhausted expanded local sectors", Reason.STOP_RECOVERY_EXHAUSTED),
    ("NO_FORWARD_PROGRESS: local anchor correction", Reason.STOP_NO_FORWARD_PROGRESS),
    ("REPEATED_CONTACT", Reason.STOP_REPEATED_CONTACT),
    ("FIT_FAILURE: local reinitialization exhausted", Reason.STOP_FIT_FAILURE),
    ("policy runtime limit reached", Reason.STOP_TIME_LIMIT),
    ("maximum search distance reached", Reason.STOP_SEARCH_LIMIT),
    ("boundary point limit reached", Reason.STOP_POINT_LIMIT),
    ("Q normal stop", Reason.STOP_USER_REQUEST),
    ("Full contour loop completed successfully.", Reason.SUCCESS),
    ("STOP_WORKSPACE_LIMIT", Reason.STOP_WORKSPACE_LIMIT),
])
def test_known_reasons_are_explicitly_classified(detail, expected):
    assert classify_stop_reason(detail) == expected


@pytest.mark.parametrize("detail", ["STOP", "FAILED", "EMERGENCY_STOP", "force chart export failed", "mystery timeout", "nonfinite sensor/robot sample"])
def test_generic_text_does_not_invent_a_cause(detail):
    assert classify_stop_reason(detail) == Reason.STOP_UNKNOWN_REASON


def test_old_nonfinite_sample_does_not_misclassify_a_new_cleanup_exception():
    recorder = TerminationRecorder()
    recorder.observe(raw=Wrench(float("nan"), 0, 0, 0, 0, 0))
    first = recorder.set_stop_reason(detail="nonfinite sensor/robot sample")
    later = recorder.set_stop_reason(detail="logger.close: RuntimeError: writer failed",
                                     exception=RuntimeError("writer failed"),
                                     source="app.cleanup.logger", terminal=False)
    assert first["reason"] == Reason.STOP_INVALID_WRENCH
    assert recorder.record is first
    assert later["reason"] == Reason.STOP_UNKNOWN_REASON
    assert "writer failed" in later["traceback"]


def test_exception_type_chain_and_operation_supply_evidence():
    try:
        try:
            raise ProtocolError("CRC-8 mismatch")
        except ProtocolError as exc:
            raise RuntimeError("outer failure") from exc
    except RuntimeError as exc:
        recorder = TerminationRecorder()
        record = recorder.set_stop_reason(exception=exc, detail="outer failure")
    assert record["reason"] == Reason.STOP_SENSOR_ERROR
    assert "ProtocolError" in record["traceback"] and "RuntimeError" in record["traceback"]
    assert "direct cause" in record["traceback"] and "test_termination.py" in record["traceback"]
    assert len(record["exception_chain"]) == 2
    assert classify_stop_reason(exception=RobotError("controller rejected command")) == Reason.STOP_MOTION_ERROR
    assert classify_stop_reason(exception=ValueError("bad value")) == Reason.STOP_UNKNOWN_REASON
    assert classify_stop_reason(exception=ValueError("bad value"), context={"operation": "load_config"}) == Reason.STOP_CONFIG_ERROR


def test_stop_snapshot_is_immutable_and_observation_is_io_free(monkeypatch, tmp_path):
    recorder = TerminationRecorder(tmp_path, print_fn=lambda _: pytest.fail("printed before physical stop"))
    robot = RobotState(1.0, np.zeros(6), np.zeros(6))
    policy = SimpleNamespace(state=SimpleNamespace(value="BOUNDARY_TRACKING"), sub_state="TANGENT_STEP",
                             _motion_queue=[(np.ones(6), "TANGENT_STEP")], active_episode=None,
                             recovery=None, probe_episodes=[], boundary_points=[])
    with monkeypatch.context() as context:
        context.setattr(Path, "mkdir", lambda *args, **kwargs: pytest.fail("mkdir before stop"))
        context.setattr(Path, "write_text", lambda *args, **kwargs: pytest.fail("write before stop"))
        recorder.observe(state="INITIALIZATION", policy=policy, robot=robot, raw=Wrench(0, 0, 0, 0, 0, 0))
        first = recorder.set_stop_reason(Reason.STOP_UNEXPECTED_CONTACT, "contact during transfer")
    assert first["state"] == "ANCHOR_TRANSFER" and first["policy_state"] == "BOUNDARY_TRACKING"
    robot.pose[:] = 9
    recorder.observe(robot=robot)
    recorder.set_stop_reason(Reason.STOP_USER_REQUEST, "later cleanup")
    assert recorder.record is first and first["tcp_pose"] == [0.0] * 6


def test_nonfinite_wrench_and_tcp_have_separate_reasons_and_strict_json(tmp_path):
    for values, expected in [(dict(raw=Wrench(float("nan"), 0, 0, 0, 0, 0)), Reason.STOP_INVALID_WRENCH),
                             (dict(robot=RobotState(0, np.full(6, np.inf), np.zeros(6))), Reason.STOP_MOTION_ERROR)]:
        recorder = TerminationRecorder(tmp_path)
        recorder.observe(**values)
        recorder.set_stop_reason(detail="nonfinite sensor/robot sample")
        directory = recorder.flush()
        payload = json.loads((directory / "termination.json").read_text(), parse_constant=lambda _: pytest.fail("nonstandard JSON constant"))
        assert payload["reason"] == expected
        assert "NaN" in str(payload) or "+Infinity" in str(payload)


def test_actual_air_force_is_separate_from_synthetic_policy_input():
    recorder = TerminationRecorder()
    recorder.observe(raw=Wrench(0, 0, 0, 0, 0, 0), processed=Wrench(2, 0, 0, 0, 0, 0),
                     force_input_mode="synthetic_policy_fixture", actual_raw_wrench=np.zeros(6),
                     actual_processed_wrench=np.zeros(6), synthetic_policy_wrench=[2, 0, 0, 0, 0, 0])
    record = recorder.set_stop_reason(Reason.SUCCESS, "fixture complete")
    assert record["actual_force"] == [0, 0, 0]
    assert record["processed_wrench"] == [0] * 6
    assert record["policy_processed_wrench"] == [2, 0, 0, 0, 0, 0]


def test_local_events_do_not_become_primary_and_later_exceptions_are_visible(tmp_path):
    messages = []
    recorder = TerminationRecorder(tmp_path, print_fn=messages.append)
    recorder.set_stop_reason(Reason.STOP_NO_CONTACT, "recover locally", terminal=False)
    assert recorder.record is None and not recorder.secondary_errors
    recorder.set_stop_reason(Reason.STOP_USER_REQUEST, "Q normal stop")
    recorder.set_stop_reason(exception=RobotError("return failed"), detail="return failed", terminal=False, source="safe_return")
    output = recorder.flush(emit=True)
    recorder.flush(emit=True)
    assert len(messages) == 1
    assert "STOP_USER_REQUEST" in messages[0] and "STOP_MOTION_ERROR" in messages[0]
    assert "return failed" in (output / "termination.txt").read_text()
    assert recorder.record["reason"] == Reason.STOP_USER_REQUEST


def test_missing_primary_and_unwritable_directory_have_honest_fallback(tmp_path, capsys):
    bad_root = tmp_path / "file"
    bad_root.write_text("existing file")
    recorder = TerminationRecorder(bad_root)
    recorder.set_stop_reason(Reason.STOP_NO_CONTACT, "recoverable", terminal=False)
    fallback = recorder.flush()
    assert fallback != bad_root
    payload = json.loads((fallback / "termination.json").read_text())
    assert payload["reason"] == Reason.STOP_UNKNOWN_REASON
    assert payload["secondary_errors"]
    assert "termination logging failed" in capsys.readouterr().err


def test_logger_summary_keeps_legacy_reason_and_accepts_late_cleanup_context(tmp_path):
    logger = ExperimentLogger(tmp_path, {"test": True})
    robot = RobotState(1, np.arange(6, dtype=float), np.zeros(6))
    wrench = Wrench(0.1, 0.2, 0.3, 0, 0, 0)
    command = PolicyCommand("RETURN_TO_START", False, np.zeros(2), 0, robot.pose.copy(), False, False)
    logger.log_sample(1, wrench, wrench, robot, command, None, None)
    logger.termination.set_stop_reason(Reason.STOP_USER_REQUEST, "Q normal stop")
    logger.write_stop_snapshot(robot, wrench, wrench, "STOP")
    logger.write_summary("STOP", "legacy human reason", 3, return_status="complete")
    logger.close()
    logger.termination.set_stop_reason(exception=PX6DError("close failed"), detail="close failed", terminal=False)
    logger.termination.flush()
    summary = json.loads((logger.run_dir / "summary.json").read_text())
    assert summary["reason"] == "legacy human reason"
    assert summary["boundary_point_count"] == 3 and summary["return_status"] == "complete"
    assert summary["termination_reason"] == Reason.STOP_USER_REQUEST
    assert summary["termination"]["tcp_pose"] == robot.pose.tolist()
    assert summary["termination_secondary_errors"][-1]["reason"] == Reason.STOP_SENSOR_ERROR
    snapshot = json.loads((logger.run_dir / "scan_stop_snapshot.json").read_text())
    assert snapshot["termination_reason"] == Reason.STOP_USER_REQUEST


@pytest.mark.parametrize("phase", ["VERTICAL_RETREAT", "MOVE_ABOVE_START", "DESCEND_TO_START"])
def test_known_safe_return_timeout_and_settled_errors(phase):
    assert classify_stop_reason(f"{phase} timed out") == Reason.STOP_TIME_LIMIT
    for measurement, unit in (("position", "m"), ("orientation", "rad")):
        detail = f"{phase} settled {measurement} error 0.015000 {unit}"
        assert classify_stop_reason(detail) == Reason.STOP_ANCHOR_ERROR
        assert classify_stop_reason("ReturnAborted: " + detail, context={"source": "app.safe_return"}) == Reason.STOP_ANCHOR_ERROR


@pytest.mark.parametrize("target", ["P0", "RESET"])
def test_final_return_target_errors_and_policy_lifecycle_errors(target):
    assert classify_stop_reason(f"final {target} position error 0.010000 m") == Reason.STOP_ANCHOR_ERROR
    assert classify_stop_reason(f"final {target} orientation error 0.050000 rad") == Reason.STOP_ANCHOR_ERROR
    assert classify_stop_reason(exception=RuntimeError("cannot start another episode before anchor return")) == Reason.STOP_ANCHOR_ERROR
    assert classify_stop_reason(exception=RuntimeError("safe return requires normal stop")) == Reason.STOP_MOTION_ERROR


@pytest.mark.parametrize("detail,expected", [
    ("ReturnAborted: return force limit exceeded", Reason.STOP_FORCE_LIMIT),
    ("RobotError: controller rejected command", Reason.STOP_MOTION_ERROR),
    ("PX6DError: serial response failed", Reason.STOP_SENSOR_ERROR),
    ("ProtocolError: CRC-8 mismatch", Reason.STOP_SENSOR_ERROR),
    ("RobotError: TCP x=0.9 is outside configured workspace", Reason.STOP_WORKSPACE_LIMIT),
])
def test_serialized_types_require_a_known_return_context(detail, expected):
    assert classify_stop_reason(detail) == Reason.STOP_UNKNOWN_REASON
    assert classify_stop_reason(detail, context={"source": "plotting.unrelated_return_chart"}) == Reason.STOP_UNKNOWN_REASON
    assert classify_stop_reason(detail, context={"source": "app.startup_return"}) == expected
    assert classify_stop_reason(detail, context={"phase": "VERTICAL_RETREAT_PRECHECK_SENSOR_READ"}) == expected
    assert classify_stop_reason("ReturnAborted: unfamiliar failure", context={"source": "app.safe_return"}) == Reason.STOP_UNKNOWN_REASON


def test_raw_force_fallback_and_actual_diagnostics_path_are_reported(tmp_path):
    messages = []
    recorder = TerminationRecorder(tmp_path, print_fn=messages.append)
    recorder.observe(raw=Wrench(1, 2, 3, 0, 0, 0))
    record = recorder.set_stop_reason(Reason.STOP_UNKNOWN_REASON, "processing failed")
    assert record["actual_force"] == [1, 2, 3]
    assert record["force_source"] == "raw_wrench" and record["force_frame"] == "sensor_frame"
    directory = recorder.flush(emit=True)
    assert f"Diagnostics: {directory / 'termination.json'}" in messages[0]
    assert "frame=sensor_frame" in messages[0]


def test_broken_snapshot_keeps_original_exception_traceback_and_last_command(tmp_path):
    class BrokenPolicy:
        state = "BOUNDARY_TRACKING"

        @property
        def active_episode(self):
            raise ValueError("diagnostic property broken")

    recorder = TerminationRecorder(tmp_path)
    command = PolicyCommand("BOUNDARY_TRACKING", True, np.array([1., 0.]), .001, np.zeros(6), False, False)
    recorder.observe(policy=BrokenPolicy(), command=command, timestamp=4.2)
    try:
        raise RuntimeError("original control exception")
    except RuntimeError as exc:
        record = recorder.set_stop_reason(exception=exc, detail="original control exception", source="app.run.exception")
    assert record["reason"] == Reason.STOP_UNKNOWN_REASON
    assert "original control exception" in record["traceback"]
    assert "test_broken_snapshot" in record["traceback"]
    assert record["state"] == "BOUNDARY_TRACKING" and record["command"]["move"] is True
    assert record["timestamp_utc"] and record["monotonic_sec"] == 4.2
    assert record["diagnostic_error"] == "diagnostic property broken"
    payload = json.loads((recorder.flush() / "termination.json").read_text())
    assert payload["exception_chain"][0]["type"] == "builtins.RuntimeError"


def test_transfer_target_precedes_the_anchor_from_completed_episode():
    old_anchor = np.zeros(6)
    transfer_target = np.ones(6)
    episode = SimpleNamespace(anchor_pose=old_anchor, contact_pose=None, phase="DONE", outcome="CONTACT")
    policy = SimpleNamespace(state="BOUNDARY_TRACKING", sub_state="TANGENT_STEP", active_episode=None,
                             probe_episodes=[episode], _motion_queue=[(transfer_target, "TANGENT_STEP")])
    recorder = TerminationRecorder()
    recorder.observe(policy=policy)
    record = recorder.set_stop_reason(Reason.STOP_UNEXPECTED_CONTACT, "contact during transfer")
    assert record["anchor_pose"] == transfer_target.tolist()
    assert record["state"] == "ANCHOR_TRANSFER"
    assert record["probe_outcome"] == "CONTACT"


def test_recovery_exhaustion_distinguishes_missing_contact_and_rejected_candidates():
    episodes = [SimpleNamespace(probe_id=i, recovery_id=0, purpose="RECOVERY", phase="DONE",
                                return_completed=True, contact_pose=None, anchor_pose=np.zeros(6),
                                outcome="NO_CONTACT" if i < 2 else "CONTACT",
                                rejection_reason="" if i < 2 else "PREVIOUSLY_VISITED_BOUNDARY") for i in range(3)]
    policy = SimpleNamespace(state="BOUNDARY_RECOVERY", active_episode=None, _motion_queue=[],
                             probe_episodes=episodes, recovery_records=[{}],
                             recovery=SimpleNamespace(anchor=np.ones(6), attempted_directions=[1, 2, 3], candidates=[np.ones(6)]))
    recorder = TerminationRecorder()
    recorder.observe(policy=policy)
    record = recorder.set_stop_reason(Reason.STOP_RECOVERY_EXHAUSTED, "no accepted candidate")
    assert record["anchor_pose"] == [1] * 6
    assert record["recovery_candidate_count"] == 1
    assert record["recovery_outcome_counts"] == {"NO_CONTACT": 2, "CONTACT": 1}
    assert record["recovery_rejection_counts"] == {"PREVIOUSLY_VISITED_BOUNDARY": 1}
    assert len(record["recovery_probe_summary"]) == 3
