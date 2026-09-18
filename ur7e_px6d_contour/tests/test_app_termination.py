"""Exercise formal scan exit paths with disconnected device doubles only."""
from pathlib import Path
from types import SimpleNamespace
import json

import numpy as np
import pytest
import yaml

from app import main as app
from calibration.scan_calibration import build_scan_calibration
from config.loader import load_config
from core.models import RobotState, Wrench
from experiment_logging.termination import TerminationRecorder
from policy.rule_policy import RuleBasedPolicy
from robot.rtde_controller import RobotError
from sensor.px6d_reader import PX6DError


ROOT = Path(__file__).resolve().parents[1]
ZERO = Wrench(0, 0, 0, 0, 0, 0)


def setup_scan(monkeypatch, tmp_path, *, keys=(None, "Q"), wrench=ZERO,
               connect_error=None, read_error=None, motion_error=None,
               close_error=None, return_reason=""):
    config = load_config(ROOT / "config.yaml")
    config["logging"]["output_root"] = str(tmp_path / "runs")
    config["logging"]["show_contour_result_after_scan"] = False
    config["preprocessing"]["baseline"]["capture_on_start"] = False
    pose = np.array([.4, -.2, .3, 0, 3.14, 0])
    reference = pose.copy()
    reference[0] += .03
    calibration = build_scan_calibration(pose, reference, config["robot"]["robot_ip"],
                                         active_tcp_offset=config["tcp"]["offset"])
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(config), encoding="utf-8")
    (tmp_path / config["calibration"]["file"]).write_text(yaml.safe_dump(calibration), encoding="utf-8")

    class Reader:
        firmware = "OFFLINE TEST"
        closed = False

        def connect(self):
            if connect_error:
                raise connect_error

        def read_wrench(self):
            if read_error:
                raise read_error
            return wrench

        def close(self):
            self.closed = True

    class Controller:
        closed = False
        stop_calls = 0

        def __init__(self):
            self.pose = pose.copy()
            self.commands = []

        def connect(self, **kwargs):
            pass

        def read_state(self):
            return RobotState(1.0, self.pose.copy(), np.zeros(6))

        read_diagnostic_state = read_state

        def command_planar_velocity(self, direction, speed, period):
            self.commands.append((np.array(direction).copy(), speed, period))
            if motion_error:
                raise motion_error
            self.pose[:2] += np.asarray(direction) * speed * period

        def stop(self):
            self.stop_calls += 1

        safe_stop_motion = stop

        def close(self):
            self.closed = True
            if close_error:
                raise close_error

    class Keyboard:
        def __init__(self):
            self.keys = iter(keys)

        def __enter__(self):
            return self

        def __exit__(self, *_):
            pass

        def poll(self):
            value = next(self.keys, "Q")
            if isinstance(value, BaseException):
                raise value
            return value

    class Return:
        def __init__(self, *args, **kwargs):
            pass

        def execute(self):
            return SimpleNamespace(status="aborted" if return_reason else "complete",
                                   abort_reason=return_reason)

    reader, controller = Reader(), Controller()
    monkeypatch.setattr(app, "PX6DReader", lambda *a, **kw: reader)
    monkeypatch.setattr(app, "URRTDEController", lambda *a, **kw: controller)
    monkeypatch.setattr(app, "OperatorKeyboard", Keyboard)
    monkeypatch.setattr(app, "SafeReturnExecutor", Return)
    monkeypatch.setattr("builtins.input", lambda *_: "START")
    monkeypatch.setattr(app.time, "sleep", lambda *_: None)
    from tools import visualize_run
    monkeypatch.setattr(visualize_run, "create_visualization", lambda *args: tmp_path / "unused.png")
    monkeypatch.setattr(visualize_run, "create_strategy_debug", lambda *args: (tmp_path / "unused.png", []))
    args = SimpleNamespace(config=str(path), sensor="real", execute=True)
    termination = TerminationRecorder(output_root=tmp_path, print_fn=lambda *_: None)
    return args, termination, reader, controller


def test_sensor_connection_failure_has_reason_and_file_before_logger(monkeypatch, tmp_path):
    args, term, reader, controller = setup_scan(monkeypatch, tmp_path,
                                               connect_error=PX6DError("PX6D frame timeout"))
    assert app.run(args, termination=term) == 1
    assert term.record["reason"] == "STOP_SENSOR_ERROR"
    assert "PX6DError" in term.record["traceback"]
    assert reader.closed and controller.closed
    assert not controller.commands
    files = list(tmp_path.rglob("termination.json"))
    assert files
    assert json.loads(files[0].read_text())["reason"] == "STOP_SENSOR_ERROR"


@pytest.mark.parametrize("key", ["Q", "ESC", KeyboardInterrupt("operator interrupted")])
def test_operator_stop_records_pre_stop_policy_and_last_command(monkeypatch, tmp_path, key):
    args, term, _, controller = setup_scan(monkeypatch, tmp_path, keys=(None, key))
    app.run(args, termination=term)
    assert term.record["reason"] == "STOP_USER_REQUEST"
    assert term.record["policy_state"] == "TARGET_SEARCH"
    assert controller.commands and controller.stop_calls
    assert term.record["command"]["move"] is True
    assert term.record["anchor_pose"] is not None
    assert term.record["raw_wrench"] == [0] * 6


def test_unknown_policy_exception_preserves_traceback_and_prior_command(monkeypatch, tmp_path):
    args, term, _, controller = setup_scan(monkeypatch, tmp_path, keys=(None, None))

    class BrokenPolicy(RuleBasedPolicy):
        calls = 0

        def update(self, *args):
            self.calls += 1
            if self.calls == 2:
                raise RuntimeError("unexplained branch invariant")
            return super().update(*args)

    monkeypatch.setattr(app, "RuleBasedPolicy", BrokenPolicy)
    assert app.run(args, termination=term) == 1
    assert term.record["reason"] == "STOP_UNKNOWN_REASON"
    assert "unexplained branch invariant" in term.record["traceback"]
    assert term.record["policy_state"] == "TARGET_SEARCH"
    assert term.record["command"]["move"] is True
    assert term.record["raw_wrench"] == [0] * 6
    assert controller.stop_calls


@pytest.mark.parametrize("failure, expected", [
    (RobotError("speedL failed: controller rejected command"), "STOP_MOTION_ERROR"),
    (RobotError("TCP x=0.400000 is outside configured workspace"), "STOP_WORKSPACE_LIMIT"),
])
def test_motion_exception_records_the_attempted_command(monkeypatch, tmp_path, failure, expected):
    args, term, _, controller = setup_scan(monkeypatch, tmp_path, motion_error=failure)
    assert app.run(args, termination=term) == 1
    assert term.record["reason"] == expected
    assert term.record["command"]["move"] is True
    assert term.record["tcp_pose"][:3] == pytest.approx([.4, -.2, .3])
    assert "RobotError" in term.record["traceback"]
    assert controller.stop_calls


def test_invalid_wrench_stays_visible_in_standard_json(monkeypatch, tmp_path):
    args, term, _, _ = setup_scan(monkeypatch, tmp_path, wrench=Wrench(float("nan"), 0, 0, 0, 0, 0))
    app.run(args, termination=term)
    assert term.record["reason"] == "STOP_INVALID_WRENCH"
    path = next(tmp_path.rglob("termination.json"))
    # Reject JSON's nonstandard bare NaN/Infinity tokens; their diagnostic
    # representation still needs to survive normalization.
    data = json.loads(path.read_text(), parse_constant=lambda value: pytest.fail(value))
    assert data["reason"] == "STOP_INVALID_WRENCH"
    assert data["raw_wrench"][0] in ("NaN", "nan", None)


def test_cleanup_error_does_not_hide_scan_cause_or_skip_sensor_close(monkeypatch, tmp_path):
    args, term, reader, controller = setup_scan(monkeypatch, tmp_path,
                                               read_error=PX6DError("lost sensor response"),
                                               close_error=RuntimeError("opaque cleanup error"))
    with pytest.raises(RuntimeError, match="opaque cleanup error"):
        app.run(args, termination=term)
    assert term.record["reason"] == "STOP_SENSOR_ERROR"
    assert "lost sensor response" in term.record["traceback"]
    assert any("opaque cleanup error" in event.get("traceback", "") for event in term.events)
    assert controller.closed and reader.closed


def test_safe_return_failure_is_separate_from_original_user_stop(monkeypatch, tmp_path):
    args, term, _, _ = setup_scan(monkeypatch, tmp_path,
                                  return_reason="TCP x=0.400000 is outside configured workspace")
    assert app.run(args, termination=term) == 2
    assert term.record["reason"] == "STOP_USER_REQUEST"
    assert any(event["reason"] == "STOP_WORKSPACE_LIMIT" and "workspace" in event["detail"]
               for event in term.events)


def test_configuration_failure_without_devices_is_also_recorded(monkeypatch, tmp_path):
    def fail(_):
        raise FileNotFoundError("scan configuration missing")

    monkeypatch.setattr(app, "load_config", fail)
    term = TerminationRecorder(output_root=tmp_path, print_fn=lambda *_: None)
    with pytest.raises(FileNotFoundError):
        app.run(SimpleNamespace(config="missing.yaml"), termination=term)
    assert term.record["reason"] == "STOP_CONFIG_ERROR"
    assert "FileNotFoundError" in term.record["traceback"]
    assert list(tmp_path.rglob("termination.json"))


def test_system_exit_during_scan_keeps_exception_before_summary_fallback(monkeypatch, tmp_path):
    args, term, _, controller = setup_scan(monkeypatch, tmp_path,
                                           keys=(None, SystemExit("unexplained process exit")))
    with pytest.raises(SystemExit, match="unexplained process exit"):
        app.run(args, termination=term)
    assert term.record["reason"] == "STOP_UNKNOWN_REASON"
    assert "unexplained process exit" in term.record["detail"]
    assert "SystemExit" in term.record["traceback"]
    assert term.record["policy_state"] == "TARGET_SEARCH"
    assert controller.closed
