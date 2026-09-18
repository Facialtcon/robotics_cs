"""Complete probe exports happen after motion, never on transfer control ticks."""

import csv
import json
from types import SimpleNamespace

import numpy as np
import pytest

from app import main as app
from policy.probe_episode import ProbeEpisode
from policy.rule_policy import RuleBasedPolicy, State
from sensor.px6d_reader import PX6DError
from test_app_termination import setup_scan


def instrument_export(monkeypatch, controller):
    events, exports = [], []
    motion_state = {"moving": False}
    original_command, original_stop = controller.command_planar_velocity, controller.stop
    original_export, original_close = app.write_probe_logs, app.ExperimentLogger.close

    def command(*args):
        motion_state["moving"] = True
        events.append("motion")
        return original_command(*args)

    def stop():
        original_stop()
        motion_state["moving"] = False
        events.append("stop")

    def export(directory, episodes):
        assert not motion_state["moving"], "full probe history was exported during motion"
        events.append("probe_export")
        original_export(directory, episodes)
        exports.append((directory, [episode.to_record() for episode in episodes]))

    def close(logger):
        # Workspace contact export and the result figures consume these files.
        assert (logger.run_dir / "probe_episodes.json").is_file()
        assert (logger.run_dir / "probe_forces.csv").is_file()
        events.append("logger_close")
        return original_close(logger)

    monkeypatch.setattr(controller, "command_planar_velocity", command)
    monkeypatch.setattr(controller, "stop", stop)
    monkeypatch.setattr(controller, "safe_stop_motion", stop)
    monkeypatch.setattr(app, "write_probe_logs", export)
    monkeypatch.setattr(app.ExperimentLogger, "close", close)
    # This test's subject is the main scan exporter, independent of whether
    # the machine running pytest has taught a workspace calibration.
    from workspace import workspace_logger
    monkeypatch.setattr(workspace_logger, "create_optional_workspace_logger", lambda *_: None)
    return events, exports


def completed_episode(probe_id):
    anchor = np.array([.4, -.2, .3, 0., 3.14, 0.])
    episode = ProbeEpisode(probe_id, anchor, [1., 0.], float(probe_id), .01,
                           "LOCAL_INITIALIZATION", "INITIALIZATION")
    episode.contact_pose = anchor + np.array([.002, 0., 0., 0., 0., 0.])
    episode.end_pose = episode.contact_pose.copy()
    episode.returned_pose = anchor.copy()
    episode.probe_end_time = probe_id + .2
    episode.return_end_time = probe_id + .4
    episode.phase = "DONE"
    episode.outcome = "CONTACT"
    episode.return_completed = True
    episode.force_samples = [(probe_id + index * .01, index * .1, "RETURN" if index > 3 else "PROBE")
                             for index in range(7 + probe_id)]
    return episode


@pytest.mark.parametrize("exit_value, expected_code, expected_reason", [
    ("Q", 0, "STOP_USER_REQUEST"),
    ("ESC", 130, "STOP_USER_REQUEST"),
    (KeyboardInterrupt("operator interrupted"), 130, "STOP_USER_REQUEST"),
    (RuntimeError("offline loop failure"), 1, "STOP_UNKNOWN_REASON"),
    (SystemExit("offline process exit"), None, "STOP_UNKNOWN_REASON"),
])
def test_multiple_transfer_ticks_export_once_only_after_stop(
    monkeypatch, tmp_path, exit_value, expected_code, expected_reason
):
    args, termination, reader, controller = setup_scan(
        monkeypatch, tmp_path, keys=(None, None, None, exit_value)
    )
    events, exports = instrument_export(monkeypatch, controller)
    policies = []

    class TransferPolicy(RuleBasedPolicy):
        def __init__(self, config):
            super().__init__(config)
            self.state = State.BOUNDARY_TRACKING
            self.sub_state = "RAY_CLEARANCE"
            self.probe_episodes = [completed_episode(0), completed_episode(1)]
            self.calls = 0
            policies.append(self)

        def update(self, timestamp, raw, processed, robot):
            # Between probes the real policy also has no active episode.
            # This used to trigger a full rewrite after every movement command.
            assert self.active_episode is None
            assert exports == []
            self.calls += 1
            target = robot.pose.copy()
            target[0] += .001
            self.reason = "offline transfer fixture"
            return self._command(robot.pose, (target, .003), processed)

    monkeypatch.setattr(app, "RuleBasedPolicy", TransferPolicy)
    if isinstance(exit_value, SystemExit):
        with pytest.raises(SystemExit, match="offline process exit"):
            app.run(args, termination=termination)
    else:
        assert app.run(args, termination=termination) == expected_code
    assert policies[0].calls == 3
    assert len(exports) == 1
    directory, expected_records = exports[0]
    assert json.loads((directory / "probe_episodes.json").read_text()) == json.loads(json.dumps(expected_records))
    with (directory / "probe_forces.csv").open(newline="") as handle:
        force_rows = list(csv.DictReader(handle))
    assert len(force_rows) == sum(len(record["force_samples"]) for record in expected_records)
    assert {row["probe_id"] for row in force_rows} == {"0", "1"}
    with (directory / "samples.csv").open(newline="") as handle:
        samples = list(csv.DictReader(handle))
    transfer_samples = [row for row in samples if row["policy_sub_state"] == "RAY_CLEARANCE"]
    assert len(transfer_samples) == 3
    with (directory / "full_log.csv").open(newline="") as handle:
        assert list(csv.DictReader(handle)) == samples
    assert events.index("probe_export") > max(i for i, event in enumerate(events) if event == "motion")
    assert events.index("probe_export") < events.index("logger_close")
    assert termination.record["reason"] == expected_reason
    assert controller.closed and reader.closed


@pytest.mark.parametrize("exit_mode", ["normal", "sensor_failure"])
def test_incomplete_active_probe_history_survives_exit(monkeypatch, tmp_path, exit_mode):
    args, termination, reader, controller = setup_scan(monkeypatch, tmp_path, keys=(None, None, "Q"))
    events, exports = instrument_export(monkeypatch, controller)
    policies = []

    class CapturedPolicy(RuleBasedPolicy):
        def __init__(self, config):
            super().__init__(config)
            policies.append(self)

    monkeypatch.setattr(app, "RuleBasedPolicy", CapturedPolicy)
    if exit_mode == "sensor_failure":
        original_read = reader.read_wrench
        reads = []

        def read():
            reads.append(None)
            if len(reads) == 2:
                raise PX6DError("offline lost PX6D response")
            return original_read()

        monkeypatch.setattr(reader, "read_wrench", read)
    assert app.run(args, termination=termination) == (0 if exit_mode == "normal" else 1)
    assert len(exports) == 1
    directory, records = exports[0]
    assert len(records) == 1
    assert records[0]["force_samples"] == policies[0].probe_episodes[0].force_samples
    assert len(records[0]["force_samples"]) == (2 if exit_mode == "normal" else 1)
    assert not records[0]["return_completed"]
    assert records[0]["returned_pose"] is None
    assert (directory / "summary.json").is_file()
    assert (directory / "termination.json").is_file()
    expected_reason = "STOP_USER_REQUEST" if exit_mode == "normal" else "STOP_SENSOR_ERROR"
    assert termination.record["reason"] == expected_reason
    assert "stop" in events[:events.index("probe_export")]


def test_normal_export_follows_the_existing_safe_return(monkeypatch, tmp_path):
    args, termination, _, controller = setup_scan(monkeypatch, tmp_path)
    events, exports = instrument_export(monkeypatch, controller)

    class Return:
        def __init__(self, *args, **kwargs):
            pass

        def execute(self):
            events.append("safe_return_start")
            controller.command_planar_velocity([1., 0.], .001, .01)
            controller.stop()
            events.append("safe_return_finished")
            return SimpleNamespace(status="complete", abort_reason="")

    monkeypatch.setattr(app, "SafeReturnExecutor", Return)
    assert app.run(args, termination=termination) == 0
    assert len(exports) == 1
    assert events.index("probe_export") > events.index("safe_return_finished")


def test_later_logging_failure_does_not_repeat_a_successful_probe_export(monkeypatch, tmp_path):
    args, termination, _, controller = setup_scan(monkeypatch, tmp_path)
    _, exports = instrument_export(monkeypatch, controller)
    original_summary = app.ExperimentLogger.write_summary
    attempts = []

    def summary(logger, *args, **kwargs):
        attempts.append(None)
        if len(attempts) == 1:
            raise OSError("offline summary write failure")
        return original_summary(logger, *args, **kwargs)

    monkeypatch.setattr(app.ExperimentLogger, "write_summary", summary)
    assert app.run(args, termination=termination) == 1
    assert len(exports) == 1
    assert len(attempts) == 2
    assert termination.record["reason"] == "STOP_USER_REQUEST"
    assert any("offline summary write failure" in event.get("detail", "") for event in termination.events)
