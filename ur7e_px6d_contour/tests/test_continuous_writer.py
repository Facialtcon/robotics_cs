"""Finite, hardware-free checks for the real continuous run's log queue."""

import csv
import json
from threading import Event
import time

import numpy as np
import pytest

from core.models import PolicyCommand, PolicyWaypoint, RobotState, Wrench
from experiment_logging.continuous_writer import ContinuousLogWriter, ContinuousLogError
from experiment_logging.data_logger import ExperimentLogger


def sample(timestamp=1.0):
    wrench = Wrench(1, 2, 3, 0, 0, 0)
    robot = RobotState(timestamp, np.full(6, timestamp), np.zeros(6))
    command = PolicyCommand("READY", False, np.zeros(2), 0, np.zeros(6), False, False)
    return timestamp, wrench, wrench, robot, command, np.array([1., 0.]), np.array([0., 1.])


def logger(tmp_path):
    return ExperimentLogger(tmp_path, {}, extra_sample_fields=("test_extra",), workspace_logging=False)


def block_sample(wrapped):
    entered, release = Event(), Event()
    original = wrapped.log_sample

    def blocked(*args, **kwargs):
        entered.set()
        assert release.wait(1.0), "test did not release writer"
        original(*args, **kwargs)

    wrapped.log_sample = blocked
    return entered, release


def test_delayed_write_does_not_block_enqueue_or_rewind_live_observation(tmp_path):
    wrapped = logger(tmp_path)
    recorder = wrapped.termination
    entered, release = block_sample(wrapped)
    writer = ContinuousLogWriter(wrapped)
    try:
        started = time.monotonic()
        writer.log_sample(*sample(1.0))
        assert time.monotonic() - started < .045
        assert entered.wait(.5)
        writer.log_sample(*sample(2.0))
        # Keep the writer behind the control thread by more than the 30 ms cycle limit.
        time.sleep(.045)
        assert wrapped.termination is None
        assert writer.termination is recorder
        assert recorder._observed["timestamp"] == 2.0
        release.set()
        writer.flush()
        assert recorder._observed["timestamp"] == 2.0
    finally:
        release.set()
        writer.close()
    assert wrapped.termination is recorder


def test_snapshots_order_and_complete_original_csv_are_preserved(tmp_path):
    wrapped = logger(tmp_path)
    entered, release = block_sample(wrapped)
    writer = ContinuousLogWriter(wrapped)
    first = sample(1.0)
    payload = {"phase": "first", "numbers": [1]}
    try:
        writer.log_sample(*first, extra={"test_extra": "saved"})
        assert entered.wait(.5)
        first[3].pose[:] = 99
        first[5][:] = 99
        writer.write_json("return_status.json", payload)
        payload["numbers"][0] = 99
        writer.log_waypoint(PolicyWaypoint(1., "READY", "sampled", np.ones(6)))
        writer.log_sample(*sample(2.0), extra={"test_extra": "second"})
        release.set()
        writer.flush()
        assert json.loads((writer.run_dir / "return_status.json").read_text())["numbers"] == [1]
        writer.write_json("return_status.json", {"phase": "second"})
    finally:
        release.set()
        writer.close()
    writer.close()
    with (writer.run_dir / "samples.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    with (writer.run_dir / "full_log.csv").open() as stream:
        assert rows == list(csv.DictReader(stream))
    assert [float(row["tcp_x"]) for row in rows] == [1., 2.]
    assert rows[0]["target_direction_x"] == "1.0"
    assert rows[0]["test_extra"] == "saved"
    assert json.loads((writer.run_dir / "return_status.json").read_text())["phase"] == "second"
    with (writer.run_dir / "policy_waypoints.csv").open() as stream:
        assert len(list(csv.DictReader(stream))) == 1


def test_worker_failure_is_seen_later_and_still_allows_final_diagnostics(tmp_path):
    wrapped = logger(tmp_path)
    failed = Event()

    def fail(*args, **kwargs):
        failed.set()
        raise OSError("test disk failure")

    wrapped.log_sample = fail
    writer = ContinuousLogWriter(wrapped)
    writer.log_sample(*sample())
    assert failed.wait(.5)
    # A final drain also waits for the worker to publish the exception.
    with pytest.raises(ContinuousLogError, match="test disk failure"):
        writer.flush()
    with pytest.raises(ContinuousLogError, match="test disk failure"):
        writer.check_health()
    with pytest.raises(ContinuousLogError, match="test disk failure"):
        writer.write_final_waypoints([
            PolicyWaypoint(2., "STOPPED", "SAFETY_STOP", np.ones(6))])
    with pytest.raises(ContinuousLogError, match="test disk failure"):
        writer.write_summary("STOPPED", "log failed", 0)
    assert json.loads((writer.run_dir / "summary.json").read_text())["reason"] == "log failed"
    with pytest.raises(ContinuousLogError, match="test disk failure"):
        writer.close()
    assert wrapped._sample_file.closed
    with (writer.run_dir / "policy_waypoints.csv").open() as stream:
        assert [row["event_type"] for row in csv.DictReader(stream)] == ["SAFETY_STOP"]
    writer.close()


def test_failure_does_not_discard_later_already_accepted_records(tmp_path):
    wrapped = logger(tmp_path)
    entered, release = Event(), Event()
    original = wrapped.log_sample

    def fail_first(*args, **kwargs):
        if args[0] == 1.0:
            entered.set()
            assert release.wait(1.0)
            raise OSError("first record failed")
        original(*args, **kwargs)

    wrapped.log_sample = fail_first
    writer = ContinuousLogWriter(wrapped)
    try:
        writer.log_sample(*sample(1.0))
        assert entered.wait(.5)
        writer.log_sample(*sample(2.0))
        writer.write_json("return_status.json", {"phase": "saved"})
    finally:
        release.set()
        with pytest.raises(ContinuousLogError, match="first record failed"):
            writer.close()
    with (writer.run_dir / "samples.csv").open() as stream:
        assert [float(row["monotonic_sec"]) for row in csv.DictReader(stream)] == [2.0]
    assert json.loads((writer.run_dir / "return_status.json").read_text())["phase"] == "saved"


def test_final_event_failure_still_attempts_following_stop_event(tmp_path):
    wrapped = logger(tmp_path)
    original = wrapped.log_waypoint

    def fail_first(event):
        if event.event_type == "BEFORE_STOP":
            raise OSError("test final waypoint failure")
        original(event)

    wrapped.log_waypoint = fail_first
    writer = ContinuousLogWriter(wrapped)
    with pytest.raises(ContinuousLogError, match="test final waypoint failure"):
        writer.write_final_waypoints([
            PolicyWaypoint(1., "STOPPED", "BEFORE_STOP", np.ones(6)),
            PolicyWaypoint(2., "STOPPED", "SAFETY_STOP", np.ones(6))])
    with pytest.raises(ContinuousLogError, match="test final waypoint failure"):
        writer.write_summary("STOPPED", "log failed", 0)
    with pytest.raises(ContinuousLogError, match="test final waypoint failure"):
        writer.close()
    with (writer.run_dir / "policy_waypoints.csv").open() as stream:
        assert [row["event_type"] for row in csv.DictReader(stream)] == ["SAFETY_STOP"]
    assert json.loads((writer.run_dir / "summary.json").read_text())["reason"] == "log failed"


def test_json_write_failure_is_reported_and_files_are_closed(tmp_path):
    wrapped = logger(tmp_path)
    writer = ContinuousLogWriter(wrapped)
    writer.write_json("return_status.json", {"not_serializable": object()})
    with pytest.raises(ContinuousLogError, match="not JSON serializable"):
        writer.close()
    assert wrapped._sample_file.closed
    with pytest.raises(ContinuousLogError):
        writer.log_sample(*sample())


@pytest.mark.parametrize("recovers", [False, True])
def test_first_handle_close_failure_does_not_skip_other_handle_cleanup(tmp_path, recovers):
    wrapped = logger(tmp_path)
    original = wrapped._sample_file

    class FailFirstClose:
        calls = 0

        @property
        def closed(self):
            return original.closed

        def close(self):
            self.calls += 1
            if self.calls == 1 or not recovers:
                raise OSError("test first handle close failure")
            original.close()

    failing_handle = FailFirstClose()
    wrapped._sample_file = failing_handle
    writer = ContinuousLogWriter(wrapped)
    try:
        with pytest.raises(ContinuousLogError, match="test first handle close failure") as error:
            writer.close()
        assert isinstance(error.value.__cause__, OSError)
        assert failing_handle.closed is recovers
        assert failing_handle.calls == 2
        for name in ("_full_log_file", "_boundary_file", "_waypoint_file", "_recovery_ray_file"):
            assert getattr(wrapped, name).closed
        writer.close()
        assert failing_handle.calls == 2
    finally:
        original.close()


def test_queue_overflow_is_fatal_without_discarding_accepted_records(tmp_path):
    wrapped = logger(tmp_path)
    entered, release = block_sample(wrapped)
    writer = ContinuousLogWriter(wrapped, capacity=2)
    try:
        writer.log_sample(*sample(1.0))
        assert entered.wait(.5)
        writer.log_sample(*sample(2.0))
        with pytest.raises(ContinuousLogError, match="capacity"):
            writer.log_sample(*sample(3.0))
        with pytest.raises(ContinuousLogError, match="capacity"):
            writer.check_health()
    finally:
        release.set()
        with pytest.raises(ContinuousLogError, match="capacity"):
            writer.close()
    with (writer.run_dir / "samples.csv").open() as stream:
        assert len(list(csv.DictReader(stream))) == 2


def test_backlog_deadline_includes_inflight_record_and_close_is_bounded(tmp_path):
    wrapped = logger(tmp_path)
    entered, release = block_sample(wrapped)
    writer = ContinuousLogWriter(wrapped, max_pending_sec=.04)
    writer.log_sample(*sample())
    assert entered.wait(.5)
    time.sleep(.05)
    with pytest.raises(ContinuousLogError, match="pending"):
        writer.check_health()
    started = time.monotonic()
    with pytest.raises(ContinuousLogError):
        writer.close()
    assert time.monotonic() - started < .2
    # Never close handles out from under a blocked write.
    assert not wrapped._sample_file.closed
    release.set()
    writer._thread.join(.5)
    assert not writer._thread.is_alive()
    assert wrapped._sample_file.closed
    writer.close()


def test_final_snapshot_restores_current_recorder_and_drains_first(tmp_path):
    wrapped = logger(tmp_path)
    writer = ContinuousLogWriter(wrapped)
    latest = sample(3.0)
    writer.log_sample(*sample(1.0))
    writer.write_stop_snapshot(latest[3], latest[1], latest[2], "STOPPED")
    writer.close()
    assert json.loads((writer.run_dir / "scan_stop_snapshot.json").read_text())["tcp_pose"] == [3.] * 6
    assert writer.termination._observed["robot"].pose.tolist() == [3.] * 6


@pytest.mark.parametrize("kwargs", [
    {"capacity": 0}, {"capacity": 1.5}, {"max_pending_sec": 0},
    {"max_pending_sec": float("nan")}, {"max_pending_sec": float("inf")},
])
def test_invalid_bounds_are_rejected_before_taking_logger_ownership(tmp_path, kwargs):
    wrapped = logger(tmp_path)
    try:
        with pytest.raises(ValueError):
            ContinuousLogWriter(wrapped, **kwargs)
        assert wrapped.termination is not None
    finally:
        wrapped.close()
