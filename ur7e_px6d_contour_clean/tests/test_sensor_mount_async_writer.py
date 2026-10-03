"""No devices: exercise disk stalls, bounded failure, and durable log ordering."""
import json
from pathlib import Path
import threading

import numpy as np
import pytest

from tools.sensor_mount_calibration import async_writer
from tools.sensor_mount_calibration.storage import RunStore, StorageError


def raw(index=0):
    return dict(pose_id=index, split="preflight", actual_tcp_pose=np.zeros(6),
                raw_wrench=np.arange(6, dtype=float), host_monotonic=float(index),
                utc_time="2026-10-03T00:00:00Z", robot_timestamp=float(index))


def lines(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def block_first_sync(monkeypatch):
    entered, release = threading.Event(), threading.Event()
    original = async_writer.AsyncLogWriter._sync

    def blocked(writer, handle):
        if not entered.is_set():
            entered.set()
            if not release.wait(3.):
                raise OSError("test did not release disk stall")
        return original(writer, handle)

    monkeypatch.setattr(async_writer.AsyncLogWriter, "_sync", blocked)
    return entered, release


def test_stalled_disk_longer_than_observation_deadline_does_not_block_producer(tmp_path, monkeypatch):
    store = RunStore.create(tmp_path, {"initial": "preserved"})
    entered, release = block_first_sync(monkeypatch)
    store.start_async(batch_size=1)
    store.append_raw(raw(0))
    try:
        assert entered.wait(1.)
        # Hold the worker for >80 ms (the former observation deadline). The
        # producer must finish while that very same disk operation stays held.
        assert not release.wait(.12)
        second = raw(1)
        metadata = {"plan": [1, 2]}
        store.append_raw(second)
        store.write_metadata(metadata)
        store.write_metadata({"status": "sampled"})
        store.append_timing({"phase": "preflight", "elapsed_sec": .001})
        store.check_logging_health()
        second["raw_wrench"][:] = 999
        metadata["plan"].append(999)
        assert store.logging_diagnostics["queue_depth"] == 4
        assert not release.is_set()
        with pytest.raises(StorageError, match="finish logging"):
            store.write_result({"accepted": True})
    finally:
        release.set()
        store.finish_logging()
    saved = lines(store.path / "raw.jsonl")
    assert [record["pose_id"] for record in saved] == [0, 1]
    assert saved[1]["raw_wrench"] == list(range(6))
    assert json.loads((store.path / "metadata.json").read_text()) == {
        "initial": "preserved", "plan": [1, 2], "status": "sampled"}
    assert lines(store.path / "timing.jsonl") == [{"phase": "preflight", "elapsed_sec": .001}]
    assert store.logging_diagnostics["durable"]
    store.write_result({"accepted": True})
    assert json.loads((store.path / "result.json").read_text()) == {"accepted": True}


def test_active_producer_performs_no_filesystem_calls(tmp_path, monkeypatch):
    store = RunStore.create(tmp_path, {})
    producer = threading.get_ident()
    active = True
    observed = []

    def guard(name, original):
        def checked(*args, **kwargs):
            if active:
                assert threading.get_ident() != producer, f"producer performed {name}"
                observed.append(name)
            return original(*args, **kwargs)
        return checked

    for owner, name in ((Path, "open"), (Path, "exists"), (Path, "read_text"),
                        (async_writer.os, "open"), (async_writer.os, "fsync"),
                        (async_writer.os, "replace")):
        monkeypatch.setattr(owner, name, guard(name, getattr(owner, name)))
    store.start_async()
    for index in range(20):
        store.append_raw(raw(index))
        store.append_timing({"observation": index})
    store.write_metadata({"status": "preflight_completed"})
    store.check_logging_health()
    store.finish_logging()
    active = False
    assert "fsync" in observed and "replace" in observed
    assert len(lines(store.path / "raw.jsonl")) == 20
    assert len(lines(store.path / "timing.jsonl")) == 20


def test_batching_and_final_drain_keep_every_record_without_per_frame_fsync(tmp_path):
    store = RunStore.create(tmp_path, {})
    store.start_async(batch_size=64, flush_interval_sec=10.)
    for index in range(130):
        store.append_raw(raw(index))
    store.finish_logging()
    diagnostics = store.logging_diagnostics
    assert diagnostics["accepted"]["raw"] == diagnostics["written"]["raw"] == 130
    assert diagnostics["worker_io"]["fsync"]["count"] < 10
    assert diagnostics["worker_io"]["write"]["count"] == 130
    assert diagnostics["queue_depth"] == 0 and diagnostics["durable"]
    assert [record["pose_id"] for record in lines(store.path / "raw.jsonl")] == list(range(130))
    assert not (store.path / "timing.jsonl").exists()
    json.dumps(diagnostics, allow_nan=False)


def test_invalid_records_are_rejected_before_enqueue(tmp_path):
    store = RunStore.create(tmp_path, {})
    store.start_async()
    try:
        invalid = raw()
        invalid["raw_wrench"][0] = float("nan")
        with pytest.raises(ValueError, match="six finite"):
            store.append_raw(invalid)
        with pytest.raises(ValueError):
            store.append_timing({"elapsed_sec": float("nan")})
        with pytest.raises(TypeError, match="mapping"):
            store.write_metadata([])
        assert store.logging_diagnostics["accepted"] == {"raw": 0, "timing": 0, "metadata": 0}
    finally:
        store.finish_logging()
    assert (store.path / "raw.jsonl").read_bytes() == b""


def test_overflow_latches_failure_and_does_not_silently_drop_accepted_samples(tmp_path, monkeypatch):
    store = RunStore.create(tmp_path, {})
    entered, release = block_first_sync(monkeypatch)
    store.start_async(queue_capacity=2, batch_size=1)
    store.append_raw(raw(0))
    try:
        assert entered.wait(1.)
        store.append_raw(raw(1))
        store.append_raw(raw(2))
        with pytest.raises(StorageError, match="queue overflow"):
            store.append_raw(raw(3))
        with pytest.raises(StorageError, match="queue overflow"):
            store.check_logging_health()
    finally:
        release.set()
        with pytest.raises(StorageError, match="queue overflow"):
            store.finish_logging()
    assert [record["pose_id"] for record in lines(store.path / "raw.jsonl")] == [0, 1, 2]
    assert store.logging_diagnostics["accepted"]["raw"] == 3
    assert not store.logging_diagnostics["durable"]
    with pytest.raises(StorageError, match="queue overflow"):
        store.write_result({"accepted": True})
    assert not (store.path / "result.json").exists()
    store.write_metadata({"status": "aborted", "logging": store.logging_diagnostics})
    assert json.loads((store.path / "metadata.json").read_text())["status"] == "aborted"


def test_stalled_inflight_write_is_detected_without_any_new_record(tmp_path, monkeypatch):
    store = RunStore.create(tmp_path, {})
    entered, release = block_first_sync(monkeypatch)
    clock = [100.]
    store.start_async(batch_size=1, max_backlog_sec=2., clock=lambda: clock[0])
    store.append_raw(raw())
    try:
        assert entered.wait(1.)
        assert store.logging_diagnostics["queue_depth"] == 0
        clock[0] += 3.
        with pytest.raises(StorageError, match="backlog stalled"):
            store.check_logging_health()
        clock[0] = 100.
        with pytest.raises(StorageError, match="backlog stalled"):
            store.check_logging_health()
    finally:
        release.set()
        with pytest.raises(StorageError, match="backlog stalled"):
            store.finish_logging()


@pytest.mark.parametrize("operation", ["write", "fsync", "metadata"])
def test_worker_failure_prevents_result_even_without_further_samples(tmp_path, monkeypatch, operation):
    store = RunStore.create(tmp_path, {})
    failed = threading.Event()
    original = async_writer.AsyncLogWriter._timed

    def fail(writer, name, action):
        if name == ("write" if operation == "write" else "fsync"):
            failed.set()
            raise OSError(f"injected {operation} failure")
        return original(writer, name, action)

    monkeypatch.setattr(async_writer.AsyncLogWriter, "_timed", fail)
    store.start_async(batch_size=1)
    if operation == "metadata":
        store.write_metadata({"status": "preflight"})
    else:
        store.append_raw(raw())
    assert failed.wait(1.)
    with pytest.raises(StorageError, match=f"injected {operation} failure"):
        store.finish_logging()
    with pytest.raises(StorageError, match="logging worker failed"):
        store.check_logging_health()
    with pytest.raises(StorageError):
        store.write_result({"accepted": True})
    assert not (store.path / "result.json").exists()


def test_bounded_finish_timeout_keeps_late_worker_from_racing_abort_metadata(tmp_path, monkeypatch):
    store = RunStore.create(tmp_path, {"status": "initial"})
    entered, release = block_first_sync(monkeypatch)
    store.start_async(batch_size=1)
    store.append_raw(raw())
    try:
        assert entered.wait(1.)
        store.write_metadata({"status": "queued"})
        with pytest.raises(StorageError, match="drain timeout"):
            store.finish_logging(timeout_sec=.01)
        assert store.logging_diagnostics["worker_alive"]
        with pytest.raises(StorageError, match="drain timeout"):
            store.write_metadata({"status": "aborted"})
        with pytest.raises(StorageError, match="drain timeout"):
            store.write_result({"accepted": True})
        assert json.loads((store.path / "metadata.json").read_text())["status"] == "initial"
    finally:
        release.set()
        with pytest.raises(StorageError, match="drain timeout"):
            store.finish_logging()
    assert json.loads((store.path / "metadata.json").read_text())["status"] == "queued"
    store.write_metadata({"status": "aborted"})
    assert json.loads((store.path / "metadata.json").read_text())["status"] == "aborted"
    assert not (store.path / "result.json").exists()


def test_close_failure_is_not_reported_as_durable(tmp_path, monkeypatch):
    store = RunStore.create(tmp_path, {})
    original = Path.open

    class FailedClose:
        def __init__(self, handle):
            self.handle = handle

        def __getattr__(self, name):
            return getattr(self.handle, name)

        def close(self):
            self.handle.close()
            raise OSError("injected close failure")

    def open_with_failed_close(path, *args, **kwargs):
        handle = original(path, *args, **kwargs)
        if path.name == "raw.jsonl" and threading.current_thread().name == "mount-calibration-logging":
            return FailedClose(handle)
        return handle

    monkeypatch.setattr(Path, "open", open_with_failed_close)
    store.start_async()
    store.append_raw(raw())
    with pytest.raises(StorageError, match="close failure"):
        store.finish_logging()
    assert not store.logging_diagnostics["worker_alive"]
    assert not store.logging_diagnostics["durable"]
    with pytest.raises(StorageError, match="close failure"):
        store.write_result({"accepted": True})


def test_timing_sync_mode_is_optional_and_durable(tmp_path):
    store = RunStore.create(tmp_path, {})
    assert not (store.path / "timing.jsonl").exists()
    store.append_timing({"phase": "offline", "elapsed_sec": .1})
    assert lines(store.path / "timing.jsonl") == [{"phase": "offline", "elapsed_sec": .1}]
    store.finish_logging()
    store.check_logging_health()


@pytest.mark.parametrize("options", [dict(queue_capacity=0), dict(batch_size=0),
                                   dict(max_backlog_sec=float("nan")),
                                   dict(flush_interval_sec=0.)])
def test_invalid_logging_limits_rejected(tmp_path, options):
    store = RunStore.create(tmp_path, {})
    with pytest.raises(ValueError):
        store.start_async(**options)
    assert store.logging_diagnostics["mode"] == "sync"
