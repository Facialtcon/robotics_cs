"""Real queued logging with injected filesystem stalls and offline robot data."""
import json
import threading
import time

import pytest

from test_sensor_mount_session import fixture_session
from tools.sensor_mount_calibration import async_writer
from tools.sensor_mount_calibration.plan import make_plan
from tools.sensor_mount_calibration.storage import RunStore, StorageError


def block_worker_fsync(monkeypatch):
    entered, release = threading.Event(), threading.Event()
    original = async_writer.os.fsync

    def blocked(fd):
        if threading.current_thread().name == 'mount-calibration-logging' and not release.is_set():
            entered.set()
            if not release.wait(2.):
                raise OSError('test filesystem remained stalled')
        return original(fd)

    monkeypatch.setattr(async_writer.os, 'fsync', blocked)
    return entered, release


def queued_session(tmp_path, **logging_options):
    session, hardware, _, clock, start = fixture_session()
    store = RunStore.create(tmp_path, {'status': 'test'})
    store.start_async(batch_size=1, **logging_options)
    session.store = store
    return session, hardware, store, clock, start


def test_preflight_and_acquisition_continue_during_fsync_longer_than_observation_limit(tmp_path, monkeypatch):
    session, hardware, store, clock, start = queued_session(tmp_path)
    entered, release = block_worker_fsync(monkeypatch)

    def preflight(poses, heartbeat):
        assert entered.wait(1.)
        # Same phase that failed on site. The writer remains blocked while
        # fresh observations continue for longer than the unchanged 80 ms cap.
        until = time.monotonic() + .12
        count = 0
        while time.monotonic() < until:
            heartbeat()
            count += 1
            time.sleep(.005)
        assert count >= 2 and not release.is_set()
        return {'sample_count': count}

    hardware.preflight = preflight
    try:
        records = session.run(start, make_plan(start)[:1])
        assert len(records) == session.limits.samples_per_pose
        assert session.limits.observation_timeout_sec == .08
        assert not session.failed and hardware.stops == 0
        assert not release.is_set()  # Even post-preflight metadata was queued.
    finally:
        hardware.stop()
        release.set()
        store.finish_logging()
    raw = [json.loads(line) for line in (store.path / 'raw.jsonl').read_text().splitlines()]
    assert [row for row in raw if row['split'] == 'fit'] == records
    metadata = json.loads((store.path / 'metadata.json').read_text())
    assert metadata['preflight']['sample_count'] >= 2
    timings = [json.loads(line) for line in (store.path / 'timing.jsonl').read_text().splitlines()]
    assert all(t['hardware_read_wall_sec'] < .08 for t in timings)
    assert all('raw_enqueue_duration_sec' in t for t in timings)
    assert store.logging_diagnostics['worker_io']['fsync']['max_sec'] >= .12
    assert store.logging_diagnostics['durable'] is True


def test_sensor_timeout_stops_while_background_disk_is_blocked(tmp_path, monkeypatch):
    session, hardware, store, clock, start = queued_session(tmp_path)
    entered, release = block_worker_fsync(monkeypatch)
    preflight, command = hardware.preflight, hardware.command

    def wait_for_disk(poses, heartbeat):
        assert entered.wait(1.)
        return preflight(poses, heartbeat)

    def sensor_fails_on_tilt(target):
        command(target)
        hardware.failure = TimeoutError('fresh sensor request timed out')

    hardware.preflight = wait_for_disk
    hardware.command = sensor_fails_on_tilt
    try:
        with pytest.raises(TimeoutError, match='fresh sensor request') as caught:
            session.run(start, make_plan(start))
        assert not release.is_set()
        assert hardware.stops == len(hardware.commands) == 1
        assert session.failed
        assert caught.value.diagnostics['observation_timing']['stage'] == 'hardware_read'
    finally:
        release.set()
        store.finish_logging()
    events = [json.loads(line) for line in (store.path / 'timing.jsonl').read_text().splitlines()]
    assert events[-1]['kind'] == 'observation_failure'
    assert 'sensor request timed out' in events[-1]['error']


def test_queue_overflow_stops_preflight_without_any_move(tmp_path, monkeypatch):
    session, hardware, store, clock, start = queued_session(tmp_path, queue_capacity=8)
    entered, release = block_worker_fsync(monkeypatch)

    def fill_queue(poses, heartbeat):
        assert entered.wait(1.)
        for _ in range(20):
            heartbeat()
        pytest.fail('bounded queue should have rejected additional observations')

    hardware.preflight = fill_queue
    try:
        with pytest.raises(StorageError, match='overflow'):
            session.run(start, make_plan(start))
        assert hardware.stops == 1 and not hardware.commands
    finally:
        release.set()
        with pytest.raises(StorageError, match='overflow'):
            store.finish_logging()
    assert not store.logging_diagnostics['durable']
    assert store.logging_diagnostics['accepted'] == store.logging_diagnostics['written']
    with pytest.raises(StorageError):
        store.write_result({'accepted': True})


def test_writer_error_stops_preflight_and_preserves_no_result(tmp_path, monkeypatch):
    session, hardware, store, clock, start = queued_session(tmp_path)
    failed = threading.Event()
    original = async_writer.os.fsync

    def disk_full(fd):
        if threading.current_thread().name == 'mount-calibration-logging':
            failed.set()
            raise OSError('injected disk full')
        return original(fd)

    monkeypatch.setattr(async_writer.os, 'fsync', disk_full)

    def preflight(poses, heartbeat):
        assert failed.wait(1.)
        # Let the worker publish its failure; this wait is only test orchestration.
        until = time.monotonic() + 1.
        while store.logging_diagnostics['error'] is None and time.monotonic() < until:
            time.sleep(.001)
        heartbeat()
        pytest.fail('writer failure should stop preflight')

    hardware.preflight = preflight
    try:
        with pytest.raises(StorageError, match='disk full'):
            session.run(start, make_plan(start))
        assert hardware.stops == 1 and not hardware.commands
    finally:
        with pytest.raises(StorageError, match='disk full'):
            store.finish_logging()
    assert not (store.path / 'result.json').exists()
