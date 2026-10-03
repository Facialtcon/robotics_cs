"""Offline full-plan acquisition and stop behavior; all I/O is injected."""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from tools.sensor_mount_calibration.plan import (
    Limits, make_plan, interpolate_path, matrix_to_rotvec,
)
from tools.sensor_mount_calibration.session import Session, CalibrationStopped
from tools.sensor_mount_calibration.solver import calibrate, rotation_from_rotvec, CalibrationError


class Clock:
    now = 10.
    def __call__(self):
        return self.now
    def sleep(self, delay):
        self.now += delay


class Store:
    def __init__(self):
        self.raw = []
        self.metadata = {}
        self.timings = []
    def append_raw(self, record):
        self.raw.append(dict(record))
    def write_metadata(self, data):
        self.metadata.update(data)
    def check_logging_health(self):
        pass
    def append_timing(self, timing):
        self.timings.append(dict(timing))


class FakeHardware:
    def __init__(self, clock, start):
        self.clock = clock
        self.pose = np.array(start)
        self.commands = []
        self.stops = 0
        self.active = False
        self.failure = None
        self.mount = rotation_from_rotvec([.5, -.8, 1.2])
    def read(self):
        self.clock.sleep(.01)
        if self.failure:
            raise self.failure
        gravity = rotation_from_rotvec(self.pose[3:]).T @ [0, 0, -1]
        raw = np.r_[np.array([1., -2., .5])-6.2*self.mount.T@gravity, [0., .01, -.01]]
        return dict(actual_tcp_pose=self.pose.tolist(), tcp_speed=[0.]*6,
                    actual_q=[0, -1, -1, -.5, 1, 0], actual_qd=[0.]*6,
                    raw_wrench=raw.tolist(), host_monotonic=self.clock(),
                    robot_timestamp=self.clock(), utc_time='2026-10-03T00:00:00Z')
    def activate(self):
        self.active = True
    def preflight(self, poses, heartbeat):
        self.approved = list(poses)
        heartbeat()
        return {'sample_count': len(poses)}
    def command(self, target):
        assert self.active and self.stops == 0
        assert np.allclose(target, self.approved[len(self.commands)])
        self.commands.append(target)
        self.pose = np.array(target)
    def stop(self):
        self.stops += 1


def fixture_session(**overrides):
    clock, store = Clock(), Store()
    limits = replace(Limits(), settle_hold_sec=.03, samples_per_pose=25, **overrides)
    start = [.4, -.2, .3, 0., np.pi, 0.]
    hardware = FakeHardware(clock, start)
    keyboard = SimpleNamespace(poll=lambda: None)
    session = Session(hardware, store, keyboard, limits, clock=clock, sleep=clock.sleep)
    return session, hardware, store, clock, start


def test_full_plan_raw_collection_fits_actual_mount_and_returns_only_normally():
    session, hardware, store, clock, start = fixture_session()
    plan = make_plan(start)
    records = session.run(start, plan)
    result = calibrate(records)
    assert result['force_sign'] == -1
    assert np.allclose(result['rotation_sensor_to_tool'], hardware.mount)
    assert len(hardware.commands) == 32
    assert np.allclose(hardware.pose, start)
    assert len(hardware.approved) == len(hardware.commands)
    assert len({r['pose_id'] for r in records if r['split']=='validation'}) == 4
    assert set(r['split'] for r in store.raw) == {'motion', 'settling', 'preflight', 'fit', 'validation', 'reference', 'stability'}
    assert len([r for r in store.raw if r['split'] == 'reference']) == 16 * session.limits.samples_per_pose
    assert all(r['split'] in ('fit', 'validation') for r in records)
    assert len(store.metadata['reference_checks']) == 16
    assert all(check['force_delta_norm_N'] < 1e-8 for check in store.metadata['reference_checks'])
    assert hardware.stops == 0


def test_stationary_preflight_drift_stops_before_first_move():
    session, hardware, store, clock, start = fixture_session()
    read = hardware.read
    started = clock()

    def drifting_read():
        record = read()
        record['raw_wrench'][2] -= .003 * (clock()-started)
        return record

    def long_preflight(poses, heartbeat):
        hardware.approved = list(poses)
        for _ in range(400):
            clock.sleep(.065)
            heartbeat()
        return {'sample_count': len(poses)}

    hardware.read, hardware.preflight = drifting_read, long_preflight
    with pytest.raises(CalibrationError, match='原地原始力持续漂移') as caught:
        session.run(start, make_plan(start))
    assert hardware.commands == [] and hardware.stops == 1
    assert session.records == [] and session.failed
    assert caught.value.diagnostics['calibration_stage'] == 'stationary_trend'
    assert caught.value.diagnostics['force_slope_norm_N_per_s'] == pytest.approx(.003)
    assert len([r for r in store.raw if r['split'] == 'preflight']) == 400


def test_return_reference_drift_stops_without_next_spoke_or_bias_correction():
    session, hardware, store, clock, start = fixture_session()
    read = hardware.read
    reference_before = read()['raw_wrench']

    def drifting_return():
        record = read()
        if len(hardware.commands) >= 2:
            record['raw_wrench'][2] += .2
        return record

    hardware.read = drifting_return
    with pytest.raises(CalibrationError, match='同姿态原始力/矩漂移') as caught:
        session.run(start, make_plan(start))
    assert len(hardware.commands) == 2 and hardware.stops == 1
    assert caught.value.diagnostics['calibration_stage'] == 'reference'
    assert caught.value.diagnostics['force_delta_norm_N'] == pytest.approx(.2)
    context = caught.value.diagnostics['motion_context']
    assert context == dict(plan_index=2, target_pose_id=-1, is_return=True,
                           completed_poses=2, total_poses=17, after_pose_id=1,
                           tilt_deg=12., azimuth_deg=0.)
    assert len(session.records) == 2 * session.limits.samples_per_pose
    assert all(r['split'] == 'fit' for r in session.records)
    np.testing.assert_allclose(session.reference['mean_raw_wrench'], reference_before)
    reference_raw = [r for r in store.raw if r['split'] == 'reference']
    assert len(reference_raw) == session.limits.samples_per_pose
    assert reference_raw[0]['raw_wrench'][2] == pytest.approx(reference_before[2] + .2)
    assert all(record['plan_index'] == 2 for record in reference_raw)
    with pytest.raises(CalibrationStopped, match='latched'):
        session.observe()


def test_session_reports_pose_count_motion_sampling_and_repeat_checks():
    session, hardware, store, clock, start = fixture_session()
    messages = []
    session.progress = SimpleNamespace(emit=lambda text, **kwargs: messages.append(text))
    session.run(start, make_plan(start))
    assert hardware.progress is session.progress
    assert any('姿态 1/17' in text for text in messages)
    assert any('姿态 17/17' in text and '独立验证' in text for text in messages)
    assert any('位置误差' in text and '停稳' in text for text in messages)
    assert any('采样 20/25' in text for text in messages)
    assert sum('回中重复性通过' in text for text in messages) == 16
    assert messages[-1] == '全部 17 个姿态采集完成，已回到起始标定姿态。'


def test_blocked_terminal_does_not_delay_operator_stop_or_add_motion():
    import threading
    from tools.sensor_mount_calibration.progress import ProgressReporter

    session, hardware, store, clock, start = fixture_session()
    entered, release = threading.Event(), threading.Event()

    def blocked_sink(line):
        entered.set()
        release.wait(2.)

    progress = ProgressReporter(sink=blocked_sink)
    try:
        progress.emit('blocked terminal')
        assert entered.wait(1.)
        session.progress = progress
        session.keyboard.poll = lambda: 'Q' if hardware.commands else None
        with pytest.raises(CalibrationStopped, match='operator stop'):
            session.run(start, make_plan(start))
        assert hardware.stops == 1 and len(hardware.commands) == 1
        assert session.failed
    finally:
        release.set()
        assert progress.close()


@pytest.mark.parametrize('fault', ['operator', 'sensor', 'moving', 'deadline'])
def test_slow_stability_computation_keeps_safety_checks_and_never_delays_stop(monkeypatch, fault):
    import threading
    from tools.sensor_mount_calibration import session as session_module

    session, hardware, store, clock, start = fixture_session()
    entered, release, finished = threading.Event(), threading.Event(), threading.Event()
    read = hardware.read

    def blocked_check(records):
        entered.set()
        release.wait(2.)
        finished.set()
        return {'status': 'stable'}

    def observed_fault():
        if session.stage == 'stationary_trend':
            assert entered.wait(.5)
            if fault == 'sensor':
                raise TimeoutError('fresh sensor sample timed out during calculation')
        record = read()
        if entered.is_set() and fault == 'moving':
            record['tcp_speed'][0] = .0002
        return record

    monkeypatch.setattr(session_module, 'check_stationary_trend', blocked_check)
    hardware.read = observed_fault
    if fault == 'operator':
        session.keyboard.poll = lambda: 'Q' if entered.is_set() else None
    expected = TimeoutError if fault == 'sensor' else CalibrationStopped
    pattern = {'operator': 'operator stop', 'sensor': 'fresh sensor',
               'moving': 'robot moved during stationary', 'deadline': 'calculation deadline'}[fault]
    try:
        with pytest.raises(expected, match=pattern):
            session.run(start, make_plan(start))
        assert hardware.stops == 1 and hardware.commands == []
        assert session.failed and not finished.is_set(), 'stop must not wait for numerical worker'
        assert len([r for r in store.raw if r['split'] == 'preflight']) == 1
        if fault == 'deadline':
            assert len([r for r in store.raw if r['split'] == 'stability']) > 100
    finally:
        release.set()
        assert finished.wait(.5)


def test_slow_stability_check_continues_fresh_observations_then_accepts_result(monkeypatch):
    import threading
    from tools.sensor_mount_calibration import session as session_module

    session, hardware, store, clock, start = fixture_session()
    entered, release = threading.Event(), threading.Event()
    read = hardware.read
    read_times = []

    def delayed_check(records):
        entered.set()
        assert release.wait(1.)
        return {'status': 'stable'}

    def observed():
        record = read()
        if session.stage == 'stationary_trend':
            read_times.append(record['host_monotonic'])
            if len(read_times) >= 10:
                release.set()
        return record

    monkeypatch.setattr(session_module, 'check_stationary_trend', delayed_check)
    hardware.read = observed
    try:
        records = session.run(start, make_plan(start)[:1])
        assert entered.is_set() and len(read_times) >= 10
        assert np.max(np.diff(read_times)) < session.limits.observation_timeout_sec
        assert len(records) == session.limits.samples_per_pose
        assert hardware.stops == 0 and hardware.commands == []
        assert store.metadata['preflight_stability']['status'] == 'stable'
    finally:
        release.set()


@pytest.mark.parametrize('error', [TimeoutError('sensor timeout'), RuntimeError('robot fault'), KeyboardInterrupt()])
def test_fault_during_tilt_stops_without_return(error):
    session, hardware, store, clock, start = fixture_session()
    original_command = hardware.command
    def fail_after_command(target):
        original_command(target)
        hardware.failure = error
    hardware.command = fail_after_command
    with pytest.raises(type(error)):
        session.run(start, make_plan(start))
    assert hardware.stops == 1
    assert len(hardware.commands) == 1
    assert not np.allclose(hardware.pose, start)
    assert session.failed
    with pytest.raises(CalibrationStopped, match='latched'):
        session.observe()


@pytest.mark.parametrize('key', ['Q', 'ESC'])
def test_operator_stop_immediate_no_return(key):
    session, hardware, store, clock, start = fixture_session()
    session.keyboard.poll = lambda: key if hardware.commands else None
    with pytest.raises(CalibrationStopped, match='operator stop'):
        session.run(start, make_plan(start))
    assert len(hardware.commands) == hardware.stops == 1


def test_stale_rtde_rejected_before_motion():
    session, hardware, store, clock, start = fixture_session()
    read = hardware.read
    def stale():
        record = read()
        record['robot_timestamp'] = 1.
        return record
    hardware.read = stale
    with pytest.raises(CalibrationStopped, match='did not advance'):
        session.run(start, make_plan(start))
    assert hardware.commands == [] and hardware.stops == 1


def test_slow_logging_does_not_authorize_old_data():
    session, hardware, store, clock, start = fixture_session()
    append = store.append_raw
    def slow(record):
        append(record)
        clock.sleep(.09)
    store.append_raw = slow
    with pytest.raises(CalibrationStopped, match='logging'):
        session.run(start, make_plan(start))
    assert hardware.commands == [] and hardware.stops == 1


def test_joint_motion_does_not_count_as_settled():
    session, hardware, store, clock, start = fixture_session(segment_timeout_sec=.1)
    read = hardware.read
    def joint_motion():
        record = read()
        record['actual_qd'] = [.01]*6
        return record
    hardware.read = joint_motion
    with pytest.raises(CalibrationStopped, match='robot moved'):
        session.run(start, make_plan(start))
    assert not hardware.commands


def test_total_deadline_stops_before_next_segment():
    session, hardware, store, clock, start = fixture_session(total_timeout_sec=.2)
    with pytest.raises(CalibrationStopped, match='total execution deadline'):
        session.run(start, make_plan(start))
    assert hardware.stops == 1


def test_settling_requires_measured_reached_pose_not_command_acceptance():
    session, hardware, store, clock, start = fixture_session(segment_timeout_sec=.8)
    def no_motion(target):
        hardware.commands.append(target)
    hardware.command = no_motion
    with pytest.raises(CalibrationStopped, match='standstill deadline'):
        session.run(start, make_plan(start))
    assert len(hardware.commands) == hardware.stops == 1


def test_plan_is_reproducible_and_bounded_including_half_turn_start():
    start = [.4, -.2, .3, 0, np.pi, 0]
    plan = make_plan(start)
    assert plan == make_plan(start)
    path = interpolate_path(start, plan)
    assert len(path) > 500
    for p in path:
        assert p[:3] == start[:3]
        relative = rotation_from_rotvec(start[3:]).T @ rotation_from_rotvec(p[3:])
        assert np.linalg.norm(matrix_to_rotvec(relative)) <= np.deg2rad(25)+1e-7
    assert np.allclose(path[-1], start)


def test_default_cli_never_constructs_hardware_or_writes_config(monkeypatch, capsys, tmp_path):
    from tools.sensor_mount_calibration.calibrate import main
    from pathlib import Path
    config_path = Path(__file__).resolve().parents[1] / 'config.yaml'
    original = config_path.read_bytes()
    assert main(['--data-dir', str(tmp_path/'unused')]) == 0
    assert not (tmp_path/'unused').exists()
    assert config_path.read_bytes() == original
    assert '默认离线预览' in capsys.readouterr().out
