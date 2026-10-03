import csv
import json
import time
from copy import deepcopy
from threading import Event

import numpy as np
import pytest

from app import continuous_runtime as runtime
from app.configuration import prepare_real
from app.scan_startup import capture_stationary_bias
from core.models import RobotState, Wrench
from doubles import Devices, Sensor
from experiment_logging.continuous_writer import ContinuousLogWriter
from experiment_logging.data_logger import ExperimentLogger
from experiment_logging.paths import PROJECT_ROOT
from policy.continuous_tracking import ContinuousTrackingPolicy, State
from robot.rtde_controller import RobotError
from sensor.force_preprocess import WrenchPreprocessor
from test_continuous_runtime import args, prepare


@pytest.fixture
def real_config(config):
    prepare_real(config, PROJECT_ROOT/'config.yaml')
    return config


def tick(policy, now, *, raw=None, processed=None, age=0.):
    raw = Wrench(0, 0, 0, 0, 0, 0) if raw is None else raw
    processed = raw if processed is None else processed
    pose = np.r_[policy.search_geometry['origin_xy'], 0., 0., 0., 0.]
    return policy.update(now, raw, processed, RobotState(now-age, pose, np.zeros(6)))


@pytest.mark.parametrize('jitter', [.035, .060])
def test_search_timing_and_age_are_diagnostics(real_config, jitter):
    policy = ContinuousTrackingPolicy(real_config)
    tick(policy, 0.)
    assert tick(policy, jitter, age=jitter).move
    assert policy.state == State.TARGET_SEARCH
    assert 'sample_timing' in policy.diagnostics
    diagnostics = {}
    runtime.validate_cycle_timing(policy.c, .1, .1+jitter, .1, .1-jitter, diagnostics=diagnostics)
    assert {'cycle_timing', 'sample_gap', 'observation_age'} <= diagnostics.keys()


def test_high_rate_below_contact_does_not_stop(real_config):
    policy = ContinuousTrackingPolicy(real_config)
    tick(policy, 0.)
    command = tick(policy, .001, raw=Wrench(.9, 0, 0, 0, 0, 0))
    assert command.move and policy.state == State.TARGET_SEARCH
    assert policy.force_rate == pytest.approx(900.)
    assert 'force_rate' in policy.diagnostics


@pytest.mark.parametrize('processed,expected', [
    (Wrench(.2, 0, 15, 0, 0, 0), State.TARGET_SEARCH),
    (Wrench(.2, 0, 0, 2, 0, 0), State.TARGET_SEARCH),
    (Wrench(15, 0, 0, 0, 0, 0), State.FIRST_CONTACT),
    (Wrench(1, 0, 0, 0, 0, 0), State.FIRST_CONTACT),
])
def test_processed_limits_do_not_replace_contact_logic(real_config, processed, expected):
    policy = ContinuousTrackingPolicy(real_config)
    tick(policy, 0.)
    command = tick(policy, .01, raw=Wrench(15, 0, 0, 0, 0, 0), processed=processed)
    assert policy.state == expected and policy.stop_reason is None
    assert command.move is (expected == State.TARGET_SEARCH)


def test_search_ignores_time_budgets(real_config):
    real_config['continuous_tracking']['max_runtime_sec'] = 120.  # Legacy finite config also only warns.
    policy = ContinuousTrackingPolicy(real_config)
    tick(policy, 0.)
    # Keep a nominal prediction interval; neither timer may stop initial search.
    policy._started = -200
    assert tick(policy, .01).move
    assert policy.state == State.TARGET_SEARCH
    assert {'search_runtime', 'search_time'} <= policy.diagnostics.keys()


def test_past_jitter_does_not_inflate_next_step_at_search_edge(real_config):
    policy = ContinuousTrackingPolicy(real_config)
    tick(policy, 0.)
    geometry = real_config['continuous_search_geometry']
    position = (np.asarray(geometry['origin_xy']) + policy.search_direction*(geometry['usable_distance_m']-.0005))
    pose = np.r_[position, 0., 0., 0., 0.]
    zero = Wrench(0, 0, 0, 0, 0, 0)
    command = policy.update(.06, zero, zero, RobotState(.06, pose, np.zeros(6)))
    assert command.move and policy.state == State.TARGET_SEARCH


def test_force_rate_warning_does_not_disable_direction_reversal_stop(real_config):
    policy = ContinuousTrackingPolicy(real_config)
    for i in range(20):
        tick(policy, i*.01, raw=Wrench(1.5, 0, 0, 0, 0, 0))
    assert policy.state == State.CONTINUOUS_TRACKING
    tick(policy, .2, raw=Wrench(2., 0, 0, 0, 0, 0))
    assert policy.state == State.CONTINUOUS_TRACKING and policy.stop_reason is None
    assert 'force_rate' in policy.diagnostics
    command = tick(policy, .21, raw=Wrench(-1.5, 0, 0, 0, 0, 0))
    assert policy.state == State.STOP and not command.move
    assert policy.stop_reason.value == 'STOP_DIRECTION_REVERSAL'
    assert not tick(policy, .22, raw=Wrench(1.5, 0, 0, 0, 0, 0)).move


def test_tracking_admittance_numerics_are_unchanged(real_config):
    original = deepcopy(real_config); original.pop('continuous_real_execution')
    policies = [ContinuousTrackingPolicy(c) for c in (real_config, original)]
    for i in range(35):
        commands = [tick(p, i*.01, raw=Wrench(1.5 if i < 20 else 1.6, 0, 0, 0, 0, 0)) for p in policies]
        assert commands[0].state == commands[1].state
        np.testing.assert_allclose(commands[0].direction_xy*commands[0].speed,
                                   commands[1].direction_xy*commands[1].speed, atol=1e-12)


@pytest.mark.parametrize('raw', [Wrench(60, 0, 0, 0, 0, 0), Wrench(65, 0, 0, 0, 0, 0),
                               Wrench(0, 0, 0, 0, 5, 0), Wrench(np.nan, 0, 0, 0, 0, 0)])
def test_raw_extremes_and_nonfinite_still_stop(real_config, raw):
    policy = ContinuousTrackingPolicy(real_config)
    tick(policy, 0.)
    assert not tick(policy, .01, raw=raw).move
    assert policy.state == State.STOP


@pytest.mark.parametrize('away', [False, True])
def test_default_watchdog_calls_are_absent(config, monkeypatch, tmp_path, away):
    target = prepare(config, monkeypatch)
    pose = np.asarray(target); pose[0] += .003 if away else 0
    devices = Devices(config, pose)
    def forbidden(*a, **k):
        raise AssertionError('default continuous run called custom watchdog')
    devices.control.setWatchdog = devices.control.kickWatchdog = forbidden
    def factory(c):
        assert c['continuous_require_watchdog'] is False
        owner = devices.controller(c)
        owner._check_watchdog_health = owner.enable_watchdog = owner.kick_watchdog = forbidden
        return owner
    assert runtime.run(args(tmp_path), controller_factory=factory, reader_factory=Sensor) == 0
    assert any(call[0] == 'speedL' and np.any(call[1]) for call in devices.control.calls)


@pytest.mark.parametrize('jitter', [.035, .060])
def test_real_loop_continues_after_single_slow_sample(config, monkeypatch, tmp_path, jitter):
    target = prepare(config, monkeypatch)
    devices = Devices(config, target)
    class JitterSensor(Sensor):
        def read_wrench(self):
            if self.count == 5:
                time.sleep(jitter)
            return super().read_wrench()
    settings = args(tmp_path); settings.duration = .25
    assert runtime.run(settings, controller_factory=devices.controller, reader_factory=JitterSensor) == 0
    run_dir, = (tmp_path/'real/continuous').glob('run_*')
    rows = list(csv.DictReader((run_dir/'samples.csv').open()))
    slow = next(i for i, row in enumerate(rows)
                if float(row['serial_read_end'])-float(row['serial_read_start']) >= jitter)
    assert rows[slow+1]['current_state'] == 'TARGET_SEARCH'
    assert 'cycle_timing' in json.loads(rows[slow]['software_warnings'])['runtime']
    assert json.loads((run_dir/'termination.json').read_text())['reason'] == 'STOP_USER_REQUEST'


def test_bias_15n_does_not_process_before_zero(real_config, monkeypatch):
    robot_config, start = prepare_real(real_config, PROJECT_ROOT/'config.yaml')
    devices = Devices(real_config, start)
    owner = devices.controller(robot_config); owner.connect()
    sensor = Sensor(); sensor.read_wrench = lambda: Wrench(15, 0, 0, 0, 0, 0)
    processor = WrenchPreprocessor.from_config(real_config['preprocessing'])
    process = processor.process
    def forbidden(raw):
        raise AssertionError('processed force used before zero bias')
    monkeypatch.setattr(processor, 'process', forbidden)
    capture_stationary_bias(real_config, sensor, processor, owner, 3)
    np.testing.assert_allclose(process(sensor.read_wrench()).array(), np.zeros(6), atol=1e-12)
    owner.close()


def test_bias_timing_jitter_is_warning(real_config):
    robot_config, start = prepare_real(real_config, PROJECT_ROOT/'config.yaml')
    devices = Devices(real_config, start)
    owner = devices.controller(robot_config); owner.connect()
    class SlowBias(Sensor):
        def read_wrench(self):
            time.sleep(.06)
            return Wrench(15, 0, 0, 0, 0, 0)
    processor = WrenchPreprocessor.from_config(real_config['preprocessing'])
    capture_stationary_bias(real_config, SlowBias(), processor, owner, 2)
    assert 'bias_timing' in owner.diagnostics
    owner.close()


def test_phase_speed_warns_global_speed_and_polygon_stop(real_config):
    robot_config, start = prepare_real(real_config, PROJECT_ROOT/'config.yaml')
    devices = Devices(real_config, start)
    owner = devices.controller(robot_config); owner.connect(); owner.set_continuous_phase('TARGET_SEARCH')
    devices.receive.speed[0] = .028  # Over the old 1.5x search trip, below global 30 mm/s.
    owner.read_state()
    assert 'phase_speed' in owner.diagnostics and not owner.motion_fault
    devices.receive.speed[0] = .031
    with pytest.raises(RobotError, match='speed exceeds'):
        owner.read_state()
    devices.receive.speed[:] = 0
    geometry = real_config['continuous_search_geometry']
    devices.receive.pose[:2] = start[:2] + np.asarray(real_config['policy']['search_direction_xy'])*(geometry['geometric_distance_m']+.001)
    with pytest.raises(RobotError, match='boundary'):
        owner.read_state()
    owner.close()


@pytest.mark.parametrize('high', [.00065, .0008])
def test_real_speed_warning_clears_only_on_fresh_recovery(real_config, high):
    robot_config, start = prepare_real(real_config, PROJECT_ROOT/'config.yaml')
    owner = Devices(real_config, start).controller(robot_config)
    owner.set_continuous_phase('LOCAL_REACQUIRE')
    def observe(stamp, now, measured):
        owner._packet_stamp = stamp
        owner._check_continuous_speed(np.asarray(start), np.array([measured, 0, 0, 0, 0, 0]), now)
    observe(1., 10., high)
    observe(1.03, 10.03, high)
    assert 'phase_speed' in owner.diagnostics
    assert owner.speed_guard_diagnostics['speed_guard_state'] == 'WARNING'
    # A different phase or cached zero-speed read cannot clear the episode.
    owner.set_continuous_phase('STOP')
    observe(1.03, 10.04, 0.)
    assert 'phase_speed' in owner.diagnostics
    observe(1.05, 10.05, 0.)
    assert 'phase_speed' not in owner.diagnostics
    assert owner.speed_guard_diagnostics['speed_guard_state'] == 'OK'
    assert owner.speed_guard_diagnostics['speed_guard_count'] == 0
    assert owner.speed_guard_diagnostics['speed_guard_elapsed_sec'] == 0.
    observe(1.06, 10.06, .00065)
    assert owner.speed_guard_diagnostics['speed_guard_state'] == 'PENDING'
    assert owner.speed_guard_diagnostics['speed_guard_count'] == 1
    assert 'phase_speed' not in owner.diagnostics and not owner.motion_fault


def test_nonreal_speed_episode_cannot_clear_after_fault_deadline(real_config):
    robot_config, start = prepare_real(real_config, PROJECT_ROOT/'config.yaml')
    owner = Devices(real_config, start).controller(robot_config)
    owner.config['continuous_real_execution'] = False
    owner.set_continuous_phase('LOCAL_REACQUIRE')
    owner._packet_stamp = 1.
    owner._check_continuous_speed(np.asarray(start), np.array([.00065, 0, 0, 0, 0, 0]), 10.)
    owner._packet_stamp = 1.03
    with pytest.raises(RobotError, match='speed exceeds'):
        owner._check_continuous_speed(np.asarray(start), np.zeros(6), 10.03)
    assert owner.motion_fault


def test_rtde_stagnation_warns_but_disconnect_and_ur_stops_raise(real_config):
    robot_config, start = prepare_real(real_config, PROJECT_ROOT/'config.yaml')
    devices = Devices(real_config, start)
    owner = devices.controller(robot_config); owner.connect(); owner.set_continuous_phase('TARGET_SEARCH')
    owner.read_state()
    devices.receive.frozen_stamp = owner._packet_stamp
    owner._packet_seen_at -= .06
    owner.read_state()
    assert 'rtde_stagnation' in owner.diagnostics
    for flag in ('emergency', 'protective'):
        setattr(devices.receive, flag, True)
        with pytest.raises(RobotError, match='emergency/protective'):
            owner.read_state()
        setattr(devices.receive, flag, False)
    devices.receive.connected = False
    with pytest.raises(RobotError, match='disconnected'):
        owner.read_state()
    owner.close()


@pytest.mark.parametrize('source', ['px6d', 'rtde', 'raw65'])
def test_real_device_exceptions_and_raw65_end_motion(config, monkeypatch, tmp_path, source):
    target = prepare(config, monkeypatch)
    devices = Devices(config, target)
    class FaultSensor(Sensor):
        def read_wrench(self):
            if self.count >= 5:
                if source == 'px6d':
                    raise OSError('PX6D communication failed')
                if source == 'raw65':
                    return Wrench(65, 0, 0, 0, 0, 0)
                def fail():
                    raise OSError('RTDE communication failed')
                devices.receive.getActualTCPPose = fail
            return super().read_wrench()
    assert runtime.run(args(tmp_path), controller_factory=devices.controller, reader_factory=FaultSensor) == 1
    assert not devices.control.connected and not devices.receive.connected
    if source == 'raw65':
        run_dir, = (tmp_path/'real/continuous').glob('run_*')
        termination = json.loads((run_dir/'termination.json').read_text())
        assert termination['raw_wrench'][0] == 65


def test_logger_backlog_is_bounded_and_warning_only(config, tmp_path):
    entered, release = Event(), Event()
    logger = ExperimentLogger(tmp_path, config, mode='real', strategy='continuous', workspace_logging=False)
    def blocked(*a, **k):
        entered.set()
        release.wait(2.)
    logger.log_waypoint = blocked
    writer = ContinuousLogWriter(logger, capacity=1, max_pending_sec=.01, diagnostic_only=True)
    try:
        writer.log_waypoint(None)
        assert entered.wait(1.)
        time.sleep(.02)
        writer.check_health()
        writer.log_waypoint(None)
        assert writer.diagnostics['dropped_records'] == 1
        assert 'backlog' in writer.diagnostics
        assert len(writer._pending) == 1
    finally:
        release.set()
        writer.close()
