"""Air zero, independent granular background and insertion guards; no hardware."""
import csv
import json
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from app import continuous_runtime as runtime, scan_startup as startup
from calibration.scan_calibration import load_scan_calibration
from core.models import RobotState, Wrench
from doubles import Devices, Sensor, Keyboard
from experiment_logging.paths import PROJECT_ROOT
from policy.continuous_tracking import ContinuousTrackingPolicy, State
from safety.safe_return import PX6DForceMonitor, ReturnAborted
from sensor.force_preprocess import WrenchPreprocessor
from test_continuous_runtime import args, prepare
from test_real_tracking_guards import clock


def test_granular_capture_uses_real_rotation_and_keeps_air_zero_independent(config, clock, monkeypatch):
    # Known synthetic 90-degree TCP rotation plus the configured 45-degree mount.
    pose = np.array([0., 0., 0., 0., 0., np.pi/2])
    processor = WrenchPreprocessor.from_config(config['preprocessing'], tool_orientation=pose[3:])
    air = Wrench(.2, -.3, 9.4, .02, .01, -.03)
    processor.set_zero_bias([air, air])
    zero = processor.zero_bias_sensor.copy()
    processor.set_granular_baseline(Wrench(10., 10., 10., 0., 0., 0.))
    processor.process(air)  # Prior calibration/filter values cannot contaminate capture.
    medium_sensor = np.array([1.2, -.6, .4, .04, .02, .01])
    raw = Wrench.from_sequence(air.array()+medium_sensor)
    root_half = np.sqrt(.5)
    rotation = np.array([[-root_half, -root_half, 0.], [root_half, -root_half, 0.], [0., 0., 1.]])
    force_base = rotation @ medium_sensor[:3]
    torque_base = rotation @ medium_sensor[3:] + np.cross([0., 0., .024], force_base)
    expected = np.r_[force_base, torque_base]
    # The calibration must neither use the old medium subtraction nor the filter.
    np.testing.assert_allclose(processor.air_compensated_wrench_base(raw).array(), expected, atol=1e-8)
    old_state = processor.force_kalman.state.copy()
    def no_air_rezero(*args):
        raise AssertionError('granular baseline cannot call set_zero_bias')
    monkeypatch.setattr(processor, 'set_zero_bias', no_air_rezero)
    owner = SimpleNamespace(config={}, diagnostics={}, watchdog_active=False,
        read_diagnostic_state=lambda: RobotState(clock.now, pose.copy(), np.zeros(6)))
    samples = iter([raw, raw, raw])
    startup.capture_stationary_granular_baseline(config,
        SimpleNamespace(read_wrench=lambda: next(samples)), processor, owner, 3)
    np.testing.assert_array_equal(processor.zero_bias_sensor, zero)
    np.testing.assert_allclose(processor.granular_baseline_output, expected, atol=1e-8)
    assert processor.force_kalman.state is None  # Reset once after changing calibration.
    assert np.linalg.norm(old_state) > 1.
    background_only = processor.process(raw)
    np.testing.assert_allclose(background_only.array(), 0., atol=1e-8)
    policy = ContinuousTrackingPolicy(config)
    command = policy.update(0., raw, background_only, RobotState(0., pose, np.zeros(6)))
    assert command.state == 'TARGET_SEARCH' and policy.initial_contact is None
    assert np.linalg.norm(force_base[:2]) > config['policy']['contact_threshold']


@pytest.mark.parametrize('load,reason', [([8.5, 0., 0., 0., 0., 0.], 'return force'),
                                       ([0., 0., 0., 0., .8, 0.], 'return torque')])
def test_insertion_switch_enforces_compensated_limits_without_baseline_or_filter_masking(config, load, reason):
    config['continuous_real_execution'] = True
    processor = WrenchPreprocessor(0., np.eye(3), np.zeros(3), np.zeros(6), np.zeros(6))
    air = Wrench(0., 0., 9.4, 0., .8, 0.)
    reading = air
    monitor = PX6DForceMonitor(config, SimpleNamespace(read_wrench=lambda: reading), processor, raw_only=True)
    state = lambda: RobotState(0., np.zeros(6), np.zeros(6))
    monitor.sample(read_state=state)  # Before zero: raw-only, below existing hard limits.
    assert monitor.raw_only and monitor.force_frame == 'Sensor'
    assert {'return force', 'return torque'} <= monitor.diagnostics.keys()
    np.testing.assert_array_equal(processor.zero_bias_sensor, 0.)
    processor.set_zero_bias([air])
    # Deliberately install a background equal to the later overload. It must
    # not conceal total insertion load, or seed/alter the scan filter.
    processor.set_granular_baseline(Wrench.from_sequence(load))
    processor.process(air)
    saved_filter = processor.force_kalman.state.copy()
    monitor.use_air_compensated_wrench(processor)
    monitor.sample(read_state=state)  # Prime only the separate return filter at zero.
    np.testing.assert_allclose(monitor.processed.array(), 0.)
    assert not monitor.raw_only and monitor.force_frame == 'Base'
    assert monitor.enforce_processed_limits
    assert not monitor.diagnostics
    reading = Wrench.from_sequence(air.array()+load)
    with pytest.raises(ReturnAborted, match=reason+' limit exceeded: air-compensated Base'):
        monitor.sample(read_state=state)
    np.testing.assert_allclose(monitor.guard_wrench_base.array(), load)
    if reason == 'return force':
        assert np.linalg.norm(monitor.processed.force) < config['safe_return']['return_force_limit']
    np.testing.assert_array_equal(processor.zero_bias_sensor, air.array())
    np.testing.assert_array_equal(processor.force_kalman.state, saved_filter)


@pytest.mark.parametrize('air,raw,reason', [
    ([0., 0., 54., 0., 0., 0.], [0., 0., 60., 0., 0., 0.], 'absolute raw force'),
    ([0., 0., 0., 0., 4.5, 0.], [0., 0., 0., 0., 5., 0.], 'absolute raw torque'),
])
def test_raw_hard_backstop_remains_active_after_compensated_switch(config, air, raw, reason):
    # Synthetic bias makes the compensated load smaller than return limits;
    # raw hardware-range protection must still reject the hard threshold.
    config['continuous_real_execution'] = True
    processor = WrenchPreprocessor(0., np.eye(3), np.zeros(3), np.zeros(6), np.zeros(6))
    processor.set_zero_bias([Wrench.from_sequence(air)])
    monitor = PX6DForceMonitor(config, SimpleNamespace(read_wrench=lambda: Wrench.from_sequence(raw)),
                              processor, raw_only=True)
    monitor.use_air_compensated_wrench(processor)
    with pytest.raises(ReturnAborted, match=reason):
        monitor.sample(read_state=lambda: RobotState(0., np.zeros(6), np.zeros(6)))


def granular_runtime(config, monkeypatch, clock, *, background=1.4, target_force=0.):
    target = prepare(config, monkeypatch)
    original_load = runtime.load_config
    search = np.asarray(load_scan_calibration(PROJECT_ROOT/'scan_calibration.yaml')['scan_direction_xy'])
    search /= np.linalg.norm(search)
    medium = np.r_[-background*search, .4, 0., 0., 0.]
    def load(path):
        loaded = original_load(path)
        loaded['preprocessing']['granular_baseline'].update(capture_on_start=True, sample_count=3)
        return loaded
    monkeypatch.setattr(runtime, 'load_config', load)
    # Virtual time permits deterministic settling while all device factories
    # remain injected doubles beneath the unchanged production RTDE owner.
    contexts = 0
    def enter(self):
        nonlocal contexts
        contexts += 1
        self.main_loop, self.entered = contexts > 1, clock.now
        return self
    def poll(self):
        return 'Q' if self.main_loop and clock.now-self.entered >= .4 else None
    monkeypatch.setattr(Keyboard, '__enter__', enter)
    monkeypatch.setattr(Keyboard, 'poll', poll)
    monkeypatch.setattr(Keyboard, 'read_text', lambda self, prompt: '3')
    devices = Devices(config, target)
    air = np.array([0., 0., 9.4, 0., 0., 0.])
    class MediumSensor(Sensor):
        scan_reads = 0
        def read_wrench(self):
            self.count += 1
            inserted = devices.receive.pose[2] <= target[2]+1e-8
            vector = air + (medium if inserted else np.zeros(6))
            scanning = devices.owner.control is not None and not devices.owner._return_mode
            if scanning:
                self.scan_reads += 1
                if self.scan_reads > 5:
                    vector = vector + np.r_[-target_force*search, 0., 0., 0., 0.]
            return Wrench.from_sequence(vector)
    return target, devices, MediumSensor(), air, medium


@pytest.mark.parametrize('target_force', [0., 1.5])
def test_runtime_captures_independent_background_and_delivers_only_target_force_to_policy(
        config, monkeypatch, tmp_path, clock, target_force):
    target, devices, sensor, air, medium = granular_runtime(config, monkeypatch, clock, target_force=target_force)
    zeros, backgrounds, updates = [], [], []
    original_zero = WrenchPreprocessor.set_zero_bias
    original_background = WrenchPreprocessor.set_granular_baseline
    original_update = ContinuousTrackingPolicy.update
    def zero(processor, samples):
        zeros.append((processor, devices.receive.pose.copy()))
        return original_zero(processor, samples)
    def background(processor, value):
        backgrounds.append((processor, devices.receive.pose.copy(), value.array()))
        return original_background(processor, value)
    def update(policy, now, raw, processed, robot, **kwargs):
        command = original_update(policy, now, raw, processed, robot, **kwargs)
        updates.append((policy.state, processed.array()))
        return command
    monkeypatch.setattr(WrenchPreprocessor, 'set_zero_bias', zero)
    monkeypatch.setattr(WrenchPreprocessor, 'set_granular_baseline', background)
    monkeypatch.setattr(ContinuousTrackingPolicy, 'update', update)
    source_before = (PROJECT_ROOT/'config.yaml').read_bytes()
    assert runtime.run(args(tmp_path), controller_factory=devices.controller, reader_factory=lambda *a: sensor) == 0
    assert len(zeros) == 1
    processor, zero_pose = zeros[0]
    assert zero_pose[2] == pytest.approx(target[2]+config['safe_return']['return_lift_distance'])
    main_backgrounds = [b for b in backgrounds if b[0] is processor]
    assert len(main_backgrounds) == 1
    np.testing.assert_allclose(main_backgrounds[0][1], target)
    np.testing.assert_allclose(processor.zero_bias_sensor, air)
    np.testing.assert_allclose(processor.granular_baseline_output, medium, atol=1e-12)
    assert np.linalg.norm(medium[:2]) > config['policy']['contact_threshold']
    np.testing.assert_allclose(updates[0][1], 0., atol=1e-12)
    assert updates[0][0] == State.TARGET_SEARCH
    states = {state for state, _ in updates}
    if target_force:
        assert {State.FIRST_CONTACT, State.CONTINUOUS_TRACKING} <= states
    else:
        assert not {State.FIRST_CONTACT, State.CONTINUOUS_TRACKING} & states
        assert all(np.linalg.norm(w[:3]) < 1e-9 for _, w in updates)
    path, = (tmp_path/'real/continuous').glob('run_*')
    saved = json.loads((path/'granular_baseline_at_start.json').read_text())
    np.testing.assert_allclose(saved['zero_bias_sensor'], air)
    np.testing.assert_allclose(saved['granular_baseline_output'], medium, atol=1e-12)
    assert saved['source'] == 'target_free_stationary_P0_after_insertion' and saved['frame'] == 'Base'
    snapshot = yaml.safe_load((path/'config_snapshot.yaml').read_text())
    np.testing.assert_allclose(snapshot['preprocessing']['granular_baseline_output'], medium, atol=1e-12)
    assert snapshot['continuous_speed_selection']['tracking_speed_multiplier'] == 3.
    status = json.loads((path/'return_status.json').read_text())
    assert status['processed_return_limits_enforced'] and status['return_bias_captured']
    assert status['return_force_frame'] == 'Base'
    with (path/'samples.csv').open() as handle:
        rows = list(csv.DictReader(handle))
    # The callback's pre-zero standstill samples share the descent phase label;
    # actual downward commands must all use the post-zero Base protection.
    descent = [row for row in rows if row['reason'].startswith('DESCEND_TO_START')
               and float(row['commanded_speed_mps']) > 0.]
    assert descent and all(row['processed_force_frame'] == 'Base' for row in descent)
    assert (PROJECT_ROOT/'config.yaml').read_bytes() == source_before
    assert not sensor.connected and not devices.receive.connected and not devices.control.connected


def test_insertion_overload_aborts_before_background_capture_or_scan(config, monkeypatch, tmp_path, clock):
    _, devices, sensor, _, medium = granular_runtime(config, monkeypatch, clock, background=9.)
    # Capture is forbidden on an excessive load even if its background could
    # later be subtracted to zero. This goes through startup's real guard switch.
    def forbidden(*a, **kw):
        raise AssertionError('unsafe insertion must stop before granular capture')
    monkeypatch.setattr(runtime, 'capture_stationary_granular_baseline', forbidden)
    assert runtime.run(args(tmp_path), controller_factory=devices.controller, reader_factory=lambda *a: sensor) == 1
    assert not any(call[0] == 'speedL' and np.linalg.norm(call[1]) > 0. for call in devices.control.calls)
    path, = (tmp_path/'real/continuous').glob('run_*')
    assert not (path/'granular_baseline_at_start.json').exists()
    status = json.loads((path/'return_status.json').read_text())
    assert status['return_status'] == 'aborted' and status['processed_return_limits_enforced']
    assert 'return force limit exceeded: air-compensated Base' in status['return_abort_reason']
    np.testing.assert_allclose(status['return_guard_wrench_base'], medium, atol=1e-12)
    assert not sensor.connected and not devices.control.connected


def test_declining_target_free_background_confirmation_prevents_scanning(config, monkeypatch, tmp_path, clock):
    _, devices, sensor, _, _ = granular_runtime(config, monkeypatch, clock)
    monkeypatch.setattr(Keyboard, 'read_line', lambda self, prompt: 'q' if '只有颗粒背景' in prompt else '')
    assert runtime.run(args(tmp_path), controller_factory=devices.controller, reader_factory=lambda *a: sensor) == 0
    assert not any(call[0] == 'speedL' and np.linalg.norm(call[1]) > 0. for call in devices.control.calls)
    path, = (tmp_path/'real/continuous').glob('run_*')
    assert not (path/'granular_baseline_at_start.json').exists()
    assert not sensor.connected and not devices.control.connected


@pytest.mark.parametrize('granular', [dict(capture_on_start='false'),
                                     dict(capture_on_start=True, sample_count=0),
                                     dict(capture_on_start=True, sample_count=True)])
def test_invalid_background_settings_fail_before_device_connection(config, monkeypatch, tmp_path, granular):
    target = prepare(config, monkeypatch)
    original_load = runtime.load_config
    def load(path):
        loaded = original_load(path)
        loaded['preprocessing']['granular_baseline'] = granular
        return loaded
    monkeypatch.setattr(runtime, 'load_config', load)
    devices, sensor = Devices(config, target), Sensor()
    assert runtime.run(args(tmp_path), controller_factory=devices.controller, reader_factory=lambda *a: sensor) == 1
    assert devices.receive_count == devices.control_count == sensor.count == 0
