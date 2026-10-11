"""Contact release before menu-6 return, with simulated force and RTDE only."""
from copy import deepcopy
from types import SimpleNamespace
import json
import time

import numpy as np
import pytest

from app.single_point_config import load_settings
from app.single_point_startup import SinglePointRetreatMonitor
from calibration.single_point import load_calibration
from core.models import RobotState, Wrench
from robot.control_mode import ControlMode
from safety.safe_return import PX6DForceMonitor, ReturnAborted
from sensor.force_preprocess import WrenchPreprocessor
from tests.doubles import Devices, Keyboard, Sensor
from tests.test_single_point_ui import temporary_args
from tests.test_single_point import settings, bounded, fake_remote
from simulation.single_point import demo_calibration
import run_single_point_contact as entry


def monitor_rig(config, wrench=Wrench(0., 0., 0., .879, 0., 0.)):
    original = deepcopy(config)
    anchor = np.array([.54, .1, -.01, np.pi, 0., 0.])
    lifted = anchor.copy(); lifted[2] += .03
    state = RobotState(time.monotonic(), anchor.copy(), np.zeros(6))
    owner = SimpleNamespace(standstill_confirmed=True)
    sensor = SimpleNamespace(read_wrench=lambda: wrench)
    processor = WrenchPreprocessor.from_config(config['preprocessing'], tool_orientation=anchor[3:])
    monitor = SinglePointRetreatMonitor(config, sensor, processor, owner, anchor, lifted)
    return monitor, sensor, owner, state, lifted, processor, original


def test_measured_0879nm_contact_can_retreat_without_zero_or_threshold_edits(config):
    monitor, _, _, state, _, processor, original = monitor_rig(config)
    monitor.sample(read_state=lambda: state)
    assert monitor.phase == 'CONTACT_RETREAT'
    assert monitor.config['safe_return']['return_torque_limit'] == config['policy']['absolute_raw_torque_threshold']
    assert monitor.config['safe_return']['return_force_limit'] == config['policy']['absolute_raw_force_threshold']
    np.testing.assert_array_equal(processor.zero_bias_sensor, np.zeros(6))
    assert config == original


@pytest.mark.parametrize('wrench', [Wrench(60., 0., 0., 0., 0., 0.), Wrench(0., 0., 0., 5., 0., 0.)])
def test_original_hard_limits_still_abort_retreat(config, wrench):
    monitor, _, _, state, _, _, _ = monitor_rig(config, wrench)
    with pytest.raises(ReturnAborted, match='safety threshold exceeded'):
        monitor.sample(read_state=lambda: state)


def test_experiment_torque_threshold_is_diagnostic_before_release(config):
    monitor, sensor, _, state, _, processor, _ = monitor_rig(config)
    force_sensor = np.array([6., 0., 0.])
    force_base = processor.rotation_sensor_to_output @ force_sensor
    arm_torque = np.cross(processor.sensor_origin_in_output_m, force_base)
    torque_sensor = processor.rotation_sensor_to_output.T @ (.95*arm_torque/np.linalg.norm(arm_torque))
    sensor.read_wrench = lambda: Wrench.from_sequence(np.r_[force_sensor, torque_sensor])
    assert np.linalg.norm(torque_sensor) < 1.
    monitor.sample(read_state=lambda: state)
    assert np.linalg.norm(monitor.retreat_guard_wrench_base.torque) > 1.
    assert 'diagnostic only' in monitor.snapshot()['diagnostics']['retreat_experiment_threshold']
    np.testing.assert_array_equal(processor.zero_bias_sensor, np.zeros(6))


def test_retreat_still_checks_full_base_wrench_at_existing_hard_limit(config):
    monitor, sensor, _, state, _, processor, _ = monitor_rig(config)
    force_sensor = np.array([6., 0., 0.])
    arm = np.cross(processor.sensor_origin_in_output_m, processor.rotation_sensor_to_output @ force_sensor)
    torque_sensor = processor.rotation_sensor_to_output.T @ (4.99*arm/np.linalg.norm(arm))
    sensor.read_wrench = lambda: Wrench.from_sequence(np.r_[force_sensor, torque_sensor])
    assert np.linalg.norm(torque_sensor) < config['policy']['absolute_raw_torque_threshold']
    with pytest.raises(ReturnAborted, match='processed torque safety threshold exceeded'):
        monitor.sample(read_state=lambda: state)
    assert np.linalg.norm(monitor.retreat_guard_wrench_base.torque) > 5.


def test_latest_real_preflight_frame_no_longer_blocks_vertical_release(config):
    measured = Wrench.from_sequence([-1.1101737022399902, 4.394260406494141, 4.062612533569336,
                                    -.8972081542015076, -.0574939101934433, -.005619025323539972])
    monitor, _, _, state, _, processor, original = monitor_rig(config, measured)
    monitor.sample(read_state=lambda: state)
    assert np.linalg.norm(monitor.retreat_guard_wrench_base.torque) > 1.
    assert monitor.phase == 'CONTACT_RETREAT' and not monitor.last_failure
    assert 'diagnostic only' in monitor.diagnostics['retreat_experiment_threshold']
    np.testing.assert_array_equal(processor.zero_bias_sensor, np.zeros(6))
    assert config == original


@pytest.mark.parametrize('fault', ['xy', 'orientation', 'below', 'above'])
def test_retreat_allows_only_confirmed_bounded_vertical_path(config, fault):
    monitor, _, _, state, lifted, _, _ = monitor_rig(config)
    if fault == 'xy': state.pose[0] += .01
    if fault == 'orientation': state.pose[3] += .1
    if fault == 'below': state.pose[2] -= .01
    if fault == 'above': state.pose[2] = lifted[2]+.01
    with pytest.raises(ReturnAborted, match='vertical'):
        monitor.sample(read_state=lambda: state)


@pytest.mark.parametrize('load', ['torque', 'force'])
def test_low_return_limits_do_not_block_preparation_after_actual_raised_standstill(config, load):
    wrench = Wrench(9. if load == 'force' else 0., 0., 0., .879 if load == 'torque' else 0., 0., 0.)
    monitor, _, owner, state, lifted, _, _ = monitor_rig(config, wrench)
    state.pose[:] = lifted
    owner.standstill_confirmed = False
    monitor.sample(read_state=lambda: state)
    assert monitor.phase == 'CONTACT_RETREAT'
    owner.standstill_confirmed = True
    state.tcp_speed[2] = .005
    monitor.sample(read_state=lambda: state)
    assert monitor.phase == 'CONTACT_RETREAT'
    state.tcp_speed[:] = 0.
    monitor.sample(read_state=lambda: state)
    assert monitor.phase == 'RETURN_PATH' and monitor.retreat_confirmed_sec is not None
    assert monitor.config['safe_return']['return_torque_limit'] == 5.
    assert monitor.config['safe_return']['return_force_limit'] == 60.
    assert not monitor.snapshot()['original_return_limits_enforced']
    assert not monitor.last_failure


def test_sensor_loss_does_not_report_previous_force_as_fresh(config):
    monitor, sensor, _, state, _, _, _ = monitor_rig(config)
    monitor.sample(read_state=lambda: state)
    def fail(): raise TimeoutError('fake sensor lost')
    sensor.read_wrench = fail
    with pytest.raises(ReturnAborted, match='fake sensor lost.*无新鲜读数'):
        monitor.sample(read_state=lambda: state)
    assert monitor.snapshot()['last_wrench_sensor'] is None


def test_shared_return_monitor_still_rejects_0879nm_without_single_point_retreat(config):
    processor = WrenchPreprocessor.from_config(config['preprocessing'])
    sensor = SimpleNamespace(read_wrench=lambda: Wrench(0., 0., 0., .879, 0., 0.))
    monitor = PX6DForceMonitor(config, sensor, processor, raw_only=True)
    with pytest.raises(ReturnAborted, match='return torque limit exceeded'):
        monitor.sample()


@pytest.mark.parametrize('case', ['released', 'still_loaded', 'hard_limit', 'cancel'])
def test_menu_six_low_force_limits_do_not_block_return_but_hard_faults_do(
        tmp_path, monkeypatch, config, case, capsys):
    from app import single_point_ui as ui
    settings = load_settings(entry.ROOT/'single_point_experiment.yaml', config)
    data = load_calibration(entry.ROOT/'single_point_calibration.yaml')
    args = temporary_args(tmp_path, data)
    before = args.calibration.read_bytes()
    initial = np.r_[data['P_ref'], data['reference_tcp_pose'][3:]]
    devices = Devices(config, initial)
    class LoadedSensor(Sensor):
        def read_wrench(self):
            low = (devices.receive.pose[2] < initial[2]+.02 and
                   np.linalg.norm(devices.receive.pose[:2]-initial[:2]) < .001)
            torque = 5.01 if case == 'hard_limit' else 1.01 if low or case == 'still_loaded' else .05
            return Wrench(0., 0., 0., torque, 0., 0.)
    prompts = []
    def confirm(prompt, **kw):
        prompts.append(prompt)
        return not (case == 'cancel' and '完整路径' in prompt)
    monkeypatch.setattr(entry, 'confirm_enter', confirm)
    monkeypatch.setattr(entry, 'URRTDEController', devices.controller)
    monkeypatch.setattr(entry, 'make_reader', lambda cfg: LoadedSensor())
    monkeypatch.setattr(entry, 'OperatorKeyboard', Keyboard)
    monkeypatch.setattr(ui, 'read_control_mode', lambda ip: ControlMode('remote', 'fake true'))
    if case == 'hard_limit':
        with pytest.raises(ReturnAborted, match='safety threshold exceeded'):
            entry.return_to_group(args, config, settings, data, 'B')
        assert devices.control_count == 0
    else:
        entry.return_to_group(args, config, settings, data, 'B')
    moves = [call[1] for call in devices.control.calls if call[0] == 'moveL']
    if case in ('released', 'still_loaded'):
        assert moves and moves[0][:2] == initial[:2].tolist()
        assert moves[0][2] > initial[2]
        np.testing.assert_allclose(moves[-1][:2], data['groups']['B']['tcp_pose'][:2])
        assert '返回结果：complete' in capsys.readouterr().out
    else:
        assert moves == []
    assert len(prompts) == 2 and args.calibration.read_bytes() == before
    if case != 'cancel':
        run, = (tmp_path/'real'/'single_point').glob('run_*')
        diagnostic = json.loads((run/'return_force_status.json').read_text())
        assert diagnostic['air_bias_captured'] is False
        assert not (run/'air_zero.json').exists()
        assert (run/'return_force_preflight.json').exists()


@pytest.mark.parametrize('case', ['released', 'still_loaded', 'hard_limit', 'cancel'])
def test_menu_five_preparation_reuses_contact_release_before_return(
        tmp_path, monkeypatch, config, settings, fake_remote, case):
    """Reproduce the pre-experiment 0.7 Nm rejection without real hardware."""
    original_policy = deepcopy(config['policy'])
    original_return = deepcopy(config['safe_return'])
    bounded(settings)
    settings['return_lift_distance_m'] = .03
    config['preprocessing']['baseline']['sample_count'] = 3
    data = demo_calibration(config)
    calibration_before = deepcopy(data)
    initial = np.r_[data['P_ref'], data['reference_tcp_pose'][3:]]
    devices = Devices(config, initial)
    class LoadedSensor(Sensor):
        def read_wrench(self):
            self.count += 1
            touching_old_point = (devices.receive.pose[2] < initial[2]+.02 and
                                  np.linalg.norm(devices.receive.pose[:2]-initial[:2]) < .001)
            torque = 5.01 if case == 'hard_limit' else 1.01 if touching_old_point or case == 'still_loaded' else .05
            force = 4. if np.linalg.norm(devices.receive.speed[:2]) else 0.
            return Wrench(force, 0., 0., torque, 0., 0.)
    sensor = LoadedSensor()
    prompts = []
    def confirm(prompt, **kw):
        prompts.append(prompt)
        if '授权自动准备' in prompt:
            assert sensor.count == 0 and devices.control_count == 0
            return case != 'cancel'
        if '采集空气零偏' in prompt:
            assert devices.receive.pose[2] > initial[2]
        return True
    monkeypatch.setattr(entry, 'confirm_enter', confirm)
    monkeypatch.setattr(entry, 'read_tcp_offset_readonly', lambda ip: config['tcp']['offset'])
    args = SimpleNamespace(execute=True, output=tmp_path, config=entry.ROOT/'config.yaml')
    result = entry.run_trial(args, config, settings, data, 'A', controller_factory=devices.controller,
                             reader_factory=lambda cfg: sensor, keyboard_factory=Keyboard)
    run, = (tmp_path/'real'/'single_point').glob('run_*')
    moves = [call[1] for call in devices.control.calls if call[0] == 'moveL']
    if case in ('released', 'still_loaded'):
        assert result['status'] == 'contact'
        assert moves[0][:2] == initial[:2].tolist() and moves[0][2] > initial[2]
        np.testing.assert_allclose(moves[-1][:2], data['groups']['A']['tcp_pose'][:2])
        zero = json.loads((run/'air_zero.json').read_text())
        assert zero['actual_tcp_pose'][2] > initial[2]
        np.testing.assert_allclose(zero['bias_sensor'], [0., 0., 0., 1.01 if case == 'still_loaded' else .05, 0., 0.])
        assert len(prompts) == 4  # No additional confirmation or written review.
    else:
        assert result['status'] == 'aborted'
        assert not any(call[0] == 'speedL' and any(call[1]) for call in devices.control.calls)
        assert not (run/'air_zero.json').exists()
        assert not moves and devices.control_count == 0
    if case == 'cancel':
        assert sensor.count == 0 and not (run/'startup_force_preflight.json').exists()
    else:
        preflight = json.loads((run/'startup_force_preflight.json').read_text())
        assert preflight['phase'] == 'CONTACT_RETREAT'
        assert preflight['active_torque_limit_Nm'] == original_policy['absolute_raw_torque_threshold']
        assert preflight['last_wrench_sensor'][3] == pytest.approx(5.01 if case == 'hard_limit' else 1.01)
        assert bool(preflight['failure']) == (case == 'hard_limit')
    if case in ('released', 'still_loaded'):
        status = json.loads((run/'startup_force_status.json').read_text())
        assert status['phase'] == 'RETURN_PATH'
        assert status['air_bias_captured'] is True
        assert status['retreat_confirmed_sec'] is not None
        assert status['active_torque_limit_Nm'] == original_policy['absolute_raw_torque_threshold']
        assert not status['original_return_limits_enforced']
    assert config['policy'] == original_policy and config['safe_return'] == original_return
    assert data == calibration_before
