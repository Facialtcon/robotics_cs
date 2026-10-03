"""Exercise calibration safety with the production RTDE owner and fake devices."""

from copy import deepcopy
from dataclasses import replace
import time
from types import SimpleNamespace

import numpy as np
import pytest

from core.models import Wrench
from doubles import Devices, Sensor
from robot.rtde_controller import RobotError, _orientation_distance, _rotvec_to_matrix
from sensor.px6d_reader import PX6DTimeout
from tools.sensor_mount_calibration import hardware as hardware_module
from tools.sensor_mount_calibration.hardware import Hardware, HardwareError, interpolate_poses
from tools.sensor_mount_calibration.plan import Limits, make_plan, matrix_to_rotvec


def rig(config, limits=None):
    devices = Devices(config, config['dry_run']['start_pose'])
    receive, control = devices.receive, devices.control
    receive.q = np.array([-.5, -1., 1., -1., 1.2, .3])
    receive.qd = np.zeros(6)
    receive.status = 3
    receive.runtime = 2
    receive.getActualQ = lambda: receive.q.copy()
    receive.getActualQd = lambda: receive.qd.copy()
    receive.getRobotStatus = lambda: receive.status
    receive.getRuntimeState = lambda: receive.runtime
    control.isPoseWithinSafetyLimits = lambda p: True
    control.isJointsWithinSafetyLimits = lambda q: True
    control.getInverseKinematicsHasSolution = lambda p, q: True
    control.getInverseKinematics = lambda p, q: list(q)
    sensor = Sensor()
    sensor.raw = np.array([1., -2., 4., .1, .2, .3])
    sensor.error = None

    def read_wrench():
        if sensor.error:
            raise sensor.error
        return Wrench.from_sequence(sensor.raw)

    sensor.read_wrench = read_wrench

    def owner(runtime):
        runtime['settle_hold_sec'] = .001
        return devices.controller(runtime)

    hardware = Hardware(config, replace(limits or Limits(), sample_period_sec=.001),
                        controller_factory=owner, sensor_factory=lambda **kw: sensor)
    return hardware, devices, sensor


def ready(config, limits=None):
    hardware, devices, sensor = rig(config, limits)
    hardware.connect()
    hardware.activate()
    return hardware, devices, sensor


def target_for(hardware):
    target = hardware.origin_pose.copy()
    target[4] += .035
    return target


def test_construct_and_connect_are_receive_only_without_configuration_mutation(config):
    original = deepcopy(config)
    hardware, devices, sensor = rig(config)
    assert devices.receive_count == devices.control_count == 0
    hardware.connect()
    assert devices.receive_count == 1 and devices.control_count == 0
    sample = hardware.read()
    np.testing.assert_allclose(sample['raw_wrench'], sensor.raw)
    np.testing.assert_allclose(sample['actual_tcp_pose'], devices.receive.pose)
    assert sample['robot_timestamp'] > 0 and sample['utc_time'].endswith('+00:00')
    assert config == original
    hardware.close()
    assert not sensor.connected and not devices.receive.connected


def test_existing_lower_speed_acceleration_and_deceleration_limits_are_retained(config):
    config['robot'].update(max_tcp_speed=.003, speed_acceleration=.02, stop_deceleration=.1)
    hardware, devices, sensor = rig(config)
    assert hardware.command_speed == .003
    assert hardware.acceleration == .02
    assert hardware.controller.config['stop_deceleration'] == .1
    assert devices.receive_count == devices.control_count == 0
    hardware.close()


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), 0., -1.])
def test_invalid_configured_bounds_rejected_without_connecting(config, bad):
    config['robot']['max_tcp_speed'] = bad
    with pytest.raises(ValueError, match='finite positive'):
        rig(config)


def test_preflight_checks_entire_path_then_permits_exact_command(config):
    hardware, devices, sensor = ready(config)
    target = target_for(hardware)
    calls = []
    messages = []
    hardware.progress = SimpleNamespace(emit=lambda text, **kwargs: messages.append(text))
    devices.control.getInverseKinematics = lambda p, q: calls.append((p, q)) or list(q)
    report = hardware.preflight([target, hardware.origin_pose], hardware.read)
    assert report['sample_count'] >= 4
    assert len(calls) == report['sample_count']
    assert hardware.metadata['preflight_progress']['completed_points'] == report['sample_count']
    assert hardware.metadata['preflight_progress']['completed_segments'] == 2
    assert '路径预检开始' in messages[0] and '机器人保持静止' in messages[0]
    assert any('路段 2/2' in text for text in messages)
    assert '路径预检通过' in messages[-1]
    hardware.command(target)
    hardware.command(hardware.origin_pose)
    motion = [call for call in devices.control.calls if call[0] == 'moveL']
    assert len(motion) == 2
    np.testing.assert_allclose(motion[-1][1], hardware.origin_pose)
    assert not any(call[0] == 'speedL' for call in devices.control.calls)
    hardware.stop()
    with pytest.raises(HardwareError, match='forbidden'):
        hardware.command(target)
    hardware.close()
    hardware.close()
    assert sum(call[0] == 'stopL' for call in devices.control.calls) == 1


def test_sensor_timeout_immediately_stops_and_never_uses_previous_sample(config):
    hardware, devices, sensor = ready(config)
    target = target_for(hardware)
    hardware.preflight([target], hardware.read)
    sensor.error = PX6DTimeout('new sensor sample timed out')
    with pytest.raises(PX6DTimeout):
        hardware.command(target)
    assert devices.control.calls[-1] == ('stopL', True)
    assert not any(call[0] == 'moveL' for call in devices.control.calls)
    sensor.error = None
    with pytest.raises(HardwareError, match='forbidden'):
        hardware.command(target)
    hardware.close()


@pytest.mark.parametrize('field,value', [('force', 60.), ('torque', 5.), ('nonfinite', np.nan)])
def test_raw_force_torque_and_nonfinite_data_stop(config, field, value):
    hardware, devices, sensor = ready(config)
    sensor.raw[3 if field == 'torque' else 0] = value
    with pytest.raises(RobotError):
        hardware.read()
    assert devices.control.calls[-1] == ('stopL', True)
    hardware.close()


@pytest.mark.parametrize('fault', ['protective', 'emergency', 'paused', 'freedrive',
                                   'power', 'mode', 'safety', 'disconnected'])
def test_robot_abnormal_status_stops(config, fault):
    hardware, devices, sensor = ready(config)
    receive = devices.receive
    if fault in ('protective', 'emergency'):
        setattr(receive, fault, True)
    elif fault == 'paused':
        receive.runtime = 4
    elif fault == 'freedrive':
        receive.status = 7
    elif fault == 'power':
        receive.status = 2
    elif fault == 'mode':
        receive.getRobotMode = lambda: 6
    elif fault == 'safety':
        receive.getSafetyMode = lambda: 4
    elif fault == 'disconnected':
        receive.connected = False
    with pytest.raises(HardwareError):
        hardware.read()
    assert devices.control.calls[-1] == ('stopL', True)
    # close may fail stop confirmation for a real disconnected/faulted robot;
    # it must still release both device handles and must not send another move.
    try:
        hardware.close()
    except RobotError:
        pass
    assert not sensor.connected and not receive.connected
    assert not any(call[0] == 'moveL' for call in devices.control.calls)


@pytest.mark.parametrize('fault', ['translation', 'tilt', 'linear_speed', 'angular_speed',
                                   'joint_speed', 'joint_jump', 'wrist'])
def test_actual_motion_bounds_are_enforced(config, fault):
    hardware, devices, sensor = ready(config)
    receive = devices.receive
    if fault == 'translation':
        receive.pose[0] += .0021
    elif fault == 'tilt':
        receive.pose[4] += .5
    elif fault == 'linear_speed':
        receive.speed[0] = .0051
    elif fault == 'angular_speed':
        receive.speed[3] = .041
    elif fault == 'joint_speed':
        receive.qd[0] = .16
    elif fault == 'joint_jump':
        receive.q[0] += .13
    elif fault == 'wrist':
        receive.q[4] = 0
    with pytest.raises(RobotError):
        hardware.read()
    assert devices.control.calls[-1] == ('stopL', True)
    hardware.close()


@pytest.mark.parametrize('fault', ['pose_safety', 'ik', 'joint_safety', 'branch_jump', 'full_turn'])
def test_unreachable_or_discontinuous_path_is_rejected_before_any_motion(config, fault):
    hardware, devices, sensor = ready(config)
    control = devices.control
    if fault == 'pose_safety':
        control.isPoseWithinSafetyLimits = lambda p: False
    elif fault == 'ik':
        control.getInverseKinematicsHasSolution = lambda p, q: False
    elif fault == 'joint_safety':
        control.isJointsWithinSafetyLimits = lambda q: False
    else:
        step = .13 if fault == 'branch_jump' else 2 * np.pi
        control.getInverseKinematics = lambda p, q: np.array(q) + [step, 0, 0, 0, 0, 0]
    with pytest.raises(HardwareError):
        hardware.preflight([target_for(hardware)], hardware.read)
    assert not any(call[0] == 'moveL' for call in control.calls)
    assert control.calls[-1] == ('stopL', True)
    hardware.close()


def test_watchdog_deadline_cannot_be_revived_by_new_data(config):
    hardware, devices, sensor = ready(config)
    hardware._watchdog_kick_time -= 1.
    kicks = []
    devices.control.kickWatchdog = lambda: kicks.append(True) or True
    with pytest.raises(HardwareError, match='watchdog deadline'):
        hardware.read()
    assert not kicks
    hardware.close()


def wide_motion_limits():
    return replace(Limits(), inner_tilt_deg=20., outer_tilt_deg=45.,
                   interleave_tilts=True, max_tilt_rad=np.deg2rad(46.),
                   joint_excursion_rad=1.5)


def test_wide_tilt_requires_full_preflight_and_keeps_motion_rates(config):
    hardware, devices, sensor = ready(config, wide_motion_limits())
    target = make_plan(hardware.origin_pose, hardware.limits)[3]['target']
    checked = []
    devices.control.isPoseWithinSafetyLimits = lambda pose: checked.append(pose) or True
    report = hardware.preflight([target, hardware.origin_pose], hardware.read)
    assert 90 <= report['sample_count'] <= 94
    assert len(checked) == report['sample_count']
    hardware.command(target)
    sample = hardware.read()
    assert _orientation_distance(hardware.origin_pose[3:], sample['actual_tcp_pose'][3:]) == pytest.approx(np.deg2rad(45.))
    hardware.command(hardware.origin_pose)
    assert hardware.command_speed == .005
    assert hardware.acceleration == .03
    assert hardware.limits.max_translation_m == .002
    assert hardware.WATCHDOG_HZ == 10.
    assert sum(call[0] == 'moveL' for call in devices.control.calls) == 2
    hardware.close()


def test_wide_measured_tilt_beyond_hard_limit_stops_without_return(config):
    hardware, devices, sensor = ready(config, wide_motion_limits())
    origin_rotation = _rotvec_to_matrix(hardware.origin_pose[3:])
    devices.receive.pose[3:] = matrix_to_rotvec(
        _rotvec_to_matrix([np.deg2rad(46.01), 0., 0.]) @ origin_rotation)
    with pytest.raises(RobotError, match='orientation drift|tilt bound'):
        hardware.read()
    assert devices.control.calls[-1] == ('stopL', True)
    with pytest.raises(HardwareError, match='forbidden'):
        hardware.command(hardware.origin_pose)
    assert not any(call[0] == 'moveL' for call in devices.control.calls)
    hardware.close()


def test_wide_joint_excursion_is_finite_even_when_each_step_is_continuous(config):
    hardware, devices, sensor = ready(config, wide_motion_limits())
    for step in range(1, 16):
        devices.receive.q[0] = hardware.origin_q[0] + step * .099
        hardware.read()
    devices.receive.q[0] = hardware.origin_q[0] + 1.51
    with pytest.raises(HardwareError, match='joint excursion'):
        hardware.read()
    assert devices.control.calls[-1] == ('stopL', True)
    with pytest.raises(HardwareError, match='forbidden'):
        hardware.command(hardware.origin_pose)
    hardware.close()


@pytest.mark.parametrize('fault', ['pose_safety', 'ik', 'joint_safety', 'joint_excursion'])
def test_wide_preflight_still_rejects_unsafe_or_unreachable_paths_before_motion(config, fault):
    hardware, devices, sensor = ready(config, wide_motion_limits())
    control = devices.control
    if fault == 'pose_safety':
        control.isPoseWithinSafetyLimits = lambda pose: False
    elif fault == 'ik':
        control.getInverseKinematicsHasSolution = lambda pose, joints: False
    elif fault == 'joint_safety':
        control.isJointsWithinSafetyLimits = lambda joints: False
    else:
        control.getInverseKinematics = lambda pose, joints: np.array(joints) + [.04, 0., 0., 0., 0., 0.]
    target = make_plan(hardware.origin_pose, hardware.limits)[3]['target']
    with pytest.raises(HardwareError):
        hardware.preflight([target], hardware.read)
    assert not any(call[0] == 'moveL' for call in control.calls)
    assert control.calls[-1] == ('stopL', True)
    with pytest.raises(HardwareError, match='forbidden'):
        hardware.command(target)
    hardware.close()


def test_read_timings_separate_sensor_rtde_and_watchdog_without_refreshing_snapshot(config, monkeypatch):
    hardware, devices, sensor = ready(config)
    now = [time.monotonic()]
    started = now[0]
    monkeypatch.setattr(hardware_module, 'time',
                        SimpleNamespace(monotonic=lambda: now[0], sleep=time.sleep))
    original_sensor_read, original_state_read = sensor.read_wrench, hardware.controller.read_state

    def sensor_read():
        now[0] += .006
        return original_sensor_read()

    def state_read():
        now[0] += .009
        return original_state_read()

    def kick():
        now[0] += .011
        return True

    sensor.read_wrench = sensor_read
    hardware.controller.read_state = state_read
    devices.control.kickWatchdog = kick
    sample = hardware.read()
    assert sample['sensor_read_duration_sec'] == pytest.approx(.006)
    assert sample['rtde_read_duration_sec'] == pytest.approx(.009)
    assert sample['watchdog_duration_sec'] == pytest.approx(.011)
    assert sample['pair_read_duration_sec'] == pytest.approx(.015)
    assert sample['hardware_read_duration_sec'] == pytest.approx(.026)
    assert sample['host_monotonic'] == pytest.approx(started + .015)
    assert sample['host_monotonic'] < sample['hardware_read_ended_monotonic']
    assert hardware._last_good == sample['host_monotonic']
    assert hardware.last_read_diagnostics == hardware.metadata['last_read_diagnostics']
    assert hardware.last_read_diagnostics['read_completed'] is True
    hardware.close()


@pytest.mark.parametrize('fault', ['observation_deadline', 'previous_watchdog_deadline'])
def test_late_watchdog_kick_stops_and_cannot_authorize_motion(config, monkeypatch, fault):
    hardware, devices, sensor = ready(config)
    target = target_for(hardware)
    hardware.preflight([target], hardware.read)
    last_good = hardware._last_good
    now = [time.monotonic()]
    monkeypatch.setattr(hardware_module, 'time',
                        SimpleNamespace(monotonic=lambda: now[0], sleep=time.sleep))
    if fault == 'previous_watchdog_deadline':
        hardware._watchdog_kick_time = now[0] - .08
    previous_kick = hardware._watchdog_kick_time
    delay = .081 if fault == 'observation_deadline' else .03

    def kick():
        now[0] += delay
        return True

    devices.control.kickWatchdog = kick
    with pytest.raises(HardwareError, match='timeout during watchdog|watchdog deadline') as caught:
        hardware.command(target)
    timing = caught.value.diagnostics['hardware_read_timing']
    assert timing['watchdog_duration_sec'] == pytest.approx(delay)
    assert timing['hardware_read_duration_sec'] == pytest.approx(delay)
    assert timing['hardware_read_stage'] == 'watchdog'
    assert timing['read_completed'] is False
    assert hardware.metadata['last_read_diagnostics'] == timing
    assert hardware._last_good == last_good
    assert hardware._watchdog_kick_time == previous_kick
    assert devices.control.calls[-1] == ('stopL', True)
    assert not any(call[0] == 'moveL' for call in devices.control.calls)
    with pytest.raises(HardwareError, match='forbidden'):
        hardware.command(target)
    hardware.close()


def test_deadline_is_rechecked_after_observation_assembly(config, monkeypatch):
    hardware, devices, sensor = ready(config)
    now = [time.monotonic()]
    monkeypatch.setattr(hardware_module, 'time',
                        SimpleNamespace(monotonic=lambda: now[0], sleep=time.sleep))
    original_datetime = hardware_module.datetime

    def delayed_now(tz):
        now[0] += .081
        return original_datetime.now(tz)

    monkeypatch.setattr(hardware_module, 'datetime', SimpleNamespace(now=delayed_now))
    with pytest.raises(HardwareError, match='timeout before return') as caught:
        hardware.read()
    timing = caught.value.diagnostics['hardware_read_timing']
    assert timing['hardware_read_stage'] == 'observation_assembly'
    assert timing['hardware_read_duration_sec'] == pytest.approx(.081)
    assert timing['watchdog_duration_sec'] == pytest.approx(0.)
    assert devices.control.calls[-1] == ('stopL', True)
    hardware.close()


def test_sensor_timeout_preserves_exception_and_partial_timing_diagnostics(config, monkeypatch):
    hardware, devices, sensor = ready(config)
    now = [time.monotonic()]
    monkeypatch.setattr(hardware_module, 'time',
                        SimpleNamespace(monotonic=lambda: now[0], sleep=time.sleep))
    original = PX6DTimeout('timed out while reading a new sample')
    original.diagnostics = {'sensor_transport': 'test original diagnostics'}

    def timeout():
        now[0] += .016
        raise original

    sensor.read_wrench = timeout
    with pytest.raises(PX6DTimeout) as caught:
        hardware.read()
    assert caught.value is original
    assert original.diagnostics['sensor_transport'] == 'test original diagnostics'
    timing = original.diagnostics['hardware_read_timing']
    assert timing['sensor_read_duration_sec'] == pytest.approx(.016)
    assert timing['hardware_read_duration_sec'] == pytest.approx(.016)
    assert timing['rtde_read_duration_sec'] == timing['watchdog_duration_sec'] == 0.
    assert timing['hardware_read_stage'] == 'sensor_read'
    assert timing['read_completed'] is False
    assert hardware.last_read_diagnostics == timing
    assert devices.control.calls[-1] == ('stopL', True)
    hardware.close()


def test_stale_robot_packets_stop_and_cannot_authorize_motion(config):
    hardware, devices, sensor = ready(config)
    devices.receive.frozen_stamp = hardware._last_stamp
    with pytest.raises(RobotError, match='stale|timestamp|watchdog'):
        hardware.read()
    assert devices.control.calls[-1] == ('stopL', True)
    devices.receive.frozen_stamp = None
    hardware.close()


def test_immediate_duplicate_packet_waits_for_fresh_one(config):
    hardware, devices, sensor = ready(config)
    previous = hardware._last_stamp
    stamps = iter([previous, previous, previous + .002])
    devices.receive.getTimestamp = lambda: next(stamps, time.monotonic())
    sample = hardware.read()
    assert sample['robot_timestamp'] > previous
    hardware.close()


def test_interrupt_during_preflight_stops_without_motion(config):
    hardware, devices, sensor = ready(config)

    def quit_key():
        raise KeyboardInterrupt()

    with pytest.raises(KeyboardInterrupt):
        hardware.preflight([target_for(hardware)], quit_key)
    assert devices.control.calls[-1] == ('stopL', True)
    assert not any(call[0] == 'moveL' for call in devices.control.calls)
    hardware.close()


def test_tcp_mismatch_prevents_motion_and_preserves_offset(config):
    hardware, devices, sensor = rig(config)
    hardware.connect()
    original = list(config['tcp']['offset'])
    devices.receive.tcp[0] += .01
    with pytest.raises(RobotError, match='does not match'):
        hardware.activate()
    assert config['tcp']['offset'] == original
    assert not any(call[0] == 'moveL' for call in devices.control.calls)
    hardware.close()


def test_stop_failure_does_not_remove_motion_latch(config):
    hardware, devices, sensor = ready(config)

    def broken_stop(*args):
        raise RuntimeError('transport failure')

    devices.control.stopL = broken_stop
    hardware.stop()
    assert hardware.controller.stop_report['api_anomaly']
    with pytest.raises(HardwareError, match='forbidden'):
        hardware.command(hardware.origin_pose)
    hardware.close()


def test_contract_inspects_owned_control_before_watchdog_or_motion(config):
    hardware, devices, sensor = rig(config)
    hardware._inspect_sdk_contract = True
    hardware.connect()
    with pytest.raises(HardwareError, match='unsupported SDK return contract'):
        hardware.activate()
    assert devices.control_count == 1
    assert not any(call[0] in ('setWatchdog', 'moveL') for call in devices.control.calls)
    hardware.close()


def test_sensor_error_survives_secondary_stop_and_close_errors(config):
    hardware, devices, sensor = ready(config)
    original = PX6DTimeout('original sensor timeout')
    sensor.error = original

    def broken_stop(**kwargs):
        raise RuntimeError('secondary braking error')

    hardware.controller.request_stop = broken_stop
    with pytest.raises(PX6DTimeout) as caught:
        hardware.read()
    assert caught.value is original
    assert any('secondary braking error' in note for note in original.__notes__)
    hardware.close()
    assert not sensor.connected and not devices.receive.connected


def test_connection_error_survives_secondary_sensor_cleanup_error(config):
    hardware, devices, sensor = rig(config)
    original = RuntimeError('original sensor connect error')

    def broken_connect():
        raise original

    def broken_close():
        raise RuntimeError('secondary sensor close error')

    sensor.connect, sensor.close = broken_connect, broken_close
    with pytest.raises(RuntimeError, match='original sensor connect error') as caught:
        hardware.connect()
    assert caught.value is original
    assert any('secondary sensor close error' in note for note in original.__notes__)
    assert not devices.receive.connected
    hardware.close()


def test_interpolation_near_pi_covers_both_endpoint_and_internal_bounds():
    start = np.array([.4, -.2, .3, 0, np.pi, 0])
    target = start.copy()
    target[0] += .0018
    target[4] += np.deg2rad(25)
    previous = start
    path = list(interpolate_poses(start, target))
    assert len(path) >= 25
    for point in path:
        assert np.linalg.norm(point[:3] - previous[:3]) <= .001 + 1e-10
        assert _orientation_distance(point[3:], previous[3:]) <= np.deg2rad(1) + 1e-8
        previous = point
    assert _orientation_distance(path[-1][3:], target[3:]) < 1e-7
