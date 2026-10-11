"""Per-group budgets and fresh measured stops; all SDK/force inputs are fake."""
from argparse import Namespace
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from app.single_point_config import load_settings, prepare, validate_settings
from app.single_point_runtime import FreshForceReader, SinglePointTrial
from app.single_point_ui import show_trial_result
from calibration.single_point import save_group, set_reference
from robot.control_mode import ControlMode
from robot.rtde_controller import RobotError
from sensor.force_preprocess import WrenchPreprocessor
from simulation.single_point import OfflineController, OfflineForceReader, demo_calibration
from tests.doubles import Devices, Sensor
from tests.test_single_point import MemoryLog
import run_single_point_contact as entry


@pytest.fixture
def settings(config):
    values = load_settings(entry.ROOT/'single_point_experiment.yaml', config)
    values.update(workspace_enabled=False, workspace_limits=None, workspace_xy_polygon=None)
    return values


def long_calibration(config):
    data = set_reference({}, [.6816133511791436, .11456427664583589, -.010483378561170265,
                              np.pi, 0, 0], config['robot']['robot_ip'], config['tcp']['offset'])
    return save_group(data, 'A', [.5411802690012457, .4299559965236667, .9, .1, .2, .3])


def test_independent_safety_settings_survive_parameter_inheritance(tmp_path, config, settings):
    settings.update(max_approach_time_sec=90., stop_timeout_sec=1.2,
                    post_stop_record_sec=.6, workspace_enabled=True, workspace_verified=True,
                    workspace_limits=dict(x_min=.3, x_max=.9, y_min=-.4, y_max=.6, z_min=-.1, z_max=.4))
    path = tmp_path/'settings.yaml'
    path.write_text(yaml.safe_dump(settings))
    config['continuous_tracking'].update(search_max_distance=.1, search_max_time_sec=5., confirmation_timeout_sec=.2)
    loaded = load_settings(path, config)
    for key in ('max_approach_time_sec', 'stop_timeout_sec', 'post_stop_record_sec',
                'workspace_enabled', 'workspace_verified', 'workspace_limits'):
        assert loaded[key] == settings[key]
    assert loaded['speed_default_mps'] == config['continuous_tracking']['search_speed']


@pytest.mark.parametrize('boundary', ['disabled', 'missing', 'unverified', 'zero', 'nan', 'missing_axis'])
def test_real_start_refuses_invalid_boundary_before_opening_devices(monkeypatch, config, settings, boundary):
    settings.update(workspace_enabled=True, workspace_verified=True, workspace_limits=dict(
        x_min=.3, x_max=.9, y_min=-.4, y_max=.6, z_min=-.1, z_max=.4))
    if boundary == 'disabled': settings['workspace_enabled'] = False
    if boundary == 'missing': settings['workspace_limits'] = None
    if boundary == 'unverified': settings['workspace_verified'] = False
    if boundary == 'zero': settings['workspace_limits']['x_max'] = settings['workspace_limits']['x_min']
    if boundary == 'nan': settings['workspace_limits']['x_max'] = float('nan')
    if boundary == 'missing_axis': del settings['workspace_limits']['y_min']
    from app import single_point_ui as ui
    monkeypatch.setattr(ui, 'read_control_mode', lambda ip: ControlMode('remote', 'fake true'))
    def forbidden(*a, **kw): raise AssertionError('invalid boundary must not open devices')
    with pytest.raises(ValueError, match='workspace'):
        entry.run_trial(Namespace(execute=True), config, settings, long_calibration(config), 'A',
                        controller_factory=forbidden, reader_factory=forbidden)


@pytest.mark.parametrize('time_limit', [0., -1., float('inf'), float('nan'), True])
def test_experiment_time_limit_is_finite_positive(settings, time_limit):
    settings['max_approach_time_sec'] = time_limit
    with pytest.raises(ValueError, match='max_approach_time_sec'):
        validate_settings(settings)


@pytest.mark.parametrize('contact', [.40, 10.])
def test_search_passes_reference_and_only_force_completes_contact(config, settings, contact, capsys):
    data = long_calibration(config)
    settings['max_approach_time_sec'] = 24.
    effective, _, pose, direction = prepare(config, settings, data, 'A', execute=False)
    controller = OfflineController(pose, settings)
    processor = WrenchPreprocessor.from_config(effective['preprocessing'], tool_orientation=pose[3:])
    raw = OfflineForceReader(controller, pose, direction, processor, contact_distance=contact)
    reader = FreshForceReader(raw, settings, config['policy'], clock=controller.clock)
    log = MemoryLog()
    trial = SinglePointTrial(effective, settings, data, 'A', .018, controller, reader, processor, log,
                             clock=controller.clock, sleep=controller.sleep, preparation_complete=True)
    result = trial.run()
    assert result['actual_search_path_m'] > .345244
    assert result['post_stop_record_completed'] and result['fault'] == ''
    if contact == .40:
        assert result['status'] == 'contact' and result['reason'] == 'CONTACT_THRESHOLD'
        assert result['force_at_threshold']['fxy_N'] >= settings['contact_threshold_N']
        assert result['actual_before_threshold']['timestamp'] < result['threshold_trigger_sec']
    else:
        assert result['status'] == 'aborted' and result['reason'] == 'EXPERIMENT_TIME_LIMIT'
        assert result['threshold_trigger_sec'] is None and result['force_at_threshold'] is None
        show_trial_result(result)
        output = capsys.readouterr().out
        assert '异常停止' in output and '超过最大允许实验运行时间' in output
    assert result['actual_stop_tcp_speed'] is not None
    assert any(row['kind'] == 'braking' for row in log.commands)


@pytest.mark.parametrize('guard', ['workspace', 'obstacle'])
def test_live_boundary_is_checked_beyond_reference(config, settings, guard):
    from tests.test_single_point import rig
    settings.update(workspace_enabled=True, workspace_verified=True, workspace_limits=dict(
        x_min=.3, x_max=.43, y_min=-.1, y_max=.1, z_min=.1, z_max=.3))
    if guard == 'obstacle':
        settings['workspace_limits']['x_max'] = .6
        settings['forbidden_boxes'] = [dict(x_min=.43, x_max=.44, y_min=-.01,
                                          y_max=.01, z_min=.19, z_max=.21)]
    trial, controller, log = rig(config, settings, contact=10.)
    result = trial.run()
    assert result['status'] == 'aborted' and result['reason'] == 'DEVICE_OR_SAFETY_ERROR'
    assert ('workspace' if guard == 'workspace' else 'intersects') in result['fault']
    assert result['actual_search_path_m'] > .01
    assert result['threshold_trigger_sec'] is None and result['actual_stop_pose'] is not None
    assert result['actual_stop_pose'][0] < .43
    braking = next(i for i, row in enumerate(log.commands) if row['kind'] == 'braking')
    assert all(row['kind'] != 'motion' for row in log.commands[braking:])


@pytest.mark.parametrize('failure', ['height', 'orientation', 'lateral', 'reverse',
                                   'overspeed', 'rtde_error', 'stale_tcp'])
def test_actual_motion_and_rtde_faults_always_abort(config, settings, failure):
    from tests.test_single_point import rig
    trial, controller, log = rig(config, settings, contact=10.)
    normal = controller.read_state
    def bad_state():
        state = normal()
        if controller.stop_report is None and controller.clock() >= .05:
            if failure == 'height': state.pose[2] += .01
            if failure == 'orientation': state.pose[3] += .1
            if failure == 'lateral': state.pose[1] += .01
            if failure == 'reverse': state.pose[0] = trial.p0[0]-.01
            if failure == 'overspeed': state.tcp_speed[0] = .1
            if failure == 'rtde_error': raise RobotError('fake RTDE communication failure')
            if failure == 'stale_tcp': state.timestamp -= 1.
        return state
    controller.read_state = bad_state
    result = trial.run()
    assert result['status'] == 'aborted' and result['reason'] == 'DEVICE_OR_SAFETY_ERROR'
    assert result['threshold_trigger_sec'] is None and result['actual_standstill_sec'] is not None
    assert any(row['kind'] == 'motion' for row in log.commands)
    assert any(row['kind'] == 'braking' for row in log.commands)


def test_time_limit_takes_priority_over_simultaneous_force_crossing(config, settings):
    from core.models import Wrench
    from tests.test_single_point import rig
    settings['max_approach_time_sec'] = .1
    trial, controller, log = rig(config, settings, contact=10.)
    normal_process = trial.processor.process
    def crossing(raw):
        value = normal_process(raw)
        return Wrench(2., 0., 0., 0., 0., 0.) if controller.clock() >= .1 else value
    trial.processor.process = crossing
    result = trial.run()
    assert result['status'] == 'aborted' and result['reason'] == 'EXPERIMENT_TIME_LIMIT'
    assert result['threshold_trigger_sec'] is None
    assert not any(event.event_type == 'FIRST_THRESHOLD_STOP_REQUEST' for event in log.events)


def test_normal_display_has_only_direction_speed_and_force_no_travel_budget(config, settings, capsys):
    entry.show_group(long_calibration(config), 'A', settings, .018, config)
    output = capsys.readouterr().out
    for text in ('实验组：P0-A', '接近方向：', '接近速度：18.000 mm/s', '接触阈值：1.0 N', '正常停止：力达到 1.0 N'):
        assert text in output
    for text in ('行程', '倍率', '候选', '搜索距离', 'P_ref'):
        assert text not in output


def make_rtde_stop(monkeypatch, config, settings, *, failure=None):
    """Use the unchanged real RTDE stop owner with fake SDK and packet feedback."""
    import robot.rtde_controller as rtde
    clock = SimpleNamespace(now=0.)
    clock.read = lambda: clock.now
    def sleep(duration): clock.now += duration
    clock.sleep = sleep
    monkeypatch.setattr(rtde, 'time', SimpleNamespace(monotonic=clock.read, sleep=sleep))
    effective, robot_config, pose, _ = prepare(config, settings, demo_calibration(config), 'A', execute=False)
    devices = Devices(config, pose)
    owner = devices.controller(robot_config)
    packet = [0]
    def timestamp():
        if failure != 'stale' or owner.stop_report is None or clock.now-owner._stop_started < .6:
            packet[0] += 1
        return packet[0]*.002
    def speed():
        if owner.stop_report is None:
            return np.zeros(6)
        elapsed = clock.now-owner._stop_started
        moving = (failure == 'never_stops' or elapsed < .62 or
                  (failure == 'revoked' and elapsed >= .85))
        return np.array([.018 if moving else 0., 0, 0, 0, 0, 0])
    devices.receive.getTimestamp = timestamp
    devices.receive.getActualTCPSpeed = speed
    owner.connect()
    owner.activate_control(confirmed=True)
    processor = WrenchPreprocessor.from_config(effective['preprocessing'], tool_orientation=pose[3:])
    reader = FreshForceReader(Sensor(), settings, config['policy'], clock=clock.read)
    trial = SinglePointTrial(effective, settings, demo_calibration(config), 'A', .018, owner, reader,
                             processor, MemoryLog(), clock=clock.read, sleep=sleep, preparation_complete=True)
    trial._observe()
    if failure == 'mode_exit_reject':
        devices.control.stop_value = False
    if failure == 'braking_reject':
        normal = devices.control.speedL
        def rejected(*a): normal(*a); return False
        devices.control.speedL = rejected
    trial._request_stop('EXPERIMENT_TIME_LIMIT')
    return trial, owner, clock


def test_stop_confirmation_07s_then_record_05s_have_separate_deadlines(monkeypatch, config, settings):
    trial, owner, _ = make_rtde_stop(monkeypatch, config, settings)
    trial.settle()
    result = trial.summary()
    assert result['status'] == 'aborted' and result['fault'] == ''
    assert result['actual_standstill_sec']-result['braking_request_sec'] == pytest.approx(.7, abs=.011)
    assert result['post_stop_record_end_sec']-result['post_stop_record_start_sec'] >= .5
    assert result['post_stop_record_end_sec']-result['braking_request_sec'] > 1.
    assert result['post_stop_record_completed'] and result['timeout_phase'] is None
    np.testing.assert_array_equal(result['actual_stop_tcp_speed'], [0]*6)
    assert owner.stop_report['braking_return_value'] is True
    assert owner.stop_report['return_value'] is True and owner.stop_report['physical_stop'] == 'confirmed'
    assert trial.logger.samples[-1]['extra']['post_stop_record_end_sec'] == result['post_stop_record_end_sec']
    assert result['post_stop_record_completed_sec'] >= result['post_stop_record_end_sec']


@pytest.mark.parametrize('failure, kind', [('never_stops', 'STANDSTILL_NOT_CONFIRMED'),
                                         ('stale', 'STANDSTILL_NOT_CONFIRMED'),
                                         ('revoked', 'STANDSTILL_REVOKED')])
def test_measured_stop_failures_remain_failures(monkeypatch, config, settings, failure, kind):
    trial, _, _ = make_rtde_stop(monkeypatch, config, settings, failure=failure)
    with pytest.raises(RobotError):
        trial.settle()
    result = trial.summary()
    assert result['status'] == 'aborted' and result['stop_failure_kind'] == kind
    if failure == 'revoked':
        assert result['actual_stop_pose'] is not None and result['actual_standstill_sec'] is not None
    else:
        assert result['actual_standstill_sec'] is None
    if failure == 'never_stops':
        assert result['timeout_phase'] == 'BRAKING_CONFIRMATION'


@pytest.mark.parametrize('failure', ['braking_reject', 'mode_exit_reject'])
def test_sdk_rejection_is_distinct_from_measured_stop(monkeypatch, config, settings, failure):
    trial, _, _ = make_rtde_stop(monkeypatch, config, settings, failure=failure)
    trial.settle()
    result = trial.summary()
    assert result['status'] == 'aborted' and result['stop_failure_kind'] == 'BRAKING_API_FAILURE'
    assert result['actual_standstill_sec'] is not None and result['post_stop_record_completed']
    assert result['stop_report']['api_anomaly']


def test_post_stop_recording_timeout_does_not_claim_braking_failure(monkeypatch, config, settings, capsys):
    trial, _, clock = make_rtde_stop(monkeypatch, config, settings)
    def gap(duration):
        clock.now += .6 if trial.stopped_time is not None else duration
    trial.sleep = gap
    with pytest.raises(RobotError, match='post-stop data recording'):
        trial.settle()
    result = trial.summary()
    assert result['stop_failure_kind'] == 'POST_STOP_RECORDING_INCOMPLETE'
    assert result['timeout_phase'] == 'POST_STOP_RECORDING'
    assert result['actual_stop_pose'] is not None and result['actual_standstill_sec'] is not None
    assert result['status'] == 'aborted' and not result['post_stop_record_completed']
    show_trial_result(result)
    output = capsys.readouterr().out
    assert '实际停稳已确认' in output and '超时阶段：停稳后的数据记录' in output


def test_post_stop_logging_failure_preserves_stop_evidence(monkeypatch, config, settings):
    trial, _, _ = make_rtde_stop(monkeypatch, config, settings)
    normal = trial.logger.log_sample
    def fail_after_confirmation(*a, **kw):
        if trial.stopped_time is not None:
            raise OSError('fake post-stop disk failure')
        return normal(*a, **kw)
    trial.logger.log_sample = fail_after_confirmation
    trial.settle()
    result = trial.summary()
    assert result['status'] == 'aborted' and result['actual_stop_pose'] is not None
    assert result['stop_failure_kind'] == 'POST_STOP_RECORDING_INCOMPLETE'
    assert result['timeout_phase'] is None


def test_fresh_execution_read_enforces_workspace_before_sdk_send(config, settings):
    from tests.test_single_point import bounded, rig
    bounded(settings)
    trial, controller, log = rig(config, settings)
    trial.authorize_approach()
    controller.model.pose[0] = .4985
    with pytest.raises(ValueError, match='workspace'):
        controller.command_planar_velocity(trial.direction, trial.speed, .01)
    assert not any(row['kind'] == 'motion' for row in controller.command_records)


def test_braking_logging_gap_is_not_post_stop_recording_failure(monkeypatch, config, settings):
    trial, _, _ = make_rtde_stop(monkeypatch, config, settings)
    normal = trial.logger.log_sample
    def fail_during_braking(*a, **kw):
        if trial.stopped_time is None:
            raise OSError('fake braking disk failure')
        return normal(*a, **kw)
    trial.logger.log_sample = fail_during_braking
    trial.settle()
    result = trial.summary()
    assert result['status'] == 'aborted' and result['actual_standstill_sec'] is not None
    assert result['post_stop_record_completed']
    assert result['stop_failure_kind'] == 'STOP_TELEMETRY_INCOMPLETE'
    assert result['stop_failures'][0]['phase'] == 'BRAKING_CONFIRMATION'
