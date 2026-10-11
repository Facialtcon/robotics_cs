"""Per-group budgets and fresh measured stops; all SDK/force inputs are fake."""
from argparse import Namespace
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from app.single_point_config import load_settings, prepare, resolve_search_budget, validate_settings
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
    return load_settings(entry.ROOT/'single_point_experiment.yaml', config)


def long_calibration(config):
    data = set_reference({}, [.6816133511791436, .11456427664583589, -.010483378561170265,
                              np.pi, 0, 0], config['robot']['robot_ip'], config['tcp']['offset'])
    return save_group(data, 'A', [.5411802690012457, .4299559965236667, .9, .1, .2, .3])


def test_independent_budgets_survive_parameter_inheritance(tmp_path, config, settings):
    settings.update(max_search_distance_m=.38, max_approach_time_sec=90., stop_timeout_sec=1.2,
                    post_stop_record_sec=.6, search_margin_m=.005, search_distance_multiplier=1.5)
    path = tmp_path/'settings.yaml'
    path.write_text(yaml.safe_dump(settings))
    config['continuous_tracking'].update(search_max_distance=.1, search_max_time_sec=5., confirmation_timeout_sec=.2)
    loaded = load_settings(path, config)
    for key in ('max_search_distance_m', 'max_approach_time_sec', 'stop_timeout_sec',
                'post_stop_record_sec', 'search_margin_m', 'search_distance_multiplier'):
        assert loaded[key] == settings[key]
    assert loaded['speed_default_mps'] == config['continuous_tracking']['search_speed']


@pytest.mark.parametrize('mode', ['fixed', 'reference_distance'])
def test_345mm_reference_with_100mm_budget_refuses_before_device_connection(monkeypatch, config, settings, mode):
    settings.update(search_distance_mode=mode, max_search_distance_m=.1)
    from app import single_point_ui as ui
    monkeypatch.setattr(ui, 'read_control_mode', lambda ip: ControlMode('remote', 'fake true'))
    def forbidden(*a, **kw): raise AssertionError('insufficient budget must not open devices')
    with pytest.raises(ValueError, match='搜索预算不足.*345.244.*100.000'):
        entry.run_trial(Namespace(execute=True), config, settings, long_calibration(config), 'A',
                        controller_factory=forbidden, reader_factory=forbidden)


def test_geometry_budget_is_bounded_per_group_and_does_not_mutate_source(config, settings):
    data = long_calibration(config)
    data = save_group(data, 'B', [.66, .11456427664583589, -.9, -.1, .3, .7])
    settings['search_margin_m'] = .005
    original = deepcopy(settings)
    first = resolve_search_budget(settings, data, 'A')
    second = resolve_search_budget(settings, data, 'B')
    assert first['reference_distance_m'] == pytest.approx(.345244, abs=1e-6)
    assert first['max_search_distance_m'] == pytest.approx(2*first['reference_distance_m']+.005)
    assert second['max_search_distance_m'] < first['max_search_distance_m'] <= .8
    assert settings == original
    settings['max_search_distance_m'] = .3
    with pytest.raises(ValueError, match='搜索预算不足'):
        resolve_search_budget(settings, data, 'A')


def test_doubled_default_and_legacy_config_compatibility(config, settings):
    data = long_calibration(config)
    assert settings['search_distance_multiplier'] == 2.
    assert settings['max_search_distance_m'] == .8
    expanded = resolve_search_budget(settings, data, 'A')
    old_settings = deepcopy(settings)
    del old_settings['search_distance_multiplier']
    old_settings['max_search_distance_m'] = .4
    original = resolve_search_budget(old_settings, data, 'A')
    assert expanded['max_search_distance_m'] == pytest.approx(2*original['max_search_distance_m'])
    settings['max_search_distance_m'] = .6
    with pytest.raises(ValueError, match='搜索预算不足'):
        resolve_search_budget(settings, data, 'A')


@pytest.mark.parametrize('multiplier', [0., -1., float('inf'), float('nan'), True])
def test_search_multiplier_must_be_finite_positive(settings, multiplier):
    settings['search_distance_multiplier'] = multiplier
    with pytest.raises(ValueError, match='search_distance_multiplier'):
        validate_settings(settings)


@pytest.mark.parametrize('margin', [-.001, .021, float('inf'), float('nan'), True])
def test_extra_margin_is_finite_and_capped(settings, margin):
    settings['search_margin_m'] = margin
    with pytest.raises(ValueError, match='search_margin_m'):
        validate_settings(settings)


def test_time_budget_is_checked_for_selected_speed_before_approach(config, settings):
    with pytest.raises(ValueError, match='搜索时间预算不足'):
        resolve_search_budget(settings, long_calibration(config), 'A', speed=.0005)


@pytest.mark.parametrize('guard', ['workspace', 'obstacle'])
def test_full_345mm_search_keeps_workspace_and_obstacle_checks(config, settings, guard):
    settings['search_distance_multiplier'] = 1.
    settings.update(workspace_enabled=True, workspace_limits=dict(
        x_min=.4, x_max=.8, y_min=0, y_max=.5, z_min=-.1, z_max=.1))
    if guard == 'workspace':
        settings['workspace_limits']['x_max'] = .65
    else:
        settings['forbidden_boxes'] = [dict(x_min=.6, x_max=.62, y_min=.24, y_max=.31,
                                          z_min=-.03, z_max=.01)]
    with pytest.raises(ValueError, match='workspace|intersects'):
        prepare(config, settings, long_calibration(config), 'A', execute=True)


def test_doubled_search_checks_obstacles_beyond_reference(config, settings):
    data = long_calibration(config)
    point = np.array(data['groups']['A']['tcp_pose'][:2]) + .5*np.array(data['groups']['A']['direction_xy'])
    settings.update(workspace_enabled=True, workspace_limits=dict(
        x_min=.4, x_max=1., y_min=-.5, y_max=.5, z_min=-.1, z_max=.1),
        forbidden_boxes=[dict(x_min=point[0]-.005, x_max=point[0]+.005,
                              y_min=point[1]-.005, y_max=point[1]+.005, z_min=-.03, z_max=.01)])
    original = deepcopy(settings)
    original['search_distance_multiplier'] = 1.
    prepare(config, original, data, 'A', execute=True)
    with pytest.raises(ValueError, match='intersects'):
        prepare(config, settings, data, 'A', execute=True)


@pytest.mark.parametrize('contact', [.33, 10.])
def test_correct_budget_searches_past_100mm_and_brakes_for_force_or_distance(config, settings, contact, capsys):
    data = long_calibration(config)
    effective, _, pose, direction = prepare(config, settings, data, 'A', execute=False)
    resolved = effective['single_point_experiment']
    controller = OfflineController(pose, resolved)
    processor = WrenchPreprocessor.from_config(effective['preprocessing'], tool_orientation=pose[3:])
    raw = OfflineForceReader(controller, pose, direction, processor, contact_distance=contact)
    reader = FreshForceReader(raw, resolved, config['policy'], clock=controller.clock)
    log = MemoryLog()
    trial = SinglePointTrial(effective, resolved, data, 'A', .018, controller, reader, processor, log,
                             clock=controller.clock, sleep=controller.sleep, preparation_complete=True)
    result = trial.run()
    assert result['status'] == ('contact' if contact == .33 else 'no_contact')
    assert result['actual_search_path_m'] > .3 and result['post_stop_record_completed']
    assert result['fault'] == '' and result['actual_stop_tcp_speed'] is not None
    assert result['post_stop_record_end_sec']-result['post_stop_record_start_sec'] >= .5
    assert any(row['kind'] == 'braking' for row in log.commands)
    if contact == 10.:
        assert result['reason'] == 'NO_CONTACT_DISTANCE'
        trigger = result['no_contact_distance_trigger']
        # The existing before-send guard brakes before the next step crosses the budget.
        assert np.linalg.norm(np.array(trigger['pose'][:2])-pose[:2]) >= (
            resolved['max_search_distance_m']-.018/resolved['control_rate_hz'])
        assert resolved['max_search_distance_m'] == pytest.approx(.690488, abs=1e-6)
        event = next(event for event in log.events if event.event_type == 'BUDGET_STOP')
        np.testing.assert_array_equal(event.pose, trigger['pose'])
        show_trial_result(result)
        assert '未找到接触：搜索距离耗尽。' in capsys.readouterr().out


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
    trial._request_stop('NO_CONTACT_DISTANCE')
    return trial, owner, clock


def test_stop_confirmation_07s_then_record_05s_have_separate_deadlines(monkeypatch, config, settings):
    trial, owner, _ = make_rtde_stop(monkeypatch, config, settings)
    trial.settle()
    result = trial.summary()
    assert result['status'] == 'no_contact' and result['fault'] == ''
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


def test_fresh_execution_read_is_the_distance_trigger_pose(config, settings):
    from tests.test_single_point import rig
    trial, controller, _ = rig(config, settings)
    trial._observe()
    actual = controller.read_state()
    actual.pose[0] = trial.p0[0]+settings['max_search_distance_m']
    from app.single_point_runtime import SinglePointSearchLimit
    with pytest.raises(SinglePointSearchLimit):
        trial._before_send(actual)
    trial._request_stop('NO_CONTACT_DISTANCE')
    np.testing.assert_array_equal(trial.distance_trigger['pose'], actual.pose)


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
