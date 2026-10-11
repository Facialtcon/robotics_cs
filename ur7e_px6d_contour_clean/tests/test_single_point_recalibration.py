"""Calibration transactions and mode feedback, using temporary files/fake devices only."""
from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest
import yaml

from app import single_point_ui as ui
from app.single_point_config import load_settings, validate_settings
from calibration.probe_alignment import probe_tilt_deg
from calibration.single_point import (approach_direction, experiment_start_pose, load_calibration,
                                      save_group, set_reference, validate_group, write_calibration)
from robot.control_mode import ControlMode, read_control_mode
from robot.rtde_controller import RobotError
import run_single_point_contact as entry
from simulation.single_point import demo_calibration
from tests.doubles import Devices, Keyboard, Sensor
from tests.test_single_point_ui import DashboardSocket, menu_run, temporary_args


@pytest.fixture
def settings(config):
    values = load_settings(entry.ROOT/'single_point_experiment.yaml', config)
    values.update(workspace_enabled=False, workspace_limits=None, workspace_xy_polygon=None)
    return values


def reviewed_data(config, settings):
    data = demo_calibration(config)
    for name, group in list(data['groups'].items()):
        raw = group['tcp_pose'][:2] + [.9, .1, -.3, .6]
        data = save_group(data, name, raw)
        data['groups'][name]['path_review'] = dict(note='old full path review',
            max_distance_m=.1, reviewed_at='legacy timestamp')
    return data


@pytest.mark.parametrize('new_ref', [[.4, .02, .31, .3, -.2, .7], [.39001, 0, .31, .3, -.2, .7]])
def test_recalibrate_reference_keeps_raw_groups_and_legacy_metadata_updates_directions(
        tmp_path, monkeypatch, config, settings, capsys, new_ref):
    data = reviewed_data(config, settings)
    before = deepcopy(data)
    args = temporary_args(tmp_path, data)
    original_bytes = args.calibration.read_bytes()
    monkeypatch.setattr(entry, 'capture_pose', lambda *a, **kw: (np.array(new_ref), config['tcp']['offset']))
    def confirm(prompt):
        assert '保留 P0 XY' in prompt
        assert args.calibration.read_bytes() == original_bytes
        return True
    monkeypatch.setattr(entry, 'confirm_enter', confirm)
    assert menu_run(monkeypatch, args, config, settings, ['1', '1', '3', '0']) == 0
    output = capsys.readouterr().out
    assert '标定成功：P_ref' in output and '修改前 P_ref' in output and '修改后 P_ref' in output
    assert '路径核验' not in output and '回读校验：通过' in output
    loaded = load_calibration(args.calibration)
    assert loaded['reference_tcp_pose'] == new_ref and data == before
    backups = list(tmp_path.glob('calibration.backup-*.yaml'))
    assert len(backups) == 1 and backups[0].read_bytes() == original_bytes
    for name, group in loaded['groups'].items():
        old = before['groups'][name]
        assert group['tcp_pose'] == old['tcp_pose']
        assert group['captured_at'] == old['captured_at']
        assert group['path_review'] == old['path_review']
        target = experiment_start_pose(loaded, name, config['calibration']['probe_axis_tcp'])
        np.testing.assert_array_equal(target[:2], old['tcp_pose'][:2])
        assert target[2] == new_ref[2]
        assert probe_tilt_deg(target[3:], config['calibration']['probe_axis_tcp']) < 1e-5
        if name == 'A' and new_ref[0] == .39001:
            assert group['direction_xy'] is None and group['requires_recalibration']
            assert '距离过近，需要重标定' in output
            with pytest.raises(ValueError, match='距离过近'):
                validate_group(loaded, name, settings, config)
        else:
            expected = np.array(new_ref[:2])-old['tcp_pose'][:2]
            expected /= np.linalg.norm(expected)
            np.testing.assert_allclose(group['direction_xy'], expected)
            validate_group(loaded, name, settings, config)
    targets = [experiment_start_pose(loaded, name, config['calibration']['probe_axis_tcp'])
               for name in loaded['groups']]
    for target in targets[1:]:
        np.testing.assert_array_equal(target[2:], targets[0][2:])


@pytest.mark.parametrize('policy', ['1', '2'])
@pytest.mark.parametrize('stage', ['cancel', 'capture_failure'])
def test_recalibration_cancel_or_capture_failure_preserves_original_file(
        tmp_path, monkeypatch, config, settings, capsys, policy, stage):
    args = temporary_args(tmp_path, reviewed_data(config, settings))
    before = args.calibration.read_bytes()
    def capture(*a, **kw):
        assert args.calibration.read_bytes() == before
        if stage == 'capture_failure':
            raise RobotError('模拟停稳检查失败')
        return np.array([.4, .02, .3, .2, .3, .4]), config['tcp']['offset']
    monkeypatch.setattr(entry, 'capture_pose', capture)
    monkeypatch.setattr(entry, 'confirm_enter', lambda *a: False)
    assert menu_run(monkeypatch, args, config, settings, ['1', policy, '0']) == 0
    output = capsys.readouterr().out
    assert ('标定取消' if stage == 'cancel' else '模拟停稳检查失败') in output
    if stage == 'cancel' and policy == '2':
        assert '即将清空的 P0：A、B、C' in output
    assert '标定成功' not in output and args.calibration.read_bytes() == before
    assert not list(tmp_path.glob('*.backup-*'))


def test_clear_reference_groups_only_after_capture_and_final_confirmation(tmp_path, monkeypatch, config, settings, capsys):
    args = temporary_args(tmp_path, reviewed_data(config, settings))
    before = args.calibration.read_bytes()
    captured = []
    def capture(*a, **kw):
        assert args.calibration.read_bytes() == before
        captured.append(True)
        return np.array([.42, .02, .3, .1, .2, .3]), config['tcp']['offset']
    def confirm(prompt):
        assert captured == [True] and '清空上述所有 P0' in prompt
        assert args.calibration.read_bytes() == before
        return True
    monkeypatch.setattr(entry, 'capture_pose', capture)
    monkeypatch.setattr(entry, 'confirm_enter', confirm)
    assert menu_run(monkeypatch, args, config, settings, ['1', '2', '0']) == 0
    after = load_calibration(args.calibration)
    assert after['P_ref'] == [.42, .02, .3] and after['groups'] == {}
    assert list(tmp_path.glob('*.backup-*'))[0].read_bytes() == before
    assert '即将清空的 P0：A、B、C' in capsys.readouterr().out


@pytest.mark.parametrize('failure', ['write', 'readback', 'backup'])
def test_failed_save_restores_exact_original_bytes(tmp_path, monkeypatch, config, settings, capsys, failure):
    data = reviewed_data(config, settings)
    args = temporary_args(tmp_path, data)
    before = args.calibration.read_bytes()
    updated = set_reference(data, [.4, .02, .3, .2, .3, .4], config['robot']['robot_ip'],
                            config['tcp']['offset'], group_policy='keep')
    if failure == 'write':
        actual_write = entry.write_calibration
        def write_then_fail(path, value):
            actual_write(path, value)
            raise OSError('模拟替换后保存失败')
        monkeypatch.setattr(entry, 'write_calibration', write_then_fail)
    elif failure == 'readback':
        def failed_read(path): raise ValueError('模拟文件回读失败')
        monkeypatch.setattr(entry, 'load_calibration', failed_read)
    else:
        actual_open = Path.open
        def failed_backup(path, *a, **kw):
            if '.backup-' in path.name:
                raise OSError('模拟备份失败')
            return actual_open(path, *a, **kw)
        monkeypatch.setattr(Path, 'open', failed_backup)
    with pytest.raises((OSError, ValueError), match='模拟'):
        entry.save_calibration_checked(args.calibration, updated, 'P_ref')
    assert args.calibration.read_bytes() == before
    assert load_calibration(args.calibration) == data
    assert '标定成功' not in capsys.readouterr().out


@pytest.mark.parametrize('distance', [0, .000015, .004999])
def test_new_near_p0_rejected_without_changing_file(tmp_path, monkeypatch, config, settings, capsys, distance):
    data = demo_calibration(config)
    args = temporary_args(tmp_path, data)
    before = args.calibration.read_bytes()
    raw = [.4+distance, 0, .9, .1, .2, .3]
    monkeypatch.setattr(entry, 'capture_pose', lambda *a, **kw: (np.array(raw), config['tcp']['offset']))
    assert menu_run(monkeypatch, args, config, settings, ['2', 'D', '0']) == 0
    output = capsys.readouterr().out
    assert '标定失败：P0-D' in output and '距离过近' in output
    assert f'实际 {distance*1000:.3f} mm' in output and '最小要求 5.000 mm' in output
    assert '请将 P0 移远后重新采集' in output
    assert args.calibration.read_bytes() == before and '标定成功' not in output


def test_min_distance_is_independent_configurable_and_only_checks_xy(tmp_path, config, settings):
    custom = deepcopy(settings)
    custom['min_approach_calibration_distance_m'] = .02
    path = tmp_path/'settings.yaml'
    path.write_text(yaml.safe_dump(custom))
    assert load_settings(path, config)['min_approach_calibration_distance_m'] == .02
    data = set_reference({}, [0, 0, .2, .1, .2, .3], config['robot']['robot_ip'], config['tcp']['offset'])
    pose = [.005, 0, 123, .5, .7, -.8]
    data = save_group(data, 'A', pose)
    assert data['groups']['A']['tcp_pose'] == pose
    validate_group(data, 'A', settings, config)
    with pytest.raises(ValueError, match='最小要求 20.000 mm'):
        validate_group(data, 'A', custom, config)
    approach_direction(data['P_ref'], [.002, 0, -10, 0, 0, 0], min_distance_m=.001)
    with pytest.raises(ValueError, match='距离过近'):
        approach_direction(data['P_ref'], pose, min_distance_m=.02)


@pytest.mark.parametrize('bad', [0, -.001, float('nan'), float('inf'), True])
def test_minimum_distance_setting_must_be_positive(settings, bad):
    settings['min_approach_calibration_distance_m'] = bad
    with pytest.raises(ValueError, match='min_approach_calibration_distance_m'):
        validate_settings(settings)


@pytest.mark.parametrize('distance', [0, .000015])
def test_legacy_near_p0_is_viewable_but_denied_for_experiment_and_return(
        tmp_path, monkeypatch, config, settings, capsys, distance):
    data = demo_calibration(config)
    data['groups']['A']['tcp_pose'] = [.4-distance, 0, .9, .2, .3, .4]
    args = temporary_args(tmp_path, data)
    before = args.calibration.read_bytes()
    assert load_calibration(args.calibration) == data
    assert menu_run(monkeypatch, args, config, settings, ['3', '0']) == 0
    output = capsys.readouterr().out
    assert '距离过近，需要重标定' in output and 'P0-A' in output and 'P0-B' in output
    monkeypatch.setattr(ui, 'read_control_mode', lambda ip: ControlMode('remote', '模拟 true'))
    for operation in (entry.run_trial, entry.return_to_group):
        with pytest.raises(ValueError, match='距离过近'):
            operation(args, config, settings, data, 'A')
    assert args.calibration.read_bytes() == before


@pytest.mark.parametrize('contents', [None, 'not: [valid', 'schema_version: 1\nP_ref: null\ngroups: {}\n'])
@pytest.mark.parametrize('mode, expected', [('remote', '当前模式：Remote（远程控制）'),
                                         ('local', '当前状态：未处于Remote'), ('unknown', '状态：未知')])
def test_menu_seven_bypasses_calibration_and_motion_interfaces(tmp_path, monkeypatch, config, settings, capsys, contents, mode, expected):
    args = temporary_args(tmp_path, demo_calibration(config), execute=False)
    args.calibration.unlink()
    if contents is not None:
        args.calibration.write_text(contents)
    def forbidden(*a, **kw): raise AssertionError('menu 7 may only use Dashboard')
    monkeypatch.setattr(entry, 'load_calibration', forbidden)
    monkeypatch.setattr(entry, 'URRTDEController', forbidden)
    monkeypatch.setattr(entry, 'make_reader', forbidden)
    queries = []
    def query(ip):
        queries.append(ip)
        return ControlMode(mode, '模拟 Dashboard 查询超时' if mode == 'unknown' else '模拟回复')
    monkeypatch.setattr(ui, 'read_control_mode', query)
    assert menu_run(monkeypatch, args, config, settings, ['7', '0']) == 0
    output = capsys.readouterr().out
    assert queries == [config['robot']['robot_ip']] and expected in output
    if mode == 'local':
        assert '可能为Local模式或未启用Remote功能' in output and '当前模式：Local' not in output
    elif mode == 'unknown':
        assert '具体原因：模拟 Dashboard 查询超时' in output
        assert '机器人连接成功' not in output and '当前模式：Remote' not in output


def test_actual_mode_is_displayed_before_invalid_calibration_blocks_automatic_start(tmp_path, monkeypatch, config, settings, capsys):
    data = demo_calibration(config)
    data['groups']['A']['direction_xy'] = [0, 1]
    args = temporary_args(tmp_path, data)
    monkeypatch.setattr(ui, 'read_control_mode', lambda ip: ControlMode('local', '模拟 false'))
    # Invalid actual calibration data must still block both operations.
    for operation in (entry.run_trial, entry.return_to_group):
        with pytest.raises(ValueError, match='direction'):
            operation(args, config, settings, data, 'A')
        assert '当前状态：未处于Remote' in capsys.readouterr().out


def test_remote_must_be_rechecked_after_path_confirmation(tmp_path, monkeypatch, config, settings, capsys):
    from tests.test_single_point import bounded
    bounded(settings)
    data = reviewed_data(config, settings)
    args = temporary_args(tmp_path, data)
    devices = Devices(config, [.39, 0, .2, np.pi, 0, 0])
    modes = iter([ControlMode('remote', '模拟首次 true'), ControlMode('local', '模拟随后 false')])
    monkeypatch.setattr(ui, 'read_control_mode', lambda ip: next(modes))
    monkeypatch.setattr(entry, 'confirm_enter', lambda *a, **kw: True)
    monkeypatch.setattr(entry, 'read_tcp_offset_readonly', lambda ip: config['tcp']['offset'])
    result = entry.run_trial(args, config, settings, data, 'A', controller_factory=devices.controller,
                             reader_factory=lambda config: Sensor(), keyboard_factory=Keyboard)
    assert result['status'] == 'aborted' and '当前未确认 Remote 模式' in result['fault']
    assert devices.control_count == 0 and devices.control.calls == []
    output = capsys.readouterr().out
    assert '当前模式：Remote' in output and '当前状态：未处于Remote' in output


@pytest.mark.parametrize('reply, expected', [(b' TRUE \r\n', 'remote'), (b'False\n', 'local'),
                                          (b'true-but-not-exact\n', 'unknown'), (b'\xff\n', 'unknown')])
def test_dashboard_fragmented_response_and_boolean_parsing(reply, expected):
    stream = DashboardSocket([b'Connected: Universal Robots Dashboard Server\r\n', reply[:2], reply[2:]])
    result = read_control_mode('fake', connect=lambda *a, **kw: stream)
    assert result.mode == expected and stream.closed
    assert stream.sent == [b'is in remote control\n']


def test_dashboard_response_timeout_is_unknown_and_closes_socket():
    class TimedOut(DashboardSocket):
        def recv(self, count):
            if self.sent:
                raise TimeoutError('模拟响应超时')
            return super().recv(count)
    stream = TimedOut([b'Connected: Universal Robots Dashboard Server\n'])
    result = read_control_mode('fake', connect=lambda *a, **kw: stream)
    assert result.mode == 'unknown' and '模拟响应超时' in result.detail and stream.closed


@pytest.mark.parametrize('mode', ['--simulate', '--execute'])
def test_direct_group_cli_reports_near_distance_without_traceback(tmp_path, monkeypatch, config, capsys, mode):
    data = demo_calibration(config)
    data['groups']['A']['tcp_pose'] = [.400015, 0, .9, .2, .3, .4]
    args = temporary_args(tmp_path, data)
    before = args.calibration.read_bytes()
    monkeypatch.setattr(ui, 'read_control_mode', lambda ip: ControlMode('remote', '模拟 true'))
    assert entry.main([mode, '--group', 'A', '--calibration', str(args.calibration)]) == 1
    output = capsys.readouterr().out
    assert '操作失败；具体原因' in output and '实际 0.015 mm' in output
    assert 'Traceback' not in output and args.calibration.read_bytes() == before


def test_invalid_candidate_does_not_touch_file_or_create_backup(tmp_path, config, capsys):
    data = demo_calibration(config)
    args = temporary_args(tmp_path, data)
    before = args.calibration.read_bytes()
    candidate = deepcopy(data)
    candidate['P_ref'][0] += .1
    with pytest.raises(ValueError, match='reference'):
        entry.save_calibration_checked(args.calibration, candidate, 'P_ref')
    assert args.calibration.read_bytes() == before and not list(tmp_path.glob('*.backup-*'))
    assert '标定成功' not in capsys.readouterr().out
