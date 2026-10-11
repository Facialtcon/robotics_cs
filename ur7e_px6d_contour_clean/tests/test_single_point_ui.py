"""Only fake sockets/devices and temporary calibration files; no real robot."""
from argparse import Namespace
from copy import deepcopy
import json

import numpy as np
import pytest

from app import single_point_ui as ui
from app.single_point_config import load_settings
from calibration.single_point import load_calibration, save_group, set_reference, write_calibration
from robot.control_mode import ControlMode, read_control_mode
from robot.rtde_controller import RobotError
import run_single_point_contact as entry
from simulation.single_point import demo_calibration
from tests.doubles import Devices, Keyboard


@pytest.fixture
def settings(config):
    values = load_settings(entry.ROOT/'single_point_experiment.yaml', config)
    values.update(workspace_enabled=False, workspace_limits=None, workspace_xy_polygon=None)
    return values


def menu_run(monkeypatch, args, config, settings, answers):
    answers = iter(answers)
    monkeypatch.setattr('builtins.input', lambda prompt='': next(answers))
    return entry.menu(args, config, settings)


def temporary_args(tmp_path, data, *, execute=True):
    path = tmp_path/'calibration.yaml'
    write_calibration(path, data)
    return Namespace(calibration=path, execute=execute, speed=None, output=tmp_path, config=entry.ROOT/'config.yaml')


def test_menu_three_shows_multiple_groups_without_raw_json_and_does_not_write(tmp_path, monkeypatch, config, settings, capsys):
    data = demo_calibration(config)
    data = save_group(data, 'A', [.390123456789012, 0, .8, .2, .3, .4])
    data = save_group(data, 'B', [.4, -.01, -.6, 0, 0, 0])
    data['groups']['A']['path_review'] = dict(note='legacy note', max_distance_m=.1, reviewed_at='old')
    data['groups']['B']['path_review'] = {}
    args = temporary_args(tmp_path, data, execute=False)
    before = args.calibration.read_bytes()
    assert menu_run(monkeypatch, args, config, settings, ['3', '0']) == 0
    output = capsys.readouterr().out
    assert '中文标定信息表' in output and '已标定 P0 数量：3' in output
    assert 'X=400.000 mm' in output and 'Z=200.000 mm' in output
    assert 'P0-A' in output and 'P0-B' in output and 'P0-C' in output
    assert '390.123' in output and '90.00°' in output and '180.00°' in output
    assert '标定状态' in output and '有效' in output
    assert all(value not in output for value in ('路径核验', '未完成', '保存状态', 'legacy note'))
    assert 'tcp_pose' not in output and 'schema_version' not in output and '{' not in output
    assert '0.390123456789012' not in output
    assert args.calibration.read_bytes() == before and load_calibration(args.calibration) == data


def test_empty_calibration_table_is_clear(settings, capsys):
    ui.show_calibration_table(dict(P_ref=None, groups={}), settings)
    output = capsys.readouterr().out
    assert '未标定' in output and '数量：0' in output and '暂无已保存 P0' in output


def test_explicit_json_debug_is_offline_and_preserves_full_precision(tmp_path, config, capsys):
    data = demo_calibration(config)
    data = save_group(data, 'A', [.390123456789012, 0, .2, np.pi, 0, 0])
    args = temporary_args(tmp_path, data)
    before = args.calibration.read_bytes()
    assert entry.main(['--show-calibration-json', '--calibration', str(args.calibration), '--execute']) == 0
    assert json.loads(capsys.readouterr().out) == data
    assert args.calibration.read_bytes() == before


@pytest.mark.parametrize('remote', [False, None])
def test_readonly_capture_in_local_or_unknown_mode_never_creates_control(monkeypatch, config, capsys, remote):
    raw = [.390123456789012, -.009876543210987, .712345678901234, .1, .2, .3]
    devices = Devices(config, raw)
    mode = ControlMode('local', '模拟回复 false') if remote is False else ControlMode('unknown', '模拟查询超时')
    monkeypatch.setattr(ui, 'read_control_mode', lambda ip: mode)
    monkeypatch.setattr(entry, 'read_tcp_offset_readonly', lambda ip: config['tcp']['offset'])
    prompts = []
    def confirm(prompt):
        prompts.append(prompt)
        assert devices.receive_count == devices.control_count == 0
        output = capsys.readouterr().out
        assert 'Local' in output
        assert ('未处于Remote' if remote is False else '状态：未知') in output
        return True
    pose, tcp = entry.capture_pose(config, point_name='P0-A', confirm=confirm, controller_factory=devices.controller)
    output = capsys.readouterr().out
    assert all(f'[{i}/9]' in output for i in range(3, 7))
    assert '390.123 mm' in output and '-9.877 mm' in output and '712.346 mm' in output
    assert '0.390123456789012' not in output
    np.testing.assert_array_equal(pose, raw)
    assert tcp == config['tcp']['offset']
    assert devices.receive_count == 1 and devices.control_count == 0 and devices.control.calls == []
    assert not devices.receive.connected


@pytest.mark.parametrize('choice, name', [('1', 'P_ref'), ('2', 'P0-D')])
def test_calibration_success_saves_reads_back_and_keeps_existing_groups(tmp_path, monkeypatch, config, settings, capsys, choice, name):
    data = dict(schema_version=1, P_ref=None, groups={}) if choice == '1' else demo_calibration(config)
    args = temporary_args(tmp_path, data)
    original = deepcopy(data)
    raw = [.4, .01, .723456789012345, .1, -.2, .3]
    monkeypatch.setattr(entry, 'capture_pose', lambda project, **kw: (np.array(raw), config['tcp']['offset']))
    calls = []
    write, read = entry.write_calibration, entry.load_calibration
    def writing(path, updated):
        calls.append('write')
        return write(path, updated)
    def reading(path):
        calls.append('read')
        return read(path)
    monkeypatch.setattr(entry, 'write_calibration', writing)
    monkeypatch.setattr(entry, 'load_calibration', reading)
    answers = ['1', '0'] if choice == '1' else ['2', 'D', '0']
    assert menu_run(monkeypatch, args, config, settings, answers) == 0
    output = capsys.readouterr().out
    assert f'标定成功：{name}' in output and '文件回读校验通过' in output
    assert '[7/9]' in output and '[8/9]' in output and '[9/9]' in output
    assert str(args.calibration.resolve()) in output
    assert calls == ['read', 'write', 'read']
    loaded = read(args.calibration)
    if choice == '1':
        assert loaded['reference_tcp_pose'] == raw
    else:
        assert loaded['P_ref'] == original['P_ref']
        assert loaded['reference_tcp_pose'] == original['reference_tcp_pose']
        assert all(loaded['groups'][key] == value for key, value in original['groups'].items())
        assert loaded['groups']['D']['tcp_pose'] == raw
        assert 'path_review' not in loaded['groups']['D']
        assert '路径核验' not in output


@pytest.mark.parametrize('choice', ['1', '2'])
@pytest.mark.parametrize('error', [RobotError('RTDE 连接被拒绝'), RobotError('TCP 偏置不一致'),
                                  OSError('保存失败：磁盘只读'), KeyboardInterrupt('人工取消'), EOFError('确认终端关闭')])
def test_calibration_failure_or_cancel_reports_reason_and_returns_menu(tmp_path, monkeypatch, config, settings, capsys, choice, error):
    data = dict(schema_version=1, P_ref=None, groups={}) if choice == '1' else demo_calibration(config)
    args = temporary_args(tmp_path, data)
    before = args.calibration.read_bytes()
    def failing(*args, **kw): raise error
    monkeypatch.setattr(entry, 'capture_pose', failing)
    answers = [choice, '0'] if choice == '1' else [choice, 'D', '0']
    assert menu_run(monkeypatch, args, config, settings, answers) == 0
    output = capsys.readouterr().out
    assert ('标定取消' if isinstance(error, (KeyboardInterrupt, EOFError)) else '标定失败') in output
    assert str(error) in output and '标定成功' not in output and '本次操作停止' not in output
    assert output.count('========== 单点接触实验 ==========') == 2
    assert args.calibration.read_bytes() == before


def test_actual_write_failure_and_readback_mismatch_are_not_success(tmp_path, monkeypatch, config, capsys):
    original = demo_calibration(config)
    args = temporary_args(tmp_path, original)
    before = args.calibration.read_bytes()
    updated = save_group(original, 'D', [.4, .01, .2, np.pi, 0, 0])
    write = entry.write_calibration
    def failed(*a, **kw): raise OSError('磁盘空间不足')
    monkeypatch.setattr(entry, 'write_calibration', failed)
    with pytest.raises(OSError, match='磁盘空间不足'):
        entry.save_calibration_checked(args.calibration, updated, 'P0-D')
    assert args.calibration.read_bytes() == before
    monkeypatch.setattr(entry, 'write_calibration', write)
    monkeypatch.setattr(entry, 'load_calibration', lambda path: original)
    with pytest.raises(ValueError, match='回读校验不一致'):
        entry.save_calibration_checked(args.calibration, updated, 'P0-D')
    assert args.calibration.read_bytes() == before
    assert '标定成功' not in capsys.readouterr().out


def test_reference_recalibration_never_overwrites_existing_groups(tmp_path, monkeypatch, config, settings, capsys):
    data = demo_calibration(config)
    args = temporary_args(tmp_path, data)
    before = args.calibration.read_bytes()
    assert menu_run(monkeypatch, args, config, settings, ['1', '0', '0']) == 0
    output = capsys.readouterr().out
    assert '重新标定 P_ref' in output and '标定取消：P_ref' in output
    assert args.calibration.read_bytes() == before


def test_readonly_capture_tcp_mismatch_reports_specific_reason_and_closes_receive(monkeypatch, config, capsys):
    devices = Devices(config, [.4, .01, .2, .1, .2, .3])
    monkeypatch.setattr(ui, 'read_control_mode', lambda ip: ControlMode('local', '模拟回复 false'))
    monkeypatch.setattr(entry, 'read_tcp_offset_readonly', lambda ip: [0.]*6)
    with pytest.raises(RobotError, match='TCP 偏置与工程配置不一致'):
        entry.capture_pose(config, confirm=lambda prompt: True, controller_factory=devices.controller)
    assert devices.control_count == 0 and devices.control.calls == []
    assert not devices.receive.connected
    assert '[6/9]' not in capsys.readouterr().out


def test_cancel_before_read_does_not_connect(config, monkeypatch):
    devices = Devices(config, [.4, .01, .2, .1, .2, .3])
    monkeypatch.setattr(ui, 'read_control_mode', lambda ip: ControlMode('local', '模拟回复 false'))
    with pytest.raises(KeyboardInterrupt, match='取消只读标定'):
        entry.capture_pose(config, confirm=lambda prompt: False, controller_factory=devices.controller)
    assert devices.receive_count == devices.control_count == 0 and devices.control.calls == []


def test_p0_calibration_does_not_request_text_or_extra_confirmation(tmp_path, monkeypatch, config, settings, capsys):
    args = temporary_args(tmp_path, demo_calibration(config))
    before = load_calibration(args.calibration)
    monkeypatch.setattr(entry, 'capture_pose', lambda project, **kw: (np.array([.4, .01, .8, .1, .2, .3]), config['tcp']['offset']))
    def forbidden(*a, **kw): raise AssertionError('P0 capture must not add another confirmation')
    monkeypatch.setattr(entry, 'confirm_enter', forbidden)
    assert menu_run(monkeypatch, args, config, settings, ['2', 'D', '3', '0']) == 0
    output = capsys.readouterr().out
    assert '标定成功：P0-D' in output and '已标定 P0 数量：4' in output
    assert '路径核验' not in output and '核验依据' not in output
    after = load_calibration(args.calibration)
    assert after['groups']['D']['tcp_pose'] == [.4, .01, .8, .1, .2, .3]
    assert all(after['groups'][name] == group for name, group in before['groups'].items())


class DashboardSocket:
    def __init__(self, chunks): self.chunks = iter(chunks); self.sent = []; self.closed = False
    def __enter__(self): return self
    def __exit__(self, *args): self.closed = True
    def recv(self, count): return next(self.chunks, b'')
    def sendall(self, value): self.sent.append(value)
    def settimeout(self, timeout): assert 0 < timeout <= 2


@pytest.mark.parametrize('reply, expected', [(b'true\n', 'remote'), (b'false\r\n', 'local'),
    (b'Command not found\n', 'unknown'), (b'not true\n', 'unknown'), (b'', 'unknown')])
def test_dashboard_query_is_readonly_and_only_exact_boolean_confirms_mode(reply, expected):
    stream = DashboardSocket([b'Connected: Universal Robots ', b'Dashboard Server\r\n', reply])
    endpoints = []
    def connect(address, timeout): endpoints.append(address); return stream
    result = read_control_mode('fake-host', connect=connect)
    assert result.mode == expected and result.detail
    assert endpoints == [('fake-host', 29999)]
    assert stream.sent == [b'is in remote control\n'] and stream.closed


@pytest.mark.parametrize('error', [TimeoutError('timeout'), ConnectionRefusedError('refused')])
def test_dashboard_connection_failure_is_explicitly_unknown(error):
    def connect(*a, **kw): raise error
    result = read_control_mode('fake', connect=connect)
    assert result.mode == 'unknown' and str(error) in result.detail


def test_dashboard_invalid_greeting_does_not_send_commands():
    stream = DashboardSocket([b'Other service\n'])
    result = read_control_mode('fake', connect=lambda *a, **kw: stream)
    assert result.mode == 'unknown' and stream.sent == [] and stream.closed


@pytest.mark.parametrize('mode', ['local', 'unknown'])
def test_automatic_preparation_reports_remote_requirement_and_blocks_unconfirmed_mode(tmp_path, monkeypatch, config, settings, capsys, mode):
    from tests.test_single_point import bounded
    bounded(settings)
    data = demo_calibration(config)
    args = temporary_args(tmp_path, data)
    devices = Devices(config, [.39, 0, .2, np.pi, 0, 0])
    monkeypatch.setattr(entry, 'confirm_enter', lambda *a, **kw: True)
    monkeypatch.setattr(entry, 'read_tcp_offset_readonly', lambda ip: config['tcp']['offset'])
    monkeypatch.setattr(ui, 'read_control_mode', lambda ip: ControlMode(mode, '模拟查询失败' if mode == 'unknown' else '模拟回复 false'))
    from tests.doubles import Sensor
    result = entry.run_trial(args, config, settings, data, 'A', controller_factory=devices.controller,
                             reader_factory=lambda config: Sensor(), keyboard_factory=Keyboard)
    output = capsys.readouterr().out
    assert '请在示教器切换 Remote' in output and '未确认 Remote' in output
    assert ('未知' if mode == 'unknown' else 'Local') in output
    assert result['status'] == 'aborted' and '未确认 Remote' in result['fault']
    assert devices.control_count == 0 and devices.control.calls == []


@pytest.mark.parametrize('mode', ['local', 'unknown', 'remote'])
def test_return_to_group_checks_mode_after_path_confirmation_before_control(tmp_path, monkeypatch, config, settings, capsys, mode):
    data = demo_calibration(config)
    args = temporary_args(tmp_path, data)
    before = args.calibration.read_bytes()
    devices = Devices(config, [.38, -.02, .17, .2, .3, .4])
    monkeypatch.setattr(entry, 'URRTDEController', devices.controller)
    from tests.doubles import Sensor
    monkeypatch.setattr(entry, 'make_reader', lambda config: Sensor())
    class NoTextKeyboard(Keyboard):
        def read_text(self, prompt): raise AssertionError('return must not ask for written evidence')
    monkeypatch.setattr(entry, 'OperatorKeyboard', NoTextKeyboard)
    order = []
    def confirm(prompt, **kw):
        assert devices.control_count == 0
        order.append('path confirmation' if '完整路径' in prompt else 'read confirmation')
        return True
    def query(ip):
        assert devices.control_count == 0 and devices.control.calls == []
        if not order:
            order.append('initial mode check')
            return ControlMode(mode, '模拟查询结果')
        assert order == ['initial mode check', 'read confirmation', 'path confirmation']
        order.append('read-only mode check')
        return ControlMode(mode, '模拟查询结果')
    monkeypatch.setattr(entry, 'confirm_enter', confirm)
    monkeypatch.setattr(ui, 'read_control_mode', query)
    if mode == 'remote':
        entry.return_to_group(args, config, settings, data, 'A')
        assert devices.control_count == 1 and any(call[0] == 'moveL' for call in devices.control.calls)
        plan = json.loads(next((tmp_path/'real'/'single_point').glob('run_*'))
                          .joinpath('return_plan.json').read_text())
        assert plan['path_confirmed'] is True and 'site_review' not in plan
    else:
        with pytest.raises(RobotError, match='未确认 Remote'):
            entry.return_to_group(args, config, settings, data, 'A')
        assert devices.control_count == 0 and devices.control.calls == []
    output = capsys.readouterr().out
    assert '请在示教器切换 Remote' in output and '机器人控制模式' in output
    assert '核验依据' not in output
    assert args.calibration.read_bytes() == before


@pytest.mark.parametrize('failure', ['read_cancel', 'path_cancel', 'path_drift', 'workspace',
                                   'obstacle', 'force_limit', 'sensor_failure'])
def test_return_keeps_actual_guards_and_enter_confirmation_without_text(
        tmp_path, monkeypatch, config, settings, failure, capsys):
    from core.models import Wrench
    from safety.safe_return import ReturnAborted
    from sensor.px6d_reader import PX6DTimeout
    from tests.doubles import Sensor
    data = demo_calibration(config)
    data['groups']['A']['path_review'] = {}
    args = temporary_args(tmp_path, data)
    before = args.calibration.read_bytes()
    settings.update(workspace_enabled=True, workspace_limits=dict(
        x_min=.3, x_max=.5, y_min=-.1, y_max=.1, z_min=.1, z_max=.3))
    initial = [.6 if failure == 'workspace' else .38, -.02, .17, .2, .3, .4]
    devices = Devices(config, initial)
    if failure == 'obstacle':
        settings['forbidden_boxes'] = [dict(x_min=.37, x_max=.39, y_min=-.03,
                                          y_max=-.01, z_min=.18, z_max=.22)]
    class NoTextKeyboard(Keyboard):
        def read_text(self, prompt): raise AssertionError('no written evidence is allowed')
    class GuardSensor(Sensor):
        def read_wrench(self):
            if failure == 'sensor_failure':
                raise PX6DTimeout('fake sensor stream failure')
            return Wrench(10000. if failure == 'force_limit' else 0., 0., 0., 0., 0., 0.)
    confirmations = []
    def confirm(prompt, **kw):
        assert devices.control_count == 0 and not devices.control.calls
        confirmations.append(prompt)
        if '完整路径' in prompt:
            assert '完整运动路径' in capsys.readouterr().out
            if failure == 'path_drift':
                devices.receive.pose[0] += .01
            return failure != 'path_cancel'
        return failure != 'read_cancel'
    monkeypatch.setattr(entry, 'confirm_enter', confirm)
    monkeypatch.setattr(entry, 'URRTDEController', devices.controller)
    monkeypatch.setattr(entry, 'make_reader', lambda cfg: GuardSensor())
    monkeypatch.setattr(entry, 'OperatorKeyboard', NoTextKeyboard)
    monkeypatch.setattr(ui, 'read_control_mode', lambda ip: ControlMode('remote', 'fake true'))
    if failure.endswith('cancel'):
        entry.return_to_group(args, config, settings, data, 'A')
    else:
        expected = dict(path_drift='位姿已变动', workspace='workspace', obstacle='intersects',
                        force_limit='force.*(limit|threshold)', sensor_failure='sensor stream failure')[failure]
        with pytest.raises((RobotError, ValueError, ReturnAborted, PX6DTimeout), match=expected):
            entry.return_to_group(args, config, settings, data, 'A')
    assert devices.control_count == 0 and not devices.control.calls
    assert len(confirmations) == (1 if failure in ('read_cancel', 'workspace', 'obstacle') else 2)
    assert args.calibration.read_bytes() == before


def test_recapturing_p0_preserves_legacy_metadata_and_clears_invalid_marker(config):
    data = demo_calibration(config)
    data['groups']['A'].update(path_review=dict(note='historical record'), requires_recalibration=True)
    before = deepcopy(data)
    updated = save_group(data, 'A', [.38, .01, .9, .3, -.5, .7])
    assert updated['groups']['A']['path_review'] == data['groups']['A']['path_review']
    assert 'requires_recalibration' not in updated['groups']['A']
    assert data == before


def test_normal_group_and_path_coordinates_are_mm_without_raw_precision(config, settings, capsys):
    data = demo_calibration(config)
    before = deepcopy(data)
    entry.show_group(data, 'A', settings, settings['speed_default_mps'], config)
    current = np.array([.381234567890, -.02, .17, .2, .3, .4])
    target = np.array([.39, 0, .2, np.pi, 0, 0])
    _, segments = entry.plan_return(config, settings, current, target)
    ui.show_motion_path(current, target, segments)
    ui.show_trial_result(dict(status='contact', actual_stop_pose=target.tolist()))
    output = capsys.readouterr().out
    assert '381.235 mm' in output and '390.000 mm' in output and '200.000 mm' in output
    assert '0.38123456789' not in output and 'e-' not in output and 'tcp_pose' not in output
    assert data == before
