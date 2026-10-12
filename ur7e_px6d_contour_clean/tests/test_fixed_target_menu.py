"""Menu actions reuse existing functions; every calibration write stays temporary."""
from argparse import Namespace
from copy import deepcopy
import shutil

import pytest

from app import fixed_target as fixed
from calibration.single_point import load_calibration
import run_fixed_target_tracking as entry
import run_single_point_contact as single


@pytest.fixture
def args(tmp_path):
    calibration = tmp_path/'points.yaml'
    shutil.copyfile(fixed.ROOT/'single_point_calibration.yaml', calibration)
    return Namespace(config=fixed.ROOT/'config.yaml', calibration=calibration, execute=False,
        dry_run=False, p0=None, scene=fixed.ROOT/'simulation/scene_fixed_target.yaml',
        duration=None, output=tmp_path/'runs', target_geometry=tmp_path/'target.yaml')


def run_menu(monkeypatch, args, answers):
    answers = iter(answers)
    monkeypatch.setattr('builtins.input', lambda prompt='': next(answers))
    return fixed.menu(args)


def test_no_arguments_show_chinese_menu_without_loading_or_connecting(monkeypatch, capsys):
    def forbidden(*a, **kw):
        raise AssertionError('startup must only show menu')
    monkeypatch.setattr(fixed, 'load_config', forbidden)
    monkeypatch.setattr(fixed, 'run_fixed', forbidden)
    monkeypatch.setattr('builtins.input', lambda _: '0')
    assert entry.main([]) == 0
    output = capsys.readouterr().out
    assert '========== 固定目标连续跟踪实验 ==========' in output
    for text in ('开始真机实验', '标定 P_ref', '新增 / 修改 / 删除 P0',
                 '查看已有标定', '标定正方体四角点', '检查标定与工作区域', '离线模拟'):
        assert text in output


def test_all_menu_actions_route_to_existing_implementations(args, monkeypatch):
    calls = []
    monkeypatch.setattr(fixed, 'run_fixed', lambda a: calls.append(('run', a.execute, a.p0)))
    monkeypatch.setattr(fixed, 'check_calibration', lambda a: calls.append(('check', a.p0)))
    monkeypatch.setattr(single, 'calibrate_point',
        lambda a, p, s, d, name, **kw: calls.append(('calibrate', name, kw.get('reference', False))))
    monkeypatch.setattr(single, 'show_calibration_table', lambda d, s: calls.append(('view',)))
    from calibration import target_geometry
    monkeypatch.setattr(target_geometry, 'teach_target', lambda p, path: calls.append(('target', path)))
    assert run_menu(monkeypatch, args,
        ['1', 'B', '2', '3', '1', 'D', '3', '2', 'A', '4', '5', '6', '7', '0']) == 0
    assert calls == [('run', True, 'B'), ('calibrate', 'P_ref', True),
        ('calibrate', 'D', False), ('calibrate', 'A', False), ('view',),
        ('target', args.target_geometry), ('check', None), ('run', False, None)]
    assert args.execute is False and args.p0 is None


@pytest.mark.parametrize('answers', [['1', '0'], ['3', '0'], ['3', '1', '0'],
    ['3', '2', '0'], ['3', '3', '0']])
def test_selection_cancel_returns_to_main_without_hardware(args, monkeypatch, answers):
    before = args.calibration.read_bytes()
    def forbidden(*a, **kw):
        raise AssertionError('cancelled selection must not execute')
    monkeypatch.setattr(fixed, 'run_fixed', forbidden)
    monkeypatch.setattr(single, 'calibrate_point', forbidden)
    assert run_menu(monkeypatch, args, answers+['0']) == 0
    assert args.calibration.read_bytes() == before


@pytest.mark.parametrize('choice', ['1', '2', '5', '7'])
def test_cancel_inside_action_returns_to_main(args, monkeypatch, choice, capsys):
    def cancelled(*a, **kw):
        raise KeyboardInterrupt('synthetic cancellation')
    monkeypatch.setattr(fixed, 'run_fixed', cancelled)
    monkeypatch.setattr(single, 'calibrate_point', cancelled)
    from calibration import target_geometry
    monkeypatch.setattr(target_geometry, 'teach_target', cancelled)
    assert run_menu(monkeypatch, args, [choice]+(['A'] if choice == '1' else [])+['0']) == 0
    assert capsys.readouterr().out.count('========== 固定目标连续跟踪实验 ==========') == 2


def test_view_and_check_are_readonly(args, monkeypatch, capsys):
    before = args.calibration.read_bytes()
    assert run_menu(monkeypatch, args, ['4', '6', '0']) == 0
    assert 'P0-A' in capsys.readouterr().out
    assert args.calibration.read_bytes() == before
    assert not args.output.exists()


@pytest.mark.parametrize('confirm', [False, True])
def test_delete_uses_existing_confirmation_and_checked_save(args, monkeypatch, confirm):
    before = args.calibration.read_bytes()
    monkeypatch.setattr(single, 'confirm_enter', lambda _: confirm)
    assert run_menu(monkeypatch, args, ['3', '3', 'A', '0']) == 0
    assert ('A' in load_calibration(args.calibration)['groups']) is (not confirm)
    if not confirm:
        assert args.calibration.read_bytes() == before


@pytest.mark.parametrize('action,name', [('1', 'D'), ('2', 'A')])
def test_add_modify_use_real_single_point_calibration_with_fake_capture(args, config, monkeypatch, action, name):
    pose = [.58, .35, -.01, *load_calibration(args.calibration)['groups']['A']['tcp_pose'][3:]]
    monkeypatch.setattr(single, 'capture_pose', lambda *a, **kw: (pose, config['tcp']['offset']))
    assert run_menu(monkeypatch, args, ['3', action, name, '0']) == 0
    assert load_calibration(args.calibration)['groups'][name]['tcp_pose'] == pose


def test_reference_uses_real_single_point_recalibration_with_fake_capture(args, config, monkeypatch):
    before = load_calibration(args.calibration)
    pose = deepcopy(before['reference_tcp_pose'])
    pose[0] += .001
    monkeypatch.setattr(single, 'capture_pose', lambda *a, **kw: (pose, config['tcp']['offset']))
    monkeypatch.setattr(single, 'reference_recalibration_choice', lambda _: 'keep')
    monkeypatch.setattr(single, 'confirm_enter', lambda _: True)
    assert run_menu(monkeypatch, args, ['2', '0']) == 0
    after = load_calibration(args.calibration)
    assert after['P_ref'] == pose[:3]
    for name in before['groups']:
        assert after['groups'][name]['tcp_pose'] == before['groups'][name]['tcp_pose']


@pytest.mark.parametrize('argv,execute', [(['--execute', '--p0', 'A'], True),
    (['--dry-run', '--duration', '1'], False), (['--config', str(fixed.ROOT/'config.yaml')], False)])
def test_existing_cli_dispatch_preserved(monkeypatch, argv, execute):
    seen = []
    monkeypatch.setattr(entry, 'run_fixed', lambda a: seen.append(a) or 0)
    monkeypatch.setattr(entry, 'menu', lambda _: pytest.fail('explicit CLI must retain old dispatch'))
    assert entry.main(argv) == 0
    assert len(seen) == 1 and seen[0].execute is execute
