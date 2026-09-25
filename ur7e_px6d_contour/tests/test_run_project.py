"""Menu contracts. Hardware commands are intercepted before process creation."""
from pathlib import Path
from types import SimpleNamespace
import builtins
import csv
import hashlib
import json
import sys

import pytest
import yaml
import run_project as menu

ROOT = Path(__file__).resolve().parents[1]
LEGACY = {
    '1': ['-m', 'pytest', '-q'], '2': ['tools/run_mock_visualized.py'],
    '3': ['tools/check_px6d.py', '--samples', '500'], '4': ['tools/check_rtde.py'],
    '5': ['main.py', '--sensor', 'real'], '6': ['run_simulation.py'],
    '7': ['run_calibration.py'], '8': ['tools/check_scan_calibration.py'],
    '9': ['tools/read_active_tcp.py'], '10': ['save_reset_pose.py'],
    '11': ['return_to_reset.py'], '12': ['main.py', '--sensor', 'real', '--execute'],
}


@pytest.fixture(autouse=True)
def forbid_hardware_and_unexpected_children(monkeypatch):
    import socket
    import serial
    import rtde_control
    import rtde_receive
    def forbidden(*args, **kwargs):
        pytest.fail('Hardware or unplanned subprocess forbidden in menu tests')
    monkeypatch.setattr(socket.socket, 'connect', forbidden)
    monkeypatch.setattr(serial, 'Serial', forbidden)
    monkeypatch.setattr(rtde_control, 'RTDEControlInterface', forbidden)
    monkeypatch.setattr(rtde_receive, 'RTDEReceiveInterface', forbidden)
    monkeypatch.setattr(menu, 'subprocess', SimpleNamespace(Popen=forbidden))


def answers(monkeypatch, *values):
    iterator = iter(values)
    monkeypatch.setattr(builtins, 'input', lambda *args: next(iterator))


@pytest.fixture
def launches(monkeypatch):
    seen = []
    monkeypatch.setattr(menu, 'check_dependency', lambda _: None)
    def launch(command, **kwargs):
        seen.append((command, kwargs))
        return SimpleNamespace(wait=lambda: 0)
    monkeypatch.setattr(menu.subprocess, 'Popen', launch)
    return seen


@pytest.mark.parametrize('step', LEGACY)
def test_legacy_step_cli_mapping(step, monkeypatch, launches):
    answers(monkeypatch, menu.MOTION_CONFIRMATIONS.get(step, ''))
    monkeypatch.setattr(sys, 'argv', ['run_project.py', '--step', step])
    monkeypatch.setenv('PYTHONPATH', '/fake/ros')
    assert menu.main() == 0
    command, options = launches[0]
    # User explicitly requested automatic YAML save for menu 9 after unification.
    assert command == [sys.executable, *LEGACY[step], *(['--write-config'] if step == '9' else [])]
    assert options['cwd'] == ROOT
    assert 'PYTHONPATH' not in options['env']
    assert options['env']['PYTEST_DISABLE_PLUGIN_AUTOLOAD'] == '1'
    assert set(options) == {'cwd', 'env'}  # terminal inherited; no shell, pipe or input injection


@pytest.mark.parametrize('step', ['11', '12', '18'])
@pytest.mark.parametrize('word', ['', 'START', ':q', 'wrong'])
def test_motion_cancel_never_launches(step, word, monkeypatch, launches, capsys):
    answers(monkeypatch, word)
    assert menu.run_step(step) == 130
    assert not launches
    assert '成功' not in capsys.readouterr().out


@pytest.mark.parametrize('step,tail', [
    ('13', ['calibrate_workspace.py']),
    ('17', ['run_continuous_tracking.py', '--check-calibration']),
    ('18', ['run_continuous_tracking.py', '--execute']),
])
def test_new_fixed_commands(step, tail, monkeypatch, launches):
    answers(monkeypatch, menu.MOTION_CONFIRMATIONS.get(step, ''))
    assert menu.run_step(step) == 0
    assert launches[0][0] == [sys.executable, *tail]
    assert launches[0][1]['cwd'] == ROOT


@pytest.mark.parametrize('step', ['15', '16'])
def test_scene_spaces_duration_and_manual_preview(step, tmp_path, monkeypatch, launches):
    scene = tmp_path / 'scene with spaces.yaml'
    from simulation.simulator import load_simulation_config
    scene.write_text(yaml.safe_dump(load_simulation_config(ROOT / 'simulation/scene_continuous.yaml')))
    answers(monkeypatch, '/no/such/scene', str(scene), 'nan', '-1', 'inf', 'bad', '0.05')
    assert menu.run_step(step) == 0
    command = launches[0][0]
    assert command[:3] == [sys.executable, 'run_continuous_tracking.py', '--preview' if step == '15' else '--dry-run']
    assert command[command.index('--scene')+1] == str(scene)
    if step == '15':
        assert '--duration' not in command
    else:
        assert command[-2:] == ['--duration', '0.05']


@pytest.mark.parametrize('step', ['14', '15', '16', '19', '20'])
def test_dynamic_cancel(step, monkeypatch, launches, tmp_path):
    monkeypatch.setattr(menu, 'ROOT', tmp_path)
    answers(monkeypatch, ':q')
    assert menu.run_step(step) == 130
    assert not launches


def test_missing_dependency_and_invalid_input(monkeypatch, launches, capsys):
    assert menu.run_step('bad') == 2
    monkeypatch.setattr(menu, 'check_dependency', lambda _: '缺少 numpy')
    assert menu.run_step('16') == 2
    assert not launches
    assert '缺少 numpy' in capsys.readouterr().out


def test_dependency_probe_is_offline(monkeypatch):
    monkeypatch.setattr(menu.importlib.util, 'find_spec', lambda _: None)
    assert 'ur-rtde' in menu.check_dependency('18')
    assert 'matplotlib' in menu.check_dependency('15')


def test_menu_start_and_exit_no_device_or_child(monkeypatch, capsys):
    answers(monkeypatch, '0')
    monkeypatch.setattr(sys, 'argv', ['run_project.py'])
    assert menu.main() == 0
    text = capsys.readouterr().out
    for step, (name, _) in menu.STEPS.items():
        assert f'{step:>2}. {name}' in text
    for step in ('2', '5', '6', '12'):
        assert '原离散策略' in menu.STEPS[step][0]


def test_failed_child_not_success_or_automatic_next(monkeypatch, launches, capsys):
    calls = []
    monkeypatch.setattr(menu, 'run_child', lambda command: calls.append(command) or 7)
    assert menu.run_step('4') == 7
    assert len(calls) == 1
    assert '失败或中断' in capsys.readouterr().out


def test_interrupt_waits_until_child_cleanup(monkeypatch, launches, capsys):
    events = []
    def wait():
        events.append('wait')
        if len(events) < 3:
            raise KeyboardInterrupt
        events.append('cleanup_finished')
        return 0
    monkeypatch.setattr(menu.subprocess, 'Popen', lambda *a, **k: SimpleNamespace(wait=wait))
    assert menu.run_step('4') == 130
    assert events == ['wait', 'wait', 'wait', 'cleanup_finished']
    assert '退出码 130' in capsys.readouterr().out


def test_launch_error_is_reported(monkeypatch, launches):
    def fail(*a, **k):
        raise OSError('injected spawn failure')
    monkeypatch.setattr(menu.subprocess, 'Popen', fail)
    assert menu.run_step('4') == 2


@pytest.mark.parametrize('where', ['confirm', 'pause'])
def test_eof_and_interrupt_are_handled(where, monkeypatch, launches):
    if where == 'confirm':
        monkeypatch.setattr(builtins, 'input', lambda *_: (_ for _ in ()).throw(EOFError()))
        assert menu.run_step('18') == 130
        assert not launches
    else:
        inputs = iter(['4'])
        def get(*_):
            try:
                return next(inputs)
            except StopIteration:
                raise KeyboardInterrupt
        monkeypatch.setattr(builtins, 'input', get)
        monkeypatch.setattr(sys, 'argv', ['run_project.py'])
        assert menu.main() == 130
        assert len(launches) == 1


def make_run(path, strategy='continuous', mode='simulation'):
    path.mkdir(parents=True)
    (path / 'summary.json').write_text(json.dumps({'mode': mode}))
    (path / 'config_snapshot.yaml').write_text('continuous_provenance: {}\n' if strategy == 'continuous' else '{}')
    (path / 'samples.csv').write_text('force_reference\n1\n' if strategy == 'continuous' else 'commanded_speed_mps\n0\n')
    (path / 'policy_waypoints.csv').write_text('event_type\n')
    if strategy == 'discrete':
        (path / 'boundary_points.csv').write_text('x,y\n')
        (path / 'boundary_recovery_rays.csv').write_text('x,y\n')
    return path


@pytest.mark.parametrize('strategy,script', [('continuous', 'tools/visualize_continuous_run.py'), ('discrete', 'tools/visualize_run.py')])
def test_replay_selection_and_no_video(strategy, script, tmp_path, monkeypatch, launches, capsys):
    path = make_run(tmp_path / 'run with spaces', strategy, 'real')
    answers(monkeypatch, str(path))
    assert menu.run_step('19') == 0
    command = launches[0][0]
    assert command[1] == script and str(path) in command
    assert '--format' not in command or command[-2:] == ['--format', 'none']
    assert '真机' in capsys.readouterr().out


def test_type_unknown_not_inferred_from_path_or_default_config(tmp_path):
    path = make_run(tmp_path / 'real_latest', 'discrete', 'unrecorded')
    assert menu.describe_run(path) == ('原离散', '未知')
    (path / 'samples.csv').unlink()
    (path / 'config_snapshot.yaml').write_text('continuous_tracking: {max_runtime_sec: 120}\n')
    assert menu.describe_run(path) == ('未知', '未知')


@pytest.mark.parametrize('fault', ['empty', 'missing', 'unknown', 'metadata'])
def test_invalid_replay_not_launched(fault, tmp_path, monkeypatch, launches, capsys):
    path = make_run(tmp_path / 'log')
    if fault == 'empty':
        (path / 'samples.csv').write_text('force_reference\n')
    elif fault == 'missing':
        (path / 'policy_waypoints.csv').unlink()
    elif fault == 'unknown':
        (path / 'config_snapshot.yaml').write_text('{}')
    else:
        (path / 'summary.json').write_text('{broken')
    answers(monkeypatch, str(path))
    assert menu.run_step('19') == 2
    assert not launches
    assert '不能完成' in capsys.readouterr().out


def test_empty_log_listing_and_path_validation(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(menu, 'ROOT', tmp_path)
    answers(monkeypatch, '', '88', '/does/not/exist', str(tmp_path))
    assert menu.choose_run() == tmp_path
    assert '没有运行记录' in capsys.readouterr().out


def test_workspace_check_and_projection(tmp_path, monkeypatch, launches):
    calibration = tmp_path / 'box corners.yaml'
    calibration.write_bytes((ROOT / 'workspace/config/workspace_calibration.yaml').read_bytes())
    answers(monkeypatch, str(calibration))
    assert menu.run_step('14') == 0
    assert launches[-1][0] == [sys.executable, 'tools/check_workspace_calibration.py', '--calibration', str(calibration)]
    path = make_run(tmp_path / 'run')
    answers(monkeypatch, str(path), str(calibration))
    assert menu.run_step('20') == 0
    command = launches[-1][0]
    assert command[:2] == [sys.executable, 'visualize_workspace.py']
    assert command[command.index('--calibration')+1] == str(calibration)
    assert '--overwrite' not in command
    assert Path(command[-1]).parent == path


def test_offline_menu_runs_real_adapters_without_devices(tmp_path, monkeypatch):
    """Bounded preview, headless, check CLI; guards installed before construction."""
    import run_continuous_tracking as runner
    from simulation.continuous_preview import ContinuousPreview
    from matplotlib.backends.registry import backend_registry
    import matplotlib.pyplot as plt
    def forbidden(*a, **k):
        pytest.fail('Device construction forbidden')
    monkeypatch.setattr(runner.URRTDEController, '__init__', forbidden)
    monkeypatch.setattr(runner.PX6DReader, '__init__', forbidden)
    monkeypatch.setattr(runner.URRTDEController, 'verified_watchdog_contract', staticmethod(lambda: {'offline_double': True}))
    original_resolve = backend_registry.resolve_backend
    monkeypatch.setattr(backend_registry, 'resolve_backend', lambda _: ('unused', 'offline-test'))
    models = []
    def bounded_show(view):
        models.append(view.model)
        assert view.model.session.policy.c['max_runtime_sec'] is None
        view.model.start(0)
        view.model.tick(.03, work_budget_sec=10)
        view.model.stop('test injected stop')
        plt.close(view.figure)
    monkeypatch.setattr(ContinuousPreview, 'show', bounded_show)
    def in_process(command):
        command = list(command)
        if '--output' in command:
            command[command.index('--output')+1] = str(tmp_path / 'offline outputs')
        monkeypatch.setattr(sys, 'argv', command[1:])
        if command[1] == 'tools/visualize_continuous_run.py':
            from tools.visualize_continuous_run import main
            main()
            return 0
        return runner.main()
    monkeypatch.setattr(menu, 'run_child', in_process)
    answers(monkeypatch, '')
    assert menu.run_step('15') == 0
    assert models[0].status == 'ENDED'
    answers(monkeypatch, '', '.05')
    assert menu.run_step('16') == 0
    assert menu.run_step('17') == 0
    runs = list((tmp_path / 'offline outputs').glob('run_*'))
    assert len(runs) == 2
    monkeypatch.setattr(backend_registry, 'resolve_backend', original_resolve)
    before = {p: p.read_bytes() for p in runs[-1].iterdir() if p.is_file()}
    answers(monkeypatch, str(runs[-1]))
    assert menu.run_step('19') == 0
    assert (runs[-1] / 'continuous_summary.png').stat().st_size > 1000
    assert all(p.read_bytes() == content for p, content in before.items())
    assert not list(runs[-1].glob('*.mp4')) and not list(runs[-1].glob('*.gif'))


def test_workspace_read_only_validator(tmp_path):
    from tools.check_workspace_calibration import main
    path = tmp_path / 'corners.yaml'
    path.write_bytes((ROOT / 'workspace/config/workspace_calibration.yaml').read_bytes())
    before = path.read_bytes()
    assert main(['--calibration', str(path)]) == 0
    assert path.read_bytes() == before
    assert main(['--calibration', str(tmp_path / 'missing')]) == 1


def test_menu_reading_does_not_write_existing_files(tmp_path, monkeypatch, launches):
    path = make_run(tmp_path / 'experiment with spaces')
    (tmp_path / 'config.yaml').write_text('tcp: {offset: [1, 2, 3, 0, 0, 0]}')
    (tmp_path / 'site_verification.yaml').write_text('verified: false')
    def snapshot():
        return {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in tmp_path.rglob('*') if p.is_file()}
    before = snapshot()
    answers(monkeypatch, str(path))
    assert menu.run_step('19') == 0
    assert snapshot() == before


def test_document_menu_matches_source():
    guide = (ROOT / '从这里开始.md').read_text()
    for step, (name, _) in menu.STEPS.items():
        assert f'| {step} | {name} |' in guide


def test_real_terminal_echo_and_sigint_wait_for_cleanup(tmp_path):
    """Real child is a harmless Python input/sleep stub, never a hardware entry."""
    import os
    import pty
    import select
    import signal
    import subprocess
    import time
    child_code = '''import time
answer = input("START: ")
print("ANSWER=" + answer, flush=True)
try:
    while True:
        time.sleep(.01)
except KeyboardInterrupt:
    print("CLEANUP_BEGIN", flush=True)
    time.sleep(.3)
    print("CLEANUP_DONE", flush=True)
'''
    harness = tmp_path / 'terminal harness.py'
    harness.write_text(
        f'import sys\nsys.path.insert(0, {str(ROOT)!r})\nimport run_project\n'
        f'code = run_project.run_child([sys.executable, "-u", "-c", {child_code!r}])\n'
        'print("MENU_RETURN=" + str(code), flush=True)\n'
    )
    master, slave = pty.openpty()
    process = subprocess.Popen([sys.executable, str(harness)], stdin=slave, stdout=slave,
                               stderr=slave, start_new_session=True, env=menu.clean_child_environment())
    os.close(slave)
    output = b''
    def until(marker):
        nonlocal output
        deadline = time.monotonic() + 8
        while marker not in output and time.monotonic() < deadline:
            if select.select([master], [], [], .1)[0]:
                try:
                    output += os.read(master, 65536)
                except OSError:
                    break
        assert marker in output, output.decode(errors='replace')
    try:
        until(b'START: ')
        os.write(master, b'START\n')
        until(b'ANSWER=START')
        assert b'START\r\n' in output  # normal terminal echo preserved
        # A terminal Ctrl+C signals the foreground process group, including both.
        os.killpg(process.pid, signal.SIGINT)
        until(b'MENU_RETURN=130')
        assert output.index(b'CLEANUP_DONE') < output.index(b'MENU_RETURN=130')
        assert process.wait(timeout=3) == 0
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)  # only the harmless test harness
            process.wait(timeout=3)
        os.close(master)


def test_bad_metadata_in_listing_does_not_hide_other_runs(tmp_path, monkeypatch, capsys):
    broken = make_run(tmp_path / 'data' / 'broken')
    good = make_run(tmp_path / 'data' / 'valid')
    (broken / 'config_snapshot.yaml').write_text('bad: [')
    monkeypatch.setattr(menu, 'ROOT', tmp_path)
    answers(monkeypatch, str(good))
    assert menu.choose_run() == good
    output = capsys.readouterr().out
    assert str(broken) in output and str(good) in output and '元数据错误' in output
