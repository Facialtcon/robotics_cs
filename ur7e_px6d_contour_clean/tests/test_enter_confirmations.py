"""Fresh terminal confirmations and independent authorization of each action."""
import os
import pty
import sys
import termios
import threading

import pytest

from app import operator_input


def test_each_enter_is_fresh_even_with_textio_buffer_and_repeated_newlines(monkeypatch):
    master, slave = pty.openpty()
    stream = os.fdopen(slave, 'r', encoding='utf-8')
    monkeypatch.setattr(sys, 'stdin', stream)
    original = termios.tcgetattr(stream.fileno())
    os.write(master, b'18\n\n\n')
    assert stream.readline() == '18\n'  # Simulate menu input and Python read-ahead.
    prompts = [threading.Event(), threading.Event()]
    finished = threading.Event()
    results, errors = [], []
    def emit(value='', **kwargs):
        for index in range(2):
            if f'action {index}' in value:
                prompts[index].set()
    monkeypatch.setattr(operator_input, 'print', emit, raising=False)
    def worker():
        try:
            with operator_input.OperatorKeyboard() as keyboard:
                for index in range(2):
                    results.append(operator_input.confirm_enter(f'action {index}', read_line=keyboard.read_line))
        except BaseException as exc:
            errors.append(exc)
        finally:
            finished.set()
    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    try:
        assert prompts[0].wait(2)
        assert not prompts[1].wait(.05), 'buffered menu newline authorized action'
        os.write(master, b'\r\n\n')
        assert prompts[1].wait(2)
        assert not finished.wait(.05), 'repeated Enter authorized the next action'
        os.write(master, b'\n')
        assert finished.wait(2)
        assert not errors and results == [True, True]
        assert termios.tcgetattr(stream.fileno()) == original
    finally:
        if thread.is_alive():
            os.write(master, b'q')
            thread.join(2)
        stream.close()
        os.close(master)


@pytest.mark.parametrize('key', [b'q', b'Q', b'\x1b', b'\x04'])
def test_cancel_is_immediate_and_terminal_is_restored(monkeypatch, key):
    master, slave = pty.openpty()
    stream = os.fdopen(slave, 'r', encoding='utf-8')
    monkeypatch.setattr(sys, 'stdin', stream)
    original = termios.tcgetattr(stream.fileno())
    # Inject only after the fresh prompt; no Enter is sent with the exit key.
    def emit(value='', **kwargs):
        if 'action' in value:
            os.write(master, key)
    monkeypatch.setattr(operator_input, 'print', emit, raising=False)
    try:
        if key == b'\x04':
            with pytest.raises(EOFError):
                operator_input.confirm_enter('action')
        else:
            assert operator_input.confirm_enter('action') is False
        assert termios.tcgetattr(stream.fileno()) == original
    finally:
        stream.close()
        os.close(master)


def test_ctrl_c_exception_restores_terminal(monkeypatch):
    master, slave = pty.openpty()
    stream = os.fdopen(slave, 'r', encoding='utf-8')
    monkeypatch.setattr(sys, 'stdin', stream)
    original = termios.tcgetattr(stream.fileno())
    def interrupted(*args):
        raise KeyboardInterrupt  # SIGINT is raised by Python, not read as a terminal byte.
    monkeypatch.setattr(operator_input.select, 'select', interrupted)
    try:
        with pytest.raises(KeyboardInterrupt):
            operator_input.confirm_enter('action')
        assert termios.tcgetattr(stream.fileno()) == original
    finally:
        stream.close()
        os.close(master)


def test_redirected_blank_input_cannot_confirm(monkeypatch):
    from io import StringIO
    monkeypatch.setattr(sys, 'stdin', StringIO('\n\n'))
    with pytest.raises(EOFError, match='交互终端'):
        operator_input.confirm_enter('action')


def test_numeric_text_reads_decimal_and_editing_with_stationary_checks(monkeypatch):
    master, slave = pty.openpty()
    stream = os.fdopen(slave, 'r', encoding='utf-8')
    monkeypatch.setattr(sys, 'stdin', stream)
    original = termios.tcgetattr(stream.fileno())
    checks = []
    def emit(value='', **kwargs):
        if '倍率' in value:
            os.write(master, b'2.70\x7f5\n\n')
    monkeypatch.setattr(operator_input, 'print', emit, raising=False)
    try:
        with operator_input.OperatorKeyboard() as keyboard:
            keyboard.on_wait = lambda: checks.append(True)
            assert keyboard.read_text('倍率：') == '2.75'
        assert checks
        assert termios.tcgetattr(stream.fileno()) == original
    finally:
        stream.close()
        os.close(master)


@pytest.mark.parametrize('answer,accepted', [('', True), ('START', False), (' ', False), ('q', False), ('\x1b', False)])
def test_only_enter_confirms(answer, accepted):
    assert operator_input.confirm_enter('action', read_line=lambda _: answer) is accepted


@pytest.mark.parametrize('answer', ['', 'q'])
def test_workspace_save_requires_its_own_enter(tmp_path, answer):
    import json
    import calibrate_workspace
    points, output = tmp_path/'points.json', tmp_path/'workspace.yaml'
    points.write_text(json.dumps([[0, 0, .1, 0, 0, 0], [.2, 0, .1, 0, 0, 0],
                                 [.2, .1, .1, 0, 0, 0], [0, .1, .1, 0, 0, 0]]))
    prompts = []
    def read(prompt):
        assert not output.exists()
        prompts.append(prompt)
        return answer
    assert calibrate_workspace.main(['--points-file', str(points), '--output', str(output)],
                                   input_fn=read, print_fn=lambda _: None) == 0
    assert len(prompts) == 1 and output.exists() == (answer == '')


def test_each_taught_workspace_corner_needs_enter(config):
    import calibrate_workspace
    from doubles import Receive
    receiver = Receive([0, 0, .1, 0, 0, 0], config['tcp']['offset'])
    metadata, prompts = {'samples': {}}, []
    def read(prompt):
        assert len(metadata['samples']) == len(prompts)
        prompts.append(prompt)
        return '' if len(prompts) == 1 else '\x1b'
    assert calibrate_workspace.teach_corners(receiver, metadata, input_fn=read, print_fn=lambda _: None) is None
    assert len(prompts) == 2 and list(metadata['samples']) == ['P0']


@pytest.mark.parametrize('answer', ['', 'q'])
def test_reset_pose_save_only_after_enter(config, tmp_path, monkeypatch, answer):
    import rtde_receive
    import yaml
    import save_reset_pose
    from doubles import Receive
    output, source = tmp_path/'reset.yaml', tmp_path/'config.yaml'
    config['reset']['file'] = str(output)
    source.write_text(yaml.safe_dump(config))
    receiver = Receive([0, 0, .1, 0, 0, 0], config['tcp']['offset'])
    monkeypatch.setattr(sys, 'argv', ['save_reset_pose.py', '--config', str(source)])
    monkeypatch.setattr(rtde_receive, 'RTDEReceiveInterface', lambda _: receiver)
    monkeypatch.setattr(save_reset_pose, 'read_tcp_offset_readonly', lambda _: config['tcp']['offset'])
    def read(prompt):
        assert not output.exists()
        return answer
    monkeypatch.setattr(save_reset_pose, 'confirm_enter', lambda prompt: operator_input.confirm_enter(prompt, read_line=read))
    assert save_reset_pose.main() == 0
    assert output.exists() == (answer == '') and not receiver.connected


@pytest.mark.parametrize('answer', ['', 'q'])
def test_tcp_config_save_only_after_enter(config, tmp_path, monkeypatch, answer):
    import yaml
    from tools import read_active_tcp
    source = tmp_path/'config.yaml'
    source.write_text(yaml.safe_dump(config))
    original = source.read_bytes()
    offset = list(config['tcp']['offset']); offset[0] += .001
    monkeypatch.setattr(sys, 'argv', ['read_active_tcp.py', '--config', str(source), '--write-config'])
    monkeypatch.setattr(read_active_tcp, 'read_tcp_offset_readonly', lambda _: offset)
    def read(prompt):
        assert source.read_bytes() == original
        assert not list(tmp_path.glob('*.backup.yaml'))
        return answer
    monkeypatch.setattr(read_active_tcp, 'confirm_enter', lambda prompt: operator_input.confirm_enter(prompt, read_line=read))
    assert read_active_tcp.main() == 0
    assert (source.read_bytes() != original) == (answer == '')
    assert len(list(tmp_path.glob('*.backup.yaml'))) == int(answer == '')


@pytest.mark.parametrize('cancel_at', [0, 1, None])
def test_scan_points_save_with_separate_enters(config, tmp_path, monkeypatch, cancel_at):
    import rtde_receive
    import yaml
    import run_calibration
    from doubles import Receive
    output, source = tmp_path/'scan.yaml', tmp_path/'config.yaml'
    config['calibration']['file'] = str(output)
    source.write_text(yaml.safe_dump(config))
    receiver = Receive([0, 0, .1, 0, 0, 0], config['tcp']['offset'])
    monkeypatch.setattr(sys, 'argv', ['run_calibration.py', '--config', str(source)])
    monkeypatch.setattr(rtde_receive, 'RTDEReceiveInterface', lambda _: receiver)
    monkeypatch.setattr(run_calibration, 'read_tcp_offset_readonly', lambda _: config['tcp']['offset'])
    prompts = []
    def read(prompt):
        index = len(prompts)
        if index == 0:
            assert not output.exists()
        else:
            assert yaml.safe_load(output.read_text())['confirmed'] is False
            receiver.pose[0] += .02
        prompts.append(prompt)
        return 'q' if index == cancel_at else ''
    monkeypatch.setattr(run_calibration, 'confirm_enter', lambda prompt: operator_input.confirm_enter(prompt, read_line=read))
    assert run_calibration.main() == 0
    assert len(prompts) == (1 if cancel_at == 0 else 2)
    assert output.exists() == (cancel_at != 0)
    if output.exists():
        assert yaml.safe_load(output.read_text())['confirmed'] == (cancel_at is None)
    assert not receiver.connected
