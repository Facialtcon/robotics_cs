"""Explicit teaching snapshots use disposable RTDE doubles; never hardware."""
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest
import yaml

import run_calibration as tool

ROOT = Path(__file__).resolve().parents[1]
TCP = [0., 0., .25, 0., 0., 0.]
P0 = [.4, .2, .1, 0., 3.14, 0.]
P1 = [.45, .2, .1, 0., 3.14, 0.]


@pytest.fixture
def teaching(tmp_path, monkeypatch):
    config = yaml.safe_load((ROOT / 'config.yaml').read_text())
    config['tcp']['offset'] = TCP
    config['calibration']['file'] = 'scan.yaml'
    path = tmp_path / 'config.yaml'
    path.write_text(yaml.safe_dump(config))
    destination = tmp_path / 'scan.yaml'
    state = SimpleNamespace(generation=0, pose=P0, offset=TCP, sessions=[], live=[],
                            failures=0, moving=False, protective=False, reads=0, closed=False)

    class Receiver:
        def __init__(self, address):
            pass
        def getActualTCPPose(self):
            return state.pose
        def getActualTCPSpeed(self):
            return [.01 if state.moving else 0, 0, 0, 0, 0, 0]
        def isEmergencyStopped(self):
            return False
        def isProtectiveStopped(self):
            return state.protective
        def disconnect(self):
            state.closed = True

    class Control:
        # No movement, setTcp, reupload, unlock or reconnect methods exist.
        def __init__(self, address):
            self.generation = state.generation
            state.sessions.append(self)
            state.live.append(self)
        def getTCPOffset(self):
            state.reads += 1
            if self.generation != state.generation:
                raise RuntimeError('RTDE control script is not running!')
            if state.failures:
                state.failures -= 1
                raise RuntimeError('getTCPOffset() function did not succeed!')
            return state.offset
        def disconnect(self):
            state.live.remove(self)

    monkeypatch.setitem(sys.modules, 'rtde_receive', SimpleNamespace(RTDEReceiveInterface=Receiver))
    monkeypatch.setitem(sys.modules, 'rtde_control', SimpleNamespace(RTDEControlInterface=Control))
    monkeypatch.setattr(sys, 'argv', ['run_calibration.py', '--config', str(path)])
    return state, path, destination


def inputs(monkeypatch, state, actions):
    pending = iter(actions)
    def answer(prompt):
        # A control session must never be retained across any operator input.
        assert not state.live
        value, callback = next(pending)
        state.generation += 1  # model pendant movement ending the previous script
        if callback:
            callback()
        if isinstance(value, BaseException):
            raise value
        return value
    monkeypatch.setattr('builtins.input', answer)


def test_pendant_move_does_not_reuse_stopped_control_session(teaching, monkeypatch):
    state, config, destination = teaching
    before = config.read_bytes()
    inputs(monkeypatch, state, [('SAVE_P0', None), ('SAVE_DIRECTION', lambda: setattr(state, 'pose', P1))])
    assert tool.main() == 0
    saved = yaml.safe_load(destination.read_text())
    assert saved['confirmed'] is True
    assert saved['start_tcp_pose']['x'] == P0[0]
    assert saved['direction_reference_tcp_pose']['x'] == P1[0]
    assert saved['active_tcp_offset'] == TCP
    assert len(state.sessions) == state.reads == 2
    assert state.closed and not state.live and config.read_bytes() == before


def test_p1_read_failure_waits_for_explicit_resample(teaching, monkeypatch, capsys):
    state, _, destination = teaching
    def fail_p1():
        state.pose = P1
        state.failures = 1
    def check_not_saved():
        assert state.reads == 2  # no hidden retry
        saved = yaml.safe_load(destination.read_text())
        assert not saved['confirmed'] and saved['direction_reference_tcp_pose'] is None
    inputs(monkeypatch, state, [('SAVE_P0', None), ('SAVE_DIRECTION', fail_p1),
                                ('wrong', check_not_saved), ('SAVE_DIRECTION', check_not_saved)])
    assert tool.main() == 0
    assert len(state.sessions) == state.reads == 3
    assert yaml.safe_load(destination.read_text())['confirmed']
    assert '不会自动重试' in capsys.readouterr().err


@pytest.mark.parametrize('failure', ['read', 'changed_tcp', 'invalid_tcp', 'moving', 'protective'])
def test_bad_p1_cannot_create_valid_calibration(teaching, monkeypatch, failure):
    state, _, destination = teaching
    def bad():
        state.pose = P1
        if failure == 'read':
            state.failures = 1
        elif failure == 'changed_tcp':
            state.offset = [0, 0, .3, 0, 0, 0]
        elif failure == 'invalid_tcp':
            state.offset = [float('nan')] * 6
        elif failure == 'moving':
            state.moving = True
        else:
            state.protective = True
    inputs(monkeypatch, state, [('SAVE_P0', None), ('SAVE_DIRECTION', bad), ('CANCEL', None)])
    assert tool.main() == (1 if failure == 'invalid_tcp' else 0)
    saved = yaml.safe_load(destination.read_text())
    assert saved['confirmed'] is False and saved['direction_reference_tcp_pose'] is None
    assert not state.live and state.closed


@pytest.mark.parametrize('answer', ['CANCEL', '', KeyboardInterrupt(), EOFError()])
def test_cancel_before_p0_never_constructs_control_or_writes(teaching, monkeypatch, answer):
    state, _, destination = teaching
    destination.write_text('previous calibration preserved')
    inputs(monkeypatch, state, [(answer, None)])
    assert tool.main() == (130 if isinstance(answer, BaseException) else 0)
    assert not state.sessions and state.closed
    assert destination.read_text() == 'previous calibration preserved'


def test_p0_tcp_is_sampled_after_operator_confirmation(teaching, monkeypatch):
    state, _, destination = teaching
    destination.write_text('previous calibration preserved')
    inputs(monkeypatch, state, [('SAVE_P0', lambda: setattr(state, 'offset', [0, 0, .3, 0, 0, 0]))])
    assert tool.main() == 1
    assert destination.read_text() == 'previous calibration preserved'
    assert state.reads == 1 and not state.live


def test_p1_ctrl_c_keeps_incomplete_p0(teaching, monkeypatch):
    state, _, destination = teaching
    inputs(monkeypatch, state, [('SAVE_P0', None), (KeyboardInterrupt(), None)])
    assert tool.main() == 130
    assert state.reads == 1 and state.closed
    assert yaml.safe_load(destination.read_text())['confirmed'] is False


def test_robot_moving_after_tcp_connection_cannot_save(teaching, monkeypatch):
    state, _, destination = teaching
    import rtde_control
    original = rtde_control.RTDEControlInterface.getTCPOffset
    def became_moving(self):
        result = original(self)
        state.moving = True
        return result
    monkeypatch.setattr(rtde_control.RTDEControlInterface, 'getTCPOffset', became_moving)
    inputs(monkeypatch, state, [('SAVE_P0', None)])
    assert tool.main() == 1
    assert not destination.exists() and not state.live
