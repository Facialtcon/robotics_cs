"""Default safe-return braking regression with the real controller guard and fake SDK."""
from types import SimpleNamespace

import numpy as np
import pytest

from core.models import Wrench
from robot.rtde_controller import URRTDEController, WorkspaceGuard
from safety.safe_return import SafeReturnExecutor
from test_safe_return import return_config


@pytest.fixture
def rig(monkeypatch):
    import safety.safe_return as module
    class Clock:
        now = 100.
        def monotonic(self):
            return self.now
        def sleep(self, duration):
            self.now += duration
    clock = Clock()
    monkeypatch.setattr(module, 'time', clock)
    config = return_config()
    target = np.array([.1, .2, .2, 0, 0, 0])
    state = SimpleNamespace(pose=np.array([.4, -.2, .24, 0, 0, 0]), residual=[],
                            last_speed=0., moves=[], stops=0, settling=False,
                            reads=0, fault=None, forever=False, stop_delay=0.)
    c = URRTDEController(dict(max_tcp_speed=.01, stop_deceleration=.2, speed_acceleration=.03))
    c.guard = WorkspaceGuard(config['workspace']['limits'], .2, .001, .01, workspace_enabled=False)
    def speed():
        if state.residual:
            value = state.residual.pop(0)
        else:
            value = .0014325458468638563 if state.forever and state.settling else 0.
        state.last_speed = value
        return [value, 0, 0, 0, 0, 0]
    def move(target_pose, speed, acceleration, asynchronous):
        assert asynchronous is True
        assert state.last_speed <= .0001  # SDK movement must never bypass the gate
        state.moves.append((np.array(target_pose), clock.now))
        state.pose[:] = target_pose
        state.settling = False
        return True
    def stop(acceleration):
        state.stops += 1
        state.settling = True
        # Reproduce the observed residual speed across multiple post-stop reads.
        state.residual = [.0014325458468638563] * 3 + [.0008, 0.]
        clock.sleep(state.stop_delay)
    c.receive = SimpleNamespace(getActualTCPPose=lambda: state.pose.copy(), getActualTCPSpeed=speed)
    c.control = SimpleNamespace(moveL=move, stopL=stop, speedStop=lambda _: True)
    def read():
        state.reads += 1
        if state.settling:
            if state.fault == 'force':
                return Wrench(9, 0, 0, 0, 0, 0)
            if state.fault == 'sensor':
                raise RuntimeError('injected sensor failure while settling')
        return Wrench(0, 0, 0, 0, 0, 0)
    executor = SafeReturnExecutor(config, target, c, SimpleNamespace(read_wrench=read), None)
    return SimpleNamespace(config=config, target=target, state=state, c=c, clock=clock, executor=executor)


def test_waits_for_measured_standstill_before_descending(rig):
    result = rig.executor.execute()
    assert result.status == 'complete'
    assert len(rig.state.moves) == 2  # skip lift, original horizontal then vertical path
    np.testing.assert_allclose(rig.state.moves[0][0][:3], [.1, .2, .24])
    np.testing.assert_allclose(rig.state.moves[1][0], rig.target)
    assert rig.state.moves[1][1] - rig.state.moves[0][1] >= .02
    assert rig.state.stops == 2  # one stop request per segment, no retry
    assert rig.c.stop_request_accepted and rig.c._stopped


def test_no_standstill_times_out_without_next_move(rig):
    rig.state.forever = True
    rig.config['safe_return']['return_segment_timeout_sec'] = .02
    result = rig.executor.execute()
    assert result.status == 'aborted'
    assert 'standstill confirmation timed out' in result.abort_reason
    assert len(rig.state.moves) == 1
    assert rig.clock.now < 100.1


@pytest.mark.parametrize('fault,reason', [('force', 'force limit'), ('sensor', 'sensor failure')])
def test_braking_fault_blocks_descent(rig, fault, reason):
    rig.state.fault = fault
    result = rig.executor.execute()
    assert result.status == 'aborted' and reason in result.abort_reason
    assert len(rig.state.moves) == 1


def test_stop_wait_does_not_reset_original_segment_timeout(rig):
    rig.config['safe_return']['return_segment_timeout_sec'] = .02
    rig.state.stop_delay = .03
    result = rig.executor.execute()
    assert result.status == 'aborted' and 'standstill confirmation timed out' in result.abort_reason
    assert len(rig.state.moves) == 1


def test_settling_drift_keeps_existing_position_check(rig):
    original = rig.executor.reader.read_wrench
    def drift():
        if rig.state.settling:
            rig.state.pose[0] = .102
        return original()
    rig.executor.reader.read_wrench = drift
    result = rig.executor.execute()
    assert result.status == 'aborted' and 'settled position error' in result.abort_reason
    assert len(rig.state.moves) == 1


def test_settling_ctrl_c_does_not_start_next_segment(rig):
    original = rig.executor.reader.read_wrench
    def interrupt():
        if rig.state.settling:
            raise KeyboardInterrupt
        return original()
    rig.executor.reader.read_wrench = interrupt
    with pytest.raises(KeyboardInterrupt):
        rig.executor.execute()
    assert len(rig.state.moves) == 1 and not rig.c._return_mode


def test_settling_log_failure_does_not_start_descent(rig, monkeypatch):
    def fail_log(now, raw, processed, robot, target, speed, phase):
        if phase.endswith('_SETTLE'):
            raise OSError('injected settling log failure')
    monkeypatch.setattr(rig.executor, '_log_cycle', fail_log)
    result = rig.executor.execute()
    assert result.status == 'aborted' and 'settling log failure' in result.abort_reason
    assert len(rig.state.moves) == 1


def test_fresh_motion_after_settle_still_revokes_permission(rig):
    original = rig.c.receive.getActualTCPSpeed
    observed_low = [False]
    def speed():
        values = original()
        if rig.state.settling and values[0] == 0.:
            if observed_low[0]:
                values[0] = .001
            observed_low[0] = True
        return values
    rig.c.receive.getActualTCPSpeed = speed
    result = rig.executor.execute()
    assert result.status == 'aborted' and 'pending measured standstill' in result.abort_reason
    assert len(rig.state.moves) == 1
    assert rig.c.stop_request_accepted and not rig.c._stopped
