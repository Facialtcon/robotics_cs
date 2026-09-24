"""Connection and owned-script teardown using SDK doubles only."""
import sys
import time
from types import SimpleNamespace

import pytest

from robot.rtde_controller import URRTDEController, RobotError


@pytest.mark.parametrize('missing_status', [False, True])
def test_control_start_failure_preserves_receive_snapshot_before_disconnect(monkeypatch, missing_status):
    calls = []
    def read(value):
        def getter():
            assert 'disconnect' not in calls
            return value
        return getter
    receiver = SimpleNamespace(isConnected=read(True), getRobotMode=read(7),
        getSafetyMode=read(3), getRuntimeState=read(1), getSafetyStatusBits=read(4),
        getRobotStatus=read(1), isProtectiveStopped=read(True), isEmergencyStopped=read(False),
        getTimestamp=read(12.), getActualTCPPose=read([.6,.2,.1,0,0,0]),
        getActualTCPSpeed=read([0]*6), disconnect=lambda: calls.append('disconnect'))
    def fail(ip):
        raise RuntimeError('Failed to start control script, before timeout of 5 seconds')
    if missing_status:
        receiver.getRobotMode = lambda: None
        def broken_status(): raise RuntimeError('status unavailable')
        receiver.getRuntimeState = broken_status
    monkeypatch.setitem(sys.modules, 'rtde_receive', SimpleNamespace(RTDEReceiveInterface=lambda ip: receiver))
    monkeypatch.setitem(sys.modules, 'rtde_control', SimpleNamespace(RTDEControlInterface=fail))
    c = URRTDEController({'robot_ip': 'fake'})
    with pytest.raises(RobotError, match='Failed to start control script'):
        c.connect()
    assert c.connection_failed and c.receive is None
    assert c.connection_diagnostics['protective_stopped'] is True
    assert c.connection_diagnostics['tcp_pose'] == [.6,.2,.1,0,0,0]
    assert c.connection_diagnostics['freshness'] == 'not_verified'
    if missing_status:
        assert c.connection_diagnostics['robot_mode'] is None
        assert c.connection_diagnostics['runtime_state'] is None
        assert 'status unavailable' in c.connection_diagnostics['read_errors']['runtime_state']
    assert calls == ['disconnect']


def controller(speed=0.):
    calls = []
    c = URRTDEController({'continuous_settle_speed_mps': .0001, 'stop_deceleration': .2})
    c.control = SimpleNamespace(stopScript=lambda: calls.append('stopScript'),
        disconnect=lambda: calls.append('control_disconnect'))
    c.receive = SimpleNamespace(getActualTCPPose=lambda: [0]*6,
        getActualTCPSpeed=lambda: [speed,0,0,0,0,0],
        disconnect=lambda: calls.append('receive_disconnect'))
    c.watchdog_active = True
    return c, calls


def test_owned_script_ends_before_disconnect_and_only_once():
    c, calls = controller()
    assert c.finish_control_script_if_stopped()
    assert not c.watchdog_active
    c.finish_control_script_if_stopped()
    c.close()
    assert calls == ['stopScript', 'control_disconnect', 'receive_disconnect']


def test_moving_or_unreadable_robot_keeps_watchdog_armed():
    c, calls = controller(.001)
    assert not c.finish_control_script_if_stopped()
    assert c.watchdog_active and not calls
    def fail(): raise RobotError('observation unavailable')
    c.read_diagnostic_state = fail
    with pytest.raises(RobotError): c.finish_control_script_if_stopped()
    assert c.watchdog_active and not calls


def test_script_stop_failure_does_not_skip_disconnect():
    c, calls = controller()
    def fail():
        calls.append('stopScript'); raise RuntimeError('stopScript failed')
    c.control.stopScript = fail
    with pytest.raises(RobotError, match='stopScript failed'): c.close()
    assert calls[-2:] == ['control_disconnect', 'receive_disconnect']
    assert c.watchdog_active


def test_unowned_discrete_control_script_is_unchanged():
    c, calls = controller(); c.watchdog_active = False
    c.close()
    assert 'stopScript' not in calls


def test_stale_device_packet_cannot_end_watched_script():
    c, calls = controller()
    c.config['continuous_sample_age_sec'] = .02
    c.receive.getTimestamp = lambda: 1.
    c._packet_stamp = 1.
    c._packet_seen_at = time.monotonic()-.03
    with pytest.raises(RobotError, match='stale RTDE'):
        c.finish_control_script_if_stopped()
    assert c.watchdog_active and not calls


def test_angular_motion_also_keeps_watchdog_armed():
    c, calls = controller()
    c.receive.getActualTCPSpeed = lambda: [0,0,0,.001,0,0]
    assert not c.finish_control_script_if_stopped()
    assert c.watchdog_active and not calls
