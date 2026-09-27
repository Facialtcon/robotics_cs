import struct
import time
import numpy as np
import pytest
from config.loader import runtime_robot_config
from robot.rtde_controller import RobotError
from robot.tcp_identity import read_tcp_offset_readonly, TCPIdentityError
from doubles import Devices


def connected(config):
    devices = Devices(config, config['dry_run']['start_pose'])
    owner = devices.controller(runtime_robot_config(config))
    owner.connect()
    return devices, owner


def test_receive_preflight_confirmation_and_one_owner(config):
    d, c = connected(config)
    c.wait_for_standstill()
    assert d.control_count == 0
    with pytest.raises(RobotError, match='confirmation'):
        c.activate_control(confirmed=False)
    c.activate_control(confirmed=True)
    with pytest.raises(RobotError, match='already created'):
        c.activate_control(confirmed=True)
    assert d.control_count == d.receive_count == 1
    c.close(); c.close()
    assert not d.receive.connected and not d.control.connected


@pytest.mark.parametrize('exception', [None, RuntimeError('SDK transport anomaly')])
def test_api_anomaly_is_independent_of_physical_stop(config, exception):
    d, c = connected(config); c.activate_control(confirmed=True)
    c.command_planar_velocity([1, 0], .001, .01)
    d.control.stop_value = False; d.control.stop_exception = exception
    report = c.request_stop()
    assert report['api_anomaly'] and report['physical_stop'] == 'unconfirmed'
    c.wait_for_standstill()
    assert report['physical_stop'] == 'confirmed'
    assert not c.motion_fault
    assert len([x for x in d.control.calls if x[0] == 'speedStop']) == 1
    c.close()


def test_false_and_actual_motion_fails(config):
    d, c = connected(config); c.activate_control(confirmed=True)
    c.command_planar_velocity([1, 0], .001, .01)
    d.control.stop_value = False; d.control.keep_moving = True
    c.request_stop()
    with pytest.raises(RobotError, match='STOP_MOTION_ERROR'):
        c.wait_for_standstill(timeout=.04)
    assert not c.standstill_confirmed
    d.receive.speed[:] = 0; c.wait_for_standstill(); c.close()


def test_stale_zero_speed_cannot_prove_stop(config):
    d, c = connected(config)
    d.receive.frozen_stamp = 123.
    with pytest.raises(RobotError, match='stale'):
        c.wait_for_standstill()
    assert not c.standstill_confirmed and d.control_count == 0
    c.close()


def test_nonfinite_speed_and_tcp_mismatch_rejected(config):
    d, c = connected(config)
    d.receive.speed[0] = np.nan
    with pytest.raises(RobotError): c.wait_for_standstill()
    d.receive.speed[0] = 0
    d.receive.tcp[0] += .01
    with pytest.raises(RobotError, match='does not match'): c.activate_control(confirmed=True)
    assert d.control_count == 1
    assert not any(x[0] in ('speedL','moveL') for x in d.control.calls)
    c.close()


def test_standstill_hold_resets_on_motion(config):
    d, c = connected(config)
    started = time.monotonic()
    reads = 0
    def observe():
        nonlocal reads
        reads += 1
        d.receive.speed[0] = .001 if reads == 5 else 0.
        return c.read_diagnostic_state()
    c.wait_for_standstill(observe=observe)
    assert time.monotonic()-started >= .12
    c.close()


def test_read_only_cartesian_packet_handles_partial_reads(config):
    tcp = config['tcp']['offset']
    payload = struct.pack('!IB12d', 101, 4, *([0.]*6+tcp))
    data = struct.pack('!IB', len(payload)+5, 16)+payload
    class Stream:
        def __init__(self): self.data=data
        def __enter__(self): return self
        def __exit__(self,*args): pass
        def settimeout(self,value): pass
        def recv(self,n): result,self.data=self.data[:min(n,3)],self.data[min(n,3):]; return result
    def connect(address, timeout):
        assert address[1] == 30012
        return Stream()
    assert read_tcp_offset_readonly('fake', connect=connect) == tcp


def test_read_failure_prevents_motion_and_disconnect_still_happens(config):
    d,c=connected(config)
    c.activate_control(confirmed=True)
    d.receive.frozen_stamp=123.
    # Reset timestamp history to a valid, now-frozen packet stream.
    c._packet_stamp=c._packet_seen_at=None
    c.command_planar_velocity([1,0],.001,.01)
    c.request_stop()
    with pytest.raises(RobotError): c.close()
    assert not d.receive.connected and not d.control.connected


def test_stop_script_only_after_held_physical_stop(config):
    d,c=connected(config);c.activate_control(confirmed=True)
    c.command_planar_velocity([1,0],.001,.01);c.request_stop()
    assert c.finish_control_script_if_stopped() is False
    c.wait_for_standstill();c.close()
    assert len([x for x in d.control.calls if x[0]=='stopScript'])==1


def test_blocked_sdk_read_cannot_refresh_observation_age(config):
    d,c=connected(config)
    def slow_speed(): time.sleep(.03);return np.zeros(6)
    d.receive.getActualTCPSpeed=slow_speed
    with pytest.raises(RobotError,match='stale RTDE observation'):
        c.wait_for_standstill()
    assert not c.standstill_confirmed
    c.close()
