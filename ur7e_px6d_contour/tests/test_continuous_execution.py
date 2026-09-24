"""Hardware-free SDK doubles; no RTDE/serial constructors are invoked."""
from types import SimpleNamespace
import numpy as np
import pytest
from robot.rtde_controller import URRTDEController, RobotError, WorkspaceGuard


def controller():
    c = URRTDEController(dict(stop_deceleration=.2, max_tcp_speed=.03, speed_acceleration=.05,
                             continuous_sample_age_sec=.02))
    c.receive = SimpleNamespace(getActualTCPSpeed=lambda:[0]*6, getActualTCPPose=lambda:[0]*6,
                                getTimestamp=lambda:1.)
    c.control = SimpleNamespace(speedStop=lambda a:True, stopL=lambda a:None,
                                setWatchdog=lambda hz:True, kickWatchdog=lambda:True,
                                speedL=lambda *a:True)
    c.guard = WorkspaceGuard(dict(x_min=-1,x_max=1,y_min=-1,y_max=1,z_min=-1,z_max=1),0,.001,.03)
    return c


def test_successful_stop_request_is_not_actual_standstill():
    c=controller(); c._stopped=False; c._motion_mode='speed'
    c.receive.getActualTCPSpeed=lambda:[.001,0,0,0,0,0]
    c.safe_stop_motion()
    assert not c._stopped
    assert c.stop_request_accepted


def test_void_stopL_allowed_but_void_speedStop_not_accepted():
    c=controller(); c._stopped=False; c._motion_mode='linear'
    c.safe_stop_motion()
    assert c._stopped
    c=controller(); c._stopped=False; c._motion_mode='speed'
    c.control.speedStop=lambda a:None
    with pytest.raises(RobotError): c.safe_stop_motion()
    assert c.motion_fault


def test_stale_robot_packet_is_not_made_fresh_by_host_read(monkeypatch):
    import robot.rtde_controller as module
    now=[10.]
    monkeypatch.setattr(module.time,'monotonic',lambda:now[0])
    c=controller(); c.read_state()
    now[0]+=.03
    with pytest.raises(RobotError,match='stale'): c.read_state()


def test_watchdog_failure_latches_and_blocks_motion():
    c=controller()
    c.enable_watchdog(20)
    c.control.kickWatchdog=lambda:False
    with pytest.raises(RobotError): c.kick_watchdog()
    assert c.motion_fault
    with pytest.raises(RobotError): c.command_planar_velocity([1,0],.001,.01)


def test_close_disconnects_even_when_stopping_fails():
    c=controller(); c._stopped=False; c._motion_mode='speed'
    calls=[]
    c.control.speedStop=lambda a:False
    c.control.stopL=lambda a:False
    c.control.disconnect=lambda:calls.append('control')
    c.receive.disconnect=lambda:calls.append('receive')
    with pytest.raises(RobotError): c.close()
    assert calls==['control','receive']


def test_real_preflight_rejects_invalid_explicit_site_evidence_before_devices(monkeypatch):
    from config.loader import load_config
    from run_continuous_tracking import prepare_real, ROOT
    config = load_config(ROOT/'config.yaml')
    config['continuous_tracking']['site_verification']={}
    monkeypatch.setattr(URRTDEController,'connect',lambda *a,**k:pytest.fail('hardware forbidden'))
    with pytest.raises((ValueError, RobotError), match='site|bound|verification'):
        prepare_real(config, ROOT/'config.yaml')


def test_cycle_timeout_blocks_next_motion_and_does_not_hide_serial_age():
    from run_continuous_tracking import validate_cycle_timing
    c = dict(cycle_timeout_sec=.03, max_sample_gap_sec=.03, max_observation_age_sec=.02)
    with pytest.raises(RobotError):
        validate_cycle_timing(c, 1., 1.05, 1.001, .99)
    with pytest.raises(RobotError):
        validate_cycle_timing(c, 1., 1.025, 1., .99)


def test_experimental_xy_envelope_applies_on_every_state_read():
    c=controller()
    c.config.update(continuous_xy_limits=dict(x_min=-.01,x_max=.01,y_min=-.01,y_max=.01),continuous_boundary_margin=.0005)
    c.receive.getActualTCPPose=lambda:[.011,0,0,0,0,0]
    with pytest.raises(RobotError,match='boundary'): c.read_state()


def test_calibrated_sloping_sandbox_edge_blocks_actual_and_predicted_motion():
    c=controller()
    c.config.update(continuous_xy_polygon=[[0,.01],[.01,0],[0,-.01],[-.01,0]],
                    continuous_boundary_margin=.0001)
    c.receive.getActualTCPPose=lambda:[.008,.008,0,0,0,0]
    with pytest.raises(RobotError,match='sandbox boundary'):c.read_state()
    c.receive.getActualTCPPose=lambda:[.009,0,0,0,0,0]
    c.control.speedL=lambda *a:pytest.fail('outside prediction must not send speedL')
    with pytest.raises(RobotError,match='sandbox boundary'):
        c.command_planar_velocity([1,0],.02,.1)


def test_pending_stop_disallows_motion_even_if_watchdog_is_active():
    c=controller(); c.stop_request_accepted=True; c._stopped=False
    with pytest.raises(RobotError,match='standstill'): c.command_planar_velocity([1,0],.001,.01)


def test_site_record_binding_changes_with_frame_sign_and_policy():
    from copy import deepcopy
    from config.loader import load_config
    from run_continuous_tracking import site_configuration_digest, ROOT
    c=load_config(ROOT/'config.yaml'); original=site_configuration_digest(c)
    for change in ('sign','frame','tcp','safety'):
        candidate=deepcopy(c)
        if change=='sign': candidate['continuous_tracking']['force_direction_sign']=-1
        elif change=='frame': candidate['preprocessing']['coordinate_transform']['sensor_origin_in_base_m']=[.01,0,0]
        elif change=='tcp': candidate['tcp']['offset'][0]+=.001
        else: candidate['policy']['safety_force_threshold']+=1
        assert site_configuration_digest(candidate)!=original


def test_fresh_state_can_revoke_previous_standstill_before_motion():
    c=controller(); c.stop_request_accepted=True; c._stopped=True
    c.receive.getActualTCPSpeed=lambda:[.001,0,0,0,0,0]
    with pytest.raises(RobotError,match='standstill'):
        c.command_planar_velocity([1,0],.001,.01)
