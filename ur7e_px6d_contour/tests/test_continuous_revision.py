"""Regression evidence: isolated feedback samples, NOT geometric recovery proof."""
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from config.loader import load_config
from core.models import RobotState, Wrench
from policy.continuous_tracking import ContinuousTrackingPolicy, State
from robot.rtde_controller import URRTDEController, RobotError

ROOT = Path(__file__).resolve().parents[1]


def config():
    c = load_config(ROOT / 'config.yaml')
    c['continuous_tracking']['reacquire_enabled'] = True
    return c


def sample(p, t, xy=(1.5, 0), speed=0., pose=None):
    pose = np.zeros(6) if pose is None else np.array(pose, dtype=float)
    w = Wrench(*xy, 0, 0, 0, 0)
    return p.update(t, w, w, RobotState(t, pose, np.array([speed, 0, 0, 0, 0, 0])))


def settled(c=None):
    p = ContinuousTrackingPolicy(config() if c is None else c)
    for i in range(101):
        sample(p, i*.01)
    assert p.state == State.CONTINUOUS_TRACKING
    return p


def test_one_second_gap_is_not_stable_contact():
    p = ContinuousTrackingPolicy(config())
    sample(p, 0)
    sample(p, .01)
    cmd = sample(p, 1.01)
    assert p.initial_contact is None
    assert not cmd.move


def test_actual_xyz_speed_must_stay_settled_before_contact_acceptance():
    p = ContinuousTrackingPolicy(config())
    for i in range(20):
        sample(p, i*.01, speed=.002)
    assert p.initial_contact is None
    for i in range(20, 60):
        sample(p, i*.01)
    assert p.initial_contact is not None
    assert p.state == State.CONTINUOUS_TRACKING


def test_equal_magnitude_direction_reversal_never_keeps_full_motion():
    p = settled()
    cmd = sample(p, 1.01, (-1.5, 0))
    assert not cmd.move
    assert not p.direction_valid
    for i in range(102, 130):
        cmd = sample(p, i*.01, (-1.5, 0))
        assert not cmd.move


def test_direction_cancellation_and_small_vector_are_not_normalized_for_motion():
    c = config()
    c['continuous_tracking']['force_direction_filter_alpha'] = .5
    p = settled(c)
    cmd = sample(p, 1.01, (-1.5, 0))
    assert not cmd.move
    p = settled()
    assert not sample(p, 1.01, (.01, -.01)).move


def test_low_force_debounce_immediately_stops_normal_advancement():
    p = settled()
    cmd = sample(p, 1.01, (0, 0))
    assert p.state == State.CONTINUOUS_TRACKING
    assert not cmd.move
    assert p.v_t == 0 and p.v_n == 0


def test_continuous_deadband_has_no_velocity_jump():
    p = settled()
    sample(p, 1.01, (1.3499, 0))
    assert abs(p.v_n) < 1e-6


def test_sub_safety_overload_reduces_tangent():
    p = settled()
    for i in range(101, 111):
        sample(p, i*.01, (1.5 + (i-100)*.1, 0))
    assert p.state != State.STOP
    assert p.v_t < p.c['tangential_speed'] * .5


def test_weak_but_reliable_contact_updates_memory_beyond_four_mm():
    p = settled()
    for i in range(101, 651):
        sample(p, i*.01, (.9, 0), pose=[0, -(i-100)*.00001, 0, 0, 0, 0])
    assert np.linalg.norm(p.last_contact_pose[:2] - [0, -.0055]) < .0001
    for i in range(651, 700):
        cmd = sample(p, i*.01, (0, 0), pose=[0, -.0055, 0, 0, 0, 0])
    assert p.state == State.LOCAL_REACQUIRE
    assert np.allclose(p.reacquire_origin[:2], [0, -.0055])
    origin = p.reacquire_origin.copy()
    sample(p, 7., (0, 0), pose=[.0001, -.0055, 0, 0, 0, 0])
    assert np.array_equal(p.reacquire_origin, origin)


@pytest.mark.parametrize('result', [False, RuntimeError('transport failure')])
def test_stop_failure_is_latched_and_cannot_be_reported_stopped(result):
    p = URRTDEController({'stop_deceleration': .2})
    def stop(*args):
        if isinstance(result, Exception):
            raise result
        return result
    p.control = SimpleNamespace(speedStop=stop, stopL=stop)
    p.receive = SimpleNamespace(getActualTCPSpeed=lambda: [0]*6)
    p._stopped = False
    p._motion_mode = 'speed'
    with pytest.raises(RobotError):
        p.safe_stop_motion()
    assert not p._stopped
    assert p.motion_fault


def test_default_reacquire_is_disabled():
    c = load_config(ROOT / 'config.yaml')
    assert c['continuous_tracking'].get('reacquire_enabled') is False


def test_delayed_execution_has_nonzero_speed_after_stop_request():
    from simulation.simulated_robot import SimulatedRobot
    robot = SimulatedRobot([0, 0], .01, dict(x_min=-1, x_max=1, y_min=-1, y_max=1),
                           acceleration_limit=.005, delay_steps=2)
    for _ in range(50):
        robot.apply_command([1, 0], .001, True)
    robot.apply_command([0, 0], 0, False)
    assert robot.read_state().tcp_speed[0] > 0
    for _ in range(30):
        robot.apply_command([0, 0], 0, False)
    assert np.linalg.norm(robot.read_state().tcp_speed) == 0


@pytest.mark.parametrize('start_angle', [179., -179., 35.])
@pytest.mark.parametrize('hand', ['CLOCKWISE', 'COUNTERCLOCKWISE'])
def test_wrapped_smooth_rotation_respects_actual_dt_and_orthogonality(start_angle, hand):
    c=config(); c['policy']['follow_hand']=hand
    p=ContinuousTrackingPolicy(c)
    initial=1.5*np.array([np.cos(np.deg2rad(start_angle)), np.sin(np.deg2rad(start_angle))])
    for i in range(101): sample(p,i*.01,initial)
    previous=p.contact_direction.copy(); previous_v=p._velocity.copy(); now=1.
    for i in range(1,51):
        dt=.005 if i%2 else .015
        now+=dt
        a=np.deg2rad(start_angle+(now-1)*30)
        cmd=sample(p,now,1.5*np.array([np.cos(a),np.sin(a)]))
        assert p.state==State.CONTINUOUS_TRACKING
        change=np.arccos(np.clip(np.dot(previous,p.contact_direction),-1,1))
        assert change <= np.deg2rad(c['continuous_tracking']['direction_rate_deg_s'])*dt+1e-8
        assert abs(np.dot(p.contact_direction,p.tangent))<1e-12
        assert np.isclose(np.linalg.norm(p.contact_direction),1)
        velocity=cmd.direction_xy*cmd.speed
        assert np.linalg.norm(velocity-previous_v) <= c['continuous_tracking']['command_acceleration']*dt+1e-10
        previous=p.contact_direction.copy(); previous_v=velocity


def test_confirmation_drop_and_velocity_spike_reset_independent_windows():
    p=ContinuousTrackingPolicy(config())
    for i in range(6): sample(p,i*.01)
    sample(p,.06,(.9,0))
    for i in range(7,12): sample(p,i*.01,(min(1.5,.9+(i-6)*.2),0))
    assert p.initial_contact is None
    sample(p,.12,speed=.001)
    for i in range(13,20): sample(p,i*.01)
    assert p.initial_contact is None
    sample(p,.20)
    sample(p,.21)
    assert p.initial_contact is not None


def test_reacquisition_does_not_accept_sample_gap_or_moving_tcp():
    p=settled()
    for i in range(101,119): sample(p,i*.01,(0,0))
    for i in range(119,125): sample(p,i*.01,((i-118)*.25,0),speed=.001)
    assert p.state==State.LOCAL_REACQUIRE
    sample(p,2.24,speed=.001)
    assert p.state==State.STOP
    assert not any(e.event_type=='REACQUIRED' for e in p.events)


def test_unloading_saturation_without_improvement_is_bounded():
    p=settled()
    for i in range(101,116): sample(p,i*.01,(1.5+(i-100)*.1,0))
    for i in range(116,180): sample(p,i*.01,(3.,0))
    assert p.state==State.STOP
    assert 'saturated unloading' in p.reason


def test_moving_forever_during_first_confirmation_times_out():
    p=ContinuousTrackingPolicy(config())
    for i in range(110): sample(p,i*.01,speed=.002)
    assert p.state==State.STOP and p.initial_contact is None
    assert 'confirmation timeout' in p.reason


def test_stale_memory_cannot_start_recovery_after_slow_stop():
    p=settled()
    for i in range(101,171): sample(p,i*.01,(0,0),speed=.001)
    for i in range(171,190): sample(p,i*.01,(0,0))
    assert p.state==State.STOP
    assert 'stale' in p.reason


def test_realized_normal_velocity_limit_survives_direction_slew():
    c=config(); c['continuous_tracking']['normal_speed_limit']=.00001
    p=settled(c)
    for i in range(101,106): sample(p,i*.01,(1.5+(i-100)*.06,0))
    for i in range(106,125):
        a=np.deg2rad((i-105)*.5)
        cmd=sample(p,i*.01,1.8*np.array([np.cos(a),np.sin(a)]))
        assert abs(np.dot(cmd.direction_xy*cmd.speed,p.contact_direction)) <= .00001+1e-12


def test_recovery_logging_keeps_frozen_memory_and_origin_separate():
    p=settled()
    for i in range(101,119): command=sample(p,i*.01,(0,0))
    row=p.telemetry(command)
    assert row['memory_timestamp']==1.
    assert row['reacquire_origin_x']==0.
    assert row['memory_normal_x']==row['reacquire_normal_x']==1.
    assert row['reacquire_tangent_y']==-1.
    assert row['loss_detection_x']==0.


def test_new_stop_request_invalidates_old_confirmation_flag():
    p=settled(); p.stop_confirmed=True
    p.request_stop(1.01,np.zeros(6),'test new stop')
    assert not p.stop_confirmed


def test_recovery_motion_does_not_keep_origin_standstill_flag():
    p=settled()
    for i in range(101,118): sample(p,i*.01,(0,0))
    command=sample(p,1.18,(0,0))
    assert command.move
    assert not p.stop_confirmed


def low_force_pause():
    """Isolated fresh feedback; force return ramps respect the existing rate limit."""
    p = settled()
    assert not sample(p, 1.01, (.4, 0), speed=.002).move
    assert p.state == State.CONTINUOUS_TRACKING
    return p


def returning_force(index):
    return (min(1.5, .4 + (index-101)*.2), 0)


def test_low_force_return_cannot_resume_while_tcp_is_moving():
    p = low_force_pause()
    for i in range(102, 125):
        command = sample(p, i*.01, returning_force(i), speed=.002)
        assert p.state == State.CONTINUOUS_TRACKING
        assert not command.move and p.stop_requested
        assert not p.stop_confirmed
        assert command.speed == 0 and p.v_t == p.v_n == 0


def test_low_force_return_requires_both_continuous_confirmation_windows():
    p = low_force_pause()
    for i in range(102, 111):
        assert not sample(p, i*.01, returning_force(i), speed=.002).move
    for i in range(111, 119):  # still short of the 80 ms standstill window
        assert not sample(p, i*.01).move
        assert p.stop_requested
    command = sample(p, 1.19)
    assert not command.move  # confirmation sample itself remains a stop
    assert p.stop_confirmed and p.direction_valid
    assert p.contact_hold_elapsed >= p.p['contact_hold_time']
    assert sample(p, 1.20).move
    assert not p.stop_requested


def test_low_force_reconfirmation_resets_contact_and_speed_windows_independently():
    p = low_force_pause()
    for i in range(102, 109):
        assert not sample(p, i*.01, returning_force(i)).move
    # Force breaks contact confirmation but not the settled-speed window.
    assert not sample(p, 1.09, (.9, 0)).move
    assert p.contact_hold_elapsed == 0
    assert p.settle_hold_elapsed >= .06
    for i in range(110, 114):
        assert not sample(p, i*.01, (min(1.5, .9+(i-109)*.2), 0)).move
    # A velocity spike resets standstill independently of contact.
    assert not sample(p, 1.14, speed=.002).move
    assert p.settle_hold_elapsed == 0 and p.contact_hold_elapsed > 0
    for i in range(115, 123):
        assert not sample(p, i*.01).move
    assert not sample(p, 1.23).move
    assert sample(p, 1.24).move


def test_low_force_reconfirmation_timeout_is_not_extended_by_repeated_dips():
    p = low_force_pause()
    for i in range(102, 215):
        # Never reach stable contact, but avoid a continuous lost interval.
        force = (.4 if i % 2 else .6, 0)
        assert not sample(p, i*.01, force).move
    assert p.state == State.STOP
    assert 'confirmation timeout' in p.reason
    assert not any(e.event_type == 'CONTACT_LOST' for e in p.events)


def test_low_force_reconfirmation_moving_timeout():
    p = low_force_pause()
    for i in range(102, 215):
        assert not sample(p, i*.01, returning_force(i), speed=.002).move
    assert p.state == State.STOP and 'confirmation timeout' in p.reason


@pytest.mark.parametrize('enabled', [False, True])
def test_low_force_pending_preserves_sustained_loss_flow(enabled):
    cfg = config()
    cfg['continuous_tracking']['reacquire_enabled'] = enabled
    p = settled(cfg)
    for i in range(101, 117):
        assert not sample(p, i*.01, (.4, 0)).move
    assert p.state == State.CONTACT_LOST
    assert not sample(p, 1.17, (.4, 0)).move
    assert p.state == (State.LOCAL_REACQUIRE if enabled else State.STOP)


def test_low_force_reconfirmation_rejects_stale_feedback_and_direction_jump():
    p = low_force_pause()
    for i in range(102, 109):
        assert not sample(p, i*.01, returning_force(i), speed=.002).move
    assert not sample(p, 1.2).move  # gap exceeds the unchanged sample-age guard
    assert p.state == State.STOP and 'stale sample' in p.reason
    p = low_force_pause()
    assert not sample(p, 1.02, (-.6, 0)).move
    assert p.state == State.STOP and 'direction reversal' in p.reason


def test_low_force_pending_manual_stop_never_restarts():
    from experiment_logging.termination import TerminationReason
    p = low_force_pause()
    p.request_stop(1.015, np.zeros(6), 'operator stop', event='USER_STOP',
                   code=TerminationReason.STOP_USER_REQUEST)
    for i in range(102, 135):
        assert not sample(p, i*.01, returning_force(i)).move
    assert p.state == State.STOP
    assert p.stop_reason == TerminationReason.STOP_USER_REQUEST
    assert p.reason == 'operator stop'


def test_request_stop_assigns_default_code_without_overwriting_first_reason():
    from experiment_logging.termination import TerminationReason
    p = settled()
    p.request_stop(1.01, np.zeros(6), 'contact/standstill confirmation timeout')
    assert p.stop_reason is not None
    reason, code = p.reason, p.stop_reason
    p.request_stop(1.02, np.zeros(6), 'operator stop', code=TerminationReason.STOP_USER_REQUEST)
    assert (p.reason, p.stop_reason) == (reason, code)
