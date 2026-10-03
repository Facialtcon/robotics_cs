"""Direction faults stop; a calibrated model must maintain real simulated travel.

The synthetic spring has an explicitly known force convention. It is not
evidence for the mounting, stiffness or sensor sign of the physical PX6D.
"""
from copy import deepcopy

import numpy as np
import pytest

from core.models import RobotState, Wrench
from experiment_logging.termination import TerminationReason
from policy.continuous_tracking import ContinuousTrackingPolicy, State
from sensor.force_direction import control_directions


def sample(policy, now, force, *, position=(0., 0.), speed=(0., 0.)):
    wrench = Wrench(*force, 0., 0., 0., 0.)
    robot = RobotState(now, np.r_[position, np.zeros(4)], np.r_[speed, np.zeros(4)])
    return policy.update(now, wrench, wrench, robot, execution_settled=np.linalg.norm(speed) < 1e-12)


def real_policy(config):
    c = deepcopy(config)
    c['continuous_real_execution'] = True
    return c, ContinuousTrackingPolicy(c)


@pytest.mark.parametrize('code', [TerminationReason.STOP_DIRECTION_REVERSAL,
    TerminationReason.STOP_DIRECTION_UNCONFIRMED, TerminationReason.STOP_DIRECTION_NO_PROGRESS])
def test_real_direction_stop_is_terminal_not_a_warning(config, code):
    _, policy = real_policy(config)
    policy.request_stop(0., np.zeros(6), 'injected direction failure', code=code)
    command = sample(policy, .01, (1.5, 0.))
    assert policy.state == State.STOP and policy.stop_reason == code
    assert policy.stop_requested and not command.move and command.speed == 0.


def test_first_contact_rejects_force_normal_opposing_observed_approach(config):
    _, policy = real_policy(config)
    sample(policy, 0., (0., 0.))
    assert sample(policy, .01, (0., 0.), position=(.0001, 0.)).move
    for i in range(2, 25):
        command = sample(policy, i*.01, (-1.1, .2), position=(.001, 0.))
        assert not command.move
    assert policy.stop_reason == TerminationReason.STOP_DIRECTION_UNCONFIRMED
    assert 'contradicts observed search' in policy.reason
    assert policy.diagnostics['first_contact_search_normal_angle_deg'] > 150.
    assert 'TRACKING_ENTERED' not in [event.event_type for event in policy.events]
    assert policy.c['force_direction_sign'] == config['continuous_tracking']['force_direction_sign']


def test_unknown_base_force_frame_cannot_authorize_contact_feedback(config):
    c = deepcopy(config)
    c.update(continuous_real_execution=True,
             force_transform_status={'available': False, 'output_frame': 'sensor_uncalibrated'})
    with pytest.raises(ValueError, match='Base force transform unavailable before motion'):
        ContinuousTrackingPolicy(c)


def test_real_direction_change_stops_for_reconfirmation_and_timeout_is_bounded(config):
    _, policy = real_policy(config)
    for i in range(25):
        sample(policy, i*.01, (1.5, 0.))
    assert policy.state == State.CONTINUOUS_TRACKING
    angle = np.deg2rad(50.)
    force = 1.5*np.array([np.cos(angle), np.sin(angle)])
    command = sample(policy, .25, force)
    assert policy.state == State.DIRECTION_RECONFIRM
    assert policy.stop_requested and not command.move
    # An RTDE that never confirms zero speed cannot extend the deadline.
    for i in range(26, 127):
        command = sample(policy, i*.01, force, speed=(.001, 0.))
        assert not command.move
        if policy.state == State.STOP:
            break
    assert policy.stop_reason == TerminationReason.STOP_DIRECTION_UNCONFIRMED
    assert i*.01 <= .25+config['continuous_tracking']['confirmation_timeout_sec']+.01


def test_real_near_opposite_force_never_restarts_motion(config):
    _, policy = real_policy(config)
    for i in range(25):
        sample(policy, i*.01, (1.5, 0.))
    assert sample(policy, .24+.01, (-1.5, 0.)).speed == 0.
    assert policy.stop_reason == TerminationReason.STOP_DIRECTION_REVERSAL
    assert not sample(policy, .26, (1.5, 0.)).move


@pytest.mark.parametrize('hand', ['CLOCKWISE', 'COUNTERCLOCKWISE'])
@pytest.mark.parametrize('sign', [-1, 1])
def test_known_force_convention_closed_loop_maintains_contact_and_travels(config, hand, sign):
    c = deepcopy(config)
    c['continuous_real_execution'] = True
    c['force_transform_status'] = {'available': True, 'source': 'synthetic_known_mount'}
    c['continuous_tracking']['force_direction_sign'] = sign
    c['policy']['follow_hand'] = hand
    normal = np.array([-.2, -np.sqrt(.96)])
    c['policy']['search_direction_xy'] = normal.tolist()
    policy = ContinuousTrackingPolicy(c)
    # A flat boundary spring: pressing along +normal increases the measured
    # resultant. The reported sign here is specified independently of policy.
    stiffness_N_m = 1000.
    initial_force_N = 1.1
    position, speed = np.zeros(2), np.zeros(2)
    _, tangent, _ = control_directions(normal/sign, sign, hand)
    contact_forces, tracking_positions = [], []
    for i in range(700):
        force_N = max(0., initial_force_N+stiffness_N_m*float(position @ normal))
        force = force_N*normal/sign
        command = sample(policy, i*.01, force, position=position, speed=speed)
        assert policy.state != State.STOP
        if policy.state == State.CONTINUOUS_TRACKING:
            contact_forces.append(force_N)
            tracking_positions.append(position.copy())
        speed = command.direction_xy*command.speed
        position += speed*.01
    advance = float((tracking_positions[-1]-tracking_positions[0]) @ tangent)
    assert advance > .006  # millimetres of sustained travel, not state entry.
    assert min(contact_forces) > c['policy']['contact_threshold']
    assert contact_forces[-1] > contact_forces[0]+.2
    assert contact_forces[-1] <= c['continuous_tracking']['force_reference']
    assert policy.v_t == pytest.approx(c['continuous_tracking']['tangential_speed'])
    assert all(event.event_type not in ('CONTACT_LOST', 'LOCAL_REACQUIRE', 'DIRECTION_STOP_REQUEST')
               for event in policy.events)
