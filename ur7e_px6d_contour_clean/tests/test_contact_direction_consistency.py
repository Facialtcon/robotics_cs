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
from sensor.force_direction import control_directions, normal_feedback_speed


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
    c = deepcopy(config)
    c['continuous_tracking']['force_direction_sign'] = 1  # deliberately wrong for this external reaction
    _, policy = real_policy(c)
    sample(policy, 0., (0., 0.))
    assert sample(policy, .01, (0., 0.), position=(.0001, 0.)).move
    for i in range(2, 25):
        command = sample(policy, i*.01, (-1.1, .2), position=(.001, 0.))
        assert not command.move
    assert policy.stop_reason == TerminationReason.STOP_DIRECTION_UNCONFIRMED
    assert 'contradicts observed search' in policy.reason
    assert policy.diagnostics['first_contact_search_normal_angle_deg'] > 150.
    assert 'TRACKING_ENTERED' not in [event.event_type for event in policy.events]
    assert policy.c['force_direction_sign'] == 1  # contradiction must not auto-flip the configured sign


@pytest.mark.parametrize('sign,accepted', [(-1, True), (1, False)])
def test_base_minus_y_search_plus_y_external_reaction_checks_configured_sign(config, sign, accepted):
    c = deepcopy(config)
    assert c['continuous_tracking']['force_direction_sign'] == -1
    c['continuous_tracking']['force_direction_sign'] = sign
    c['policy']['search_direction_xy'] = [0., -2.]  # verify normalized search diagnostic
    _, policy = real_policy(c)
    sample(policy, 0., (0., 0.))
    assert sample(policy, .01, (0., 0.), position=(0., -.0001)).move
    for i in range(2, 25):
        command = sample(policy, i*.01, (0., 1.1), position=(0., -.001))
        if policy.state in (State.CONTINUOUS_TRACKING, State.STOP):
            break
        assert not command.move
    diagnostics = policy.diagnostics
    assert diagnostics['first_contact_force_direction_sign'] == sign
    np.testing.assert_allclose(diagnostics['first_contact_search_direction_xy'], [0., -1.])
    np.testing.assert_allclose(diagnostics['first_contact_mean_force_base_xy'], [0., 1.1])
    np.testing.assert_allclose(diagnostics['first_contact_candidate_pressing_direction_xy'], [0., sign])
    assert diagnostics['first_contact_search_normal_alignment'] == -sign
    assert diagnostics['first_contact_search_normal_angle_deg'] == (0. if accepted else 180.)
    if accepted:
        assert policy.state == State.CONTINUOUS_TRACKING and policy.stop_reason is None
        np.testing.assert_allclose(policy.contact_direction, [0., -1.])
    else:
        assert policy.state == State.STOP and not command.move
        assert policy.stop_reason == TerminationReason.STOP_DIRECTION_UNCONFIRMED
        assert 'TRACKING_ENTERED' not in [event.event_type for event in policy.events]
    assert policy.c['force_direction_sign'] == sign


@pytest.mark.parametrize('force,expected_direction', [(1.1, [0., -1.]), (1.9, [0., 1.])])
def test_external_reaction_normal_feedback_presses_at_low_force_unloads_at_high_force(config, force, expected_direction):
    assert config['continuous_tracking']['force_direction_sign'] == -1
    c = deepcopy(config)
    c['policy']['search_direction_xy'] = [0., -1.]
    _, policy = real_policy(c)
    for i in range(25):
        sample(policy, i*.01, (0., 1.5))
    assert policy.state == State.CONTINUOUS_TRACKING
    for i, magnitude in enumerate(np.linspace(1.5, force, 21)[1:], 25):
        command = sample(policy, i*.01, (0., magnitude))
    assert policy.state == State.CONTINUOUS_TRACKING
    vn = normal_feedback_speed(force, policy.c)
    assert policy.v_n == pytest.approx(vn)
    normal_velocity = vn*policy.contact_direction
    np.testing.assert_allclose(normal_velocity/np.linalg.norm(normal_velocity), expected_direction)
    # Actual policy command includes tangential velocity; its normal projection
    # must still have the same pressing/unloading sign and value.
    assert (command.direction_xy*command.speed) @ policy.contact_direction == pytest.approx(vn)


def test_simulation_keeps_its_explicit_targetward_force_convention(config):
    from experiment_logging.paths import PROJECT_ROOT
    from simulation.simulator import load_simulation_config
    from simulation.continuous_session import SimulationSession
    scene = load_simulation_config(PROJECT_ROOT/'simulation/scene_continuous.yaml')
    session = SimulationSession(config, scene)
    assert scene['continuous_tracking']['force_direction_sign'] == 1
    assert session.policy.c['force_direction_sign'] == 1
    assert session.config['continuous_tracking']['force_direction_sign'] == 1
    assert config['continuous_tracking']['force_direction_sign'] == -1


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
