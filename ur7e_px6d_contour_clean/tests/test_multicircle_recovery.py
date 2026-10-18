"""Three radii within one recovery episode, using offline TCP feedback only."""
import numpy as np
import pytest

from config.loader import runtime_robot_config
from doubles import Devices
from policy.continuous_tracking import ContinuousTrackingPolicy, State, arc_reference
from robot.rtde_controller import RobotError, check_continuous_xy
from test_real_tracking_guards import clock
from test_recovery_rounds import recovering, follow, observation, tick


@pytest.mark.parametrize('radii', [[], [0., .004, .006], [.002, .001], [.004, .006],
    [.002, float('nan')], [.002, float('inf')], [.002, True], '0.002,0.004,0.006'])
def test_invalid_radius_sequence_is_rejected(config, radii):
    config['continuous_tracking']['reacquire_radii'] = radii
    with pytest.raises(ValueError, match='reacquire_radii'):
        ContinuousTrackingPolicy(config)


@pytest.mark.parametrize('offsets', [[], [0., 60.], [0., 60., -60., 0.],
    [0., float('nan'), -60.], [0., float('inf'), -60.], [0., True, -60.], [0., '60', -60.], None])
def test_invalid_center_offsets_are_rejected(config, offsets):
    config['continuous_tracking']['reacquire_center_offsets_deg'] = offsets
    with pytest.raises(ValueError, match='reacquire_center_offsets_deg'):
        ContinuousTrackingPolicy(config)


@pytest.mark.parametrize('hand', ['CLOCKWISE', 'COUNTERCLOCKWISE'])
def test_circle_transitions_keep_origin_memory_and_continuous_commands(config, hand):
    policy = recovering(config, hand=hand, max_angle=360., origin=(.123, -.234))
    origin, normal, tangent = policy.reacquire_origin.copy(), policy._memory_normal.copy(), policy._memory_tangent.copy()
    changes = []
    # Independent Base XY expectations for the saved n=(0,-1), t=(+/-1,0).
    side = 1. if hand == 'CLOCKWISE' else -1.
    centers = origin[:2]+side*np.array([[-.002, 0.], [-.002, -.002*np.sqrt(3)], [-.003, .003*np.sqrt(3)]])
    offsets = [0., 60., -60.]
    class Observer:
        circle = 1
        reference = origin[:2].copy()
        velocity = np.zeros(2)
        radial = origin[:2]-centers[0]
        actual_angle = 0.
        def log_sample(self, now, raw, processed, robot, command, *args, extra):
            np.testing.assert_array_equal(policy.reacquire_origin, origin)
            np.testing.assert_array_equal(policy._memory_normal, normal)
            np.testing.assert_array_equal(policy._memory_tangent, tangent)
            assert policy._recovery_count == 1
            center = centers[policy.reacquire_circle-1]
            radial = robot.pose[:2]-center
            if policy.reacquire_circle == self.circle:
                # Unwrap GLOBAL polar motion around the expected centre,
                # independently of the controller's local-coordinate formula.
                delta = np.arctan2(self.radial[0]*radial[1]-self.radial[1]*radial[0], self.radial @ radial)
                self.actual_angle -= side*float(delta)
            else:
                self.actual_angle = 0.
            assert policy.reacquire_actual_heading_deg == pytest.approx(np.rad2deg(self.actual_angle), abs=1e-7)
            assert policy.reacquire_center_offset_deg == offsets[policy.reacquire_circle-1]
            np.testing.assert_allclose(policy.reacquire_center, center, atol=1e-14)
            assert np.linalg.norm(policy.reacquire_reference-center) == pytest.approx(policy.reacquire_radius, abs=1e-14)
            np.testing.assert_allclose([extra['reacquire_center_x'], extra['reacquire_center_y']], center, atol=1e-14)
            assert extra['reacquire_center_offset_deg'] == offsets[policy.reacquire_circle-1]
            if policy.reacquire_circle != self.circle or not np.any(self.velocity):
                for theta in (0., 2*np.pi):
                    endpoint = arc_reference(origin, policy._reacquire_normal, policy._reacquire_tangent,
                                             policy.reacquire_radius, theta)
                    np.testing.assert_allclose(endpoint, origin[:2], atol=1e-15)
            if policy.reacquire_circle != self.circle:
                changes.append(policy.reacquire_circle)
                assert policy.state == State.LOCAL_REACQUIRE and command.move
                assert not policy.stop_requested and policy._confirm_started is None
                assert policy.reacquire_actual_heading_deg == 0.
                assert policy.reacquire_heading_deg < 1.
                assert np.linalg.norm(policy.reacquire_reference-self.reference) <= policy.c['reacquire_speed']*policy.dt+1e-12
                assert np.linalg.norm(command.direction_xy*command.speed-self.velocity) <= policy.c['command_acceleration']*policy.dt+1e-12
                completed = policy.reacquire_circle_history[-1]
                assert np.linalg.norm(np.asarray(completed['actual_xy'])-origin[:2]) <= policy.c['reacquire_reference_freeze_error']+1e-12
                assert completed['actual_angle_deg'] >= 360.-np.rad2deg(
                    policy.c['reacquire_reference_freeze_error']/completed['radius_m'])
                assert extra['reacquire_circle'] == policy.reacquire_circle
                assert extra['reacquire_radius_m'] == policy.reacquire_radius
            self.circle = policy.reacquire_circle
            self.reference = policy.reacquire_reference.copy()
            self.velocity = command.direction_xy*command.speed
            self.radial = radial
    follow(policy, logger=Observer())
    assert changes == [2, 3]
    assert policy.stop_reason.value == 'RECOVERY_FULL_CIRCLE_NO_CONTACT'
    assert len(policy.reacquire_circle_history) == 3
    assert [c['center_offset_deg'] for c in policy.reacquire_circle_history] == offsets
    np.testing.assert_allclose([c['center_xy'] for c in policy.reacquire_circle_history], centers, atol=1e-14)
    assert policy.reacquire_path_length > .074


@pytest.mark.parametrize('real', [False, True])
@pytest.mark.parametrize('circle', [1, 2, 3])
def test_any_circle_contact_resumes_immediately_and_next_loss_restarts_at_two_mm(config, monkeypatch, real, circle):
    policy = recovering(config, real=real, max_angle=360.)
    with monkeypatch.context() as patch:
        def forbidden(*args, **kwargs):
            pytest.fail('recontact must not use stop/hold confirmation')
        patch.setattr(policy, '_confirm', forbidden)
        patch.setattr(policy, '_settled', forbidden)
        command, contacted_at, _ = follow(policy, 30., contact_circle=circle)
    assert command.move and policy.state == State.CONTINUOUS_TRACKING
    assert policy._last_time == contacted_at
    assert policy.reacquire_circle == circle
    assert len(policy.reacquire_circle_history) == circle-1
    xy = policy.tracking_pose[:2].copy()
    for _ in range(100):
        tick(policy, observation(policy._last_time+.01, xy), .3)
        if policy.state == State.LOCAL_REACQUIRE:
            break
    assert policy.state == State.LOCAL_REACQUIRE and policy._recovery_count == 2
    assert policy.reacquire_circle == 1 and policy.reacquire_radius == .002
    assert policy.reacquire_center_offset_deg == 0.
    assert policy.reacquire_circle_history == [] and policy.reacquire_actual_heading_deg == 0.


def test_last_circle_endpoint_contact_still_precedes_exhaustion(config):
    policy = recovering(config, real=False, max_angle=360.)
    normal = policy._memory_normal.copy()
    command, contact_at, _ = follow(policy, 360., contact_circle=3)
    assert command.move and policy.state == State.CONTINUOUS_TRACKING
    assert policy._last_time == contact_at and policy.reacquire_circle == 3
    assert len(policy.reacquire_circle_history) == 2
    assert policy.events[-1].event_type == 'REACQUIRED' and policy.stop_reason is None
    np.testing.assert_allclose(policy.contact_direction, normal, atol=1e-14)


def test_legacy_config_without_offsets_keeps_unrotated_centres(config):
    config['continuous_tracking'].pop('reacquire_center_offsets_deg')
    policy = recovering(config, max_angle=360.)
    command, _, _ = follow(policy, 30., contact_circle=2)
    assert command.move and policy.state == State.CONTINUOUS_TRACKING
    assert policy.reacquire_center_offset_deg == 0.
    np.testing.assert_allclose(policy.reacquire_center,
        policy.reacquire_origin[:2]-.004*policy._memory_tangent, atol=1e-14)


def test_expanded_circle_cannot_send_velocity_beyond_existing_workspace(config, clock):
    # This synthetic allowed region fits the 2 mm circle but not the 4 mm
    # circle. Use the production RTDE owner with its actual/predicted guards.
    robot = runtime_robot_config(config)
    robot.update(continuous_xy_limits=dict(x_min=-.006, x_max=.02, y_min=-.02, y_max=.02),
        continuous_boundary_margin=.0005, continuous_tracking_boundary_margin=.0005,
        continuous_search_boundary_margin=.0005,
        continuous_speed_limits=dict(LOCAL_REACQUIRE=config['continuous_tracking']['reacquire_speed']))
    devices = Devices(config, np.zeros(6))
    owner = devices.controller(robot)
    owner.connect()
    owner.activate_control(confirmed=True)
    policy = recovering(config, max_angle=360.)
    class Execution:
        def log_sample(self, now, raw, processed, state, command, *args, extra):
            clock.now += .01
            devices.receive.pose = state.pose.copy()
            devices.receive.speed = state.tcp_speed.copy()
            owner.set_continuous_phase(command.state)
            check_continuous_xy(owner.config, state.pose[:2])
            calls = len(devices.control.calls)
            try:
                owner.command_planar_velocity(command.direction_xy, command.speed, .01)
            except RobotError:
                assert len(devices.control.calls) == calls  # Rejected before SDK speedL.
                raise
    try:
        with pytest.raises(RobotError, match='boundary'):
            follow(policy, logger=Execution())
        assert policy.reacquire_circle == 2 and policy._recovery_count == 1
        check_continuous_xy(owner.config, devices.receive.pose[:2])
    finally:
        owner.request_stop(nonblocking=True)
        owner.close()
