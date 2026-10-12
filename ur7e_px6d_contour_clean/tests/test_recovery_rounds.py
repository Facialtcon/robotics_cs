"""Synthetic Base-force observations, never physical mounting verification."""
from copy import deepcopy

import numpy as np
import pytest

from core.models import RobotState, Wrench
from policy.continuous_tracking import ContinuousTrackingPolicy, State, arc_reference
from policy.boundary_estimation import handed_tangent
from experiment_logging.termination import TerminationRecorder, continuous_label


def observation(now, xy, velocity=(0., 0.)):
    return RobotState(now, np.r_[xy, np.zeros(4)], np.r_[velocity, np.zeros(4)])


def tick(policy, robot, magnitude):
    # Real fixture: environment-on-probe (-1). Simulator fixture: targetward (+1).
    force = np.array([0., -1.])*policy.c['force_direction_sign']*magnitude
    wrench = Wrench(*force, 0., 0., 0., 0.)
    command = policy.update(robot.timestamp, wrench, wrench, robot,
        execution_settled=np.linalg.norm(robot.tcp_speed[:2]) <= policy.c['settle_speed_mps'])
    return command, wrench


def recovering(config, *, real=True, hand='CLOCKWISE', multiplier=1., max_angle=90.):
    c = deepcopy(config)
    c['continuous_real_execution'] = real
    c['continuous_tracking'].update(force_direction_sign=-1 if real else 1,
        reacquire_max_angle_deg=max_angle)
    c['continuous_tracking']['tangential_speed'] *= multiplier
    c['policy'].update(follow_hand=hand, search_direction_xy=[0., -1.])
    policy = ContinuousTrackingPolicy(c)
    for i in range(100):
        tick(policy, observation(i*.01, (0., 0.)), 1.5)
    assert policy.state == State.CONTINUOUS_TRACKING
    for i in range(100, 140):
        tick(policy, observation(i*.01, (0., 0.)), 0.)
        if policy.state == State.LOCAL_REACQUIRE:
            break
    assert policy.state == State.LOCAL_REACQUIRE and policy._recovery_count == 1
    np.testing.assert_allclose(policy._memory_normal, [0., -1.])
    np.testing.assert_allclose(policy._memory_tangent, handed_tangent([0., -1.], hand))
    return policy


def follow(policy, contact_angle=None, tracking_ratio=1., logger=None):
    """Simple synthetic velocity follower with actual braking feedback."""
    xy, velocity = policy.reacquire_origin[:2].copy(), np.zeros(2)
    now = policy._last_time
    frozen, contact_at = [], None
    for _ in range(10000):
        now += .01
        contacted = contact_angle is not None and policy.reacquire_heading_deg >= contact_angle
        if contacted and contact_at is None:
            contact_at = now
        robot = observation(now, xy, velocity)
        command, wrench = tick(policy, robot, 1.5 if contacted else 0.)
        if logger is not None:
            logger.log_sample(now, wrench, wrench, robot, command,
                policy.contact_direction, policy.tangent, extra=policy.telemetry(command))
        if policy._confirm_started is not None:
            frozen.append(policy.reacquire_reference.copy())
            assert not command.move
        if policy.state != State.LOCAL_REACQUIRE:
            break
        desired = command.direction_xy*command.speed*tracking_ratio
        delta = desired-velocity
        # Retain the configured acceleration cap, including the braking tail.
        limit = policy.c['command_acceleration']*.01
        velocity += delta*min(1., limit/max(np.linalg.norm(delta), 1e-20))
        xy += velocity*.01
        assert command.speed <= policy.c['reacquire_speed']+1e-12
    return command, contact_at, frozen


@pytest.mark.parametrize('real', [False, True])
@pytest.mark.parametrize('hand', ['CLOCKWISE', 'COUNTERCLOCKWISE'])
@pytest.mark.parametrize('angle,max_angle', [(40., 60.), (40., 90.), (70., 90.)])
def test_sustained_synthetic_contact_confirms_after_braking(config, real, hand, angle, max_angle):
    policy = recovering(config, real=real, hand=hand, max_angle=max_angle)
    command, contact_at, frozen = follow(policy, angle)
    assert policy.state == State.CONTINUOUS_TRACKING and not command.move
    assert policy.reacquire_heading_deg >= angle
    assert policy.reacquire_elapsed_sec > 0.
    assert policy._last_time-contact_at >= policy.c['settle_hold_sec']
    assert len(frozen) > 2
    np.testing.assert_allclose(frozen, np.tile(frozen[0], (len(frozen), 1)))
    assert 'REACQUIRE_STOP_REQUEST' in [e.event_type for e in policy.events]
    assert policy.events[-1].event_type == 'REACQUIRED'


@pytest.mark.parametrize('max_angle', [60., 90.])
def test_no_contact_still_has_finite_angle_stop(config, max_angle):
    policy = recovering(config, max_angle=max_angle)
    command, _, _ = follow(policy)
    assert policy.state == State.STOP and not command.move
    assert policy.reason == 'local reacquire angle exhausted'
    assert policy.reacquire_heading_deg == pytest.approx(max_angle)
    assert policy.reacquire_elapsed_sec < 8.
    assert policy.reacquire_path_length < .0035


def test_multiplier_does_not_change_recovery_commands_or_budgets(config):
    base, scaled = recovering(config), recovering(config, multiplier=5.)
    a, _, _ = follow(base, 70.)
    b, _, _ = follow(scaled, 70.)
    assert a.state == b.state == 'CONTINUOUS_TRACKING'
    assert base.reacquire_elapsed_sec == pytest.approx(scaled.reacquire_elapsed_sec)
    assert base.reacquire_path_length == pytest.approx(scaled.reacquire_path_length)
    np.testing.assert_allclose(base.reacquire_reference, scaled.reacquire_reference)
    for key in ('reacquire_speed', 'reacquire_max_time_sec', 'reacquire_max_path', 'reacquire_max_distance', 'boundary_margin'):
        assert base.c[key] == scaled.c[key]


def test_following_delay_with_healthy_progress_can_finish_arc(config):
    policy = recovering(config)
    command, _, _ = follow(policy, tracking_ratio=.5)
    assert policy.state == State.STOP and not command.move
    assert policy.reason == 'local reacquire angle exhausted'
    assert policy.reacquire_heading_deg == pytest.approx(90.)
    assert policy.reacquire_elapsed_sec > 8.
    assert policy.reacquire_path_length < .0035


def at_endpoint(policy):
    policy._theta = np.deg2rad(policy.c['reacquire_max_angle_deg'])
    policy.reacquire_heading_deg = policy.c['reacquire_max_angle_deg']
    policy.reacquire_reference = arc_reference(policy.reacquire_origin, policy._memory_normal,
        policy._memory_tangent, policy.c['reacquire_radius'], policy._theta)
    return policy.reacquire_reference.copy()


@pytest.mark.parametrize('real', [False, True])
def test_endpoint_contact_precedes_angle_exhaustion(config, real):
    policy = recovering(config, real=real)
    xy = at_endpoint(policy)
    for i in range(1, 25):
        command, _ = tick(policy, observation(policy._reacquire+i*.01, xy), 1.5)
        assert not command.move
        if policy.state != State.LOCAL_REACQUIRE:
            break
    assert policy.state == State.CONTINUOUS_TRACKING
    assert policy.events[-1].event_type == 'REACQUIRED'


@pytest.mark.parametrize('real', [False, True])
def test_confirmation_can_complete_after_removed_total_time_budget(config, real):
    policy = recovering(config, real=real)
    xy = at_endpoint(policy)
    deadline = policy._reacquire+8.
    # Fresh synthetic history leading to an unfinished confirmation window.
    policy._last_time = deadline-.03
    command, _ = tick(policy, observation(deadline-.02, xy), 1.5)
    assert policy._confirm_started is not None and not command.move
    frozen = policy.reacquire_reference.copy()
    tick(policy, observation(deadline-.01, xy), 1.5)
    for i in range(20):
        command, _ = tick(policy, observation(deadline+i*.01, xy), 1.5)
        if policy.state == State.CONTINUOUS_TRACKING:
            break
    assert policy.state == State.CONTINUOUS_TRACKING and not command.move
    assert policy._recovery_count == 1
    np.testing.assert_array_equal(policy.reacquire_reference, frozen)


def record(policy, magnitude=0.):
    recorder = TerminationRecorder(strategy='continuous')
    robot = observation(policy._last_time, policy.reacquire_reference)
    force = Wrench(0., magnitude, 0., 0., 0., 0.)
    recorder.observe(policy=policy, robot=robot, processed=force, timestamp=robot.timestamp,
                     processed_force_frame='Base')
    recorder.set_stop_reason(policy.stop_reason, policy.reason)
    return recorder


def test_continuous_termination_reads_memory_and_preserves_stopped_round(config):
    policy = recovering(config)
    memory = policy.last_reliable_contact['pose'].copy()
    follow(policy)
    recorder = record(policy)
    payload = recorder.record
    assert payload['last_contact'] == memory.tolist()
    assert '最后方向有效' in payload['last_contact_meaning']
    info = payload['local_reacquire']
    assert info['round'] == 1 and info['caused_stop'] and not info['active']
    assert info['origin_pose'] == policy.reacquire_origin.tolist()
    assert info['normal'] == policy._memory_normal.tolist()
    assert info['tangent'] == policy._memory_tangent.tolist()
    assert info['max_angle_deg'] == 90. and info['angle_deg'] == pytest.approx(90.)
    assert info['elapsed_sec'] == policy.reacquire_elapsed_sec
    assert info['actual_path_m'] == policy.reacquire_path_length
    assert info['current_fxy_N'] == 0. and info['contact_threshold_N'] == 1.
    assert info['stop_detail'] == 'local reacquire angle exhausted'
    assert payload['anchor_pose'] is None and payload['previous_local_reacquire'] is None
    text = recorder._report_text(payload)
    assert '第 1 次局部找回达到角度上限，未确认重新接触' in text
    assert '局部找回起点' in text and 'Current anchor' not in text


def test_historical_recovery_is_not_current_anchor_and_null_is_not_stop_pose(config):
    policy = recovering(config)
    follow(policy, 40.)
    recorder = record(policy, 1.5)
    assert recorder.record['local_reacquire'] is None
    assert recorder.record['previous_local_reacquire']['round'] == 1
    assert '当前不活动' in recorder._report_text(recorder.record)
    policy.last_contact_pose = policy.last_reliable_contact = None
    assert record(policy).record['last_contact'] is None
    policy.last_reliable_contact = dict(pose=np.ones(6))
    assert record(policy).record['last_contact'] == [1.]*6


def test_second_round_is_counted_across_tracking_and_waypoint_labels(config, tmp_path):
    from experiment_logging.data_logger import ExperimentLogger
    import csv
    policy = recovering(config)
    follow(policy, 40.)
    for _ in range(5):
        tick(policy, observation(policy._last_time+.01, (.005, -.003)), 1.5)
    for _ in range(40):
        tick(policy, observation(policy._last_time+.01, (.005, -.003)), 0.)
        if policy.state == State.LOCAL_REACQUIRE:
            break
    assert policy._recovery_count == 2 and policy.state == State.LOCAL_REACQUIRE
    follow(policy)
    info = record(policy).record['local_reacquire']
    assert info['round'] == 2 and info['caused_stop']
    np.testing.assert_allclose(info['origin_pose'][:2], [.005, -.003])
    with ExperimentLogger(tmp_path, config, mode='simulation', strategy='continuous', workspace_logging=False) as logger:
        for event in policy.events:
            logger.log_waypoint(event)
    with (logger.run_dir/'policy_waypoints.csv').open() as handle:
        rows = list(csv.DictReader(handle))
    starts = [r for r in rows if r['event_type'] == 'LOCAL_REACQUIRE']
    assert [r['reacquire_round'] for r in starts] == ['1', '2']
    success, = [r for r in rows if r['event_type'] == 'REACQUIRED']
    assert success['event_label'] == '第 1 次局部找回成功，继续贴边'
    assert all('拐角' not in r['event_label'] for r in rows)


def test_discrete_contact_and_anchor_report_remain_unchanged():
    from types import SimpleNamespace
    episode = SimpleNamespace(contact_pose=np.ones(6), anchor_pose=np.zeros(6))
    policy = SimpleNamespace(state='STOP', probe_episodes=[episode])
    recorder = TerminationRecorder()
    recorder.observe(policy=policy)
    recorder.set_stop_reason('STOP_USER_REQUEST')
    assert recorder.record['last_contact'] == [1.]*6
    assert recorder.record['anchor_pose'] == [0.]*6
    assert 'local_reacquire' not in recorder.record
    assert 'Current anchor:' in recorder._report_text(recorder.record)


def test_labels_do_not_claim_corner_identification():
    assert continuous_label('CONTACT_LOST') == '接触减弱持续达到判定条件'
    assert continuous_label('LOCAL_REACQUIRE', 2) == '第 2 次局部找回开始'
    assert continuous_label('REACQUIRED', 2) == '第 2 次局部找回成功，继续贴边'


@pytest.mark.parametrize('real', [False, True])
@pytest.mark.parametrize('hand', ['CLOCKWISE', 'COUNTERCLOCKWISE'])
def test_actual_full_circle_stops_once_without_old_local_budgets(config, real, hand):
    policy = recovering(config, real=real, hand=hand, max_angle=360.)
    assert policy.c['reacquire_radius'] == .002
    assert policy.c['reacquire_speed'] == .0005
    # Legacy values, if present in a selected config, no longer terminate this arc.
    policy.c.update(reacquire_max_time_sec=8., reacquire_max_path=.004, reacquire_max_distance=.004)
    command, _, _ = follow(policy)
    assert policy.state == State.STOP and not command.move
    assert policy.stop_reason.value == policy.reason == 'RECOVERY_FULL_CIRCLE_NO_CONTACT'
    assert policy.reacquire_heading_deg == 360.
    tolerance_deg = np.rad2deg(policy.c['reacquire_reference_freeze_error']/policy.c['reacquire_radius'])
    assert policy.reacquire_actual_heading_deg >= 360.-tolerance_deg
    assert policy.reacquire_elapsed_sec > 8.
    assert policy.reacquire_path_length > .012
    assert policy.reacquire_max_displacement > .0035
    assert policy._recovery_count == 1
    np.testing.assert_allclose(policy.reacquire_reference, policy.reacquire_origin[:2], atol=1e-15)
    assert not tick(policy, observation(policy._last_time+.01, policy.tracking_pose[:2]), 0.)[0].move
    payload = record(policy).record
    assert payload['reason'] == 'RECOVERY_FULL_CIRCLE_NO_CONTACT'
    assert payload['local_reacquire']['actual_angle_deg'] == policy.reacquire_actual_heading_deg
    assert payload['local_reacquire']['tracking_error_m'] == policy.reacquire_tracking_error


@pytest.mark.parametrize('real', [False, True])
@pytest.mark.parametrize('angle', [180., 350.])
def test_late_circle_contact_uses_original_standstill_and_contact_confirmation(config, real, angle):
    policy = recovering(config, real=real, max_angle=360.)
    command, contact_at, frozen = follow(policy, angle)
    assert policy.state == State.CONTINUOUS_TRACKING and not command.move
    assert policy._last_time-contact_at >= policy.c['settle_hold_sec']
    assert policy.reacquire_heading_deg >= angle
    assert policy.reacquire_elapsed_sec > 8.
    assert policy.events[-1].event_type == 'REACQUIRED'
    np.testing.assert_allclose(frozen, np.tile(frozen[0], (len(frozen), 1)))


@pytest.mark.parametrize('real', [False, True])
def test_reference_circle_endpoint_at_origin_cannot_prove_actual_completion(config, real):
    policy = recovering(config, real=real, max_angle=360.)
    xy = at_endpoint(policy)
    for i in range(101):
        command, _ = tick(policy, observation(policy._last_time+.01, xy), 0.)
        if i == 0:
            assert policy.state == State.LOCAL_REACQUIRE
            assert policy.reacquire_actual_heading_deg == pytest.approx(0.)
        if policy.state == State.STOP:
            break
    assert policy.stop_reason.value == 'STOP_MOTION_ERROR'
    assert policy.reason == 'local reacquire execution failure: no actual arc progress'
    assert not command.move
    assert policy.diagnostics['recovery_execution']['reference_angle_deg'] == 360.
    assert policy.diagnostics['recovery_execution']['actual_angle_deg'] == pytest.approx(0.)


def test_arc_formula_unchanged_for_entire_circle(config):
    origin = np.array([.1, -.2])
    normal, tangent = np.array([1., 0.]), np.array([0., 1.])
    radius = config['continuous_tracking']['reacquire_radius']
    assert radius == .002 and config['continuous_tracking']['reacquire_max_angle_deg'] == 360.
    for degrees, offset in [(0, [0, 0]), (90, [radius, -radius]),
                            (180, [0, -2*radius]), (270, [-radius, -radius]), (360, [0, 0])]:
        np.testing.assert_allclose(arc_reference(origin, normal, tangent, radius, np.deg2rad(degrees)),
            origin+offset, atol=1e-15)


def test_full_circle_csv_and_html_preserve_actual_tcp_instead_of_reference(config, tmp_path):
    import csv
    import json
    import re
    from experiment_logging.data_logger import ExperimentLogger
    from policy.continuous_tracking import EXTRA_SAMPLE_FIELDS
    from visualization.run_plots import load_trace
    from visualization.run_html import write_html_replay
    policy = recovering(config, real=False, max_angle=360.)
    with ExperimentLogger(tmp_path, config, mode='simulation', strategy='continuous',
            workspace_logging=False, extra_sample_fields=EXTRA_SAMPLE_FIELDS) as logger:
        follow(policy, logger=logger)
        for event in policy.events:
            logger.log_waypoint(event)
    before = (logger.run_dir/'samples.csv').read_bytes()
    with (logger.run_dir/'samples.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    actual = np.array([[float(row['tcp_x']), float(row['tcp_y'])] for row in rows])
    reference = np.array([[float(row['reacquire_reference_x']), float(row['reacquire_reference_y'])] for row in rows])
    assert np.max(np.linalg.norm(actual-reference, axis=1)) > 1e-5
    assert float(rows[-1]['reacquire_heading_deg']) == 360.
    assert float(rows[-1]['reacquire_actual_heading_deg']) > 357.
    for key in ('reacquire_origin_x', 'reacquire_origin_y', 'reacquire_tracking_error',
                'tcp_vx', 'tcp_vy', 'fxy', 'reason'):
        assert rows[-1][key] != ''
    page = write_html_replay(load_trace(logger.run_dir), tmp_path)
    payload = json.loads(re.search(r'id="replay-data">(.*?)</script>', page.read_text(), re.S)[1])
    np.testing.assert_array_equal(np.array(payload['samples'])[:, 1:3], actual*1000.)
    assert any(e['kind'] == 'RECOVERY_FULL_CIRCLE_NO_CONTACT' for e in payload['events'])
    assert (logger.run_dir/'samples.csv').read_bytes() == before
