"""Measured arrival is a stop request, not permission to start the next leg."""
from pathlib import Path

import numpy as np
import pytest

from config.loader import load_config
from core.models import RobotState, Wrench
from policy.rule_policy import RuleBasedPolicy, State


ZERO = Wrench(0, 0, 0, 0, 0, 0)


def transfer_policy():
    config = load_config(Path(__file__).resolve().parents[1] / "config.yaml")["policy"]
    policy = RuleBasedPolicy(config)
    policy.state = State.BOUNDARY_TRACKING
    q = np.array([.61, .12, .093, 1.51, -2.75, 0.])
    anchor = q.copy()
    anchor[0] -= .003
    completed = []
    policy._queue_transfer([(q, "RAY_CLEARANCE"), (anchor, "TANGENT_STEP")],
                           lambda now, pose: completed.append((now, pose.copy())))
    return policy, q, anchor, completed


def sample(policy, now, pose, speed=(0., 0., 0.), force=ZERO):
    return policy.update(now, ZERO, force, RobotState(now, pose.copy(), np.r_[speed, np.zeros(3)]))


def test_each_transfer_leg_waits_for_measured_stop_and_a_full_hold():
    policy, q, anchor, completed = transfer_policy()
    assert not sample(policy, 1., q, speed=(.012, 0., 0.)).move
    assert policy._motion_queue[0][1] == "RAY_CLEARANCE"
    assert policy._pending_tracking_clearance is None
    assert not sample(policy, 1.1, q).move
    assert not sample(policy, 1.149, q).move
    assert len(policy._motion_queue) == 2
    assert not sample(policy, 1.151, q).move
    assert policy._motion_queue[0][1] == "TANGENT_STEP"
    assert np.array_equal(policy._pending_tracking_clearance, q)
    command = sample(policy, 1.2, q)
    assert command.move and command.speed == pytest.approx(.012)
    assert np.array_equal(command.target_pose, anchor)
    sample(policy, 1.3, anchor)
    assert not completed
    sample(policy, 1.351, anchor)
    assert len(completed) == 1 and not policy._motion_queue
    assert np.array_equal(completed[0][1], anchor)


def test_hardware_arrival_speed_does_not_advance_to_tangent_before_force_stop():
    # run_20260914_112353_812233: measured position, Fxy, and linear-speed norm.
    # Direction of the speed is immaterial to the norm-based readiness check.
    policy, _, _, completed = transfer_policy()
    q = policy._motion_queue[0][0]
    q[:2] = [.61335504864972, .1192629288002036]
    arrived = q.copy()
    arrived[:2] = [.6133872751350032, .11916144918561242]
    command = sample(policy, 304443.124042104, arrived, (.006396472, 0., 0.),
                     Wrench(.952692195, 0, 0, 0, 0, 0))
    assert not command.move and len(policy._motion_queue) == 2
    assert policy.sub_state == "RAY_CLEARANCE"
    stopped = q.copy()
    stopped[:2] = [.6133945308492742, .11911414739824397]
    command = sample(policy, 304443.174434647, stopped, (.0003342796, 0., 0.),
                     Wrench(-.30823162425526873, .9887021203421926, 0, 0, 0, 0))
    assert not command.move and not completed
    assert policy.state == State.STOP
    record = policy.termination.record
    assert record["reason"] == "STOP_UNEXPECTED_CONTACT"
    assert record["policy_sub_state"] == "RAY_CLEARANCE"
    assert "phase=RAY_CLEARANCE" in record["detail"]
    assert "threshold=1.000000 N" in record["detail"]


def test_braking_drift_finishes_stopping_before_slow_correction():
    policy, q, _, completed = transfer_policy()
    sample(policy, 1., q, (.012, 0., 0.))
    drift = q.copy()
    drift[0] += .0003
    assert not sample(policy, 1.02, drift, (.002, 0., 0.)).move
    command = sample(policy, 1.04, drift)
    assert command.move and command.direction_xy[0] == -1
    assert np.array_equal(command.target_pose, q)
    assert command.speed == pytest.approx(policy.config["probe_speed"])
    assert policy._transfer_arrived_at == 1.
    sample(policy, 1.1, q)
    sample(policy, 1.151, q)
    assert policy._motion_queue[0][1] == "TANGENT_STEP" and not completed


@pytest.mark.parametrize("disturbance", ["position", "speed"])
def test_interrupted_stability_restarts_hold_without_restarting_timeout(disturbance):
    policy, q, _, _ = transfer_policy()
    sample(policy, 1., q)
    pose, speed = q.copy(), np.zeros(3)
    if disturbance == "position":
        pose[0] += .00021
    else:
        speed[2] = .00051
    sample(policy, 1.04, pose, speed)
    assert policy._transfer_arrived_at == 1. and policy._transfer_settle_since is None
    sample(policy, 1.07, q)
    sample(policy, 1.119, q)
    assert len(policy._motion_queue) == 2
    sample(policy, 1.121, q)
    assert len(policy._motion_queue) == 1


@pytest.mark.parametrize("blocked_by", ["moving", "position", "late_frame"])
def test_transfer_readiness_timeout_reports_anchor_error_without_starting_next_leg(blocked_by):
    policy, q, _, completed = transfer_policy()
    sample(policy, 1., q, (.001, 0., 0.))
    pose, speed = q.copy(), np.zeros(3)
    if blocked_by == "moving":
        speed[0] = .001
    elif blocked_by == "position":
        pose[0] += .001
    else:
        sample(policy, 1.01, q)
    command = sample(policy, 3.01, pose, speed)
    assert not command.move and not completed
    assert policy.state == State.STOP
    record = policy.termination.record
    assert record["reason"] == "STOP_ANCHOR_ERROR"
    for text in ("phase=RAY_CLEARANCE", "position_error=", "tcp_linear_speed=", "elapsed=", "Fxy="):
        assert text in record["detail"]
    assert policy._pending_tracking_clearance is None


@pytest.mark.parametrize("speed", [[float("nan"), 0., 0.], [0., float("inf"), 0.],
                                   [0., 0.], ["invalid", 0., 0.]])
def test_invalid_speed_is_a_motion_error_and_cannot_complete_transfer(speed):
    policy, q, _, completed = transfer_policy()
    command = policy._execute_transfer(1., ZERO, q, tcp_speed=speed)
    assert not command.move and not completed
    assert policy.termination.record["reason"] == "STOP_MOTION_ERROR"


def test_approach_travel_does_not_consume_the_readiness_deadline():
    policy, q, _, _ = transfer_policy()
    outside = q.copy()
    outside[0] += .01
    for now in (1., 10., 20.):
        assert sample(policy, now, outside).move
        assert policy._transfer_arrived_at is None
    sample(policy, 21., q)
    sample(policy, 21.051, q)
    assert policy._motion_queue[0][1] == "TANGENT_STEP"


def test_skipping_tolerance_band_stops_instead_of_reversing_at_full_speed_forever():
    policy, q, _, completed = transfer_policy()
    before, after = q.copy(), q.copy()
    before[0] += .00025
    after[0] -= .00025
    assert sample(policy, 1., before, (-.012, 0., 0.)).move
    assert not sample(policy, 1.05, after, (-.012, 0., 0.)).move
    assert policy._transfer_arrived_at == 1.05
    assert not completed and len(policy._motion_queue) == 2
    # Stationary feedback permits only a slow correction of the SAME endpoint.
    correction = sample(policy, 1.1, after)
    assert correction.move and correction.direction_xy[0] == 1.
    assert correction.speed == pytest.approx(policy.config["probe_speed"])
    for now, pose in ((1.2, before), (1.4, after), (2., before), (3.06, after)):
        assert not sample(policy, now, pose, (.012, 0., 0.)).move
    assert not completed
    assert policy.termination.record["reason"] == "STOP_ANCHOR_ERROR"
