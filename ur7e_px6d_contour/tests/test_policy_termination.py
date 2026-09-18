"""Policy diagnostics preserve control decisions and capture the stop cause."""
from pathlib import Path

import numpy as np
import pytest

from core.models import RobotState, Wrench
from experiment_logging.termination import TerminationReason
from policy.local_recovery import BoundaryRecovery
from policy.probe_episode import ProbeEpisode
from policy.rule_policy import RuleBasedPolicy, State
from simulation.simulator import load_simulation_config


ZERO = Wrench(0, 0, 0, 0, 0, 0)


def policy_config():
    return load_simulation_config(Path(__file__).resolve().parents[1] / "simulation/scene_square.yaml")["policy"]


def update(policy, now=0., pose=None, raw=ZERO, processed=ZERO):
    robot = RobotState(now, np.zeros(6) if pose is None else pose, np.zeros(6))
    return policy.update(now, raw, processed, robot)


def settle_return(episode, now, pose, config):
    """Complete return only after consecutive stationary, unloaded samples."""
    period = 1.0 / config["control_rate_hz"]
    hold = config.get("return_settle_hold_sec", config["contact_hold_time"])
    for index in range(int(np.ceil(hold / period)) + 2):
        timestamp = now + index * period
        assert episode.tick(timestamp, pose, ZERO, config, tcp_speed=np.zeros(6)) is None
        if index == 0:
            assert not episode.return_completed
        if episode.return_completed:
            assert episode.return_end_time - now >= hold
            return episode.return_end_time
    pytest.fail("consecutive ready samples did not complete anchor return")


def returned_contact(policy, xy, now):
    contact = np.r_[xy, np.zeros(4)]
    anchor = contact.copy()
    anchor[0] -= .02
    episode = ProbeEpisode(len(policy.probe_episodes), anchor, [1, 0], now,
                           .02, policy.state.value, "TRACKING")
    policy.probe_episodes.append(episode)
    force = Wrench(1, 0, 0, 0, 0, 0)
    episode.tick(now, contact, force, policy.config)
    episode.tick(now + .1, contact, force, policy.config)
    settle_return(episode, now + .2, anchor, policy.config)
    assert episode.return_completed
    return episode


def tracking_policy():
    policy = RuleBasedPolicy(policy_config())
    contacts = [returned_contact(policy, [.02, y], i) for i, y in enumerate((-.006, -.003, 0))]
    estimate = policy.tracker.update([e.contact_pose for e in contacts], [1, 0], reset=True)
    policy._set_estimate(estimate)
    policy._accept(contacts)
    policy.state = State.BOUNDARY_TRACKING
    return policy


def test_unknown_stop_is_explicit_and_records_pre_abort_episode_without_output(capsys):
    policy = RuleBasedPolicy(policy_config())
    command = update(policy)
    assert command.move
    episode = policy.active_episode
    policy.request_stop("unclassified diagnostic fixture halt")
    record = policy.termination.record
    assert record["reason"] == TerminationReason.STOP_UNKNOWN_REASON.value
    assert record["detail"] == policy.reason == "unclassified diagnostic fixture halt"
    assert record["policy_state"] == State.TARGET_SEARCH.value
    assert record["probe_id"] == episode.probe_id
    assert record["probe_phase"] == "PROBE"
    assert episode.outcome == "ABORTED" and policy.state == State.STOP
    assert record["command"]["move"] is True
    # A later stopped command and cleanup request must not replace first cause.
    assert not update(policy, .1).move
    policy.request_stop("operator stop")
    assert policy.reason == "operator stop"  # original policy behavior retained
    assert policy.termination.record == record
    assert record["detail"] == "unclassified diagnostic fixture halt"
    assert record["command"]["move"] is True
    assert capsys.readouterr().out == ""


@pytest.mark.parametrize("invalid", ["raw", "processed", "tcp"])
def test_nonfinite_sensor_and_robot_samples_are_classified_from_observed_evidence(invalid):
    policy = RuleBasedPolicy(policy_config())
    kwargs = {}
    if invalid == "tcp":
        pose = np.zeros(6)
        pose[0] = float("nan")
        kwargs["pose"] = pose
    else:
        kwargs[invalid] = Wrench(float("nan"), 0, 0, 0, 0, 0)
    command = update(policy, **kwargs)
    record = policy.termination.record
    expected = (TerminationReason.STOP_MOTION_ERROR if invalid == "tcp"
                else TerminationReason.STOP_INVALID_WRENCH)
    assert record["reason"] == expected.value
    assert record["detail"] == command.reason == "nonfinite sensor/robot sample"
    assert record["raw_wrench"] is not None and record["processed_wrench"] is not None
    assert record["tcp_pose"] is not None
    assert not command.move and policy.state == State.STOP


def test_processed_force_limit_records_original_reason_and_wrench():
    policy = RuleBasedPolicy(policy_config())
    force = Wrench(16, 0, 0, 0, 0, 0)
    command = update(policy, processed=force)
    record = policy.termination.record
    assert record["reason"] == TerminationReason.STOP_FORCE_LIMIT.value
    assert record["detail"] == command.reason == "processed force safety threshold exceeded"
    assert record["processed_wrench"][0] == 16
    assert record["raw_wrench"] == [0, 0, 0, 0, 0, 0]


def test_transfer_stop_freezes_state_and_last_motion_before_stopped_command():
    policy = RuleBasedPolicy(policy_config())
    policy.state = State.BOUNDARY_TRACKING
    anchor = np.array([.004, .002, 0, 0, 0, 0])
    policy._queue_transfer([(anchor, "TANGENT_STEP")], lambda t, p: None)
    moving = update(policy)
    assert moving.move
    stopped = update(policy, .1, processed=Wrench(1, 0, 0, 0, 0, 0))
    record = policy.termination.record
    assert record["reason"] == TerminationReason.STOP_UNEXPECTED_CONTACT.value
    assert record["policy_state"] == State.BOUNDARY_TRACKING.value
    assert record["policy_sub_state"] == "TANGENT_STEP"
    assert record["command"]["move"] is True
    assert record["command"]["target_pose"] == anchor.tolist()
    assert record["anchor_pose"] is not None
    assert stopped.reason.startswith("unexpected contact during anchor transfer:")
    assert "phase=TANGENT_STEP" in stopped.reason
    assert "threshold=" in stopped.reason
    assert not stopped.move and policy.state == State.STOP


def test_recovery_exhaustion_has_terminal_record_with_attempt_context():
    policy = tracking_policy()
    anchor = policy.probe_episodes[-1].anchor_pose.copy()
    policy.state = State.BOUNDARY_RECOVERY
    policy.recovery = BoundaryRecovery(anchor, policy.current_target_direction,
                                       policy.current_tangent, policy.boundary_points[-1].pose,
                                       policy.config)
    while policy.recovery.next_direction() is not None:
        pass
    attempts = len(policy.recovery.attempted_directions)
    policy._next_recovery_probe(4., anchor)
    record = policy.termination.record
    assert policy.state == State.STOP
    assert record["reason"] == TerminationReason.STOP_RECOVERY_EXHAUSTED.value
    assert record["detail"] == policy.reason == "boundary recovery exhausted expanded local sectors"
    assert record["policy_state"] == State.BOUNDARY_RECOVERY.value
    assert record["recovery_attempted_count"] == attempts
    assert record["anchor_pose"] == anchor.tolist()
    assert record["last_contact"] == policy.boundary_points[-1].pose.tolist()


@pytest.mark.parametrize("xy,reason", [
    ([.024, -.002], TerminationReason.STOP_NO_FORWARD_PROGRESS),
    ([.020, 0], TerminationReason.STOP_REPEATED_CONTACT),
    ([.030, .005], TerminationReason.STOP_FIT_FAILURE),
])
def test_local_tracking_failure_events_do_not_terminate_or_change_correction(xy, reason):
    policy = tracking_policy()
    history = list(policy.boundary_points)
    episode = returned_contact(policy, xy, 4.)
    policy._dispatch_completed(episode, episode.return_end_time, episode.returned_pose)
    assert policy.termination.record is None
    event = policy.termination.events[-1]
    assert event["reason"] == reason.value and event["terminal"] is False
    assert policy.boundary_points == history
    assert not policy.recovery_records
    assert policy.state == (State.LOCAL_INITIALIZATION if reason == TerminationReason.STOP_FIT_FAILURE
                            else State.BOUNDARY_TRACKING)
    assert policy._motion_queue


def test_no_contact_dispatch_records_nonterminal_event_then_starts_recovery():
    policy = tracking_policy()
    anchor = np.zeros(6)
    episode = ProbeEpisode(len(policy.probe_episodes), anchor, [1, 0], 4., .02,
                           policy.state.value, "TRACKING")
    policy.probe_episodes.append(episode)
    end = anchor.copy()
    end[0] = .02
    episode.tick(4.1, end, ZERO, policy.config)
    settle_return(episode, 4.2, anchor, policy.config)
    policy._dispatch_completed(episode, episode.return_end_time, anchor)
    assert policy.termination.record is None
    event = policy.termination.events[-1]
    assert event["reason"] == TerminationReason.STOP_NO_CONTACT.value
    assert event["terminal"] is False
    assert policy.state == State.BOUNDARY_RECOVERY
    assert policy.active_episode.purpose == "RECOVERY"


@pytest.mark.parametrize("completion", ["tracking", "confirmation"])
def test_policy_closure_records_success_before_changing_completion_state(completion):
    policy = tracking_policy()
    # Exercise the policy's own closure predicate with a short test budget;
    # independent target-geometry closure is covered by simulation regression.
    policy.config.update(loop_closure_enabled=True, loop_closure_min_boundary_points=4,
                         loop_closure_min_path_length=.01, loop_closure_distance=.03)
    if completion == "tracking":
        episode = returned_contact(policy, [.02, .008], 4.)
        state_before = State.BOUNDARY_TRACKING
        policy._tracked(episode, episode.return_end_time, episode.returned_pose)
    else:
        state_before = policy.state = State.BOUNDARY_CONFIRMATION
        episodes = [returned_contact(policy, [.02, y], 4. + i)
                    for i, y in enumerate((.01, .013, .016))]
        policy.recovery = BoundaryRecovery(episodes[0].anchor_pose, policy.current_target_direction,
                                           policy.current_tangent, policy.boundary_points[-1].pose,
                                           policy.config)
        policy.recovery_records.append({"confirmed_contact": None})
        policy._confirmation_contacts = episodes[:-1]
        policy._confirmation_direction = np.array([1., 0.])
        policy._confirmed(episodes[-1], episodes[-1].return_end_time, episodes[-1].returned_pose)
    record = policy.termination.record
    assert record["reason"] == TerminationReason.SUCCESS.value
    assert record["policy_state"] == state_before.value
    assert record["detail"] == policy.reason == "local contact trajectory loop closed"
    assert policy.state == State.LOOP_COMPLETE


def test_return_completion_does_not_invent_success_or_replace_scan_stop():
    policy = RuleBasedPolicy(policy_config())
    policy.state = State.RETURN_TO_START
    policy.complete_return_to_start()
    assert policy.termination.record["reason"] == TerminationReason.STOP_UNKNOWN_REASON.value
    assert policy.state == State.STOP and policy.reason == ""

    policy = RuleBasedPolicy(policy_config())
    policy.request_normal_stop()
    record = policy.termination.record
    assert record["reason"] == TerminationReason.STOP_USER_REQUEST.value
    policy.begin_return_to_start()
    policy.complete_return_to_start()
    assert policy.termination.record == record
    assert policy.state == State.STOP and policy.reason == "operator normal stop"
