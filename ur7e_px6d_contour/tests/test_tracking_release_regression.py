"""Offline replay of ep4 from run_20260914_104553_980818; no hardware.

The fixture ends at the observed historical STOP. Subsequent release/transfer
samples below are explicitly synthetic and do not claim measured robot success.
Earlier episodes seed the historical state; their old return decisions are not
revalidated by this test.
"""
from copy import deepcopy
import csv
import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from core.models import RobotState, Wrench
from experiment_logging.termination import TerminationReason
from policy.probe_episode import ProbeEpisode
from policy.rule_policy import RuleBasedPolicy, State


ROOT = Path(__file__).resolve().parents[1]
FIXTURE = Path(__file__).parent / "fixtures/tracking_release_20260914.json"
ZERO = Wrench(0, 0, 0, 0, 0, 0)


@pytest.fixture
def observed_run():
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


def _seed_before_probe4(observed):
    policy = RuleBasedPolicy(observed["policy_config"])
    assert policy.config == observed["policy_config"]
    for record in observed["history"]:
        episode = ProbeEpisode(
            record["probe_id"], record["anchor_pose"], record["probe_direction"],
            record["probe_start_time"], record["max_probe_distance"],
            record["policy_state"], record["purpose"],
        )
        for name in ("contact_pose", "end_pose", "returned_pose"):
            setattr(episode, name, np.array(record[name]))
        for name in ("contact_time", "probe_end_time", "return_end_time",
                     "return_completed", "outcome", "initialization_round", "max_force"):
            setattr(episode, name, record[name])
        episode.contact_wrench = Wrench.from_sequence(record["contact_wrench"])
        episode.phase = "DONE"
        policy.probe_episodes.append(episode)
    ordered = [policy.probe_episodes[index]
               for index in observed["accepted_initialization_order"]]
    estimate = policy.tracker.update(
        [episode.contact_pose for episode in ordered], policy.search_direction, reset=True)
    assert estimate is not None
    policy._set_estimate(estimate)
    policy._accept(ordered)
    policy.state = State.BOUNDARY_TRACKING
    assert len(policy.boundary_points) == 3
    assert not policy.probe_episodes[0].accepted_as_boundary

    # Bind through the queue callbacks at the measured endpoints. Synthetic
    # stationary holds seed this historical entry under the new transfer gate;
    # they are not additional measured frames or a replay of the old transfer.
    clearance = observed["clearance_completion"]
    entry = observed["tracking_entry"]
    policy._queue_transfer([
        (np.array(clearance["pose"]), "RAY_CLEARANCE"),
        (np.array(entry["pose"]), "TANGENT_STEP"),
    ], policy._begin_tracking_probe)
    for frame in (clearance, entry):
        assert policy.active_episode is None
        hold = policy.config.get("return_settle_hold_sec", policy.config["contact_hold_time"])
        policy._execute_transfer(frame["timestamp"] - hold - .001,
                                 Wrench.from_sequence(frame["processed"]),
                                 np.array(frame["pose"]), tcp_speed=np.zeros(6))
        command = policy._execute_transfer(
            frame["timestamp"], Wrench.from_sequence(frame["processed"]),
            np.array(frame["pose"]), tcp_speed=np.zeros(6),
        )
        assert not command.move
    episode = policy.active_episode
    assert episode.probe_id == 4 and episode.purpose == "TRACKING"
    assert policy._tracking_entry[0] == episode.probe_id
    assert np.array_equal(policy._tracking_entry[1], clearance["pose"])
    assert np.array_equal(policy._tracking_entry[2], episode.anchor_pose)
    assert not policy._motion_queue
    return policy, episode


def _observed_update(policy, frame):
    return policy.update(
        frame["timestamp"], Wrench.from_sequence(frame["raw"]),
        Wrench.from_sequence(frame["processed"]),
        RobotState(frame["timestamp"], np.array(frame["pose"]), np.array(frame["speed"])),
    )


def _replay_to_historical_stop(observed):
    policy, episode = _seed_before_probe4(observed)
    for frame in observed["frames"]:
        command = _observed_update(policy, frame)
        assert policy.state == State.BOUNDARY_TRACKING
        assert policy.active_episode is episode
        assert not episode.return_completed and not episode.accepted_as_boundary
        assert len(policy.boundary_points) == 3
        assert not policy._motion_queue
    assert frame["historical_state"] == "STOP"
    assert observed["historical_stop_reason"] == "unexpected contact during anchor transfer"
    assert np.hypot(*frame["processed"][:2]) > policy.config["contact_threshold"]
    assert np.linalg.norm(np.array(frame["pose"][:2]) - episode.anchor_pose[:2]) < policy.config["position_tolerance"]
    assert episode.phase == "RETURN" and episode.outcome == "CONTACT"
    assert not command.move and command.speed == 0
    assert policy.termination.record is None
    assert np.array_equal(episode.contact_pose, observed["expected_contact_pose"])
    return policy, episode


def _synthetic_update(policy, timestamp, pose, *, processed=ZERO, raw=ZERO):
    """Explicit synthetic continuation: fixed TCP pose and zero actual speed."""
    return policy.update(timestamp, raw, processed,
                         RobotState(timestamp, np.array(pose), np.zeros(6)))


def _synthetic_release(policy, episode, observed):
    final = observed["frames"][-1]
    period = 1.0 / policy.config["control_rate_hz"]
    hold = policy.config.get("return_settle_hold_sec", policy.config["contact_hold_time"])
    start = final["timestamp"] + period
    for index in range(int(np.ceil(hold / period)) + 2):
        now = start + index * period
        command = _synthetic_update(policy, now, final["pose"])
        assert not command.move and command.speed == 0
        if index == 0:
            assert not episode.return_completed
        if episode.return_completed:
            assert episode.return_end_time - start >= hold
            return now
        assert not episode.accepted_as_boundary and len(policy.boundary_points) == 3
        assert policy.active_episode is episode and not policy._motion_queue
    pytest.fail("synthetic continuous unloaded/zero-speed hold did not complete return")


def test_observed_residual_force_waits_then_retraces_verified_clearance(observed_run):
    original_config = deepcopy(observed_run["policy_config"])
    policy, episode = _replay_to_historical_stop(observed_run)
    old_tangent = policy.current_tangent.copy()
    now = _synthetic_release(policy, episode, observed_run)
    assert episode.return_completed and episode.accepted_as_boundary
    assert len(policy.boundary_points) == 4
    assert policy.active_episode is None and len(policy.probe_episodes) == 5
    assert policy.state == State.BOUNDARY_TRACKING
    assert policy.termination.record is None and not policy.recovery_records
    assert not np.allclose(policy.current_tangent, old_tangent)
    (clearance, phase), (next_anchor, next_phase) = policy._motion_queue
    assert (phase, next_phase) == ("CLEARANCE_ROLLBACK", "TANGENT_STEP")
    assert np.array_equal(clearance, observed_run["clearance_completion"]["pose"])
    normal, tangent = policy.current_target_direction, policy.current_tangent
    gap = -np.dot(clearance[:2] - episode.contact_pose[:2], normal)
    assert gap == pytest.approx(.00451867002675806, abs=1e-12)
    assert np.dot(next_anchor[:2] - episode.contact_pose[:2], tangent) == pytest.approx(.003)
    assert -np.dot(next_anchor[:2] - episode.contact_pose[:2], normal) >= policy.config["retract_distance"]
    assert np.dot(next_anchor[:2] - clearance[:2], normal) == pytest.approx(0, abs=1e-12)

    # The new probe cannot start before both known rollback and tangent legs
    # complete. These endpoint/midpoint samples are synthetic, not run data.
    returned = np.array(observed_run["frames"][-1]["pose"])
    command = _synthetic_update(policy, now + .1, returned)
    assert command.move and np.array_equal(command.target_pose, clearance)
    assert policy.active_episode is None and len(policy.probe_episodes) == 5
    midpoint = (returned + clearance) / 2
    command = _synthetic_update(policy, now + .2, midpoint)
    assert command.move and np.array_equal(command.target_pose, clearance)
    assert policy.active_episode is None
    assert not _synthetic_update(policy, now + .3, clearance).move
    assert policy.active_episode is None
    assert policy._motion_queue[0][1] == "CLEARANCE_ROLLBACK"
    assert not _synthetic_update(policy, now + .361, clearance).move
    assert policy._motion_queue[0][1] == "TANGENT_STEP"
    command = _synthetic_update(policy, now + .4, clearance)
    assert command.move and np.array_equal(command.target_pose, next_anchor)
    assert policy.active_episode is None and len(policy.probe_episodes) == 5
    assert not _synthetic_update(policy, now + .5, next_anchor).move
    assert policy.active_episode is None
    assert not _synthetic_update(policy, now + .561, next_anchor).move
    assert policy.active_episode.probe_id == 5
    assert policy.active_episode.probe_start_time >= episode.return_end_time
    assert np.array_equal(policy.active_episode.anchor_pose, next_anchor)
    assert policy._tracking_entry[0] == 5
    assert np.array_equal(policy._tracking_entry[1], clearance)
    assert not policy._motion_queue and not policy.recovery_records
    assert policy.config == original_config == observed_run["policy_config"]


def test_persistent_return_load_times_out_with_anchor_error(observed_run):
    policy, episode = _replay_to_historical_stop(observed_run)
    final = observed_run["frames"][-1]
    timeout = policy.config.get("return_settle_timeout_sec", 2.0)
    for index in range(1, int(np.ceil(timeout / .1)) + 3):
        now = final["timestamp"] + index * .1
        command = _synthetic_update(
            policy, now, final["pose"], processed=Wrench.from_sequence(final["processed"]),
            raw=Wrench.from_sequence(final["raw"]),
        )
        assert not command.move and command.speed == 0
        assert not episode.return_completed and not episode.accepted_as_boundary
        assert len(policy.boundary_points) == 3 and len(policy.probe_episodes) == 5
        if policy.state == State.STOP:
            break
    assert policy.state == State.STOP
    record = policy.termination.record
    assert record["reason"] == TerminationReason.STOP_ANCHOR_ERROR.value
    assert "return readiness timeout" in record["detail"]
    assert record["probe_phase"] == "RETURN" and record["probe_return_completed"] is False
    assert record["processed_wrench"] == final["processed"]
    assert not policy._motion_queue and not policy.recovery_records


@pytest.mark.parametrize("phase", ["CLEARANCE_ROLLBACK", "TANGENT_STEP"])
def test_fallback_transfer_retains_contact_guard(observed_run, phase):
    policy, episode = _replay_to_historical_stop(observed_run)
    now = _synthetic_release(policy, episode, observed_run)
    pose = np.array(observed_run["frames"][-1]["pose"])
    if phase == "TANGENT_STEP":
        pose = policy._motion_queue[0][0].copy()
        _synthetic_update(policy, now + .1, pose)
        _synthetic_update(policy, now + .161, pose)
        now += .161
    assert policy._motion_queue[0][1] == phase
    force = Wrench(policy.config["contact_threshold"], 0, 0, 0, 0, 0)
    command = _synthetic_update(policy, now + .1, pose, processed=force)
    assert not command.move and command.speed == 0
    assert policy.state == State.STOP
    assert policy.termination.record["reason"] == TerminationReason.STOP_UNEXPECTED_CONTACT.value
    assert policy.termination.record["policy_sub_state"] == phase
    assert len(policy.probe_episodes) == 5 and policy.active_episode is None


@pytest.mark.parametrize("binding", ["missing", "wrong_probe_id", "wrong_anchor"])
def test_limited_clearance_without_matching_entry_never_translates(observed_run, binding):
    policy, episode = _replay_to_historical_stop(observed_run)
    # Fault injection only: the positive fixture always creates entry via the
    # real transfer callbacks, then this test deliberately invalidates it.
    probe_id, clearance, anchor = policy._tracking_entry
    if binding == "missing":
        policy._tracking_entry = None
    elif binding == "wrong_probe_id":
        policy._tracking_entry = (probe_id - 1, clearance, anchor)
    else:
        wrong_anchor = anchor.copy()
        wrong_anchor[0] += .001
        policy._tracking_entry = (probe_id, clearance, wrong_anchor)
    _synthetic_release(policy, episode, observed_run)
    assert episode.return_completed
    assert policy.state == State.STOP
    assert policy.termination.record["reason"] == TerminationReason.STOP_ANCHOR_ERROR.value
    assert "no verified tracking entry path" in policy.reason
    assert policy.active_episode is None and len(policy.probe_episodes) == 5
    assert not policy._motion_queue and not policy.recovery_records


@pytest.mark.parametrize("name", [
    "return_settle_hold_sec", "return_settled_speed_mps", "return_settle_timeout_sec",
])
@pytest.mark.parametrize("value", [0, -.1, float("nan"), float("inf"), -float("inf")])
def test_invalid_return_readiness_settings_are_rejected(observed_run, name, value):
    with pytest.raises(ValueError, match=name):
        RuleBasedPolicy({**observed_run["policy_config"], name: value})


def test_return_timeout_cannot_be_shorter_than_required_hold(observed_run):
    with pytest.raises(ValueError, match="at least return_settle_hold_sec"):
        RuleBasedPolicy({**observed_run["policy_config"],
                         "return_settle_hold_sec": .1, "return_settle_timeout_sec": .05})


def test_portable_subset_matches_original_observed_run_when_available(observed_run):
    run_dir = ROOT / observed_run["source_run"]
    if not run_dir.exists():
        pytest.skip("optional original-run provenance check; portable replay still runs")
    config = yaml.safe_load((run_dir / "config_snapshot.yaml").read_text(encoding="utf-8"))
    assert config["policy"] == observed_run["policy_config"]
    selected = [observed_run["clearance_completion"], observed_run["tracking_entry"],
                *observed_run["frames"]]
    expected = {frame["timestamp"]: frame for frame in selected}
    with (run_dir / "samples.csv").open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            timestamp = float(row["monotonic_sec"])
            if timestamp > selected[-1]["timestamp"]:
                break
            if timestamp not in expected:
                continue
            frame = expected.pop(timestamp)
            for name, prefix, suffixes in [
                ("pose", "tcp_", ("x", "y", "z", "rx", "ry", "rz")),
                ("speed", "tcp_v", ("x", "y", "z", "rx", "ry", "rz")),
                ("raw", "raw_", ("fx", "fy", "fz", "tx", "ty", "tz")),
                ("processed", "d", ("fx", "fy", "fz", "tx", "ty", "tz")),
            ]:
                assert frame[name] == [float(row[prefix + suffix]) for suffix in suffixes]
            assert frame["historical_state"] == row["current_state"]
            assert frame["historical_sub_state"] == row["policy_sub_state"]
    assert not expected
