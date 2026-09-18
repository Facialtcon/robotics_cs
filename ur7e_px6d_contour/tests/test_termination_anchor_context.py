"""Termination anchors describe the current action, not retained recovery history."""
from types import SimpleNamespace

import numpy as np
import pytest

from experiment_logging.termination import TerminationReason, TerminationRecorder


# Exact probe35/old recovery anchors from run_20260914_114734_831948.
TRACKING_ANCHOR = np.array([
    .5539541108173914, .07861381247764523, .09337316031622783,
    1.5132686069033294, -2.7528855455288332, .0006617723993003177,
])
OLD_RECOVERY_ANCHOR = np.array([
    .5582290784127943, .11359415332359415, .09336844500027242,
    1.513299996292237, -2.7528297142729574, .0006310633432533837,
])


def episode(anchor, *, phase="DONE", return_target=None, purpose="TRACKING", probe_id=35):
    return SimpleNamespace(
        probe_id=probe_id, anchor_pose=np.array(anchor),
        return_target_pose=None if return_target is None else np.array(return_target),
        phase=phase, purpose=purpose, return_completed=phase == "DONE",
        outcome="CONTACT", contact_pose=None,
    )


def policy_context(state="BOUNDARY_TRACKING", **overrides):
    policy = SimpleNamespace(
        state=state, sub_state="DONE", active_episode=None, _motion_queue=[],
        probe_episodes=[episode(TRACKING_ANCHOR, return_target=TRACKING_ANCHOR)],
        recovery=SimpleNamespace(anchor=OLD_RECOVERY_ANCHOR.copy(),
                                 attempted_directions=[0, 10], candidates=[]),
        recovery_records=[{}], boundary_points=[],
    )
    vars(policy).update(overrides)
    return policy


def snapshot(policy):
    recorder = TerminationRecorder()
    recorder.observe(policy=policy, phase="POLICY_UPDATE")
    return recorder.set_stop_reason(
        TerminationReason.STOP_ANCHOR_ERROR,
        "insufficient tracking normal clearance", source="policy.request_stop",
    )


@pytest.mark.parametrize("state,sub_state", [
    ("BOUNDARY_TRACKING", "DONE"),
    (SimpleNamespace(value="BOUNDARY_TRACKING"), "CONFIRMATION_ROLLBACK"),
    ("LOCAL_INITIALIZATION", "LATERAL_OFFSET"),
])
def test_completed_tracking_anchor_is_not_replaced_by_old_recovery(state, sub_state):
    policy = policy_context(state, sub_state=sub_state)
    record = snapshot(policy)
    assert record["anchor_pose"] == TRACKING_ANCHOR.tolist()
    assert record["anchor_pose"] != OLD_RECOVERY_ANCHOR.tolist()
    assert record["probe_anchor_pose"] == TRACKING_ANCHOR.tolist()
    assert record["probe_return_target_pose"] == TRACKING_ANCHOR.tolist()
    assert record["probe_id"] == 35 and record["probe_return_completed"] is True
    assert np.array_equal(policy.recovery.anchor, OLD_RECOVERY_ANCHOR)


@pytest.mark.parametrize("state", ["BOUNDARY_RECOVERY", SimpleNamespace(value="BOUNDARY_CONFIRMATION")])
def test_current_recovery_and_confirmation_keep_their_fixed_recovery_anchor(state):
    record = snapshot(policy_context(state))
    assert record["anchor_pose"] == OLD_RECOVERY_ANCHOR.tolist()
    assert record["recovery_attempted_count"] == 2


@pytest.mark.parametrize("state", ["BOUNDARY_TRACKING", "BOUNDARY_RECOVERY", "BOUNDARY_CONFIRMATION"])
def test_pending_queue_target_precedes_completed_probe_and_recovery_anchor(state):
    target = TRACKING_ANCHOR.copy()
    target[1] += .005
    record = snapshot(policy_context(state, _motion_queue=[(target, "TANGENT_STEP")]))
    assert record["anchor_pose"] == target.tolist()
    assert record["state"] == "ANCHOR_TRANSFER"
    assert record["probe_anchor_pose"] == TRACKING_ANCHOR.tolist()


def test_active_probe_precedes_retained_recovery_anchor():
    active = episode(TRACKING_ANCHOR, phase="PROBE")
    record = snapshot(policy_context("BOUNDARY_RECOVERY", active_episode=active))
    assert record["anchor_pose"] == TRACKING_ANCHOR.tolist()
    assert record["probe_phase"] == "PROBE"


@pytest.mark.parametrize("active", [True, False])
def test_local_acquisition_return_reports_local_target_and_preserves_search_anchor(active):
    distant_start = np.array([.62119268, .25264117, .09337, 1.5133, -2.7529, .00066])
    acquisition = episode(distant_start, phase="RETURN" if active else "DONE",
                          return_target=TRACKING_ANCHOR, purpose="ACQUISITION", probe_id=0)
    policy = policy_context("TARGET_SEARCH" if active else "LOCAL_INITIALIZATION",
                            active_episode=acquisition if active else None,
                            probe_episodes=[acquisition])
    record = snapshot(policy)
    assert record["anchor_pose"] == TRACKING_ANCHOR.tolist()
    assert record["probe_return_target_pose"] == TRACKING_ANCHOR.tolist()
    assert record["probe_anchor_pose"] == distant_start.tolist()


@pytest.mark.parametrize("active", [True, False])
def test_missing_return_target_falls_back_to_episode_anchor(active):
    current = episode(TRACKING_ANCHOR, phase="RETURN" if active else "DONE")
    record = snapshot(policy_context(active_episode=current if active else None,
                                     probe_episodes=[current]))
    assert record["anchor_pose"] == TRACKING_ANCHOR.tolist()
    assert record["probe_return_target_pose"] is None
