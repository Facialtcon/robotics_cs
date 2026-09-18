"""Physical invariants replacing the archived force-derived corner FSM tests."""
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from core.models import RobotState, Wrench
from policy.boundary_estimation import estimate_boundary, handed_tangent
from policy.local_recovery import BoundaryRecovery
from policy.probe_episode import ProbeEpisode
from policy.rule_policy import RuleBasedPolicy, State
from simulation.simulator import load_simulation_config

ZERO = Wrench(0, 0, 0, 0, 0, 0)


def policy_config():
    return load_simulation_config(Path(__file__).resolve().parents[1] / "simulation/scene_square.yaml")["policy"]


def tick(policy, now, pose, processed=ZERO, raw=None):
    return policy.update(now, raw or processed, processed, RobotState(now, np.asarray(pose, dtype=float), np.zeros(6)))


def settle_return(episode, now, pose, config, sample=None):
    """Supply consecutive stationary, unloaded TCP samples at the return pose."""
    period = 1.0 / config["control_rate_hz"]
    hold = config.get("return_settle_hold_sec", config["contact_hold_time"])
    for index in range(int(np.ceil(hold / period)) + 2):
        timestamp = now + index * period
        if sample is None:
            assert episode.tick(timestamp, pose, ZERO, config, tcp_speed=np.zeros(6)) is None
        else:
            assert not sample(timestamp).move
        if index == 0:
            assert not episode.return_completed
        if episode.return_completed:
            assert episode.return_end_time - now >= hold
            return episode.return_end_time
    pytest.fail("consecutive ready samples did not complete anchor return")


@pytest.mark.parametrize("force", [Wrench(0, 1, 0, 0, 0, 0), Wrench(-1, 0, 0, 0, 0, 0)])
def test_first_force_never_supplies_a_tangent_and_return_follows_search(force):
    policy = RuleBasedPolicy(policy_config())
    origin = np.zeros(6)
    assert tick(policy, 0, origin).move
    contact = origin.copy(); contact[0] = .02
    for now in (.02, .04, .061):
        assert not tick(policy, now, contact, force).move
        assert policy.current_tangent is None
        assert not policy.boundary_points
    command = tick(policy, .08, contact, force)
    assert np.allclose(command.direction_xy, [-1, 0])
    local_anchor = policy.active_episode.return_target_pose.copy()
    assert np.array_equal(command.target_pose, local_anchor)
    assert not np.array_equal(local_anchor, origin)
    assert policy.state == State.TARGET_SEARCH  # no decision until anchor return
    settle_return(policy.active_episode, .10, local_anchor, policy.config,
                  sample=lambda now: tick(policy, now, local_anchor))
    assert policy.state == State.LOCAL_INITIALIZATION
    assert policy.probe_episodes[0].return_completed
    assert np.array_equal(policy._initial_base_anchor, local_anchor)


@pytest.mark.parametrize("outcome", ["CONTACT", "NO_CONTACT", "UNSTABLE_CONTACT"])
def test_episode_returns_same_immutable_anchor_before_completion(outcome):
    config = policy_config(); anchor = np.zeros(6)
    episode = ProbeEpisode(1, anchor, [1, 1], 0, .01, "BOUNDARY_TRACKING", "TRACKING")
    anchor[0] = 10  # constructor owns its anchor, not caller memory
    with pytest.raises(ValueError):
        episode.anchor_pose[0] = 3
    end = np.zeros(6); end[:2] = episode.probe_direction * .01
    if outcome == "CONTACT":
        episode.tick(0, end, Wrench(1, 0, 0, 0, 0, 0), config)
        episode.tick(.05, end, Wrench(1, 0, 0, 0, 0, 0), config)
    elif outcome == "UNSTABLE_CONTACT":
        episode.tick(0, end, Wrench(1, 0, 0, 0, 0, 0), config)
        episode.tick(.02, end, ZERO, config)
    else:
        episode.tick(0, end, ZERO, config)
    assert episode.phase == "RETURN" and not episode.return_completed
    assert episode.outcome == outcome
    target, _ = episode.tick(.10, end, ZERO, config)
    assert np.array_equal(target, np.zeros(6))
    settle_return(episode, .12, np.zeros(6), config)
    assert episode.return_completed and episode.phase == "DONE"


def test_cannot_start_second_probe_while_first_not_returned():
    policy = RuleBasedPolicy(policy_config())
    tick(policy, 0, np.zeros(6))
    with pytest.raises(RuntimeError, match="anchor return"):
        policy._start_probe(.1, np.zeros(6), [0, 1], .01, "TRACKING")


@pytest.mark.parametrize("hand", ["CLOCKWISE", "COUNTERCLOCKWISE"])
def test_oblique_probe_pca_tangent_depends_on_contact_geometry(hand):
    points = [[.02, -.01], [.02, 0], [.02, .01]]
    estimate = estimate_boundary(points, [1, .7], hand)
    assert abs(estimate.tangent[0]) < 1e-10
    assert np.dot(estimate.tangent, handed_tangent([1, 0], hand)) > .999
    assert estimate.residual < 1e-10
    assert estimate_boundary([points[0]], [1, .7], hand) is None
    assert estimate_boundary([points[0]] * 3, [1, .7], hand) is None


def test_progressive_sectors_expand_without_direction_sign_jumps():
    config = policy_config()
    config["recovery_sector_extents_deg"] = [12, 35, 72, 143]
    recovery = BoundaryRecovery(np.zeros(6), np.array([1, 0]), np.array([0, 1]), np.zeros(6), config)
    results = []
    while (value := recovery.next_direction()) is not None:
        results.append(value)
    angles = [value[0] for value in results]
    assert np.all(np.diff(angles) > 0) and angles[-1] == 143
    assert all(direction[1] < 0 for _, direction in results)
    assert len(results) > 10
    assert np.array_equal(recovery.anchor, np.zeros(6))


def test_recovery_defers_old_tangent_projection_but_rejects_recent_duplicates():
    recovery = BoundaryRecovery(np.zeros(6), np.array([1, 0]), np.array([0, 1]), np.zeros(6), policy_config())
    # One new-face point at a sharp corner cannot establish reverse traversal.
    assert not recovery.candidate_rejection(np.array([0, -.004]), [])
    point = SimpleNamespace(pose=np.array([.001, .002]))
    assert recovery.candidate_rejection(point.pose, [point]) == "PREVIOUSLY_VISITED_BOUNDARY"
    assert not recovery.candidate_rejection(np.array([.003, .005]), [point])


@pytest.mark.parametrize("force,raw", [
    (Wrench(16, 0, 0, 0, 0, 0), None),
    (Wrench(0, 0, 0, 3, 0, 0), None),
    (ZERO, Wrench(21, 0, 0, 0, 0, 0)),
    (ZERO, Wrench(0, 0, 0, 6, 0, 0)),
    (Wrench(float("nan"), 0, 0, 0, 0, 0), None),
])
def test_force_torque_and_nonfinite_samples_fail_closed(force, raw):
    policy = RuleBasedPolicy(policy_config())
    tick(policy, 0, np.zeros(6))
    command = tick(policy, .02, np.zeros(6), force, raw)
    assert command.state == "STOP" and not command.move
    assert not policy.probe_episodes[0].return_completed
    assert policy.probe_episodes[0].outcome == "ABORTED"


def test_force_rate_guard_applies_during_search_and_return():
    for returning in (False, True):
        policy = RuleBasedPolicy(policy_config())
        tick(policy, 0, np.zeros(6))
        if returning:
            policy.active_episode.phase = "RETURN"
        command = tick(policy, .001, np.zeros(6), Wrench(.2, 0, 0, 0, 0, 0))
        assert not command.move and "force rate exceeded" in command.reason


def test_anchor_transfer_is_force_guarded():
    policy = RuleBasedPolicy(policy_config())
    policy.state = State.BOUNDARY_TRACKING
    target = np.zeros(6); target[0] = .004
    policy._queue_transfer([(target, "TANGENT_STEP")], lambda t, p: None)
    command = tick(policy, 0, np.zeros(6), Wrench(1, 0, 0, 0, 0, 0))
    assert not command.move and command.state == "STOP"
    assert "anchor transfer" in command.reason


def test_planar_increment_never_overshoots_target():
    policy = RuleBasedPolicy(policy_config())
    target = np.zeros(6); target[0] = .00003
    command = policy._command(np.zeros(6), (target, .03))
    assert command.speed / policy.config["control_rate_hz"] == pytest.approx(.00003)


def test_normal_stop_return_state_sequence_is_explicit():
    policy = RuleBasedPolicy(policy_config())
    with pytest.raises(RuntimeError):
        policy.begin_return_to_start()
    policy.request_normal_stop()
    assert policy.state == State.STOP_SCAN
    policy.begin_return_to_start()
    assert policy.state == State.RETURN_TO_START
    policy.complete_return_to_start()
    assert policy.state == State.STOP


def test_null_runtime_limits_and_invalid_configuration():
    config = policy_config(); config["max_runtime_sec"] = None; config["max_boundary_points"] = None
    policy = RuleBasedPolicy(config)
    tick(policy, 0, np.zeros(6))
    assert tick(policy, 1000, np.zeros(6)).state == "TARGET_SEARCH"
    for key, value in [("probe_speed", 0), ("follow_hand", "RANDOM"),
                       ("recovery_sector_extents_deg", [50, 20]), ("search_direction_xy", [0, 0])]:
        with pytest.raises(ValueError):
            RuleBasedPolicy({**config, key: value})


def test_initialization_fits_three_same_speed_probes_excluding_biased_search():
    config = policy_config()
    config.update(search_speed=.009, probe_speed=.003, local_fit_max_residual=.0006)
    policy = RuleBasedPolicy(config)
    origin = np.zeros(6)
    tick(policy, 0, origin)
    acquisition = policy.active_episode
    # SEARCH triggers 2 mm deeper than subsequent low-speed probes.
    first = origin.copy()
    first[0] = .020
    acquisition.tick(.1, first, Wrench(1, 0, 0, 0, 0, 0), config)
    acquisition.tick(.2, first, Wrench(1, 0, 0, 0, 0, 0), config)
    local_anchor = acquisition.return_target_pose.copy()
    settle_return(acquisition, .3, local_anchor, config)
    policy.active_episode = None
    policy._dispatch_completed(acquisition, acquisition.return_end_time, local_anchor)
    assert not policy._initial_contacts
    now, pose = .4, local_anchor.copy()
    local_episodes = []
    for index in range(3):
        # Advance the force-guarded anchor transfers to their endpoints.
        while policy._motion_queue:
            pose = policy._motion_queue[0][0].copy()
            policy._execute_transfer(now, ZERO, pose)
            now += .1
        episode = policy.active_episode
        assert episode.purpose == "INITIALIZATION"
        motion = episode.tick(now, pose, ZERO, config)
        assert motion[1] == .003
        contact = pose.copy()
        contact[0] = .018
        episode.tick(now + .1, contact, Wrench(1, 0, 0, 0, 0, 0), config)
        episode.tick(now + .2, contact, Wrench(1, 0, 0, 0, 0, 0), config)
        assert not episode.return_completed
        assert not policy.boundary_points
        settle_return(episode, now + .3, pose, config)
        policy.active_episode = None
        policy._dispatch_completed(episode, episode.return_end_time, pose)
        local_episodes.append(episode)
        now += .5
        if index < 2:
            assert policy.state == State.LOCAL_INITIALIZATION
            assert not policy.boundary_points
    assert policy.state == State.BOUNDARY_TRACKING
    assert len(policy.boundary_points) == 3
    assert not acquisition.accepted_as_boundary
    assert all(e.return_completed and e.accepted_as_boundary for e in local_episodes)
    assert all(p.pose[0] == pytest.approx(.018) for p in policy.boundary_points)
    assert abs(policy.current_tangent[0]) < 1e-10
    assert np.allclose(
        [e.anchor_pose[1] for e in local_episodes],
        [-config['initialization_lateral_offset'], 0, config['initialization_lateral_offset']],
    )


@pytest.mark.parametrize("xy,diagnostic", [
    ([[0, 0], [0, .003]], "contacts=2/3"),
    ([[0, 0], [0, 0], [0, .003]], "min_spacing=0.000 mm"),
    # The three measured contacts from the failed hardware run, in metres.
    ([[.6147459857648001, .11979058644795292],
      [.6134401759379375, .12176287790694063],
      [.6162672370264982, .12192680460032904]], "residual=0.951 mm"),
])
def test_invalid_initialization_stops_with_measured_diagnostics(xy, diagnostic):
    policy = RuleBasedPolicy({**policy_config(), "local_fit_max_residual": .0006,
                              "initialization_max_retries": 0})
    contacts = [SimpleNamespace(contact_pose=np.r_[point, np.zeros(4)], rejection_reason="")
                for point in xy]
    policy._initial_contacts = contacts
    policy._finish_initialization(1.0, np.zeros(6))
    assert policy.state == State.STOP
    assert not policy.boundary_points
    assert diagnostic in policy.reason
    assert "limit=0.600 mm" in policy.reason
    assert all(e.rejection_reason == policy.reason for e in contacts)


def initialization_policy(**settings):
    config = {**policy_config(), "local_fit_max_residual": .0006,
              "initialization_lateral_offset": .0015, **settings}
    policy = RuleBasedPolicy(config)
    acquisition = ProbeEpisode(0, np.zeros(6), [1, 0], 0, .05, "TARGET_SEARCH", "ACQUISITION")
    contact = np.array([.02, 0, 0, 0, 0, 0])
    force = Wrench(1, 0, 0, 0, 0, 0)
    acquisition.tick(.01, contact, force, config)
    acquisition.tick(.061, contact, force, config)
    settle_return(acquisition, .1, acquisition.return_target_pose.copy(), config)
    policy.probe_episodes.append(acquisition)
    policy._acquired(acquisition, acquisition.return_end_time, acquisition.returned_pose)
    return policy


def complete_initial_round(policy, depths, now=1.0):
    episodes = []
    pose = np.zeros(6)
    existing_points = len(policy.boundary_points)
    for depth in depths:
        while policy._motion_queue:
            pose = policy._motion_queue[0][0].copy()
            policy._execute_transfer(now, ZERO, pose)
            now += .1
        episode = policy.active_episode
        assert episode.purpose == "INITIALIZATION"
        contact = pose.copy()
        contact[:2] += episode.probe_direction * ((depth - pose[0]) / episode.probe_direction[0])
        force = Wrench(1, 0, 0, 0, 0, 0)
        episode.tick(now, contact, force, policy.config)
        episode.tick(now + .1, contact, force, policy.config)
        # The next round/decision cannot start until the probe returns.
        assert episode.phase == "RETURN"
        assert len(policy.boundary_points) == existing_points
        settle_return(episode, now + .2, pose, policy.config)
        policy.active_episode = None
        policy._dispatch_completed(episode, episode.return_end_time, pose)
        episodes.append(episode)
        now += .3
    return episodes, now, pose


def test_failed_initialization_retraces_anchors_and_recovers_without_accepting_bad_batch():
    policy = initialization_policy()
    failed, now, _ = complete_initial_round(policy, [.018, .020, .018])
    assert policy.state == State.LOCAL_INITIALIZATION
    assert "INITIALIZATION_RETRY 1/3" in policy.reason
    assert not policy.boundary_points and policy.current_tangent is None
    assert all(e.return_completed and e.rejection_reason for e in failed)
    rollback = [p for p, _ in policy._motion_queue]
    expected = list(reversed(policy._initial_anchor_path[:-1]))
    assert len(rollback) == len(expected) == 3
    assert all(np.array_equal(a, b) for a, b in zip(rollback, expected))
    fresh, _, _ = complete_initial_round(policy, [.018, .018, .018], now)
    assert policy.state == State.BOUNDARY_TRACKING
    assert all(e.accepted_as_boundary for e in fresh)
    assert all(not e.accepted_as_boundary for e in failed)
    assert [e.initialization_round for e in fresh] == [2, 2, 2]
    assert fresh[0].to_record()["initialization_round"] == 2
    assert len(policy.boundary_points) == 3


def test_retries_move_to_one_side_to_avoid_mixing_two_faces():
    policy = initialization_policy()
    _, now, _ = complete_initial_round(policy, [.018, .020, .018])
    _, now, _ = complete_initial_round(policy, [.018, .020, .018], now)
    fresh, _, _ = complete_initial_round(policy, [.018, .018, .018], now)
    assert policy.state == State.BOUNDARY_TRACKING
    assert [e.initialization_round for e in fresh] == [3, 3, 3]
    assert np.allclose([e.anchor_pose[1] for e in fresh], [.0015, .003, .0045])
    assert len(policy.boundary_points) == 3


def test_initialization_retries_exhaust_after_twelve_local_probes_at_returned_anchor():
    policy = initialization_policy()
    now = 1.0
    for _ in range(4):
        episodes, now, pose = complete_initial_round(policy, [.018, .020, .018], now)
    assert policy.state == State.STOP
    assert "retries exhausted" in policy.reason
    assert not policy.boundary_points and not policy._motion_queue
    assert len(policy.probe_episodes) == 13  # acquisition + 12 local probes
    assert all(e.return_completed for e in policy.probe_episodes)
    assert np.array_equal(pose, episodes[-1].anchor_pose)
    assert np.allclose([e.anchor_pose[1] for e in episodes], [-.0045, -.003, -.0015])


def test_initialization_no_contact_batch_retries_without_reusing_acquisition():
    policy = initialization_policy()
    now = 1.0
    for _ in range(3):
        while policy._motion_queue:
            pose = policy._motion_queue[0][0].copy()
            policy._execute_transfer(now, ZERO, pose)
            now += .1
        episode = policy.active_episode
        end = pose.copy()
        end[:2] += episode.probe_direction * episode.max_probe_distance
        episode.tick(now, end, ZERO, policy.config)
        settle_return(episode, now + .1, pose, policy.config)
        policy.active_episode = None
        policy._dispatch_completed(episode, episode.return_end_time, pose)
        now += .2
    assert policy.state == State.LOCAL_INITIALIZATION
    assert "contacts=0/3" in policy.reason
    assert "INITIALIZATION_RETRY" in policy.reason
    assert not policy.boundary_points


@pytest.mark.parametrize("trigger", ["contact", "force", "timeout", "operator"])
def test_initialization_retry_preserves_stop_guards(trigger):
    policy = initialization_policy()
    _, now, pose = complete_initial_round(policy, [.018, .020, .018])
    if trigger == "operator":
        policy.request_stop("operator stop")
    if trigger == "timeout":
        now = policy._initial_started_at + 120.0
    force = Wrench(16 if trigger == "force" else 1, 0, 0, 0, 0, 0) if trigger in {"force", "contact"} else ZERO
    command = tick(policy, now, pose, force)
    assert command.state == "STOP" and not command.move


@pytest.mark.parametrize("settings", [
    {"initialization_max_retries": -1}, {"initialization_max_retries": 4},
    {"initialization_max_retries": 1.5}, {"initialization_timeout_sec": 0},
    {"initialization_timeout_sec": float("nan")},
])
def test_initialization_retry_budgets_are_validated(settings):
    with pytest.raises(ValueError):
        RuleBasedPolicy({**policy_config(), **settings})


def returned_probe(policy, xy, now, *, purpose="TRACKING", direction=(1, 0), outcome="CONTACT"):
    """Inject an observed ray, completing its actual hold/return lifecycle."""
    contact = np.r_[xy, np.zeros(4)]
    direction = np.asarray(direction, dtype=float)
    direction /= np.linalg.norm(direction)
    anchor = contact.copy()
    anchor[:2] -= .02 * direction
    episode = ProbeEpisode(len(policy.probe_episodes), anchor, direction, now,
                           .02, policy.state.value, purpose)
    policy.probe_episodes.append(episode)
    force = Wrench(1, 0, 0, 0, 0, 0) if outcome == "CONTACT" else ZERO
    episode.tick(now, contact, force, policy.config)
    if outcome == "CONTACT":
        episode.tick(now + .1, contact, force, policy.config)
    assert episode.phase == "RETURN" and not episode.return_completed
    settle_return(episode, now + .2, anchor, policy.config)
    assert episode.return_completed and episode.outcome == outcome
    return episode


def tracking_policy():
    """Seed the coordinator with three returned, accepted straight-face probes."""
    policy = RuleBasedPolicy(policy_config())
    contacts = [returned_probe(policy, [.02, y], i)
                for i, y in enumerate((-.006, -.003, 0))]
    estimate = policy.tracker.update([e.contact_pose for e in contacts], [1, 0], reset=True)
    policy._set_estimate(estimate)
    policy._accept(contacts)
    policy.state = State.BOUNDARY_TRACKING
    return policy


def dispatch_returned(policy, episode):
    policy._dispatch_completed(episode, episode.return_end_time, episode.returned_pose)


def test_tracking_no_contact_starts_sector_search_only_after_ray_return():
    policy = tracking_policy()
    episode = returned_probe(policy, [.02, .008], 4, outcome="NO_CONTACT")
    dispatch_returned(policy, episode)
    assert episode.rejection_reason == "NO_CONTACT"
    assert policy.state == State.BOUNDARY_RECOVERY
    assert len(policy.recovery_records) == 1
    assert policy.active_episode.purpose == "RECOVERY"
    assert policy.active_episode.probe_start_time >= episode.return_end_time
    assert np.array_equal(policy.active_episode.anchor_pose, episode.returned_pose)


@pytest.mark.parametrize("xy,reason", [
    ([.024, -.002], "NO_FORWARD_PROGRESS"),
    ([.020, 0], "REPEATED_CONTACT"),
])
def test_tracking_progress_failures_correct_returned_anchor_without_sector_search(xy, reason):
    policy = tracking_policy()
    history = list(policy.boundary_points)
    old_tangent = policy.current_tangent.copy()
    episode = returned_probe(policy, xy, 4)
    dispatch_returned(policy, episode)
    assert episode.rejection_reason == reason
    assert policy.state == State.BOUNDARY_TRACKING
    assert not policy.recovery_records and policy.active_episode is None
    assert policy.boundary_points == history and not episode.accepted_as_boundary
    assert np.dot(policy.current_tangent, old_tangent) > .999
    clearance, anchor = [pose for pose, _ in policy._motion_queue]
    # The first waypoint stays on the returned, observed free ray.
    assert clearance[1] == pytest.approx(episode.anchor_pose[1])
    assert episode.anchor_pose[0] <= clearance[0] < episode.contact_pose[0]
    progress = np.dot(anchor[:2] - history[-1].pose[:2], old_tangent)
    expected = policy.config["tangent_step"] * (1.5 if reason == "REPEATED_CONTACT" else 1)
    assert progress == pytest.approx(expected)


def test_tracking_anchor_corrections_are_bounded_before_local_reinitialization():
    policy = tracking_policy()
    for attempt in range(3):
        episode = returned_probe(policy, [.024, -.002], 4 + attempt)
        dispatch_returned(policy, episode)
        if attempt < 2:
            assert policy.state == State.BOUNDARY_TRACKING
    assert policy.state == State.LOCAL_INITIALIZATION
    assert policy._tracking_retry_counts["NO_FORWARD_PROGRESS"] == 3
    assert "after two anchor corrections" in policy.reason
    assert not policy.recovery_records
    assert all(not e.accepted_as_boundary for e in policy.probe_episodes[3:])


def test_fit_failure_collects_fresh_local_contacts_preserving_history_and_search():
    policy = tracking_policy()
    original_search = policy.search_direction.copy()
    history = list(policy.boundary_points)
    # Enough forward displacement, but incompatible with the previous local face.
    episode = returned_probe(policy, [.030, .005], 4, direction=(1, .2))
    dispatch_returned(policy, episode)
    assert episode.rejection_reason == "FIT_FAILURE"
    assert policy.state == State.LOCAL_INITIALIZATION
    assert not policy.recovery_records and not episode.accepted_as_boundary
    assert np.array_equal(policy.search_direction, original_search)
    assert np.array_equal(policy._initial_direction, episode.probe_direction)
    assert np.array_equal(policy._initial_base_anchor, episode.returned_pose)
    assert policy.boundary_points == history
    fresh, _, _ = complete_initial_round(policy, [.020, .020, .020], 5)
    assert policy.state == State.BOUNDARY_TRACKING
    assert all(e.purpose == "INITIALIZATION" and e.return_completed for e in fresh)
    assert policy.boundary_points[:len(history)] == history
    assert all(e is not episode for e in policy._initial_contacts)
    assert all(p.pose[0] == pytest.approx(.020) for p in policy.boundary_points)
    assert np.array_equal(policy.search_direction, original_search)


def test_local_reinitialization_fits_overlapping_points_without_republishing_them():
    policy = tracking_policy()
    for attempt in range(3):
        episode = returned_probe(policy, [.020, 0], 4 + attempt)
        dispatch_returned(policy, episode)
    assert policy.state == State.LOCAL_INITIALIZATION
    history = list(policy.boundary_points)
    fresh, _, _ = complete_initial_round(policy, [.020, .020, .020], 8)
    assert policy.boundary_points[:3] == history
    assert len(policy.tracker.contacts) == 3
    assert [e.accepted_as_boundary for e in fresh] == [False, False, True]
    assert len(policy.boundary_points) == 4
    assert policy.boundary_points[-1].pose[1] == pytest.approx(.003)
    assert policy.state == State.BOUNDARY_TRACKING


def test_recovery_confirmation_still_rejects_true_tangent_reversal():
    policy = tracking_policy()
    anchor = np.zeros(6)
    policy.recovery = BoundaryRecovery(anchor, policy.current_target_direction,
                                       policy.current_tangent, policy.boundary_points[-1].pose,
                                       policy.config)
    policy.state = State.BOUNDARY_CONFIRMATION
    direction = [-1, -.1]
    first = returned_probe(policy, [.030, -.004], 4, purpose="RECOVERY", direction=direction)
    second = returned_probe(policy, [.030, -.007], 5, purpose="CONFIRMATION", direction=direction)
    policy._confirmation_contacts = [first]
    policy._confirmation_direction = first.probe_direction
    policy._confirmation_anchor_path = [first.anchor_pose, second.anchor_pose]
    policy._confirmed(second, second.return_end_time, second.returned_pose)
    assert first.rejection_reason == second.rejection_reason == "CONFIRMATION_REVERSED"
    assert not first.accepted_as_boundary and not second.accepted_as_boundary
    assert len(policy.boundary_points) == 3
    assert policy._motion_queue[0][1] == "CONFIRMATION_ROLLBACK"


def test_counterclockwise_initialization_retraces_known_anchors_to_published_frontier():
    policy = initialization_policy(follow_hand="COUNTERCLOCKWISE", tangent_step=.003)
    episodes, now, pose = complete_initial_round(policy, [.018, .018, .018])
    frontier = episodes[0]  # Negative Y is forward, opposite the collection order.
    assert np.array_equal(policy.boundary_points[-1].pose, frontier.contact_pose)
    assert np.array_equal(pose, episodes[-1].anchor_pose)
    assert all(phase == "INITIALIZATION_ALIGN" for _, phase in policy._motion_queue)
    assert np.array_equal([p for p, _ in policy._motion_queue],
                          [episodes[1].anchor_pose, episodes[0].anchor_pose])
    assert policy.active_episode is None
    # Tracking is not scheduled until both old anchor-transfer segments are retraced.
    for expected in (episodes[1].anchor_pose, episodes[0].anchor_pose):
        target = policy._motion_queue[0][0]
        assert np.array_equal(target, expected)
        pose = target.copy()
        policy._execute_transfer(now, ZERO, pose)
        now += .1
        assert policy.sub_state == "INITIALIZATION_ALIGN"
        policy._execute_transfer(now, ZERO, pose)
        now += .1
        assert policy.active_episode is None
    assert policy.state == State.BOUNDARY_TRACKING
    assert policy._motion_queue[0][1] == "RAY_CLEARANCE"
    next_anchor = policy._motion_queue[-1][0].copy()
    assert np.dot(next_anchor[:2] - frontier.contact_pose[:2], policy.current_tangent) == pytest.approx(.003)
    while policy._motion_queue:
        pose = policy._motion_queue[0][0].copy()
        policy._execute_transfer(now, ZERO, pose)
        now += .1
    assert policy.active_episode.purpose == "TRACKING"
    assert np.array_equal(policy.active_episode.anchor_pose, next_anchor)
