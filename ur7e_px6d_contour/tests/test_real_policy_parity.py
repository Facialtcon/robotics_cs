"""Shared-policy integration and real-tolerance recovery checks, without hardware."""

from copy import deepcopy
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

from app import main as real_app
from calibration.scan_calibration import build_scan_calibration
from config.loader import load_config
from core.models import RobotState, Wrench
from policy.local_recovery import BoundaryRecovery
from policy.probe_episode import ProbeEpisode
from policy.rule_policy import RuleBasedPolicy, State
from simulation import simulator


ROOT = Path(__file__).resolve().parents[1]
ZERO = Wrench(0, 0, 0, 0, 0, 0)
CONTACT = Wrench(2, 0, 0, 0, 0, 0)


def _settle_return(episode, now, pose, config, sample=None):
    """Keep measured pose, velocity and force ready for the real-config hold."""
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


def test_real_and_simulator_import_the_identical_policy_class():
    assert real_app.RuleBasedPolicy is simulator.RuleBasedPolicy is RuleBasedPolicy


def test_real_entry_preserves_policy_parameters_except_calibrated_search(monkeypatch, tmp_path):
    """Reach the real policy constructor, stopping before any device connection."""
    source = load_config(ROOT / "config.yaml")
    original = deepcopy(source)
    config_path = tmp_path / "config.yaml"
    config_path.write_text(yaml.safe_dump(source), encoding="utf-8")
    initial_file = config_path.read_bytes()
    start = [.4, -.2, .3, 0, 3.14, 0]
    reference = [.43, -.16, .3, 0, 3.14, 0]
    calibration = build_scan_calibration(
        start, reference, source["robot"]["robot_ip"],
        active_tcp_offset=source["tcp"]["offset"],
    )
    (tmp_path / source["calibration"]["file"]).write_text(
        yaml.safe_dump(calibration), encoding="utf-8"
    )
    captured = {}

    class PolicyConstructed(Exception):
        pass

    class DisconnectedDevice:
        def __init__(self, *args, **kwargs):
            pass

        def connect(self, *args, **kwargs):
            pytest.fail("configuration inspection must not connect any hardware")

    class DisconnectedController(DisconnectedDevice):
        def __init__(self, config):
            captured["robot"] = deepcopy(config)

    def capture_policy(config):
        captured["policy"] = deepcopy(config)
        raise PolicyConstructed

    monkeypatch.setattr(real_app, "PX6DReader", DisconnectedDevice)
    monkeypatch.setattr(real_app, "URRTDEController", DisconnectedController)
    monkeypatch.setattr(real_app, "RuleBasedPolicy", capture_policy)
    monkeypatch.setattr(sys, "argv", [
        "main.py", "--config", str(config_path), "--sensor", "real", "--execute",
    ])
    with pytest.raises(PolicyConstructed):
        real_app.run(real_app.parse_args())

    expected = deepcopy(original["policy"])
    expected["search_direction_xy"] = captured["policy"]["search_direction_xy"]
    assert captured["policy"] == expected
    assert captured["policy"]["search_direction_xy"] == pytest.approx([.6, .8])
    assert captured["robot"]["fixed_z"] == start[2]
    assert captured["robot"]["fixed_orientation"] == start[3:]
    assert source == original
    assert config_path.read_bytes() == initial_file


def _tracking_policy():
    """Three observed, returned contacts using the unmodified real policy config."""
    policy = RuleBasedPolicy(load_config(ROOT / "config.yaml")["policy"])
    episodes = []
    for index, y in enumerate((.006, .003, 0)):
        anchor = np.array([0, y, .3, 0, 3.14, 0])
        contact = anchor.copy()
        contact[0] += .02
        episode = ProbeEpisode(index, anchor, [1, 0], index, .02,
                               State.LOCAL_INITIALIZATION.value, "INITIALIZATION")
        episode.tick(index, contact, CONTACT, policy.config)
        episode.tick(index + .1, contact, CONTACT, policy.config)
        _settle_return(episode, index + .2, anchor, policy.config)
        assert episode.return_completed
        episodes.append(episode)
    policy.probe_episodes.extend(episodes)
    estimate = policy.tracker.update([e.contact_pose for e in episodes], [1, 0], reset=True)
    policy._set_estimate(estimate)
    policy._accept(episodes)
    policy.state = State.BOUNDARY_TRACKING
    return policy


def _update(policy, now, pose, tcp_speed=None):
    speed = np.zeros(6) if tcp_speed is None else np.asarray(tcp_speed)
    return policy.update(now, ZERO, ZERO, RobotState(now, pose.copy(), speed))


def _settle_transfer(policy, now, pose):
    """Retain the fixed queue target until consecutive actual zero-speed samples."""
    pending = policy._motion_queue[0]
    cursor = policy.recovery.cursor
    period = 1.0 / policy.config["control_rate_hz"]
    hold = policy.config.get("return_settle_hold_sec", policy.config["contact_hold_time"])
    for index in range(int(np.ceil(hold / period)) + 2):
        timestamp = now + index * period
        assert not _update(policy, timestamp, pose, tcp_speed=np.zeros(6)).move
        advanced = not policy._motion_queue or policy._motion_queue[0] is not pending
        if index == 0:
            assert not advanced
        if advanced:
            assert timestamp - now >= hold
            assert any(point.timestamp == timestamp and point.event_type == pending[1] + "_COMPLETED"
                       for point in policy.policy_waypoints)
            return timestamp
        assert policy.active_episode is None
        assert policy.recovery.cursor == cursor
        assert policy._motion_queue[0] is pending
    pytest.fail("consecutive settled samples did not complete anchor transfer")


def test_real_tolerance_recovery_returns_do_not_accumulate_anchor_error():
    policy = _tracking_policy()
    anchor = np.array([.017, -.003, .3, 0, 3.14, 0])
    policy._enter_recovery(4, anchor)
    tolerance = policy.config["position_tolerance"]
    assert tolerance == pytest.approx(.0002)
    offset = np.array([.75 * tolerance, 0, 0, 0, 0, 0])
    completed = []
    for index in range(8):
        episode = policy.active_episode
        assert episode.purpose == "RECOVERY"
        assert np.array_equal(episode.anchor_pose, anchor)
        endpoint = episode.anchor_pose.copy()
        endpoint[:2] += episode.probe_direction * episode.max_probe_distance
        now = 5 + index
        _update(policy, now, endpoint)
        assert episode.phase == "RETURN" and episode.outcome == "NO_CONTACT"
        returned = episode.anchor_pose + offset
        _settle_return(episode, now + .1, returned, policy.config,
                       sample=lambda timestamp: _update(policy, timestamp, returned))
        assert episode.return_completed
        assert np.array_equal(episode.returned_pose, anchor + offset)
        assert np.array_equal(policy.active_episode.anchor_pose, anchor)
        completed.append(episode)
    assert len(policy.boundary_recovery_rays) == len(completed)
    assert np.array_equal(policy.recovery.anchor, anchor)
    assert not policy._motion_queue


def test_recovery_waits_for_fixed_anchor_alignment_before_consuming_a_ray():
    policy = _tracking_policy()
    anchor = np.array([.017, -.003, .3, 0, 3.14, 0])
    policy.recovery = BoundaryRecovery(
        anchor, policy.current_target_direction, policy.current_tangent,
        policy.boundary_points[-1].pose, policy.config,
    )
    policy.recovery_records.append({})
    tolerance = policy.config["position_tolerance"]
    outside = anchor.copy()
    outside[0] += 2 * tolerance
    policy._next_recovery_probe(4, outside)
    assert policy.active_episode is None
    assert policy.recovery.cursor == 0
    assert policy._motion_queue[0][1] == "RECOVERY_ANCHOR_ALIGN"
    assert np.array_equal(policy._motion_queue[0][0], anchor)
    command = _update(policy, 4.1, outside)
    assert command.move and command.direction_xy[0] < 0
    assert policy.recovery.cursor == 0
    assert policy.active_episode is None
    arrived = anchor.copy()
    arrived[0] += .5 * tolerance
    # At-position motion must stop before the recovery ray can consume its
    # direction. The following hold uses separate measured zero-speed frames.
    moving = np.array([policy.config["tangent_speed"], 0, 0, 0, 0, 0])
    command = _update(policy, 4.19, arrived, tcp_speed=moving)
    assert not command.move and command.speed == 0
    assert policy.active_episode is None and policy.recovery.cursor == 0
    assert np.array_equal(policy._motion_queue[0][0], anchor)
    _settle_transfer(policy, 4.2, arrived)
    assert not policy._motion_queue
    assert policy.recovery.cursor == 1
    assert np.array_equal(policy.active_episode.anchor_pose, anchor)


def test_confirmation_rollback_realigns_to_the_same_recovery_anchor():
    policy = _tracking_policy()
    anchor = np.array([.017, -.003, .3, 0, 3.14, 0])
    policy._enter_recovery(4, anchor)
    first = policy.active_episode
    contact = anchor.copy()
    contact[:2] += .004 * first.probe_direction
    first.tick(4.1, contact, CONTACT, policy.config)
    first.tick(4.2, contact, CONTACT, policy.config)
    offset = np.array([.00015, 0, 0, 0, 0, 0])
    _settle_return(first, 4.3, anchor + offset, policy.config)
    assert first.return_completed
    policy.active_episode = None
    policy._dispatch_completed(first, first.return_end_time, first.returned_pose)
    assert policy.state == State.BOUNDARY_CONFIRMATION
    confirmation_anchor = policy._motion_queue[0][0].copy()
    _settle_transfer(policy, 4.4, confirmation_anchor)
    confirmation = policy.active_episode
    assert confirmation.purpose == "CONFIRMATION"
    endpoint = confirmation.anchor_pose.copy()
    endpoint[:2] += confirmation.probe_direction * confirmation.max_probe_distance
    _update(policy, 4.5, endpoint)
    _settle_return(confirmation, 4.6, confirmation.anchor_pose, policy.config,
                   sample=lambda timestamp: _update(policy, timestamp, confirmation.anchor_pose))
    assert confirmation.return_completed and confirmation.outcome == "NO_CONTACT"
    assert policy._motion_queue[0][1] == "CONFIRMATION_ROLLBACK"
    assert np.array_equal(policy._motion_queue[0][0], first.returned_pose)
    cursor = policy.recovery.cursor

    # Returning within tolerance of the confirmation's observed start can
    # still leave the TCP outside tolerance of the frozen recovery anchor.
    _settle_transfer(policy, 4.7, first.returned_pose + offset)
    assert policy.active_episode is None
    assert policy.recovery.cursor == cursor
    assert policy._motion_queue[0][1] == "RECOVERY_ANCHOR_ALIGN"
    _settle_transfer(policy, 4.8, anchor + offset)
    assert policy.recovery.cursor == cursor + 1
    assert policy.active_episode.purpose == "RECOVERY"
    assert np.array_equal(policy.active_episode.anchor_pose, anchor)
