"""Real configuration speed routing checks; no device interfaces are created."""
from pathlib import Path

import numpy as np
import pytest

from config.loader import load_config, runtime_robot_config
from core.models import Wrench
from policy.probe_episode import ProbeEpisode
from policy.rule_policy import RuleBasedPolicy
from safety.safe_return import validate_return_configuration


ROOT = Path(__file__).resolve().parents[1]
ZERO = Wrench(0, 0, 0, 0, 0, 0)


def test_real_scan_and_return_speed_profile_fits_runtime_controller_limit():
    config = load_config(ROOT / "config.yaml")
    expected = {
        "search_speed": .018, "probe_speed": .006, "tangent_speed": .012,
        "retract_speed": .012, "recovery_probe_speed": .006,
    }
    assert {key: config["policy"][key] for key in expected} == expected
    assert config["safe_return"]["return_speed"] == .030
    assert config["safe_return"]["return_vertical_speed"] == .018
    runtime = runtime_robot_config(config)
    assert runtime["max_tcp_speed"] == .030
    assert max(*expected.values(), config["safe_return"]["return_speed"],
               config["safe_return"]["return_vertical_speed"]) <= runtime["max_tcp_speed"]
    assert runtime["speed_acceleration"] == .050
    assert runtime["stop_deceleration"] == .200
    assert config["safe_return"]["return_acceleration"] == .030
    assert config["policy"]["position_tolerance"] == .0002
    assert config["policy"]["contact_threshold"] == 1.0
    assert config["policy"]["control_rate_hz"] == 100.0
    validate_return_configuration(config, config["dry_run"]["start_pose"])
    assert RuleBasedPolicy(config["policy"]).config == config["policy"]


@pytest.mark.parametrize("purpose,expected_speed", [
    ("ACQUISITION", .018), ("INITIALIZATION", .006), ("TRACKING", .006),
    ("CONFIRMATION", .006), ("RECOVERY", .006),
])
def test_episode_uses_real_configured_probe_speed(purpose, expected_speed):
    config = load_config(ROOT / "config.yaml")["policy"]
    pose = np.zeros(6)
    episode = ProbeEpisode(0, pose, [1, 0], 0, .01, "BOUNDARY_TRACKING", purpose)
    target, speed = episode.tick(0, pose, ZERO, config, tcp_speed=np.zeros(6))
    assert speed == expected_speed
    assert np.array_equal(target[:2], [.01, 0])
    episode.phase = "RETURN"
    displaced = pose.copy()
    displaced[0] = .005
    target, speed = episode.tick(.1, displaced, ZERO, config, tcp_speed=np.zeros(6))
    assert speed == .012
    assert np.array_equal(target, pose)
