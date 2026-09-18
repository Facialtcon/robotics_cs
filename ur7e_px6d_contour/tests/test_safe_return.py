import numpy as np

from core.models import RobotState, Wrench
from safety.safe_return import (
    SafeReturnExecutor,
    calculate_safe_return_z,
    validate_return_configuration,
)
from tools.visualize_run import STATE_COLORS, STATE_ORDER


class FakeReader:
    def __init__(self, wrench=None):
        self.wrench = wrench or Wrench(0, 0, 0, 0, 0, 0)

    def read_wrench(self):
        return self.wrench


class FakeController:
    def __init__(self, pose, require_return_mode=False):
        self.pose = np.asarray(pose, dtype=float)
        self.target = None
        self.moves = []
        self.stop_count = 0
        self.return_mode = False
        self.require_return_mode = require_return_mode

    def safe_stop_motion(self, force=False):
        del force
        self.stop_count += 1

    def read_state(self):
        if self.require_return_mode and not self.return_mode:
            raise RuntimeError("fixed-pose scan guard is active")
        if self.target is not None:
            self.pose = self.target.copy()
            self.target = None
        return RobotState(0.0, self.pose.copy(), np.zeros(6))

    def begin_return_mode(self):
        self.return_mode = True

    def end_return_mode(self):
        self.return_mode = False

    def move_linear_async(self, target, speed, acceleration):
        self.moves.append((np.asarray(target).copy(), speed, acceleration))
        self.target = np.asarray(target).copy()


def return_config():
    return {
        "robot": {"max_tcp_speed": 0.01},
        "workspace": {
            "enabled": False,
            "limits": {
                "x_min": -1, "x_max": 1, "y_min": -1, "y_max": 1,
                "z_min": 0, "z_max": 1,
            }
        },
        "policy": {
            "control_rate_hz": 100,
            "absolute_raw_force_threshold": 50,
            "absolute_raw_torque_threshold": 5,
        },
        "safe_return": {
            "return_lift_distance": 0.03,
            "return_speed": 0.005,
            "return_vertical_speed": 0.003,
            "return_acceleration": 0.03,
            "return_force_limit": 8,
            "return_torque_limit": 0.7,
            "return_position_tolerance": 0.0005,
            "return_orientation_tolerance": 0.01,
            "return_segment_timeout_sec": 1,
            "auto_return_after_normal_stop": True,
        },
    }


def test_safe_return_uses_vertical_horizontal_vertical_segments():
    config = return_config()
    p0 = np.asarray([0.1, 0.2, 0.2, 0, 3.14, 0])
    controller = FakeController([0.4, -0.2, 0.2, 0, 3.14, 0])
    result = SafeReturnExecutor(
        config, p0, controller, FakeReader(), preprocessor=None
    ).execute()
    assert result.status == "complete"
    assert len(controller.moves) == 3
    assert np.allclose(controller.moves[0][0][:3], [0.4, -0.2, 0.23])
    assert np.allclose(controller.moves[1][0], [0.1, 0.2, 0.23, 0, 3.14, 0])
    assert np.allclose(controller.moves[2][0], p0)
    assert not controller.return_mode


def test_startup_return_enters_return_mode_before_reading_away_pose():
    config = return_config()
    p0 = np.asarray([0.1, 0.2, 0.2, 0, 0, 0])
    controller = FakeController([0.4, -0.2, 0.2, 0, 0, 0], require_return_mode=True)
    result = SafeReturnExecutor(config, p0, controller, FakeReader(), None).execute()
    assert result.status == "complete"
    assert not controller.return_mode


def test_resume_from_safe_height_does_not_stack_another_lift():
    assert calculate_safe_return_z(0.128, 0.093, 0.03, 0.0005) == 0.128
    assert np.isclose(calculate_safe_return_z(0.098, 0.093, 0.03, 0.0005), 0.128)


def test_return_from_below_target_clears_target_before_horizontal_motion():
    assert np.isclose(calculate_safe_return_z(0.10, 0.20, 0.03, 0.0005), 0.23)


def test_resume_from_safe_height_skips_zero_distance_vertical_move():
    config = return_config()
    p0 = np.asarray([0.1, 0.2, 0.2, 0, 0, 0])
    controller = FakeController([0.4, -0.2, 0.24, 0, 0, 0])
    result = SafeReturnExecutor(config, p0, controller, FakeReader(), None).execute()
    assert result.status == "complete"
    assert len(controller.moves) == 2
    assert np.allclose(controller.moves[0][0], [0.1, 0.2, 0.24, 0, 0, 0])


def test_force_limit_aborts_without_forcing_remaining_segments():
    config = return_config()
    p0 = np.asarray([0.1, 0.2, 0.2, 0, 0, 0])
    controller = FakeController([0.4, -0.2, 0.2, 0, 0, 0])
    result = SafeReturnExecutor(
        config, p0, controller, FakeReader(Wrench(9, 0, 0, 0, 0, 0)), None
    ).execute()
    assert result.status == "aborted"
    assert "force limit" in result.abort_reason
    assert len(controller.moves) == 0
    assert controller.stop_count >= 2


def test_return_configuration_requires_positive_lift_distance():
    config = return_config()
    config["safe_return"]["return_lift_distance"] = 0.0
    try:
        validate_return_configuration(config, [0, 0, 0.2, 0, 0, 0])
    except Exception as exc:
        assert "must be positive" in str(exc)
    else:
        raise AssertionError("invalid safe return lift distance was accepted")


def test_visualizer_knows_every_return_state():
    assert set(STATE_ORDER) <= set(STATE_COLORS)
