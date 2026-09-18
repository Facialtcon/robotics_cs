import numpy as np
import pytest

from robot.rtde_controller import (
    RobotError,
    SimulatedController,
    URRTDEController,
    WorkspaceGuard,
    _orientation_distance,
    validate_execution_configuration,
)


def test_simulator_only_integrates_xy():
    robot = SimulatedController([0, 0, 0.3, 1, 2, 3], 0.01)
    robot.connect()
    robot.command_planar_velocity([1, 0], 0.005, 2.0)
    assert np.allclose(robot.read_state().pose, [0.01, 0, 0.3, 1, 2, 3])


def test_workspace_guard_rejects_predicted_escape():
    guard = WorkspaceGuard(
        {"x_min": 0, "x_max": 1, "y_min": 0, "y_max": 1, "z_min": 0.2, "z_max": 0.4},
        0.3,
        0.001,
        0.01,
    )
    with pytest.raises(RobotError, match="outside"):
        guard.check_predicted_pose([0.999, 0.5, 0.3, 0, 0, 0], [1, 0], 0.01, 1.0)


def test_workspace_guard_can_be_disabled_without_valid_limits():
    guard = WorkspaceGuard(
        {"x_min": 0, "x_max": 0, "y_min": 0, "y_max": 0, "z_min": 0, "z_max": 0},
        0.3,
        0.001,
        0.01,
        workspace_enabled=False,
    )
    guard.check_workspace([100, -100, 50, 0, 0, 0])


def test_orientation_distance_handles_equivalent_pi_rotation_vectors():
    assert _orientation_distance([0, np.pi, 0], [0, -np.pi, 0]) < 1e-7


def test_controller_tcp_verification_falls_back_to_control_interface():
    class Control:
        def getTCPOffset(self):
            return [0.0, 0.0, 0.25, 0.0, 0.0, 0.0]

    controller = object.__new__(URRTDEController)
    controller.receive = object()
    controller.control = Control()
    controller.config = {
        "tcp_offset": [0.0, 0.0, 0.25, 0.0, 0.0, 0.0],
        "tcp_offset_tolerance": 1e-4,
        "allow_unverified_active_tcp": False,
    }
    controller._verify_tcp()


def test_execution_configuration_does_not_require_confirmation_flags():
    config = {
        "execution": {"allow_robot_motion": False},
        "workspace": {
            "confirmed": False,
            "limits": {
                "x_min": 0.0,
                "x_max": 1.0,
                "y_min": 0.0,
                "y_max": 1.0,
                "z_min": 0.0,
                "z_max": 1.0,
            },
        },
        "tcp": {"confirmed": False, "offset": [0, 0, 0, 0, 0, 0]},
        "preprocessing": {"coordinate_transform": {"confirmed": False}},
        "policy": {"thresholds_confirmed": False, "probe_direction_confirmed": False},
        "safe_return": {"confirmed": False},
        "robot": {"max_tcp_speed": 0.01},
    }
    validate_execution_configuration(config)
