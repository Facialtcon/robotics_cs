"""Point-TCP kinematics with the same command interface used by the policy loop."""

from __future__ import annotations

import numpy as np

from core.models import RobotState


class SimulatedRobot:
    def __init__(self, start_point, dt: float, workspace: dict):
        self.start_point = np.asarray(start_point, dtype=float)
        if self.start_point.shape != (2,) or dt <= 0.0:
            raise ValueError("start_point must be XY and dt must be positive")
        self.dt = float(dt)
        self.workspace = {key: float(value) for key, value in workspace.items()}
        self.reset()

    def reset(self) -> None:
        self.pose = np.asarray((self.start_point[0], self.start_point[1], 0.0, 0.0, 0.0, 0.0))
        self.tcp_speed = np.zeros(6)
        self.time = 0.0

    def read_state(self) -> RobotState:
        return RobotState(self.time, self.pose.copy(), self.tcp_speed.copy())

    def apply_command(self, direction_xy, speed: float, move: bool) -> None:
        self.tcp_speed[:] = 0.0
        if move:
            direction = np.asarray(direction_xy, dtype=float)
            norm = float(np.linalg.norm(direction))
            if norm <= 0.0 or speed <= 0.0:
                raise ValueError("moving command requires nonzero direction and speed")
            self.tcp_speed[:2] = direction / norm * float(speed)
            self.pose[:2] += self.tcp_speed[:2] * self.dt
        self.time += self.dt

    def inside_workspace(self) -> bool:
        return (
            self.workspace["x_min"] <= self.pose[0] <= self.workspace["x_max"]
            and self.workspace["y_min"] <= self.pose[1] <= self.workspace["y_max"]
        )
