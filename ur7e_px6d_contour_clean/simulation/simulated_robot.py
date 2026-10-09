"""Point-TCP kinematics with the same command interface used by the policy loop."""

from __future__ import annotations

import numpy as np
from collections import deque

from core.models import RobotState


class SimulatedRobot:
    def __init__(self, start_point, dt: float, workspace: dict, *, acceleration_limit=None, delay_steps=0):
        self.start_point = np.asarray(start_point, dtype=float)
        if self.start_point.shape != (2,) or dt <= 0.0:
            raise ValueError("start_point must be XY and dt must be positive")
        self.dt = float(dt)
        self.workspace = {key: float(value) for key, value in workspace.items()}
        if acceleration_limit is not None and (not np.isfinite(acceleration_limit) or acceleration_limit <= 0):
            raise ValueError("acceleration_limit must be positive m/s^2")
        if not isinstance(delay_steps, int) or delay_steps < 0:
            raise ValueError("delay_steps must be a nonnegative integer")
        self.record_commands = False
        self.acceleration_limit = acceleration_limit
        self.delay_steps = delay_steps
        self.reset()

    def reset(self) -> None:
        self.pose = np.asarray((self.start_point[0], self.start_point[1], 0.0, 0.0, 0.0, 0.0))
        self.tcp_speed = np.zeros(6)
        self.time = 0.0
        self.command_records = deque()
        self.command_sequence = 0
        self._pending = deque(np.zeros(2) for _ in range(self.delay_steps))

    def read_state(self) -> RobotState:
        return RobotState(self.time, self.pose.copy(), self.tcp_speed.copy())

    def apply_command(self, direction_xy, speed: float, move: bool) -> None:
        desired = np.zeros(2)
        if move:
            direction = np.asarray(direction_xy, dtype=float)
            norm = float(np.linalg.norm(direction))
            if norm <= 0.0 or speed <= 0.0:
                raise ValueError("moving command requires nonzero direction and speed")
            desired = direction / norm * float(speed)
        self.command_sequence += 1
        if self.record_commands:
            self.command_records.append(dict(sequence=self.command_sequence, kind='simulation',
                velocity=[*desired, 0., 0., 0., 0.], host_monotonic=self.time,
                return_monotonic=self.time, accepted=True, source='simulation.apply_command'))
        self._pending.append(desired)
        desired = self._pending.popleft()
        delta = desired - self.tcp_speed[:2]
        if self.acceleration_limit is not None:
            limit = self.acceleration_limit * self.dt
            if np.linalg.norm(delta) > limit:
                delta *= limit / np.linalg.norm(delta)
        self.tcp_speed[:2] += delta
        self.pose[:2] += self.tcp_speed[:2] * self.dt
        self.time += self.dt

    def inside_workspace(self) -> bool:
        return (
            self.workspace["x_min"] <= self.pose[0] <= self.workspace["x_max"]
            and self.workspace["y_min"] <= self.pose[1] <= self.workspace["y_max"]
        )
