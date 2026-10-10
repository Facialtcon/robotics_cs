"""Offline adapter around the existing acceleration-limited SimulatedRobot."""
from copy import deepcopy

import numpy as np

from calibration.single_point import set_reference, save_group
from core.models import Wrench
from simulation.simulated_robot import SimulatedRobot


def demo_calibration(project):
    pose = [0.4, 0., .2, np.pi, 0., 0.]
    data = set_reference({}, pose, project['robot']['robot_ip'], project['tcp']['offset'])
    for name, xy in [('A', [.39, 0.]), ('B', [.4, -.01]), ('C', [.41, 0.])]:
        start = deepcopy(pose); start[:2] = xy
        data = save_group(data, name, start)
    return data


class OfflineController:
    is_dry_run = True
    watchdog_active = False
    def __init__(self, pose, settings):
        self.s = settings
        self.model = SimulatedRobot(pose[:2], 1/settings['control_rate_hz'],
            dict(x_min=-2., x_max=2., y_min=-2., y_max=2.),
            acceleration_limit=settings.get('speed_acceleration_mps2', .002))
        self.model.pose[:] = pose
        self.model.record_commands = True
        self.command_records = self.model.command_records
        self.desired = np.zeros(2)
        self.stop_report = None
        self.standstill_confirmed = False
        self.low_since = None
        self.observation_timing = {}
    def clock(self): return self.model.time
    def read_state(self): return self.model.read_state()
    read_diagnostic_state = read_state
    def sleep(self, duration):
        # Fixed simulation ticks, with acceleration-limited motion and braking.
        self.model.dt = max(duration, 1/self.s['control_rate_hz'])
        if self.stop_report is not None:
            self.model.acceleration_limit = self.s.get('stop_deceleration_mps2', .002)
        direction = self.desired if np.linalg.norm(self.desired) else np.array([1., 0.])
        # Physics integration doesn't invent extra controller-send records.
        self.model.record_commands = False
        self.model.apply_command(direction, np.linalg.norm(self.desired), bool(np.linalg.norm(self.desired)))
        self.model.record_commands = True
    def _command(self, velocity, kind):
        self.desired = np.asarray(velocity)
        self.model.command_sequence += 1
        self.command_records.append(dict(sequence=self.model.command_sequence, kind=kind,
            velocity=[*self.desired, 0., 0., 0., 0.], host_monotonic=self.clock(),
            return_monotonic=self.clock(), accepted=True, source='simulation.single_point'))
    def command_planar_velocity(self, direction, speed, duration):
        if hasattr(self, 'before_velocity_send'):
            self.before_velocity_send(self.read_state())
        self._command(np.asarray(direction)*speed, 'motion')
    def request_stop(self, *, nonblocking=True):
        if self.stop_report is None:
            self.stop_report = dict(request_host_monotonic=self.clock(), physical_stop='unconfirmed', api_anomaly=False)
            self._command([0., 0.], 'braking')
        return self.stop_report
    def poll_stop(self):
        low = np.linalg.norm(self.model.tcp_speed[:3]) <= self.s['settle_speed_mps']
        self.low_since = (self.clock() if self.low_since is None else self.low_since) if low else None
        self.standstill_confirmed = self.low_since is not None and self.clock()-self.low_since >= self.s['settle_hold_sec']
        if self.standstill_confirmed and self.stop_report:
            self.stop_report['physical_stop'] = 'confirmed'
        return self.standstill_confirmed
    def kick_watchdog(self): pass
    def close(self): pass


class OfflineForceReader:
    def __init__(self, controller, pose, direction, processor, *, contact_distance=.002):
        self.controller, self.start = controller, np.asarray(pose[:2])
        self.direction, self.processor = np.asarray(direction), processor
        self.contact_distance = contact_distance
    def read_wrench(self):
        displacement = float((self.controller.read_state().pose[:2]-self.start) @ self.direction)
        force = max(0., displacement-self.contact_distance)*4000
        base = np.r_[-force*self.direction, 0.]
        sensor = self.processor.rotation_sensor_to_output.T @ base
        return Wrench.from_sequence([*sensor, 0., 0., 0.])
