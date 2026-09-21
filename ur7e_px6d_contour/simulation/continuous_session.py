"""Shared hardware-free assembly and fixed control step. No GUI or real drivers."""
from copy import deepcopy
from dataclasses import dataclass

import numpy as np

from core.models import RobotState, Wrench, PolicyCommand
from policy.continuous_tracking import ContinuousTrackingPolicy
from safety.force_guard import force_safety_reason
from sensor.force_preprocess import WrenchPreprocessor
from simulation.geometry import CircleTarget, create_target
from simulation.simulated_robot import SimulatedRobot
from simulation.simulated_force_sensor import SimulatedForceSensor, PHYSICAL_FORCE_METADATA

SIMULATION_SAMPLE_FIELDS = ('sim_components_available', 'sim_signed_distance_m', 'sim_penetration_m',
                           'sim_compression_m', 'sim_segment_penetration',
                           'sim_object_fx', 'sim_object_fy', 'sim_friction_fx', 'sim_friction_fy',
                           'sim_background_fx', 'sim_background_fy', 'sim_noise_fx', 'sim_noise_fy',
                           'physical_force_available', 'sim_normal_physical_fx', 'sim_normal_physical_fy',
                           'sim_boundary_physical_fx', 'sim_boundary_physical_fy',
                           'sim_environment_physical_fx', 'sim_environment_physical_fy',
                           'sim_robot_estimate_fx', 'sim_robot_estimate_fy')

def validate_scene(scene):
    """Validate an independent copy; never move the target or calibration points."""
    scene = deepcopy(scene)
    def xy(key):
        value = np.asarray(scene[key], dtype=float)
        if value.shape != (2,) or not np.isfinite(value).all():
            raise ValueError(f'{key} must contain two finite XY coordinates [m]')
        return value
    center, start, p0, p1 = (xy(k) for k in
        ('target_center', 'start_point', 'calibration_point_0', 'calibration_point_1'))
    direction = p1-p0
    if np.linalg.norm(direction) < 1e-9:
        raise ValueError('P0/P1 must be distinct')
    scene['scan_direction_xy'] = (direction/np.linalg.norm(direction)).tolist()
    for key in ('target_width', 'target_height', 'target_radius'):
        if key in scene and (not np.isfinite(float(scene[key])) or float(scene[key]) <= 0):
            raise ValueError(f'{key} must be finite and positive [m]')
    if not np.isfinite(float(scene.get('target_rotation_deg', 0))):
        raise ValueError('target rotation must be finite [deg]')
    bounds = scene['container']
    low = np.array([bounds['x_min'], bounds['y_min']], dtype=float)
    high = np.array([bounds['x_max'], bounds['y_max']], dtype=float)
    if not np.isfinite([low, high]).all() or np.any(low >= high):
        raise ValueError('container requires finite ordered XY limits')
    for key, point in (('start_point', start), ('P0', p0), ('P1', p1)):
        if np.any(point < low) or np.any(point > high):
            raise ValueError(f'{key} must be inside container')
    target = create_target(scene)
    # Use exact circle extrema; sampled boundaries can miss a protruding sliver.
    boundary = np.array([center-target.radius, center+target.radius]) if isinstance(target, CircleTarget) else target.boundary_points()
    if np.any(boundary < low) or np.any(boundary > high):
        raise ValueError('entire target must be inside container')
    force = scene['force_model']
    for key in ('probe_tip_radius', 'noise_std', 'contact_stiffness', 'friction_coefficient', 'granular_drag_force'):
        value = float(force.get(key, 0))
        if not np.isfinite(value) or value < 0:
            raise ValueError(f'force_model.{key} must be finite and nonnegative')
    if target.signed_distance_and_outward_normal(start)[0] <= float(force.get('probe_tip_radius', 0)):
        raise ValueError('simulation must start outside the target sensing envelope')
    seed = force.get('random_seed')
    if not isinstance(seed, int) or seed < 0:
        raise ValueError('force_model.random_seed must be a nonnegative integer')
    return scene


@dataclass
class SimulationSample:
    time: float
    robot: RobotState
    raw: Wrench
    processed: Wrench
    command: PolicyCommand
    telemetry: dict
    tangent: np.ndarray
    inward: np.ndarray
    events: tuple
    diagnostics: dict
    simulation_telemetry: dict


class SimulationSession:
    """Each step observes t, commands once, and integrates to t + dt (no snapping)."""
    def __init__(self, config, scene):
        self.config = deepcopy(config)
        self.config['force_display'] = deepcopy(PHYSICAL_FORCE_METADATA)
        self.scene = validate_scene(scene)
        self.dt = 1 / float(config['policy']['control_rate_hz'])
        # Snapshot the actual continuous rate, not the old scene's discrete rate.
        self.scene['simulation']['dt'] = self.dt
        self.scene['policy']['control_rate_hz'] = 1/self.dt
        self.scene['policy']['search_direction_xy'] = self.scene['scan_direction_xy']
        self.config['continuous_simulation'] = self.scene
        self.config['policy']['search_direction_xy'] = self.scene['scan_direction_xy']
        alpha = self.config['preprocessing']['filter_alpha']
        self.config['preprocessing'] = deepcopy(self.scene['preprocessing'])
        self.config['preprocessing']['filter_alpha'] = alpha
        # Geometry remains exclusively in the environment, not policy input.
        self.policy = ContinuousTrackingPolicy({k: self.config[k] for k in ('continuous_tracking', 'policy', 'robot')})
        dynamics = dict(acceleration_limit=.005, delay_steps=2)
        self.config['continuous_simulation_execution'] = dynamics
        self.robot = SimulatedRobot(self.scene['start_point'], self.dt, self.scene['container'], **dynamics)
        self.target = create_target(self.scene)
        self.sensor = SimulatedForceSensor(self.target, self.scene['force_model'])
        self.preprocessor = WrenchPreprocessor.from_config(self.config['preprocessing'])
        self._previous_xy = self.robot.pose[:2].copy()

    def capture_bias(self, poll=lambda: None, observe=lambda **kw: None):
        baseline = self.config['preprocessing'].get('baseline', {'capture_on_start': True, 'sample_count': 100})
        if not baseline.get('capture_on_start', True):
            return
        samples = []
        for _ in range(int(baseline['sample_count'])):
            if poll() in ('Q', 'ESC'):
                raise KeyboardInterrupt
            raw = self.sensor.read_wrench(self.robot.pose[:2], np.zeros(2))[0]
            observe(raw=raw, phase='SENSOR_BIAS')
            if not np.all(np.isfinite(raw.array())):
                raise ValueError('nonfinite bias sample')
            reason = force_safety_reason(raw, self.preprocessor.process(raw), self.config['policy'])
            if reason:
                raise ValueError(reason)
            samples.append(raw)
        self.preprocessor.set_zero_bias(samples)

    def step(self):
        robot = self.robot.read_state()
        now = self.robot.time
        raw, diagnostics = self.sensor.read_wrench(robot.pose[:2], robot.tcp_speed[:2])
        processed = self.preprocessor.process(raw)
        command = self.policy.update(now, raw, processed, robot)
        predicted = robot.pose[:2] + command.direction_xy * command.speed * self.dt
        bounds = self.robot.workspace
        if not (bounds['x_min'] <= predicted[0] <= bounds['x_max'] and
                bounds['y_min'] <= predicted[1] <= bounds['y_max']):
            self.policy.request_stop(now, robot.pose, 'simulated TCP left container/workspace')
            command = self.policy._command(robot.pose)
        self.robot.apply_command(command.direction_xy, command.speed, command.move)
        # Independent audit only. Geometry/component truth never enters policy.
        from simulation.physical_validation import segment_enters_interior
        crossed = segment_enters_interior(self._previous_xy, robot.pose[:2], self.target)
        self._previous_xy = robot.pose[:2].copy()
        simulation_telemetry = dict(zip(SIMULATION_SAMPLE_FIELDS, (
            1, diagnostics['signed_distance'], diagnostics['penetration'], diagnostics['tip_compression'], int(crossed),
            *diagnostics['object_force'], *diagnostics['friction_force'], *diagnostics['background_force'], *diagnostics['noise_force'],
            1, *diagnostics['normal_physical'], *diagnostics['boundary_physical'],
            *diagnostics['environment_physical'], *diagnostics['robot_estimate'])))
        return SimulationSample(now, robot, raw, processed, command, self.policy.telemetry(command),
                                self.policy.tangent.copy(), self.policy.contact_direction.copy(), tuple(self.policy.events),
                                diagnostics, simulation_telemetry)
