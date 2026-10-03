"""Hardware-free runtime integration with explicitly synthetic known mounts.

Neither matrix used here describes or modifies the physical installation.
"""
import json

import numpy as np
import pytest

from app import continuous_runtime as runtime
from calibration.scan_calibration import load_scan_calibration
from core.models import Wrench
from doubles import Devices, Sensor
from experiment_logging.paths import PROJECT_ROOT
from robot.rtde_controller import _rotvec_to_matrix
from test_continuous_runtime import args, prepare


@pytest.mark.parametrize('mount_source', ['sensor_to_tool', 'base_at_saved_pose'])
def test_vertical_startup_transforms_synthetic_mount_at_actual_pose_before_policy(
        config, monkeypatch, tmp_path, mount_source):
    target = np.asarray(prepare(config, monkeypatch))
    saved = np.asarray(load_scan_calibration(
        PROJECT_ROOT/'scan_calibration.yaml')['start_tcp_pose'])
    # A non-identity synthetic mount exercises multiplication order and the
    # old-pose -> fixed mount -> new-pose route, without claiming a calibration.
    mount = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
    sensor_force = np.array([.2, -.13, .07])
    base_at_saved = _rotvec_to_matrix(saved[3:]) @ mount
    original_load = runtime.load_config

    def load(path):
        loaded = original_load(path)
        preprocessing = loaded['preprocessing']
        preprocessing['filter_alpha'] = 0.
        preprocessing['gravity_wrench_sensor'] = [0.]*6
        preprocessing['granular_baseline_output'] = [0.]*6
        transform = preprocessing['coordinate_transform']
        transform['sensor_origin_in_base_m'] = [0.]*3
        if mount_source == 'sensor_to_tool':
            transform['rotation_sensor_to_tool'] = mount.tolist()
            transform['reference_tool_orientation'] = None
        else:
            transform['rotation_sensor_to_tool'] = None
            transform['rotation_sensor_to_base'] = base_at_saved.tolist()
            transform['reference_tool_orientation'] = saved[3:].tolist()
        return loaded

    monkeypatch.setattr(runtime, 'load_config', load)
    initial_pose = saved.copy()
    initial_pose[0] += .003  # Exercise the horizontal leg as well as alignment.
    devices = Devices(config, initial_pose)
    original_move = devices.control.moveL

    def move_with_small_observed_pose_error(pose, speed, acceleration, asynchronous):
        result = original_move(pose, speed, acceleration, asynchronous)
        # Remain inside existing startup tolerances, but make actual RTDE pose
        # measurably different from the planned target. A target-only transform
        # would therefore fail the force assertions below.
        devices.receive.pose[3] += .001
        return result

    monkeypatch.setattr(devices.control, 'moveL', move_with_small_observed_pose_error)

    class SyntheticSensor(Sensor):
        loading = False

        def read_wrench(self):
            self.count += 1
            force = sensor_force if self.loading else np.zeros(3)
            return Wrench.from_sequence([*force, 0., 0., 0.])

    sensor = SyntheticSensor()
    original_bias = runtime.capture_stationary_bias
    bias_poses = []

    def capture_bias(*positional, **keywords):
        assert not sensor.loading
        bias_poses.append(devices.receive.pose.copy())
        result = original_bias(*positional, **keywords)
        sensor.loading = True
        return result

    monkeypatch.setattr(runtime, 'capture_stationary_bias', capture_bias)
    original_update = runtime.ContinuousTrackingPolicy.update
    observations = []

    def observe_update(policy, now, raw, processed, robot, **keywords):
        observations.append((robot.pose.copy(), raw.force.copy(), processed.force.copy()))
        return original_update(policy, now, raw, processed, robot, **keywords)

    monkeypatch.setattr(runtime.ContinuousTrackingPolicy, 'update', observe_update)
    assert runtime.run(args(tmp_path), controller_factory=devices.controller,
                       reader_factory=lambda *unused: sensor) == 0
    assert len(bias_poses) == 1
    assert len(observations) >= 2
    moves = [np.asarray(call[1]) for call in devices.control.calls if call[0] == 'moveL']
    assert len(moves) == 4
    np.testing.assert_allclose(moves[0][3:], saved[3:])
    assert moves[0][2] > saved[2]
    for pose in moves[1:]:
        np.testing.assert_allclose(pose[3:], target[3:])
    assert np.linalg.norm(bias_poses[0][3:]-target[3:]) == pytest.approx(.001)
    planned_base = _rotvec_to_matrix(target[3:]) @ mount
    for pose, raw_force, policy_force in observations:
        actual_base = _rotvec_to_matrix(pose[3:]) @ mount
        np.testing.assert_allclose(raw_force, sensor_force)
        np.testing.assert_allclose(policy_force, actual_base @ sensor_force, atol=1e-12)
        assert np.linalg.norm(policy_force-planned_base @ sensor_force) > 1e-6
        assert np.linalg.norm(policy_force-base_at_saved @ sensor_force) > 1e-3

    run_dir, = (tmp_path/'real/continuous').glob('run_*')
    status = json.loads((run_dir/'force_transform_at_start.json').read_text())
    assert status['available'] is True and status['output_frame'] == 'Base'
    assert status['orientation_source'] == 'rtde_actual_tcp_at_start'
    np.testing.assert_allclose(status['tool_orientation'], bias_poses[0][3:])
    np.testing.assert_allclose(status['rotation_sensor_to_base'],
                               _rotvec_to_matrix(bias_poses[0][3:]) @ mount, atol=1e-12)
    termination = json.loads((run_dir/'termination.json').read_text())
    assert termination['reason'] == 'STOP_USER_REQUEST'
    assert not devices.control.connected and not devices.receive.connected
    assert not sensor.connected
