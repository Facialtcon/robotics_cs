"""Hardware-free runtime integration with explicitly synthetic known mounts.

Neither matrix used here describes or modifies the physical installation.
"""
import json
import csv

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
@pytest.mark.parametrize('kalman_enabled', [True, False])
def test_vertical_startup_transforms_synthetic_mount_at_actual_pose_before_policy(
        config, monkeypatch, tmp_path, mount_source, kalman_enabled):
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
        preprocessing['kalman'] = dict(enabled=kalman_enabled,
            process_noise=.01, measurement_noise=.25, initial_covariance=1.)
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
        loading_count = 0

        def read_wrench(self):
            self.count += 1
            scan_loading = self.loading and not devices.owner._return_mode
            if scan_loading:
                self.loading_count += 1
            force = sensor_force * self.loading_count if scan_loading else np.zeros(3)
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
    estimate, covariance = None, 1.
    for i, (pose, raw_force, policy_force) in enumerate(observations, 1):
        actual_base = _rotvec_to_matrix(pose[3:]) @ mount
        np.testing.assert_allclose(raw_force, i*sensor_force)
        measurement = actual_base @ raw_force
        if estimate is None or not kalman_enabled:
            estimate = measurement
        else:
            gain = (covariance+.01)/(covariance+.01+.25)
            estimate = estimate + gain*(measurement-estimate)
            covariance = (1-gain)*(covariance+.01)
        np.testing.assert_allclose(policy_force, estimate, atol=1e-12)
        if i == 1:
            assert np.linalg.norm(policy_force-planned_base @ sensor_force) > 1e-6
            assert np.linalg.norm(policy_force-base_at_saved @ sensor_force) > 1e-3
        if i == 2 and kalman_enabled:
            assert np.linalg.norm(policy_force-measurement) > 1e-3  # alpha=0 EMA would pass through

    run_dir, = (tmp_path/'real/continuous').glob('run_*')
    with (run_dir/'samples.csv').open(newline='') as handle:
        scan_rows = [row for row in csv.DictReader(handle) if row['current_state'] != 'RETURN_TO_START']
    assert len(scan_rows) == len(observations)
    for row, (pose, raw_force, policy_force) in zip(scan_rows, observations):
        assert row['processed_force_frame'] == 'Base'
        np.testing.assert_allclose([float(row[f'raw_f{axis}']) for axis in 'xyz'], raw_force)
        np.testing.assert_allclose([float(row[f'force_base_f{axis}']) for axis in 'xyz'],
                                   _rotvec_to_matrix(pose[3:]) @ mount @ raw_force)
        np.testing.assert_allclose([float(row[f'filtered_force_base_f{axis}']) for axis in 'xyz'], policy_force)
        np.testing.assert_allclose([float(row[f'df{axis}']) for axis in 'xyz'], policy_force)
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


def test_runtime_timeout_after_valid_sample_never_updates_kalman_or_retries(
        config, monkeypatch, tmp_path):
    from sensor.force_preprocess import WrenchPreprocessor
    from sensor.px6d_reader import PX6DTimeout

    target = prepare(config, monkeypatch)
    devices = Devices(config, target)
    original_load = runtime.load_config
    def load(path):
        loaded = original_load(path)
        loaded['preprocessing']['baseline']['capture_on_start'] = False
        return loaded
    monkeypatch.setattr(runtime, 'load_config', load)

    class TimeoutSensor(Sensor):
        scan_reads = 0
        def read_wrench(self):
            self.count += 1
            if devices.owner.control is None or devices.owner._return_mode:
                return Wrench(0., 0., 0., 0., 0., 0.)
            self.scan_reads += 1
            assert self.scan_reads <= 2, 'failed acquisition must not be retried during cleanup'
            if self.scan_reads == 2:
                raise PX6DTimeout('offline force-chain timeout')
            return Wrench(.1, .2, .3, 0., 0., 0.)

    sensor, updates = TimeoutSensor(), []
    original_process = WrenchPreprocessor.process
    def observe_process(processor, raw):
        output = original_process(processor, raw)
        updates.append((processor, processor.force_kalman.state.copy(),
                        processor.force_kalman.covariance.copy()))
        return output
    monkeypatch.setattr(WrenchPreprocessor, 'process', observe_process)
    settings = args(tmp_path)
    settings.duration = 1.0
    assert runtime.run(settings, controller_factory=devices.controller,
                       reader_factory=lambda *unused: sensor) == 1
    assert sensor.scan_reads == 2 and len(updates) == 1
    processor, state, covariance = updates[0]
    np.testing.assert_array_equal(processor.force_kalman.state, state)
    np.testing.assert_array_equal(processor.force_kalman.covariance, covariance)
    run_dir, = (tmp_path/'real/continuous').glob('run_*')
    with (run_dir/'samples.csv').open(newline='') as handle:
        rows = [r for r in csv.DictReader(handle) if r['current_state'] != 'RETURN_TO_START']
    assert len(rows) == 1
    record = json.loads((run_dir/'termination.json').read_text())
    assert record['reason'] == 'STOP_SENSOR_ERROR'  # existing PX6D error classification
