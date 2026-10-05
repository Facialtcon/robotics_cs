"""No pre-rotation zero, no in-contact alignment, and observed tilt reports."""
import numpy as np
import pytest

from app import scan_startup
from core.models import Wrench
from doubles import Sensor
from safety.safe_return import PX6DForceMonitor, ReturnAborted
from sensor.force_preprocess import WrenchPreprocessor
from test_real_tracking_guards import clock, real


def vertical_case(real):
    config, start, devices, owner = real
    start = start.copy()
    start[3:] = [0, np.pi, 0]
    owner.config['fixed_orientation'] = start[3:].tolist()
    config['continuous_probe_alignment'] = dict(axis_tcp=[0, 0, 1])
    devices.receive.pose[:] = start
    devices.receive.pose[0] += .003
    devices.receive.pose[3:] = [0, 3.05, 0]
    return config, start, devices, owner


def test_vertical_return_never_captures_old_pose_bias(real, clock, monkeypatch, capsys):
    config, start, devices, owner = vertical_case(real)
    def forbidden(*args, **kwargs):
        raise AssertionError('zero bias is only allowed after the final orientation')
    monkeypatch.setattr(scan_startup, 'capture_stationary_bias', forbidden)
    prompts, reports = [], {}
    class Logger:
        def log_sample(self, *args, **kwargs):
            assert kwargs['extra']['processed_force_frame'] == 'Sensor'
        def write_json(self, name, value): reports[name] = value
    def confirm(prompt):
        prompts.append(prompt)
        assert '脱离目标及颗粒' in prompt and '旋转空间' in prompt
        assert devices.control_count == 0
        return ''
    assert scan_startup.startup_scan(config, start, owner, Sensor(), Logger(), lambda: None, confirm=confirm)
    assert len(prompts) == 1
    moves = [np.array(c[1]) for c in devices.control.calls if c[0] == 'moveL']
    assert len(moves) == 4
    np.testing.assert_array_equal(moves[0][3:], [0, 3.05, 0])
    assert moves[0][2] > start[2]
    np.testing.assert_array_equal(moves[1][:3], moves[0][:3])
    for pose in moves[1:]: np.testing.assert_array_equal(pose[3:], start[3:])
    assert reports['startup_alignment.json']['actual_before_tilt_deg'] > 5
    assert reports['startup_alignment.json']['actual_after_tilt_deg'] < 1e-5
    assert reports['return_status.json']['return_bias_captured'] is False
    assert '探针轴倾角' in capsys.readouterr().out


def test_cancel_vertical_return_never_activates_control_or_zeroes(real, clock):
    config, start, devices, owner = vertical_case(real)
    sensor = Sensor()
    with pytest.raises(KeyboardInterrupt):
        scan_startup.startup_scan(config, start, owner, sensor, None, lambda: None, confirm=lambda _: 'q')
    assert devices.control_count == 0 and sensor.count == 0


def test_raw_return_force_limit_prevents_motion_without_zero(real, clock):
    config, start, devices, owner = vertical_case(real)
    sensor = Sensor()
    sensor.read_wrench = lambda: Wrench(9, 0, 0, 0, 0, 0)
    with pytest.raises(Exception, match='return force limit'):
        scan_startup.startup_scan(config, start, owner, sensor, None, lambda: None, confirm=lambda _: '')
    assert not any(c[0] == 'moveL' for c in devices.control.calls)


def test_raw_monitor_ignores_wrong_old_bias_and_transform(config):
    processor = WrenchPreprocessor.from_config(config['preprocessing'])
    processor.zero_bias_sensor[:] = 100
    sensor = Sensor()
    sensor.read_wrench = lambda: Wrench(3, 4, 0, 0, 0, 0)
    monitor = PX6DForceMonitor(config, sensor, processor, raw_only=True)
    monitor.sample()
    assert monitor.force_frame == 'Sensor'
    np.testing.assert_array_equal(monitor.processed.array(), monitor.raw.array())
    sensor.read_wrench = lambda: Wrench(9, 0, 0, 0, 0, 0)
    with pytest.raises(ReturnAborted, match='return force limit'): monitor.sample()


@pytest.mark.parametrize('already_at_p0', [False, True])
def test_runtime_captures_single_bias_above_p0_before_vertical_insertion(config, clock, monkeypatch, tmp_path, already_at_p0):
    from app import continuous_runtime as runtime
    from doubles import Devices
    from test_continuous_runtime import args, prepare
    from calibration.scan_calibration import load_scan_calibration
    from experiment_logging.paths import PROJECT_ROOT
    target = prepare(config, monkeypatch)
    saved = load_scan_calibration(PROJECT_ROOT/'scan_calibration.yaml')['start_tcp_pose']
    devices = Devices(config, target if already_at_p0 else saved)
    original = WrenchPreprocessor.set_zero_bias
    captures = []
    def capture(processor, samples):
        captures.append(devices.receive.pose.copy())
        above = np.array(target)
        above[2] += config['safe_return']['return_lift_distance']
        np.testing.assert_allclose(devices.receive.pose, above)
        assert any(c[0] == 'moveL' for c in devices.control.calls)
        assert not any(c[0] == 'speedL' and np.linalg.norm(c[1][:2]) > 0 for c in devices.control.calls)
        return original(processor, samples)
    monkeypatch.setattr(WrenchPreprocessor, 'set_zero_bias', capture)
    assert runtime.run(args(tmp_path), controller_factory=devices.controller, reader_factory=Sensor) == 0
    assert len(captures) == 1
    moves = [np.array(c[1]) for c in devices.control.calls if c[0] == 'moveL']
    np.testing.assert_allclose(moves[-1], target)
    assert moves[-1][2] < captures[0][2]
    np.testing.assert_allclose(moves[-1][[0, 1, 3, 4, 5]], captures[0][[0, 1, 3, 4, 5]])


def test_insertion_load_is_preserved_instead_of_zeroed_at_depth(config, clock, monkeypatch, tmp_path):
    import json
    from app import continuous_runtime as runtime
    from doubles import Devices
    from test_continuous_runtime import args, prepare
    target = prepare(config, monkeypatch)
    devices = Devices(config, target)
    class LoadedSensor(Sensor):
        def read_wrench(self):
            self.count += 1
            # Synthetic unloaded offset above P0, added insertion load at depth.
            force = .2 if devices.receive.pose[2] > target[2]+.015 else 1.2
            return Wrench(0., 0., force, 0., 0., 0.)
    forces = []
    update = runtime.ContinuousTrackingPolicy.update
    def observe(policy, now, raw, processed, *args, **kwargs):
        forces.append(processed.fz)
        return update(policy, now, raw, processed, *args, **kwargs)
    monkeypatch.setattr(runtime.ContinuousTrackingPolicy, 'update', observe)
    assert runtime.run(args(tmp_path), controller_factory=devices.controller, reader_factory=LoadedSensor) == 0
    directory, = (tmp_path/'real/continuous').glob('run_*')
    bias = json.loads((directory/'scan_bias_at_start.json').read_text())
    assert bias['reference'] == 'above_P0_before_vertical_insertion'
    assert bias['tcp_pose'][2] == pytest.approx(target[2]+.030)
    assert bias['zero_bias_sensor'][2] == pytest.approx(.2)
    assert forces and min(forces) > .5  # Insertion force reaches the real preprocessing/policy path.


def test_bias_failure_above_p0_prevents_descent(config, clock, monkeypatch, tmp_path):
    import json
    from app import continuous_runtime as runtime
    from doubles import Devices
    from sensor.px6d_reader import PX6DTimeout
    from test_continuous_runtime import args, prepare
    target = prepare(config, monkeypatch)
    devices = Devices(config, target)
    def fail(processor, samples):
        raise PX6DTimeout('injected failure while capturing raised scan bias')
    monkeypatch.setattr(WrenchPreprocessor, 'set_zero_bias', fail)
    assert runtime.run(args(tmp_path), controller_factory=devices.controller, reader_factory=Sensor) == 1
    assert not any(c[0] == 'moveL' and np.isclose(c[1][2], target[2]) for c in devices.control.calls)
    directory, = (tmp_path/'real/continuous').glob('run_*')
    assert json.loads((directory/'termination.json').read_text())['reason'] == 'STOP_SENSOR_ERROR'
    assert not devices.receive.connected and not devices.control.connected
