"""Direction faults traverse policy -> execution owner -> measured stop."""
import json
import numpy as np
import pytest

from app import continuous_runtime as runtime
from core.models import Wrench
from doubles import Devices, Sensor
from experiment_logging.paths import PROJECT_ROOT
from calibration.scan_calibration import load_scan_calibration
from test_continuous_runtime import args, prepare
from test_real_tracking_guards import clock


def test_bad_force_direction_never_reaches_tracking_speedl(config, monkeypatch, tmp_path, clock):
    target = prepare(config, monkeypatch)
    direction = np.array(load_scan_calibration(PROJECT_ROOT/'scan_calibration.yaml')['scan_direction_xy'])
    devices = Devices(config, target)
    class ContactSensor(Sensor):
        def read_wrench(self):
            self.count += 1
            distance = float((devices.receive.pose[:2]-target[:2]) @ direction)
            force = -1.5*direction if distance > .0003 else np.zeros(2)
            return Wrench(*force, 0, 0, 0, 0)
    sensor = ContactSensor()
    settings = args(tmp_path)
    settings.duration = 5.
    assert runtime.run(settings, controller_factory=devices.controller, reader_factory=lambda *a: sensor) == 1
    directory, = (tmp_path/'real/continuous').glob('run_*')
    stopped = json.loads((directory/'termination.json').read_text())
    assert stopped['reason'] == 'STOP_DIRECTION_UNCONFIRMED'
    assert 'contradicts observed search' in stopped['detail']
    summary = json.loads((directory/'summary.json').read_text())
    assert summary['stop_observation']['standstill_confirmed']
    calls = [np.array(c[1]) for c in devices.control.calls if c[0] == 'speedL']
    moving = [v for v in calls if np.linalg.norm(v[:2]) > 0]
    assert moving
    # All movement was the authorized approach, never opposite feedback or an arc.
    for velocity in moving:
        assert float(velocity[:2] @ direction) > 0
        np.testing.assert_allclose(velocity[:2]/np.linalg.norm(velocity[:2]), direction)
    np.testing.assert_array_equal(calls[-1], np.zeros(6))
    assert not sensor.connected and not devices.receive.connected


@pytest.mark.parametrize('case,error', [
    ('unknown_mount', 'rotation_sensor_to_tool'),
    ('missing_reference_matrix', 'reference_tool_orientation requires rotation_sensor_to_base'),
    ('invalid_mount', 'proper 3x3 rotation matrix')])
def test_force_transform_configuration_fails_before_any_device_or_startup(config, monkeypatch, tmp_path, case, error):
    prepare(config, monkeypatch)
    configured_loader = runtime.load_config
    def load(path):
        value = configured_loader(path)
        transform = value['preprocessing']['coordinate_transform']
        transform['reference_tool_orientation'] = None
        if case == 'missing_reference_matrix':
            transform['reference_tool_orientation'] = [0., 0., 0.]
            transform['rotation_sensor_to_base'] = None
        elif case == 'invalid_mount':
            transform['rotation_sensor_to_tool'] = np.diag([1., 1., -1.]).tolist()
        return value
    calls = []
    def forbidden(*args):
        calls.append(args)
        raise AssertionError('device factory called for invalid force configuration')
    monkeypatch.setattr(runtime, 'load_config', load)
    assert runtime.run(args(tmp_path), controller_factory=forbidden, reader_factory=forbidden) == 1
    assert calls == []  # Includes readers/controllers whose constructor might access hardware.
    directory, = (tmp_path/'real/continuous').glob('run_*')
    stopped = json.loads((directory/'termination.json').read_text())
    assert stopped['reason'] == 'STOP_CONFIG_ERROR'
    assert stopped['phase'] == 'CONFIG_PREFLIGHT'
    assert error in stopped['detail']
    assert stopped['raw_wrench'] is None and stopped['processed_wrench'] is None
    assert not (directory/'samples.csv').exists()
    if case == 'unknown_mount':
        status = stopped['force_transform_status']
        assert status['source'] == 'unknown_sensor_installation'
        assert status['rotation_sensor_to_tool'] is None
        assert status['rotation_sensor_to_base'] is None
        assert status['tool_orientation'] is not None
