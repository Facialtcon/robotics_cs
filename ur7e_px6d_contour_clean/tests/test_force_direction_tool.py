import subprocess
import sys

import numpy as np
import pytest
import yaml

from core.models import Wrench
from doubles import Sensor
from experiment_logging.paths import PROJECT_ROOT
from tools import check_force_direction as tool


def test_readonly_tool_reports_rotated_3_4_5_without_any_rtde_access(monkeypatch):
    import rtde_control, rtde_receive
    attempts = []
    def forbidden(*a, **k):
        attempts.append(True)
        raise AssertionError('force direction tool must not touch any RTDE interface')
    monkeypatch.setattr(rtde_control, 'RTDEControlInterface', forbidden)
    monkeypatch.setattr(rtde_receive, 'RTDEReceiveInterface', forbidden)
    sensor = Sensor(); sensor.read_wrench = lambda: Wrench(3, 4, 0, 0, 0, 0)
    messages = []
    before = (PROJECT_ROOT/'config.yaml').read_bytes()
    assert tool.run(reader_factory=lambda *a: sensor, samples=1, bias_samples=0, emit=messages.append,
                    tool_orientation=[0., 0., 0.]) == 0
    output = '\n'.join(messages)
    assert 'processed Fx=-0.707107' in output and 'processed Fy=4.949747' in output
    assert 'Fxy=5.000000' in output and 'normalized=[-0.141421, 0.989949]' in output
    assert 'Base +X' in output and 'Base +Y' in output
    assert not attempts and not sensor.connected
    assert (PROJECT_ROOT/'config.yaml').read_bytes() == before


def test_tool_applies_configured_transform_and_displays_sign_without_saving(config, tmp_path):
    # Explicit synthetic legacy-reference fixture, independent of the site mount.
    config['preprocessing']['coordinate_transform']['rotation_sensor_to_tool'] = None
    config['preprocessing']['coordinate_transform']['sensor_origin_in_tool_m'] = None
    config['preprocessing']['coordinate_transform']['rotation_sensor_to_base'] = [[0,-1,0],[1,0,0],[0,0,1]]
    config['preprocessing']['coordinate_transform']['reference_tool_orientation'] = [0, 0, 0]
    config['continuous_tracking']['force_direction_sign'] = -1
    config['calibration']['file'] = str(PROJECT_ROOT/'scan_calibration.yaml')
    path = tmp_path/'config.yaml'; path.write_text(yaml.safe_dump(config))
    sensor = Sensor(); sensor.read_wrench = lambda: Wrench(3, 4, 0, 0, 0, 0)
    messages = []
    before = path.read_bytes()
    assert tool.run(path, reader_factory=lambda *a: sensor, samples=1, bias_samples=0, emit=messages.append,
                    tool_orientation=[0, 0, 0]) == 0
    output = '\n'.join(messages)
    assert 'normalized=[-0.800000, 0.600000]' in output
    assert 'configured n=[0.800000, -0.600000]' in output
    assert path.read_bytes() == before


def test_software_bias_only_uses_raw_then_reports_direction():
    sensor = Sensor()
    readings = iter([Wrench(15, 0, 0, 0, 0, 0)]*2+[Wrench(18, 4, 0, 0, 0, 0)])
    sensor.read_wrench = lambda: next(readings)
    output = []
    assert tool.run(reader_factory=lambda *a: sensor, bias_samples=2, samples=1, emit=output.append,
                    confirm=lambda _: '', tool_orientation=[0., 0., 0.]) == 0
    assert any('normalized=[-0.141421, 0.989949]' in line for line in output)


def test_optional_direction_tool_cancel_prevents_connection_and_bias():
    sensor = Sensor()
    def forbidden():
        raise AssertionError('cancelled bias must not open sensor')
    sensor.connect = forbidden
    assert tool.run(reader_factory=lambda *a: sensor, bias_samples=2, samples=1,
                    confirm=lambda _: 'q', emit=lambda _: None) == 0
    assert sensor.count == 0


def test_tool_zero_force_is_finite():
    magnitude, normal = tool.direction_values(Wrench(0, 0, 0, 0, 0, 0))
    assert magnitude == 0
    np.testing.assert_array_equal(normal, [0, 0])


@pytest.mark.parametrize('kind', ['nan', 'communication', 'operator'])
def test_tool_closes_px6d_on_error_or_user_stop(kind):
    sensor = Sensor()
    def read():
        if kind == 'communication': raise OSError('PX6D read failed')
        return Wrench(np.nan, 0, 0, 0, 0, 0)
    sensor.read_wrench = read
    assert tool.run(reader_factory=lambda *a: sensor, samples=1, bias_samples=0,
                    poll=lambda: 'Q' if kind == 'operator' else None, emit=lambda _: None) == (0 if kind == 'operator' else 1)
    assert not sensor.connected


def test_standalone_help_imports_no_robot_modules():
    code = """
import runpy, sys
sys.argv = ['tools/check_force_direction.py', '--help']
try:
    runpy.run_path('tools/check_force_direction.py', run_name='__main__')
except SystemExit as exc:
    assert exc.code == 0
assert not any(n.startswith(('rtde_', 'robot.')) for n in sys.modules)
"""
    result = subprocess.run([sys.executable, '-c', code], cwd=PROJECT_ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert '--bias-samples' in result.stdout
    assert '--verify' not in result.stdout and '--status' not in result.stdout
