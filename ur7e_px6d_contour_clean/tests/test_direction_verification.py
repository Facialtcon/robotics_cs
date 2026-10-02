import json
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from core.models import Wrench
from doubles import Sensor
from experiment_logging.paths import PROJECT_ROOT
from sensor.force_direction import AXES, load_direction_verification
from tools import check_force_direction as tool


@pytest.mark.parametrize('correct', [True, False])
def test_six_axis_verification_records_measured_mapping_without_motion_or_config_edit(config, tmp_path, monkeypatch, correct):
    config['calibration']['file'] = str(PROJECT_ROOT/'scan_calibration.yaml')
    config['preprocessing']['baseline']['sample_count'] = 2
    config['preprocessing']['coordinate_transform']['rotation_sensor_to_base'] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    config['continuous_tracking']['force_direction_sign'] = -1
    source = tmp_path/'config.yaml'
    source.write_text(yaml.safe_dump(config))
    before = source.read_bytes()
    assert not load_direction_verification(config, source)['verified']
    sensor = Sensor()
    sensor.value, sensor.reads = np.zeros(3), []
    def read():
        sensor.reads.append(sensor.value.copy())
        return Wrench(*sensor.value, 0., 0., 0.)
    sensor.read_wrench = read
    matrix = np.asarray(config['preprocessing']['coordinate_transform']['rotation_sensor_to_base'])
    def confirm(prompt):
        if 'UNLOADED' in prompt:
            return 'UNLOADED'
        label = next(label for label in AXES if f'Base {label}' in prompt)
        sensor.value = (1 if correct else -1)*matrix.T @ AXES[label]
        return label
    monkeypatch.setattr(tool, 'time', SimpleNamespace(monotonic=lambda: 1., sleep=lambda seconds: None))
    result = tool.verify(source, reader_factory=lambda *a: sensor, confirm=confirm, emit=lambda _: None)
    assert result == (0 if correct else 1)
    assert not sensor.connected and source.read_bytes() == before
    assert len(sensor.reads) == 302 and np.count_nonzero(np.linalg.norm(sensor.reads, axis=1) == 0) == 2
    record = json.loads((tmp_path/'calibration/force_direction_verification.json').read_text())
    assert record['completed'] and len(record['measurements']) == 6
    for label in ('+Z', '-Z'):
        assert record['measurements'][label]['unloading_direction'] == [0., 0.]
    status = load_direction_verification(config, source)
    assert status['verified'] is correct
    if correct:
        assert status['physical_force_multiplier'] == 1
        config['continuous_tracking']['force_direction_sign'] = 1
        assert not load_direction_verification(config, source)['verified']


def test_direction_tool_requires_unloaded_confirmation_before_any_read(config, tmp_path):
    config['calibration']['file'] = str(PROJECT_ROOT/'scan_calibration.yaml')
    source = tmp_path/'config.yaml'; source.write_text(yaml.safe_dump(config))
    sensor = Sensor()
    def forbidden():
        raise AssertionError('must not open before unloaded confirmation')
    sensor.connect = forbidden
    assert tool.verify(source, reader_factory=lambda *a: sensor, confirm=lambda _: 'cancel', emit=lambda _: None) == 1
    assert sensor.count == 0
    assert not load_direction_verification(config, source)['verified']
