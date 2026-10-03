"""Offline geometry and configuration checks; no physical alignment claim."""
from copy import deepcopy

import numpy as np
import pytest

from app.configuration import prepare_real
from calibration.probe_alignment import downward_probe_orientation, probe_tilt_deg
from calibration.scan_calibration import load_scan_calibration
from experiment_logging.paths import PROJECT_ROOT
from robot.rtde_controller import _rotvec_to_matrix, _orientation_distance


@pytest.mark.parametrize('orientation,axis', [
    ([1.6408227308559094, -2.578707670713777, .03580868991476346], [0, 0, 1]),
    ([0, 0, 0], [0, 0, 1]),
    ([0, np.pi, 0], [0, 0, 1]),
    ([.2, -.5, .8], [1, 0, 0]),
    ([0, 0, np.pi], [0, 1, 1]),
])
def test_shortest_rotation_aligns_configured_probe_without_extra_twist(orientation, axis):
    before = probe_tilt_deg(orientation, axis)
    target = downward_probe_orientation(orientation, axis)
    unit = np.asarray(axis) / np.linalg.norm(axis)
    np.testing.assert_allclose(_rotvec_to_matrix(target) @ unit, [0, 0, -1], atol=1e-12)
    assert _orientation_distance(orientation, target) == pytest.approx(np.radians(before), abs=1e-8)
    np.testing.assert_allclose(downward_probe_orientation(target, axis), target, atol=1e-12)


@pytest.mark.parametrize('axis', [[0, 0, 0], [1, 2], [0, 0, np.nan]])
def test_invalid_axis_rejected(axis):
    with pytest.raises(ValueError, match='probe_axis_tcp'):
        downward_probe_orientation([0, 0, 0], axis)


def test_saved_pose_derived_consistently_and_tcp_and_limits_preserved(config):
    protected = deepcopy({k: config[k] for k in ('tcp', 'workspace', 'robot', 'safe_return')})
    path = PROJECT_ROOT / config['calibration']['file']
    original_bytes = path.read_bytes()
    saved = load_scan_calibration(path, require_tcp_offset=True)
    robot, start = prepare_real(config, PROJECT_ROOT / 'config.yaml')
    alignment = config['continuous_probe_alignment']
    assert alignment['saved_tilt_deg'] == pytest.approx(5.046, abs=.02)
    assert alignment['target_tilt_deg'] < 1e-5
    np.testing.assert_allclose(start[:3], saved['start_tcp_pose'][:3])
    np.testing.assert_allclose(start[3:], robot['fixed_orientation'])
    np.testing.assert_allclose(start[3:], config['continuous_calibration']['fixed_orientation'])
    np.testing.assert_allclose(start[3:], alignment['target_orientation'])
    assert config['continuous_saved_calibration'] == saved
    assert path.read_bytes() == original_bytes
    for key, value in protected.items(): assert config[key] == value
    assert config['continuous_tracking']['search_speed'] == .018


def test_explicit_legacy_hold_pose_keeps_saved_orientation(config):
    config['calibration']['vertical_probe'] = False
    saved = load_scan_calibration(PROJECT_ROOT / config['calibration']['file'], require_tcp_offset=True)
    robot, start = prepare_real(config, PROJECT_ROOT / 'config.yaml')
    assert 'continuous_probe_alignment' not in config
    np.testing.assert_array_equal(start, saved['start_tcp_pose'])
    np.testing.assert_array_equal(robot['fixed_orientation'], saved['fixed_orientation'])
