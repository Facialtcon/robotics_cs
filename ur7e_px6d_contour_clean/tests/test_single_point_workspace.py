"""Copied laboratory XY boundaries; all motion interfaces are fake."""
from copy import deepcopy

import numpy as np
import pytest

from app.single_point_config import check_segment, load_settings, prepare
from app.single_point_startup import plan_return
from calibration.single_point import load_calibration
from robot.rtde_controller import RobotError
from robot.tcp_identity import tcp_offsets_match
from run_single_point_contact import ROOT
from simulation.single_point import demo_calibration
from tests.doubles import Devices
from workspace.workspace_transform import load_calibration as load_workspace


@pytest.fixture
def settings(config):
    return load_settings(ROOT/'single_point_experiment.yaml', config)


def test_copied_boundary_matches_original_raw_corners_and_tool(config, settings):
    source = load_workspace(ROOT/'workspace/config/workspace_calibration.yaml')
    polygon = [[source['raw_points'][f'P{i}'][axis] for axis in 'xy'] for i in range(4)]
    assert settings['workspace_xy_polygon'] == polygon
    assert settings['workspace_enabled'] and settings['workspace_verified']
    for i, axis in enumerate('xy'):
        assert settings['workspace_limits'][axis+'_min'] == min(point[i] for point in polygon)
        assert settings['workspace_limits'][axis+'_max'] == max(point[i] for point in polygon)
    assert 'z_min' not in settings['workspace_limits'] and 'z_max' not in settings['workspace_limits']
    assert source['metadata']['robot_ip'] == config['robot']['robot_ip']
    assert tcp_offsets_match(source['metadata']['configured_tcp_offset'], config['tcp']['offset'],
                             config['tcp']['offset_tolerance'])


@pytest.mark.parametrize('name', ['A', 'B', 'C'])
def test_existing_lab_groups_use_copied_boundary_without_moving_scan_height(config, settings, name):
    data = load_calibration(ROOT/'single_point_calibration.yaml')
    before = deepcopy(data)
    _, rc, pose, _ = prepare(config, settings, data, name, execute=True)
    assert pose[2] == data['P_ref'][2] == rc['fixed_z']
    assert pose[2] < 0.  # recorded workspace plane is at +263 mm, not a Z limit
    assert rc['continuous_xy_polygon'] == settings['workspace_xy_polygon']
    assert rc['continuous_boundary_margin'] == pytest.approx(.002)
    assert rc['workspace_enabled'] is False  # only the legacy XYZ box is off
    assert not rc.get('continuous_real_execution')
    assert data == before


def test_tilted_edge_rejects_point_inside_aabb(config, settings):
    point = [.410, -.400, -.010]
    for i, axis in enumerate('xy'):
        assert settings['workspace_limits'][axis+'_min'] < point[i] < settings['workspace_limits'][axis+'_max']
    with pytest.raises(ValueError, match='calibrated workspace XY boundary'):
        check_segment(point, point, settings)


def test_original_corner_edge_keeps_probe_clearance(settings):
    polygon = np.asarray(settings['workspace_xy_polygon'])
    midpoint = (polygon[2]+polygon[3])/2
    inside = polygon.mean(axis=0)-midpoint
    inside /= np.linalg.norm(inside)
    for distance, accepted in [(.001, False), (.004, True)]:
        point = np.r_[midpoint+inside*distance, -.01]
        if accepted:
            check_segment(point, point, settings)
        else:
            with pytest.raises(ValueError, match='workspace'):
                check_segment(point, point, settings)


def test_return_lift_is_not_confused_with_workspace_plane(config, settings):
    data = load_calibration(ROOT/'single_point_calibration.yaml')
    _, _, start, _ = prepare(config, settings, data, 'A', execute=True)
    _, _, target, _ = prepare(config, settings, data, 'C', execute=True)
    _, segments = plan_return(config, settings, start, target)
    assert segments[0][1][2] == pytest.approx(data['P_ref'][2]+settings['return_lift_distance_m'])
    assert segments[-1][1][2] == data['P_ref'][2]


def test_shared_sdk_guard_rejects_actual_polygon_exit_before_velocity_send(config, settings):
    data = load_calibration(ROOT/'single_point_calibration.yaml')
    effective, rc, pose, _ = prepare(config, settings, data, 'A', execute=True)
    devices = Devices(effective, pose)
    owner = devices.controller(rc)
    try:
        owner.connect()
        owner.activate_control(confirmed=True)
        devices.receive.pose[:2] = [.410, -.400]
        with pytest.raises(RobotError, match='boundary'):
            owner.command_planar_velocity([1, 0], .018, .01)
        assert not any(call[0] == 'speedL' for call in devices.control.calls)
    finally:
        owner.close()


@pytest.mark.parametrize('polygon', [[[0, 0]]*4, [[0, 0], [1, 1], [0, 1], [1, 0]], [[0, 0]]*3])
def test_invalid_copied_polygon_refuses_preparation(config, settings, polygon):
    settings['workspace_xy_polygon'] = polygon
    data = load_calibration(ROOT/'single_point_calibration.yaml')
    with pytest.raises(ValueError, match='workspace'):
        prepare(config, settings, data, 'A', execute=True)


def test_synthetic_demo_stays_inside_copied_boundary(config, settings):
    data = demo_calibration(config, settings=settings)
    for name in data['groups']:
        _, _, pose, _ = prepare(config, settings, data, name, execute=False)
        check_segment(pose[:3], pose[:3], settings)
