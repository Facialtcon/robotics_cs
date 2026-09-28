from copy import deepcopy

import numpy as np
import pytest

from app import configuration
from core.models import RobotState, Wrench
from experiment_logging.paths import PROJECT_ROOT
from experiment_logging.termination import TerminationReason
from policy.continuous_tracking import ContinuousTrackingPolicy
from robot.rtde_controller import RobotError
from safety.search_geometry import ray_polygon_distance


@pytest.mark.parametrize('reverse', [False, True])
def test_ray_uses_oblique_edges_not_aabb(reverse):
    polygon = np.array([[0., 0.], [.4, 0.], [.3, .4], [.1, .4]])
    if reverse:
        polygon = polygon[::-1]
    assert ray_polygon_distance([.2, .2], [2, 0], polygon) == pytest.approx(.15)
    # Ray through a vertex; neither duplicate intersection nor parallel edges
    # may turn a finite budget into an unlimited one.
    assert ray_polygon_distance([.2, .2], [.1, .2], polygon) == pytest.approx(np.hypot(.1, .2))


@pytest.mark.parametrize('origin,direction,polygon', [
    ([.5, .2], [1, 0], [[0, 0], [.4, 0], [.4, .4], [0, .4]]),
    ([.2, .2], [0, 0], [[0, 0], [.4, 0], [.4, .4], [0, .4]]),
    ([.2, .2], [1, 0], [[0, 0], [.4, .4], [.4, 0], [0, .4]]),
    ([.2, .2], [1, np.nan], [[0, 0], [.4, 0], [.4, .4], [0, .4]]),
    ([.4, .2], [1, 0], [[0, 0], [.4, 0], [.4, .4], [0, .4]]),
])
def test_invalid_forward_geometry_fails_closed(origin, direction, polygon):
    with pytest.raises(RobotError):
        ray_polygon_distance(origin, direction, polygon)


def test_real_budget_reaches_beyond_130mm_p1_and_legacy_100mm(config, monkeypatch):
    scan = configuration.load_scan_calibration(PROJECT_ROOT/'scan_calibration.yaml', require_tcp_offset=True)
    scan = deepcopy(scan)
    origin = np.asarray(scan['start_tcp_pose'][:2])
    scan['scan_direction_xy'] = [1., 0.]
    scan['direction_reference_tcp_pose'] = list(scan['start_tcp_pose'])
    scan['direction_reference_tcp_pose'][0] += .130
    polygon = origin + np.array([[-.1, -.2], [.3, -.2], [.3, .2], [-.1, .2]])
    sandbox = dict(raw_points={f'P{i}': dict(x=x, y=y, z=0.) for i, (x, y) in enumerate(polygon)})
    monkeypatch.setattr(configuration, 'load_scan_calibration', lambda *a, **k: scan)
    monkeypatch.setattr(configuration, 'load_workspace_calibration', lambda *a: sandbox)
    config['workspace']['enabled'] = False
    config['continuous_tracking'].pop('real_test_xy_limits', None)
    robot_config, _ = configuration.prepare_real(config, PROJECT_ROOT/'config.yaml')
    geometry = config['continuous_search_geometry']
    margin = config['continuous_tracking']['search_boundary_margin']
    assert geometry['geometric_distance_m'] == pytest.approx(.3)
    assert geometry['usable_distance_m'] == pytest.approx(.3-margin)
    assert geometry['usable_distance_m'] > .130
    assert config['continuous_tracking']['search_max_distance'] == .10
    assert robot_config['continuous_xy_polygon'] == polygon.tolist()

    policy = ContinuousTrackingPolicy(config)
    wrench = Wrench(0, 0, 0, 0, 0, 0)
    pose = np.array(scan['start_tcp_pose'])
    def update(t):
        return policy.update(t, wrench, wrench, RobotState(t, pose.copy(), np.zeros(6)))
    update(0.)
    pose[0] += .130
    assert update(.01).move  # The old 100 mm budget must no longer stop real search.
    pose[:2] = origin + [geometry['usable_distance_m']-.0001, 0]
    assert not update(.02).move  # Next 0.18 mm step would enter the reserved margin.
    assert policy.stop_reason == TerminationReason.STOP_SEARCH_LIMIT


def test_budget_uses_calibrated_origin_even_if_first_read_is_offset(config):
    origin = np.array(config['dry_run']['start_pose'])
    config['continuous_search_geometry'] = dict(origin_xy=origin[:2].tolist(), usable_distance_m=.3)
    policy = ContinuousTrackingPolicy(config)
    wrench = Wrench(0, 0, 0, 0, 0, 0)
    pose = origin.copy(); pose[0] += .002
    policy.update(0., wrench, wrench, RobotState(0., pose, np.zeros(6)))
    pose[0] = origin[0] + .3 - .0001
    command = policy.update(.01, wrench, wrench, RobotState(.01, pose, np.zeros(6)))
    assert not command.move and policy.stop_reason == TerminationReason.STOP_SEARCH_LIMIT


def test_execution_fresh_read_rechecks_search_budget(config):
    from doubles import Devices
    from robot.rtde_controller import SearchLimitReached
    robot_config, start = configuration.prepare_real(config, PROJECT_ROOT/'config.yaml')
    robot_config['continuous_search_geometry']['usable_distance_m'] = .002
    devices = Devices(config, start)
    owner = devices.controller(robot_config)
    owner.connect(); owner.activate_control(confirmed=True)
    owner.set_continuous_phase('TARGET_SEARCH')
    owner.enable_watchdog(config['continuous_tracking']['watchdog_frequency_hz'])
    owner.kick_watchdog()
    direction = np.asarray(config['policy']['search_direction_xy'])
    devices.receive.pose[:2] = start[:2]+direction*.0019
    with pytest.raises(SearchLimitReached):
        owner.command_planar_velocity(direction, .018, .01)
    assert not any(call[0] == 'speedL' for call in devices.control.calls)
    owner.close()
