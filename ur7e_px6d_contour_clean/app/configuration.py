#!/usr/bin/env python3
"""Offline calibration checks and unchanged execution bounds for both strategies."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

from calibration.scan_calibration import (load_scan_calibration, resolve_calibration_path,
                                          validate_calibration_constraints)
from config.loader import runtime_robot_config
from policy.continuous_tracking import validate_config
from robot.rtde_controller import URRTDEController, RobotError, validate_execution_configuration, check_continuous_xy, CONTINUOUS_TRIP_FACTOR, CONTINUOUS_HARD_FACTOR, CONTINUOUS_DEBOUNCE_SEC, CONTINUOUS_DEBOUNCE_PACKETS
from robot.tcp_identity import tcp_offsets_match
from safety.safe_return import validate_return_configuration
from workspace.workspace_transform import load_calibration as load_workspace_calibration

ROOT = Path(__file__).resolve().parents[1]
def workspace_calibration_path(config, config_path):
    return resolve_calibration_path(config_path, config['continuous_tracking'].get(
        'workspace_calibration_file', 'workspace/config/workspace_calibration.yaml'))


def site_configuration_digest(config, config_path=ROOT / "config.yaml"):
    c = {k:v for k,v in config['continuous_tracking'].items()
         if k not in ('site_verification', 'search_boundary_margin')}  # derived execution clearance
    payload = dict(tcp=config['tcp'], preprocessing=config['preprocessing'], continuous=c,
                   robot=config['robot'], workspace=config['workspace'], policy=config['policy'],
                   safe_return=config['safe_return'],
                   calibration=config['calibration'], calibration_file_sha256=hashlib.sha256(
                       resolve_calibration_path(config_path, config['calibration']['file']).read_bytes()).hexdigest(),
                   workspace_calibration_file_sha256=hashlib.sha256(
                       workspace_calibration_path(config, config_path).read_bytes()).hexdigest())
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()


def prepare_real(config, config_path):
    """Read the project's saved scan and sandbox calibrations before devices exist."""
    budget = config['continuous_tracking']['max_runtime_sec']
    if budget is None or isinstance(budget, bool) or not np.isfinite(float(budget)) or float(budget) <= 0:
        raise RobotError('real continuous_tracking.max_runtime_sec must be finite and positive')
    validate_execution_configuration(config)
    validate_config(config)
    c = config['continuous_tracking']
    scan_path = resolve_calibration_path(config_path, config['calibration']['file'])
    sandbox_path = workspace_calibration_path(config, config_path)
    calibration = load_scan_calibration(scan_path, require_tcp_offset=True)
    sandbox = load_workspace_calibration(sandbox_path)
    if calibration['robot_ip'] != config['robot']['robot_ip']:
        raise RobotError('scan calibration robot_ip does not match config')
    if not tcp_offsets_match(calibration['active_tcp_offset'], config['tcp']['offset'],
                             float(config['tcp']['offset_tolerance'])):
        raise RobotError('scan calibration TCP does not match configured TCP')
    validate_calibration_constraints(calibration, float(config['calibration']['min_direction_calibration_distance']),
                                     float(config['calibration']['max_direction_calibration_z_difference']))
    validate_return_configuration(config, calibration['start_tcp_pose'])
    for name in ('return_lift_distance', 'return_speed', 'return_vertical_speed', 'return_acceleration',
                 'return_force_limit', 'return_torque_limit', 'return_position_tolerance',
                 'return_orientation_tolerance', 'return_segment_timeout_sec'):
        if not np.isfinite(float(config['safe_return'][name])):
            raise RobotError(f'safe_return.{name} must be finite')
    count = config['safe_return']['startup_bias_sample_count']
    if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
        raise RobotError('safe_return.startup_bias_sample_count must be a positive integer')
    # Preserve acquisition metadata, including configuration-only TCP provenance.
    # The actual active TCP is still read/checked by controller.connect().
    metadata = sandbox.get('metadata', {})
    if metadata.get('robot_ip') and metadata['robot_ip'] != config['robot']['robot_ip']:
        raise RobotError('workspace calibration robot_ip does not match config')
    if metadata.get('configured_tcp_offset') is not None and not tcp_offsets_match(
            metadata['configured_tcp_offset'], config['tcp']['offset'], float(config['tcp']['offset_tolerance'])):
        raise RobotError('workspace calibration TCP does not match configured TCP')
    digest = site_configuration_digest(config, config_path)
    verification = c.get('site_verification')
    # Reuse the existing project's calibration instead of requiring a second
    # boolean attestation. If explicitly supplied, a record must still be valid.
    if verification is not None:
        if not isinstance(verification, dict):
            raise RobotError('site verification must be a mapping when supplied')
        for key in ('force_sign_checked', 'base_transform_checked', 'watchdog_stop_verified'):
            if verification.get(key) is not True:
                raise RobotError(f'site verification missing: {key}')
        if not verification.get('operator') or not verification.get('checked_at'):
            raise RobotError('site verification must identify operator and time')
        if verification.get('configuration_sha256') != digest:
            raise RobotError('site verification does not match current tool/frame/sign/configuration/calibrations')
        config['continuous_reviewed_configuration_sha256'] = digest
    if c['reacquire_enabled'] and not (isinstance(verification, dict) and verification.get('recovery_verified') is True):
        raise RobotError('real recovery requires separate site verification')

    # Use measured XY corners, not the fitted rectangle or its mean Z. The
    # polygon guard below also checks sloping edges; its AABB alone is insufficient.
    polygon = np.array([[sandbox['raw_points'][f'P{i}'][axis] for axis in 'xy'] for i in range(4)])
    bounds = config['workspace']['limits'] if config['workspace'].get('enabled', True) else c.get('real_test_xy_limits')
    if bounds is None:
        bounds = {f'{axis}_{side}': float(reducer(polygon[:,i]))
                  for i, axis in enumerate('xy') for side, reducer in [('min',np.min),('max',np.max)]}
    if not isinstance(bounds, dict):
        raise RobotError('invalid continuous site XY bounds')
    margin = float(c['boundary_margin'])
    for axis in 'xy':
        low, high = float(bounds[f'{axis}_min']), float(bounds[f'{axis}_max'])
        if not np.isfinite([low, high]).all() or high-low <= 2*margin:
            raise RobotError('invalid continuous site XY bounds')
    watchdog_contract = URRTDEController.verified_watchdog_contract()
    speed_limits = dict(TARGET_SEARCH=float(c['search_speed']),
                        CONTINUOUS_TRACKING=float(np.hypot(c['tangential_speed'], c['normal_speed_limit'])),
                        LOCAL_REACQUIRE=float(c['reacquire_speed']))
    deceleration = float(config['robot']['stop_deceleration'])
    if not np.isfinite(deceleration) or deceleration <= 0:
        raise RobotError('stop_deceleration must be finite and positive')
    def stopping_margin(nominal):
        hard = min(CONTINUOUS_HARD_FACTOR * nominal, 1.2 * float(config['robot']['max_tcp_speed']))
        return hard * (CONTINUOUS_DEBOUNCE_SEC + 1/float(c['watchdog_frequency_hz'])) + hard**2/(2*deceleration)
    max_tracking = max(speed_limits['CONTINUOUS_TRACKING'], speed_limits['LOCAL_REACQUIRE'])
    required_margin = stopping_margin(max_tracking)
    if margin < required_margin or 1/float(c['watchdog_frequency_hz']) <= float(c['cycle_timeout_sec']):
        raise RobotError('stop margin/watchdog deadline incompatible with execution budgets')
    # Search alone needs more clearance. Include the entire allowed transient
    # band, its finite confirmation delay, watchdog interval and ideal braking.
    # This is an engineering budget to validate on site, not a stopping guarantee.
    search_margin = max(margin, stopping_margin(speed_limits['TARGET_SEARCH']))
    if search_margin >= float(c['search_max_distance']):
        raise RobotError('search stopping margin consumes search distance budget')
    c['search_boundary_margin'] = search_margin  # runtime snapshot only; saved calibration is unchanged
    config['continuous_sdk_contract'] = watchdog_contract
    config['continuous_loaded_configuration_sha256'] = digest
    config['policy']['search_direction_xy'] = list(calibration['scan_direction_xy'])
    robot_config = runtime_robot_config(config)
    robot_config.update(fixed_z=calibration['fixed_z'], fixed_orientation=calibration['fixed_orientation'],
                        continuous_require_watchdog=True, continuous_sample_age_sec=float(c['max_observation_age_sec']),
                        continuous_settle_speed_mps=float(c['settle_speed_mps']), continuous_xy_limits=dict(bounds),
                        continuous_xy_polygon=polygon.tolist(), continuous_speed_limits=speed_limits,
                        continuous_tracking_boundary_margin=margin, continuous_search_boundary_margin=search_margin,
                        continuous_boundary_margin=margin)
    check_continuous_xy({**robot_config, 'continuous_boundary_margin': search_margin}, calibration['start_tcp_pose'][:2])
    config['continuous_calibration'] = calibration
    config['continuous_workspace_calibration'] = sandbox
    config['continuous_calibration_sources'] = dict(scan=str(scan_path.resolve()), workspace=str(sandbox_path.resolve()))
    config['continuous_execution_envelope'] = dict(raw_xy_polygon=polygon.tolist(), xy_limits=dict(bounds), margin_m=margin,
        search_margin_m=search_margin, nominal_speed_limits_mps=speed_limits,
        trip_factor=CONTINUOUS_TRIP_FACTOR, hard_factor=CONTINUOUS_HARD_FACTOR,
        debounce_sec=CONTINUOUS_DEBOUNCE_SEC, debounce_packets=CONTINUOUS_DEBOUNCE_PACKETS)
    return robot_config, np.asarray(calibration['start_tcp_pose'])


def calibration_summary(config):
    """An offline-readable report, also printed before execute connects devices."""
    scan = config['continuous_calibration']
    return dict(sources=config['continuous_calibration_sources'],
                scan_start_tcp_pose=scan['start_tcp_pose'],
                scan_direction_reference_tcp_pose=scan['direction_reference_tcp_pose'],
                scan_direction_xy=scan['scan_direction_xy'],
                fixed_z=scan['fixed_z'], fixed_orientation=scan['fixed_orientation'],
                sandbox_raw_corners=config['continuous_workspace_calibration']['raw_points'],
                execution_envelope=config['continuous_execution_envelope'],
                loaded_configuration_sha256=config['continuous_loaded_configuration_sha256'])




def prepare_discrete(config, config_path):
    """Saved scan identity and geometry for the original discrete policy."""
    validate_execution_configuration(config)
    path = resolve_calibration_path(config_path, config['calibration']['file'])
    calibration = load_scan_calibration(path, require_tcp_offset=True)
    if calibration['robot_ip'] != config['robot']['robot_ip']:
        raise RobotError('scan calibration robot_ip does not match config')
    if not tcp_offsets_match(calibration['active_tcp_offset'], config['tcp']['offset'], config['tcp']['offset_tolerance']):
        raise RobotError('scan calibration TCP does not match config')
    validate_calibration_constraints(calibration, config['calibration']['min_direction_calibration_distance'],
                                    config['calibration']['max_direction_calibration_z_difference'])
    start = np.asarray(calibration['start_tcp_pose'])
    validate_return_configuration(config, start)
    config['policy']['search_direction_xy'] = list(calibration['scan_direction_xy'])
    robot_config = runtime_robot_config(config)
    robot_config.update(fixed_z=calibration['fixed_z'], fixed_orientation=calibration['fixed_orientation'])
    return robot_config, start
