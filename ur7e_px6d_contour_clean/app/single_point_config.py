"""Independent experiment settings and conservative geometric path checks."""
from copy import deepcopy
from pathlib import Path
import math

import numpy as np
import yaml

from calibration.single_point import DEFAULT_MIN_APPROACH_DISTANCE_M, validate_group, vector
from config.loader import runtime_robot_config

# Copy values from the selected original config at load time, so the two entry
# points cannot silently drift as the operator updates hardware parameters.
PROJECT_PARAMETERS = {
    'speed_max_mps': ('robot', 'max_tcp_speed'),
    'speed_default_mps': ('continuous_tracking', 'search_speed'),
    'contact_threshold_N': ('policy', 'contact_threshold'),
    'control_rate_hz': ('policy', 'control_rate_hz'),
    'max_force_age_sec': ('sensor', 'timeout_sec'),
    'max_robot_age_sec': ('continuous_tracking', 'max_observation_age_sec'),
    'max_cycle_sec': ('continuous_tracking', 'cycle_timeout_sec'),
    'settle_speed_mps': ('continuous_tracking', 'settle_speed_mps'),
    'settle_hold_sec': ('continuous_tracking', 'settle_hold_sec'),
    'start_position_tolerance_m': ('continuous_tracking', 'startup_position_tolerance'),
    'fixed_z_tolerance_m': ('robot', 'fixed_z_tolerance'),
    'orientation_tolerance_rad': ('robot', 'orientation_tolerance_rad'),
    'path_clearance_m': ('continuous_tracking', 'boundary_margin'),
    'watchdog_frequency_hz': ('continuous_tracking', 'watchdog_frequency_hz'),
    'require_watchdog': ('continuous_tracking', 'continuous_require_watchdog'),
    'return_lift_distance_m': ('safe_return', 'return_lift_distance'),
    'return_speed_mps': ('safe_return', 'return_speed'),
    'return_vertical_speed_mps': ('safe_return', 'return_vertical_speed'),
    'speed_acceleration_mps2': ('robot', 'speed_acceleration'),
    'stop_deceleration_mps2': ('robot', 'stop_deceleration'),
}


def inherit_parameters(settings, project):
    result = deepcopy(settings)
    if result.get('inherit_project_parameters', False):
        for key, (section, field) in PROJECT_PARAMETERS.items():
            result[key] = deepcopy(project[section][field])
        result['speed_presets_mps'] = sorted(set(float(project['policy'][key])
            for key in ('probe_speed', 'tangent_speed', 'search_speed')))
    return result


def validate_speed(speed, settings):
    speed = float(speed)
    if not math.isfinite(speed) or not settings['speed_min_mps'] <= speed <= settings['speed_max_mps']:
        raise ValueError(f"speed must be in [{settings['speed_min_mps']}, {settings['speed_max_mps']}] m/s")
    return speed


def validate_settings(settings):
    settings.setdefault('min_approach_calibration_distance_m', DEFAULT_MIN_APPROACH_DISTANCE_M)
    settings.setdefault('workspace_verified', False)
    settings.setdefault('workspace_xy_polygon', None)
    if not isinstance(settings['workspace_verified'], bool):
        raise ValueError('workspace_verified must be a boolean')
    positive = ('speed_min_mps', 'speed_max_mps', 'speed_default_mps', 'contact_threshold_N',
        'unloaded_max_fxy_N', 'max_approach_time_sec',
        'precontact_record_sec', 'post_stop_record_sec', 'control_rate_hz', 'max_force_age_sec',
        'max_robot_age_sec', 'max_cycle_sec', 'settle_speed_mps', 'settle_hold_sec', 'stop_timeout_sec',
        'start_position_tolerance_m', 'fixed_z_tolerance_m', 'orientation_tolerance_rad',
        'probe_tilt_max_deg', 'probe_diameter_m', 'path_clearance_m', 'braking_margin_m',
        'watchdog_frequency_hz', 'return_speed_mps', 'min_approach_calibration_distance_m')
    for key in positive:
        value = settings[key]
        if isinstance(value, bool) or not math.isfinite(float(value)) or float(value) <= 0:
            raise ValueError(f'{key} must be finite and positive')
    if settings['precontact_record_sec'] < 1:
        raise ValueError('stationary precontact recording must last at least one second')
    if settings['speed_min_mps'] > settings['speed_max_mps']:
        raise ValueError('minimum speed must not exceed maximum speed')
    validate_speed(settings['speed_default_mps'], settings)
    for key in ('return_speed_mps', 'return_vertical_speed_mps'):
        speed = float(settings.get(key, settings['return_speed_mps']))
        if not np.isfinite(speed) or not 0 < speed <= settings['speed_max_mps']:
            raise ValueError(f'{key} must be positive and within the TCP speed limit')
    for speed in settings['speed_presets_mps']:
        validate_speed(speed, settings)
    if settings['unloaded_max_fxy_N'] >= settings['contact_threshold_N']:
        raise ValueError('unloaded force must be below contact threshold')
    if not np.isclose(settings['probe_diameter_m'], .003, atol=1e-12, rtol=0):
        raise ValueError('this experiment uses the 3 mm probe')
    if settings.get('require_watchdog', True) and settings['max_cycle_sec'] >= 1/settings['watchdog_frequency_hz']:
        raise ValueError('cycle budget must be shorter than watchdog deadline')
    return settings


def require_motion_boundary(settings):
    """A finite, independently configured and site-verified workspace is mandatory."""
    if settings.get('workspace_enabled') is not True:
        raise ValueError('workspace 安全边界未启用：拒绝真机实验；请配置并现场验证单点实验工作空间。')
    if not isinstance(settings.get('workspace_limits'), dict):
        raise ValueError('workspace 安全边界未配置：拒绝真机实验。')
    if settings.get('workspace_verified') is not True:
        raise ValueError('workspace 安全边界尚未经过现场验证：拒绝真机实验；验证后设置 workspace_verified: true。')


def load_settings(path, project=None):
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError('single point settings must be a mapping')
    if project is None and data.get('inherit_project_parameters', False):
        from config.loader import load_config
        project = load_config(Path(path).resolve().parent/'config.yaml')
    return validate_settings(inherit_parameters(data, project))


def shared_workspace_config(project, settings):
    """Keep a taught XY boundary separate from the shared controller's XYZ box."""
    configured = settings['workspace_limits'] or project['workspace']['limits']
    xy_only = settings.get('workspace_xy_polygon') is not None and not any(
        key in configured for key in ('z_min', 'z_max'))
    # The legacy guard constructor requires six keys even with its XYZ box off.
    # Retain its unused original Z entries; never infer Z safety from the plane.
    limits = deepcopy(project['workspace']['limits'])
    limits.update(deepcopy(configured))
    return dict(enabled=settings.get('workspace_enabled', True) and not xy_only, limits=limits)


def check_segment(start, end, settings):
    """Swept TCP proxy with probe-radius clearance; operator must check whole tool."""
    start, end = vector(start, 3, 'path start'), vector(end, 3, 'path end')
    bounds = settings['workspace_limits']
    enabled = settings.get('workspace_enabled', True)
    if enabled and not isinstance(bounds, dict):
        raise ValueError('workspace_limits must be configured when workspace checking is enabled')
    pad = settings['probe_diameter_m']/2 + settings['path_clearance_m']
    polygon = settings.get('workspace_xy_polygon')
    axes = 'xy' if polygon is not None and not any(key in (bounds or {}) for key in ('z_min', 'z_max')) else 'xyz'
    for i, axis in enumerate(axes if enabled else ''):
        try:
            lo, hi = float(bounds[axis+'_min']), float(bounds[axis+'_max'])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f'workspace_limits 缺少有效的 {axis} 边界') from exc
        if not np.isfinite([lo, hi]).all() or lo+pad >= hi-pad:
            raise ValueError('invalid independent workspace')
        if min(start[i], end[i]) < lo+pad or max(start[i], end[i]) > hi-pad:
            raise ValueError('approach/return path outside independent workspace clearance')
    if enabled and polygon is not None:
        # Reuse the existing convex four-corner checker. Both endpoints inside
        # the inset convex polygon imply the entire straight segment is inside.
        from robot.rtde_controller import RobotError, check_continuous_xy
        try:
            for point in (start, end):
                check_continuous_xy(dict(continuous_xy_polygon=polygon, continuous_boundary_margin=pad), point[:2])
        except (RobotError, TypeError, ValueError) as exc:
            raise ValueError(f'path outside calibrated workspace XY boundary: {exc}') from exc
    for box in settings['forbidden_boxes']:
        low = np.array([box[a+'_min'] for a in 'xyz'], dtype=float)
        high = np.array([box[a+'_max'] for a in 'xyz'], dtype=float)
        if not np.isfinite(np.r_[low, high]).all() or np.any(low >= high):
            raise ValueError('invalid forbidden box')
        low, high = low-pad, high+pad
        begin, finish = 0., 1.
        for i, delta in enumerate(end-start):
            if abs(delta) < 1e-12:
                if not low[i] <= start[i] <= high[i]:
                    finish = -1.; break
            else:
                pair = sorted(((low[i]-start[i])/delta, (high[i]-start[i])/delta))
                begin, finish = max(begin, pair[0]), min(finish, pair[1])
        if begin <= finish:
            raise ValueError('path intersects a forbidden fixture/obstacle; adjust P0/direction')


def prepare(project, settings, calibration, name, *, execute, speed=None, approach=True):
    validate_settings(settings)
    pose, direction = validate_group(calibration, name, settings, project)
    if speed is not None:
        validate_speed(speed, settings)
    if execute and approach:
        require_motion_boundary(settings)
    effective = deepcopy(project)
    # No granular medium. Change only this run snapshot, never config.yaml.
    effective['preprocessing']['granular_baseline_output'] = [0.] * 6
    effective['preprocessing']['granular_baseline']['capture_on_start'] = False
    effective['policy']['contact_threshold'] = settings['contact_threshold_N']  # replay compatibility
    effective['policy']['control_rate_hz'] = settings['control_rate_hz']
    effective['robot']['max_tcp_speed'] = min(project['robot']['max_tcp_speed'], settings['speed_max_mps'])
    effective['continuous_tracking']['enabled'] = False
    effective['continuous_tracking'].pop('contact_lost_threshold', None)
    effective['continuous_tracking']['settle_speed_mps'] = settings['settle_speed_mps']
    effective['continuous_tracking'].update(cycle_timeout_sec=settings['max_cycle_sec'],
        max_observation_age_sec=settings['max_robot_age_sec'], max_sample_gap_sec=settings['max_cycle_sec'])
    effective['single_point_experiment'] = deepcopy(settings)
    effective['single_point_calibration'] = deepcopy(calibration)
    effective['experiment'] = dict(kind='single_point_contact' if execute else 'synthetic_demo',
                                   title='Multi-Directional Single-Point Contact Experiment', p0=name)
    if settings['contact_threshold_N'] >= project['policy']['safety_force_threshold']:
        raise ValueError('contact threshold must be below the existing hard processed-force limit')
    if settings['workspace_limits'] is not None or not settings.get('workspace_enabled', True):
        effective['workspace'] = shared_workspace_config(project, settings)
        end = pose[:3].copy()
        end[:2] += direction * (settings['speed_max_mps']/settings['control_rate_hz']+settings['braking_margin_m'])
        if approach:
            check_segment(pose[:3], end, settings)
    elif execute:
        raise ValueError('independent workspace is required')
    robot_config = runtime_robot_config(effective)
    robot_config.update(fixed_z=float(pose[2]), fixed_orientation=pose[3:].tolist(),
        fixed_z_tolerance=settings['fixed_z_tolerance_m'], orientation_tolerance_rad=settings['orientation_tolerance_rad'],
        allow_unverified_active_tcp=False, settle_hold_sec=settings['settle_hold_sec'],
        confirmation_timeout_sec=settings['stop_timeout_sec'], observation_period_sec=1/settings['control_rate_hz'],
        # These shared execution keys enable strict freshness/watchdog guards only.
        # No continuous policy, phase envelope or recovery strategy is loaded.
        continuous_require_watchdog=execute and settings.get('require_watchdog', True),
        continuous_sample_age_sec=settings['max_robot_age_sec'],
        continuous_settle_speed_mps=settings['settle_speed_mps'])
    if settings.get('workspace_enabled') and settings.get('workspace_xy_polygon') is not None:
        # Activate only the existing geometry guard, never the continuous policy.
        robot_config.update(continuous_xy_polygon=deepcopy(settings['workspace_xy_polygon']),
                            continuous_boundary_margin=settings['probe_diameter_m']/2+settings['path_clearance_m'])
    return effective, robot_config, pose, direction
