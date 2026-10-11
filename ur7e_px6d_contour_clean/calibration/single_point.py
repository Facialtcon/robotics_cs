"""Independent fixed reference and multiple P0s; no movement or scan calibration."""
from copy import deepcopy
from pathlib import Path
import re

import numpy as np
import yaml

from calibration.probe_alignment import downward_probe_orientation, probe_tilt_deg
from experiment_logging.paths import wall_time_fields
from robot.tcp_identity import normalize_tcp_offset, tcp_offsets_match

LIMITATION = ('探针直径 3 mm：P_ref 是几何参考，不保证不同方向接触同一物理点。'
              '请检查整条接近路径是否先碰到物体其他部分或夹具；不会自动移动到 P_ref。')
DEFAULT_MIN_APPROACH_DISTANCE_M = 0.005


def vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f'{name} must contain {size} finite numbers')
    return result


def calibration_distance_m(reference, pose):
    return float(np.linalg.norm(vector(reference, 3, 'P_ref')[:2] - vector(pose, 6, 'P0')[:2]))


def approach_direction(reference, pose, *, min_distance_m=DEFAULT_MIN_APPROACH_DISTANCE_M):
    if isinstance(min_distance_m, bool) or not np.isfinite(min_distance_m) or min_distance_m <= 0:
        raise ValueError('min_approach_calibration_distance_m must be finite and positive')
    delta = vector(reference, 3, 'P_ref')[:2] - vector(pose, 6, 'P0')[:2]
    distance = np.linalg.norm(delta)
    if distance < min_distance_m or distance <= 1e-12:
        raise ValueError(f'P0 与 P_ref 的 XY 距离过近：实际 {distance*1000:.3f} mm，'
                         f'最小要求 {min_distance_m*1000:.3f} mm；请将 P0 移远后重新采集标定')
    return delta / distance


def group_calibration_issue(calibration, name, min_distance_m=DEFAULT_MIN_APPROACH_DISTANCE_M):
    group = calibration['groups'][name]
    try:
        approach_direction(calibration['P_ref'], group['tcp_pose'], min_distance_m=min_distance_m)
    except ValueError as exc:
        return str(exc)
    if group.get('requires_recalibration') or group.get('direction_xy') is None:
        return '该组需要重新标定；没有有效的接近方向'
    return None


def set_reference(calibration, pose, robot_ip, tcp_offset, *, group_policy=None,
                  min_distance_m=DEFAULT_MIN_APPROACH_DISTANCE_M):
    if calibration.get('groups') and group_policy not in ('keep', 'clear'):
        raise ValueError('P_ref is fixed until an explicit keep/clear P0 choice is made')
    result = deepcopy(calibration)
    pose = vector(pose, 6, 'reference TCP pose')
    result.update(schema_version=1, P_ref=pose[:3].tolist(), reference_tcp_pose=pose.tolist(),
                  reference_captured_at=wall_time_fields(), robot_ip=str(robot_ip),
                  active_tcp_offset=normalize_tcp_offset(tcp_offset))
    if group_policy == 'keep':
        for group in result['groups'].values():
            try:
                direction = approach_direction(result['P_ref'], group['tcp_pose'], min_distance_m=min_distance_m)
            except ValueError:
                group['direction_xy'] = None
                group['requires_recalibration'] = True
            else:
                group['direction_xy'] = direction.tolist()
                group.pop('requires_recalibration', None)
    else:
        result['groups'] = {}
    return result


def save_group(calibration, name, pose, *, min_distance_m=DEFAULT_MIN_APPROACH_DISTANCE_M):
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,40}', name):
        raise ValueError('P0 name must use 1–40 ASCII letters, numbers, underscore or hyphen')
    pose = vector(pose, 6, 'P0')
    direction = approach_direction(calibration['P_ref'], pose, min_distance_m=min_distance_m)
    result = deepcopy(calibration)
    # Preserve legacy metadata when recapturing a group; new groups need only
    # actual calibration data. Legacy fields never grant movement permission.
    group = result.setdefault('groups', {}).setdefault(name, {})
    group.update(tcp_pose=pose.tolist(), direction_xy=direction.tolist(),
                 captured_at=wall_time_fields(), active_tcp_offset=deepcopy(calibration['active_tcp_offset']))
    group.pop('requires_recalibration', None)
    return result


def write_calibration(path, calibration):
    path = Path(path)
    # Preserve the old file until serialization has completed successfully.
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(yaml.safe_dump(calibration, sort_keys=False, allow_unicode=True), encoding='utf-8')
    temporary.replace(path)


def load_calibration(path):
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    return validate_calibration(data)


def validate_calibration(data):
    """Validate in memory without rejecting legacy short-distance records."""
    if not isinstance(data, dict) or data.get('schema_version') != 1 or not isinstance(data.get('groups'), dict):
        raise ValueError('invalid single-point calibration schema')
    if data.get('P_ref') is not None:
        reference = vector(data['P_ref'], 3, 'P_ref')
        pose = vector(data['reference_tcp_pose'], 6, 'reference pose')
        if not np.allclose(reference, pose[:3], atol=1e-12, rtol=0):
            raise ValueError('P_ref differs from recorded reference pose')
        normalize_tcp_offset(data['active_tcp_offset'])
        for name, group in data['groups'].items():
            distance = calibration_distance_m(reference, group['tcp_pose'])
            saved = group.get('direction_xy')
            if saved is None and group.get('requires_recalibration'):
                continue
            saved = vector(saved, 2, name)
            # Legacy short/coincident P0s remain viewable, but validation at use
            # time rejects them according to the current configurable minimum.
            if distance >= DEFAULT_MIN_APPROACH_DISTANCE_M:
                expected = approach_direction(reference, group['tcp_pose'])
                if not np.allclose(saved, expected, atol=1e-9, rtol=0):
                    raise ValueError(f'{name}: saved direction differs from fixed reference')
    elif data['groups']:
        raise ValueError('P0 groups require a fixed P_ref')
    return data


def experiment_start_pose(calibration, name, probe_axis_tcp):
    """Only P0 XY participates; reference Z and heading define every trial."""
    recorded = vector(calibration['groups'][name]['tcp_pose'], 6, 'P0')
    reference = vector(calibration['reference_tcp_pose'], 6, 'reference TCP pose')
    if not np.allclose(vector(calibration['P_ref'], 3, 'P_ref'), reference[:3], atol=1e-12, rtol=0):
        raise ValueError('P_ref differs from recorded reference pose')
    orientation = downward_probe_orientation(reference[3:], probe_axis_tcp)
    return np.r_[recorded[:2], reference[2], orientation]


def validate_group(calibration, name, settings, project):
    group = calibration['groups'][name]
    pose = vector(group['tcp_pose'], 6, 'P0')
    minimum = settings.get('min_approach_calibration_distance_m', DEFAULT_MIN_APPROACH_DISTANCE_M)
    issue = group_calibration_issue(calibration, name, minimum)
    if issue:
        raise ValueError(f'P0-{name}：{issue}；禁止用于自动运动')
    direction = approach_direction(calibration['P_ref'], pose, min_distance_m=minimum)
    if not np.allclose(direction, vector(group['direction_xy'], 2, 'direction'), atol=1e-9, rtol=0):
        raise ValueError('saved approach direction is invalid')
    pose = experiment_start_pose(calibration, name, project['calibration']['probe_axis_tcp'])
    if probe_tilt_deg(pose[3:], project['calibration']['probe_axis_tcp']) > settings['probe_tilt_max_deg']:
        raise ValueError('derived probe orientation must point vertically down')
    if calibration['robot_ip'] != project['robot']['robot_ip']:
        raise ValueError('calibration robot identity differs from config')
    for tcp in (calibration['active_tcp_offset'], group['active_tcp_offset']):
        if not tcp_offsets_match(tcp, project['tcp']['offset'], project['tcp']['offset_tolerance']):
            raise ValueError('calibrated TCP differs from active configuration')
    return pose, direction
