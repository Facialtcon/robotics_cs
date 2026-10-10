"""Independent fixed reference and multiple P0s; no movement or scan calibration."""
from copy import deepcopy
from pathlib import Path
import re

import numpy as np
import yaml

from calibration.probe_alignment import probe_tilt_deg
from experiment_logging.paths import wall_time_fields
from robot.rtde_controller import _orientation_distance
from robot.tcp_identity import normalize_tcp_offset, tcp_offsets_match

LIMITATION = ('探针直径 3 mm：P_ref 是几何参考，不保证不同方向接触同一物理点。'
              '请检查整条接近路径是否先碰到物体其他部分或夹具；不会自动移动到 P_ref。')


def vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f'{name} must contain {size} finite numbers')
    return result


def approach_direction(reference, pose):
    delta = vector(reference, 3, 'P_ref')[:2] - vector(pose, 6, 'P0')[:2]
    distance = np.linalg.norm(delta)
    if distance < 1e-6:
        raise ValueError('P0 and P_ref coincide in XY or are too close')
    return delta / distance


def set_reference(calibration, pose, robot_ip, tcp_offset):
    if calibration.get('groups'):
        raise ValueError('P_ref is fixed: delete all P0 groups before recalibrating the reference')
    result = deepcopy(calibration)
    pose = vector(pose, 6, 'reference TCP pose')
    result.update(schema_version=1, P_ref=pose[:3].tolist(), reference_tcp_pose=pose.tolist(),
                  reference_captured_at=wall_time_fields(), robot_ip=str(robot_ip),
                  active_tcp_offset=normalize_tcp_offset(tcp_offset), groups={})
    return result


def save_group(calibration, name, pose, *, path_note='', reviewed_distance_m=None):
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,40}', name):
        raise ValueError('P0 name must use 1–40 ASCII letters, numbers, underscore or hyphen')
    pose = vector(pose, 6, 'P0')
    direction = approach_direction(calibration['P_ref'], pose)
    if reviewed_distance_m is not None and (not np.isfinite(reviewed_distance_m) or reviewed_distance_m <= 0):
        raise ValueError('reviewed distance must be positive')
    result = deepcopy(calibration)
    result.setdefault('groups', {})[name] = dict(tcp_pose=pose.tolist(), direction_xy=direction.tolist(),
        captured_at=wall_time_fields(), active_tcp_offset=deepcopy(calibration['active_tcp_offset']),
        path_review=dict(note=path_note.strip(), max_distance_m=reviewed_distance_m,
                         reviewed_at=wall_time_fields() if path_note.strip() else None))
    return result


def write_calibration(path, calibration):
    path = Path(path)
    # Preserve the old file until serialization has completed successfully.
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(yaml.safe_dump(calibration, sort_keys=False, allow_unicode=True), encoding='utf-8')
    temporary.replace(path)


def load_calibration(path):
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data, dict) or data.get('schema_version') != 1 or not isinstance(data.get('groups'), dict):
        raise ValueError('invalid single-point calibration schema')
    if data.get('P_ref') is not None:
        reference = vector(data['P_ref'], 3, 'P_ref')
        pose = vector(data['reference_tcp_pose'], 6, 'reference pose')
        if not np.allclose(reference, pose[:3], atol=1e-12, rtol=0):
            raise ValueError('P_ref differs from recorded reference pose')
        normalize_tcp_offset(data['active_tcp_offset'])
        for name, group in data['groups'].items():
            expected = approach_direction(reference, group['tcp_pose'])
            if not np.allclose(vector(group['direction_xy'], 2, name), expected, atol=1e-9, rtol=0):
                raise ValueError(f'{name}: saved direction differs from fixed reference')
    elif data['groups']:
        raise ValueError('P0 groups require a fixed P_ref')
    return data


def validate_group(calibration, name, settings, project, *, require_review=True):
    group = calibration['groups'][name]
    pose = vector(group['tcp_pose'], 6, 'P0')
    reference_pose = vector(calibration['reference_tcp_pose'], 6, 'reference TCP pose')
    direction = approach_direction(calibration['P_ref'], pose)
    if not np.allclose(direction, vector(group['direction_xy'], 2, 'direction'), atol=1e-9, rtol=0):
        raise ValueError('saved approach direction is invalid')
    if abs(pose[2]-reference_pose[2]) > settings['fixed_z_tolerance_m']:
        raise ValueError('all P0s and P_ref must share fixed Z')
    if _orientation_distance(pose[3:], reference_pose[3:]) > settings['orientation_tolerance_rad']:
        raise ValueError('all P0s must share the fixed reference orientation')
    if probe_tilt_deg(pose[3:], project['calibration']['probe_axis_tcp']) > settings['probe_tilt_max_deg']:
        raise ValueError('probe must point vertically down; adjust manually, no automatic alignment')
    if calibration['robot_ip'] != project['robot']['robot_ip']:
        raise ValueError('calibration robot identity differs from config')
    for tcp in (calibration['active_tcp_offset'], group['active_tcp_offset']):
        if not tcp_offsets_match(tcp, project['tcp']['offset'], project['tcp']['offset_tolerance']):
            raise ValueError('calibrated TCP differs from active configuration')
    review = group.get('path_review', {})
    if require_review and (not review.get('note') or not review.get('reviewed_at') or
                          float(review.get('max_distance_m') or 0) < settings['max_search_distance_m']):
        raise ValueError('approach path not reviewed for the full maximum search distance; adjust calibration')
    return pose, direction
