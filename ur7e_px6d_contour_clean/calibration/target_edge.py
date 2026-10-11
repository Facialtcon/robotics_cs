"""Read-only taught TCP centers define a local straight edge in Base XY (SI)."""
from copy import deepcopy
from pathlib import Path
import os
import tempfile
import time

import numpy as np
import yaml

from experiment_logging.paths import wall_time_fields

DEFAULT_MIN_EDGE_DISTANCE_M = .005


def vector(value, size, label):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f'{label} 必须是 {size} 维有限数值')
    return result


def edge_geometry(a, b, minimum=DEFAULT_MIN_EDGE_DISTANCE_M):
    a, b = vector(a, 6, 'A点'), vector(b, 6, 'B点')
    minimum = float(minimum)
    if not np.isfinite(minimum) or minimum <= 0:
        raise ValueError('目标直边最小点间距离必须为有限正数')
    delta = b[:2]-a[:2]
    length = float(np.linalg.norm(delta))
    if not np.isfinite(length):
        raise ValueError('A、B的XY距离不是有限数值')
    if length < minimum:
        raise ValueError(f'A、B的XY距离 {length*1000:.3f} mm，小于最小距离 {minimum*1000:.3f} mm；请移远后重新采集')
    tangent = delta/length
    return dict(tangent_base_xy=tangent.tolist(), normal_base_xy=[float(-tangent[1]), float(tangent[0])],
                length_xy_m=length, direction_deg=float(np.degrees(np.arctan2(*tangent[::-1]))))


def make_edge(a, b, robot_ip, tcp_offset, *, minimum=DEFAULT_MIN_EDGE_DISTANCE_M,
              a_time=None, b_time=None):
    return dict(schema_version=1, frame='Base', units='SI',
                A=dict(tcp_pose=vector(a, 6, 'A点').tolist(), captured_at=a_time or wall_time_fields()),
                B=dict(tcp_pose=vector(b, 6, 'B点').tolist(), captured_at=b_time or wall_time_fields()),
                calibrated_at=wall_time_fields(), robot_ip=str(robot_ip),
                tcp_offset=vector(tcp_offset, 6, 'TCP配置').tolist(),
                min_point_distance_m=float(minimum), **edge_geometry(a, b, minimum))


def validate_edge(data):
    if not isinstance(data, dict) or data.get('frame') != 'Base' or data.get('units') != 'SI':
        raise ValueError('目标直边标定必须使用 Base 坐标系和 SI 单位')
    expected = edge_geometry(data['A']['tcp_pose'], data['B']['tcp_pose'], data['min_point_distance_m'])
    for key, value in expected.items():
        actual = np.asarray(data[key], dtype=float)
        if actual.shape != np.asarray(value).shape or not np.isfinite(actual).all() or not np.allclose(actual, value, rtol=0, atol=1e-10):
            raise ValueError(f'目标直边标定的 {key} 与实际 A/B 坐标不一致')
    vector(data['tcp_offset'], 6, 'TCP配置')
    if not data.get('robot_ip') or not data.get('calibrated_at'):
        raise ValueError('目标直边标定缺少机器人或时间信息')
    return data


def load_edge(path):
    with Path(path).open(encoding='utf-8') as stream:
        return validate_edge(yaml.safe_load(stream))


def _replace_bytes(path, content):
    fd, temporary = tempfile.mkstemp(prefix=path.name+'.', suffix='.tmp', dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        Path(temporary).replace(path)
    finally:
        Path(temporary).unlink(missing_ok=True)


def save_edge(path, data):
    """Validate in memory; back up, atomically replace and verify; roll back on failure."""
    candidate = deepcopy(validate_edge(data))
    path = Path(path).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    content = yaml.safe_dump(candidate, allow_unicode=True, sort_keys=False).encode('utf-8')
    if yaml.safe_load(content) != candidate:
        raise ValueError('目标直边标定序列化校验失败，未保存')
    original = path.read_bytes() if path.exists() else None
    backup = None
    if original is not None:
        backup = path.with_name(f'{path.stem}.backup-{time.time_ns()}{path.suffix}')
        with backup.open('xb') as stream:
            stream.write(original)
    replaced = False
    try:
        _replace_bytes(path, content)
        replaced = True
        reloaded = load_edge(path)
        if reloaded != candidate:
            raise ValueError('目标直边标定文件回读不一致')
    except BaseException:
        if replaced:
            if original is None:
                path.unlink(missing_ok=True)
            else:
                _replace_bytes(path, original)
        raise
    return reloaded, backup
