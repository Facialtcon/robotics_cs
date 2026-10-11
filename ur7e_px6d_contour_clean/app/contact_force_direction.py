"""Optional measurement analysis only; never supplies commands or control guards."""
from copy import deepcopy
from pathlib import Path

import numpy as np
import yaml

from calibration.target_edge import load_edge, validate_edge, vector
from robot.tcp_identity import tcp_offsets_match


def force_angle(force_xy, tangent_xy, *, min_force_N=1e-6):
    force = vector(force_xy, 2, 'Base XY力')
    tangent = vector(tangent_xy, 2, 'Base XY边界切向')
    magnitude, tnorm = float(np.linalg.norm(force)), float(np.linalg.norm(tangent))
    if not np.isfinite([magnitude, tnorm]).all() or magnitude <= min_force_N or tnorm <= 1e-12:
        raise ValueError('力模长过小或边界切向无效，无法计算夹角')
    theta = float(np.degrees(np.arccos(np.clip(abs(np.dot(force, tangent/tnorm))/magnitude, 0., 1.))))
    return dict(force_direction_deg=float(np.degrees(np.arctan2(force[1], force[0]))),
                angle_to_edge_deg=theta, perpendicular_error_deg=90.-theta, fxy_N=magnitude)


def snapshot_edge(path, project):
    """A missing/incompatible edge only disables analysis, never the experiment."""
    path = Path(path).resolve()
    snapshot = dict(source_path=str(path), status='unavailable', calibration=None)
    if not path.exists():
        snapshot['reason'] = '未标定目标直边，仅跳过边界夹角分析'
        return snapshot
    try:
        snapshot['calibration'] = deepcopy(load_edge(path))
        edge = snapshot['calibration']
        if edge['robot_ip'] != project['robot']['robot_ip'] or not tcp_offsets_match(
                edge['tcp_offset'], project['tcp']['offset'], project['tcp']['offset_tolerance']):
            raise ValueError('目标直边标定的机器人IP或TCP配置与本次实验不一致')
        snapshot['status'] = 'available'
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        snapshot['reason'] = f'目标直边标定不可用：{exc}；仅跳过夹角分析'
    return snapshot


def analyze_contact(snapshot, result):
    """Use the saved threshold measurement frame, never settling samples/averages."""
    analysis = dict(schema_version=1, frame='Base', status='unavailable',
        target_edge=deepcopy(snapshot), threshold_trigger_sec=result.get('threshold_trigger_sec'),
        force_sample_sec=result.get('force_trigger_sample_sec'),
        actual_at_threshold=deepcopy(result.get('actual_at_threshold')),
        force_at_threshold=deepcopy(result.get('force_at_threshold')),
        time_meaning='阈值判断实际使用的测量帧及主机时间；不是连续物理时间中首次接触的精确瞬间',
        arrow_meaning='Base XY滤波后力方向示意；长度归一化，与力大小无关')
    if snapshot.get('status') != 'available':
        analysis['reason'] = snapshot.get('reason', '未标定目标直边')
        return analysis
    try:
        edge = validate_edge(snapshot['calibration'])
        trigger = result.get('force_at_threshold')
        if not trigger or result.get('threshold_trigger_sec') is None:
            raise ValueError('没有记录力阈值触发帧，无法计算夹角')
        if trigger.get('frame') != 'Base':
            raise ValueError('触发力的 Base 坐标系未明确记录，无法计算夹角')
        filtered = vector(trigger['processed_wrench_base'], 6, '触发帧滤波后Base力')[:2]
        analysis.update(filtered_base_xy_N=filtered.tolist(), **force_angle(filtered, edge['tangent_base_xy']))
        before = trigger.get('unfiltered_force_base')
        analysis['unfiltered_base_xy_N'] = None
        analysis['unfiltered_analysis'] = None
        if before is not None:
            try:
                before_xy = vector(before, 3, '触发帧滤波前Base力')[:2]
                analysis['unfiltered_base_xy_N'] = before_xy.tolist()
                analysis['unfiltered_analysis'] = force_angle(before_xy, edge['tangent_base_xy'])
            except ValueError as exc:
                analysis['unfiltered_reason'] = str(exc)
        else:
            analysis['unfiltered_reason'] = '旧记录未保存同一触发帧的滤波前Base力，未用Sensor力替代'
        analysis.update(status='available', boundary_direction_deg=edge['direction_deg'])
    except (ValueError, KeyError, TypeError) as exc:
        analysis.update(status='unavailable', reason=str(exc))
    return analysis
