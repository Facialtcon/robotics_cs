"""Raw-data quality checks for a fixed-bias gravity model; never compensate data.

The preflight trend uses observations already collected at the starting pose.
Reference checks compare separate visits to that same actual pose. Neither
check estimates a new tare, changes samples, or supplies data to the solver.
"""

from dataclasses import asdict, dataclass

import numpy as np

from .solver import CalibrationError, CalibrationLimits, rotation_from_rotvec
from .plan import matrix_to_rotvec


POSITION_LIMIT_M = .0003
ORIENTATION_LIMIT_DEG = .06
REFERENCE_FORCE_LIMIT_N = .10
REFERENCE_TORQUE_LIMIT_NM = .02


@dataclass(frozen=True)
class StabilityLimits:
    """Explicit acquisition quality budget; raw safety limits are separate."""
    reference_force_limit_n: float = REFERENCE_FORCE_LIMIT_N
    reference_torque_limit_nm: float = REFERENCE_TORQUE_LIMIT_NM
    force_slope_limit_n_per_s: float = .001
    endpoint_force_delta_limit_n: float = .03
    warning_reference_force_n: float = REFERENCE_FORCE_LIMIT_N
    warning_force_slope_n_per_s: float = .001
    warning_endpoint_force_delta_n: float = .03

    def __post_init__(self):
        if any(not np.isfinite(value) or value <= 0 for value in asdict(self).values()):
            raise ValueError('stability limits must be finite and positive')
        if (self.warning_reference_force_n > self.reference_force_limit_n
                or self.warning_force_slope_n_per_s > self.force_slope_limit_n_per_s
                or self.warning_endpoint_force_delta_n > self.endpoint_force_delta_limit_n):
            raise ValueError('stability warnings must not exceed rejection limits')


def _arrays(records):
    samples = list(records)
    if not samples:
        raise CalibrationError('静止检查没有原始数据')
    try:
        times = np.asarray([r['host_monotonic'] for r in samples], dtype=float)
        poses = np.asarray([r['actual_tcp_pose'] for r in samples], dtype=float)
        wrench = np.asarray([r['raw_wrench'] for r in samples], dtype=float)
    except (KeyError, TypeError, ValueError) as exc:
        raise CalibrationError(f'静止检查原始数据无效: {exc}') from exc
    if (times.shape != (len(samples),) or poses.shape != (len(samples), 6)
            or wrench.shape != (len(samples), 6)
            or not all(np.isfinite(a).all() for a in (times, poses, wrench))
            or np.any(np.diff(times) <= 0)):
        raise CalibrationError('静止检查数据非有限、维度错误或采样时间未递增')
    return samples, times, poses, wrench


def _pose_summary(poses):
    rotations = np.asarray([rotation_from_rotvec(p[3:]) for p in poses])
    u, _, vt = np.linalg.svd(rotations.mean(axis=0))
    mean_rotation = u @ np.diag([1., 1., np.linalg.det(u @ vt)]) @ vt
    angles = np.degrees(np.arccos(np.clip(
        (np.einsum('nij,ij->n', rotations, mean_rotation) - 1.) / 2., -1., 1.)))
    # O(n) conservative diameter bounds avoid a large pairwise distance matrix
    # while the robot watchdog remains active. Rotation vectors are never
    # averaged: equivalent +/-pi representations must not appear to be zero.
    position_bound = float(np.linalg.norm(np.ptp(poses[:, :3], axis=0)))
    orientation_bound = float(2. * angles.max())
    representative = np.r_[poses[:, :3].mean(axis=0), matrix_to_rotvec(mean_rotation)]
    return dict(mean_actual_tcp_pose=representative.tolist(),
                mean_rotation_base_tool=mean_rotation.tolist(),
                position_span_bound_m=position_bound,
                orientation_span_bound_deg=orientation_bound,
                position_limit_m=POSITION_LIMIT_M,
                orientation_limit_deg=ORIENTATION_LIMIT_DEG)


def _check_pose(summary):
    if (summary['position_span_bound_m'] > POSITION_LIMIT_M
            or summary['orientation_span_bound_deg'] > ORIENTATION_LIMIT_DEG):
        summary['status'] = 'rejected_pose'
        raise CalibrationError('静止检查期间实际 TCP 姿态变化过大；不能据此判断原始力漂移', summary)


def make_reference(records):
    """Summarize one stopped reference visit, preserving raw forces and torques."""
    samples, times, poses, wrench = _arrays(records)
    limits = CalibrationLimits()
    if len(samples) < limits.min_samples_per_pose or times[-1] - times[0] < limits.min_pose_duration_s:
        raise CalibrationError('回中参考采样数量或持续时间不足',
                               dict(sample_count=len(samples), duration_s=float(times[-1]-times[0])))
    std = wrench.std(axis=0, ddof=1)
    mean = wrench.mean(axis=0)
    peaks = np.abs(wrench - mean).max(axis=0)
    result = dict(status='stable', sample_count=len(samples), duration_s=float(times[-1]-times[0]),
                  start_monotonic=float(times[0]), end_monotonic=float(times[-1]),
                  utc_start=samples[0].get('utc_time'), utc_end=samples[-1].get('utc_time'),
                  mean_raw_wrench=mean.tolist(), raw_wrench_std=std.tolist(),
                  raw_wrench_peak_deviation=peaks.tolist(), **_pose_summary(poses))
    _check_pose(result)
    if (std[:3].max() > limits.max_force_std_n or std[3:].max() > limits.max_torque_std_nm
            or peaks[:3].max() > limits.max_force_peak_deviation_n
            or peaks[3:].max() > limits.max_torque_peak_deviation_nm):
        result['status'] = 'rejected_wrench_stability'
        raise CalibrationError('回中参考原始力/矩在单次采样内不稳定', result)
    return result


def check_reference(reference, current_records, limits=None):
    """Compare a fresh stopped return against the immutable first reference."""
    limits = limits or StabilityLimits()
    current = make_reference(current_records)
    try:
        initial_force = np.asarray(reference['mean_raw_wrench'], dtype=float)
        initial_pose = np.asarray(reference['mean_actual_tcp_pose'], dtype=float)
        initial_rotation = np.asarray(reference['mean_rotation_base_tool'], dtype=float)
        if (initial_force.shape != (6,) or initial_pose.shape != (6,)
                or initial_rotation.shape != (3, 3)
                or not all(np.isfinite(v).all() for v in (initial_force, initial_pose, initial_rotation))):
            raise ValueError('invalid reference summary')
    except (KeyError, TypeError, ValueError) as exc:
        raise CalibrationError(f'初始回中参考数据无效: {exc}') from exc
    position_delta = float(np.linalg.norm(np.asarray(current['mean_actual_tcp_pose'])[:3]-initial_pose[:3]))
    orientation_delta = float(np.degrees(np.arccos(np.clip(
        (np.trace(initial_rotation.T @ np.asarray(current['mean_rotation_base_tool']))-1.)/2., -1., 1.))))
    delta = np.asarray(current['mean_raw_wrench']) - initial_force
    result = dict(status='stable', initial_reference=reference, current_reference=current,
                  position_delta_m=position_delta, orientation_delta_deg=orientation_delta,
                  position_limit_m=POSITION_LIMIT_M, orientation_limit_deg=ORIENTATION_LIMIT_DEG,
                  force_delta_sensor_N=delta[:3].tolist(), force_delta_norm_N=float(np.linalg.norm(delta[:3])),
                  torque_delta_sensor_Nm=delta[3:].tolist(), torque_delta_norm_Nm=float(np.linalg.norm(delta[3:])),
                  force_limit_N=limits.reference_force_limit_n, torque_limit_Nm=limits.reference_torque_limit_nm,
                  stability_limits=asdict(limits), warnings=[])
    if position_delta > POSITION_LIMIT_M or orientation_delta > ORIENTATION_LIMIT_DEG:
        result['status'] = 'rejected_pose'
        raise CalibrationError('回中实际姿态与初始参考不一致；不能据此判断原始力漂移', result)
    if (result['force_delta_norm_N'] > limits.reference_force_limit_n
            or result['torque_delta_norm_Nm'] > limits.reference_torque_limit_nm):
        result['status'] = 'rejected_raw_drift'
        raise CalibrationError(
            f"同姿态原始力/矩漂移，固定零偏模型不成立：力差 {result['force_delta_norm_N']:.4f} N"
            f"（上限 {limits.reference_force_limit_n:.2f} N），矩差 {result['torque_delta_norm_Nm']:.5f} Nm"
            f"（上限 {limits.reference_torque_limit_nm:.2f} Nm）；停止标定，不扣零、不保存安装参数", result)
    if result['force_delta_norm_N'] > limits.warning_reference_force_n:
        result['status'] = 'within_coarse_budget'
        result['warnings'].append('回中原始力变化超过保守档容限；仅允许继续粗标定采集，须通过最终拟合、独立验证和角度不确定度检查。')
    return result


def check_stationary_trend(records, limits=None):
    """Reject an observed force trend during preflight, using at most 30 seconds.

    A short preflight is explicitly inconclusive; later reference comparisons
    still enforce the same fixed bias assumption. OLS uncertainty describes
    sampling noise only and is reported together with endpoint differences.
    """
    limits = limits or StabilityLimits()
    records = list(records)
    if not records:
        return dict(status='insufficient_duration', sample_count=0, duration_s=0.)
    samples, times, poses, wrench = _arrays(records)
    selected = times >= times[-1] - 30.
    first = int(np.flatnonzero(selected)[0])
    samples, times, poses, wrench = samples[first:], times[first:], poses[first:], wrench[first:]
    duration = float(times[-1] - times[0])
    result = dict(status='insufficient_duration', sample_count=len(samples), duration_s=duration,
                  min_duration_s=20., max_window_s=30., min_sample_count=100,
                  utc_start=samples[0].get('utc_time'), utc_end=samples[-1].get('utc_time'),
                  stability_limits=asdict(limits), warnings=[])
    if duration < 20. or len(samples) < 100:
        return result
    result.update(_pose_summary(poses))
    _check_pose(result)
    x = times - times.mean()
    force = wrench[:, :3]
    denominator = float(x @ x)
    slope = x @ (force - force.mean(axis=0)) / denominator
    residual = force - force.mean(axis=0) - x[:, None] * slope
    slope_uncertainty = float(np.sqrt(np.sum(residual**2) / (len(samples)-2) / denominator))
    slope_norm = float(np.linalg.norm(slope))
    lower_bound = max(0., slope_norm - 3. * slope_uncertainty)
    first_force = force[times <= times[0]+5.].mean(axis=0)
    last_force = force[times >= times[-1]-5.].mean(axis=0)
    endpoint_delta = last_force - first_force
    result.update(status='stable', force_slope_N_per_s=slope.tolist(),
                  force_slope_norm_N_per_s=slope_norm,
                  force_slope_uncertainty_N_per_s=slope_uncertainty,
                  force_slope_3sigma_lower_N_per_s=lower_bound,
                  endpoint_first_mean_force_N=first_force.tolist(),
                  endpoint_last_mean_force_N=last_force.tolist(),
                  endpoint_force_delta_N=endpoint_delta.tolist(),
                  endpoint_force_delta_norm_N=float(np.linalg.norm(endpoint_delta)),
                  force_slope_limit_N_per_s=limits.force_slope_limit_n_per_s,
                  endpoint_force_delta_limit_N=limits.endpoint_force_delta_limit_n)
    if lower_bound > limits.force_slope_limit_n_per_s and result['endpoint_force_delta_norm_N'] > limits.endpoint_force_delta_limit_n:
        result['status'] = 'rejected_raw_drift'
        raise CalibrationError(
            f'原地原始力持续漂移，固定零偏模型不稳定：趋势 {slope_norm*60.:.4f} N/min，'
            f"首尾均值差 {result['endpoint_force_delta_norm_N']:.4f} N；"
            '停止标定，待原始读数稳定后重新开始，不扣零、不保存安装参数', result)
    if lower_bound > limits.warning_force_slope_n_per_s and result['endpoint_force_delta_norm_N'] > limits.warning_endpoint_force_delta_n:
        result['status'] = 'within_coarse_budget'
        result['warnings'].append('原地趋势超过保守档容限；当前仅在粗标定采集预算内，不代表零偏已稳定，后续不扣除漂移。')
    return result


def check_recorded_stability(records, limits=None):
    """Repeat recorded acquisition checks before any offline gravity fit.

    Legacy files without explicit reference samples remain readable. Settling
    and motion frames are never guessed to be new reference visits. Recorded
    preflight evidence is checked even when no tilt or reference was acquired.
    """
    samples = list(records)
    if any(not isinstance(record, dict) for record in samples):
        raise CalibrationError('离线稳定性检查需要原始采样记录对象')
    diagnostics = dict(preflight=None, reference_baseline=None, references=[], reference_count=0)
    try:
        diagnostics['preflight'] = check_stationary_trend(
            [record for record in samples if record.get('split') == 'preflight'], limits)
    except CalibrationError as exc:
        diagnostics['preflight'] = dict(exc.diagnostics)
        exc.diagnostics = {**exc.diagnostics, 'recorded_stability': diagnostics}
        raise
    references = {}
    for record in samples:
        if record.get('split') != 'reference':
            continue
        pose_id = record.get('pose_id')
        if isinstance(pose_id, bool) or not isinstance(pose_id, (int, str)):
            raise CalibrationError('离线回中参考缺少有效 pose_id', {'recorded_stability': diagnostics})
        references.setdefault(pose_id, []).append(record)
    if not references:
        return diagnostics
    first_fit = next((record for record in samples if record.get('split') == 'fit'), None)
    if first_fit is None:
        raise CalibrationError('离线数据存在回中参考，但缺少初始拟合姿态', {'recorded_stability': diagnostics})
    first_id = first_fit.get('pose_id')
    if isinstance(first_id, bool) or not isinstance(first_id, (int, str)):
        raise CalibrationError('离线初始拟合姿态缺少有效 pose_id', {'recorded_stability': diagnostics})
    initial_records = [record for record in samples
                       if record.get('split') == 'fit' and record.get('pose_id') == first_id]
    checking_pose_id = first_id
    try:
        baseline = make_reference(initial_records)
        diagnostics['reference_baseline'] = dict(baseline, pose_id=first_id)
        for pose_id, visit in references.items():
            checking_pose_id = pose_id
            result = check_reference(baseline, visit, limits)
            diagnostics['references'].append(dict(result, pose_id=pose_id))
            diagnostics['reference_count'] += 1
    except CalibrationError as exc:
        diagnostics['failed_check'] = dict(exc.diagnostics, pose_id=checking_pose_id)
        exc.diagnostics = {**exc.diagnostics, 'recorded_stability': diagnostics}
        raise
    return diagnostics
