"""Offline failure evidence, never a calibration acceptance or compensation step.

Short, independent straight lines on either side of a candidate time separate
a sustained discontinuity from smooth changes during tilt. This is a description
of the recorded signal, not an identification of its physical cause.
"""

import numpy as np

from .solver import CalibrationError, rotation_from_rotvec
from .stability import make_reference


def _vector(value, size=6):
    try:
        array = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return None
    return array if array.shape == (size,) and np.isfinite(array).all() else None


def _angle(first, second):
    a = rotation_from_rotvec(first[3:])
    b = rotation_from_rotvec(second[3:])
    return float(np.degrees(np.arccos(np.clip((np.trace(a.T @ b)-1.)/2., -1., 1.))))


def _reference_history(records):
    first_fit = next((r for r in records if r.get('split') == 'fit'), None)
    groups = {}
    for record in records:
        split, pose_id = record.get('split'), record.get('pose_id')
        if not isinstance(pose_id, (int, str)) or isinstance(pose_id, bool):
            continue
        if split == 'reference' or (first_fit is not None and split == 'fit'
                                   and pose_id == first_fit.get('pose_id')):
            groups.setdefault((split, pose_id), []).append(record)
    visits, baseline = [], None
    for (split, pose_id), samples in groups.items():
        try:
            visit = make_reference(samples)
        except CalibrationError as exc:
            visit = dict(exc.diagnostics, status='insufficient_or_unstable', reason=str(exc))
        visit = dict(visit, split=split, pose_id=pose_id)
        if baseline is None and split == 'fit' and visit.get('status') == 'stable':
            baseline = visit
        if baseline is not None and 'mean_raw_wrench' in visit:
            delta = np.asarray(visit['mean_raw_wrench']) - baseline['mean_raw_wrench']
            position_delta = float(np.linalg.norm(
                np.asarray(visit['mean_actual_tcp_pose'])[:3]
                - np.asarray(baseline['mean_actual_tcp_pose'])[:3]))
            orientation_delta = _angle(baseline['mean_actual_tcp_pose'], visit['mean_actual_tcp_pose'])
            visit.update(force_delta_sensor_N=delta[:3].tolist(),
                         force_delta_norm_N=float(np.linalg.norm(delta[:3])),
                         torque_delta_sensor_Nm=delta[3:].tolist(),
                         torque_delta_norm_Nm=float(np.linalg.norm(delta[3:])),
                         position_delta_m=position_delta, orientation_delta_deg=orientation_delta,
                         comparable_actual_pose=position_delta <= .0003 and orientation_delta <= .06)
        visits.append(visit)
    return visits


def _line(times, values):
    """OLS intercept at the candidate boundary, retaining uncertainty from noise."""
    if len(times) < 12 or np.ptp(times) < .12:
        return None
    x = times - times.mean()
    denominator = float(x @ x)
    if denominator <= 0:
        return None
    slope = x @ (values-values.mean(axis=0)) / denominator
    intercept = values.mean(axis=0) - times.mean()*slope
    residual = values - intercept - times[:, None]*slope
    variance = np.sum(residual**2, axis=0) / (len(times)-2)
    uncertainty = np.sqrt(variance * (1./len(times)+times.mean()**2/denominator))
    return intercept, slope, uncertainty


def _step_at(index, times, wrench):
    center = times[index]
    fits = []
    for width in (.2, .4):
        lo, gap_left, gap_right, hi = np.searchsorted(
            times, [center-width, center-.025, center+.025, center+width])
        if hi >= len(times) or np.max(np.diff(times[lo:hi]), initial=0.) > .08:
            return None
        before = _line(times[lo:gap_left]-center, wrench[lo:gap_left])
        after = _line(times[gap_right:hi]-center, wrench[gap_right:hi])
        if before is None or after is None:
            return None
        fits.append((before, after, lo, gap_left, gap_right, hi))
    before, after, lo, gap_left, gap_right, hi = fits[1]
    delta = after[0] - before[0]
    short_delta = fits[0][1][0] - fits[0][0][0]
    uncertainty = np.hypot(before[2], after[2])
    force_norm, torque_norm = float(np.linalg.norm(delta[:3])), float(np.linalg.norm(delta[3:]))
    force_noise = float(np.linalg.norm(uncertainty[:3]))
    torque_noise = float(np.linalg.norm(uncertainty[3:]))
    force_detected = force_norm >= .10 and force_norm > 5.*force_noise
    torque_detected = torque_norm >= .003 and torque_norm > 5.*torque_noise
    if not (force_detected or torque_detected):
        return None
    part = slice(0, 3) if force_detected else slice(3, 6)
    magnitude = float(np.linalg.norm(delta[part]))
    # A smooth curve or brief pulse tends to disagree across these two scales.
    if np.linalg.norm(short_delta[part]-delta[part]) > .45*magnitude:
        return None
    later_lo, later_hi = np.searchsorted(times, [center+.45, center+.75])
    if later_hi >= len(times) or later_hi-later_lo < 15:
        return None
    if np.max(np.diff(times[lo:later_hi]), initial=0.) > .08:
        return None
    # A single straight line extrapolated for 0.75 s would confuse ordinary
    # gravity curvature with loss of persistence, especially for heavier tools.
    # Test a shared smooth cubic background plus one lasting offset across the
    # complete window. These diagnostic fit coefficients never alter samples.
    indices = np.r_[np.arange(lo, gap_left), np.arange(gap_right, later_hi)]
    x = times[indices]-center
    design = np.column_stack([np.ones(len(x)), x, x*x, x*x*x, x > 0.])
    coefficients, _, _, _ = np.linalg.lstsq(design, wrench[indices], rcond=None)
    persistent_delta = coefficients[-1]
    residual = wrench[indices]-design @ coefficients
    variance = np.sum(residual**2, axis=0)/(len(x)-5)
    persistent_uncertainty = np.sqrt(variance*np.linalg.inv(design.T @ design)[-1, -1])
    differences = np.diff(wrench[lo:later_hi], axis=0)
    noise = np.median(np.abs(differences-np.median(differences, axis=0)), axis=0)/(.67449*np.sqrt(2.))
    model_rms = np.sqrt(np.mean(residual**2, axis=0))
    residual_floor = .01 if force_detected else .0003
    if np.linalg.norm(model_rms[part]) > max(residual_floor, 2.*np.linalg.norm(noise[part])):
        return None
    projection = float(persistent_delta[part] @ delta[part] / magnitude)
    if (projection < .60*magnitude or np.linalg.norm(persistent_delta[part]-delta[part]) > .45*magnitude
            or np.linalg.norm(persistent_delta[part]) < 3.*np.linalg.norm(persistent_uncertainty[part])):
        return None
    return dict(index=index, score=magnitude / max(float(np.linalg.norm(uncertainty[part])), 1e-6),
                force_change_detected=force_detected, torque_change_detected=torque_detected,
                before_mean_raw_wrench=wrench[lo:gap_left].mean(axis=0).tolist(),
                after_mean_raw_wrench=wrench[gap_right:hi].mean(axis=0).tolist(),
                before_boundary_raw_wrench=before[0].tolist(), after_boundary_raw_wrench=after[0].tolist(),
                force_delta_sensor_N=delta[:3].tolist(), force_delta_norm_N=force_norm,
                torque_delta_sensor_Nm=delta[3:].tolist(), torque_delta_norm_Nm=torque_norm,
                boundary_uncertainty_raw_wrench=uncertainty.tolist(),
                persistent_boundary_delta_raw_wrench=persistent_delta.tolist(),
                persistence_model_rms_raw_wrench=model_rms.tolist(),
                before_sample_count=int(gap_left-lo), after_sample_count=int(hi-gap_right),
                persistence_sample_count=int(later_hi-later_lo), persistence_until_s=.75)


def _change_candidates(records, end_time):
    # Only the final 20 seconds are searched. Additional 0.75 seconds support
    # the persistence check, including reference frames after the motion ends.
    recent = [r for r in records if end_time-20.4 <= r['host_monotonic'] <= end_time+.75]
    truncated = len(recent) > 10000
    recent = recent[-10000:]
    if len(recent) < 75:
        return [], dict(sample_count=len(recent), truncated=truncated, searched_candidate_count=0)
    times = np.array([r['host_monotonic'] for r in recent])
    wrench = np.array([r['raw_wrench'] for r in recent])
    candidates, checked, next_time = [], 0, -np.inf
    for index, record in enumerate(recent):
        if (record.get('split') not in ('motion', 'settling') or times[index] < end_time-20.
                or times[index] > end_time or times[index] < next_time):
            continue
        checked += 1
        next_time = times[index]+.035
        candidate = _step_at(index, times, wrench)
        if candidate is not None:
            candidates.append(candidate)
    candidates.sort(key=lambda c: c['score'], reverse=True)
    selected = []
    for candidate in candidates:
        index = candidate['index']
        if any(abs(times[index]-times[c['index']]) < .8 for c in selected):
            continue
        # Refine the boundary to recorded sample times near the coarse maximum.
        nearby = np.flatnonzero(np.abs(times-times[index]) <= .04)
        refined = [_step_at(int(i), times, wrench) for i in nearby
                   if recent[i].get('split') in ('motion', 'settling')]
        candidate = max([candidate]+[c for c in refined if c is not None], key=lambda c: c['score'])
        selected.append(candidate)
        if len(selected) == 3:
            break
    for candidate in selected:
        index = candidate['index']
        previous, current = recent[max(0, index-1)], recent[index]
        candidate.update(host_monotonic=float(times[index]), utc_time=current.get('utc_time'),
                         boundary_previous_utc_time=previous.get('utc_time'),
                         split=current.get('split'), time_resolution_s=float(times[index]-times[max(0, index-1)]))
        first_pose, second_pose = _vector(previous.get('actual_tcp_pose')), _vector(current.get('actual_tcp_pose'))
        if first_pose is not None and second_pose is not None:
            candidate.update(before_actual_tcp_pose=first_pose.tolist(), after_actual_tcp_pose=second_pose.tolist(),
                             adjacent_position_delta_m=float(np.linalg.norm(second_pose[:3]-first_pose[:3])),
                             adjacent_orientation_delta_deg=_angle(first_pose, second_pose))
        first_q, second_q = _vector(previous.get('actual_q')), _vector(current.get('actual_q'))
        if first_q is not None and second_q is not None:
            candidate.update(before_actual_q=first_q.tolist(), after_actual_q=second_q.tolist(),
                             adjacent_joint_delta_rad=(second_q-first_q).tolist(),
                             adjacent_max_joint_delta_rad=float(np.max(np.abs(second_q-first_q))))
        del candidate['index']
    return selected, dict(sample_count=len(recent), truncated=truncated, searched_candidate_count=checked,
                         start_monotonic=float(end_time-20.), end_monotonic=float(end_time))


def _continuous_reference_trend(history):
    visits = [v for v in history if v.get('status') == 'stable' and v.get('comparable_actual_pose')]
    if len(visits) < 4:
        return None
    times = np.array([.5*(v['start_monotonic']+v['end_monotonic']) for v in visits])
    if np.ptp(times) < 20.:
        return None
    forces = np.array([v['mean_raw_wrench'][:3] for v in visits])
    delta = forces[-1]-forces[0]
    magnitude = float(np.linalg.norm(delta))
    if magnitude < .10:
        return None
    x = times-times.mean()
    slope = x @ (forces-forces.mean(axis=0)) / float(x @ x)
    residual = forces-forces.mean(axis=0)-x[:, None]*slope
    max_residual = float(np.max(np.linalg.norm(residual, axis=1)))
    increments = np.diff(forces, axis=0) @ (delta/magnitude)
    if max_residual > max(.03, .2*magnitude) or np.mean(increments >= -.01) < .8:
        return None
    return dict(reference_visit_count=len(visits), force_slope_N_per_s=slope.tolist(),
                force_endpoint_delta_N=delta.tolist(), force_endpoint_delta_norm_N=magnitude,
                maximum_line_residual_N=max_residual)


def analyze_failure_records(records, failure_diagnostics=None):
    """Return JSON-compatible evidence without changing records or pass/fail.

    Missing or corrupt samples are counted, never interpolated. A discontinuity
    candidate requires two time scales, noise significance and persistence;
    unobserved physical causes and unobserved changes remain unknown.
    """
    original = list(records)
    valid, invalid, nonmonotonic, missing_pose, missing_q = [], 0, 0, 0, 0
    previous_time = -np.inf
    for record in original:
        if not isinstance(record, dict):
            invalid += 1
            continue
        try:
            timestamp = float(record['host_monotonic'])
        except (KeyError, TypeError, ValueError):
            invalid += 1
            continue
        if not np.isfinite(timestamp) or _vector(record.get('raw_wrench')) is None:
            invalid += 1
            continue
        if timestamp <= previous_time:
            nonmonotonic += 1
            continue
        previous_time = timestamp
        missing_pose += _vector(record.get('actual_tcp_pose')) is None
        missing_q += _vector(record.get('actual_q')) is None
        valid.append(dict(record, host_monotonic=timestamp,
                          raw_wrench=_vector(record['raw_wrench']).tolist()))
    times = np.array([r['host_monotonic'] for r in valid])
    quality = dict(input_record_count=len(original), valid_wrench_record_count=len(valid),
                   invalid_record_count=invalid, nonmonotonic_record_count=nonmonotonic,
                   missing_actual_pose_count=missing_pose, missing_actual_q_count=missing_q,
                   gap_over_80ms_count=int(np.sum(np.diff(times) > .08)),
                   maximum_gap_s=float(np.max(np.diff(times), initial=0.)))
    history = _reference_history([r for r in valid if _vector(r.get('actual_tcp_pose')) is not None])
    references = [v for v in history if v['split'] == 'reference' and 'start_monotonic' in v]
    end_time = references[-1]['start_monotonic'] if references else (times[-1] if len(times) else 0.)
    candidates, search = _change_candidates(valid, end_time)
    trend = _continuous_reference_trend(history)
    classification = 'unknown'
    summary = '现有记录不足以区分持续漂移与突变；原始力变化的物理原因仍需现场检查。'
    if any(c['force_change_detected'] for c in candidates):
        classification = 'abrupt_force_change'
        summary = ('近期运动记录存在持续的原始力突变候选；不能仅凭记录归因于热漂移、'
                   '线缆、接触或传感器内部变化。')
    elif trend is not None:
        classification = 'continuous_drift'
        summary = '多个同实际姿态参考值呈持续变化趋势；这描述信号变化，不能确定其物理原因。'
    elif candidates:
        summary = '近期运动记录存在持续的原始力矩突变候选，原始力变化类型及物理原因尚不能确定。'
    elif isinstance(failure_diagnostics, dict):
        # Existing stationary checks provide useful evidence when no motion was
        # executed. Do not infer a trend from the word "drift" in an error.
        slope = failure_diagnostics.get('force_slope_3sigma_lower_N_per_s')
        endpoint = failure_diagnostics.get('endpoint_force_delta_norm_N')
        if isinstance(slope, (int, float)) and isinstance(endpoint, (int, float)) and slope > .001 and endpoint > .03:
            classification = 'continuous_drift'
            summary = '原地预检记录支持持续变化趋势；这描述信号变化，不能确定其物理原因。'
    return dict(classification=classification, summary=summary, reference_history=history,
                data_quality=quality, change_candidates=candidates, reference_trend=trend,
                candidate_search=search, analysis_policy=dict(
                    diagnostic_only=True, modifies_records=False, controls_acceptance=False,
                    force_candidate_threshold_N=.10, torque_candidate_threshold_Nm=.003,
                    candidate_window_s=20., fitting_half_windows_s=[.2, .4],
                    persistence_window_s=[.45, .75], minimum_noise_ratio=5.,
                    persistence_model='shared smooth cubic plus a single lasting offset; diagnostic only',
                    maximum_search_records=10000, maximum_reported_candidates=3,
                    caveat='候选时间为局部双侧拟合定位，受采样及过渡时间影响；不是物理原因判定。'))
