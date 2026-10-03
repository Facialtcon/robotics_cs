"""Deterministic bounded tilts about Base horizontal axes, with spoke returns."""

from dataclasses import asdict, dataclass
import math

import numpy as np

from robot.rtde_controller import _rotvec_to_matrix, _orientation_distance


@dataclass(frozen=True)
class Limits:
    # Fixed limits: failed identification never expands these automatically.
    inner_tilt_deg: float = 12.
    outer_tilt_deg: float = 25.
    interleave_tilts: bool = False
    max_translation_m: float = .002
    max_tilt_rad: float = math.radians(26)  # 25° targets + bounded tracking margin
    max_linear_speed: float = .005
    max_angular_speed: float = .04
    max_joint_speed: float = .15
    command_speed: float = .005
    acceleration: float = .03
    stop_deceleration: float = .2
    observation_timeout_sec: float = .08
    sample_period_sec: float = .01
    joint_step_rad: float = .12
    joint_excursion_rad: float = .8
    segment_timeout_sec: float = 120.
    total_timeout_sec: float = 2700.
    settle_hold_sec: float = .7
    settle_linear_speed: float = .0001
    settle_angular_speed: float = .001
    position_tolerance_m: float = .0005
    orientation_tolerance_rad: float = .003
    samples_per_pose: int = 80

    def __post_init__(self):
        angles = (self.inner_tilt_deg, self.outer_tilt_deg, self.max_tilt_rad)
        if any(not math.isfinite(value) for value in angles):
            raise ValueError('calibration tilt limits must be finite')
        if not 0 < self.inner_tilt_deg < self.outer_tilt_deg <= 45:
            raise ValueError('calibration tilt limits require 0 < inner < outer <= 45 degrees')
        minimum_hard_limit = math.radians(self.outer_tilt_deg + 1.)
        if not minimum_hard_limit - 1e-12 <= self.max_tilt_rad <= math.radians(46.) + 1e-12:
            raise ValueError('calibration tilt hard limit must include 1 degree tracking margin and not exceed 46 degrees')
        if not isinstance(self.interleave_tilts, bool):
            raise ValueError('interleave_tilts must be a boolean')

    def as_dict(self):
        return asdict(self)


def matrix_to_rotvec(matrix):
    r = np.asarray(matrix, dtype=float)
    angle = float(np.arccos(np.clip((np.trace(r) - 1) / 2, -1, 1)))
    if angle < 1e-9:
        return np.zeros(3)
    if np.pi - angle < 1e-6:
        _, vectors = np.linalg.eigh((r + r.T) / 2)
        axis = vectors[:, -1]
        skew = np.array([r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1]])
        if np.dot(axis, skew) < 0:
            axis = -axis
        return angle * axis
    return angle / (2 * np.sin(angle)) * np.array(
        [r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1]])


def make_plan(actual_start_pose, limits=None):
    limits = limits or Limits()
    start = np.asarray(actual_start_pose, dtype=float)
    if start.shape != (6,) or not np.isfinite(start).all():
        raise ValueError('start pose must have six finite components')
    orientation = _rotvec_to_matrix(start[3:])
    plan = [dict(pose_id=0, split='fit', tilt_deg=0., azimuth_deg=0.,
                 target=start.tolist(), acquire=True)]
    # Center + two amplitudes identify the centered gravity's third dimension.
    # Outer diagonal poses are withheld in their entirety before collecting data.
    if limits.interleave_tilts:
        # Keep opposing directions close in time, with both amplitudes sampled
        # before moving to the next direction. This never changes raw forces.
        direction_order = (0, 180, 90, 270, 45, 225, 135, 315)
        schedule = [(tilt, azimuth) for azimuth in direction_order
                    for tilt in (limits.inner_tilt_deg, limits.outer_tilt_deg)]
    else:
        schedule = [(tilt, azimuth) for tilt in (limits.inner_tilt_deg, limits.outer_tilt_deg)
                    for azimuth in range(0, 360, 45)]
    for tilt, azimuth in schedule:
        axis = np.array([math.cos(math.radians(azimuth)), math.sin(math.radians(azimuth)), 0.])
        target = start.copy()
        target[3:] = matrix_to_rotvec(_rotvec_to_matrix(axis * math.radians(tilt)) @ orientation)
        pose_id = sum(p['acquire'] for p in plan)
        split = 'validation' if tilt == limits.outer_tilt_deg and azimuth % 90 else 'fit'
        plan.append(dict(pose_id=pose_id, split=split, tilt_deg=tilt,
                         azimuth_deg=float(azimuth), target=target.tolist(), acquire=True))
        # Explicitly displayed, preflighted return after each spoke, including last.
        plan.append(dict(pose_id=-1, split='motion', tilt_deg=0., azimuth_deg=0.,
                         target=start.tolist(), acquire=False))
    return plan


def interpolate_path(start, plan, step_rad=math.radians(1)):
    """Same geodesic orientation interpolation as each Cartesian moveL segment."""
    current = np.asarray(start, dtype=float)
    path = [current.tolist()]
    for point in plan:
        target = np.asarray(point['target'], dtype=float)
        r0 = _rotvec_to_matrix(current[3:])
        relative = matrix_to_rotvec(r0.T @ _rotvec_to_matrix(target[3:]))
        count = max(1, math.ceil(np.linalg.norm(relative) / step_rad),
                    math.ceil(np.linalg.norm(target[:3]-current[:3]) / .001))
        for i in range(1, count+1):
            f = i / count
            pose = current.copy()
            pose[:3] = current[:3] + f * (target[:3]-current[:3])
            pose[3:] = matrix_to_rotvec(r0 @ _rotvec_to_matrix(f * relative))
            path.append(pose.tolist())
        current = target
    return path


def at_target(actual, target, limits):
    return (np.linalg.norm(np.asarray(actual)[:3]-np.asarray(target)[:3]) <= limits.position_tolerance_m
            and _orientation_distance(np.asarray(actual)[3:], np.asarray(target)[3:]) <= limits.orientation_tolerance_rad)


def describe_plan(start, plan, limits, tcp_offset):
    flange_sweep = 2 * np.linalg.norm(np.asarray(tcp_offset)[:3]) * np.sin(limits.max_tilt_rad / 2)
    rows = [
        f'实际起始 TCP [m,rad]: {np.asarray(start).tolist()}',
        '重力依据：Base −Z；开始前必须确认机器人基座水平、工具悬空无接触。',
        '固定 TCP 位置，绕 Base 水平轴倾斜；每个倾斜点沿原路径返回起始姿态。',
        '姿态编号 / 倾角° / 倾斜轴方位° / 数据用途：',
    ]
    rows.extend(f"  {p['pose_id']:2d} / {p['tilt_deg']:4.0f} / {p['azimuth_deg']:3.0f} / {p['split']} → 起始姿态"
                for p in plan if p['acquire'])
    rows.extend([
        f'命令平移 0、两档倾角 {limits.inner_tilt_deg:g}° / {limits.outer_tilt_deg:g}°、倾角最大 {limits.outer_tilt_deg:g}°；实际平移上限 {limits.max_translation_m*1000:g} mm，转角硬限 {math.degrees(limits.max_tilt_rad):g}°（含跟踪余量）。',
        f'实测速度上限：平移 {limits.max_linear_speed:g} m/s，角速度 {limits.max_angular_speed:g} rad/s，关节 {limits.max_joint_speed:g} rad/s。',
        f'moveL speed={limits.command_speed:g}，acceleration={limits.acceleration:g}；每段 ≤{limits.segment_timeout_sec:g}s，总执行 ≤{limits.total_timeout_sec:g}s。',
        f'关节预检：路径每 ≤1° / 1mm 采样，相邻关节差 ≤{limits.joint_step_rad:g}rad，各关节相对起点 ≤{limits.joint_excursion_rad:g}rad（{math.degrees(limits.joint_excursion_rad):.1f}°）。',
        f'仅 TCP 偏置引起的法兰位置变化可达 {flange_sweep*1000:.1f} mm；整套工具的扫掠范围还取决于实际外形。',
        '确认整套工具、法兰、各机械臂连杆和线缆全程空间及余量；IK/安全限位预检不等于碰撞检测。',
        '正常完成按上述路径回到起始标定姿态；异常立即停止，不自动返回，不进入扫描。',
        f'预检期间检查原地力漂移；每次回中后再采 {limits.samples_per_pose} 帧检查固定零偏重复性，不采零、不扣除参考力。',
    ])
    return '\n'.join(rows)
