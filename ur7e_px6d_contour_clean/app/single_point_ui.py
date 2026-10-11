"""Operator-facing single-point displays; units/rounding never alter data."""
import unicodedata

import numpy as np

from robot.control_mode import read_control_mode
from robot.rtde_controller import RobotError


def xyz_mm(pose):
    return '，'.join(f'{axis}={float(value)*1000:.3f} mm' for axis, value in zip('XYZ', pose[:3]))


def pose_text(pose):
    angles = '，'.join(f'{float(value):.3f}' for value in pose[3:])
    return f'{xyz_mm(pose)}；姿态旋转向量（rad）=[{angles}]'


def direction_angle(direction):
    return float(np.degrees(np.arctan2(direction[1], direction[0])))


def path_reviewed(group, settings):
    review = group.get('path_review', {})
    return bool(review.get('note') and review.get('reviewed_at') and
                float(review.get('max_distance_m') or 0) >= settings['max_search_distance_m'])


def _table(headers, rows):
    def width(value):
        return sum(2 if unicodedata.east_asian_width(c) in ('W', 'F') else 1 for c in value)
    sizes = [max(width(row[i]) for row in [headers, *rows]) for i in range(len(headers))]
    def output(row):
        print(' | '.join(value + ' '*(size-width(value)) for value, size in zip(row, sizes)))
    output(headers)
    print('-+-'.join('-'*size for size in sizes))
    for row in rows:
        output(row)


def show_calibration_table(data, settings):
    reference = data.get('P_ref')
    print('\n========== 中文标定信息表 ==========')
    print('固定参考点 P_ref：' + ('未标定' if reference is None else xyz_mm(reference)))
    groups = data['groups']
    print(f'已标定 P0 数量：{len(groups)}')
    if not groups:
        print('暂无已保存 P0；请先标定 P_ref，再新增 P0。')
        return
    rows = []
    for name, group in groups.items():
        rows.append([f'P0-{name}', f'{group["tcp_pose"][0]*1000:.3f}', f'{group["tcp_pose"][1]*1000:.3f}',
            f'{direction_angle(group["direction_xy"]):.2f}°', '已保存',
            '已完成' if path_reviewed(group, settings) else '未完成'])
    _table(['名称', 'X (mm)', 'Y (mm)', '方向角（Base +X）', '保存状态', '路径核验'], rows)
    print('方向角以 Base +X 为 0°，逆时针为正；P0 仅使用 XY，扫描高度统一为 P_ref.z。')


def show_motion_path(current, target, segments):
    print(f'当前实际 TCP：{pose_text(current)}')
    print(f'生成的实验起点：{pose_text(target)}')
    labels = dict(VERTICAL_RETREAT='安全抬升', ALIGN_PROBE_ORIENTATION='调正探针',
                  MOVE_ABOVE_START='移动至 P0 上方', DESCEND_TO_START='下降到统一实验高度')
    print('完整运动路径（坐标 mm，姿态 rad）：')
    for index, (phase, point, speed) in enumerate(segments, 1):
        print(f'{index}. {labels[phase]} ({phase})：{pose_text(point)}；速度 {speed*1000:.3f} mm/s')


def report_control_mode(robot_ip):
    status = read_control_mode(robot_ip)
    labels = dict(local='Local / Remote 未启用', remote='Remote', unknown='未知（未确认 Remote）')
    print(f'只读控制模式检查：{labels.get(status.mode, "未知")}；{status.detail}', flush=True)
    return status


def require_remote_mode(robot_ip):
    status = report_control_mode(robot_ip)
    if status.mode != 'remote':
        raise RobotError('未确认 Remote，未授权自动运动；请在示教器切换 Remote 后重新核验路径。' + status.detail)
    return status


def show_trial_result(result):
    labels = dict(contact='接触阈值触发，已完成停车记录', no_contact='距离或时间预算内未找到接触', aborted='实验失败/已中止')
    print(f'实验结果：{labels.get(result["status"], result["status"])}')
    if result.get('reason'):
        print(f'停止原因：{result["reason"]}')
    if result.get('fault'):
        print(f'具体原因：{result["fault"]}')
    for error in result.get('cleanup_errors', []):
        print(f'设备关闭失败：{error}')
    if result.get('actual_stop_pose'):
        print(f'实际停车坐标：{xyz_mm(result["actual_stop_pose"])}')
