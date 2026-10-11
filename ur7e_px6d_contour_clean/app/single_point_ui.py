"""Operator-facing single-point displays; units/rounding never alter data."""
import unicodedata

import numpy as np

from calibration.single_point import (DEFAULT_MIN_APPROACH_DISTANCE_M, calibration_distance_m,
                                      group_calibration_issue)
from robot.control_mode import read_control_mode
from robot.rtde_controller import RobotError


def xyz_mm(pose):
    return '，'.join(f'{axis}={float(value)*1000:.3f} mm' for axis, value in zip('XYZ', pose[:3]))


def pose_text(pose):
    angles = '，'.join(f'{float(value):.3f}' for value in pose[3:])
    return f'{xyz_mm(pose)}；姿态旋转向量（rad）=[{angles}]'


def direction_angle(direction):
    return float(np.degrees(np.arctan2(direction[1], direction[0])))


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
        issue = group_calibration_issue(data, name, settings.get('min_approach_calibration_distance_m',
                                                               DEFAULT_MIN_APPROACH_DISTANCE_M))
        rows.append([f'P0-{name}', f'{group["tcp_pose"][0]*1000:.3f}', f'{group["tcp_pose"][1]*1000:.3f}',
            f'{calibration_distance_m(reference, group["tcp_pose"])*1000:.3f}',
            '—' if issue else f'{direction_angle(group["direction_xy"]):.2f}°',
            ('距离过近，需要重标定' if '距离过近' in issue else '需要重标定') if issue else '有效'])
    _table(['名称', 'X (mm)', 'Y (mm)', 'XY距离 (mm)', '接近方向角（Base +X）', '标定状态'], rows)
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
    print('\n========== 机器人控制模式 ==========\n正在通过 Dashboard 29999 只读查询…', flush=True)
    status = read_control_mode(robot_ip)
    if status.mode == 'remote':
        print('机器人连接成功\n当前模式：Remote（远程控制）\n可以准备执行电脑控制实验。实际运动仍需单独确认。', flush=True)
    elif status.mode == 'local':
        print('机器人连接成功\n当前状态：未处于Remote\n可能为Local模式或未启用Remote功能。\n'
              '可进行人工示教与只读标定。自动运动前需要切换Remote。', flush=True)
    else:
        print(f'模式查询失败\n状态：未知\n具体原因：{status.detail}', flush=True)
    return status


def require_remote_mode(robot_ip):
    status = report_control_mode(robot_ip)
    if status.mode != 'remote':
        raise RobotError('当前未确认 Remote 模式。请在示教器中切换 Remote，'
                         '然后重新进行运动路径核验与确认。' + status.detail)
    return status


def show_trial_result(result):
    labels = dict(contact='接触阈值触发，已完成停车记录', no_contact='距离或时间预算内未找到接触', aborted='实验失败/已中止')
    if result['status'] == 'no_contact' and result.get('reason') == 'NO_CONTACT_DISTANCE':
        print('未找到接触：搜索距离耗尽。')
    else:
        print(f'实验结果：{labels.get(result["status"], result["status"])}')
    if result.get('reason'):
        print(f'停止原因：{result["reason"]}')
    if result.get('fault'):
        print(f'具体原因：{result["fault"]}')
    failures = dict(BRAKING_API_FAILURE='制动指令失败或被 SDK 拒绝',
                    STOP_TELEMETRY_INCOMPLETE='制动过程数据记录未完成',
                    STANDSTILL_NOT_CONFIRMED='未能确认实际停稳',
                    STANDSTILL_REVOKED='停稳后再次发生运动',
                    POST_STOP_RECORDING_INCOMPLETE='停车数据记录未完成')
    if result.get('stop_failure_kind'):
        print('停车阶段结果：' + failures.get(result['stop_failure_kind'], result['stop_failure_kind']))
    if result.get('actual_standstill_sec') is not None:
        print('实际停稳已确认；停稳后记录：' + ('完成' if result.get('post_stop_record_completed') else '未完成'))
    if result.get('timeout_phase'):
        phases = dict(BRAKING_CONFIRMATION='制动与实际停稳确认', POST_STOP_RECORDING='停稳后的数据记录')
        print('超时阶段：' + phases.get(result['timeout_phase'], result['timeout_phase']))
    for error in result.get('cleanup_errors', []):
        print(f'设备关闭失败：{error}')
    if result.get('actual_stop_pose'):
        print(f'实际停车坐标：{xyz_mm(result["actual_stop_pose"])}')
