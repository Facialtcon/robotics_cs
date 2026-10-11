#!/usr/bin/env python3
"""Multi-Directional Single-Point Contact Experiment: offline by default."""
import argparse
import csv
import json
from pathlib import Path
import time

import numpy as np

from app.operator_input import OperatorKeyboard, confirm_enter
from app.single_point_config import load_settings, prepare, validate_speed
from app.single_point_runtime import EXTRA_FIELDS, FreshForceReader, SinglePointTrial
from app.contact_force_direction import snapshot_edge, analyze_contact
from app.single_point_startup import plan_return, prepare_startup, SinglePointRetreatMonitor
from app.single_point_ui import (xyz_mm, pose_text, direction_angle, show_calibration_table,
                                show_motion_path, report_control_mode, require_remote_mode, show_trial_result)
from calibration.single_point import (LIMITATION, load_calibration, save_group, set_reference,
                                      validate_group, write_calibration, experiment_start_pose,
                                      DEFAULT_MIN_APPROACH_DISTANCE_M, group_calibration_issue, validate_calibration)
from calibration.target_edge import make_edge, save_edge, DEFAULT_MIN_EDGE_DISTANCE_M
from experiment_logging.paths import wall_time_fields
from config.loader import load_config, runtime_robot_config
from experiment_logging.continuous_writer import ContinuousLogWriter
from experiment_logging.data_logger import ExperimentLogger
from robot.rtde_controller import URRTDEController, RobotError
from robot.rtde_controller import _orientation_distance
from robot.tcp_identity import read_tcp_offset_readonly, tcp_offsets_match
from safety.safe_return import SafeReturnExecutor
from sensor.force_preprocess import WrenchPreprocessor
from sensor.px6d_reader import PX6DReader

ROOT = Path(__file__).resolve().parent


def make_reader(project):
    s = project['sensor']
    return PX6DReader(s['serial_port'], s['baudrate'], s['timeout_sec'], s['poll_rate_hz'], s['startup_delay_sec'])


def show_group(data, name, settings, speed, project):
    group = data['groups'][name]
    print(f'实验组：P0-{name}')
    print(f'接近方向：{direction_angle(group["direction_xy"]):.2f}°（相对 Base +X，逆时针为正）')
    print(f'接近速度：{speed*1000:.3f} mm/s')
    print(f'接触阈值：{settings["contact_threshold_N"]:.1f} N')
    print(f'正常停止：力达到 {settings["contact_threshold_N"]:.1f} N')


def record_trial_termination(logger, result):
    """Supply single-point evidence explicitly to the unchanged shared logger."""
    from experiment_logging.termination import TerminationReason
    recorder = logger.termination
    if recorder is None or recorder.record is not None:
        return
    events = result.get('safety_events', [])
    if result.get('status') == 'contact':
        code = TerminationReason.SUCCESS
    elif events:
        code = TerminationReason.STOP_FORCE_LIMIT
    elif result.get('force_read_error'):
        code = TerminationReason.STOP_SENSOR_ERROR
    elif result.get('stop_failure_kind'):
        code = TerminationReason.STOP_MOTION_ERROR
    elif result.get('reason') == 'EXPERIMENT_TIME_LIMIT':
        code = TerminationReason.STOP_TIME_LIMIT
    elif result.get('reason') == 'USER_ABORT':
        code = TerminationReason.STOP_USER_REQUEST
    else:
        return  # Preserve shared classification for other device/config faults.
    context = dict(single_point_result=result)
    if events:
        first = events[0]
        context.update(raw=first['raw_wrench_sensor'], processed=first['processed_wrench_base'],
                       tcp_pose=first['tcp_pose'], phase=first['phase'], timestamp=first['first_sec'])
    recorder.set_stop_reason(code, detail=result.get('fault') or result.get('reason', ''),
                             source='single_point.final_result', state=result.get('state', 'STOP'), **context)


def capture_pose(project, *, point_name='标定点', confirm=confirm_enter, controller_factory=URRTDEController):
    print(LIMITATION)
    print(f'[1/9] 标定 {point_name}：请在示教器选择 Local 模式，通过示教器人工移动 TCP 到标定位置并停稳。', flush=True)
    print('建议使用Local模式人工示教。当前将执行只读标定，不会控制机器人运动。')
    print('本次只读取状态和 TCP 偏置，不创建 RTDE Control，不发送运动命令；只读标定不要求 Remote。')
    status = report_control_mode(project['robot']['robot_ip'])
    if status.mode == 'unknown':
        print('模式无法确认；可继续只读标定，仍须检查停稳和 TCP 配置。')
    if not confirm('[2/9] 人工移动完成后按 Enter，开始只读标定'):
        raise KeyboardInterrupt('操作人员取消只读标定，未读取或保存新标定')
    controller = controller_factory(runtime_robot_config(project))
    try:
        print('[3/9] 正在连接 RTDE Receive（只读）…', flush=True)
        controller.connect()
        print('[4/9] 正在读取 TCP 状态与实际位姿…', flush=True)
        state = controller.wait_for_standstill()
        print('[5/9] 停稳检查通过；正在通过只读端口 30012 检查当前 TCP 配置…', flush=True)
        tcp = read_tcp_offset_readonly(project['robot']['robot_ip'])
        if not tcp_offsets_match(tcp, project['tcp']['offset'], project['tcp']['offset_tolerance']):
            raise RobotError('当前 TCP 偏置与工程配置不一致，拒绝保存。当前：' + pose_text(tcp))
        print('[5/9] TCP 配置检查通过。', flush=True)
        print(f'[6/9] 已读取 {point_name}：{pose_text(state.pose)}', flush=True)
        return state.pose.copy(), tcp
    finally:
        controller.close()  # No Control instance was activated.


def save_calibration_checked(path, updated, point_name):
    path = Path(path).resolve()
    validate_calibration(updated)  # Validate the complete candidate before touching the existing file.
    print(f'[7/9] 正在保存 {point_name} 标定：{path}', flush=True)
    original = path.read_bytes() if path.exists() else None
    backup = None
    if original is not None:
        backup = path.with_name(f'{path.stem}.backup-{time.time_ns()}{path.suffix}')
        with backup.open('xb') as stream:
            stream.write(original)
        print(f'原标定备份：{backup}', flush=True)
    try:
        write_calibration(path, updated)
        print('[8/9] 正在重新读取文件，核验完整标定及原始精度…', flush=True)
        reloaded = load_calibration(path)
        if reloaded != updated:
            raise ValueError(f'文件回读校验不一致，不能确认保存成功：{path}')
    except BaseException:
        if backup is not None:
            backup.replace(path)
            print('保存或回读失败，已恢复原始标定文件。', flush=True)
        elif path.exists():
            path.unlink()  # A failed first save must not leave an unverified file.
        raise
    if point_name == 'P_ref':
        point = updated['P_ref']
    else:
        point = updated['groups'].get(point_name.removeprefix('P0-'), {}).get('tcp_pose')
    print(f'[9/9] 标定成功：{point_name}；文件回读校验通过。', flush=True)
    if point is not None:
        print(f'XY坐标：X={point[0]*1000:.3f} mm，Y={point[1]*1000:.3f} mm')
    print(f'保存位置：{path}\n回读校验：通过', flush=True)
    return reloaded


def calibrate_target_edge(args, project, *, capture=None):
    capture = capture_pose if capture is None else capture
    path = Path(getattr(args, 'target_edge_calibration', ROOT/'target_edge_calibration.yaml')).resolve()
    if path in {Path(getattr(args, 'calibration', ROOT/'single_point_calibration.yaml')).resolve(),
                (ROOT/'single_point_calibration.yaml').resolve()}:
        raise ValueError('目标直边必须使用独立文件，禁止覆盖 P_ref/P0 标定')
    print('\n========== 标定目标直边 ==========')
    print('请将示教器切换 Local 模式，手动将 TCP 放到同一条实际直边的 A、B 两个位置。')
    print('两点尽量使用一致的探针接触方式；探针直径 3 mm，TCP 中心不一定是真实接触点，不进行半径补偿。')
    print('仅通过 RTDE Receive 读取实际坐标，检查停稳及只读 TCP 配置；不会控制运动。')
    minimum = float(getattr(args, 'min_edge_calibration_distance_m', DEFAULT_MIN_EDGE_DISTANCE_M))
    if not np.isfinite(minimum) or minimum <= 0:
        raise ValueError('目标直边最小点间距离必须为有限正数')
    print(f'A、B 最小 XY 间距：{minimum*1000:.3f} mm。')
    a, tcp_a = capture(project, point_name='目标直边 A')
    a_time = wall_time_fields()
    print(f'A点已暂存：{xyz_mm(a)}；请手动移动到同一直边的 B 点。')
    b, tcp_b = capture(project, point_name='目标直边 B')
    b_time = wall_time_fields()
    if not tcp_offsets_match(tcp_a, tcp_b, project['tcp']['offset_tolerance']):
        raise ValueError('A、B采集时TCP配置不同，原目标直边文件保持不变')
    candidate = make_edge(a, b, project['robot']['robot_ip'], tcp_a,
                          minimum=minimum, a_time=a_time, b_time=b_time)
    print(f'正在保存目标直边标定并回读校验：{path}', flush=True)
    saved, backup = save_edge(path, candidate)
    if backup is not None:
        print(f'原目标直边标定备份：{backup}')
    print('\n========== 目标直边标定成功 ==========')
    for name in ('A', 'B'):
        pose = saved[name]['tcp_pose']
        print(f'{name}点：X={pose[0]*1000:.3f} mm，Y={pose[1]*1000:.3f} mm')
    print(f'边界方向：{saved["direction_deg"]:.2f}°\n边界长度：{saved["length_xy_m"]*1000:.3f} mm')
    print(f'保存成功。\n保存位置：{path}\n回读校验：通过')
    return saved


def show_force_direction_analysis(analysis):
    print('\n========== 接触力方向分析 ==========')
    if analysis['status'] != 'available':
        print(f'未计算：{analysis["reason"]}')
        return
    for label, key in [('标定边界方向', 'boundary_direction_deg'), ('接触力方向', 'force_direction_deg'),
                       ('力与边界夹角', 'angle_to_edge_deg'), ('垂直方向偏差', 'perpendicular_error_deg')]:
        print(f'{label}：{analysis[key]:.2f}°')
    print(f'理想垂直夹角：90°\n触发时Fxy：{analysis["fxy_N"]:.3f} N')


def reference_recalibration_choice(data):
    print('\n========== 重新标定 P_ref ==========')
    print(f'当前参考点：{xyz_mm(data["P_ref"])}')
    print('已有P0：' + '、'.join(data['groups']))
    print('1. 重新标定P_ref，保留所有P0的XY\n2. 重新标定P_ref，清空所有P0\n0. 取消')
    choice = input('选择重标定方式：').strip()
    if choice == '0':
        raise KeyboardInterrupt('已取消 P_ref 重标定，原标定文件完全不变')
    if choice not in ('1', '2'):
        raise ValueError('无效重标定方式；请输入 0、1 或 2，原标定未修改')
    return 'keep' if choice == '1' else 'clear'


def show_reference_changes(before, after, settings, project, policy):
    print(f'修改前 P_ref：{xyz_mm(before["P_ref"])}')
    print(f'修改后 P_ref：{xyz_mm(after["P_ref"])}')
    print(f'统一实验高度：{after["P_ref"][2]*1000:.3f} mm；统一竖直姿态将由新参考姿态推导。')
    if policy == 'clear':
        print('即将清空的 P0：' + '、'.join(before['groups']))
        return
    minimum = settings.get('min_approach_calibration_distance_m', DEFAULT_MIN_APPROACH_DISTANCE_M)
    for name, group in after['groups'].items():
        old_issue = group_calibration_issue(before, name, minimum)
        new_issue = group_calibration_issue(after, name, minimum)
        old_angle = '无有效方向' if old_issue else f'{direction_angle(before["groups"][name]["direction_xy"]):.2f}°'
        new_angle = '无有效方向，需要重标定' if new_issue else f'{direction_angle(group["direction_xy"]):.2f}°'
        print(f'P0-{name} XY 保持不变：X={group["tcp_pose"][0]*1000:.3f} mm，'
              f'Y={group["tcp_pose"][1]*1000:.3f} mm；方向 {old_angle} → {new_angle}')
        # Validate the derived start with the same geometry/TCP guards as a new P0.
        if not new_issue:
            validate_group(after, name, settings, project)
        else:
            print(f'  {new_issue}；该组禁止自动运动。')


def calibrate_point(args, project, settings, data, name, *, reference=False):
    label = 'P_ref' if reference else f'P0-{name}'
    policy = None
    if reference and data['groups']:
        policy = reference_recalibration_choice(data)
    if not reference and data.get('P_ref') is None:
        raise ValueError('尚未保存 P_ref，请先执行菜单 1 标定参考点')
    pose, tcp = capture_pose(project, point_name=label)
    minimum = settings.get('min_approach_calibration_distance_m', DEFAULT_MIN_APPROACH_DISTANCE_M)
    if reference:
        if policy == 'keep' and not tcp_offsets_match(tcp, data['active_tcp_offset'], project['tcp']['offset_tolerance']):
            raise RobotError('TCP 配置自原参考标定后发生变化，不能保留旧 P0；原标定未修改')
        updated = set_reference(data, pose, project['robot']['robot_ip'], tcp,
                                group_policy=policy, min_distance_m=minimum)
        if policy:
            show_reference_changes(data, updated, settings, project, policy)
            if not confirm_enter('确认保存新 P_ref；' + ('清空上述所有 P0' if policy == 'clear' else '保留 P0 XY 并更新接近方向')):
                raise KeyboardInterrupt('已取消保存新 P_ref，原标定文件完全不变')
    else:
        if not tcp_offsets_match(tcp, data['active_tcp_offset'], project['tcp']['offset_tolerance']):
            raise RobotError('TCP 配置自参考标定后发生变化，拒绝保存 P0')
        updated = save_group(data, name, pose, min_distance_m=minimum)
        validate_group(updated, name, settings, project)
    reloaded = save_calibration_checked(args.calibration, updated, label)
    return reloaded


def run_trial(args, project, settings, calibration, name, *, speed=None,
              controller_factory=URRTDEController, reader_factory=make_reader, keyboard_factory=OperatorKeyboard):
    execute = args.execute
    if execute:
        print('自动运动阶段：请在示教器切换 Remote 模式；程序只查询模式，不会自动切换。')
        report_control_mode(project['robot']['robot_ip'])
    speed = validate_speed(settings['speed_default_mps'] if speed is None else speed, settings)
    effective, robot_config, pose, direction = prepare(project, settings, calibration, name, execute=execute, speed=speed)
    settings = effective['single_point_experiment']
    if execute:
        print(f'实验组：P0-{name}；当前为准备阶段，尚未启动接近实验。')
    else:
        show_group(calibration, name, settings, speed, project)
    if execute:
        if settings.get('require_watchdog', True):
            URRTDEController.verified_watchdog_contract()  # inspect installed SDK; no connection
        if not confirm_enter('确认选择该 P0；读取实际位姿并检查设备，随后展示自动准备路径'):
            return None
    edge_snapshot = snapshot_edge(getattr(args, 'target_edge_calibration', ROOT/'target_edge_calibration.yaml'), project)
    if edge_snapshot['status'] != 'available':
        print(edge_snapshot['reason'])
    controller = reader = logger = trial = None
    result = analysis = None
    try:
        if execute:
            controller = controller_factory(robot_config)
            controller.connect(allow_start_away_from_fixed_pose=True)
            clock, sleep = time.monotonic, time.sleep
        else:
            from simulation.single_point import OfflineController
            controller = OfflineController(pose, settings)
            clock, sleep = controller.clock, controller.sleep
        processor = WrenchPreprocessor.from_config(effective['preprocessing'], tool_orientation=pose[3:])
        if processor.force_transform_status['output_frame'] != 'Base':
            raise RobotError('measured Sensor to Base transform unavailable')
        if execute:
            controller.wait_for_standstill()
            if not tcp_offsets_match(read_tcp_offset_readonly(project['robot']['robot_ip']),
                                     project['tcp']['offset'], project['tcp']['offset_tolerance']):
                raise RobotError('actual TCP differs from calibrated TCP')
            reader = reader_factory(effective); reader.connect()
        else:
            from simulation.single_point import OfflineForceReader
            reader = OfflineForceReader(controller, pose, direction, processor)
        fresh = FreshForceReader(reader, settings, effective['policy'], clock=clock)
        base_logger = ExperimentLogger(args.output, effective, mode='real' if execute else 'simulation',
            strategy='single_point', config_source=args.config, extra_sample_fields=EXTRA_FIELDS, workspace_logging=False)
        logger = ContinuousLogWriter(base_logger, capacity=256, max_pending_sec=2.)
        logger.write_json('single_point_calibration_snapshot.json', calibration)
        logger.write_json('single_point_experiment_snapshot.json', settings)
        logger.write_json('target_edge_calibration_snapshot.json', edge_snapshot)
        with keyboard_factory() as keyboard:
            if execute:
                prepare_startup(effective, settings, pose, controller, fresh, processor, logger, keyboard,
                                confirm=confirm_enter)
                presets = [float(v)*1000 for v in settings['speed_presets_mps']]
                print(f'速度预设 mm/s: {presets}；允许范围 {settings["speed_min_mps"]*1000}–{settings["speed_max_mps"]*1000}')
                choice = keyboard.read_text(f'输入名义接近速度 mm/s（Enter 使用 {speed*1000:g}）：')
                if choice.strip():
                    speed = validate_speed(float(choice)/1000, settings)
                show_group(calibration, name, settings, speed, project)
            else:
                processor.set_zero_bias([fresh.read_wrench()])
            effective['single_point_selected_group'] = name
            effective['single_point_nominal_speed_mps'] = speed
            logger.write_config_snapshot(effective)
            trial = SinglePointTrial(effective, settings, calibration, name, speed, controller, fresh,
                processor, logger, poll=keyboard.poll if execute else lambda: None, clock=clock, sleep=sleep,
                preparation_complete=True)
            if execute:
                print('当前仅记录静止数据；最终 Enter 确认后才启用实验力判定并开始接近。')
            trial.record_precontact()
            if execute:
                keyboard.on_wait = trial.hold_once
                if not confirm_enter('静止数据已记录至少 1 秒；确认沿所示方向接近，'
                                     'Fxy 达阈值立即制动；安全边界触发时异常停止',
                                     read_line=keyboard.read_line):
                    raise KeyboardInterrupt('approach cancelled')
                trial.authorize_approach()  # fresh start checks only after final Enter
            result = trial.run()
    except BaseException as exc:
        # Never let sensor, keyboard or logging exceptions skip braking.
        if controller is not None and execute and controller.control is not None:
            controller.request_stop(nonblocking=True)
            if trial is not None:
                trial.reason = trial.reason or 'DEVICE_OR_OPERATOR_ERROR'
                trial.fault = f'{type(exc).__name__}: {exc}'
                try:
                    trial.settle()
                except BaseException as stop_exc:
                    trial.fault += f'; {stop_exc}'
            else:
                controller.wait_for_standstill()
        result = trial.summary() if trial is not None else dict(status='aborted', fault=f'{type(exc).__name__}: {exc}')
    finally:
        errors = []
        # Close devices only after braking observations; close owner before log drain.
        for device in (controller, reader):
            if device is not None and hasattr(device, 'close'):
                try:
                    device.close()
                except Exception as exc:
                    errors.append(f'cleanup: {exc}')
        if logger is not None:
            run_dir = logger.run_dir
            try:
                if trial is not None:
                    trial._drain_commands()
                elif controller is not None:
                    logger.log_commands(list(controller.command_records))
                    controller.command_records.clear()
                result = result or dict(status='aborted', fault='no result')
                if errors:
                    result.update(status='aborted', cleanup_errors=errors)
                logger.write_json('single_point_result.json', result)
                analysis = analyze_contact(edge_snapshot, result)
                logger.write_json('contact_force_direction_analysis.json', analysis)
                record_trial_termination(logger, result)
                logger.write_summary(result.get('state', 'STOP'), result.get('reason', result.get('fault', '')), 0,
                                     single_point_result=result)
            finally:
                logger.close()
            if (run_dir/'samples.csv').exists():
                # An aborted preparation can contain only safe-return samples;
                # the shared scan replay intentionally excludes those samples.
                with (run_dir/'samples.csv').open(encoding='utf-8', newline='') as source:
                    has_trial_samples = any(row['current_state'] != 'RETURN_TO_START' for row in csv.DictReader(source))
                if has_trial_samples:
                    from visualization.run_html import write_html_replay
                    from visualization.run_plots import load_trace, write_launcher
                    write_html_replay(load_trace(run_dir), run_dir)
                    write_launcher(run_dir)
            print(f'运行目录：{run_dir}')
        if errors and logger is None:
            raise RobotError('; '.join(errors))
    show_trial_result(result)
    show_force_direction_analysis(analysis if analysis is not None else analyze_contact(edge_snapshot, result))
    return result


def return_to_group(args, project, settings, data, name):
    print('当前为自动返回 P0，未启动接近实验；接触阈值不参与返回判定。')
    print('自动返回阶段：请在示教器切换 Remote 模式；程序只查询模式，不会自动切换。')
    report_control_mode(project['robot']['robot_ip'])
    effective, robot_config, pose, _ = prepare(project, settings, data, name, execute=True, approach=False)
    if settings['return_lift_distance_m'] is None:
        raise RobotError('缺少返回路径验证；自动返回被拒绝，请手动移动到安全位置')
    if not confirm_enter('连接设备读取当前位姿并展示返回路径；此确认不授权运动'):
        return
    controller, reader, logger = URRTDEController(robot_config), make_reader(effective), None
    monitor = None
    try:
        controller.connect()
        current = controller.wait_for_standstill().pose.copy()
        return_config, segments = plan_return(effective, settings, current, pose)
        show_motion_path(current, pose, segments)
        print('以上路径检查只覆盖探针半径和已填障碍盒；必须现场检查整套探针、工具、目标和夹具的扫掠空间。')
        print('准备及返回 P0 不使用实验或返回的低力/力矩阈值；保留原有硬限值 '
              f'{effective["policy"]["absolute_raw_force_threshold"]:.3f} N / '
              f'{effective["policy"]["absolute_raw_torque_threshold"]:.3f} N·m；'
              '不会在接触位置采零。')
        with OperatorKeyboard() as keyboard:
            keyboard.on_wait = lambda: controller.read_diagnostic_state()
            if not confirm_enter('确认上述完整路径及抬升高度安全；自动返回所选 P0', read_line=keyboard.read_line):
                return
            observed = controller.read_diagnostic_state()
            if (np.linalg.norm(observed.pose[:3]-current[:3]) > settings['start_position_tolerance_m'] or
                _orientation_distance(observed.pose[3:], current[3:]) > settings['orientation_tolerance_rad']):
                raise RobotError('机器人位姿已变动，必须重新展示并核验路径')
            reader.connect()
            processor = WrenchPreprocessor.from_config(return_config['preprocessing'], tool_orientation=current[3:])
            logger = ExperimentLogger(args.output, return_config, mode='real', strategy='single_point',
                                      config_source=args.config, workspace_logging=False)
            fresh = FreshForceReader(reader, settings, return_config['policy'])
            monitor = SinglePointRetreatMonitor(return_config, fresh, processor, controller,
                                                current, segments[0][1])
            try:
                monitor.sample(read_state=controller.read_diagnostic_state)
            finally:
                (logger.run_dir/'return_force_preflight.json').write_text(
                    json.dumps(monitor.snapshot(), ensure_ascii=False, indent=2), encoding='utf-8')
            print(f'返回前实测原始力：{np.linalg.norm(monitor.raw.force):.3f} N；'
                  f'力矩：{np.linalg.norm(monitor.raw.torque):.3f} N·m。')
            if monitor.retreat_guard_wrench_base is not None:
                print('抬升保护使用含力臂的 Base 力矩：'
                      f'{np.linalg.norm(monitor.retreat_guard_wrench_base.torque):.3f} N·m；'
                      f'原有硬限值 {return_config["policy"]["absolute_raw_torque_threshold"]:.3f} N·m。')
            (logger.run_dir/'return_plan.json').write_text(json.dumps(dict(p0=name, initial=current.tolist(),
                segments=[dict(phase=k, pose=p.tolist(), speed=v) for k,p,v in segments],
                path_confirmed=True), ensure_ascii=False, indent=2), encoding='utf-8')
            require_remote_mode(project['robot']['robot_ip'])
            controller.activate_control(confirmed=True)
            controller.enable_watchdog(settings['watchdog_frequency_hz']); controller.kick_watchdog()
            result = SafeReturnExecutor(return_config, pose, controller, force_monitor=monitor,
                                        logger=logger, poll=keyboard.poll, target_label=name).execute()
            print(f'返回结果：{result.status}；原因：{result.abort_reason or "已完成"}')
            print(f'实际返回坐标：{xyz_mm(result.final_pose)}')
    finally:
        try:
            controller.close()
        finally:
            try:
                reader.close()
            finally:
                if logger is not None:
                    try:
                        if monitor is not None:
                            (logger.run_dir/'return_force_status.json').write_text(
                                json.dumps(monitor.snapshot(), ensure_ascii=False, indent=2), encoding='utf-8')
                        logger.log_commands(list(controller.command_records))
                    finally:
                        logger.close()


def select_group(data):
    names = list(data['groups'])
    if not names:
        raise ValueError('先标定 P_ref 并新增 P0')
    print('========== 选择接近方向 ==========')
    for i, name in enumerate(names, 1): print(f'{i}. P0-{name}')
    index = int(input('选择序号（只选择，不运动）：'))
    if not 1 <= index <= len(names): raise ValueError('无效序号')
    return names[index-1]


def menu(args, project, settings):
    while True:
        print('\n========== 单点接触实验 ==========\n1. 标定固定接触参考点\n2. 新增P0\n3. 查看所有P0\n'
              '4. 修改或删除P0\n5. 开始单点接触实验\n6. 返回指定P0\n7. 查询Local/Remote模式\n8. 标定目标直边\n0. 返回')
        try:
            choice = input('选择：').strip()
        except (KeyboardInterrupt, EOFError):
            print('操作已取消，已退出菜单；未修改标定文件。')
            return 0
        if choice == '0': return 0
        label = '目标直边' if choice == '8' else 'P_ref' if choice == '1' else 'P0'
        try:
            if choice == '7':
                report_control_mode(project['robot']['robot_ip']); continue
            if choice == '8':
                if not args.execute:
                    print('离线菜单不连接设备；只读目标直边标定必须显式使用 --execute。')
                else:
                    calibrate_target_edge(args, project)
                continue
            data = load_calibration(args.calibration)
            if choice == '3':
                show_calibration_table(data, settings); continue
            if choice in ('1', '2', '4', '6') and not args.execute:
                print('离线菜单不连接设备；标定和返回必须显式使用 --execute。'); continue
            if choice == '1':
                calibrate_point(args, project, settings, data, 'P_ref', reference=True)
            elif choice in ('2', '4'):
                name = input('P0 名称（如 A/B/C）：').strip() if choice == '2' else select_group(data)
                label = f'P0-{name}'
                if choice == '2' and name in data['groups']: raise ValueError('P0 已存在，请用修改菜单')
                if choice == '4' and input('输入 delete 删除，Enter 重新采集：').strip() == 'delete':
                    if confirm_enter(f'删除 P0-{name}'):
                        del data['groups'][name]; save_calibration_checked(args.calibration, data, f'删除 P0-{name}')
                    else:
                        print(f'已取消删除 P0-{name}，原标定保持不变。')
                    continue
                calibrate_point(args, project, settings, data, name)
            elif choice == '5':
                name = select_group(data)
                run_trial(args, project, settings, data, name, speed=args.speed)
            elif choice == '6':
                return_to_group(args, project, settings, data, select_group(data))
            else:
                print('无效菜单选项，请输入 0–8。')
        except (KeyboardInterrupt, EOFError) as exc:
            print(f'标定取消：{label}；原因：{exc}' if choice in ('1', '2', '4', '8') else f'操作已取消；原因：{exc}')
        except Exception as exc:
            prefix = f'标定失败：{label}' if choice in ('1', '2', '4', '8') else '操作失败'
            path = getattr(args, 'target_edge_calibration', ROOT/'target_edge_calibration.yaml') if choice == '8' else args.calibration
            print(f'{prefix}；具体原因：{exc}\n标定文件：{Path(path).resolve()}')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true', help='explicit hardware mode; further Enter confirmations required')
    mode.add_argument('--simulate', action='store_true', help='offline generated A/B/C calibration when no --group given')
    parser.add_argument('--config', type=Path, default=ROOT/'config.yaml')
    parser.add_argument('--settings', type=Path, default=ROOT/'single_point_experiment.yaml')
    parser.add_argument('--calibration', type=Path, default=ROOT/'single_point_calibration.yaml')
    parser.add_argument('--target-edge-calibration', type=Path, default=ROOT/'target_edge_calibration.yaml',
                        help='independent optional target edge calibration (SI)')
    parser.add_argument('--min-edge-calibration-distance-m', type=float, default=DEFAULT_MIN_EDGE_DISTANCE_M,
                        help='minimum XY spacing for taught edge points, default 0.005 m')
    parser.add_argument('--group', help='saved P0 name, e.g. A')
    parser.add_argument('--speed', type=float, help='nominal speed in m/s; hardware prompt allows adjustment')
    parser.add_argument('--show-config', action='store_true', help='print resolved original and experiment parameters offline')
    parser.add_argument('--show-calibration-json', action='store_true', help='offline debug: print full SI calibration data')
    parser.add_argument('--output', type=Path, default=ROOT/'data')
    args = parser.parse_args(argv)
    if args.show_calibration_json:
        print(json.dumps(load_calibration(args.calibration), ensure_ascii=False, indent=2))
        return 0
    project = load_config(args.config)
    settings = load_settings(args.settings, project)
    if args.show_config:
        print(json.dumps(dict(config_source=str(args.config.resolve()),
            robot=project['robot'], sensor=project['sensor'], tcp=project['tcp'],
            workspace=project['workspace'], safe_return=project['safe_return'],
            single_point_experiment=settings), ensure_ascii=False, indent=2))
        return 0
    print(LIMITATION)
    try:
        if args.simulate and not args.group:
            from simulation.single_point import demo_calibration
            data = demo_calibration(project, settings=settings)
            result = run_trial(args, project, settings, data, 'A', speed=args.speed)
        elif args.group:
            data = load_calibration(args.calibration)
            result = run_trial(args, project, settings, data, args.group, speed=args.speed)
        else:
            return menu(args, project, settings)
    except (KeyboardInterrupt, EOFError) as exc:
        print(f'操作已取消；原因：{exc}')
        return 1
    except Exception as exc:
        print(f'操作失败；具体原因：{exc}\n标定文件：{args.calibration.resolve()}')
        return 1
    return 0 if result is None or result['status'] == 'contact' else 1


if __name__ == '__main__':
    raise SystemExit(main())
