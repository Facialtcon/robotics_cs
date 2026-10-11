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
from app.single_point_startup import plan_return, prepare_startup
from app.single_point_ui import (xyz_mm, pose_text, direction_angle, show_calibration_table,
                                show_motion_path, report_control_mode, require_remote_mode, show_trial_result)
from calibration.single_point import (LIMITATION, load_calibration, save_group, set_reference,
                                      validate_group, write_calibration, experiment_start_pose)
from config.loader import load_config, runtime_robot_config
from experiment_logging.continuous_writer import ContinuousLogWriter
from experiment_logging.data_logger import ExperimentLogger
from robot.rtde_controller import URRTDEController, RobotError
from robot.rtde_controller import _orientation_distance
from robot.tcp_identity import read_tcp_offset_readonly, tcp_offsets_match
from safety.safe_return import SafeReturnExecutor, PX6DForceMonitor
from sensor.force_preprocess import WrenchPreprocessor
from sensor.px6d_reader import PX6DReader

ROOT = Path(__file__).resolve().parent


def make_reader(project):
    s = project['sensor']
    return PX6DReader(s['serial_port'], s['baudrate'], s['timeout_sec'], s['poll_rate_hz'], s['startup_delay_sec'])


def show_group(data, name, settings, speed, project):
    group = data['groups'][name]
    print(LIMITATION)
    target = experiment_start_pose(data, name, project['calibration']['probe_axis_tcp'])
    print(f'选择实验组：P0-{name}；标定 XY：X={group["tcp_pose"][0]*1000:.3f} mm，Y={group["tcp_pose"][1]*1000:.3f} mm')
    print(f'固定参考点 P_ref：{xyz_mm(data["P_ref"])}')
    print(f'本次实验起点：{pose_text(target)}')
    print(f'接近方向：{direction_angle(group["direction_xy"]):.2f}°（相对 Base +X，逆时针为正）')
    print(f'名义速度：{speed*1000:.3f} mm/s；接触力阈值：{settings["contact_threshold_N"]:.3f} N；'
          f'最大搜索距离：{settings["max_search_distance_m"]*1000:.3f} mm')
    print('正常停止条件：Fxy 首次达到力阈值；P_ref 的 XY 不是运动终点。')


def capture_pose(project, *, point_name='标定点', confirm=confirm_enter, controller_factory=URRTDEController):
    print(LIMITATION)
    print(f'[1/9] 标定 {point_name}：请在示教器选择 Local 模式，通过示教器人工移动 TCP 到标定位置并停稳。', flush=True)
    print('本次只读取状态和 TCP 偏置，不创建 RTDE Control，不发送运动命令；只读标定不要求 Remote。')
    if not confirm('[2/9] 人工移动完成后按 Enter，开始只读标定'):
        raise KeyboardInterrupt('操作人员取消只读标定，未读取或保存新标定')
    controller = controller_factory(runtime_robot_config(project))
    try:
        print('[3/9] 正在连接 RTDE Receive（只读）…', flush=True)
        controller.connect()
        report_control_mode(project['robot']['robot_ip'])  # Unknown mode does not block read-only calibration.
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
    print(f'[7/9] 正在保存 {point_name} 标定：{path}', flush=True)
    write_calibration(path, updated)
    print('[8/9] 正在重新读取文件，核验完整标定及原始精度…', flush=True)
    reloaded = load_calibration(path)
    if reloaded != updated:
        raise ValueError(f'文件回读校验不一致，不能确认保存成功：{path}')
    print(f'[9/9] 标定成功：{point_name}；文件回读校验通过。\n保存文件：{path}', flush=True)
    return reloaded


def calibrate_point(args, project, settings, data, name, *, reference=False):
    label = 'P_ref' if reference else f'P0-{name}'
    if reference and data['groups']:
        raise ValueError('P_ref 已被现有 P0 使用，拒绝覆盖固定参考点；现有标定保持不变')
    if not reference and data.get('P_ref') is None:
        raise ValueError('尚未保存 P_ref，请先执行菜单 1 标定参考点')
    pose, tcp = capture_pose(project, point_name=label)
    if reference:
        updated = set_reference(data, pose, project['robot']['robot_ip'], tcp)
    else:
        if not tcp_offsets_match(tcp, data['active_tcp_offset'], project['tcp']['offset_tolerance']):
            raise RobotError('TCP 配置自参考标定后发生变化，拒绝保存 P0')
        updated = save_group(data, name, pose)
        validate_group(updated, name, settings, project, require_review=False)
        show_group(updated, name, settings, settings['speed_default_mps'], project)
        print('原始 P0 仅取 XY；请在 P_ref.z 和统一竖直姿态下核验直到最大搜索距离的完整接近路径。')
        note = input('路径核验依据（空值仅保存坐标，路径保持未核验）：').strip()
        if note and not confirm_enter('确认已在现场核验完整接近路径'):
            raise KeyboardInterrupt('路径核验已取消，未保存本次新标定')
        updated = save_group(data, name, pose, path_note=note,
                             reviewed_distance_m=settings['max_search_distance_m'] if note else None)
    reloaded = save_calibration_checked(args.calibration, updated, label)
    if not reference:
        print('路径核验：' + ('已完成' if note else '未完成；坐标已保存，暂不能进行该组真机实验'))
    return reloaded


def run_trial(args, project, settings, calibration, name, *, speed=None,
              controller_factory=URRTDEController, reader_factory=make_reader, keyboard_factory=OperatorKeyboard):
    execute = args.execute
    speed = validate_speed(settings['speed_default_mps'] if speed is None else speed, settings)
    effective, robot_config, pose, direction = prepare(project, settings, calibration, name, execute=execute)
    show_group(calibration, name, settings, speed, project)
    if execute:
        print('自动运动阶段：请在示教器切换 Remote 模式；程序只查询模式，不会自动切换。')
        if settings.get('require_watchdog', True):
            URRTDEController.verified_watchdog_contract()  # inspect installed SDK; no connection
        if not confirm_enter('确认选择该 P0；读取实际位姿并检查设备，随后展示自动准备路径'):
            return None
    controller = reader = logger = trial = None
    result = None
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
            trial.record_precontact()
            if execute:
                keyboard.on_wait = trial.hold_once
                if not confirm_enter('静止数据已记录至少 1 秒；确认沿所示方向接近，Fxy 达阈值立即制动', read_line=keyboard.read_line):
                    raise KeyboardInterrupt('approach cancelled')
                trial.hold_once()  # fresh stationary/force check after final Enter
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
    return result


def return_to_group(args, project, settings, data, name):
    effective, robot_config, pose, _ = prepare(project, settings, data, name, execute=True)
    if settings['return_lift_distance_m'] is None:
        raise RobotError('缺少返回路径验证；自动返回被拒绝，请手动移动到安全位置')
    print('自动返回阶段：请在示教器切换 Remote 模式；程序只查询模式，不会自动切换。')
    if not confirm_enter('连接设备读取当前位姿并展示返回路径；此确认不授权运动'):
        return
    controller, reader, logger = URRTDEController(robot_config), make_reader(effective), None
    try:
        controller.connect()
        current = controller.wait_for_standstill().pose.copy()
        return_config, segments = plan_return(effective, settings, current, pose)
        show_motion_path(current, pose, segments)
        print('以上路径检查只覆盖探针半径和已填障碍盒；必须现场检查整套探针、工具、目标和夹具的扫掠空间。')
        with OperatorKeyboard() as keyboard:
            keyboard.on_wait = lambda: controller.read_diagnostic_state()
            note = keyboard.read_text('填写这条完整返回路径的现场核验依据（空值拒绝自动返回）：').strip()
            if not note:
                raise RobotError('无法确认完整返回路径安全；拒绝自动返回，请手动移动')
            if not confirm_enter('确认上述完整路径、核验依据及抬升高度安全；自动返回所选 P0', read_line=keyboard.read_line):
                return
            observed = controller.read_diagnostic_state()
            if (np.linalg.norm(observed.pose[:3]-current[:3]) > settings['start_position_tolerance_m'] or
                _orientation_distance(observed.pose[3:], current[3:]) > settings['orientation_tolerance_rad']):
                raise RobotError('机器人位姿已变动，必须重新展示并核验路径')
            reader.connect()
            processor = WrenchPreprocessor.from_config(return_config['preprocessing'], tool_orientation=current[3:])
            monitor = PX6DForceMonitor(return_config, reader, processor, raw_only=True)
            monitor.sample(read_state=controller.read_diagnostic_state)
            logger = ExperimentLogger(args.output, return_config, mode='real', strategy='single_point',
                                      config_source=args.config, workspace_logging=False)
            (logger.run_dir/'return_plan.json').write_text(json.dumps(dict(p0=name, initial=current.tolist(),
                segments=[dict(phase=k, pose=p.tolist(), speed=v) for k,p,v in segments],
                site_review=note), ensure_ascii=False, indent=2), encoding='utf-8')
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
            reader.close()
            if logger is not None:
                logger.log_commands(list(controller.command_records)); logger.close()


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
              '4. 修改或删除P0\n5. 开始单点接触实验\n6. 返回指定P0\n0. 返回')
        choice = input('选择：').strip()
        if choice == '0': return 0
        label = 'P_ref' if choice == '1' else 'P0'
        try:
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
                print('无效菜单选项，请输入 0–6。')
        except (KeyboardInterrupt, EOFError) as exc:
            print(f'标定取消：{label}；原因：{exc}' if choice in ('1', '2', '4') else f'操作已取消；原因：{exc}')
        except Exception as exc:
            prefix = f'标定失败：{label}' if choice in ('1', '2', '4') else '操作失败'
            print(f'{prefix}；具体原因：{type(exc).__name__}: {exc}\n标定文件：{Path(args.calibration).resolve()}')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true', help='explicit hardware mode; further Enter confirmations required')
    mode.add_argument('--simulate', action='store_true', help='offline generated A/B/C calibration when no --group given')
    parser.add_argument('--config', type=Path, default=ROOT/'config.yaml')
    parser.add_argument('--settings', type=Path, default=ROOT/'single_point_experiment.yaml')
    parser.add_argument('--calibration', type=Path, default=ROOT/'single_point_calibration.yaml')
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
    if args.simulate and not args.group:
        from simulation.single_point import demo_calibration
        data = demo_calibration(project)
        result = run_trial(args, project, settings, data, 'A', speed=args.speed)
    elif args.group:
        data = load_calibration(args.calibration)
        result = run_trial(args, project, settings, data, args.group, speed=args.speed)
    else:
        return menu(args, project, settings)
    return 0 if result is None or result['status'] in ('contact', 'no_contact') else 1


if __name__ == '__main__':
    raise SystemExit(main())
