#!/usr/bin/env python3
"""Independent automatic gravity calibration. Default: offline plan only."""

from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np

from app.operator_input import OperatorKeyboard, confirm_enter
from config.loader import load_config
from tools.sensor_mount_calibration.plan import Limits, make_plan, describe_plan
from tools.sensor_mount_calibration.session import Session, CalibrationStopped
from tools.sensor_mount_calibration.solver import calibrate, CalibrationError
from tools.sensor_mount_calibration.storage import RunStore, preview_config_update, commit_config_update
from tools.sensor_mount_calibration.progress import ProgressReporter
from tools.sensor_mount_calibration.stability import check_recorded_stability
from tools.sensor_mount_calibration.diagnostics import analyze_failure_records


def report(result):
    print(f"拟合姿态 RMSE: {result['fit_rmse_N']:.5f} N；独立姿态 RMSE: {result['validation_rmse_N']:.5f} N")
    print('rotation_sensor_to_tool（列为 Sensor 轴在当前 TCP 中的方向）：')
    print(np.array2string(np.asarray(result['rotation_sensor_to_tool']), precision=10))
    print(f"det={np.linalg.det(result['rotation_sensor_to_tool']):.10f}；力作用方符号={result['force_sign']:+d}")
    print(f"诊断重量={result['weight_N']:.5f} N；固定零偏={result['bias_sensor_N']} N")
    print('符号独立于安装旋转；零偏、重量和符号仅存诊断，不写入扫描补偿或方向控制。')


def report_failure(exc):
    """Explain numerical rejection only after robot shutdown, without installing candidates."""
    diagnostics = getattr(exc, 'diagnostics', {})
    context = diagnostics.get('motion_context') or {}
    if context.get('is_return'):
        print(f'失败路段：姿态编号 {context.get("after_pose_id")} 后回中，'
              f'倾角 {context.get("tilt_deg")}°、方向 {context.get("azimuth_deg")}°；'
              f'已采 {context.get("completed_poses")}/{context.get("total_poses")} 个姿态。', flush=True)
    if 'force_delta_sensor_N' in diagnostics:
        print('同姿态原始力变化 [Fx,Fy,Fz]：'
              f'{np.array2string(np.asarray(diagnostics["force_delta_sensor_N"]), precision=5)} N；'
              f'位置差 {diagnostics["position_delta_m"]*1000:.5f} mm，'
              f'姿态差 {diagnostics["orientation_delta_deg"]:.5f}°。', flush=True)
    analysis = diagnostics.get('failure_analysis') or {}
    if analysis.get('summary'):
        print('原始记录诊断：' + analysis['summary'], flush=True)
    for candidate in analysis.get('change_candidates', [])[:1]:
        utc = candidate.get('utc_time')
        try:
            stamp = datetime.fromisoformat(utc.replace('Z', '+00:00')).astimezone().isoformat(timespec='milliseconds')
        except (AttributeError, TypeError, ValueError):
            stamp = str(utc or candidate.get('host_monotonic'))
        force_change = np.asarray(candidate['force_delta_sensor_N'])
        torque_change = np.asarray(candidate['torque_delta_sensor_Nm'])
        print(f'突变候选时刻：{stamp}；局部趋势估计 ΔF='
              f'{np.array2string(force_change, precision=5)} N，ΔM='
              f'{np.array2string(torque_change, precision=6)} Nm。', flush=True)
        print('原始均值、实际姿态、关节变化和采样分辨率已存入 metadata.json 的 failure_analysis。', flush=True)
    candidates = diagnostics.get('sign_candidates', [])
    for candidate in candidates:
        held = candidate.get('validation_rmse_N')
        validation = f'，独立验证 RMSE {held:.5f} N' if held is not None else ''
        print(f'诊断候选：符号 {candidate["force_sign"]:+d}，拟合 RMSE '
              f'{candidate["fit_rmse_N"]:.5f} N{validation}（未保存安装结果）。', flush=True)
    if analysis.get('classification') == 'abrupt_force_change':
        print('记录中存在运动附近的力/矩突变；不能仅归因于预热不足，也不能靠重新采零掩盖。'
              '请检查该路段线缆是否卡住/拉拽/擦碰、工具或传感器连接是否松动、是否发生接触。'
              '这些是排查方向，日志不能单独确定具体物理原因；排除后重新采集。', flush=True)
    elif diagnostics.get('reason_code') in ('stationary_force_drift', 'reference_force_drift') or '漂移' in str(exc):
        print('同一实际姿态的原始力不重复，当前不满足固定零偏模型。'
              '请确认工具无接触、安装连接牢固、线缆无牵拉；仅凭此项检查不能认定是预热不足。'
              '程序不会自动采零或扣除漂移。', flush=True)


def attach_failure_analysis(exc, records):
    """Best-effort explanation after shutdown; never replace the primary error."""
    try:
        analysis = analyze_failure_records(records, getattr(exc, 'diagnostics', {}))
        exc.diagnostics = {**getattr(exc, 'diagnostics', {}), 'failure_analysis': analysis}
    except Exception as analysis_exc:
        exc.add_note(f'raw failure analysis unavailable: {type(analysis_exc).__name__}: {analysis_exc}')


def _metadata(config_path, limits, mode):
    return dict(mode=mode, config_path=str(config_path),
                config_sha256=hashlib.sha256(Path(config_path).read_bytes()).hexdigest(),
                motion_limits=limits.as_dict(), gravity_base_unit=[0, 0, -1],
                transform_convention='R_base_sensor = R_base_tool @ rotation_sensor_to_tool',
                status='started')


def offline(raw_path, config_path, data_root, limits):
    records = [json.loads(line) for line in Path(raw_path).read_text().splitlines() if line.strip()]
    selected = [r for r in records if r.get('split') in ('fit', 'validation')]
    store = RunStore.create(data_root, {**_metadata(config_path, limits, 'offline_replay'),
                                       'source_raw': str(Path(raw_path).resolve())})
    for record in records:
        store.append_raw(record)
    try:
        print(f'离线解算：读取 {len(records)} 帧，拟合/验证采样 {len(selected)} 帧；检查原地趋势与回中重复性。', flush=True)
        stability = check_recorded_stability(records)
        store.write_metadata({'recorded_stability': stability})
        print('开始固定零偏重力拟合及完整姿态验证。', flush=True)
        result = calibrate(selected)
    except CalibrationError as exc:
        attach_failure_analysis(exc, records)
        store.write_metadata(dict(status='rejected', reason=str(exc), diagnostics=exc.diagnostics))
        print(f'标定拒绝，不生成安装结果：{exc}\n原始数据/诊断：{store.path}', file=sys.stderr)
        report_failure(exc)
        return 2
    result['configuration_saved'] = False
    store.write_result(result)
    store.write_metadata(dict(status='validated_offline'))
    report(result)
    print(f'离线结果：{store.path}；配置未写入。')
    return 0


def execute(config, config_path, data_root, limits):
    # Hardware imports and construction occur exclusively under --execute.
    from tools.sensor_mount_calibration.hardware import Hardware

    with OperatorKeyboard() as keyboard:
        if not keyboard.enabled:
            raise ValueError('--execute 需要前台交互终端，禁止管道预先输入 Enter')
        store = RunStore.create(data_root, _metadata(config_path, limits, 'execute'))
        hardware = Hardware(config, limits)
        session = Session(hardware, store, keyboard, limits)
        config_saved = False
        failure = None
        logging_finish_attempted = False
        progress = None
        print(f'独立标定数据目录：{store.path}', flush=True)
        try:
            # Raw records AND stage metadata go to a bounded background writer.
            # No open/write/flush/fsync is allowed on the live observation path.
            store.start_async()
            print('正在连接 PX6D 与 RTDE 读取接口，等待传感器初始化；此时不运动。', flush=True)
            hardware.connect()
            print('设备已连接，读取实际 TCP 并生成有限倾斜计划。', flush=True)
            initial = session.observe()
            start = initial['actual_tcp_pose']
            session.origin = np.asarray(start, dtype=float)
            plan = make_plan(start)
            store.write_metadata(dict(start_pose=start, plan=plan, hardware=hardware.metadata))
            print(describe_plan(start, plan, limits, config['tcp']['offset']), flush=True)
            print(f"原始力/矩硬限：{config['policy']['absolute_raw_force_threshold']:g} N / "
                  f"{config['policy']['absolute_raw_torque_threshold']:g} Nm；保留机器人原有安全限制。")
            # read_line owns input while waiting; its TCIFLUSH discards old Enter.
            keyboard.on_wait = lambda: session.observe(poll_keyboard=False)
            if not confirm_enter('确认基座水平、上述整机/线缆空间和完整返回路径，开始自动标定。',
                                 read_line=keyboard.read_line):
                raise CalibrationStopped('operator cancelled before motion')
            keyboard.on_wait = None
            store.write_metadata(dict(base_level_confirmed=True))
            progress = ProgressReporter()
            session.progress = progress
            records = session.run(start, plan)
            progress.emit('采集完成，正在停止并关闭机器人控制。', force=True)
            hardware.stop()
            hardware.close()
            # Wait for durable data only after robot control has been closed.
            logging_finish_attempted = True
            progress.emit('正在确认全部原始数据落盘，最长等待 5s。', force=True)
            store.finish_logging(timeout_sec=5.)
            store.write_metadata(dict(logging=store.logging_diagnostics,
                                      hardware=hardware.metadata,
                                      last_observation_timing=getattr(session, 'last_observation_timing', {})))
            progress.close(timeout_sec=.5)
            print(f'数据已落盘：{len(records)} 帧拟合/验证采样。正在联合求安装旋转、固定零偏、重量及作用方符号，并验证留出姿态。', flush=True)
            result = calibrate(records)
            print('拟合及独立姿态验证通过，准备配置差异。', flush=True)
            result.update(configuration_saved=False, actual_start_pose=start,
                          active_tcp_offset=hardware.metadata.get('active_tcp_offset'),
                          base_level_confirmed=True)
            update = preview_config_update(config_path, result['rotation_sensor_to_tool'])
            # A change during acquisition cannot silently rebind the active TCP.
            expected = json.loads((store.path / 'metadata.json').read_text())['config_sha256']
            if update.original_sha256 != expected:
                raise ValueError('配置在采集期间发生修改；保留数据并拒绝写入，请离线核对')
            result['proposed_config_changes'] = update.changes
            store.write_result(result)
            report(result)
            print('拟修改配置：\n' + (update.diff or '安装旋转已经一致，无需修改。'))
            for note in update.notes:
                print(note)
            if update.changes and confirm_enter('备份并保存以上安装参数。', read_line=keyboard.read_line):
                print('正在备份原配置并保存安装旋转。', flush=True)
                backup = commit_config_update(update, store.path / 'config_backup')
                config_saved = True
                result.update(configuration_saved=True, configuration_backup=str(backup))
                store.write_result(result)
                print(f'配置已更新；原配置备份：{backup}')
            store.write_metadata(dict(status='completed', configuration_saved=result['configuration_saved']))
            print(f'原始数据、验证误差及计算结果：{store.path}')
            return 0
        except BaseException as exc:
            failure = exc
            keyboard.on_wait = None
            # Stop precedes disk diagnostics. Never attempt a return after faults.
            cleanup_errors = []
            devices_closed = False
            try:
                hardware.stop()
            except BaseException as stop_exc:
                cleanup_errors.append(f'stop request: {stop_exc}')
            try:
                hardware.close()
                devices_closed = True
            except BaseException as close_exc:
                cleanup_errors.append(f'device cleanup: {close_exc}')
            if progress is not None:
                progress.emit(f'标定未完成：{exc}；正在保存诊断。', force=True)
                progress.close(timeout_sec=.5)
            if not logging_finish_attempted:
                logging_finish_attempted = True
                try:
                    store.finish_logging(timeout_sec=5.)
                except BaseException as drain_exc:
                    cleanup_errors.append(f'raw logging finalization: {drain_exc}')
            if isinstance(exc, CalibrationError) and devices_closed and store.logging_diagnostics.get('durable'):
                print('设备已关闭；正在分析失败前的原始力/矩和实际姿态，定位持续漂移或突变。', flush=True)
                try:
                    raw_records = [json.loads(line) for line in (store.path/'raw.jsonl').read_text().splitlines() if line.strip()]
                    attach_failure_analysis(exc, raw_records)
                except Exception as analysis_exc:
                    exc.add_note(f'raw failure analysis unavailable: {type(analysis_exc).__name__}: {analysis_exc}')
            try:
                store.write_metadata(dict(status='rejected_or_stopped', reason=f'{type(exc).__name__}: {exc}',
                                          diagnostics=getattr(exc, 'diagnostics', {}),
                                          configuration_saved=config_saved, cleanup_errors=cleanup_errors,
                                          logging=store.logging_diagnostics, hardware=hardware.metadata,
                                          last_observation_timing=getattr(session, 'last_observation_timing', {})))
            except BaseException as log_exc:
                cleanup_errors.append(f'failure logging: {log_exc}')
            saved_status = '安装配置已写入，后续步骤失败' if config_saved else '未更新安装配置'
            print(f'已尝试请求停止，不自动返回；{saved_status}。诊断：{store.path}', file=sys.stderr)
            for error in cleanup_errors:
                exc.add_note(error)
                print(error, file=sys.stderr)
            report_failure(exc)
            raise
        finally:
            keyboard.on_wait = None
            try:
                hardware.close()
            except BaseException as close_exc:
                if failure is None:
                    raise
                failure.add_note(f'device cleanup also failed: {close_exc}')
                print(f'设备关闭异常（原始故障保留）：{close_exc}', file=sys.stderr)
            finally:
                if progress is not None:
                    progress.close(timeout_sec=.5)


def main(argv=None):
    parser = argparse.ArgumentParser(description='PX6D 自动多姿态重力安装旋转标定；默认离线、无连接、无运动')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true', help='连接设备，Enter 后执行已展示的有限计划')
    mode.add_argument('--offline', type=Path, metavar='RAW_JSONL', help='只重算已有原始数据；不连接设备或写配置')
    parser.add_argument('--config', type=Path, default=ROOT / 'config.yaml')
    parser.add_argument('--data-dir', type=Path, default=ROOT / 'calibration_data' / 'sensor_mount')
    args = parser.parse_args(argv)
    limits = Limits()
    try:
        config = load_config(args.config)
        if args.offline:
            return offline(args.offline, args.config, args.data_dir, limits)
        if args.execute:
            return execute(config, args.config, args.data_dir, limits)
        start = config['dry_run']['start_pose']
        print('默认离线预览：以下是示例 TCP，非实际姿态；未连接设备，不会运动或修改配置。')
        print(describe_plan(start, make_plan(start), limits, config['tcp']['offset']))
        print('真机入口：在同一命令后加 --execute；实际计划将根据读取的 TCP 重新生成。')
        return 0
    except (KeyboardInterrupt, EOFError):
        print('操作已取消。', file=sys.stderr)
        return 130
    except Exception as exc:
        print(f'标定未完成：{type(exc).__name__}: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
