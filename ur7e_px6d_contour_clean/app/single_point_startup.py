"""Confirmed single-point preparation using the existing safe return executor."""
from copy import deepcopy
import time

import numpy as np

from app.operator_input import confirm_enter
from app.scan_startup import capture_stationary_bias, hold_startup_confirmation
from app.single_point_config import check_segment
from calibration.probe_alignment import probe_tilt_deg
from robot.rtde_controller import RobotError, _orientation_distance
from safety.safe_return import SafeReturnExecutor, PX6DForceMonitor, return_trajectory, print_return_plan


def plan_return(project, settings, current, target):
    if settings['return_lift_distance_m'] is None:
        raise RobotError('自动准备/返回未核验：请配置现场确认的抬升距离，不能假设 30 mm 安全；请手动移到安全位置')
    effective = deepcopy(project)
    effective['workspace'] = dict(enabled=settings.get('workspace_enabled', True),
                                 limits=deepcopy(settings['workspace_limits'] or project['workspace']['limits']))
    effective['safe_return'].update(return_lift_distance=settings['return_lift_distance_m'],
        return_speed=settings['return_speed_mps'],
        return_vertical_speed=settings.get('return_vertical_speed_mps', settings['return_speed_mps']))
    segments = return_trajectory(effective, current, target)
    previous = np.asarray(current)[:3]
    for _, point, _ in segments:
        check_segment(previous, point[:3], settings)
        previous = point[:3]
    return effective, segments


def prepare_startup(project, settings, target, controller, reader, processor, logger, keyboard,
                    *, confirm=confirm_enter):
    """Return only after lift, alignment, raised air zero and unloaded descent."""
    current = controller.wait_for_standstill().pose.copy()
    config, segments = plan_return(project, settings, current, target)
    axis = project['calibration']['probe_axis_tcp']
    print(f'当前实际 TCP: {current.tolist()}\n本次生成的实验起点: {target.tolist()}')
    print_return_plan(config, current, target, segments)
    print('准备顺序：安全抬升 → 探针轴对准 Base -Z → P0 的 XY 上方 → 空气零偏 → 下降到 P_ref.z。')
    print('请现场核验完整工具、目标和夹具的扫掠空间、旋转空间及抬升高度；P_ref 的 XY 不是运动终点。')
    monitor = PX6DForceMonitor(config, reader, processor, raw_only=True)

    def hold(anchor, *, unloaded=False):
        hold_startup_confirmation(config, anchor, controller)
        state = monitor.sample(read_state=controller.read_diagnostic_state)
        if unloaded:
            if (probe_tilt_deg(state.pose[3:], axis) > settings['probe_tilt_max_deg'] or
                np.linalg.norm(monitor.guard_wrench_base.force) >= settings['unloaded_max_fxy_N']):
                raise RobotError('准备位置探针须竖直且无接触；禁止在接触状态采零或开始接近')
        if logger is not None:
            logger.check_health()

    keyboard.on_wait = lambda: hold(current)
    if not confirm('确认上述完整准备路径、抬升及旋转空间安全；按 Enter 授权自动准备运动',
                   read_line=keyboard.read_line):
        raise KeyboardInterrupt('preparation path not confirmed')
    observed = controller.wait_for_standstill()
    if (np.linalg.norm(observed.pose[:3]-current[:3]) > settings['start_position_tolerance_m'] or
        _orientation_distance(observed.pose[3:], current[3:]) > settings['orientation_tolerance_rad']):
        raise RobotError('机器人位姿已变动，必须重新展示并核验准备路径')
    # Repeat geometric and force guards on the fresh pose before opening Control.
    plan_return(project, settings, observed.pose, target)
    monitor.sample(read_state=controller.read_diagnostic_state)
    logger.write_json('startup_plan.json', dict(initial=current.tolist(), target=target.tolist(),
        segments=[dict(phase=k, pose=p.tolist(), speed=v) for k, p, v in segments],
        path_confirmed=True, site_review=settings['site_validation_note']))
    controller.activate_control(confirmed=True)
    controller.enable_watchdog(settings['watchdog_frequency_hz'])
    controller.kick_watchdog()

    def zero_above(above):
        # SafeReturnExecutor has just confirmed a held stop with watchdog kicks.
        # A second blocking wait would reset that hold without servicing it.
        state = controller.read_diagnostic_state()
        hold_startup_confirmation(config, above, controller)
        if (not controller.standstill_confirmed or state.pose[2] <= target[2] or
            probe_tilt_deg(state.pose[3:], axis) > settings['probe_tilt_max_deg']):
            raise RobotError('空气零偏只能在已调正、停稳且高于实验高度的安全位置采集')
        keyboard.on_wait = lambda: hold(above)
        if not confirm('已到 P0 上方安全高度；确认探针在空气中且完全无接触，按 Enter 采集空气零偏',
                       read_line=keyboard.read_line):
            raise KeyboardInterrupt('raised air zero cancelled')
        hold(above)
        processor.set_tool_orientation(state.pose[3:])
        capture_stationary_bias(config, reader, processor, controller,
            int(project['preprocessing']['baseline']['sample_count']), poll=keyboard.poll, logger=logger)
        logger.write_json('air_zero.json', dict(bias_sensor=processor.zero_bias_sensor.tolist(),
            actual_tcp_pose=controller.read_diagnostic_state().pose.tolist(),
            transform=processor.force_transform_status, timestamp=time.monotonic(), location='above_selected_P0'))
        monitor.use_air_compensated_wrench(processor)
        hold(above, unloaded=True)

    result = SafeReturnExecutor(config, target, controller, force_monitor=monitor, logger=logger,
        poll=keyboard.poll, target_label='single-point prepared P0').execute(before_descent=zero_above)
    if result.status != 'complete':
        raise RobotError(f'单点准备未完成，禁止接近：{result.abort_reason}')
    final = controller.read_state()
    if not controller.standstill_confirmed:
        raise RobotError('准备完成后未保持实际停稳，禁止接近')
    hold(target, unloaded=True)
    if abs(final.pose[2]-target[2]) > settings['fixed_z_tolerance_m']:
        raise RobotError('准备完成后的扫描高度不等于 P_ref.z')
    keyboard.on_wait = lambda: hold(target, unloaded=True)
    logger.write_json('startup_preparation.json', dict(status='complete', initial=current.tolist(),
        target=target.tolist(), actual_final_tcp_pose=final.pose.tolist(),
        probe_axis_tcp=axis, actual_final_tilt_deg=probe_tilt_deg(final.pose[3:], axis),
        configured_tcp_offset=project['tcp']['offset'], air_zero_location='above_selected_P0'))
    return result
