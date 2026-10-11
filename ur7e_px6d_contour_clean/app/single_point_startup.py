"""Confirmed single-point preparation using the existing safe return executor."""
from copy import deepcopy
import time

import numpy as np

from app.operator_input import confirm_enter
from app.scan_startup import capture_stationary_bias, hold_startup_confirmation
from app.single_point_config import check_segment, shared_workspace_config
from app.single_point_ui import show_motion_path, require_remote_mode
from calibration.probe_alignment import probe_tilt_deg
from robot.rtde_controller import RobotError, _orientation_distance
from safety.safe_return import SafeReturnExecutor, PX6DForceMonitor, ReturnAborted, return_trajectory
from safety.force_guard import force_safety_reason


class SinglePointRetreatMonitor(PX6DForceMonitor):
    """Preparation/return use hard backstops; experiment limits are diagnostics.

    Shared by automatic preparation and menu 6. It never zeros an inserted
    probe or changes the shared return executor, runner, or sensor transform.
    """
    def __init__(self, config, reader, processor, controller, current, lifted):
        super().__init__(deepcopy(config), reader, processor, raw_only=True)
        self.controller = controller
        self.anchor = np.asarray(current, dtype=float).copy()
        self.lifted = np.asarray(lifted, dtype=float).copy()
        self.return_limits = {key: config['safe_return'][key] for key in
                              ('return_force_limit', 'return_torque_limit')}
        self.phase = 'CONTACT_RETREAT'
        self.initial_wrench = None
        self.retreat_guard_wrench_base = None
        self.retreat_confirmed_sec = None
        self.last_failure = ''
        self.retreat_policy = deepcopy(config['policy'])
        self.retreat_policy.update(safety_force_threshold=config['policy']['absolute_raw_force_threshold'],
                                   safety_torque_threshold=config['policy']['absolute_raw_torque_threshold'])
        self.config['safe_return'].update(return_force_limit=self.retreat_policy['safety_force_threshold'],
                                          return_torque_limit=self.retreat_policy['safety_torque_threshold'])

    def sample(self, *, read_state=None):
        self.raw = None  # A failed fresh read must never report a previous sample.
        self.robot_state = None
        self.retreat_guard_wrench_base = None
        def guarded_state():
            if read_state is None:
                raise ReturnAborted('contact retreat requires fresh actual TCP feedback')
            state = read_state()
            self.robot_state = state
            if self.initial_wrench is None:
                self.initial_wrench = self.raw.array().tolist()
            if self.preprocessor.force_transform_status['output_frame'] != 'Base':
                raise ReturnAborted('Base transform unavailable for preparation/return protection')
            self.preprocessor.set_tool_orientation(state.pose[3:])
            self.retreat_guard_wrench_base = self.preprocessor.air_compensated_wrench_base(self.raw)
            diagnostic = force_safety_reason(self.raw, self.retreat_guard_wrench_base, self.config['policy'])
            if diagnostic:
                self.diagnostics['retreat_experiment_threshold'] = diagnostic + '; diagnostic only before final experiment authorization'
            reason = force_safety_reason(self.raw, self.retreat_guard_wrench_base, self.retreat_policy)
            if reason:
                raise ReturnAborted(reason)
            if self.phase == 'CONTACT_RETREAT':
                tolerance = float(self.config['safe_return']['return_position_tolerance'])
                if (np.linalg.norm(state.pose[:2]-self.anchor[:2]) > tolerance or
                    _orientation_distance(state.pose[3:], self.anchor[3:]) >
                        self.config['safe_return']['return_orientation_tolerance'] or
                    not self.anchor[2]-tolerance <= state.pose[2] <= self.lifted[2]+tolerance):
                    raise ReturnAborted('contact retreat must remain on its confirmed vertical +Z path')
                if (abs(state.pose[2]-self.lifted[2]) <= tolerance and
                    self.controller.standstill_confirmed and
                    np.linalg.norm(state.tcp_speed[:3]) <= self.config['continuous_tracking']['settle_speed_mps'] and
                    np.linalg.norm(state.tcp_speed[3:]) <= .005):
                    self.phase = 'RETURN_PATH'
                    self.retreat_confirmed_sec = time.monotonic()
                    self.diagnostics['single_point_return_phase'] = 'vertical retreat held stopped; preparation/return still uses hard backstops only'
            return state
        try:
            return super().sample(read_state=guarded_state)
        except Exception as exc:
            # FreshForceReader carries the actual over-limit raw frame on its
            # exception. Preserve evidence even when no TCP read followed it.
            if self.raw is None and getattr(exc, 'raw', None) is not None:
                self.raw = exc.raw
                if self.initial_wrench is None:
                    self.initial_wrench = self.raw.array().tolist()
            force = '无新鲜读数' if self.raw is None else f'{np.linalg.norm(self.raw.force):.3f}'
            torque = '无新鲜读数' if self.raw is None else f'{np.linalg.norm(self.raw.torque):.3f}'
            limits = self.config['safe_return']
            base_detail = ('' if self.retreat_guard_wrench_base is None else
                           f'；Base 力/力矩（含力臂）='
                           f'{np.linalg.norm(self.retreat_guard_wrench_base.force):.3f} N / '
                           f'{np.linalg.norm(self.retreat_guard_wrench_base.torque):.3f} N·m')
            self.last_failure = (f'{exc}; 阶段={self.phase}；原始 Sensor 力={force} N，'
                                 f'力矩={torque} N·m{base_detail}；本阶段限值='
                                 f'{limits["return_force_limit"]} N / {limits["return_torque_limit"]} N·m；'
                                 '未在接触位置采零')
            raise ReturnAborted(self.last_failure) from exc

    def snapshot(self):
        return dict(phase=self.phase, initial_wrench_sensor=self.initial_wrench,
                    last_wrench_sensor=None if self.raw is None else self.raw.array().tolist(),
                    force_read_start_sec=None if self.raw is None else getattr(self.reader, 'start', None),
                    force_read_end_sec=None if self.raw is None else getattr(self.reader, 'end', None),
                    actual_tcp_pose=None if self.robot_state is None else self.robot_state.pose.tolist(),
                    actual_tcp_speed=None if self.robot_state is None else self.robot_state.tcp_speed.tolist(),
                    retreat_guard_wrench_base=(None if self.retreat_guard_wrench_base is None else
                                              self.retreat_guard_wrench_base.array().tolist()),
                    retreat_anchor=self.anchor.tolist(), lifted_target=self.lifted.tolist(),
                    retreat_confirmed_sec=self.retreat_confirmed_sec,
                    active_force_limit_N=self.config['safe_return']['return_force_limit'],
                    active_torque_limit_Nm=self.config['safe_return']['return_torque_limit'],
                    original_return_limits=self.return_limits, original_return_limits_enforced=False,
                    air_bias_captured=self.enforce_processed_limits,
                    diagnostics=deepcopy(self.diagnostics), failure=self.last_failure)


def plan_return(project, settings, current, target):
    if settings['return_lift_distance_m'] is None:
        raise RobotError('自动准备/返回未核验：请配置现场确认的抬升距离，不能假设 30 mm 安全；请手动移到安全位置')
    effective = deepcopy(project)
    effective['workspace'] = shared_workspace_config(project, settings)
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
    print('准备自动运动：请在示教器切换 Remote 模式；模式只读检查通过后仍需路径核验和 Enter 确认。')
    show_motion_path(current, target, segments)
    print('准备顺序：安全抬升 → 探针轴对准 Base -Z → P0 的 XY 上方 → 空气零偏 → 下降到 P_ref.z。')
    print('准备和返回阶段不使用实验及返回的低力/力矩阈值拦截；正式启动检查在最终 Enter 后执行。')
    print('请现场核验完整工具、目标和夹具的扫掠空间、旋转空间及抬升高度；P_ref 的 XY 不是运动终点。')
    # Preparation/return use only hard backstops. The trial owns experiment
    # thresholds and the unloaded-start check after final Enter authorization.
    monitor = SinglePointRetreatMonitor(config, reader, processor, controller, current, segments[0][1])

    def hold(anchor):
        hold_startup_confirmation(config, anchor, controller)
        monitor.sample(read_state=controller.read_diagnostic_state)
        if logger is not None:
            logger.check_health()

    # Waiting for path authorization is read-only. Do not apply the motion
    # force monitor before Enter; the fresh preflight below remains mandatory.
    keyboard.on_wait = lambda: hold_startup_confirmation(config, current, controller)
    if not confirm('确认上述完整准备路径、抬升及旋转空间安全；按 Enter 授权自动准备运动',
                   read_line=keyboard.read_line):
        raise KeyboardInterrupt('preparation path not confirmed')
    observed = controller.wait_for_standstill()
    if (np.linalg.norm(observed.pose[:3]-current[:3]) > settings['start_position_tolerance_m'] or
        _orientation_distance(observed.pose[3:], current[3:]) > settings['orientation_tolerance_rad']):
        raise RobotError('机器人位姿已变动，必须重新展示并核验准备路径')
    # Repeat geometric and force guards on the fresh pose before opening Control.
    plan_return(project, settings, observed.pose, target)
    try:
        monitor.sample(read_state=controller.read_diagnostic_state)
    finally:
        logger.write_json('startup_force_preflight.json', monitor.snapshot())
    logger.write_json('startup_plan.json', dict(initial=current.tolist(), target=target.tolist(),
        segments=[dict(phase=k, pose=p.tolist(), speed=v) for k, p, v in segments],
        path_confirmed=True))
    require_remote_mode(project['robot']['robot_ip'])
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
        hold(above)

    try:
        result = SafeReturnExecutor(config, target, controller, force_monitor=monitor, logger=logger,
            poll=keyboard.poll, target_label='single-point prepared P0').execute(before_descent=zero_above)
    finally:
        logger.write_json('startup_force_status.json', monitor.snapshot())
    if result.status != 'complete':
        raise RobotError(f'单点准备未完成，禁止接近：{result.abort_reason}')
    final = controller.read_state()
    if not controller.standstill_confirmed:
        raise RobotError('准备完成后未保持实际停稳，禁止接近')
    hold(target)
    if abs(final.pose[2]-target[2]) > settings['fixed_z_tolerance_m']:
        raise RobotError('准备完成后的扫描高度不等于 P_ref.z')
    # Preparation is complete. Waiting for speed selection/final experiment
    # authorization must not keep running the preparation force monitor.
    keyboard.on_wait = lambda: hold_startup_confirmation(config, target, controller)
    logger.write_json('startup_preparation.json', dict(status='complete', initial=current.tolist(),
        target=target.tolist(), actual_final_tcp_pose=final.pose.tolist(),
        probe_axis_tcp=axis, actual_final_tilt_deg=probe_tilt_deg(final.pose[3:], axis),
        configured_tcp_offset=project['tcp']['offset'], air_zero_location='above_selected_P0'))
    return result
