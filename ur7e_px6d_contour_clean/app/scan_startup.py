"""Shared confirmed activation, stationary bias and force-monitored startup return."""
import time
import numpy as np

from robot.rtde_controller import RobotError, _orientation_distance, _rotvec_to_matrix
from safety.force_guard import raw_safety_reason
from safety.safe_return import SafeReturnExecutor, PX6DForceMonitor, return_trajectory, print_return_plan
from sensor.force_preprocess import WrenchPreprocessor
from app.operator_input import confirm_enter


def startup_position_tolerance(config):
    return float(config['continuous_tracking']['startup_position_tolerance']
                 if config.get('continuous_real_execution') else config['policy']['position_tolerance'])


def check_startup_stationary(config, state, controller, *, anchor=None):
    c = config['continuous_tracking']
    real = config.get('continuous_real_execution', False)
    translation = np.linalg.norm(state.tcp_speed[:3])
    angular = np.linalg.norm(state.tcp_speed[3:])
    drift = 0. if anchor is None else np.linalg.norm(state.pose[:3]-anchor[:3])
    if (translation > float(c['startup_speed_mps'] if real else c['settle_speed_mps']) or
        angular > (float(c['startup_angular_speed_rad_s']) if real else .005) or
        drift > startup_position_tolerance(config) or
        (anchor is not None and _orientation_distance(state.pose[3:], anchor[3:]) >
         config['safe_return']['return_orientation_tolerance'])):
        raise RobotError('robot moved during stationary startup/bias check')
    if real and (translation > c['settle_speed_mps'] or angular > .005 or
                 drift > config['policy']['position_tolerance']):
        controller.diagnostics['startup_stationary'] = (
            f'WARNING: startup noise: translation={translation:g} m/s, angular={angular:g} rad/s, drift={drift:g} m')


def capture_stationary_bias(config, reader, preprocessor, controller, count, *, poll=lambda: None, logger=None):
    anchor = controller.read_diagnostic_state().pose.copy()
    samples = []
    for _ in range(int(count)):
        started = time.monotonic()
        if poll() in ('Q', 'ESC'):
            raise KeyboardInterrupt('operator stopped bias acquisition')
        raw = reader.read_wrench()
        reason = raw_safety_reason(raw, config['policy'])
        if reason:
            raise RobotError(reason)
        state = controller.read_diagnostic_state()
        check_startup_stationary(config, state, controller, anchor=anchor)
        if logger is not None and hasattr(logger, 'check_health'):
            logger.check_health()
        if time.monotonic()-started > config['continuous_tracking']['cycle_timeout_sec']:
            if config.get('continuous_real_execution'):
                controller.diagnostics['bias_timing'] = 'WARNING: bias cycle timing jitter'
            elif controller.watchdog_active:
                raise RobotError('bias cycle timed out')
        if controller.config.get('continuous_require_watchdog') and controller.watchdog_active:
            controller._check_watchdog_health()
            controller.kick_watchdog()
        samples.append(raw)
        time.sleep(max(0., 1/float(config['sensor']['poll_rate_hz'])-(time.monotonic()-started)))
    preprocessor.set_zero_bias(samples)


def hold_startup_confirmation(config, start, controller):
    """Observe standstill during an operator prompt; keep an existing watchdog alive."""
    check_startup_stationary(config, controller.read_diagnostic_state(), controller, anchor=start)
    if controller.config.get('continuous_require_watchdog') and controller.watchdog_active:
        controller._check_watchdog_health()
        controller.kick_watchdog()


def startup_scan(config, start, controller, reader, logger, poll, *, confirm=None, before_descent=None):
    startup = {'startup': True} if config.get('continuous_real_execution') else {}
    current = controller.wait_for_standstill(**startup)
    segments = return_trajectory(config, current.pose, start)
    away = (np.linalg.norm(current.pose[:3]-start[:3]) > startup_position_tolerance(config) or
            _orientation_distance(current.pose[3:], start[3:]) > config['safe_return']['return_orientation_tolerance'])
    print(f'当前 TCP: {current.pose.tolist()}\n扫描 P0: {start.tolist()}')
    print(f'初始方向 P0 -> P1: {config["policy"]["search_direction_xy"]}')
    print_return_plan(config, current.pose, start, segments, scan=True)
    alignment = config.get('continuous_probe_alignment')
    def tilt(pose):
        axis = np.asarray(alignment['axis_tcp'], dtype=float)
        axis /= np.linalg.norm(axis)
        return float(np.degrees(np.arccos(np.clip((_rotvec_to_matrix(pose[3:]) @ axis) @ [0., 0., -1.], -1., 1.))))
    def report_alignment(final):
        if not alignment:
            return
        record = dict(alignment, actual_before_tilt_deg=tilt(current.pose),
            actual_after_tilt_deg=tilt(final.pose), actual_final_tcp_pose=final.pose.tolist(),
            configured_tcp_offset=controller.config.get('tcp_offset'),
            reference='Base -Z; physical downward requires a level robot base')
        print(f"探针轴倾角（相对 Base -Z）: {record['actual_before_tilt_deg']:.4f}° -> "
              f"{record['actual_after_tilt_deg']:.4f}°；目标 {tilt(start):.4f}°。")
        if logger is not None:
            logger.write_json('startup_alignment.json', record)
    if alignment:
        print(f'当前探针倾角 {tilt(current.pose):.4f}°，目标 {tilt(start):.4f}°（Base -Z，基座须水平）。')
    if not away and before_descent is None:
        print('Startup return: already at P0 and aligned; no return motion required.')
        report_alignment(current)
        return False
    temporary = WrenchPreprocessor.from_config(config['preprocessing'])
    if not alignment and before_descent is None:
        if not confirm_enter('探针须脱离目标、静止空载；即将采集启动返回用零偏。', read_line=confirm):
            raise KeyboardInterrupt('bias not confirmed')
        fresh = controller.wait_for_standstill(**startup)
        if (np.linalg.norm(fresh.pose[:3]-current.pose[:3]) > startup_position_tolerance(config) or
            _orientation_distance(fresh.pose[3:], current.pose[3:]) > config['safe_return']['return_orientation_tolerance']):
            raise RobotError('robot moved during path confirmation')
        capture_stationary_bias(config, reader, temporary, controller,
            config['safe_return']['startup_bias_sample_count'], poll=poll, logger=logger)
    prompt = ('机器人静止，基座水平，抬升后有足够旋转空间；核对路径，即将垂直抬升脱离颗粒、'
              '调正探针并移至扫描 P0 正上方，在抬升位置采零后再下降到 P0。' if before_descent is not None else
              '探针已脱离目标及颗粒、静止空载，基座水平，抬升后有足够旋转空间；'
              '核对路径，即将抬升、调正探针并自动返回扫描 P0（此时不采零）。' if alignment else
              '核对完整返回路径和姿态；即将自动返回扫描 P0。')
    if not confirm_enter(prompt, read_line=confirm):
        raise KeyboardInterrupt('startup return not confirmed')
    fresh = controller.wait_for_standstill(**startup)
    if (np.linalg.norm(fresh.pose[:3]-current.pose[:3]) > startup_position_tolerance(config) or
        _orientation_distance(fresh.pose[3:], current.pose[3:]) > config['safe_return']['return_orientation_tolerance']):
        raise RobotError('robot moved during path confirmation')
    monitor = PX6DForceMonitor(config, reader, temporary, raw_only=bool(alignment or before_descent))
    if before_descent is not None:
        # Verify the existing raw force guard before opening control for the
        # mandatory lift. Preserve sensor errors and any force-limit evidence.
        try:
            monitor.sample(read_state=controller.read_diagnostic_state)
        finally:
            recorder = getattr(logger, 'termination', None)
            if recorder is not None:
                recorder.observe(raw=monitor.raw, processed=monitor.processed, robot=fresh,
                                 processed_force_frame=monitor.force_frame)
    controller.activate_control(confirmed=True)
    if away or before_descent is not None:
        if controller.config.get('continuous_require_watchdog'):
            controller.enable_watchdog(config['continuous_tracking']['watchdog_frequency_hz'])
        result = SafeReturnExecutor(config, start, controller,
            force_monitor=monitor,
            logger=logger, poll=poll).execute(before_descent=before_descent)
        if result.status != 'complete':
            raise RobotError(f'startup return aborted: {result.abort_reason}')
    report_alignment(controller.read_diagnostic_state())
    return away or before_descent is not None
