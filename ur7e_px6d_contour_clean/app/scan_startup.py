"""Shared confirmed activation, stationary bias and force-monitored startup return."""
import time
import numpy as np

from robot.rtde_controller import RobotError, _orientation_distance
from safety.force_guard import raw_safety_reason
from safety.safe_return import SafeReturnExecutor, PX6DForceMonitor, return_trajectory
from sensor.force_preprocess import WrenchPreprocessor


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


def startup_scan(config, start, controller, reader, logger, poll, *, confirm=None):
    startup = {'startup': True} if config.get('continuous_real_execution') else {}
    current = controller.wait_for_standstill(**startup)
    segments = return_trajectory(config, current.pose, start)
    away = (np.linalg.norm(current.pose[:3]-start[:3]) > startup_position_tolerance(config) or
            _orientation_distance(current.pose[3:], start[3:]) > config['safe_return']['return_orientation_tolerance'])
    print(f'当前 TCP: {current.pose.tolist()}\n扫描 P0: {start.tolist()}')
    print(f'初始方向 P0 -> P1: {config["policy"]["search_direction_xy"]}')
    if away:
        for phase, point, speed in segments:
            print(f'{phase}: {point.tolist()}, speed={speed} m/s')
    print('PX6D 力监控参与扫描与返回；请确认空载零偏、力方向、完整路径和现场急停。')
    if (confirm or input)('输入 START 执行必要的启动返回并开始扫描：').strip() != 'START':
        raise KeyboardInterrupt('START not confirmed')
    fresh = controller.wait_for_standstill(**startup)
    if (np.linalg.norm(fresh.pose[:3]-current.pose[:3]) > startup_position_tolerance(config) or
        _orientation_distance(fresh.pose[3:], current.pose[3:]) > config['safe_return']['return_orientation_tolerance']):
        raise RobotError('robot moved during path confirmation')
    temporary = WrenchPreprocessor.from_config(config['preprocessing'])
    if away:
        capture_stationary_bias(config, reader, temporary, controller,
            config['safe_return']['startup_bias_sample_count'], poll=poll, logger=logger)
    controller.activate_control(confirmed=True)
    if away:
        if controller.config.get('continuous_require_watchdog'):
            controller.enable_watchdog(config['continuous_tracking']['watchdog_frequency_hz'])
        result = SafeReturnExecutor(config, start, controller,
            force_monitor=PX6DForceMonitor(config, reader, temporary), logger=logger, poll=poll).execute()
        if result.status != 'complete':
            raise RobotError(f'startup return aborted: {result.abort_reason}')
    return away
