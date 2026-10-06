#!/usr/bin/env python3
"""Independent experimental runner. Defaults to entirely offline simulation."""
from __future__ import annotations

import argparse
import json
from copy import deepcopy
from pathlib import Path
import time

import numpy as np
from core.models import Wrench

from app.operator_input import OperatorKeyboard, confirm_enter
from config.loader import load_config
from experiment_logging.data_logger import ExperimentLogger
from experiment_logging.continuous_writer import ContinuousLogWriter
from experiment_logging.termination import TerminationReason, TerminationRecorder
from policy.continuous_tracking import ContinuousTrackingPolicy, EXTRA_SAMPLE_FIELDS, State
from robot.rtde_controller import URRTDEController, RobotError, SearchLimitReached, SPEED_GUARD_FIELDS, check_continuous_xy
from sensor.force_preprocess import WrenchPreprocessor
from safety.force_guard import continuous_force_reason, raw_safety_reason
from sensor.px6d_reader import PX6DError, PX6DReader
from simulation.continuous_session import SIMULATION_SAMPLE_FIELDS

ROOT = Path(__file__).resolve().parents[1]
TIMING_FIELDS = ('serial_read_start', 'serial_read_end', 'tcp_read_start', 'tcp_read_end',
                 'rtde_device_timestamp', 'rtde_packet_stagnation_sec', 'wrench_host_age_sec',
                 'tcp_host_age_sec', 'command_send_time', 'command_return_time', 'timing_source',
                 'cycle_start_time', 'loop_start_interval_sec', 'runtime_stop_state',
                 'actual_xyz_speed_mps', 'stop_api_anomaly', 'software_warnings',
                 'speedl_sequence', 'speedl_kind', 'speedl_host_monotonic',
                 'speedl_vx', 'speedl_vy', 'speedl_accepted')
TRACKING_SPEED_FIELDS = ('tracking_base_speed_mps', 'tracking_speed_multiplier',
                         'tracking_nominal_speed_mps')


def choose_tracking_speed(base_speed, max_speed, *, read_text, emit=print):
    """Select this run's tangent speed from the original base, never a prior choice."""
    while True:
        answer = read_text('贴边扫描速度倍率（例如 1、2、3、5；回车默认 1 倍）：')
        if answer.strip().lower() in ('q', 'esc', '\x1b'):
            raise KeyboardInterrupt('operator cancelled tracking speed choice')
        try:
            multiplier = float(answer.strip()) if answer.strip() else 1.
        except ValueError:
            emit('请输入有限正数倍率。', flush=True)
            continue
        if not np.isfinite(multiplier) or multiplier <= 0:
            emit('请输入有限正数倍率。', flush=True)
            continue
        nominal_speed = base_speed * multiplier
        if not np.isfinite(nominal_speed) or nominal_speed > max_speed or nominal_speed <= 0:
            emit(f'名义贴边速度须大于 0 且不超过总速度上限 {max_speed*1000:g} mm/s，请重新输入。', flush=True)
            continue
        emit(f'基准速度：{base_speed*1000:g} mm/s；所选倍率：{multiplier:g} 倍；'
             f'本次名义贴边速度：{nominal_speed*1000:g} mm/s。\n'
             '实际速度仍受原有受力减速、加速度限制和速度限幅影响。', flush=True)
        return dict(zip(TRACKING_SPEED_FIELDS, (base_speed, multiplier, nominal_speed)))


from app.configuration import prepare_real, calibration_summary, site_configuration_digest
from app.scan_startup import (startup_scan, capture_stationary_bias,
                             capture_stationary_granular_baseline,
                             startup_position_tolerance, check_startup_stationary, hold_startup_confirmation)
from experiment_logging.paths import git_provenance

def validate_cycle_timing(c, start, now, oldest_observation, previous_start=None, *, diagnostics=None):
    values = [start, now, oldest_observation] + ([] if previous_start is None else [previous_start])
    if not np.all(np.isfinite(values)):
        raise RobotError('invalid host observation time')
    def report(key, message):
        if diagnostics is None:
            raise RobotError(message)
        diagnostics[key] = f'WARNING: {message}'
    if now < start or oldest_observation > now:
        report('host_clock', 'invalid host observation time')
    if previous_start is not None and not 0 < start-previous_start <= float(c['max_sample_gap_sec']):
        report('sample_gap', f'control cycle gap {start-previous_start:g} s')
    if now-start > float(c['cycle_timeout_sec']):
        report('cycle_timing', f'control cycle {now-start:.6f} s (including device/log latency)')
    if now-oldest_observation > float(c['max_observation_age_sec']):
        report('observation_age', f'host observation age {now-oldest_observation:g} s')



def check_start(controller, start, config):
    robot = controller.read_state()
    if np.linalg.norm(robot.pose[:3] - start[:3]) > startup_position_tolerance(config):
        raise RobotError("start TCP is not at calibrated P0; position it before this experiment")
    if config.get('continuous_real_execution'):
        check_startup_stationary(config, robot, controller, anchor=start)
    else:
        linear_speed = np.linalg.norm(robot.tcp_speed[:3])
        angular_speed = np.linalg.norm(robot.tcp_speed[3:])
        if linear_speed > 1e-4 or angular_speed > .005:
            raise RobotError('robot must be stationary at P0 before starting')
    return robot


def stop_after_exception(controller, termination):
    """A failed stop attempt must not skip disconnect/diagnostic cleanup."""
    try:
        controller.request_stop(nonblocking=True)
    except Exception as exc:
        termination.set_stop_reason(detail=str(exc), exception=exc,
                                    source="continuous.stop_failure", terminal=False)


def run(args, *, controller_factory=URRTDEController, reader_factory=PX6DReader):
    if getattr(args, 'preview', False):
        if args.execute:
            raise ValueError('--preview and --execute are mutually exclusive')
        from simulation.continuous_preview import launch_preview
        try:
            return launch_preview(args, load_config(args.config), git_provenance())
        except KeyboardInterrupt:
            print('Continuous preview: Ctrl+C; run ended, no contour completion claim', flush=True)
            return 0
        except Exception as exc:
            print(f'Continuous preview stopped: {type(exc).__name__}: {exc}', flush=True)
            return 1
    controller = reader = logger = policy = preprocessor = None
    robot = raw = processed = None
    termination = TerminationRecorder(output_root=getattr(args, "output", None), mode="real" if args.execute else "simulation", strategy="continuous", config_source=args.config)
    result = 0
    now = 0.0
    config = None
    stop_snapshot_info = dict(source='unavailable', standstill_confirmed=False)
    previous_cycle_start = None
    last_wrench_observed = None
    sensor_failed = False
    robot_connect_started = False
    timing = dict.fromkeys(TIMING_FIELDS, '')
    diagnostics = {}
    def software_warnings():
        return dict(runtime=dict(diagnostics), controller=dict(getattr(controller, 'diagnostics', {})),
                    policy=dict(getattr(policy, 'diagnostics', {})), logging=getattr(logger, 'diagnostics', {}))
    try:
        termination.observe(phase='CONFIG_PREFLIGHT')
        config = deepcopy(load_config(args.config))
        config['continuous_provenance'] = git_provenance()
        # Load the saved project calibrations before optional duration shortening;
        # validate any explicit extra record; local recovery is enabled by config.
        if args.execute:
            if getattr(args, 'enable_reacquire', False):
                raise RobotError('--enable-reacquire is offline only')
            robot_config, start = prepare_real(config, args.config)
            geometry = config['continuous_search_geometry']
            print(f"Search geometric distance to sandbox boundary: {geometry['geometric_distance_m']*1000:.4f} mm\n"
                  f"Search stopping margin: {geometry['stopping_margin_m']*1000:.4f} mm\n"
                  f"Effective TARGET_SEARCH distance: {geometry['usable_distance_m']*1000:.4f} mm", flush=True)
            if 'continuous_calibration_sources' in config:
                print("Using saved scan / sandbox calibration:\n"+
                      json.dumps(calibration_summary(config),ensure_ascii=False,indent=2),flush=True)
        elif getattr(args, 'enable_reacquire', False):
            config['continuous_tracking']['reacquire_enabled'] = True
        if args.duration is not None:
            if not np.isfinite(args.duration) or args.duration <= 0:
                raise ValueError("--duration must be finite and positive")
            if not args.execute:
                budget = config["continuous_tracking"]["max_runtime_sec"]
                config["continuous_tracking"]["max_runtime_sec"] = (
                    args.duration if budget is None else min(args.duration, float(budget)))
            else:
                print('WARNING: --duration applies to simulation only; real continuous execution has no runtime stop budget.', flush=True)
        dt = 1 / float(config["policy"]["control_rate_hz"])
        baseline = config['preprocessing'].get('baseline', {'capture_on_start': True, 'sample_count': 100})
        granular = config['preprocessing'].get('granular_baseline', dict(
            capture_on_start=bool(baseline.get('capture_on_start', True)), sample_count=baseline['sample_count']))
        if not isinstance(granular.get('capture_on_start', True), bool):
            raise ValueError('granular_baseline.capture_on_start must be boolean')
        if args.execute and granular.get('capture_on_start', True):
            if not baseline.get('capture_on_start', True):
                raise ValueError('granular baseline capture requires the raised-position air zero')
            count = granular.get('sample_count', baseline['sample_count'])
            if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
                raise ValueError('granular_baseline.sample_count must be a positive integer')
        if args.execute:
            preprocessor = WrenchPreprocessor.from_config(config["preprocessing"], tool_orientation=start[3:])
            config['force_transform_status'] = preprocessor.force_transform_status
            config['force_transform_status']['orientation_source'] = 'planned_scan_target_not_measured'
            termination.observe(force_transform_status=config['force_transform_status'],
                processed_force_frame=config['force_transform_status']['output_frame'])
            # A supplied mounting transform is not a physical verification of
            # the force sign. Missing installation data must remain explicit.
            config['force_display'] = dict(schema_version=1,
                frame=preprocessor.force_transform_status['output_frame'], force_source='processed_wrench',
                physical_sign_confirmed=False, base_frame_confirmed=False, physical_available=False)
            print('Force transform: '+json.dumps(config['force_transform_status'], ensure_ascii=False), flush=True)
            policy = ContinuousTrackingPolicy(config)  # validate before any connection
            controller = controller_factory(robot_config)
            s = config["sensor"]
            reader = reader_factory(s["serial_port"], s["baudrate"], s["timeout_sec"],
                                s["poll_rate_hz"], s["startup_delay_sec"])
        else:
            from simulation.simulator import load_simulation_config
            from simulation.continuous_session import SimulationSession
            session = SimulationSession(config, load_simulation_config(args.scene))
            config = session.config
            policy, controller, reader = session.policy, session.robot, session.sensor
            preprocessor = session.preprocessor
        output = args.output
        # Latch the freshly loaded base before any run-only choice. Offline
        # simulation retains its noninteractive default; execution asks below.
        base_tangential_speed = float(config['continuous_tracking']['tangential_speed'])
        selection = dict(zip(TRACKING_SPEED_FIELDS, (base_tangential_speed, 1., base_tangential_speed)))
        config['continuous_speed_selection'] = selection
        timing.update(selection)
        # Existing optional workspace exporter renders on close and assumes
        # real Base coordinates; use only our explicit offline replay here.
        logger = ExperimentLogger(output, config, mode="real" if args.execute else "simulation", strategy="continuous", config_source=args.config, extra_sample_fields=EXTRA_SAMPLE_FIELDS + TIMING_FIELDS + TRACKING_SPEED_FIELDS + SIMULATION_SAMPLE_FIELDS + SPEED_GUARD_FIELDS,
                                  workspace_logging=False)
        logger.termination = termination.bind(logger.run_dir)
        if args.execute:
            # Disk work stays off the device thread. Backlog/write diagnostics
            # cannot end motion; the bounded queue records any dropped samples.
            logger = ContinuousLogWriter(logger, max_pending_sec=float(
                config['continuous_tracking']['confirmation_timeout_sec']), diagnostic_only=True)
        termination.observe(policy=policy, processed_force_frame=config.get('force_display', {}).get('frame', 'Base'))
        print(f"Continuous run: {logger.run_dir}", flush=True)
        if args.execute:
            termination.observe(phase='SENSOR_CONNECT')
            reader.connect()
            robot_connect_started = True
            termination.observe(phase='ROBOT_CONNECT')
            controller.connect(allow_start_away_from_fixed_pose=True)
        else:
            robot = controller.read_state()
        startup_return_done = False
        with OperatorKeyboard() as keyboard:
            if args.execute:
                def prepare_descent(above):
                    # Reuse MOVE_ABOVE_START's safe height. Never zero at scan
                    # depth, including a startup already positioned at P0.
                    termination.observe(phase='SCAN_BIAS_ABOVE_P0')
                    state = controller.read_state()
                    check_startup_stationary(config, state, controller, anchor=above)
                    preprocessor.set_tool_orientation(state.pose[3:])
                    keyboard.on_wait = lambda: hold_startup_confirmation(config, above, controller)
                    if baseline.get('capture_on_start', True):
                        if not confirm_enter('P0 正上方抬升位置：确认探针脱离目标及颗粒、静止空载；即将采集扫描零偏。',
                                             read_line=keyboard.read_line):
                            raise KeyboardInterrupt('bias not confirmed')
                        capture_stationary_bias(config, reader, preprocessor, controller,
                            baseline['sample_count'], poll=keyboard.poll, logger=logger)
                        logger.write_json('scan_bias_at_start.json', dict(
                            reference='above_P0_before_vertical_insertion', tcp_pose=state.pose.tolist(),
                            zero_bias_sensor=preprocessor.zero_bias_sensor.tolist(), sample_count=baseline['sample_count']))
                    selection = choose_tracking_speed(base_tangential_speed, float(config['robot']['max_tcp_speed']),
                                                      read_text=keyboard.read_text)
                    # Only the desired tangential component changes; retain all
                    # return/reacquire/normal speeds and execution protections.
                    config['continuous_tracking']['tangential_speed'] = selection['tracking_nominal_speed_mps']
                    policy.c['tangential_speed'] = selection['tracking_nominal_speed_mps']
                    # Reuse prepare_real's combined tangent/normal envelope
                    # with this run's selected tangent, leaving other phases
                    # and the global TCP speed guard unchanged.
                    tracking_limit = float(np.hypot(policy.c['tangential_speed'], policy.c['normal_speed_limit']))
                    controller.config['continuous_speed_limits']['CONTINUOUS_TRACKING'] = tracking_limit
                    config['continuous_execution_envelope']['nominal_speed_limits_mps']['CONTINUOUS_TRACKING'] = tracking_limit
                    config['continuous_speed_selection'] = selection
                    timing.update(selection)
                    logger.write_config_snapshot(config)
                    logger.write_json('tracking_speed_selection.json', selection)
                    if not confirm_enter('即将垂直向下插入至 P0 扫描深度，停稳并处理颗粒背景力后，从 P0 沿保存方向开始扫描。',
                                         read_line=keyboard.read_line):
                        raise KeyboardInterrupt('scan not confirmed')
                    hold_startup_confirmation(config, above, controller)
                    termination.observe(phase='SCAN_INSERTION')
                    return preprocessor if baseline.get('capture_on_start', True) else None
                termination.observe(phase='STARTUP_RETURN')
                startup_return_done = startup_scan(config, start, controller, reader, logger,
                    keyboard.poll, confirm=keyboard.read_line, before_descent=prepare_descent)
                robot = check_start(controller, start, config)
                preprocessor.set_tool_orientation(robot.pose[3:])
                actual_transform = dict(preprocessor.force_transform_status,
                    orientation_source='rtde_actual_tcp_at_start', host_monotonic=robot.timestamp)
                termination.observe(force_transform_status=actual_transform)
                logger.write_json('force_transform_at_start.json', actual_transform)
                keyboard.on_wait = lambda: hold_startup_confirmation(config, start, controller)
                background_diagnostics = None
                if granular.get('capture_on_start', True):
                    termination.observe(phase='GRANULAR_BASELINE_AT_P0')
                    if not confirm_enter('P0 扫描深度：确认只有颗粒背景、未接触目标且静止；即将采集颗粒 baseline（不重采传感器 zero bias）。',
                                         read_line=keyboard.read_line):
                        raise KeyboardInterrupt('granular baseline not confirmed')
                    background_diagnostics = capture_stationary_granular_baseline(config, reader, preprocessor, controller,
                        granular.get('sample_count', baseline['sample_count']), poll=keyboard.poll, logger=logger)
                    background_source = 'target_free_stationary_P0_after_insertion'
                else:
                    # An explicit configured Base background is independent of
                    # air zero; calibration resets the scan filter once only.
                    preprocessor.set_granular_baseline(Wrench.from_sequence(preprocessor.granular_baseline_output))
                    background_source = 'configured_Base_background'
                config['preprocessing']['granular_baseline_output'] = preprocessor.granular_baseline_output.tolist()
                logger.write_json('granular_baseline_at_start.json', dict(
                    source=background_source, frame='Base', tcp_pose=controller.read_state().pose.tolist(),
                    granular_baseline_output=preprocessor.granular_baseline_output.tolist(),
                    zero_bias_sensor=preprocessor.zero_bias_sensor.tolist(),
                    diagnostics=background_diagnostics,
                    sample_count=granular.get('sample_count', baseline['sample_count']) if granular.get('capture_on_start', True) else 0,
                    condition='target-free background; no target force may be calibrated away'))
                logger.write_config_snapshot(config)
            if not args.execute:
                session.capture_bias(keyboard.poll, termination.observe)
        if args.execute and not startup_return_done and controller.config.get('continuous_require_watchdog'):
            controller.enable_watchdog(float(policy.c['watchdog_frequency_hz']))
        with OperatorKeyboard() as keyboard:
            while policy.state != State.STOP or (args.execute and controller.stop_state != 'STOPPED'):
                cycle_start = time.monotonic()
                now = cycle_start if args.execute else controller.time
                key = keyboard.poll()
                if key in ("Q", "ESC"):
                    policy.request_stop(now, robot.pose, f"{key} operator stop", event="USER_STOP",
                                        code=TerminationReason.STOP_USER_REQUEST)
                    if not args.execute:
                        break
                    controller.request_stop(nonblocking=True)  # Before another potentially blocking sensor read.
                termination.observe(phase="SENSOR_READ")
                if args.execute:
                    logger.check_health()
                    validate_cycle_timing(policy.c, cycle_start, time.monotonic(), cycle_start, previous_cycle_start, diagnostics=diagnostics)
                    serial_start = time.monotonic()
                    raw = reader.read_wrench()
                    termination.observe(raw=raw)
                    raw_error = raw_safety_reason(raw, config['policy'])
                    if raw_error:
                        raise RobotError(raw_error)
                    serial_end = time.monotonic()
                    # Read TCP AFTER potentially blocking serial I/O. Both host
                    # acquisition intervals are kept; this is not hard sync.
                    tcp_start = time.monotonic()
                    robot = controller.read_state()
                    controller.poll_stop()
                    speed_diagnostics = dict(controller.speed_guard_diagnostics)
                    tcp_end = time.monotonic()
                    now = tcp_end
                    validate_cycle_timing(policy.c, cycle_start, now, serial_start, previous_cycle_start, diagnostics=diagnostics)
                    timing.update(serial_read_start=serial_start, serial_read_end=serial_end,
                                  tcp_read_start=tcp_start, tcp_read_end=tcp_end,
                                  wrench_host_age_sec=now-serial_start, tcp_host_age_sec=now-robot.timestamp,
                                  timing_source='host_read_intervals; PX6D sample timestamp unavailable')
                    timing.update({k:v for k,v in controller.observation_timing.items() if k in TIMING_FIELDS})
                else:
                    sample = session.step()
                    robot, raw, processed, command, now = sample.robot, sample.raw, sample.processed, sample.command, sample.time
                    timing.update(serial_read_start=now, serial_read_end=now, tcp_read_start=now, tcp_read_end=now,
                                  rtde_device_timestamp='', rtde_packet_stagnation_sec=0,
                                  wrench_host_age_sec=0, tcp_host_age_sec=0, timing_source='synthetic common clock')
                last_wrench_observed = timing['serial_read_end']
                timing['cycle_start_time'] = cycle_start if args.execute else now
                timing['loop_start_interval_sec'] = ('' if previous_cycle_start is None else cycle_start-previous_cycle_start) if args.execute else dt
                termination.observe(raw=raw, robot=robot, phase="WRENCH_PROCESSING")
                if args.execute:
                    preprocessor.set_tool_orientation(robot.pose[3:])
                    # processed.force is filtered_force_base; no policy-local rotation.
                    processed = preprocessor.process(raw)
                    termination.observe(processed=processed, timestamp=now, phase="POLICY_UPDATE",
                        processed_force_frame=preprocessor.force_transform_status['output_frame'])
                    # Policy STOP is terminal for its algorithm, but the runtime
                    # keeps all hard force checks alive throughout braking.
                    if policy.state == State.STOP:
                        if not np.isfinite(np.r_[raw.array(), processed.array()]).all():
                            raise RobotError('nonfinite stopping wrench')
                        force_error = continuous_force_reason(raw, processed, config['policy'], diagnostics)
                        if force_error:
                            raise RobotError(force_error)
                    command = policy.update(now, raw, processed, robot,
                                            execution_settled=controller.standstill_confirmed)
                    if command.move and policy.state == State.TARGET_SEARCH:
                        predicted = robot.pose[:2] + command.direction_xy*command.speed*dt
                        try:
                            check_continuous_xy(controller.config, predicted)
                        except RobotError:
                            # An oblique edge or a tighter workspace can exhaust
                            # perpendicular clearance before the ray budget.
                            policy.request_stop(now, robot.pose, 'search execution boundary reached',
                                code=TerminationReason.STOP_SEARCH_LIMIT, event='BUDGET_STOP')
                            command = policy.update(now, raw, processed, robot)
                    termination.observe(command=command, phase='COMMAND_EXECUTION')
                    controller.set_continuous_phase(policy.state.value, stop_confirmed=policy.stop_confirmed)
                if args.execute:
                    # Keep timing and writer observations as diagnostics.
                    # Zero commands (especially first contact) take priority over
                    # watchdog/logging diagnostics. set_continuous_phase above
                    # retains the execution owner's stop/resume interlock.
                    early_stop = not command.move and (not controller.standstill_confirmed or
                        policy.state in (State.STOP, State.FIRST_CONTACT, State.DIRECTION_RECONFIRM, State.CONTACT_LOST))
                    if early_stop:
                        timing['command_send_time'] = time.monotonic()
                        controller.request_stop(nonblocking=True)
                        timing['command_return_time'] = time.monotonic()
                    logger.check_health()
                    validate_cycle_timing(policy.c, cycle_start, time.monotonic(), serial_start, previous_cycle_start, diagnostics=diagnostics)
                    # Only an explicitly enabled watchdog participates here.
                    if controller.config.get('continuous_require_watchdog') and controller.watchdog_active:
                        if controller._watchdog_last_kick is not None:
                            controller._check_watchdog_health()
                        controller.kick_watchdog()
                        validate_cycle_timing(policy.c, cycle_start, time.monotonic(), serial_start, previous_cycle_start, diagnostics=diagnostics)
                        logger.check_health()
                    if not early_stop:
                        timing['command_send_time'] = time.monotonic()
                    if command.move:
                        try:
                            controller.command_planar_velocity(command.direction_xy, command.speed, dt)
                        except SearchLimitReached as exc:
                            # The execution read is newer than the policy sample.
                            # Exhaustion between those reads is still a normal stop.
                            policy.request_stop(now, robot.pose, str(exc),
                                code=TerminationReason.STOP_SEARCH_LIMIT, event='BUDGET_STOP')
                            command = policy.update(now, raw, processed, robot)
                            controller.set_continuous_phase(policy.state.value)
                            controller.request_stop(nonblocking=True)
                    if not early_stop:
                        timing['command_return_time'] = time.monotonic()
                    validate_cycle_timing(policy.c, cycle_start, time.monotonic(), serial_start, previous_cycle_start, diagnostics=diagnostics)
                    timing.update(runtime_stop_state=controller.stop_state,
                                  actual_xyz_speed_mps=float(np.linalg.norm(robot.tcp_speed[:3])),
                                  stop_api_anomaly=bool(controller.stop_report and controller.stop_report['api_anomaly']),
                                  software_warnings=json.dumps(software_warnings(), ensure_ascii=False))
                    # The last actual SDK call is separate from the policy output.
                    # An unchanged sequence means no new speedL call this cycle.
                    sent = getattr(controller, 'last_speedl', {})
                    velocity = sent.get('velocity', ['', ''])
                    timing.update(speedl_sequence=sent.get('sequence', ''),
                        speedl_kind=sent.get('kind', ''), speedl_host_monotonic=sent.get('host_monotonic', ''),
                        speedl_vx=velocity[0], speedl_vy=velocity[1], speedl_accepted=sent.get('accepted', ''),
                        processed_force_frame=config.get('force_display', {}).get('frame', 'Base'))
                else:
                    timing['command_send_time'] = now
                    timing['command_return_time'] = controller.time
                # Real runs enqueue snapshots; the disk worker never calls devices.
                logger.log_sample(now, raw, processed, robot, command, policy.contact_direction,
                                  policy.tangent, extra={**preprocessor.force_log_fields, **policy.telemetry(command), **timing,
                                      **(speed_diagnostics if args.execute else {}),
                                      **({'sim_components_available': 0, 'physical_force_available': 0} if args.execute else sample.simulation_telemetry)})
                while policy.events:
                    if args.execute and policy.events[0].event_type == 'FIRST_CONTACT':
                        print('FIRST CONTACT CONFIRMED\n'
                              f'Fxy = {policy.fxy:.6f} N\n'
                              f'contact normal n = {policy.contact_direction.tolist()}\n'
                              f'tangent t = {policy.tangent.tolist()}\n'
                              f'follow hand = {policy.follow_hand}\n'
                              f'force_direction_sign = {policy.c["force_direction_sign"]}', flush=True)
                    logger.log_waypoint(policy.events[0])
                    # If a later enqueue fails, finalization writes only the
                    # unaccepted suffix, without duplicating persisted events.
                    del policy.events[0]
                if args.execute:
                    logger.check_health()
                    validate_cycle_timing(policy.c, cycle_start, time.monotonic(), serial_start, previous_cycle_start, diagnostics=diagnostics)
                    previous_cycle_start = cycle_start
                    time.sleep(max(0, dt - (time.monotonic() - cycle_start)))
    except KeyboardInterrupt as exc:
        interrupted_at = time.monotonic() if args.execute else now
        if controller is not None and args.execute:
            stop_after_exception(controller, termination)
        termination.observe(timestamp=interrupted_at, wrench_is_last_valid_sample=True,
            last_valid_wrench_host_monotonic=last_wrench_observed,
            last_valid_wrench_age_sec=None if last_wrench_observed is None else max(0., interrupted_at-last_wrench_observed))
        # An interrupted request is also an uncertain exchange, even if the
        # operator interrupt (rather than a timeout) initiated the stop.
        sensor_failed = bool(getattr(reader, '_requires_resynchronization', False))
        if sensor_failed:
            termination.observe(sensor_diagnostics=getattr(exc, 'sensor_diagnostics', None))
        detail = str(exc) or 'Ctrl+C'
        termination.set_stop_reason(TerminationReason.STOP_USER_REQUEST, detail, source="continuous.runner")
        if policy is not None:
            policy.request_stop(now, np.zeros(6) if robot is None else robot.pose,
                                detail, event="USER_STOP", code=TerminationReason.STOP_USER_REQUEST)
    except Exception as exc:
        failure_time = time.monotonic() if args.execute else now
        # Stop before any error logging or cleanup, including logger failures.
        if controller is not None and args.execute:
            stop_after_exception(controller, termination)
            if getattr(controller, 'connection_failed', False):
                snapshot = controller.connection_diagnostics
                termination.observe(phase='ROBOT_CONNECT', robot_connection=snapshot,
                                    tcp_pose=snapshot.get('tcp_pose'), tcp_speed=snapshot.get('tcp_speed'))
                print('Robot connection failure snapshot (freshness not verified):\n'+
                      json.dumps(snapshot, ensure_ascii=False, indent=2), flush=True)
        sensor_failed = isinstance(exc, PX6DError)
        termination.observe(timestamp=failure_time, failure_host_monotonic=failure_time,
            last_valid_wrench_host_monotonic=last_wrench_observed,
            last_valid_wrench_age_sec=None if last_wrench_observed is None else max(0., failure_time-last_wrench_observed),
            wrench_is_last_valid_sample=True)
        if sensor_failed:
            # Preserve the failed request before cleanup. No extra sensor read,
            # recovery or disk write may delay the stop requested above.
            termination.observe(sensor_diagnostics=getattr(exc, 'sensor_diagnostics', None))
            if policy is not None and robot is not None:
                policy._event(failure_time, robot.pose, 'SENSOR_FAILURE')
        speed_limit_observation = getattr(exc, 'speed_limit_observation', None)
        if speed_limit_observation is not None:
            # Keep the rejected read separate from the last accepted state and
            # the later post-stop observation. Stop has already been requested.
            termination.observe(speed_limit_observation=speed_limit_observation)
        termination.set_stop_reason(detail=str(exc), exception=exc, source="continuous.runner")
        if policy is not None:
            policy.request_stop(failure_time, np.zeros(6) if robot is None else robot.pose, str(exc))
            if getattr(controller, 'stop_state', None) == 'FAILED':
                policy.stop_reason, policy.reason = TerminationReason.STOP_MOTION_ERROR, str(exc)
        print(f"Continuous tracking stopped: {type(exc).__name__}: {exc}", flush=True)
        result = 1
    finally:
        # Always stop before diagnostics, disk writes or resource disconnection.
        if (controller is not None and args.execute and not robot_connect_started
                and getattr(controller, 'receive', None) is None
                and getattr(controller, 'control', None) is None):
            # Sensor startup may fail before any robot connection or command.
            # There is no RTDE observation to make and no physical stop claim.
            stop_snapshot_info = dict(source='not_connected', standstill_confirmed=False,
                motion_status='no_motion_commanded_by_this_task',
                runtime_stop_state='NOT_CONNECTED', stop_requests=[])
        elif (controller is not None and args.execute and getattr(controller, 'connection_failed', False)
                and getattr(controller, 'receive', None) is None):
            # connect() already closed its partially initialized interfaces.
            # Preserve the root error and do not pretend the cached pose proves standstill.
            stop_snapshot_info = dict(source='connection_failed', standstill_confirmed=False,
                                      robot_connection=controller.connection_diagnostics)
        elif controller is not None:
            try:
                if args.execute:
                    controller.request_stop(nonblocking=True)
                    # A timed-out PX6D reply is untagged: do not issue another
                    # request during braking. RTDE standstill monitoring remains.
                    monitoring = not sensor_failed
                    def observe_terminal_stop():
                        nonlocal monitoring, result
                        # Continue device observations through the existing hold.
                        # The optional watchdog is separate from software warnings.
                        if monitoring and controller.control is not None and not controller.motion_fault:
                            try:
                                started = time.monotonic()
                                sample = reader.read_wrench()
                                state = controller.read_state()
                                preprocessor.set_tool_orientation(state.pose[3:])
                                wrench = preprocessor.process(sample)
                                if not np.isfinite(np.r_[sample.array(), wrench.array()]).all():
                                    raise RobotError('nonfinite stopping wrench')
                                force_error = continuous_force_reason(sample, wrench, config['policy'], diagnostics)
                                if force_error:
                                    raise RobotError(force_error)
                                logger.check_health()
                                validate_cycle_timing(config['continuous_tracking'], started, time.monotonic(), started,
                                                      diagnostics=diagnostics)
                                if controller.config.get('continuous_require_watchdog') and controller.watchdog_active:
                                    controller._check_watchdog_health()
                                    controller.kick_watchdog()
                                return state
                            except Exception as exc:
                                monitoring, result = False, 1
                                termination.set_stop_reason(detail=str(exc), exception=exc,
                                    source='continuous.stop_monitor', terminal=False)
                        return controller.read_diagnostic_state()
                    # Normal terminal stops already completed in the main loop.
                    # Exceptional cleanup drives the same one-tick monitor with
                    # independent observation cycles, never inside a scan tick.
                    while controller.stop_state != 'STOPPED':
                        started = time.monotonic()
                        robot = observe_terminal_stop()
                        if controller.poll_stop():
                            break
                        time.sleep(max(0., dt-(time.monotonic()-started)))
                    if policy is not None and robot is not None:
                        confirmed_at = (controller.stop_report or {}).get('confirmed_host_monotonic', robot.timestamp)
                        policy._event(confirmed_at, robot.pose, 'EXECUTION_STOP_CONFIRMED')
                    controller.finish_control_script_if_stopped()
                    stop_snapshot_info = dict(source='fresh_rtde_actual_speed', standstill_confirmed=True,
                        runtime_stop_state=controller.stop_state,
                        stop_requests=controller.stop_history, **controller.observation_timing)
                else:
                    # Integrate the execution adapter's actual braking tail.
                    for _ in range(int(float(config['continuous_tracking']['confirmation_timeout_sec'])/dt)+1):
                        controller.apply_command([0, 0], 0, False)
                        robot = controller.read_state()
                        if np.linalg.norm(robot.tcp_speed[:3]) <= float(config['continuous_tracking']['settle_speed_mps']):
                            break
                    stop_snapshot_info = dict(source='post_stop_simulation', host_age_sec=0,
                                              standstill_confirmed=bool(np.linalg.norm(robot.tcp_speed[:3]) <= float(config['continuous_tracking']['settle_speed_mps'])))
            except Exception as exc:
                result = 1
                stop_snapshot_info.update(source='last_valid_sample', standstill_confirmed=False,
                                          read_error=str(exc), host_age_sec=None if robot is None else max(0., time.monotonic()-robot.timestamp))
                if args.execute:
                    stop_snapshot_info.update(runtime_stop_state=controller.stop_state,
                                              stop_requests=controller.stop_history)
                termination.set_stop_reason(detail=str(exc), exception=exc, source='continuous.stop_observation', terminal=False)
        if args.execute:
            termination.observe(wrench_is_last_valid_sample=True,
                last_valid_wrench_host_monotonic=last_wrench_observed,
                last_valid_wrench_age_sec=None if last_wrench_observed is None else max(0., time.monotonic()-last_wrench_observed))
        if policy is not None and policy.state == State.STOP:
            termination.set_stop_reason(policy.stop_reason, policy.reason, source="continuous.policy")
            if policy.stop_reason not in (TerminationReason.STOP_USER_REQUEST, TerminationReason.STOP_TIME_LIMIT,
                                          TerminationReason.STOP_SEARCH_LIMIT):
                result = 1
        for resource in (controller, reader):
            if resource is not None and hasattr(resource, "close"):
                try:
                    resource.close()
                except Exception as exc:
                    result = 1
                    termination.set_stop_reason(detail=str(exc), exception=exc, source="continuous.cleanup")
        if logger is not None:
            try:
                if policy is not None:
                    final_writes = []
                    if args.execute:
                        final_writes.append((logger.write_final_waypoints, (policy.events,), {}))
                    else:
                        final_writes.extend((logger.log_waypoint, (event,), {}) for event in policy.events)
                    if robot is not None:
                        final_writes.append((logger.write_stop_snapshot,
                            (robot, raw, processed, policy.state.value), dict(extra=dict(
                                stop_observation=stop_snapshot_info, wrench_is_last_valid_sample=True,
                                wrench_age_sec=None if last_wrench_observed is None else max(
                                    0., (time.monotonic() if args.execute else controller.time)-last_wrench_observed)))))
                    final_writes.append((logger.write_summary,
                        (policy.state.value, policy.reason, int(policy.initial_contact is not None)), dict(
                            initial_contact=None if policy.initial_contact is None else policy.initial_contact.tolist(),
                            last_contact_pose=None if policy.last_contact_pose is None else policy.last_contact_pose.tolist(),
                            mode="real" if args.execute else "simulation", stop_observation=stop_snapshot_info,
                            first_threshold_pose=None if policy.first_threshold_pose is None else policy.first_threshold_pose.tolist(),
                            provenance=config.get('continuous_provenance'), software_warnings=software_warnings())))
                    for write, positional, keywords in final_writes:
                        try:
                            write(*positional, **keywords)
                        except Exception as exc:
                            result = 1
                            termination.set_stop_reason(detail=str(exc), exception=exc, source="continuous.final_logging")
            except Exception as exc:
                result = 1
                termination.set_stop_reason(detail=str(exc), exception=exc, source="continuous.final_logging")
            finally:
                try:
                    logger.close()
                except Exception as exc:
                    result = 1
                    termination.set_stop_reason(detail=str(exc), exception=exc, source="continuous.logger_close")
        if args.execute:
            warnings = software_warnings()
            termination.observe(software_warnings=warnings)
            if any(any(component.values()) for component in warnings.values()):
                print('WARNING / diagnostics: '+json.dumps(warnings, ensure_ascii=False), flush=True)
        termination.flush(emit=True)
        # Devices and log files are closed. Plotting never runs in a control tick.
        if logger is not None and policy is not None and policy.initial_contact is not None:
            try:
                from tools.visualize_continuous_run import render_contact_trajectory
                for path in render_contact_trajectory(logger.run_dir):
                    print(f'TCP 轨迹图：{path}', flush=True)
            except Exception as exc:
                print(f'轨迹图生成失败，可离线回放重试：{type(exc).__name__}: {exc}', flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config.yaml")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true", help="real UR7e + PX6D; requires P0 and Enter confirmation")
    mode.add_argument("--preview", action="store_true", help="interactive offline SIMULATION / SYNTHETIC FORCE")
    mode.add_argument("--dry-run", action="store_true", help="offline simulation (default)")
    parser.add_argument("--scene", type=Path, default=ROOT / "simulation/scene_continuous.yaml")
    parser.add_argument("--duration", type=float, help="simulation/preview duration; ignored during real execution")
    parser.add_argument("--output", type=Path)
    parser.add_argument('--enable-reacquire', action='store_true', help='offline experimental arc recovery only')
    parser.add_argument('--site-digest', action='store_true', help='print current config binding hash; no devices')
    parser.add_argument('--check-calibration', action='store_true', help='read and validate saved P0/P1 and sandbox corners; no devices')
    args = parser.parse_args()
    if args.check_calibration:
        config=load_config(args.config)
        prepare_real(config,args.config)
        print(json.dumps(calibration_summary(config),ensure_ascii=False,indent=2))
        print('Offline calibration check only; no device connection or motion.')
        return 0
    if args.site_digest:
        print(site_configuration_digest(load_config(args.config), args.config))
        return 0
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
