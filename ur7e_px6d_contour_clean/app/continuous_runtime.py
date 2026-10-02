#!/usr/bin/env python3
"""Independent experimental runner. Defaults to entirely offline simulation."""
from __future__ import annotations

import argparse
import json
from copy import deepcopy
from pathlib import Path
import time

import numpy as np

from app.operator_input import OperatorKeyboard
from config.loader import load_config
from experiment_logging.data_logger import ExperimentLogger
from experiment_logging.continuous_writer import ContinuousLogWriter
from experiment_logging.termination import TerminationReason, TerminationRecorder
from policy.continuous_tracking import ContinuousTrackingPolicy, EXTRA_SAMPLE_FIELDS, State
from robot.rtde_controller import URRTDEController, RobotError, SearchLimitReached, SPEED_GUARD_FIELDS, check_continuous_xy
from sensor.force_preprocess import WrenchPreprocessor
from safety.force_guard import continuous_force_reason, raw_safety_reason
from sensor.px6d_reader import PX6DReader
from simulation.continuous_session import SIMULATION_SAMPLE_FIELDS

ROOT = Path(__file__).resolve().parents[1]
TIMING_FIELDS = ('serial_read_start', 'serial_read_end', 'tcp_read_start', 'tcp_read_end',
                 'rtde_device_timestamp', 'rtde_packet_stagnation_sec', 'wrench_host_age_sec',
                 'tcp_host_age_sec', 'command_send_time', 'command_return_time', 'timing_source',
                 'cycle_start_time', 'loop_start_interval_sec', 'runtime_stop_state',
                 'actual_xyz_speed_mps', 'stop_api_anomaly', 'software_warnings')


from app.configuration import prepare_real, calibration_summary, site_configuration_digest
from app.scan_startup import (startup_scan, capture_stationary_bias,
                             startup_position_tolerance, check_startup_stationary)
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
    timing = dict.fromkeys(TIMING_FIELDS, '')
    diagnostics = {}
    def software_warnings():
        return dict(runtime=dict(diagnostics), controller=dict(getattr(controller, 'diagnostics', {})),
                    policy=dict(getattr(policy, 'diagnostics', {})), logging=getattr(logger, 'diagnostics', {}))
    try:
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
        if args.execute:
            # Existing site direction checks do not identify the physical object
            # on which the measured force acts. Record that distinction explicitly;
            # offline display must not guess a physical sign from targetward control.
            config['force_display']=dict(schema_version=1,frame='Base',force_source='processed_wrench',
                force_convention='unconfirmed',estimate_method='quasistatic_planar_balance',
                physical_sign_confirmed=False,base_frame_confirmed=False,physical_available=False)
            policy = ContinuousTrackingPolicy(config)  # validate before any connection
            controller = controller_factory(robot_config)
            s = config["sensor"]
            reader = reader_factory(s["serial_port"], s["baudrate"], s["timeout_sec"],
                                s["poll_rate_hz"], s["startup_delay_sec"])
            preprocessor = WrenchPreprocessor.from_config(config["preprocessing"])
        else:
            from simulation.simulator import load_simulation_config
            from simulation.continuous_session import SimulationSession
            session = SimulationSession(config, load_simulation_config(args.scene))
            config = session.config
            policy, controller, reader = session.policy, session.robot, session.sensor
            preprocessor = session.preprocessor
        output = args.output
        # Existing optional workspace exporter renders on close and assumes
        # real Base coordinates; use only our explicit offline replay here.
        logger = ExperimentLogger(output, config, mode="real" if args.execute else "simulation", strategy="continuous", config_source=args.config, extra_sample_fields=EXTRA_SAMPLE_FIELDS + TIMING_FIELDS + SIMULATION_SAMPLE_FIELDS + SPEED_GUARD_FIELDS,
                                  workspace_logging=False)
        logger.termination = termination.bind(logger.run_dir)
        if args.execute:
            # Disk work stays off the device thread. Backlog/write diagnostics
            # cannot end motion; the bounded queue records any dropped samples.
            logger = ContinuousLogWriter(logger, max_pending_sec=float(
                config['continuous_tracking']['confirmation_timeout_sec']), diagnostic_only=True)
        termination.observe(policy=policy, processed_force_frame="Base")
        print(f"Continuous run: {logger.run_dir}", flush=True)
        if args.execute:
            reader.connect()
            controller.connect(allow_start_away_from_fixed_pose=True)
        else:
            robot = controller.read_state()
        startup_return_done = False
        with OperatorKeyboard() as keyboard:
            if args.execute:
                startup_return_done = startup_scan(config, start, controller, reader, logger,
                                                          keyboard.poll, confirm=keyboard.read_line)
                robot = check_start(controller, start, config)
            baseline = config["preprocessing"].get("baseline", {"capture_on_start": True, "sample_count": 100})
            if not args.execute:
                session.capture_bias(keyboard.poll, termination.observe)
            elif baseline.get("capture_on_start", True):
                capture_stationary_bias(config, reader, preprocessor, controller,
                    baseline['sample_count'], poll=keyboard.poll, logger=logger)
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
                    processed = preprocessor.process(raw)
                    termination.observe(processed=processed, timestamp=now, phase="POLICY_UPDATE")
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
                    logger.check_health()
                    validate_cycle_timing(policy.c, cycle_start, time.monotonic(), serial_start, previous_cycle_start, diagnostics=diagnostics)
                    # Only an explicitly enabled watchdog participates here.
                    if controller.config.get('continuous_require_watchdog') and controller.watchdog_active:
                        if controller._watchdog_last_kick is not None:
                            controller._check_watchdog_health()
                        controller.kick_watchdog()
                        validate_cycle_timing(policy.c, cycle_start, time.monotonic(), serial_start, previous_cycle_start, diagnostics=diagnostics)
                        logger.check_health()
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
                    elif (not controller.standstill_confirmed or
                          policy.state in (State.STOP, State.FIRST_CONTACT, State.DIRECTION_RECONFIRM, State.CONTACT_LOST)):
                        controller.request_stop(nonblocking=True)
                    timing['command_return_time'] = time.monotonic()
                    validate_cycle_timing(policy.c, cycle_start, time.monotonic(), serial_start, previous_cycle_start, diagnostics=diagnostics)
                    timing.update(runtime_stop_state=controller.stop_state,
                                  actual_xyz_speed_mps=float(np.linalg.norm(robot.tcp_speed[:3])),
                                  stop_api_anomaly=bool(controller.stop_report and controller.stop_report['api_anomaly']),
                                  software_warnings=json.dumps(software_warnings(), ensure_ascii=False))
                else:
                    timing['command_send_time'] = now
                    timing['command_return_time'] = controller.time
                # Real runs enqueue snapshots; the disk worker never calls devices.
                logger.log_sample(now, raw, processed, robot, command, policy.contact_direction,
                                  policy.tangent, extra={**policy.telemetry(command), **timing,
                                      **(speed_diagnostics if args.execute else {}),
                                      **({'sim_components_available': 0, 'physical_force_available': 0} if args.execute else sample.simulation_telemetry)})
                while policy.events:
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
        if controller is not None and args.execute:
            stop_after_exception(controller, termination)
        detail = str(exc) or 'Ctrl+C'
        termination.set_stop_reason(TerminationReason.STOP_USER_REQUEST, detail, source="continuous.runner")
        if policy is not None:
            policy.request_stop(now, np.zeros(6) if robot is None else robot.pose,
                                detail, event="USER_STOP", code=TerminationReason.STOP_USER_REQUEST)
    except Exception as exc:
        # Stop before any error logging or cleanup, including logger failures.
        if controller is not None and args.execute:
            stop_after_exception(controller, termination)
            if getattr(controller, 'connection_failed', False):
                snapshot = controller.connection_diagnostics
                termination.observe(phase='ROBOT_CONNECT', robot_connection=snapshot,
                                    tcp_pose=snapshot.get('tcp_pose'), tcp_speed=snapshot.get('tcp_speed'))
                print('Robot connection failure snapshot (freshness not verified):\n'+
                      json.dumps(snapshot, ensure_ascii=False, indent=2), flush=True)
        speed_limit_observation = getattr(exc, 'speed_limit_observation', None)
        if speed_limit_observation is not None:
            # Keep the rejected read separate from the last accepted state and
            # the later post-stop observation. Stop has already been requested.
            termination.observe(speed_limit_observation=speed_limit_observation)
        termination.set_stop_reason(detail=str(exc), exception=exc, source="continuous.runner")
        if policy is not None:
            policy.request_stop(now, np.zeros(6) if robot is None else robot.pose, str(exc))
            if getattr(controller, 'stop_state', None) == 'FAILED':
                policy.stop_reason, policy.reason = TerminationReason.STOP_MOTION_ERROR, str(exc)
        print(f"Continuous tracking stopped: {type(exc).__name__}: {exc}", flush=True)
        result = 1
    finally:
        # Always stop before diagnostics, disk writes or resource disconnection.
        if (controller is not None and args.execute and getattr(controller, 'connection_failed', False)
                and getattr(controller, 'receive', None) is None):
            # connect() already closed its partially initialized interfaces.
            # Preserve the root error and do not pretend the cached pose proves standstill.
            stop_snapshot_info = dict(source='connection_failed', standstill_confirmed=False,
                                      robot_connection=controller.connection_diagnostics)
        elif controller is not None:
            try:
                if args.execute:
                    controller.request_stop(nonblocking=True)
                    monitoring = True
                    def observe_terminal_stop():
                        nonlocal monitoring, result
                        # Continue device observations through the existing hold.
                        # The optional watchdog is separate from software warnings.
                        if monitoring and controller.control is not None and not controller.motion_fault:
                            try:
                                started = time.monotonic()
                                sample = reader.read_wrench()
                                wrench = preprocessor.process(sample)
                                if not np.isfinite(np.r_[sample.array(), wrench.array()]).all():
                                    raise RobotError('nonfinite stopping wrench')
                                force_error = continuous_force_reason(sample, wrench, config['policy'], diagnostics)
                                if force_error:
                                    raise RobotError(force_error)
                                state = controller.read_state()
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
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config.yaml")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true", help="real UR7e + PX6D; requires P0 and START")
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
