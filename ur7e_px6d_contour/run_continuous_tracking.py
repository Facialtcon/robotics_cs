#!/usr/bin/env python3
"""Independent experimental runner. Defaults to entirely offline simulation."""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from copy import deepcopy
from pathlib import Path
import time

import numpy as np
import yaml

from app.operator_input import OperatorKeyboard
from calibration.scan_calibration import (load_scan_calibration, resolve_calibration_path,
                                          validate_calibration_constraints)
from config.loader import load_config, runtime_robot_config
from experiment_logging.data_logger import ExperimentLogger
from experiment_logging.termination import TerminationReason, TerminationRecorder
from policy.continuous_tracking import ContinuousTrackingPolicy, EXTRA_SAMPLE_FIELDS, State
from robot.rtde_controller import URRTDEController, RobotError, validate_execution_configuration
from robot.tcp_identity import tcp_offsets_match
from safety.force_guard import force_safety_reason
from sensor.force_preprocess import WrenchPreprocessor
from sensor.px6d_reader import PX6DReader
from simulation.continuous_session import SIMULATION_SAMPLE_FIELDS

ROOT = Path(__file__).resolve().parent
TIMING_FIELDS = ('serial_read_start', 'serial_read_end', 'tcp_read_start', 'tcp_read_end',
                 'rtde_device_timestamp', 'rtde_packet_stagnation_sec', 'wrench_host_age_sec',
                 'tcp_host_age_sec', 'command_send_time', 'command_return_time', 'timing_source',
                 'cycle_start_time', 'loop_start_interval_sec')


def site_configuration_digest(config, config_path=ROOT / "config.yaml"):
    c = {k:v for k,v in config['continuous_tracking'].items() if k != 'site_verification'}
    payload = dict(tcp=config['tcp'], preprocessing=config['preprocessing'], continuous=c,
                   robot=config['robot'], workspace=config['workspace'], policy=config['policy'],
                   calibration=config['calibration'], calibration_file_sha256=hashlib.sha256(
                       resolve_calibration_path(config_path, config['calibration']['file']).read_bytes()).hexdigest())
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()


def git_provenance():
    def read(*args):
        return subprocess.check_output(['git', *args], cwd=ROOT, text=True).strip()
    try:
        return dict(branch=read('branch', '--show-current'), commit=read('rev-parse', 'HEAD'),
                    dirty=bool(read('status', '--porcelain')), status=read('status', '--short'))
    except Exception as exc:
        return dict(unavailable=str(exc))


def validate_cycle_timing(c, start, now, oldest_observation, previous_start=None):
    values = [start, now, oldest_observation] + ([] if previous_start is None else [previous_start])
    if not np.all(np.isfinite(values)) or now < start or oldest_observation > now:
        raise RobotError('invalid host observation time')
    if previous_start is not None and not 0 < start-previous_start <= float(c['max_sample_gap_sec']):
        raise RobotError('control cycle gap exceeded')
    if now-start > float(c['cycle_timeout_sec']):
        raise RobotError(f'control cycle timeout: {now-start:.6f} s (including device/log latency)')
    if now-oldest_observation > float(c['max_observation_age_sec']):
        raise RobotError('stale host observation; no new motion permitted')



def check_start(controller, start, config):
    robot = controller.read_state()
    if np.linalg.norm(robot.pose[:3] - start[:3]) > float(config["policy"]["position_tolerance"]):
        raise RobotError("start TCP is not at calibrated P0; position it before this experiment")
    if np.linalg.norm(robot.tcp_speed) > 1e-4:
        raise RobotError("robot must be stationary at P0 before starting")
    return robot


def stop_after_exception(controller, termination):
    """A failed stop attempt must not skip disconnect/diagnostic cleanup."""
    try:
        controller.stop()
    except Exception as exc:
        termination.set_stop_reason(detail=str(exc), exception=exc,
                                    source="continuous.stop_failure", terminal=False)


def prepare_real(config, config_path):
    """Use the same calibration/TCP binding as the discrete entry point."""
    budget = config['continuous_tracking']['max_runtime_sec']
    if budget is None or isinstance(budget, bool) or not np.isfinite(float(budget)) or float(budget) <= 0:
        raise RobotError('real continuous_tracking.max_runtime_sec must be finite and positive')
    validate_execution_configuration(config)
    c = config['continuous_tracking']
    verification = c.get('site_verification')
    if not isinstance(verification, dict):
        raise RobotError('continuous site verification is missing; refusing device connection')
    for key in ('force_sign_checked', 'base_transform_checked', 'watchdog_stop_verified'):
        if verification.get(key) is not True:
            raise RobotError(f'site verification missing: {key}')
    if not verification.get('operator') or not verification.get('checked_at'):
        raise RobotError('site verification must identify operator and time')
    if verification.get('configuration_sha256') != site_configuration_digest(config, config_path):
        raise RobotError('site verification does not match current tool/frame/sign/configuration')
    if c['reacquire_enabled'] and verification.get('recovery_verified') is not True:
        raise RobotError('real recovery requires separate site verification')
    bounds = config['workspace']['limits'] if config['workspace'].get('enabled', True) else c.get('real_test_xy_limits')
    if not isinstance(bounds, dict):
        raise RobotError('finite site XY test bounds required when workspace is disabled')
    margin = float(c['boundary_margin'])
    for axis in 'xy':
        low, high = float(bounds[f'{axis}_min']), float(bounds[f'{axis}_max'])
        if not np.isfinite([low, high]).all() or high-low <= 2*margin:
            raise RobotError('invalid continuous site XY bounds')
    watchdog_contract = URRTDEController.verified_watchdog_contract()
    max_command = max(float(c['search_speed']), float(c['reacquire_speed']),
                      np.hypot(float(c['tangential_speed']), float(c['normal_speed_limit'])))
    required_margin = max_command / float(c['watchdog_frequency_hz']) + max_command**2 / (2*float(config['robot']['stop_deceleration']))
    if margin < required_margin or 1/float(c['watchdog_frequency_hz']) <= float(c['cycle_timeout_sec']):
        raise RobotError('stop margin/watchdog deadline incompatible with execution budgets')
    config['continuous_sdk_contract'] = watchdog_contract
    config['continuous_reviewed_configuration_sha256'] = site_configuration_digest(config, config_path)
    calibration = load_scan_calibration(resolve_calibration_path(config_path, config["calibration"]["file"]),
                                        require_tcp_offset=True)
    if calibration["robot_ip"] != config["robot"]["robot_ip"]:
        raise RobotError("scan calibration robot_ip does not match config")
    if not tcp_offsets_match(calibration["active_tcp_offset"], config["tcp"]["offset"],
                             float(config["tcp"]["offset_tolerance"])):
        raise RobotError("scan calibration TCP does not match configured TCP")
    validate_calibration_constraints(calibration, float(config["calibration"]["min_direction_calibration_distance"]),
                                     float(config["calibration"]["max_direction_calibration_z_difference"]))
    config["policy"]["search_direction_xy"] = calibration["scan_direction_xy"]
    robot_config = runtime_robot_config(config)
    robot_config.update(fixed_z=calibration["fixed_z"], fixed_orientation=calibration["fixed_orientation"],
                        continuous_require_watchdog=True, continuous_sample_age_sec=float(c['max_observation_age_sec']),
                        continuous_settle_speed_mps=float(c['settle_speed_mps']), continuous_xy_limits=dict(bounds),
                        continuous_speed_limit=max_command,
                        continuous_boundary_margin=margin)
    config["continuous_calibration"] = calibration
    # Save calibrated container for offline visualization only, not motion authorization.
    path = ROOT / "workspace/config/workspace_calibration.yaml"
    if path.is_file():
        with path.open(encoding="utf-8") as handle:
            config["continuous_workspace_calibration"] = yaml.safe_load(handle)
    return robot_config, np.asarray(calibration["start_tcp_pose"])


def run(args):
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
    controller = reader = logger = policy = None
    robot = raw = processed = None
    termination = TerminationRecorder(output_root=ROOT / "data")
    result = 0
    now = 0.0
    config = None
    stop_snapshot_info = dict(source='unavailable', standstill_confirmed=False)
    previous_cycle_start = None
    last_wrench_observed = None
    timing = dict.fromkeys(TIMING_FIELDS, '')
    try:
        config = deepcopy(load_config(args.config))
        config['continuous_provenance'] = git_provenance()
        # Validate real attestation against the reviewed config before optional
        # CLI duration shortening; never silently authorize recovery in real mode.
        if args.execute:
            if getattr(args, 'enable_reacquire', False):
                raise RobotError('--enable-reacquire is offline only')
            robot_config, start = prepare_real(config, args.config)
        elif getattr(args, 'enable_reacquire', False):
            config['continuous_tracking']['reacquire_enabled'] = True
        if args.duration is not None:
            if not np.isfinite(args.duration) or args.duration <= 0:
                raise ValueError("--duration must be finite and positive")
            # CLI may shorten the reviewed configuration budget, never raise it.
            budget = config["continuous_tracking"]["max_runtime_sec"]
            config["continuous_tracking"]["max_runtime_sec"] = (
                args.duration if budget is None else min(args.duration, float(budget)))
        dt = 1 / float(config["policy"]["control_rate_hz"])
        if args.execute:
            # Existing site direction checks do not identify the physical object
            # on which the measured force acts. Record that distinction explicitly;
            # offline display must not guess a physical sign from targetward control.
            config['force_display']=dict(schema_version=1,frame='Base',force_source='processed_wrench',
                force_convention='unconfirmed',estimate_method='quasistatic_planar_balance',
                physical_sign_confirmed=False,base_frame_confirmed=False,physical_available=False)
            policy = ContinuousTrackingPolicy(config)  # validate before any connection
            controller = URRTDEController(robot_config)
            s = config["sensor"]
            reader = PX6DReader(s["serial_port"], s["baudrate"], s["timeout_sec"],
                                s["poll_rate_hz"], s["startup_delay_sec"])
            preprocessor = WrenchPreprocessor.from_config(config["preprocessing"])
        else:
            from simulation.simulator import load_simulation_config
            from simulation.continuous_session import SimulationSession
            session = SimulationSession(config, load_simulation_config(args.scene))
            config = session.config
            policy, controller, reader = session.policy, session.robot, session.sensor
            preprocessor = session.preprocessor
        output = args.output or resolve_calibration_path(args.config, config["logging"]["output_root"])
        # Existing optional workspace exporter renders on close and assumes
        # real Base coordinates; use only our explicit offline replay here.
        logger = ExperimentLogger(output, config, extra_sample_fields=EXTRA_SAMPLE_FIELDS + TIMING_FIELDS + SIMULATION_SAMPLE_FIELDS,
                                  workspace_logging=False)
        logger.termination = termination.bind(logger.run_dir)
        termination.observe(policy=policy, processed_force_frame="Base")
        print(f"Continuous run: {logger.run_dir}", flush=True)
        if args.execute:
            reader.connect()
            controller.connect()  # fixed Z/orientation, workspace, active TCP checks
            controller.safe_stop_motion(force=True)
            robot = check_start(controller, start, config)
        else:
            robot = controller.read_state()
        with OperatorKeyboard() as keyboard:
            baseline = config["preprocessing"].get("baseline", {"capture_on_start": True, "sample_count": 100})
            if not args.execute:
                session.capture_bias(keyboard.poll, termination.observe)
            elif baseline.get("capture_on_start", True):
                samples = []
                for _ in range(int(baseline["sample_count"])):
                    if keyboard.poll() in ("Q", "ESC"):
                        raise KeyboardInterrupt
                    raw = reader.read_wrench() if args.execute else reader.read_wrench(robot.pose[:2], np.zeros(2))[0]
                    last_wrench_observed = time.monotonic() if args.execute else controller.time
                    termination.observe(raw=raw, phase="SENSOR_BIAS")
                    if not np.all(np.isfinite(raw.array())):
                        raise ValueError("nonfinite bias sample")
                    processed = preprocessor.process(raw)
                    reason = force_safety_reason(raw, processed, config["policy"])
                    if reason:
                        raise RobotError(reason)
                    if args.execute:
                        current = controller.read_state()
                        if np.linalg.norm(current.pose[:3] - robot.pose[:3]) > float(config["policy"]["position_tolerance"]):
                            raise RobotError("robot moved during bias capture")
                        time.sleep(1 / float(config["sensor"]["poll_rate_hz"]))
                    samples.append(raw)
                preprocessor.set_zero_bias(samples)
        if args.execute:
            print(f"P0: {start.tolist()}\nSearch P0 -> P1: {policy.search_direction.tolist()}")
            print(f"MUST CONFIRM ON SITE: force sign={policy.c['force_direction_sign']}, "
                  f"F_ref={policy.c['force_reference']} N, workspace enabled={config['workspace'].get('enabled', True)}")
            print("Q / ESC / Ctrl+C: stop in place. No automatic return for this experiment.")
            if input("已核对空载零偏、Base 力方向、P0、现场及急停，输入 START：").strip() != "START":
                policy.request_stop(now, robot.pose, "START not confirmed", event="USER_STOP",
                                    code=TerminationReason.STOP_USER_REQUEST)
            else:
                # The operator may have moved the robot while at the prompt.
                robot = check_start(controller, start, config)
                # START/bias are stationary and precede watchdog activation.
                # All control calls remain on this one thread.
                controller.enable_watchdog(float(policy.c['watchdog_frequency_hz']))
        with OperatorKeyboard() as keyboard:
            while policy.state != State.STOP:
                cycle_start = time.monotonic()
                now = cycle_start if args.execute else controller.time
                key = keyboard.poll()
                if key in ("Q", "ESC"):
                    policy.request_stop(now, robot.pose, f"{key} operator stop", event="USER_STOP",
                                        code=TerminationReason.STOP_USER_REQUEST)
                    break
                termination.observe(phase="SENSOR_READ")
                if args.execute:
                    validate_cycle_timing(policy.c, cycle_start, time.monotonic(), cycle_start, previous_cycle_start)
                    serial_start = time.monotonic()
                    raw = reader.read_wrench()
                    serial_end = time.monotonic()
                    # Read TCP AFTER potentially blocking serial I/O. Both host
                    # acquisition intervals are kept; this is not hard sync.
                    tcp_start = time.monotonic()
                    robot = controller.read_state()
                    tcp_end = time.monotonic()
                    now = tcp_end
                    validate_cycle_timing(policy.c, cycle_start, now, serial_start, previous_cycle_start)
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
                    command = policy.update(now, raw, processed, robot)
                if args.execute:
                    # Health validation immediately before feeding the robot-side
                    # watchdog. Disk or device stalls prevent the next kick.
                    validate_cycle_timing(policy.c, cycle_start, time.monotonic(), serial_start, previous_cycle_start)
                    # DIRECTION_RECONFIRM is nonterminal: keep the watchdog
                    # armed while issuing stop and collecting fresh feedback.
                    if policy.state != State.STOP:
                        controller.kick_watchdog()
                    timing['command_send_time'] = time.monotonic()
                    if command.move:
                        controller.command_planar_velocity(command.direction_xy, command.speed, dt)
                    else:
                        controller.stop()
                    timing['command_return_time'] = time.monotonic()
                    validate_cycle_timing(policy.c, cycle_start, time.monotonic(), serial_start, previous_cycle_start)
                else:
                    timing['command_send_time'] = now
                    timing['command_return_time'] = controller.time
                # Motion/stop precedes disk writes. No rendering in this loop.
                logger.log_sample(now, raw, processed, robot, command, policy.contact_direction,
                                  policy.tangent, extra={**policy.telemetry(command), **timing,
                                      **({'sim_components_available': 0, 'physical_force_available': 0} if args.execute else sample.simulation_telemetry)})
                for event in policy.events:
                    logger.log_waypoint(event)
                policy.events.clear()
                if args.execute:
                    # A logging stall cannot be followed by another motion/kick.
                    validate_cycle_timing(policy.c, cycle_start, time.monotonic(), serial_start, previous_cycle_start)
                    previous_cycle_start = cycle_start
                    time.sleep(max(0, dt - (time.monotonic() - cycle_start)))
    except KeyboardInterrupt:
        if controller is not None and args.execute:
            stop_after_exception(controller, termination)
        termination.set_stop_reason(TerminationReason.STOP_USER_REQUEST, "Ctrl+C", source="continuous.runner")
        if policy is not None:
            policy.request_stop(now, np.zeros(6) if robot is None else robot.pose,
                                "Ctrl+C", event="USER_STOP", code=TerminationReason.STOP_USER_REQUEST)
    except Exception as exc:
        # Stop before any error logging or cleanup, including logger failures.
        if controller is not None and args.execute:
            stop_after_exception(controller, termination)
        termination.set_stop_reason(detail=str(exc), exception=exc, source="continuous.runner")
        if policy is not None:
            policy.request_stop(now, np.zeros(6) if robot is None else robot.pose, str(exc))
        print(f"Continuous tracking stopped: {type(exc).__name__}: {exc}", flush=True)
        result = 1
    finally:
        # Always stop before diagnostics, disk writes or resource disconnection.
        if controller is not None:
            try:
                if args.execute:
                    controller.stop()
                    deadline = time.monotonic() + float(config['continuous_tracking']['confirmation_timeout_sec'])
                    settled_since = None
                    while time.monotonic() < deadline:
                        current = controller.read_diagnostic_state()
                        observed = time.monotonic()
                        if observed-current.timestamp > float(config['continuous_tracking']['max_observation_age_sec']):
                            raise RobotError('stale post-stop observation')
                        robot = current
                        low_speed = np.linalg.norm(current.tcp_speed[:3]) <= float(config['continuous_tracking']['settle_speed_mps'])
                        settled_since = (observed if settled_since is None else settled_since) if low_speed else None
                        stop_snapshot_info = dict(source='post_stop_host_observation', host_age_sec=observed-current.timestamp,
                                                  standstill_confirmed=False, **controller.observation_timing)
                        if settled_since is not None and observed-settled_since >= float(config['continuous_tracking']['settle_hold_sec']):
                            stop_snapshot_info['standstill_confirmed'] = not bool(controller.motion_fault)
                            break
                        # No watchdog kicks during terminal cleanup. Robot-side
                        # timeout remains armed, including if a read blocks.
                        time.sleep(dt)
                    if not stop_snapshot_info.get('standstill_confirmed'):
                        raise RobotError('post-stop standstill could not be confirmed')
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
                termination.set_stop_reason(detail=str(exc), exception=exc, source='continuous.stop_observation', terminal=False)
        if policy is not None and policy.state == State.STOP:
            termination.set_stop_reason(policy.stop_reason, policy.reason, source="continuous.policy")
            if policy.stop_reason not in (TerminationReason.STOP_USER_REQUEST, TerminationReason.STOP_TIME_LIMIT):
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
                    for event in policy.events:
                        logger.log_waypoint(event)
                    if robot is not None:
                        logger.write_stop_snapshot(robot, raw, processed, policy.state.value,
                                                   extra=dict(stop_observation=stop_snapshot_info,
                                                              wrench_is_last_valid_sample=True,
                                                              wrench_age_sec=None if last_wrench_observed is None else max(0., (time.monotonic() if args.execute else controller.time)-last_wrench_observed)))
                    logger.write_summary(policy.state.value, policy.reason, int(policy.initial_contact is not None),
                                         initial_contact=None if policy.initial_contact is None else policy.initial_contact.tolist(),
                                         last_contact_pose=None if policy.last_contact_pose is None else policy.last_contact_pose.tolist(),
                                         mode="real" if args.execute else "simulation", stop_observation=stop_snapshot_info,
                                         first_threshold_pose=None if policy.first_threshold_pose is None else policy.first_threshold_pose.tolist(),
                                         provenance=config.get('continuous_provenance'))
            except Exception as exc:
                result = 1
                termination.set_stop_reason(detail=str(exc), exception=exc, source="continuous.final_logging")
            finally:
                try:
                    logger.close()
                except Exception as exc:
                    result = 1
                    termination.set_stop_reason(detail=str(exc), exception=exc, source="continuous.logger_close")
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
    parser.add_argument("--duration", type=float, help="finite preview duration (default: manual stop); shorten budget in other modes")
    parser.add_argument("--output", type=Path)
    parser.add_argument('--enable-reacquire', action='store_true', help='offline experimental arc recovery only')
    parser.add_argument('--site-digest', action='store_true', help='print current config binding hash; no devices')
    args = parser.parse_args()
    if args.site_digest:
        print(site_configuration_digest(load_config(args.config), args.config))
        return 0
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
