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
from experiment_logging.continuous_writer import ContinuousLogWriter
from experiment_logging.termination import TerminationReason, TerminationRecorder
from policy.continuous_tracking import ContinuousTrackingPolicy, EXTRA_SAMPLE_FIELDS, State, validate_config
from robot.rtde_controller import (URRTDEController, RobotError, validate_execution_configuration,
                                   check_continuous_xy, _orientation_distance, SPEED_GUARD_FIELDS,
                                   CONTINUOUS_TRIP_FACTOR, CONTINUOUS_HARD_FACTOR, CONTINUOUS_DEBOUNCE_SEC,
                                   CONTINUOUS_DEBOUNCE_PACKETS)
from robot.tcp_identity import tcp_offsets_match
from safety.force_guard import force_safety_reason
from safety.safe_return import SafeReturnExecutor, validate_return_configuration
from sensor.force_preprocess import WrenchPreprocessor
from sensor.px6d_reader import PX6DReader, PX6DTimeout
from simulation.continuous_session import SIMULATION_SAMPLE_FIELDS
from workspace.workspace_transform import load_calibration as load_workspace_calibration

ROOT = Path(__file__).resolve().parent
TIMING_FIELDS = ('serial_read_start', 'serial_read_end', 'tcp_read_start', 'tcp_read_end',
                 'rtde_device_timestamp', 'rtde_packet_stagnation_sec', 'wrench_host_age_sec',
                 'tcp_host_age_sec', 'command_send_time', 'command_return_time', 'timing_source',
                 'cycle_start_time', 'loop_start_interval_sec')


def workspace_calibration_path(config, config_path):
    return resolve_calibration_path(config_path, config['continuous_tracking'].get(
        'workspace_calibration_file', 'workspace/config/workspace_calibration.yaml'))


def site_configuration_digest(config, config_path=ROOT / "config.yaml"):
    c = {k:v for k,v in config['continuous_tracking'].items()
         if k not in ('site_verification', 'search_boundary_margin')}  # derived execution clearance
    payload = dict(tcp=config['tcp'], preprocessing=config['preprocessing'], continuous=c,
                   robot=config['robot'], workspace=config['workspace'], policy=config['policy'],
                   safe_return=config['safe_return'],
                   calibration=config['calibration'], calibration_file_sha256=hashlib.sha256(
                       resolve_calibration_path(config_path, config['calibration']['file']).read_bytes()).hexdigest(),
                   workspace_calibration_file_sha256=hashlib.sha256(
                       workspace_calibration_path(config, config_path).read_bytes()).hexdigest())
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


class ContinuousStartupReturn(SafeReturnExecutor):
    """Reuse the existing return path; add continuous timing/input supervision."""

    def __init__(self, *args, poll, **kwargs):
        super().__init__(*args, **kwargs)
        self.poll = poll
        self.cycle_started = time.monotonic()
        self.previous_cycle = None

    def _observe(self, **context):
        if self.poll() in ('Q', 'ESC'):
            raise KeyboardInterrupt('operator stopped startup return')
        if context.get('phase', '').endswith('_SENSOR_READ'):
            self.cycle_started = time.monotonic()
        super()._observe(**context)

    def _check_force(self, raw, processed):
        if not np.all(np.isfinite(raw.array())) or not np.all(np.isfinite(processed.array())):
            raise RobotError('nonfinite startup return wrench')
        super()._check_force(raw, processed)

    def _log_cycle(self, now, raw, processed, robot, target, speed, phase):
        # The shared precheck logs before its force check. Validate first so a
        # failed sensor/force/log cycle can never refresh the watchdog.
        self._check_force(raw, processed)
        super()._log_cycle(now, raw, processed, robot, target, speed, phase)
        check_log = getattr(self.logger, 'check_health', None)
        if check_log is not None:
            check_log()
        validate_cycle_timing(self.config['continuous_tracking'], self.cycle_started,
                              time.monotonic(), min(self.cycle_started, robot.timestamp),
                              self.previous_cycle)
        if self.controller._watchdog_last_kick is not None:
            self.controller._check_watchdog_health()
        self.controller.kick_watchdog()
        # A slow SDK call must not authorize motion from an expired sample.
        validate_cycle_timing(self.config['continuous_tracking'], self.cycle_started,
                              time.monotonic(), min(self.cycle_started, robot.timestamp),
                              self.previous_cycle)
        if check_log is not None:
            check_log()
        self.previous_cycle = self.cycle_started

    def _segment_position_tolerance(self, phase):
        tolerance = super()._segment_position_tolerance(phase)
        if phase == 'DESCEND_TO_START':
            return min(tolerance, float(self.config['policy']['position_tolerance']))
        return tolerance

    def _stop_at_segment_end(self, phase, target):
        # A blocking stopL was consuming ~40 ms between two 30 ms-budgeted
        # sensor cycles. Keep sensing throughout braking instead of dropping
        # the gap check or treating stop-command acceptance as standstill.
        self.controller.request_return_stop()
        deadline = time.monotonic() + float(self.config['continuous_tracking']['confirmation_timeout_sec'])
        settled_since = None
        while True:
            self._observe(phase=f'{phase}_SETTLE_SENSOR_READ', return_target=target)
            raw = self.reader.read_wrench()
            self._observe(raw=raw, phase=f'{phase}_SETTLE_PROCESSING')
            processed = self.preprocessor.process(raw) if self.preprocessor is not None else raw
            self._check_force(raw, processed)
            state = self.controller.read_state()
            self._observe(robot=state, processed=processed, phase=f'{phase}_SETTLE')
            self._log_cycle(self.cycle_started, raw, processed, state, target, 0., f'{phase}_SETTLE')
            now = time.monotonic()
            if now >= deadline:
                raise RobotError(f'{phase} standstill confirmation timed out')
            low_speed = np.linalg.norm(state.tcp_speed) <= float(self.config['continuous_tracking']['settle_speed_mps'])
            settled_since = (now if settled_since is None else settled_since) if low_speed else None
            if settled_since is not None and now-settled_since >= float(self.config['continuous_tracking']['settle_hold_sec']):
                return state
            time.sleep(max(0., self.period-(time.monotonic()-self.cycle_started)))

    def _after_abort_stop(self):
        # Terminal only: a successful return must retain its script/watchdog
        # for the scan. End an aborted run before console/disk I/O can stall.
        try:
            self.controller.finish_control_script_if_stopped()
        except Exception as exc:
            if self.termination is not None:
                self.termination.set_stop_reason(detail=str(exc), exception=exc,
                    source='continuous.return_script_cleanup', terminal=False)


def check_startup_bias_stationary(controller, anchor, config):
    current = controller.read_diagnostic_state()
    if time.monotonic()-current.timestamp > float(config['continuous_tracking']['max_observation_age_sec']):
        raise RobotError('stale startup bias observation')
    if (np.linalg.norm(current.tcp_speed) > float(config['continuous_tracking']['settle_speed_mps'])
            or np.linalg.norm(current.pose[:3]-anchor[:3]) > float(config['policy']['position_tolerance'])
            or _orientation_distance(current.pose[3:], anchor[3:]) > float(config['safe_return']['return_orientation_tolerance'])):
        raise RobotError('robot moved during startup return bias capture')
    return current


def startup_return_to_p0(config, start, controller, reader, logger, poll, *, confirm=None):
    """Return True only after the shared force-monitored startup path completes.

    START moves before return when needed, so no blocking user prompt occurs
    with the watchdog armed. Temporary return bias never becomes scan bias.
    """
    controller.safe_stop_motion(force=True)
    current = controller.read_diagnostic_state()
    logger.termination.observe(robot=current, phase='STARTUP_POSE')
    if (np.linalg.norm(current.pose[:3]-start[:3]) <= float(config['policy']['position_tolerance'])
            and _orientation_distance(current.pose[3:], start[3:]) <=
            float(config['safe_return']['return_orientation_tolerance'])):
        return False
    print('Current TCP is away from calibrated P0. START will run the existing '
          'safe lift / move above P0 / descend, then continuous tracking.', flush=True)
    confirm = input if confirm is None else confirm
    if confirm('输入 START 后按回车，执行安全返回 P0 并开始连续扫描（Ctrl+C 取消；运动中 Q / Esc 停止）：').strip() != 'START':
        raise KeyboardInterrupt('START not confirmed')
    temporary = WrenchPreprocessor.from_config(config['preprocessing'])
    samples = []
    deadline = time.monotonic() + float(config['continuous_tracking']['confirmation_timeout_sec'])
    settled_since = None
    # A forced stop and fresh stationary observations precede temporary bias.
    while True:
        if poll() in ('Q', 'ESC'):
            raise KeyboardInterrupt('operator stopped startup return')
        current = controller.read_diagnostic_state()
        now = time.monotonic()
        if now-current.timestamp > float(config['continuous_tracking']['max_observation_age_sec']):
            raise RobotError('stale startup return standstill observation')
        if now >= deadline:
            raise RobotError('startup return standstill confirmation timed out')
        low_speed = np.linalg.norm(current.tcp_speed) <= float(config['continuous_tracking']['settle_speed_mps'])
        settled_since = (now if settled_since is None else settled_since) if low_speed else None
        if settled_since is not None and now-settled_since >= float(config['continuous_tracking']['settle_hold_sec']):
            break
        time.sleep(1/float(config['policy']['control_rate_hz']))
    anchor = current.pose.copy()
    count = int(config['safe_return']['startup_bias_sample_count'])
    # Only this stationary, pre-motion phase can recover a timeout. Keep both
    # the number of retries and total time finite; a timeout restarts the window.
    bias_deadline = time.monotonic() + count/float(config['sensor']['poll_rate_hz']) + float(config['continuous_tracking']['confirmation_timeout_sec'])
    retries = []
    resync_needed = discard_first = False
    while len(samples) < count:
        if poll() in ('Q', 'ESC'):
            raise KeyboardInterrupt('operator stopped startup return')
        if time.monotonic() >= bias_deadline:
            raise PX6DTimeout('startup bias acquisition time budget exhausted')
        cycle_start = time.monotonic()
        try:
            if resync_needed:
                reader.resynchronize_after_timeout()
                resync_needed = False
                check_startup_bias_stationary(controller, anchor, config)
                if poll() in ('Q', 'ESC'):
                    raise KeyboardInterrupt('operator stopped startup return')
                cycle_start = time.monotonic()
            raw = reader.read_wrench()
        except PX6DTimeout as exc:
            controller.safe_stop_motion(force=True)
            check_startup_bias_stationary(controller, anchor, config)
            retries.append(dict(attempt=len(retries)+1, detail=str(exc),
                                discarded_samples=len(samples), monotonic_sec=time.monotonic()))
            (logger.run_dir/'startup_bias_retries.json').write_text(
                json.dumps(retries, ensure_ascii=False, indent=2), encoding='utf-8')
            if len(retries) >= 3 or time.monotonic() >= bias_deadline:
                raise PX6DTimeout('startup bias timeout retry budget exhausted') from exc
            if poll() in ('Q', 'ESC'):
                raise KeyboardInterrupt('operator stopped startup return')
            print(f'启动零偏读取暂时超时，保持停止并重新采集（重试 {len(retries)}/2）。', flush=True)
            samples.clear()
            temporary = WrenchPreprocessor.from_config(config['preprocessing'])
            resync_needed = discard_first = True
            continue
        if time.monotonic() >= bias_deadline:
            raise PX6DTimeout('startup bias acquisition time budget exhausted')
        if not np.all(np.isfinite(raw.array())):
            raise RobotError('nonfinite startup bias sample')
        processed = temporary.process(raw)
        reason = force_safety_reason(raw, processed, config['policy'])
        if reason:
            raise RobotError(f'startup bias: {reason}')
        current = check_startup_bias_stationary(controller, anchor, config)
        validate_cycle_timing(config['continuous_tracking'], cycle_start, time.monotonic(), min(cycle_start, current.timestamp))
        logger.termination.observe(robot=current, raw=raw, processed=processed, phase='STARTUP_RETURN_BIAS')
        if discard_first:
            discard_first = False
            temporary = WrenchPreprocessor.from_config(config['preprocessing'])
        else:
            samples.append(raw)
        time.sleep(1/float(config['sensor']['poll_rate_hz']))
    temporary.set_zero_bias(samples)
    executor = ContinuousStartupReturn(config, start, controller, reader, temporary,
                                       logger=logger, poll=poll)
    controller.enable_watchdog(float(config['continuous_tracking']['watchdog_frequency_hz']))
    result = executor.execute()
    if result.status != 'complete':
        raise RobotError(f'automatic startup return to P0 aborted: {result.abort_reason}')
    check_start(controller, start, config)
    print('AUTOMATIC STARTUP RETURN TO P0 COMPLETE', flush=True)
    return True


def stop_after_exception(controller, termination):
    """A failed stop attempt must not skip disconnect/diagnostic cleanup."""
    try:
        controller.stop()
        finish_script = getattr(controller, 'finish_control_script_if_stopped', None)
        if finish_script is not None:
            finish_script()
    except Exception as exc:
        termination.set_stop_reason(detail=str(exc), exception=exc,
                                    source="continuous.stop_failure", terminal=False)


def prepare_real(config, config_path):
    """Read the project's saved scan and sandbox calibrations before devices exist."""
    budget = config['continuous_tracking']['max_runtime_sec']
    if budget is None or isinstance(budget, bool) or not np.isfinite(float(budget)) or float(budget) <= 0:
        raise RobotError('real continuous_tracking.max_runtime_sec must be finite and positive')
    validate_execution_configuration(config)
    validate_config(config)
    c = config['continuous_tracking']
    scan_path = resolve_calibration_path(config_path, config['calibration']['file'])
    sandbox_path = workspace_calibration_path(config, config_path)
    calibration = load_scan_calibration(scan_path, require_tcp_offset=True)
    sandbox = load_workspace_calibration(sandbox_path)
    if calibration['robot_ip'] != config['robot']['robot_ip']:
        raise RobotError('scan calibration robot_ip does not match config')
    if not tcp_offsets_match(calibration['active_tcp_offset'], config['tcp']['offset'],
                             float(config['tcp']['offset_tolerance'])):
        raise RobotError('scan calibration TCP does not match configured TCP')
    validate_calibration_constraints(calibration, float(config['calibration']['min_direction_calibration_distance']),
                                     float(config['calibration']['max_direction_calibration_z_difference']))
    validate_return_configuration(config, calibration['start_tcp_pose'])
    for name in ('return_lift_distance', 'return_speed', 'return_vertical_speed', 'return_acceleration',
                 'return_force_limit', 'return_torque_limit', 'return_position_tolerance',
                 'return_orientation_tolerance', 'return_segment_timeout_sec'):
        if not np.isfinite(float(config['safe_return'][name])):
            raise RobotError(f'safe_return.{name} must be finite')
    count = config['safe_return']['startup_bias_sample_count']
    if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
        raise RobotError('safe_return.startup_bias_sample_count must be a positive integer')
    # Preserve acquisition metadata, including configuration-only TCP provenance.
    # The actual active TCP is still read/checked by controller.connect().
    metadata = sandbox.get('metadata', {})
    if metadata.get('robot_ip') and metadata['robot_ip'] != config['robot']['robot_ip']:
        raise RobotError('workspace calibration robot_ip does not match config')
    if metadata.get('configured_tcp_offset') is not None and not tcp_offsets_match(
            metadata['configured_tcp_offset'], config['tcp']['offset'], float(config['tcp']['offset_tolerance'])):
        raise RobotError('workspace calibration TCP does not match configured TCP')
    digest = site_configuration_digest(config, config_path)
    verification = c.get('site_verification')
    # Reuse the existing project's calibration instead of requiring a second
    # boolean attestation. If explicitly supplied, a record must still be valid.
    if verification is not None:
        if not isinstance(verification, dict):
            raise RobotError('site verification must be a mapping when supplied')
        for key in ('force_sign_checked', 'base_transform_checked', 'watchdog_stop_verified'):
            if verification.get(key) is not True:
                raise RobotError(f'site verification missing: {key}')
        if not verification.get('operator') or not verification.get('checked_at'):
            raise RobotError('site verification must identify operator and time')
        if verification.get('configuration_sha256') != digest:
            raise RobotError('site verification does not match current tool/frame/sign/configuration/calibrations')
        config['continuous_reviewed_configuration_sha256'] = digest
    if c['reacquire_enabled'] and not (isinstance(verification, dict) and verification.get('recovery_verified') is True):
        raise RobotError('real recovery requires separate site verification')

    # Use measured XY corners, not the fitted rectangle or its mean Z. The
    # polygon guard below also checks sloping edges; its AABB alone is insufficient.
    polygon = np.array([[sandbox['raw_points'][f'P{i}'][axis] for axis in 'xy'] for i in range(4)])
    bounds = config['workspace']['limits'] if config['workspace'].get('enabled', True) else c.get('real_test_xy_limits')
    if bounds is None:
        bounds = {f'{axis}_{side}': float(reducer(polygon[:,i]))
                  for i, axis in enumerate('xy') for side, reducer in [('min',np.min),('max',np.max)]}
    if not isinstance(bounds, dict):
        raise RobotError('invalid continuous site XY bounds')
    margin = float(c['boundary_margin'])
    for axis in 'xy':
        low, high = float(bounds[f'{axis}_min']), float(bounds[f'{axis}_max'])
        if not np.isfinite([low, high]).all() or high-low <= 2*margin:
            raise RobotError('invalid continuous site XY bounds')
    watchdog_contract = URRTDEController.verified_watchdog_contract()
    speed_limits = dict(TARGET_SEARCH=float(c['search_speed']),
                        CONTINUOUS_TRACKING=float(np.hypot(c['tangential_speed'], c['normal_speed_limit'])),
                        LOCAL_REACQUIRE=float(c['reacquire_speed']))
    deceleration = float(config['robot']['stop_deceleration'])
    if not np.isfinite(deceleration) or deceleration <= 0:
        raise RobotError('stop_deceleration must be finite and positive')
    def stopping_margin(nominal):
        hard = min(CONTINUOUS_HARD_FACTOR * nominal, 1.2 * float(config['robot']['max_tcp_speed']))
        return hard * (CONTINUOUS_DEBOUNCE_SEC + 1/float(c['watchdog_frequency_hz'])) + hard**2/(2*deceleration)
    max_tracking = max(speed_limits['CONTINUOUS_TRACKING'], speed_limits['LOCAL_REACQUIRE'])
    required_margin = stopping_margin(max_tracking)
    if margin < required_margin or 1/float(c['watchdog_frequency_hz']) <= float(c['cycle_timeout_sec']):
        raise RobotError('stop margin/watchdog deadline incompatible with execution budgets')
    # Search alone needs more clearance. Include the entire allowed transient
    # band, its finite confirmation delay, watchdog interval and ideal braking.
    # This is an engineering budget to validate on site, not a stopping guarantee.
    search_margin = max(margin, stopping_margin(speed_limits['TARGET_SEARCH']))
    if search_margin >= float(c['search_max_distance']):
        raise RobotError('search stopping margin consumes search distance budget')
    c['search_boundary_margin'] = search_margin  # runtime snapshot only; saved calibration is unchanged
    config['continuous_sdk_contract'] = watchdog_contract
    config['continuous_loaded_configuration_sha256'] = digest
    config['policy']['search_direction_xy'] = list(calibration['scan_direction_xy'])
    robot_config = runtime_robot_config(config)
    robot_config.update(fixed_z=calibration['fixed_z'], fixed_orientation=calibration['fixed_orientation'],
                        continuous_require_watchdog=True, continuous_sample_age_sec=float(c['max_observation_age_sec']),
                        continuous_settle_speed_mps=float(c['settle_speed_mps']), continuous_xy_limits=dict(bounds),
                        continuous_xy_polygon=polygon.tolist(), continuous_speed_limits=speed_limits,
                        continuous_tracking_boundary_margin=margin, continuous_search_boundary_margin=search_margin,
                        continuous_boundary_margin=margin)
    check_continuous_xy({**robot_config, 'continuous_boundary_margin': search_margin}, calibration['start_tcp_pose'][:2])
    config['continuous_calibration'] = calibration
    config['continuous_workspace_calibration'] = sandbox
    config['continuous_calibration_sources'] = dict(scan=str(scan_path.resolve()), workspace=str(sandbox_path.resolve()))
    config['continuous_execution_envelope'] = dict(raw_xy_polygon=polygon.tolist(), xy_limits=dict(bounds), margin_m=margin,
        search_margin_m=search_margin, nominal_speed_limits_mps=speed_limits,
        trip_factor=CONTINUOUS_TRIP_FACTOR, hard_factor=CONTINUOUS_HARD_FACTOR,
        debounce_sec=CONTINUOUS_DEBOUNCE_SEC, debounce_packets=CONTINUOUS_DEBOUNCE_PACKETS)
    return robot_config, np.asarray(calibration['start_tcp_pose'])


def calibration_summary(config):
    """An offline-readable report, also printed before execute connects devices."""
    scan = config['continuous_calibration']
    return dict(sources=config['continuous_calibration_sources'],
                scan_start_tcp_pose=scan['start_tcp_pose'],
                scan_direction_reference_tcp_pose=scan['direction_reference_tcp_pose'],
                scan_direction_xy=scan['scan_direction_xy'],
                fixed_z=scan['fixed_z'], fixed_orientation=scan['fixed_orientation'],
                sandbox_raw_corners=config['continuous_workspace_calibration']['raw_points'],
                execution_envelope=config['continuous_execution_envelope'],
                loaded_configuration_sha256=config['continuous_loaded_configuration_sha256'])


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
        # Load the saved project calibrations before optional duration shortening;
        # validate any explicit extra record and never auto-authorize recovery.
        if args.execute:
            if getattr(args, 'enable_reacquire', False):
                raise RobotError('--enable-reacquire is offline only')
            robot_config, start = prepare_real(config, args.config)
            if 'continuous_calibration_sources' in config:
                print("Using saved scan / sandbox calibration:\n"+
                      json.dumps(calibration_summary(config),ensure_ascii=False,indent=2),flush=True)
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
        logger = ExperimentLogger(output, config, extra_sample_fields=EXTRA_SAMPLE_FIELDS + TIMING_FIELDS + SIMULATION_SAMPLE_FIELDS + SPEED_GUARD_FIELDS,
                                  workspace_logging=False)
        logger.termination = termination.bind(logger.run_dir)
        if args.execute:
            # Disk work stays off the device thread, with bounded snapshots and
            # explicit failure/backlog checks before every healthy-cycle kick.
            logger = ContinuousLogWriter(logger, max_pending_sec=float(
                config['continuous_tracking']['confirmation_timeout_sec']))
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
                startup_return_done = startup_return_to_p0(config, start, controller, reader, logger,
                                                          keyboard.poll, confirm=keyboard.read_line)
                robot = check_start(controller, start, config)
            baseline = config["preprocessing"].get("baseline", {"capture_on_start": True, "sample_count": 100})
            if not args.execute:
                session.capture_bias(keyboard.poll, termination.observe)
            elif baseline.get("capture_on_start", True):
                samples = []
                for _ in range(int(baseline["sample_count"])):
                    bias_cycle_start = time.monotonic()
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
                        if np.linalg.norm(current.tcp_speed) > float(config['continuous_tracking']['settle_speed_mps']):
                            raise RobotError('robot not stationary during bias capture')
                        if startup_return_done:
                            logger.check_health()
                            validate_cycle_timing(policy.c, bias_cycle_start, time.monotonic(), bias_cycle_start)
                            controller._check_motion_authorized()
                            controller.kick_watchdog()
                            validate_cycle_timing(policy.c, bias_cycle_start, time.monotonic(), bias_cycle_start)
                        time.sleep(1 / float(config["sensor"]["poll_rate_hz"]))
                    samples.append(raw)
                preprocessor.set_zero_bias(samples)
        if args.execute:
            print(f"P0: {start.tolist()}\nSearch P0 -> P1: {policy.search_direction.tolist()}")
            print(f"MUST CONFIRM ON SITE: force sign={policy.c['force_direction_sign']}, "
                  f"F_ref={policy.c['force_reference']} N, workspace enabled={config['workspace'].get('enabled', True)}")
            print("Q / ESC / Ctrl+C: stop in place. No automatic return after stopping.")
            if not startup_return_done and input("已核对空载零偏、Base 力方向、P0、现场及急停，输入 START：").strip() != "START":
                policy.request_stop(now, robot.pose, "START not confirmed", event="USER_STOP",
                                    code=TerminationReason.STOP_USER_REQUEST)
            else:
                # The operator may have moved the robot while at the prompt.
                robot = check_start(controller, start, config)
                # A startup return leaves the watchdog armed; never reset its
                # deadline after the baseline or any intervening disk stall.
                if startup_return_done:
                    logger.check_health()
                    controller._check_motion_authorized()
                else:
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
                    logger.check_health()
                    validate_cycle_timing(policy.c, cycle_start, time.monotonic(), cycle_start, previous_cycle_start)
                    serial_start = time.monotonic()
                    raw = reader.read_wrench()
                    serial_end = time.monotonic()
                    # Read TCP AFTER potentially blocking serial I/O. Both host
                    # acquisition intervals are kept; this is not hard sync.
                    tcp_start = time.monotonic()
                    robot = controller.read_state()
                    speed_diagnostics = dict(controller.speed_guard_diagnostics)
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
                    termination.observe(command=command, phase='COMMAND_EXECUTION')
                    controller.set_continuous_phase(policy.state.value, stop_confirmed=policy.stop_confirmed)
                if args.execute:
                    # Device freshness and bounded writer health are both required.
                    logger.check_health()
                    validate_cycle_timing(policy.c, cycle_start, time.monotonic(), serial_start, previous_cycle_start)
                    # DIRECTION_RECONFIRM is nonterminal: keep the watchdog
                    # armed while issuing stop and collecting fresh feedback.
                    if policy.state != State.STOP:
                        controller.kick_watchdog()
                        validate_cycle_timing(policy.c, cycle_start, time.monotonic(), serial_start, previous_cycle_start)
                        logger.check_health()
                    timing['command_send_time'] = time.monotonic()
                    if command.move:
                        controller.command_planar_velocity(command.direction_xy, command.speed, dt)
                    else:
                        controller.stop()
                    timing['command_return_time'] = time.monotonic()
                    validate_cycle_timing(policy.c, cycle_start, time.monotonic(), serial_start, previous_cycle_start)
                    if policy.state == State.STOP:
                        finish_script = getattr(controller, 'finish_control_script_if_stopped', None)
                        if finish_script is not None:
                            finish_script()
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
                    validate_cycle_timing(policy.c, cycle_start, time.monotonic(), serial_start, previous_cycle_start)
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
                    controller.stop()
                    finish_script = getattr(controller, 'finish_control_script_if_stopped', None)
                    if finish_script is not None:
                        finish_script()
                    deadline = time.monotonic() + float(config['continuous_tracking']['confirmation_timeout_sec'])
                    settled_since = None
                    while time.monotonic() < deadline:
                        current = controller.read_diagnostic_state()
                        observed = time.monotonic()
                        if observed-current.timestamp > float(config['continuous_tracking']['max_observation_age_sec']):
                            raise RobotError('stale post-stop observation')
                        robot = current
                        low_speed = np.linalg.norm(current.tcp_speed[:3]) <= float(config['continuous_tracking']['settle_speed_mps'])
                        if low_speed and finish_script is not None:
                            finish_script()
                        settled_since = (observed if settled_since is None else settled_since) if low_speed else None
                        stop_snapshot_info = dict(source='post_stop_host_observation', host_age_sec=observed-current.timestamp,
                                                  standstill_confirmed=False, **controller.observation_timing)
                        if settled_since is not None and observed-settled_since >= float(config['continuous_tracking']['settle_hold_sec']):
                            stop_snapshot_info['standstill_confirmed'] = not bool(controller.motion_fault)
                            break
                        # Never kick in terminal cleanup. Until fresh standstill
                        # permits stopping our script, timeout remains armed.
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
                            provenance=config.get('continuous_provenance'))))
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
