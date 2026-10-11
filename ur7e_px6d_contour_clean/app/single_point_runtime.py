"""One straight approach, first force crossing, braking, measured standstill.

This module never imports a tracking/recovery policy. P_ref is log metadata,
never a motion target or completion condition. Device ownership is unchanged.
"""
from copy import deepcopy
import time

import numpy as np

from calibration.single_point import experiment_start_pose
from calibration.probe_alignment import probe_tilt_deg
from core.models import PolicyCommand, PolicyWaypoint, Wrench
from robot.rtde_controller import RobotError, _orientation_distance
from safety.force_guard import force_safety_reason, raw_safety_reason

EXTRA_FIELDS = ('force_read_start_sec', 'force_read_end_sec', 'force_valid',
    'force_timestamp_source', 'rtde_device_timestamp_sec', 'contact_threshold_N',
    'p0_name', 'p0_x', 'p0_y', 'p0_z', 'reference_x', 'reference_y', 'reference_z',
    'approach_direction_x', 'approach_direction_y', 'nominal_approach_speed_mps',
    'threshold_trigger_sec', 'braking_request_sec', 'standstill_confirmed_sec',
    'stop_phase', 'stop_failure_kind', 'timeout_phase', 'post_stop_record_start_sec',
    'post_stop_record_end_sec', 'standstill_speed_mps', 'force_safety_reason',
    'processed_torque_norm_Nm', 'force_read_error', 'experiment_checks_active',
    'experiment_checks_activated_sec')


class SinglePointSafetyLimit(RobotError):
    """The independent experiment time limit expired before a velocity send."""


class ForceSafetyThresholdExceeded(RobotError):
    """A fresh finite measurement exceeded a limit; the sensor still works."""
    def __init__(self, reason, raw):
        super().__init__(reason)
        self.raw = raw


class FreshForceReader:
    """Bound each fresh request; host intervals are not sensor acquisition clocks."""
    def __init__(self, reader, settings, hard_limits, *, clock=time.monotonic):
        self.reader, self.settings, self.hard_limits, self.clock = reader, settings, hard_limits, clock
        self.start = self.end = None

    def read_wrench(self, *, enforce_limits=True):
        started = self.clock()
        raw = self.reader.read_wrench()  # PX6D issues a fresh validated request every call.
        ended = self.clock()
        if ended < started or ended-started > self.settings['max_force_age_sec']:
            raise RobotError('stale force request; cannot authorize motion')
        source_stamp = getattr(self.reader, 'sample_timestamp', ended)
        if not np.isfinite(source_stamp) or not 0 <= ended-source_stamp <= self.settings['max_force_age_sec']:
            raise RobotError('stale force sample')
        self.start, self.end = started, ended
        reason = raw_safety_reason(raw, self.hard_limits)
        if reason:
            if not np.isfinite(raw.array()).all():
                raise RobotError(reason)
            if enforce_limits:
                raise ForceSafetyThresholdExceeded(reason, raw)
        return raw


class SinglePointTrial:
    def __init__(self, project, settings, calibration, name, speed, controller, reader,
                 preprocessor, logger, *, poll=lambda: None, clock=time.monotonic, sleep=time.sleep,
                 preparation_complete=False):
        from app.single_point_config import validate_speed
        self.project, self.s, self.calibration, self.name = project, settings, calibration, name
        self.speed = validate_speed(speed, settings)
        self.controller, self.reader, self.processor, self.logger = controller, reader, preprocessor, logger
        self.poll, self.clock, self.sleep = poll, clock, sleep
        self.p0 = experiment_start_pose(calibration, name, project['calibration']['probe_axis_tcp'])
        self.preparation_complete = preparation_complete
        self.experiment_checks_active = False
        self.experiment_checks_activated_sec = None
        self.direction = np.asarray(calibration['groups'][name]['direction_xy'], dtype=float)
        self.state = 'READY'
        self.robot = self.raw = self.processed = None
        self.force_valid = False
        self.previous_pose = None
        self.path_distance = 0.
        self.approach_start = self.threshold_time = self.braking_time = self.stopped_time = None
        self.trigger_force_time = self.stop_pose = None
        self.previous_actual = self.actual_before = self.actual_at_threshold = None
        self.peak_filtered = self.peak_unfiltered = 0.
        self.reason = ''
        self.fault = ''
        self.stop_requested = False
        self.force_failed = False
        self.force_failure_detail = ''
        self.force_safety_reason = ''
        self.safety_events = []
        self.last_motion_cycle = None
        self.trigger_force = None
        self.stop_speed = None
        self.stop_phase = 'NOT_REQUESTED'
        self.stop_failure_kind = self.timeout_phase = None
        self.stop_failures = []
        self.post_record_start = self.post_record_end = None
        self.post_record_finished = None
        self.post_record_complete = False
        self.stop_confirmation_deadline = self.post_record_deadline = None
        self.controller.before_velocity_send = self._before_send

    def _before_send(self, robot):
        if not self.experiment_checks_active or self.stop_requested:
            raise RobotError('formal approach not authorized or already stopped; velocity send refused')
        now = self.clock()
        if not self.force_valid or now-self.reader.start > self.s['max_force_age_sec']:
            raise RobotError('force expired during execution read; velocity send refused')
        if now-robot.timestamp > self.s['max_robot_age_sec']:
            raise RobotError('TCP sample expired before SDK send')
        delta = robot.pose[:2]-self.p0[:2]
        along = float(delta @ self.direction)
        if self.approach_start is not None and now-self.approach_start >= self.s['max_approach_time_sec']:
            raise SinglePointSafetyLimit('EXPERIMENT_TIME_LIMIT')
        if np.linalg.norm(delta-along*self.direction) > self.s['start_position_tolerance_m']:
            raise RobotError('fresh execution pose departed from calibrated ray')
        from app.single_point_config import check_segment
        end = robot.pose[:3].copy()
        end[:2] += self.direction*(self.speed/self.s['control_rate_hz']+self.s['braking_margin_m'])
        check_segment(robot.pose[:3], end, self.s)

    def _event(self, kind, stamp, *, pose=None):
        self.logger.log_waypoint(PolicyWaypoint(stamp, self.state, kind,
                                               self.robot.pose.copy() if pose is None else np.asarray(pose),
                                               target_direction_xy=self.direction.copy()))

    def _drain_commands(self):
        records = list(self.controller.command_records)
        self.controller.command_records.clear()
        self.logger.log_commands(records)

    def _observe(self, *, diagnostic=False):
        started = self.clock()
        self.force_valid = False
        self.force_safety_reason = ''
        if not self.force_failed:
            try:
                self.raw = self.reader.read_wrench(enforce_limits=self.experiment_checks_active or diagnostic)
            except ForceSafetyThresholdExceeded as exc:
                if not diagnostic:
                    raise
                # Braking is already requested. Preserve the valid over-limit
                # sample and keep reading; it is not a disconnected sensor.
                self.raw = exc.raw
            except Exception as exc:
                if not diagnostic:
                    raise
                self.force_failed = True
                self.force_failure_detail = f'{type(exc).__name__}: {exc}'
        self.robot = (self.controller.read_diagnostic_state() if diagnostic else self.controller.read_state())
        now = self.clock()
        if not np.isfinite(np.r_[self.robot.pose, self.robot.tcp_speed, self.robot.timestamp]).all():
            raise RobotError('invalid TCP observation')
        if not 0 <= now-self.robot.timestamp <= self.s['max_robot_age_sec']:
            raise RobotError('stale TCP observation')
        if not self.force_failed:
            self.processor.set_tool_orientation(self.robot.pose[3:])
            if self.processor.force_transform_status['output_frame'] != 'Base':
                raise RobotError('verified Sensor to Base transform is required')
            self.processed = self.processor.process(self.raw)
            if not np.isfinite(self.processed.array()).all():
                raise RobotError('invalid processed wrench')
            if now-self.reader.start > self.s['max_force_age_sec']:
                raise RobotError('force sample expired while reading robot state')
            self.force_valid = True
            self.peak_filtered = max(self.peak_filtered, float(np.hypot(self.processed.fx, self.processed.fy)))
            self.peak_unfiltered = max(self.peak_unfiltered, float(np.linalg.norm(self.processor.force_base[:2])))
            reason = force_safety_reason(self.raw, self.processed, self.project['policy'])
            if reason and (self.experiment_checks_active or diagnostic):
                self._note_force_safety(reason, now)
                if not diagnostic:
                    raise RobotError(reason)
        if now-started > self.s['max_cycle_sec']:
            raise RobotError('observation cycle deadline exceeded')
        return now

    def _note_force_safety(self, reason, stamp):
        """Keep bounded, measured safety evidence separately from stop failures."""
        policy = self.project['policy']
        checks = {
            'processed force safety threshold exceeded': (self.processed.force, 'safety_force_threshold', 'N'),
            'processed torque safety threshold exceeded': (self.processed.torque, 'safety_torque_threshold', 'N·m'),
            'absolute raw force safety threshold exceeded': (self.raw.force, 'absolute_raw_force_threshold', 'N'),
            'absolute raw torque safety threshold exceeded': (self.raw.torque, 'absolute_raw_torque_threshold', 'N·m'),
        }
        self.force_safety_reason = reason
        # Record every exceeded limit, even when the shared guard returns only
        # its first match. This does not change the guard or any thresholds.
        for detail, (vector, key, unit) in checks.items():
            value, limit = float(np.linalg.norm(vector)), float(policy[key])
            if value < limit:
                continue
            event = next((item for item in self.safety_events if item['reason'] == detail), None)
            if event is None:
                event = dict(reason=detail, phase=self.stop_phase if self.stop_requested else self.state,
                    first_sec=stamp, force_read_start_sec=self.reader.start, force_read_end_sec=self.reader.end,
                    value=value, limit=limit, unit=unit, sample_count=0, peak_value=value, peak_sec=stamp,
                    peak_phase=self.stop_phase if self.stop_requested else self.state,
                    raw_wrench_sensor=self.raw.array().tolist(),
                    processed_wrench_base=self.processed.array().tolist(), tcp_pose=self.robot.pose.tolist())
                self.safety_events.append(event)
                message = f'{detail}: {value:.3f} >= {limit:.3f} {unit}'
                self.fault = f'{self.fault}; {message}' if self.fault else message
            event['sample_count'] += 1
            event['last_sec'] = stamp
            if value > event['peak_value']:
                event.update(peak_value=value, peak_sec=stamp,
                             peak_phase=self.stop_phase if self.stop_requested else self.state)

    def _stationary(self, *, check_unloaded=True):
        r = self.robot
        if (np.linalg.norm(r.pose[:3]-self.p0[:3]) > self.s['start_position_tolerance_m'] or
            abs(r.pose[2]-self.p0[2]) > self.s['fixed_z_tolerance_m'] or
            _orientation_distance(r.pose[3:], self.p0[3:]) > self.s['orientation_tolerance_rad'] or
            probe_tilt_deg(r.pose[3:], self.project['calibration']['probe_axis_tcp']) > self.s['probe_tilt_max_deg'] or
            np.linalg.norm(r.tcp_speed[:3]) > self.s['settle_speed_mps'] or
            np.linalg.norm(r.tcp_speed[3:]) > .005):
            raise RobotError('prepared P0 must be stationary at reference Z with a downward probe')
        if check_unloaded and (np.hypot(self.processed.fx, self.processed.fy) >= self.s['unloaded_max_fxy_N'] or
            np.linalg.norm(self.processor.air_compensated_wrench_base(self.raw).force) >= self.s['unloaded_max_fxy_N']):
            raise RobotError('P0 is not unloaded; do not zero against the target')

    def _record(self, stamp):
        self._drain_commands()
        missing = Wrench.from_sequence([float('nan')]*6)
        extra = dict(force_read_start_sec=self.reader.start if self.force_valid else '',
            force_read_end_sec=self.reader.end if self.force_valid else '', force_valid=int(self.force_valid),
            force_timestamp_source='host_request_interval_not_device_acquisition',
            rtde_device_timestamp_sec=getattr(self.controller, 'observation_timing', {}).get('rtde_device_timestamp', ''),
            contact_threshold_N=self.s['contact_threshold_N'], p0_name=self.name,
            **dict(zip(('p0_x', 'p0_y', 'p0_z'), self.p0[:3])),
            **dict(zip(('reference_x', 'reference_y', 'reference_z'), self.calibration['P_ref'])),
            approach_direction_x=self.direction[0], approach_direction_y=self.direction[1],
            nominal_approach_speed_mps=self.speed, threshold_trigger_sec=self.threshold_time if self.threshold_time is not None else '',
            braking_request_sec=self.braking_time if self.braking_time is not None else '',
            standstill_confirmed_sec=self.stopped_time if self.stopped_time is not None else '',
            stop_phase=self.stop_phase, stop_failure_kind=self.stop_failure_kind or '',
            timeout_phase=self.timeout_phase or '',
            post_stop_record_start_sec=self.post_record_start if self.post_record_start is not None else '',
            post_stop_record_end_sec=self.post_record_end if self.post_record_end is not None else '',
            standstill_speed_mps=float(np.linalg.norm(self.stop_speed[:3])) if self.stop_speed is not None else '',
            force_safety_reason=self.force_safety_reason,
            processed_torque_norm_Nm=float(np.linalg.norm(self.processed.torque)) if self.force_valid else '',
            force_read_error=self.force_failure_detail,
            experiment_checks_active=int(self.experiment_checks_active),
            experiment_checks_activated_sec=self.experiment_checks_activated_sec if
                self.experiment_checks_activated_sec is not None else '',
            processed_force_frame='Base',
            **(self.processor.force_log_fields if self.force_valid else {}))
        # PolicyCommand is only the existing immutable log format here.
        command = PolicyCommand(self.state, self.state == 'APPROACH', self.direction.copy(),
            self.speed if self.state == 'APPROACH' else 0., self.p0.copy(), self.threshold_time is not None,
            False, reason=self.reason)
        self.logger.log_sample(stamp, self.raw if self.force_valid else missing,
            self.processed if self.force_valid else missing, self.robot, command, self.direction, None, extra=extra)

    def _kick(self):
        if self.controller.watchdog_active:
            self.controller._check_watchdog_health()
            self.controller.kick_watchdog()

    def hold_once(self):
        if self.poll() in ('Q', 'ESC'):
            raise KeyboardInterrupt('operator stopped before approach')
        stamp = self._observe()
        self._stationary(check_unloaded=self.experiment_checks_active)
        self._kick()
        self._record(stamp)
        self.logger.check_health()

    def authorize_approach(self):
        """Called after the existing final Enter; validate before any velocity."""
        self.experiment_checks_active = True
        self.experiment_checks_activated_sec = self.clock()
        self.hold_once()

    def record_precontact(self):
        started = self.clock()
        while self.clock()-started < self.s['precontact_record_sec']:
            cycle = self.clock()
            self.hold_once()
            self.sleep(max(0., 1/self.s['control_rate_hz']-(self.clock()-cycle)))

    def _request_stop(self, reason, *, contact=False):
        if self.stop_requested:
            return
        self.reason = reason
        self.threshold_time = self.clock() if contact else None
        # First action at crossing: request braking. No disk I/O precedes it.
        requested_at = self.clock()
        report = self.controller.request_stop(nonblocking=True)
        self.braking_time = (report or {}).get('request_host_monotonic', requested_at)
        self.stop_requested = True
        self.stop_phase = 'BRAKING_CONFIRMATION'
        self.state = 'CONTACT_STOP' if contact else 'SETTLING'
        if contact:
            self.trigger_force_time = self.reader.end
            self.actual_before = deepcopy(self.previous_actual)
            self.actual_at_threshold = dict(timestamp=self.robot.timestamp,
                velocity=self.robot.tcp_speed.tolist(), pose=self.robot.pose.tolist())
            self.trigger_force = dict(frame='Base', fxy_N=float(np.hypot(self.processed.fx, self.processed.fy)),
                                      processed_wrench_base=self.processed.array().tolist(),
                                      unfiltered_force_base=self.processor.force_base.tolist(),
                                      force_read_start_sec=self.reader.start,
                                      force_read_end_sec=self.reader.end,
                                      raw_wrench_sensor=self.raw.array().tolist())
        if (report or {}).get('api_anomaly'):
            self._stop_fault('BRAKING_API_FAILURE', 'braking API failed or rejected the command')
        if self.robot is not None:
            try:
                self._event('FIRST_THRESHOLD_STOP_REQUEST' if contact else
                            'USER_STOP' if reason == 'USER_ABORT' else 'SAFETY_STOP',
                            self.threshold_time if contact else self.braking_time)
            except Exception as exc:
                self.fault = self.fault or f'event logging failed: {exc}'

    def _approach_guard(self, stamp):
        from app.single_point_config import check_segment
        check_segment(self.robot.pose[:3], self.robot.pose[:3], self.s)
        delta = self.robot.pose[:2]-self.p0[:2]
        along = float(delta @ self.direction)
        lateral = float(np.linalg.norm(delta-along*self.direction))
        if (lateral > self.s['start_position_tolerance_m'] or along < -self.s['start_position_tolerance_m'] or
            abs(self.robot.pose[2]-self.p0[2]) > self.s['fixed_z_tolerance_m'] or
            _orientation_distance(self.robot.pose[3:], self.p0[3:]) > self.s['orientation_tolerance_rad']):
            raise RobotError('TCP deviated from calibrated straight XY approach')
        if self.previous_pose is not None:
            self.path_distance += float(np.linalg.norm(self.robot.pose[:3]-self.previous_pose[:3]))
        self.previous_pose = self.robot.pose.copy()
        if np.linalg.norm(self.robot.tcp_speed[:3]) > self.s['speed_max_mps']*1.2:
            raise RobotError('actual approach speed exceeds independent limit')
        if stamp-self.approach_start >= self.s['max_approach_time_sec']:
            return 'EXPERIMENT_TIME_LIMIT'
        return None

    def approach(self):
        if not self.preparation_complete:
            raise RobotError('startup preparation incomplete; approach refused')
        self.experiment_checks_active = True
        if self.experiment_checks_activated_sec is None:
            self.experiment_checks_activated_sec = self.clock()
        self.state = 'APPROACH'
        self.approach_start = self.clock()
        self.previous_pose = self.p0.copy()
        while not self.stop_requested:
            cycle = self.clock()
            if self.last_motion_cycle is not None and cycle-self.last_motion_cycle > self.s['max_cycle_sec']:
                raise RobotError('approach observation gap exceeded; expired prior command cycle')
            self.last_motion_cycle = cycle
            if self.poll() in ('Q', 'ESC'):
                self._request_stop('USER_ABORT'); break
            stamp = self._observe()
            # Position of P_ref never appears in these termination conditions.
            limit_reason = self._approach_guard(stamp)
            if limit_reason:
                self._request_stop(limit_reason)
            elif np.hypot(self.processed.fx, self.processed.fy) >= self.s['contact_threshold_N']:
                self._request_stop('CONTACT_THRESHOLD', contact=True)
            else:
                self.previous_actual = dict(timestamp=self.robot.timestamp, velocity=self.robot.tcp_speed.tolist())
                self._kick()
                self.logger.check_health()
                if self.clock()-self.reader.start > self.s['max_force_age_sec']:
                    raise RobotError('force expired before command')
                if self.clock()-cycle > self.s['max_cycle_sec']:
                    raise RobotError('command cycle deadline exceeded')
                self.controller.command_planar_velocity(self.direction, self.speed, 1/self.s['control_rate_hz'])
                if self.clock()-cycle > self.s['max_cycle_sec']:
                    raise RobotError('speed command cycle deadline exceeded')
            self._record(stamp)
            self.sleep(max(0., 1/self.s['control_rate_hz']-(self.clock()-cycle)))

    def _stop_fault(self, kind, message, *, timeout_phase=None):
        priority = dict(STOP_TELEMETRY_INCOMPLETE=0, POST_STOP_RECORDING_INCOMPLETE=1, BRAKING_API_FAILURE=2,
                        STANDSTILL_NOT_CONFIRMED=3, STANDSTILL_REVOKED=3)
        if self.stop_failure_kind is None or priority[kind] > priority[self.stop_failure_kind]:
            self.stop_failure_kind = kind
        if not any(item['kind'] == kind for item in self.stop_failures):
            self.stop_failures.append(dict(kind=kind, phase=self.stop_phase, detail=message))
            self.fault = f'{self.fault}; {message}' if self.fault else message
        if timeout_phase:
            self.timeout_phase = timeout_phase

    def _validate_stop_observation(self):
        now = self.clock()
        if (not np.isfinite(np.r_[self.robot.pose, self.robot.tcp_speed, self.robot.timestamp]).all() or
            not 0 <= now-self.robot.timestamp <= self.s['max_robot_age_sec']):
            raise RobotError('stop confirmation TCP sample is stale or invalid')
        return (np.linalg.norm(self.robot.tcp_speed[:3]) <= self.s['settle_speed_mps'] and
                np.linalg.norm(self.robot.tcp_speed[3:]) <= .005)

    def settle(self):
        self.state = 'SETTLING'
        self.stop_phase = 'POST_STOP_RECORDING' if self.stopped_time is not None else 'BRAKING_CONFIRMATION'
        report = getattr(self.controller, 'stop_report', None) or {}
        if self.braking_time is None:
            self.braking_time = report.get('request_host_monotonic', self.clock())
        self.stop_confirmation_deadline = self.braking_time+self.s['stop_timeout_sec']
        record_failed = False
        while True:
            cycle = self.clock()
            if self.stopped_time is None and cycle >= self.stop_confirmation_deadline:
                message = 'actual TCP standstill was not confirmed within braking confirmation budget'
                self._stop_fault('STANDSTILL_NOT_CONFIRMED', message, timeout_phase='BRAKING_CONFIRMATION')
                raise RobotError(message)
            if self.stopped_time is not None and cycle > self.post_record_deadline:
                message = 'post-stop data recording did not complete within its independent budget'
                self._stop_fault('POST_STOP_RECORDING_INCOMPLETE', message, timeout_phase='POST_STOP_RECORDING')
                raise RobotError(message)
            if self.poll() in ('Q', 'ESC'):
                self.fault = self.fault or 'operator aborted during braking; still monitoring actual stop'
            try:
                stamp = self._observe(diagnostic=True)
            except Exception as exc:
                self.fault = self.fault or f'{type(exc).__name__}: {exc}'
                # TCP/processing errors are observation failures, not evidence
                # of a failed force sensor. Continue fresh TCP stop monitoring.
                self.force_valid = False
                record_failed = record_failed or self.stopped_time is not None
                self._stop_fault('POST_STOP_RECORDING_INCOMPLETE' if self.stopped_time is not None else
                                 'STOP_TELEMETRY_INCOMPLETE', f'settling observation failed: {type(exc).__name__}: {exc}')
                try:
                    self.robot = self.controller.read_diagnostic_state()
                except Exception as tcp_exc:
                    self._stop_fault('POST_STOP_RECORDING_INCOMPLETE' if self.stopped_time is not None else
                                     'STANDSTILL_NOT_CONFIRMED', f'fresh stop telemetry unavailable: {tcp_exc}')
                    raise
                stamp = self.clock()
            try:
                low = self._validate_stop_observation()
            except RobotError as exc:
                self._stop_fault('POST_STOP_RECORDING_INCOMPLETE' if self.stopped_time is not None else
                                 'STANDSTILL_NOT_CONFIRMED', str(exc))
                raise
            if self.stopped_time is not None and not low:
                message = 'actual standstill was revoked during post-stop recording'
                self._stop_fault('STANDSTILL_REVOKED', message)
                raise RobotError(message)
            try:
                stopped = self.controller.poll_stop()
            except Exception as exc:
                kind = 'STANDSTILL_REVOKED' if self.stopped_time is not None else 'STANDSTILL_NOT_CONFIRMED'
                self._stop_fault(kind, str(exc), timeout_phase='BRAKING_CONFIRMATION' if
                                 self.stopped_time is None and self.clock() >= self.stop_confirmation_deadline else None)
                raise
            report = getattr(self.controller, 'stop_report', None) or {}
            if report.get('api_anomaly'):
                self._stop_fault('BRAKING_API_FAILURE', 'braking API failed or rejected the command')
            if self.stopped_time is not None and not stopped:
                message = 'actual standstill was revoked during post-stop recording'
                self._stop_fault('STANDSTILL_REVOKED', message)
                raise RobotError(message)
            if self.force_failed:
                self.fault = self.fault or 'force sensor failed during settling'
                record_failed = record_failed or self.stopped_time is not None
                self._stop_fault('POST_STOP_RECORDING_INCOMPLETE' if self.stopped_time is not None else
                                 'STOP_TELEMETRY_INCOMPLETE',
                                 f'force data unavailable during settling: {self.force_failure_detail}; TCP monitoring continued')
            if stopped and self.stopped_time is None:
                # poll_stop can perform a fresh mode-exit read. Record a TCP
                # observation after that operation as the confirmed stop pose.
                try:
                    self.robot = self.controller.read_diagnostic_state()
                    stamp = self.clock()
                    low = self._validate_stop_observation()
                except Exception as exc:
                    self._stop_fault('STANDSTILL_NOT_CONFIRMED', f'fresh confirmation telemetry unavailable: {exc}')
                    raise
                if not low:
                    self._stop_fault('STANDSTILL_NOT_CONFIRMED', 'controller stop confirmation disagrees with actual speed')
                    raise RobotError('controller stop confirmation disagrees with actual speed')
                if stamp >= self.stop_confirmation_deadline:
                    message = 'actual TCP standstill was not confirmed within braking confirmation budget'
                    self._stop_fault('STANDSTILL_NOT_CONFIRMED', message, timeout_phase='BRAKING_CONFIRMATION')
                    raise RobotError(message)
                self.stopped_time = stamp
                self.stop_pose = self.robot.pose.tolist()
                self.stop_speed = self.robot.tcp_speed.tolist()
                self.stop_phase = 'POST_STOP_RECORDING'
                self.post_record_start = stamp
                # Permit one bounded observation cycle to finish the final sample.
                self.post_record_deadline = stamp+self.s['post_stop_record_sec']+self.s['max_cycle_sec']
                try:
                    self._event('EXECUTION_STOP_CONFIRMED', stamp)
                except Exception as exc:
                    self.fault = self.fault or f'stop event logging failed: {exc}'
            try:
                self._kick()
            except Exception as exc:
                self.fault = self.fault or f'watchdog failed during braking: {exc}'
            try:
                final_sample = stopped and self.stopped_time is not None and stamp-self.stopped_time >= self.s['post_stop_record_sec']
                if final_sample:
                    self.post_record_end = stamp
                self._record(stamp)
                self.logger.check_health()
            except Exception as exc:
                self.fault = self.fault or f'braking telemetry logging failed: {exc}'
                record_failed = record_failed or self.stopped_time is not None
                self._stop_fault('POST_STOP_RECORDING_INCOMPLETE' if self.stopped_time is not None else
                                 'STOP_TELEMETRY_INCOMPLETE', f'braking telemetry logging failed: {exc}')
            if stopped and self.stopped_time is not None and stamp-self.stopped_time >= self.s['post_stop_record_sec']:
                finished = self.clock()
                if finished > self.post_record_deadline:
                    message = 'post-stop data recording did not complete within its independent budget'
                    self._stop_fault('POST_STOP_RECORDING_INCOMPLETE', message, timeout_phase='POST_STOP_RECORDING')
                    raise RobotError(message)
                self.post_record_complete = not record_failed
                self.post_record_finished = finished if self.post_record_complete else None
                self.stop_phase = 'COMPLETE' if self.post_record_complete else 'POST_STOP_RECORDING'
                self.state = 'FINISHED'
                return
            self.sleep(max(0., 1/self.s['control_rate_hz']-(self.clock()-cycle)))

    def run(self):
        try:
            self.approach()
        except BaseException as exc:
            if isinstance(exc, SinglePointSafetyLimit):
                reason = 'EXPERIMENT_TIME_LIMIT'
            elif isinstance(exc, KeyboardInterrupt):
                self.fault = f'{type(exc).__name__}: {exc}'
                reason = 'USER_ABORT'
            else:
                self.fault = f'{type(exc).__name__}: {exc}'
                reason = 'DEVICE_OR_SAFETY_ERROR'
            # Stop before logging diagnostics, including logger/sensor faults.
            self._request_stop(reason)
        try:
            self.settle()
        except BaseException as exc:
            self.fault = self.fault or f'{type(exc).__name__}: {exc}'
        return self.summary()

    def summary(self):
        return dict(status='contact' if self.threshold_time is not None and self.stopped_time is not None and
                    self.post_record_complete and not self.fault else 'aborted',
            state=self.state, reason=self.reason, fault=self.fault, p0_name=self.name, p0=self.p0.tolist(),
            contact_triggered=self.threshold_time is not None, safety_events=deepcopy(self.safety_events),
            force_read_error=self.force_failure_detail,
            experiment_checks_active=self.experiment_checks_active,
            experiment_checks_activated_sec=self.experiment_checks_activated_sec,
            P_ref=self.calibration['P_ref'], direction_xy=self.direction.tolist(), nominal_speed_mps=self.speed,
            contact_threshold_N=self.s['contact_threshold_N'], threshold_trigger_sec=self.threshold_time,
            force_trigger_sample_sec=self.trigger_force_time,
            braking_request_sec=self.braking_time, actual_standstill_sec=self.stopped_time,
            actual_before_threshold=self.actual_before, actual_at_threshold=self.actual_at_threshold,
            actual_stop_pose=self.stop_pose,
            actual_stop_tcp_speed=self.stop_speed, force_at_threshold=self.trigger_force,
            stop_phase=self.stop_phase, stop_failure_kind=self.stop_failure_kind,
            stop_failures=deepcopy(self.stop_failures), timeout_phase=self.timeout_phase,
            stop_confirmation_deadline_sec=self.stop_confirmation_deadline,
            post_stop_record_start_sec=self.post_record_start, post_stop_record_end_sec=self.post_record_end,
            post_stop_record_completed_sec=self.post_record_finished,
            post_stop_record_deadline_sec=self.post_record_deadline, post_stop_record_completed=self.post_record_complete,
            max_approach_time_sec=self.s['max_approach_time_sec'],
            peak_filtered_fxy_N=self.peak_filtered, peak_unfiltered_fxy_N=self.peak_unfiltered,
            actual_search_path_m=self.path_distance,
            time_basis='host monotonic; force request interval, actual TCP read, command send sampled separately',
            stop_report=deepcopy(getattr(self.controller, 'stop_report', None)))
