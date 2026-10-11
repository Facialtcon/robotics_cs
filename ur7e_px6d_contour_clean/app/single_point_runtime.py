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
    'post_stop_record_end_sec', 'standstill_speed_mps', 'distance_trigger_sec')


class SinglePointSearchLimit(RobotError):
    """Fresh execution read reached the independent distance budget."""


class FreshForceReader:
    """Bound each fresh request; host intervals are not sensor acquisition clocks."""
    def __init__(self, reader, settings, hard_limits, *, clock=time.monotonic):
        self.reader, self.settings, self.hard_limits, self.clock = reader, settings, hard_limits, clock
        self.start = self.end = None

    def read_wrench(self):
        started = self.clock()
        raw = self.reader.read_wrench()  # PX6D issues a fresh validated request every call.
        ended = self.clock()
        if ended < started or ended-started > self.settings['max_force_age_sec']:
            raise RobotError('stale force request; cannot authorize motion')
        reason = raw_safety_reason(raw, self.hard_limits)
        if reason:
            raise RobotError(reason)
        source_stamp = getattr(self.reader, 'sample_timestamp', ended)
        if not np.isfinite(source_stamp) or not 0 <= ended-source_stamp <= self.settings['max_force_age_sec']:
            raise RobotError('stale force sample')
        self.start, self.end = started, ended
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
        self.last_motion_cycle = None
        self.distance_trigger = None
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
        now = self.clock()
        if not self.force_valid or now-self.reader.start > self.s['max_force_age_sec']:
            raise RobotError('force expired during execution read; velocity send refused')
        if now-robot.timestamp > self.s['max_robot_age_sec']:
            raise RobotError('TCP sample expired before SDK send')
        delta = robot.pose[:2]-self.p0[:2]
        along = float(delta @ self.direction)
        if along+self.speed/self.s['control_rate_hz'] >= self.s['max_search_distance_m']:
            self.distance_trigger = self._actual_observation(robot)
            raise SinglePointSearchLimit('NO_CONTACT_DISTANCE')
        if np.linalg.norm(delta-along*self.direction) > self.s['start_position_tolerance_m']:
            raise RobotError('fresh execution pose departed from calibrated ray')
        from app.single_point_config import check_segment
        if self.s['workspace_limits'] is not None:
            end = robot.pose[:3].copy()
            end[:2] += self.direction*(self.speed/self.s['control_rate_hz']+self.s['braking_margin_m'])
            check_segment(robot.pose[:3], end, self.s)

    def _actual_observation(self, robot):
        return dict(timestamp=robot.timestamp, observed_host_sec=self.clock(),
                    pose=robot.pose.tolist(), velocity=robot.tcp_speed.tolist())

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
        if not self.force_failed:
            try:
                self.raw = self.reader.read_wrench()
            except Exception:
                if not diagnostic:
                    raise
                self.force_failed = True
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
            if reason:
                raise RobotError(reason)
        if now-started > self.s['max_cycle_sec']:
            raise RobotError('observation cycle deadline exceeded')
        return now

    def _stationary(self):
        r = self.robot
        if (np.linalg.norm(r.pose[:3]-self.p0[:3]) > self.s['start_position_tolerance_m'] or
            abs(r.pose[2]-self.p0[2]) > self.s['fixed_z_tolerance_m'] or
            _orientation_distance(r.pose[3:], self.p0[3:]) > self.s['orientation_tolerance_rad'] or
            probe_tilt_deg(r.pose[3:], self.project['calibration']['probe_axis_tcp']) > self.s['probe_tilt_max_deg'] or
            np.linalg.norm(r.tcp_speed[:3]) > self.s['settle_speed_mps'] or
            np.linalg.norm(r.tcp_speed[3:]) > .005):
            raise RobotError('prepared P0 must be stationary at reference Z with a downward probe')
        if (np.hypot(self.processed.fx, self.processed.fy) >= self.s['unloaded_max_fxy_N'] or
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
            distance_trigger_sec=self.distance_trigger['observed_host_sec'] if self.distance_trigger else '',
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
        self._stationary()
        self._kick()
        self._record(stamp)
        self.logger.check_health()

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
        if reason == 'NO_CONTACT_DISTANCE' and self.distance_trigger is None and self.robot is not None:
            self.distance_trigger = self._actual_observation(self.robot)
        if contact:
            self.trigger_force_time = self.reader.end
            self.actual_before = deepcopy(self.previous_actual)
            self.actual_at_threshold = dict(timestamp=self.robot.timestamp,
                velocity=self.robot.tcp_speed.tolist(), pose=self.robot.pose.tolist())
        if (report or {}).get('api_anomaly'):
            self._stop_fault('BRAKING_API_FAILURE', 'braking API failed or rejected the command')
        if self.robot is not None:
            try:
                self._event('FIRST_THRESHOLD_STOP_REQUEST' if contact else
                            'USER_STOP' if reason == 'USER_ABORT' else 'BUDGET_STOP',
                            self.threshold_time if contact else self.braking_time,
                            pose=self.distance_trigger['pose'] if reason == 'NO_CONTACT_DISTANCE' and self.distance_trigger else None)
            except Exception as exc:
                self.fault = self.fault or f'event logging failed: {exc}'

    def _approach_guard(self, stamp):
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
        if self.path_distance+self.speed/self.s['control_rate_hz'] >= self.s['max_search_distance_m']:
            return 'NO_CONTACT_DISTANCE'
        if stamp-self.approach_start >= self.s['max_approach_time_sec']:
            return 'NO_CONTACT_TIME'
        return None

    def approach(self):
        if not self.preparation_complete:
            raise RobotError('startup preparation incomplete; approach refused')
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
            if np.hypot(self.processed.fx, self.processed.fy) >= self.s['contact_threshold_N']:
                self._request_stop('CONTACT_THRESHOLD', contact=True)
            elif limit_reason:
                self._request_stop(limit_reason)
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
                # A sensor failure must not prevent TCP braking telemetry.
                self.force_failed = True
                self.force_valid = False
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
                                 'STOP_TELEMETRY_INCOMPLETE', 'force sensor failed during settling; TCP monitoring continued')
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
            if isinstance(exc, SinglePointSearchLimit):
                reason = 'NO_CONTACT_DISTANCE'
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
                    self.post_record_complete and not self.fault else 'no_contact' if self.reason.startswith('NO_CONTACT') and
                    self.stopped_time is not None and self.post_record_complete and not self.fault else 'aborted',
            state=self.state, reason=self.reason, fault=self.fault, p0_name=self.name, p0=self.p0.tolist(),
            P_ref=self.calibration['P_ref'], direction_xy=self.direction.tolist(), nominal_speed_mps=self.speed,
            contact_threshold_N=self.s['contact_threshold_N'], threshold_trigger_sec=self.threshold_time,
            force_trigger_sample_sec=self.trigger_force_time,
            braking_request_sec=self.braking_time, actual_standstill_sec=self.stopped_time,
            actual_before_threshold=self.actual_before, actual_at_threshold=self.actual_at_threshold,
            actual_stop_pose=self.stop_pose,
            actual_stop_tcp_speed=self.stop_speed, no_contact_distance_trigger=self.distance_trigger,
            stop_phase=self.stop_phase, stop_failure_kind=self.stop_failure_kind,
            stop_failures=deepcopy(self.stop_failures), timeout_phase=self.timeout_phase,
            stop_confirmation_deadline_sec=self.stop_confirmation_deadline,
            post_stop_record_start_sec=self.post_record_start, post_stop_record_end_sec=self.post_record_end,
            post_stop_record_completed_sec=self.post_record_finished,
            post_stop_record_deadline_sec=self.post_record_deadline, post_stop_record_completed=self.post_record_complete,
            max_search_distance_m=self.s['max_search_distance_m'],
            max_approach_time_sec=self.s['max_approach_time_sec'],
            peak_filtered_fxy_N=self.peak_filtered, peak_unfiltered_fxy_N=self.peak_unfiltered,
            actual_search_path_m=self.path_distance,
            time_basis='host monotonic; force request interval, actual TCP read, command send sampled separately',
            stop_report=deepcopy(getattr(self.controller, 'stop_report', None)))
