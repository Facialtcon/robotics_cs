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
    'threshold_trigger_sec', 'braking_request_sec', 'standstill_confirmed_sec')


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
            raise SinglePointSearchLimit('NO_CONTACT_DISTANCE')
        if np.linalg.norm(delta-along*self.direction) > self.s['start_position_tolerance_m']:
            raise RobotError('fresh execution pose departed from calibrated ray')
        from app.single_point_config import check_segment
        if self.s['workspace_limits'] is not None:
            end = robot.pose[:3].copy()
            end[:2] += self.direction*(self.speed/self.s['control_rate_hz']+self.s['braking_margin_m'])
            check_segment(robot.pose[:3], end, self.s)

    def _event(self, kind, stamp):
        self.logger.log_waypoint(PolicyWaypoint(stamp, self.state, kind, self.robot.pose.copy(),
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
        self.state = 'CONTACT_STOP' if contact else 'SETTLING'
        if contact:
            self.trigger_force_time = self.reader.end
            self.actual_before = deepcopy(self.previous_actual)
            self.actual_at_threshold = dict(timestamp=self.robot.timestamp,
                velocity=self.robot.tcp_speed.tolist(), pose=self.robot.pose.tolist())
        if (report or {}).get('api_anomaly'):
            self.fault = 'braking API failed or rejected the command'
        if self.robot is not None:
            try:
                self._event('FIRST_THRESHOLD_STOP_REQUEST' if contact else
                            'USER_STOP' if reason == 'USER_ABORT' else 'BUDGET_STOP',
                            self.threshold_time if contact else self.braking_time)
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

    def settle(self):
        self.state = 'SETTLING'
        deadline = self.clock()+self.s['stop_timeout_sec']
        while self.clock() < deadline:
            cycle = self.clock()
            if self.poll() in ('Q', 'ESC'):
                self.fault = self.fault or 'operator aborted during braking; still monitoring actual stop'
            try:
                stamp = self._observe(diagnostic=True)
            except Exception as exc:
                self.fault = self.fault or f'{type(exc).__name__}: {exc}'
                # A sensor failure must not prevent TCP braking telemetry.
                self.force_failed = True
                self.force_valid = False
                self.robot = self.controller.read_diagnostic_state()
                stamp = self.clock()
            stopped = self.controller.poll_stop()
            if self.stopped_time is not None and not stopped:
                self.fault = self.fault or 'actual standstill was revoked during post-stop recording'
            if self.force_failed:
                self.fault = self.fault or 'force sensor failed during settling'
            if stopped and self.stopped_time is None:
                # poll_stop can perform a fresh mode-exit read. Record a TCP
                # observation after that operation as the confirmed stop pose.
                self.robot = self.controller.read_diagnostic_state()
                stamp = self.clock()
                if not 0 <= stamp-self.robot.timestamp <= self.s['max_robot_age_sec']:
                    raise RobotError('stop confirmation TCP sample is stale')
                if (np.linalg.norm(self.robot.tcp_speed[:3]) > self.s['settle_speed_mps'] or
                    np.linalg.norm(self.robot.tcp_speed[3:]) > .005):
                    raise RobotError('controller stop confirmation disagrees with actual speed')
                self.stopped_time = stamp
                self.stop_pose = self.robot.pose.tolist()
                try:
                    self._event('EXECUTION_STOP_CONFIRMED', stamp)
                except Exception as exc:
                    self.fault = self.fault or f'stop event logging failed: {exc}'
            try:
                self._kick()
            except Exception as exc:
                self.fault = self.fault or f'watchdog failed during braking: {exc}'
            try:
                self._record(stamp)
            except Exception as exc:
                self.fault = self.fault or f'braking telemetry logging failed: {exc}'
            if stopped and self.stopped_time is not None and stamp-self.stopped_time >= self.s['post_stop_record_sec']:
                self.state = 'FINISHED'
                return
            self.sleep(max(0., 1/self.s['control_rate_hz']-(self.clock()-cycle)))
        raise RobotError('actual TCP did not confirm and remain stopped within stop budget')

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
                    not self.fault else 'no_contact' if self.reason.startswith('NO_CONTACT') and
                    self.stopped_time is not None and not self.fault else 'aborted',
            state=self.state, reason=self.reason, fault=self.fault, p0_name=self.name, p0=self.p0.tolist(),
            P_ref=self.calibration['P_ref'], direction_xy=self.direction.tolist(), nominal_speed_mps=self.speed,
            contact_threshold_N=self.s['contact_threshold_N'], threshold_trigger_sec=self.threshold_time,
            force_trigger_sample_sec=self.trigger_force_time,
            braking_request_sec=self.braking_time, actual_standstill_sec=self.stopped_time,
            actual_before_threshold=self.actual_before, actual_at_threshold=self.actual_at_threshold,
            actual_stop_pose=self.stop_pose,
            peak_filtered_fxy_N=self.peak_filtered, peak_unfiltered_fxy_N=self.peak_unfiltered,
            actual_search_path_m=self.path_distance,
            time_basis='host monotonic; force request interval, actual TCP read, command send sampled separately',
            stop_report=deepcopy(getattr(self.controller, 'stop_report', None)))
