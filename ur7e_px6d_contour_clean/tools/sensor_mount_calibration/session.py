"""Finite acquisition state machine; any exception revokes motion authorization."""

import time
import threading
from collections import deque
import numpy as np

from robot.rtde_controller import _orientation_distance
from .plan import at_target
from .stability import StabilityLimits, check_stationary_trend, make_reference, check_reference


class CalibrationStopped(RuntimeError):
    pass


class Session:
    def __init__(self, hardware, store, keyboard, limits, *, clock=time.monotonic, sleep=time.sleep):
        self.hardware, self.store, self.keyboard, self.limits = hardware, store, keyboard, limits
        self.clock, self.sleep = clock, sleep
        self.records = []
        self.started = None
        self.origin = None
        self.last_stamp = None
        self.failed = False
        self.last_observation_timing = {}
        self.progress = None
        self.stage = 'waiting'
        self.reference = None
        self.reference_checks = []
        self.motion_context = None
        self.stability_limits = StabilityLimits()

    def emit(self, message, *, key=None, force=False):
        if self.progress is not None:
            self.progress.emit(message, key=key, force=force)

    def _abort(self, poll_keyboard=True):
        if self.failed:
            raise CalibrationStopped('session fault is latched')
        if poll_keyboard and self.keyboard.poll() in ('Q', 'ESC'):
            raise CalibrationStopped('operator stop')
        if self.started is not None and self.clock()-self.started > self.limits.total_timeout_sec:
            raise CalibrationStopped('total execution deadline exceeded')
        self.store.check_logging_health()

    def observe(self, split='motion', pose_id=-1, *, poll_keyboard=True):
        self._abort(poll_keyboard)
        before = self.clock()
        timing = dict(kind='observation', phase=split, pose_id=pose_id,
                      observe_started_monotonic=before, stage='hardware_read',
                      observation_limit_sec=self.limits.observation_timeout_sec)
        record = None
        try:
            record = dict(self.hardware.read())
            timing.update(hardware_read_wall_sec=self.clock()-before,
                          robot_timestamp=record['robot_timestamp'],
                          sample_host_monotonic=record['host_monotonic'])
            for key in ('sensor_read_duration_sec', 'rtde_read_duration_sec',
                        'watchdog_duration_sec', 'hardware_read_duration_sec'):
                if key in record:
                    timing[key] = record[key]
            self._abort(poll_keyboard)
            if self.clock()-before > self.limits.observation_timeout_sec:
                raise CalibrationStopped('observation deadline exceeded')
            record.update(split=split, pose_id=pose_id)
            if self.motion_context is not None:
                record['plan_index'] = self.motion_context['plan_index']
            # Real execution enqueues an immutable record; worker owns ALL disk I/O.
            timing['stage'] = 'raw_enqueue'
            enqueue_started = self.clock()
            self.store.append_raw(record)
            timing['raw_enqueue_duration_sec'] = self.clock()-enqueue_started
            self._abort(poll_keyboard)
            if (self.clock()-before > self.limits.observation_timeout_sec or
                    self.clock()-float(record['host_monotonic']) > self.limits.observation_timeout_sec):
                raise CalibrationStopped('observation became stale during raw logging enqueue')
            timing['stage'] = 'safety_checks'
            checks_started = self.clock()
            pose = np.asarray(record['actual_tcp_pose'], float)
            speed = np.asarray(record['tcp_speed'], float)
            if (pose.shape != (6,) or speed.shape != (6,) or
                    not np.isfinite(pose).all() or not np.isfinite(speed).all()):
                raise CalibrationStopped('nonfinite robot observation')
            stamp = float(record['robot_timestamp'])
            if not np.isfinite(stamp) or (self.last_stamp is not None and stamp <= self.last_stamp):
                raise CalibrationStopped('RTDE packet did not advance; stale data rejected')
            self.last_stamp = stamp
            if (np.linalg.norm(speed[:3]) > self.limits.max_linear_speed or
                    np.linalg.norm(speed[3:]) > self.limits.max_angular_speed):
                raise CalibrationStopped('measured TCP speed exceeds calibration limit')
            if self.origin is not None:
                if (np.linalg.norm(pose[:3]-self.origin[:3]) > self.limits.max_translation_m or
                        _orientation_distance(pose[3:], self.origin[3:]) > self.limits.max_tilt_rad):
                    raise CalibrationStopped('actual TCP left displayed calibration envelope')
            timing.update(safety_checks_duration_sec=self.clock()-checks_started,
                          observe_to_timing_enqueue_sec=self.clock()-before, stage='timing_enqueue')
            self.store.append_timing(timing)
            self._abort(poll_keyboard)
            # Even queue/diagnostic work must not turn a stale read into permission.
            if (self.clock()-before > self.limits.observation_timeout_sec or
                    self.clock()-float(record['host_monotonic']) > self.limits.observation_timeout_sec):
                raise CalibrationStopped('observation deadline exceeded after timing enqueue')
            self.last_observation_timing = dict(timing, stage='complete',
                                                observe_duration_sec=self.clock()-before)
            return record
        except BaseException as exc:
            timing.update(kind='observation_failure', error=f'{type(exc).__name__}: {exc}',
                          observe_duration_sec=self.clock()-before)
            if record is not None:
                timing['sample_age_sec'] = self.clock()-float(record['host_monotonic'])
            self.last_observation_timing = dict(timing)
            exc.diagnostics = {**getattr(exc, 'diagnostics', {}), 'observation_timing': dict(timing)}
            try:
                self.store.append_timing(timing)
            except BaseException as log_exc:
                exc.add_note(f'failure timing could not be queued: {log_exc}')
            raise

    def stopped(self, record):
        speed = np.asarray(record['tcp_speed'])
        return (np.linalg.norm(speed[:3]) <= self.limits.settle_linear_speed
                and np.linalg.norm(speed[3:]) <= self.limits.settle_angular_speed
                and np.max(np.abs(record['actual_qd'])) <= self.limits.settle_angular_speed)

    def _stability_check(self, check, *args, target, deadline=None):
        """Compute on a fixed snapshot while the owner keeps reading fresh data.

        The worker performs pure numerical work only. It never owns hardware,
        logging, or motion permission. A late worker cannot resume a failed run.
        """
        outcome, done = {}, threading.Event()
        deadline = min(self.clock()+5., deadline if deadline is not None else float('inf'))

        def calculate():
            try:
                outcome['result'] = check(*args)
            except BaseException as exc:
                outcome['error'] = exc
            finally:
                done.set()

        self._abort()
        threading.Thread(target=calculate, name='mount-stability-check', daemon=True).start()
        while True:
            record = self.observe('stability')
            if not self.stopped(record) or not at_target(record['actual_tcp_pose'], target, self.limits):
                raise CalibrationStopped('robot moved during stationary stability check')
            if self.clock() >= deadline:
                raise CalibrationStopped('stationary stability calculation deadline exceeded')
            # A bounded yield lets numerical work progress even with an injected
            # test clock; every decision still follows a new robot/sensor read.
            if done.wait(.001):
                self._abort()
                if self.clock() >= deadline:
                    raise CalibrationStopped('stationary stability calculation deadline exceeded')
                if 'error' in outcome:
                    raise outcome['error']
                return outcome['result']
            self.emit('正在检查原始力稳定性；持续监测新鲜力/矩与机器人状态。', key='stability')
            self.sleep(self.limits.sample_period_sec)

    def settle(self, target, deadline, *, label='当前姿态'):
        low_since = None
        while self.clock() < deadline:
            record = self.observe('settling')
            low = self.stopped(record) and at_target(record['actual_tcp_pose'], target, self.limits)
            low_since = (self.clock() if low_since is None else low_since) if low else None
            position_error = np.linalg.norm(np.asarray(record['actual_tcp_pose'])[:3]-np.asarray(target)[:3])
            angle_error = np.degrees(_orientation_distance(record['actual_tcp_pose'][3:], target[3:]))
            held = self.clock()-low_since if low_since is not None else 0.
            self.stage = 'settling' if low else 'moving'
            self.emit(f'{label}：{"停稳确认" if low else "移动中"}，位置误差 {position_error*1000:.3f}mm，'
                      f'角度误差 {angle_error:.2f}°，停稳 {held:.1f}/{self.limits.settle_hold_sec:.1f}s，'
                      f'本段剩余 {max(0., deadline-self.clock()):.0f}s。', key='motion')
            if low_since is not None and self.clock()-low_since >= self.limits.settle_hold_sec:
                self.emit(f'{label}：已停稳。', force=True)
                return
            self.sleep(self.limits.sample_period_sec)
        raise CalibrationStopped('segment/standstill deadline exceeded')

    def acquire(self, point, deadline, *, label='当前姿态'):
        samples = []
        self.stage = 'reference' if point['split'] == 'reference' else 'sampling'
        self.emit(f'{label}：开始原始力采样 0/{self.limits.samples_per_pose}。', force=True)
        for index in range(self.limits.samples_per_pose):
            if self.clock() >= deadline:
                raise CalibrationStopped('segment acquisition deadline exceeded')
            self.sleep(self.limits.sample_period_sec)
            record = self.observe(point['split'], point['pose_id'])
            if not self.stopped(record) or not at_target(record['actual_tcp_pose'], point['target'], self.limits):
                raise CalibrationStopped('pose moved during stationary acquisition')
            samples.append(record)
            if point['split'] in ('fit', 'validation'):
                self.records.append(record)
            if (index+1) % 20 == 0:
                self.emit(f'{label}：采样 {index+1}/{self.limits.samples_per_pose}。', key='sampling')
        self.emit(f'{label}：采样完成 {len(samples)}/{self.limits.samples_per_pose}。', force=True)
        return samples

    def run(self, start, plan):
        self.origin = np.asarray(start, dtype=float)
        self.started = self.clock()
        try:
            self.stage = 'activating'
            self.hardware.progress = self.progress
            self.emit('开始自动标定：检查停稳并激活控制。', force=True)
            # Activation/IK are authorized by the same fresh Enter as this finite plan.
            current = self.observe()
            if not at_target(current['actual_tcp_pose'], start, self.limits) or not self.stopped(current):
                raise CalibrationStopped('robot moved since displayed plan; generate a new plan')
            self.hardware.activate()
            self.stage = 'preflight'
            preflight_samples = deque(maxlen=6000)

            def heartbeat():
                record = self.observe('preflight')
                if not self.stopped(record) or not at_target(record['actual_tcp_pose'], start, self.limits):
                    raise CalibrationStopped('robot moved during stationary preflight')
                preflight_samples.append(record)
                return record

            preflight = self.hardware.preflight([point['target'] for point in plan[1:]], heartbeat)
            self.store.write_metadata({'preflight': preflight})
            self.stage = 'stationary_trend'
            self.emit('检查预检期间同一姿态的原始力趋势，确认固定零偏稳定性。', force=True)
            trend = self._stability_check(check_stationary_trend, list(preflight_samples),
                                          self.stability_limits, target=start)
            self.store.write_metadata({'preflight_stability': trend})
            if trend.get('status') == 'insufficient_duration':
                self.emit('原地趋势窗口不足，后续每次回中均检查原始力重复性。', force=True)
            elif trend.get('status') == 'within_coarse_budget':
                self.emit(f'粗标定原地趋势：{trend["force_slope_norm_N_per_s"]*60:.4f} N/min，'
                          f'首尾变化 {trend["endpoint_force_delta_norm_N"]:.4f} N。', force=True)
            else:
                self.emit('原地趋势检查通过；开始多姿态采样。', force=True)
            for warning in trend.get('warnings', []):
                self.emit('质量提示：' + warning, force=True)
            total = sum(bool(point['acquire']) for point in plan)
            completed = 0
            previous_acquisition = None
            for plan_index, point in enumerate(plan):
                self.motion_context = dict(plan_index=plan_index, target_pose_id=point['pose_id'],
                                           is_return=not point['acquire'], completed_poses=completed,
                                           total_poses=total)
                if not point['acquire'] and previous_acquisition is not None:
                    self.motion_context.update(after_pose_id=previous_acquisition['pose_id'],
                                               tilt_deg=previous_acquisition['tilt_deg'],
                                               azimuth_deg=previous_acquisition['azimuth_deg'])
                self._abort()
                record = self.observe()
                deadline = self.clock() + self.limits.segment_timeout_sec
                if point['acquire']:
                    purpose = '独立验证' if point['split'] == 'validation' else '拟合'
                    label = f'姿态 {completed+1}/{total}，倾角 {point["tilt_deg"]:g}°、方向 {point["azimuth_deg"]:g}°（{purpose}）'
                else:
                    label = f'姿态 {completed}/{total} 后返回起始姿态'
                self.emit(label, force=True)
                if not at_target(record['actual_tcp_pose'], point['target'], self.limits):
                    self._abort()
                    self.hardware.command(point['target'])
                self.settle(point['target'], deadline, label=label)
                if point['acquire']:
                    samples = self.acquire(point, deadline, label=label)
                    completed += 1
                    previous_acquisition = point
                    if self.reference is None:
                        self.reference = self._stability_check(make_reference, samples,
                                                               target=start, deadline=deadline)
                        self.store.write_metadata({'stability_reference': self.reference})
                    self.emit(f'采集进度 {completed}/{total}（{100*completed/total:.0f}%），已用 {self.clock()-self.started:.1f}s。', force=True)
                else:
                    reference_point = dict(point, split='reference', pose_id=1000+completed)
                    samples = self.acquire(reference_point, deadline, label='回中重复性检查')
                    check = self._stability_check(check_reference, self.reference, samples, self.stability_limits,
                                                 target=point['target'], deadline=deadline)
                    check['motion_context'] = dict(self.motion_context)
                    self.reference_checks.append(check)
                    self.store.write_metadata({'reference_checks': self.reference_checks})
                    self.emit(f'回中重复性通过：原始力变化 {check["force_delta_norm_N"]:.4f} N / '
                              f'{check["force_limit_N"]:.2f} N；参考力仅用于检查。', force=True)
                    for warning in check.get('warnings', []):
                        self.emit('质量提示：' + warning, force=True)
            self.stage = 'acquisition_complete'
            self.emit(f'全部 {total} 个姿态采集完成，已回到起始标定姿态。', force=True)
            return self.records
        except BaseException as exc:
            self.failed = True
            try:
                self.hardware.stop()
            except BaseException as stop_exc:
                exc.add_note(f'stop request also failed: {stop_exc}')
            exc.diagnostics = {**getattr(exc, 'diagnostics', {}), 'calibration_stage': self.stage,
                               'motion_context': self.motion_context}
            self.emit(f'标定中止（{self.stage}）：{exc}；已请求停止，不自动返回。', force=True)
            raise
