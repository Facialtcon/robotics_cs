"""Experimental velocity force feedback; no shape inference or discrete probes.

The signed force direction is only an interaction-direction estimate. Friction
can contaminate it; +1 being inward is a simulator convention, not a PX6D fact.
"""
from __future__ import annotations

from enum import Enum
from collections import deque
import numpy as np

from core.models import PolicyCommand, PolicyWaypoint
from experiment_logging.termination import TerminationReason, classify_stop_reason
from policy.boundary_estimation import handed_tangent, unit
from safety.force_guard import ForceRateGuard, force_safety_reason, continuous_force_reason
from sensor.force_direction import control_directions, normal_feedback_speed
from sensor.force_preprocess import IndependentForceKalman

# Conservative software budgets, not measured contact stiffness. At 0.5 mm/s,
# 0.2 mm needs >=0.4 s plus acceleration/filter settling; never use the old
# 0.5 s warning as proof of wrong physical sign. Total budgets never restart.
UNLOAD_DEFAULTS = dict(unload_resume_force=2.0, unload_resume_hold_sec=.1,
    unload_force_window_sec=.25, unload_evaluation_sec=1.5,
    unload_min_displacement_m=.0002, unload_max_displacement_m=.001,
    unload_max_time_sec=3., unload_resume_ramp_sec=.5)


class State(str, Enum):
    READY = "READY"
    TARGET_SEARCH = "TARGET_SEARCH"
    FIRST_CONTACT = "FIRST_CONTACT"
    CONTINUOUS_TRACKING = "CONTINUOUS_TRACKING"
    DIRECTION_RECONFIRM = "DIRECTION_RECONFIRM"
    CONTACT_LOST = "CONTACT_LOST"
    LOCAL_REACQUIRE = "LOCAL_REACQUIRE"
    STOP = "STOP"


EXTRA_SAMPLE_FIELDS = (
    "filtered_fxy", "force_direction_x", "force_direction_y",
    "contact_direction_x", "contact_direction_y", "force_reference", "force_error",
    "v_t", "v_n", "command_vx", "command_vy", "command_speed",
    "contact_lost_timer", "follow_hand", "force_direction_sign", "force_rate",
    "reacquire_heading_deg",
)


def validate_config(config: dict) -> None:
    c, p = config["continuous_tracking"], config["policy"]
    u = {**UNLOAD_DEFAULTS, **c}
    if any(not np.isfinite(float(u[k])) or float(u[k]) <= 0 for k in UNLOAD_DEFAULTS):
        raise ValueError('unload budgets must be finite and positive')
    if not (c['force_reference']+c['force_deadband'] <= u['unload_resume_force'] < c['overload_tangent_zero_force']):
        raise ValueError('unload resume threshold must precede tangent-zero threshold')
    filter_settle = IndependentForceKalman(config['preprocessing'].get('kalman', {})).settling_time(
        p['control_rate_hz'])
    minimum_time = u['unload_min_displacement_m']/c['normal_speed_limit'] + filter_settle + u['unload_force_window_sec']
    if not (minimum_time <= u['unload_evaluation_sec'] < u['unload_max_time_sec'] and
            u['unload_min_displacement_m'] < u['unload_max_displacement_m']):
        raise ValueError('unload evaluation must allow measurable displacement and filter settling within finite budgets')
    if c.get("enabled") is not True:
        raise ValueError("continuous_tracking.enabled must be true for this entry point")
    positive = ("tangential_speed", "force_reference", "force_gain", "normal_speed_limit",
                "search_speed", "search_max_distance", "search_max_time_sec",
                "contact_lost_threshold", "contact_lost_hold_sec", "reacquire_speed",
                "reacquire_max_angle_deg", "reacquire_angular_speed_deg_s",
                "reacquire_max_time_sec", "reacquire_max_distance",
                "visualization_fps", "max_sample_gap_sec", "max_observation_age_sec",
                "cycle_timeout_sec", "settle_speed_mps", "settle_hold_sec", "confirmation_timeout_sec",
                "direction_min_force", "direction_min_filtered_force", "direction_jump_deg",
                "direction_rate_deg_s", "command_acceleration", "overload_tangent_zero_force",
                "overload_stall_sec", "overload_improvement_force", "memory_max_age_sec",
                "reacquire_radius", "reacquire_max_path", "reacquire_position_gain",
                "reacquire_reference_freeze_error", "reacquire_max_tracking_error",
                "boundary_margin", "reacquire_min_progress", "watchdog_frequency_hz",
                "direction_slow_rate_deg_s", "direction_slow_residual_deg", "direction_reconfirm_residual_deg",
                "direction_reconfirm_rate_deg_s", "direction_reversal_deg", "direction_confirm_hold_sec",
                "direction_confirm_spread_deg", "direction_min_progress", "direction_resume_scale",
                "direction_resume_sec", "direction_resume_distance")
    for key in positive:
        if not np.isfinite(float(c[key])) or float(c[key]) <= 0:
            raise ValueError(f"continuous_tracking.{key} must be finite and positive")
    if config.get('continuous_real_execution'):
        for key in ('startup_speed_mps', 'startup_angular_speed_rad_s', 'startup_position_tolerance',
                    'hard_z_drift_m', 'hard_orientation_drift_rad'):
            if not np.isfinite(float(c[key])) or float(c[key]) <= 0:
                raise ValueError(f'continuous_tracking.{key} must be finite and positive')
    budget = c['max_runtime_sec']
    if budget is not None and (isinstance(budget, bool) or not np.isfinite(float(budget)) or float(budget) <= 0):
        raise ValueError('continuous_tracking.max_runtime_sec must be null or finite and positive')
    if not 0 <= float(c["force_deadband"]) < float(c["force_reference"]):
        raise ValueError("force_deadband must be in [0, force_reference)")
    if not 0 <= float(c["force_direction_filter_alpha"]) < 1:
        raise ValueError("force_direction_filter_alpha must be in [0, 1)")
    if isinstance(c["force_direction_sign"], bool) or c["force_direction_sign"] not in (-1, 1):
        raise ValueError("force_direction_sign must be +1 or -1")
    if p["follow_hand"] not in ("CLOCKWISE", "COUNTERCLOCKWISE"):
        raise ValueError("invalid follow_hand")
    for key in ("contact_threshold", "contact_hold_time", "control_rate_hz", "force_rate_limit",
                "safety_force_threshold", "safety_torque_threshold",
                "absolute_raw_force_threshold", "absolute_raw_torque_threshold"):
        if not np.isfinite(float(p[key])) or float(p[key]) <= 0:
            raise ValueError(f"policy.{key} must be finite and positive")
    cap = float(config["robot"]["max_tcp_speed"])
    if not np.isfinite(cap) or cap <= 0:
        raise ValueError("invalid robot.max_tcp_speed")
    for key in ("tangential_speed", "normal_speed_limit", "search_speed", "reacquire_speed"):
        if float(c[key]) > cap:
            raise ValueError(f"continuous_tracking.{key} exceeds robot.max_tcp_speed")
    if float(c["search_speed"]) > float(p["search_speed"]) or float(c["reacquire_speed"]) > float(p["recovery_probe_speed"]):
        raise ValueError("continuous search/reacquire must not exceed existing speeds")
    if float(c["search_max_distance"]) > float(p["max_search_distance"]):
        raise ValueError("continuous search distance exceeds existing search budget")
    if not (float(c["contact_lost_threshold"]) < float(p["contact_threshold"])
            <= float(c["force_reference"]) - float(c["force_deadband"])):
        raise ValueError("require lost_threshold < contact_threshold <= reference - deadband")
    if float(c["force_reference"]) + float(c["force_deadband"]) >= float(p["safety_force_threshold"]):
        raise ValueError("reference + deadband must be below processed force safety limit")
    if float(c["reacquire_max_angle_deg"]) > 90:
        raise ValueError("local reacquire half-angle must not exceed 90 degrees")
    if not isinstance(c["reacquire_enabled"], bool):
        raise ValueError("reacquire_enabled must be boolean")
    if not 0 < float(c["direction_min_coherence"]) <= 1:
        raise ValueError("direction_min_coherence must be in (0, 1]")
    if not 0 < float(c["direction_jump_deg"]) < 90:
        raise ValueError("direction_jump_deg must be in (0, 90)")
    if not (c['direction_slow_residual_deg'] < c['direction_reconfirm_residual_deg'] <= c['direction_jump_deg']
            < c['direction_reversal_deg'] < 180):
        raise ValueError('invalid direction slow/reconfirm/reversal thresholds')
    if not (c['direction_slow_rate_deg_s'] < c['direction_reconfirm_rate_deg_s']
            and 0 < c['direction_resume_scale'] <= 1
            and p['contact_hold_time'] <= c['direction_confirm_hold_sec'] < c['confirmation_timeout_sec']):
        raise ValueError('invalid direction confirmation/ramp parameters')
    if type(c['direction_reconfirm_max_attempts']) is not int or c['direction_reconfirm_max_attempts'] < 1:
        raise ValueError('direction_reconfirm_max_attempts must be a positive integer')
    if not (c['direction_confirm_spread_deg'] < c['direction_jump_deg']
            and c['direction_resume_distance'] < c['direction_min_progress']):
        raise ValueError('invalid direction spread / progress window')
    if not (float(c["direction_min_filtered_force"]) <= float(c["direction_min_force"])
            <= float(c["contact_lost_threshold"])):
        raise ValueError("direction force thresholds must not exceed lost threshold")
    if not (float(c["force_reference"]) + float(c["force_deadband"])
            < float(c["overload_tangent_zero_force"]) < float(p["safety_force_threshold"])):
        raise ValueError("overload slowing must precede the hard force threshold")
    if float(c["command_acceleration"]) > float(config["robot"]["speed_acceleration"]):
        raise ValueError("command acceleration exceeds existing robot acceleration")
    if not (float(c["reacquire_reference_freeze_error"]) < float(c["reacquire_max_tracking_error"])
            < float(c["reacquire_max_distance"]) - float(c["boundary_margin"])):
        raise ValueError("invalid recovery tracking-error budgets")
    if not float(c["boundary_margin"]) < min(float(c["reacquire_max_distance"]), float(c["reacquire_max_path"])):
        raise ValueError("boundary margin consumes recovery budget")
    if float(c["settle_hold_sec"]) >= float(c["confirmation_timeout_sec"]):
        raise ValueError("confirmation timeout must exceed settle hold")
    if float(c["max_sample_gap_sec"]) < 1 / float(p["control_rate_hz"]):
        raise ValueError("sample gap must cover nominal control period")
    unit(p["search_direction_xy"])



def angle_between(a, b):
    """Signed shortest angle, including crossings of +/-pi."""
    return float(np.arctan2(a[0]*b[1]-a[1]*b[0], np.dot(a, b)))


def arc_reference(origin, normal, tangent, radius, theta):
    return np.asarray(origin)[:2] + radius * (np.sin(theta)*normal + (np.cos(theta)-1)*tangent)


DIAGNOSTIC_FIELDS = (
    "cycle_dt", "direction_valid", "direction_confidence", "direction_delta_deg", "direction_limited",
    "measurement_direction_x", "measurement_direction_y", "stop_requested", "stop_confirmed",
    "contact_hold_elapsed", "settle_hold_elapsed", "reacquire_origin_x", "reacquire_origin_y",
    "reacquire_reference_x", "reacquire_reference_y", "reacquire_tracking_error", "reacquire_path_length",
    "reacquire_speed_path_length", "reacquire_raw_pose_path_length", "reacquire_max_displacement",
    "memory_timestamp", "memory_confidence", "limit_reason",
    "memory_x", "memory_y", "memory_normal_x", "memory_normal_y", "loss_detection_x", "loss_detection_y",
    "reacquire_normal_x", "reacquire_normal_y", "reacquire_tangent_x", "reacquire_tangent_y",
)
EXTRA_SAMPLE_FIELDS += DIAGNOSTIC_FIELDS
DIRECTION_FIELDS = ('measurement_jump_deg', 'estimate_residual_deg', 'measurement_rate_deg_s',
                    'direction_rate_filtered_deg_s', 'direction_speed_scale', 'direction_reconfirm_elapsed',
                    'direction_reconfirm_count', 'direction_reconfirm_attempts', 'direction_resume_active',
                    'direction_resume_progress_m', 'direction_phase')
EXTRA_SAMPLE_FIELDS += DIRECTION_FIELDS
EXTRA_SAMPLE_FIELDS += ('force_rate_guard_active',)
UNLOAD_FIELDS = ('tangent_limit_reason', 'unload_active', 'unload_elapsed_sec',
                'unload_displacement_m', 'unload_projected_displacement_m',
                'unload_force_mean_N', 'unload_force_improvement_N', 'unload_force_slope_N_s',
                'unload_resume_scale', 'settle_hold_start_host_monotonic',
                'tcp_displacement_from_contact_m', 'actual_v_n_mps', 'actual_v_t_mps')
EXTRA_SAMPLE_FIELDS += UNLOAD_FIELDS


class ContinuousTrackingPolicy:
    def __init__(self, config):
        validate_config(config)
        self.c, self.p = {**UNLOAD_DEFAULTS, **config['continuous_tracking']}, dict(config['policy'])
        self.real_execution = bool(config.get('continuous_real_execution'))
        self.force_transform_status = dict(config.get('force_transform_status') or {})
        # Configuration is known before any device connects. Do not let SEARCH
        # move and only discover a missing installation when contact is made.
        if self.real_execution and self.force_transform_status.get('available') is False:
            raise ValueError('Base force transform unavailable before motion: '
                f"{self.force_transform_status.get('reason', 'unknown installation or TCP orientation')}. "
                'Set preprocessing.coordinate_transform.rotation_sensor_to_tool '
                '(measured PX6D axes in active TCP coordinates), or a measured '
                'rotation_sensor_to_base together with its reference_tool_orientation. '
                'A placeholder identity and the probe axis alone are not installation calibration.')
        self.diagnostics = {}
        self.search_geometry = config.get('continuous_search_geometry')
        self.execution_settled = None
        self.max_speed = float(config['robot']['max_tcp_speed'])
        self.nominal_dt = 1 / float(self.p['control_rate_hz'])
        self.dt = self.nominal_dt
        self.follow_hand = self.p['follow_hand']
        self.search_direction = unit(self.p['search_direction_xy'])
        self.state, self.reason, self.stop_reason = State.READY, '', None
        self.events = []
        self.initial_contact = self.first_threshold_pose = self.last_contact_pose = None
        self.last_reliable_tangent = self.last_contact_direction = None
        self.last_reliable_contact = None
        self.tracking_pose = self.loss_detection_pose = self.reacquire_origin = None
        self.tracking_velocity = np.zeros(2)
        self.reacquire_reference = None
        self.force_direction = np.zeros(2)
        self.contact_direction = np.zeros(2)
        self.tangent = np.zeros(2)
        self.measurement_direction = np.zeros(2)
        self._filtered = None
        self._filtered_magnitude = 0.
        self._previous_measurement = None
        self._velocity = np.zeros(2)
        self._rate = ForceRateGuard()
        self.force_rate_guard_active = False
        self._last_time = self._started = self._lost = self._reacquire = None
        self._start_pose = None
        self._confirm_started = self._force_since = self._settle_since = None
        self._low_force_pending = False
        self._last_valid_measurement = self._last_valid_measurement_time = None
        self.measurement_jump_deg = self.estimate_residual_deg = self.measurement_rate_deg_s = 0.
        self.direction_rate_filtered_deg_s = 0.
        self.direction_speed_scale = 1.
        self.direction_reconfirm_count = self.direction_reconfirm_attempts = 0
        self._direction_attempt_pose = None
        self._direction_resume_started = self._direction_resume_pose = self._direction_resume_tangent = None
        self.direction_resume_progress_m = 0.
        self._confirmation_vectors = deque()
        self._last_recovery_origin = None
        self._recovery_count = 0
        self._recovery_previous_pose = None
        self._recovery_previous_observation_time = None
        self._recovery_previous_speed = 0.
        self._memory_normal = self._memory_tangent = None
        self._theta = 0.
        self.fxy = self.filtered_fxy = self.force_rate = self.lost_timer = 0.
        self.v_t = self.v_n = self.reacquire_heading_deg = 0.
        self.contact_flag = self.direction_valid = self.stop_requested = self.stop_confirmed = False
        self.direction_confidence = self.direction_delta_deg = 0.
        self.direction_limited = False
        self.contact_hold_elapsed = self.settle_hold_elapsed = 0.
        self.reacquire_tracking_error = self.reacquire_path_length = 0.
        self.reacquire_speed_path_length = self.reacquire_raw_pose_path_length = 0.
        self.reacquire_max_displacement = 0.
        self.unload_active = False
        self._unload_started = self._unload_release_since = self._unload_resumed = None
        self._unload_resume_announced = False
        self._unload_pose = self._unload_direction = None
        self._force_window = deque()
        self.unload_elapsed = self.unload_displacement = self.unload_projected = 0.
        self.unload_force_mean = self.unload_improvement = self.unload_force_slope = 0.
        self.unload_resume_scale = 1.
        self.tangent_limit_reason = 'READY'
        self._contact_low_speed_seen = False

    def _begin_unload(self, now, pose):
        self.unload_active = True
        self._unload_started, self._unload_pose = now, pose[:2].copy()
        self._unload_direction = -self.contact_direction.copy()
        self._unload_initial_force = self.fxy  # Already EMA-filtered; do not average in pre-contact low force.
        self._unload_release_since = self._unload_resumed = None
        self.unload_displacement = self.unload_projected = self.unload_elapsed = self.unload_improvement = 0.
        self._event(now, pose, 'UNLOAD_STARTED')

    def _observe_unload(self, now, pose):
        self._force_window.append((now, self.fxy))
        while len(self._force_window) > 1 and self._force_window[0][0] < now-self.c['unload_force_window_sec']:
            self._force_window.popleft()
        values = np.asarray(self._force_window)
        self.unload_force_mean = float(np.mean(values[:, 1]))
        centered = values[:, 0]-np.mean(values[:, 0])
        self.unload_force_slope = float(centered @ (values[:, 1]-self.unload_force_mean) /
            (centered @ centered)) if centered @ centered > 1e-12 else 0.
        if not self.unload_active:
            return
        self.unload_elapsed = now-self._unload_started
        delta = pose[:2]-self._unload_pose
        self.unload_displacement = max(self.unload_displacement, float(np.linalg.norm(delta)))
        self.unload_projected = float(delta @ self._unload_direction)
        self.unload_improvement = self._unload_initial_force-self.unload_force_mean
        reason = code = None
        if (self.unload_displacement >= self.c['unload_max_displacement_m'] or
                self.unload_elapsed >= self.c['unload_max_time_sec']):
            reason, code = 'unload total time/displacement budget exhausted', TerminationReason.STOP_UNLOAD_BUDGET
        elif self._release_unload_if_ready(now, pose):
            return
        elif self.unload_elapsed >= self.c['unload_evaluation_sec']:
            if self.unload_projected < self.c['unload_min_displacement_m']:
                reason, code = 'unload: insufficient actual TCP displacement', TerminationReason.STOP_UNLOAD_NO_MOTION
            elif self.unload_improvement < self.c['overload_improvement_force']:
                reason, code = 'unload: TCP moved but smoothed resultant force did not improve', TerminationReason.STOP_UNLOAD_INEFFECTIVE
        if reason:
            self._event(now, pose, code.value)
            self.request_stop(now, pose, reason, code=code)

    def _release_unload_if_ready(self, now, pose):
        low = self.fxy <= self.c['unload_resume_force'] and self.unload_force_mean <= self.c['unload_resume_force']
        self._unload_release_since = (now if self._unload_release_since is None else self._unload_release_since) if low else None
        if self._unload_release_since is None or now-self._unload_release_since < self.c['unload_resume_hold_sec']:
            return False
        self.unload_active = False
        self._unload_resumed, self._unload_resume_announced = now, False
        self._event(now, pose, 'UNLOAD_FORCE_RELEASED')
        return True

    def _unload_tangent_scale(self, now, pose):
        if not self.unload_active and self.fxy >= self.c['overload_tangent_zero_force']:
            self._begin_unload(now, pose)
        if self.unload_active:
            self.tangent_limit_reason = 'HIGH_FORCE_UNLOADING'
            return 0.
        if self._unload_resumed is not None:
            if not self._unload_resume_announced:
                self._unload_resumed, self._unload_resume_announced = now, True
                self._event(now, pose, 'TANGENT_RESUME_STARTED')
            scale = min(1., max(0., (now-self._unload_resumed)/self.c['unload_resume_ramp_sec']))
            self.tangent_limit_reason = 'UNLOAD_RECOVERY_RAMP' if scale < 1 else ''
            if scale >= 1:
                self._unload_resumed = None
                self._event(now, pose, 'TANGENT_RESUME_COMPLETED')
            return scale
        return 1.

    def _event(self, now, pose, name):
        self.events.append(PolicyWaypoint(now, self.state.value, name, pose.copy(),
                           target_direction_xy=self.contact_direction.copy(), tangent_xy=self.tangent.copy()))

    def request_stop(self, now, pose, reason, *, event='SAFETY_STOP', code=None):
        if self.state != State.STOP:
            self.state, self.reason, self.stop_reason = State.STOP, reason, code
            self._event(now, pose, event)
        if self.stop_reason is None:
            self.stop_reason = classify_stop_reason(self.reason)
        self.stop_requested = True
        self.stop_confirmed = False
        self.v_t = self.v_n = 0.
        self._velocity[:] = 0
        self.unload_active = False
        self.tangent_limit_reason = str(getattr(self.stop_reason, 'value', self.stop_reason))

    def _command(self, pose, velocity=None):
        # None is an immediate stop request, never acceleration-smoothed.
        if velocity is None:
            self._velocity[:] = 0
            self.v_t = self.v_n = 0.
        else:
            velocity = np.asarray(velocity, dtype=float)
            norm = np.linalg.norm(velocity)
            if norm > self.max_speed:
                velocity = velocity * (self.max_speed / norm)
            delta = velocity - self._velocity
            limit = float(self.c['command_acceleration']) * self.dt
            if np.linalg.norm(delta) > limit:
                delta *= limit / np.linalg.norm(delta)
            self._velocity += delta
            # Report realized components, not pre-limit desired values.
            self.v_t = float(np.dot(self._velocity, self.tangent))
            self.v_n = float(np.dot(self._velocity, self.contact_direction))
        speed = float(np.linalg.norm(self._velocity))
        return PolicyCommand(self.state.value, speed > 1e-12,
                             self._velocity / speed if speed > 1e-12 else np.zeros(2), speed,
                             pose.copy(), self.contact_flag, False, self.reason)

    def _reset_confirmation(self, now):
        self._confirm_started = now
        self._force_since = self._settle_since = None
        self._confirmation_vectors.clear()
        self.contact_hold_elapsed = self.settle_hold_elapsed = 0.
        self.stop_confirmed = False
        self._contact_low_speed_seen = False

    def _settled(self, now, robot):
        was_confirmed = self.stop_confirmed
        if np.linalg.norm(robot.tcp_speed[:3]) <= float(self.c['settle_speed_mps']):
            if self.state == State.FIRST_CONTACT and not self._contact_low_speed_seen:
                self._event(now, robot.pose, 'FIRST_CONTACT_LOW_SPEED_OBSERVED')
                self._contact_low_speed_seen = True
            if self._settle_since is None:
                self._settle_since = now
        else:
            self._settle_since = None
        self.settle_hold_elapsed = 0. if self._settle_since is None else now-self._settle_since
        self.stop_confirmed = (self.settle_hold_elapsed + 1e-12 >= float(self.c['settle_hold_sec'])
                               and self.execution_settled is not False)
        if self.stop_confirmed and not was_confirmed:
            self._event(now, robot.pose, 'STANDSTILL_CONFIRMED')
        return self.stop_confirmed

    def _confirm(self, now, robot, vector):
        self.stop_requested = True
        settled = self._settled(now, robot)
        if self.contact_flag:
            if self._force_since is None:
                self._force_since = now
            self._confirmation_vectors.append((now, vector.copy()))
            # Only the most recent continuous stable-force window initializes
            # local direction. Never touch WrenchPreprocessor or sensor bias.
            window = (float(self.c['direction_confirm_hold_sec']) if self.state == State.DIRECTION_RECONFIRM
                      else float(self.p['contact_hold_time']))
            while len(self._confirmation_vectors) > 1 and self._confirmation_vectors[1][0] < now-window:
                self._confirmation_vectors.popleft()
        else:
            self._force_since = None
            self._confirmation_vectors.clear()
        self.contact_hold_elapsed = 0. if self._force_since is None else now-self._force_since
        hold = (float(self.c['direction_confirm_hold_sec']) if self.state == State.DIRECTION_RECONFIRM
                else float(self.p['contact_hold_time']))
        if self.contact_hold_elapsed + 1e-12 >= hold and settled:
            values = np.array([v for _, v in self._confirmation_vectors])
            mean = np.mean(values, axis=0)
            magnitude = float(np.linalg.norm(mean))
            coherence = magnitude / float(np.mean(np.linalg.norm(values, axis=1)))
            if self.state == State.DIRECTION_RECONFIRM:
                spread = max(abs(np.rad2deg(angle_between(mean, v))) for v in values)
                self.direction_confidence, self.filtered_fxy = coherence, magnitude
                if spread > float(self.c['direction_confirm_spread_deg']) or coherence < float(self.c['direction_min_coherence']):
                    self._force_since = now
                    self._confirmation_vectors.clear()
                    self.contact_hold_elapsed = 0.
                    # Keep the original episode deadline; unstable windows are
                    # not new attempts and can never extend the timeout.
                    magnitude = 0.
            if magnitude >= float(self.c['direction_min_filtered_force']) and coherence >= float(self.c['direction_min_coherence']):
                candidate, tangent, _ = control_directions(
                    mean, self.c['force_direction_sign'], self.follow_hand)
                if (self.real_execution and self.state == State.FIRST_CONTACT and
                        float((robot.pose[:2]-self._start_pose[:2]) @ self.search_direction) > 1e-6):
                    # Search displacement is a consistency check, not a normal
                    # calibration. A pressing normal opposing the approach that
                    # established contact cannot authorize inward feedback. Do
                    # not flip the sign, rotate force data, or invent a normal.
                    alignment = float(candidate @ self.search_direction)
                    self.diagnostics['first_contact_search_normal_alignment'] = alignment
                    self.diagnostics['first_contact_search_normal_angle_deg'] = float(
                        np.rad2deg(np.arccos(np.clip(alignment, -1., 1.))))
                    if alignment <= 0.:
                        self.request_stop(now, robot.pose,
                            'contact pressing direction contradicts observed search approach; check force frame/sign',
                            code=TerminationReason.STOP_DIRECTION_UNCONFIRMED)
                        return False
                self._filtered, self._filtered_magnitude = mean, float(np.mean(np.linalg.norm(values, axis=1)))
                self.contact_direction, self.tangent = candidate, tangent
                self._previous_measurement = vector / self.fxy
                self.direction_valid, self.direction_confidence = True, coherence
                self.filtered_fxy = magnitude
                self._remember(now, robot.pose)
                return True
        if now-self._confirm_started >= float(self.c['confirmation_timeout_sec']):
            direction = self.state == State.DIRECTION_RECONFIRM
            if self.real_execution and not direction:
                self.diagnostics['contact_direction_confirmation'] = 'WARNING: contact/direction confirmation taking longer than configured interval'
            else:
                self.request_stop(now, robot.pose, 'direction reconfirmation timeout' if direction else 'contact/standstill confirmation timeout',
                                  code=TerminationReason.STOP_DIRECTION_UNCONFIRMED if direction else None)
        return False

    def _remember(self, now, pose):
        self.last_contact_pose = pose.copy()
        self.last_reliable_tangent = self.tangent.copy()
        self.last_contact_direction = self.contact_direction.copy()
        self.last_reliable_contact = dict(timestamp=now, pose=pose.copy(),
                                         normal=self.contact_direction.copy(), tangent=self.tangent.copy(),
                                         confidence=self.direction_confidence)

    def _direction(self, now, pose, vector):
        self.direction_valid = False
        self.direction_limited = False
        if self.fxy < float(self.c['direction_min_force']):
            self.direction_confidence = 0.
            return False
        measurement = vector / self.fxy
        if (self.measurement_jump_deg > float(self.c['direction_jump_deg']) or
                self.estimate_residual_deg > float(self.c['direction_reconfirm_residual_deg']) or
                self.direction_rate_filtered_deg_s > float(self.c['direction_reconfirm_rate_deg_s'])):
            self._begin_direction_reconfirm(now, pose, 'measurement change / estimate lag')
            return False
        alpha = float(self.c['force_direction_filter_alpha']) ** (self.dt / self.nominal_dt)
        # Existing direction/coherence estimator, downstream of Base Kalman.
        # This is not sensor-wrench preprocessing; retain the tracking strategy.
        self._filtered = vector.copy() if self._filtered is None else alpha*self._filtered+(1-alpha)*vector
        self._filtered_magnitude = alpha*self._filtered_magnitude+(1-alpha)*self.fxy
        self.filtered_fxy = float(np.linalg.norm(self._filtered))
        self.direction_confidence = self.filtered_fxy / max(self._filtered_magnitude, float(self.c['direction_min_force']))
        if (self.filtered_fxy < float(self.c['direction_min_filtered_force'])
                or self.direction_confidence < float(self.c['direction_min_coherence'])):
            self._begin_direction_reconfirm(now, pose, 'low filtered magnitude / direction coherence')
            return False
        candidate, _, _ = control_directions(self._filtered, self.c['force_direction_sign'], self.follow_hand)
        angle = angle_between(self.contact_direction, candidate)
        self.direction_delta_deg = float(np.rad2deg(angle))
        limit = np.deg2rad(float(self.c['direction_rate_deg_s'])) * self.dt
        limited = float(np.clip(angle, -limit, limit))
        self.direction_limited = abs(angle) > limit
        n = self.contact_direction
        self.contact_direction = unit([np.cos(limited)*n[0]-np.sin(limited)*n[1],
                                       np.sin(limited)*n[0]+np.cos(limited)*n[1]])
        self.tangent = handed_tangent(self.contact_direction, self.follow_hand)
        self._previous_measurement = measurement
        self.direction_valid = True
        rate_scale = 1 / (1 + (self.direction_rate_filtered_deg_s / float(self.c['direction_slow_rate_deg_s']))**2)
        residual_scale = np.clip((float(self.c['direction_reconfirm_residual_deg']) - self.estimate_residual_deg) /
                                (float(self.c['direction_reconfirm_residual_deg']) - float(self.c['direction_slow_residual_deg'])), .15, 1.)
        target_scale = max(.15, rate_scale) * residual_scale
        # Rate evidence is filtered; recovery of speed is deliberately slower
        # than reduction. Neither dt nor the force correction formula changes.
        self.direction_speed_scale = min(target_scale, self.direction_speed_scale + self.dt/.5)
        return True

    def _measure_direction(self, now, vector):
        self.measurement_jump_deg = self.estimate_residual_deg = self.measurement_rate_deg_s = 0.
        if self.fxy < float(self.c['direction_min_force']):
            return
        direction = vector/self.fxy
        if self._last_valid_measurement is not None:
            self.measurement_jump_deg = abs(np.rad2deg(angle_between(self._last_valid_measurement, direction)))
            interval = now-self._last_valid_measurement_time
            self.measurement_rate_deg_s = self.measurement_jump_deg/max(interval, 1e-12)
        if np.linalg.norm(self.contact_direction) > 0:
            self.estimate_residual_deg = abs(np.rad2deg(angle_between(self.contact_direction,
                                                        float(self.c['force_direction_sign'])*direction)))
        alpha = float(self.c['force_direction_filter_alpha']) ** (self.dt/self.nominal_dt)
        self.direction_rate_filtered_deg_s = alpha*self.direction_rate_filtered_deg_s + (1-alpha)*self.measurement_rate_deg_s
        self._last_valid_measurement, self._last_valid_measurement_time = direction.copy(), now
        if (self.state in (State.CONTINUOUS_TRACKING, State.DIRECTION_RECONFIRM) and
                max(self.measurement_jump_deg, self.estimate_residual_deg) >= float(self.c['direction_reversal_deg'])):
            self.direction_valid = False
            self.request_stop(now, self.tracking_pose, 'ambiguous near-opposite force direction reversal; sign unknown',
                              code=TerminationReason.STOP_DIRECTION_REVERSAL)

    def _begin_direction_reconfirm(self, now, pose, cause):
        if self._direction_attempt_pose is None or np.linalg.norm(pose[:2]-self._direction_attempt_pose) >= float(self.c['direction_min_progress']):
            self.direction_reconfirm_attempts = 0
            self._direction_attempt_pose = pose[:2].copy()
        if self.direction_reconfirm_attempts >= int(self.c['direction_reconfirm_max_attempts']):
            self.request_stop(now, pose, 'repeated direction reconfirmation without progress',
                              code=TerminationReason.STOP_DIRECTION_NO_PROGRESS)
            return
        self.direction_reconfirm_attempts += 1
        self.direction_reconfirm_count += 1
        self.state, self.reason = State.DIRECTION_RECONFIRM, cause
        self.direction_valid = False
        self.stop_requested = True
        self._lost = None
        self._reset_confirmation(now)
        self._event(now, pose, 'DIRECTION_STOP_REQUEST')

    def _direction_reconfirm(self, now, robot, vector):
        self.stop_requested, self.direction_valid = True, False
        # Deadline precedes acceptance: a window completing just after the
        # budget cannot be accepted because of floating-point sample times.
        if now-self._confirm_started+1e-12 >= float(self.c['confirmation_timeout_sec']):
            self.request_stop(now, robot.pose, 'direction reconfirmation timeout',
                              code=TerminationReason.STOP_DIRECTION_UNCONFIRMED)
            return self._command(robot.pose)
        if self.fxy < float(self.c['contact_lost_threshold']):
            if self._lost is None:
                self._lost = now
            self.lost_timer = now-self._lost
            if self.lost_timer+1e-12 >= float(self.c['contact_lost_hold_sec']):
                self.state = State.CONTACT_LOST
                self.loss_detection_pose = robot.pose.copy()
                self._low_force_pending = False
                self._direction_resume_started = None
                self._confirm_started = now
                self._event(now, robot.pose, 'CONTACT_LOST')
                return self._command(robot.pose)
        else:
            self._lost, self.lost_timer = None, 0.
        if self._confirm(now, robot, vector):
            self.state, self.reason = State.CONTINUOUS_TRACKING, ''
            self._confirm_started = None
            self._low_force_pending = False
            self._direction_resume_started = now
            self._direction_resume_pose = robot.pose[:2].copy()
            self._direction_resume_tangent = self.tangent.copy()
            self.direction_resume_progress_m = 0.
            self.direction_speed_scale = float(self.c['direction_resume_scale'])
            self._event(now, robot.pose, 'DIRECTION_CONFIRMED')
        return self._command(robot.pose)

    def _validate_direction_resume(self, now, pose):
        if self._direction_resume_started is None:
            return True
        self.direction_resume_progress_m = float(np.dot(pose[:2]-self._direction_resume_pose, self._direction_resume_tangent))
        if self.fxy >= float(self.c['overload_tangent_zero_force']):
            if self.real_execution:
                self.diagnostics['direction_restart_overload'] = 'WARNING: direction restart overload; tangent held at zero'
                return True  # Existing load_scale zeros tangent, keeping outward normal correction.
            else:
                self.request_stop(now, pose, 'direction restart overload', code=TerminationReason.STOP_FORCE_LIMIT)
                return False
        if self.direction_resume_progress_m < -float(self.c['boundary_margin']):
            self.request_stop(now, pose, 'direction restart backward progress', code=TerminationReason.STOP_DIRECTION_NO_PROGRESS)
            return False
        if self.direction_resume_progress_m >= float(self.c['direction_resume_distance']):
            self._direction_resume_started = None
            self._event(now, pose, 'DIRECTION_RESUME_VERIFIED')
        elif now-self._direction_resume_started >= float(self.c['direction_resume_sec']):
            self.request_stop(now, pose, 'direction restart insufficient progress', code=TerminationReason.STOP_DIRECTION_NO_PROGRESS)
            return False
        return True

    def _begin_recovery(self, now, pose, *, robot=None):
        if not self.c['reacquire_enabled']:
            if self.real_execution:
                self.diagnostics['recovery_disabled'] = 'WARNING: recovery disabled; holding for contact'
            else:
                self.request_stop(now, pose, 'contact lost; experimental recovery disabled', code=TerminationReason.STOP_NO_CONTACT)
            return
        memory = self.last_reliable_contact
        if memory is None or now-memory['timestamp'] > float(self.c['memory_max_age_sec']):
            if self.real_execution:
                self.diagnostics['recovery_memory'] = 'WARNING: recovery memory stale or unavailable'
                if memory is None:
                    return  # Hold for contact; no new direction or search is invented.
            else:
                self.request_stop(now, pose, 'recovery memory stale or unreliable')
                return
        if memory['confidence'] < float(self.c['direction_min_coherence']):
            if self.real_execution:
                self.diagnostics['recovery_confidence'] = 'WARNING: recovery memory confidence below configured threshold'
            else:
                self.request_stop(now, pose, 'recovery memory stale or unreliable')
                return
        if (self._last_recovery_origin is not None
                and np.linalg.norm(pose[:2]-self._last_recovery_origin[:2]) < float(self.c['reacquire_min_progress'])):
            if self.real_execution:
                self.diagnostics['recovery_progress'] = 'WARNING: repeated contact loss without spatial progress'
            else:
                self.request_stop(now, pose, 'repeated contact loss without spatial progress')
                return
        self.reacquire_origin = pose.copy()
        self._last_recovery_origin = pose.copy()
        self._memory_normal, self._memory_tangent = memory['normal'].copy(), memory['tangent'].copy()
        self._reacquire, self._theta = now, 0.
        self.reacquire_path_length = self.reacquire_tracking_error = 0.
        self.reacquire_speed_path_length = self.reacquire_raw_pose_path_length = 0.
        self.reacquire_max_displacement = 0.
        self._recovery_previous_pose = pose[:2].copy()
        self._recovery_previous_observation_time = now if robot is None else robot.timestamp
        self._recovery_previous_speed = float(np.linalg.norm(
            self.tracking_velocity if robot is None else robot.tcp_speed[:2]))
        self.reacquire_reference = pose[:2].copy()
        self._confirm_started = None
        self.state = State.LOCAL_REACQUIRE
        self._recovery_count += 1
        self._event(now, pose, 'LOCAL_REACQUIRE')

    def _observe_reacquire_path(self, robot):
        """Integrate actual planar speed using HOST RTDE observation times.

        Summing every pose difference counts stationary encoder/pose variation
        as travel. Keep that sum for diagnosis, not as the travel budget. The
        independent radial lower bound still catches displacement with missing
        velocity evidence; it must not be repeatedly added to the integral.
        """
        pose = robot.pose
        self.reacquire_raw_pose_path_length += float(np.linalg.norm(pose[:2]-self._recovery_previous_pose))
        self._recovery_previous_pose = pose[:2].copy()
        elapsed = robot.timestamp-self._recovery_previous_observation_time
        if elapsed > 0:
            speed = float(np.linalg.norm(robot.tcp_speed[:2]))
            self.reacquire_speed_path_length += .5*(self._recovery_previous_speed+speed)*elapsed
            self._recovery_previous_observation_time = robot.timestamp
            self._recovery_previous_speed = speed
        self.reacquire_max_displacement = max(self.reacquire_max_displacement,
            float(np.linalg.norm(pose[:2]-self.reacquire_origin[:2])))
        self.reacquire_path_length = max(self.reacquire_speed_path_length,
                                        self.reacquire_max_displacement)

    def _recover(self, now, robot, vector):
        pose = robot.pose
        self._observe_reacquire_path(robot)
        self.reacquire_tracking_error = float(np.linalg.norm(self.reacquire_reference-pose[:2]))
        margin = float(self.c['boundary_margin'])
        limits = (
            (now >= self._reacquire+float(self.c['reacquire_max_time_sec']), 'time'),
            (np.linalg.norm(pose[:2]-self.reacquire_origin[:2]) >= float(self.c['reacquire_max_distance'])-margin, 'displacement'),
            (self.reacquire_path_length >= float(self.c['reacquire_max_path'])-margin, 'path'),
            (self.reacquire_tracking_error > float(self.c['reacquire_max_tracking_error']), 'tracking error'),
        )
        for exceeded, reason in limits:
            if exceeded:
                if self.real_execution:
                    if reason == 'tracking error':
                        self.diagnostics[f'recovery_{reason}'] = f'WARNING: local reacquire {reason} budget exceeded'
                        continue  # Existing reference freezes when following error is large.
                    # No measured motion means the travel budget may never
                    # expire. Keep this recovery episode bounded by its
                    # configured time even while contact is being confirmed.
                    if reason != 'time' and (self.contact_flag or self._confirm_started is not None):
                        continue  # Stop and confirm contact before declaring spatial exhaustion.
                self.request_stop(now, pose, f'local reacquire {reason} budget exhausted',
                                  code=TerminationReason.STOP_RECOVERY_EXHAUSTED, event='BUDGET_STOP')
                return self._command(pose)
        if self.contact_flag and self._confirm_started is None:
            self._reset_confirmation(now)
            self._event(now, pose, 'REACQUIRE_STOP_REQUEST')
        if self._confirm_started is not None:
            # Reference freezes for the entire confirmation. Failure stops;
            # there is no time-driven jump or second search strategy.
            if self._confirm(now, robot, vector):
                self.state = State.CONTINUOUS_TRACKING
                self._lost = None
                self.lost_timer = 0.
                self._event(now, pose, 'REACQUIRED')
                self._confirm_started = None
            return self._command(pose)
        if self._theta >= np.deg2rad(float(self.c['reacquire_max_angle_deg'])) and self.reacquire_tracking_error < float(self.c['reacquire_reference_freeze_error']):
            self.request_stop(now, pose, 'local reacquire angle exhausted', code=TerminationReason.STOP_RECOVERY_EXHAUSTED, event='BUDGET_STOP')
            return self._command(pose)
        if self.reacquire_tracking_error <= float(self.c['reacquire_reference_freeze_error']):
            # omega <= path speed / radius, independent of target geometry.
            omega = min(np.deg2rad(float(self.c['reacquire_angular_speed_deg_s'])),
                        float(self.c['reacquire_speed'])/float(self.c['reacquire_radius']))
            self._theta = min(self._theta+omega*self.dt, np.deg2rad(float(self.c['reacquire_max_angle_deg'])))
        self.reacquire_reference = arc_reference(self.reacquire_origin, self._memory_normal, self._memory_tangent,
                                                 float(self.c['reacquire_radius']), self._theta)
        self.reacquire_heading_deg = float(np.rad2deg(self._theta))
        velocity = float(self.c['reacquire_position_gain']) * (self.reacquire_reference-pose[:2])
        norm = np.linalg.norm(velocity)
        if norm > float(self.c['reacquire_speed']):
            velocity *= float(self.c['reacquire_speed'])/norm
        predicted = pose[:2] + velocity*self.dt
        if max(float(self.c['reacquire_radius'])*self._theta,
               self.reacquire_path_length+float(np.linalg.norm(velocity))*self.dt) >= float(self.c['reacquire_max_path'])-margin:
            self.request_stop(now, pose, 'local reacquire planned path exhausted',
                              code=TerminationReason.STOP_RECOVERY_EXHAUSTED, event='BUDGET_STOP')
            return self._command(pose)
        if max(np.linalg.norm(self.reacquire_reference-self.reacquire_origin[:2]),
               np.linalg.norm(predicted-self.reacquire_origin[:2])) >= float(self.c['reacquire_max_distance'])-margin:
            self.request_stop(now, pose, 'local reacquire planned displacement exhausted',
                              code=TerminationReason.STOP_RECOVERY_EXHAUSTED, event='BUDGET_STOP')
            return self._command(pose)
        self.stop_requested = False
        self.stop_confirmed = False
        return self._command(pose, velocity)

    def update(self, now, raw, processed, robot, *, execution_settled=None):
        self.execution_settled = execution_settled
        pose = robot.pose
        self.tracking_pose = pose.copy()
        self.tracking_velocity = robot.tcp_speed[:2].copy()
        self.v_t = self.v_n = 0.
        self.force_rate_guard_active = False
        if self.state == State.STOP:
            return self._command(pose)
        self.tangent_limit_reason = '' if self.state == State.CONTINUOUS_TRACKING else self.state.value
        if not np.all(np.isfinite(np.r_[now, robot.timestamp, raw.array(), processed.array(), pose, robot.tcp_speed])):
            self.request_stop(now, pose, 'nonfinite sensor/robot sample')
            return self._command(pose)
        self.dt = self.nominal_dt if self._last_time is None else now-self._last_time
        age = now-robot.timestamp  # RobotState timestamp is HOST observation time only.
        if (self.dt <= 0 or self.dt > float(self.c['max_sample_gap_sec'])
                or age < -1e-6 or age > float(self.c['max_observation_age_sec'])):
            if self.real_execution:
                self.diagnostics['sample_timing'] = f'WARNING: sample interval={self.dt:g} s, observation age={age:g} s'
                if self.dt <= 0:
                    self.dt = self.nominal_dt
            else:
                self._reset_confirmation(now)
                self.request_stop(now, pose, 'stale sample / invalid control interval', code=TerminationReason.STOP_STALE_DATA)
                return self._command(pose)
        self._last_time = now
        self.fxy = float(np.hypot(processed.fx, processed.fy))
        self.force_rate = self._rate.update(now, self.fxy)
        # Snapshot the guard used for this sample, before any phase transition.
        # Acquisition and stopped confirmation still run all hard F/T checks.
        self.force_rate_guard_active = self.state == State.CONTINUOUS_TRACKING and not self._low_force_pending
        safety = (continuous_force_reason(raw, processed, self.p, self.diagnostics) if self.real_execution
                  else force_safety_reason(raw, processed, self.p))
        if self.force_rate > float(self.p['force_rate_limit']):
            if self.real_execution:
                self.diagnostics['force_rate'] = f'WARNING: force rate {self.force_rate:g} N/s'
            elif safety is None and self.force_rate_guard_active:
                safety = f'force rate exceeded: {self.force_rate:.3f} N/s'
        if self.real_execution:
            self.force_rate_guard_active = False
        if safety:
            self.request_stop(now, pose, safety, code=TerminationReason.STOP_FORCE_LIMIT)
            return self._command(pose)
        self._observe_unload(now, pose)
        if self.state == State.STOP:
            return self._command(pose)
        vector = np.array([processed.fx, processed.fy])
        searching = self.state in (State.READY, State.TARGET_SEARCH)
        if not (self.real_execution and searching):
            self._measure_direction(now, vector)
        if self.state == State.STOP:
            return self._command(pose)
        self.contact_flag = self.fxy >= float(self.p['contact_threshold'])
        self.force_direction = vector/self.fxy if self.fxy >= float(self.c['direction_min_force']) else np.zeros(2)
        self.measurement_direction = self.force_direction.copy()
        if self._started is None:
            self._started, self._start_pose = now, pose.copy()
        budget = self.c['max_runtime_sec']
        if budget is not None and now-self._started >= float(budget):
            if self.real_execution:
                self.diagnostics['search_runtime' if searching else 'runtime'] = 'WARNING: configured runtime budget exceeded'
            else:
                self.request_stop(now, pose, 'maximum experiment duration reached', event='BUDGET_STOP', code=TerminationReason.STOP_TIME_LIMIT)
                return self._command(pose)
        if self.state in (State.READY, State.TARGET_SEARCH):
            if self.contact_flag:
                self.state = State.FIRST_CONTACT
                self.first_threshold_pose = pose.copy()
                self._reset_confirmation(now)
                self._event(now, pose, 'FIRST_THRESHOLD_STOP_REQUEST')
            elif self.state == State.READY:
                self.state = State.TARGET_SEARCH
                return self._command(pose)
            else:
                geometry = self.search_geometry
                origin = self._start_pose[:2] if geometry is None else np.asarray(geometry['origin_xy'])
                limit = (float(self.c['search_max_distance'])-float(self.c.get('search_boundary_margin', self.c['boundary_margin']))
                         if geometry is None else float(geometry['usable_distance_m']))
                prediction_dt = self.dt if geometry is None else max(self.dt, self.nominal_dt)
                if self.real_execution:
                    # The next command lasts one nominal tick. Past jitter is
                    # diagnostic and must not enlarge the geometric lookahead.
                    prediction_dt = self.nominal_dt
                time_exhausted = now-self._started >= float(self.c['search_max_time_sec'])
                if time_exhausted and self.real_execution:
                    self.diagnostics['search_time'] = 'WARNING: initial search time budget exceeded'
                if (np.linalg.norm(pose[:2]-origin)+float(self.c['search_speed'])*prediction_dt
                        >= limit or (time_exhausted and not self.real_execution)):
                    self.request_stop(now, pose, 'initial search budget exhausted', code=TerminationReason.STOP_SEARCH_LIMIT, event='BUDGET_STOP')
                    return self._command(pose)
                self.stop_requested = False
                return self._command(pose, self.search_direction*float(self.c['search_speed']))
        if self.state == State.FIRST_CONTACT:
            self.tangent_limit_reason = ('FIRST_CONTACT_SETTLE_HOLD' if
                np.linalg.norm(robot.tcp_speed[:3]) <= self.c['settle_speed_mps'] else 'FIRST_CONTACT_BRAKING')
            if self._confirm(now, robot, vector):
                self.initial_contact = pose.copy()
                self.state = State.CONTINUOUS_TRACKING
                self._event(now, pose, 'FIRST_CONTACT')
                self._event(now, pose, 'TRACKING_ENTERED')
                if self.fxy >= self.c['overload_tangent_zero_force']:
                    self._begin_unload(now, pose)
                if self.state == State.CONTINUOUS_TRACKING:
                    self.tangent_limit_reason = 'TRACKING_TRANSITION_STOP'
                self._confirm_started = None
            elif (self.real_execution and self.stop_confirmed and
                  (not self.contact_flag or
                   now-self._confirm_started >= float(self.c['confirmation_timeout_sec']))):
                # Retry only after the existing policy AND execution standstill
                # confirmation. Keep the original search origin/direction/budget.
                self.state = State.TARGET_SEARCH
                self._reset_confirmation(now)
                self._confirm_started = None
                self.direction_valid = False
                self.diagnostics['first_contact_retry'] = 'WARNING: first contact not confirmed; resuming original target search'
                self._event(now, pose, 'FIRST_CONTACT_RETRY_SEARCH')
                # This transition sample remains stopped; the next SEARCH tick
                # applies the unchanged contact and polygon checks before moving.
            return self._command(pose)
        if self.state == State.DIRECTION_RECONFIRM:
            return self._direction_reconfirm(now, robot, vector)
        if self.state == State.CONTINUOUS_TRACKING:
            self.stop_confirmed = False
            if self.fxy < float(self.c['contact_lost_threshold']):
                self.tangent_limit_reason = 'LOW_FORCE_CONTACT_CONFIRMATION'
                self.direction_valid = False
                self.stop_requested = True
                if not self._low_force_pending:
                    self._low_force_pending = True
                    self._reset_confirmation(now)
                if self._lost is None:
                    self._lost = now
                    self._event(now, pose, 'LOW_FORCE_STOP_REQUEST')
                # Keep one deadline for the whole pause, including repeated
                # dips. Low force resets contact evidence, not settled speed.
                self._confirm(now, robot, vector)
                if self.state == State.STOP:
                    return self._command(pose)
                self.lost_timer = now-self._lost
                if self.lost_timer+1e-12 >= float(self.c['contact_lost_hold_sec']):
                    self.state = State.CONTACT_LOST
                    self._low_force_pending = False
                    self._direction_resume_started = None
                    self.loss_detection_pose = pose.copy()
                    self._confirm_started = now
                    self._event(now, pose, 'CONTACT_LOST')
                return self._command(pose)
            self._lost, self.lost_timer = None, 0.
            if self._low_force_pending:
                self.tangent_limit_reason = 'LOW_FORCE_CONTACT_CONFIRMATION'
                self.stop_requested = True
                if not self._direction(now, pose, vector):
                    self._force_since = None
                    self._confirmation_vectors.clear()
                    self.contact_hold_elapsed = 0.
                    return self._command(pose)
                if self._confirm(now, robot, vector):
                    self._low_force_pending = False
                    self._confirm_started = None
                    self._event(now, pose, 'LOW_FORCE_RECONFIRMED')
                # Even the successful confirmation sample remains a stop.
                return self._command(pose)
            self.stop_requested = False
            if not self.unload_active and self.fxy >= self.c['overload_tangent_zero_force']:
                self._begin_unload(now, pose)
                if self.state == State.STOP:
                    return self._command(pose)
            if not self._direction(now, pose, vector):
                if self.state != State.STOP:
                    self.tangent_limit_reason = 'DIRECTION_CONFIRMATION'
                self.stop_requested = True
                return self._command(pose)
            self._remember(now, pose)  # Includes reliable 0.5--1 N samples.
            if not self._validate_direction_resume(now, pose):
                return self._command(pose)
            vn = normal_feedback_speed(self.fxy, self.c)
            self.unload_resume_scale = self._unload_tangent_scale(now, pose)
            if self.state == State.STOP:
                return self._command(pose)
            if self.unload_active:
                vn = min(0., vn)  # Do not resume inward advance during the force-release hold.
            high_start = float(self.c['force_reference'])+float(self.c['force_deadband'])
            load_scale = np.clip((float(self.c['overload_tangent_zero_force'])-self.fxy) /
                                 (float(self.c['overload_tangent_zero_force'])-high_start), 0, 1)
            contact_scale = np.clip((self.fxy-float(self.c['contact_lost_threshold'])) /
                                    (float(self.p['contact_threshold'])-float(self.c['contact_lost_threshold'])), 0, 1)
            vt = float(self.c['tangential_speed'])*load_scale*contact_scale*self.direction_confidence
            vt *= (min(self.direction_speed_scale, float(self.c['direction_resume_scale']))
                   if self._direction_resume_started is not None else self.direction_speed_scale)
            vt *= self.unload_resume_scale
            if not self.tangent_limit_reason:
                self.tangent_limit_reason = ('FORCE_LOAD_SCALING' if load_scale < 1 else
                    'DIRECTION_SPEED_SCALING' if self.direction_speed_scale < 1 else '')
            return self._command(pose, vt*self.tangent+vn*self.contact_direction)
        if self.state == State.CONTACT_LOST:
            self.stop_requested = True
            if self.real_execution and self.contact_flag:
                if self._confirm(now, robot, vector):
                    self.state = State.CONTINUOUS_TRACKING
                    self._lost, self.lost_timer, self._confirm_started = None, 0., None
                    self._event(now, pose, 'REACQUIRED')
                return self._command(pose)
            if self._settled(now, robot):
                self._begin_recovery(now, pose, robot=robot)
            elif now-self._confirm_started >= float(self.c['confirmation_timeout_sec']):
                self.request_stop(now, pose, 'contact lost stop confirmation timeout')
            return self._command(pose)
        if self.state == State.LOCAL_REACQUIRE:
            return self._recover(now, robot, vector)
        return self._command(pose)

    def telemetry(self, command):
        velocity = command.direction_xy*command.speed
        memory = self.last_reliable_contact or {}
        origin = [float('nan')]*2 if self.reacquire_origin is None else self.reacquire_origin[:2]
        reference = [float('nan')]*2 if self.reacquire_reference is None else self.reacquire_reference
        missing = [float('nan')]*2
        memory_pose = memory.get('pose', missing)[:2]
        memory_normal = memory.get('normal', missing)
        loss = missing if self.loss_detection_pose is None else self.loss_detection_pose[:2]
        recovery_normal = missing if self._memory_normal is None else self._memory_normal
        recovery_tangent = missing if self._memory_tangent is None else self._memory_tangent
        result = dict(zip(EXTRA_SAMPLE_FIELDS, (
            self.filtered_fxy, *self.force_direction, *self.contact_direction,
            float(self.c['force_reference']), float(self.c['force_reference'])-self.fxy,
            self.v_t, self.v_n, *velocity, command.speed, self.lost_timer, self.follow_hand,
            self.c['force_direction_sign'], self.force_rate, self.reacquire_heading_deg,
            self.dt, int(self.direction_valid), self.direction_confidence, self.direction_delta_deg,
            int(self.direction_limited), *self.measurement_direction, int(self.stop_requested), int(self.stop_confirmed),
            self.contact_hold_elapsed, self.settle_hold_elapsed, *origin, *reference,
            self.reacquire_tracking_error, self.reacquire_path_length,
            self.reacquire_speed_path_length, self.reacquire_raw_pose_path_length,
            self.reacquire_max_displacement,
            memory.get('timestamp', float('nan')), memory.get('confidence', 0.), self.reason,
            *memory_pose, *memory_normal, *loss, *recovery_normal, *recovery_tangent,
        )))
        result.update(zip(DIRECTION_FIELDS, (
            self.measurement_jump_deg, self.estimate_residual_deg, self.measurement_rate_deg_s,
            self.direction_rate_filtered_deg_s, self.direction_speed_scale,
            0. if self.state != State.DIRECTION_RECONFIRM else max(0., self._last_time-self._confirm_started),
            self.direction_reconfirm_count, self.direction_reconfirm_attempts, int(self._direction_resume_started is not None),
            self.direction_resume_progress_m, 'RECONFIRM' if self.state == State.DIRECTION_RECONFIRM else
            (self.state.value if self.state != State.CONTINUOUS_TRACKING else
             ('VERIFY_RESUME' if self._direction_resume_started is not None else ('LOW_FORCE_CONFIRM' if self._low_force_pending else 'TRACK'))))))
        result['force_rate_guard_active'] = int(self.force_rate_guard_active)
        result.update(zip(UNLOAD_FIELDS, (self.tangent_limit_reason, int(self.unload_active), self.unload_elapsed,
            self.unload_displacement, self.unload_projected, self.unload_force_mean, self.unload_improvement,
            self.unload_force_slope, self.unload_resume_scale,
            self._settle_since if self._settle_since is not None else '',
            '' if self.first_threshold_pose is None else float(np.linalg.norm(self.tracking_pose[:2]-self.first_threshold_pose[:2])),
            float(self.tracking_velocity @ self.contact_direction), float(self.tracking_velocity @ self.tangent))))
        return result
