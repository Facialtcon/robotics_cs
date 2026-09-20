"""Experimental velocity force feedback; no shape inference or discrete probes.

The signed force direction is only an interaction-direction estimate. Friction
can contaminate it; +1 being inward is a simulator convention, not a PX6D fact.
"""
from __future__ import annotations

from enum import Enum
import numpy as np

from core.models import PolicyCommand, PolicyWaypoint
from experiment_logging.termination import TerminationReason
from policy.boundary_estimation import handed_tangent, unit
from safety.force_guard import ForceRateGuard, force_safety_reason


class State(str, Enum):
    READY = "READY"
    TARGET_SEARCH = "TARGET_SEARCH"
    FIRST_CONTACT = "FIRST_CONTACT"
    CONTINUOUS_TRACKING = "CONTINUOUS_TRACKING"
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
    if c.get("enabled") is not True:
        raise ValueError("continuous_tracking.enabled must be true for this entry point")
    positive = ("tangential_speed", "force_reference", "force_gain", "normal_speed_limit",
                "search_speed", "search_max_distance", "search_max_time_sec",
                "contact_lost_threshold", "contact_lost_hold_sec", "reacquire_speed",
                "reacquire_max_angle_deg", "reacquire_angular_speed_deg_s",
                "reacquire_max_time_sec", "reacquire_max_distance", "max_runtime_sec",
                "visualization_fps")
    for key in positive:
        if not np.isfinite(float(c[key])) or float(c[key]) <= 0:
            raise ValueError(f"continuous_tracking.{key} must be finite and positive")
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
    unit(p["search_direction_xy"])


class ContinuousTrackingPolicy:
    def __init__(self, config: dict):
        validate_config(config)
        self.c = dict(config["continuous_tracking"])
        self.p = dict(config["policy"])
        self.max_speed = float(config["robot"]["max_tcp_speed"])
        self.dt = 1 / float(self.p["control_rate_hz"])
        self.follow_hand = self.p["follow_hand"]
        self.search_direction = unit(self.p["search_direction_xy"])
        self.state = State.READY
        self.reason = ""
        self.stop_reason = None
        self.events = []
        self.initial_contact = None
        self.last_contact_pose = None
        self.last_reliable_tangent = None
        self.last_contact_direction = None
        self.force_direction = np.zeros(2)
        self.contact_direction = np.zeros(2)
        self.tangent = np.zeros(2)
        self._filtered = None
        self._rate = ForceRateGuard()
        self._last_time = self._started = self._candidate = self._lost = self._reacquire = None
        self._start_pose = None
        self.fxy = self.filtered_fxy = self.force_rate = self.lost_timer = 0.0
        self.v_t = self.v_n = self.reacquire_heading_deg = 0.0
        self.contact_flag = False

    def _event(self, now, pose, name):
        self.events.append(PolicyWaypoint(now, self.state.value, name, pose.copy(),
                           target_direction_xy=self.contact_direction.copy(), tangent_xy=self.tangent.copy()))

    def request_stop(self, now, pose, reason, *, event="SAFETY_STOP", code=None):
        if self.state != State.STOP:
            self.state, self.reason = State.STOP, reason
            self.stop_reason = code
            self._event(now, pose, event)
        self.v_t = self.v_n = 0.0

    def _command(self, pose, velocity=None):
        velocity = np.zeros(2) if velocity is None else np.asarray(velocity, dtype=float)
        speed = float(np.linalg.norm(velocity))
        if speed > self.max_speed:
            scale = self.max_speed / speed
            velocity *= scale
            self.v_t *= scale
            self.v_n *= scale
            speed = self.max_speed
        return PolicyCommand(self.state.value, speed > 1e-12,
                             velocity / speed if speed > 1e-12 else np.zeros(2), speed,
                             pose.copy(), self.contact_flag, False, self.reason)

    def _stable(self, now):
        if not self.contact_flag:
            self._candidate = None
            return False
        if self._candidate is None:
            self._candidate = now
        return now - self._candidate + 1e-12 >= float(self.p["contact_hold_time"])

    def _remember(self, pose):
        self.last_contact_pose = pose.copy()
        self.last_reliable_tangent = self.tangent.copy()
        self.last_contact_direction = self.contact_direction.copy()

    def update(self, now, raw, processed, robot):
        pose = robot.pose
        self.v_t = self.v_n = 0.0
        if self.state == State.STOP:
            return self._command(pose)
        # Safety always precedes contact transitions, budgets and feedback.
        if not np.all(np.isfinite(np.r_[now, raw.array(), processed.array(), pose, robot.tcp_speed])):
            self.request_stop(now, pose, "nonfinite sensor/robot sample")
            return self._command(pose)
        if self._last_time is not None and now <= self._last_time:
            self.request_stop(now, pose, "non-increasing control timestamp")
            return self._command(pose)
        self._last_time = now
        self.fxy = float(np.hypot(processed.fx, processed.fy))
        self.force_rate = self._rate.update(now, self.fxy)
        safety = force_safety_reason(raw, processed, self.p)
        if safety is None and self.force_rate > float(self.p["force_rate_limit"]):
            safety = f"force rate exceeded: {self.force_rate:.3f} N/s"
        if safety:
            self.request_stop(now, pose, safety, code=TerminationReason.STOP_FORCE_LIMIT)
            return self._command(pose)
        vector = np.array([processed.fx, processed.fy])
        alpha = float(self.c["force_direction_filter_alpha"])
        self._filtered = vector if self._filtered is None else alpha * self._filtered + (1-alpha) * vector
        self.filtered_fxy = float(np.linalg.norm(self._filtered))
        self.force_direction = vector / self.fxy if self.fxy > 1e-12 else np.zeros(2)
        self.contact_flag = self.fxy >= float(self.p["contact_threshold"])
        # Below loss threshold a direction is unreliable. Retain the last
        # estimate while the magnitude EMA continues to be logged.
        if self.fxy >= float(self.c["contact_lost_threshold"]) and self.filtered_fxy > 1e-12:
            self.contact_direction = float(self.c["force_direction_sign"]) * self._filtered / self.filtered_fxy
            self.tangent = handed_tangent(self.contact_direction, self.follow_hand)
        if self._started is None:
            self._started, self._start_pose = now, pose.copy()
            return self._command(pose)  # one observable READY cycle
        if now - self._started >= float(self.c["max_runtime_sec"]):
            self.request_stop(now, pose, "maximum experiment duration reached", event="BUDGET_STOP",
                              code=TerminationReason.STOP_TIME_LIMIT)
            return self._command(pose)
        if self.state == State.READY:
            self.state = State.TARGET_SEARCH
        if self.state == State.TARGET_SEARCH:
            distance = float(np.linalg.norm(pose[:2] - self._start_pose[:2]))
            if (distance + float(self.c["search_speed"]) * self.dt >= float(self.c["search_max_distance"])
                    or now - self._started >= float(self.c["search_max_time_sec"])):
                self.request_stop(now, pose, "initial search budget exhausted", event="BUDGET_STOP",
                                  code=TerminationReason.STOP_SEARCH_LIMIT)
                return self._command(pose)
            if self._stable(now):
                self.state = State.FIRST_CONTACT
                self.initial_contact = pose.copy()
                self._remember(pose)
                self._event(now, pose, "FIRST_CONTACT")
                return self._command(pose)
            # Stop even during contact confirmation; never keep pushing while
            # waiting for the stable-contact hold to expire.
            return self._command(pose, None if self.contact_flag else self.search_direction * float(self.c["search_speed"]))
        if self.state == State.FIRST_CONTACT:
            self.state = State.CONTINUOUS_TRACKING
            self._candidate = None
        if self.state == State.CONTINUOUS_TRACKING:
            if self.fxy < float(self.c["contact_lost_threshold"]):
                if self._lost is None:
                    self._lost = now
                self.lost_timer = now - self._lost
                if self.lost_timer + 1e-12 >= float(self.c["contact_lost_hold_sec"]):
                    self.state = State.CONTACT_LOST
                    self._event(now, pose, "CONTACT_LOST")
                    return self._command(pose)
            else:
                self._lost = None
                self.lost_timer = 0.0
                if self.contact_flag:
                    self._remember(pose)
            error = float(self.c["force_reference"]) - self.fxy
            self.v_n = (0.0 if abs(error) <= float(self.c["force_deadband"]) else
                        float(np.clip(float(self.c["force_gain"]) * error,
                                      -float(self.c["normal_speed_limit"]), float(self.c["normal_speed_limit"]))))
            self.v_t = float(self.c["tangential_speed"])
            return self._command(pose, self.v_t * self.tangent + self.v_n * self.contact_direction)
        if self.state == State.CONTACT_LOST:
            self.state = State.LOCAL_REACQUIRE
            self._reacquire = now
            self._candidate = None
            self._event(now, pose, "LOCAL_REACQUIRE")
            return self._command(pose)
        if self.state == State.LOCAL_REACQUIRE:
            elapsed = now - self._reacquire
            distance = float(np.linalg.norm(pose[:2] - self.last_contact_pose[:2]))
            if (elapsed >= float(self.c["reacquire_max_time_sec"])
                    or distance >= float(self.c["reacquire_max_distance"])):
                self.request_stop(now, pose, "local reacquire budget exhausted", event="BUDGET_STOP",
                                  code=TerminationReason.STOP_RECOVERY_EXHAUSTED)
                return self._command(pose)
            if self._stable(now):
                self.state = State.CONTINUOUS_TRACKING
                self._lost = None
                self.lost_timer = 0.0
                self._remember(pose)
                self._event(now, pose, "REACQUIRED")
                return self._command(pose)  # immediately stop search on reacquisition
            if self.contact_flag:
                return self._command(pose)
            # Triangular heading sweep: 0 -> +A -> -A -> +A, constant angular
            # speed. Start toward last contact; handed tangent selects first side.
            angle = float(self.c["reacquire_max_angle_deg"])
            travel = elapsed * float(self.c["reacquire_angular_speed_deg_s"])
            phase = (travel + angle) % (4 * angle)
            heading = angle - abs(phase - 2 * angle)
            self.reacquire_heading_deg = heading
            theta = np.deg2rad(heading)
            direction = (np.cos(theta) * self.last_contact_direction
                         + np.sin(theta) * self.last_reliable_tangent)
            velocity = float(self.c["reacquire_speed"]) * direction
            predicted = pose[:2] + velocity * self.dt
            if np.linalg.norm(predicted - self.last_contact_pose[:2]) >= float(self.c["reacquire_max_distance"]):
                self.request_stop(now, pose, "local reacquire distance exhausted", event="BUDGET_STOP",
                                  code=TerminationReason.STOP_RECOVERY_EXHAUSTED)
                return self._command(pose)
            return self._command(pose, velocity)
        return self._command(pose)

    def telemetry(self, command):
        velocity = command.direction_xy * command.speed
        return dict(zip(EXTRA_SAMPLE_FIELDS, (
            self.filtered_fxy, *self.force_direction, *self.contact_direction,
            float(self.c["force_reference"]), float(self.c["force_reference"]) - self.fxy,
            self.v_t, self.v_n, *velocity, command.speed, self.lost_timer,
            self.follow_hand, self.c["force_direction_sign"], self.force_rate,
            self.reacquire_heading_deg,
        )))
