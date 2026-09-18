"""One immutable-anchor experiment: guarded probe -> hold -> same-path return.

The next decision is permitted only after return_completed. Safety/operator
stops may interrupt return, and are honestly logged as such. Successful target
acquisition returns to a local point on its observed ray; its original search
anchor remains unchanged for audit.
"""
from dataclasses import dataclass, field
import numpy as np
from core.models import Wrench
from policy.boundary_estimation import unit


class ReturnReadinessError(RuntimeError):
    """The designated return target could not become ready for another action."""


@dataclass
class ProbeEpisode:
    probe_id: int
    anchor_pose: np.ndarray
    probe_direction: np.ndarray
    probe_start_time: float
    max_probe_distance: float
    policy_state: str
    purpose: str
    recovery_id: int | None = None
    angle_deg: float = 0.0
    phase: str = "PROBE"
    probe_end_time: float | None = None
    return_end_time: float | None = None
    outcome: str = "IN_PROGRESS"
    contact_pose: np.ndarray | None = None
    contact_wrench: Wrench | None = None
    contact_time: float | None = None
    end_pose: np.ndarray | None = None
    returned_pose: np.ndarray | None = None
    max_force: float = 0.0
    return_completed: bool = False
    accepted_as_boundary: bool = False
    rejection_reason: str = ""
    force_samples: list = field(default_factory=list)
    hold_since: float | None = None
    initialization_round: int | None = None
    return_settle_since: float | None = None
    return_settle_started_at: float | None = None
    return_target_pose: np.ndarray = field(init=False)

    def __post_init__(self):
        self.anchor_pose = np.asarray(self.anchor_pose, dtype=float).copy()
        self.anchor_pose.setflags(write=False)
        self.return_target_pose = self.anchor_pose.copy()
        self.return_target_pose.setflags(write=False)
        self.probe_direction = unit(self.probe_direction)
        self.probe_direction.setflags(write=False)

    def tick(self, now, pose, wrench, config, tcp_speed=None):
        """Return (target, requested speed), or None for a stationary sample."""
        fxy = float(np.hypot(wrench.fx, wrench.fy))
        self.max_force = max(self.max_force, fxy)
        self.force_samples.append((float(now), fxy, self.phase))
        if self.phase == "RETURN":
            position_error = float(np.linalg.norm(pose[:2] - self.return_target_pose[:2]))
            at_anchor = position_error <= float(config["position_tolerance"])
            # Legacy direct episode callers omit velocity; shared policy callers
            # pass measured TCP speed. Both paths still require a continuous hold.
            try:
                speed = np.zeros(3) if tcp_speed is None else np.asarray(tcp_speed, dtype=float)
            except (TypeError, ValueError) as exc:
                raise ReturnReadinessError(
                    f"return readiness invalid TCP linear speed: probe={self.probe_id}"
                ) from exc
            if speed.shape not in ((3,), (6,)) or not np.all(np.isfinite(speed[:3])):
                raise ReturnReadinessError(f"return readiness invalid TCP linear speed: probe={self.probe_id}")
            linear_speed = float(np.linalg.norm(speed[:3]))
            speed_limit = float(config.get("return_settled_speed_mps", .0005))
            hold_time = float(config.get("return_settle_hold_sec", config["contact_hold_time"]))
            timeout = float(config.get("return_settle_timeout_sec", 2.0))
            if at_anchor and self.return_settle_started_at is None:
                self.return_settle_started_at = now
            ready = at_anchor and linear_speed <= speed_limit and fxy < float(config["contact_threshold"])
            if ready:
                if self.return_settle_since is None:
                    self.return_settle_since = now
            else:
                self.return_settle_since = None
            held = 0.0 if self.return_settle_since is None else now - self.return_settle_since
            elapsed = 0.0 if self.return_settle_started_at is None else now - self.return_settle_started_at
            if ready and held >= hold_time and elapsed <= timeout:
                self.return_completed = True
                self.return_end_time = now
                self.returned_pose = pose.copy()
                self.phase = "DONE"
                return None
            # Start this bound only when the return first reaches its target.
            # A long search return is still allowed; subsequent drift cannot
            # keep restarting the readiness deadline indefinitely.
            if self.return_settle_started_at is not None and elapsed >= timeout:
                raise ReturnReadinessError(
                    f"return readiness timeout: probe={self.probe_id}, "
                    f"position_error={position_error * 1000:.6f} mm "
                    f"(limit={float(config['position_tolerance']) * 1000:.6f} mm), "
                    f"tcp_linear_speed={linear_speed * 1000:.6f} mm/s "
                    f"(limit={speed_limit * 1000:.6f} mm/s), "
                    f"Fxy={fxy:.6f} N (must be <{float(config['contact_threshold']):.6f} N), "
                    f"ready_hold={held:.6f}/{hold_time:.6f} s, "
                    f"elapsed={elapsed:.6f}/{timeout:.6f} s"
                )
            if at_anchor:
                return None
            return self.return_target_pose, float(config["retract_speed"])
        if self.phase == "DONE":
            return None
        # FIRST threshold crossing stops motion; the hold never pushes deeper.
        if fxy >= float(config["contact_threshold"]):
            if self.hold_since is None:
                self.hold_since = now
                self.contact_pose = pose.copy()
                self.contact_wrench = wrench
                self.contact_time = now
            if now - self.hold_since >= float(config["contact_hold_time"]):
                if self.purpose == "ACQUISITION":
                    self._set_acquisition_return_target(config)
                self._finish_probe(now, pose, "CONTACT")
            else:
                self.phase = "HOLD"
            return None
        if self.phase == "HOLD":
            self.contact_pose = None
            self.contact_wrench = None
            self._finish_probe(now, pose, "UNSTABLE_CONTACT")
            return None
        distance = float(np.dot(pose[:2] - self.anchor_pose[:2], self.probe_direction))
        if distance >= self.max_probe_distance - 1e-9:
            self._finish_probe(now, pose, "NO_CONTACT")
            return None
        target = self.anchor_pose.copy()
        target[:2] += self.probe_direction * self.max_probe_distance
        speed = config["search_speed"] if self.purpose == "ACQUISITION" else config["probe_speed"]
        if self.purpose == "RECOVERY":
            speed = config.get("recovery_probe_speed", speed)
        if self.purpose in {"RECOVERY", "CONFIRMATION"}:
            onset = float(config.get("corner_approach_force_ratio", .5)) * float(config["contact_threshold"])
            # Start braking before contact, using the existing episode peak to
            # avoid reaccelerating when the filtered force momentarily falls.
            # A fresh ray has a fresh peak. HOLD/RETURN and safety take priority
            # above this speed-only limit; the target and direction are unchanged.
            if self.max_force >= onset:
                speed = min(float(speed), float(config.get("corner_approach_speed", .002)))
        return target, float(speed)

    def _set_acquisition_return_target(self, config):
        """Retreat locally on the observed search segment, without moving P0."""
        clearance = float(config["initialization_clearance"] if "initialization_clearance" in config
                          else float(config["retract_distance"]) * 3)
        contact = np.asarray(self.contact_pose, dtype=float)
        if not np.isfinite(clearance) or clearance <= 0 or not np.all(np.isfinite(contact)):
            raise ReturnReadinessError("acquisition return target requires finite contact and positive clearance")
        ray = contact[:2] - self.anchor_pose[:2]
        length = float(np.linalg.norm(ray))
        if not np.isfinite(length):
            raise ReturnReadinessError("acquisition return target requires a finite observed search ray")
        target = contact.copy()
        if length <= 1e-10:
            target[:2] = self.anchor_pose[:2]
        else:
            target[:2] -= ray * (min(clearance, length) / length)
        target.setflags(write=False)
        self.return_target_pose = target

    def _finish_probe(self, now, pose, outcome):
        self.probe_end_time = now
        self.end_pose = pose.copy()
        self.outcome = outcome
        self.phase = "RETURN"
        self.return_settle_since = None
        self.return_settle_started_at = None

    def abort(self, reason):
        if not self.return_completed:
            self.outcome = "ABORTED"
            self.rejection_reason = reason

    def to_record(self):
        def vector(value):
            return None if value is None else np.asarray(value).tolist()
        return {
            "probe_id": self.probe_id, "anchor_pose": vector(self.anchor_pose),
            "return_target_pose": vector(self.return_target_pose),
            "probe_direction": vector(self.probe_direction),
            "probe_start_time": self.probe_start_time, "probe_end_time": self.probe_end_time,
            "return_end_time": self.return_end_time, "max_probe_distance": self.max_probe_distance,
            "contact": self.contact_pose is not None, "outcome": self.outcome,
            "contact_pose": vector(self.contact_pose), "end_pose": vector(self.end_pose),
            "returned_pose": vector(self.returned_pose), "max_force": self.max_force,
            "return_completed": self.return_completed, "accepted_as_boundary": self.accepted_as_boundary,
            "policy_state": self.policy_state, "purpose": self.purpose,
            "initialization_round": self.initialization_round,
            "recovery_id": self.recovery_id, "angle_deg": self.angle_deg,
            "rejection_reason": self.rejection_reason, "force_samples": list(self.force_samples),
        }
