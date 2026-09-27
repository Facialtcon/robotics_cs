"""Shared real/sim policy coordinator; geometry never comes from force or a map.

High-level states dispatch completed probe episodes to small handlers. Every
handler runs at the episode's settled return target, never at its contact endpoint.
"""
from enum import Enum
import numpy as np
from core.models import BoundaryPoint, PolicyCommand, PolicyWaypoint, RecoveryRay
from experiment_logging.termination import TerminationRecorder, TerminationReason
from policy.boundary_estimation import estimate_boundary, handed_tangent, left, unit
from policy.local_tracking import InsufficientClearanceError, LocalBoundaryTracker
from policy.local_recovery import BoundaryRecovery
from policy.probe_episode import ProbeEpisode, ReturnReadinessError
from safety.force_guard import ForceRateGuard, force_safety_reason


class State(str, Enum):
    TARGET_SEARCH = "TARGET_SEARCH"
    LOCAL_INITIALIZATION = "LOCAL_INITIALIZATION"
    BOUNDARY_TRACKING = "BOUNDARY_TRACKING"
    BOUNDARY_RECOVERY = "BOUNDARY_RECOVERY"
    BOUNDARY_CONFIRMATION = "BOUNDARY_CONFIRMATION"
    STOP = "STOP"
    STOP_SCAN = "STOP_SCAN"
    RETURN_TO_START = "RETURN_TO_START"
    LOOP_COMPLETE = "LOOP_COMPLETE"
    SEARCH = "TARGET_SEARCH"  # entry-point compatibility, not another state
    LOST = "STOP"


class RuleBasedPolicy:
    def __init__(self, config):
        self.config = dict(config)
        self._validate_config()
        self.state = State.TARGET_SEARCH
        self.current_target_direction = None
        self.current_tangent = None
        self.current_target = None
        self.possible_corner = False  # retained CSV field; no corner classifier
        self.boundary_points = []
        self.policy_waypoints = []
        self.probe_episodes = []
        self.active_episode = None
        self.boundary_recovery_rays = []
        self.recovery_records = []
        self.tracker = LocalBoundaryTracker(self.config, self.follow_hand)
        self.recovery = None
        self._motion_queue = []
        self._after_motion = None
        self._transfer_arrived_at = None
        self._transfer_settle_since = None
        self._transfer_previous_delta = None
        self._initial_contacts = []
        self._initial_anchors = []
        self._initial_probe_length = 0.0
        self._initial_round = 0
        self._initial_started_at = None
        self._initial_base_anchor = None
        self._initial_anchor_path = []
        self._initial_episodes = []
        self._initial_transfer_history = []
        self._initial_probe_entries = {}
        self._initial_direction = self.search_direction.copy()
        self._tracking_retry_counts = {}
        self._pending_tracking_clearance = None
        self._pending_tracking_approach_start = None
        self._tracking_entry = None
        self._tracking_approach_start = None
        self._confirmation_contacts = []
        self._confirmation_direction = None
        self._confirmation_anchor_path = []
        self._confirmation_retry_count = 0
        self._force_rate_guard = ForceRateGuard()
        self._started_at = None
        self.reason = ""
        self.sub_state = "READY"
        self.termination = TerminationRecorder()

    def _validate_config(self):
        positive = ("control_rate_hz", "contact_threshold", "contact_hold_time", "search_speed",
                    "probe_speed", "tangent_speed", "retract_speed", "retract_distance",
                    "tangent_step", "max_probe_distance", "max_search_distance",
                    "position_tolerance", "force_rate_limit", "safety_force_threshold",
                    "safety_torque_threshold", "absolute_raw_force_threshold", "absolute_raw_torque_threshold")
        for name in positive:
            if not np.isfinite(float(self.config[name])) or float(self.config[name]) <= 0:
                raise ValueError(f"policy.{name} must be finite and positive")
        if self.config["contact_threshold"] >= self.config["safety_force_threshold"]:
            raise ValueError("contact_threshold must be below safety_force_threshold")
        self.follow_hand = str(self.config.get("follow_hand", "CLOCKWISE")).upper()
        if self.follow_hand not in {"CLOCKWISE", "COUNTERCLOCKWISE"}:
            raise ValueError("invalid follow_hand")
        self.search_direction = unit(self.config["search_direction_xy"])
        if self.search_direction.shape != (2,):
            raise ValueError("search direction must be XY")
        sectors = self.config.get("recovery_sector_extents_deg", [20, 50, 90, 130, 170])
        if not sectors or not np.all(np.diff([0, *sectors]) > 0) or sectors[-1] >= 180:
            raise ValueError("recovery sectors must increase strictly between 0 and 180 degrees")
        if float(self.config.get("recovery_angular_resolution_deg", 10)) <= 0:
            raise ValueError("recovery angular resolution must be positive")
        retries = self.config.get("initialization_max_retries", 3)
        if isinstance(retries, bool) or not isinstance(retries, int) or not 0 <= retries <= 3:
            raise ValueError("initialization_max_retries must be an integer from 0 to 3")
        timeout = float(self.config.get("initialization_timeout_sec", 120.0))
        if not np.isfinite(timeout) or timeout <= 0:
            raise ValueError("initialization_timeout_sec must be finite and positive")
        readiness = {
            "return_settle_hold_sec": self.config.get("contact_hold_time", 0.05),
            "return_settled_speed_mps": 0.0005,
            "return_settle_timeout_sec": 2.0,
        }
        for name, default in readiness.items():
            value = float(self.config.get(name, default))
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"policy.{name} must be finite and positive")
            readiness[name] = value
        if readiness["return_settle_timeout_sec"] < readiness["return_settle_hold_sec"]:
            raise ValueError("return_settle_timeout_sec must be at least return_settle_hold_sec")
        for name, default in (("corner_approach_force_ratio", .5), ("corner_approach_speed", .002)):
            value = self.config.get(name, default)
            try:
                numeric = float(value)
            except (TypeError, ValueError) as exc:
                raise ValueError(f"policy.{name} must be finite and positive") from exc
            if isinstance(value, (bool, np.bool_)) or not np.isfinite(numeric) or numeric <= 0:
                raise ValueError(f"policy.{name} must be finite and positive")
        if float(self.config.get("corner_approach_force_ratio", .5)) >= 1:
            raise ValueError("policy.corner_approach_force_ratio must be below 1")

    def update(self, timestamp, raw, processed, robot):
        """Safety -> episode/motion execution -> returned-episode dispatch."""
        self.termination.observe(policy=self, robot=robot, raw=raw, processed=processed, timestamp=timestamp)
        pose = robot.pose
        if self._started_at is None:
            self._started_at = timestamp
        if self.state in {State.STOP, State.STOP_SCAN, State.RETURN_TO_START, State.LOOP_COMPLETE}:
            return self._command(pose)
        reason = self._safety_reason(timestamp, raw, processed, pose)
        if reason:
            self.request_stop(reason)
            return self._command(pose)
        if self._motion_queue:
            return self._execute_transfer(timestamp, processed, pose, tcp_speed=robot.tcp_speed)
        if self.active_episode is None:
            self._start_probe(timestamp, pose, self.search_direction,
                              self.config["max_search_distance"], "ACQUISITION")
        episode = self.active_episode
        try:
            motion = episode.tick(timestamp, pose, processed, self.config,
                                  tcp_speed=robot.tcp_speed)
        except ReturnReadinessError as exc:
            self.sub_state = episode.phase
            self.request_stop(f"anchor return failed: {exc}",
                              termination_reason=TerminationReason.STOP_ANCHOR_ERROR)
            return self._command(pose, wrench=processed)
        self.sub_state = episode.phase
        if episode.return_completed:
            self.active_episode = None
            self._waypoint(timestamp, pose, "RETURN_COMPLETED")
            self._dispatch_completed(episode, timestamp, pose)
            return self._command(pose)
        return self._command(pose, motion, processed)

    def _safety_reason(self, now, raw, processed, pose):
        if not np.all(np.isfinite(np.r_[raw.array(), processed.array(), pose])):
            return "nonfinite sensor/robot sample"
        reason = force_safety_reason(raw, processed, self.config)
        rate = self._force_rate_guard.update(now, float(np.hypot(processed.fx, processed.fy)))
        if reason:
            return reason
        if rate > float(self.config["force_rate_limit"]):
            return f"force rate exceeded: {rate:.3f} N/s"
        if (self.state == State.LOCAL_INITIALIZATION and self._initial_started_at is not None
                and now - self._initial_started_at >= float(self.config.get("initialization_timeout_sec", 120.0))):
            return f"local initialization timeout: round={self._initial_round + 1}"
        limit = self.config.get("max_runtime_sec")
        if limit is not None and now - self._started_at >= float(limit):
            return "policy runtime limit reached"
        return None

    def _command(self, pose, motion=None, wrench=None):
        target = pose.copy() if motion is None else motion[0].copy()
        delta = target[:2] - pose[:2]
        distance = float(np.linalg.norm(delta))
        moving = motion is not None and distance > 1e-10
        # Cap each commanded increment at the remaining distance (no endpoint overshoot).
        speed = min(float(motion[1]), distance * float(self.config["control_rate_hz"])) if moving else 0.0
        episode = self.active_episode
        command = PolicyCommand(
            state=self.state.value, move=moving,
            direction_xy=delta / distance if moving else np.zeros(2), speed=speed,
            target_pose=target, contact_flag=wrench is not None and np.hypot(wrench.fx, wrench.fy) >= self.config["contact_threshold"],
            possible_corner=False, reason=self.reason, policy_sub_state=self.sub_state,
            recovery_id=None if episode is None else episode.recovery_id,
            ray_index=None if episode is None else episode.probe_id,
            ray_theta_deg=None if episode is None else episode.angle_deg,
            probe_id=None if episode is None else episode.probe_id,
        )
        self.termination.observe(command=command)
        return command

    def _start_probe(self, now, anchor, direction, length, purpose, angle=0.0):
        if self.active_episode is not None:
            raise RuntimeError("cannot start another episode before anchor return")
        if purpose != "INITIALIZATION":
            self._initial_transfer_history = []
            self._initial_probe_entries = {}
        recovery_id = len(self.recovery_records) - 1 if purpose in {"RECOVERY", "CONFIRMATION"} else None
        self.active_episode = ProbeEpisode(len(self.probe_episodes), anchor, direction, now,
                                           float(length), self.state.value, purpose, recovery_id, angle)
        if purpose == "INITIALIZATION":
            self.active_episode.initialization_round = self._initial_round + 1
            self._initial_episodes.append(self.active_episode)
        self.probe_episodes.append(self.active_episode)
        # An approach path belongs only to the probe it actually produced.
        # Recovery/initialization rays must never reuse a prior tracking entry.
        self._tracking_entry = None
        self._tracking_approach_start = None
        self._pending_tracking_clearance = None
        self._pending_tracking_approach_start = None
        self.sub_state = "PROBE"
        self._waypoint(now, anchor, "PROBE_ANCHOR")

    def _dispatch_completed(self, episode, now, pose):
        assert episode.return_completed
        handlers = {"ACQUISITION": self._acquired, "INITIALIZATION": self._initialized,
                    "TRACKING": self._tracked, "RECOVERY": self._recovered,
                    "CONFIRMATION": self._confirmed}
        handlers[episode.purpose](episode, now, pose)

    def _queue_transfer(self, poses, callback):
        self._motion_queue = [(pose.copy(), phase) for pose, phase in poses]
        self._after_motion = callback
        self._transfer_arrived_at = None
        self._transfer_settle_since = None
        self._transfer_previous_delta = None
        self.sub_state = self._motion_queue[0][1]

    def _execute_transfer(self, now, wrench, pose, tcp_speed=None):
        target, phase = self._motion_queue[0]
        self.sub_state = phase
        fxy = float(np.hypot(wrench.fx, wrench.fy))
        if fxy >= self.config["contact_threshold"]:
            self.request_stop(
                f"unexpected contact during anchor transfer: phase={phase}, "
                f"Fxy={fxy:.6f} N, threshold={self.config['contact_threshold']:.6f} N",
                termination_reason=TerminationReason.STOP_UNEXPECTED_CONTACT)
            return self._command(pose, wrench=wrench)
        try:
            speed = np.zeros(3) if tcp_speed is None else np.asarray(tcp_speed, dtype=float)
            if speed.shape not in ((3,), (6,)) or not np.all(np.isfinite(speed[:3])):
                raise ValueError("expected finite XYZ linear speed")
        except (TypeError, ValueError):
            self.request_stop(f"anchor transfer invalid TCP linear speed: phase={phase}",
                              termination_reason=TerminationReason.STOP_MOTION_ERROR)
            return self._command(pose, wrench=wrench)
        delta = target[:2] - pose[:2]
        position_error = float(np.linalg.norm(delta))
        at_target = position_error <= float(self.config["position_tolerance"])
        crossed_target = (self._transfer_previous_delta is not None
                          and float(np.dot(delta, self._transfer_previous_delta)) < 0)
        self._transfer_previous_delta = delta.copy()
        linear_speed = float(np.linalg.norm(speed[:3]))
        speed_limit = float(self.config.get("return_settled_speed_mps", .0005))
        hold_time = float(self.config.get("return_settle_hold_sec", self.config["contact_hold_time"]))
        timeout = float(self.config.get("return_settle_timeout_sec", 2.0))
        # A delayed sample may skip the entire tolerance band. Crossing the
        # endpoint must also stop and start the bounded settling window.
        if (at_target or crossed_target) and self._transfer_arrived_at is None:
            self._transfer_arrived_at = now
        ready = at_target and linear_speed <= speed_limit
        if ready:
            if self._transfer_settle_since is None:
                self._transfer_settle_since = now
        else:
            self._transfer_settle_since = None
        held = 0.0 if self._transfer_settle_since is None else now - self._transfer_settle_since
        elapsed = 0.0 if self._transfer_arrived_at is None else now - self._transfer_arrived_at
        if ready and held >= hold_time and elapsed <= timeout:
            self._waypoint(now, pose, phase + "_COMPLETED")
            if self.state == State.LOCAL_INITIALIZATION:
                self._remember_initial_pose(pose)
            if phase == "CONFIRMATION_CLEARANCE":
                # A rejected confirmation must retrace this measured ray
                # waypoint before returning to the previous probe's anchor.
                self._confirmation_anchor_path.append(pose.copy())
            if phase in {"RAY_CLEARANCE", "CLEARANCE_ROLLBACK"}:
                self._pending_tracking_clearance = pose.copy()
                if (len(self._motion_queue) > 1
                        and self._motion_queue[1][1] in {"RAY_CLEARANCE", "CLEARANCE_ROLLBACK"}):
                    # Keep adjacent executed legs, never replace a rollback
                    # polyline by a diagonal from its first to last point.
                    self._pending_tracking_approach_start = pose.copy()
            self._motion_queue.pop(0)
            self._transfer_arrived_at = None
            self._transfer_settle_since = None
            self._transfer_previous_delta = None
            if not self._motion_queue:
                callback, self._after_motion = self._after_motion, None
                callback(now, pose)
            return self._command(pose, wrench=wrench)
        if self._transfer_arrived_at is not None and elapsed >= timeout:
            self.request_stop(
                f"anchor transfer readiness timeout: phase={phase}, "
                f"position_error={position_error * 1000:.6f} mm "
                f"(limit={self.config['position_tolerance'] * 1000:.6f} mm), "
                f"tcp_linear_speed={linear_speed * 1000:.6f} mm/s "
                f"(limit={speed_limit * 1000:.6f} mm/s), Fxy={fxy:.6f} N, "
                f"ready_hold={held:.6f}/{hold_time:.6f} s, elapsed={elapsed:.6f}/{timeout:.6f} s",
                termination_reason=TerminationReason.STOP_ANCHOR_ERROR)
            return self._command(pose, wrench=wrench)
        # Stop at first arrival, keeping this waypoint until the measured TCP
        # has settled. If braking carries it outside tolerance, finish stopping
        # before correcting the same target at the local probe speed.
        if at_target or crossed_target or (self._transfer_arrived_at is not None and linear_speed > speed_limit):
            return self._command(pose, wrench=wrench)
        requested_speed = float(self.config["tangent_speed"])
        if self._transfer_arrived_at is not None:
            requested_speed = min(requested_speed, float(self.config["probe_speed"]))
        return self._command(pose, (target, requested_speed), wrench)

    def _acquired(self, episode, now, pose):
        if episode.outcome != "CONTACT":
            self.request_stop("target acquisition failed after return to search anchor")
            return
        self.state = State.LOCAL_INITIALIZATION
        # SEARCH uses a different speed and may trigger at a different depth.
        # Its contact only locates the neighborhood; fit three fresh low-speed
        # probes, including the center ray, under the same approach conditions.
        self._initial_contacts = []
        self._initial_round = 0
        self._initial_started_at = now
        self._initial_direction = self.search_direction.copy()
        # Acquisition has already retreated along its observed search ray to
        # the local return target. Continue from its actual settled TCP pose;
        # never return to P0 and then traverse the search ray a second time.
        self._initial_base_anchor = pose.copy()
        self._initial_transfer_history = [pose.copy()]
        self._initial_probe_entries = {}
        self._initial_probe_length = float(self.config.get("initialization_probe_distance", self.config["max_probe_distance"] * 2))
        self._waypoint(now, pose, "INITIAL_ANCHOR_COMPLETED")
        self._begin_initial_round(now, pose)

    def _begin_initial_round(self, now, pose):
        # One repeat tests repeatability. Then try contiguous one-sided windows
        # to find a local face when the first window straddled a transition.
        # No inferred object geometry or pooling/cherry-picking across rounds.
        windows = ((-1, 0, 1), (-1, 0, 1), (1, 2, 3), (-3, -2, -1))
        offset = float(self.config.get("initialization_lateral_offset", self.config["tangent_step"] / 2))
        lateral = left(self._initial_direction)
        self._initial_contacts = []
        self._initial_episodes = []
        self._initial_anchor_path = [pose.copy()]
        # Keep earlier rounds and their actual rollback endpoints: a later
        # window can still need the verified path that entered reinitialization.
        self._remember_initial_pose(pose)
        self._initial_probe_entries = {}
        self._initial_anchors = []
        for sign in windows[self._initial_round]:
            nearby = self._initial_base_anchor.copy()
            nearby[:2] += sign * offset * lateral
            self._initial_anchors.append(nearby)
        self._waypoint(now, pose, f"INITIALIZATION_ROUND_{self._initial_round + 1}")
        self._next_initial_probe(now, pose)

    def _next_initial_probe(self, now, pose):
        if not self._initial_anchors:
            self._finish_initialization(now, pose)
            return
        anchor = self._initial_anchors.pop(0)
        self._queue_transfer([(anchor, "LATERAL_OFFSET")], self._start_initial_probe)

    def _start_initial_probe(self, now, pose):
        self._initial_anchor_path.append(pose.copy())
        self._remember_initial_pose(pose)
        self._start_probe(now, pose, self._initial_direction,
                          self._initial_probe_length, "INITIALIZATION")
        self._initial_probe_entries[self.active_episode.probe_id] = (
            self.active_episode.anchor_pose.copy(), len(self._initial_transfer_history) - 1)

    def _remember_initial_pose(self, pose):
        """Record executed endpoints, including probe returns, without planning."""
        if not self._initial_transfer_history or not np.array_equal(self._initial_transfer_history[-1], pose):
            self._initial_transfer_history.append(pose.copy())

    def _initial_entry_index(self, episode):
        entry = self._initial_probe_entries.get(episode.probe_id)
        if entry is None or not np.array_equal(entry[0], episode.anchor_pose):
            return None
        index = entry[1]
        if (not isinstance(index, int) or not 0 <= index < len(self._initial_transfer_history)
                or not np.array_equal(self._initial_transfer_history[index], episode.anchor_pose)):
            return None
        return index

    def _retry_initialization(self, now, pose, details):
        reason = f"local initialization failed: round={self._initial_round + 1}, {details}"
        retries = int(self.config.get("initialization_max_retries", 3))
        exhausted = self._initial_round >= retries
        if exhausted:
            reason = f"local initialization retries exhausted: {reason}"
        for episode in self._initial_episodes:
            episode.rejection_reason = reason
        # Also support callers that supplied an explicit completed-contact batch.
        for episode in self._initial_contacts:
            episode.rejection_reason = reason
        if exhausted:
            self.request_stop(reason)
            return
        self._initial_round += 1
        self.reason = (
            f"INITIALIZATION_RETRY {self._initial_round}/{retries}: {details}; "
            "return along anchor path, then collect three fresh contacts"
        )
        print(self.reason)
        # All probes have already returned. Retrace the executed anchor-transfer
        # polyline before starting another window, never cross from contact.
        path = [(p, "INITIALIZATION_ROLLBACK") for p in reversed(self._initial_anchor_path[:-1])]
        self._queue_transfer(path, self._begin_initial_round)

    def _initialized(self, episode, now, pose):
        self._remember_initial_pose(pose)
        if episode.outcome == "CONTACT":
            self._initial_contacts.append(episode)
        self._next_initial_probe(now, pose)

    def _finish_initialization(self, now, pose):
        contacts = self._initial_contacts
        poses = [e.contact_pose for e in contacts]
        min_spacing = float(self.config.get("minimum_boundary_point_spacing", 0.001))
        residual_limit = float(self.config.get("local_fit_max_residual", 0.001))
        # Measure the complete initialization batch, even if the tracking
        # window is configured smaller. Never silently accept two samples.
        estimate = estimate_boundary(poses, self._initial_direction, self.follow_hand, min_span=0.0)
        spacing = min(
            (float(np.linalg.norm(a[:2] - b[:2]))
             for i, a in enumerate(poses) for b in poses[i + 1:]),
            default=0.0,
        )
        residual_text = "n/a" if estimate is None else f"{estimate.residual * 1000:.3f} mm"
        details = (
            f"contacts={len(contacts)}/3, min_spacing={spacing * 1000:.3f} mm "
            f"(required>={min_spacing * 1000:.3f} mm), residual={residual_text} "
            f"(limit={residual_limit * 1000:.3f} mm), "
            f"probe_speed={float(self.config['probe_speed']):.6f} m/s"
        )
        if (len(contacts) < 3 or estimate is None or spacing < min_spacing
                or estimate.residual > residual_limit):
            self._retry_initialization(now, pose, details)
            return
        ordered = sorted(contacts, key=lambda e: np.dot(e.contact_pose[:2], estimate.tangent))
        if (self.current_tangent is not None
                and np.dot(estimate.tangent, self.current_tangent) < np.cos(np.deg2rad(135))):
            self._retry_initialization(now, pose, "local reinitialization reversed the follow direction")
            return
        estimate = self.tracker.update([e.contact_pose for e in ordered], self._initial_direction, reset=True)
        if estimate is None:
            self._retry_initialization(now, pose, f"tracking fit failed: {details}")
            return
        self.reason = f"local initialization complete: round={self._initial_round + 1}, {details}"
        print(self.reason)
        # Fit unordered initialization samples; publish in follow-hand order.
        self._set_estimate(estimate)
        # Reinitialization samples may overlap the already published boundary.
        # Keep them for the local fit, but publish only new forward samples,
        # measuring progress in the NEW face's frame (also valid at sharp turns).
        published = [point.pose for point in self.boundary_points]
        fresh = []
        for item in ordered:
            contact = item.contact_pose
            if self.boundary_points and (np.dot(contact[:2] - published[-1][:2], estimate.tangent) < min_spacing
                              or any(np.linalg.norm(contact[:2] - p[:2]) < min_spacing for p in published[-8:])):
                continue
            fresh.append(item)
            published.append(contact)
        self._accept(fresh)
        if self.state == State.STOP:
            return
        # Publication order and the next ray must share the same frontier.
        # Reach its anchor via the already executed polyline, not a diagonal
        # from the last collected ray to a different ray's clearance point.
        frontier = ordered[-1]
        history_index = self._initial_entry_index(frontier)
        if history_index is not None:
            # This includes actual RETURN endpoints between probe anchors.
            path = [(p, "INITIALIZATION_ALIGN") for p in
                    reversed(self._initial_transfer_history[history_index:-1])]
        else:
            # Legacy explicit completed-batch callers have no transfer ledger.
            # Alignment compatibility does not grant them an invented retreat.
            frontier_index = next(i for i, e in enumerate(self._initial_episodes) if e is frontier)
            path = [(p, "INITIALIZATION_ALIGN") for p in
                    reversed(self._initial_anchor_path[frontier_index + 1:-1])]

        def resume(t, p):
            self._resume_tracking(frontier, now=t, pose=p, reference_contact=self.boundary_points[-1].pose)

        if path:
            self._queue_transfer(path, resume)
        else:
            resume(now, pose)

    def _set_estimate(self, estimate):
        self.current_tangent = estimate.tangent.copy()
        self.current_target_direction = estimate.target_side.copy()

    def _accept(self, episodes):
        for episode in episodes:
            if episode.accepted_as_boundary:
                continue
            episode.accepted_as_boundary = True
            self.boundary_points.append(BoundaryPoint(
                len(self.boundary_points), episode.contact_time, episode.contact_pose.copy(), episode.contact_wrench,
                self.current_target_direction.copy(), self.current_tangent.copy(), False))
        limit = self.config.get("max_boundary_points")
        if limit is not None and len(self.boundary_points) >= int(limit):
            self.request_stop("boundary point limit reached")

    def _resume_tracking(self, episode, *, now=None, pose=None, tangent_step=None, reference_contact=None):
        if self.state == State.STOP:
            return
        self.state = State.BOUNDARY_TRACKING
        phase = "RAY_CLEARANCE"
        rollback_via = None
        try:
            clearance, anchor = self.tracker.next_anchor_path(
                episode, tangent_step=tangent_step, reference_contact=reference_contact)
        except InsufficientClearanceError as exc:
            if episode.purpose == "INITIALIZATION":
                self._resume_initial_clearance(episode, now, pose, tangent_step, reference_contact, exc)
                return
            # A capped short ray is not permission to transfer close to contact.
            # Retrace the actually executed approach of THIS probe. The final
            # tangent leg alone may be too close under the newly observed fit;
            # its preceding clearance leg can still lead to a usable free point.
            entry = self._tracking_entry
            if (entry is None or entry[0] != episode.probe_id
                    or not np.array_equal(entry[2], episode.anchor_pose)):
                self.request_stop(f"tracking anchor planning failed: {exc}; no verified tracking entry path",
                                  termination_reason=TerminationReason.STOP_ANCHOR_ERROR)
                return
            try:
                clearance, anchor = self.tracker.next_anchor_path(
                    episode, tangent_step=tangent_step, reference_contact=reference_contact,
                    verified_clearance_pose=entry[1])
            except InsufficientClearanceError as entry_error:
                approach = self._tracking_approach_start
                if (approach is None or approach[0] != episode.probe_id
                        or not np.array_equal(approach[2], episode.anchor_pose)):
                    self.request_stop(
                        f"tracking anchor planning failed: {entry_error}; verified entry clearance unavailable; "
                        "no matching preceding clearance leg",
                        termination_reason=TerminationReason.STOP_ANCHOR_ERROR)
                    return
                try:
                    clearance, anchor = self.tracker.next_anchor_path(
                        episode, tangent_step=tangent_step, reference_contact=reference_contact,
                        verified_clearance_pose=approach[1])
                except ValueError as approach_error:
                    self.request_stop(
                        f"tracking anchor planning failed: {approach_error}; verified approach clearance unavailable",
                        termination_reason=TerminationReason.STOP_ANCHOR_ERROR)
                    return
                # A -> Q -> B is the reverse of the executed B -> Q -> A.
                # Both legs retain the ordinary transfer force/readiness guards.
                rollback_via = entry[1].copy()
            except ValueError as fallback_error:
                self.request_stop(f"tracking anchor planning failed: {fallback_error}; verified entry clearance unavailable",
                                  termination_reason=TerminationReason.STOP_ANCHOR_ERROR)
                return
            phase = "CLEARANCE_ROLLBACK"
            legs = 2 if rollback_via is not None else 1
            self.reason = (f"TRACKING_CLEARANCE_LIMITED: {exc}; retrace {legs} verified "
                           "transfer leg(s) before tangent step")
            print(self.reason)
            self._waypoint(episode.return_end_time if now is None else now,
                           episode.returned_pose if pose is None else pose,
                           "TRACKING_CLEARANCE_LIMITED")
        except ValueError as exc:
            # No ray/normal geometry means there is no justified transfer.
            episode.rejection_reason = "FIT_FAILURE"
            self._reinitialize_tracking(episode, episode.return_end_time if now is None else now,
                                        episode.returned_pose if pose is None else pose,
                                        f"invalid anchor geometry: {exc}")
            return
        self._pending_tracking_clearance = None
        start = episode.returned_pose if pose is None else pose
        self._pending_tracking_approach_start = None if start is None else start.copy()
        path = [] if rollback_via is None else [(rollback_via, phase)]
        self._queue_transfer(path + [(clearance, phase), (anchor, "TANGENT_STEP")],
                             self._begin_tracking_probe)

    def _resume_initial_clearance(self, episode, now, pose, tangent_step, reference_contact, ray_error):
        """Retrace this frontier's own entry history until normal clearance fits."""
        index = self._initial_entry_index(episode)
        start = episode.returned_pose if pose is None else pose
        if (index is None or start is None
                or np.linalg.norm(start[:2] - episode.anchor_pose[:2]) > self.config["position_tolerance"]):
            self.request_stop(
                f"initialization frontier probe={episode.probe_id} anchor planning failed: "
                f"{ray_error}; no matching initialization entry path",
                termination_reason=TerminationReason.STOP_ANCHOR_ERROR)
            return
        path = []
        last_error = ray_error
        for previous in reversed(self._initial_transfer_history[:index]):
            path.append((previous, "CLEARANCE_ROLLBACK"))
            try:
                clearance, anchor = self.tracker.next_anchor_path(
                    episode, tangent_step=tangent_step, reference_contact=reference_contact,
                    verified_clearance_pose=previous)
            except InsufficientClearanceError as exc:
                last_error = exc
                continue
            except ValueError as exc:
                last_error = exc
                break
            self.reason = (f"INITIALIZATION_CLEARANCE_LIMITED: frontier probe={episode.probe_id}; "
                           f"retrace {len(path)} executed legs before tangent step")
            print(self.reason)
            self._waypoint(episode.return_end_time if now is None else now, start,
                           "INITIALIZATION_CLEARANCE_LIMITED")
            self._pending_tracking_clearance = None
            self._pending_tracking_approach_start = start.copy()
            self._queue_transfer(path + [(anchor, "TANGENT_STEP")], self._begin_tracking_probe)
            return
        self.request_stop(
            f"initialization frontier probe={episode.probe_id} anchor planning failed: "
            f"{last_error}; verified initialization entry path has insufficient clearance",
            termination_reason=TerminationReason.STOP_ANCHOR_ERROR)

    def _begin_tracking_probe(self, now, pose):
        clearance = self._pending_tracking_clearance
        approach_start = self._pending_tracking_approach_start
        self._start_probe(now, pose, self.current_target_direction,
                          self.config["max_probe_distance"], "TRACKING")
        if clearance is not None:
            self._tracking_entry = (self.active_episode.probe_id, clearance.copy(),
                                    self.active_episode.anchor_pose.copy())
            if approach_start is not None:
                self._tracking_approach_start = (self.active_episode.probe_id, approach_start.copy(),
                                                self.active_episode.anchor_pose.copy())
        self._pending_tracking_clearance = None
        self._pending_tracking_approach_start = None

    def _tracked(self, episode, now, pose):
        if episode.outcome == "NO_CONTACT":
            self.termination.set_stop_reason(
                TerminationReason.STOP_NO_CONTACT, detail="NO_CONTACT", policy=self,
                terminal=False, source="policy._tracked")
            episode.rejection_reason = "NO_CONTACT"
            self._waypoint(now, pose, "TRACKING_NO_CONTACT")
            self._enter_recovery(now, pose)
            return
        if episode.outcome != "CONTACT":
            # A transient threshold crossing does not establish a missing edge.
            episode.rejection_reason = "UNSTABLE_CONTACT"
            attempts = self._tracking_retry_counts.get("UNSTABLE_CONTACT", 0) + 1
            self._tracking_retry_counts["UNSTABLE_CONTACT"] = attempts
            if attempts > 2:
                self.request_stop("tracking unstable contact after two same-ray retries")
            else:
                self.reason = "UNSTABLE_CONTACT: retry the returned probe ray"
                self._start_probe(now, pose, episode.probe_direction, episode.max_probe_distance, "TRACKING")
            return
        displacement = episode.contact_pose[:2] - self.boundary_points[-1].pose[:2]
        progress = float(np.dot(displacement, self.current_tangent))
        spacing = float(self.config.get("minimum_boundary_point_spacing", 0.001))
        print(f"TRACKING observation probe={episode.probe_id}: current contact point [mm]="
              f"{(episode.contact_pose[:2] * 1000).tolist()}, old tangent={self.current_tangent.tolist()}, "
              f"probe direction={episode.probe_direction.tolist()}, measured tangent progress [mm]={progress * 1000:.6f}")
        repeated = any(np.linalg.norm(episode.contact_pose[:2] - point.pose[:2]) < spacing
                       for point in self.boundary_points[-8:])
        if repeated or progress < spacing:
            failure = "REPEATED_CONTACT" if repeated else "NO_FORWARD_PROGRESS"
            self._correct_tracking_anchor(episode, now, pose, failure)
            return
        old_contacts, old_estimate = self.tracker.contacts, self.tracker.estimate
        estimate = self.tracker.update([episode.contact_pose], episode.probe_direction)
        if estimate is None or np.dot(estimate.tangent, self.current_tangent) < 0:
            self.tracker.contacts, self.tracker.estimate = old_contacts, old_estimate
            episode.rejection_reason = "FIT_FAILURE"
            self._reinitialize_tracking(episode, now, pose, "inconsistent or reversed local fit")
            return
        self._tracking_retry_counts.clear()
        self._set_estimate(estimate)
        self._accept([episode])
        if self._loop_closed():
            self.termination.set_stop_reason(
                TerminationReason.SUCCESS, detail="local contact trajectory loop closed",
                policy=self, source="policy._tracked")
            self.state, self.reason = State.LOOP_COMPLETE, "local contact trajectory loop closed"
            return
        self._resume_tracking(episode, now=now, pose=pose)

    def _correct_tracking_anchor(self, episode, now, pose, failure):
        """Repair local sampling progress at the returned ray, without sector search."""
        self.termination.set_stop_reason(
            TerminationReason.STOP_REPEATED_CONTACT if failure == "REPEATED_CONTACT"
            else TerminationReason.STOP_NO_FORWARD_PROGRESS,
            detail=failure, policy=self, terminal=False, source="policy._correct_tracking_anchor")
        episode.rejection_reason = failure
        self._waypoint(now, pose, "TRACKING_" + failure)
        attempts = self._tracking_retry_counts.get(failure, 0) + 1
        self._tracking_retry_counts[failure] = attempts
        if attempts > 2:
            self._reinitialize_tracking(episode, now, pose, failure + " after two anchor corrections")
            return
        step = float(self.config["tangent_step"])
        if failure == "REPEATED_CONTACT":
            # Re-establish the handed tangent from accepted contacts, excluding
            # the duplicate, and advance the sampling anchor farther along it.
            estimate = estimate_boundary(self.tracker.contacts, self.current_target_direction, self.follow_hand)
            if estimate is None or np.dot(estimate.tangent, self.current_tangent) <= 0:
                self._reinitialize_tracking(episode, now, pose, "repeated contact has no consistent tangent")
                return
            self.tracker.estimate = estimate
            self._set_estimate(estimate)
            step *= 1.0 + 0.5 * attempts
        planned_progress = float(np.dot(episode.anchor_pose[:2] - self.boundary_points[-1].pose[:2],
                                        self.current_tangent))
        self.reason = (f"{failure}: local anchor correction {attempts}/2; "
                       f"previous anchor tangent progress={planned_progress * 1000:.3f} mm")
        print(self.reason)
        self._resume_tracking(episode, now=now, pose=pose, tangent_step=step,
                              reference_contact=self.boundary_points[-1].pose)

    def _reinitialize_tracking(self, episode, now, pose, details):
        """Reuse bounded initialization near a returned contact, preserving search calibration."""
        self.termination.set_stop_reason(
            TerminationReason.STOP_FIT_FAILURE, detail=details, policy=self,
            terminal=False, source="policy._reinitialize_tracking")
        attempts = self._tracking_retry_counts.get("FIT_FAILURE", 0) + 1
        self._tracking_retry_counts["FIT_FAILURE"] = attempts
        if attempts > 2:
            self.request_stop("FIT_FAILURE: local reinitialization exhausted: " + details)
            return
        self.state = State.LOCAL_INITIALIZATION
        self.reason = "FIT_FAILURE: reinitialize local boundary: " + details
        print(self.reason)
        self._waypoint(now, pose, "TRACKING_FIT_FAILURE")
        self._initial_direction = episode.probe_direction.copy()
        self._initial_base_anchor = pose.copy()
        # Capture only adjacent, executed segments belonging to this source
        # probe, before INITIALIZATION correctly clears its tracking binding.
        self._initial_transfer_history = []
        self._initial_probe_entries = {}
        entry, approach = self._tracking_entry, self._tracking_approach_start
        if (entry is not None and entry[0] == episode.probe_id
                and np.array_equal(entry[2], episode.anchor_pose)):
            if (approach is not None and approach[0] == episode.probe_id
                    and np.array_equal(approach[2], episode.anchor_pose)):
                self._remember_initial_pose(approach[1])
            self._remember_initial_pose(entry[1])
            self._remember_initial_pose(episode.anchor_pose)
        self._remember_initial_pose(pose)
        self._initial_probe_length = float(self.config.get("initialization_probe_distance", episode.max_probe_distance))
        self._initial_round = 0
        self._initial_started_at = now
        self._begin_initial_round(now, pose)

    def _enter_recovery(self, now, pose):
        if not self.config.get("boundary_recovery_enabled", True):
            self.request_stop("boundary lost; recovery disabled")
            return
        self.state = State.BOUNDARY_RECOVERY
        self.recovery = BoundaryRecovery(pose, self.current_target_direction.copy(), self.current_tangent.copy(),
                                         self.boundary_points[-1].pose.copy(), self.config)
        self.recovery_records.append({"corner_id": len(self.recovery_records), "timestamp": now,
            "pc": self.boundary_points[-1].pose.copy(), "p_clear": pose.copy(), "p_anchor": pose.copy(),
            "old_target_direction": self.current_target_direction.copy(),
            "old_tangent": self.current_tangent.copy(), "confirmed_contact": None})
        self._next_recovery_probe(now, pose)

    def _next_recovery_probe(self, now, pose):
        self.state = State.BOUNDARY_RECOVERY
        # Every ray belongs to the same nominal free-space anchor. Do not
        # redefine that anchor from each return's tolerated position error.
        if np.linalg.norm(pose[:2] - self.recovery.anchor[:2]) > self.config["position_tolerance"]:
            self._queue_transfer([(self.recovery.anchor, "RECOVERY_ANCHOR_ALIGN")],
                                 self._next_recovery_probe)
            return
        result = self.recovery.next_direction()
        if result is None:
            self.request_stop("boundary recovery exhausted expanded local sectors")
            return
        angle, direction = result
        self.reason = (f"RECOVERY probe {self.recovery.cursor}/{len(self.recovery.angles)}: "
                       f"angle={angle:.1f} deg")
        print(f"RECOVERY_DIRECTION {self.reason} "
              f"anchor_mm={(self.recovery.anchor[:2] * 1000).tolist()} "
              f"direction={direction.tolist()} old_tangent={self.recovery.old_tangent.tolist()}")
        self._start_probe(now, self.recovery.anchor, direction, self.config.get("recovery_ray_length", self.config["max_probe_distance"]),
                          "RECOVERY", angle)

    def _recovered(self, episode, now, pose):
        self.boundary_recovery_rays.append(RecoveryRay(
            episode.recovery_id, episode.probe_id, episode.angle_deg, episode.anchor_pose.copy(),
            episode.probe_direction.copy(), episode.max_probe_distance,
            float(np.linalg.norm(episode.end_pose[:2] - episode.anchor_pose[:2])),
            None if episode.contact_pose is None else episode.contact_pose.copy(), episode.outcome))
        rejection = "" if episode.outcome != "CONTACT" else self.recovery.candidate_rejection(episode.contact_pose, self.boundary_points)
        episode.rejection_reason = rejection
        if episode.outcome != "CONTACT" or rejection:
            self._next_recovery_probe(now, pose)
            return
        self.recovery.candidates.append(episode.contact_pose.copy())
        self.state = State.BOUNDARY_CONFIRMATION
        self._confirmation_contacts = [episode]
        self._confirmation_direction = episode.probe_direction.copy()
        self._confirmation_anchor_path = [pose.copy()]
        self._confirmation_retry_count = 0
        self._next_confirmation_probe(now, pose)

    def _next_confirmation_probe(self, now, pose):
        contacts = self._confirmation_contacts
        step = float(self.config.get("confirm_step", self.config["tangent_step"] / 2))
        if len(contacts) == 1:
            # One contact cannot establish a face. Keep the existing provisional
            # lateral sample until a second contact validates local geometry.
            anchor = pose.copy()
            anchor[:2] += handed_tangent(self._confirmation_direction, self.follow_hand) * step
            path = [(anchor, "CONFIRMATION_OFFSET")]
        else:
            _, estimate = self._confirmation_fit()
            if estimate is None:
                self._reject_confirmation(contacts[-1], "CONFIRMATION_UNSTABLE_TANGENT")
                return
            # Use the same validated local suffix for BOTH the probe normal and
            # anchor. Rotating only the probe from the old oblique-ray anchor
            # can place the next contact behind the latest confirmed contact.
            planner = LocalBoundaryTracker(self.config, self.follow_hand)
            planner.estimate = estimate
            try:
                clearance, anchor = planner.next_anchor_path(contacts[-1], tangent_step=step)
            except ValueError as exc:
                self._reject_confirmation(contacts[-1], f"CONFIRMATION_ANCHOR_ERROR: {exc}")
                return
            previous_direction = self._confirmation_direction.copy()
            self._confirmation_direction = estimate.target_side.copy()
            # The temporary fit is only for sampling. Publish/replace the main
            # tracking estimate only after all existing confirmation checks pass.
            print(
                f"CONFIRMATION_DIRECTION probe={contacts[-1].probe_id} "
                f"old_direction={previous_direction.tolist()} "
                f"probe_direction={self._confirmation_direction.tolist()} "
                f"tangent={estimate.tangent.tolist()} "
                f"contact_mm={(contacts[-1].contact_pose[:2] * 1000).tolist()} "
                f"next_anchor_mm={(anchor[:2] * 1000).tolist()}"
            )
            path = [(clearance, "CONFIRMATION_CLEARANCE"), (anchor, "CONFIRMATION_OFFSET")]
        # Preserve the settled return start as well as every actual intermediate
        # clearance; rollback must never replace the executed polyline by a chord.
        if not np.array_equal(pose, self._confirmation_anchor_path[-1]):
            self._confirmation_anchor_path.append(pose.copy())
        self._queue_transfer(path, self._start_confirmation_probe)

    def _start_confirmation_probe(self, now, pose):
        self._confirmation_anchor_path.append(pose.copy())
        self._start_probe(now, pose, self._confirmation_direction,
                          self.config.get("recovery_ray_length", self.config["max_probe_distance"]), "CONFIRMATION")

    def _confirmed(self, episode, now, pose):
        if episode.outcome == "UNSTABLE_CONTACT":
            self._confirmation_retry_count += 1
            episode.rejection_reason = "CONFIRMATION_UNSTABLE_CONTACT"
            if self._confirmation_retry_count <= 2:
                self.reason = ("CONFIRMATION_UNSTABLE_CONTACT: retry the returned probe ray "
                               f"{self._confirmation_retry_count}/2; probe={episode.probe_id}")
                print(self.reason)
                self._waypoint(now, pose, "CONFIRMATION_UNSTABLE_CONTACT_RETRY")
                # Return readiness has already passed. Keep the immutable ray
                # anchor and all valid contacts; do not offset or expand sectors.
                self._start_probe(now, episode.anchor_pose, episode.probe_direction,
                                  episode.max_probe_distance, "CONFIRMATION")
            else:
                self._reject_confirmation(episode, "CONFIRMATION_UNSTABLE_CONTACT")
            return
        if episode.outcome != "CONTACT":
            self._reject_confirmation(episode, "CONFIRMATION_NO_CONTACT")
            return
        self._confirmation_retry_count = 0
        previous = self._confirmation_contacts[-1].contact_pose
        if np.linalg.norm(episode.contact_pose[:2] - previous[:2]) < float(self.config.get("minimum_boundary_point_spacing", 0.001)):
            self._reject_confirmation(episode, "CONFIRMATION_NO_DISPLACEMENT")
            return
        self._confirmation_contacts.append(episode)
        fit_contacts, estimate = self._confirmation_fit()
        if estimate is None:
            self._reject_confirmation(episode, "CONFIRMATION_UNSTABLE_TANGENT")
            return
        displacement = episode.contact_pose[:2] - previous[:2]
        if np.dot(displacement, estimate.tangent) <= 0 or np.dot(estimate.tangent, self.recovery.old_tangent) < np.cos(np.deg2rad(135)):
            self._reject_confirmation(episode, "CONFIRMATION_REVERSED")
            return
        required = max(3, int(self.config.get("confirmation_contact_count", 3)))
        if len(self._confirmation_contacts) < required:
            self._next_confirmation_probe(now, pose)
            return
        # A recovery ray may touch the transition itself. Accept only the newest
        # locally collinear suffix, never force two sides into one PCA line.
        estimate = self.tracker.update([e.contact_pose for e in fit_contacts], self._confirmation_direction, reset=True)
        if estimate is None:
            self._reject_confirmation(episode, "CONFIRMATION_UNSTABLE_TANGENT")
            return
        self._set_estimate(estimate)
        self._accept(fit_contacts)
        self._tracking_retry_counts.clear()
        self.recovery_records[-1]["confirmed_contact"] = episode.contact_pose.copy()
        if self._loop_closed():
            self.termination.set_stop_reason(
                TerminationReason.SUCCESS, detail="local contact trajectory loop closed",
                policy=self, source="policy._confirmed")
            self.state, self.reason = State.LOOP_COMPLETE, "local contact trajectory loop closed"
            return
        self.reason = "BOUNDARY REACQUIRED"
        self._resume_tracking(episode, now=now, pose=pose)

    def _confirmation_fit(self):
        contacts = self._confirmation_contacts
        maximum = min(len(contacts), int(self.config.get("local_fit_window", 3)))
        residual_limit = float(self.config.get("local_fit_max_residual", 0.001))
        for count in range(maximum, 1, -1):
            suffix = contacts[-count:]
            estimate = estimate_boundary(
                [item.contact_pose for item in suffix], self._confirmation_direction, self.follow_hand
            )
            if estimate is not None and estimate.residual <= residual_limit:
                return suffix, estimate
        return [], None

    def _reject_confirmation(self, episode, reason):
        self.reason = f"{reason}: reject candidate after probe={episode.probe_id}; retrace confirmation path"
        print(self.reason)
        self.termination.set_stop_reason(
            TerminationReason.STOP_NO_CONTACT if reason in {"CONFIRMATION_NO_CONTACT", "CONFIRMATION_UNSTABLE_CONTACT"}
            else TerminationReason.STOP_REPEATED_CONTACT if reason == "CONFIRMATION_NO_DISPLACEMENT"
            else TerminationReason.STOP_ANCHOR_ERROR if reason.startswith("CONFIRMATION_ANCHOR_ERROR")
            else TerminationReason.STOP_FIT_FAILURE,
            detail=reason, policy=self, terminal=False, source="policy._reject_confirmation")
        for candidate in [*self._confirmation_contacts, episode]:
            candidate.rejection_reason = reason
        # Retrace the already executed anchor-transfer polyline, not a diagonal.
        path = [(p, "CONFIRMATION_ROLLBACK") for p in reversed(self._confirmation_anchor_path[:-1])]
        self._queue_transfer(path, self._next_recovery_probe)

    def _loop_closed(self):
        if not self.config.get("loop_closure_enabled", False):
            return False
        points = self.boundary_points
        if len(points) < int(self.config.get("loop_closure_min_boundary_points", 24)):
            return False
        path = np.asarray([p.pose[:2] for p in points])
        cumulative = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        minimum_path = float(self.config.get("loop_closure_min_path_length", 0.25))
        maximum_distance = float(self.config.get("loop_closure_distance", 0.012))
        minimum_alignment = np.cos(np.deg2rad(self.config.get("loop_closure_tangent_tolerance_deg", 30)))
        # Confirmation can append a small locally fitted batch. Any new point in
        # that batch may be the closest passage through the start region.
        window = int(self.config.get("local_fit_window", 3))
        for index in range(max(1, len(points) - window), len(points)):
            if (cumulative[index] >= minimum_path
                    and np.linalg.norm(path[index] - path[0]) <= maximum_distance
                    and np.dot(points[index].tangent_xy, points[0].tangent_xy) >= minimum_alignment):
                return True
        return False

    def _waypoint(self, now, pose, event):
        self.policy_waypoints.append(PolicyWaypoint(now, self.state.value, event, pose.copy()))

    def request_stop(self, reason="operator stop", *, termination_reason=None):
        self.termination.set_stop_reason(termination_reason, detail=reason, policy=self, source="policy.request_stop")
        if self.active_episode is not None:
            self.active_episode.abort(reason)
        self.state, self.reason = State.STOP, reason

    def request_normal_stop(self, reason="operator normal stop"):
        self.request_stop(reason)
        self.state = State.STOP_SCAN

    def begin_return_to_start(self):
        if self.state != State.STOP_SCAN:
            raise RuntimeError("safe return requires normal stop")
        self.state = State.RETURN_TO_START

    def complete_return_to_start(self):
        if self.termination.record is None:
            self.termination.set_stop_reason(
                TerminationReason.STOP_UNKNOWN_REASON,
                detail="return to start completed without a recorded scan termination",
                policy=self, source="policy.complete_return_to_start")
        self.state = State.STOP

    @property
    def current_normal(self):
        return self.current_target_direction

    @property
    def recovery_anchor(self):
        return None if self.recovery is None else self.recovery.anchor

    @property
    def recovery_direction(self):
        episode = self.active_episode
        return None if episode is None or episode.recovery_id is None else episode.probe_direction

    @property
    def recovery_attempted_directions(self):
        return [] if self.recovery is None else self.recovery.attempted_directions

    @property
    def current_recovery_ray(self):
        episode = self.active_episode
        if episode is None or episode.recovery_id is None:
            return None
        return {"anchor_pose": episode.anchor_pose.copy(), "direction_xy": episode.probe_direction.copy(),
                "planned_length": episode.max_probe_distance, "theta_deg": episode.angle_deg}
