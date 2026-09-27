"""Short-horizon contact geometry and next-anchor planning."""
import numpy as np
from policy.boundary_estimation import estimate_boundary, unit


class InsufficientClearanceError(ValueError):
    """The known free path cannot supply the requested normal clearance."""

    def __init__(self, available_clearance, required_clearance, *, source="ray"):
        self.available_clearance = float(available_clearance)
        self.required_clearance = float(required_clearance)
        self.clearance_source = source
        super().__init__(
            "insufficient tracking normal clearance: "
            f"available={self.available_clearance * 1000:.6f} mm, "
            f"required={self.required_clearance * 1000:.6f} mm, source={source}"
        )


class LocalBoundaryTracker:
    def __init__(self, config, hand):
        self.config, self.hand = config, hand
        self.contacts = []
        self.estimate = None

    def update(self, poses, successful_direction, reset=False):
        candidates = ([] if reset else self.contacts) + [p.copy() for p in poses]
        recent = candidates[-int(self.config.get("local_fit_window", 3)):]
        estimate = estimate_boundary(recent, successful_direction, self.hand,
                                     float(self.config.get("minimum_boundary_point_spacing", 0.001)))
        if estimate is None or estimate.residual > float(self.config.get("local_fit_max_residual", 0.001)):
            return None
        self.contacts, self.estimate = recent, estimate
        return estimate

    def next_anchor_path(self, episode, *, tangent_step=None, reference_contact=None,
                         verified_clearance_pose=None):
        """Return ray-clearance and tangent-transfer waypoints, after ray return.

        ``tangent_step`` is progress relative to the contact, not the length of
        the tangent transfer. An optional previously accepted contact also acts
        as a progress reference when correcting a duplicate/backward sample.

        A short observed probe ray raises ``InsufficientClearanceError``. The
        caller may supply a point on the already executed transfer that led to
        this episode's anchor, and must reach it by reversing that known path.
        Its contact-relative offset is geometry only, not a commanded retreat
        from contact or an extension of the observed probe ray.

        When the observed ray allows, reserve the configured position tolerance
        and fit residual beyond the minimum clearance. These are planning
        margins, not guaranteed bounds on actual motion or boundary error.
        """
        tangent = unit(self.estimate.tangent)
        normal = unit(self.estimate.target_side)
        step = float(self.config["tangent_step"] if tangent_step is None else tangent_step)
        if not np.isfinite(step) or step <= 0:
            raise ValueError("tracking tangent step must be finite and positive")
        contact = episode.contact_pose[:2]
        # Use the actually observed anchor/contact segment, so the clearance
        # point cannot extend behind the known free anchor on a shallow ray.
        free_ray = contact - episode.anchor_pose[:2]
        available_retreat = float(np.linalg.norm(free_ray))
        if not np.isfinite(available_retreat) or available_retreat <= 1e-10:
            raise ValueError("tracking contact has no verified retreat ray")
        ray_direction = free_ray / available_retreat
        inward_component = float(np.dot(ray_direction, normal))
        if inward_component <= 1e-8:
            raise ValueError("tracking probe ray has no inward normal component")
        clearance = float(self.config["retract_distance"])
        if not np.isfinite(clearance) or clearance <= 0:
            raise ValueError("tracking retreat distance must be finite and positive")
        margin_terms = np.asarray([
            self.config.get("position_tolerance", 0.0),
            self.config.get("local_fit_max_residual", 0.0),
        ], dtype=float)
        if not np.all(np.isfinite(margin_terms)) or np.any(margin_terms < 0):
            raise ValueError("tracking clearance margins must be finite and nonnegative")
        margin = float(margin_terms.sum())
        planned_clearance = clearance + margin
        # Preserve normal clearance even after oblique recovery probes. When
        # the finite verified ray is shorter, cap at its original anchor. The
        # extra planning margin never lowers the minimum accepted clearance.
        retreat_length = min(planned_clearance / inward_component, available_retreat)
        retreat = -ray_direction * retreat_length
        clearance_pose = episode.contact_pose.copy()
        clearance_pose[:2] += retreat
        clearance_source = "ray"
        if verified_clearance_pose is not None:
            verified_pose = np.asarray(verified_clearance_pose, dtype=float)
            if verified_pose.shape != clearance_pose.shape or not np.all(np.isfinite(verified_pose)):
                raise ValueError("verified tracking clearance pose must match the finite contact pose")
            clearance_pose = verified_pose.copy()
            retreat = clearance_pose[:2] - contact
            clearance_source = "verified_transfer"
        reference = contact if reference_contact is None else np.asarray(reference_contact)[:2]
        required_progress = max(step, step + float(np.dot(reference - contact, tangent)))
        # Compensate *all* tangential displacement introduced by the retreat.
        # Merely adding `step * tangent` to the retreat point can move backwards.
        tangent_transfer = required_progress - float(np.dot(retreat, tangent))
        next_anchor = clearance_pose.copy()
        next_anchor[:2] += tangent * tangent_transfer
        actual_progress = float(np.dot(next_anchor[:2] - contact, tangent))
        normal_clearance = -float(np.dot(retreat, normal))
        clearance_limited = normal_clearance < clearance - 1e-9
        print(
            "TRACKING_ANCHOR "
            f"current_contact_point_mm={np.round(contact * 1000, 4).tolist()} "
            f"old_tangent={np.round(tangent, 6).tolist()} "
            f"probe_direction={np.round(episode.probe_direction, 6).tolist()} "
            f"retreat_displacement_mm={np.round(retreat * 1000, 4).tolist()} "
            f"actual_tangent_progress_mm={actual_progress * 1000:.4f} "
            f"normal_clearance_mm={normal_clearance * 1000:.4f} "
            f"planned_normal_clearance_mm={planned_clearance * 1000:.4f} "
            f"clearance_margin_mm={margin * 1000:.4f} "
            f"clearance_limited={clearance_limited} "
            f"clearance_source={clearance_source} "
            f"retreat_displacement_kind={'geometry_offset' if verified_clearance_pose is not None else 'probe_ray'}"
        )
        if clearance_limited:
            raise InsufficientClearanceError(normal_clearance, clearance, source=clearance_source)
        return clearance_pose, next_anchor
