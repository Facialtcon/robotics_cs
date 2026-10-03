"""Derive a downward probe pose without changing the configured TCP offset.

The probe axis is a physical input expressed in active TCP coordinates.  Base
-Z is downward only when the robot base is level.  These geometric calculations
do not establish sensor mounting or force polarity.
"""
from __future__ import annotations

import numpy as np

from robot.rtde_controller import _rotvec_to_matrix


def unit_probe_axis(axis_tcp):
    axis = np.asarray(axis_tcp, dtype=float)
    if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) <= 1e-12:
        raise ValueError('calibration.probe_axis_tcp must be a finite nonzero 3-vector')
    return axis / np.linalg.norm(axis)


def probe_tilt_deg(orientation, axis_tcp=(0, 0, 1)):
    """Angle between the configured physical probe axis and Base -Z."""
    axis_base = _rotvec_to_matrix(orientation) @ unit_probe_axis(axis_tcp)
    return float(np.degrees(np.arccos(np.clip(-axis_base[2], -1., 1.))))


def _matrix_to_rotvec(matrix, reference):
    cosine = float(np.clip((np.trace(matrix) - 1.) / 2., -1., 1.))
    skew = np.array([matrix[2, 1] - matrix[1, 2],
                     matrix[0, 2] - matrix[2, 0], matrix[1, 0] - matrix[0, 1]]) / 2.
    sine = float(np.linalg.norm(skew))
    angle = float(np.arctan2(sine, cosine))
    if angle < 1e-10:
        return skew
    if sine > 1e-8:
        return angle * skew / sine
    # A downward TCP +Z normally gives a pi rotation: the skew formula is
    # singular there.  Its symmetric part still has the rotation axis at +1.
    _, vectors = np.linalg.eigh((matrix + matrix.T) / 2.)
    axis = vectors[:, -1]
    if float(axis @ np.asarray(reference)) < 0:
        axis = -axis
    return angle * axis


def downward_probe_orientation(orientation, axis_tcp=(0, 0, 1)):
    """Apply the shortest Base rotation carrying the probe axis onto Base -Z.

    No additional twist about the probe is applied, preserving its original
    heading as far as the minimum tilt correction permits.
    """
    original = np.asarray(orientation, dtype=float)
    if original.shape != (3,) or not np.isfinite(original).all():
        raise ValueError('probe orientation must contain three finite values')
    rotation = _rotvec_to_matrix(original)
    axis = rotation @ unit_probe_axis(axis_tcp)
    target = np.array([0., 0., -1.])
    cross = np.cross(axis, target)
    sine = float(np.linalg.norm(cross))
    cosine = float(np.clip(axis @ target, -1., 1.))
    if sine <= 1e-12:
        if cosine > 0:
            return original.copy()
        # Exactly upward has no unique minimum swing axis.  Pick a TCP basis
        # direction orthogonal to the probe, deterministically, without twist.
        basis = rotation[:, int(np.argmin(np.abs(unit_probe_axis(axis_tcp))))]
        swing_axis = np.cross(axis, basis)
        swing_axis /= np.linalg.norm(swing_axis)
        swing = 2. * np.outer(swing_axis, swing_axis) - np.eye(3)
    else:
        k = np.array([[0., -cross[2], cross[1]],
                      [cross[2], 0., -cross[0]], [-cross[1], cross[0], 0.]])
        swing = np.eye(3) + k + k @ k * ((1. - cosine) / sine**2)
    return _matrix_to_rotvec(swing @ rotation, original)
