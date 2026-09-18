#!/usr/bin/env python3
"""Validate and visualize scan_calibration.yaml without connecting to hardware."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config.loader import load_config
from calibration.scan_calibration import (
    CalibrationError,
    load_scan_calibration,
    resolve_calibration_path,
    validate_calibration_constraints,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(ROOT / "config.yaml"))
    parser.add_argument("--output", default=str(ROOT / "scan_calibration_preview.png"))
    args = parser.parse_args()

    config = load_config(args.config)
    calibration_path = resolve_calibration_path(
        args.config, config["calibration"]["file"]
    )
    calibration = load_scan_calibration(calibration_path, require_tcp_offset=True)
    validate_calibration_constraints(
        calibration,
        float(config["calibration"]["min_direction_calibration_distance"]),
        float(config["calibration"]["max_direction_calibration_z_difference"]),
    )

    point_0 = np.asarray(calibration["start_tcp_pose"], dtype=float)
    point_1 = np.asarray(calibration["direction_reference_tcp_pose"], dtype=float)
    direction = np.asarray(calibration["scan_direction_xy"], dtype=float)
    print(f"P0: {point_0.tolist()}")
    print(f"P1: {point_1.tolist()}")
    print(f"scan_direction_xy: {direction.tolist()}")
    print(f"active_tcp_offset: {calibration['active_tcp_offset']}")
    print("Initial search direction is defined by P0 -> P1.")

    delta = point_1[:2] - point_0[:2]
    margin = max(float(np.linalg.norm(delta)) * 0.35, 0.01)
    figure, axis = plt.subplots(figsize=(7, 6))
    axis.scatter(point_0[0], point_0[1], color="tab:blue", s=70, label="P0 (start)")
    axis.scatter(point_1[0], point_1[1], color="tab:orange", s=70, label="P1 (direction reference)")
    axis.annotate(
        "",
        xy=point_1[:2],
        xytext=point_0[:2],
        arrowprops={"arrowstyle": "->", "color": "tab:red", "lw": 2.5},
    )
    midpoint = (point_0[:2] + point_1[:2]) / 2.0
    axis.text(
        midpoint[0],
        midpoint[1] + margin * 0.25,
        f"Initial Scan Direction\n[{direction[0]:+.4f}, {direction[1]:+.4f}]",
        ha="center",
        color="tab:red",
    )
    axis.set_xlim(min(point_0[0], point_1[0]) - margin, max(point_0[0], point_1[0]) + margin)
    axis.set_ylim(min(point_0[1], point_1[1]) - margin, max(point_0[1], point_1[1]) + margin)
    axis.set_xlabel("TCP X (m)")
    axis.set_ylabel("TCP Y (m)")
    axis.set_title("P0 -> P1 Initial Scan Direction")
    axis.grid(True, alpha=0.3)
    axis.axis("equal")
    axis.legend(loc="best")
    figure.tight_layout()
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=160)
    plt.close(figure)
    print(f"Preview saved: {output}")
    print("This tool did not connect to RTDE or command robot motion.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (CalibrationError, OSError, ValueError) as exc:
        print(f"Scan calibration check failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
