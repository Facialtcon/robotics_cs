#!/usr/bin/env python3
"""Read the UR controller's active TCP offset without sending motion commands."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config.loader import load_config
from robot.tcp_identity import normalize_tcp_offset, tcp_offsets_match


def validate_tcp_offset(values) -> list[float]:
    return normalize_tcp_offset(values, "active TCP offset")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(ROOT / "config.yaml"))
    args = parser.parse_args()
    config = load_config(args.config)
    try:
        import rtde_control
    except ImportError as exc:
        print("ur-rtde is not installed in this Python environment", file=sys.stderr)
        raise SystemExit(2) from exc

    print("Connecting an RTDE Control interface only to read getTCPOffset().")
    print("No speedL, moveL, moveJ, servo, forceMode, or setTcp command will be sent.")
    control = rtde_control.RTDEControlInterface(config["robot"]["robot_ip"])
    try:
        offset = validate_tcp_offset(control.getTCPOffset())
    finally:
        if hasattr(control, "disconnect"):
            control.disconnect()

    configured = validate_tcp_offset(config["tcp"]["offset"])
    tolerance = float(config["tcp"]["offset_tolerance"])
    print(f"robot_ip={config['robot']['robot_ip']}")
    print(f"active_tcp_offset={offset}")
    print("Copy this into config.yaml:")
    print(f"  offset: {offset}")
    if tcp_offsets_match(offset, configured, tolerance):
        print(f"config_match=true (tolerance={tolerance} m/rad)")
        return 0
    print(f"config_match=false (configured={configured}, tolerance={tolerance} m/rad)")
    print("Read succeeded; update tcp.offset before execute mode.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except SystemExit:
        raise
    except Exception as exc:
        print(f"active TCP read failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(1)
