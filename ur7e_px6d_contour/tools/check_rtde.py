#!/usr/bin/env python3
"""Read-only RTDE receive check; does not construct a control interface."""

from __future__ import annotations

import argparse
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from config.loader import load_config  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(ROOT / "config.yaml"))
    args = parser.parse_args()
    config = load_config(args.config)
    try:
        import rtde_receive
    except ImportError as exc:
        print("ur-rtde is not installed in this Python environment", file=sys.stderr)
        raise SystemExit(2) from exc
    receiver = rtde_receive.RTDEReceiveInterface(config["robot"]["robot_ip"])
    try:
        print(f"robot_ip={config['robot']['robot_ip']}")
        print(f"actual_tcp_pose={receiver.getActualTCPPose()}")
        print(f"actual_tcp_speed={receiver.getActualTCPSpeed()}")
        if hasattr(receiver, "getTCPOffset"):
            print(f"active_tcp_offset={receiver.getTCPOffset()}")
        print(f"robot_mode={receiver.getRobotMode()}")
        print(f"safety_mode={receiver.getSafetyMode()}")
    finally:
        if hasattr(receiver, "disconnect"):
            receiver.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

