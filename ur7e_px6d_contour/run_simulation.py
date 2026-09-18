#!/usr/bin/env python3
"""Launch the independent 2D simulator; no UR, PX6D, ROS, or RTDE required."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from simulation.simulator import ContourSimulator, load_simulation_config  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scene",
        default=str(ROOT / "simulation" / "scene_square.yaml"),
        help="scene YAML; defaults to the ideal 100 mm square",
    )
    parser.add_argument("--no-gui", action="store_true", help="run without a window and save results")
    parser.add_argument("--max-steps", type=int, help="headless safety step limit")
    parser.add_argument("--save-animation", action="store_true", help="save MP4, falling back to GIF")
    parser.add_argument("--shape", choices=("square", "circle", "triangle"), help="override scene shape")
    parser.add_argument("--center", nargs=2, type=float, metavar=("X", "Y"), help="target center in metres")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.no_gui:
        import matplotlib
        matplotlib.use("Agg")
    config = load_simulation_config(args.scene)
    if args.save_animation:
        config["visualization"]["save_animation"] = True
    simulator = ContourSimulator(config)
    if args.shape or args.center:
        simulator.set_interactive_target(args.shape or simulator.target_shape, args.center)
    print(f"场景：{simulator.config['scene_name']}")
    print(f"initial_scan_direction={simulator.config['scan_direction_xy']}")
    print("此模拟器不连接 UR7e、PX6D、ROS 或 RTDE。")
    if args.no_gui:
        simulator.run_headless(args.max_steps)
        output = simulator.finalize()
        print(f"模拟完成：{output}")
        print(f"状态序列：{list(dict.fromkeys(simulator.states_seen))}")
        print(f"边界点：{len(simulator.policy.boundary_points)}")
        print(f"平均边界距离：{simulator.boundary_error_mean() * 1000.0:.3f} mm")
        return 0 if simulator.loop_completed else 1

    from simulation.top_view import SimulationView

    print("控制：1/2/3 切换方/圆/三角；左拖物体；Shift+左拖同时平移 P0/P1。")
    print("SPACE 暂停/继续，Q 正常停止，ESC 紧急停止并关闭，R 重置。")
    SimulationView(simulator).show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
