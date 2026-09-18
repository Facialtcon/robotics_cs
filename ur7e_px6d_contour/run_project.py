#!/usr/bin/env python3
"""中文分阶段入口：只读/模拟与明确确认的真机运动分区。"""

from __future__ import annotations

import argparse
import importlib.util
import os
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent


def clean_child_environment() -> dict[str, str]:
    """Keep this non-ROS project isolated from a sourced ROS shell."""
    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    # ROS installs pytest entry-point plugins globally.  The project tests do
    # not use third-party pytest plugins, so disable automatic discovery.
    environment["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"
    return environment

STEPS = {
    "1": ("检查代码（不连接任何设备）", [sys.executable, "-m", "pytest", "-q"]),
    "2": (
        "正方形整圈验证并保存一张轨迹图（不连接任何设备）",
        [sys.executable, "tools/run_mock_visualized.py"],
    ),
    "3": ("只读取 PX6D（机器人不会动）", [sys.executable, "tools/check_px6d.py", "--samples", "500"]),
    "4": ("只读取 UR7e 状态（机器人不会动）", [sys.executable, "tools/check_rtde.py"]),
    "5": ("真实 PX6D 驱动规则 dry-run（不连接机器人）", [sys.executable, "main.py", "--sensor", "real"]),
    "6": ("可选方/圆/三角并可拖动的实时动画（不连接任何设备）", [sys.executable, "run_simulation.py"]),
    "7": (
        "标定扫描起点 P0 和初始扫描方向（机器人不移动）",
        [sys.executable, "run_calibration.py"],
    ),
    "8": (
        "检查扫描标定结果（不连接任何设备）",
        [sys.executable, "tools/check_scan_calibration.py"],
    ),
    "9": (
        "读取示教器 active TCP offset（不发送运动命令）",
        [sys.executable, "tools/read_active_tcp.py"],
    ),
    "10": (
        "将任意静止位置保存为 TCP 绑定复位点（不发送运动命令）",
        [sys.executable, "save_reset_pose.py"],
    ),
    "11": (
        "从当前位置返回复位目标；未另存时用扫描 P0（真机运动）",
        [sys.executable, "return_to_reset.py"],
    ),
    "12": (
        "执行真正的 UR7e + PX6D 轮廓扫描（真机运动）",
        [sys.executable, "main.py", "--sensor", "real", "--execute"],
    ),
}

MOTION_CONFIRMATIONS = {
    "11": "OPEN_REAL_RESET",
    "12": "OPEN_REAL_SCAN",
}


def print_menu() -> None:
    print(
        """
============================================================
 UR7e + PX6D 安全运行菜单
============================================================
【测试、模拟、只读；不发送机器人运动命令】
 1. 检查代码
 2. 正方形整圈验证并保存轨迹图
 3. 只读取 PX6D
 4. 只读取 UR7e 当前状态
 5. 真实 PX6D + 规则 dry-run
 6. 方/圆/三角可拖动实时动画
 7. 标定扫描起点 P0 和初始扫描方向
 8. 检查扫描标定结果
 9. 读取示教器 active TCP offset
10. 将任意静止位置保存为 TCP 绑定复位点

【真机运动；必须现场监护并再次输入确认词】
11. 从当前位置返回复位目标（未另存时使用扫描 P0）
12. 执行真正的 UR7e + PX6D 轮廓扫描

0. 退出

注意：11、12 会发送真机运动命令；三段返回不具备碰撞规划能力。
      先完成 1～10 的对应检查，再进入真机运动区。
"""
    )


def check_dependency(step: str) -> str | None:
    required = {
        "1": ("pytest", "pytest"),
        "2": ("matplotlib", "matplotlib"),
        "3": ("serial", "pyserial"),
        "4": ("rtde_receive", "ur-rtde"),
        "5": ("serial", "pyserial"),
        "6": ("matplotlib", "matplotlib"),
        "7": ("rtde_receive", "ur-rtde"),
        "8": ("matplotlib", "matplotlib"),
        "9": ("rtde_control", "ur-rtde"),
        "10": ("rtde_control", "ur-rtde"),
        "11": ("rtde_control", "ur-rtde"),
        "12": ("rtde_control", "ur-rtde"),
    }
    if step not in required:
        return None
    module, package = required[step]
    if importlib.util.find_spec(module) is None:
        return f"缺少 {package}。先运行：{sys.executable} -m pip install -r requirements.txt"
    return None


def run_step(step: str) -> int:
    if step not in STEPS:
        print("输入无效，请输入 0～12。")
        return 2
    dependency_error = check_dependency(step)
    if dependency_error:
        print(f"\n不能开始：{dependency_error}\n")
        return 2
    name, command = STEPS[step]
    if step in MOTION_CONFIRMATIONS:
        confirmation = MOTION_CONFIRMATIONS[step]
        print(f"\n警告：{name}")
        print("该步骤可能立即连接控制接口并发送真机运动命令。")
        answer = input(f"若现场、急停和完整路径均已检查，输入 {confirmation} 继续：").strip()
        if answer != confirmation:
            print("未确认，不启动真机程序。")
            return 0
    print(f"\n开始：{name}", flush=True)
    print("执行：" + " ".join(command), flush=True)
    print("按 Ctrl+C 可以停止。\n", flush=True)
    try:
        result = subprocess.run(
            command,
            cwd=ROOT,
            check=False,
            env=clean_child_environment(),
        )
    except KeyboardInterrupt:
        print("\n已收到 Ctrl+C。")
        return 130
    if result.returncode == 0:
        print(f"\n完成：{name}")
    else:
        print(f"\n失败：{name}（退出码 {result.returncode}）")
        print("请保存终端中的完整错误信息；不要跳到下一阶段。")
    return result.returncode


def main() -> int:
    parser = argparse.ArgumentParser(
        description="UR7e + PX6D 中文分阶段入口"
    )
    parser.add_argument("--step", choices=sorted(STEPS), help="直接执行指定步骤")
    args = parser.parse_args()
    if args.step:
        return run_step(args.step)

    while True:
        print_menu()
        try:
            selected = input("请输入数字并按回车：").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n已退出。")
            return 0
        if selected == "0":
            print("已退出。")
            return 0
        run_step(selected)
        input("\n按回车返回菜单……")


if __name__ == "__main__":
    raise SystemExit(main())
