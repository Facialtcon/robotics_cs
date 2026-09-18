#!/usr/bin/env python3
"""Save any stationary current TCP pose as a TCP-bound reset target; no motion."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

from calibration.reset_pose import (
    ResetPoseError,
    build_reset_pose,
    resolve_reset_pose_path,
    write_reset_pose,
)
from config.loader import load_config
from robot.tcp_identity import read_active_tcp_offset, tcp_offsets_match


STATIONARY_LINEAR_SPEED_LIMIT = 0.0005
STATIONARY_ANGULAR_SPEED_LIMIT = 0.005


def read_stationary_state(receiver) -> tuple[list[float], list[float]]:
    if hasattr(receiver, "isEmergencyStopped") and receiver.isEmergencyStopped():
        raise ResetPoseError("拒绝保存：UR emergency stop is active。")
    if hasattr(receiver, "isProtectiveStopped") and receiver.isProtectiveStopped():
        raise ResetPoseError("拒绝保存：UR protective stop is active。")
    pose = np.asarray(receiver.getActualTCPPose(), dtype=float)
    speed = np.asarray(receiver.getActualTCPSpeed(), dtype=float)
    if (
        pose.shape != (6,)
        or speed.shape != (6,)
        or not np.all(np.isfinite(pose))
        or not np.all(np.isfinite(speed))
    ):
        raise ResetPoseError("RTDE 返回了无效的 TCP pose/speed。")
    if float(np.linalg.norm(speed[:3])) > STATIONARY_LINEAR_SPEED_LIMIT:
        raise ResetPoseError("拒绝保存：TCP 线速度未静止。")
    if float(np.linalg.norm(speed[3:])) > STATIONARY_ANGULAR_SPEED_LIMIT:
        raise ResetPoseError("拒绝保存：TCP 角速度未静止。")
    return pose.tolist(), speed.tolist()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(Path(__file__).with_name("config.yaml")))
    args = parser.parse_args()
    config = load_config(args.config)
    destination = resolve_reset_pose_path(args.config, config["reset"]["file"])
    try:
        import rtde_control
        import rtde_receive
    except ImportError:
        print("缺少 ur-rtde，请先安装 requirements.txt。", file=sys.stderr)
        return 2

    receiver = None
    control = None
    try:
        robot_ip = config["robot"]["robot_ip"]
        receiver = rtde_receive.RTDEReceiveInterface(robot_ip)
        control = rtde_control.RTDEControlInterface(robot_ip)
        pose, speed = read_stationary_state(receiver)
        active_tcp = read_active_tcp_offset(control)
        configured_tcp = config["tcp"]["offset"]
        tolerance = float(config["tcp"]["offset_tolerance"])

        print("本程序只读取 TCP pose/speed 和 active TCP，不发送任何运动或 setTcp 命令。")
        print(f"当前位置（将保存为复位点）: {pose}")
        print(f"当前 TCP speed: {speed}")
        print(f"当前 active TCP offset: {active_tcp}")
        if not tcp_offsets_match(active_tcp, configured_tcp, tolerance):
            print("警告：active TCP 与 config.yaml 不一致；正式扫描仍会被拒绝。")
            print("这个复位点会绑定当前 active TCP，因此仍可用于独立复位。")
        print("请确认此位置适合作为复位终点，且从现场可能位置到它的三段路径安全。")
        answer = input("确认后输入 SAVE_RESET_POSE：").strip()
        if answer != "SAVE_RESET_POSE":
            print("未输入 SAVE_RESET_POSE，不保存。")
            return 0

        # Re-read after confirmation so moving the robot during review cannot
        # silently save the stale pose shown before the prompt.
        pose, speed = read_stationary_state(receiver)
        final_active_tcp = read_active_tcp_offset(control)
        if not tcp_offsets_match(active_tcp, final_active_tcp, tolerance):
            raise ResetPoseError("确认期间 active TCP 发生变化，拒绝保存。")
        payload = build_reset_pose(pose, final_active_tcp, robot_ip)
        write_reset_pose(destination, payload)
        print(f"复位点已保存: {destination}")
        print(f"reset_tcp_pose={pose}")
        print(f"bound_active_tcp_offset={final_active_tcp}")
        return 0
    except (ResetPoseError, OSError, RuntimeError, ValueError) as exc:
        print(f"复位点保存失败：{type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        for interface in (control, receiver):
            if interface is not None and hasattr(interface, "disconnect"):
                try:
                    interface.disconnect()
                except Exception:
                    pass


if __name__ == "__main__":
    raise SystemExit(main())
