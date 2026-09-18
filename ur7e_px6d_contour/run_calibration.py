#!/usr/bin/env python3
"""Teach TCP-bound P0/P1; reads RTDE state/TCP and never commands robot motion."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

from calibration.scan_calibration import (
    CalibrationError,
    build_scan_calibration,
    pose_to_mapping,
    resolve_calibration_path,
    validate_direction_calibration,
    write_scan_calibration,
)
from config.loader import load_config
from robot.tcp_identity import read_active_tcp_offset, tcp_offsets_match


STATIONARY_SPEED_LIMIT = 0.0005


def _check_robot_safety(receiver) -> None:
    if hasattr(receiver, "isEmergencyStopped") and receiver.isEmergencyStopped():
        raise CalibrationError("拒绝标定：UR emergency stop is active。")
    if hasattr(receiver, "isProtectiveStopped") and receiver.isProtectiveStopped():
        raise CalibrationError("拒绝标定：UR protective stop is active。")


def _read_stationary_pose(receiver, name: str) -> list[float]:
    _check_robot_safety(receiver)
    pose = np.asarray(receiver.getActualTCPPose(), dtype=float)
    speed = np.asarray(receiver.getActualTCPSpeed(), dtype=float)
    if (
        pose.shape != (6,)
        or speed.shape != (6,)
        or not np.all(np.isfinite(pose))
        or not np.all(np.isfinite(speed))
    ):
        raise CalibrationError("RTDE 返回了无效的 TCP pose/speed。")
    print(f"{name} TCP pose: {pose.tolist()}")
    print(f"当前 TCP speed: {speed.tolist()}")
    if float(np.linalg.norm(speed[:3])) > STATIONARY_SPEED_LIMIT:
        raise CalibrationError("拒绝保存：TCP 当前没有静止。")
    return pose.tolist()


def _write_incomplete_p0(
    destination: Path,
    pose: list[float],
    robot_ip: str,
    active_tcp_offset: list[float],
) -> None:
    """Invalidate any old P1 as soon as a new P0 is accepted."""
    write_scan_calibration(
        destination,
        {
            "confirmed": False,
            "robot_ip": robot_ip,
            "active_tcp_offset": active_tcp_offset,
            "start_tcp_pose": pose_to_mapping(pose),
            "direction_reference_tcp_pose": None,
            "scan_direction_xy": None,
            "fixed_z": float(pose[2]),
            "fixed_orientation": {
                "rx": float(pose[3]),
                "ry": float(pose[4]),
                "rz": float(pose[5]),
            },
            "calibration_timestamp": None,
        },
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(Path(__file__).with_name("config.yaml")))
    args = parser.parse_args()
    config = load_config(args.config)
    calibration_config = config["calibration"]
    destination = resolve_calibration_path(args.config, calibration_config["file"])
    min_distance = float(calibration_config["min_direction_calibration_distance"])
    max_z_difference = float(
        calibration_config["max_direction_calibration_z_difference"]
    )
    try:
        import rtde_receive
    except ImportError:
        print("缺少 ur-rtde，请先安装 requirements.txt。", file=sys.stderr)
        return 2

    try:
        import rtde_control
    except ImportError:
        print("当前 ur-rtde 无法读取 active TCP；拒绝生成未绑定 TCP 的扫描标定。", file=sys.stderr)
        return 2

    receiver = None
    control = None
    try:
        robot_ip = config["robot"]["robot_ip"]
        receiver = rtde_receive.RTDEReceiveInterface(robot_ip)
        control = rtde_control.RTDEControlInterface(robot_ip)
        active_tcp = read_active_tcp_offset(control)
        configured_tcp = config["tcp"]["offset"]
        tolerance = float(config["tcp"]["offset_tolerance"])
        print("本程序只读取 RTDE 状态和 active TCP，不会发送运动或 setTcp 命令。")
        print(f"active TCP offset: {active_tcp}")
        if not tcp_offsets_match(active_tcp, configured_tcp, tolerance):
            raise CalibrationError(
                "拒绝标定：active TCP 与 config.yaml 的 tcp.offset 不一致。"
                f" active={active_tcp}, config={configured_tcp}。"
                "先在示教器确认正确的探针 TCP；若当前 active TCP 正确，再把它写入 config.yaml。"
            )
        print("请使用 UR7e 示教器将 TCP 移动到扫描起点 P0。")
        answer = input("就位并静止后请输入 SAVE_P0：").strip()
        if answer != "SAVE_P0":
            print("未输入 SAVE_P0，不保存。")
            return 0
        point_0 = _read_stationary_pose(receiver, "P0")
        _write_incomplete_p0(destination, point_0, robot_ip, active_tcp)
        print(f"P0 已保存为未完成标定：{destination}")

        print("\n请继续使用示教器，沿你希望机器人正式扫描的方向移动一小段距离，到达 P1。")
        print("P1 只用于定义初始扫描方向，不是第二个扫描点或轮廓采样点。")
        while True:
            answer = input("P1 就位并静止后请输入 SAVE_DIRECTION（输入 CANCEL 取消）：").strip()
            if answer == "CANCEL":
                print("方向标定已取消；当前文件保持未完成状态，正式扫描将拒绝启动。")
                return 0
            if answer != "SAVE_DIRECTION":
                print("请输入 SAVE_DIRECTION，或输入 CANCEL 取消。")
                continue
            try:
                point_1 = _read_stationary_pose(receiver, "P1")
                active_tcp_at_p1 = read_active_tcp_offset(control)
                if not tcp_offsets_match(active_tcp, active_tcp_at_p1, tolerance):
                    raise CalibrationError("P0 与 P1 之间 active TCP 发生变化，请从 P0 重新标定。")
                direction, xy_distance, z_difference = validate_direction_calibration(
                    point_0, point_1, min_distance, max_z_difference
                )
            except CalibrationError as exc:
                print(f"方向标定无效：{exc}", file=sys.stderr)
                print("请重新移动 P1 后再次输入 SAVE_DIRECTION。")
                continue

            payload = build_scan_calibration(
                point_0,
                point_1,
                robot_ip,
                active_tcp_offset=active_tcp,
            )
            write_scan_calibration(destination, payload)
            print(f"P1 已保存：{point_1}")
            print(f"P0→P1 XY 距离：{xy_distance:.6f} m")
            print(f"P0/P1 Z 差值：{z_difference:.6f} m")
            print(f"INITIAL SCAN DIRECTION: {direction.tolist()}")
            print(f"双点扫描标定完成：{destination}")
            print("注意：当前 TCP 仍位于 P1。")
            print("执行正式扫描命令后，程序会先按三段安全路径自动返回 P0，再等待 START。")
            print(f"P0：{point_0}")
            return 0
    except (CalibrationError, OSError, RuntimeError, ValueError) as exc:
        print(f"扫描标定失败：{type(exc).__name__}: {exc}", file=sys.stderr)
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
