#!/usr/bin/env python3
"""Teach four workspace corners using RTDE Receive, or import four saved poses.

This independent tool never commands robot motion or changes scan calibration.
Workspace P0/P1/P2/P3 are corner labels, separate from scan P0/P1.
"""
from __future__ import annotations

import argparse
from app.operator_input import confirm_enter
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import sys
import time

import numpy as np
import yaml

from robot.tcp_identity import normalize_tcp_offset

PROJECT_ROOT = Path(__file__).resolve().parent
POINT_NAMES = ("P0", "P1", "P2", "P3")
POSE_FIELDS = ("x", "y", "z", "rx", "ry", "rz")
STATIONARY_LINEAR_SPEED = 0.0005  # m/s; rejects a moving teaching sample only.
STATIONARY_ANGULAR_SPEED = 0.01  # rad/s; never sends a stop command.


class InvalidTeachingSample(ValueError):
    pass


def _pose(values, label):
    if isinstance(values, dict):
        if not all(name in values for name in POSE_FIELDS):
            raise InvalidTeachingSample(f"{label} 必须包含 x/y/z/rx/ry/rz 六项。")
        values = [values[name] for name in POSE_FIELDS]
    array = np.asarray(values, dtype=float)
    if array.shape != (6,) or not np.all(np.isfinite(array)):
        raise InvalidTeachingSample(f"{label} 必须是六个有限值 [x,y,z,rx,ry,rz]。")
    return array.tolist()


def load_points_file(path):
    """Read JSON/YAML only; accepted data is four full TCP poses, never XY only."""
    source = Path(path).expanduser().resolve()
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    metadata = payload.get("metadata", {}) if isinstance(payload, dict) else {}
    if isinstance(payload, dict) and "raw_points" in payload:
        payload = payload["raw_points"]
    if isinstance(payload, list) and len(payload) == 4:
        payload = dict(zip(POINT_NAMES, payload))
    if not isinstance(payload, dict) or not all(name in payload for name in POINT_NAMES):
        raise ValueError("点文件必须包含 P0/P1/P2/P3，或按此顺序包含四个完整六维 TCP pose。")
    points = {name: _pose(payload[name], name) for name in POINT_NAMES}
    return points, {"input_mode": "offline_import", "import_source": str(source),
                    "import_metadata": metadata, "active_tcp_verified": False,
                    "tcp_identity_source": "not_verified_by_offline_import"}


def _online_metadata(config_path, robot_ip):
    source = Path(config_path).expanduser().resolve()
    if source.is_file():
        config = yaml.safe_load(source.read_text(encoding="utf-8"))
        if not isinstance(config, dict):
            raise ValueError("配置文件必须是 YAML mapping。")
    elif robot_ip:
        config = {}
    else:
        raise FileNotFoundError(f"找不到只读配置 {source}；也可以使用 --robot-ip 指定机器人地址。")
    address = robot_ip or config.get("robot", {}).get("robot_ip")
    if not isinstance(address, str) or not address.strip():
        raise ValueError("需要 --robot-ip 或 config.yaml 的 robot.robot_ip。")
    configured_tcp = config.get("tcp", {}).get("offset")
    if configured_tcp is not None:
        configured_tcp = normalize_tcp_offset(configured_tcp, "configured TCP offset")
    return address, {"input_mode": "rtde_receive_teaching", "robot_ip": address,
                     "config_source": str(source) if source.is_file() else None,
                     "configured_tcp_offset": configured_tcp, "active_tcp_verified": False,
                     "tcp_identity_source": "configuration_only_not_active_readback",
                     "stationary_linear_speed_limit_mps": STATIONARY_LINEAR_SPEED,
                     "stationary_angular_speed_limit_radps": STATIONARY_ANGULAR_SPEED,
                     "samples": {}}


def read_corner(receiver, name, *, clock=time.time):
    """One observed pose and speed; rejected samples cannot enter calibration."""
    if hasattr(receiver, "isConnected") and not receiver.isConnected():
        raise ConnectionError("RTDE Receive 已断开。")
    for method in ("isEmergencyStopped", "isProtectiveStopped"):
        if hasattr(receiver, method) and getattr(receiver, method)():
            raise InvalidTeachingSample(f"拒绝保存 {name}：{method} 当前为 true。")
    started = clock()
    pose = _pose(receiver.getActualTCPPose(), name)
    if not hasattr(receiver, "getActualTCPSpeed"):
        raise RuntimeError("当前 Receive 接口不能读取 TCP speed，无法核实采样时静止。")
    speed = _pose(receiver.getActualTCPSpeed(), name + " TCP speed")
    completed = clock()
    if np.linalg.norm(speed[:3]) > STATIONARY_LINEAR_SPEED or np.linalg.norm(speed[3:]) > STATIONARY_ANGULAR_SPEED:
        raise InvalidTeachingSample(f"拒绝保存 {name}：TCP 仍在运动，speed={speed}；请在示教器停止后重采。")
    metadata = {"tcp_pose": pose, "tcp_speed": speed,
                "read_timestamp_utc": datetime.fromtimestamp(completed, timezone.utc).isoformat(),
                "read_duration_sec": completed - started}
    return pose, metadata


def teach_corners(receiver, metadata, *, input_fn=None, print_fn=print, clock=time.time):
    print_fn("四角顺序：workspace P0 → P1 → P2 → P3，沿箱体四边依次示教。")
    print_fn("workspace X 轴为 P0→P1；Y 轴为 P0→P3。这些角点不是旧扫描标定的 P0/P1。")
    print_fn("请用示教器移动；程序只读取，不会发送运动或停止命令。各点保存完整 TCP 位姿。")
    print_fn("配置 TCP 仅作为声明保存，未在线核验 active TCP；请在示教器保持同一正确探针 TCP。")
    points = {}
    for name in POINT_NAMES:
        while True:
            if not confirm_enter(f'将 TCP 手动移至 workspace {name} 并静止，即将读取角点。', read_line=input_fn):
                return None
            try:
                pose, sample = read_corner(receiver, name, clock=clock)
            except InvalidTeachingSample as exc:
                print_fn(str(exc))
                continue
            points[name] = pose
            metadata["samples"][name] = sample
            print_fn(f"{name} raw TCP={pose}; speed={sample['tcp_speed']}; time={sample['read_timestamp_utc']}")
            break
    return points


def print_calibration(calibration, output, *, print_fn=print):
    print_fn("四角采样与矩形拟合预览（单位 m；TCP 旋转向量单位 rad）：")
    print_fn(json.dumps({name: calibration.get(name) for name in (
        "raw_points", "rectified_points", "length", "width", "average_z", "origin",
        "rotation_workspace_to_base", "rotation_base_to_workspace", "diagnostics",
    )}, ensure_ascii=False, indent=2))
    print_fn("raw_points 保留实测六维值；rectified_points 仅为拟合矩形。原扫描方向和控制坐标不变。")
    print_fn(f"目标文件：{output}")


def parse_args(argv=None):
    from workspace.workspace_transform import DEFAULT_CALIBRATION_PATH

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", help="RTDE Receive 地址；默认只读 config.yaml 中的 robot.robot_ip")
    parser.add_argument("--config", default=str(PROJECT_ROOT / "config.yaml"))
    parser.add_argument("--output", default=str(DEFAULT_CALIBRATION_PATH))
    parser.add_argument("--points-file", help="离线 JSON/YAML 四个六维 TCP pose；不连接机器人")
    return parser.parse_args(argv)


def main(argv=None, *, receiver_factory=None, input_fn=None, print_fn=print, clock=time.time):
    from workspace.workspace_calibrator import fit_workspace, save_calibration

    receiver = None
    try:
        args = parse_args(argv)
        destination = Path(args.output).expanduser().resolve()
        protected = {(PROJECT_ROOT / "config.yaml").resolve(), (PROJECT_ROOT / "scan_calibration.yaml").resolve(),
                     Path(args.config).expanduser().resolve()}
        if destination in protected:
            raise ValueError("workspace 输出必须使用独立文件，不能覆盖配置或已有扫描方向标定。")
        if args.points_file:
            points, metadata = load_points_file(args.points_file)
            print_fn("离线导入：不读取机器人，不验证这些点对应当前实际箱体或 active TCP。")
        else:
            address, metadata = _online_metadata(args.config, args.robot_ip)
            if receiver_factory is None:
                try:
                    from rtde_receive import RTDEReceiveInterface
                except ImportError as exc:
                    raise RuntimeError("当前 Python 环境缺少 ur-rtde Receive；没有连接或保存标定。") from exc
                receiver_factory = RTDEReceiveInterface
            receiver = receiver_factory(address)
            points = teach_corners(receiver, metadata, input_fn=input_fn, print_fn=print_fn, clock=clock)
            if points is None:
                print_fn("已取消，未写入有效 workspace 标定。")
                return 0
        calibration = fit_workspace(points, metadata=metadata)
        print_calibration(calibration, destination, print_fn=print_fn)
        if not confirm_enter('核对四角、轴方向和拟合误差；即将保存 workspace 标定。', read_line=input_fn):
            print_fn('已取消，未写入有效 workspace 标定。')
            return 0
        if destination.exists():
            stamp = datetime.fromtimestamp(clock(), timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
            backup = destination.with_name(destination.stem + "." + stamp + ".backup" + destination.suffix)
            if backup.exists():
                raise FileExistsError(f"备份文件已存在，保留现有标定：{backup}")
            shutil.copy2(destination, backup)
            print_fn(f"已保留原文件备份：{backup}")
        save_calibration(calibration, destination)
        print_fn(f"workspace 标定已保存：{destination}")
        print_fn("此文件仅供独立坐标转换/显示使用；没有修改 scan_calibration.yaml 或 config.yaml。")
        return 0
    except (KeyboardInterrupt, EOFError):
        print_fn("输入中断，未完成的 workspace 标定不保存。")
        return 130
    except Exception as exc:
        print_fn(f"Workspace 标定失败：{type(exc).__name__}: {exc}")
        return 1
    finally:
        if receiver is not None and hasattr(receiver, "disconnect"):
            try:
                receiver.disconnect()
            except Exception as exc:
                print_fn(f"RTDE Receive 断开失败：{type(exc).__name__}: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
