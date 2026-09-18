#!/usr/bin/env python3
"""Explicitly confirmed, force-monitored return to the saved reset pose."""

from __future__ import annotations

import argparse
from copy import deepcopy
import sys
import time
from pathlib import Path

import numpy as np

from calibration.reset_pose import (
    load_reset_pose,
    resolve_reset_pose_path,
)
from calibration.scan_calibration import load_scan_calibration, resolve_calibration_path
from config.loader import load_config, runtime_robot_config
from experiment_logging.data_logger import ExperimentLogger
from robot.rtde_controller import RobotError, URRTDEController
from robot.tcp_identity import read_active_tcp_offset, tcp_offsets_match
from safety.safe_return import (
    SafeReturnExecutor,
    calculate_safe_return_z,
    validate_return_configuration,
)
from sensor.force_preprocess import WrenchPreprocessor
from sensor.px6d_reader import PX6DReader


def load_return_target(config_path: str, config: dict) -> dict:
    """Prefer an explicit reset pose, otherwise use the saved scan P0."""
    reset_path = resolve_reset_pose_path(config_path, config["reset"]["file"])
    if reset_path.is_file():
        reset = load_reset_pose(reset_path)
        return {
            "pose": list(reset["reset_tcp_pose"]),
            "active_tcp_offset": list(reset["active_tcp_offset"]),
            "robot_ip": reset["robot_ip"],
            "label": "RESET",
            "source": str(reset_path),
            "legacy_tcp_binding": False,
        }

    calibration_path = resolve_calibration_path(
        config_path, config["calibration"]["file"]
    )
    calibration = load_scan_calibration(calibration_path)
    bound_tcp = calibration.get("active_tcp_offset")
    legacy_binding = bound_tcp is None
    if legacy_binding:
        # Historical calibrations predate explicit TCP metadata.  Their P0 was
        # operated under the then-configured TCP; never bind it to whatever
        # unrelated TCP happens to be active now.
        bound_tcp = config["tcp"]["offset"]
    return {
        "pose": list(calibration["start_tcp_pose"]),
        "active_tcp_offset": list(bound_tcp),
        "robot_ip": calibration.get("robot_ip"),
        "label": "P0",
        "source": f"{calibration_path} -> start_tcp_pose",
        "legacy_tcp_binding": legacy_binding,
    }


def read_current_state_and_tcp(robot_ip: str) -> tuple[list[float], list[float], list[float]]:
    try:
        import rtde_control
        import rtde_receive
    except ImportError as exc:
        raise RobotError("ur-rtde is not installed") from exc
    receiver = None
    control = None
    try:
        receiver = rtde_receive.RTDEReceiveInterface(robot_ip)
        control = rtde_control.RTDEControlInterface(robot_ip)
        if hasattr(receiver, "isEmergencyStopped") and receiver.isEmergencyStopped():
            raise RobotError("UR emergency stop is active")
        if hasattr(receiver, "isProtectiveStopped") and receiver.isProtectiveStopped():
            raise RobotError("UR protective stop is active")
        pose = np.asarray(receiver.getActualTCPPose(), dtype=float)
        speed = np.asarray(receiver.getActualTCPSpeed(), dtype=float)
        if pose.shape != (6,) or speed.shape != (6,) or not np.all(np.isfinite(pose)) or not np.all(np.isfinite(speed)):
            raise RobotError("RTDE returned an invalid TCP pose/speed")
        if float(np.linalg.norm(speed[:3])) > 0.0005 or float(np.linalg.norm(speed[3:])) > 0.005:
            raise RobotError("TCP is not stationary; refusing to plan reset")
        active_tcp = read_active_tcp_offset(control)
        return pose.tolist(), speed.tolist(), active_tcp
    finally:
        for interface in (control, receiver):
            if interface is not None and hasattr(interface, "disconnect"):
                try:
                    interface.disconnect()
                except Exception:
                    pass


def capture_temporary_bias(reader, preprocessor, count: int, interval: float) -> None:
    print(f"采集 {count} 个静止临时偏置样本；不会执行 PX6D 硬件清零。")
    samples = []
    for _ in range(count):
        samples.append(reader.read_wrench())
        if interval > 0.0:
            time.sleep(interval)
    preprocessor.set_zero_bias(samples)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(Path(__file__).with_name("config.yaml")))
    args = parser.parse_args()
    config = load_config(args.config)
    return_target = load_return_target(args.config, config)
    robot_ip = config["robot"]["robot_ip"]
    if return_target["robot_ip"] != robot_ip:
        raise RobotError("return target robot_ip does not match config.yaml")
    target = np.asarray(return_target["pose"], dtype=float)
    validate_return_configuration(config, target)

    current, speed, active_tcp = read_current_state_and_tcp(robot_ip)
    tolerance = float(config["tcp"]["offset_tolerance"])
    if not tcp_offsets_match(
        return_target["active_tcp_offset"], active_tcp, tolerance
    ):
        raise RobotError(
            f"current active TCP {active_tcp} does not match the TCP bound to "
            f"{return_target['label']} {return_target['active_tcp_offset']}. "
            "Select the TCP used when this target was saved, then rerun; no motion was sent."
        )
    safe_z = calculate_safe_return_z(
        float(current[2]),
        float(target[2]),
        float(config["safe_return"]["return_lift_distance"]),
        float(config["safe_return"]["return_position_tolerance"]),
    )
    print(f"当前位置: {current}")
    print(f"当前 TCP speed: {speed}")
    print(f"复位目标来源: {return_target['source']}")
    print(f"复位目标 {return_target['label']}: {target.tolist()}")
    print(f"目标绑定的 active TCP: {return_target['active_tcp_offset']}")
    if return_target["legacy_tcp_binding"]:
        print("说明：这是旧版 P0，文件没有 TCP 字段；根据旧运行配置使用 config.yaml 的 TCP 绑定。")
    print(f"本次安全高度 Z: {safe_z:.6f} m")
    print("路径：当前点沿 Base +Z 上升 → 安全高度平移到复位点上方 → 垂直到复位点")
    print("注意：这是受力监控的三段笛卡尔路径，不具备碰撞规划或自动避障能力。")
    configured_tcp = config["tcp"]["offset"]
    if not tcp_offsets_match(active_tcp, configured_tcp, tolerance):
        print("提示：当前 active TCP 与扫描配置不一致；本次只允许返回与它绑定的复位点。")
        print("复位完成后仍不能直接正式扫描，必须先修正 TCP 配置并重新做 P0/P1 标定。")
    confirmation = (
        "RETURN_TO_P0" if return_target["label"] == "P0" else "RETURN_TO_RESET"
    )
    answer = input(f"现场逐段检查完成后，输入 {confirmation} 才会运动：").strip()
    if answer != confirmation:
        print("未确认，不执行复位运动。")
        return 0

    effective_config = deepcopy(config)
    effective_config["tcp"]["offset"] = list(return_target["active_tcp_offset"])
    sensor_config = effective_config["sensor"]
    reader = PX6DReader(
        sensor_config["serial_port"],
        sensor_config["baudrate"],
        sensor_config["timeout_sec"],
        sensor_config["poll_rate_hz"],
        sensor_config["startup_delay_sec"],
    )
    robot_config = runtime_robot_config(effective_config)
    # The reset target is authorized by its own captured TCP identity.  This
    # does not mutate or silently bless a mismatched scan configuration.
    robot_config["tcp_offset"] = list(return_target["active_tcp_offset"])
    controller = URRTDEController(robot_config)
    preprocessor = WrenchPreprocessor.from_config(effective_config["preprocessing"])
    logger = None
    try:
        reader.connect()
        capture_temporary_bias(
            reader,
            preprocessor,
            int(effective_config["safe_return"]["startup_bias_sample_count"]),
            1.0 / float(sensor_config["poll_rate_hz"]),
        )
        controller.connect(allow_start_away_from_fixed_pose=True)
        logger = ExperimentLogger(
            effective_config["logging"]["output_root"], effective_config
        )
        result = SafeReturnExecutor(
            effective_config,
            target,
            controller,
            reader,
            preprocessor,
            logger=logger,
            target_label=return_target["label"],
        ).execute()
        logger.write_summary(
            "STOP",
            "manual reset return",
            0,
            return_status=result.status,
            return_abort_reason=result.abort_reason,
        )
        return 0 if result.status == "complete" else 2
    except KeyboardInterrupt:
        controller.safe_stop_motion()
        print("Ctrl+C：紧急停止，不继续自动复位。", file=sys.stderr)
        return 130
    except Exception as exc:
        controller.safe_stop_motion()
        print(f"SAFE RESET ABORTED: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        controller.close()
        reader.close()
        if logger is not None:
            logger.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except SystemExit:
        raise
    except Exception as exc:
        print(f"拒绝复位：{type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(1)
