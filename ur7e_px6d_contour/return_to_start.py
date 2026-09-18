#!/usr/bin/env python3
"""Explicitly confirmed manual safe return to calibrated scan P0."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

from calibration.scan_calibration import load_scan_calibration, resolve_calibration_path
from config.loader import load_config, runtime_robot_config
from experiment_logging.data_logger import ExperimentLogger
from robot.rtde_controller import RobotError, URRTDEController
from robot.tcp_identity import tcp_offsets_match
from safety.safe_return import (
    SafeReturnExecutor,
    calculate_safe_return_z,
    validate_return_configuration,
)
from sensor.force_preprocess import WrenchPreprocessor
from sensor.px6d_reader import PX6DReader


def read_current_pose(robot_ip: str) -> list[float]:
    try:
        import rtde_receive
    except ImportError as exc:
        raise RobotError("ur-rtde is not installed") from exc
    receiver = rtde_receive.RTDEReceiveInterface(robot_ip)
    try:
        if hasattr(receiver, "isEmergencyStopped") and receiver.isEmergencyStopped():
            raise RobotError("UR emergency stop is active")
        if hasattr(receiver, "isProtectiveStopped") and receiver.isProtectiveStopped():
            raise RobotError("UR protective stop is active")
        return [float(value) for value in receiver.getActualTCPPose()]
    finally:
        if hasattr(receiver, "disconnect"):
            receiver.disconnect()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(Path(__file__).with_name("config.yaml")))
    args = parser.parse_args()
    config = load_config(args.config)
    calibration_path = resolve_calibration_path(args.config, config["calibration"]["file"])
    calibration = load_scan_calibration(calibration_path, require_tcp_offset=True)
    p0 = np.asarray(calibration["start_tcp_pose"], dtype=float)
    if calibration.get("robot_ip") != config["robot"]["robot_ip"]:
        raise RobotError("scan calibration robot_ip does not match config.yaml")
    if not tcp_offsets_match(
        calibration["active_tcp_offset"],
        config["tcp"]["offset"],
        float(config["tcp"]["offset_tolerance"]),
    ):
        raise RobotError("scan calibration TCP does not match config.yaml; recalibrate P0/P1")
    validate_return_configuration(config, p0)

    current = read_current_pose(config["robot"]["robot_ip"])
    lift_distance = float(config["safe_return"]["return_lift_distance"])
    computed_safe_z = calculate_safe_return_z(
        float(current[2]),
        float(p0[2]),
        lift_distance,
        float(config["safe_return"]["return_position_tolerance"]),
    )
    print(f"当前位置: {current}")
    print(f"标定 P0: {p0.tolist()}")
    print(f"向 UR Base +Z 抬升: {lift_distance:.6f} m")
    print(f"本次计算安全高度 Z: {computed_safe_z:.6f} m")
    print("路径：当前点垂直上升 → 安全高度平移到 P0 上方 → 垂直下降到 P0")
    answer = input("现场检查完成后，输入 RETURN_TO_P0 才会运动：").strip()
    if answer != "RETURN_TO_P0":
        print("未确认，不连接控制接口，不执行运动。")
        return 0

    sensor_config = config["sensor"]
    reader = PX6DReader(
        sensor_config["serial_port"],
        sensor_config["baudrate"],
        sensor_config["timeout_sec"],
        sensor_config["poll_rate_hz"],
        sensor_config["startup_delay_sec"],
    )
    controller = URRTDEController(runtime_robot_config(config))
    preprocessor = WrenchPreprocessor.from_config(config["preprocessing"])
    logger = None
    try:
        reader.connect()
        controller.connect(allow_start_away_from_fixed_pose=True)
        logger = ExperimentLogger(config["logging"]["output_root"], config)
        executor = SafeReturnExecutor(
            config, p0, controller, reader, preprocessor, logger=logger
        )
        result = executor.execute()
        logger.write_summary(
            "STOP", "manual return", 0,
            return_status=result.status,
            return_abort_reason=result.abort_reason,
        )
        return 0 if result.status == "complete" else 2
    except KeyboardInterrupt:
        controller.safe_stop_motion()
        print("Ctrl+C：紧急停止，不继续自动返回。", file=sys.stderr)
        return 130
    except Exception as exc:
        controller.safe_stop_motion()
        print(f"SAFE RETURN ABORTED: {type(exc).__name__}: {exc}", file=sys.stderr)
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
        print(f"拒绝返回：{type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(1)
