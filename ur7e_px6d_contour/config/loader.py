"""Load project YAML and validate cross-module configuration."""

from __future__ import annotations

from pathlib import Path

import yaml


def load_config(path: str | Path) -> dict:
    source = Path(path).resolve()
    with source.open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    if not isinstance(config, dict):
        raise ValueError("config root must be a mapping")
    for section in (
        "robot", "sensor", "preprocessing", "policy", "workspace", "tcp",
        "logging", "calibration", "reset", "safe_return", "dry_run", "execution",
    ):
        if not isinstance(config.get(section), dict):
            raise ValueError(f"missing config section: {section}")
    if config["robot"]["max_tcp_speed"] <= 0:
        raise ValueError("robot.max_tcp_speed must be positive")
    rates = (config["sensor"]["poll_rate_hz"], config["policy"]["control_rate_hz"])
    if any(float(rate) <= 0 for rate in rates):
        raise ValueError("poll and control rates must be positive")
    if float(config["sensor"]["startup_delay_sec"]) < 0.0:
        raise ValueError("sensor.startup_delay_sec cannot be negative")
    if float(config["calibration"]["min_direction_calibration_distance"]) <= 0.0:
        raise ValueError("calibration.min_direction_calibration_distance must be positive")
    if float(config["calibration"]["max_direction_calibration_z_difference"]) < 0.0:
        raise ValueError(
            "calibration.max_direction_calibration_z_difference cannot be negative"
        )
    if not isinstance(config["workspace"].get("enabled", True), bool):
        raise ValueError("workspace.enabled must be true or false")
    if float(config["safe_return"]["return_lift_distance"]) <= 0.0:
        raise ValueError("safe_return.return_lift_distance must be positive")
    if int(config["safe_return"]["startup_bias_sample_count"]) <= 0:
        raise ValueError("safe_return.startup_bias_sample_count must be positive")
    return config


def runtime_robot_config(config: dict) -> dict:
    result = dict(config["robot"])
    result["workspace_limits"] = dict(config["workspace"]["limits"])
    result["workspace_enabled"] = bool(config["workspace"].get("enabled", True))
    result["tcp_offset"] = list(config["tcp"]["offset"])
    result["tcp_offset_tolerance"] = float(config["tcp"]["offset_tolerance"])
    result["allow_unverified_active_tcp"] = bool(config["tcp"]["allow_unverified_active_tcp"])
    return result
