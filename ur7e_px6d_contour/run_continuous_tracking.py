#!/usr/bin/env python3
"""Independent experimental runner. Defaults to entirely offline simulation."""
from __future__ import annotations

import argparse
from copy import deepcopy
from pathlib import Path
import time

import numpy as np
import yaml

from app.operator_input import OperatorKeyboard
from calibration.scan_calibration import (load_scan_calibration, resolve_calibration_path,
                                          validate_calibration_constraints)
from config.loader import load_config, runtime_robot_config
from experiment_logging.data_logger import ExperimentLogger
from experiment_logging.termination import TerminationReason, TerminationRecorder
from policy.continuous_tracking import ContinuousTrackingPolicy, EXTRA_SAMPLE_FIELDS, State
from robot.rtde_controller import URRTDEController, RobotError, validate_execution_configuration
from robot.tcp_identity import tcp_offsets_match
from safety.force_guard import force_safety_reason
from sensor.force_preprocess import WrenchPreprocessor
from sensor.px6d_reader import PX6DReader

ROOT = Path(__file__).resolve().parent


def check_start(controller, start, config):
    robot = controller.read_state()
    if np.linalg.norm(robot.pose[:3] - start[:3]) > float(config["policy"]["position_tolerance"]):
        raise RobotError("start TCP is not at calibrated P0; position it before this experiment")
    if np.linalg.norm(robot.tcp_speed) > 1e-4:
        raise RobotError("robot must be stationary at P0 before starting")
    return robot


def stop_after_exception(controller, termination):
    """A failed stop attempt must not skip disconnect/diagnostic cleanup."""
    try:
        controller.stop()
    except Exception as exc:
        termination.set_stop_reason(detail=str(exc), exception=exc,
                                    source="continuous.stop_failure", terminal=False)


def prepare_real(config, config_path):
    """Use the same calibration/TCP binding as the discrete entry point."""
    validate_execution_configuration(config)
    calibration = load_scan_calibration(resolve_calibration_path(config_path, config["calibration"]["file"]),
                                        require_tcp_offset=True)
    if calibration["robot_ip"] != config["robot"]["robot_ip"]:
        raise RobotError("scan calibration robot_ip does not match config")
    if not tcp_offsets_match(calibration["active_tcp_offset"], config["tcp"]["offset"],
                             float(config["tcp"]["offset_tolerance"])):
        raise RobotError("scan calibration TCP does not match configured TCP")
    validate_calibration_constraints(calibration, float(config["calibration"]["min_direction_calibration_distance"]),
                                     float(config["calibration"]["max_direction_calibration_z_difference"]))
    config["policy"]["search_direction_xy"] = calibration["scan_direction_xy"]
    robot_config = runtime_robot_config(config)
    robot_config.update(fixed_z=calibration["fixed_z"], fixed_orientation=calibration["fixed_orientation"])
    config["continuous_calibration"] = calibration
    # Save calibrated container for offline visualization only, not motion authorization.
    path = ROOT / "workspace/config/workspace_calibration.yaml"
    if path.is_file():
        with path.open(encoding="utf-8") as handle:
            config["continuous_workspace_calibration"] = yaml.safe_load(handle)
    return robot_config, np.asarray(calibration["start_tcp_pose"])


def run(args):
    controller = reader = logger = policy = None
    robot = raw = processed = None
    termination = TerminationRecorder(output_root=ROOT / "data")
    result = 0
    now = 0.0
    config = None
    try:
        config = deepcopy(load_config(args.config))
        if args.duration is not None:
            if not np.isfinite(args.duration) or args.duration <= 0:
                raise ValueError("--duration must be finite and positive")
            # CLI may shorten the reviewed configuration budget, never raise it.
            config["continuous_tracking"]["max_runtime_sec"] = min(
                args.duration, float(config["continuous_tracking"]["max_runtime_sec"]))
        dt = 1 / float(config["policy"]["control_rate_hz"])
        if args.execute:
            robot_config, start = prepare_real(config, args.config)
            policy = ContinuousTrackingPolicy(config)  # validate before any connection
            controller = URRTDEController(robot_config)
            s = config["sensor"]
            reader = PX6DReader(s["serial_port"], s["baudrate"], s["timeout_sec"],
                                s["poll_rate_hz"], s["startup_delay_sec"])
            preprocessor = WrenchPreprocessor.from_config(config["preprocessing"])
        else:
            from simulation.simulator import load_simulation_config
            from simulation.geometry import create_target
            from simulation.simulated_robot import SimulatedRobot
            from simulation.simulated_force_sensor import SimulatedForceSensor
            scene = load_simulation_config(args.scene)
            config["continuous_simulation"] = scene
            config["policy"]["search_direction_xy"] = scene["scan_direction_xy"]
            # Synthetic wrench already uses Base coordinates; do not apply the
            # site's PX6D extrinsics or gravity to synthetic Base-frame values.
            alpha = config["preprocessing"]["filter_alpha"]
            config["preprocessing"] = deepcopy(scene["preprocessing"])
            config["preprocessing"]["filter_alpha"] = alpha
            policy = ContinuousTrackingPolicy(config)
            controller = SimulatedRobot(scene["start_point"], dt, scene["container"])
            target = create_target(scene)
            reader = SimulatedForceSensor(target, scene["force_model"])
            preprocessor = WrenchPreprocessor.from_config(config["preprocessing"])
            if target.signed_distance_and_outward_normal(controller.pose[:2])[0] <= float(scene["force_model"].get("probe_tip_radius", 0)):
                raise ValueError("simulation must start outside the target sensing envelope")
        output = args.output or resolve_calibration_path(args.config, config["logging"]["output_root"])
        # Existing optional workspace exporter renders on close and assumes
        # real Base coordinates; use only our explicit offline replay here.
        logger = ExperimentLogger(output, config, extra_sample_fields=EXTRA_SAMPLE_FIELDS,
                                  workspace_logging=False)
        logger.termination = termination.bind(logger.run_dir)
        termination.observe(policy=policy, processed_force_frame="Base")
        print(f"Continuous run: {logger.run_dir}", flush=True)
        if args.execute:
            reader.connect()
            controller.connect()  # fixed Z/orientation, workspace, active TCP checks
            controller.safe_stop_motion(force=True)
            robot = check_start(controller, start, config)
        else:
            robot = controller.read_state()
        with OperatorKeyboard() as keyboard:
            baseline = config["preprocessing"].get("baseline", {"capture_on_start": True, "sample_count": 100})
            if baseline.get("capture_on_start", True):
                samples = []
                for _ in range(int(baseline["sample_count"])):
                    if keyboard.poll() in ("Q", "ESC"):
                        raise KeyboardInterrupt
                    raw = reader.read_wrench() if args.execute else reader.read_wrench(robot.pose[:2], np.zeros(2))[0]
                    termination.observe(raw=raw, phase="SENSOR_BIAS")
                    if not np.all(np.isfinite(raw.array())):
                        raise ValueError("nonfinite bias sample")
                    processed = preprocessor.process(raw)
                    reason = force_safety_reason(raw, processed, config["policy"])
                    if reason:
                        raise RobotError(reason)
                    if args.execute:
                        current = controller.read_state()
                        if np.linalg.norm(current.pose[:3] - robot.pose[:3]) > float(config["policy"]["position_tolerance"]):
                            raise RobotError("robot moved during bias capture")
                        time.sleep(1 / float(config["sensor"]["poll_rate_hz"]))
                    samples.append(raw)
                preprocessor.set_zero_bias(samples)
        if args.execute:
            print(f"P0: {start.tolist()}\nSearch P0 -> P1: {policy.search_direction.tolist()}")
            print(f"MUST CONFIRM ON SITE: force sign={policy.c['force_direction_sign']}, "
                  f"F_ref={policy.c['force_reference']} N, workspace enabled={config['workspace'].get('enabled', True)}")
            print("Q / ESC / Ctrl+C: stop in place. No automatic return for this experiment.")
            if input("已核对空载零偏、Base 力方向、P0、现场及急停，输入 START：").strip() != "START":
                policy.request_stop(now, robot.pose, "START not confirmed", event="USER_STOP",
                                    code=TerminationReason.STOP_USER_REQUEST)
            else:
                # The operator may have moved the robot while at the prompt.
                robot = check_start(controller, start, config)
        with OperatorKeyboard() as keyboard:
            while policy.state != State.STOP:
                cycle_start = time.monotonic()
                now = cycle_start if args.execute else controller.time
                key = keyboard.poll()
                if key in ("Q", "ESC"):
                    policy.request_stop(now, robot.pose, f"{key} operator stop", event="USER_STOP",
                                        code=TerminationReason.STOP_USER_REQUEST)
                    break
                termination.observe(phase="SENSOR_READ")
                robot = controller.read_state()
                raw = reader.read_wrench() if args.execute else reader.read_wrench(robot.pose[:2], robot.tcp_speed[:2])[0]
                termination.observe(raw=raw, robot=robot, phase="WRENCH_PROCESSING")
                processed = preprocessor.process(raw)
                # Time force samples after read, not before blocking device I/O.
                now = time.monotonic() if args.execute else controller.time
                termination.observe(processed=processed, timestamp=now, phase="POLICY_UPDATE")
                command = policy.update(now, raw, processed, robot)
                if args.execute:
                    if command.move:
                        controller.command_planar_velocity(command.direction_xy, command.speed, dt)
                    else:
                        controller.stop()
                else:
                    predicted = robot.pose[:2] + command.direction_xy * command.speed * dt
                    bounds = controller.workspace
                    if not (bounds["x_min"] <= predicted[0] <= bounds["x_max"] and
                            bounds["y_min"] <= predicted[1] <= bounds["y_max"]):
                        policy.request_stop(now, robot.pose, "simulated TCP left container/workspace")
                        command = policy._command(robot.pose)
                    controller.apply_command(command.direction_xy, command.speed, command.move)
                # Motion/stop precedes disk writes. No rendering in this loop.
                logger.log_sample(now, raw, processed, robot, command, policy.contact_direction,
                                  policy.tangent, extra=policy.telemetry(command))
                for event in policy.events:
                    logger.log_waypoint(event)
                policy.events.clear()
                if args.execute:
                    time.sleep(max(0, dt - (time.monotonic() - cycle_start)))
    except KeyboardInterrupt:
        if controller is not None and args.execute:
            stop_after_exception(controller, termination)
        termination.set_stop_reason(TerminationReason.STOP_USER_REQUEST, "Ctrl+C", source="continuous.runner")
        if policy is not None:
            policy.request_stop(now, np.zeros(6) if robot is None else robot.pose,
                                "Ctrl+C", event="USER_STOP", code=TerminationReason.STOP_USER_REQUEST)
    except Exception as exc:
        # Stop before any error logging or cleanup, including logger failures.
        if controller is not None and args.execute:
            stop_after_exception(controller, termination)
        termination.set_stop_reason(detail=str(exc), exception=exc, source="continuous.runner")
        if policy is not None:
            policy.request_stop(now, np.zeros(6) if robot is None else robot.pose, str(exc))
        print(f"Continuous tracking stopped: {type(exc).__name__}: {exc}", flush=True)
        result = 1
    finally:
        # Record the primary cause in memory before cleanup can add failures.
        if policy is not None and policy.state == State.STOP:
            termination.set_stop_reason(policy.stop_reason, policy.reason, source="continuous.policy")
            if policy.stop_reason not in (TerminationReason.STOP_USER_REQUEST, TerminationReason.STOP_TIME_LIMIT):
                result = 1
        # Independent cleanup steps: a close/log failure cannot skip robot stop.
        for resource in (controller, reader):
            if resource is not None and hasattr(resource, "close"):
                try:
                    resource.close()
                except Exception as exc:
                    result = 1
                    termination.set_stop_reason(detail=str(exc), exception=exc, source="continuous.cleanup")
        if logger is not None:
            try:
                if policy is not None:
                    for event in policy.events:
                        logger.log_waypoint(event)
                    if robot is not None:
                        logger.write_stop_snapshot(robot, raw, processed, policy.state.value)
                    logger.write_summary(policy.state.value, policy.reason, int(policy.initial_contact is not None),
                                         initial_contact=None if policy.initial_contact is None else policy.initial_contact.tolist(),
                                         last_contact_pose=None if policy.last_contact_pose is None else policy.last_contact_pose.tolist(),
                                         mode="real" if args.execute else "simulation")
            except Exception as exc:
                result = 1
                termination.set_stop_reason(detail=str(exc), exception=exc, source="continuous.final_logging")
            finally:
                try:
                    logger.close()
                except Exception as exc:
                    result = 1
                    termination.set_stop_reason(detail=str(exc), exception=exc, source="continuous.logger_close")
        termination.flush(emit=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config.yaml")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true", help="real UR7e + PX6D; requires P0 and START")
    mode.add_argument("--dry-run", action="store_true", help="offline simulation (default)")
    parser.add_argument("--scene", type=Path, default=ROOT / "simulation/scene_continuous.yaml")
    parser.add_argument("--duration", type=float, help="shorten maximum duration in seconds")
    parser.add_argument("--output", type=Path)
    return run(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
