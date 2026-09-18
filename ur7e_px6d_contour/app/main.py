#!/usr/bin/env python3
"""System orchestrator: sensor -> processing -> policy -> robot -> logging."""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

from app.operator_input import OperatorKeyboard
from calibration.scan_calibration import (
    load_scan_calibration,
    resolve_calibration_path,
    validate_calibration_constraints,
)
from config.loader import load_config, runtime_robot_config
from core.models import PolicyCommand
from experiment_logging.data_logger import ExperimentLogger
from experiment_logging.probe_log import write_probe_logs
from experiment_logging.termination import TerminationReason, TerminationRecorder
from policy.rule_policy import RuleBasedPolicy, State
from robot.rtde_controller import (
    RobotError,
    SimulatedController,
    URRTDEController,
    _orientation_distance,
    validate_execution_configuration,
)
from robot.tcp_identity import tcp_offsets_match
from safety.safe_return import SafeReturnExecutor, validate_return_configuration
from sensor.force_preprocess import WrenchPreprocessor
from sensor.px6d_reader import MockPX6DReader, PX6DReader


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(PROJECT_ROOT / "config.yaml"))
    parser.add_argument("--sensor", choices=("mock", "real"), default=None)
    parser.add_argument("--execute", action="store_true", help="enable the real RTDE motion path")
    parser.add_argument(
        "--acknowledge-risk",
        default=None,
        help="deprecated compatibility option; no longer required",
    )
    return parser.parse_args()


def capture_bias(reader, preprocessor: WrenchPreprocessor, count: int, interval: float, *, termination=None) -> None:
    print(f"Capturing {count} stationary software-bias samples; hardware zero is NOT invoked.")
    samples = []
    for _ in range(count):
        raw = reader.read_wrench()
        if termination is not None:
            termination.observe(raw=raw)
        samples.append(raw)
        if interval > 0:
            time.sleep(interval)
    preprocessor.set_zero_bias(samples)


def format_dry_run(command, processed, policy) -> str:
    normal = policy.current_target_direction
    tangent = policy.current_tangent
    normal_text = "unset" if normal is None else f"[{normal[0]:+.3f}, {normal[1]:+.3f}]"
    tangent_text = "unset" if tangent is None else f"[{tangent[0]:+.3f}, {tangent[1]:+.3f}]"
    return (
        f"{command.state:16s} Fxy={processed.force[:2].dot(processed.force[:2]) ** 0.5:6.3f} N "
        f"n={normal_text} t={tangent_text} "
        f"cmd=[{command.direction_xy[0]:+.3f},{command.direction_xy[1]:+.3f}] "
        f"v={command.speed:.4f} target_xy=[{command.target_pose[0]:+.4f},{command.target_pose[1]:+.4f}] "
        f"{command.reason}"
    )


def refresh_emergency_pose(controller, previous):
    """Get a fresh pose after stopping, or retain the last readable sample."""
    try:
        return controller.read_diagnostic_state(), True
    except Exception:
        return previous, False


def run(args: argparse.Namespace, *, termination=None) -> int:
    """Keep diagnosis alive even before device/logger construction succeeds."""
    termination = termination or TerminationRecorder(output_root=PROJECT_ROOT / "data")
    termination.observe(state="INITIALIZATION", phase="CONFIGURATION")
    try:
        return _run(args, termination)
    except BaseException as exc:
        termination.set_stop_reason(detail=f"{type(exc).__name__}: {exc}", exception=exc,
                                    source="app.run.unhandled")
        raise
    finally:
        if termination.record is None:
            termination.set_stop_reason(TerminationReason.STOP_UNKNOWN_REASON,
                                        "Scan exited without a recorded termination cause", source="app.run.finally")
        termination.flush(emit=True)


def _run(args: argparse.Namespace, termination: TerminationRecorder) -> int:
    config = load_config(args.config)
    termination.output_root = Path(config["logging"]["output_root"])
    sensor_kind = args.sensor or config["execution"]["default_sensor"]
    dry_run = not args.execute
    if dry_run and sensor_kind == "mock":
        # The default offline experiment now measures a full square loop.
        from simulation.simulator import ContourSimulator, load_simulation_config
        import matplotlib

        matplotlib.use("Agg")
        print("Mock dry-run: ideal 100 mm square, shared contour policy, no hardware connection.")
        simulator = ContourSimulator(load_simulation_config(PROJECT_ROOT / "simulation" / "scene_square.yaml"))
        simulator.termination = termination
        simulator.policy.termination = termination
        termination.observe(policy=simulator.policy, robot=simulator.robot.read_state(),
                            raw=simulator.last_raw, processed=simulator.last_processed,
                            command=simulator.last_command, phase="SIMULATION")
        simulator.run_headless()
        simulator.finalize()
        return 0 if simulator.loop_completed else 1
    calibration = None
    start_pose = None
    if args.execute:
        validate_execution_configuration(config)
        if sensor_kind != "real":
            raise RobotError("real robot execution requires --sensor real")
        calibration_path = resolve_calibration_path(
            args.config, config["calibration"]["file"]
        )
        calibration = load_scan_calibration(
            calibration_path, require_tcp_offset=True
        )
        if calibration.get("robot_ip") != config["robot"]["robot_ip"]:
            raise RobotError("scan calibration robot_ip does not match config.yaml")
        if not tcp_offsets_match(
            calibration["active_tcp_offset"],
            config["tcp"]["offset"],
            float(config["tcp"]["offset_tolerance"]),
        ):
            raise RobotError(
                "scan calibration was captured with a different TCP than config.yaml; "
                "select/verify the probe TCP and run python3 run_calibration.py again"
            )
        validate_calibration_constraints(
            calibration,
            float(config["calibration"]["min_direction_calibration_distance"]),
            float(config["calibration"]["max_direction_calibration_z_difference"]),
        )
        # In execute mode the SEARCH direction has exactly one source: the
        # validated P0 -> P1 result.  The policy later derives its contour
        # following direction from live F/T after CONTACT, as before.
        config["policy"]["search_direction_xy"] = list(
            calibration["scan_direction_xy"]
        )
        start_pose = np.asarray(calibration["start_tcp_pose"], dtype=float)
        validate_return_configuration(config, start_pose)

    sensor_config = config["sensor"]
    if sensor_kind == "real":
        reader = PX6DReader(
            sensor_config["serial_port"],
            sensor_config["baudrate"],
            sensor_config["timeout_sec"],
            sensor_config["poll_rate_hz"],
            sensor_config["startup_delay_sec"],
        )
    else:
        reader = MockPX6DReader(
            sample_rate_hz=float(sensor_config["poll_rate_hz"]),
            contact_force=float(config["dry_run"]["mock_contact_force"]),
        )

    robot_config = runtime_robot_config(config)
    if args.execute:
        assert calibration is not None
        robot_config["fixed_z"] = float(calibration["fixed_z"])
        robot_config["fixed_orientation"] = list(calibration["fixed_orientation"])
    controller = (
        SimulatedController(config["dry_run"]["start_pose"], robot_config["max_tcp_speed"])
        if dry_run
        else URRTDEController(robot_config)
    )
    preprocessor = WrenchPreprocessor.from_config(config["preprocessing"])
    policy_config = dict(config["policy"])
    if dry_run:
        # Keep unattended mock runs finite without imposing those completion
        # limits on formal robot contour following.
        policy_config["max_boundary_points"] = config["dry_run"].get(
            "max_boundary_points", policy_config.get("max_boundary_points")
        )
        policy_config["max_runtime_sec"] = config["dry_run"].get(
            "max_runtime_sec", policy_config.get("max_runtime_sec")
        )
        policy_config["probe_direction_sign"] = config["dry_run"].get(
            "probe_direction_sign", policy_config["probe_direction_sign"]
        )
    policy = RuleBasedPolicy(policy_config)
    policy.termination = termination
    termination.observe(policy=policy)
    logger = None
    probe_logs_exported = False
    final_reason = "not started"
    return_status = "not_requested"
    return_abort_reason = ""
    next_print = 0.0
    logged_boundaries = 0
    logged_waypoints = 0
    logged_recovery_rays = 0
    period = 1.0 / float(config["policy"]["control_rate_hz"])
    last_robot = None
    last_raw = None
    last_processed = None
    last_command = None
    emergency = False
    normal_stop = False
    result_code = 0

    try:
        termination.observe(phase="SENSOR_CONNECT")
        reader.connect()
        print(f"PX6D source connected: {sensor_kind}, firmware={reader.firmware}")
        termination.observe(phase="ROBOT_CONNECT")
        if args.execute:
            controller.connect(allow_start_away_from_fixed_pose=True)
        else:
            controller.connect()
        print("Controller mode: DRY-RUN (no robot connection, no robot commands)" if dry_run else "Controller mode: REAL RTDE")
        logger = ExperimentLogger(config["logging"]["output_root"], config)
        logger.termination = termination
        termination.bind(logger.run_dir)
        print(f"Logging to {logger.run_dir}")
        if args.execute:
            assert start_pose is not None
            termination.observe(phase="STARTUP_POSE")
            current = controller.read_diagnostic_state()
            last_robot = current
            termination.observe(robot=current)
            position_error = float(np.linalg.norm(current.pose[:3] - start_pose[:3]))
            orientation_error = _orientation_distance(current.pose[3:], start_pose[3:])
            position_tolerance = float(config["safe_return"]["return_position_tolerance"])
            orientation_tolerance = float(config["safe_return"]["return_orientation_tolerance"])
            if position_error > position_tolerance or orientation_error > orientation_tolerance:
                print(
                    "Current TCP is not at P0; automatic startup return will run "
                    f"(position_error={position_error:.6f} m, "
                    f"orientation_error={orientation_error:.6f} rad)."
                )
                print(
                    "Capturing a stationary temporary bias for startup-return "
                    "incremental force monitoring."
                )
                startup_return_preprocessor = WrenchPreprocessor.from_config(
                    config["preprocessing"]
                )
                if hasattr(reader, "set_context"):
                    reader.set_context("STARTUP_RETURN_BASELINE")
                termination.observe(phase="SENSOR_BIAS")
                capture_bias(
                    reader,
                    startup_return_preprocessor,
                    int(config["safe_return"]["startup_bias_sample_count"]),
                    1.0 / float(sensor_config["poll_rate_hz"]),
                    termination=termination,
                )
                termination.observe(phase="STARTUP_RETURN")
                startup_return = SafeReturnExecutor(
                    config,
                    start_pose,
                    controller,
                    reader,
                    startup_return_preprocessor,
                    logger=logger,
                    policy=policy,
                ).execute()
                return_status = f"startup_{startup_return.status}"
                return_abort_reason = startup_return.abort_reason
                if startup_return.status != "complete":
                    final_reason = "automatic startup return to P0 aborted"
                    termination.set_stop_reason(detail=return_abort_reason or final_reason,
                                                source="app.startup_return", return_status=return_status,
                                                return_abort_reason=return_abort_reason)
                    policy.request_stop(final_reason)
                    return 2
                current = controller.read_state()
                last_robot = current
                termination.observe(robot=current)
                position_error = float(np.linalg.norm(current.pose[:3] - start_pose[:3]))
                orientation_error = _orientation_distance(current.pose[3:], start_pose[3:])
                if position_error > position_tolerance or orientation_error > orientation_tolerance:
                    raise RobotError(
                        "automatic startup return reported complete but P0 tolerance check failed"
                    )
                print("AUTOMATIC STARTUP RETURN TO P0 COMPLETE")
            else:
                print("Current TCP is already at calibrated P0; startup return skipped.")
        baseline = config["preprocessing"]["baseline"]
        if baseline["capture_on_start"]:
            if hasattr(reader, "set_context"):
                reader.set_context("BASELINE")
            termination.observe(phase="SENSOR_BIAS")
            capture_bias(
                reader,
                preprocessor,
                int(baseline["sample_count"]),
                1.0 / float(sensor_config["poll_rate_hz"]),
                termination=termination,
            )
        if args.execute:
            termination.observe(phase="START_CONFIRMATION")
            assert calibration is not None
            print(f"START TCP:\n{calibration['start_tcp_pose']}")
            print(
                "DIRECTION REFERENCE TCP:\n"
                f"{calibration['direction_reference_tcp_pose']}"
            )
            print(f"INITIAL SCAN DIRECTION:\n{calibration['scan_direction_xy']}")
            print(f"FIXED Z:\n{calibration['fixed_z']}")
            print(f"FIXED ORIENTATION:\n{calibration['fixed_orientation']}")
            print("Initial search direction is defined by P0 -> P1.")
            answer = input(
                "已核对 P0、P1 方向、现场和急停后，输入 START 开始扫描："
            ).strip()
            if answer != "START":
                final_reason = "START not confirmed"
                termination.set_stop_reason(TerminationReason.STOP_USER_REQUEST, final_reason,
                                            source="app.start_confirmation")
                policy.request_stop(final_reason)
                logger.write_summary(State.STOP.value, final_reason, 0)
                print("未输入 START，不发送扫描运动。")
                return 0
            print("扫描中：Q = 正常停止并安全返回 P0；ESC/Ctrl+C = 紧急停止且不返回。")

        with OperatorKeyboard() as keyboard:
            while policy.state not in {State.STOP, State.STOP_SCAN, State.LOOP_COMPLETE}:
                cycle_start = time.monotonic()
                termination.observe(phase="SCAN", policy=policy)
                key = keyboard.poll()
                if key == "ESC":
                    emergency = True
                    final_reason = "ESC emergency stop"
                    termination.set_stop_reason(TerminationReason.STOP_USER_REQUEST, final_reason,
                                                source="app.keyboard.ESC", policy=policy)
                    controller.safe_stop_motion() if args.execute else controller.stop()
                    last_robot, _ = refresh_emergency_pose(controller, last_robot)
                    policy.request_stop(final_reason)
                    if last_robot is not None:
                        logger.write_stop_snapshot(
                            last_robot, last_raw, last_processed, State.STOP.value
                        )
                    break
                if key == "Q":
                    termination.set_stop_reason(TerminationReason.STOP_USER_REQUEST, "Q normal stop",
                                                source="app.keyboard.Q", policy=policy)
                    controller.safe_stop_motion() if args.execute else controller.stop()
                    policy.request_normal_stop()
                    normal_stop = True
                    final_reason = "Q normal stop"
                    # Capture P_stop and wrench after the scan motion has
                    # actually stopped, rather than reusing the prior cycle.
                    termination.observe(phase="SENSOR_READ_AFTER_STOP")
                    last_raw = reader.read_wrench()
                    termination.observe(raw=last_raw)
                    last_processed = preprocessor.process(last_raw)
                    termination.observe(processed=last_processed, phase="ROBOT_READ_AFTER_STOP")
                    last_robot = controller.read_state()
                    termination.observe(robot=last_robot)
                    stop_command = PolicyCommand(
                        state=State.STOP_SCAN.value,
                        move=False,
                        direction_xy=np.zeros(2),
                        speed=0.0,
                        target_pose=last_robot.pose.copy(),
                        contact_flag=False,
                        possible_corner=policy.possible_corner,
                        reason=final_reason,
                    )
                    logger.log_sample(
                        cycle_start, last_raw, last_processed, last_robot, stop_command,
                        policy.current_target_direction, policy.current_tangent,
                    )
                    logger.write_stop_snapshot(
                        last_robot, last_raw, last_processed, State.STOP_SCAN.value
                    )
                    break

                if hasattr(reader, "set_context"):
                    reader.set_context(policy.state.value)
                termination.observe(phase="SENSOR_READ")
                raw = reader.read_wrench()
                last_raw = raw
                termination.observe(raw=raw, phase="WRENCH_PROCESSING")
                processed = preprocessor.process(raw)
                last_processed = processed
                termination.observe(processed=processed, phase="ROBOT_READ")
                robot = controller.read_state()
                last_robot = robot
                termination.observe(robot=robot, phase="POLICY_UPDATE")
                command = policy.update(cycle_start, raw, processed, robot)
                last_command = command
                termination.observe(command=command, phase="MOTION_COMMAND")
                last_raw, last_processed, last_robot = raw, processed, robot
                if command.move:
                    controller.command_planar_velocity(command.direction_xy, command.speed, period)
                else:
                    controller.stop()
                if termination.record is not None:
                    termination.flush(emit=True)
                termination.observe(phase="SCAN_LOGGING")
                logger.log_sample(
                    cycle_start,
                    raw,
                    processed,
                    robot,
                    command,
                    policy.current_target_direction,
                    policy.current_tangent,
                )
                while logged_boundaries < len(policy.boundary_points):
                    logger.log_boundary(policy.boundary_points[logged_boundaries])
                    logged_boundaries += 1
                while logged_waypoints < len(policy.policy_waypoints):
                    logger.log_waypoint(policy.policy_waypoints[logged_waypoints])
                    logged_waypoints += 1
                while logged_recovery_rays < len(policy.boundary_recovery_rays):
                    logger.log_recovery_ray(policy.boundary_recovery_rays[logged_recovery_rays])
                    logged_recovery_rays += 1
                final_reason = command.reason
                if dry_run and cycle_start >= next_print:
                    print(format_dry_run(command, processed, policy))
                    next_print = cycle_start + float(config["dry_run"]["print_interval_sec"])
                remaining = period - (time.monotonic() - cycle_start)
                if remaining > 0:
                    time.sleep(remaining)

        normal_policy_reasons = {
            "boundary point limit reached",
            "maximum experiment duration reached",
            "maximum search distance reached",
            "optional loop closure detected",
        }
        if not normal_stop and not emergency and final_reason in normal_policy_reasons:
            controller.stop()
            policy.request_normal_stop()
            normal_stop = True
            if last_robot is not None:
                logger.write_stop_snapshot(
                    last_robot, last_raw, last_processed, State.STOP_SCAN.value
                )
        elif not normal_stop and policy.state == State.STOP:
            emergency = True
            return_status = "skipped_emergency_stop"
        controller.stop()
        if termination.record is None:
            termination.set_stop_reason(detail=final_reason, source="app.scan_loop_exit", policy=policy)
        termination.flush(emit=True)
        if normal_stop and args.execute and bool(
            config["safe_return"]["auto_return_after_normal_stop"]
        ):
            assert start_pose is not None
            policy.begin_return_to_start()
            termination.observe(phase="SAFE_RETURN", policy=policy)
            executor = SafeReturnExecutor(
                config,
                start_pose,
                controller,
                reader,
                preprocessor,
                logger=logger,
                policy=policy,
            )
            return_result = executor.execute()
            return_status = return_result.status
            return_abort_reason = return_result.abort_reason
            if return_result.status == "complete":
                policy.complete_return_to_start()
            else:
                termination.set_stop_reason(detail=return_abort_reason or "safe return aborted without a reason",
                                            source="app.safe_return", terminal=False,
                                            return_status=return_status, return_abort_reason=return_abort_reason)
                policy.request_stop(return_abort_reason)
                result_code = 2
        elif normal_stop and dry_run:
            return_status = "dry_run_not_applicable"
            policy.request_stop("dry-run normal stop")
        elif normal_stop:
            return_status = "disabled_by_config"
            policy.request_stop("automatic return disabled")
        elif emergency:
            return_status = "skipped_emergency_stop"
            if last_robot is not None:
                print(f"Current/last readable TCP: {last_robot.pose.tolist()}", file=sys.stderr)
            if start_pose is not None:
                print(f"Calibrated P0: {start_pose.tolist()}", file=sys.stderr)
            print("Emergency stop: inspect robot state manually; no automatic return.", file=sys.stderr)

        while logged_waypoints < len(policy.policy_waypoints):
            logger.log_waypoint(policy.policy_waypoints[logged_waypoints])
            logged_waypoints += 1
        while logged_recovery_rays < len(policy.boundary_recovery_rays):
            logger.log_recovery_ray(policy.boundary_recovery_rays[logged_recovery_rays])
            logged_recovery_rays += 1
        # Full episode/force history is exported only after scanning and any
        # safe return have stopped, before logger.close consumes these records.
        write_probe_logs(logger.run_dir, policy.probe_episodes)
        probe_logs_exported = True
        logger.write_summary(
            policy.state.value,
            final_reason,
            len(policy.boundary_points),
            return_status=return_status,
            return_abort_reason=return_abort_reason,
            calibrated_p0=None if start_pose is None else start_pose.tolist(),
        )
        logger.flush()
        run_dir = logger.run_dir
        logger.close()
        logger = None
        try:
            from tools.visualize_run import create_strategy_debug, create_visualization

            result_path = create_visualization(run_dir, run_dir / "contour_result.png")
            print(f"Contour result saved: {result_path}")
            strategy_path, recovery_paths = create_strategy_debug(run_dir)
            print(f"Scan strategy debug saved: {strategy_path}")
            for recovery_path in recovery_paths:
                print(f"Boundary recovery debug saved: {recovery_path}")
            if (
                args.execute
                and normal_stop
                and return_status == "complete"
                and bool(config["logging"].get("show_contour_result_after_scan", True))
            ):
                try:
                    subprocess.Popen(
                        ["xdg-open", str(result_path)],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                except Exception as exc:
                    print(f"Could not open contour result viewer: {exc}", file=sys.stderr)
        except Exception as exc:
            termination.set_stop_reason(detail=f"Contour result generation failed: {type(exc).__name__}: {exc}",
                                        exception=exc, source="app.visualization", terminal=False)
            print(f"Contour result generation failed: {exc}", file=sys.stderr)
        print(f"Scan exited: state={policy.state.value}, termination_reason={termination.record['reason']}, "
              f"detail={termination.record['detail']}, points={len(policy.boundary_points)}, return={return_status}")
        return 130 if emergency else result_code
    except KeyboardInterrupt as exc:
        final_reason = "Ctrl+C"
        termination.set_stop_reason(TerminationReason.STOP_USER_REQUEST, final_reason,
                                    exception=exc, source="app.keyboard.CtrlC", policy=policy)
        emergency = True
        return_status = "skipped_emergency_stop"
        controller.safe_stop_motion() if args.execute else controller.stop()
        last_robot, fresh_pose = refresh_emergency_pose(controller, last_robot)
        print("\nCtrl+C emergency stop; automatic return is disabled.", file=sys.stderr)
        if last_robot is not None:
            label = "Current TCP" if fresh_pose else "Last readable TCP"
            print(f"{label}: {last_robot.pose.tolist()}", file=sys.stderr)
        if start_pose is not None:
            print(f"Calibrated P0: {start_pose.tolist()}", file=sys.stderr)
        if logger is not None and last_robot is not None:
            logger.write_stop_snapshot(
                last_robot, last_raw, last_processed, State.STOP.value
            )
        print("Inspect robot state manually before any return attempt.", file=sys.stderr)
        return 130
    except Exception as exc:
        final_reason = f"{type(exc).__name__}: {exc}"
        termination.set_stop_reason(detail=final_reason, exception=exc, source="app.run.exception", policy=policy)
        emergency = True
        return_status = "skipped_emergency_stop"
        controller.safe_stop_motion() if args.execute else controller.stop()
        last_robot, fresh_pose = refresh_emergency_pose(controller, last_robot)
        print(f"FAIL-CLOSED: {final_reason}", file=sys.stderr)
        if last_robot is not None:
            label = "Current TCP" if fresh_pose else "Last readable TCP"
            print(f"{label}: {last_robot.pose.tolist()}", file=sys.stderr)
        if start_pose is not None:
            print(f"Calibrated P0: {start_pose.tolist()}", file=sys.stderr)
        if logger is not None and last_robot is not None:
            logger.write_stop_snapshot(
                last_robot, last_raw, last_processed, State.STOP.value
            )
        print("Automatic return was not attempted; inspect robot state manually.", file=sys.stderr)
        return 1
    finally:
        pending_exception = sys.exc_info()[1]
        if pending_exception is not None and termination.record is None:
            termination.set_stop_reason(detail=f"{type(pending_exception).__name__}: {pending_exception}",
                                        exception=pending_exception, source="app.pending_exit", policy=policy)
        # Keep the existing stop/close order. A cleanup error must not erase the
        # scan's cause or prevent the other existing cleanup calls and report.
        cleanup_exception = None
        for name, cleanup in (("controller.stop", controller.stop),
                              ("controller.close", controller.close),
                              ("reader.close", reader.close)):
            try:
                cleanup()
            except Exception as exc:
                if cleanup_exception is None:
                    cleanup_exception = exc
                termination.set_stop_reason(detail=f"{name}: {type(exc).__name__}: {exc}",
                                            exception=exc, source="app.cleanup." + name,
                                            terminal=termination.record is None)
        if logger is not None:
            try:
                if not probe_logs_exported:
                    write_probe_logs(logger.run_dir, policy.probe_episodes)
                    probe_logs_exported = True
                logger.write_summary(
                    policy.state.value,
                    final_reason,
                    len(policy.boundary_points),
                    return_status=return_status,
                    return_abort_reason=return_abort_reason,
                    calibrated_p0=None if start_pose is None else start_pose.tolist(),
                )
            except Exception as exc:
                if cleanup_exception is None:
                    cleanup_exception = exc
                termination.set_stop_reason(detail=f"Final scan logging: {type(exc).__name__}: {exc}",
                                            exception=exc, source="app.cleanup.logging",
                                            terminal=termination.record is None)
            finally:
                try:
                    logger.close()
                except Exception as exc:
                    if cleanup_exception is None:
                        cleanup_exception = exc
                    termination.set_stop_reason(detail=f"Logger close: {type(exc).__name__}: {exc}",
                                                exception=exc, source="app.cleanup.logger",
                                                terminal=termination.record is None)
        termination.flush(emit=True)
        if cleanup_exception is not None:
            # Preserve the original failure exit; diagnosis must not turn a
            # failed cleanup into a successful scan process return.
            raise cleanup_exception


def cli_main() -> int:
    """Diagnose command-line/startup failures through the same reporting layer."""
    termination = TerminationRecorder(output_root=PROJECT_ROOT / "data")
    termination.observe(state="INITIALIZATION", phase="COMMAND_LINE")
    try:
        return run(parse_args(), termination=termination)
    except SystemExit as exc:
        if exc.code not in (None, 0) and termination.record is None:
            termination.set_stop_reason(TerminationReason.STOP_CONFIG_ERROR,
                                        f"Command-line argument parsing exited with code {exc.code}",
                                        exception=exc, source="app.cli")
            termination.flush(emit=True)
        raise
    except KeyboardInterrupt as exc:
        if termination.record is None:
            termination.set_stop_reason(TerminationReason.STOP_USER_REQUEST, "Ctrl+C during startup",
                                        exception=exc, source="app.cli")
        termination.flush(emit=True)
        return 130
    except Exception as exc:
        if termination.record is None:
            termination.set_stop_reason(detail=f"{type(exc).__name__}: {exc}",
                                        exception=exc, source="app.cli")
        termination.flush(emit=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(cli_main())
