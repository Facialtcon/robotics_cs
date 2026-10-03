"""Original discrete policy, with one explicit device lifetime and finalization."""
import argparse
import time
from dataclasses import replace
from pathlib import Path

from app.configuration import prepare_discrete
from app.operator_input import OperatorKeyboard, confirm_enter
from app.scan_startup import startup_scan, capture_stationary_bias, hold_startup_confirmation
from config.loader import load_config, runtime_robot_config
from experiment_logging.data_logger import ExperimentLogger
from experiment_logging.paths import PROJECT_ROOT
from experiment_logging.probe_log import write_probe_logs
from experiment_logging.termination import TerminationRecorder, TerminationReason
from policy.rule_policy import RuleBasedPolicy, State
from robot.rtde_controller import URRTDEController, SimulatedController, RobotError
from safety.safe_return import SafeReturnExecutor, PX6DForceMonitor
from sensor.force_preprocess import WrenchPreprocessor
from sensor.px6d_reader import PX6DReader


def run(args, *, controller_factory=URRTDEController, reader_factory=PX6DReader):
    execute = args.execute
    termination = TerminationRecorder(mode='real' if execute else 'simulation', strategy='discrete',
        config_source=args.config, output_root=getattr(args, 'output', None))
    controller = reader = logger = policy = None
    robot = raw = processed = start = None
    result, reason, return_status = 0, 'not started', 'not_requested'
    stop_info = dict(standstill_confirmed=False)
    returned = None
    try:
        config = load_config(args.config)
        sensor_kind = args.sensor or config['execution']['default_sensor']
        if not execute and sensor_kind == 'mock':
            from simulation.simulator import ContourSimulator, load_simulation_config
            scene = load_simulation_config(PROJECT_ROOT/'simulation/scene_square.yaml')
            scene['_data_root'] = getattr(args, 'output', None)
            simulator = ContourSimulator(scene)
            simulator.run_headless()
            simulator.finalize()
            return 0 if simulator.loop_completed else 1
        if execute and sensor_kind != 'real':
            raise RobotError('real robot execution requires --sensor real')
        robot_config, start = prepare_discrete(config, args.config) if execute else (runtime_robot_config(config), None)
        controller = controller_factory(robot_config) if execute else SimulatedController(
            config['dry_run']['start_pose'], robot_config['max_tcp_speed'])
        settings = config['sensor']
        reader = reader_factory(settings['serial_port'], settings['baudrate'], settings['timeout_sec'],
                                settings['poll_rate_hz'], settings['startup_delay_sec'])
        preprocessing = WrenchPreprocessor.from_config(config['preprocessing'],
            tool_orientation=start[3:] if execute else None)
        config['force_transform_status'] = preprocessing.force_transform_status
        config.setdefault('force_display', {}).update(
            frame=preprocessing.force_transform_status['output_frame'],
            base_frame_confirmed=False, physical_sign_confirmed=False)
        policy_config = dict(config['policy'])
        if not execute:
            for key in ('max_boundary_points', 'max_runtime_sec', 'probe_direction_sign'):
                policy_config[key] = config['dry_run'].get(key, policy_config[key])
        policy = RuleBasedPolicy(policy_config)
        policy.termination = termination
        logger = ExperimentLogger(getattr(args, 'output', None), config,
            mode='real' if execute else 'simulation', strategy='discrete',
            config_source=args.config, workspace_logging=execute)
        logger.termination = termination.bind(logger.run_dir)
        # A real sensor with a simulated robot is explicitly distinguished in metadata/snapshot.
        from experiment_logging.paths import annotate_run
        annotate_run(logger.run_dir, devices=dict(robot='real' if execute else 'simulated', sensor=sensor_kind))
        period = 1 / float(config['policy']['control_rate_hz'])
        reader.connect()
        controller.connect()
        with OperatorKeyboard() as keyboard:
            if execute:
                startup_return_done = startup_scan(config, start, controller, reader, logger, keyboard.poll, confirm=keyboard.read_line)
                preprocessing.set_tool_orientation(controller.read_state().pose[3:])
                keyboard.on_wait = lambda: hold_startup_confirmation(config, start, controller)
            baseline = config['preprocessing']['baseline']
            if baseline['capture_on_start']:
                if not confirm_enter('探针须脱离目标、静止空载；即将采集扫描零偏。', read_line=keyboard.read_line):
                    raise KeyboardInterrupt('bias not confirmed')
                if execute:
                    capture_stationary_bias(config, reader, preprocessing, controller, baseline['sample_count'],
                        poll=keyboard.poll, logger=logger)
                else:
                    samples = []
                    for _ in range(int(baseline['sample_count'])):
                        if keyboard.poll() in ('Q', 'ESC'):
                            raise KeyboardInterrupt('operator stopped baseline')
                        samples.append(reader.read_wrench())
                        time.sleep(period)
                    preprocessing.set_zero_bias(samples)
            if execute:
                if not confirm_enter('即将从 P0 沿保存方向开始离散扫描。', read_line=keyboard.read_line):
                    raise KeyboardInterrupt('scan not confirmed')
                hold_startup_confirmation(config, start, controller)
                if not startup_return_done:
                    controller.activate_control(confirmed=True)
            print(f'离散扫描日志: {logger.run_dir}')
            print('Q 正常停止并按配置返回 P0；ESC/Ctrl+C 原地停止。')
            counts = [0, 0, 0]
            normal = False
            while policy.state not in {State.STOP, State.STOP_SCAN, State.LOOP_COMPLETE}:
                cycle_start = time.monotonic()
                key = keyboard.poll()
                if key in ('Q', 'ESC'):
                    reason = f'{key} operator stop'
                    normal = key == 'Q'
                    termination.set_stop_reason(TerminationReason.STOP_USER_REQUEST, reason, source='discrete.keyboard')
                    if normal:
                        policy.request_normal_stop()
                    else:
                        policy.request_stop(reason)
                    break
                raw = reader.read_wrench()
                robot = controller.read_state()
                if execute:
                    preprocessing.set_tool_orientation(robot.pose[3:])
                processed = preprocessing.process(raw)
                frame = preprocessing.force_transform_status['output_frame']
                termination.observe(policy=policy, robot=robot, raw=raw, processed=processed,
                                    phase='SCAN', processed_force_frame=frame)
                command = policy.update(cycle_start, raw, processed, robot)
                reason = command.reason
                if command.move:
                    if execute and controller._stop_pending and not controller.standstill_confirmed:
                        command = replace(command, move=False, speed=0., reason='waiting for fresh held standstill')
                    else:
                        controller.command_planar_velocity(command.direction_xy, command.speed, period)
                elif not execute or not controller.standstill_confirmed:
                    controller.stop()
                logger.log_sample(cycle_start, raw, processed, robot, command,
                                  policy.current_target_direction, policy.current_tangent,
                                  extra={'processed_force_frame': frame})
                for i, (items, write) in enumerate(((policy.boundary_points, logger.log_boundary),
                    (policy.policy_waypoints, logger.log_waypoint), (policy.boundary_recovery_rays, logger.log_recovery_ray))):
                    while counts[i] < len(items):
                        write(items[counts[i]])
                        counts[i] += 1
                time.sleep(max(0., period-(time.monotonic()-cycle_start)))
            normal = normal or reason in {'boundary point limit reached', 'maximum experiment duration reached',
                'maximum search distance reached', 'optional loop closure detected', 'policy runtime limit reached'} or policy.state == State.LOOP_COMPLETE
            controller.stop()
            if execute:
                # Keep PX6D checks alive through ordinary braking and any return.
                monitor = PX6DForceMonitor(config, reader, preprocessing)
                def observe_stop():
                    monitor.sample()
                    return controller.read_diagnostic_state()
                robot = controller.wait_for_standstill(observe=observe_stop)
            else:
                robot = controller.read_state()
            logger.write_stop_snapshot(robot, raw, processed, policy.state.value)
            if normal and execute and config['safe_return']['auto_return_after_normal_stop']:
                if policy.state != State.STOP_SCAN:
                    policy.request_normal_stop()
                policy.begin_return_to_start()
                returned = SafeReturnExecutor(config, start, controller, force_monitor=monitor,
                    logger=logger, poll=keyboard.poll).execute()
                return_status = returned.status
                if returned.status != 'complete':
                    raise RobotError(returned.abort_reason)
                policy.complete_return_to_start()
            elif not normal:
                return_status = 'skipped_emergency_stop'
                result = 130 if termination.record and termination.record['reason'] == TerminationReason.STOP_USER_REQUEST.value else 1
            else:
                return_status = 'disabled_by_config' if execute else 'dry_run_not_applicable'
    except KeyboardInterrupt as exc:
        reason, result, return_status = str(exc) or 'Ctrl+C', 130, 'skipped_emergency_stop'
        termination.set_stop_reason(TerminationReason.STOP_USER_REQUEST, reason, source='discrete.interrupt')
    except Exception as exc:
        reason, result, return_status = f'{type(exc).__name__}: {exc}', 1, 'skipped_error'
        termination.set_stop_reason(detail=reason, exception=exc, source='discrete.runtime')
    finally:
        # Stop and fresh standstill precede logging, rendering and disconnection.
        if controller is not None:
            try:
                controller.stop()
                robot = controller.wait_for_standstill() if execute else controller.read_state()
                stop_info = dict(standstill_confirmed=True, stop_requests=controller.stop_history if execute else [])
            except Exception as exc:
                result = 1
                termination.set_stop_reason(TerminationReason.STOP_MOTION_ERROR, str(exc), source='discrete.stop', terminal=False)
        for resource in (controller, reader):
            if resource is not None:
                try:
                    resource.close()
                except Exception as exc:
                    result = 1
                    termination.set_stop_reason(detail=str(exc), exception=exc, source='discrete.close', terminal=False)
        if logger is not None:
            try:
                if policy is not None:
                    write_probe_logs(logger.run_dir, policy.probe_episodes)
                if robot is not None:
                    logger.write_stop_snapshot(robot, raw, processed, policy.state.value,
                                               extra=dict(stop_observation=stop_info))
                logger.write_summary(policy.state.value, reason, len(policy.boundary_points),
                    return_status=return_status, return_abort_reason='' if returned is None else returned.abort_reason,
                    stop_observation=stop_info)
            except Exception as exc:
                result = 1
                termination.set_stop_reason(detail=str(exc), exception=exc, source='discrete.logging', terminal=False)
            finally:
                logger.close()
        if termination.record is not None:
            termination.flush(emit=True)
    if logger is not None and result == 0:
        from tools.visualize_run import create_visualization
        try:
            create_visualization(logger.run_dir, logger.run_dir/'contour_result.png')
        except Exception as exc:
            print(f'轨迹绘图失败（原始日志已保存）: {exc}')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=PROJECT_ROOT/'config.yaml')
    parser.add_argument('--sensor', choices=('mock', 'real'), default=None)
    parser.add_argument('--execute', action='store_true')
    return run(parser.parse_args())
