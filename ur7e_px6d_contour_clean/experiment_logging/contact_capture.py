"""Small capture helpers. No motion command, policy change, or visualization."""
from dataclasses import replace
from copy import deepcopy
import time

import numpy as np


def drain_commands(controller, logger):
    records = getattr(controller, 'command_records', None)
    if records:
        logger.log_commands(list(records))
        records.clear()


def stationary_preroll(config, controller, reader, preprocessor, logger, poll, *, duration=1.):
    """Record a full second before search using existing startup/force guards."""
    from app.scan_startup import check_startup_stationary
    from core.models import PolicyCommand
    from robot.rtde_controller import RobotError
    from safety.force_guard import continuous_force_reason, raw_safety_reason
    anchor = controller.read_diagnostic_state().pose.copy()
    preprocessor = deepcopy(preprocessor)  # Recording must not advance the control filter.
    command = PolicyCommand('PRECONTACT', False, np.zeros(2), 0., anchor, False, False)
    first = None
    diagnostics = {}
    while True:
        started = time.monotonic()
        if poll() in ('Q', 'ESC'):
            raise KeyboardInterrupt('operator stopped precontact recording')
        raw = reader.read_wrench()
        serial_end = time.monotonic()
        error = raw_safety_reason(raw, config['policy'])
        if error:
            raise RobotError(error)
        robot = controller.read_diagnostic_state()
        check_startup_stationary(config, robot, controller, anchor=anchor)
        preprocessor.set_tool_orientation(robot.pose[3:])
        processed = preprocessor.process(raw)
        error = continuous_force_reason(raw, processed, config['policy'], diagnostics)
        if error:
            raise RobotError(error)
        if controller.config.get('continuous_require_watchdog') and controller.watchdog_active:
            controller._check_watchdog_health()
            controller.kick_watchdog()
        drain_commands(controller, logger)
        timing = {k: v for k, v in controller.observation_timing.items()
                  if k in ('tcp_read_start', 'tcp_read_end', 'rtde_device_timestamp', 'rtde_packet_stagnation_sec')}
        logger.log_sample(robot.timestamp, raw, processed, robot, replace(command, target_pose=robot.pose.copy()),
            None, None, extra={**preprocessor.force_log_fields, **timing,
                'processed_force_frame': 'Base', 'serial_read_start': started, 'serial_read_end': serial_end,
                'timing_source': 'host_read_intervals; PX6D sample timestamp unavailable'})
        if first is None:
            first = robot.timestamp
        if robot.timestamp-first >= duration:
            break
        logger.check_health()
        time.sleep(max(0., 1/float(config['policy']['control_rate_hz'])-(time.monotonic()-started)))


def simulation_preroll(session, logger):
    """Synthetic stationary history before t=0; leave motion/policy clocks intact."""
    from core.models import RobotState, PolicyCommand
    processor = deepcopy(session.preprocessor)
    robot = session.robot.read_state()
    count = int(np.ceil(1./session.dt))+1
    command = PolicyCommand('PRECONTACT', False, np.zeros(2), 0., robot.pose.copy(), False, False)
    for i in range(count):
        now = (i-count)*session.dt
        raw, _ = session.sensor.read_wrench(robot.pose[:2], np.zeros(2))
        processed = processor.process(raw)
        logger.log_sample(now, raw, processed, RobotState(now, robot.pose.copy(), robot.tcp_speed.copy()),
            command, None, None, extra={**processor.force_log_fields, 'processed_force_frame': 'Base',
                'serial_read_start': now, 'serial_read_end': now,
                'tcp_read_start': now, 'tcp_read_end': now, 'timing_source': 'synthetic common clock'})
