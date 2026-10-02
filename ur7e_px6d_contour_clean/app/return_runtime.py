"""UR-only manual reset/P0 return. No sensor or tracking policy imports."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import yaml

from calibration.reset_pose import load_reset_pose, resolve_reset_pose_path
from calibration.scan_calibration import load_scan_calibration, resolve_calibration_path
from config.loader import load_config, runtime_robot_config
from experiment_logging.paths import PROJECT_ROOT, create_run, read_metadata
from robot.rtde_controller import RobotError, URRTDEController, _orientation_distance
from robot.tcp_identity import tcp_offsets_match
from safety.safe_return import SafeReturnExecutor, return_trajectory, print_return_plan
from app.operator_input import OperatorKeyboard, confirm_enter


def load_return_target(config_path, config, *, p0_only=False):
    reset_path = resolve_reset_pose_path(config_path, config['reset']['file'])
    if reset_path.is_file() and not p0_only:
        target = load_reset_pose(reset_path)
        return dict(pose=target['reset_tcp_pose'], tcp=target['active_tcp_offset'],
                    robot_ip=target['robot_ip'], label='RESET', source=str(reset_path))
    path = resolve_calibration_path(config_path, config['calibration']['file'])
    target = load_scan_calibration(path, require_tcp_offset=True)
    return dict(pose=target['start_tcp_pose'], tcp=target['active_tcp_offset'],
                robot_ip=target['robot_ip'], label='P0', source=str(path))


class ReturnLog:
    def __init__(self, config, data_root=None):
        self.run_dir = create_run('real', 'return', config['_config_source'], data_root=data_root)
        (self.run_dir/'config_snapshot.yaml').write_text(yaml.safe_dump(config, allow_unicode=True), encoding='utf-8')
        self.handle = None
        self.writer = None

    def log_return(self, state, phase, target):
        if self.writer is None:
            self.handle = (self.run_dir/'samples.csv').open('w', newline='', encoding='utf-8')
            self.writer = csv.writer(self.handle)
            axes = ('x', 'y', 'z', 'rx', 'ry', 'rz')
            self.writer.writerow(['monotonic_sec', 'phase'] + [f'tcp_{x}' for x in axes]
                                 + [f'tcp_v{x}' for x in axes])
        self.writer.writerow([state.timestamp, phase, *state.pose, *state.tcp_speed])

    def write_json(self, name, payload):
        (self.run_dir/name).write_text(json.dumps(payload, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')

    def close(self):
        if self.handle is not None:
            self.handle.close()


def run(config_path=PROJECT_ROOT/'config.yaml', *, controller_factory=URRTDEController,
        confirm=None, poll=None, data_root=None, p0_only=False):
    config = load_config(config_path)
    target = load_return_target(config_path, config, p0_only=p0_only)
    if target['robot_ip'] != config['robot']['robot_ip']:
        raise RobotError('return target belongs to a different robot')
    if not tcp_offsets_match(target['tcp'], config['tcp']['offset'], config['tcp']['offset_tolerance']):
        raise RobotError('return target TCP does not match configuration')
    controller = controller_factory(runtime_robot_config(config))
    logger = ReturnLog(config, data_root)
    result, status, reason = 1, 'aborted', ''
    try:
        controller.connect()
        current = controller.wait_for_standstill()
        print('UR-only return:\n没有 PX6D 外部力监控。\n依赖现场确认及 UR 自身安全系统。')
        print(f'当前 TCP: {current.pose.tolist()}\n目标 {target["label"]}: {target["pose"]}\n来源: {target["source"]}')
        segments = return_trajectory(config, current.pose, target['pose'])
        print_return_plan(config, current.pose, target['pose'], segments)
        if not confirm_enter('核对完整四段路径；即将返回上述目标。', read_line=confirm):
            result, status, reason = 0, 'cancelled', 'return not confirmed'
        else:
            fresh = controller.wait_for_standstill()
            if (np.linalg.norm(fresh.pose[:3]-current.pose[:3]) > config['safe_return']['return_position_tolerance'] or
                _orientation_distance(fresh.pose[3:], current.pose[3:]) > config['safe_return']['return_orientation_tolerance']):
                raise RobotError('robot moved during confirmation; review the return path again')
            controller.activate_control(confirmed=True)
            returned = SafeReturnExecutor(config, target['pose'], controller, force_monitor=None,
                logger=logger, poll=poll, target_label=target['label']).execute()
            result = 0 if returned.status == 'complete' else 1
            status, reason = returned.status, returned.abort_reason
    except (Exception, KeyboardInterrupt) as exc:
        reason = f'{type(exc).__name__}: {exc}'
    finally:
        try:
            controller.close()
        except Exception as exc:
            result, status, reason = 1, 'aborted', f'{reason}; {exc}'
        logger.write_json('summary.json', dict(**read_metadata(logger.run_dir), status=status,
            reason=reason, target=target, stop_requests=controller.stop_history))
        status_path = logger.run_dir/'return_status.json'
        detail = json.loads(status_path.read_text()) if status_path.exists() else {}
        detail.update(return_status=status, return_abort_reason=reason,
            return_target=target['pose'], force_monitor=False, stop_requests=controller.stop_history)
        logger.write_json('return_status.json', detail)
        logger.close()
    print(f'Return {status}: {reason}\n日志: {logger.run_dir}')
    return result


def main(*, p0_only=False):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=PROJECT_ROOT/'config.yaml')
    args = parser.parse_args()
    with OperatorKeyboard() as keyboard:
        return run(args.config, confirm=keyboard.read_line, poll=keyboard.poll, p0_only=p0_only)
