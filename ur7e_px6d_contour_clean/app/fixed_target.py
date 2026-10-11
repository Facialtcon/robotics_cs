"""Fixed-target scene adapter; no target-truth or single-point trial dependency."""
from copy import deepcopy
from pathlib import Path

import numpy as np
import yaml

from app.configuration import prepare_real
from calibration.scan_calibration import calculate_scan_direction_xy, resolve_calibration_path
from calibration.single_point import vector
from config.loader import load_config
from robot.control_mode import read_control_mode
from robot.rtde_controller import RobotError
from robot.tcp_identity import tcp_offsets_match
from safety.safe_return import return_trajectory

ROOT = Path(__file__).resolve().parents[1]


def load_settings(path):
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError('fixed-target settings must be a mapping')
    unknown = set(data) - {'calibration_file', 'workspace'}
    if unknown:
        raise ValueError(f'unknown fixed-target settings: {sorted(unknown)}')
    return data


def read_starts(settings, settings_path):
    path = resolve_calibration_path(settings_path, settings['calibration_file'])
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict) or data.get('schema_version') != 1 or not data.get('groups'):
        raise ValueError('fixed-target P0/P_ref calibration requires saved groups')
    return data, path


def prepare_fixed(config, config_path, settings, settings_path, p0):
    """Replace only scene geometry before the original real-execution preflight."""
    data, path = read_starts(settings, settings_path)
    reference_xyz = vector(data['P_ref'], 3, 'P_ref')
    if not np.allclose(reference_xyz, vector(data['reference_tcp_pose'][:3], 3, 'P_ref XYZ'), atol=1e-12, rtol=0):
        raise ValueError('P_ref differs from its saved reference pose')
    group = data['groups'][p0]
    measured = vector(group['tcp_pose'], 6, 'P0 pose')
    if data['robot_ip'] != config['robot']['robot_ip']:
        raise RobotError('P0/P_ref robot identity differs from config.yaml')
    for tcp in (data['active_tcp_offset'], group['active_tcp_offset']):
        if not tcp_offsets_match(tcp, config['tcp']['offset'], config['tcp']['offset_tolerance']):
            raise RobotError('P0/P_ref TCP differs from config.yaml')
    # P_ref supplies direction and common Z only. Its posture is ignored.
    # The original preflight derives the downward posture from the taught P0.
    start = measured.copy()
    start[2] = reference_xyz[2]
    reference = np.r_[reference_xyz, measured[3:]]
    direction = calculate_scan_direction_xy(start, reference)
    workspace = settings.get('workspace')
    if not isinstance(workspace, dict) or workspace.get('verified') is not True:
        raise RobotError('固定目标工作区域须经现场确认：配置 workspace.verified: true 和有效 XYZ limits；不使用颗粒箱边界。')
    if set(workspace) - {'verified', 'limits'}:
        raise ValueError('fixed workspace accepts only verified and limits')
    limits = workspace.get('limits')
    if not isinstance(limits, dict) or set(limits) != {f'{a}_{s}' for a in 'xyz' for s in ('min', 'max')}:
        raise RobotError('固定目标工作区域必须包含独立的六个 XYZ limits')
    limits = {key: float(value) for key, value in limits.items()}
    for axis in 'xyz':
        lo, hi = limits[axis+'_min'], limits[axis+'_max']
        if not np.isfinite([lo, hi]).all() or lo >= hi:
            raise RobotError('固定目标工作区域 limits 必须有限且 min < max')
    config['workspace'] = dict(enabled=True, limits=limits)
    corners = [[limits['x_min'], limits['y_min']], [limits['x_max'], limits['y_min']],
               [limits['x_max'], limits['y_max']], [limits['x_min'], limits['y_max']]]
    scan = dict(robot_ip=data['robot_ip'], active_tcp_offset=deepcopy(data['active_tcp_offset']),
                start_tcp_pose=start.tolist(), direction_reference_tcp_pose=reference.tolist(),
                scan_direction_xy=direction.tolist(), fixed_z=float(reference[2]),
                fixed_orientation=reference[3:].tolist())
    region = dict(raw_points={f'P{i}': dict(x=x, y=y) for i, (x, y) in enumerate(corners)})
    # The granular project's optional attestation is tied to its old files.
    config['continuous_tracking']['site_verification'] = None
    config['fixed_target_start'] = dict(p0=p0, measured_tcp_pose=measured.tolist(),
                                      P_ref=reference_xyz.tolist(), reference_is_endpoint=False)
    robot_config, start = prepare_real(config, config_path, scene_calibration=dict(scan=scan, workspace=region,
        sources=dict(scan=str(path.resolve()), workspace=str(Path(settings_path).resolve()))))
    return_trajectory(config, start, start)  # Existing check also covers the raised air-zero height.
    return robot_config, start


def configure_fixed(config):
    config['fixed_target_experiment'] = True
    config['experiment'] = dict(kind='fixed_target_tracking', title='固定目标连续跟踪')
    config['preprocessing'].setdefault('granular_baseline', {})['capture_on_start'] = False
    config['preprocessing']['granular_baseline_output'] = [0.] * 6
    print('固定目标连续跟踪；最终继承配置（workspace 将由独立现场区域替换）：\n' +
          yaml.safe_dump({key: config[key] for key in ('robot', 'sensor', 'tcp', 'preprocessing',
              'policy', 'continuous_tracking', 'safe_return', 'logging')},
              allow_unicode=True, sort_keys=False), flush=True)


def require_remote(robot_ip):
    status = read_control_mode(robot_ip)
    print(f'机器人控制模式：{status.mode}；{status.detail}', flush=True)
    if status.mode != 'remote':
        raise RobotError('自动实验要求 Remote 模式；' + status.detail)


def run_fixed(args, *, controller_factory=None, reader_factory=None, mode_check=None):
    from app.continuous_runtime import run
    from visualization.fixed_target import generate_fixed_visualization
    settings = load_settings(args.experiment_config)
    if args.execute and args.p0 is None:
        data, _ = read_starts(settings, args.experiment_config)
        names = list(data['groups'])
        if len(names) == 1:
            args.p0 = names[0]
        else:
            from app.operator_input import OperatorKeyboard
            print('已标定 P0：' + '、'.join(names), flush=True)
            with OperatorKeyboard() as keyboard:
                args.p0 = keyboard.read_text('选择 P0 名称：').strip()
        if args.p0 not in names:
            raise ValueError('请选择一个已标定的 P0')
    def configure(config):
        configure_fixed(config)
        if not args.execute:
            config['experiment']['kind'] = 'synthetic_demo'
            print('离线模式：使用原模拟器及独立合成正方形场景；不读取现场 P0 或真实目标轮廓。', flush=True)
    def simulate(config, scene):
        from simulation.continuous_session import SimulationSession
        # Use the inherited actuator acceleration for this mock environment.
        # All policy formulas, force limits and the original model delay remain.
        return SimulationSession(config, scene, execution_dynamics=dict(
            acceleration_limit=config['robot']['speed_acceleration']))
    options = dict(configure_experiment=configure,
        prepare_execution=lambda c, p: prepare_fixed(c, p, settings, args.experiment_config, args.p0),
        check_control_mode=mode_check or require_remote,
        simulation_factory=simulate,
        visualize_run=lambda path: generate_fixed_visualization(path, args.target_geometry if args.execute else None))
    if controller_factory is not None:
        options['controller_factory'] = controller_factory
    if reader_factory is not None:
        options['reader_factory'] = reader_factory
    return run(args, **options)


def check_calibration(args):
    settings = load_settings(args.experiment_config)
    data, _ = read_starts(settings, args.experiment_config)
    names = [args.p0] if args.p0 else list(data['groups'])
    for name in names:
        config = load_config(args.config)
        configure_fixed(config)
        _, start = prepare_fixed(config, args.config, settings, args.experiment_config, name)
        print(yaml.safe_dump(dict(p0=name, start=start.tolist(),
            direction=config['policy']['search_direction_xy'],
            workspace=config['workspace'], execution_envelope=config['continuous_execution_envelope']),
            allow_unicode=True, sort_keys=False))
    return 0
