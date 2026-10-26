"""Fixed-start adapter for the shared continuous runner; never changes project defaults."""
from copy import deepcopy
from pathlib import Path

import numpy as np

from app.configuration import prepare_real, workspace_calibration_path, search_stopping_margin
from calibration.sandbox_scan import load_record
from calibration.scan_calibration import resolve_calibration_path, load_scan_calibration
from policy.coverage_search import CoverageSearch
from robot.rtde_controller import RobotError
from robot.tcp_identity import tcp_offsets_match
from safety.safe_return import return_trajectory
from workspace.sandbox_scan import plan_coverage
from workspace.workspace_transform import load_calibration

ROOT = Path(__file__).resolve().parents[1]


def calibration_path(config, config_path, override=None):
    return Path(override).resolve() if override else resolve_calibration_path(
        config_path, config.get('sandbox_scan', {}).get('calibration_file', 'calibration_data/sandbox_scan.yaml'))


def read_calibration(config, config_path, path):
    path = Path(path).resolve()
    if path.exists() or path != calibration_path(config, config_path):
        return load_record(path)
    # Existing successful runs already have every required teach point. Use
    # those records directly until the operator chooses to teach a new set.
    scan_path = resolve_calibration_path(config_path, config['calibration']['file'])
    workspace_path = workspace_calibration_path(config, config_path)
    scan = load_scan_calibration(scan_path, require_tcp_offset=True)
    return dict(p0_tcp_pose=scan['start_tcp_pose'], scan_z=scan['fixed_z'],
        robot_ip=scan['robot_ip'], active_tcp_offset=scan['active_tcp_offset'],
        workspace=load_calibration(workspace_path),
        imported_sources=dict(scan=str(scan_path.resolve()), workspace=str(workspace_path.resolve())))


def make_plan(config, record):
    options = config.get('sandbox_scan', {})
    if not isinstance(options, dict):
        raise ValueError('sandbox_scan 配置必须为 mapping')
    def number(key, default):
        try:
            return float(options.get(key, default))
        except (TypeError, ValueError) as exc:
            raise ValueError(f'sandbox_scan.{key} 必须为数值') from exc
    # Inherit the existing search stopping clearance unless explicitly set.
    # A tiny numerical pad keeps exactly coincident fitted/raw edges usable.
    plan = plan_coverage(record['workspace'], record['p0_tcp_pose'],
        scan_z=number('scan_z', record['scan_z']),
        line_spacing=number('line_spacing', .01),
        boundary_margin=number('boundary_margin', search_stopping_margin(config)+
            record['workspace']['diagnostics']['fit_max_xy_m']+1e-9))
    plan['search_speed_mps'] = float(config['continuous_tracking']['search_speed'])
    plan['tracking_base_speed_mps'] = float(config['continuous_tracking']['tangential_speed'])
    p0 = np.asarray(plan['p0_tcp_pose'])
    start = p0.copy()
    start[2] = plan['scan_z']
    if plan['entry_kind'] == 'lift_translate_descend':
        start[:2] = plan['coverage_start_xy']
        segments = return_trajectory(config, p0, start, via_pose=p0)
        plan['entry_xyz'] = [p0[:3].tolist()] + [pose[:3].tolist() for phase,pose,_ in segments
                                               if phase != 'ALIGN_PROBE_ORIENTATION']
    else:
        plan['entry_xyz'] = [np.r_[p0[:2], plan['scan_z']].tolist(),
                             [*plan['coverage_start_xy'], plan['scan_z']]]
    plan['runtime_start_pose'] = start.tolist()
    return plan


def prepare_sandbox(config, config_path, path, *, expected_plan=None):
    record = read_calibration(config, config_path, path)
    if record['robot_ip'] != config['robot']['robot_ip']:
        raise RobotError('沙箱 P0 标定的 robot_ip 与配置不一致')
    if not tcp_offsets_match(record['active_tcp_offset'], config['tcp']['offset'], config['tcp']['offset_tolerance']):
        raise RobotError('沙箱 P0 标定 TCP 与配置不一致')
    plan = make_plan(config, record)
    if expected_plan is not None and plan != expected_plan:
        raise ValueError('预览后配置或标定发生变化，请重新生成预览')
    start = np.asarray(plan['runtime_start_pose'])
    direction = np.asarray(plan['first_direction_xy'])
    if plan['entry_kind'] == 'planar_search':
        delta = np.asarray(plan['coverage_start_xy'])-start[:2]
        if np.linalg.norm(delta) > config['policy']['position_tolerance']:
            direction = delta/np.linalg.norm(delta)
    # A synthetic direction reference satisfies the shared two-point adapter;
    # it is never taught, persisted as P1, or used as an endpoint.
    reference = start.copy()
    reference[:2] += direction*float(config['calibration']['min_direction_calibration_distance'])*1.01
    scan = dict(robot_ip=record['robot_ip'], active_tcp_offset=record['active_tcp_offset'],
        start_tcp_pose=start.tolist(), fixed_z=plan['scan_z'], fixed_orientation=start[3:].tolist(),
        scan_direction_xy=direction.tolist(), direction_reference_tcp_pose=reference.tolist())
    workspace_path = workspace_calibration_path(config, config_path)
    # The NEW rectangle only plans search; post-contact motion uses the same
    # saved original execution region as run_continuous_tracking.py.
    original_workspace = load_calibration(workspace_path)
    search_path = ([start[:2].tolist()] if plan['entry_kind'] == 'planar_search' else []) + plan['coverage_xy']
    try:
        robot_config, start = prepare_real(config, config_path, search_path=search_path,
            scene_calibration=dict(scan=scan, workspace=original_workspace,
                sources=dict(scan=(record.get('imported_sources', {}).get('scan') or str(Path(path).resolve())+'#sandbox_scan'),
                             workspace=str(workspace_path.resolve()))))
    except RobotError as exc:
        if 'boundary' in str(exc):
            raise RobotError('新覆盖路线与原工程运动边界或停车余量冲突；请调整新模式范围或 boundary_margin。'
                             f'具体原因：{exc}') from exc
        raise
    # Validate the configured workspace's XYZ bounds before any connection.
    if config['workspace'].get('enabled', True):
        bounds = config['workspace']['limits']
        for xy in search_path:
            for i, axis in enumerate('xyz'):
                value = xy[i] if i < 2 else plan['scan_z']
                if not bounds[f'{axis}_min'] <= value <= bounds[f'{axis}_max']:
                    raise RobotError(f'覆盖路线超出原工程 workspace {axis} 边界')
    # Teaching finishes at a corner, which may be on the old polygon edge.
    # Only the existing force-monitored startup return can traverse outside
    # the sandbox plane. Configured workspace bounds and force guards remain.
    # Restore the original polygon above the startup target, before insertion.
    keys = ('continuous_xy_polygon', 'continuous_xy_limits')
    robot_config['sandbox_entry_xy_guard'] = {k: robot_config[k] for k in keys}
    for key in keys:
        robot_config[key] = None
    config['sandbox_scan_plan'] = deepcopy(plan)
    return robot_config, start


def search_strategy(config, start):
    return CoverageSearch(config['sandbox_scan_plan'], start, config['policy']['position_tolerance'])


def run_sandbox(args, plan, *, controller_factory=None, reader_factory=None):
    from app.continuous_runtime import run
    from visualization.sandbox_scan import generate_run_visualization
    options = dict(visualize_run=generate_run_visualization, prepare_execution=lambda c,p: prepare_sandbox(c,p,args.calibration,expected_plan=plan),
                   search_strategy_factory=search_strategy)
    if controller_factory is not None:
        options['controller_factory'] = controller_factory
    if reader_factory is not None:
        options['reader_factory'] = reader_factory
    return run(args, **options)
