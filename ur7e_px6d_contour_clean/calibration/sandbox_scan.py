"""Read, teach and persist raw sandbox points; geometry is computed by workspace modules."""
from copy import deepcopy
from pathlib import Path
import time

import numpy as np
import yaml

from calibration.single_point import write_calibration
from calibration.scan_calibration import resolve_calibration_path, pose_to_mapping, POSE_FIELDS
from config.loader import runtime_robot_config
from experiment_logging.paths import wall_time_fields
from robot.rtde_controller import URRTDEController
from robot.tcp_identity import tcp_offsets_match, read_tcp_offset_readonly


def raw_workspace(workspace):
    """Normalize pose data only; ignore cached fits in older saved records."""
    if not isinstance(workspace, dict) or not isinstance(workspace.get('raw_points'), dict):
        raise ValueError('沙箱标定缺少四角原始 raw_points')
    raw = workspace['raw_points']
    if set(raw) != {f'P{i}' for i in range(4)}:
        raise ValueError('沙箱四角原始点必须包含 P0/P1/P2/P3（对应 C0/C1/C2/C3）')
    points = {}
    for name, pose in raw.items():
        values = [pose[field] for field in POSE_FIELDS] if isinstance(pose, dict) else pose
        points[name] = pose_to_mapping(values)
    return dict(raw_points=points, metadata=deepcopy(workspace.get('metadata', {})))


def load_raw_workspace(path):
    return raw_workspace(yaml.safe_load(Path(path).read_text(encoding='utf-8')))


def load_record(path):
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data, dict) or not isinstance(data.get('sandbox_scan'), dict):
        raise ValueError('尚未保存固定起点沙箱标定；请在新入口选择重新标记')
    record = deepcopy(data['sandbox_scan'])
    pose = np.asarray(record['p0_tcp_pose'], dtype=float)
    if pose.shape != (6,) or not np.isfinite(pose).all() or not np.isfinite(record['scan_z']):
        raise ValueError('沙箱标定中的 P0 或扫描高度无效')
    record['workspace'] = raw_workspace(record['workspace'])
    return record


def save_record(path, record, *, updated_fields=None):
    path = Path(path).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    original = path.read_bytes() if path.exists() else None
    data = yaml.safe_load(original) if original is not None else {}
    if not isinstance(data, dict):
        raise ValueError('原扫描标定必须为 YAML mapping；未覆盖原文件')
    if updated_fields is None:
        stored = deepcopy(record)
        stored['workspace'] = raw_workspace(stored['workspace'])
    else:
        # Preserve unrelated saved fields verbatim, including legacy metadata
        # and cached geometry when only P0 changes. New planning ignores caches.
        stored = deepcopy(data.get('sandbox_scan', record))
        for field in updated_fields:
            stored[field] = deepcopy(record[field])
        if 'workspace' in updated_fields:
            stored['workspace'] = raw_workspace(stored['workspace'])
    data['sandbox_scan'] = stored
    if original is not None:
        path.with_name(f'{path.stem}.backup-{time.time_ns()}{path.suffix}').write_bytes(original)
    try:
        write_calibration(path, data)
        load_record(path)
    except BaseException:
        if original is None:
            path.unlink(missing_ok=True)
        else:
            path.write_bytes(original)
        raise


def teach(project, path, *, controller_factory=URRTDEController, confirm=None,
          scope='all', existing=None):
    from app.operator_input import confirm_enter
    confirm = confirm or confirm_enter
    if scope not in ('all', 'p0', 'corners'):
        raise ValueError(f'未知标定操作：{scope}')
    record = (deepcopy(existing) if existing is not None else
              load_record(path) if Path(path).exists() else None)
    if scope != 'all' and record is None:
        raise ValueError('单独重标需要已有标定，请先加载原始点或选择重新标定全部')
    if scope != 'all' and (record['robot_ip'] != project['robot']['robot_ip'] or
        not tcp_offsets_match(record['active_tcp_offset'], project['tcp']['offset'], project['tcp']['offset_tolerance'])):
        raise ValueError('已有标定的机器人或 TCP 与当前配置不一致，不能混合更新点位')
    controller = controller_factory(runtime_robot_config(project))
    names = ('P0',) if scope == 'p0' else ('C0','C1','C2','C3') if scope == 'corners' else ('P0','C0','C1','C2','C3')
    print('人工示教：'+ ' → '.join(names)+'。原始点位单独保存；已有扫描高度保持不变。')
    points = {}
    try:
        controller.connect()  # Receive-only; never activate_control().
        for name in names:
            if not confirm(f'用示教器移至 {name} 并停稳，按 Enter 读取'):
                raise KeyboardInterrupt('标记取消，原标定保留')
            active_tcp = read_tcp_offset_readonly(project['robot']['robot_ip'])
            if not tcp_offsets_match(active_tcp, project['tcp']['offset'], project['tcp']['offset_tolerance']):
                raise ValueError('标记过程中 active TCP 与配置不一致；原标定保留')
            points[name] = controller.wait_for_standstill().pose.tolist()
            print(f'{name}: {points[name]}', flush=True)
        if not tcp_offsets_match(active_tcp, project['tcp']['offset'], project['tcp']['offset_tolerance']):
            raise ValueError('active TCP 与配置不一致')
        if record is None:
            old_path = resolve_calibration_path(project['_config_source'], project['calibration']['file'])
            old = yaml.safe_load(old_path.read_text(encoding='utf-8')) if old_path.exists() else {}
            # Only first creation initializes scan Z from the existing convention.
            # Re-teaching never substitutes the newly measured P0.z for scan Z.
            scan_z = old.get('fixed_z') if isinstance(old, dict) else None
            if scan_z is None:
                scan_z = project['robot'].get('fixed_z')
            if scan_z is None:
                scan_z = points['P0'][2]
            record = dict(scan_z=float(scan_z))
        captured = wall_time_fields()
        fields = []
        if scope in ('p0', 'all'):
            record.update(p0_tcp_pose=points['P0'], p0_captured_at=captured)
            fields += ['p0_tcp_pose', 'p0_captured_at']
        if scope in ('corners', 'all'):
            workspace = deepcopy(record.get('workspace', {}))
            workspace['raw_points'] = {f'P{i}': points[f'C{i}'] for i in range(4)}
            record.update(workspace=raw_workspace(workspace), corners_captured_at=captured)
            fields += ['workspace', 'corners_captured_at']
        if scope == 'all':
            record.update(robot_ip=project['robot']['robot_ip'], active_tcp_offset=active_tcp, captured_at=captured)
        save_record(path, record, updated_fields=None if scope == 'all' else fields)
    finally:
        controller.close()
    print(f'已保存 {Path(path).resolve()}；扫描高度保持为 {record["scan_z"]:g} m。')
    return record
