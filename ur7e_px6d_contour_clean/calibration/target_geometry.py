"""Four taught Base XY corners, exclusively for display/offline analysis."""
from pathlib import Path
import time

import numpy as np
import yaml

from calibration.single_point import write_calibration
from experiment_logging.paths import wall_time_fields


def make_geometry(points):
    # Read XY only: neither saved Z nor posture affects this display contour.
    corners = {f'P{i}': np.asarray(points[f'P{i}'][:2], dtype=float).tolist() for i in range(1, 5)}
    return validate_geometry(dict(schema_version=1, frame='Base', units='m',
        usage='visualization_only', corners=corners, captured_at=wall_time_fields()))


def validate_geometry(data):
    if not isinstance(data, dict) or data.get('schema_version') != 1 or data.get('frame') != 'Base' or data.get('units') != 'm':
        raise ValueError('target geometry must be four Base XY corners in meters')
    if data.get('usage') != 'visualization_only':
        raise ValueError('target geometry must be marked visualization_only')
    xy = np.asarray([data['corners'][f'P{i}'] for i in range(1, 5)], dtype=float)
    if xy.shape != (4, 2) or not np.isfinite(xy).all():
        raise ValueError('P1/P2/P3/P4 must contain finite XY only')
    edges = np.roll(xy, -1, axis=0)-xy
    turns = edges[:, 0]*np.roll(edges, -1, axis=0)[:, 1]-edges[:, 1]*np.roll(edges, -1, axis=0)[:, 0]
    if not (np.all(turns > 0) or np.all(turns < 0)):
        raise ValueError('teach P1/P2/P3/P4 in perimeter order, clockwise or counterclockwise')
    return data


def load_geometry(path):
    return validate_geometry(yaml.safe_load(Path(path).read_text(encoding='utf-8')))


def outline_mm(data):
    data = validate_geometry(data)
    xy = np.asarray([data['corners'][f'P{i}'] for i in range(1, 5)], dtype=float)
    return np.vstack((xy, xy[0]))*1000.


def save_geometry(path, data):
    validate_geometry(data)
    path = Path(path).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    original = path.read_bytes() if path.exists() else None
    if original is not None:
        backup = path.with_name(f'{path.stem}.backup-{time.time_ns()}{path.suffix}')
        backup.write_bytes(original)
    try:
        write_calibration(path, data)
        if load_geometry(path) != data:
            raise ValueError('target geometry readback differs from taught corners')
    except BaseException:
        if original is None:
            path.unlink(missing_ok=True)
        else:
            path.write_bytes(original)
        raise


def teach_target(project, path, *, controller_factory=None, confirm=None):
    from app.operator_input import confirm_enter
    from config.loader import runtime_robot_config
    from robot.rtde_controller import URRTDEController
    # connect() is Receive-only until activate_control(); never activate here.
    controller = (controller_factory or URRTDEController)(runtime_robot_config(project))
    confirm = confirm or confirm_enter
    points = {}
    print('目标轮廓只用于可视化：按周界顺序人工示教 P1、P2、P3、P4；只保存 Base XY。', flush=True)
    try:
        controller.connect()
        for i in range(1, 5):
            name = f'P{i}'
            if not confirm(f'在示教器人工移动并停稳于 {name} 后按 Enter 读取 Base XY'):
                raise KeyboardInterrupt('target teaching cancelled; existing calibration preserved')
            points[name] = controller.wait_for_standstill().pose[:2].tolist()
            print(f'{name} Base XY [m]：{points[name]}', flush=True)
        save_geometry(path, make_geometry(points))
    finally:
        controller.close()
    print(f'目标可视化标定已保存：{Path(path).resolve()}', flush=True)
