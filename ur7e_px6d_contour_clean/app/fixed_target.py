"""Fixed-target scene adapter; no target-truth or single-point trial dependency."""
from copy import deepcopy
from pathlib import Path

import numpy as np
import yaml

from app.configuration import prepare_real, workspace_calibration_path, load_workspace_calibration
from calibration.scan_calibration import calculate_scan_direction_xy
from calibration.single_point import vector
from config.loader import load_config
from robot.rtde_controller import RobotError
from robot.tcp_identity import tcp_offsets_match

ROOT = Path(__file__).resolve().parents[1]


def read_starts(calibration_path):
    path = Path(calibration_path).resolve()
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict) or data.get('schema_version') != 1 or not data.get('groups'):
        raise ValueError('fixed-target P0/P_ref calibration requires saved groups')
    return data, path


def prepare_fixed(config, config_path, calibration_path, p0):
    """Replace only scene geometry before the original real-execution preflight."""
    data, path = read_starts(calibration_path)
    reference_xyz = vector(data['P_ref'], 3, 'P_ref')
    if not np.allclose(reference_xyz, vector(data['reference_tcp_pose'][:3], 3, 'P_ref XYZ'), atol=1e-12, rtol=0):
        raise ValueError('P_ref differs from its saved reference pose')
    group = data['groups'][p0]
    measured = vector(group['tcp_pose'], 6, 'P0 pose')
    if not tcp_offsets_match(group['active_tcp_offset'], data['active_tcp_offset'],
                             config['tcp']['offset_tolerance']):
        raise RobotError('P0 TCP differs from saved P_ref TCP')
    # P_ref supplies direction and common Z only. Its posture is ignored.
    # The original preflight derives the downward posture from the taught P0.
    start = measured.copy()
    start[2] = reference_xyz[2]
    reference = np.r_[reference_xyz, measured[3:]]
    direction = calculate_scan_direction_xy(start, reference)
    # Reuse the main experiment's measured workspace, including its metadata.
    # Target truth remains exclusively an offline visualization input.
    workspace_path = workspace_calibration_path(config, config_path)
    region = load_workspace_calibration(workspace_path)
    scan = dict(robot_ip=data['robot_ip'], active_tcp_offset=deepcopy(data['active_tcp_offset']),
                start_tcp_pose=start.tolist(), direction_reference_tcp_pose=reference.tolist(),
                scan_direction_xy=direction.tolist(), fixed_z=float(reference[2]),
                fixed_orientation=reference[3:].tolist())
    config['fixed_target_start'] = dict(p0=p0, measured_tcp_pose=measured.tolist(),
                                      P_ref=reference_xyz.tolist(), reference_is_endpoint=False)
    return prepare_real(config, config_path, scene_calibration=dict(scan=scan, workspace=region,
        sources=dict(scan=str(path), workspace=str(workspace_path.resolve()))))


def configure_fixed(config):
    config['fixed_target_experiment'] = True
    config['experiment'] = dict(kind='fixed_target_tracking', title='固定目标连续跟踪')
    config['preprocessing'].setdefault('granular_baseline', {})['capture_on_start'] = False
    config['preprocessing']['granular_baseline_output'] = [0.] * 6
    print('固定目标连续跟踪；最终继承配置：\n' +
          yaml.safe_dump({key: config[key] for key in ('robot', 'sensor', 'tcp', 'preprocessing',
              'policy', 'continuous_tracking', 'workspace', 'safe_return', 'logging')},
              allow_unicode=True, sort_keys=False), flush=True)


def run_fixed(args, *, controller_factory=None, reader_factory=None):
    from app.continuous_runtime import run
    from visualization.fixed_target import generate_fixed_visualization
    if args.execute and args.p0 is None:
        data, _ = read_starts(args.calibration)
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
    options = dict(configure_experiment=configure,
        prepare_execution=lambda c, p: prepare_fixed(c, p, args.calibration, args.p0),
        visualize_run=lambda path: generate_fixed_visualization(path, args.target_geometry if args.execute else None))
    if controller_factory is not None:
        options['controller_factory'] = controller_factory
    if reader_factory is not None:
        options['reader_factory'] = reader_factory
    return run(args, **options)


def check_calibration(args):
    data, _ = read_starts(args.calibration)
    names = [args.p0] if args.p0 else list(data['groups'])
    for name in names:
        config = load_config(args.config)
        configure_fixed(config)
        _, start = prepare_fixed(config, args.config, args.calibration, name)
        print(yaml.safe_dump(dict(p0=name, start=start.tolist(),
            direction=config['policy']['search_direction_xy'],
            workspace=config['workspace'], execution_envelope=config['continuous_execution_envelope']),
            allow_unicode=True, sort_keys=False))
    return 0


def _menu_text(prompt):
    answer = input(prompt).strip()
    if answer.lower() in ('0', 'q', 'esc'):
        raise KeyboardInterrupt('返回主菜单')
    return answer


def _select_p0(data):
    names = list(data['groups'])
    if not names:
        raise ValueError('暂无 P0，请先标定 P_ref 并新增 P0')
    for i, name in enumerate(names, 1):
        print(f'{i}. P0-{name}')
    answer = _menu_text('选择 P0 序号或名称（0 返回）：')
    if answer in names:
        return answer
    if answer.isdigit() and 1 <= int(answer) <= len(names):
        return names[int(answer)-1]
    raise ValueError('请选择已有 P0')


def menu(args):
    """Chinese operation entry; hardware and motion remain in existing functions."""
    while True:
        print('\n========== 固定目标连续跟踪实验 ==========\n'
              '1. 开始真机实验\n2. 标定 P_ref\n3. 新增 / 修改 / 删除 P0\n'
              '4. 查看已有标定\n5. 标定正方体四角点\n6. 检查标定与工作区域\n'
              '7. 离线模拟\n0. 退出')
        try:
            choice = input('请选择：').strip()
        except (KeyboardInterrupt, EOFError):
            return 0
        if choice == '0':
            return 0
        try:
            if choice not in tuple('1234567'):
                print('请输入 0–7。')
                continue
            operation = deepcopy(args)
            if choice == '7':
                operation.execute = False
                run_fixed(operation)
                continue
            if choice == '6':
                operation.p0 = None
                check_calibration(operation)
                continue
            project = load_config(args.config)
            if choice == '5':
                from calibration.target_geometry import teach_target
                teach_target(project, args.target_geometry)
                continue
            from calibration.single_point import load_calibration
            data = (load_calibration(args.calibration) if args.calibration.exists() else
                    dict(schema_version=1, P_ref=None, groups={}))
            if choice == '1':
                operation.p0 = _select_p0(data)
                operation.execute = True
                run_fixed(operation)
                continue
            # Reuse the single-point calibration UI, capture and checked writes.
            # Its trial settings never enter run_fixed or the continuous policy.
            import run_single_point_contact as single
            settings = single.load_settings(ROOT/'single_point_experiment.yaml', project)
            if choice == '4':
                single.show_calibration_table(data, settings)
            elif choice == '2':
                single.calibrate_point(operation, project, settings, data, 'P_ref', reference=True)
            else:
                print('1. 新增 P0\n2. 修改 P0\n3. 删除 P0\n0. 返回')
                action = _menu_text('请选择 P0 操作：')
                if action == '1':
                    name = _menu_text('P0 名称（如 A/B/C；0 返回）：')
                    if not name or name in data['groups']:
                        raise ValueError('名称为空或 P0 已存在，请使用修改操作')
                    single.calibrate_point(operation, project, settings, data, name)
                elif action in ('2', '3'):
                    name = _select_p0(data)
                    if action == '2':
                        single.calibrate_point(operation, project, settings, data, name)
                    elif single.confirm_enter(f'删除 P0-{name}'):
                        del data['groups'][name]
                        single.save_calibration_checked(args.calibration, data, f'删除 P0-{name}')
                else:
                    raise ValueError('请选择 0–3')
        except (KeyboardInterrupt, EOFError) as exc:
            print(f'操作已取消：{exc}')
        except Exception as exc:
            print(f'操作失败：{exc}')
