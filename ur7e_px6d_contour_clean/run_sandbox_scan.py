#!/usr/bin/env python3
"""固定起点、全沙箱覆盖搜索；共用原连续跟踪运行器。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from app.sandbox_scan import ROOT, calibration_path, make_plan, prepare_sandbox, run_sandbox, read_calibration
from calibration.sandbox_scan import teach
from config.loader import load_config
from visualization.sandbox_scan import save_preview, describe_route


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config.yaml')
    parser.add_argument('--calibration', type=Path, help='默认 calibration_data/sandbox_scan.yaml；与旧扫描标定分开保存')
    calibration = parser.add_mutually_exclusive_group()
    calibration.add_argument('--recalibrate', action='store_true', help='重新标定全部，保存并更新预览（只读机器人）')
    calibration.add_argument('--recalibrate-p0', action='store_true', help='仅重标 P0，保留四角和扫描高度，保存并预览')
    calibration.add_argument('--recalibrate-corners', action='store_true', help='仅重标四角，保留 P0 和扫描高度，保存并预览')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--preview', action='store_true', help='离线加载、规划、保存预览，不连接设备')
    mode.add_argument('--execute', action='store_true', help='加载标定、预览后沿原 Enter 确认流程执行真机搜索')
    parser.add_argument('--output', type=Path, help='沿用原日志目录规则')
    parser.add_argument('--preview-dir', type=Path, default=ROOT/'data/sandbox_preview')
    args = parser.parse_args(argv)
    try:
        project = load_config(args.config)
        args.calibration = calibration_path(project, args.config, args.calibration)
        if not argv:
            print('固定起点沙箱扫描\n1. 加载已有标定并开始\n2. 仅重新标定 P0，保存并预览\n'
                  '3. 仅重新标定沙箱四角，保存并预览\n4. 重新标定全部，保存并预览\n5. 离线生成预览\n0. 退出')
            choice = input('请选择：').strip()
            if choice in ('0', 'q', 'esc'):
                return 0
            if choice not in ('1','2','3','4','5'):
                raise ValueError('请选择 0–5')
            args.recalibrate_p0 = choice == '2'
            args.recalibrate_corners = choice == '3'
            args.recalibrate = choice == '4'
            args.execute = choice == '1'
            args.preview = choice == '5'
        scope = 'p0' if args.recalibrate_p0 else 'corners' if args.recalibrate_corners else 'all' if args.recalibrate else None
        if args.preview and scope:
            raise ValueError('--preview 为纯离线模式；请单独使用所选的重新标定选项，保存后会自动更新预览')
        if scope:
            existing = (read_calibration(project, args.config, args.calibration)
                        if scope != 'all' and not args.calibration.exists() else None)
            teach(project, args.calibration, scope=scope, existing=existing)
        if not args.execute and not args.preview and scope is None:
            parser.error('请选择 --preview、--execute 或一项重新标定操作')
        record = read_calibration(project, args.config, args.calibration)
        if record.get('imported_sources'):
            print('尚未保存独立沙箱标定；直接复用已有已确认扫描 P0、扫描高度及箱体四角，不修改原文件。', flush=True)
        plan = make_plan(project, record)
        # Full shared execution preflight is also offline. It preserves original
        # motion boundaries and reports conflicts before opening any devices.
        prepare_sandbox(project, args.config, args.calibration, expected_plan=plan)
        print(json.dumps(plan, ensure_ascii=False, indent=2), flush=True)
        print(f'原扫描标定高度与 P0 示教高度分别保留；搜索速度 {plan["search_speed_mps"]*1000:g} mm/s。')
        info = describe_route(plan)
        print(info['summary']+'\nP0 位置：'+info['p0_location_text'], flush=True)
        print(' → '.join(step['label'] for step in info['first_steps']), flush=True)
        preview = save_preview(plan, args.preview_dir, show=bool(args.execute or scope))
        print(f'轨迹预览：{preview.resolve()}\n完整规划：{(args.preview_dir/"sandbox_scan_plan.json").resolve()}', flush=True)
        if not args.execute:
            print('标定已保存并更新预览；未启动扫描运动。' if scope else '离线预览完成；未连接设备。')
            return 0
        print('核对预览；接下来的原启动流程按 Enter 确认路径、空气采零与下降，保留原速度倍率选择。', flush=True)
        # Optional fields expected by the unchanged common runner.
        args.duration = None
        args.enable_reacquire = False
        args.preview = False
        return run_sandbox(args, plan)
    except (KeyboardInterrupt, EOFError):
        print('已取消；未请求新的运动。')
        return 130
    except (ValueError, KeyError, OSError, RuntimeError) as exc:
        print(f'沙箱扫描无法启动：{type(exc).__name__}: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
