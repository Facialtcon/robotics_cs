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
from visualization.sandbox_scan import save_preview


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config.yaml')
    parser.add_argument('--calibration', type=Path, help='默认 calibration_data/sandbox_scan.yaml；与旧扫描标定分开保存')
    parser.add_argument('--recalibrate', action='store_true', help='重新人工标记 P0 和四角（只读机器人）')
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
            print('固定起点沙箱扫描\n1. 加载已有标定并开始\n2. 重新标记 P0 / 四角并开始\n3. 离线生成预览\n0. 退出')
            choice = input('请选择：').strip()
            if choice in ('0', 'q', 'esc'):
                return 0
            if choice not in ('1','2','3'):
                raise ValueError('请选择 0–3')
            args.recalibrate = choice == '2'
            args.execute = choice in ('1','2')
            args.preview = choice == '3'
        if args.preview and args.recalibrate:
            raise ValueError('--preview 为纯离线模式；重新标记请单独使用 --recalibrate')
        if args.recalibrate:
            teach(project, args.calibration)
        if not args.execute and not args.preview:
            # --recalibrate alone teaches only; other flags do not imply motion.
            return 0 if args.recalibrate else parser.error('请选择 --preview 或 --execute')
        record = read_calibration(project, args.config, args.calibration)
        if record.get('imported_sources'):
            print('尚未保存独立沙箱标定；直接复用已有已确认扫描 P0、扫描高度及箱体四角，不修改原文件。', flush=True)
        plan = make_plan(project, record)
        # Full shared execution preflight is also offline. It preserves original
        # motion boundaries and reports conflicts before opening any devices.
        prepare_sandbox(project, args.config, args.calibration, expected_plan=plan)
        print(json.dumps(plan, ensure_ascii=False, indent=2), flush=True)
        print(f'原扫描标定高度与 P0 示教高度分别保留；搜索速度 {plan["search_speed_mps"]*1000:g} mm/s。')
        preview = save_preview(plan, args.preview_dir, show=args.execute)
        print(f'轨迹预览：{preview.resolve()}\n完整规划：{(args.preview_dir/"sandbox_scan_plan.json").resolve()}', flush=True)
        if args.preview:
            print('离线预览完成；未连接设备。')
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
