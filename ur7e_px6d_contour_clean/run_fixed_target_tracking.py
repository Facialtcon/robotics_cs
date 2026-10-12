#!/usr/bin/env python3
"""Fixed cube continuous tracking, using the existing continuous runtime."""
import argparse
import sys
from pathlib import Path

from app.fixed_target import ROOT, run_fixed, check_calibration, menu


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config.yaml')
    parser.add_argument('--calibration', type=Path, default=ROOT/'single_point_calibration.yaml',
                        help='existing saved P0/P_ref calibration')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true', help='real hardware, using the original config.yaml safety interfaces')
    mode.add_argument('--dry-run', action='store_true', help='offline synthetic square demonstration')
    parser.add_argument('--p0', help='saved P0 group, e.g. A, B, C')
    parser.add_argument('--scene', type=Path, default=ROOT/'simulation/scene_fixed_target.yaml')
    parser.add_argument('--duration', type=float, help='offline duration only; real tracking retains original stopping rules')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--target-geometry', type=Path, default=ROOT/'target_geometry_calibration.yaml',
                        help='offline visualization only; never read by control')
    parser.add_argument('--calibrate-target', action='store_true', help='read-only teach P1/P2/P3/P4 Base XY for visualization')
    parser.add_argument('--check-calibration', action='store_true', help='offline P0/P_ref and fixed workspace validation')
    args = parser.parse_args(argv)
    if not argv:
        return menu(args)
    if args.calibrate_target:
        from calibration.target_geometry import teach_target
        from config.loader import load_config
        teach_target(load_config(args.config), args.target_geometry)
        return 0
    if args.check_calibration:
        return check_calibration(args)
    return run_fixed(args)


if __name__ == '__main__':
    raise SystemExit(main())
