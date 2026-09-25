#!/usr/bin/env python3
"""Read-only CLI for the existing workspace calibration validator."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from workspace.workspace_transform import DEFAULT_CALIBRATION_PATH, load_calibration


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration', type=Path, default=DEFAULT_CALIBRATION_PATH)
    args = parser.parse_args(argv)
    try:
        source = args.calibration.expanduser().resolve()
        calibration = load_calibration(source)
        print(f'箱体标定来源：{source}')
        print(json.dumps(calibration, ensure_ascii=False, indent=2))
        print('离线几何一致性检查通过；未连接设备，未验证现场或 active TCP，未写入文件。')
        return 0
    except (ValueError, OSError) as exc:
        print(f'箱体标定检查失败：{exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
