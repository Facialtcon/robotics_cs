#!/usr/bin/env python3
"""Run the ideal square experiment and save one XY result figure."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path



ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    print("离线验证 100 mm 正方形完整一圈；结果为一张 XY 轨迹图。", flush=True)
    return subprocess.run(
        [sys.executable, str(ROOT / "run_simulation.py"), "--no-gui"],
        cwd=ROOT, check=False,
    ).returncode


if __name__ == "__main__":
    raise SystemExit(main())
