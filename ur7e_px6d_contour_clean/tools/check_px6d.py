#!/usr/bin/env python3
"""Read-only PX6D health check. It never sends the hardware-zero command."""

from __future__ import annotations

import argparse
import pathlib
import statistics
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from config.loader import load_config  # noqa: E402
from sensor.px6d_reader import PX6DReader  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(ROOT / "config.yaml"))
    parser.add_argument("--samples", type=int, default=500)
    args = parser.parse_args()
    config = load_config(args.config)
    sensor = config["sensor"]
    values = []
    with PX6DReader(
        sensor["serial_port"],
        sensor["baudrate"],
        sensor["timeout_sec"],
        sensor["poll_rate_hz"],
        sensor["startup_delay_sec"],
    ) as reader:
        print(f"device={sensor['serial_port']} firmware={reader.firmware}")
        # Exclude USB CDC startup/firmware settling from the sample-rate result.
        started = time.monotonic()
        for _ in range(args.samples):
            values.append(reader.read_wrench().array())
        elapsed = time.monotonic() - started
    labels = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
    for index, label in enumerate(labels):
        column = [row[index] for row in values]
        print(f"{label}: mean={statistics.fmean(column):+.6f} std={statistics.pstdev(column):.6f} min={min(column):+.6f} max={max(column):+.6f}")
    print(f"rate={len(values) / elapsed:.2f} Hz samples={len(values)} crc_errors=0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
