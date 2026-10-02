#!/usr/bin/env python3
"""Read-only PX6D health check. It never sends the hardware-zero command."""

from __future__ import annotations

import argparse
import json
import pathlib
import statistics
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from config.loader import load_config  # noqa: E402
from sensor.px6d_reader import PX6DError, PX6DReader  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(ROOT / "config.yaml"))
    parser.add_argument("--samples", type=int, default=500)
    parser.add_argument('--diagnostics-output', type=pathlib.Path,
                        help='write bounded host request diagnostics once, after closing the sensor')
    args = parser.parse_args()
    if args.samples <= 0:
        parser.error('--samples must be positive')
    config = load_config(args.config)
    sensor = config["sensor"]
    values = []
    reader = PX6DReader(
        sensor["serial_port"],
        sensor["baudrate"],
        sensor["timeout_sec"],
        sensor["poll_rate_hz"],
        sensor["startup_delay_sec"],
    )
    failure = None
    try:
        with reader:
            print(f"device={sensor['serial_port']} firmware={reader.firmware}")
            # Exclude USB CDC startup/firmware settling from the sample-rate result.
            started = time.monotonic()
            for _ in range(args.samples):
                values.append(reader.read_wrench().array())
            elapsed = time.monotonic() - started
    except PX6DError as exc:
        failure = f'{type(exc).__name__}: {exc}'
    finally:
        diagnostics = reader.diagnostics_snapshot()
        if args.diagnostics_output:
            args.diagnostics_output.write_text(json.dumps(dict(
                schema_version=1, sensor_config=sensor, firmware=reader.firmware,
                error=failure, samples_completed=len(values), sensor_diagnostics=diagnostics),
                ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    if failure:
        print(failure, file=sys.stderr)
        print(json.dumps(diagnostics, ensure_ascii=False), file=sys.stderr)
        return 1
    labels = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
    for index, label in enumerate(labels):
        column = [row[index] for row in values]
        print(f"{label}: mean={statistics.fmean(column):+.6f} std={statistics.pstdev(column):.6f} min={min(column):+.6f} max={max(column):+.6f}")
    print(f"rate={len(values) / elapsed:.2f} Hz samples={len(values)} crc_errors={diagnostics['crc_failures_total']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
