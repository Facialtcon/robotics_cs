#!/usr/bin/env python3
"""Read-only PX6D health check. It never sends the hardware-zero command."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import pathlib
import statistics
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from config.loader import load_config  # noqa: E402
from sensor.px6d_reader import (PX6DError, PX6DReader, HEADER, CMD_STREAM, CMD_GET_VERSION,
                               WRENCH_PACKET_SIZE, VERSION_PACKET_SIZE, crc8)  # noqa: E402


def observe_after_failure(reader, duration_sec=1.0):
    """Passive diagnostic ONLY: no requests, retries, bias or usable wrench.

    Called only by this standalone tool after a failed exchange, before close.
    The runtime reader never waits here and its failure latch remains set.
    """
    started = previous = time.monotonic()
    port = reader._port
    evidence = bytearray(reader._buffer[:4096])
    result = dict(time_source='host monotonic; not sensor sampling time',
                  started_host_monotonic=started, observation_budget_sec=duration_sec,
                  request_id=reader._request_id, tx_bytes=0, rx_bytes=0,
                  buffered_bytes_before=len(reader._buffer), first_rx_host_monotonic=None,
                  last_rx_host_monotonic=None, max_loop_gap_sec=0., valid_packets=[], error=None)
    try:
        while True:
            now = time.monotonic()
            result['max_loop_gap_sec'] = max(result['max_loop_gap_sec'], now-previous)
            previous = now
            if now-started >= duration_sec:
                break
            waiting = port.in_waiting
            if waiting:
                data = port.read(min(waiting, 4096))
                received = time.monotonic()
                result['rx_bytes'] += len(data)
                evidence.extend(data[:max(0, 4096-len(evidence))])
                if data:
                    if result['first_rx_host_monotonic'] is None:
                        result['first_rx_host_monotonic'] = received
                    result['last_rx_host_monotonic'] = received
            time.sleep(min(.001, max(0., started+duration_sec-time.monotonic())))
        result['serial_bytes_at_end'] = port.in_waiting
        result['serial_output_bytes_at_end'] = port.out_waiting
    except Exception as exc:
        result['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        result['elapsed_sec'] = time.monotonic()-started
        result['captured_prefix_hex'] = bytes(evidence[:256]).hex()
    # Inspect a bounded evidence prefix after the passive wait. No packet is
    # handed to read_wrench or allowed to authorize a subsequent request.
    for offset in range(max(0, len(evidence)-3)):
        if evidence[offset:offset+2] != HEADER:
            continue
        command = evidence[offset+3]
        size = {CMD_STREAM: WRENCH_PACKET_SIZE, CMD_GET_VERSION: VERSION_PACKET_SIZE}.get(command)
        if size is None:
            continue
        candidate = bytes(evidence[offset:offset+size])
        if len(candidate) == size and crc8(candidate[:-1]) == candidate[-1]:
            result['valid_packets'].append(dict(offset=offset, command=command,
                device_id=candidate[2], packet_hex=candidate.hex()))
    return result


class DiagnosticPX6DReader(PX6DReader):
    """Standalone capture extends evidence after failure, never the request deadline."""
    passive_observation = None

    def close(self):
        try:
            if self._port is not None and self._requires_resynchronization and self.passive_observation is None:
                self.passive_observation = observe_after_failure(self)
        finally:
            super().close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(ROOT / "config.yaml"))
    parser.add_argument("--samples", type=int, default=None)
    parser.add_argument('--diagnostics-output', type=pathlib.Path,
                        help='write bounded host request diagnostics once, after closing the sensor')
    parser.add_argument('--comparison-dir', type=pathlib.Path,
                        help='save independent software comparison, raw bytes and USB evidence together')
    parser.add_argument('--system-only', action='store_true',
                        help='inspect USB/ownership/permissions without opening the sensor or robot')
    parser.add_argument('--campaign', action='store_true',
                        help='Enter-guided original / known-good / original cable comparisons, same USB port')
    parser.add_argument('--duration', type=float, default=30., help='comparison collection seconds per round (max 300)')
    parser.add_argument('--repeats', type=int, default=2, help='comparison repeats per implementation (1 to 5)')
    parser.add_argument('--poll-rate-hz', type=float, help='diagnostic-only rate override; never saves scan config')
    args = parser.parse_args()
    if args.samples is not None and args.samples <= 0:
        parser.error('--samples must be positive')
    if (args.system_only or args.campaign or args.poll_rate_hz is not None) and not args.comparison_dir:
        parser.error('--system-only/--campaign/--poll-rate-hz require --comparison-dir')
    if not math.isfinite(args.duration) or not 0 < args.duration <= 300 or not 1 <= args.repeats <= 5:
        parser.error('comparison duration must be in (0, 300] seconds and repeats in [1, 5]')
    if args.poll_rate_hz is not None and (not math.isfinite(args.poll_rate_hz) or args.poll_rate_hz <= 0):
        parser.error('--poll-rate-hz must be positive and finite')
    config = load_config(args.config)
    sensor = config["sensor"]
    if args.comparison_dir:
        if args.diagnostics_output:
            parser.error('--comparison-dir already contains results; do not combine --diagnostics-output')
        from tools.px6d_comparison import comparison_main
        try:
            return comparison_main(args, sensor)
        except (ValueError, EOFError) as exc:
            print(str(exc), file=sys.stderr)
            return 2
        except KeyboardInterrupt:
            return 130
    args.samples = 500 if args.samples is None else args.samples
    values = []
    reader = DiagnosticPX6DReader(
        sensor["serial_port"],
        sensor["baudrate"],
        sensor["timeout_sec"],
        sensor["poll_rate_hz"],
        sensor["startup_delay_sec"],
    )
    failure = None
    interrupted = False
    started = None
    elapsed = 0.
    read_durations = []
    invalid_sample = None
    started_utc = datetime.now(timezone.utc).isoformat()
    try:
        with reader:
            print(f"device={sensor['serial_port']} firmware={reader.firmware}")
            # Exclude USB CDC startup/firmware settling from the sample-rate result.
            started = time.monotonic()
            try:
                for _ in range(args.samples):
                    read_started = time.monotonic()
                    try:
                        value = reader.read_wrench().array()
                        if not all(math.isfinite(component) for component in value):
                            invalid_sample = [repr(float(component)) for component in value]
                            raise PX6DError('nonfinite PX6D wrench; collection stopped')
                        values.append(value)
                    finally:
                        read_durations.append(time.monotonic()-read_started)
            finally:
                elapsed = time.monotonic()-started  # Exclude close/passive observation.
    except PX6DError as exc:
        failure = f'{type(exc).__name__}: {exc}'
    except KeyboardInterrupt:
        interrupted = True
        failure = 'KeyboardInterrupt: operator stopped collection'
    except Exception as exc:
        failure = f'{type(exc).__name__}: {exc}'
    finally:
        try:
            reader.close()  # Also covers an interrupt while __enter__/connect is running.
        except Exception as exc:
            failure = (failure+'; ' if failure else '')+f'close failed: {type(exc).__name__}: {exc}'
        diagnostics = reader.diagnostics_snapshot()
        stats = {label: dict(mean=statistics.fmean(row[index] for row in values),
                            std=statistics.pstdev(row[index] for row in values),
                            min=min(row[index] for row in values), max=max(row[index] for row in values))
                 for index, label in enumerate(('Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz'))} if values else {}
        timing = dict(count=len(read_durations),
                      total_sec=sum(read_durations),
                      max_sec=max(read_durations, default=0.),
                      mean_sec=statistics.fmean(read_durations) if read_durations else 0.)
        if args.diagnostics_output:
            args.diagnostics_output.parent.mkdir(parents=True, exist_ok=True)
            args.diagnostics_output.write_text(json.dumps(dict(
                schema_version=2, started_utc=started_utc, finished_utc=datetime.now(timezone.utc).isoformat(),
                sensor_config=sensor, firmware=reader.firmware, requested_samples=args.samples,
                error=failure, samples_completed=len(values), collection_elapsed_sec=elapsed,
                read_call_timing=timing, wrench_statistics=stats, invalid_sample=invalid_sample,
                passive_after_failure=getattr(reader, 'passive_observation', None),
                sensor_diagnostics=diagnostics),
                ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    if failure:
        print(failure, file=sys.stderr)
        print(json.dumps(diagnostics, ensure_ascii=False), file=sys.stderr)
        return 130 if interrupted else 1
    labels = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
    for index, label in enumerate(labels):
        column = [row[index] for row in values]
        print(f"{label}: mean={statistics.fmean(column):+.6f} std={statistics.pstdev(column):.6f} min={min(column):+.6f} max={max(column):+.6f}")
    print(f"rate={len(values) / elapsed:.2f} Hz samples={len(values)} crc_errors={diagnostics['crc_failures_total']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
