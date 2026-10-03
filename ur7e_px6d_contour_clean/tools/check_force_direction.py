#!/usr/bin/env python3
"""Read PX6D to inspect the configured Base transform/sign; never access UR."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from app.operator_input import OperatorKeyboard, confirm_enter
from config.loader import load_config
from sensor.force_preprocess import WrenchPreprocessor
from sensor.px6d_reader import PX6DReader
from sensor.force_direction import control_directions


def direction_values(processed):
    values = processed.array()
    if not np.isfinite(values).all():
        raise ValueError('nonfinite processed PX6D wrench')
    xy = values[:2]
    fxy = float(np.linalg.norm(xy))
    return fxy, xy/fxy if fxy > 0. else np.zeros(2)


def run(config_path=ROOT/'config.yaml', *, reader_factory=PX6DReader,
        poll=lambda: None, samples=None, bias_samples=None, display_hz=5., emit=print, confirm=None,
        tool_orientation=None):
    config = load_config(config_path)
    if samples is not None and samples <= 0:
        raise ValueError('samples must be positive')
    if not np.isfinite(display_hz) or display_hz <= 0:
        raise ValueError('display_hz must be finite and positive')
    if bias_samples is None:
        bias_samples = int(config['preprocessing']['baseline']['sample_count'])
    if bias_samples < 0:
        raise ValueError('bias_samples must be nonnegative')
    # This is an operator-supplied pose, never a claimed robot measurement.
    # Imports stay inside run so --help does not load robot interfaces.
    if tool_orientation is None:
        from calibration.scan_calibration import load_scan_calibration, resolve_calibration_path
        from calibration.probe_alignment import downward_probe_orientation
        saved = load_scan_calibration(resolve_calibration_path(config_path, config['calibration']['file']))
        tool_orientation = saved['fixed_orientation']
        if config['calibration'].get('vertical_probe', True):
            tool_orientation = downward_probe_orientation(tool_orientation,
                config['calibration'].get('probe_axis_tcp', [0, 0, 1]))
    processor = WrenchPreprocessor.from_config(config['preprocessing'], tool_orientation=tool_orientation)
    sensor = config['sensor']
    reader = reader_factory(sensor['serial_port'], sensor['baudrate'], sensor['timeout_sec'],
                            sensor['poll_rate_hz'], sensor['startup_delay_sec'])
    sign = config['continuous_tracking']['force_direction_sign']
    emit('PX6D only: no UR connection or robot motion; config is not modified.')
    emit(f'Assumed tool orientation (not measured by this tool): {np.asarray(tool_orientation).tolist()}')
    emit(f'Force transform: {processor.force_transform_status}')
    if not processor.force_transform_status['available']:
        emit('安装旋转未知：仅显示 sensor 分量，不能据此声称 Base 力或卸力方向。工具不修改配置。')
    else:
        emit(f'rotation_sensor_to_base = {processor.rotation_sensor_to_output.tolist()}')
    emit(f'force_direction_sign = {sign}')
    period = 1./float(sensor['poll_rate_hz'])
    def read():
        if poll() in ('Q', 'ESC'):
            raise KeyboardInterrupt
        raw = reader.read_wrench()
        if not np.isfinite(raw.array()).all():
            raise ValueError('nonfinite raw PX6D wrench')
        return raw
    try:
        if bias_samples and not confirm_enter('探针须脱离目标、静止空载。即将采集一次软件零偏。', read_line=confirm):
            raise KeyboardInterrupt
        reader.connect()
        if bias_samples:
            emit(f'Keep the probe unloaded and stationary at the displayed final orientation: capturing {bias_samples} bias samples.')
            baseline = []
            for _ in range(bias_samples):
                started = time.monotonic()
                baseline.append(read())
                time.sleep(max(0., period-(time.monotonic()-started)))
            processor.set_zero_bias(baseline)
            emit('Software bias ready (memory only).')
        else:
            emit('Software bias capture disabled; using configured preprocessing without a new zero.')
        emit('施加在探针上的力向量指向 Base +X / Base +Y（不是“从该侧推来”）。')
        emit('normalized is before force_direction_sign; configured n = sign * normalized. Q / Esc / Ctrl+C exits.')
        count, last_display = 0, None
        while samples is None or count < samples:
            started = time.monotonic()
            raw = read()
            processed = processor.process(raw)
            fxy, normal = direction_values(processed)
            count += 1
            if last_display is None or started-last_display >= 1./display_hz or count == samples:
                signed = sign*normal
                emit(f'sensor debiased N={(raw.force-processor.zero_bias_sensor[:3]).tolist()}  '
                     f'{processor.force_transform_status["output_frame"]} processed N={processed.force.tolist()}')
                if processor.force_transform_status['available']:
                    n, tangent, unload = control_directions(processed.force, sign, config['policy']['follow_hand'],
                        minimum_force=config['continuous_tracking']['direction_min_force'])
                    emit(f'unloading -n={unload.tolist()}  t={tangent.tolist()}')
                emit(f'processed Fx={processed.fx:.6f} N  processed Fy={processed.fy:.6f} N  '
                     f'Fxy={fxy:.6f} N  normalized=[{normal[0]:.6f}, {normal[1]:.6f}]  '
                     + (f'configured n=[{signed[0]:.6f}, {signed[1]:.6f}]' if processor.force_transform_status['available']
                      else 'configured n=unavailable (sensor -> Base mounting unknown)'))
                last_display = started
            time.sleep(max(0., period-(time.monotonic()-started)))
        return 0
    except KeyboardInterrupt:
        emit('Force direction check stopped by operator.')
        return 0
    except Exception as exc:
        emit(f'Force direction check failed: {type(exc).__name__}: {exc}')
        return 1
    finally:
        reader.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config.yaml')
    parser.add_argument('--samples', type=int, help='optional finite number of displayed-phase samples')
    parser.add_argument('--bias-samples', type=int, help='default: configured count; 0 skips software zero')
    parser.add_argument('--display-hz', type=float, default=5., help='terminal refresh rate (sensor rate unchanged)')
    parser.add_argument('--tool-orientation', type=float, nargs=3, metavar=('RX', 'RY', 'RZ'),
                        help='known actual TCP rotation vector [rad]; default is the derived final scan target, not a measurement')
    args = parser.parse_args()
    with OperatorKeyboard() as keyboard:
        return run(args.config, poll=keyboard.poll, samples=args.samples,
                   bias_samples=args.bias_samples, display_hz=args.display_hz, confirm=keyboard.read_line,
                   tool_orientation=args.tool_orientation)


if __name__ == '__main__':
    raise SystemExit(main())
