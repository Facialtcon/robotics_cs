#!/usr/bin/env python3
"""Read PX6D to inspect the configured Base transform/sign; never access UR."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import getpass
import json
from pathlib import Path
import sys
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from app.operator_input import OperatorKeyboard
from config.loader import load_config
from sensor.force_preprocess import WrenchPreprocessor
from sensor.px6d_reader import PX6DReader
from sensor.force_direction import (AXES, control_directions, direction_binding,
    evaluate_direction_measurements, load_direction_verification, verification_path)


def direction_values(processed):
    values = processed.array()
    if not np.isfinite(values).all():
        raise ValueError('nonfinite processed PX6D wrench')
    xy = values[:2]
    fxy = float(np.linalg.norm(xy))
    return fxy, xy/fxy if fxy > 0. else np.zeros(2)


def run(config_path=ROOT/'config.yaml', *, reader_factory=PX6DReader,
        poll=lambda: None, samples=None, bias_samples=None, display_hz=5., emit=print, confirm=input):
    config = load_config(config_path)
    if samples is not None and samples <= 0:
        raise ValueError('samples must be positive')
    if not np.isfinite(display_hz) or display_hz <= 0:
        raise ValueError('display_hz must be finite and positive')
    if bias_samples is None:
        bias_samples = int(config['preprocessing']['baseline']['sample_count'])
    if bias_samples < 0:
        raise ValueError('bias_samples must be nonnegative')
    processor = WrenchPreprocessor.from_config(config['preprocessing'])
    sensor = config['sensor']
    reader = reader_factory(sensor['serial_port'], sensor['baudrate'], sensor['timeout_sec'],
                            sensor['poll_rate_hz'], sensor['startup_delay_sec'])
    sign = config['continuous_tracking']['force_direction_sign']
    emit('PX6D only: no UR connection or robot motion; config is not modified.')
    emit('Values use the configured rotation_sensor_to_base; confirm the actual Base axes on site.')
    emit(f'rotation_sensor_to_base = {processor.rotation_sensor_to_output.tolist()}')
    emit(f'force_direction_sign = {sign}')
    emit('verification = '+json.dumps(load_direction_verification(config, config_path), ensure_ascii=False))
    period = 1./float(sensor['poll_rate_hz'])
    def read():
        if poll() in ('Q', 'ESC'):
            raise KeyboardInterrupt
        raw = reader.read_wrench()
        if not np.isfinite(raw.array()).all():
            raise ValueError('nonfinite raw PX6D wrench')
        return raw
    try:
        if bias_samples and confirm('探针必须脱离目标、静止空载且处于保存的扫描姿态。输入 UNLOADED 才采一次零偏：').strip() != 'UNLOADED':
            raise KeyboardInterrupt
        reader.connect()
        if bias_samples:
            emit(f'Keep the probe unloaded and stationary at the saved scan orientation: capturing {bias_samples} bias samples.')
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
                n, tangent, unload = control_directions(processed.force, sign, config['policy']['follow_hand'],
                    minimum_force=config['continuous_tracking']['direction_min_force'])
                emit(f'sensor debiased N={(raw.force-processor.zero_bias_sensor[:3]).tolist()}  '
                     f'Base processed N={processed.force.tolist()}  unloading -n={unload.tolist()}  t={tangent.tolist()}')
                emit(f'processed Fx={processed.fx:.6f} N  processed Fy={processed.fy:.6f} N  '
                     f'Fxy={fxy:.6f} N  normalized=[{normal[0]:.6f}, {normal[1]:.6f}]  '
                     f'configured n=[{signed[0]:.6f}, {signed[1]:.6f}]')
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


def verify(config_path=ROOT/'config.yaml', *, output=None, operator=None, reader_factory=PX6DReader,
           confirm=input, emit=print, window_samples=50):
    """One stationary session, one initial bias, six applied-force checks; no UR imports."""
    config = load_config(config_path)
    processor = WrenchPreprocessor.from_config(config['preprocessing'])
    destination = Path(output) if output is not None else verification_path(config, config_path)
    record = dict(schema_version=1, timestamp_utc=datetime.now(timezone.utc).isoformat(),
        operator=operator or getpass.getuser(), configuration_sha256=direction_binding(config, config_path),
        unloaded_bias_confirmed=False, completed=False, measurements={},
        time_source='host monotonic; sensor sample timestamp unavailable')
    sensor = config['sensor']
    reader = reader_factory(sensor['serial_port'], sensor['baudrate'], sensor['timeout_sec'],
                            sensor['poll_rate_hz'], sensor['startup_delay_sec'])
    def capture(count):
        rows = []
        for _ in range(count):
            started = time.monotonic()
            raw = reader.read_wrench()
            if (not np.isfinite(raw.array()).all() or np.linalg.norm(raw.force) >= config['policy']['absolute_raw_force_threshold']
                    or np.linalg.norm(raw.array()[3:]) >= config['policy']['absolute_raw_torque_threshold']):
                raise ValueError('invalid/raw force limit during direction verification')
            rows.append(raw)
            time.sleep(max(0., 1./sensor['poll_rate_hz']-(time.monotonic()-started)))
        return rows
    result = 1
    try:
        emit('仅连接 PX6D；不连接机器人、不运动、不修改旋转/符号。验证配置值，不推算安装旋转。')
        emit('当前状态：'+json.dumps(load_direction_verification(config, config_path), ensure_ascii=False))
        scan_path = Path(config['calibration']['file'])
        if not scan_path.is_absolute():
            scan_path = Path(config_path).resolve().parent/scan_path
        orientation = yaml.safe_load(scan_path.read_text()).get('fixed_orientation')
        record['saved_scan_orientation_rotvec_rad'] = orientation
        emit(f'请在示教器核对保存的扫描姿态（旋转向量 rad）：{orientation}')
        if confirm('探针脱离目标、静止空载，保持保存的扫描姿态；输入 UNLOADED 开始唯一一次采零：').strip() != 'UNLOADED':
            raise KeyboardInterrupt
        record['unloaded_bias_confirmed'] = True
        reader.connect()
        baseline = capture(int(config['preprocessing']['baseline']['sample_count']))
        processor.set_zero_bias(baseline)
        baseline_std = np.std([w.force for w in baseline], axis=0)
        record.update(zero_bias_sensor=processor.zero_bias_sensor.tolist(), baseline_std_N=baseline_std.tolist())
        if np.linalg.norm(baseline_std) > .15:
            raise ValueError('unloaded bias was not stable; stop and inspect, no automatic re-zero')
        emit('采零已结束。下面只施加 0.5～3 N 小力，随时 Ctrl+C 退出；过程中没有重新采零入口。')
        for label in AXES:
            if confirm(f'先释放上一次力。施加在探针上的力向量指向 Base {label}，保持稳定，输入 {label} 采样：').strip() != label:
                raise KeyboardInterrupt
            started = time.monotonic()
            samples = capture(window_samples)
            corrected = np.asarray([w.force for w in samples])-processor.zero_bias_sensor[:3]
            record['measurements'][label] = dict(sample_count=len(samples),
                sensor_debiased_mean_N=np.mean(corrected, axis=0).tolist(),
                sensor_std_N=np.std(corrected, axis=0).tolist(),
                start_host_monotonic=started, end_host_monotonic=time.monotonic())
            processed = [processor.process(w) for w in samples][-1]
            n, tangent, unload = control_directions(processed.force,
                config['continuous_tracking']['force_direction_sign'], config['policy']['follow_hand'],
                minimum_force=config['continuous_tracking']['direction_min_force'])
            assessment = evaluate_direction_measurements(config, record['measurements'])['axes'][label]
            record['measurements'][label].update(processed_base_N=processed.force.tolist(),
                n=n.tolist(), tangent=tangent.tolist(), unloading_direction=unload.tolist(), **assessment)
            emit(f'{label}: sensor 去偏置均值={np.mean(corrected, axis=0).tolist()} N; '
                 f'Base 均值={assessment["base_reported_force_N"]} N; '
                 f'n={n.tolist()}; t={tangent.tolist()}; 卸力 -n={unload.tolist()}; '
                 f'误差={assessment["angle_error_deg"]:.1f}°; passed={assessment["passed"]}')
            if not np.any(n):
                emit('平面力不足，n/t/卸力方向不可用；零向量表示不可用，不是运动指令。')
        record['assessment'] = evaluate_direction_measurements(config, record['measurements'])
        record['completed'] = True
        result = 0 if record['assessment']['verified'] else 1
        emit('验证通过；记录仅适用于当前传感器安装、TCP 和保存姿态。' if result == 0 else
             '验证未通过：保留测量结果，核对安装旋转和受力定义；不会自动翻转符号或改配置。')
    except (Exception, KeyboardInterrupt) as exc:
        record['error'] = f'{type(exc).__name__}: {exc}'
        emit(record['error'])
    finally:
        try:
            reader.close()
        except Exception as exc:
            record['cleanup_error'] = f'{type(exc).__name__}: {exc}'
            result = 1
        record['sensor_diagnostics'] = reader.diagnostics_snapshot() if hasattr(reader, 'diagnostics_snapshot') else {}
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        emit(f'验证记录：{destination}')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config.yaml')
    parser.add_argument('--samples', type=int, help='optional finite number of displayed-phase samples')
    parser.add_argument('--bias-samples', type=int, help='default: configured count; 0 skips software zero')
    parser.add_argument('--display-hz', type=float, default=5., help='terminal refresh rate (sensor rate unchanged)')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--verify', action='store_true', help='record six known applied force directions, one unloaded bias')
    mode.add_argument('--status', action='store_true', help='offline verification/config binding check; no devices')
    parser.add_argument('--output', type=Path, help='verification JSON, default: configured verification path')
    parser.add_argument('--operator', help='operator identity in verification record')
    args = parser.parse_args()
    if args.status:
        result = load_direction_verification(load_config(args.config), args.config)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result['verified'] else 1
    if args.verify:
        if args.bias_samples is not None or args.samples is not None:
            parser.error('--verify uses one configured bias window and 50 samples per axis; --bias-samples/--samples are monitor options')
        return verify(args.config, output=args.output, operator=args.operator)
    with OperatorKeyboard() as keyboard:
        return run(args.config, poll=keyboard.poll, samples=args.samples,
                   bias_samples=args.bias_samples, display_hz=args.display_hz, confirm=keyboard.read_line)


if __name__ == '__main__':
    raise SystemExit(main())
