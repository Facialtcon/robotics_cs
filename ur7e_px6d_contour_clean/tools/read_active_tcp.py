#!/usr/bin/env python3
"""Read the UR controller's active TCP offset without sending motion commands."""

from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import os
import sys
import tempfile
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config.loader import load_config
from app.operator_input import confirm_enter
from robot.tcp_identity import read_tcp_offset_readonly, normalize_tcp_offset, tcp_offsets_match


def validate_tcp_offset(values) -> list[float]:
    return normalize_tcp_offset(values, "active TCP offset")


def save_tcp_offset(path: Path, original: bytes, offset: list[float]) -> Path | None:
    """Change only the six offset values, preserving other YAML and a byte-exact backup."""
    offset = validate_tcp_offset(offset)
    text = original.decode("utf-8")
    data = yaml.safe_load(text)
    node = yaml.compose(text)

    def field(mapping, name):
        if not isinstance(mapping, yaml.MappingNode):
            raise ValueError(f"{name}: expected YAML mapping")
        matches = [value for key, value in mapping.value if key.value == name]
        if len(matches) != 1:
            raise ValueError(f"{name}: expected exactly one explicit YAML field")
        return matches[0]

    values = field(field(node, "tcp"), "offset")
    if not isinstance(values, yaml.SequenceNode) or len(values.value) != 6:
        raise ValueError("tcp.offset must contain six explicit YAML values")
    replacements = []
    for value, item in zip(offset, values.value):
        if not isinstance(item, yaml.ScalarNode):
            raise ValueError("tcp.offset must contain scalar values")
        replacements.append((item.start_mark.index, item.end_mark.index, repr(value)))
    updated = text
    for start, end, value in sorted(replacements, reverse=True):
        updated = updated[:start] + value + updated[end:]
    expected = deepcopy(data)
    expected["tcp"]["offset"] = offset
    if yaml.safe_load(updated) != expected:
        raise ValueError("TCP update would affect other YAML fields; refusing write")
    if path.read_bytes() != original:
        raise RuntimeError("Configuration changed during TCP read; refusing overwrite")
    if validate_tcp_offset(data["tcp"]["offset"]) == offset:
        return None

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    backup = path.with_name(f"{path.stem}.tcp_{stamp}.backup{path.suffix}")
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=f".{path.name}.", delete=False) as handle:
            temporary = Path(handle.name)
            os.fchmod(handle.fileno(), path.stat().st_mode & 0o777)
            handle.write(updated.encode("utf-8"))
            handle.flush()
            os.fsync(handle.fileno())
        with backup.open("xb") as handle:
            handle.write(original)
            handle.flush()
            os.fsync(handle.fileno())
        if path.read_bytes() != original:
            raise RuntimeError("Configuration changed before save; refusing overwrite")
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return backup


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(ROOT / "config.yaml"))
    parser.add_argument("--write-config", action="store_true", help="save active TCP to tcp.offset, with a backup; default: print only")
    args = parser.parse_args()
    config_path = Path(args.config).expanduser().resolve()
    original = config_path.read_bytes()
    config = load_config(config_path)
    if config_path.read_bytes() != original:
        raise RuntimeError("Configuration changed while loading")
    print('读取 UR read-only 30012 Cartesian Info；不创建 RTDE Control。')
    offset = read_tcp_offset_readonly(config['robot']['robot_ip'])

    configured = validate_tcp_offset(config["tcp"]["offset"])
    tolerance = float(config["tcp"]["offset_tolerance"])
    print(f"robot_ip={config['robot']['robot_ip']}")
    print(f"active_tcp_offset={offset}")
    if args.write_config:
        print(f"previous_config_tcp_offset={configured}")
        if not confirm_enter(f'即将把上述 active TCP 保存到 {config_path}，改动时备份原配置。'):
            print('已取消，未写配置。')
            return 0
        backup = save_tcp_offset(config_path, original, offset)
        if backup is None:
            print(f"TCP 已完全一致，无需改写：{config_path}")
        else:
            print(f"TCP 已写入：{config_path} -> tcp.offset")
            print(f"原配置备份：{backup}")
        print("请核对输出；TCP 变化后需重新采集相关扫描、箱体及复位标定，再离线检查。")
        return 0
    print("Copy this into config.yaml:")
    print(f"  offset: {offset}")
    if tcp_offsets_match(offset, configured, tolerance):
        print(f"config_match=true (tolerance={tolerance} m/rad)")
        return 0
    print(f"config_match=false (configured={configured}, tolerance={tolerance} m/rad)")
    print("Read succeeded; update tcp.offset before execute mode.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except SystemExit:
        raise
    except (KeyboardInterrupt, EOFError):
        print('已取消，未写配置。')
        raise SystemExit(130)
    except Exception as exc:
        print(f"active TCP read failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(1)
