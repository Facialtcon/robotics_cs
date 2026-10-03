"""Independent calibration records and narrowly scoped, reviewed config writes.

The scanner consumes ``preprocessing.coordinate_transform``. R_TS is a proper
sensor-to-active-TCP rotation; fitted bias, mass and force polarity are diagnostic
only and this module never writes them into scanning/compensation parameters.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass, field
from datetime import datetime, timezone
import difflib
import fcntl
import hashlib
import json
import os
from pathlib import Path
import stat
import tempfile
from typing import Any
import uuid

import numpy as np
import yaml
from yaml.nodes import MappingNode, Node, ScalarNode, SequenceNode

from .async_writer import AsyncLogWriter, StorageError


def _json_default(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"cannot serialize {type(value).__name__}")


def _json(value: Any, *, indent: int | None = None) -> str:
    return json.dumps(value, ensure_ascii=False, allow_nan=False,
                      default=_json_default, indent=indent)


def _sync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _atomic_bytes(path: Path, payload: bytes) -> None:
    descriptor, name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        _sync_directory(path.parent)
    finally:
        temporary.unlink(missing_ok=True)


@dataclass
class RunStore:
    """A distinct attempt directory, with bounded worker logging during motion.

    Offline callers retain synchronous durable writes. Live acquisition explicitly
    starts the worker before enabling control and must finish it after stopping the
    robot, before solving or installing a result.
    """

    path: Path
    _logging: AsyncLogWriter | None = field(default=None, init=False, repr=False)
    _async_active: bool = field(default=False, init=False, repr=False)

    def start_async(self, *, queue_capacity: int = 512, max_backlog_sec: float = 2.,
                    flush_interval_sec: float = .25, batch_size: int = 64, **kwargs) -> None:
        """Start once; subsequent data/metadata writes only validate and enqueue."""
        if self._logging is not None:
            raise StorageError("asynchronous logging has already been started")
        self._logging = AsyncLogWriter(
            self.path, queue_capacity=queue_capacity, max_backlog_sec=max_backlog_sec,
            flush_interval_sec=flush_interval_sec, batch_size=batch_size, **kwargs)
        self._async_active = True

    def check_logging_health(self) -> None:
        """Nonblocking guard for acquisition and before every motion command."""
        if self._logging is not None:
            self._logging.check_health()

    @property
    def logging_diagnostics(self) -> dict:
        if self._logging is None:
            return {"mode": "sync", "state": "idle", "worker_alive": False}
        return self._logging.diagnostics

    def finish_logging(self, timeout_sec: float = 5.) -> None:
        """After stopping the robot, drain durably within the requested timeout.

        After a failed drain, results remain forbidden. Metadata can record the abort
        only once the worker has exited, so it cannot race a pending metadata update.
        """
        if self._logging is None:
            return
        try:
            self._logging.finish(timeout_sec)
        finally:
            if not self._logging.is_alive:
                self._async_active = False

    @classmethod
    def create(cls, root: str | Path, metadata: dict) -> "RunStore":
        root = Path(root).resolve()
        root.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        path = root / f"mount_{stamp}_{uuid.uuid4().hex[:8]}"
        path.mkdir()
        store = cls(path)
        (path / "raw.jsonl").touch(exist_ok=False)
        store.write_metadata(metadata)
        return store

    def append_raw(self, record: dict) -> None:
        """Store unprocessed force/torque with the paired actual TCP observation."""
        required = {"pose_id", "split", "actual_tcp_pose", "raw_wrench",
                    "host_monotonic", "utc_time", "robot_timestamp"}
        missing = required.difference(record)
        if missing:
            raise ValueError(f"raw record is missing {sorted(missing)}")
        for name in ("actual_tcp_pose", "raw_wrench"):
            values = np.asarray(record[name], dtype=float)
            if values.shape != (6,) or not np.isfinite(values).all():
                raise ValueError(f"{name} must contain six finite values")
        for name in ("host_monotonic", "robot_timestamp"):
            if not np.isfinite(float(record[name])):
                raise ValueError(f"{name} must be finite")
        if not isinstance(record["utc_time"], str) or not record["utc_time"]:
            raise ValueError("utc_time must be a nonempty timestamp string")
        if record["split"] not in {"fit", "validation", "reference", "motion", "settling", "preflight", "stability"}:
            raise ValueError("unknown raw-record split")
        encoded = _json(record) + "\n"
        if self._async_active:
            self._logging.enqueue("raw", encoded)
            return
        self.check_logging_health()
        with (self.path / "raw.jsonl").open("a", encoding="utf-8") as handle:
            handle.write(encoded)
            handle.flush()
            os.fsync(handle.fileno())

    def write_metadata(self, metadata: dict) -> None:
        """Merge stage updates without discarding initial plan/config provenance."""
        if not isinstance(metadata, dict):
            raise TypeError("metadata update must be a mapping")
        if self._async_active:
            self._logging.enqueue("metadata", _json(metadata))
            return
        path = self.path / "metadata.json"
        previous = json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}
        if not isinstance(previous, dict):
            raise ValueError("saved metadata must be a mapping")
        previous.update(metadata)
        _atomic_bytes(path, (_json(previous, indent=2) + "\n").encode())

    def write_result(self, result: dict) -> None:
        self.check_logging_health()
        if self._async_active:
            raise StorageError("finish logging durably before writing a calibration result")
        _atomic_bytes(self.path / "result.json", (_json(result, indent=2) + "\n").encode())

    def append_timing(self, event: dict) -> None:
        """Store host-side timing separately from untouched sensor samples."""
        if not isinstance(event, dict):
            raise TypeError("timing event must be a mapping")
        encoded = _json(event) + "\n"
        if self._async_active:
            self._logging.enqueue("timing", encoded)
            return
        self.check_logging_health()
        with (self.path / "timing.jsonl").open("a", encoding="utf-8") as handle:
            handle.write(encoded)
            handle.flush()
            os.fsync(handle.fileno())


def _proper_rotation(value: Any) -> np.ndarray:
    rotation = np.asarray(value, dtype=float)
    if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
            or not np.allclose(rotation.T @ rotation, np.eye(3), rtol=0, atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1., rtol=0, atol=1e-6)):
        raise ValueError("rotation_sensor_to_tool must be a finite proper rotation (det=+1)")
    return rotation


def _field(node: Node, key: str) -> Node | None:
    if not isinstance(node, MappingNode):
        raise ValueError(f"expected YAML mapping containing {key}")
    found = [value for name, value in node.value
             if isinstance(name, ScalarNode) and name.value == key]
    if len(found) > 1:
        raise ValueError(f"duplicate YAML key: {key}")
    return found[0] if found else None


def _reject_ambiguous_yaml(node: Node, seen: set[int] | None = None) -> None:
    """Reject aliases/duplicate/merge keys rather than editing shared YAML nodes."""
    seen = set() if seen is None else seen
    if id(node) in seen:
        raise ValueError("config update does not support YAML aliases")
    seen.add(id(node))
    if isinstance(node, MappingNode):
        names = []
        for key, value in node.value:
            if not isinstance(key, ScalarNode) or key.value == "<<":
                raise ValueError("config update requires ordinary scalar YAML keys")
            if key.value in names:
                raise ValueError(f"duplicate YAML key: {key.value}")
            names.append(key.value)
            _reject_ambiguous_yaml(value, seen)
    elif isinstance(node, SequenceNode):
        for value in node.value:
            _reject_ambiguous_yaml(value, seen)


def _content_end(node: Node) -> int:
    # Block-node end_mark includes following comments/whitespace; preserve those.
    if isinstance(node, SequenceNode) and not node.flow_style and node.value:
        return _content_end(node.value[-1])
    if isinstance(node, MappingNode) and not node.flow_style and node.value:
        return _content_end(node.value[-1][1])
    return node.end_mark.index


def _replace_transform_field(source: str, key: str, value: Any) -> str:
    document = yaml.compose(source, Loader=yaml.SafeLoader)
    preprocessing = _field(document, "preprocessing")
    transform = _field(preprocessing, "coordinate_transform")
    if not isinstance(transform, MappingNode):
        raise ValueError("preprocessing.coordinate_transform must be a mapping")
    existing = _field(transform, key)
    replacement = _json(value)
    if existing is not None:
        key_node = next(name for name, child in transform.value if child is existing)
        if (existing.start_mark.line != key_node.start_mark.line and
                existing.start_mark.column <= key_node.start_mark.column):
            # PyYAML commonly writes an indentless block sequence. A replacement
            # flow value/scalar needs indentation beyond its parent key.
            replacement = " " * (key_node.start_mark.column + 2 - existing.start_mark.column) + replacement
        return source[:existing.start_mark.index] + replacement + source[_content_end(existing):]
    if transform.flow_style:
        index = transform.end_mark.index - 1  # Closing brace, preserving other fields.
        prefix = ", " if transform.value else ""
        return source[:index] + prefix + key + ": " + replacement + source[index:]
    if not transform.value:
        raise ValueError("empty block coordinate_transform is unsupported")
    key_node = transform.value[0][0]
    index = source.rfind("\n", 0, key_node.start_mark.index) + 1
    newline = "\r\n" if "\r\n" in source else "\n"
    line = " " * key_node.start_mark.column + key + ": " + replacement + newline
    return source[:index] + line + source[index:]


@dataclass(frozen=True)
class ConfigUpdate:
    path: Path
    original_bytes: bytes
    updated_bytes: bytes
    changes: dict
    notes: tuple[str, ...]
    diff: str

    @property
    def original_sha256(self) -> str:
        return hashlib.sha256(self.original_bytes).hexdigest()


def preview_config_update(config_path: str | Path, rotation_sensor_to_tool: Any) -> ConfigUpdate:
    """Prepare exact reviewable bytes, without mutating configuration.

    A valid legacy R_BS has lower precedence than direct R_TS and is preserved.
    An invalid legacy placeholder is nulled because the existing preprocessor
    validates R_BS before choosing R_TS. All unrelated values remain identical.
    """
    path = Path(config_path).resolve()
    original = path.read_bytes()
    source = original.decode("utf-8")
    document = yaml.compose(source, Loader=yaml.SafeLoader)
    if document is None:
        raise ValueError("configuration is empty")
    _reject_ambiguous_yaml(document)
    config = yaml.safe_load(source)
    if not isinstance(config, dict) or not isinstance(config.get("preprocessing"), dict):
        raise ValueError("configuration requires preprocessing mapping")
    transform = config["preprocessing"].get("coordinate_transform")
    if not isinstance(transform, dict):
        raise ValueError("configuration requires preprocessing.coordinate_transform mapping")
    rotation = _proper_rotation(rotation_sensor_to_tool)
    wanted = rotation.tolist()
    updated = copy.deepcopy(config)
    updated_transform = updated["preprocessing"]["coordinate_transform"]
    changes = {}
    if transform.get("rotation_sensor_to_tool") != wanted:
        changes["preprocessing.coordinate_transform.rotation_sensor_to_tool"] = {
            "before": transform.get("rotation_sensor_to_tool"), "after": wanted}
        updated_transform["rotation_sensor_to_tool"] = wanted
        source = _replace_transform_field(source, "rotation_sensor_to_tool", wanted)
    notes = ["R_TS maps sensor components into the active TCP; scanner uses R_BS = R_BT @ R_TS.",
             "Bias, weight, force polarity, origins, TCP, payload, scan speeds and compensation are unchanged."]
    legacy = transform.get("rotation_sensor_to_base")
    if legacy is not None:
        try:
            _proper_rotation(legacy)
        except (ValueError, TypeError):
            changes["preprocessing.coordinate_transform.rotation_sensor_to_base"] = {
                "before": legacy, "after": None}
            updated_transform["rotation_sensor_to_base"] = None
            source = _replace_transform_field(source, "rotation_sensor_to_base", None)
            notes.append("Invalid legacy R_BS placeholder is set to null: old loader validates it before R_TS.")
        else:
            notes.append("Valid legacy R_BS is preserved; direct R_TS takes precedence at every actual TCP pose.")
    if yaml.safe_load(source) != updated:
        raise ValueError("targeted YAML update changed unrelated configuration; refusing write")
    # Verify the real consumer, without scanning startup or device construction.
    from sensor.force_preprocess import WrenchPreprocessor
    processor = WrenchPreprocessor.from_config(updated["preprocessing"], tool_orientation=[0., 0., 0.])
    if not np.allclose(processor.rotation_sensor_to_output, rotation, atol=1e-10):
        raise ValueError("existing scanner does not consume sensor-to-tool rotation in the expected direction")
    processor.set_tool_orientation([0., 0., np.pi / 2])
    rz = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    if not processor.force_transform_status["available"] or not np.allclose(
            processor.rotation_sensor_to_output, rz @ rotation, atol=1e-10):
        raise ValueError("existing scanner cannot update mounting transform from actual TCP pose")
    diff = "".join(difflib.unified_diff(original.decode("utf-8").splitlines(keepends=True),
                                      source.splitlines(keepends=True), fromfile=str(path),
                                      tofile=f"{path} (proposed)"))
    return ConfigUpdate(path, original, source.encode("utf-8"), changes, tuple(notes), diff)


def commit_config_update(update: ConfigUpdate, backup_dir: str | Path) -> Path | None:
    """After caller's fresh Enter approval, back up exact original and atomically replace.

    A changed source invalidates the preview. Advisory locking also serializes
    concurrent calibration writers; byte and inode rechecks catch normal edits.
    Returns None when the reviewed configuration is already up to date.
    """
    path = update.path
    with path.open("rb") as current:
        fcntl.flock(current.fileno(), fcntl.LOCK_EX)
        original_stat = os.fstat(current.fileno())

        def unchanged() -> None:
            path_stat = path.stat()
            if (path_stat.st_ino != original_stat.st_ino or
                    path_stat.st_dev != original_stat.st_dev or
                    path.read_bytes() != update.original_bytes):
                raise RuntimeError("configuration changed since preview; no update performed")

        unchanged()
        if update.updated_bytes == update.original_bytes:
            return None
        backup_root = Path(backup_dir).resolve()
        backup_root.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        backup = backup_root / f"{path.name}.{stamp}.{uuid.uuid4().hex[:8]}.bak"
        with backup.open("xb") as saved:
            saved.write(update.original_bytes)
            saved.flush()
            os.fsync(saved.fileno())
        _sync_directory(backup_root)
        descriptor, name = tempfile.mkstemp(prefix=f".{path.name}.mount-", dir=path.parent)
        temporary = Path(name)
        try:
            with os.fdopen(descriptor, "wb") as target:
                os.fchmod(target.fileno(), stat.S_IMODE(original_stat.st_mode))
                target.write(update.updated_bytes)
                target.flush()
                os.fsync(target.fileno())
            unchanged()
            os.replace(temporary, path)
            _sync_directory(path.parent)
        finally:
            temporary.unlink(missing_ok=True)
        return backup
