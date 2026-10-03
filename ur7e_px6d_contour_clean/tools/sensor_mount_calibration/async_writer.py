"""Bounded calibration logging, with all filesystem work on one daemon worker.

Only short in-memory sections hold the condition lock. A stalled filesystem can
neither block a producer nor extend the caller's bounded finish timeout.
"""
from __future__ import annotations

from collections import deque
import json
import math
import os
from pathlib import Path
import tempfile
import threading
import time
from typing import Callable


class StorageError(RuntimeError):
    """A calibration log cannot safely accept or durably finish its records."""


class AsyncLogWriter:
    def __init__(self, path: Path, *, queue_capacity: int = 512,
                 max_backlog_sec: float = 2., flush_interval_sec: float = .25,
                 batch_size: int = 64, clock: Callable[[], float] = time.monotonic):
        if (not isinstance(queue_capacity, int) or queue_capacity < 1
                or not isinstance(batch_size, int) or batch_size < 1):
            raise ValueError("queue capacity and batch size must be positive integers")
        if any(not math.isfinite(value) or value <= 0
               for value in (max_backlog_sec, flush_interval_sec)):
            raise ValueError("logging deadlines must be finite and positive")
        self.path = path
        self.capacity = queue_capacity
        self.max_backlog_sec = max_backlog_sec
        self.flush_interval_sec = flush_interval_sec
        self.batch_size = batch_size
        self.clock = clock
        self._condition = threading.Condition()
        self._queue: deque[tuple[str, str, float]] = deque()
        self._active_at: float | None = None
        self._closing = False
        self._finished = False
        self._error: str | None = None
        self._highwater = 0
        self._accepted = {name: 0 for name in ("raw", "timing", "metadata")}
        self._written = dict.fromkeys(self._accepted, 0)
        self._metrics = {name: {"count": 0, "last_sec": 0., "max_sec": 0.}
                         for name in ("write", "fsync")}
        self._thread = threading.Thread(target=self._run, name="mount-calibration-logging", daemon=True)
        self._thread.start()

    def _age_locked(self) -> float:
        timestamps = ([self._queue[0][2]] if self._queue else [])
        if self._active_at is not None:
            timestamps.append(self._active_at)
        return max(0., self.clock() - min(timestamps)) if timestamps else 0.

    def _latch_locked(self, message: str) -> None:
        if self._error is None:
            self._error = message

    def _health_locked(self) -> None:
        if self._age_locked() > self.max_backlog_sec:
            self._latch_locked(f"logging backlog stalled for more than {self.max_backlog_sec:g} s")
        if self._error is not None:
            raise StorageError(self._error)

    def check_health(self) -> None:
        with self._condition:
            self._health_locked()

    def enqueue(self, kind: str, encoded: str) -> None:
        """Accept an immutable JSON snapshot or fail immediately; never drop it."""
        with self._condition:
            self._health_locked()
            if self._closing or self._finished:
                raise StorageError("logging is already finishing or finished")
            if len(self._queue) >= self.capacity:
                self._latch_locked(f"logging queue overflow (capacity {self.capacity}); record not accepted")
                raise StorageError(self._error)
            self._queue.append((kind, encoded, self.clock()))
            self._accepted[kind] += 1
            self._highwater = max(self._highwater, len(self._queue))
            self._condition.notify()

    @property
    def is_alive(self) -> bool:
        return self._thread.is_alive()

    @property
    def diagnostics(self) -> dict:
        with self._condition:
            return {
                "mode": "async", "state": ("failed" if self._error else
                    "finished" if self._finished else "finishing" if self._closing else "running"),
                "worker_alive": self.is_alive, "error": self._error,
                "queue_capacity": self.capacity, "queue_depth": len(self._queue),
                "queue_highwater": self._highwater,
                "oldest_pending_age_sec": self._age_locked(),
                "max_backlog_sec": self.max_backlog_sec,
                "flush_interval_sec": self.flush_interval_sec, "batch_size": self.batch_size,
                "accepted": dict(self._accepted), "written": dict(self._written),
                "durable": self._finished and self._error is None,
                "worker_io": {name: dict(metric) for name, metric in self._metrics.items()},
            }

    def finish(self, timeout_sec: float = 5.) -> None:
        if not math.isfinite(timeout_sec) or timeout_sec < 0:
            raise ValueError("finish timeout must be finite and nonnegative")
        with self._condition:
            self._closing = True
            self._condition.notify_all()
        self._thread.join(timeout_sec)
        with self._condition:
            if self.is_alive:
                self._latch_locked(f"logging drain timeout after {timeout_sec:g} s; worker still active")
            if self._error is not None:
                raise StorageError(self._error)
            if not self._finished or self._accepted != self._written:
                self._latch_locked("logging did not durably finish every accepted record")
                raise StorageError(self._error)

    def _timed(self, name: str, operation):
        started = self.clock()
        try:
            return operation()
        finally:
            elapsed = max(0., self.clock() - started)
            with self._condition:
                metric = self._metrics[name]
                metric["count"] += 1
                metric["last_sec"] = elapsed
                metric["max_sec"] = max(metric["max_sec"], elapsed)

    def _sync(self, handle) -> None:
        def flush():
            handle.flush()
            os.fsync(handle.fileno())
        self._timed("fsync", flush)

    def _sync_directory(self) -> None:
        descriptor = os.open(self.path, os.O_RDONLY | os.O_DIRECTORY)
        try:
            self._timed("fsync", lambda: os.fsync(descriptor))
        finally:
            os.close(descriptor)

    def _metadata(self, encoded: str) -> None:
        path = self.path / "metadata.json"
        previous = json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}
        if not isinstance(previous, dict):
            raise ValueError("saved metadata must be a mapping")
        previous.update(json.loads(encoded))
        payload = json.dumps(previous, ensure_ascii=False, allow_nan=False, indent=2) + "\n"
        descriptor, name = tempfile.mkstemp(prefix=".metadata.json.", dir=self.path)
        temporary = Path(name)
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
                self._timed("write", lambda: handle.write(payload))
                self._sync(handle)
            os.replace(temporary, path)
            self._sync_directory()
        finally:
            temporary.unlink(missing_ok=True)

    def _run(self) -> None:
        handles = {}
        dirty = set()
        pending_directory_sync = False
        last_flush = self.clock()
        batch_count = 0

        def flush():
            nonlocal last_flush, batch_count, pending_directory_sync
            for kind in tuple(dirty):
                self._sync(handles[kind])
                dirty.remove(kind)
            if pending_directory_sync:
                self._sync_directory()
                pending_directory_sync = False
            last_flush = self.clock()
            batch_count = 0

        try:
            while True:
                with self._condition:
                    if not self._queue and not self._closing:
                        self._condition.wait(max(.001, self.flush_interval_sec - (self.clock() - last_flush)))
                    item = self._queue.popleft() if self._queue else None
                    finishing = self._closing and item is None
                    self._active_at = item[2] if item else self.clock()
                if item:
                    kind, encoded, _ = item
                    if kind == "metadata":
                        self._metadata(encoded)
                    else:
                        if kind not in handles:
                            handles[kind] = (self.path / f"{kind}.jsonl").open("a", encoding="utf-8")
                            pending_directory_sync = True
                        self._timed("write", lambda: handles[kind].write(encoded))
                        dirty.add(kind)
                        batch_count += 1
                    with self._condition:
                        self._written[kind] += 1
                if finishing or batch_count >= self.batch_size or self.clock() - last_flush >= self.flush_interval_sec:
                    flush()
                with self._condition:
                    self._active_at = None
                if finishing:
                    break
        except BaseException as exc:
            with self._condition:
                self._latch_locked(f"logging worker failed: {type(exc).__name__}: {exc}")
        finally:
            # close() can also block on a broken filesystem; it stays on this
            # daemon thread, inside the same bounded caller join.
            for handle in handles.values():
                try:
                    handle.close()
                except BaseException as exc:
                    with self._condition:
                        self._latch_locked(f"logging close failed: {type(exc).__name__}: {exc}")
            with self._condition:
                self._active_at = None
                self._finished = True
                self._condition.notify_all()
