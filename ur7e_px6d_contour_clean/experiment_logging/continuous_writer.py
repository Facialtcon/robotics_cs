"""Bounded disk writer for real continuous runs; never calls robot or sensor APIs.

The control thread owns termination observations. Only immutable snapshots cross
to the disk thread, which reuses ExperimentLogger's existing file formats.
"""

from collections import deque
from copy import deepcopy
import json
import math
from pathlib import Path
from threading import Condition, Thread
import time


class ContinuousLogError(OSError):
    """A write failed, or the bounded logging backlog cannot keep up."""


class ContinuousLogWriter:
    def __init__(self, logger, *, capacity=64, max_pending_sec=1.0, diagnostic_only=False):
        if isinstance(capacity, bool) or not isinstance(capacity, int) or capacity <= 0:
            raise ValueError("log capacity must be a positive integer")
        if not math.isfinite(max_pending_sec) or max_pending_sec <= 0:
            raise ValueError("log max_pending_sec must be finite and positive")
        self.run_dir = logger.run_dir
        self.termination = logger.termination
        self._logger = logger
        self._capacity = capacity
        self._max_pending_sec = float(max_pending_sec)
        self._pending = deque()
        self._condition = Condition()
        self._error = None
        self._diagnostic_only = diagnostic_only
        self._diagnostics = {}
        self._dropped_records = 0
        self._closing = False
        self._closed = False
        # An older queued sample must not replace the latest control observation.
        logger.termination = None
        self._thread = Thread(target=self._work, name="continuous-log-writer", daemon=True)
        try:
            self._thread.start()
        except BaseException:
            logger.termination = self.termination
            raise

    def _remember_error(self, message, cause=None):
        self._diagnostics['write_error'] = f'WARNING: {message}'
        if self._error is None:
            self._error = ContinuousLogError(message)
            self._error.__cause__ = cause
        return self._error

    def _check_locked(self):
        if self._error is not None:
            if not self._diagnostic_only:
                raise self._error
        if self._pending and time.monotonic() - self._pending[0][0] > self._max_pending_sec:
            message = f"continuous log record pending longer than {self._max_pending_sec:g} s"
            if self._diagnostic_only:
                self._diagnostics['backlog'] = f'WARNING: {message}'
            else:
                raise self._remember_error(message)

    @property
    def diagnostics(self):
        with self._condition:
            return {**self._diagnostics, 'dropped_records': self._dropped_records}

    def check_health(self):
        """Observe writer health; real continuous runs keep it diagnostic only."""
        with self._condition:
            self._check_locked()

    def _enqueue(self, method, args, kwargs):
        with self._condition:
            self._check_locked()
            if self._closing:
                raise ContinuousLogError("continuous log writer is closed")
            # Includes the in-flight item: at most capacity snapshots are owned.
            if len(self._pending) >= self._capacity:
                if self._diagnostic_only:
                    self._dropped_records += 1
                    self._diagnostics['backlog'] = f'WARNING: log queue full; {self._dropped_records} records dropped'
                    return
                raise self._remember_error(f"continuous log queue reached capacity {self._capacity}")
            timestamp = time.monotonic()
            copied_args, copied_kwargs = deepcopy((args, kwargs))
            self._pending.append((timestamp, method, copied_args, copied_kwargs))
            self._condition.notify()

    def log_sample(self, monotonic_sec, raw, processed, robot, command,
                   target_direction, tangent, *, extra=None):
        if self.termination is not None:
            self.termination.observe(robot=robot, raw=raw, processed=processed,
                command=command, timestamp=monotonic_sec,
                processed_force_frame=(extra or {}).get('processed_force_frame',
                    getattr(self._logger, 'processed_force_frame', 'configured_output_frame')))
        self._enqueue("log_sample", (monotonic_sec, raw, processed, robot, command,
                                     target_direction, tangent), {"extra": extra})

    def log_waypoint(self, waypoint):
        self._enqueue("log_waypoint", (waypoint,), {})

    def write_json(self, name, payload):
        """Queue a run-directory JSON file in the same order as sample records."""
        if Path(name).name != str(name):
            raise ValueError("log JSON name must be a filename inside the run directory")
        self._enqueue("write_json", (name, payload), {})

    def _work(self):
        try:
            while True:
                with self._condition:
                    self._condition.wait_for(lambda: self._pending or self._closing)
                    if not self._pending:
                        break
                    _, method, args, kwargs = self._pending[0]
                try:
                    if method == "write_json":
                        name, payload = args
                        with (self.run_dir / name).open("w", encoding="utf-8") as stream:
                            json.dump(payload, stream, ensure_ascii=False, indent=2)
                    else:
                        getattr(self._logger, method)(*args, **kwargs)
                except Exception as exc:
                    with self._condition:
                        self._remember_error(f"continuous log write failed: {exc}", exc)
                finally:
                    with self._condition:
                        self._pending.popleft()
                        self._condition.notify_all()
        finally:
            # Only the worker closes files, after its last write. If a disk call
            # blocks, close() can return with an error without racing the handles.
            self._logger.termination = self.termination
            try:
                self._logger.close()
            except Exception as exc:
                with self._condition:
                    self._remember_error(f"continuous log close failed: {exc}", exc)
                # ExperimentLogger.close is sequential: one failed flush/close
                # otherwise prevents every later handle from being closed.
                # Retry each remaining resource here, on the same disk thread.
                for name in ("_samples", "_full_log", "_boundaries",
                             "_waypoints", "_recovery_rays", "_workspace_logger"):
                    resource = getattr(self._logger, name, None)
                    if resource is None or getattr(resource, "closed", False):
                        continue
                    try:
                        resource.close()
                    except Exception as cleanup_error:
                        with self._condition:
                            self._remember_error(
                                f"continuous log cleanup failed: {cleanup_error}", cleanup_error)
            finally:
                with self._condition:
                    self._closed = True
                    self._condition.notify_all()

    def _final_operation(self, method, *args, **kwargs):
        # Called only after motion has stopped. Wait for accepted records, then
        # attempt final diagnostics even if an earlier individual write failed.
        with self._condition:
            if self._closing:
                raise ContinuousLogError("continuous log writer is closed")
            deadline = time.monotonic() + self._max_pending_sec
            while self._pending:
                remaining = min(deadline, self._pending[0][0] + self._max_pending_sec) - time.monotonic()
                if remaining <= 0:
                    error = self._remember_error("continuous log records still pending at drain deadline")
                    if self._diagnostic_only:
                        return
                    raise error
                self._condition.wait(remaining)
            self._logger.termination = self.termination
            try:
                if method == "write_final_waypoints":
                    first_error = None
                    for event in args[0]:
                        try:
                            self._logger.log_waypoint(event)
                        except Exception as exc:
                            if first_error is None:
                                first_error = exc
                    if first_error is not None:
                        raise first_error
                else:
                    getattr(self._logger, method)(*args, **kwargs)
            except Exception as exc:
                error = self._remember_error(f"continuous log final write failed: {exc}", exc)
                if not self._diagnostic_only:
                    raise error
            finally:
                self._logger.termination = None
            self._check_locked()

    def write_summary(self, *args, **kwargs):
        self._final_operation("write_summary", *args, **kwargs)

    def write_final_waypoints(self, events):
        """Persist stop events after draining, even when an earlier write failed."""
        self._final_operation("write_final_waypoints", events)

    def write_stop_snapshot(self, *args, **kwargs):
        self._final_operation("write_stop_snapshot", *args, **kwargs)

    def flush(self):
        self._final_operation("flush")

    def close(self):
        with self._condition:
            if self._closing and self._closed:
                return
            self._closing = True
            self._condition.notify_all()
        self._thread.join(self._max_pending_sec)
        with self._condition:
            if self._thread.is_alive():
                error = self._remember_error("continuous log records still pending; cleanup deferred to disk writer")
                if not self._diagnostic_only:
                    raise error
            self._check_locked()
