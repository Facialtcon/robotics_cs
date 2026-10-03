"""Best-effort terminal progress; terminal I/O never runs on the control thread.

Call ``emit`` with an already formatted message. Stage transitions have no key,
or use ``force=True``; recurring observations use a fixed key and are limited to
one enqueue per second. Progress must never authorize or inhibit robot motion.
"""

from __future__ import annotations

from collections import deque
import threading
import time


class ProgressReporter:
    def __init__(self, *, sink=None, queue_capacity=128, interval_sec=1.,
                 clock=time.monotonic, wall_clock=time.time, autostart=True):
        if queue_capacity < 1:
            raise ValueError('progress queue capacity must be positive')
        if interval_sec < 1.:
            raise ValueError('progress interval must be at least one second')
        self._sink = sink if sink is not None else self._print
        self._capacity = queue_capacity
        self._interval = interval_sec
        self._clock = clock
        self._wall_clock = wall_clock
        self._started_at = clock()
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._closing = threading.Event()
        self._queue = deque()
        self._last_by_key = {}
        self._thread = None
        self._failed = False
        self._counts = dict(enqueued=0, written=0, dropped=0, throttled=0,
                            coalesced=0, output_failures=0)
        if autostart:
            self.start()

    @staticmethod
    def _print(line):
        print(line, flush=True)

    def start(self):
        """Start outside the live observation loop; repeated calls are harmless."""
        if self._thread is None and not self._closing.is_set():
            self._thread = threading.Thread(target=self._run,
                                            name='mount-calibration-progress',
                                            daemon=True)
            self._thread.start()
        return self

    def emit(self, message: str, *, key: str | None = None, force=False) -> bool:
        """Nonblocking enqueue. False only means this display update was skipped."""
        if self._thread is None or self._closing.is_set() or self._failed:
            return False
        # Even transient queue-lock contention must not extend an 80 ms safety
        # observation. A missed display line has no effect on raw data logging.
        if not self._lock.acquire(blocking=False):
            return False
        try:
            if self._closing.is_set() or self._failed:
                return False
            now = self._clock()
            heartbeat = key is not None and not force
            if heartbeat and now - self._last_by_key.get(key, float('-inf')) < self._interval:
                self._counts['throttled'] += 1
                return False
            # Preserve stage messages. If a terminal stalls, keep the newest
            # queued heartbeat for each key instead of replaying stale progress.
            if heartbeat:
                for index, queued in enumerate(self._queue):
                    if queued[0] == key:
                        del self._queue[index]
                        self._counts['coalesced'] += 1
                        break
            if len(self._queue) >= self._capacity:
                for index, queued in enumerate(self._queue):
                    if queued[0] is not None:
                        del self._queue[index]
                        self._counts['dropped'] += 1
                        break
                else:
                    # A permanently blocked terminal cannot retain unbounded
                    # stage messages; leave those already accepted in order.
                    self._counts['dropped'] += 1
                    return False
            if heartbeat:
                # The integration uses a fixed small key set. Bound bookkeeping
                # too, so arbitrary external keys cannot cause unbounded growth.
                if key not in self._last_by_key and len(self._last_by_key) >= self._capacity:
                    del self._last_by_key[next(iter(self._last_by_key))]
                self._last_by_key[key] = now
            self._queue.append((key if heartbeat else None, now, self._wall_clock(), message))
            self._counts['enqueued'] += 1
            self._wake.set()
            return True
        finally:
            self._lock.release()

    def _run(self):
        while True:
            self._wake.wait()
            with self._lock:
                if not self._queue:
                    if self._closing.is_set():
                        return
                    self._wake.clear()
                    continue
                _, observed_at, wall_time, message = self._queue.popleft()
            try:
                stamp = time.strftime('%H:%M:%S', time.localtime(wall_time))
                elapsed = max(0., observed_at - self._started_at)
                self._sink(f'[{stamp} +{elapsed:6.1f}s] {message}')
            except BaseException:
                # Output is diagnostic only. Broken pipes, closed terminals, or
                # sink failures must not raise into the robot control thread.
                with self._lock:
                    self._failed = True
                    self._counts['output_failures'] += 1
                    self._counts['dropped'] += len(self._queue)
                    self._queue.clear()
                return
            with self._lock:
                self._counts['written'] += 1

    @property
    def stats(self):
        with self._lock:
            return {**self._counts, 'queue_depth': len(self._queue),
                    'worker_alive': self._thread is not None and self._thread.is_alive()}

    def close(self, timeout_sec=.5) -> bool:
        """Drain for at most 0.5 s; a blocked daemon cannot delay robot cleanup."""
        self._closing.set()
        self._wake.set()
        if self._thread is None:
            return True
        self._thread.join(timeout=max(0., min(float(timeout_sec), .5)))
        alive = self._thread.is_alive()
        if alive and self._lock.acquire(blocking=False):
            try:
                self._counts['dropped'] += len(self._queue)
                self._queue.clear()
            finally:
                self._lock.release()
        return not alive

    def __enter__(self):
        return self.start()

    def __exit__(self, *_args):
        self.close()
