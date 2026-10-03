"""Progress is observable, bounded, and independent of terminal responsiveness."""

import threading
import time

import pytest

from tools.sensor_mount_calibration.progress import ProgressReporter


class HeldTerminal:
    def __init__(self):
        self.entered = threading.Event()
        self.release = threading.Event()
        self.lines = []
        self.threads = []

    def __call__(self, line):
        self.threads.append(threading.get_ident())
        self.entered.set()
        if not self.release.wait(3.):
            raise RuntimeError('test did not release terminal')
        self.lines.append(line)


def test_slow_terminal_never_blocks_observation_or_robot_cleanup():
    terminal = HeldTerminal()
    progress = ProgressReporter(sink=terminal)
    try:
        assert progress.emit('开始路径预检')
        assert terminal.entered.wait(1.)
        # The same write stays blocked longer than the observation deadline.
        assert not terminal.release.wait(.12)
        started = time.monotonic()
        assert progress.emit('预检 125/600 点', key='preflight')
        assert progress.emit('开始姿态 1/17', force=True)
        assert time.monotonic() - started < .05
        assert not progress.close(timeout_sec=.02)
        assert time.monotonic() - started < .15
        assert progress.stats['queue_depth'] == 0
        assert terminal.threads == [terminal.threads[0]]
        assert terminal.threads[0] != threading.get_ident()
        assert progress._thread.daemon
    finally:
        terminal.release.set()
        progress.close()


def test_stages_remain_ordered_and_heartbeat_is_throttled_and_coalesced():
    terminal = HeldTerminal()
    clock = [100.]
    progress = ProgressReporter(sink=terminal, clock=lambda: clock[0])
    try:
        assert progress.emit('开始路径预检')
        assert terminal.entered.wait(1.)
        assert progress.emit('预检 1/600 点', key='preflight')
        clock[0] += .9
        assert not progress.emit('预检 90/600 点', key='preflight')
        clock[0] += .2
        assert progress.emit('预检 110/600 点', key='preflight')
        assert progress.emit('开始姿态 1/17', force=True)
        assert progress.emit('采样 20/80', key='sampling')
        assert not progress.emit('采样 21/80', key='sampling')
        assert progress.emit('完成姿态 1/17', force=True)
    finally:
        terminal.release.set()
        assert progress.close()
    assert [line.split('] ', 1)[1] for line in terminal.lines] == [
        '开始路径预检', '预检 110/600 点', '开始姿态 1/17', '采样 20/80', '完成姿态 1/17']
    assert '+   1.1s]' in terminal.lines[1]
    assert progress.stats['throttled'] == 2
    assert progress.stats['coalesced'] == 1


def test_queue_overflow_discards_heartbeat_before_accepted_stage_messages():
    terminal = HeldTerminal()
    progress = ProgressReporter(sink=terminal, queue_capacity=3)
    try:
        assert progress.emit('第一行')
        assert terminal.entered.wait(1.)
        assert progress.emit('阶段一')
        assert progress.emit('进度一', key='moving')
        assert progress.emit('阶段二')
        assert progress.emit('阶段三')
        assert not progress.emit('阶段四')
        assert progress.stats['queue_depth'] == 3
        assert progress.stats['dropped'] == 2
    finally:
        terminal.release.set()
        assert progress.close()
    assert [line.split('] ', 1)[1] for line in terminal.lines] == [
        '第一行', '阶段一', '阶段二', '阶段三']


def test_busy_queue_lock_never_blocks_owner():
    progress = ProgressReporter(sink=lambda line: None)
    try:
        progress._lock.acquire()
        try:
            started = time.monotonic()
            assert not progress.emit('应跳过的进度', key='moving')
            assert time.monotonic() - started < .05
        finally:
            progress._lock.release()
    finally:
        assert progress.close()


def test_terminal_failure_is_diagnostic_only_and_does_not_escape():
    attempted = threading.Event()

    def broken_terminal(line):
        attempted.set()
        raise BrokenPipeError('terminal closed')

    progress = ProgressReporter(sink=broken_terminal)
    assert progress.emit('开始采集')
    assert attempted.wait(1.)
    assert progress.close()
    assert progress.stats['output_failures'] == 1
    assert not progress.emit('不能再输出')


def test_default_sink_flushes_on_worker_only(monkeypatch):
    calls = []
    monkeypatch.setattr('builtins.print',
                        lambda *args, **kwargs: calls.append((args, kwargs, threading.get_ident())))
    with ProgressReporter() as progress:
        assert progress.emit('求解完成，独立姿态验证通过')
    assert len(calls) == 1
    args, kwargs, thread_id = calls[0]
    assert '求解完成，独立姿态验证通过' in args[0]
    assert kwargs == {'flush': True}
    assert thread_id != threading.get_ident()


def test_start_and_close_are_idempotent_and_closed_reporter_stays_closed():
    lines = []
    progress = ProgressReporter(sink=lines.append, autostart=False)
    assert not progress.emit('尚未启动')
    assert progress.start() is progress
    worker = progress._thread
    assert progress.start()._thread is worker
    assert progress.emit('结果已保存')
    assert progress.close()
    assert progress.close()
    assert progress.start()._thread is worker
    assert not progress.emit('关闭之后')
    assert len(lines) == 1


@pytest.mark.parametrize('arguments', [{'queue_capacity': 0}, {'interval_sec': .5}])
def test_invalid_progress_limits_fail_before_worker_start(arguments):
    with pytest.raises(ValueError):
        ProgressReporter(**arguments)
