"""Bounded startup tests using in-memory RTDE doubles; no device constructors."""
import itertools
import argparse
import json
import time
from types import SimpleNamespace

import numpy as np
import pytest

import run_continuous_tracking as runner
from config.loader import load_config, runtime_robot_config
from core.models import Wrench
from experiment_logging.data_logger import ExperimentLogger
from experiment_logging.termination import TerminationRecorder
from robot.rtde_controller import URRTDEController, WorkspaceGuard, RobotError


@pytest.fixture
def setup(tmp_path, monkeypatch):
    config = load_config(runner.ROOT / 'config.yaml')
    config['safe_return']['startup_bias_sample_count'] = 3
    start = np.array([.6, .2, .093, 0, 3.14, 0])
    rc = runtime_robot_config(config)
    rc.update(continuous_require_watchdog=True, continuous_sample_age_sec=.02,
              continuous_speed_limit=.0012, continuous_settle_speed_mps=.0001,
              continuous_xy_polygon=[[.4, -.3], [.8, -.3], [.8, .5], [.4, .5]],
              continuous_boundary_margin=.0005)
    c = URRTDEController(rc)
    pose = start.copy(); pose[0] += .05; pose[2] += .029567
    moves = []; stamps = itertools.count(); kicks = []
    def move(target, speed, acceleration, asynchronous):
        moves.append((np.array(target), speed)); pose[:] = target
        return True
    c.control = SimpleNamespace(speedStop=lambda a: True, stopL=lambda a, asynchronous=False: None,
        stopScript=lambda: None,
        setWatchdog=lambda hz: True, kickWatchdog=lambda: kicks.append(time.monotonic()) or True,
        moveL=move)
    c.receive = SimpleNamespace(getActualTCPPose=lambda: pose.copy(),
        getActualTCPSpeed=lambda: np.zeros(6), getTimestamp=lambda: next(stamps)*.002)
    c.guard = WorkspaceGuard(rc['workspace_limits'], start[2], .001, rc['max_tcp_speed'],
        fixed_orientation=start[3:], orientation_tolerance_rad=.01745, workspace_enabled=False)
    reader = SimpleNamespace(read_wrench=lambda: Wrench(0, 0, 0, 0, 0, 0))
    logger = ExperimentLogger(tmp_path, config, workspace_logging=False)
    logger.termination = TerminationRecorder(output_root=tmp_path).bind(logger.run_dir)
    monkeypatch.setattr('builtins.input', lambda prompt: 'START')
    yield SimpleNamespace(config=config, start=start, c=c, pose=pose, moves=moves,
                          reader=reader, logger=logger, kicks=kicks)
    logger.close()


def run_return(s, poll=lambda: None):
    return runner.startup_return_to_p0(s.config, s.start, s.c, s.reader, s.logger, poll)


@pytest.mark.parametrize('offset, expected_segments', [(.029567, 2), (.005, 3)])
def test_height_offset_uses_shared_return_and_logs(setup, offset, expected_segments):
    s = setup
    s.pose[2] = s.start[2] + offset
    assert run_return(s)
    assert len(s.moves) == expected_segments
    safe_z = s.start[2] + (.03 if expected_segments == 2 else offset+.03)
    assert s.moves[0][0][2] == pytest.approx(safe_z)
    assert np.allclose(s.moves[-2][0][:2], s.start[:2])
    assert np.allclose(s.pose, s.start)
    assert [speed for _, speed in s.moves] == ([.018] if expected_segments == 3 else []) + [.03, .018]
    assert not s.c._return_mode and s.c.watchdog_active and s.kicks
    assert json.loads((s.logger.run_dir / 'return_status.json').read_text())['return_status'] == 'complete'
    runner.check_start(s.c, s.start, s.config)


def test_already_at_p0_skips_return_and_does_not_arm_watchdog(setup):
    s = setup; s.pose[:] = s.start
    assert not run_return(s)
    assert not s.moves and not s.c.watchdog_active


@pytest.mark.parametrize('key', ['Q', 'ESC'])
def test_operator_stop_during_startup_baseline_never_moves(setup, key):
    with pytest.raises(KeyboardInterrupt):
        run_return(setup, lambda: key)
    assert not setup.moves


def test_rejected_start_never_moves(setup, monkeypatch):
    monkeypatch.setattr('builtins.input', lambda prompt: 'no')
    with pytest.raises(KeyboardInterrupt): run_return(setup)
    assert not setup.moves


@pytest.mark.parametrize('value', [float('nan'), 100.])
def test_invalid_or_overloaded_baseline_blocks_return(setup, value):
    setup.reader.read_wrench = lambda: Wrench(value, 0, 0, 0, 0, 0)
    with pytest.raises((ValueError, RobotError)): run_return(setup)
    assert not setup.moves


def test_return_target_cannot_leave_sandbox(setup):
    s = setup; s.c.begin_return_mode(); s.c.enable_watchdog(20); s.c.kick_watchdog()
    with pytest.raises(RobotError, match='sandbox boundary'):
        s.c.move_linear_async([.9, .2, .15, 0, 3.14, 0], .01, .03)
    assert not s.moves


def test_return_speed_scope_does_not_relax_tracking_limit(setup):
    s = setup; s.pose[:] = s.start
    s.c.receive.getActualTCPSpeed = lambda: [.018, 0, 0, 0, 0, 0]
    s.c.begin_return_mode()
    s.c.read_state()
    s.c.end_return_mode()
    with pytest.raises(RobotError, match='continuous measured speed'):
        s.c.read_state()


@pytest.mark.parametrize('failure', ['force', 'nan', 'serial_delay', 'log_error', 'log_delay', 'watchdog'])
def test_return_precheck_failure_stops_before_first_move(setup, monkeypatch, failure):
    s = setup
    reads = itertools.count()
    def read():
        if next(reads) >= s.config['safe_return']['startup_bias_sample_count']:
            if failure == 'serial_delay': time.sleep(.04)
            if failure in ('force', 'nan'):
                return Wrench(100 if failure == 'force' else float('nan'), 0, 0, 0, 0, 0)
        return Wrench(0, 0, 0, 0, 0, 0)
    s.reader.read_wrench = read
    if failure.startswith('log_'):
        def log(*a, **k):
            if failure == 'log_error': raise OSError('test disk full')
            time.sleep(.04)
        monkeypatch.setattr(s.logger, 'log_sample', log)
    if failure == 'watchdog':
        s.c.control.kickWatchdog = lambda: False
    with pytest.raises(RobotError, match='aborted'):
        run_return(s)
    assert not s.moves and not s.kicks
    assert not s.c._return_mode
    status = json.loads((s.logger.run_dir / 'return_status.json').read_text())
    assert status['return_status'] == 'aborted'


def test_stale_packets_block_startup(setup):
    setup.c.receive.getTimestamp = lambda: 1.
    with pytest.raises(RobotError, match='stale RTDE'):
        run_return(setup)
    assert not setup.moves


def test_never_settles_times_out_without_motion(setup):
    setup.config['continuous_tracking']['confirmation_timeout_sec'] = .03
    setup.c.receive.getActualTCPSpeed = lambda: [.001, 0, 0, 0, 0, 0]
    with pytest.raises(RobotError, match='standstill confirmation timed out'):
        run_return(setup)
    assert not setup.moves


def test_stop_in_return_does_not_execute_remaining_segments(setup):
    with pytest.raises(KeyboardInterrupt):
        run_return(setup, lambda: 'Q' if setup.moves else None)
    # Runner handles KeyboardInterrupt and requests stop before all cleanup.
    setup.c.stop()
    assert len(setup.moves) == 1 and not setup.c._return_mode


@pytest.mark.parametrize('operator_stop, delayed_log_and_gradual_descent', [
    (False, False), (True, False), (False, True),
])
def test_runner_returns_then_rebiases_and_runs_shared_policy(
        setup, monkeypatch, tmp_path, operator_stop, delayed_log_and_gradual_descent):
    s = setup
    s.config['continuous_tracking']['max_runtime_sec'] = .04
    s.config['preprocessing']['baseline']['sample_count'] = 2
    s.reader.connect = lambda: None
    s.reader.close = lambda: None
    moves = []
    s.c.control.speedL = lambda *a: moves.append(a) or True
    submitted = []
    written = []
    descent_stops = []
    if delayed_log_and_gradual_descent:
        from threading import current_thread

        original_log = ExperimentLogger.log_sample
        original_submit = runner.ContinuousLogWriter.log_sample

        def log(self, *args, **kwargs):
            assert current_thread().name == 'continuous-log-writer'
            if not written:
                time.sleep(.045)  # Disk latency must not consume a device cycle.
            written.append(args[0])
            return original_log(self, *args, **kwargs)

        def submit(self, *args, **kwargs):
            submitted.append(args[0])
            return original_submit(self, *args, **kwargs)

        monkeypatch.setattr(ExperimentLogger, 'log_sample', log)
        monkeypatch.setattr(runner.ContinuousLogWriter, 'log_sample', submit)
        original_move = s.c.control.moveL
        descending = [False]
        approach_errors = iter((.000433, .00019))

        def move(target, speed, acceleration, asynchronous):
            if np.allclose(target, s.start):
                s.moves.append((np.array(target), speed))
                descending[0] = True
                return True
            return original_move(target, speed, acceleration, asynchronous)

        def pose():
            if descending[0]:
                s.pose[:] = s.start
                s.pose[2] += next(approach_errors, .00019)
            return s.pose.copy()

        def stop(acceleration, asynchronous=False):
            if descending[0]:
                descent_stops.append(float(np.linalg.norm(s.pose[:3] - s.start[:3])))
                descending[0] = False

        s.c.control.moveL = move
        s.c.control.stopL = stop
        s.c.receive.getActualTCPPose = pose
    def connect(allow_start_away_from_fixed_pose=False):
        assert allow_start_away_from_fixed_pose
    s.c.connect = connect
    monkeypatch.setattr(runner, 'URRTDEController', lambda cfg: s.c)
    monkeypatch.setattr(runner, 'PX6DReader', lambda *a: s.reader)
    monkeypatch.setattr(runner, 'load_config', lambda p: s.config)
    monkeypatch.setattr(runner, 'prepare_real', lambda *a: (s.c.config, s.start))
    if operator_stop:
        monkeypatch.setattr(runner.OperatorKeyboard, 'poll', lambda self: 'Q' if s.moves else None)
    confirmations = []
    monkeypatch.setattr('builtins.input', lambda p: confirmations.append(p) or 'START')
    counts = []
    original = runner.WrenchPreprocessor.set_zero_bias
    def bias(self, samples):
        counts.append(len(samples)); return original(self, samples)
    monkeypatch.setattr(runner.WrenchPreprocessor, 'set_zero_bias', bias)
    out = tmp_path / 'runner'
    args = argparse.Namespace(preview=False, execute=True, config=runner.ROOT/'config.yaml',
                              duration=None, output=out)
    assert runner.run(args) == 0
    assert len(confirmations) == 1
    run_dir = next(out.glob('run_*'))
    if operator_stop:
        assert counts == [3] and len(s.moves) == 1 and not moves
        assert json.loads((run_dir/'termination.json').read_text())['reason'] == 'STOP_USER_REQUEST'
        assert s.c.control is None and s.c.receive is None
        return
    assert counts == [3, 2]  # independent return and scan bias captures
    assert moves and len(s.moves) == 2
    assert json.loads((run_dir/'return_status.json').read_text())['return_status'] == 'complete'
    assert json.loads((run_dir/'termination.json').read_text())['reason'] == 'STOP_TIME_LIMIT'
    if delayed_log_and_gradual_descent:
        import csv

        assert descent_stops == pytest.approx([.00019])
        assert written == submitted and len(written) > 0
        with (run_dir/'samples.csv').open() as stream:
            samples = list(csv.DictReader(stream))
        with (run_dir/'full_log.csv').open() as stream:
            full_log = list(csv.DictReader(stream))
        assert samples == full_log and len(samples) == len(submitted)
        descent = [r for r in samples if r['reason'] == 'DESCEND_TO_START']
        assert [float(r['tcp_z']) - s.start[2] for r in descent] == pytest.approx([.000433, .00019])
        assert all(float(r['commanded_speed_mps']) > 0 for r in descent)
        assert any(r['reason'] == 'DESCEND_TO_START_SETTLE' for r in samples)
        assert any(r['current_state'] == runner.State.TARGET_SEARCH.value for r in samples)
        assert s.config['continuous_tracking']['cycle_timeout_sec'] == .03
        assert s.config['policy']['position_tolerance'] == .0002


def test_return_parameters_are_validated_and_bound_before_device_connection(monkeypatch):
    config = load_config(runner.ROOT/'config.yaml')
    before = runner.site_configuration_digest(config)
    config['safe_return']['return_speed'] *= .5
    assert runner.site_configuration_digest(config) != before
    config['safe_return']['return_speed'] = float('nan')
    monkeypatch.setattr(URRTDEController, 'connect', lambda *a, **k: pytest.fail('hardware forbidden'))
    with pytest.raises(RobotError, match='must be finite'):
        runner.prepare_real(config, runner.ROOT/'config.yaml')


def test_transient_bias_timeout_restarts_entire_window_before_return(setup, monkeypatch):
    from sensor.px6d_reader import PX6DTimeout
    s = setup; calls = []; captured = []
    original = runner.WrenchPreprocessor.set_zero_bias
    def bias(self, samples):
        captured.append([w.fx for w in samples]); return original(self, samples)
    monkeypatch.setattr(runner.WrenchPreprocessor, 'set_zero_bias', bias)
    values = iter([.2, PX6DTimeout('test timeout'), .3, .1, .1, .1])
    def read():
        value = next(values, .1)
        if isinstance(value, Exception): raise value
        calls.append('read'); return Wrench(value, 0, 0, 0, 0, 0)
    s.reader.read_wrench = read
    def resync():
        assert not s.moves and not s.c.watchdog_active
        calls.append('resync')
    s.reader.resynchronize_after_timeout = resync
    assert run_return(s)
    assert calls.count('resync') == 1
    assert captured == [[.1, .1, .1]]  # pre-timeout and warmup data excluded
    assert len(json.loads((s.logger.run_dir/'startup_bias_retries.json').read_text())) == 1


def test_persistent_bias_timeout_is_bounded_and_never_moves(setup):
    from sensor.px6d_reader import PX6DTimeout
    s = setup; attempts = []
    def read():
        attempts.append(1); raise PX6DTimeout('test timeout')
    s.reader.read_wrench = read
    s.reader.resynchronize_after_timeout = lambda: None
    with pytest.raises(PX6DTimeout): run_return(s)
    assert len(attempts) == 3
    assert not s.moves and not s.c.watchdog_active


def test_timeout_during_return_remains_fail_fast(setup):
    from sensor.px6d_reader import PX6DTimeout
    s = setup; reads = itertools.count()
    def read():
        if next(reads) >= 3: raise PX6DTimeout('test motion-phase timeout')
        return Wrench(0, 0, 0, 0, 0, 0)
    s.reader.read_wrench = read
    s.reader.resynchronize_after_timeout = lambda: pytest.fail('retry outside stationary bias')
    with pytest.raises(RobotError, match='aborted'): run_return(s)
    assert not s.moves


def test_motion_during_bias_timeout_prevents_retry(setup):
    from sensor.px6d_reader import PX6DTimeout
    s = setup
    def read():
        s.pose[0] += .001
        raise PX6DTimeout('test timeout')
    s.reader.read_wrench = read
    s.reader.resynchronize_after_timeout = lambda: pytest.fail('moving robot must not retry')
    with pytest.raises(RobotError, match='moved during'): run_return(s)
    assert not s.moves


def test_serial_disconnect_is_not_retried(setup):
    from sensor.px6d_reader import PX6DError
    s = setup; calls = []
    def read():
        calls.append(1); raise PX6DError('serial disconnected')
    s.reader.read_wrench = read
    s.reader.resynchronize_after_timeout = lambda: pytest.fail('disconnect must stop')
    with pytest.raises(PX6DError, match='disconnected'): run_return(s)
    assert calls == [1] and not s.moves


def test_operator_can_cancel_timeout_recovery(setup):
    from sensor.px6d_reader import PX6DTimeout
    s = setup; timed_out = []
    def read():
        timed_out.append(1); raise PX6DTimeout('test timeout')
    s.reader.read_wrench = read
    s.reader.resynchronize_after_timeout = lambda: pytest.fail('cancelled recovery')
    with pytest.raises(KeyboardInterrupt):
        run_return(s, lambda: 'ESC' if timed_out else None)
    assert not s.moves


def test_resynchronization_timeouts_share_same_retry_budget(setup):
    from sensor.px6d_reader import PX6DTimeout
    s = setup; calls = []
    def fail():
        calls.append(1); raise PX6DTimeout('test timeout')
    s.reader.read_wrench = s.reader.resynchronize_after_timeout = fail
    with pytest.raises(PX6DTimeout, match='budget exhausted'): run_return(s)
    assert len(calls) == 3 and not s.moves


def test_startup_bias_total_time_budget_is_enforced(setup, monkeypatch):
    from sensor.px6d_reader import PX6DTimeout
    from core.models import RobotState
    s = setup
    class Clock:
        t = time.monotonic()
        def monotonic(self): return self.t
        def sleep(self, dt): self.t += dt
    clock = Clock()
    monkeypatch.setattr(runner, 'time', clock)
    s.c.read_diagnostic_state = lambda: RobotState(clock.t, s.pose.copy(), np.zeros(6))
    calls = []
    def read():
        calls.append(1)
        if len(calls) == 1: raise PX6DTimeout('test timeout')
        return Wrench(0, 0, 0, 0, 0, 0)
    s.reader.read_wrench = read
    s.reader.resynchronize_after_timeout = lambda: clock.sleep(2)
    with pytest.raises(PX6DTimeout, match='time budget'): run_return(s)
    assert not s.moves and not s.c.watchdog_active


def test_segment_braking_is_monitored_without_blocking_cycle_gap(setup):
    s = setup; stops = []; braking_until = [0.]; stop_times = []
    def stop(a, asynchronous=False):
        stops.append(asynchronous)
        if asynchronous:
            stop_times.append(time.monotonic())
            braking_until[0] = time.monotonic() + .04
        else:
            time.sleep(.04)  # reproduces measured inter-segment stopL delay
    s.c.control.stopL = stop
    s.c.receive.getActualTCPSpeed = lambda: ([0, 0, .003, 0, 0, 0]
        if time.monotonic() < braking_until[0] else [0]*6)
    original_move = s.c.control.moveL
    def move(*args):
        if stop_times:
            assert time.monotonic()-stop_times[-1] >= .12
        return original_move(*args)
    s.c.control.moveL = move
    assert run_return(s)
    assert stops == [True]*len(s.moves)
    assert len(s.kicks) > 10  # sensing/watchdog continues through braking


def test_segment_that_never_settles_cannot_start_next_move(setup):
    s = setup
    s.config['continuous_tracking']['confirmation_timeout_sec'] = .15
    def stop(a, asynchronous=False):
        s.c.receive.getActualTCPSpeed = lambda: [0, 0, .003, 0, 0, 0]
    s.c.control.stopL = stop
    with pytest.raises(RobotError, match='standstill confirmation timed out'):
        run_return(s)
    assert len(s.moves) == 1


@pytest.mark.parametrize('failure', ['force', 'nan', 'sensor_timeout', 'log_delay', 'stop_rejected'])
def test_braking_fault_never_advances_to_next_segment(setup, monkeypatch, failure):
    from sensor.px6d_reader import PX6DTimeout
    s = setup; braking = []
    def stop(a, asynchronous=False):
        if asynchronous: braking.append(1)
        if failure == 'stop_rejected' and asynchronous: return False
    s.c.control.stopL = stop
    def read():
        if braking:
            if failure == 'sensor_timeout': raise PX6DTimeout('test braking timeout')
            if failure in ('force', 'nan'):
                return Wrench(100 if failure == 'force' else float('nan'), 0, 0, 0, 0, 0)
        return Wrench(0, 0, 0, 0, 0, 0)
    s.reader.read_wrench = read
    original_log = s.logger.log_sample
    kicks_before = []
    def log(*args, **kwargs):
        if braking and failure == 'log_delay':
            kicks_before.append(len(s.kicks)); time.sleep(.04)
        return original_log(*args, **kwargs)
    monkeypatch.setattr(s.logger, 'log_sample', log)
    with pytest.raises(RobotError, match='aborted'): run_return(s)
    assert len(s.moves) == 1
    if kicks_before: assert len(s.kicks) == kicks_before[0]


def test_watchdog_deadline_cannot_be_reset_during_braking(setup):
    s = setup
    def stop(a, asynchronous=False):
        if asynchronous:
            s.c._watchdog_last_kick = time.monotonic()-.06
    s.c.control.stopL = stop
    with pytest.raises(RobotError, match='watchdog deadline exceeded'): run_return(s)
    assert len(s.moves) == 1


def test_segment_braking_can_be_cancelled(setup):
    s = setup; braking = []
    def stop(a, asynchronous=False):
        if asynchronous: braking.append(1)
    s.c.control.stopL = stop
    with pytest.raises(KeyboardInterrupt):
        run_return(s, lambda: 'ESC' if braking else None)
    s.c.stop()
    assert len(s.moves) == 1 and not s.c._return_mode


def test_settling_interruption_restarts_confirmation_hold(setup):
    s = setup; stop_times = []
    def stop(a, asynchronous=False):
        if asynchronous: stop_times.append(time.monotonic())
    def speed():
        elapsed = time.monotonic()-stop_times[-1] if stop_times else -1
        return [0, 0, .003, 0, 0, 0] if .03 <= elapsed < .06 else [0]*6
    s.c.control.stopL = stop
    s.c.receive.getActualTCPSpeed = speed
    original_move = s.c.control.moveL
    def move(*args):
        if stop_times: assert time.monotonic()-stop_times[-1] >= .14
        return original_move(*args)
    s.c.control.moveL = move
    assert run_return(s)


def test_aborted_return_ends_script_before_status_file_write(setup, monkeypatch):
    from sensor.px6d_reader import PX6DTimeout
    s = setup; count = itertools.count(); saved = []
    def read():
        if next(count) >= 3: raise PX6DTimeout('test return timeout')
        return Wrench(0,0,0,0,0,0)
    s.reader.read_wrench = read
    original = runner.ContinuousStartupReturn._save_status
    def save(self, result):
        assert result.status == 'aborted'
        assert not s.c.watchdog_active and s.c.control_script_stop_requested
        saved.append(1); return original(self, result)
    monkeypatch.setattr(runner.ContinuousStartupReturn, '_save_status', save)
    with pytest.raises(RobotError, match='aborted'): run_return(s)
    assert saved == [1]


def _return_test_clock(monkeypatch):
    """Advance device doubles without sleeping or changing process-wide time."""
    import robot.rtde_controller as rtde
    import safety.safe_return as safe_return

    class Clock:
        t = 10.

        def monotonic(self):
            return self.t

        def sleep(self, duration):
            self.t += duration

    clock = Clock()
    for module in (runner, rtde, safe_return):
        monkeypatch.setattr(module, 'time', clock)
    return clock


def test_slow_watchdog_kick_blocks_motion_after_return_precheck(setup, monkeypatch):
    s = setup
    clock = _return_test_clock(monkeypatch)
    s.c.begin_return_mode()
    s.c.enable_watchdog(20)

    def kick():
        clock.sleep(.045)
        s.kicks.append(clock.monotonic())
        return True

    s.c.control.kickWatchdog = kick
    executor = runner.ContinuousStartupReturn(
        s.config, s.start, s.c, s.reader, None, logger=s.logger, poll=lambda: None)
    with pytest.raises(RobotError, match='control cycle timeout'):
        executor._run_segment('MOVE_ABOVE_START', s.start, .018)
    assert len(s.kicks) == 1
    assert not s.moves
    assert s.config['continuous_tracking']['cycle_timeout_sec'] == .03


@pytest.mark.parametrize('continuous, phase, expected_error', [
    (True, 'DESCEND_TO_START', .00019),
    (True, 'MOVE_ABOVE_START', .000433),
    (False, 'DESCEND_TO_START', .000433),
])
def test_final_descent_reaches_scan_start_tolerance_before_braking(
        setup, monkeypatch, continuous, phase, expected_error):
    from safety.safe_return import SafeReturnExecutor

    s = setup
    clock = _return_test_clock(monkeypatch)
    s.pose[:] = s.start
    s.pose[2] += .005
    s.c.begin_return_mode()
    s.c.enable_watchdog(20)
    s.c.kick_watchdog()
    moving = [False]
    approach_errors = iter((.000433, .00019, .0001))
    stopped_errors = []

    def move(target, speed, acceleration, asynchronous):
        s.moves.append((np.array(target), speed))
        moving[0] = True
        return True

    def pose():
        if moving[0]:
            s.pose[:] = s.start
            s.pose[2] += next(approach_errors, .0001)
        return s.pose.copy()

    def stop(acceleration, asynchronous=False):
        stopped_errors.append(float(np.linalg.norm(s.pose[:3] - s.start[:3])))
        moving[0] = False

    def read():
        clock.sleep(.001)
        return Wrench(0, 0, 0, 0, 0, 0)

    s.c.control.moveL = move
    s.c.control.stopL = stop
    s.c.receive.getActualTCPPose = pose
    s.reader.read_wrench = read
    kwargs = {'poll': lambda: None} if continuous else {}
    cls = runner.ContinuousStartupReturn if continuous else SafeReturnExecutor
    executor = cls(s.config, s.start, s.c, s.reader, None, logger=s.logger, **kwargs)
    executor._run_segment(phase, s.start, .018)

    assert stopped_errors == pytest.approx([expected_error])
    assert s.config['safe_return']['return_position_tolerance'] == .0005
    assert s.config['policy']['position_tolerance'] == .0002
