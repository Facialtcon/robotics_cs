"""Four-stage returns and preflight/physical-stop separation, without devices."""
import numpy as np
import pytest

from app import return_runtime, scan_startup
from doubles import Devices, Sensor
from experiment_logging.paths import PROJECT_ROOT
from robot.rtde_controller import RobotError
from safety.safe_return import SafeReturnExecutor, PX6DForceMonitor, return_trajectory
from sensor.force_preprocess import WrenchPreprocessor
from test_real_tracking_guards import clock, real


def misalign(devices, start):
    devices.receive.pose[:] = start
    devices.receive.pose[0] += .003
    devices.receive.pose[3:] = 0.


def test_startup_lifts_aligns_translates_descends_with_stops(real, clock, capsys):
    config, start, devices, owner = real
    misalign(devices, start)
    initial = devices.receive.pose.copy()
    plan = return_trajectory(config, initial, start)
    moves = []
    original = devices.control.moveL
    # The owner marks RUNNING before the SDK call; the preceding stop report
    # must still prove that the measured hold completed.
    def checked_move(*args):
        assert owner.stop_state == 'RUNNING'
        assert owner.stop_history[-1]['physical_stop'] == 'confirmed'
        moves.append((devices.receive.pose.copy(), np.asarray(args[0]), args[1], clock.now))
        return original(*args)
    devices.control.moveL = checked_move
    def confirm(_):
        text = capsys.readouterr().out
        assert devices.control_count == 0
        if '零偏' not in _:
            return ''
        for label in ('Current orientation:', 'Target scan orientation:', 'Orientation error:',
                      '1. VERTICAL_RETREAT', '2. ALIGN_PROBE_ORIENTATION',
                      '3. MOVE_ABOVE_START', '4. DESCEND_TO_START',
                      'Probe will be aligned to the calibrated scan orientation at safe height.'):
            assert label in text
        return ''
    assert scan_startup.startup_scan(config, start, owner, Sensor(), None, lambda: None, confirm=confirm)
    assert len(moves) == 4
    fixed = config['continuous_calibration']['fixed_orientation']
    np.testing.assert_array_equal(moves[0][1][:2], initial[:2])
    np.testing.assert_array_equal(moves[0][1][3:], initial[3:])
    np.testing.assert_array_equal(moves[1][1][:3], moves[1][0][:3])
    assert moves[1][0][2] == pytest.approx(plan[0][1][2])
    for _, target, _, _ in moves[1:]: np.testing.assert_array_equal(target[3:], fixed)
    np.testing.assert_array_equal(moves[-1][1], start)
    assert [m[2] for m in moves] == [.018, .018, .027, .018]
    assert config['robot']['max_tcp_speed'] == .03
    calls = [c[0] for c in devices.control.calls if c[0] in ('moveL', 'stopL')]
    assert calls == ['moveL', 'stopL']*4


def test_already_aligned_skips_rotation(real, clock, capsys):
    config, start, devices, owner = real
    devices.receive.pose[0] += .003
    owner.activate_control(confirmed=True)
    result = SafeReturnExecutor(config, start, owner).execute()
    assert result.status == 'complete'
    assert 'ALIGN_PROBE_ORIENTATION: already aligned; skipped' in capsys.readouterr().out
    moves = [c[1] for c in devices.control.calls if c[0] == 'moveL']
    assert len(moves) == 3
    for target in moves: np.testing.assert_array_equal(target[3:], start[3:])


@pytest.mark.parametrize('failed_segment', [0, 1])
def test_retreat_or_alignment_failure_prevents_later_segments(real, clock, failed_segment):
    config, start, devices, owner = real
    misalign(devices, start)
    owner.activate_control(confirmed=True)
    move = devices.control.moveL
    targets = []
    def fail(target, *args):
        targets.append(np.asarray(target))
        if len(targets)-1 == failed_segment: return False
        return move(target, *args)
    devices.control.moveL = fail
    result = SafeReturnExecutor(config, start, owner).execute()
    assert result.status == 'aborted'
    assert len(targets) == failed_segment+1
    if failed_segment == 0:
        np.testing.assert_array_equal(targets[0][3:], np.zeros(3))
    else:
        assert targets[1][2] > start[2]
        np.testing.assert_array_equal(targets[1][:3], targets[0][:3])


def test_retreat_stop_failure_cannot_rotate(real, clock):
    config, start, devices, owner = real
    misalign(devices, start)
    owner.activate_control(confirmed=True)
    move = devices.control.moveL
    def moving(*args):
        result = move(*args)
        devices.receive.speed[0] = .0008
        devices.control.keep_moving = True
        return result
    devices.control.moveL = moving
    result = SafeReturnExecutor(config, start, owner).execute()
    assert result.status == 'aborted' and 'did not settle' in result.abort_reason
    assert len([c for c in devices.control.calls if c[0] == 'moveL']) == 1
    assert owner.stop_state == 'FAILED'
    with pytest.raises(RobotError, match='did not settle'): owner.close()


def test_low_retreat_endpoint_cannot_authorize_alignment(real, clock):
    config, start, devices, owner = real
    misalign(devices, start)
    owner.activate_control(confirmed=True)
    move = devices.control.moveL
    def below_safe_height(*args):
        result = move(*args)
        devices.receive.pose[2] -= .0008  # Within startup's 1 mm, below the safe-height tolerance.
        return result
    devices.control.moveL = below_safe_height
    monitor = PX6DForceMonitor(config, Sensor(), WrenchPreprocessor.from_config(config['preprocessing']))
    deadline = clock.now+2.
    result = SafeReturnExecutor(config, start, owner, force_monitor=monitor,
                                poll=lambda: 'Q' if clock.now > deadline else None).execute()
    assert result.status == 'aborted' and 'safe retreat height' in result.abort_reason
    assert len([c for c in devices.control.calls if c[0] == 'moveL']) == 1


def test_aligned_startup_return_skips_rotation_within_position_tolerance(real, clock, capsys):
    config, start, devices, owner = real
    devices.receive.pose[0] += .003
    owner.activate_control(confirmed=True)
    move = devices.control.moveL
    count = 0
    def retreat_error(*args):
        nonlocal count
        count += 1
        result = move(*args)
        if count == 1: devices.receive.pose[2] -= .0008
        return result
    devices.control.moveL = retreat_error
    monitor = PX6DForceMonitor(config, Sensor(), WrenchPreprocessor.from_config(config['preprocessing']))
    deadline = clock.now+2.
    result = SafeReturnExecutor(config, start, owner, force_monitor=monitor,
                                poll=lambda: 'Q' if clock.now > deadline else None).execute()
    assert result.status == 'complete' and count == 3
    assert 'ALIGN_PROBE_ORIENTATION: already aligned; skipped' in capsys.readouterr().out


@pytest.mark.parametrize('axis,high,low', [(0, .0008, .0001), (3, .008, .005)])
def test_return_waits_for_strict_speed_and_80ms_before_next_segment(real, clock, axis, high, low):
    config, start, devices, owner = real
    misalign(devices, start)
    owner.activate_control(confirmed=True)
    move = devices.control.moveL
    pending = None
    previous_low_at = None
    count = 0
    def move_with_slow_braking(*args):
        nonlocal pending, previous_low_at, count
        if previous_low_at is not None:
            assert clock.now-previous_low_at >= .08-1e-9
        result = move(*args)
        pending = clock.now+.12
        previous_low_at = pending
        count += 1
        return result
    def speed():
        velocity = np.zeros(6)
        if pending is not None:
            velocity[axis] = high if clock.now < pending else low
        return velocity
    devices.control.moveL = move_with_slow_braking
    devices.receive.getActualTCPSpeed = speed
    executor = SafeReturnExecutor(config, start, owner)
    observe = executor.observe
    high_observations = []
    def observed():
        state = observe()
        if pending is not None and clock.now < pending:
            assert not owner.standstill_confirmed
            high_observations.append(state.tcp_speed[axis])
        return state
    executor.observe = observed
    result = executor.execute()
    assert result.status == 'complete' and count == 4
    assert high_observations and all(v == high for v in high_observations)


def test_default_wait_is_strict_but_explicit_preflight_accepts_noise(real, clock):
    _, _, devices, owner = real
    devices.receive.speed[0] = .0003
    with pytest.raises(RobotError, match='did not settle'):
        owner.wait_for_standstill(startup=False, timeout=.1)
    owner.wait_for_standstill(startup=True)
    assert owner.standstill_confirmed
    owner.begin_return_mode()
    with pytest.raises(RobotError, match='did not settle'):
        owner.wait_for_standstill(startup=True, timeout=.1)
    owner.end_return_mode()


def test_startup_flag_cannot_relax_a_started_scan(real, clock):
    _, _, devices, owner = real
    owner.activate_control(confirmed=True)
    owner.set_continuous_phase('TARGET_SEARCH')
    owner.command_planar_velocity([1, 0], .0008, .01)
    with pytest.raises(RobotError, match='did not settle'):
        owner.wait_for_standstill(startup=True, timeout=.1)
    assert not owner.standstill_confirmed


def test_manual_return_uses_same_four_segment_alignment(config, clock, tmp_path):
    target = np.asarray(return_runtime.load_return_target(PROJECT_ROOT/'config.yaml', config)['pose'])
    devices = Devices(config, target)
    misalign(devices, target)
    assert return_runtime.run(controller_factory=devices.controller, confirm=lambda _: '', data_root=tmp_path) == 0
    moves = [c[1] for c in devices.control.calls if c[0] == 'moveL']
    assert len(moves) == 4
    np.testing.assert_array_equal(moves[0][3:], np.zeros(3))
    np.testing.assert_array_equal(moves[1][:3], moves[0][:3])
    np.testing.assert_array_equal(moves[1][3:], target[3:])
    np.testing.assert_array_equal(moves[-1], target)
