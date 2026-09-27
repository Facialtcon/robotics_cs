"""Real speed guard with SDK doubles; runner writes only to pytest tmp_path."""
import json

import numpy as np
import pytest

import run_continuous_tracking as runner
from robot.rtde_controller import RobotError
from test_continuous_execution import controller
from test_continuous_run import fake_real


BASE_LIMIT = float(np.hypot(.001, .0005))
TRIP_LIMIT = 1.2 * BASE_LIMIT


def fault_observation():
    device = controller()
    device.config.update(continuous_speed_limits=dict(TARGET_SEARCH=.018,
        CONTINUOUS_TRACKING=BASE_LIMIT, LOCAL_REACQUIRE=.0005),
        continuous_search_boundary_margin=.004, continuous_tracking_boundary_margin=.0005)
    device.set_continuous_phase('CONTINUOUS_TRACKING')
    pose = [.01, .02, 0, 0, 0, 0]
    speed = [0, -.002, .001, 0, 0, 0]  # severe tracking overrun: immediate stop
    reads = []
    device.receive.getActualTCPPose = lambda: (reads.append('pose') or pose)
    device.receive.getActualTCPSpeed = lambda: (reads.append('speed') or speed)
    device.control.speedL = lambda *a: pytest.fail('fault must block speedL')
    with pytest.raises(RobotError, match='continuous measured speed') as caught:
        device.command_planar_velocity([0, -1], .001, .01)
    assert reads == ['pose', 'speed']  # No second device read for diagnostics.
    return caught.value, pose, speed


def test_fault_captures_rejected_read_before_stop_can_replace_it():
    error, pose, speed = fault_observation()
    observation = error.speed_limit_observation
    assert observation['tcp_pose'] == pose
    assert observation['tcp_speed'] == speed
    assert observation['measured_xyz_speed_mps'] == pytest.approx(np.sqrt(5) * .001)
    assert observation['speed_guard_state'] == 'HARD_TRIP'
    assert observation['speed_guard_phase'] == 'CONTINUOUS_TRACKING'
    assert observation['configured_speed_limit_mps'] == BASE_LIMIT
    assert observation['trip_limit_mps'] == TRIP_LIMIT
    assert observation['tolerance_factor'] == 1.2
    assert observation['source'] == 'rtde_state_read_rejected_by_continuous_speed_guard'
    assert observation['host_read_start_monotonic_sec'] > 0
    assert observation['last_checked_device_timestamp_sec'] == 1.
    assert 'measured_xyz=' in str(error) and 'trip_limit=' in str(error)
    pose[:] = [0] * 6
    speed[:] = [0] * 6
    assert observation['tcp_pose'][0] == .01
    assert observation['tcp_speed'][2] == .001


@pytest.mark.parametrize('value,rejected', [
    (np.nextafter(TRIP_LIMIT, 0), False),
    (TRIP_LIMIT, False),
    (np.nextafter(TRIP_LIMIT, np.inf), True),
])
def test_legacy_unphased_controller_callers_keep_strict_boundary(value, rejected):
    device = controller()
    device.config['continuous_speed_limit'] = BASE_LIMIT
    device.receive.getActualTCPSpeed = lambda: [0, 0, value, 0, 0, 0]
    if rejected:
        with pytest.raises(RobotError, match='continuous measured speed'):
            device.read_state()
    else:
        assert device.read_state().tcp_speed[2] == value


def test_return_exemption_and_general_maximum_still_apply():
    device = controller()
    device.config['continuous_speed_limit'] = BASE_LIMIT
    device._return_mode = True
    device.receive.getActualTCPSpeed = lambda: [0, 0, .002, 0, 0, 0]
    assert device.read_state().tcp_speed[2] == .002
    device.receive.getActualTCPSpeed = lambda: [0, 0, .04, 0, 0, 0]
    with pytest.raises(RobotError, match='measured TCP speed exceeds maximum'):
        device.read_state()


def test_runner_stops_before_recording_fault_and_keeps_post_stop_separate(fake_real, monkeypatch):
    arguments, _, calls, fake_controller, _ = fake_real
    error, _, _ = fault_observation()

    def fail(*args):
        calls.append('fault')
        raise error

    original_observe = runner.TerminationRecorder.observe

    def observe(self, **kwargs):
        if 'speed_limit_observation' in kwargs:
            calls.append('record_fault')
        return original_observe(self, **kwargs)

    monkeypatch.setattr(fake_controller, 'command_planar_velocity', fail)
    monkeypatch.setattr(runner.TerminationRecorder, 'observe', observe)
    assert runner.run(arguments) == 1
    fault_index = calls.index('fault')
    assert calls[fault_index + 1:fault_index + 3] == ['stop', 'record_fault']
    assert 'motion' not in calls
    run_dir = next(arguments.output.glob('run_*'))
    record = json.loads((run_dir / 'termination.json').read_text())
    assert record['reason'] == 'STOP_MOTION_ERROR'
    assert record['speed_limit_observation'] == error.speed_limit_observation
    assert record['context']['speed_limit_observation'] == error.speed_limit_observation
    assert record['tcp_speed'] == [0] * 6  # Previous accepted observation.
    stop = json.loads((run_dir / 'scan_stop_snapshot.json').read_text())
    assert stop['tcp_speed'] == [0] * 6  # Fresh post-stop observation.
    assert error.speed_limit_observation['tcp_speed'][2] == .001
