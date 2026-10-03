"""Exercise the actual execution owner with injected SDKs; no devices."""
import numpy as np
import pytest

from robot.rtde_controller import RobotError
from test_real_tracking_guards import clock, real


def test_contact_stop_then_resume_records_exact_speedl_without_overwrite(real, clock):
    config, start, devices, owner = real
    owner.activate_control(confirmed=True)
    owner.set_continuous_phase('TARGET_SEARCH')
    owner.command_planar_velocity([0, -1], .001, .01)
    assert owner.last_speedl['kind'] == 'motion'
    np.testing.assert_allclose(owner.last_speedl['velocity'], [0, -.001, 0, 0, 0, 0])
    owner.set_continuous_phase('FIRST_CONTACT')
    owner.request_stop(nonblocking=True)
    stopped_sequence = owner.speedl_count
    assert owner.last_speedl['kind'] == 'braking'
    np.testing.assert_array_equal(owner.last_speedl['velocity'], np.zeros(6))
    with pytest.raises(RobotError, match='standstill'):
        owner.command_planar_velocity([1, 0], .001, .01)
    assert owner.speedl_count == stopped_sequence
    owner.wait_for_standstill()
    owner.set_continuous_phase('CONTINUOUS_TRACKING', stop_confirmed=True)
    owner.command_planar_velocity([1, 0], .001, .01)
    assert owner.speedl_count == stopped_sequence+1
    assert owner.last_speedl['kind'] == 'motion' and owner.last_speedl['accepted'] is True
    assert owner.stop_state == 'RUNNING' and not owner._stop_pending
    np.testing.assert_allclose(devices.receive.speed, [.001, 0, 0, 0, 0, 0])
    # An ordinary subsequent state observation cannot reissue the old stop.
    owner.read_state()
    owner.poll_stop()
    assert owner.speedl_count == stopped_sequence+1
    assert devices.control.calls[-1] == ('speedL', [.001, 0, 0, 0, 0, 0])
