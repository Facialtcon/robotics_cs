"""Bounded low-force pauses with synthetic Base-frame reaction forces, offline."""
from copy import deepcopy

import numpy as np
import pytest

from core.models import RobotState, Wrench
from policy.continuous_tracking import ContinuousTrackingPolicy, State


def sample(policy, now, magnitude, *, settled=True, direction_deg=0.):
    # Formal environment-on-probe sign: reaction -X, pressing/search +X.
    angle = np.deg2rad(direction_deg)
    wrench = Wrench(-magnitude*np.cos(angle), -magnitude*np.sin(angle), 0., 0., 0., 0.)
    speed = np.zeros(6) if settled else np.array([.003, 0., 0., 0., 0., 0.])
    return policy.update(now, wrench, wrench, RobotState(now, np.zeros(6), speed),
                         execution_settled=settled)


def tracking(config):
    value = deepcopy(config)
    value['continuous_real_execution'] = True  # Real policy rules, no real devices.
    policy = ContinuousTrackingPolicy(value)
    for i in range(101):
        sample(policy, i*.01, 1.5)
    assert policy.state == State.CONTINUOUS_TRACKING
    assert policy.c['force_direction_sign'] == -1
    return policy


@pytest.mark.parametrize('recovered_force', [.5, .75, .99])
@pytest.mark.parametrize('repeated_dips', [False, True])
def test_partial_low_force_recovery_enters_original_reacquire_at_fixed_deadline(
        config, recovered_force, repeated_dips):
    policy = tracking(config)
    memory = deepcopy(policy.last_reliable_contact)
    assert not sample(policy, 1.01, .4, settled=False).move
    started = policy._confirm_started
    timeout = policy.c['confirmation_timeout_sec']
    ticks = round(timeout/.01)
    for i in range(1, ticks):
        force = .4 if repeated_dips and i % 10 == 0 else recovered_force
        command = sample(policy, started+i*.01, force)
        assert policy.state == State.CONTINUOUS_TRACKING and not command.move
        assert policy._confirm_started == started  # Dips cannot restart this pause.
    command = sample(policy, started+timeout, recovered_force)
    assert policy.state == State.CONTACT_LOST and not command.move
    assert not policy._low_force_pending
    command = sample(policy, started+timeout+.01, recovered_force)
    assert policy.state == State.LOCAL_REACQUIRE and not command.move
    assert policy.last_reliable_contact['timestamp'] == memory['timestamp']
    assert started+timeout+.01-memory['timestamp'] <= policy.c['memory_max_age_sec']
    assert 'recovery_memory' not in policy.diagnostics
    np.testing.assert_array_equal(policy._memory_normal, memory['normal'])
    np.testing.assert_array_equal(policy._memory_tangent, memory['tangent'])
    assert [e.event_type for e in policy.events][-2:] == ['STANDSTILL_CONFIRMED', 'LOCAL_REACQUIRE']
    assert any(e.event_type == 'CONTACT_LOST' for e in policy.events)
    assert policy.p['contact_threshold'] == config['policy']['contact_threshold']
    assert policy.c['contact_lost_threshold'] == config['continuous_tracking']['contact_lost_threshold']
    assert sample(policy, started+timeout+.02, recovered_force).move


def test_low_force_contact_confirmed_before_deadline_resumes_tracking(config):
    policy = tracking(config)
    assert not sample(policy, 1.01, .4, settled=False).move
    for i in range(102, 108):
        assert not sample(policy, i*.01, .75).move
    confirmed = None
    for i in range(108, 130):
        command = sample(policy, i*.01, 1.0)
        if not policy._low_force_pending:
            confirmed = i*.01
            assert not command.move  # Confirmation sample stays stopped.
            break
    assert confirmed is not None
    assert policy.state == State.CONTINUOUS_TRACKING and policy._confirm_started is None
    assert policy.events[-1].event_type == 'LOW_FORCE_RECONFIRMED'
    assert not any(e.event_type == 'CONTACT_LOST' for e in policy.events)
    assert sample(policy, confirmed+.01, 1.0).move


def test_low_force_timeout_never_explores_without_physical_stop_confirmation(config):
    policy = tracking(config)
    sample(policy, 1.01, .4, settled=False)
    deadline = 1.01+policy.c['confirmation_timeout_sec']
    for i in range(102, round(deadline*100)+1):
        assert not sample(policy, i*.01, .75, settled=False).move
    assert policy.state == State.CONTACT_LOST
    # Force now crosses the contact threshold, but actual motion must still
    # finish stopping; this must not reopen an unlimited confirmation wait.
    stop_deadline = deadline+policy.c['confirmation_timeout_sec']
    for i in range(round(deadline*100)+1, round(stop_deadline*100)+2):
        command = sample(policy, i*.01, 1.2, settled=False)
        assert not command.move
        if policy.state == State.STOP:
            break
    assert policy.state == State.STOP
    assert policy.reason == 'contact lost stop confirmation timeout'
    assert not any(e.event_type == 'LOCAL_REACQUIRE' for e in policy.events)


@pytest.mark.parametrize('timeout,memory_age,valid', [(1., .5, False), (1., 1.5, True),
                                                   (2., 1.5, False), (2., 2.5, True)])
def test_memory_configuration_covers_the_entire_low_force_wait(config, timeout, memory_age, valid):
    config['continuous_tracking'].update(confirmation_timeout_sec=timeout, memory_max_age_sec=memory_age)
    if valid:
        ContinuousTrackingPolicy(config)
    else:
        with pytest.raises(ValueError, match='memory_max_age_sec must cover'):
            ContinuousTrackingPolicy(config)


def test_first_contact_cannot_wait_forever_for_physical_stop(config):
    config['continuous_real_execution'] = True
    policy = ContinuousTrackingPolicy(config)
    assert sample(policy, 0., 1.2, settled=False).state == 'FIRST_CONTACT'
    for i in range(1, 102):
        command = sample(policy, i*.01, 1.2, settled=False)
        assert not command.move
        if policy.state == State.STOP:
            break
    assert policy.state == State.STOP
    assert policy.reason == 'contact/standstill confirmation timeout'


@pytest.mark.parametrize('magnitude', [.5, .75, .99])
@pytest.mark.parametrize('angles', [(60., -60.), (0., 180., 90., -90.)])
def test_partial_recovery_direction_jumps_cannot_interrupt_or_extend_low_force_wait(config, magnitude, angles):
    policy = tracking(config)
    memory = deepcopy(policy.last_reliable_contact)
    sample(policy, 1.01, .4, settled=False)
    started = policy._confirm_started
    deadline = started+policy.c['confirmation_timeout_sec']
    old_measurement = policy._last_valid_measurement.copy()
    old_measurement_time = policy._last_valid_measurement_time
    for i in range(1, 100):
        command = sample(policy, started+i*.01, magnitude, direction_deg=angles[i % len(angles)])
        assert not command.move and policy.state == State.CONTINUOUS_TRACKING
        assert policy._low_force_pending and not policy.direction_valid
        assert policy._confirm_started == started
        np.testing.assert_array_equal(policy._last_valid_measurement, old_measurement)
        assert policy._last_valid_measurement_time == old_measurement_time
        assert policy.last_reliable_contact['timestamp'] == memory['timestamp']
    command = sample(policy, deadline, magnitude, direction_deg=angles[0])
    assert not command.move and policy.state == State.CONTACT_LOST
    assert policy._confirm_started == started
    command = sample(policy, deadline+.01, magnitude, direction_deg=angles[-1])
    assert not command.move and policy.state == State.LOCAL_REACQUIRE
    np.testing.assert_array_equal(policy._memory_normal, memory['normal'])
    np.testing.assert_array_equal(policy._memory_tangent, memory['tangent'])
    assert policy.direction_reconfirm_count == 0 and policy.stop_reason is None
    assert not any(e.event_type == 'DIRECTION_STOP_REQUEST' for e in policy.events)


def test_unstable_above_threshold_recovery_uses_original_deadline(config):
    policy = tracking(config)
    sample(policy, 1.01, .4, settled=False)
    started = policy._confirm_started
    deadline = started+policy.c['confirmation_timeout_sec']
    # Each jump is below the unchanged 150-degree reversal stop. The stable
    # contact window lacks coherence, so it cannot confirm a new direction.
    for i in range(1, 100):
        command = sample(policy, started+i*.01, 1.1, direction_deg=60.*(-1)**i)
        assert not command.move and policy.state == State.CONTINUOUS_TRACKING
        assert policy._low_force_pending and policy._confirm_started == started
    assert sample(policy, deadline, 1.1, direction_deg=60.).state == 'CONTACT_LOST'
    assert sample(policy, deadline+.01, 1.1, direction_deg=-60.).state == 'LOCAL_REACQUIRE'
    assert policy.direction_reconfirm_count == 0 and policy.stop_reason is None


def test_recovery_direction_confirms_only_after_contact_threshold_is_reached(config):
    policy = tracking(config)
    sample(policy, 1.01, .4, settled=False)
    started = policy._confirm_started
    for i in range(102, 115):
        assert not sample(policy, i*.01, .75, direction_deg=90.*(-1)**i).move
        assert policy._low_force_pending and policy._confirm_started == started
    # At exactly the existing threshold, a sustained coherent 60-degree
    # direction can be confirmed while stopped, within the original deadline.
    for i in range(115, 135):
        command = sample(policy, i*.01, 1., direction_deg=60.)
        assert not command.move and policy.state == State.CONTINUOUS_TRACKING
        if not policy._low_force_pending:
            break
        assert policy._confirm_started == started
    assert not policy._low_force_pending and policy._confirm_started is None
    assert policy.events[-1].event_type == 'LOW_FORCE_RECONFIRMED'
    np.testing.assert_allclose(policy.contact_direction, [.5, np.sqrt(.75)])
    assert policy.direction_reconfirm_count == 0


def test_reliable_force_reversal_still_stops_during_low_force_recovery(config):
    policy = tracking(config)
    sample(policy, 1.01, .4, settled=False)
    command = sample(policy, 1.02, 1.1, direction_deg=180.)
    assert not command.move and policy.state == State.STOP
    assert policy.stop_reason.value == 'STOP_DIRECTION_REVERSAL'
