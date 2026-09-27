"""Force-rate scope only: real policy with synthetic observations, no devices."""
from pathlib import Path
import csv

import numpy as np
import pytest

from config.loader import load_config
from core.models import RobotState, Wrench
from experiment_logging.termination import TerminationReason
from policy.continuous_tracking import ContinuousTrackingPolicy, State, EXTRA_SAMPLE_FIELDS

ROOT = Path(__file__).resolve().parents[1]
POSE = np.zeros(6)


def tick(policy, now, force, speed=0., raw=None):
    processed = Wrench(force, 0, 0, 0, 0, 0)
    return policy.update(now, processed if raw is None else raw, processed,
                         RobotState(now, POSE.copy(), np.array([speed, 0, 0, 0, 0, 0])))


def in_phase(state):
    config = load_config(ROOT/'config.yaml')
    config['continuous_tracking']['reacquire_enabled'] = True  # offline fixture only
    p = ContinuousTrackingPolicy(config)
    p.state = state
    p._started = p._last_time = 0.
    p._start_pose = POSE.copy()
    p._rate.update(0., .4)
    p.contact_direction = np.array([1., 0.])
    p.tangent = np.array([0., -1.])
    p.direction_confidence = 1.
    p._remember(0., POSE)
    if state in (State.FIRST_CONTACT, State.DIRECTION_RECONFIRM, State.CONTACT_LOST):
        p._reset_confirmation(0.)
    if state == State.LOCAL_REACQUIRE:
        p._begin_recovery(0., POSE)
    return p


@pytest.mark.parametrize('state', [State.READY, State.TARGET_SEARCH, State.FIRST_CONTACT,
                                   State.DIRECTION_RECONFIRM, State.CONTACT_LOST, State.LOCAL_REACQUIRE])
def test_contact_acquisition_and_stopped_phases_record_rate_without_terminal_trip(state):
    p = in_phase(state)
    command = tick(p, .01, 1.2, speed=.018)
    assert p.force_rate == pytest.approx(80.)
    assert p.state == (State.FIRST_CONTACT if state in (State.READY, State.TARGET_SEARCH) else state)
    assert not command.move and p.stop_requested
    assert p.stop_reason != TerminationReason.STOP_FORCE_LIMIT
    assert p.telemetry(command)['force_rate_guard_active'] == 0
    assert p.telemetry(command)['force_rate'] == pytest.approx(80.)


@pytest.mark.parametrize('state', [state for state in State if state != State.STOP])
@pytest.mark.parametrize('kind', ['processed_force', 'processed_torque', 'raw_force', 'raw_torque'])
def test_hard_wrench_limits_precede_contact_in_every_nonterminal_phase(state, kind):
    p = in_phase(state)
    processed = Wrench(1.2, 0, 0, 0, 0, 0)
    raw = processed
    if kind == 'processed_force':
        processed = Wrench(13, 0, 0, 0, 0, 0)
    elif kind == 'processed_torque':
        processed = Wrench(1.2, 0, 0, 1.1, 0, 0)
    elif kind == 'raw_force':
        raw = Wrench(61, 0, 0, 0, 0, 0)
    else:
        raw = Wrench(1.2, 0, 0, 0, 0, 5.1)
    command = p.update(.01, raw, processed, RobotState(.01, POSE.copy(), np.zeros(6)))
    assert p.state == State.STOP and not command.move
    assert p.stop_reason == TerminationReason.STOP_FORCE_LIMIT
    assert ('absolute raw' if kind.startswith('raw') else 'processed') in p.reason
    assert p.first_threshold_pose is None


def test_search_rate_below_contact_threshold_is_diagnostic_only():
    p = in_phase(State.TARGET_SEARCH)
    command = tick(p, .01, .9, speed=.018)
    assert p.force_rate == pytest.approx(50.)
    assert p.state == State.TARGET_SEARCH and command.move
    assert p.telemetry(command)['force_rate_guard_active'] == 0


def test_high_onset_then_braking_then_confirmed_tracking_and_rate_trip():
    p = ContinuousTrackingPolicy(load_config(ROOT/'config.yaml'))
    for i in range(201):
        command = tick(p, i*.01, .4, speed=.018)
    assert command.speed == pytest.approx(.018)
    command = tick(p, 2.01, 1.2, speed=.018)
    assert p.force_rate == pytest.approx(80.)
    assert p.state == State.FIRST_CONTACT and not command.move
    assert p.stop_requested and p.first_threshold_pose is not None
    for i, speed in zip(range(202, 206), [.018, .014, .008, .003]):
        command = tick(p, i*.01, 1.7, speed=speed)
        assert p.state == State.FIRST_CONTACT and not command.move
        assert not p.force_rate_guard_active and not p.stop_confirmed
    for i in range(206, 214):
        assert not tick(p, i*.01, 1.7).move
        assert p.state == State.FIRST_CONTACT
    command = tick(p, 2.14, 1.7)
    assert p.state == State.CONTINUOUS_TRACKING and p.stop_confirmed
    assert not command.move and not p.force_rate_guard_active
    command = tick(p, 2.15, 1.7)
    assert command.move and p.force_rate_guard_active
    command = tick(p, 2.16, 2.2)
    assert p.state == State.STOP and not command.move
    assert p.stop_reason == TerminationReason.STOP_FORCE_LIMIT
    assert 'force rate exceeded' in p.reason
    assert p.telemetry(command)['force_rate_guard_active'] == 1  # decision for this sample


def test_tracking_positive_rate_guard_still_uses_original_limit():
    p = in_phase(State.CONTINUOUS_TRACKING)
    assert p.p['force_rate_limit'] == 30.
    command = tick(p, .01, 1.2)
    assert not command.move and p.state == State.STOP
    assert p.force_rate == pytest.approx(80.)
    assert p.stop_reason == TerminationReason.STOP_FORCE_LIMIT
    assert p.force_rate_guard_active


def test_first_tracking_sample_uses_previous_confirmation_force_without_reset():
    p = in_phase(State.FIRST_CONTACT)
    for i in range(1, 10):
        command = tick(p, i*.01, 1.2)
    assert p.state == State.CONTINUOUS_TRACKING and not command.move
    assert not p.force_rate_guard_active
    command = tick(p, .10, 1.7)
    assert p.state == State.STOP and not command.move
    assert p.force_rate == pytest.approx(50.) and p.force_rate_guard_active


def test_tracking_low_force_pause_is_reacquisition_until_confirmation_finishes():
    p = in_phase(State.CONTINUOUS_TRACKING)
    assert not tick(p, .01, .4).move
    assert p._low_force_pending
    command = tick(p, .02, 1.2, speed=.001)
    assert p.state == State.CONTINUOUS_TRACKING and not command.move
    assert p.force_rate == pytest.approx(80.) and not p.force_rate_guard_active
    for i in range(3, 12):
        command = tick(p, i*.01, 1.2)
        assert not command.move and not p.force_rate_guard_active
    assert not p._low_force_pending
    assert tick(p, .12, 1.2).move and p.force_rate_guard_active


def test_local_reacquire_spike_freezes_search_reference_and_confirms_before_motion():
    p = in_phase(State.LOCAL_REACQUIRE)
    command = tick(p, .01, .4)
    assert command.move
    reference = p.reacquire_reference.copy()
    theta = p._theta
    command = tick(p, .02, 1.2, speed=.0005)
    assert not command.move and p.state == State.LOCAL_REACQUIRE
    assert p.force_rate == pytest.approx(80.) and not p.force_rate_guard_active
    assert p.events[-1].event_type == 'REACQUIRE_STOP_REQUEST'
    for i in range(3, 12):
        command = tick(p, i*.01, 1.2)
        assert not command.move
        np.testing.assert_array_equal(p.reacquire_reference, reference)
        assert p._theta == theta
    assert p.state == State.CONTINUOUS_TRACKING and p.stop_confirmed
    assert not p.force_rate_guard_active
    assert tick(p, .12, 1.2).move and p.force_rate_guard_active


def test_stop_remains_terminal_and_does_not_consume_new_force_samples():
    p = in_phase(State.CONTINUOUS_TRACKING)
    command = tick(p, .01, 1.2)
    assert p.state == State.STOP and p.force_rate_guard_active
    last_rate, last_sample = p.force_rate, p._rate.previous
    command = tick(p, .02, 2.)
    assert not command.move and p.state == State.STOP
    assert not p.force_rate_guard_active
    assert p.force_rate == last_rate and p._rate.previous == last_sample


def test_rate_telemetry_and_legacy_csv_replay(tmp_path):
    from experiment_logging.data_logger import ExperimentLogger
    from tools.visualize_continuous_run import read_run, make_figure
    from robot.rtde_controller import SPEED_GUARD_FIELDS
    import matplotlib.pyplot as plt
    cfg = load_config(ROOT/'config.yaml')
    p = ContinuousTrackingPolicy(cfg)
    logger = ExperimentLogger(tmp_path, cfg, extra_sample_fields=EXTRA_SAMPLE_FIELDS+SPEED_GUARD_FIELDS,
                              workspace_logging=False)
    for i in range(13):
        force = .4 if i == 0 else 1.2
        command = tick(p, i*.01, force)
        wrench = Wrench(force,0,0,0,0,0)
        logger.log_sample(i*.01, wrench, wrench, RobotState(i*.01,POSE,np.zeros(6)),
                          command,p.contact_direction,p.tangent,extra=p.telemetry(command))
    logger.close()
    path = logger.run_dir/'samples.csv'
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    assert float(rows[1]['force_rate']) == pytest.approx(80.)
    assert rows[1]['force_rate_guard_active'] == '0'
    assert rows[-1]['force_rate_guard_active'] == '1'
    def replay():
        data,config,events=read_run(logger.run_dir)
        fig,update=make_figure(data,config,events)
        update(len(data['time'])-1)
        plt.close(fig)
        return data
    current = replay()
    fields = [key for key in rows[0] if key != 'force_rate_guard_active']
    with path.open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields,extrasaction='ignore')
        writer.writeheader(); writer.writerows(rows)
    legacy = replay()
    np.testing.assert_array_equal(current['time'],legacy['time'])
