"""Phase execution with real policy/controller and fake receive/control only."""
from types import SimpleNamespace

import numpy as np
import pytest

from config.loader import load_config
from core.models import RobotState, Wrench
from policy.continuous_tracking import ContinuousTrackingPolicy, State
from robot.rtde_controller import (RobotError, ContinuousSpeedLimitError,
    CONTINUOUS_TRIP_FACTOR, CONTINUOUS_HARD_FACTOR, CONTINUOUS_DEBOUNCE_SEC)
from run_continuous_tracking import ROOT, prepare_real
from test_continuous_execution import controller
from test_continuous_saved_calibration import saved


@pytest.fixture
def phased(monkeypatch):
    import robot.rtde_controller as module
    c = controller()
    clock = SimpleNamespace(now=10., stamp=1., speed=.018)
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock.now)
    c.config.update(continuous_speed_limits=dict(TARGET_SEARCH=.018,
        CONTINUOUS_TRACKING=float(np.hypot(.001, .0005)), LOCAL_REACQUIRE=.0005),
        continuous_search_boundary_margin=.004, continuous_tracking_boundary_margin=.0005)
    c.receive.getTimestamp = lambda: clock.stamp
    c.receive.getActualTCPSpeed = lambda: [0, clock.speed, 0, 0, 0, 0]
    c.set_continuous_phase('TARGET_SEARCH')
    return c, clock


def advance(clock, speed, dt=.01, fresh=True):
    clock.now += dt
    if fresh:
        clock.stamp += dt
    clock.speed = speed


def test_preflight_separates_envelopes_and_search_clearance(saved):
    cfg, path, _, _ = saved
    robot, _ = prepare_real(cfg, path)
    limits = robot['continuous_speed_limits']
    assert limits == dict(TARGET_SEARCH=.018, CONTINUOUS_TRACKING=np.hypot(.001,.0005), LOCAL_REACQUIRE=.0005)
    assert 'continuous_speed_limit' not in robot
    hard = CONTINUOUS_HARD_FACTOR * .018
    expected = hard * (CONTINUOUS_DEBOUNCE_SEC + 1/20) + hard**2/(2*.2)
    assert robot['continuous_search_boundary_margin'] == pytest.approx(expected)
    assert robot['continuous_tracking_boundary_margin'] == .0005
    assert cfg['continuous_tracking']['search_boundary_margin'] == pytest.approx(expected)
    assert cfg['continuous_tracking']['boundary_margin'] == .0005


def test_search_reaches_original_18_mm_s_with_existing_acceleration():
    cfg = load_config(ROOT/'config.yaml')
    p = ContinuousTrackingPolicy(cfg)
    w = Wrench(0,0,0,0,0,0)
    for i in range(201):
        command = p.update(i*.01,w,w,RobotState(i*.01,np.zeros(6),np.zeros(6)))
    assert command.state == 'TARGET_SEARCH'
    assert command.speed == pytest.approx(.018)
    assert cfg['policy']['search_speed'] == .018
    np.testing.assert_allclose(command.direction_xy,p.search_direction)


def test_search_motion_uses_search_envelope(phased):
    c, clock = phased
    calls = []
    c.control.speedL = lambda *args: calls.append(args) or True
    c.command_planar_velocity([0,1], .018, .01)
    assert calls[0][0] == [0,.018,0,0,0,0]
    assert c.speed_guard_diagnostics['speed_guard_nominal_mps'] == .018
    assert c.speed_guard_diagnostics['speed_guard_state'] == 'OK'
    with pytest.raises(RobotError,match='command exceeds'):
        c.command_planar_velocity([0,1], .019, .01)


@pytest.mark.parametrize('phase', ['TARGET_SEARCH','CONTINUOUS_TRACKING'])
def test_isolated_small_overrun_recovers_on_fresh_packet(phased, phase):
    c, clock = phased
    c.set_continuous_phase(phase)
    nominal = c.config['continuous_speed_limits'][phase]
    clock.speed = 1.25*nominal
    c.read_state()
    assert c.speed_guard_diagnostics['speed_guard_state'] == 'PENDING'
    advance(clock, nominal)
    c.read_state()
    assert c.speed_guard_diagnostics['speed_guard_state'] == 'OK'
    assert c.speed_guard_diagnostics['speed_guard_count'] == 0


@pytest.mark.parametrize('phase', ['TARGET_SEARCH','CONTINUOUS_TRACKING'])
def test_sustained_overrun_fails_closed_with_bounded_fresh_samples(phased, phase):
    c, clock = phased
    c.set_continuous_phase(phase)
    nominal = c.config['continuous_speed_limits'][phase]
    clock.speed = 1.25*nominal
    c.read_state()
    advance(clock, clock.speed)
    c.read_state()
    advance(clock, clock.speed)
    with pytest.raises(ContinuousSpeedLimitError) as caught:
        c.read_state()
    d = caught.value.speed_limit_observation
    assert d['speed_guard_state'] == 'SUSTAINED_TRIP'
    assert d['speed_guard_count'] == 3
    assert d['speed_guard_elapsed_sec'] == pytest.approx(.02)
    with pytest.raises(RobotError,match='motion fault latched'):
        c.command_planar_velocity([0,1],nominal,.01)


@pytest.mark.parametrize('phase', ['TARGET_SEARCH','CONTINUOUS_TRACKING'])
def test_severe_overspeed_immediate_and_no_motion(phased, phase):
    c, clock = phased
    c.set_continuous_phase(phase)
    nominal = c.config['continuous_speed_limits'][phase]
    clock.speed = np.nextafter(CONTINUOUS_HARD_FACTOR*nominal,np.inf)
    c.control.speedL = lambda *a: pytest.fail('must not send motion')
    with pytest.raises(ContinuousSpeedLimitError) as caught:
        c.command_planar_velocity([0,1],nominal,.01)
    assert caught.value.speed_limit_observation['speed_guard_state'] == 'HARD_TRIP'
    assert caught.value.speed_limit_observation['speed_guard_elapsed_sec'] == 0
    stops=[]
    c.control.speedStop=lambda a:stops.append(a) or True
    c.stop()
    assert stops == [.2]  # even when the local controller initially believed it was stopped


def test_duplicate_packets_neither_count_nor_clear_and_deadline_is_finite(phased):
    c, clock = phased
    clock.speed = .023
    c.read_state()
    for _ in range(3):
        advance(clock,.018,.004,fresh=False)
        c.read_state()
        assert c.speed_guard_diagnostics['speed_guard_count'] == 1
        assert c.speed_guard_diagnostics['speed_guard_state'] == 'PENDING'
    advance(clock,.018,.008,fresh=False)
    with pytest.raises(ContinuousSpeedLimitError):
        c.read_state()


def test_stale_packet_still_fails_before_debounce(phased):
    c, clock = phased
    c.read_state()
    advance(clock,.018,.021,fresh=False)
    with pytest.raises(RobotError,match='stale RTDE'):
        c.read_state()


def test_fresh_packet_count_bounds_fast_receive_stream(phased):
    c, clock = phased
    clock.speed = .023
    c.read_state()
    advance(clock,.023,.002)
    c.read_state()
    advance(clock,.023,.002)
    with pytest.raises(ContinuousSpeedLimitError):
        c.read_state()
    assert c.speed_guard_diagnostics['speed_guard_elapsed_sec'] < .02


def test_first_contact_retains_braking_envelope_but_never_authorizes_motion(phased):
    c, clock = phased
    c._stopped = False
    c.set_continuous_phase('FIRST_CONTACT')
    c.stop()
    assert c.stop_request_accepted and not c._stopped
    c.read_state()  # residual 18 mm/s is not compared with tracking speed
    assert c.speed_guard_diagnostics['speed_guard_nominal_mps'] == .018
    with pytest.raises(RobotError,match='standstill'):
        c.set_continuous_phase('CONTINUOUS_TRACKING',stop_confirmed=True)
    advance(clock,0.)
    c.read_state()
    with pytest.raises(RobotError,match='new motion forbidden'):
        c.command_planar_velocity([0,1],.018,.01)
    with pytest.raises(RobotError,match='standstill'):
        c.set_continuous_phase('CONTINUOUS_TRACKING',stop_confirmed=False)
    c.set_continuous_phase('CONTINUOUS_TRACKING',stop_confirmed=True)
    advance(clock,.018)
    with pytest.raises(ContinuousSpeedLimitError):
        c.read_state()
    assert c.speed_guard_diagnostics['speed_guard_nominal_mps'] == np.hypot(.001,.0005)


def test_phase_changes_cannot_restart_pending_excursion(phased):
    c, clock = phased
    clock.speed=.023
    c.read_state()
    c.set_continuous_phase('FIRST_CONTACT')
    advance(clock,.023,.02)
    with pytest.raises(ContinuousSpeedLimitError):
        c.read_state()


def test_policy_contact_trigger_stops_search_and_requires_held_actual_standstill():
    p = ContinuousTrackingPolicy(load_config(ROOT/'config.yaml'))
    def tick(i,force,speed):
        w=Wrench(force,0,0,0,0,0)
        return p.update(i*.01,w,w,RobotState(i*.01,np.zeros(6),np.array([speed,0,0,0,0,0])))
    for i in range(201):
        command=tick(i,0,.018)
    assert command.speed == pytest.approx(.018)
    for i in range(201,206):
        command=tick(i,(i-200)*.2,.018)
    assert p.state == State.FIRST_CONTACT and not command.move
    assert p.stop_requested
    for i in range(206,226):
        assert not tick(i,1.,.018).move
        assert p.state == State.FIRST_CONTACT
    for i in range(226,234):
        assert not tick(i,1.,0).move
        assert p.state == State.FIRST_CONTACT
    assert not tick(234,1.,0).move  # successful confirmation sample still stops
    assert p.state == State.CONTINUOUS_TRACKING and p.stop_confirmed
    command=tick(235,1.,0)
    assert command.move and command.speed <= np.hypot(.001,.0005)


def test_search_boundary_uses_extra_clearance_only(phased):
    c, clock = phased
    c.config['continuous_xy_limits']=dict(x_min=-1,x_max=1,y_min=-1,y_max=1)
    c.receive.getActualTCPPose=lambda:[.998,0,0,0,0,0]
    clock.speed=0
    with pytest.raises(RobotError,match='boundary'):
        c.read_state()
    c.set_continuous_phase('CONTINUOUS_TRACKING')
    c.read_state()


def test_unmodified_simulation_at_18_mm_s_preserves_force_stop(tmp_path):
    from test_continuous_run import args
    import run_continuous_tracking as runner
    import json
    assert runner.run(args(tmp_path)) == 1
    summary=json.loads(next(tmp_path.glob('run_*/summary.json')).read_text())
    assert summary['termination_reason'] == 'STOP_FORCE_LIMIT'
    # Existing low-deceleration simulator is not evidence of successful contact at 18 mm/s.
    assert summary['initial_contact'] is None


def test_first_contact_never_settling_times_out_without_tracking_motion():
    p = ContinuousTrackingPolicy(load_config(ROOT/'config.yaml'))
    w = Wrench(1.,0,0,0,0,0)
    for i in range(110):
        t=i*.01
        command=p.update(t,w,w,RobotState(t,np.zeros(6),np.array([.018,0,0,0,0,0])))
        assert not command.move
    assert p.state == State.STOP
    assert 'confirmation timeout' in p.reason
    assert p.initial_contact is None


def test_search_distance_budget_reserves_phase_clearance(saved):
    cfg,path,_,_=saved
    prepare_real(cfg,path)
    p=ContinuousTrackingPolicy(cfg)
    w=Wrench(0,0,0,0,0,0)
    p.update(0,w,w,RobotState(0,np.zeros(6),np.zeros(6)))
    position=p.search_direction*(p.c['search_max_distance']-p.c['search_boundary_margin'])
    command=p.update(.01,w,w,RobotState(.01,np.r_[position,0,0,0,0],np.zeros(6)))
    assert not command.move and p.state == State.STOP
    assert p.reason == 'initial search budget exhausted'


def test_general_maximum_and_legacy_discrete_do_not_use_tracking_limit(phased):
    c, clock = phased
    c._return_mode=True
    c.set_continuous_phase('CONTINUOUS_TRACKING')
    c.read_state()  # startup return retains the original general limit
    advance(clock,.037)
    with pytest.raises(RobotError,match='maximum plus tolerance'):
        c.read_state()
    discrete=controller()
    discrete.receive.getActualTCPSpeed=lambda:[.018,0,0,0,0,0]
    discrete.command_planar_velocity([1,0],.018,.01)
    assert not discrete.speed_guard_diagnostics


def test_phased_csv_additions_preserve_legacy_replay(tmp_path, monkeypatch):
    import csv
    import run_continuous_tracking as runner
    from test_continuous_run import args
    from tools.visualize_continuous_run import read_run
    from robot.rtde_controller import SPEED_GUARD_FIELDS
    from experiment_logging.data_logger import ExperimentLogger
    cfg=load_config(ROOT/'config.yaml')
    p=ContinuousTrackingPolicy(cfg)
    c=controller()
    c.config.update(continuous_speed_limits=dict(TARGET_SEARCH=.018,
        CONTINUOUS_TRACKING=float(np.hypot(.001,.0005)),LOCAL_REACQUIRE=.0005),
        continuous_search_boundary_margin=.004,continuous_tracking_boundary_margin=.0005)
    c.set_continuous_phase('TARGET_SEARCH')
    c.receive.getActualTCPSpeed=lambda:[0,.018,0,0,0,0]
    robot=c.read_state()
    w=Wrench(0,0,0,0,0,0)
    command=p.update(robot.timestamp,w,w,robot)
    cfg['continuous_provenance']={'offline_double':True}
    logger=ExperimentLogger(tmp_path,cfg,extra_sample_fields=runner.EXTRA_SAMPLE_FIELDS+SPEED_GUARD_FIELDS,
                            workspace_logging=False)
    logger.log_sample(robot.timestamp,w,w,robot,command,p.contact_direction,p.tangent,
                      extra={**p.telemetry(command),**c.speed_guard_diagnostics})
    logger.close()
    path=logger.run_dir/'samples.csv'
    with path.open() as handle:
        rows=list(csv.DictReader(handle))
    r=rows[0]
    assert r['current_state']=='TARGET_SEARCH'
    assert float(r['speed_guard_nominal_mps'])==.018
    assert float(r['speed_guard_trip_mps'])==pytest.approx(.0216)
    assert float(r['tcp_vy'])==float(r['tcp_speed_mps'])==.018
    assert float(r['command_speed'])==0  # READY -> SEARCH stops on the first sample
    assert all(r[key]!='' for key in SPEED_GUARD_FIELDS)
    read_run(logger.run_dir)
    # Removing only newly added columns reproduces an old CSV schema.
    fields=[key for key in r if key not in SPEED_GUARD_FIELDS]
    with path.open('w') as handle:
        writer=csv.DictWriter(handle,fieldnames=fields,extrasaction='ignore')
        writer.writeheader();writer.writerows(rows)
    read_run(logger.run_dir)
