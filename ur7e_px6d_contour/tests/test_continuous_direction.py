"""Direction reconfirmation: isolated streams plus actual geometry, no devices."""
from pathlib import Path
import numpy as np
import pytest
from config.loader import load_config
from core.models import RobotState, Wrench
from policy.continuous_tracking import ContinuousTrackingPolicy, State

ROOT=Path(__file__).resolve().parents[1]


def step(p,t,deg=0.,force=1.5,speed=0.,xy=(0.,0.)):
    a=np.deg2rad(deg); w=Wrench(force*np.cos(a),force*np.sin(a),0,0,0,0)
    return p.update(t,w,w,RobotState(t,np.r_[xy,0,0,0,0],np.array([speed,0,0,0,0,0])))


def tracking():
    p=ContinuousTrackingPolicy(load_config(ROOT/'config.yaml'))
    for i in range(101): step(p,i*.01)
    return p


def test_separate_jump_from_estimate_lag_and_slow_before_pause():
    p=tracking(); speeds=[]
    for i in range(101,141):
        cmd=step(p,i*.01,(i-100)*2.)
        speeds.append(cmd.speed)
        if p.state.value=='DIRECTION_RECONFIRM':break
    assert min(speeds)<p.c['tangential_speed']*.8
    assert p.state.value=='DIRECTION_RECONFIRM'
    assert not cmd.move
    assert p.measurement_jump_deg==pytest.approx(2.)
    assert p.estimate_residual_deg>p.measurement_jump_deg


def test_new_stable_direction_can_exceed_90_deg_without_reversing_sign():
    p=tracking(); old=p.contact_direction.copy()
    cmd=step(p,1.01,120,speed=.002)
    assert p.state.value=='DIRECTION_RECONFIRM' and not cmd.move
    np.testing.assert_array_equal(p.contact_direction,old)
    for i in range(102,131):
        assert not step(p,i*.01,120,speed=.002).move
        np.testing.assert_array_equal(p.contact_direction,old)
    for i in range(131,138): assert not step(p,i*.01,120).move
    for i in range(138,160):
        cmd=step(p,i*.01,120)
        if p.state==State.CONTINUOUS_TRACKING: break
    assert p.state==State.CONTINUOUS_TRACKING and not cmd.move
    np.testing.assert_allclose(p.contact_direction,[-.5,np.sqrt(3)/2],atol=1e-8)
    cmd=step(p,(i+1)*.01,120)
    assert cmd.move and cmd.speed <= p.c['tangential_speed']*.3
    assert np.dot(cmd.direction_xy,p.tangent)>0


def test_single_reverse_spike_is_terminal_not_a_new_direction():
    p=tracking(); old=p.contact_direction.copy()
    assert not step(p,1.01,180).move
    for i in range(102,140): assert not step(p,i*.01,180).move
    assert p.state==State.STOP and 'reversal' in p.reason
    np.testing.assert_array_equal(p.contact_direction,old)


@pytest.mark.parametrize('moving,alternating',[(True,False),(False,True)])
def test_reconfirm_must_settle_and_hold_direction_or_timeout(moving,alternating):
    p=tracking(); step(p,1.01,60,speed=.002)
    for i in range(102,250):
        cmd=step(p,i*.01,60+(20 if alternating and i%2 else 0),speed=.002 if moving else 0)
        assert not cmd.move
    assert p.state==State.STOP
    assert 'direction' in p.reason and 'timeout' in p.reason


def test_repeat_reconfirmation_without_progress_has_finite_exit():
    p=tracking(); t=1.
    for angle in [60,0,60,0,60,0]:
        for _ in range(30):
            t+=.01; step(p,t,angle)
            if p.state==State.STOP: break
        if p.state==State.STOP:break
    assert p.state==State.STOP
    assert 'progress' in p.reason
    reason=p.reason
    p.request_stop(t,np.zeros(6),'operator stop')
    assert p.reason==reason


def test_direction_pause_loss_uses_default_loss_route_and_stale_samples_stop():
    p=tracking(); step(p,1.01,60)
    for i in range(102,124): assert not step(p,i*.01,60,force=0.).move
    assert p.state==State.STOP and 'contact lost' in p.reason
    p=tracking(); step(p,1.01,60)
    assert not step(p,1.1,60).move
    assert p.state==State.STOP and 'stale' in p.reason


def test_synthetic_components_and_geometry_diagnostics_are_logged_separately():
    from simulation.continuous_session import SimulationSession
    from simulation.simulator import load_simulation_config
    s=SimulationSession(load_config(ROOT/'config.yaml'),load_simulation_config(ROOT/'simulation/scene_continuous.yaml'))
    s.capture_bias()
    for _ in range(1100):
        sample=s.step();s.policy.events.clear()
        meta=sample.diagnostics
        components=sum((meta[k] for k in ('object_force','friction_force','background_force','noise_force')))
        np.testing.assert_allclose(components,sample.raw.force[:2],atol=1e-14)
    assert meta['signed_distance']>=0
    assert 'sim_signed_distance_m' in sample.simulation_telemetry


def test_preview_magnitude_scales_velocity_panel_and_manual_views(tmp_path):
    import matplotlib
    matplotlib.use('Agg',force=True)
    import matplotlib.pyplot as plt
    from simulation.continuous_preview import PreviewRun,ContinuousPreview
    from simulation.simulator import load_simulation_config
    from simulation.continuous_view import FORCE_MM_PER_N,VELOCITY_MM_PER_MM_S
    from types import SimpleNamespace
    model=PreviewRun(load_config(ROOT/'config.yaml'),load_simulation_config(ROOT/'simulation/scene_continuous.yaml'),tmp_path)
    view=ContinuousPreview(model)
    assert view.xy.get_subplotspec().get_gridspec().get_width_ratios()==[7,3]
    assert view.viewport.mode=='Target'
    model.start(0);model.tick(.2,work_budget_sec=10);view.draw()
    last=model.history[-1];xy=last.robot.pose[:2]*1000
    np.testing.assert_allclose(view.arrows['measured'].xy-xy,last.processed.force[:2]*FORCE_MM_PER_N)
    np.testing.assert_allclose(view.arrows['command'].xy-xy,last.command.direction_xy*last.command.speed*1000*VELOCITY_MM_PER_MM_S)
    assert view.velocity.get_ylabel()=='Velocity [mm/s]'
    assert view.velocity_cursor.get_xdata()[0]==view.cursor.get_xdata()[0]==last.time
    view.select_view('Probe');view.toggle_follow();assert view.viewport.follow
    view.xy.set_xlim(-82,-45);view.xy.set_ylim(-12,12)
    assert not view.viewport.follow
    model.pause(.2);view.draw()
    assert view.xy.get_xlim()==(-82,-45)
    view.viewport.scroll(SimpleNamespace(inaxes=view.xy,xdata=-65,ydata=0,button='up'))
    limits=view.xy.get_xlim();view.draw();assert view.xy.get_xlim()==limits
    view.toggle_components();assert view.show_components
    model.stop();view.draw()
    assert not view.arrows['command'].get_visible()
    assert not view.arrows['inward'].get_visible()
    plt.close(view.figure)


@pytest.mark.parametrize('name,rotation,shift',[('circle',0,(0,0)),('square',0,(0,0)),('triangle',0,(0,0)),('square',30,(.01,.015)),('empty',0,(0,0))])
def test_first_turn_closed_loop_geometry_and_failures(tmp_path,name,rotation,shift):
    from simulation.continuous_validation import run_direction_case
    report=run_direction_case(name,output_root=tmp_path,duration=180,rotation_deg=rotation,translation=shift)
    assert report['max_penetration_m']<=1e-9
    assert report['segment_penetration_count']==0
    assert not report['abnormal_compression']
    assert report['processed_force_range_N'][1]<2.25
    if name=='empty':
        assert not report['first_turn_validated']
        assert report['termination_reason']=='STOP_SEARCH_LIMIT'
    else:
        assert report['first_turn_validated'],report
        assert report['post_turn_net_progress_m']>=.005
        assert report['termination_reason']=='STOP_TIME_LIMIT'
        assert not report['contour_complete']
        if name!='circle':assert report['direction_confirmed_count']>=1 and report['resume_verified_count']>=1


def test_low_force_pause_can_enter_direction_confirmation_and_enabled_loss_route():
    p=tracking();step(p,1.01,force=.4,speed=.002)
    assert not step(p,1.02,60,force=.6,speed=.002).move
    assert p.state.value=='DIRECTION_RECONFIRM'
    for i in range(103,107):assert not step(p,i*.01,60,force=min(1.5,.6+(i-102)*.2),speed=.002).move
    p.c['reacquire_enabled']=True  # isolated branch test, not a change to defaults
    for i in range(107,124):assert not step(p,i*.01,60,force=0).move
    assert p.state==State.LOCAL_REACQUIRE


def test_two_confirmations_with_real_progress_then_restart_load_guard():
    p=tracking();t=1.;xy=np.zeros(2)
    for angle in [60,120]:
        t+=.01;assert not step(p,t,angle,xy=xy).move
        assert p.state.value=='DIRECTION_RECONFIRM'
        for _ in range(25):
            t+=.01;step(p,t,angle,xy=xy)
        assert p.state==State.CONTINUOUS_TRACKING
        # Isolated observation stream to verify retry bookkeeping; the separate
        # geometric cases, not these injected poses, establish physical progress.
        for _ in range(250):
            t+=.01;xy=xy+p.tangent*.00001;step(p,t,angle,xy=xy)
        assert p.state==State.CONTINUOUS_TRACKING
    assert p.direction_reconfirm_count==2
    t+=.01;step(p,t,60,xy=xy)
    for _ in range(25):t+=.01;step(p,t,60,xy=xy)
    for f in [1.7,1.9,2.1,2.3]:t+=.01;cmd=step(p,t,60,force=f,xy=xy)
    assert not cmd.move and p.state==State.STOP and 'overload' in p.reason


def test_direction_confirmation_cannot_succeed_just_after_its_deadline():
    p=tracking();step(p,1.01,60,speed=.002)
    for i in range(102,194):assert not step(p,i*.01,60,speed=.002).move
    # Settling would finish at 2.02 s, after the 2.01 s fixed deadline.
    for i in range(194,204):assert not step(p,i*.01,60).move
    assert p.state==State.STOP
    assert p.stop_reason.value=='STOP_DIRECTION_UNCONFIRMED'
    assert not any(e.event_type=='DIRECTION_CONFIRMED' for e in p.events)
