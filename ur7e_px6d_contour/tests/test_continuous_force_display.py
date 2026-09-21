"""Physical diagnostics and display only; no hardware and no force-mode control."""
from pathlib import Path
import numpy as np
import pytest
from config.loader import load_config
from simulation.geometry import create_target
from simulation.simulated_force_sensor import SimulatedForceSensor

ROOT=Path(__file__).resolve().parents[1]

def sensor(mu=.2,drag=.3,noise=0.):
    target=create_target(dict(target_shape='rectangle',target_center=[.02,0],target_width=.04,target_height=.04))
    return SimulatedForceSensor(target,dict(random_seed=7,granular_drag_force=drag,noise_std=noise,
        contact_stiffness=1800.,probe_tip_radius=.001,friction_coefficient=mu))

def test_physical_normal_friction_and_legacy_signal_have_distinct_signs():
    raw,d=sensor().read_wrench([-.0005,0],[0,.001])
    np.testing.assert_allclose(d['normal_physical'],[-.9,0])
    np.testing.assert_allclose(d['boundary_physical'],[-.9,-.18])
    np.testing.assert_allclose(d['environment_physical'],[-.9,-.48])
    np.testing.assert_allclose(d['robot_estimate'],[.9,.48])
    np.testing.assert_allclose(raw.force[:2],[.9,-.48])
    assert np.dot(d['friction_force'],[0,.001])<0

def test_no_contact_has_no_boundary_force_but_robot_balances_drag():
    _,d=sensor(noise=.8).read_wrench([-.01,0],[0,.001])
    np.testing.assert_array_equal(d['boundary_physical'],[0,0])
    np.testing.assert_allclose(d['robot_estimate'],[0,.3])

def test_noise_is_excluded_and_stable_sliding_needs_no_artificial_variation():
    quiet,noisy=sensor(mu=0,drag=0),sensor(mu=0,drag=0,noise=.8)
    for _ in range(20):
        raw,a=quiet.read_wrench([-.0005,0],[0,.001]);other,b=noisy.read_wrench([-.0005,0],[0,.001])
        for key in ('normal_physical','boundary_physical','environment_physical','robot_estimate'):
            np.testing.assert_array_equal(a[key],b[key])
        np.testing.assert_allclose(a['boundary_physical'],[-.9,0])
        np.testing.assert_allclose(a['robot_estimate'],-a['boundary_physical'])
        np.testing.assert_allclose(other.force[:2],sum(b[k] for k in ('object_force','friction_force','background_force','noise_force')))

def test_contact_loading_unloading_and_zero_are_data_driven():
    s=sensor(mu=0,drag=0)
    forces=[np.linalg.norm(s.read_wrench([x,0],[0,.001])[1]['boundary_physical']) for x in [-.00075,-.0005,-.00075,-.002]]
    np.testing.assert_allclose(forces,[.45,.9,.45,0])

def test_common_arrow_scale_zero_unknown_and_default_debug_hidden():
    import matplotlib;matplotlib.use('Agg',force=True)
    import matplotlib.pyplot as plt
    from simulation.continuous_view import VectorDisplay,FORCE_MM_PER_N
    fig,ax=plt.subplots();v=VectorDisplay(ax)
    v.draw([0,0],[8,9],[1,0],[1,0],[1,0],[0,1],valid=True,
           physical=dict(blue=np.array([1.,0]),red=np.array([-2.,0]),blue_label='Boundary contact (model)',note=''))
    assert {k for k,a in v.arrows.items() if a.get_visible()}=={'boundary','robot_estimate'}
    np.testing.assert_allclose(v.arrows['boundary'].xy,[FORCE_MM_PER_N,0])
    np.testing.assert_allclose(v.arrows['robot_estimate'].xy,[-2*FORCE_MM_PER_N,0])
    v.draw([0,0],[0,0],[0,0],[0,0],[0,0],[0,0],valid=False,
           physical=dict(blue=np.zeros(2),red=None,blue_label='Boundary contact (model)',note='unavailable'))
    assert not any(a.get_visible() for a in v.arrows.values())
    assert 'unavailable' in v.robot_text.get_text() and '0.00 N' in v.boundary_text.get_text()
    plt.close(fig)

def test_missing_or_unconfirmed_physical_metadata_does_not_guess_sign():
    from simulation.continuous_view import force_demonstration
    row=dict(dfx=1.,dfy=2.)
    for meta in ({},dict(schema_version=1,force_convention='unknown')):
        d=force_demonstration(row,meta,simulated=False)
        assert d['blue'] is None and d['red'] is None
        assert d['blue_label']=='Measured environment resultant'
    meta=dict(schema_version=1,force_convention='environment_on_probe',frame='Base',
              physical_sign_confirmed=True,base_frame_confirmed=True,calibration_reference='offline checked record',
              force_source='processed_wrench',estimate_method='quasistatic_planar_balance')
    d=force_demonstration(row,meta,simulated=False)
    np.testing.assert_allclose(d['blue'],[1,2]);np.testing.assert_allclose(d['red'],[-1,-2])
    assert 'uncertainty' in d['note']

def test_preview_defaults_and_display_switches_do_not_advance_simulation(tmp_path):
    import matplotlib;matplotlib.use('Agg',force=True)
    import matplotlib.pyplot as plt
    from simulation.continuous_preview import PreviewRun,ContinuousPreview
    from simulation.simulator import load_simulation_config
    m=PreviewRun(load_config(ROOT/'config.yaml'),load_simulation_config(ROOT/'simulation/scene_continuous.yaml'),tmp_path)
    v=ContinuousPreview(m)
    assert not v.debug and v.tcp.get_marker()=='o'
    assert not v.probe.get_visible() and not v.event_points.get_visible()
    assert not v.velocity.get_visible() and v.xy.get_legend() is None
    m.start(0);m.tick(.1,work_budget_sec=10)
    before=(m.session.robot.time,m.session.robot.pose.copy(),repr(m.session.sensor.rng.bit_generator.state))
    v.toggle_debug();v.draw();assert v.velocity.get_visible()
    v.toggle_debug();v.draw();v.select_view('Probe');v.draw()
    assert before[0]==m.session.robot.time and before[2]==repr(m.session.sensor.rng.bit_generator.state)
    np.testing.assert_array_equal(before[1],m.session.robot.pose)
    m.stop();plt.close(v.figure)

@pytest.mark.parametrize('mu',[.03,.20])
def test_display_modes_preserve_every_control_step_and_replay_force_values(tmp_path,mu):
    import matplotlib;matplotlib.use('Agg',force=True)
    import matplotlib.pyplot as plt
    from simulation.continuous_preview import PreviewRun,ContinuousPreview
    from simulation.continuous_session import SimulationSession
    from simulation.simulator import load_simulation_config
    from tools.visualize_continuous_run import read_run,make_figure
    cfg=load_config(ROOT/'config.yaml');scene=load_simulation_config(ROOT/'simulation/scene_continuous.yaml')
    scene['force_model']['friction_coefficient']=mu
    from copy import deepcopy
    force_model=deepcopy(scene['force_model'])
    m=PreviewRun(cfg,scene,tmp_path);v=ContinuousPreview(m)
    reference=SimulationSession(cfg,scene);reference.capture_bias();m.start(0)
    for i in range(1150):
        a=reference.step();b=m.session.step();m._record(b);reference.policy.events.clear()
        if i%50==0:v.toggle_debug();v.draw();v.toggle_components();v.draw()
        assert a.command.state==b.command.state
        np.testing.assert_array_equal(a.raw.array(),b.raw.array())
        np.testing.assert_array_equal(a.processed.array(),b.processed.array())
        np.testing.assert_array_equal(a.command.direction_xy,b.command.direction_xy)
        assert a.command.speed==b.command.speed and a.command.move==b.command.move
    m.stop();v.debug=False;v.draw()
    assert m.scene['force_model']==force_model and m.session.sensor.config==force_model
    data,cfg,events=read_run(m.run_dir);fig,update=make_figure(data,cfg,events);update(len(data['time'])-1)
    from simulation.continuous_view import FORCE_MM_PER_N
    expected=m.history[-1].diagnostics['normal_physical']
    assert np.linalg.norm(expected)>0
    np.testing.assert_allclose([data['sim_normal_physical_fx'][-1],data['sim_normal_physical_fy'][-1]],expected)
    np.testing.assert_allclose(v.arrows['boundary'].xy-v.arrows['boundary'].get_position(),expected*FORCE_MM_PER_N)
    for key in ('boundary','robot_estimate'):
        np.testing.assert_allclose(v.arrows[key].xy,fig.continuous_vectors.arrows[key].xy)
        assert v.arrows[key].get_visible()==fig.continuous_vectors.arrows[key].get_visible()
    assert not fig.axes[2].get_visible()
    legend=fig.axes[0].get_legend()
    assert legend is None or not legend.get_visible()
    for size in [(12,8),(20,8)]:
        fig.set_size_inches(*size);update(len(data['time'])-1);fig.canvas.draw()
        origin,x,y=fig.axes[0].transData.transform([[0,0],[1,0],[0,1]])
        assert np.linalg.norm(x-origin)==pytest.approx(np.linalg.norm(y-origin))
        note=next(t for t in fig.texts if 'Quasi-static' in t.get_text())
        renderer=fig.canvas.get_renderer();box=note.get_window_extent(renderer)
        assert box.y0>fig.continuous_debug_button.ax.get_window_extent(renderer).y1
        assert box.y1<fig.axes[0].xaxis.label.get_window_extent(renderer).y0
    plt.close(fig);plt.close(v.figure)

def test_legacy_logs_and_invalid_physical_values_remain_unavailable():
    from simulation.continuous_view import force_demonstration
    from simulation.simulated_force_sensor import PHYSICAL_FORCE_METADATA
    row=dict(physical_force_available=1,sim_normal_physical_fx=float('nan'),sim_normal_physical_fy=0,
             sim_boundary_physical_fx=float('nan'),sim_boundary_physical_fy=0,
             sim_robot_estimate_fx=float('nan'),sim_robot_estimate_fy=0)
    for meta in ({},PHYSICAL_FORCE_METADATA):
        result=force_demonstration(row,meta,simulated=True)
        assert result['blue'] is None and result['red'] is None

def test_session_logs_metadata_and_physical_vector_sums():
    from simulation.continuous_session import SimulationSession
    from simulation.simulator import load_simulation_config
    s=SimulationSession(load_config(ROOT/'config.yaml'),load_simulation_config(ROOT/'simulation/scene_continuous.yaml'))
    x=s.step();d=x.simulation_telemetry
    assert s.config['force_display']['schema_version']==1
    assert s.config['force_display']['force_source']=='synthetic_contact_and_drag_model'
    assert d['physical_force_available']==1
    for axis in ('x','y'):
        assert d['sim_boundary_physical_f'+axis]==pytest.approx(d['sim_normal_physical_f'+axis]+d['sim_friction_f'+axis])
        assert d['sim_robot_estimate_f'+axis]==pytest.approx(-d['sim_boundary_physical_f'+axis]-d['sim_background_f'+axis])


@pytest.mark.parametrize('shape,local_point,rotation',[
    ('circle',[.0205,0],37.),
    ('rectangle',[-.0205,.005],0.),
    ('rectangle',[-.0205,.005],31.),
    # A corner has an effective model normal, not a unique straight-edge normal.
    ('rectangle',[.0203,.0204],31.),
    ('triangle',[.0205,0],37.),
])
def test_blue_is_outward_model_normal_with_normal_load_not_resultant(shape,local_point,rotation):
    import matplotlib;matplotlib.use('Agg',force=True)
    import matplotlib.pyplot as plt
    from simulation.continuous_view import force_demonstration,VectorDisplay,FORCE_MM_PER_N
    from simulation.simulated_force_sensor import PHYSICAL_FORCE_METADATA
    angle=np.deg2rad(rotation)
    transform=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
    center=np.array([.03,-.01]);position=center+transform@local_point
    target=create_target(dict(target_shape=shape,target_center=center,target_radius=.02,
                              target_width=.04,target_height=.04,target_rotation_deg=rotation))
    distance,normal=target.signed_distance_and_outward_normal(position)
    tangent=np.array([-normal[1],normal[0]])
    cfg=dict(sensor().config)
    _,diagnostics=SimulatedForceSensor(target,cfg).read_wrench(position,tangent*.001)
    row={'physical_force_available':1}
    for name in ('normal_physical','boundary_physical','robot_estimate'):
        row.update({f'sim_{name}_f{axis}':value for axis,value in zip('xy',diagnostics[name])})
    physical=force_demonstration(row,PHYSICAL_FORCE_METADATA,simulated=True)
    load=cfg['contact_stiffness']*(cfg['probe_tip_radius']-distance)
    assert load>0
    np.testing.assert_allclose(physical['blue'],normal*load,atol=1e-12)
    assert np.dot(physical['blue'],tangent)==pytest.approx(0,abs=1e-12)
    assert np.dot(physical['blue'],normal)>0
    assert np.linalg.norm(physical['blue'])==pytest.approx(load)
    assert np.linalg.norm(physical['blue'])<np.linalg.norm(diagnostics['boundary_physical'])
    np.testing.assert_allclose(physical['red'],-(physical['blue']+diagnostics['friction_force']+diagnostics['background_force']))
    assert not np.allclose(physical['red'],-physical['blue'])
    assert physical['blue_label']=='法向反力 / Normal reaction'
    assert '法向反力示意；摩擦分量未绘制' in physical['note']
    debug=force_demonstration(row,PHYSICAL_FORCE_METADATA,simulated=True,components_visible=True)
    assert '摩擦分量未绘制' not in debug['note']
    np.testing.assert_array_equal(debug['blue'],physical['blue'])
    np.testing.assert_array_equal(debug['red'],physical['red'])
    fig,ax=plt.subplots();display=VectorDisplay(ax)
    display.draw(position*1000,[0,0],[0,0],[0,0],tangent,-normal,valid=True,physical=physical)
    for arrow,component in [('boundary','blue'),('robot_estimate','red')]:
        np.testing.assert_allclose(display.arrows[arrow].get_position(),position*1000)
        np.testing.assert_allclose(display.arrows[arrow].xy-position*1000,physical[component]*FORCE_MM_PER_N)
    assert not display.arrows['friction'].get_visible()
    plt.close(fig)


def test_missing_normal_never_projects_boundary_resultant_and_zero_contact_hides_blue():
    import matplotlib;matplotlib.use('Agg',force=True)
    import matplotlib.pyplot as plt
    from simulation.continuous_view import force_demonstration,VectorDisplay
    from simulation.simulated_force_sensor import PHYSICAL_FORCE_METADATA
    row=dict(physical_force_available=1,sim_boundary_physical_fx=-.9,sim_boundary_physical_fy=-.18,
             sim_robot_estimate_fx=.9,sim_robot_estimate_fy=.48)
    assert force_demonstration(row,PHYSICAL_FORCE_METADATA,simulated=True)['blue'] is None
    _,d=sensor().read_wrench([-.01,0],[0,.001])
    for name in ('normal_physical','boundary_physical','robot_estimate'):
        row.update({f'sim_{name}_f{axis}':value for axis,value in zip('xy',d[name])})
    physical=force_demonstration(row,PHYSICAL_FORCE_METADATA,simulated=True)
    np.testing.assert_array_equal(physical['blue'],[0,0])
    np.testing.assert_allclose(physical['red'],[0,.3])
    fig,ax=plt.subplots();display=VectorDisplay(ax)
    display.draw([0,0],[0,0],[0,0],[0,0],[0,0],[0,0],valid=False,physical=physical)
    assert not display.arrows['boundary'].get_visible()
    assert display.arrows['robot_estimate'].get_visible()
    plt.close(fig)
