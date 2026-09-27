"""Surface friction editor: bounded offline runs, never real devices."""
from copy import deepcopy
from pathlib import Path
import json
import numpy as np
import pytest
import yaml
from config.loader import load_config
from simulation.continuous_preview import PreviewRun,ContinuousPreview
from simulation.continuous_session import SimulationSession
from simulation.simulator import load_simulation_config

ROOT=Path(__file__).resolve().parents[1]

@pytest.fixture
def model(tmp_path):
    cfg=load_config(ROOT/'config.yaml');cfg['continuous_tracking']['max_runtime_sec']=12
    cfg['continuous_tracking']['search_speed']=.001  # historical contact-memory fixture
    return PreviewRun(cfg,load_simulation_config(ROOT/'simulation/scene_continuous.yaml'),tmp_path)

def test_default_is_continuous_only():
    assert load_simulation_config(ROOT/'simulation/scene_continuous.yaml')['force_model']['friction_coefficient']==.20
    assert load_simulation_config(ROOT/'simulation/simulation_config.yaml')['force_model']['friction_coefficient']==.03
    assert load_simulation_config(ROOT/'simulation/scene_direction_triangle.yaml')['force_model']['friction_coefficient']==.03

@pytest.mark.parametrize('state',['READY','RUNNING','PAUSED'])
def test_apply_preserves_environment_but_resets_every_session_state(model,state):
    old=model.session;scene=deepcopy(model.scene)
    if state!='READY':
        model.start(0);model.set_speed(10,0)
        for i in range(11):model.tick((i+1)*.1,work_budget_sec=10)
        assert old.policy.last_reliable_contact is not None
        if state=='PAUSED':model.pause(1.1)
        old_dir=model.run_dir;old_logger=model.logger
    model.set_surface_friction(.03)
    assert model.session is not old and model.status=='READY' and model.logger is None
    scene['force_model']['friction_coefficient']=.03
    assert model.scene==scene
    fresh=model.session
    assert fresh.robot.time==0 and fresh.policy.state.value=='READY'
    assert fresh.policy.last_reliable_contact is None and fresh.policy.initial_contact is None
    assert fresh.preprocessor is not old.preprocessor and fresh.sensor is not old.sensor
    assert not model.history and not model.path_history and not model.events
    assert fresh.sensor.config['friction_coefficient']==.03
    if state!='READY':
        assert old_logger._sample_file.closed and old_dir.is_dir()
        summary=json.loads((old_dir/'summary.json').read_text())
        assert 'friction_coefficient' in summary['reason'] and '0.2 -> 0.03' in summary['reason']
        assert 'FRICTION_CHANGED' in (old_dir/'policy_waypoints.csv').read_text()
        assert yaml.safe_load((old_dir/'config_snapshot.yaml').read_text())['continuous_simulation']['force_model']['friction_coefficient']==.2
    model.start(0);assert model.session is fresh
    assert yaml.safe_load((model.run_dir/'config_snapshot.yaml').read_text())['continuous_simulation']['force_model']['friction_coefficient']==.03
    model.stop()

def test_editor_draft_apply_save_load_and_no_plot_shrink(model,tmp_path):
    import matplotlib;matplotlib.use('Agg',force=True)
    import matplotlib.pyplot as plt
    v=ContinuousPreview(model);original=model.session;v.draw()
    bounds=v.xy.get_position(original=True).bounds
    v.friction_button._observers.process('clicked',None);v.draw()
    np.testing.assert_allclose(v.xy.get_position(original=True).bounds,bounds)
    v.friction_box.set_val('0.03');v.draw()
    from types import SimpleNamespace
    v.friction_box.capturekeystrokes=True
    v.on_key(SimpleNamespace(key='3'));v.on_key(SimpleNamespace(key='enter'))
    assert model.scene['target_shape']=='circle' and model.status=='READY'
    v.friction_box.capturekeystrokes=False
    assert model.session is original and model.scene['force_model']['friction_coefficient']==.2
    assert '0.20' in v.friction_current.get_text()
    v.friction_apply._observers.process('clicked',None);v.draw()
    assert model.scene['force_model']['friction_coefficient']==.03 and '0.03' in v.friction_current.get_text()
    v.adjust_friction(1);assert float(v.friction_box.text)==.04
    assert model.scene['force_model']['friction_coefficient']==.03
    v.adjust_friction(-1);assert float(v.friction_box.text)==.03
    saved=model.save_scene(tmp_path/'mu03.yaml')
    v.friction_box.set_val('0.20');v.apply_friction()
    v.invoke(lambda:model.load_scene(saved))
    assert float(v.friction_box.text)==.03 and '0.03' in v.friction_current.get_text()
    headless=SimulationSession(model.base_config,load_simulation_config(saved))
    assert headless.sensor.config==model.session.sensor.config
    for i in range(30):
        a=headless.step();b=model.session.step()
        np.testing.assert_array_equal(a.raw.array(),b.raw.array())
        assert a.command.state==b.command.state and a.command.speed==b.command.speed
    for size in [(16,9),(12,8)]:
        v.figure.set_size_inches(*size);v.draw();v.figure.canvas.draw()
        renderer=v.figure.canvas.get_renderer()
        assert not v.friction_current.get_window_extent(renderer).overlaps(v.friction_button.ax.get_window_extent(renderer))
        assert v.friction_current.get_window_extent(renderer).x1<=v.figure.bbox.x1
        for debug in (False,True):
            for settings in (False,True):
                v.debug=debug
                if v.settings!=settings:v.toggle_settings()
                v.draw();v.figure.canvas.draw()
                caption=v.friction_caption.get_window_extent(renderer)
                assert caption.y1<v.force.xaxis.label.get_window_extent(renderer).y0
                assert not v.velocity.get_visible()
                assert v.friction_status.get_window_extent(renderer).x1<=v.figure.bbox.x1
                for axis in v.friction_axes:
                    assert axis.get_window_extent(renderer).y0>v.force_note.get_window_extent(renderer).y1
    plt.close(v.figure)

@pytest.mark.parametrize('value',['',' ','-0.01','NaN','inf','-inf','abc'])
def test_invalid_editor_values_are_readable_and_leave_session_untouched(model,value):
    import matplotlib;matplotlib.use('Agg',force=True)
    import matplotlib.pyplot as plt
    v=ContinuousPreview(model);v.toggle_friction();old=model.session
    v.friction_box.set_val(value);v.apply_friction();v.draw()
    assert model.session is old and model.scene['force_model']['friction_coefficient']==.2
    assert v.friction_error
    plt.close(v.figure)

def test_surface_mu_changes_sensor_and_control_feedback_not_background(model):
    from simulation.geometry import create_target
    from simulation.simulated_force_sensor import SimulatedForceSensor
    from sensor.force_preprocess import WrenchPreprocessor
    target=create_target(dict(target_shape='rectangle',target_center=[.02,0],target_width=.04,target_height=.04))
    results=[]
    for mu in [.03,.20]:
        cfg=deepcopy(model.scene['force_model']);cfg.update(friction_coefficient=mu,noise_std=0)
        raw,d=SimulatedForceSensor(target,cfg).read_wrench([-.0005,0],[0,.001])
        pre=WrenchPreprocessor.from_config(model.session.config['preprocessing'])
        results.append((raw,d,pre.process(raw)))
        assert np.linalg.norm(d['friction_force'])==pytest.approx(mu*np.linalg.norm(d['object_force']))
    a,b=results
    np.testing.assert_array_equal(a[1]['object_force'],b[1]['object_force'])
    np.testing.assert_array_equal(a[1]['background_force'],b[1]['background_force'])
    assert a[0].fy!=b[0].fy and a[2].fy!=b[2].fy
