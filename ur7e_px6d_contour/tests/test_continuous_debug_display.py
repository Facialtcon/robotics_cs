"""Display-only regressions; finite offline frames, no hardware."""
from pathlib import Path
import numpy as np
import pytest
import matplotlib
matplotlib.use('Agg',force=True)
import matplotlib.pyplot as plt
from simulation import continuous_view as view


def draw(v, **kwargs):
    args=dict(valid=True,debug=True,simulated=True,components_enabled=False,
              physical=dict(blue=np.array([-2.,0]),red=np.array([2.,.6]),blue_label=view.NORMAL_REACTION_LABEL))
    args.update(kwargs)
    v.draw([3.,4.],[2.,-.6],[1.,0],[.9,.1],[.6,.8],[1.,0],**args)


def test_shared_styles_legend_and_toggles():
    styles=view.DISPLAY_STYLES
    keys=('boundary','robot_estimate','measured','tangent','inward','command','actual')
    assert len({(styles[k]['color'],styles[k]['linestyle']) for k in keys})==len(keys)
    assert styles['measured']['color']!=styles['boundary']['color']
    assert styles['inward']['color']!=styles['robot_estimate']['color']
    fig,ax=plt.subplots();v=view.VectorDisplay(ax)
    draw(v);legend=v.legend
    assert set(v.legend_keys)==set(keys)|{'executed'}
    from matplotlib.colors import to_rgba
    from matplotlib.lines import Line2D
    for key,proxy in zip(v.legend_keys,legend.legend_handles):
        color=proxy.get_color() if isinstance(proxy,Line2D) else proxy.get_edgecolor()
        assert to_rgba(color)==to_rgba(styles[key]['color'])
        assert proxy.get_linestyle()==styles[key]['linestyle']
    assert '仿真合成' in ' '.join(t.get_text() for t in legend.get_texts())
    draw(v);assert v.legend is legend  # no per-frame reconstruction
    draw(v,components_enabled=True,components={'friction':[0,0]})
    assert set(view.COMPONENT_KEYS)<=set(v.legend_keys)
    labels=dict(zip(v.legend_keys,[t.get_text() for t in v.legend.get_texts()]))
    assert '零' in labels['friction'] and '不可用' in labels['noise']
    assert not v.arrows['friction'].get_visible() and not v.arrows['noise'].get_visible()
    draw(v,components_enabled=False)
    assert not set(view.COMPONENT_KEYS)&set(v.legend_keys)
    draw(v,debug=False)
    assert not v.legend.get_visible()
    assert {k for k,a in v.arrows.items() if a.get_visible()}=={'boundary','robot_estimate'}
    plt.close(fig)


def test_estimated_tangent_is_not_rotated_to_truth_and_real_has_no_truth():
    fig,ax=plt.subplots();v=view.VectorDisplay(ax)
    draw(v,true_tangent=True)
    np.testing.assert_allclose(v.arrows['tangent'].xy-[3,4],np.array([.6,.8])*view.DIRECTION_LENGTH_MM)
    line=np.column_stack(v.true_line.get_data())
    assert v.true_line.get_visible()
    assert np.dot(line[1]-line[0],[-2,0])==pytest.approx(0)
    assert v.truth_angle==pytest.approx(np.degrees(np.arccos(.8)))
    assert '0–90' in v.truth_note
    draw(v,true_tangent=True,simulated=False)
    assert not v.true_line.get_visible() and '不可用' in v.truth_note
    assert 'true_tangent' not in v.legend_keys
    draw(v,true_tangent=True,physical=dict(blue=np.zeros(2),red=np.zeros(2),blue_label=view.NORMAL_REACTION_LABEL))
    assert not v.true_line.get_visible() and v.truth_angle is None
    plt.close(fig)


@pytest.mark.parametrize('shape',['circle','rotated_rectangle'])
def test_preview_replay_styles_truth_layout_and_control_independence(tmp_path,shape):
    from copy import deepcopy
    from config.loader import load_config
    from simulation.simulator import load_simulation_config
    from simulation.continuous_preview import PreviewRun,ContinuousPreview
    from simulation.continuous_session import SimulationSession
    from tools.visualize_continuous_run import read_run,make_figure
    root=Path(__file__).resolve().parents[1]
    cfg=load_config(root/'config.yaml');cfg['continuous_tracking']['max_runtime_sec']=40
    cfg['continuous_tracking']['search_speed']=.001  # historical moving-arrow fixture
    scene=load_simulation_config(root/'simulation/scene_continuous.yaml')
    if shape=='rotated_rectangle':scene.update(target_shape=shape,target_width=.08,target_height=.08,target_rotation_deg=30)
    original=deepcopy(scene)
    m=PreviewRun(cfg,scene,tmp_path);v=ContinuousPreview(m)
    assert not v.show_true_tangent
    reference=SimulationSession(cfg,scene);reference.capture_bias();m.start(0)
    for i in range(3000):
        a=reference.step();b=m.session.step();m._record(b);reference.policy.events.clear()
        if i%300==0:
            state=(m.session.robot.time,repr(m.session.sensor.rng.bit_generator.state))
            v.toggle_debug();v.toggle_components();v.toggle_true_tangent();v.draw()
            assert state==(m.session.robot.time,repr(m.session.sensor.rng.bit_generator.state))
        np.testing.assert_array_equal(a.raw.array(),b.raw.array())
        np.testing.assert_array_equal(a.processed.array(),b.processed.array())
        np.testing.assert_array_equal(a.command.direction_xy,b.command.direction_xy)
        assert (a.command.move,a.command.speed,a.command.state)==(b.command.move,b.command.speed,b.command.state)
    assert m.scene['force_model']==original['force_model']
    v.debug=True;v.show_components=True;v.show_true_tangent=True;v.draw()
    moving_index=2999
    moving_arrows={k:(np.array(a.xy),a.get_visible()) for k,a in v.arrows.items()}
    moving_labels=[t.get_text() for t in v.vectors.legend.get_texts()]
    moving_truth=np.array(v.vectors.true_line.get_data())
    assert v.arrows['tangent'].get_visible() and v.vectors.true_line.get_visible()
    state=(m.session.robot.time,repr(m.session.sensor.rng.bit_generator.state))
    v.toggle_settings();v.draw()
    v.truth_button._observers.process('clicked',None)
    assert not v.show_true_tangent and not v.vectors.true_line.get_visible()
    assert 'true_tangent' not in v.vectors.legend_keys
    v.truth_button._observers.process('clicked',None)
    assert v.show_true_tangent and v.vectors.true_line.get_visible()
    assert state==(m.session.robot.time,repr(m.session.sensor.rng.bit_generator.state))
    v.toggle_settings();v.draw()
    np.testing.assert_allclose(v.arrows['tangent'].xy-b.robot.pose[:2]*1000,
                              b.tangent/np.linalg.norm(b.tangent)*view.DIRECTION_LENGTH_MM)
    m.stop();v.debug=True;v.show_components=True;v.show_true_tangent=True;v.draw()
    data,cfg,events=read_run(m.run_dir)
    fig,update=make_figure(data,cfg,events,debug=True,components=True,true_tangent=True)
    update(moving_index)
    for k,(xy,visible) in moving_arrows.items():
        np.testing.assert_allclose(fig.continuous_vectors.arrows[k].xy,xy)
        assert fig.continuous_vectors.arrows[k].get_visible()==visible
    assert [t.get_text() for t in fig.continuous_vectors.legend.get_texts()]==moving_labels
    np.testing.assert_allclose(fig.continuous_vectors.true_line.get_data(),moving_truth)
    update(len(data['time'])-1)
    r=fig.continuous_vectors
    for k,arrow in v.arrows.items():
        np.testing.assert_allclose(arrow.xy,r.arrows[k].xy)
        assert arrow.get_visible()==r.arrows[k].get_visible()
        assert arrow.arrow_patch.get_edgecolor()==r.arrows[k].arrow_patch.get_edgecolor()
        assert arrow.arrow_patch.get_linestyle()==r.arrows[k].arrow_patch.get_linestyle()
    assert [t.get_text() for t in v.vectors.legend.get_texts()]==[t.get_text() for t in r.legend.get_texts()]
    np.testing.assert_allclose(v.vectors.true_line.get_data(),r.true_line.get_data())
    for size in [(16,9),(12,8)]:
        for f,redraw,xy,vectors in [(v.figure,v.draw,v.xy,v.vectors),(fig,lambda:update(len(data['time'])-1),fig.axes[0],r)]:
            f.set_size_inches(*size);redraw();f.canvas.draw();renderer=f.canvas.get_renderer()
            legend=vectors.legend.get_window_extent(renderer)
            assert legend.x0>=0 and legend.x1<=f.bbox.x1
            assert legend.y0>xy.get_window_extent(renderer).y1
            assert legend.y1<f.bbox.y1
            text_boxes=[t.get_window_extent(renderer) for t in vectors.legend.get_texts()]
            for i,box in enumerate(text_boxes):
                assert all(not box.overlaps(other) for other in text_boxes[i+1:])
            assert not vectors.boundary_text.get_window_extent(renderer).overlaps(vectors.robot_text.get_window_extent(renderer))
        assert v.vectors.legend.get_window_extent(v.figure.canvas.get_renderer()).y1<v.status_text.get_window_extent(v.figure.canvas.get_renderer()).y0
    v.debug=False;v.draw();assert not v.vectors.legend.get_visible()
    assert v.executed.get_visible() and not v.event_points.get_visible() and not v.probe.get_visible()
    assert not v.vectors.true_line.get_visible()
    assert all(not a.get_visible() for k,a in v.arrows.items() if k not in ('boundary','robot_estimate'))
    fig.continuous_debug=False;update(len(data['time'])-1)
    assert not r.legend.get_visible() and not r.true_line.get_visible()
    assert not fig.continuous_truth_button.ax.get_visible()
    plt.close(v.figure);plt.close(fig)


def test_real_replay_never_offers_or_infers_truth(tmp_path):
    from config.loader import load_config
    from simulation.simulator import load_simulation_config
    from simulation.continuous_preview import PreviewRun
    from tools.visualize_continuous_run import read_run,make_figure
    root=Path(__file__).resolve().parents[1]
    cfg=load_config(root/'config.yaml');cfg['continuous_tracking']['max_runtime_sec']=1
    m=PreviewRun(cfg,load_simulation_config(root/'simulation/scene_continuous.yaml'),tmp_path)
    m.start(0);m.tick(.1,work_budget_sec=10);m.stop()
    data,cfg,events=read_run(m.run_dir)
    # An offline fixture with real-log metadata; no device imports or connection.
    cfg.pop('continuous_simulation')
    cfg['force_display']=dict(schema_version=1,frame='Base',force_source='processed_wrench',
        force_convention='unconfirmed',estimate_method='quasistatic_planar_balance')
    fig,update=make_figure(data,cfg,events,debug=True,components=True,true_tangent=True)
    assert not fig.continuous_truth_button.ax.get_visible()
    assert not fig.continuous_vectors.true_line.get_visible()
    assert '不可用' in fig.continuous_vectors.truth_note
    assert 'true_tangent' not in fig.continuous_vectors.legend_keys
    labels=' '.join(t.get_text() for t in fig.continuous_vectors.legend.get_texts())
    assert 'Measured environment resultant' in labels and 'processed Base' in labels
    assert 'Normal reaction' not in labels
    fig.continuous_component_button._observers.process('clicked',None)
    assert not set(view.COMPONENT_KEYS)&set(fig.continuous_vectors.legend_keys)
    plt.close(fig)
