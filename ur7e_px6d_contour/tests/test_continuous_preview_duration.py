"""Manual preview lifetime; all tests have bounded steps or injected shutdown."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import csv
import json
import numpy as np
import pytest
import yaml
from config.loader import load_config
from policy.continuous_tracking import ContinuousTrackingPolicy, State
from simulation.continuous_preview import PreviewRun, ContinuousPreview, launch_preview
from simulation.simulator import load_simulation_config

ROOT=Path(__file__).resolve().parents[1]

@pytest.mark.parametrize('duration',[None,.05,180.])
def test_preview_cli_overrides_only_its_config_copy(tmp_path,monkeypatch,duration):
    from matplotlib.backends.registry import backend_registry
    cfg=load_config(ROOT/'config.yaml');original=deepcopy(cfg);seen=[]
    monkeypatch.setattr(backend_registry,'resolve_backend',lambda _:('unused','offline-test'))
    # No GUI/hardware, no unbounded run: inspect the constructed session only.
    monkeypatch.setattr(ContinuousPreview,'show',lambda self:seen.append(self.model))
    args=SimpleNamespace(duration=duration,output=tmp_path,scene=ROOT/'simulation/scene_continuous.yaml')
    assert launch_preview(args,cfg,{})==0
    m=seen[0];assert m.session.policy.c['max_runtime_sec']==duration
    assert cfg==original and cfg['continuous_tracking']['max_runtime_sec']==120
    m.reset();assert m.session.policy.c['max_runtime_sec']==duration
    import matplotlib.pyplot as plt
    plt.close('all')

def test_manual_preview_runs_past_120_with_bounded_cache_and_complete_logs(tmp_path):
    cfg=load_config(ROOT/'config.yaml');cfg['continuous_tracking']['max_runtime_sec']=None
    m=PreviewRun(cfg,load_simulation_config(ROOT/'simulation/scene_direction_triangle.yaml'),tmp_path)
    m.start(0);m.set_speed(10,0)
    try:
        for i in range(130):
            m.tick((i+1)*.1,work_budget_sec=10)
            assert m.status=='RUNNING'
        assert m.session.robot.time>129.9
        assert m.session.policy.state==State.CONTINUOUS_TRACKING
        assert len(m.history)<=3001 and len(m.path_history)<=4096 and len(m.events)<=200
        assert not m.session.policy.events
        # CSV is already flushed while running; it is not just a final dump.
        with (m.run_dir/'samples.csv').open() as f:assert sum(1 for _ in f)>12000
    finally:m.stop('test operator stop')
    m.stop()
    with (m.run_dir/'samples.csv').open() as f:rows=list(csv.DictReader(f))
    assert len(rows)>=13000
    assert yaml.safe_load((m.run_dir/'config_snapshot.yaml').read_text())['continuous_tracking']['max_runtime_sec'] is None
    assert json.loads((m.run_dir/'summary.json').read_text())['termination_reason']=='STOP_USER_REQUEST'

@pytest.mark.parametrize('budget',[0,-1,float('inf'),float('nan'),True])
def test_invalid_runtime_budget_rejected(budget):
    cfg=load_config(ROOT/'config.yaml');cfg['continuous_tracking']['max_runtime_sec']=budget
    with pytest.raises(ValueError,match='max_runtime_sec'):ContinuousTrackingPolicy(cfg)

def test_real_entry_rejects_null_before_device_or_site_work(monkeypatch):
    import run_continuous_tracking as runner
    cfg=load_config(ROOT/'config.yaml');cfg['continuous_tracking']['max_runtime_sec']=None
    monkeypatch.setattr(runner,'validate_execution_configuration',lambda _:pytest.fail('real setup started'))
    with pytest.raises(runner.RobotError,match='max_runtime_sec'):runner.prepare_real(cfg,ROOT/'config.yaml')

@pytest.mark.parametrize('failure,code',[
    ('force','STOP_FORCE_LIMIT'),('torque','STOP_FORCE_LIMIT'),('nonfinite','STOP_UNKNOWN_REASON'),
    ('stale','STOP_STALE_DATA'),('direction','STOP_DIRECTION_UNCONFIRMED'),('search','STOP_SEARCH_LIMIT')])
def test_null_runtime_does_not_disable_protections(failure,code):
    from core.models import RobotState,Wrench
    cfg=load_config(ROOT/'config.yaml');cfg['continuous_tracking']['max_runtime_sec']=None
    p=ContinuousTrackingPolicy(cfg)
    def step(t,force=(1.5,0,0),torque=(0,0,0),speed=0,xy=(0,0)):
        w=Wrench(*force,*torque)
        return p.update(t,w,w,RobotState(t,np.r_[xy,0,0,0,0],np.array([speed,0,0,0,0,0])))
    if failure=='search':
        step(0,(0,0,0));cmd=step(.01,(0,0,0),xy=(.101,0))
    else:
        for i in range(101):step(i*.01)
        if failure=='force':cmd=step(1.01,(100,0,0))
        elif failure=='torque':cmd=step(1.01,torque=(10,0,0))
        elif failure=='nonfinite':cmd=step(1.01,(float('nan'),0,0))
        elif failure=='stale':cmd=step(1.1)
        else:
            for i in range(101,204):cmd=step(i*.01,(.75,1.5*np.sqrt(3)/2,0),speed=.002)
    assert p.state==State.STOP and not cmd.move and p.stop_reason.value==code

def test_manual_preview_log_failure_stops_and_closes(tmp_path,monkeypatch):
    cfg=load_config(ROOT/'config.yaml');cfg['continuous_tracking']['max_runtime_sec']=None
    m=PreviewRun(cfg,load_simulation_config(ROOT/'simulation/scene_continuous.yaml'),tmp_path);m.start(0)
    logger=m.logger
    def fail(*args,**kwargs):raise OSError('injected disk write failure')
    monkeypatch.setattr(logger,'log_sample',fail)
    with pytest.raises(OSError,match='disk write'):m.tick(.01,work_budget_sec=10)
    assert m.status=='ENDED' and m.logger is None and logger._sample_file.closed
    assert 'disk write' in m.session.policy.reason
    m.stop()

def test_interrupt_during_bias_is_user_stop_and_still_closes_logs(tmp_path,monkeypatch):
    cfg=load_config(ROOT/'config.yaml');cfg['continuous_tracking']['max_runtime_sec']=None
    m=PreviewRun(cfg,load_simulation_config(ROOT/'simulation/scene_continuous.yaml'),tmp_path)
    def interrupt(**kwargs):raise KeyboardInterrupt
    monkeypatch.setattr(m.session,'capture_bias',interrupt)
    with pytest.raises(KeyboardInterrupt):m.start(0)
    assert m.status=='ENDED' and m.logger is None
    summary=json.loads((m.run_dir/'summary.json').read_text())
    assert summary['termination_reason']=='STOP_USER_REQUEST'
    m.stop()

@pytest.mark.parametrize('shutdown',['q','escape','close','interrupt'])
def test_manual_shutdown_paths_are_idempotent(tmp_path,monkeypatch,shutdown):
    import matplotlib;matplotlib.use('Agg',force=True)
    import matplotlib.pyplot as plt
    cfg=load_config(ROOT/'config.yaml');cfg['continuous_tracking']['max_runtime_sec']=None
    m=PreviewRun(cfg,load_simulation_config(ROOT/'simulation/scene_continuous.yaml'),tmp_path)
    v=ContinuousPreview(m);m.start(0);m.tick(.03,work_budget_sec=10);logger=m.logger
    if shutdown=='interrupt':
        def interrupt():raise KeyboardInterrupt
        monkeypatch.setattr(plt,'show',interrupt);v.show()
    elif shutdown=='close':v.on_close(None)
    else:
        v.toggle_settings();v.path_box.capturekeystrokes=True
        v.on_key(SimpleNamespace(key=shutdown))
    assert m.status=='ENDED' and m.session.policy.stop_reason.value=='STOP_USER_REQUEST'
    v.on_close(None);m.stop();assert logger._sample_file.closed
    plt.close(v.figure)

@pytest.mark.parametrize('debug,settings',[(False,False),(False,True),(True,False),(True,True)])
def test_footer_has_dedicated_space_and_manual_mode_label(tmp_path,debug,settings):
    import matplotlib;matplotlib.use('Agg',force=True)
    import matplotlib.pyplot as plt
    cfg=load_config(ROOT/'config.yaml');cfg['continuous_tracking']['max_runtime_sec']=None
    m=PreviewRun(cfg,load_simulation_config(ROOT/'simulation/scene_continuous.yaml'),tmp_path)
    v=ContinuousPreview(m);v.debug=debug
    if settings:v.toggle_settings()
    m.message='Saved: /a/long/experiment/path/'*8
    for size in [(16,9),(12,8)]:
        v.figure.set_size_inches(*size);v.draw();v.figure.canvas.draw()
        renderer=v.figure.canvas.get_renderer();box=v.force_note.get_window_extent(renderer)
        assert '仿真时间：' in v.status_text.get_text() and '手动停止模式' in v.status_text.get_text()
        assert box.y1 < v.xy.xaxis.label.get_window_extent(renderer).y0
        for button in v.buttons:
            if button.ax.get_visible():assert box.y0>button.ax.get_window_extent(renderer).y1
        if settings:assert box.y0>v.path_box.ax.get_window_extent(renderer).y1
        assert box.x0>=0 and box.x1<=v.figure.bbox.x1
        assert not v.geometry_text.get_visible() and not v.event_text.get_visible() and not v.message_text.get_visible()
        if debug:
            assert not v.force.get_xlabel()
            assert v.velocity.title.get_window_extent(renderer).y1<v.force.get_window_extent(renderer).y0
    plt.close(v.figure)
