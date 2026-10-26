"""Independent raw-point updates and preview/executor agreement, without hardware."""
from copy import deepcopy
import numpy as np
import pytest
import yaml

from app.sandbox_scan import ROOT, make_plan, read_calibration
from calibration import sandbox_scan as calibration
from doubles import Devices
from policy.coverage_search import CoverageSearch
from test_sandbox_scan import record_for, rectangle, coverage_geometry
from visualization.sandbox_scan import describe_route, plot_plan
from workspace.sandbox_scan import plan_coverage


def saved_legacy_record(config, path):
    # A real pre-extension shape, including a derived cache which must not be
    # rewritten just because P0 is re-taught.
    record = record_for(config, size=.04)
    record['workspace']['metadata']['operator_note'] = 'preserve corners'
    record['captured_at'] = dict(legacy_timestamp=123)
    record['other_setting'] = dict(keep=True)
    data = dict(sandbox_scan=record, unrelated=dict(keep='original'))
    path.write_text(yaml.safe_dump(data))
    return data


@pytest.mark.parametrize('scope', ['p0','corners','all'])
def test_independent_teaching_preserves_other_fields_and_scan_height(config,tmp_path,monkeypatch,scope):
    path=tmp_path/'saved.yaml';before=saved_legacy_record(config,path)
    record=before['sandbox_scan']
    new_p0=[.61,.11,.43,0.,np.pi,0.]
    corners=[]
    for i in range(4):
        old=record['workspace']['raw_points'][f'P{i}']
        corners.append([old['x']+.001,old['y'],.15+i*.02,old['rx'],old['ry'],old['rz']])
    readings=iter([new_p0] if scope=='p0' else corners if scope=='corners' else [new_p0]+corners)
    devices=Devices(config,new_p0)
    prompts=[]
    def confirm(prompt):
        prompts.append(prompt);devices.receive.pose=np.array(next(readings));return True
    monkeypatch.setattr(calibration,'read_tcp_offset_readonly',lambda _:config['tcp']['offset'])
    calibration.teach(config,path,scope=scope,controller_factory=devices.controller,confirm=confirm)
    after=yaml.safe_load(path.read_text());updated=after['sandbox_scan']
    assert updated['scan_z']==record['scan_z']==.03  # New P0.z is .43, not scan Z.
    assert after['unrelated']==before['unrelated']
    assert updated['other_setting']==record['other_setting']
    if scope=='p0':
        assert len(prompts)==1 and 'P0' in prompts[0]
        assert updated['p0_tcp_pose']==new_p0
        assert {k:v for k,v in updated.items() if k not in ('p0_tcp_pose','p0_captured_at')} == {
            k:v for k,v in record.items() if k not in ('p0_tcp_pose','p0_captured_at')}
    elif scope=='corners':
        assert len(prompts)==4 and all(f'C{i}' in prompt for i,prompt in enumerate(prompts))
        assert {k:v for k,v in updated.items() if k not in ('workspace','corners_captured_at')} == {
            k:v for k,v in record.items() if k not in ('workspace','corners_captured_at')}
    else:
        assert len(prompts)==5 and updated['p0_tcp_pose']==new_p0
    if scope!='p0':
        assert updated['workspace']['metadata']==record['workspace']['metadata']
        for i,pose in enumerate(corners):
            assert list(updated['workspace']['raw_points'][f'P{i}'].values())==pose
    assert devices.receive_count==1 and devices.control_count==0 and not devices.control.calls
    assert not devices.receive.connected
    assert list(tmp_path.glob('*.backup-*.yaml'))


@pytest.mark.parametrize('scope,cancel_after', [('p0',0),('corners',2),('all',3)])
def test_partial_cancellation_preserves_file_and_input_record(config,tmp_path,monkeypatch,scope,cancel_after):
    path=tmp_path/'saved.yaml';saved_legacy_record(config,path)
    original=path.read_bytes();existing=calibration.load_record(path);unchanged=deepcopy(existing)
    devices=Devices(config,[.62,.12,.35,0.,np.pi,0.]);calls=[]
    def confirm(_):
        calls.append(True);return len(calls)<=cancel_after
    monkeypatch.setattr(calibration,'read_tcp_offset_readonly',lambda _:config['tcp']['offset'])
    with pytest.raises(KeyboardInterrupt,match='取消'):
        calibration.teach(config,path,scope=scope,existing=existing,
                          controller_factory=devices.controller,confirm=confirm)
    assert path.read_bytes()==original and existing==unchanged
    assert not list(tmp_path.glob('*.backup-*.yaml'))
    assert devices.control_count==0 and not devices.receive.connected


def test_p0_only_can_import_old_calibrations_without_changing_them(config,tmp_path,monkeypatch):
    path=tmp_path/'sandbox.yaml'
    config['sandbox_scan']=dict(calibration_file=str(path))
    old_paths=[ROOT/'scan_calibration.yaml',ROOT/'workspace/config/workspace_calibration.yaml']
    old_bytes=[p.read_bytes() for p in old_paths]
    existing=read_calibration(config,ROOT/'config.yaml',path);before=deepcopy(existing)
    devices=Devices(config,[.61,.11,.43,0.,np.pi,0.])
    monkeypatch.setattr(calibration,'read_tcp_offset_readonly',lambda _:config['tcp']['offset'])
    calibration.teach(config,path,scope='p0',existing=existing,
                      controller_factory=devices.controller,confirm=lambda _:True)
    result=calibration.load_record(path)
    assert result['workspace']==before['workspace'] and result['scan_z']==before['scan_z']
    assert result['p0_tcp_pose'][2]==.43 and existing==before
    assert [p.read_bytes() for p in old_paths]==old_bytes


@pytest.mark.parametrize('choice,scope', [('2','p0'),('3','corners'),('4','all')])
def test_menu_independent_teaching_replans_without_starting_motion(config,tmp_path,monkeypatch,choice,scope):
    import run_sandbox_scan as entry
    path=tmp_path/'saved.yaml';saved_legacy_record(config,path)
    config['sandbox_scan']=dict(calibration_file=str(path),boundary_margin=.002,line_spacing=.01)
    config['continuous_tracking']['workspace_calibration_file']=str(ROOT/'workspace/config/workspace_calibration.yaml')
    monkeypatch.setattr(entry,'load_config',lambda _:deepcopy(config))
    monkeypatch.setattr('builtins.input',lambda _:choice)
    calls=[]
    def teach(project,path,**kwargs):
        assert kwargs['scope']==scope
        record=calibration.load_record(path);record['p0_tcp_pose'][:2]=[.63,.13]
        calibration.save_record(path,record)
    monkeypatch.setattr(entry,'teach',teach)
    def preview(plan,directory,**kwargs):
        assert kwargs['show'] is True
        calls.append(plan);return tmp_path/'preview.png'
    monkeypatch.setattr(entry,'save_preview',preview)
    monkeypatch.setattr(entry,'run_sandbox',lambda *a,**k:pytest.fail('teaching must not start scanning'))
    assert entry.main([])==0
    assert len(calls)==1
    assert calls[0]==make_plan(config,calibration.load_record(path))
    assert calls[0]['p0_tcp_pose'][:2]==[.63,.13]


@pytest.mark.parametrize('option,scope', [('--recalibrate-p0','p0'),('--recalibrate-corners','corners'),('--recalibrate','all')])
def test_calibration_flags_select_scope_and_refresh_preview(config,tmp_path,monkeypatch,option,scope):
    import run_sandbox_scan as entry
    path=tmp_path/'saved.yaml';saved_legacy_record(config,path)
    config['continuous_tracking']['workspace_calibration_file']=str(ROOT/'workspace/config/workspace_calibration.yaml')
    monkeypatch.setattr(entry,'load_config',lambda _:deepcopy(config))
    scopes=[];previews=[]
    monkeypatch.setattr(entry,'teach',lambda c,p,**kwargs:scopes.append(kwargs['scope']))
    monkeypatch.setattr(entry,'save_preview',lambda plan,directory,**kwargs:previews.append(plan) or tmp_path/'p.png')
    monkeypatch.setattr(entry,'run_sandbox',lambda *a,**k:pytest.fail('calibration-only motion'))
    assert entry.main(['--calibration',str(path),option])==0
    assert scopes==[scope] and len(previews)==1


@pytest.mark.parametrize('p0,location,entry', [((.59,.12),'outside_raw','lift_translate_descend'),
    ((.601,.12),'inside_raw_outside_effective','lift_translate_descend'),
    ((.62,.12),'inside_effective','planar_search')])
def test_p0_location_explanation_does_not_restrict_or_change_entry(config,p0,location,entry):
    config['sandbox_scan']=dict(boundary_margin=.004,line_spacing=.01)
    plan=make_plan(config,record_for(config,p0=p0,size=.04));before=deepcopy(plan)
    info=describe_route(plan)
    assert info['p0_location']==location and plan['entry_kind']==entry
    assert plan==before


@pytest.mark.parametrize('angle,reverse', [(0.,False),(31.,False),(133.,True)])
def test_preview_order_and_directions_match_executor_points(config,angle,reverse):
    region=rectangle(angle=angle,reverse=reverse)
    plan=plan_coverage(coverage_geometry(region,.004),[.6,.1,.25,0.,np.pi,0.],scan_z=.03,line_spacing=.019)
    points=CoverageSearch(plan,np.array(plan['p0_tcp_pose']),config['policy']['position_tolerance']).points
    info=describe_route(plan)
    for i,segment in enumerate(info['segments']):
        np.testing.assert_array_equal([segment['start_xy'],segment['end_xy']],points[i:i+2])
    assert [s['kind'] for s in info['first_steps']]==['scan','connector','scan','connector','scan','connector','scan']
    scans=np.diff(points,axis=0)[::2]
    assert np.all(np.sum(scans[:-1]*scans[1:],axis=1)<0)  # Alternating actual direction.
    assert info['line_count']==len(points)//2
    assert info['actual_spacing_m']==pytest.approx(plan['actual_line_spacing_m'])
    # Display ignores potentially stale candidate metadata: ordered points win.
    plan.update(scan_axis=1-plan['scan_axis'],reverse_lines=not plan['reverse_lines'],reverse_first=not plan['reverse_first'])
    assert describe_route(plan)==info


@pytest.mark.parametrize('axis,expected', [(0,'短边'),(1,'长边')])
def test_preview_side_description_uses_real_geometry(config,axis,expected):
    plan=make_plan(config,calibration.load_record(ROOT/'calibration_data/sandbox_scan.yaml'))
    corners=np.array(plan['effective_corners_xy'])
    # Two valid routes with different along-scan axes, regardless of metadata.
    order=[0,1,2,3] if axis==0 else [0,3,2,1]
    plan['coverage_xy']=corners[order].tolist()
    info=describe_route(plan)
    assert info['scan_side']==expected
    assert info['first_steps'][0]['direction']!=info['first_steps'][2]['direction']


@pytest.mark.parametrize('size,dpi', [((16,10),100),((11,7.1),100),((5.5,3.55),200)])
def test_preview_text_fits_and_arrows_follow_the_same_ordered_path(config,size,dpi):
    from matplotlib.backend_bases import ResizeEvent
    plan=make_plan(config,calibration.load_record(ROOT/'calibration_data/sandbox_scan.yaml'))
    fig=plot_plan(plan);fig.set_size_inches(*size);fig.set_dpi(dpi)
    fig.canvas.callbacks.process('resize_event',ResizeEvent('resize_event',fig.canvas))
    fig.canvas.draw()
    renderer=fig.canvas.get_renderer();main,information,zoom,_=fig.axes
    text_bounds=[]
    for text in information.texts:
        bounds=text.get_window_extent(renderer)
        assert fig.bbox.contains(bounds.x0,bounds.y0) and fig.bbox.contains(bounds.x1,bounds.y1)
        text_bounds.append(bounds)
    assert not any(a.overlaps(b) for a,b in zip(text_bounds,text_bounds[1:]))
    assert text_bounds[-1].y0>zoom.get_tightbbox(renderer).y1
    assert main.get_aspect()==zoom.get_aspect()==1.
    route=np.array(plan['coverage_xy'])*1000
    arrows=[t for t in main.texts if t.get_text()=='' and t.arrow_patch is not None]
    assert len(arrows)>=len(route)-1
    for arrow,(a,b) in zip(arrows,zip(route,route[1:])):
        vector=np.asarray(arrow.xy)-np.asarray(arrow.xyann)
        assert np.dot(vector,b-a)>0
        np.testing.assert_allclose((np.asarray(arrow.xy)+arrow.xyann)/2,(a+b)/2)
    # Every scan is a separate ordered artist; connectors sit above boundaries.
    segments=main.lines[3:3+len(route)-1]
    assert len({line.get_color() for line in segments[0:7:2]})==4
    assert segments[1].get_color()!=segments[0].get_color()
    assert segments[1].get_zorder()>main.lines[2].get_zorder()
    for i,line in enumerate(zoom.lines[3:]):
        np.testing.assert_array_equal(line.get_xydata(),route[i:i+2])
