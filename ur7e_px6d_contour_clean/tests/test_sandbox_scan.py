"""Coverage geometry, compatibility and shared runtime handoff; hardware forbidden by conftest."""
from copy import deepcopy
import json

import numpy as np
import pytest
import yaml

from app.configuration import prepare_real
from app.sandbox_scan import ROOT, make_plan, prepare_sandbox
from calibration.sandbox_scan import load_record, save_record
from calibration.scan_calibration import load_scan_calibration
from core.models import RobotState, Wrench
from policy.continuous_tracking import ContinuousTrackingPolicy, State
from policy.coverage_search import CoverageSearch
from safety.safe_return import return_trajectory
from simulation.simulated_robot import SimulatedRobot
from workspace.sandbox_scan import plan_coverage
from workspace.workspace_calibrator import fit_workspace, effective_scan_rectangle
from workspace.workspace_transform import WorkspaceTransform

ZERO = Wrench(0.,0.,0.,0.,0.,0.)


def coverage_geometry(workspace, margin):
    return effective_scan_rectangle(fit_workspace(workspace['raw_points'], method='least_squares'), margin)


def rectangle(angle=31., noise=0., reverse=False, heights=(0.,0.,0.,0.)):
    theta = np.deg2rad(angle)
    rotation = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
    xy = np.array([[0,0],[.137,0],[.137,.083],[0,.083]]) @ rotation.T + [.6,.1]
    xy += noise*np.array([[.2,-.1],[-.1,.3],[.3,.1],[-.2,-.2]])
    poses = [np.r_[point,z,0.,np.pi,0.] for point,z in zip(xy,heights)]
    if reverse:
        poses = poses[::-1]
    return fit_workspace({f'P{i}':pose for i,pose in enumerate(poses)})


@pytest.mark.parametrize('angle', [0.,31.,89.,133.])
@pytest.mark.parametrize('reverse', [False,True])
def test_rotated_noisy_rectangles_last_line_and_all_segments(angle, reverse):
    region = fit_workspace(rectangle(angle, noise=.002, reverse=reverse, heights=(-2.,.3,9.,1.))['raw_points'],
                           method='least_squares')
    geometry = effective_scan_rectangle(region, .004)
    transform = WorkspaceTransform(region)
    corners = np.array([region['rectified_points'][f'R{i}'] for i in range(4)])
    edges = np.diff(np.vstack((corners[:,:2],corners[0,:2])),axis=0)
    assert np.dot(edges[0],edges[1]) == pytest.approx(0.,abs=1e-12)
    assert region['diagnostics']['fit_max_xy_m'] < .002
    for p0_local in ([-.02,-.03],[.04,.05],[.15,.1],[-.02,.05],[.07,.04]):
        p0 = np.r_[transform.workspace_to_base([*p0_local,0.]), 0.,np.pi,0.]
        plan = plan_coverage(geometry,p0,scan_z=-.05,line_spacing=.019)
        route = np.array(plan['coverage_xy'])
        samples = np.vstack([a+(b-a)*np.linspace(0,1,21)[:,None] for a,b in zip(route,route[1:])])
        local = transform.base_to_workspace(np.c_[samples,np.zeros(len(samples))])[:,:2]
        low,high = np.array(geometry['local_min_xy']),np.array(geometry['local_max_xy'])
        assert np.all(local >= low-1e-12) and np.all(local <= high+1e-12)
        path_local = transform.base_to_workspace(np.c_[route,np.zeros(len(route))])[:,:2]
        cross = 1-plan['scan_axis']
        offsets = path_local[::2,cross]
        assert min(offsets) == pytest.approx(low[cross])
        assert max(offsets) == pytest.approx(high[cross])
        assert np.max(abs(np.diff(offsets))) <= .019+1e-12
        effective = np.array(plan['effective_corners_xy'])
        assert np.linalg.norm(route[0]-p0[:2]) == pytest.approx(min(np.linalg.norm(effective-p0[:2],axis=1)))
        assert plan['scan_z'] == -.05


def test_corner_heights_have_no_effect_on_xy_or_scan_z():
    a,b = rectangle(),rectangle(heights=(-20.,4.,7.,.5))
    p0 = [.6,.1,.2,0.,np.pi,0.]
    plans = [plan_coverage(coverage_geometry(r,.005),p0,scan_z=.03,line_spacing=.02) for r in (a,b)]
    for field in ('coverage_xy','rectified_corners_xy','effective_corners_xy','entry_xy'):
        np.testing.assert_allclose(plans[0][field],plans[1][field],atol=1e-12)


@pytest.mark.parametrize('spacing,margin,message', [(0.,0.,'间距'),(-1.,0.,'间距'),(np.nan,0.,'间距'),(.01,.1,'为空'),(.01,-.01,'余量')])
def test_invalid_planning_parameters(spacing,margin,message):
    with pytest.raises(ValueError,match=message):
        plan_coverage(coverage_geometry(rectangle(),margin),[.6,.1,0.,0.,0.,0.],scan_z=0.,line_spacing=spacing)


def test_degenerate_corners_fail_before_planning():
    with pytest.raises(ValueError,match='duplicate|collinear'):
        fit_workspace({f'P{i}':[i*.01,0.,i,0.,0.,0.] for i in range(4)})


def record_for(config, *, p0=(.605,.105), size=.012):
    xy = np.array([[.6,.1],[.6+size,.1],[.6+size,.1+size],[.6,.1+size]])
    region = fit_workspace({f'P{i}':np.r_[point,.2+i*.01,0.,np.pi,0.] for i,point in enumerate(xy)})
    return dict(robot_ip=config['robot']['robot_ip'],active_tcp_offset=config['tcp']['offset'],
        p0_tcp_pose=[*p0,.25,0.,np.pi,0.],scan_z=.03,workspace=region)


def test_old_config_calibration_and_default_policy_unchanged(config,tmp_path):
    scan = ROOT/'scan_calibration.yaml'
    old = scan.read_bytes()
    original = load_scan_calibration(scan)
    before = deepcopy(config); prepare_real(before,ROOT/'config.yaml')
    path = tmp_path/'new_scan.yaml'; save_record(path,record_for(config))
    assert load_record(path)['scan_z'] == .03
    assert scan.read_bytes() == old and load_scan_calibration(scan) == original
    after = deepcopy(config); prepare_real(after,ROOT/'config.yaml')
    assert before == after
    policies = [ContinuousTrackingPolicy(deepcopy(config)),ContinuousTrackingPolicy(deepcopy(config),search_strategy=None)]
    pose = np.array([.5,.1,0.,0.,0.,0.])
    for i in range(40):
        now = i*.01
        sample = ZERO if i < 15 else Wrench(1.5,0.,0.,0.,0.,0.)
        cmds = [p.update(now,sample,sample,RobotState(now,pose.copy(),np.zeros(6))) for p in policies]
        assert cmds[0].state == cmds[1].state
        np.testing.assert_array_equal(cmds[0].direction_xy*cmds[0].speed,cmds[1].direction_xy*cmds[1].speed)
    assert [e.event_type for e in policies[0].events] == [e.event_type for e in policies[1].events]


def test_extension_save_preserves_existing_keys_and_backup(config,tmp_path):
    path = tmp_path/'calibration.yaml'
    existing = yaml.safe_load((ROOT/'scan_calibration.yaml').read_text())
    path.write_text(yaml.safe_dump(existing)); old = path.read_bytes()
    save_record(path,record_for(config))
    data = yaml.safe_load(path.read_text())
    assert {k:v for k,v in data.items() if k != 'sandbox_scan'} == existing
    assert next(tmp_path.glob('*.backup-*.yaml')).read_bytes() == old


def test_outside_entry_lifts_and_does_not_replace_tracking_boundaries(config,tmp_path):
    record = record_for(config,p0=(.3,.1))
    config['sandbox_scan'] = dict(boundary_margin=.002,line_spacing=.005)
    path = tmp_path/'scan.yaml';save_record(path,record)
    robot,start = prepare_sandbox(config,ROOT/'config.yaml',path)
    plan = config['sandbox_scan_plan']
    assert plan['entry_kind'] == 'lift_translate_descend' and robot['continuous_search_geometry'] is None
    original = deepcopy(config); original.pop('sandbox_scan_plan')
    baseline,_ = prepare_real(original,ROOT/'config.yaml')
    assert robot['sandbox_entry_xy_guard']['continuous_xy_polygon'] == baseline['continuous_xy_polygon']
    assert robot['continuous_tracking_boundary_margin'] == baseline['continuous_tracking_boundary_margin']
    p0 = np.array(record['p0_tcp_pose'])
    segments = return_trajectory(config,p0,start,via_pose=p0)
    assert segments[0][0] == 'VERTICAL_RETREAT'
    for phase,pose,_ in segments[:-1]:
        assert pose[2] >= max(p0[2],start[2]+config['safe_return']['return_lift_distance'])-1e-12
    assert segments[-1][0] == 'DESCEND_TO_START'
    np.testing.assert_allclose(segments[-1][1],start)


def test_preview_is_offline_and_equal_scale(config,tmp_path):
    from run_sandbox_scan import main
    from visualization.sandbox_scan import plot_plan
    config['sandbox_scan'] = dict(boundary_margin=.002,line_spacing=.005)
    config_file = tmp_path/'config.yaml'
    config['continuous_tracking']['workspace_calibration_file'] = str(ROOT/'workspace/config/workspace_calibration.yaml')
    config_file.write_text(yaml.safe_dump(config))
    calibration = tmp_path/'saved.yaml';save_record(calibration,record_for(config))
    preview = tmp_path/'preview'
    assert main(['--config',str(config_file),'--calibration',str(calibration),'--preview','--preview-dir',str(preview)]) == 0
    assert (preview/'sandbox_scan_preview.png').is_file()
    plan=json.loads((preview/'sandbox_scan_plan.json').read_text())
    fig=plot_plan(plan);fig.canvas.draw()
    assert fig.axes[0].get_aspect() == 1.
    ax=fig.axes[0]
    assert np.diff(ax.get_xticks())[0] == pytest.approx(np.diff(ax.get_yticks())[0])
    import matplotlib.pyplot as plt
    plt.close(fig)


def test_full_route_with_delayed_accelerating_simulator(config):
    record = record_for(config,size=.025)
    config['sandbox_scan'] = dict(line_spacing=.007,boundary_margin=.002)
    plan=make_plan(config,record);start=np.array(plan['runtime_start_pose'])
    strategy=CoverageSearch(plan,start,config['policy']['position_tolerance'])
    policy=ContinuousTrackingPolicy(config,search_strategy=strategy)
    robot=SimulatedRobot(start[:2],.01,dict(x_min=.5,x_max=.8,y_min=0.,y_max=.3),acceleration_limit=.005,delay_steps=2)
    visited=[]
    for i in range(20000):
        state=robot.read_state(); command=policy.update(robot.time,ZERO,ZERO,state)
        visited.append(state.pose[:2])
        if policy.state == State.STOP:
            break
        robot.apply_command(command.direction_xy,command.speed,command.move)
    assert strategy.completed and strategy.index == len(plan['coverage_xy'])
    assert policy.reason == '区域扫描完成，未检测到目标' and policy.stop_reason.value == 'SUCCESS'
    points=np.array(visited)
    assert np.all(points >= [.602-1e-10,.102-1e-10]) and np.all(points <= [.623+1e-10,.123+1e-10])
    reached=[e.pose[:2] for e in policy.events if e.event_type=='COVERAGE_WAYPOINT_REACHED']
    assert len(reached)==len(plan['coverage_xy'])
    assert np.max(np.linalg.norm(np.array(reached)-plan['coverage_xy'],axis=1)) <= config['policy']['position_tolerance']


def test_reverse_leg_contact_uses_original_confirmation_tracking_recovery(config):
    record=record_for(config,size=.025);config['sandbox_scan']=dict(boundary_margin=.002,line_spacing=.007)
    plan=make_plan(config,record);start=np.array(plan['runtime_start_pose'])
    strategy=CoverageSearch(plan,start,config['policy']['position_tolerance'])
    policy=ContinuousTrackingPolicy(config,search_strategy=strategy)
    policy.real_execution=True;policy.c['force_direction_sign']=-1
    strategy.index=3;strategy.origin=np.r_[plan['coverage_xy'][2],start[2:]]
    approach=np.array(plan['coverage_xy'][3])-np.array(plan['coverage_xy'][2]);approach/=np.linalg.norm(approach)
    pose=strategy.origin.copy();pose[:2]+=approach*.005
    sample=Wrench(*(-1.5*approach),0.,0.,0.,0.)
    for i in range(30):
        velocity=np.r_[approach*.005,0.,0.,0.,0.] if i==0 else np.zeros(6)
        policy.update(i*.01,sample,sample,RobotState(i*.01,pose.copy(),velocity),execution_settled=i>0)
    assert policy.initial_contact is not None and policy.state==State.CONTINUOUS_TRACKING
    np.testing.assert_allclose(policy.search_direction,approach)
    assert type(policy) is ContinuousTrackingPolicy
    assert policy._confirm.__func__ is ContinuousTrackingPolicy._confirm
    assert policy._recover.__func__ is ContinuousTrackingPolicy._recover
    before=strategy.index
    for i in range(30,160):
        policy.update(i*.01,ZERO,ZERO,RobotState(i*.01,pose.copy(),np.zeros(6)),execution_settled=True)
        if policy.state==State.LOCAL_REACQUIRE:
            break
    assert policy.state==State.LOCAL_REACQUIRE and strategy.index==before
    assert any(e.event_type=='LOCAL_REACQUIRE' for e in policy.events)


def test_teaching_reuses_readonly_capture_keeps_old_files_and_height(config,tmp_path,monkeypatch):
    from calibration import sandbox_scan as calibration
    from doubles import Devices
    old_scan=(ROOT/'scan_calibration.yaml').read_bytes()
    old_workspace=(ROOT/'workspace/config/workspace_calibration.yaml').read_bytes()
    devices=Devices(config,[.605,.105,.25,0.,np.pi,0.])
    poses=iter([[.605,.105,.25,0.,np.pi,0.],
                [.6,.1,.11,0.,np.pi,0.],[.62,.1,.22,0.,np.pi,0.],
                [.62,.12,.33,0.,np.pi,0.],[.6,.12,.44,0.,np.pi,0.]])
    def confirm(_):
        devices.receive.pose=np.array(next(poses));return True
    monkeypatch.setattr(calibration,'read_tcp_offset_readonly',lambda _:config['tcp']['offset'])
    path=tmp_path/'saved.yaml'
    data=calibration.teach(config,path,controller_factory=devices.controller,confirm=confirm)
    assert data['scan_z']==load_scan_calibration(ROOT/'scan_calibration.yaml')['fixed_z']
    assert load_record(path)['p0_tcp_pose'][2]==.25
    assert devices.control_count==0 and not devices.control.calls
    assert (ROOT/'scan_calibration.yaml').read_bytes()==old_scan
    assert (ROOT/'workspace/config/workspace_calibration.yaml').read_bytes()==old_workspace
    previous=path.read_bytes()
    with pytest.raises(KeyboardInterrupt):
        calibration.teach(config,path,controller_factory=Devices(config,[.6,.1,0.,0.,np.pi,0.]).controller,confirm=lambda _:False)
    assert path.read_bytes()==previous


@pytest.mark.parametrize('outside', [False,True])
def test_shared_real_runner_mock_entry_contact_and_arc_recovery(config,tmp_path,monkeypatch,outside):
    from argparse import Namespace
    import csv
    import time
    from app import continuous_runtime as runtime
    from app.sandbox_scan import run_sandbox
    from doubles import Devices, Sensor, Keyboard
    from robot.rtde_controller import _rotvec_to_matrix
    from test_continuous_runtime import prepare
    prepare(config,monkeypatch)
    # Preserve the real transform configuration; the sensor below converts its
    # synthetic Base reaction back into the supplied mount's sensor frame.
    config['preprocessing']['baseline']['sample_count']=2
    config['preprocessing']['granular_baseline']['capture_on_start']=False
    config['sandbox_scan']=dict(boundary_margin=.002,line_spacing=.005)
    config_file=tmp_path/'config.yaml'
    config['continuous_tracking']['workspace_calibration_file']=str(ROOT/'workspace/config/workspace_calibration.yaml')
    config_file.write_text(yaml.safe_dump(config))
    monkeypatch.setattr(runtime,'load_config',lambda _:deepcopy(config))
    record=record_for(config,p0=(.3,.1) if outside else (.605,.105))
    path=tmp_path/'saved.yaml';save_record(path,record)
    preview=make_plan(config,record)
    preflight=deepcopy(config);_,start=prepare_sandbox(preflight,config_file,path)
    initial=np.array(record['p0_tcp_pose']);initial[2]=start[2]
    if not outside:
        # The complete teach flow leaves TCP at its last corner, potentially
        # exactly on the original sandbox boundary. Startup must still lift.
        corner=preflight['continuous_workspace_calibration']['raw_points']['P3']
        initial[:3]=[corner[a] for a in 'xyz']
    devices=Devices(config,initial)
    instances=[]
    class Keys(Keyboard):
        def __enter__(self):
            instances.append(self);self.main=len(instances)>1;self.started=time.monotonic();return self
        def poll(self):
            return 'Q' if self.main and time.monotonic()-self.started>1.6 else None
    monkeypatch.setattr(runtime,'OperatorKeyboard',Keys)
    mount=np.array(config['preprocessing']['coordinate_transform']['rotation_sensor_to_tool'])
    class ContactSensor(Sensor):
        scan_reads=0
        def read_wrench(self):
            force=np.zeros(3)
            if not devices.owner._return_mode:
                self.scan_reads+=1
                # Build the observed approach from actual velocity, then hold
                # reaction through the original contact confirmation window.
                if self.scan_reads==12:
                    v=devices.receive.speed[:2]
                    self.approach=v/np.linalg.norm(v)
                if 12<=self.scan_reads<45:
                    force[:2]=-1.5*self.approach
            sensor_force=(_rotvec_to_matrix(devices.receive.pose[3:])@mount).T@force
            return Wrench(*sensor_force,0.,0.,0.)
    settings=Namespace(config=config_file,calibration=path,execute=True,preview=False,duration=None,
                       output=tmp_path/'logs',enable_reacquire=False)
    monkeypatch.setattr('visualization.sandbox_scan.generate_run_visualization',lambda _:[])
    assert run_sandbox(settings,preview,controller_factory=devices.controller,reader_factory=ContactSensor)==0
    run,=(settings.output/'real/continuous').glob('run_*')
    with (run/'samples.csv').open() as stream:
        rows=list(csv.DictReader(stream))
    states={r['current_state'] for r in rows}
    assert {'TARGET_SEARCH','FIRST_CONTACT','CONTINUOUS_TRACKING','LOCAL_REACQUIRE'}<=states
    assert devices.control_count==devices.receive_count==1
    assert not devices.receive.connected and not devices.control.connected
    assert 'sandbox_entry_xy_guard' not in devices.owner.config
    assert devices.owner.config['continuous_xy_polygon']==preflight['continuous_execution_envelope']['raw_xy_polygon']
    result=json.loads((run/'sandbox_scan_result.json').read_text())
    assert not result['completed'] and result['first_contact_pose'] is not None
    assert (run/'sandbox_scan_plan.json').is_file()
    moves=[call[1] for call in devices.control.calls if call[0]=='moveL']
    if outside:
        # Every horizontal move before insertion stays at the inherited safe height.
        assert all(p[2]>=record['p0_tcp_pose'][2] for p in moves[:-1])
        np.testing.assert_allclose(moves[-1][:3],start[:3])
        assert any(np.allclose(p[:2],record['p0_tcp_pose'][:2]) for p in moves)


def test_shared_runner_no_target_completes_normally_and_writes_overlay(config,tmp_path,monkeypatch,capsys):
    from argparse import Namespace
    import csv
    from app import continuous_runtime as runtime
    from app.sandbox_scan import run_sandbox
    from doubles import Devices, Sensor, Keyboard
    config['preprocessing']['baseline']['sample_count']=2
    config['preprocessing']['granular_baseline']['capture_on_start']=False
    config['continuous_tracking']['search_speed']=.001
    config['sandbox_scan']=dict(line_spacing=.01,boundary_margin=.0002)
    config['continuous_tracking']['workspace_calibration_file']=str(ROOT/'workspace/config/workspace_calibration.yaml')
    monkeypatch.setattr(runtime,'load_config',lambda _:deepcopy(config))
    monkeypatch.setattr(runtime,'OperatorKeyboard',Keyboard)
    record=record_for(config,p0=(.6007,.1007),size=.0014)
    path=tmp_path/'saved.yaml';save_record(path,record)
    plan=make_plan(config,record)
    devices=Devices(config,record['p0_tcp_pose'])
    settings=Namespace(config=ROOT/'config.yaml',calibration=path,execute=True,preview=False,
        duration=None,output=tmp_path/'logs',enable_reacquire=False)
    assert run_sandbox(settings,plan,controller_factory=devices.controller,reader_factory=Sensor)==0
    run,=(settings.output/'real/continuous').glob('run_*')
    result=json.loads((run/'sandbox_scan_result.json').read_text())
    termination=json.loads((run/'termination.json').read_text())
    assert result['completed'] and result['first_contact_pose'] is None
    assert result['reached_waypoints']==len(plan['coverage_xy'])
    assert termination['reason']=='SUCCESS'
    assert '区域扫描完成，未检测到目标' in capsys.readouterr().out
    with (run/'policy_waypoints.csv').open() as stream:
        assert any(e['event_type']=='COVERAGE_COMPLETE' for e in csv.DictReader(stream))
    assert (run/'sandbox_scan_actual.png').is_file()
    assert not devices.receive.connected and not devices.control.connected


def test_unconfirmed_candidate_resumes_same_coverage_waypoint(config):
    config['sandbox_scan']=dict(line_spacing=.007,boundary_margin=.002)
    plan=make_plan(config,record_for(config,size=.025))
    start=np.array(plan['runtime_start_pose'])
    search=CoverageSearch(plan,start,config['policy']['position_tolerance'])
    policy=ContinuousTrackingPolicy(config,search_strategy=search)
    policy.real_execution=True
    threshold=Wrench(1.1,0.,0.,0.,0.,0.)
    policy.update(0.,threshold,threshold,RobotState(0.,start.copy(),np.zeros(6)),execution_settled=True)
    assert policy.state==State.FIRST_CONTACT
    for i in range(1,15):
        policy.update(i*.01,ZERO,ZERO,RobotState(i*.01,start.copy(),np.zeros(6)),execution_settled=True)
    assert policy.state==State.TARGET_SEARCH and policy.initial_contact is None
    assert search.index==0 and not search.completed
    assert any(e.event_type=='FIRST_CONTACT_RETRY_SEARCH' for e in policy.events)


def test_default_missing_new_calibration_reuses_saved_successful_calibrations(config,tmp_path):
    from app.sandbox_scan import calibration_path,read_calibration
    config['sandbox_scan']=dict(calibration_file=str(tmp_path/'new_sandbox.yaml'))
    path=calibration_path(config,ROOT/'config.yaml')
    scan=(ROOT/'scan_calibration.yaml').read_bytes()
    workspace=(ROOT/'workspace/config/workspace_calibration.yaml').read_bytes()
    record=read_calibration(config,ROOT/'config.yaml',path)
    assert record['imported_sources']['scan']==str(ROOT/'scan_calibration.yaml')
    plan=make_plan(config,record)
    prepare_sandbox(config,ROOT/'config.yaml',path,expected_plan=plan)
    assert not path.exists()
    assert (ROOT/'scan_calibration.yaml').read_bytes()==scan
    assert (ROOT/'workspace/config/workspace_calibration.yaml').read_bytes()==workspace


@pytest.mark.parametrize('show', [False, True])
def test_preview_never_creates_a_gui_in_control_process(config,tmp_path,monkeypatch,show):
    import sys
    from pathlib import Path
    import matplotlib.pyplot as plt
    from visualization import sandbox_scan as preview
    config['sandbox_scan']=dict(line_spacing=.007,boundary_margin=.002)
    plan=make_plan(config,record_for(config,size=.025))
    viewers=[]
    def forbidden(*args,**kwargs):
        raise AssertionError('GUI event handling must stay out of the terminal/control process')
    monkeypatch.setattr(plt,'figure',forbidden)
    monkeypatch.setattr(plt,'show',forbidden)
    def launch(command,**options):
        assert command[0]==sys.executable
        assert options['cwd']==ROOT
        assert options['start_new_session']
        saved=json.loads(Path(command[-1]).read_text())
        assert saved['coverage_xy']==plan['coverage_xy']
        viewers.append(command)
    monkeypatch.setattr(preview.subprocess,'Popen',launch)
    path=preview.save_preview(plan,tmp_path,show=show)
    pixels=plt.imread(path)
    assert pixels.shape[:2]==(1500,2400)
    assert np.std(pixels[:,:,:3])>.05  # Complete plot, not an empty canvas.
    assert len(viewers)==int(show)
