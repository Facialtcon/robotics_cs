"""Reuse project calibration without hardware or fabricated attestations."""
from pathlib import Path
import numpy as np
import pytest
import yaml
import run_continuous_tracking as runner
from config.loader import load_config
from calibration.scan_calibration import load_scan_calibration
from workspace.workspace_transform import load_calibration

ROOT=Path(__file__).resolve().parents[1]

@pytest.fixture
def saved(tmp_path,monkeypatch):
    cfg=load_config(ROOT/'config.yaml')
    scan=tmp_path/'scan.yaml';scan.write_bytes((ROOT/'scan_calibration.yaml').read_bytes())
    sandbox=tmp_path/'corners.yaml';sandbox.write_bytes((ROOT/'workspace/config/workspace_calibration.yaml').read_bytes())
    cfg['calibration']['file']='scan.yaml'
    cfg['continuous_tracking']['workspace_calibration_file']='corners.yaml'
    path=tmp_path/'config.yaml';path.write_text(yaml.safe_dump(cfg))
    monkeypatch.setattr(runner.URRTDEController,'verified_watchdog_contract',staticmethod(lambda:dict(offline_double=True)))
    monkeypatch.setattr(runner.URRTDEController,'connect',lambda *a,**k:pytest.fail('hardware forbidden'))
    monkeypatch.setattr(runner.PX6DReader,'connect',lambda *a,**k:pytest.fail('hardware forbidden'))
    return cfg,path,scan,sandbox


def test_reuses_saved_start_direction_and_corners_without_site_record(saved):
    cfg,path,scan,sandbox=saved
    old_scan,old_sandbox=scan.read_bytes(),sandbox.read_bytes()
    robot,start=runner.prepare_real(cfg,path)
    expected=load_scan_calibration(scan,require_tcp_offset=True)
    corners=load_calibration(sandbox)
    np.testing.assert_array_equal(start,expected['start_tcp_pose'])
    np.testing.assert_array_equal(cfg['policy']['search_direction_xy'],expected['scan_direction_xy'])
    assert robot['fixed_z']==expected['fixed_z']!=corners['average_z']
    assert robot['fixed_orientation']==expected['fixed_orientation']
    assert cfg['continuous_tracking']['site_verification'] is None
    assert cfg['continuous_workspace_calibration']==corners
    polygon=[[corners['raw_points'][f'P{i}'][axis] for axis in ('x','y')] for i in range(4)]
    assert robot['continuous_xy_polygon']==polygon
    assert cfg['continuous_calibration_sources']['workspace']==str(sandbox.resolve())
    assert robot['continuous_require_watchdog']
    assert scan.read_bytes()==old_scan and sandbox.read_bytes()==old_sandbox
    policy=runner.ContinuousTrackingPolicy(cfg)
    direction=np.array(expected['scan_direction_xy'])
    np.testing.assert_array_equal(policy.search_direction,direction/np.linalg.norm(direction))


@pytest.mark.parametrize('bad',['missing_workspace','bad_workspace','tcp','robot_ip','outside_start','site_record','recovery'])
def test_bad_saved_data_rejected_before_connect(saved,bad):
    cfg,path,scan,sandbox=saved
    if bad=='missing_workspace':cfg['continuous_tracking']['workspace_calibration_file']='not_saved.yaml'
    elif bad=='bad_workspace':
        data=yaml.safe_load(sandbox.read_text());data['rectified_points']['R0'][0]+=.01
        sandbox.write_text(yaml.safe_dump(data))
    elif bad=='tcp':cfg['tcp']['offset'][0]+=.01
    elif bad=='robot_ip':cfg['robot']['robot_ip']='192.168.1.99'
    elif bad=='outside_start':cfg['continuous_tracking']['real_test_xy_limits']=dict(x_min=0,x_max=.1,y_min=0,y_max=.1)
    elif bad=='site_record':cfg['continuous_tracking']['site_verification']={}
    elif bad=='recovery':cfg['continuous_tracking']['reacquire_enabled']=True
    with pytest.raises((ValueError,runner.RobotError,FileNotFoundError)):
        runner.prepare_real(cfg,path)


def test_digest_includes_saved_workspace(saved):
    cfg,path,scan,sandbox=saved
    digest=runner.site_configuration_digest(cfg,path)
    sandbox.write_text(sandbox.read_text()+'\n# new calibration file contents\n')
    assert runner.site_configuration_digest(cfg,path)!=digest


def test_read_only_cli_never_constructs_devices(saved,monkeypatch,capsys):
    cfg,path,scan,sandbox=saved
    monkeypatch.setattr(runner.URRTDEController,'__init__',lambda *a,**k:pytest.fail('robot construction forbidden'))
    monkeypatch.setattr(runner.PX6DReader,'__init__',lambda *a,**k:pytest.fail('sensor construction forbidden'))
    monkeypatch.setattr('sys.argv',['run_continuous_tracking.py','--config',str(path),'--check-calibration'])
    assert runner.main()==0
    output=capsys.readouterr().out
    assert str(scan) in output and str(sandbox) in output
    assert '0.6211867726148192' in output and 'no device connection' in output


def test_rotated_raw_corners_not_only_axis_aligned_box():
    from robot.rtde_controller import check_continuous_xy
    cfg=dict(continuous_xy_limits=dict(x_min=-1,x_max=1,y_min=-1,y_max=1),
             continuous_xy_polygon=[[0,1],[1,0],[0,-1],[-1,0]],continuous_boundary_margin=.01)
    check_continuous_xy(cfg,[0,0])
    with pytest.raises(runner.RobotError,match='boundary'):check_continuous_xy(cfg,[.8,.8])
    with pytest.raises(runner.RobotError,match='boundary'):check_continuous_xy(cfg,[.995,0])
    cfg['continuous_xy_polygon'].reverse();check_continuous_xy(cfg,[0,0])
