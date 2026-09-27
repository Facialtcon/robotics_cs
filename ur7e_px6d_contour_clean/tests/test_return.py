import json
import numpy as np
import pytest
from app.return_runtime import run, load_return_target
from config.loader import runtime_robot_config
from experiment_logging.paths import PROJECT_ROOT, read_metadata
from safety.safe_return import SafeReturnExecutor, PX6DForceMonitor, return_trajectory
from sensor.force_preprocess import WrenchPreprocessor
from doubles import Devices, Sensor
from core.models import Wrench


def test_manual_return_uses_no_sensor_and_one_control(config, tmp_path):
    target = load_return_target(PROJECT_ROOT/'config.yaml', config)['pose']
    initial = np.array(target); initial[0] += .01
    d = Devices(config, initial)
    def confirm(prompt):
        assert d.control_count == 0
        return 'RETURN'
    assert run(controller_factory=d.controller, confirm=confirm, data_root=tmp_path) == 0
    assert d.control_count == 1
    moves = [x[1] for x in d.control.calls if x[0]=='moveL']
    assert len(moves) == 3
    assert moves[0][2] > initial[2]
    assert moves[1][2] == moves[0][2] and moves[1][:2] == target[:2]
    assert moves[2] == target
    path, = (tmp_path/'real/return').glob('run_*')
    assert read_metadata(path)['strategy'] == 'return'
    summary = json.loads((path/'summary.json').read_text())
    assert summary['status'] == 'complete'
    assert all(s['physical_stop']=='confirmed' for s in summary['stop_requests'])
    assert not (path/'policy_waypoints.csv').exists()
    import csv
    with (path/'samples.csv').open() as handle:
        row = next(csv.DictReader(handle))
    assert all(f'tcp_v{axis}' in row for axis in ('x', 'y', 'z', 'rx', 'ry', 'rz'))


def test_cancel_never_constructs_control(config,tmp_path):
    target = load_return_target(PROJECT_ROOT/'config.yaml', config)['pose']
    d=Devices(config,target)
    assert run(controller_factory=d.controller,confirm=lambda _: 'no',data_root=tmp_path)==0
    assert d.control_count==0 and not d.control.calls


def test_motion_during_confirmation_rejects_before_control(config,tmp_path):
    target=load_return_target(PROJECT_ROOT/'config.yaml',config)['pose'];d=Devices(config,target)
    def confirm(_): d.receive.pose[0]+=.01;return 'RETURN'
    assert run(controller_factory=d.controller,confirm=confirm,data_root=tmp_path)==1
    assert d.control_count==0


def test_return_shared_force_monitor_aborts_before_first_move(config):
    pose=np.array(config['dry_run']['start_pose']);d=Devices(config,pose)
    c=d.controller(runtime_robot_config(config));c.connect();c.activate_control(confirmed=True)
    sensor=Sensor();sensor.read_wrench=lambda: Wrench(100,0,0,0,0,0)
    monitor=PX6DForceMonitor(config,sensor,WrenchPreprocessor.from_config(config['preprocessing']))
    target=pose.copy();target[0]+=.01
    result=SafeReturnExecutor(config,target,c,force_monitor=monitor).execute()
    assert result.status=='aborted'
    assert not any(x[0]=='moveL' for x in d.control.calls)
    assert c.standstill_confirmed
    c.close()


def test_return_validates_entire_path_before_first_move(config):
    pose=np.array(config['dry_run']['start_pose']);config['workspace']['enabled']=True
    config['workspace']['limits']=dict(x_min=0,x_max=1,y_min=-1,y_max=1,z_min=0,z_max=pose[2]+.01)
    with pytest.raises(Exception,match='outside workspace'):
        return_trajectory(config,pose,pose)
