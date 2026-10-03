from argparse import Namespace
import json
import numpy as np
import pytest
from app import discrete_runtime as runtime
from calibration.scan_calibration import load_scan_calibration
from experiment_logging.paths import PROJECT_ROOT, read_metadata
from doubles import Devices,Sensor,Keyboard


def test_fake_discrete_scan_shared_startup_and_normal_return(config,monkeypatch,tmp_path):
    target=load_scan_calibration(PROJECT_ROOT/'scan_calibration.yaml')['start_tcp_pose']
    initial=np.array(target);initial[0]+=.003
    d=Devices(config,initial);sensor=Sensor()
    original=runtime.load_config
    def load(path):
        c=original(path);c['preprocessing']['baseline']['sample_count']=2
        c['safe_return']['startup_bias_sample_count']=2
        c['policy']['max_runtime_sec']=.06
        return c
    monkeypatch.setattr(runtime,'load_config',load)
    monkeypatch.setattr(runtime,'OperatorKeyboard',Keyboard)
    args=Namespace(config=PROJECT_ROOT/'config.yaml',execute=True,sensor='real',output=tmp_path)
    assert runtime.run(args,controller_factory=d.controller,reader_factory=lambda *a:sensor)==0
    assert d.control_count==d.receive_count==1
    assert not d.receive.connected and not sensor.connected
    path,=(tmp_path/'real/discrete').glob('run_*')
    assert read_metadata(path)['strategy']=='discrete'
    data=json.loads((path/'summary.json').read_text())
    assert data['stop_observation']['standstill_confirmed']
    assert data['return_status']=='complete'


def test_discrete_escape_never_auto_returns(config,monkeypatch,tmp_path):
    target=load_scan_calibration(PROJECT_ROOT/'scan_calibration.yaml')['start_tcp_pose'];d=Devices(config,target)
    class StopKeyboard(Keyboard):
        calls=0
        def poll(self):
            type(self).calls+=1
            return 'ESC' if self.calls>2 else None
    original=runtime.load_config
    def load(path):
        c=original(path);c['preprocessing']['baseline']['sample_count']=2;return c
    monkeypatch.setattr(runtime,'load_config',load)
    monkeypatch.setattr(runtime,'OperatorKeyboard',StopKeyboard)
    args=Namespace(config=PROJECT_ROOT/'config.yaml',execute=True,sensor='real',output=tmp_path)
    assert runtime.run(args,controller_factory=d.controller,reader_factory=Sensor)==130
    assert not any(x[0]=='moveL' for x in d.control.calls)


def test_discrete_known_mount_uses_final_actual_pose(config,monkeypatch,tmp_path):
    from core.models import Wrench
    from sensor.force_preprocess import _tool_rotation
    target=load_scan_calibration(PROJECT_ROOT/'scan_calibration.yaml')['start_tcp_pose']
    d=Devices(config,target)
    original=runtime.load_config
    def load(path):
        c=original(path)
        c['preprocessing']['baseline']['capture_on_start']=False
        c['preprocessing']['coordinate_transform']['rotation_sensor_to_tool']=np.eye(3).tolist()
        c['policy']['max_runtime_sec']=.04
        return c
    class ForceSensor(Sensor):
        def read_wrench(self):
            return Wrench(.01, .02, .03, 0., 0., 0.)
    observed=[]
    update=runtime.RuleBasedPolicy.update
    def record(self,now,raw,processed,robot):
        observed.append((processed.force.copy(),robot.pose.copy()))
        return update(self,now,raw,processed,robot)
    monkeypatch.setattr(runtime,'load_config',load)
    monkeypatch.setattr(runtime,'OperatorKeyboard',Keyboard)
    monkeypatch.setattr(runtime.RuleBasedPolicy,'update',record)
    args=Namespace(config=PROJECT_ROOT/'config.yaml',execute=True,sensor='real',output=tmp_path)
    assert runtime.run(args,controller_factory=d.controller,reader_factory=ForceSensor)==0
    assert observed
    for force,pose in observed:
        np.testing.assert_allclose(force,_tool_rotation(pose[3:]) @ [.01,.02,.03])
