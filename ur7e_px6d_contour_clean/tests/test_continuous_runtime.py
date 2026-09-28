from argparse import Namespace
import json
import time
import numpy as np
import pytest
from app import continuous_runtime as runtime
from calibration.scan_calibration import load_scan_calibration
from experiment_logging.paths import PROJECT_ROOT, read_metadata
from doubles import Devices, Sensor, Keyboard


def args(tmp_path, execute=True):
    settings = Namespace(config=PROJECT_ROOT/'config.yaml', scene=PROJECT_ROOT/'simulation/scene_continuous.yaml',
        preview=False,execute=execute,enable_reacquire=False,duration=.08,output=tmp_path)
    Keyboard.test_args = settings
    return settings


def prepare(config, monkeypatch, *, watchdog=False):
    original = runtime.load_config
    def load(path):
        c=original(path)
        c['preprocessing']['baseline']['sample_count']=2
        c['safe_return']['startup_bias_sample_count']=2
        c['continuous_tracking']['continuous_require_watchdog']=watchdog
        return c
    monkeypatch.setattr(runtime,'load_config',load)
    monkeypatch.setattr(runtime,'OperatorKeyboard',Keyboard)
    monkeypatch.setattr(Keyboard,'answer','START')
    # Real SEARCH no longer ends on a software time budget. These fixtures use
    # an explicit operator Q after the requested test interval, after startup.
    contexts = 0
    def enter(self):
        nonlocal contexts
        contexts += 1
        self.main_loop = contexts > 1
        self.entered = time.monotonic()
        return self
    def poll(self):
        if self.key:
            return self.key
        if self.main_loop and time.monotonic()-self.entered >= Keyboard.test_args.duration:
            return 'Q'
        return None
    monkeypatch.setattr(Keyboard, '__enter__', enter)
    monkeypatch.setattr(Keyboard, 'poll', poll)
    target=load_scan_calibration(PROJECT_ROOT/'scan_calibration.yaml')['start_tcp_pose']
    return target


@pytest.mark.parametrize('away',[False,True])
def test_fake_real_continuous_owns_one_control_and_correct_logs(config,monkeypatch,tmp_path,away):
    target=prepare(config,monkeypatch)
    pose=np.array(target);pose[0]+=.003 if away else 0.
    d=Devices(config,pose);sensor=Sensor()
    assert runtime.run(args(tmp_path),controller_factory=d.controller,reader_factory=lambda *a:sensor)==0
    assert d.control_count==d.receive_count==1
    assert not d.receive.connected and not d.control.connected and not sensor.connected
    assert any(x[0]=='speedL' for x in d.control.calls)
    assert any(x[0]=='moveL' for x in d.control.calls)==away
    path,=(tmp_path/'real/continuous').glob('run_*')
    assert read_metadata(path)['mode']=='real'
    summary=json.loads((path/'summary.json').read_text())
    assert summary['stop_observation']['standstill_confirmed']
    assert summary['strategy']=='continuous'
    if away:
        status=json.loads((path/'return_status.json').read_text())
        assert status['force_monitor'] is True


def test_cancel_before_control_is_clean_exit(config,monkeypatch,tmp_path):
    target=prepare(config,monkeypatch);monkeypatch.setattr(Keyboard,'answer','cancel')
    d=Devices(config,target)
    assert runtime.run(args(tmp_path),controller_factory=d.controller,reader_factory=Sensor)==0
    assert d.control_count==0


def test_false_stop_api_can_still_confirm_physical_stop(config,monkeypatch,tmp_path):
    target=prepare(config,monkeypatch);d=Devices(config,target);d.control.stop_value=False
    assert runtime.run(args(tmp_path),controller_factory=d.controller,reader_factory=Sensor)==0
    path,=(tmp_path/'real/continuous').glob('run_*')
    summary=json.loads((path/'summary.json').read_text())
    reports=summary['stop_observation']['stop_requests']
    assert any(r['api_anomaly'] and r['physical_stop']=='confirmed' for r in reports)


def test_sensor_exception_stops_and_closes(config,monkeypatch,tmp_path):
    target=prepare(config,monkeypatch);d=Devices(config,target)
    class FailingSensor(Sensor):
        def read_wrench(self):
            if self.count >= 5: raise RuntimeError('injected PX6D failure')
            return super().read_wrench()
    sensor=FailingSensor()
    assert runtime.run(args(tmp_path),controller_factory=d.controller,reader_factory=lambda *a:sensor)==1
    assert d.control_count==1
    assert any(x[0]=='speedStop' for x in d.control.calls)
    assert not sensor.connected and not d.receive.connected


def test_simulation_continuous_uses_simulation_path(tmp_path):
    assert runtime.run(args(tmp_path,False))==0
    path,=(tmp_path/'simulation/continuous').glob('run_*')
    assert read_metadata(path)['strategy']=='continuous'
    assert not (tmp_path/'real').exists()


def test_fake_contact_brakes_confirms_and_enters_tracking(config,monkeypatch,tmp_path):
    import csv
    from core.models import Wrench
    target=prepare(config,monkeypatch);d=Devices(config,target)
    class ContactSensor(Sensor):
        def read_wrench(self):
            self.count+=1
            return Wrench(0. if self.count<10 else 1.5,0.,0.,0.,0.,0.)
    settings=args(tmp_path);settings.duration=.6
    assert runtime.run(settings,controller_factory=d.controller,reader_factory=ContactSensor)==0
    path,=(tmp_path/'real/continuous').glob('run_*')
    with (path/'samples.csv').open() as f:
        states={row['current_state'] for row in csv.DictReader(f)}
    assert {'TARGET_SEARCH','FIRST_CONTACT','CONTINUOUS_TRACKING'} <= states


def test_terminal_hold_keeps_healthy_watchdog_alive(config,monkeypatch,tmp_path):
    import time
    target=prepare(config,monkeypatch,watchdog=True);d=Devices(config,target)
    kicked=[]
    def kick(): kicked.append(time.monotonic());return True
    def finish():
        assert time.monotonic()-kicked[-1]<.05
        return True
    d.control.kickWatchdog=kick;d.control.stopScript=finish
    assert runtime.run(args(tmp_path),controller_factory=d.controller,reader_factory=Sensor)==0
    assert len(kicked)>10


def test_async_disk_failure_warns_without_ending_search(config,monkeypatch,tmp_path):
    target=prepare(config,monkeypatch);d=Devices(config,target);sensor=Sensor()
    def fail(*args,**kwargs): raise OSError('injected disk full')
    monkeypatch.setattr(runtime.ExperimentLogger,'log_sample',fail)
    assert runtime.run(args(tmp_path),controller_factory=d.controller,reader_factory=lambda *a:sensor)==0
    assert not d.receive.connected and not d.control.connected and not sensor.connected
    path,=(tmp_path/'real/continuous').glob('run_*')
    assert (path/'metadata.json').exists() and (path/'termination.json').exists()
    summary=json.loads((path/'summary.json').read_text())
    assert 'injected disk full' in summary['software_warnings']['logging']['write_error']
    assert json.loads((path/'termination.json').read_text())['reason']=='STOP_USER_REQUEST'
    assert any(x[0]=='speedStop' for x in d.control.calls)
