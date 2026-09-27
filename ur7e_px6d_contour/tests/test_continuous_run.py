import argparse
import csv
import json
from pathlib import Path

import numpy as np
import pytest

import run_continuous_tracking as runner
from core.models import RobotState, Wrench
from config.loader import load_config, runtime_robot_config
from tools.visualize_continuous_run import frame_indices, make_figure, read_run, render

ROOT = Path(__file__).resolve().parents[1]


def args(tmp_path, execute=False):
    return argparse.Namespace(config=ROOT / "config.yaml", execute=execute,
                              duration=15, scene=ROOT / "simulation/scene_continuous.yaml", output=tmp_path)


@pytest.fixture
def offline_run(tmp_path, monkeypatch):
    # Historical 1 mm/s geometry/replay fixture; current 18 mm/s safety behavior
    # is independently tested in test_continuous_phase_speed.py.
    cfg = load_config(ROOT / 'config.yaml')
    cfg['continuous_tracking']['search_speed'] = .001
    monkeypatch.setattr(runner, 'load_config', lambda _: cfg)
    assert runner.run(args(tmp_path)) == 0
    return next(tmp_path.glob("run_*"))


def test_closed_loop_logs_and_offline_summary(offline_run):
    with (offline_run / "samples.csv").open() as handle:
        rows = list(csv.DictReader(handle))
    assert 1500 <= len(rows) <= 1502
    assert {r["current_state"] for r in rows} >= {"TARGET_SEARCH", "FIRST_CONTACT", "CONTINUOUS_TRACKING", "STOP"}
    assert all(float(r["command_speed"]) <= .03 for r in rows)
    assert any(abs(float(r["tangent_x"])) > .01 for r in rows if r["current_state"] == "CONTINUOUS_TRACKING")
    for key in ("raw_fx", "raw_tz", "dfx", "dtz", "tcp_vx", "filtered_fxy", "force_direction_x",
                "contact_direction_y", "force_error", "v_t", "v_n", "contact_lost_timer", "force_direction_sign"):
        assert all(r[key] != "" for r in rows)
    summary = json.loads((offline_run / "summary.json").read_text())
    assert summary["initial_contact"] is not None
    assert summary["termination_reason"] == "STOP_TIME_LIMIT"
    artifacts = render(offline_run, output_format="none")
    assert artifacts[0].stat().st_size > 1000
    data, config, events = read_run(offline_run)
    fig, update = make_figure(data, config, events)
    assert fig.axes[0].get_aspect() == 1
    update(len(data["time"])-1)
    assert fig.axes[1].lines[-1].get_xdata()[0] == data["time"][-1]
    import matplotlib.pyplot as plt
    plt.close(fig)


def test_replay_fallback_and_actual_gif(offline_run, monkeypatch):
    from matplotlib.animation import FFMpegWriter
    monkeypatch.setattr(FFMpegWriter, "isAvailable", classmethod(lambda cls: False))
    paths = render(offline_run, fps=.2, output_format="auto")
    assert paths[-1].suffix == ".gif" and paths[-1].stat().st_size > 1000
    # Explicit encoder failure must leave the summary available.
    monkeypatch.setattr(FFMpegWriter, "saving", lambda *a, **k: (_ for _ in ()).throw(RuntimeError("missing encoder")))
    assert len(render(offline_run, fps=.2, output_format="mp4")) == 1


def test_time_sampling_uses_timestamps():
    times = np.array([0, .04, .09, .11, .21, .23])
    assert frame_indices(times, 10).tolist() == [0, 2, 3, 5]
    with pytest.raises(ValueError):
        frame_indices(times, 0)


def test_control_does_not_call_plotting(tmp_path, monkeypatch):
    import matplotlib.pyplot as plt
    def forbidden(*a, **k):
        raise AssertionError("plotting in control")
    monkeypatch.setattr(plt, "subplots", forbidden)
    monkeypatch.setattr(plt, "plot", forbidden)
    monkeypatch.setattr(plt, "pause", forbidden)
    # At 18 mm/s the unchanged low-deceleration model stops on force safety.
    assert runner.run(args(tmp_path)) == 1
    summary = json.loads(next(tmp_path.glob('run_*/summary.json')).read_text())
    assert summary['termination_reason'] == 'STOP_FORCE_LIMIT'


@pytest.fixture
def fake_real(tmp_path, monkeypatch):
    config = load_config(ROOT / "config.yaml")
    config["preprocessing"]["baseline"]["capture_on_start"] = False
    config["continuous_tracking"]["max_runtime_sec"] = .04
    calls = []
    class Controller:
        observation_timing = {}
        speed_guard_diagnostics = {}
        motion_fault = ''
        def set_continuous_phase(self, phase, *, stop_confirmed=False):
            self.phase = phase
        def enable_watchdog(self, frequency):
            calls.append('watchdog_enable')
        def kick_watchdog(self):
            calls.append('watchdog_kick')
        def read_diagnostic_state(self):
            return self.read_state()
        def __init__(self, cfg):
            calls.append("controller_construct")
        def connect(self, allow_start_away_from_fixed_pose=False):
            assert allow_start_away_from_fixed_pose
            calls.append("connect")
        def safe_stop_motion(self, force=False):
            calls.append("stop")
        def read_state(self):
            return RobotState(runner.time.monotonic(), np.zeros(6), np.zeros(6))
        def command_planar_velocity(self, *args):
            calls.append("motion")
        def stop(self):
            calls.append("stop")
        def close(self):
            self.stop()
            calls.append("close")
    class Reader:
        def __init__(self, *a):
            pass
        def connect(self):
            pass
        def read_wrench(self):
            return Wrench(0, 0, 0, 0, 0, 0)
        def close(self):
            calls.append("sensor_close")
    monkeypatch.setattr(runner, "load_config", lambda p: config)
    monkeypatch.setattr(runner, "prepare_real", lambda *a: (runtime_robot_config(config), np.zeros(6)))
    monkeypatch.setattr(runner, "URRTDEController", Controller)
    monkeypatch.setattr(runner, "PX6DReader", Reader)
    monkeypatch.setattr("builtins.input", lambda prompt: "START")
    return args(tmp_path, True), config, calls, Controller, Reader


@pytest.mark.parametrize("failure", ["sensor", "logger", "controller", "force", "keyboard_interrupt"])
def test_runtime_failures_stop_before_cleanup(fake_real, monkeypatch, failure):
    arguments, config, calls, controller, reader = fake_real
    if failure == "sensor":
        def fail(self):
            calls.append("failure")
            raise RuntimeError("sensor disconnected")
        monkeypatch.setattr(reader, "read_wrench", fail)
    elif failure == "force":
        monkeypatch.setattr(reader, "read_wrench", lambda self: Wrench(60, 0, 0, 0, 0, 0))
    elif failure == "logger":
        def fail(*a, **k):
            calls.append("failure")
            raise OSError("disk full")
        monkeypatch.setattr(runner.ExperimentLogger, "log_sample", fail)
    elif failure == "keyboard_interrupt":
        def fail(self):
            calls.append("failure")
            raise KeyboardInterrupt
        monkeypatch.setattr(reader, "read_wrench", fail)
    else:
        def fail(*a):
            calls.append("failure")
            raise runner.RobotError("protective stop")
        monkeypatch.setattr(controller, "command_planar_velocity", fail)
    result = runner.run(arguments)
    assert result == (0 if failure == "keyboard_interrupt" else 1)
    assert "close" in calls and "sensor_close" in calls
    if "failure" in calls:
        i = calls.index("failure")
        assert calls[i+1] == "stop"
    assert calls.index("close") < calls.index("sensor_close")
    run_dir = next(arguments.output.glob("run_*"))
    with (run_dir / "policy_waypoints.csv").open() as handle:
        events = list(csv.DictReader(handle))
    expected = "USER_STOP" if failure == "keyboard_interrupt" else "SAFETY_STOP"
    assert events[-1]["event_type"] == expected


@pytest.mark.parametrize("key", ["Q", "ESC"])
def test_keyboard_stops_without_motion(fake_real, monkeypatch, key):
    arguments, _, calls, _, _ = fake_real
    monkeypatch.setattr(runner.OperatorKeyboard, "poll", lambda self: key)
    assert runner.run(arguments) == 0
    assert "motion" not in calls and "stop" in calls


def test_start_pose_mismatch_refuses_motion(fake_real, monkeypatch):
    arguments, _, calls, controller, _ = fake_real
    monkeypatch.setattr(controller, "read_state", lambda self: RobotState(0, np.ones(6), np.zeros(6)))
    assert runner.run(arguments) == 1
    assert "motion" not in calls


def test_bad_config_prevents_connection(fake_real):
    arguments, config, calls, _, _ = fake_real
    config["continuous_tracking"]["force_direction_sign"] = 0
    assert runner.run(arguments) == 1
    assert "controller_construct" not in calls


def test_start_pose_rechecked_after_confirmation(fake_real, monkeypatch):
    arguments, _, calls, controller, _ = fake_real
    def confirm(prompt):
        monkeypatch.setattr(controller, "read_state", lambda self: RobotState(0, np.ones(6), np.zeros(6)))
        return "START"
    monkeypatch.setattr("builtins.input", confirm)
    assert runner.run(arguments) == 1
    assert "motion" not in calls


def test_safety_cause_survives_cleanup_failure(fake_real, monkeypatch):
    arguments, _, _, controller, reader = fake_real
    monkeypatch.setattr(reader, "read_wrench", lambda self: Wrench(60, 0, 0, 0, 0, 0))
    def failed_close(self):
        raise OSError("disconnect failed")
    monkeypatch.setattr(controller, "close", failed_close)
    assert runner.run(arguments) == 1
    summary = json.loads(next(arguments.output.glob("run_*/summary.json")).read_text())
    assert summary["termination_reason"] == "STOP_FORCE_LIMIT"


def test_disk_write_does_not_block_healthy_control_cycles(fake_real, monkeypatch):
    from threading import Event
    arguments, _, calls, controller, _ = fake_real
    released = Event()
    written = []
    original=runner.ExperimentLogger.log_sample
    original_motion = controller.command_planar_velocity
    def motion(self, *args):
        original_motion(self, *args)
        if calls.count('motion') >= 3:
            released.set()
    def slow_log(*a,**kw):
        if not written:
            # Only continuing control can release this disk write. The timeout
            # keeps the regression finite even with the old synchronous logger.
            written.append(released.wait(.2))
        original(*a,**kw)
    monkeypatch.setattr(runner.ExperimentLogger,'log_sample',slow_log)
    monkeypatch.setattr(controller, 'command_planar_velocity', motion)
    try:
        assert runner.run(arguments) == 0
        assert written == [True]
    finally:
        released.set()
    run_dir = next(arguments.output.glob('run_*'))
    with (run_dir/'samples.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) >= 4 and rows[-1]['current_state'] == 'STOP'


def test_watchdog_call_timeout_blocks_following_motion(fake_real, monkeypatch):
    arguments, _, calls, controller, _ = fake_real
    def slow_kick(self):
        calls.append('watchdog_kick')
        if calls.count('watchdog_kick') == 2:
            runner.time.sleep(.045)
    monkeypatch.setattr(controller, 'kick_watchdog', slow_kick)
    assert runner.run(arguments) == 1
    assert calls.count('watchdog_kick') == 2
    assert 'motion' not in calls


def test_partial_event_enqueue_failure_preserves_each_event_once(fake_real, monkeypatch):
    from core.models import PolicyWaypoint
    arguments, _, _, _, _ = fake_real
    original_update = runner.ContinuousTrackingPolicy.update
    original_enqueue = runner.ContinuousLogWriter.log_waypoint
    enqueued = []
    def update(self, now, raw, processed, robot):
        command = original_update(self, now, raw, processed, robot)
        self.events.extend(PolicyWaypoint(now, self.state.value, label, robot.pose.copy())
                           for label in ('EXTRA_FIRST', 'EXTRA_SECOND'))
        return command
    def enqueue(self, event):
        if enqueued:
            raise OSError('test partial event queue failure')
        original_enqueue(self, event)
        enqueued.append(event.event_type)
    monkeypatch.setattr(runner.ContinuousTrackingPolicy, 'update', update)
    monkeypatch.setattr(runner.ContinuousLogWriter, 'log_waypoint', enqueue)
    assert runner.run(arguments) == 1
    run_dir = next(arguments.output.glob('run_*'))
    with (run_dir/'policy_waypoints.csv').open() as stream:
        events = [row['event_type'] for row in csv.DictReader(stream)]
    assert events.count(enqueued[0]) == 1
    assert events.count('EXTRA_FIRST') == events.count('EXTRA_SECOND') == 1
    assert events[-1] == 'SAFETY_STOP'
    assert (run_dir/'summary.json').exists()
    assert (run_dir/'scan_stop_snapshot.json').exists()


def test_slow_serial_read_never_sends_motion(fake_real,monkeypatch):
    arguments,_,calls,_,reader=fake_real
    def slow(self):
        runner.time.sleep(.04)
        return Wrench(0,0,0,0,0,0)
    monkeypatch.setattr(reader,'read_wrench',slow)
    assert runner.run(arguments)==1
    assert 'motion' not in calls and 'watchdog_kick' not in calls


def test_watchdog_setup_failure_prevents_first_motion(fake_real,monkeypatch):
    arguments,_,calls,controller,_=fake_real
    def failed(*a): raise runner.RobotError('setWatchdog rejected')
    monkeypatch.setattr(controller,'enable_watchdog',failed)
    assert runner.run(arguments)==1
    assert 'motion' not in calls


def test_final_snapshot_uses_fresh_post_stop_pose(fake_real,monkeypatch):
    arguments,_,calls,controller,_=fake_real
    reads = []
    def diagnostic(self):
        reads.append(1)
        return self.read_state() if len(reads) == 1 else RobotState(runner.time.monotonic(),np.full(6,.123),np.zeros(6))
    monkeypatch.setattr(controller,'read_diagnostic_state',diagnostic)
    assert runner.run(arguments)==0
    snapshot=json.loads(next(arguments.output.glob('run_*/scan_stop_snapshot.json')).read_text())
    assert snapshot['tcp_pose']==[.123]*6
    assert snapshot['stop_observation']['source']=='post_stop_host_observation'
    assert snapshot['stop_observation']['standstill_confirmed']


def test_final_snapshot_marks_unavailable_post_stop_state(fake_real,monkeypatch):
    arguments,_,_,controller,_=fake_real
    reads = []
    def unavailable(self):
        reads.append(1)
        if len(reads) == 1: return self.read_state()
        raise runner.RobotError('stale RTDE package')
    monkeypatch.setattr(controller,'read_diagnostic_state',unavailable)
    assert runner.run(arguments)==1
    snapshot=json.loads(next(arguments.output.glob('run_*/scan_stop_snapshot.json')).read_text())
    assert snapshot['stop_observation']['source']=='last_valid_sample'
    assert not snapshot['stop_observation']['standstill_confirmed']
    assert 'host_age_sec' in snapshot['stop_observation']


def test_failed_connection_keeps_snapshot_without_reading_closed_interface(fake_real, monkeypatch):
    arguments, _, _, controller, _ = fake_real
    def fail(self, **kwargs):
        self.connection_failed = True
        self.receive = None
        self.connection_diagnostics = dict(protective_stopped=True, freshness='not_verified',
                                          tcp_pose=[.6,.2,.1,0,0,0], tcp_speed=[0]*6)
        raise runner.RobotError('Failed to start control script, before timeout of 5 seconds')
    monkeypatch.setattr(controller, 'connect', fail)
    monkeypatch.setattr(controller, 'read_diagnostic_state', lambda self: pytest.fail('closed receive must not be read'))
    assert runner.run(arguments) == 1
    report = json.loads(next(arguments.output.glob('run_*/termination.json')).read_text())
    assert report['robot_connection']['protective_stopped'] is True
    assert report['tcp_pose'] == [.6,.2,.1,0,0,0]
    assert report['reason'] == 'STOP_MOTION_ERROR'
    assert not report['secondary_errors']


def test_terminal_hold_does_not_expire_owned_watchdog(fake_real, monkeypatch):
    arguments, _, calls, controller, _ = fake_real
    original_enable, original_kick = controller.enable_watchdog, controller.kick_watchdog
    def enable(self, hz):
        self.watched = True; self.last_kick = runner.time.monotonic()
        original_enable(self, hz)
    def kick(self):
        self.last_kick = runner.time.monotonic(); original_kick(self)
    def diagnostic(self):
        if getattr(self, 'watched', False) and runner.time.monotonic()-self.last_kick >= .05:
            raise runner.RobotError('simulated C207 watchdog expired during terminal hold')
        return self.read_state()
    def finish(self):
        if getattr(self, 'watched', False):
            diagnostic(self)
            calls.append('stopScript'); self.watched = False
            return True
        return False
    monkeypatch.setattr(controller, 'enable_watchdog', enable)
    monkeypatch.setattr(controller, 'kick_watchdog', kick)
    monkeypatch.setattr(controller, 'read_diagnostic_state', diagnostic)
    monkeypatch.setattr(controller, 'finish_control_script_if_stopped', finish, raising=False)
    original_log = runner.ExperimentLogger.log_sample
    def log(self, now, raw, processed, robot, command, *args, **kwargs):
        if command.state == 'STOP':
            assert 'stopScript' in calls  # end before terminal disk I/O
        return original_log(self, now, raw, processed, robot, command, *args, **kwargs)
    monkeypatch.setattr(runner.ExperimentLogger, 'log_sample', log)
    assert runner.run(arguments) == 0
    assert calls.count('stopScript') == 1
    assert calls.index('stopScript') < calls.index('close')
    assert 'watchdog_kick' not in calls[calls.index('stopScript')+1:]
    snapshot = json.loads(next(arguments.output.glob('run_*/scan_stop_snapshot.json')).read_text())
    assert snapshot['stop_observation']['standstill_confirmed']


def test_replay_shows_simulated_target_and_separate_motion_segments(offline_run):
    data,config,events=read_run(offline_run)
    fig,_=make_figure(data,config,events)
    labels=[line.get_label() for line in fig.axes[0].lines]
    assert any(p.get_label()=='simulation target truth' for p in fig.axes[0].patches)
    assert 'reliable contact TCP path' in labels
    assert 'search / lost / recovery TCP path' in labels
    assert any(line.get_marker()=='o' and line.get_markersize()==4 for line in fig.axes[0].lines)
    import matplotlib.pyplot as plt
    plt.close(fig)


def test_animation_frame_budget_applies_to_all_encoders():
    from tools.visualize_continuous_run import frame_indices
    indices=frame_indices(np.linspace(0,10000,100000),10,max_frames=240)
    assert len(indices)<=240
    assert indices[-1]==99999


def test_identical_geometric_observation_stream_across_three_entry_adapters(tmp_path,monkeypatch):
    """Replay one actual closed-loop stream through offline, preview and FAKE RTDE.

    This is adapter equivalence, not a hardware/physics authorization test.
    Every device class is replaced before invoking the execution adapter.
    """
    from copy import deepcopy
    import yaml
    import simulation.continuous_session as session_module
    import simulation.continuous_preview as preview_module
    from simulation.simulator import load_simulation_config
    from policy.continuous_tracking import State
    original=session_module.SimulationSession
    cfg=load_config(ROOT/'config.yaml');cfg['continuous_tracking']['max_runtime_sec']=12
    scene=load_simulation_config(ROOT/'simulation/scene_continuous.yaml')
    scene.update(target_shape='rectangle',target_width=.004,target_height=.004,target_center=[0.,0.],
                 start_point=[-.004,0.],calibration_point_0=[-.004,0.],calibration_point_1=[0.,0.])
    scene['preprocessing']['baseline']={'capture_on_start':False}
    cfg['preprocessing']=deepcopy(scene['preprocessing']);cfg['preprocessing']['filter_alpha']=.8
    cfg['policy']['search_direction_xy']=[1.,0.]
    master=original(cfg,scene);master.capture_bias();stream=[]
    for _ in range(1500):
        sample=master.step();stream.append(sample);master.policy.events.clear()
        if master.policy.state==State.STOP:break
    assert any(s.command.state=='DIRECTION_RECONFIRM' for s in stream)
    scene_path=tmp_path/'stream_scene.yaml';scene_path.write_text(yaml.safe_dump(scene))
    def at(t):return stream[min(round(t/master.dt),len(stream)-1)]
    def session_factory(config,scene):
        session=original(config,scene)
        session.robot.read_state=lambda:at(session.robot.time).robot
        session.sensor.read_wrench=lambda xy,v:(at(session.robot.time).raw,at(session.robot.time).diagnostics)
        return session
    monkeypatch.setattr(session_module,'SimulationSession',session_factory)
    monkeypatch.setattr(preview_module,'SimulationSession',session_factory)
    monkeypatch.setattr(runner,'load_config',lambda path:deepcopy(cfg))
    arguments=args(tmp_path/'headless');arguments.scene=scene_path;arguments.duration=12
    runner.run(arguments)
    model=preview_module.PreviewRun(cfg,scene,tmp_path/'preview');model.start(0);wall=0
    while model.status=='RUNNING':wall+=.1;model.tick(wall,work_budget_sec=10)
    class Clock:
        t=0.
        def monotonic(self):return self.t
        def sleep(self,dt):self.t+=dt
    clock=Clock();device_calls=[]
    class FakeRobot:
        observation_timing={};motion_fault=''
        speed_guard_diagnostics={}
        def set_continuous_phase(self, phase, *, stop_confirmed=False):pass
        def __init__(self,cfg):device_calls.append('fake_robot')
        def connect(self, allow_start_away_from_fixed_pose=False):
            assert allow_start_away_from_fixed_pose
        def safe_stop_motion(self,**kwargs):pass
        def read_state(self):
            s=at(clock.t).robot
            return RobotState(clock.t,s.pose.copy(),s.tcp_speed.copy())
        def read_diagnostic_state(self):return RobotState(clock.t,at(clock.t).robot.pose.copy(),np.zeros(6))
        def command_planar_velocity(self,*args):device_calls.append('move')
        def stop(self):device_calls.append('stop')
        def enable_watchdog(self,*args):pass
        def kick_watchdog(self):pass
        def close(self):pass
    class FakeSensor:
        def __init__(self,*args):device_calls.append('fake_sensor')
        def connect(self):pass
        def read_wrench(self):return at(clock.t).raw
        def close(self):pass
    # Use a module-local clock object; do not alter time used by logger/GUI.
    monkeypatch.setattr(runner,'time',clock)
    monkeypatch.setattr(runner,'URRTDEController',FakeRobot)
    monkeypatch.setattr(runner,'PX6DReader',FakeSensor)
    monkeypatch.setattr(runner,'prepare_real',lambda *a:(cfg['robot'],stream[0].robot.pose.copy()))
    monkeypatch.setattr('builtins.input',lambda *a:'START')
    arguments=args(tmp_path/'fake_execution',True);arguments.duration=12
    runner.run(arguments)
    def read(path):
        with (path/'samples.csv').open() as f:return list(csv.DictReader(f))
    offline=read(next((tmp_path/'headless').glob('run_*')))
    preview=read(model.run_dir)
    executed=read(next((tmp_path/'fake_execution').glob('run_*')))
    assert len(offline)==len(executed)==len(stream)
    keys=['tcp_x','tcp_y','tcp_vx','tcp_vy','dfx','dfy','command_speed','command_vx','command_vy',
          'measurement_jump_deg','estimate_residual_deg','direction_speed_scale']
    for index,(a,b,c) in enumerate(zip(offline,preview,executed)):
        assert a['current_state']==b['current_state']==c['current_state'],index
        np.testing.assert_allclose([float(a[k]) for k in keys],[float(b[k]) for k in keys],atol=1e-10)
        np.testing.assert_allclose([float(a[k]) for k in keys],[float(c[k]) for k in keys],atol=1e-10)
        assert c['sim_components_available']=='0' and c['sim_object_fx']==''
        if a['current_state']=='DIRECTION_RECONFIRM':assert float(c['command_speed'])==0
    assert 'move' in device_calls and 'stop' in device_calls


def test_replay_unknown_hardware_components_and_target_are_not_invented(offline_run):
    from copy import deepcopy
    data,config,events=read_run(offline_run)
    # Simulate the information contract of a real log: no object geometry or
    # independently measurable component forces. Raw/processed totals remain.
    config=deepcopy(config);config.pop('continuous_simulation')
    data['sim_components_available'][:]=0
    for key in list(data):
        if key.startswith(('sim_object_','sim_friction_','sim_background_','sim_noise_')):data[key][:]=np.nan
    fig,update=make_figure(data,config,events,view='probe',components=True)
    update(len(data['time'])-1)
    assert fig.axes[0].get_aspect()==1
    assert fig.axes[2].get_ylabel()=='Velocity [mm/s]'
    assert 'simulation target truth' not in [line.get_label() for line in fig.axes[0].lines]
    assert not fig.continuous_vectors.arrows['object'].get_visible()
    assert 'unavailable' in fig.continuous_vectors.boundary_text.get_text()
    assert 'Measured environment resultant' in fig.continuous_vectors.boundary_text.get_text()
    assert not fig.continuous_vectors.arrows['robot_estimate'].get_visible()
    assert fig.axes[1].lines[-1].get_xdata()[0]==fig.axes[2].lines[-1].get_xdata()[0]
    import matplotlib.pyplot as plt
    plt.close(fig)
