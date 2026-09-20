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
def offline_run(tmp_path):
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
    paths = render(offline_run, fps=.2)
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
    assert runner.run(args(tmp_path)) == 0


@pytest.fixture
def fake_real(tmp_path, monkeypatch):
    config = load_config(ROOT / "config.yaml")
    config["preprocessing"]["baseline"]["capture_on_start"] = False
    config["continuous_tracking"]["max_runtime_sec"] = .04
    calls = []
    class Controller:
        observation_timing = {}
        motion_fault = ''
        def enable_watchdog(self, frequency):
            calls.append('watchdog_enable')
        def kick_watchdog(self):
            calls.append('watchdog_kick')
        def read_diagnostic_state(self):
            return self.read_state()
        def __init__(self, cfg):
            calls.append("controller_construct")
        def connect(self):
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


def test_logging_stall_stops_and_never_kicks_again(fake_real, monkeypatch):
    arguments, _, calls, _, _ = fake_real
    original=runner.ExperimentLogger.log_sample
    def slow_log(*a,**kw):
        original(*a,**kw)
        calls.append('log_block')
        runner.time.sleep(.04)
    monkeypatch.setattr(runner.ExperimentLogger,'log_sample',slow_log)
    assert runner.run(arguments)==1
    tail=calls[calls.index('log_block')+1:]
    assert tail[0]=='stop'
    assert 'motion' not in tail and 'watchdog_kick' not in tail


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
    monkeypatch.setattr(controller,'read_diagnostic_state',lambda self:RobotState(runner.time.monotonic(),np.full(6,.123),np.zeros(6)))
    assert runner.run(arguments)==0
    snapshot=json.loads(next(arguments.output.glob('run_*/scan_stop_snapshot.json')).read_text())
    assert snapshot['tcp_pose']==[.123]*6
    assert snapshot['stop_observation']['source']=='post_stop_host_observation'
    assert snapshot['stop_observation']['standstill_confirmed']


def test_final_snapshot_marks_unavailable_post_stop_state(fake_real,monkeypatch):
    arguments,_,_,controller,_=fake_real
    def unavailable(self): raise runner.RobotError('stale RTDE package')
    monkeypatch.setattr(controller,'read_diagnostic_state',unavailable)
    assert runner.run(arguments)==1
    snapshot=json.loads(next(arguments.output.glob('run_*/scan_stop_snapshot.json')).read_text())
    assert snapshot['stop_observation']['source']=='last_valid_sample'
    assert not snapshot['stop_observation']['standstill_confirmed']
    assert 'host_age_sec' in snapshot['stop_observation']


def test_replay_shows_simulated_target_and_separate_motion_segments(offline_run):
    data,config,events=read_run(offline_run)
    fig,_=make_figure(data,config,events)
    labels=[line.get_label() for line in fig.axes[0].lines]
    assert 'simulation target truth' in labels
    assert 'reliable contact TCP path' in labels
    assert 'search / lost / recovery TCP path' in labels
    assert any('enlarged' in line.get_label() for line in fig.axes[0].lines)
    import matplotlib.pyplot as plt
    plt.close(fig)


def test_animation_frame_budget_applies_to_all_encoders():
    from tools.visualize_continuous_run import frame_indices
    indices=frame_indices(np.linspace(0,10000,100000),10,max_frames=240)
    assert len(indices)<=240
    assert indices[-1]==99999
