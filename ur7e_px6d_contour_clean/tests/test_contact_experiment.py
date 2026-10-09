"""Contact acquisition/replay tests with synthetic measurements and injected clocks."""
import csv
from collections import deque
from threading import Event
from types import SimpleNamespace

import matplotlib.pyplot as plt
from matplotlib.backend_bases import MouseEvent
import numpy as np
import pytest

from tools.visualize_contact_experiment import make_demo
from visualization.run_plots import load_trace, target_outline
from visualization.contact_report import collision_details, command_at, planar_speed
from visualization.run_animation import RunAnimation


@pytest.fixture
def demo(tmp_path):
    return make_demo(tmp_path)


def test_both_six_dimensional_velocities_and_true_sdk_send_times(demo):
    with (demo/'samples.csv').open() as f:
        rows = list(csv.DictReader(f))
    with (demo/'tcp_commands.csv').open() as f:
        commands = list(csv.DictReader(f))
    assert len(rows) == len(commands) == 601
    row = rows[120]
    actual_keys = ('tcp_vx','tcp_vy','tcp_vz','tcp_vrx','tcp_vry','tcp_vrz')
    sent_keys = tuple('commanded_tcp_'+k for k in ('vx','vy','vz','wx','wy','wz'))
    actual = np.array([float(row[k]) for k in actual_keys])
    sent = np.array([float(row[k]) for k in sent_keys])
    assert actual[0] == pytest.approx(.006)
    assert sent[0] == .008 and actual[0] != sent[0]
    np.testing.assert_allclose(actual[3:], [.001,-.002,.003])
    np.testing.assert_array_equal(sent[3:], np.zeros(3))
    assert row['velocity_frame'] == 'Base'
    assert float(row['commanded_tcp_timestamp']) > float(row['actual_tcp_timestamp'])
    trace=load_trace(demo)
    np.testing.assert_allclose(command_at(trace,1.5)[:2], [.008,0])
    np.testing.assert_allclose(command_at(trace,1.501)[:2], [0,0])
    assert len(trace.command_time) == 601


def test_sensor_raw_retained_and_all_base_wrench_components_rotated(demo):
    with (demo/'samples.csv').open() as f:
        row=list(csv.DictReader(f))[250]
    raw=np.array([float(row['raw_'+k]) for k in ('fx','fy','fz','tx','ty','tz')])
    base=np.array([float(row['raw_base_'+k]) for k in ('fx','fy','fz','tx','ty','tz')])
    r=np.array([[2**-.5,-2**-.5,0],[2**-.5,2**-.5,0],[0,0,1]])
    np.testing.assert_allclose(base, np.r_[r@raw[:3],r@raw[3:]])
    assert not np.allclose(raw[:2],base[:2])
    for k in ('fx','fy','fz','tx','ty','tz'):
        assert float(row['force_base_'+k]) == pytest.approx(float(row['filtered_force_base_'+k]))


def test_first_collision_distinct_from_tracking_and_excludes_collision_frame(demo):
    trace=load_trace(demo)
    first=trace.first_contact_index
    assert trace.time[first] == 1.5
    assert next(e['time'] for e in trace.events if e['kind']=='TRACKING_ENTERED') == 2.
    # Host RTDE read completes just before the policy decides contact: this same
    # collision row still must not be presented as the pre-collision observation.
    trace.actual_time[first] -= .0001
    detail=collision_details(trace)
    assert detail['precontact_one_second_available']
    assert detail['actual_before_time_sec'] == pytest.approx(1.49)
    assert detail['actual_before_xy_mm_s'] == pytest.approx([6.,0.,6.])
    assert detail['command_before_xy_mm_s'] == pytest.approx([8.,0.,8.])
    assert detail['after_contact'][0]['actual_xy_mm_s'] == pytest.approx([3.,0.,3.])


def test_missing_legacy_speed_or_event_is_not_reconstructed(demo):
    with (demo/'samples.csv').open() as f:
        rows=list(csv.DictReader(f)); fields=list(rows[0])
    for row in rows:
        row['commanded_speed_mps']='123'  # Unrelated policy telemetry is not a command trace.
        for k in fields:
            if k.startswith('commanded_tcp_') or k.startswith('tcp_v'):
                row[k]=''
    with (demo/'samples.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    (demo/'tcp_commands.csv').unlink()
    (demo/'policy_waypoints.csv').unlink()
    trace=load_trace(demo)
    assert trace.first_contact_index is None
    assert np.isnan(trace.actual_velocity).all()
    assert len(trace.command_time)==0
    assert np.isnan(command_at(trace,3.)).all()


def test_widgets_click_scrub_zoom_arrows_and_component_selection(demo):
    trace=load_trace(demo)
    view=RunAnimation(trace,animate=True)
    try:
        view.figure.canvas.draw()
        view.slider.set_val(3.125)
        i=trace.sample_index(3.125)
        assert not view.clock.playing and view.current_index==i
        assert not view.current_tcp.get_animated() and not view.force_arrow.get_animated()
        view.figure.canvas.draw()  # A paused full redraw must include current artists.
        assert view.cursor.get_xdata()[0] == view.velocity_cursor.get_xdata()[0] == trace.time[i]
        np.testing.assert_allclose(view.current_tcp.get_data(), trace.xy_mm[i].reshape(2,1))
        np.testing.assert_allclose([view.force_arrow.U[0],view.force_arrow.V[0]], trace.force[i,:2]*view.force_scale)
        np.testing.assert_allclose([view.velocity_arrow.U[0],view.velocity_arrow.V[0]],
                                   trace.actual_velocity[i,:2]*1000*view.velocity_scale)
        for axis in (view.force_axis,view.velocity_axis):
            x,y=axis.transData.transform((4.105,0.))
            event=MouseEvent('button_press_event',view.figure.canvas,x,y,button=1)
            view.figure.canvas.callbacks.process('button_press_event',event)
            assert view.current_index == trace.sample_index(event.xdata)
            assert view.cursor.get_xdata()[0] == view.velocity_cursor.get_xdata()[0]
        view.component_selector.set_active(3)
        assert [pair[0].get_visible() for pair in view.velocity_lines]==[False,False,True]
        view.zoom('contact_zoom')
        assert view.force_axis.get_xlim()==view.velocity_axis.get_xlim()
        assert view.current_index==150
        view.zoom('lost_zoom'); assert view.current_index==400
    finally:
        plt.close(view.figure)


def test_explicit_cube_geometry_rotates_without_scale_distortion():
    outline=target_outline(dict(center_base_mm=[640,360],side_mm=50,rotation_deg=37))
    np.testing.assert_allclose(np.linalg.norm(np.diff(outline,axis=0),axis=1),50.)
    np.testing.assert_allclose(np.mean(outline[:-1],axis=0),[640,360])
    assert target_outline(None) is None
    with pytest.raises(ValueError): target_outline(dict(side_mm=50))


def test_lossless_writer_keeps_full_raw_samples_during_disk_backlog(tmp_path,config):
    from experiment_logging.continuous_writer import ContinuousLogWriter
    from experiment_logging.data_logger import ExperimentLogger
    from core.models import RobotState,PolicyCommand,Wrench
    logger=ExperimentLogger(tmp_path,config,mode='simulation',strategy='continuous',workspace_logging=False)
    original=logger._samples.writerow
    entered,release=Event(),Event()
    def blocked(row):
        entered.set(); release.wait(2.); original(row)
    logger._samples.writerow=blocked
    writer=ContinuousLogWriter(logger,capacity=1,diagnostic_only=True,lossless=True,max_pending_sec=2.)
    command=PolicyCommand('TARGET_SEARCH',False,np.zeros(2),0.,np.zeros(6),False,False)
    try:
        writer.log_sample(1.,Wrench(1,2,3,4,5,6),Wrench(1,2,3,4,5,6),
            RobotState(1.,np.zeros(6),np.zeros(6)),command,None,None)
        assert entered.wait(1.)
        writer.log_sample(2.,Wrench(6,5,4,3,2,1),Wrench(6,5,4,3,2,1),
            RobotState(2.,np.zeros(6),np.zeros(6)),command,None,None)
        assert writer.diagnostics['dropped_records']==0 and len(writer._pending)==2
    finally:
        release.set(); writer.close()
    with (logger.run_dir/'samples.csv').open() as f: rows=list(csv.DictReader(f))
    assert [float(r['raw_fx']) for r in rows]==[1.,6.]


def test_preroll_records_full_second_without_commands_or_control_filter_mutation(config,monkeypatch):
    from experiment_logging import contact_capture
    from core.models import RobotState,Wrench
    from sensor.force_preprocess import WrenchPreprocessor
    now=[0.]
    monkeypatch.setattr(contact_capture,'time',SimpleNamespace(monotonic=lambda:now[0],sleep=lambda dt:now.__setitem__(0,now[0]+dt)))
    controller=SimpleNamespace(config={'continuous_require_watchdog':False},watchdog_active=False,
        observation_timing={},command_records=deque(), diagnostics={},
        read_diagnostic_state=lambda:RobotState(now[0],np.zeros(6),np.zeros(6)))
    rows=[]
    logger=SimpleNamespace(log_sample=lambda *a,**kw:rows.append((a,kw)),check_health=lambda:None)
    prep=WrenchPreprocessor.from_config(config['preprocessing'],tool_orientation=[0,0,0])
    contact_capture.stationary_preroll(config,controller,SimpleNamespace(read_wrench=lambda:Wrench(0,0,0,0,0,0)),
        prep,logger,lambda:None)
    assert rows[-1][0][0]-rows[0][0][0]>=1.
    assert all(row[0][4].state=='PRECONTACT' for row in rows)
    assert prep.force_base is None and prep.filtered_force_base is None
