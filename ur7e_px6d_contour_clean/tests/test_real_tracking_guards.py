"""Real execution regressions, using only injected RTDE/PX6D and a virtual clock."""
import json
import csv
import time
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest

from app import continuous_runtime as runtime, scan_startup as startup
from app.configuration import prepare_real
from core.models import RobotState, Wrench
import doubles
from doubles import Devices, Sensor, Keyboard
from experiment_logging.paths import PROJECT_ROOT
from experiment_logging.data_logger import ExperimentLogger
from policy.continuous_tracking import ContinuousTrackingPolicy, State
from robot import rtde_controller as rtde
from safety import safe_return as returns
from sensor.force_preprocess import WrenchPreprocessor
from test_continuous_runtime import args, prepare


@pytest.fixture
def clock(monkeypatch):
    class Clock:
        now = 1000.
        def monotonic(self): return self.now
        def sleep(self, duration):
            self.now += max(0., duration)
            time.sleep(0)  # Let the production asynchronous logger drain.
    clock = Clock()
    for module in (runtime, startup, rtde, returns, doubles):
        monkeypatch.setattr(module, 'time', clock)
    return clock


@pytest.fixture
def real(config):
    robot_config, start = prepare_real(config, PROJECT_ROOT/'config.yaml')
    devices = Devices(config, start)
    owner = devices.controller(robot_config)
    owner.connect()
    yield config, np.asarray(start), devices, owner
    devices.receive.speed[:] = 0
    owner.close()


def sample(policy, now, force=1.5, *, pose=None, speed=0., execution_settled=None):
    pose = np.zeros(6) if pose is None else pose
    wrench = Wrench(force, 0, 0, 0, 0, 0)
    return policy.update(now, wrench, wrench, RobotState(now, pose, np.array([speed, 0, 0, 0, 0, 0])),
                         execution_settled=execution_settled)


def tracking(config):
    policy = ContinuousTrackingPolicy(config)
    for i in range(101): sample(policy, i*.01)
    assert policy.state == State.CONTINUOUS_TRACKING
    return policy


def lose(policy):
    for i in range(101, 119):
        command = sample(policy, i*.01, 0.)
        assert policy.state != State.STOP
    assert policy.state == State.LOCAL_REACQUIRE
    return command


def test_first_contact_loss_restarts_search_then_confirms_next_contact(real):
    config, start, *_ = real
    policy = ContinuousTrackingPolicy(config)
    assert sample(policy, 0., 0., pose=start).state == 'TARGET_SEARCH'
    assert sample(policy, .01, .7, pose=start).move
    assert sample(policy, .02, 1.2, pose=start, speed=.018).state == 'FIRST_CONTACT'
    # A force dip during braking never authorizes a search command.
    for i in range(3, 10):
        command = sample(policy, i*.01, .7, pose=start, speed=.003, execution_settled=False)
        assert command.state == 'FIRST_CONTACT' and not command.move
    # The policy's low-speed hold is insufficient until the execution owner agrees.
    for i in range(10, 25):
        command = sample(policy, i*.01, .7, pose=start, execution_settled=False)
        assert command.state == 'FIRST_CONTACT' and not command.move
    command = sample(policy, .25, .7, pose=start, execution_settled=True)
    assert command.state == 'TARGET_SEARCH' and not command.move
    assert policy.initial_contact is None and policy.stop_reason is None
    assert not policy._confirmation_vectors and policy._confirm_started is None
    assert policy._started == 0.
    np.testing.assert_array_equal(policy._start_pose, start)
    # Retry retains the original direction, 18 mm/s target and acceleration ramp.
    for i in range(26, 220):
        command = sample(policy, i*.01, .7, pose=start)
        assert command.state == 'TARGET_SEARCH' and command.move
        np.testing.assert_allclose(command.direction_xy, config['policy']['search_direction_xy'])
    assert command.speed == pytest.approx(.018)
    assert sample(policy, 2.20, 1.2, pose=start, speed=.018).state == 'FIRST_CONTACT'
    for i in range(221, 240):
        command = sample(policy, i*.01, 1.2, pose=start, execution_settled=True)
    assert command.state == 'CONTINUOUS_TRACKING' and command.move
    assert [e.event_type for e in policy.events].count('FIRST_CONTACT_RETRY_SEARCH') == 1


def test_unconfirmed_first_contact_retries_after_existing_timeout(real):
    config, start, *_ = real
    policy = ContinuousTrackingPolicy(config)
    sample(policy, 0., 1.2, pose=start)
    for i in range(1, 110):
        # Contact magnitude stays high, but alternating direction cannot initialize a normal.
        command = sample(policy, i*.01, 1.2*(-1)**i, pose=start, execution_settled=True)
        assert not command.move
        if command.state == 'TARGET_SEARCH': break
    assert command.state == 'TARGET_SEARCH' and policy.initial_contact is None
    assert i*.01 >= config['continuous_tracking']['confirmation_timeout_sec']
    # High force immediately starts another stationary confirmation, never pushes forward.
    command = sample(policy, (i+1)*.01, 1.2, pose=start, execution_settled=True)
    assert command.state == 'FIRST_CONTACT' and not command.move


def test_first_contact_retry_keeps_original_sandbox_search_budget(real):
    config, start, *_ = real
    policy = ContinuousTrackingPolicy(config)
    sample(policy, 0., 0., pose=start)
    near_edge = start.copy()
    geometry = config['continuous_search_geometry']
    near_edge[:2] += policy.search_direction*(geometry['usable_distance_m']-.0001)
    sample(policy, .01, 1.2, pose=near_edge)
    for i in range(2, 11):
        command = sample(policy, i*.01, .7, pose=near_edge, execution_settled=True)
        if command.state == 'TARGET_SEARCH': break
    assert command.state == 'TARGET_SEARCH' and not command.move
    command = sample(policy, (i+1)*.01, .7, pose=near_edge, execution_settled=True)
    assert command.state == 'STOP' and policy.stop_reason.value == 'STOP_SEARCH_LIMIT'


def test_first_contact_raw_hard_limit_precedes_retry(real):
    config, start, *_ = real
    policy = ContinuousTrackingPolicy(config)
    sample(policy, 0., 1.2, pose=start)
    for i in range(1, 9):
        sample(policy, i*.01, 1.2, pose=start, execution_settled=False)
    raw, processed = Wrench(65, 0, 0, 0, 0, 0), Wrench(.7, 0, 0, 0, 0, 0)
    command = policy.update(.09, raw, processed, RobotState(.09, start, np.zeros(6)), execution_settled=True)
    assert command.state == 'STOP' and not command.move
    assert policy.stop_reason.value == 'STOP_FORCE_LIMIT'
    assert not any(e.event_type == 'FIRST_CONTACT_RETRY_SEARCH' for e in policy.events)


@pytest.mark.parametrize('linear,angular,accepted', [
    (.0003, .009, True), (.001, .01, True), (.0011, 0., False), (0., .0101, False),
])
def test_startup_checks_linear_and_angular_speeds_separately(real, clock, linear, angular, accepted):
    config, start, devices, owner = real
    devices.receive.pose[0] += .0005
    devices.receive.speed[:] = [linear, 0., 0., angular, 0., 0.]
    if accepted:
        runtime.check_start(owner, start, config)
    else:
        with pytest.raises(rtde.RobotError, match='moved'):
            runtime.check_start(owner, start, config)


def test_noisy_startup_and_half_mm_p0_offset_succeed_without_return(real, clock):
    config, start, devices, owner = real
    devices.receive.speed[0] = .0003
    devices.receive.speed[3] = .007
    devices.receive.pose[0] += .0005
    sensor = Sensor()
    assert startup.startup_scan(config, start, owner, sensor, None, lambda: None,
                                confirm=lambda _: '') is False
    assert not any(call[0] == 'moveL' for call in devices.control.calls)
    runtime.check_start(owner, start, config)
    processor = WrenchPreprocessor.from_config(config['preprocessing'])
    def noisy_read():
        devices.receive.pose[0] += .0001
        return Wrench(15, 0, 0, 0, 0, 0)
    sensor.read_wrench = noisy_read
    startup.capture_stationary_bias(config, sensor, processor, owner, 5)
    assert 'startup_stationary' in owner.diagnostics
    np.testing.assert_allclose(processor.process(Wrench(15, 0, 0, 0, 0, 0)).array(), 0., atol=1e-12)
    devices.receive.speed[:] = 0
    owner.close()


@pytest.mark.parametrize('kind', ['translation', 'rotation', 'drift'])
def test_obvious_motion_rejects_bias(real, clock, kind):
    config, start, devices, owner = real
    sensor = Sensor()
    def read():
        if kind == 'translation': devices.receive.speed[0] = .002
        if kind == 'rotation': devices.receive.speed[3] = .02
        if kind == 'drift': devices.receive.pose[0] += .002
        return Wrench(15, 0, 0, 0, 0, 0)
    sensor.read_wrench = read
    with pytest.raises(rtde.RobotError, match='moved'):
        startup.capture_stationary_bias(config, sensor,
            WrenchPreprocessor.from_config(config['preprocessing']), owner, 1)
    devices.receive.speed[:] = 0
    owner.close()


def test_obvious_motion_rejects_start_before_control(real, clock):
    config, start, devices, owner = real
    devices.receive.speed[0] = .002
    with pytest.raises(rtde.RobotError, match='did not settle'):
        startup.startup_scan(config, start, owner, Sensor(), None, lambda: None, confirm=lambda _: '')
    assert devices.control_count == 0
    devices.receive.speed[:] = 0
    owner.close()


def test_startup_tolerance_does_not_relax_first_contact_braking(real, clock):
    config, start, devices, owner = real
    devices.receive.speed[0] = .0003
    owner.wait_for_standstill(startup=True)
    assert owner.standstill_confirmed
    owner.activate_control(confirmed=True)
    speed_l = devices.control.speedL
    def noisy_brake(*a):
        result = speed_l(*a)
        devices.receive.speed[0] = .0003
        return result
    devices.control.speedL = noisy_brake
    owner.continuous_phase = 'FIRST_CONTACT'
    owner.request_stop(nonblocking=True)
    assert owner._stop_pending
    for _ in range(15):
        clock.sleep(.01)
        owner.read_state()
        assert not owner.poll_stop()
    policy = ContinuousTrackingPolicy(config)
    for i in range(31):
        command = sample(policy, i*.01, speed=.0003)
        assert command.state == 'FIRST_CONTACT' and not command.move
    devices.receive.speed[:] = 0
    devices.control.speedL = speed_l
    owner.wait_for_standstill()
    owner.close()


@pytest.mark.parametrize('raw,processed,aborted', [
    (Wrench(15, 0, 0, 0, 0, 0), Wrench(9, 0, 0, 0, .8, 0), False),
    (Wrench(65, 0, 0, 0, 0, 0), Wrench(0, 0, 0, 0, 0, 0), True),
    (Wrench(60, 0, 0, 0, 0, 0), Wrench(0, 0, 0, 0, 0, 0), True),
    (Wrench(0, 0, 0, 0, 5, 0), Wrench(0, 0, 0, 0, 0, 0), True),
    (Wrench(np.nan, 0, 0, 0, 0, 0), Wrench(0, 0, 0, 0, 0, 0), True),
])
def test_return_soft_loads_warn_raw_extremes_abort(real, clock, tmp_path, raw, processed, aborted):
    config, start, devices, owner = real
    owner.activate_control(confirmed=True)
    devices.receive.pose[0] += .003
    monitor = returns.PX6DForceMonitor(config, SimpleNamespace(read_wrench=lambda: raw),
                                    SimpleNamespace(process=lambda _: processed))
    logger = ExperimentLogger(tmp_path, config, mode='real', strategy='continuous',
                              extra_sample_fields=('software_warnings',), workspace_logging=False)
    try:
        result = returns.SafeReturnExecutor(config, start, owner, force_monitor=monitor, logger=logger).execute()
    finally:
        logger.close()
    assert result.status == ('aborted' if aborted else 'complete')
    assert any(call[0] == 'moveL' for call in devices.control.calls) is not aborted
    if not aborted:
        assert {'return force', 'return torque'} <= owner.diagnostics.keys()
        with (logger.run_dir/'samples.csv').open() as handle:
            rows = list(csv.DictReader(handle))
        assert rows and float(rows[0]['raw_fx']) == 15. and float(rows[0]['dfx']) == 9.
        assert 'return force' in json.loads(rows[0]['software_warnings'])['return_monitor']
    owner.close()


def test_real_contact_loss_reacquires_and_resumes(real):
    config, *_ = real
    policy = tracking(config)
    lose(policy)
    states = [policy.state]
    for i in range(119, 140):
        command = sample(policy, i*.01)
        states.append(policy.state)
        if policy.state == State.LOCAL_REACQUIRE: assert not command.move
    assert policy.state == State.CONTINUOUS_TRACKING and command.move
    assert State.STOP not in states
    assert {'CONTACT_LOST', 'LOCAL_REACQUIRE', 'REACQUIRED'} <= {e.event_type for e in policy.events}


@pytest.mark.parametrize('missing_memory', [False, True])
def test_unavailable_recovery_holds_then_accepts_contact(real, missing_memory):
    config, *_ = real
    policy = tracking(config)
    if missing_memory: policy.last_reliable_contact = None
    else: policy.c['reacquire_enabled'] = False
    for i in range(101, 220):
        command = sample(policy, i*.01, 0.)
        assert not command.move and policy.state != State.STOP
    assert policy.state == State.CONTACT_LOST
    for i in range(220, 245): sample(policy, i*.01)
    assert policy.state == State.CONTINUOUS_TRACKING


def test_contact_at_recovery_budget_edge_can_confirm(real):
    config, *_ = real
    policy = tracking(config)
    lose(policy)
    pose = np.array([.004, 0, 0, 0, 0, 0])
    for i in range(119, 140): sample(policy, i*.01, pose=pose)
    assert policy.state == State.CONTINUOUS_TRACKING


def test_recovery_memory_and_tracking_error_warn_but_spatial_limit_stops(real):
    config, *_ = real
    policy = tracking(config)
    policy.last_reliable_contact['timestamp'] = -10.
    policy._last_recovery_origin = np.zeros(6)
    lose(policy)
    policy.reacquire_reference = np.array([.001, 0])
    assert sample(policy, 1.19, 0.).move
    assert policy.state == State.LOCAL_REACQUIRE
    assert {'recovery_memory', 'recovery_progress', 'recovery_tracking error'} <= policy.diagnostics.keys()
    assert sample(policy, 1.20, 0., pose=np.array([.004, 0, 0, 0, 0, 0])).state == 'STOP'
    assert 'displacement' in policy.reason


@pytest.mark.parametrize('force', [0., 1.5])
def test_recovery_time_budget_stops_without_measurable_progress(real, force):
    config, *_ = real
    policy = tracking(config)
    lose(policy)
    deadline = policy._reacquire+policy.c['reacquire_max_time_sec']
    # Fresh zero-speed feedback at the same pose must not wait forever once
    # pose noise no longer falsely consumes the path budget. No budget reset
    # is granted for contact arriving on the deadline.
    now = policy._last_time
    while now+.01 < deadline:
        now += .01
        command = sample(policy, now, 0.)
        assert policy.state == State.LOCAL_REACQUIRE
    command = sample(policy, deadline, force)
    assert policy.state == State.STOP and not command.move
    assert policy.stop_reason.value == 'STOP_RECOVERY_EXHAUSTED'
    assert policy.reason == 'local reacquire time budget exhausted'


def test_real_overload_uses_measured_budget_then_stops_without_motion(real):
    config, *_ = real
    policy = tracking(config)
    for i in range(101, 180): command = sample(policy, i*.01, 4.)
    assert policy.state == State.CONTINUOUS_TRACKING and command.move
    assert policy.tangent_limit_reason == 'HIGH_FORCE_UNLOADING'
    velocity = command.direction_xy*command.speed
    assert np.dot(velocity, policy.tangent) == pytest.approx(0.)
    assert np.dot(velocity, policy.contact_direction) == pytest.approx(-policy.c['normal_speed_limit'])
    for i in range(180, 280): command = sample(policy, i*.01, 4.)
    assert policy.stop_reason.value == 'STOP_UNLOAD_NO_MOTION' and not command.move


def test_simulation_overload_stall_remains_terminal(real):
    config, *_ = real
    config['continuous_real_execution'] = False
    policy = tracking(config)
    for i, force in enumerate(np.linspace(1.5, 4., 26)[1:], 101): sample(policy, i*.01, force)
    for i in range(126, 290): sample(policy, i*.01, 4.)
    assert policy.state == State.STOP and policy.stop_reason.value == 'STOP_UNLOAD_NO_MOTION'


@pytest.mark.parametrize('kind,small,large', [('z', .0015, .006), ('rotation', 2., 6.)])
def test_real_pose_drift_warns_but_large_drift_stops_and_commands_stay_planar(real, clock, kind, small, large):
    config, start, devices, owner = real
    owner.activate_control(confirmed=True)
    owner.set_continuous_phase('CONTINUOUS_TRACKING')
    # Zero orientation reference makes the angle increments exact for this fake.
    if kind == 'rotation':
        owner.guard.fixed_orientation = np.zeros(3)
        devices.receive.pose[3:] = 0.
    def change(value):
        if kind == 'z': devices.receive.pose[2] = start[2]+value
        else: devices.receive.pose[3] = np.deg2rad(value)
    change(small)
    owner.read_state()
    owner.command_planar_velocity([1, 0], .001, .01)
    assert devices.control.calls[-1][1][2:] == [0., 0., 0., 0.]
    assert 'pose_drift' in owner.diagnostics
    change(large)
    with pytest.raises(rtde.RobotError, match='hard tolerance'): owner.read_state()
    change(0.)
    owner.close()


def test_direction_reconfirm_can_finish_after_timeout(real):
    config, *_ = real
    policy = tracking(config)
    policy.state = State.DIRECTION_RECONFIRM
    policy._reset_confirmation(0.)
    for i in range(101, 125): sample(policy, i*.01)
    assert policy.state == State.CONTINUOUS_TRACKING
    policy._direction_resume_started = 0.
    policy._direction_resume_pose = np.zeros(2)
    policy._direction_resume_tangent = policy.tangent.copy()
    assert sample(policy, 1.25, 4.).move
    assert policy.state == State.CONTINUOUS_TRACKING
    assert 'direction_restart_overload' in policy.diagnostics


@pytest.mark.parametrize('backwards', [True, False])
def test_direction_no_progress_pauses_then_retries(real, backwards):
    config, *_ = real
    policy = tracking(config)
    policy._direction_resume_started = -5.
    policy._direction_resume_pose = np.zeros(2)
    policy._direction_resume_tangent = policy.tangent.copy()
    pose = np.zeros(6)
    if backwards: pose[:2] = -.001*policy.tangent
    assert not sample(policy, 1.01, pose=pose).move
    assert policy.state == State.CONTINUOUS_TRACKING
    assert sample(policy, 1.02, pose=pose).move


def test_legacy_real_runtime_budget_is_warning_but_simulation_stops(real):
    config, *_ = real
    config['continuous_tracking']['max_runtime_sec'] = 120.
    for is_real in (True, False):
        c = deepcopy(config); c['continuous_real_execution'] = is_real
        policy = tracking(c)
        policy._started = -120.
        command = sample(policy, 1.01)
        assert command.state == ('CONTINUOUS_TRACKING' if is_real else 'STOP')
        if is_real: assert 'runtime' in policy.diagnostics


def test_real_fake_runtime_passes_120_seconds_in_tracking(config, monkeypatch, tmp_path, clock):
    target = prepare(config, monkeypatch)
    devices = Devices(config, target)
    observed = []
    update = ContinuousTrackingPolicy.update
    def record(self, now, *a, **kw):
        command = update(self, now, *a, **kw)
        observed.append((now-self._started, command.state))
        return command
    monkeypatch.setattr(ContinuousTrackingPolicy, 'update', record)
    jumped = False
    class ContactSensor(Sensor):
        def read_wrench(self):
            nonlocal jumped
            self.count += 1
            if devices.owner.continuous_phase == 'CONTINUOUS_TRACKING' and not jumped:
                clock.now += 121.
                jumped = True
            return Wrench(0. if self.count < 10 else 1.5, 0, 0, 0, 0, 0)
    def poll(self):
        return 'Q' if self.main_loop and observed and observed[-1][0] > 120 else None
    monkeypatch.setattr(Keyboard, 'poll', poll)
    assert runtime.run(args(tmp_path), controller_factory=devices.controller, reader_factory=ContactSensor) == 0
    assert any(age > 120 and state == 'CONTINUOUS_TRACKING' for age, state in observed)
    run_dir, = (tmp_path/'real/continuous').glob('run_*')
    assert json.loads((run_dir/'termination.json').read_text())['reason'] == 'STOP_USER_REQUEST'


def test_fake_runtime_contact_loss_recovers_without_terminal_stop(config, monkeypatch, tmp_path, clock):
    target = prepare(config, monkeypatch)
    devices = Devices(config, target)
    phases = []
    stage = 'contact'
    class LossSensor(Sensor):
        def read_wrench(self):
            nonlocal stage
            self.count += 1
            phase = devices.owner.continuous_phase
            phases.append(phase)
            if phase == 'CONTINUOUS_TRACKING' and stage == 'contact': stage = 'loss'
            if phase == 'LOCAL_REACQUIRE': stage = 'recover'
            if phase == 'CONTINUOUS_TRACKING' and stage == 'recover': stage = 'done'
            force = 0. if self.count < 10 or stage == 'loss' else 1.5
            return Wrench(force, 0, 0, 0, 0, 0)
    def poll(self):
        return 'Q' if self.main_loop and (stage == 'done' or len(phases) > 500) else None
    monkeypatch.setattr(Keyboard, 'poll', poll)
    assert runtime.run(args(tmp_path), controller_factory=devices.controller, reader_factory=LossSensor) == 0
    assert stage == 'done'
    assert {'FIRST_CONTACT', 'CONTINUOUS_TRACKING', 'CONTACT_LOST', 'LOCAL_REACQUIRE'} <= set(phases)
    run_dir, = (tmp_path/'real/continuous').glob('run_*')
    assert json.loads((run_dir/'termination.json').read_text())['reason'] == 'STOP_USER_REQUEST'


@pytest.mark.parametrize('stop_value', [True, False])
def test_fake_runtime_first_contact_loss_brakes_retries_and_tracks(config, monkeypatch, tmp_path, clock, stop_value, capsys):
    import test_stopping_cycles as stopping
    monkeypatch.setattr(stopping, 'time', clock)
    target = prepare(config, monkeypatch)
    devices = Devices(config, target)
    braking = stopping.braking_device(devices, stop_value=stop_value)
    stage, retry_samples = 'initial', 0
    transitions = []
    def factory(c):
        owner = devices.controller(c)
        set_phase = owner.set_continuous_phase
        def observe_transition(phase, **kwargs):
            if phase != owner.continuous_phase:
                transitions.append((owner.continuous_phase, phase))
                if owner.continuous_phase == 'FIRST_CONTACT' and phase in ('TARGET_SEARCH', 'CONTINUOUS_TRACKING'):
                    assert owner.stop_state == 'STOPPED' and owner.standstill_confirmed
            return set_phase(phase, **kwargs)
        owner.set_continuous_phase = observe_transition
        return owner
    class TransientContactSensor(Sensor):
        def read_wrench(self):
            nonlocal stage, retry_samples
            self.count += 1
            phase = devices.owner.continuous_phase
            if phase == 'FIRST_CONTACT' and stage == 'initial': stage = 'lost'
            if phase == 'TARGET_SEARCH' and stage == 'lost': stage = 'retry'
            if stage == 'retry':
                retry_samples += 1
                if retry_samples >= 210: stage = 'second_contact'
            if phase == 'CONTINUOUS_TRACKING': stage = 'done'
            force = 0. if self.count < 210 else (.7 if stage in ('lost', 'retry') else 1.2)
            return Wrench(force, 0, 0, 0, 0, 0)
    def poll(self):
        return 'Q' if self.main_loop and (stage == 'done' or clock.now > 1008.) else None
    monkeypatch.setattr(Keyboard, 'poll', poll)
    assert runtime.run(args(tmp_path), controller_factory=factory, reader_factory=TransientContactSensor) == 0
    assert stage == 'done'
    assert transitions.count(('TARGET_SEARCH', 'FIRST_CONTACT')) == 2
    output = capsys.readouterr().out
    assert output.count('FIRST CONTACT CONFIRMED') == 1
    for label in ('Fxy = ', 'contact normal n = ', 'tangent t = ', 'follow hand = ', 'force_direction_sign = '):
        assert label in output
    assert transitions.count(('FIRST_CONTACT', 'TARGET_SEARCH')) == 1
    assert ('FIRST_CONTACT', 'CONTINUOUS_TRACKING') in transitions
    assert len(braking['mode_exits']) >= 2
    run_dir, = (tmp_path/'real/continuous').glob('run_*')
    with (run_dir/'samples.csv').open() as handle:
        rows = list(csv.DictReader(handle))
    search_rows = [r for r in rows if r['current_state'] == 'TARGET_SEARCH']
    assert max(float(r['commanded_speed_mps']) for r in search_rows) == pytest.approx(.018)
    summary = json.loads((run_dir/'summary.json').read_text())
    assert 'first_contact_retry' in summary['software_warnings']['policy']
    assert json.loads((run_dir/'termination.json').read_text())['reason'] == 'STOP_USER_REQUEST'


@pytest.mark.parametrize('away', [False, True])
def test_startup_noise_survives_return_and_first_search_command(config, monkeypatch, tmp_path, clock, away):
    target = prepare(config, monkeypatch)
    pose = np.asarray(target).copy(); pose[0] += .003 if away else .0005
    devices = Devices(config, pose)
    speed = devices.receive.getActualTCPSpeed
    def noisy_speed():
        value = speed()
        if (not devices.owner._scan_motion_started and not devices.owner._return_mode
                and not devices.owner._stop_pending):
            value[0] += .0003
            value[3] += .007
        return value
    devices.receive.getActualTCPSpeed = noisy_speed
    search_started = None
    def poll(self):
        nonlocal search_started
        if self.main_loop and devices.owner._scan_motion_started:
            if search_started is None: search_started = clock.now
            if clock.now-search_started > .05: return 'Q'
        if clock.now > 1005.: return 'Q'  # Bound a failed test without hiding its result.
        return None
    monkeypatch.setattr(Keyboard, 'poll', poll)
    assert runtime.run(args(tmp_path), controller_factory=devices.controller, reader_factory=Sensor) == 0
    assert any(call[0] == 'speedL' and np.linalg.norm(call[1][:2]) > 0 for call in devices.control.calls)
    assert any(call[0] == 'moveL' for call in devices.control.calls) is away


def test_return_segment_time_budget_only_warns(real, clock):
    config, start, devices, owner = real
    config['safe_return']['return_segment_timeout_sec'] = .02
    devices.receive.pose[0] += .003
    owner.activate_control(confirmed=True)
    move = devices.control.moveL
    pending = None
    def delayed_move(*args):
        nonlocal pending
        pending = (clock.now+.08, args)
        return True
    devices.control.moveL = delayed_move
    def poll():
        nonlocal pending
        if pending is not None and clock.now >= pending[0]:
            move(*pending[1]); pending = None
        return None
    monitor = returns.PX6DForceMonitor(config, Sensor(), WrenchPreprocessor.from_config(config['preprocessing']))
    result = returns.SafeReturnExecutor(config, start, owner, force_monitor=monitor, poll=poll).execute()
    assert result.status == 'complete'
    assert 'return_progress' in owner.diagnostics
