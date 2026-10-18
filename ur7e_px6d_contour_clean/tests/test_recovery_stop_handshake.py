"""Fresh RTDE reads can revoke a stop between policy update and SDK send."""
import csv
import json

import numpy as np
import pytest

from app import continuous_runtime as runtime
from core.models import Wrench
from doubles import Devices, Keyboard, Sensor
from policy.continuous_tracking import State
from robot.rtde_controller import StopPendingStandstill
from test_continuous_runtime import args, prepare
from test_real_tracking_guards import clock


@pytest.mark.parametrize('case', ['threshold_noise', 'still_moving', 'motion_fault'])
def test_recovery_waits_for_real_stop_without_advancing_arc(config, monkeypatch, tmp_path, clock, case):
    target = prepare(config, monkeypatch, synthetic_force_sign=1)
    devices = Devices(config, target)
    inside_send, race_at, stop_started = False, None, None
    rejected, resumed, noise, waits = [], [], [], []
    policies = []
    actual_speed = devices.receive.getActualTCPSpeed
    actual_pose = devices.receive.getActualTCPPose
    pose_reads = 0
    def pose():
        nonlocal pose_reads
        value = actual_pose()
        if race_at is not None and not resumed:
            pose_reads += 1
            value[0] += 10e-6*(-1)**pose_reads  # Stationary encoder variation.
        return value
    monkeypatch.setattr(devices.receive, 'getActualTCPPose', pose)

    def speed():
        nonlocal race_at
        owner = devices.owner
        if inside_send and owner.continuous_phase == 'LOCAL_REACQUIRE' and owner.stop_pending and race_at is None:
            race_at = clock.now
        if race_at is not None and not resumed:
            elapsed = clock.now-race_at
            if case == 'still_moving':
                value = .002
            elif elapsed < .05:
                threshold = owner.config['continuous_settle_speed_mps']
                value = threshold*(1.01 if int(elapsed/.01) % 2 == 0 else .99)
            else:
                value = 0.
            noise.append(value)
            return np.array([value, 0., 0., 0., 0., 0.])
        return actual_speed()
    monkeypatch.setattr(devices.receive, 'getActualTCPSpeed', speed)

    def factory(c):
        owner = devices.controller(c)
        send = owner.command_planar_velocity
        def command(direction, velocity, duration):
            nonlocal inside_send, stop_started
            inside_send = True
            # A genuinely new RTDE packet arrives between the two reads.
            clock.now += .001
            if case == 'motion_fault' and owner.continuous_phase == 'LOCAL_REACQUIRE':
                owner.motion_fault = 'synthetic recovery motion fault'
            calls = len(devices.control.calls)
            try:
                send(direction, velocity, duration)
                if owner.continuous_phase == 'LOCAL_REACQUIRE':
                    resumed.append(clock.now)
            except StopPendingStandstill:
                assert len(devices.control.calls) == calls  # No SDK motion was sent.
                assert not owner.standstill_confirmed and owner.stop_pending
                assert owner.stop_state == 'STOPPED'  # A previously held stop was revoked.
                stop_started = owner._stop_started
                rejected.append(clock.now)
                raise
            finally:
                inside_send = False
        monkeypatch.setattr(owner, 'command_planar_velocity', command)
        return owner

    update = runtime.ContinuousTrackingPolicy.update
    def observe(policy, now, raw, processed, robot, **kwargs):
        if not policies:
            policies.append(policy)
        before = (policy._theta, policy.reacquire_actual_heading_deg)
        command = update(policy, now, raw, processed, robot, **kwargs)
        if command.state == 'CONTINUOUS_TRACKING' and policy._recovery_count == 0:
            policy._recovery_count = 39  # Same handshake at the reported 40th recovery.
        if policy.state == State.LOCAL_REACQUIRE and policy.tangent_limit_reason == 'RECOVERY_WAIT_STANDSTILL':
            assert not command.move and policy.stop_requested
            assert (policy._theta, policy.reacquire_actual_heading_deg) == before == (0., 0.)
            np.testing.assert_array_equal(policy.reacquire_reference, policy.reacquire_origin[:2])
            assert devices.owner._stop_started == stop_started
            waits.append(now)
        if waits and policy.state == State.LOCAL_REACQUIRE and command.move and not resumed:
            assert policy.reacquire_actual_heading_deg == 0.  # Includes the final waiting observation.
        return command
    monkeypatch.setattr(runtime.ContinuousTrackingPolicy, 'update', observe)

    class LossSensor(Sensor):
        scan_reads = 0
        lost = False
        def read_wrench(self):
            if devices.owner.control is not None and not devices.owner._return_mode:
                self.scan_reads += 1
                if devices.owner.continuous_phase == 'CONTINUOUS_TRACKING':
                    self.lost = True
            force = 1.5 if self.scan_reads >= 10 and not self.lost else 0.
            return Wrench(force, 0., 0., 0., 0., 0.)
    def poll(self):
        return 'Q' if self.main_loop and (len(resumed) >= 5 or clock.now > 1005.) else None
    monkeypatch.setattr(Keyboard, 'poll', poll)
    from visualization import run_plots
    generate = run_plots.generate_run_visualization
    saved_checks = []
    def after_save(path):
        assert not devices.control.connected and not devices.receive.connected
        assert (path/'scan_stop_snapshot.json').exists() and (path/'summary.json').exists()
        originals = {p.name: p.read_bytes() for p in path.iterdir() if p.suffix in ('.csv', '.json', '.txt', '.yaml')}
        artifacts = generate(path)
        assert [p.name for p in artifacts if p.suffix == '.png'] == ['experiment_overview.png']
        assert [p.name for p in artifacts if p.suffix == '.html'] == ['实验回放.html']
        for name, content in originals.items():
            assert (path/name).read_bytes() == content
        saved_checks.append(True)
        return artifacts
    monkeypatch.setattr(run_plots, 'generate_run_visualization', after_save)
    result = runtime.run(args(tmp_path), controller_factory=factory, reader_factory=LossSensor)
    log, = (tmp_path/'real/continuous').glob('run_*')
    termination = json.loads((log/'termination.json').read_text())
    policy, = policies
    assert policy._recovery_count == 40
    assert 'stop request pending measured standstill' not in termination['detail']
    if case == 'threshold_noise':
        assert result == 0 and rejected and len(waits) >= 5 and len(resumed) == 5
        threshold = devices.owner.config['continuous_settle_speed_mps']
        assert any(0. < v < threshold for v in noise) and any(v > threshold for v in noise)
        assert resumed[0]-race_at >= config['continuous_tracking']['settle_hold_sec']+.04
        assert termination['reason'] == 'STOP_USER_REQUEST'
        rows = list(csv.DictReader((log/'samples.csv').open()))
        paused = [r for r in rows if r['tangent_limit_reason'] == 'RECOVERY_WAIT_STANDSTILL']
        assert len(paused) >= 5
        assert all(float(r['command_speed']) == float(r['reacquire_heading_deg']) ==
                   float(r['reacquire_actual_heading_deg']) == 0. for r in paused)
        assert policy.reacquire_heading_deg > 0. and policy.reacquire_circle == 1
    else:
        assert result == 1 and not resumed
        assert termination['reason'] == 'STOP_MOTION_ERROR'
        assert ('did not settle' if case == 'still_moving' else 'synthetic recovery motion fault') in termination['detail']
    assert len(list(log.glob('*.png'))) == 1 and len(list(log.glob('*.html'))) == 1
    assert (log/'tcp_commands.csv').exists() and (log/'policy_waypoints.csv').exists()
    assert saved_checks == [True]
