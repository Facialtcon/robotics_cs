"""Both real CLI paths over Mock RTDE/PX6D, including granular startup."""
from copy import deepcopy
import json
import sys

import numpy as np
import pytest
import yaml

from app import continuous_runtime as runtime
from app.configuration import prepare_real
from app.fixed_target import ROOT, configure_fixed, prepare_fixed
from core.models import Wrench
from doubles import Devices, Keyboard, Sensor
from policy.continuous_tracking import ContinuousTrackingPolicy, State
from robot.rtde_controller import _rotvec_to_matrix
from test_real_tracking_guards import clock


@pytest.mark.parametrize('fixed', [False, True], ids=['granular-main-cli', 'fixed-target-cli'])
def test_cli_uses_shared_policy_for_recontact_second_loss_and_full_circle(
        config, monkeypatch, tmp_path, clock, fixed):
    import run_continuous_tracking as main_entry
    import run_fixed_target_tracking as fixed_entry
    import run_project
    from visualization import fixed_target, run_plots

    assert main_entry.main is runtime.main
    assert run_project.STEPS['18'][1][1:] == ['run_continuous_tracking.py', '--execute']
    source = (ROOT/'config.yaml').read_bytes()
    effective = deepcopy(config)
    # Shorten acquisition only; control, filter, speed and recovery settings
    # remain exactly those loaded from the project's config.yaml.
    effective['preprocessing']['baseline']['sample_count'] = 3
    effective['preprocessing']['granular_baseline']['sample_count'] = 3
    effective['safe_return']['startup_bias_sample_count'] = 3
    monkeypatch.setattr(runtime, 'load_config', lambda _: deepcopy(effective))
    prepared = deepcopy(effective)
    if fixed:
        configure_fixed(prepared)
        _, start = prepare_fixed(prepared, ROOT/'config.yaml', ROOT/'single_point_calibration.yaml', 'A')
    else:
        _, start = prepare_real(prepared, ROOT/'config.yaml')
    approach = np.asarray(prepared['policy']['search_direction_xy'])
    devices = Devices(config, start)
    air = np.array([0., 0., 9.4, 0., 0., 0.])
    medium = np.array([.4, -.2, .3]) if not fixed else np.zeros(3)
    policies, transitions, stops, prompts = [], [], [], []
    contexts = 0

    class TestKeyboard(Keyboard):
        def __enter__(self):
            nonlocal contexts
            contexts += 1
            self.main_loop = contexts > 1
            return self
        def poll(self):
            # A virtual-time guard makes a stalled regression fail finitely.
            return 'Q' if clock.now > 1200. else None
        def read_line(self, prompt):
            prompts.append(prompt)
            return ''
        def read_text(self, prompt):
            return '1'

    class TestSensor(Sensor):
        scan_reads = 0
        def read_wrench(self):
            base = medium.copy() if devices.receive.pose[2] <= start[2]+1e-8 else np.zeros(3)
            if devices.owner.control is not None and not devices.owner._return_mode:
                self.scan_reads += 1
                policy = policies[0] if policies else None
                magnitude = 0. if self.scan_reads < 5 else 1.5
                if policy is not None and policy.state == State.CONTINUOUS_TRACKING:
                    magnitude = .3
                elif policy is not None and policy.state == State.LOCAL_REACQUIRE:
                    magnitude = (1.2 if policy._recovery_count == 1 and
                                 policy.reacquire_actual_heading_deg >= 20. else .3)
                base[:2] += -magnitude*approach
            rotation = _rotvec_to_matrix(devices.receive.pose[3:]) @ np.asarray(
                config['preprocessing']['coordinate_transform']['rotation_sensor_to_tool'])
            return Wrench.from_sequence(air+np.r_[rotation.T @ base, np.zeros(3)])

    update = ContinuousTrackingPolicy.update
    def observe(policy, now, raw, processed, robot, **kwargs):
        if not policies:
            policies.append(policy)
        assert policies[0] is policy and type(policy) is ContinuousTrackingPolicy
        before, previous, index = policy.state, policy._velocity.copy(), len(devices.control.calls)
        command = update(policy, now, raw, processed, robot, **kwargs)
        if before == State.LOCAL_REACQUIRE and policy.state == State.CONTINUOUS_TRACKING:
            assert np.hypot(processed.fx, processed.fy) >= config['policy']['contact_threshold']
            assert command.move and not policy.stop_requested and not policy.stop_confirmed
            assert policy._confirm_started is None
            velocity = command.direction_xy*command.speed
            assert np.linalg.norm(velocity-previous) <= policy.c['command_acceleration']*policy.dt+1e-12
            transitions.append((index, velocity))
        return command
    monkeypatch.setattr(ContinuousTrackingPolicy, 'update', observe)
    monkeypatch.setattr(runtime, 'OperatorKeyboard', TestKeyboard)
    # Plot generation is unrelated to control; preserve all production logs.
    monkeypatch.setattr(run_plots, 'generate_run_visualization', lambda _: [])
    monkeypatch.setattr(fixed_target, 'generate_fixed_visualization', lambda *a: [])
    run = runtime.run
    def injected_run(args, **kwargs):
        def factory(c):
            owner = devices.controller(c)
            request_stop = owner.request_stop
            def stop(*a, **kw):
                stops.append(len(devices.control.calls))
                return request_stop(*a, **kw)
            monkeypatch.setattr(owner, 'request_stop', stop)
            return owner
        return run(args, controller_factory=factory, reader_factory=TestSensor, **kwargs)
    monkeypatch.setattr(runtime, 'run', injected_run)
    argv = ['--execute', '--output', str(tmp_path/'runs')]
    if fixed:
        result = fixed_entry.main(argv+['--p0', 'A'])
    else:
        monkeypatch.setattr(sys, 'argv', ['run_continuous_tracking.py', *argv])
        result = main_entry.main()
    # Recovery exhaustion is a recorded nonzero CLI result, not an exception.
    assert result == 1
    policy, = policies
    assert policy.stop_reason.value == 'RECOVERY_FULL_CIRCLE_NO_CONTACT'
    assert policy._recovery_count == 2 and policy.reacquire_heading_deg == 360.
    assert policy.reacquire_circle == 3 and policy.reacquire_radius == .006
    assert policy.reacquire_actual_heading_deg > 357.
    transition, = transitions
    index, velocity = transition
    assert index not in stops and devices.control.calls[index][0] == 'speedL'
    np.testing.assert_allclose(devices.control.calls[index][1][:2], velocity)
    log, = (tmp_path/'runs/real/continuous').glob('run_*')
    background = json.loads((log/'granular_baseline_at_start.json').read_text())
    np.testing.assert_allclose(background['zero_bias_sensor'], air, atol=1e-8)
    np.testing.assert_allclose(background['granular_baseline_output'][:3], medium, atol=1e-8)
    assert background['sample_count'] == (0 if fixed else 3)
    assert len(prompts) == (3 if fixed else 4)
    snapshot = yaml.safe_load((log/'config_snapshot.yaml').read_text())
    assert snapshot['continuous_tracking'] == prepared['continuous_tracking']
    assert snapshot['preprocessing']['kalman'] == config['preprocessing']['kalman']
    scan_source = 'single_point_calibration.yaml' if fixed else 'scan_calibration.yaml'
    assert snapshot['continuous_calibration_sources']['scan'] == str(ROOT/scan_source)
    assert snapshot['continuous_calibration_sources']['workspace'] == str(ROOT/'workspace/config/workspace_calibration.yaml')
    assert json.loads((log/'termination.json').read_text())['reason'] == 'RECOVERY_FULL_CIRCLE_NO_CONTACT'
    assert not devices.control.connected and not devices.receive.connected
    assert (ROOT/'config.yaml').read_bytes() == source
