"""Air validation exercises the real execution adapter without opening hardware."""

from copy import deepcopy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from app import real_validation
from config.loader import load_config
from core.models import RobotState, Wrench


ROOT = Path(__file__).resolve().parents[1]
ZERO = Wrench(0, 0, 0, 0, 0, 0)


class FakeClock:
    def __init__(self):
        self.now = 0.0
        self.observers = []

    def __call__(self):
        return self.now

    def sleep(self, duration):
        assert duration >= 0
        for observer in self.observers:
            observer(float(duration))
        self.now += float(duration)


class FakeActualController:
    """Measured-pose device double: motion continues until a stop is accepted."""

    is_dry_run = False

    def __init__(self, clock, *, ignore_stop=False):
        self.clock = clock
        self.pose = np.array([.4, -.2, .3, 0, 3.14, 0])
        self.origin = self.pose.copy()
        self.velocity = np.zeros(6)
        self.commands = []
        self.measured_poses = [self.pose.copy()]
        self.connected = False
        self.closed = False
        self.stop_calls = 0
        self.ignore_stop = ignore_stop
        self.clock.observers.append(self._integrate)

    def _integrate(self, duration):
        self.pose += self.velocity * duration
        self.measured_poses.append(self.pose.copy())

    def connect(self, **kwargs):
        self.connected = True

    def read_state(self):
        assert self.connected
        return RobotState(self.clock(), self.pose.copy(), self.velocity.copy())

    def read_diagnostic_state(self):
        return self.read_state()

    def command_planar_velocity(self, direction_xy, speed, duration):
        assert self.connected
        direction = np.asarray(direction_xy, dtype=float)
        self.commands.append((direction.copy(), float(speed), float(duration)))
        self.velocity[:] = 0
        self.velocity[:2] = direction * speed
        # Emulate the synchronous command's elapsed execution period. Additional
        # time spent outside the call also moves the TCP until explicitly stopped.
        self.clock.sleep(float(duration))

    def safe_stop_motion(self, force=False):
        self.stop_calls += 1
        if not self.ignore_stop:
            self.velocity[:] = 0

    def stop(self):
        self.safe_stop_motion()

    def close(self):
        self.closed = True
        self.connected = False


class FakeRealReader:
    firmware = "offline-test-device"

    def __init__(self, controller, *, after_motion=ZERO):
        self.controller = controller
        self.after_motion = after_motion
        self.connected = False
        self.closed = False
        self.reads = []

    def connect(self):
        self.connected = True

    def read_wrench(self):
        assert self.connected
        self.reads.append((len(self.controller.commands), self.controller.velocity.copy()))
        return self.after_motion if self.controller.commands else ZERO

    def close(self):
        self.closed = True
        self.connected = False


def make_runner(tmp_path, *, after_motion=ZERO, ignore_stop=False, **options):
    config, policy_config, _ = real_validation.load_validation_config(ROOT / "config.yaml")
    clock = FakeClock()
    controller = FakeActualController(clock, ignore_stop=ignore_stop)
    reader = FakeRealReader(controller, after_motion=after_motion)
    runner = real_validation.AirValidationRunner(
        config, policy_config, controller, reader,
        options=real_validation.AirValidationOptions(**options),
        clock=clock, sleep=clock.sleep, output_dir=tmp_path,
        print_fn=lambda *args, **kwargs: None,
    )
    return runner, controller, reader, config, policy_config


def test_preview_never_constructs_a_hardware_device(monkeypatch, tmp_path):
    def forbidden(*args, **kwargs):
        pytest.fail("preview must not construct or connect a hardware device")

    monkeypatch.setattr(real_validation, "URRTDEController", forbidden)
    monkeypatch.setattr(real_validation, "PX6DReader", forbidden)
    args = SimpleNamespace(config=str(ROOT / "config.yaml"), execute=False,
                           speed_mm_s=1.0, output_root=str(tmp_path))
    assert real_validation.main(args) == 0


def test_declined_air_arming_sends_no_motion(tmp_path):
    runner, controller, reader, _, _ = make_runner(tmp_path)
    summary = runner.run(confirm=lambda: False, poll_key=lambda: None)
    assert summary["status"] == "not_armed"
    assert not controller.commands
    assert controller.closed and reader.closed


def test_loading_air_validation_does_not_change_scan_parameters_or_source_yaml():
    path = ROOT / "config.yaml"
    original_bytes = path.read_bytes()
    source = load_config(path)
    original = deepcopy(source)
    config, policy_config, calibration = real_validation.load_validation_config(path)
    expected_policy = deepcopy(source["policy"])
    expected_policy["search_direction_xy"] = calibration["scan_direction_xy"]
    assert policy_config == expected_policy
    assert source == original
    assert path.read_bytes() == original_bytes
    # Runtime objects must be independently editable without changing loaded
    # scan parameters used by the runner's safety layer.
    snapshot = deepcopy(config)
    policy_config["tangent_step"] *= 2
    assert config == snapshot


@pytest.mark.parametrize("wrench", [
    Wrench(100, 0, 0, 0, 0, 0),
    Wrench(float("nan"), 0, 0, 0, 0, 0),
    Wrench(1.1, 0, 0, 0, 0, 0),
])
def test_real_force_and_nonfinite_samples_stop_despite_virtual_contact_input(tmp_path, wrench):
    runner, controller, reader, config, _ = make_runner(tmp_path, after_motion=wrench)
    summary = runner.run(confirm=lambda: True, poll_key=lambda: None)
    assert summary["status"] == "failed"
    assert controller.commands
    assert controller.stop_calls > 0
    assert np.linalg.norm(controller.velocity) == 0
    assert controller.closed and reader.closed
    assert summary["physical_contact_validated"] is False
    if wrench.fx == 1.1:
        # EMA alone would initially mask this physical contact. The air runner
        # must separately guard the unfiltered increment over its real bias.
        first_filtered = (1 - config["preprocessing"]["filter_alpha"]) * wrench.fx
        assert first_filtered < config["policy"]["contact_threshold"] < wrench.fx
        assert len(controller.commands) == 1


def test_stop_requires_measured_settling_not_only_successful_method_return(tmp_path):
    runner, controller, reader, _, _ = make_runner(
        tmp_path, ignore_stop=True, settle_timeout_sec=.1,
    )
    summary = runner.run(confirm=lambda: True, poll_key=lambda: None)
    assert summary["status"] == "failed"
    assert controller.commands
    assert controller.stop_calls > 0
    assert np.linalg.norm(controller.velocity) > 0
    assert controller.closed and reader.closed


def test_air_boundary_prevents_the_actual_tcp_leaving_the_approved_region(tmp_path):
    runner, controller, _, _, _ = make_runner(tmp_path, half_extent=.002)
    summary = runner.run(confirm=lambda: True, poll_key=lambda: None)
    assert summary["status"] == "failed"
    displacement = np.asarray(controller.measured_poses)[:, :3] - controller.origin[:3]
    assert np.max(np.abs(displacement)) <= .002 + 1e-9


def test_initial_moving_tcp_cannot_be_used_as_a_stationary_bias(tmp_path):
    runner, controller, reader, _, _ = make_runner(tmp_path)
    controller.velocity[0] = .001
    summary = runner.run(confirm=lambda: True, poll_key=lambda: None)
    if summary["status"] == "failed":
        assert not controller.commands
    else:
        assert summary["status"] == "complete"
        # Monitoring during deceleration is allowed. The baseline itself must
        # have at least the configured number of stationary pre-motion samples.
        stationary = [speed for count, speed in reader.reads
                      if count == 0 and np.linalg.norm(speed) <= .0001]
        assert len(stationary) >= runner.config["preprocessing"]["baseline"]["sample_count"]


def test_scripted_air_run_uses_shared_policy_with_capped_actual_motion(tmp_path):
    runner, controller, reader, config, policy_config = make_runner(tmp_path)
    original_config, original_policy = deepcopy(config), deepcopy(policy_config)
    summary = runner.run(confirm=lambda: True, poll_key=lambda: None)
    assert summary["status"] == "complete", summary
    assert summary["physical_contact_validated"] is False
    assert all(summary["coverage"].values())
    assert config == original_config and policy_config == original_policy
    assert controller.closed and reader.closed
    assert controller.commands and max(speed for _, speed, _ in controller.commands) <= .001
    displacement = np.asarray(controller.measured_poses)[:, :3] - controller.origin[:3]
    assert np.max(np.abs(displacement)) <= .03 + 1e-9
    assert np.all(displacement[:, 2] == 0)
    baseline_reads = [speed for count, speed in reader.reads if count == 0]
    assert len(baseline_reads) >= config["preprocessing"]["baseline"]["sample_count"]
    assert all(np.linalg.norm(speed) <= .0001 for speed in baseline_reads)

    episodes = json.loads((tmp_path / "probe_episodes.json").read_text(encoding="utf-8"))
    initial = [item for item in episodes if item["purpose"] == "INITIALIZATION"]
    tracking = [item for item in episodes if item["purpose"] == "TRACKING"]
    recovery = [item for item in episodes if item["purpose"] == "RECOVERY"]
    assert len(initial) >= 3 and all(item["return_completed"] for item in initial)
    assert len(tracking) >= 3
    assert [item["outcome"] for item in tracking[:3]] == ["CONTACT", "CONTACT", "NO_CONTACT"]
    assert sum(item["return_completed"] for item in recovery) >= 2
    # The shared coordinator can schedule the next ray in the same tick that
    # completes the second. Stopping that unexecuted ray must remain visible.
    assert all(item["return_completed"] or
               (item["outcome"] == "ABORTED" and item["end_pose"] is None)
               for item in recovery)
    assert all(item["anchor_pose"] == recovery[0]["anchor_pose"] for item in recovery)

    sources = json.loads((tmp_path / "policy_source_snapshot.json").read_text(encoding="utf-8"))
    for name in ("rule_policy.py", "local_tracking.py", "local_recovery.py", "probe_episode.py"):
        path = (ROOT / "policy" / name).resolve()
        assert sources[str(path)] == hashlib.sha256(path.read_bytes()).hexdigest()
    assert Path(summary["policy_source"]).resolve() == (ROOT / "policy/rule_policy.py").resolve()

    rows = [json.loads(line) for line in (tmp_path / "trajectory.jsonl").read_text(encoding="utf-8").splitlines()]
    previous = None
    observed_motion = False
    by_id = {item["probe_id"]: item for item in episodes}
    for row in rows:
        pose = np.asarray(row["current_tcp_pose"])
        movement = np.asarray(row["actual_movement"])
        expected = np.zeros(6) if previous is None else pose - previous["current_tcp_pose"]
        assert np.allclose(movement, expected, atol=1e-12, rtol=0)
        assert row["movement_corresponds_to"] == "previous_command_interval"
        assert row["actual_raw_wrench"] == [0] * 6
        assert row["actual_processed_wrench"] == [0] * 6
        if previous is not None:
            assert row["previous_command_direction"] == previous["command_direction"]
            assert row["previous_command_phase"] == previous["phase"]
            assert row["previous_executed_speed"] == previous["executed_speed"]
        if np.linalg.norm(movement[:2]) > 1e-10:
            observed_motion = True
            assert np.allclose(movement[:2] / np.linalg.norm(movement[:2]),
                               row["previous_command_direction"], atol=1e-8)
        if row["phase"] == "BIAS":
            assert np.linalg.norm(row["actual_tcp_speed"][:3]) <= .0001
        if row["probe_id"] is not None:
            assert row["target_anchor"] == by_id[row["probe_id"]]["anchor_pose"]
        if row["executed_speed"] > 0:
            assert row["target_anchor"] is not None
            assert len(row["command_target_pose"]) == 6
        previous = row
    assert observed_motion
    assert any(np.linalg.norm(row["synthetic_policy_wrench"][:3]) >= 1 for row in rows)
