import ast
import builtins
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest
import yaml

import calibrate_workspace as cli


ROOT = Path(__file__).resolve().parents[1]
POINTS = {
    "P0": [.6, .2, .1000, 1.1, -2.2, .001],
    "P1": [.8, .2, .1001, 1.2, -2.1, .002],
    "P2": [.8, .3, .0999, 1.3, -2.0, .003],
    "P3": [.6, .3, .1000, 1.4, -1.9, .004],
}


class FakeReceive:
    def __init__(self, points=None, speeds=None):
        self.points = list(POINTS.values()) if points is None else points
        self.speeds = [[0.] * 6 for _ in self.points] if speeds is None else speeds
        self.index = -1
        self.disconnected = False

    def getActualTCPPose(self):
        self.index += 1
        return self.points[self.index]

    def getActualTCPSpeed(self):
        return self.speeds[self.index]

    def isConnected(self):
        return True

    def getTCPOffset(self):
        pytest.fail("active TCP was queried despite receive-only declared metadata")

    def disconnect(self):
        self.disconnected = True


def answers(*values):
    iterator = iter(values)
    return lambda _: next(iterator)


def minimal_config(tmp_path):
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump({"robot": {"robot_ip": "192.0.2.10"},
                                    "tcp": {"offset": [0, 0, .2, 0, 0, 0]}}))
    return path


def test_teaching_reads_only_receive_and_preserves_all_four_raw_poses(tmp_path):
    config = minimal_config(tmp_path)
    config_before = config.read_bytes()
    output = tmp_path / "workspace.yaml"
    receiver = FakeReceive()
    factory_calls = []

    def factory(address):
        factory_calls.append(address)
        return receiver

    result = cli.main(["--config", str(config), "--output", str(output)], receiver_factory=factory,
                      input_fn=answers("", "", "", "", "SAVE"), print_fn=lambda _: None, clock=lambda: 1000.)
    assert result == 0 and receiver.disconnected
    assert factory_calls == ["192.0.2.10"]
    assert config.read_bytes() == config_before
    payload = yaml.safe_load(output.read_text())
    for name, expected in POINTS.items():
        assert [payload["raw_points"][name][field] for field in cli.POSE_FIELDS] == expected
        assert payload["metadata"]["samples"][name]["tcp_pose"] == expected
        assert payload["metadata"]["samples"][name]["tcp_speed"] == [0.] * 6
        assert payload["metadata"]["samples"][name]["read_timestamp_utc"]
    assert payload["metadata"]["configured_tcp_offset"] == [0, 0, .2, 0, 0, 0]
    assert payload["metadata"]["active_tcp_verified"] is False
    assert set(payload["rectified_points"]) == {"R0", "R1", "R2", "R3"}
    source = ast.parse(Path(cli.__file__).read_text())
    imports = [node.module for node in ast.walk(source) if isinstance(node, ast.ImportFrom)]
    imports += [alias.name for node in ast.walk(source) if isinstance(node, ast.Import) for alias in node.names]
    assert "rtde_control" not in imports and "robot.rtde_controller" not in imports


def test_moving_corner_is_rejected_and_retried_without_any_stop_command(tmp_path):
    receiver = FakeReceive([POINTS["P0"], *POINTS.values()], [[.01, 0, 0, 0, 0, 0], *([[0.] * 6] * 4)])
    output = tmp_path / "workspace.yaml"
    messages = []
    result = cli.main(["--robot-ip", "192.0.2.10", "--output", str(output)],
                      receiver_factory=lambda _: receiver, input_fn=answers("", "", "", "", "", "SAVE"),
                      print_fn=messages.append)
    assert result == 0 and receiver.index == 4
    assert any("仍在运动" in message for message in messages)
    assert yaml.safe_load(output.read_text())["raw_points"]["P0"]["x"] == .6


@pytest.mark.parametrize("pose", [[float("nan")] * 6, [0.] * 5])
def test_invalid_pose_cannot_be_accepted(pose):
    with pytest.raises(cli.InvalidTeachingSample):
        cli.read_corner(FakeReceive([pose]), "P0")


@pytest.mark.parametrize("input_values", [("ABORT",), ("", "", "", "", "CANCEL")])
def test_cancel_does_not_write_a_calibration(tmp_path, input_values):
    receiver = FakeReceive()
    output = tmp_path / "workspace.yaml"
    result = cli.main(["--robot-ip", "192.0.2.10", "--output", str(output)],
                      receiver_factory=lambda _: receiver, input_fn=answers(*input_values), print_fn=lambda _: None)
    assert result == 0 and receiver.disconnected
    assert not output.exists()


def test_missing_receive_module_does_not_write_or_construct_a_connection(tmp_path, monkeypatch):
    original_import = builtins.__import__

    def importing(name, *args, **kwargs):
        if name == "rtde_receive":
            raise ImportError("injected missing SDK")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", importing)
    output = tmp_path / "workspace.yaml"
    result = cli.main(["--robot-ip", "192.0.2.10", "--output", str(output)], print_fn=lambda _: None)
    assert result == 1 and not output.exists()


def test_communication_failure_closes_receive_without_writing(tmp_path):
    receiver = FakeReceive()

    def failed_read():
        raise RuntimeError("injected RTDE disconnect")

    receiver.getActualTCPPose = failed_read
    output = tmp_path / "workspace.yaml"
    result = cli.main(["--robot-ip", "192.0.2.10", "--output", str(output)],
                      receiver_factory=lambda _: receiver, input_fn=answers(""), print_fn=lambda _: None)
    assert result == 1 and receiver.disconnected and not output.exists()


def test_offline_import_skips_device_and_config_and_backs_up_existing_output(tmp_path):
    source = tmp_path / "points.json"
    source.write_text(json.dumps(list(POINTS.values())))
    output = tmp_path / "workspace.yaml"
    output.write_text("old calibration preserved")

    def forbidden(_):
        pytest.fail("offline import constructed a receive interface")

    result = cli.main(["--points-file", str(source), "--config", str(tmp_path / "missing.yaml"), "--output", str(output)],
                      receiver_factory=forbidden, input_fn=answers("SAVE"), print_fn=lambda _: None)
    assert result == 0
    backups = list(tmp_path.glob("workspace.*.backup.yaml"))
    assert len(backups) == 1 and backups[0].read_text() == "old calibration preserved"
    payload = yaml.safe_load(output.read_text())
    assert payload["metadata"]["input_mode"] == "offline_import"
    assert payload["metadata"]["active_tcp_verified"] is False


def test_actual_offline_cli_writes_only_the_selected_output(tmp_path):
    source = tmp_path / "points.yaml"
    source.write_text(yaml.safe_dump({"raw_points": POINTS}))
    output = tmp_path / "workspace.yaml"
    result = subprocess.run([sys.executable, str(ROOT / "calibrate_workspace.py"), "--points-file", str(source),
                             "--output", str(output)], input="SAVE\n", text=True, capture_output=True, cwd=ROOT)
    assert result.returncode == 0, result.stdout + result.stderr
    assert output.is_file()
    assert "离线导入" in result.stdout and "active TCP" in result.stdout


def test_offline_xy_only_input_is_not_a_valid_tcp_calibration(tmp_path):
    source = tmp_path / "points.json"
    source.write_text(json.dumps([[0, 0], [1, 0], [1, 1], [0, 1]]))
    output = tmp_path / "workspace.yaml"
    assert cli.main(["--points-file", str(source), "--output", str(output)], print_fn=lambda _: None) == 1
    assert not output.exists()


def test_workspace_output_cannot_overwrite_existing_control_configuration(tmp_path):
    config = minimal_config(tmp_path)
    original = config.read_bytes()
    source = tmp_path / "points.json"
    source.write_text(json.dumps(POINTS))
    result = cli.main(["--points-file", str(source), "--config", str(config), "--output", str(config)],
                      input_fn=answers("SAVE"), print_fn=lambda _: None)
    assert result == 1 and config.read_bytes() == original
