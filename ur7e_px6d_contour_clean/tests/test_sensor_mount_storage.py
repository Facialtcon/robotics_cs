"""Hardware-free persistence and integration checks for mount calibration."""
import copy
import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from core.models import Wrench
from sensor.force_preprocess import WrenchPreprocessor
from tools.sensor_mount_calibration.storage import (
    RunStore, commit_config_update, preview_config_update,
)


RX = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
RZ = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])


CONFIG_TEXT = """# preserve this exact spelling and these comments
robot: {max_tcp_speed: 0.030, payload: 3.0}
tcp: {offset: [0.001, 0, 0.25, 0, 0, 0]}
preprocessing:
  filter_alpha: 0.0
  gravity_wrench_sensor: [1, 2, 3, 0, 0, 0] # do not fit this
  granular_baseline_output: [0, 0, 0, 0, 0, 0]
  coordinate_transform:
    rotation_sensor_to_tool: null # mounting value
    reference_tool_orientation: null
    rotation_sensor_to_base:
      - [1, 0, 0]
      - [0, 1, 0]
      - [0, 0, 1]
    # Preserve this origin comment.
    sensor_origin_in_base_m: [0, 0, 0]
policy: {search_speed: 0.018, probe_direction_sign: -1}
continuous_tracking: {force_direction_sign: 1}
"""


def write_config(tmp_path, text=CONFIG_TEXT):
    path = tmp_path / "config.yaml"
    path.write_bytes(text.encode())
    return path


def test_preview_only_installation_changes_and_no_write(tmp_path):
    path = write_config(tmp_path)
    original = path.read_bytes()
    preview = preview_config_update(path, RX)
    assert path.read_bytes() == original
    assert list(preview.changes) == ["preprocessing.coordinate_transform.rotation_sensor_to_tool"]
    assert "# mounting value" in preview.updated_bytes.decode()
    assert "# Preserve this origin comment." in preview.updated_bytes.decode()
    before, after = yaml.safe_load(original), yaml.safe_load(preview.updated_bytes)
    before["preprocessing"]["coordinate_transform"]["rotation_sensor_to_tool"] = RX.tolist()
    assert before == after
    assert original.split(b"rotation_sensor_to_tool:")[0] == preview.updated_bytes.split(b"rotation_sensor_to_tool:")[0]
    assert original.split(b"# mounting value")[1] == preview.updated_bytes.split(b"# mounting value")[1]
    assert "rotation_sensor_to_tool" in preview.diff
    assert len(preview.original_sha256) == 64


def test_exact_backup_atomic_commit_preserves_permissions_and_current_pose(tmp_path):
    path = write_config(tmp_path)
    path.chmod(0o640)
    original = path.read_bytes()
    preview = preview_config_update(path, RX)
    backup = commit_config_update(preview, tmp_path / "mount_data" / "backups")
    assert backup.read_bytes() == original
    assert path.read_bytes() == preview.updated_bytes
    assert path.stat().st_mode & 0o777 == 0o640
    saved = yaml.safe_load(path.read_bytes())
    processor = WrenchPreprocessor.from_config(saved["preprocessing"], tool_orientation=[0., 0., np.pi / 2])
    np.testing.assert_allclose(processor.rotation_sensor_to_output, RZ @ RX, atol=1e-14)
    # Existing gravity config is still applied exactly once by the scanner.
    output = processor.process(Wrench.from_sequence([2., 4., 6., 0., 0., 0.]))
    np.testing.assert_allclose(output.force, RZ @ RX @ np.array([1., 2., 3.]), atol=1e-14)
    processor.set_tool_orientation([0., 0., 0.])
    np.testing.assert_allclose(processor.rotation_sensor_to_output, RX, atol=1e-14)


@pytest.mark.parametrize("legacy", ["TODO", [[-1, 0, 0], [0, -1, 0], [0, 0, -1]],
                                   [[1, 0], [0, 1]], [0, 0, 0]])
def test_invalid_legacy_placeholder_no_longer_blocks_direct_rotation(tmp_path, legacy):
    config = yaml.safe_load(CONFIG_TEXT)
    config["preprocessing"]["coordinate_transform"]["rotation_sensor_to_base"] = legacy
    path = write_config(tmp_path, yaml.safe_dump(config, sort_keys=False))
    before = path.read_bytes()
    preview = preview_config_update(path, RX)
    expected = copy.deepcopy(config)
    expected["preprocessing"]["coordinate_transform"].update(
        rotation_sensor_to_tool=RX.tolist(), rotation_sensor_to_base=None)
    assert yaml.safe_load(preview.updated_bytes) == expected
    assert len(preview.changes) == 2
    assert any("placeholder" in note for note in preview.notes)
    backup = commit_config_update(preview, tmp_path / "backups")
    assert backup.read_bytes() == before
    processor = WrenchPreprocessor.from_config(expected["preprocessing"], tool_orientation=[0., 0., 0.])
    np.testing.assert_allclose(processor.rotation_sensor_to_output, RX)


def test_valid_legacy_rotation_and_reference_remain_lower_priority(tmp_path):
    config = yaml.safe_load(CONFIG_TEXT)
    transform = config["preprocessing"]["coordinate_transform"]
    transform["rotation_sensor_to_base"] = RZ.tolist()
    transform["reference_tool_orientation"] = [.4, .5, .6]
    path = write_config(tmp_path, yaml.safe_dump(config, sort_keys=False))
    preview = preview_config_update(path, RX)
    updated = yaml.safe_load(preview.updated_bytes)
    updated_transform = updated["preprocessing"]["coordinate_transform"]
    assert updated_transform["rotation_sensor_to_base"] == RZ.tolist()
    assert updated_transform["reference_tool_orientation"] == [.4, .5, .6]
    assert len(preview.changes) == 1


def test_missing_direct_rotation_added_without_changing_any_other_value(tmp_path):
    path = write_config(tmp_path, CONFIG_TEXT.replace("    rotation_sensor_to_tool: null # mounting value\n", ""))
    preview = preview_config_update(path, RX)
    expected = yaml.safe_load(path.read_bytes())
    expected["preprocessing"]["coordinate_transform"]["rotation_sensor_to_tool"] = RX.tolist()
    assert yaml.safe_load(preview.updated_bytes) == expected
    assert b"# Preserve this origin comment." in preview.updated_bytes


def test_existing_multiline_rotation_preserves_following_comments(tmp_path):
    text = CONFIG_TEXT.replace("null # mounting value", "\n      - [1, 0, 0]\n      - [0, 1, 0]\n      - [0, 0, 1] # mounting value\n    # retained after matrix")
    path = write_config(tmp_path, text)
    preview = preview_config_update(path, RX)
    assert b"# mounting value\n    # retained after matrix" in preview.updated_bytes
    assert yaml.safe_load(preview.updated_bytes)["preprocessing"]["coordinate_transform"]["rotation_sensor_to_tool"] == RX.tolist()


def test_preview_supports_existing_crlf_without_reformatting(tmp_path):
    path = write_config(tmp_path, CONFIG_TEXT.replace("\n", "\r\n"))
    preview = preview_config_update(path, RX)
    assert preview.updated_bytes.count(b"\r\n") == path.read_bytes().count(b"\r\n")


def test_reflection_cannot_be_saved_as_mount_rotation(tmp_path):
    path = write_config(tmp_path)
    with pytest.raises(ValueError, match="proper rotation"):
        preview_config_update(path, -RX)
    assert path.read_text() == CONFIG_TEXT


def test_stale_preview_refuses_to_overwrite_new_user_changes(tmp_path):
    path = write_config(tmp_path)
    preview = preview_config_update(path, RX)
    path.write_text(CONFIG_TEXT + "# operator changed configuration\n")
    with pytest.raises(RuntimeError, match="changed since preview"):
        commit_config_update(preview, tmp_path / "backups")
    assert path.read_text().endswith("# operator changed configuration\n")
    assert not (tmp_path / "backups").exists()


def test_second_write_from_same_preview_rejected(tmp_path):
    path = write_config(tmp_path)
    preview = preview_config_update(path, RX)
    backup = commit_config_update(preview, tmp_path / "backups")
    with pytest.raises(RuntimeError, match="changed since preview"):
        commit_config_update(preview, tmp_path / "backups")
    assert list((tmp_path / "backups").iterdir()) == [backup]


def test_replace_failure_retains_original_and_exact_backup(tmp_path, monkeypatch):
    from tools.sensor_mount_calibration import storage
    path = write_config(tmp_path)
    original = path.read_bytes()
    preview = preview_config_update(path, RX)

    def fail_replace(source, target):
        raise OSError("injected atomic replacement failure")

    monkeypatch.setattr(storage.os, "replace", fail_replace)
    with pytest.raises(OSError, match="replacement failure"):
        commit_config_update(preview, tmp_path / "backups")
    assert path.read_bytes() == original
    backups = list((tmp_path / "backups").iterdir())
    assert len(backups) == 1 and backups[0].read_bytes() == original
    assert not list(tmp_path.glob(".config.yaml.mount-*"))


def test_already_correct_configuration_needs_no_backup_or_write(tmp_path):
    config = yaml.safe_load(CONFIG_TEXT)
    config["preprocessing"]["coordinate_transform"]["rotation_sensor_to_tool"] = RX.tolist()
    path = write_config(tmp_path, yaml.safe_dump(config, sort_keys=False))
    preview = preview_config_update(path, RX)
    assert not preview.diff and not preview.changes
    assert commit_config_update(preview, tmp_path / "backups") is None
    assert not (tmp_path / "backups").exists()


def test_nonzero_unknown_origin_refused_without_editing_tcp_or_origin(tmp_path):
    path = write_config(tmp_path, CONFIG_TEXT.replace("sensor_origin_in_base_m: [0, 0, 0]", "sensor_origin_in_base_m: [0, 0, 0.02]"))
    original = path.read_bytes()
    with pytest.raises(ValueError, match="nonzero sensor_origin_in_base_m"):
        preview_config_update(path, RX)
    assert path.read_bytes() == original


@pytest.mark.parametrize("text", [CONFIG_TEXT + "preprocessing: {}\n",
                                CONFIG_TEXT.replace("rotation_sensor_to_tool: null", "rotation_sensor_to_tool: null\n    rotation_sensor_to_tool: null"),
                                CONFIG_TEXT + "other: &a [1, 2]\nanother: *a\n"])
def test_ambiguous_yaml_refused(tmp_path, text):
    path = write_config(tmp_path, text)
    with pytest.raises(ValueError, match="duplicate|aliases"):
        preview_config_update(path, RX)
    assert path.read_text() == text


def test_raw_records_keep_actual_pose_raw_force_and_timing_separate_from_result(tmp_path):
    metadata = {"gravity_base": [0, 0, -1], "base_level_confirmed": True}
    store = RunStore.create(tmp_path / "sensor_mount_calibration_data", metadata)
    raw = dict(pose_id=4, split="validation", actual_tcp_pose=[.1, .2, .3, .4, .5, .6],
               raw_wrench=[2., -3., 7., .1, .2, .3], host_monotonic=42.1,
               utc_time="2026-10-03T01:02:03+00:00", robot_timestamp=123.5)
    store.append_raw(raw)
    result = dict(rotation_sensor_to_tool=RX, force_polarity=-1,
                  bias_sensor_n=[1., 2., 3.], weight_n=9.8)
    store.write_result(result)
    assert json.loads((store.path / "raw.jsonl").read_text()) == raw
    saved = json.loads((store.path / "result.json").read_text())
    assert saved["bias_sensor_n"] == [1., 2., 3.]
    assert saved["force_polarity"] == -1
    assert json.loads((store.path / "metadata.json").read_text()) == metadata
    assert sorted(p.name for p in store.path.iterdir()) == ["metadata.json", "raw.jsonl", "result.json"]
    second = RunStore.create(store.path.parent, metadata)
    assert second.path != store.path


def test_malformed_raw_record_not_appended(tmp_path):
    store = RunStore.create(tmp_path, {})
    raw = dict(pose_id=0, split="fit", actual_tcp_pose=[0] * 6,
               raw_wrench=[0, 0, float("nan"), 0, 0, 0], host_monotonic=1.,
               utc_time="2026-10-03T00:00:00Z", robot_timestamp=2.)
    with pytest.raises(ValueError, match="six finite"):
        store.append_raw(raw)
    assert (store.path / "raw.jsonl").read_bytes() == b""


def test_preflight_raw_capture_and_stage_metadata_keep_initial_provenance(tmp_path):
    initial = {"configuration_sha256": "original-config", "plan": [{"pose_id": 0}],
               "status": "created"}
    store = RunStore.create(tmp_path, initial)
    raw = dict(pose_id=-1, split="preflight", actual_tcp_pose=[0.] * 6,
               raw_wrench=[1., 2., 3., .1, .2, .3], host_monotonic=1.,
               utc_time="2026-10-03T00:00:00Z", robot_timestamp=2.)
    store.append_raw(raw)
    store.write_metadata({"preflight": {"checked_points": 600}})
    store.write_metadata({"status": "complete"})
    assert json.loads((store.path / "raw.jsonl").read_text()) == raw
    assert json.loads((store.path / "metadata.json").read_text()) == {
        **initial, "preflight": {"checked_points": 600}, "status": "complete"}
