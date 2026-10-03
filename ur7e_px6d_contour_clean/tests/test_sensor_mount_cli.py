"""Offline CLI workflow: real solver/storage/config integration, injected I/O."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from tools.sensor_mount_calibration import calibrate as cli
from tools.sensor_mount_calibration import hardware as hardware_module
from tools.sensor_mount_calibration.plan import make_plan
from tools.sensor_mount_calibration.solver import rotation_from_rotvec


START = [.4, -.2, .3, 0., np.pi, 0.]
MOUNT = rotation_from_rotvec([.7, -1.1, .4])


def synthetic_records(limits=None):
    """Nonzero fixed bias, unknown weight, arbitrary mount and independent poses."""
    rng = np.random.default_rng(709)
    records = []
    for point in make_plan(START, limits):
        if not point['acquire']:
            continue
        gravity = rotation_from_rotvec(point['target'][3:]).T @ [0., 0., -1.]
        force = np.array([1.2, -.8, 2.1]) - 7.3 * MOUNT.T @ gravity
        for _ in range(20):
            stamp = 100. + len(records) * .02
            records.append(dict(pose_id=point['pose_id'], split=point['split'],
                actual_tcp_pose=point['target'], raw_wrench=np.r_[
                    force + rng.normal(0., .003, 3), rng.normal(0., .0005, 3)].tolist(),
                host_monotonic=stamp, robot_timestamp=stamp,
                utc_time='2026-10-03T00:00:00Z'))
    return records


@pytest.fixture
def workflow(tmp_path, monkeypatch):
    path = tmp_path / 'config.yaml'
    original = (Path(__file__).resolve().parents[1] / 'config.yaml').read_bytes()
    path.write_bytes(original)
    harness = SimpleNamespace(path=path, original=original, root=tmp_path / 'mount_data',
                              events=[], answers=['', ''], records=synthetic_records(),
                              hardware=None, enter_started=False, acquired=None,
                              enabled=True, before_acquisition=None, prompts=[])

    class Keyboard:
        enabled = True
        on_wait = None

        def __enter__(self):
            self.enabled = harness.enabled
            return self

        def __exit__(self, *_args):
            harness.events.append('keyboard_closed')

        def poll(self):
            return None

        def read_line(self, prompt):
            harness.prompts.append(prompt)
            number = len(harness.prompts)
            if number == 1:
                assert harness.hardware.connected
                assert not harness.hardware.active
                assert not harness.enter_started
            else:
                assert harness.hardware.closed
                assert harness.acquired is not None
            if self.on_wait:
                self.on_wait()
            answer = harness.answers.pop(0)
            harness.events.append(f'confirmation_{number}')
            if isinstance(answer, BaseException):
                raise answer
            if number == 1 and answer == '':
                harness.enter_started = True
            return answer

    class Hardware:
        def __init__(self, config, limits):
            harness.hardware = self
            self.connected = self.active = self.closed = False
            self.limits = limits
            self.metadata = {'active_tcp_offset': config['tcp']['offset'],
                             'sdk_contract': 'offline test double'}

        def connect(self):
            self.connected = True
            harness.events.append('connected')

        def activate(self):
            assert harness.enter_started, 'activation must be preceded by first Enter'
            self.active = True
            harness.events.append('activated')

        def stop(self):
            harness.events.append('stopped')

        def close(self):
            self.closed = True
            harness.events.append('closed')

    class Session:
        def __init__(self, hardware, store, keyboard, limits):
            self.hardware, self.store = hardware, store
            self.origin = None
            self.stamp = 90.

        def observe(self, *, poll_keyboard=True):
            self.stamp += .02
            record = dict(pose_id=-1, split='motion', actual_tcp_pose=list(START),
                          raw_wrench=[1., 2., 3., 0., 0., 0.], host_monotonic=self.stamp,
                          robot_timestamp=self.stamp, utc_time='2026-10-03T00:00:00Z')
            self.store.append_raw(record)
            harness.events.append('observed')
            return record

        def run(self, start, plan):
            assert start == START and plan == make_plan(START, self.hardware.limits)
            self.hardware.activate()
            self.store.write_metadata({'preflight': {'sample_count': 600}})
            if harness.before_acquisition:
                harness.before_acquisition()
            for record in harness.records:
                self.store.append_raw(record)
            harness.acquired = deepcopy(harness.records)
            harness.events.append('acquired')
            return harness.acquired

    monkeypatch.setattr(cli, 'OperatorKeyboard', Keyboard)
    monkeypatch.setattr(cli, 'Session', Session)
    monkeypatch.setattr(hardware_module, 'Hardware', Hardware)
    harness.run = lambda *extra: cli.main(['--execute', '--config', str(path), '--data-dir', str(harness.root), *extra])
    return harness


def one_run(harness):
    runs = list(harness.root.iterdir())
    assert len(runs) == 1
    return runs[0]


@pytest.mark.parametrize('cancel_key', ['q', '\x1b', 'START'])
def test_first_confirmation_cannot_activate_without_enter(workflow, cancel_key):
    workflow.answers = [cancel_key]
    assert workflow.run() == 2
    assert 'activated' not in workflow.events
    assert workflow.acquired is None and workflow.hardware.closed
    assert workflow.path.read_bytes() == workflow.original
    run = one_run(workflow)
    assert not (run / 'result.json').exists()
    assert 'cancelled before motion' in json.loads((run / 'metadata.json').read_text())['reason']


def test_execute_requires_interactive_terminal_before_device_construction(workflow):
    workflow.enabled = False
    assert workflow.run() == 2
    assert workflow.hardware is None
    assert not workflow.root.exists()
    assert workflow.path.read_bytes() == workflow.original


def test_both_enters_save_only_rotation_after_complete_validation_and_exact_backup(workflow, capsys):
    assert workflow.run() == 0
    assert len(workflow.prompts) == 2
    assert workflow.events.index('confirmation_1') < workflow.events.index('activated')
    assert workflow.events.index('acquired') < workflow.events.index('closed') < workflow.events.index('confirmation_2')
    run = one_run(workflow)
    result = json.loads((run / 'result.json').read_text())
    metadata = json.loads((run / 'metadata.json').read_text())
    assert result['configuration_saved'] is True
    assert result['force_sign'] == -1
    np.testing.assert_allclose(result['rotation_sensor_to_tool'], MOUNT, atol=.002)
    assert result['validation_rmse_N'] < .01
    assert metadata['config_sha256'] and metadata['plan'] and metadata['preflight']
    assert metadata['status'] == 'completed' and metadata['configuration_saved'] is True
    assert Path(result['configuration_backup']).read_bytes() == workflow.original
    assert len(list((run / 'config_backup').iterdir())) == 1
    before, after = yaml.safe_load(workflow.original), yaml.safe_load(workflow.path.read_bytes())
    before['preprocessing']['coordinate_transform']['rotation_sensor_to_tool'] = result['rotation_sensor_to_tool']
    assert before == after
    raw = [json.loads(line) for line in (run / 'raw.jsonl').read_text().splitlines()]
    assert [r for r in raw if r['split'] in ('fit', 'validation')] == workflow.acquired
    output = capsys.readouterr().out
    stages = ['正在连接 PX6D', '设备已连接', '数据已落盘', '拟合及独立姿态验证通过',
              '正在备份原配置', '配置已更新']
    assert [output.index(stage) for stage in stages] == sorted(output.index(stage) for stage in stages)


@pytest.mark.parametrize('cancel_key', ['q', '\x1b'])
def test_final_confirmation_cancel_preserves_config_and_validated_result(workflow, cancel_key):
    workflow.answers = ['', cancel_key]
    assert workflow.run() == 0
    assert 'activated' in workflow.events and len(workflow.prompts) == 2
    assert workflow.path.read_bytes() == workflow.original
    run = one_run(workflow)
    result = json.loads((run / 'result.json').read_text())
    assert result['configuration_saved'] is False
    assert result['validation_rmse_N'] < .01
    assert result['proposed_config_changes']
    assert not (run / 'config_backup').exists()


def test_solver_failure_archives_reason_without_result_or_config_update(workflow):
    for record in workflow.records:
        record['raw_wrench'][:3] = [1., 2., 3.]
    assert workflow.run() == 2
    assert workflow.path.read_bytes() == workflow.original
    assert len(workflow.prompts) == 1
    assert workflow.hardware.closed
    run = one_run(workflow)
    metadata = json.loads((run / 'metadata.json').read_text())
    assert 'insufficient gravity force signal' in metadata['reason']
    assert metadata['diagnostics'] and metadata['preflight'] and metadata['config_sha256']
    assert (run / 'raw.jsonl').exists()
    assert not (run / 'result.json').exists() and not (run / 'config_backup').exists()


def test_config_changed_during_acquisition_refuses_save_without_overwriting_user_edit(workflow):
    changed = workflow.original + b'\n# operator edit while samples were collected\n'
    workflow.before_acquisition = lambda: workflow.path.write_bytes(changed)
    assert workflow.run() == 2
    assert workflow.path.read_bytes() == changed
    assert len(workflow.prompts) == 1
    run = one_run(workflow)
    assert not (run / 'result.json').exists() and not (run / 'config_backup').exists()
    assert '配置在采集期间发生修改' in json.loads((run / 'metadata.json').read_text())['reason']


def test_final_ctrl_c_leaves_validated_result_unsaved_and_reports_primary_reason(workflow):
    workflow.answers = ['', KeyboardInterrupt()]
    assert workflow.run() == 130
    assert workflow.path.read_bytes() == workflow.original
    run = one_run(workflow)
    assert json.loads((run / 'result.json').read_text())['configuration_saved'] is False
    assert 'KeyboardInterrupt' in json.loads((run / 'metadata.json').read_text())['reason']
    assert workflow.hardware.closed and not (run / 'config_backup').exists()


@pytest.mark.parametrize('primary_type, expected_code', [(TimeoutError, 2), (KeyboardInterrupt, 130)])
@pytest.mark.parametrize('failure_logging_fails', [False, True])
def test_primary_sensor_or_ctrl_c_failure_survives_stop_close_and_logging_errors(
        workflow, monkeypatch, capsys, primary_type, expected_code, failure_logging_fails):
    primary = primary_type('primary acquisition failure')
    write_metadata = cli.RunStore.write_metadata

    def controlled_metadata(store, metadata):
        if failure_logging_fails and metadata.get('status') == 'rejected_or_stopped':
            raise OSError('secondary diagnostic write failure')
        return write_metadata(store, metadata)

    monkeypatch.setattr(cli.RunStore, 'write_metadata', controlled_metadata)

    def fail_during_acquisition():
        def fail_stop():
            workflow.events.append('stop_failed')
            raise OSError('secondary stop request failure')

        def fail_close():
            workflow.events.append('close_failed')
            raise OSError('secondary device close failure')

        workflow.hardware.stop = fail_stop
        workflow.hardware.close = fail_close
        raise primary

    workflow.before_acquisition = fail_during_acquisition
    assert workflow.run() == expected_code
    assert 'activated' in workflow.events
    assert workflow.events.index('stop_failed') < workflow.events.index('close_failed')
    assert workflow.path.read_bytes() == workflow.original
    run = one_run(workflow)
    assert not (run / 'result.json').exists() and not (run / 'config_backup').exists()
    notes = '\n'.join(primary.__notes__)
    assert 'secondary stop request failure' in notes
    assert 'secondary device close failure' in notes
    errors = capsys.readouterr().err
    assert 'secondary stop request failure' in errors and 'secondary device close failure' in errors
    if failure_logging_fails:
        assert 'secondary diagnostic write failure' in notes and 'secondary diagnostic write failure' in errors
    else:
        metadata = json.loads((run / 'metadata.json').read_text())
        assert metadata['reason'] == f'{primary_type.__name__}: primary acquisition failure'
        assert metadata['configuration_saved'] is False
        assert 'secondary stop request failure' in '\n'.join(metadata['cleanup_errors'])
    if primary_type is TimeoutError:
        assert 'TimeoutError: primary acquisition failure' in errors
    else:
        assert '操作已取消。' in errors


def test_result_write_failure_after_commit_reports_saved_configuration_accurately(workflow, monkeypatch, capsys):
    write_result = cli.RunStore.write_result

    def fail_after_commit(store, result):
        if result.get('configuration_saved'):
            raise OSError('injected result archive failure after config commit')
        return write_result(store, result)

    monkeypatch.setattr(cli.RunStore, 'write_result', fail_after_commit)
    assert workflow.run() == 2
    assert len(workflow.prompts) == 2
    assert workflow.path.read_bytes() != workflow.original
    run = one_run(workflow)
    metadata = json.loads((run / 'metadata.json').read_text())
    assert metadata['configuration_saved'] is True
    assert 'result archive failure after config commit' in metadata['reason']
    before, after = yaml.safe_load(workflow.original), yaml.safe_load(workflow.path.read_bytes())
    archived = json.loads((run / 'result.json').read_text())
    before['preprocessing']['coordinate_transform']['rotation_sensor_to_tool'] = archived['rotation_sensor_to_tool']
    assert before == after
    backup, = (run / 'config_backup').iterdir()
    assert backup.read_bytes() == workflow.original
    errors = capsys.readouterr().err
    assert '安装配置已写入，后续步骤失败' in errors
    assert '未更新安装配置' not in errors
    assert workflow.hardware.closed


def test_durable_log_drain_precedes_solver_and_configuration_confirmation(workflow, monkeypatch):
    finish, fit = cli.RunStore.finish_logging, cli.calibrate

    def checked_finish(store, timeout_sec=5.):
        assert workflow.hardware.closed, 'robot must close before any blocking disk drain'
        finish(store, timeout_sec)
        assert store.logging_diagnostics['durable']
        workflow.events.append('durable_logs')

    def checked_fit(records, *args):
        assert 'durable_logs' in workflow.events
        return fit(records, *args)

    monkeypatch.setattr(cli.RunStore, 'finish_logging', checked_finish)
    monkeypatch.setattr(cli, 'calibrate', checked_fit)
    assert workflow.run() == 0
    assert workflow.events.index('closed') < workflow.events.index('durable_logs')
    assert workflow.events.index('durable_logs') < workflow.events.index('confirmation_2')


def test_log_finalization_failure_blocks_solver_result_and_config_save(workflow, monkeypatch):
    from tools.sensor_mount_calibration.storage import StorageError
    finish = cli.RunStore.finish_logging

    def failed_finish(store, timeout_sec=5.):
        assert workflow.hardware.closed
        finish(store, timeout_sec)
        raise StorageError('injected final durability failure')

    def forbidden_fit(_records, *args):
        pytest.fail('failed durability must not reach the solver or config update')

    monkeypatch.setattr(cli.RunStore, 'finish_logging', failed_finish)
    monkeypatch.setattr(cli, 'calibrate', forbidden_fit)
    assert workflow.run() == 2
    assert len(workflow.prompts) == 1
    assert workflow.path.read_bytes() == workflow.original
    run = one_run(workflow)
    assert not (run / 'result.json').exists() and not (run / 'config_backup').exists()
    metadata = json.loads((run / 'metadata.json').read_text())
    assert 'final durability failure' in metadata['reason']
    assert metadata['configuration_saved'] is False


@pytest.mark.parametrize('accepted', [True, False])
def test_offline_replay_uses_real_solver_without_config_write_or_devices(tmp_path, monkeypatch, accepted):
    path = tmp_path / 'config.yaml'
    original = (Path(__file__).resolve().parents[1] / 'config.yaml').read_bytes()
    path.write_bytes(original)
    records = synthetic_records()
    if not accepted:
        records = [record for record in records if record['pose_id'] == 0]
    source = tmp_path / 'source.jsonl'
    source.write_text(''.join(json.dumps(record) + '\n' for record in records))

    def forbidden(*_args, **_kwargs):
        raise AssertionError('offline replay must not construct devices or request Enter')

    monkeypatch.setattr(hardware_module, 'Hardware', forbidden)
    monkeypatch.setattr(cli, 'OperatorKeyboard', forbidden)
    root = tmp_path / 'replay_data'
    code = cli.main(['--offline', str(source), '--config', str(path), '--data-dir', str(root)])
    assert code == (0 if accepted else 2)
    assert path.read_bytes() == original
    run, = root.iterdir()
    assert [json.loads(line) for line in (run / 'raw.jsonl').read_text().splitlines()] == records
    metadata = json.loads((run / 'metadata.json').read_text())
    assert metadata['mode'] == 'offline_replay' and metadata['source_raw'] == str(source)
    assert (run / 'result.json').exists() is accepted
    assert not (run / 'config_backup').exists()
    if accepted:
        assert json.loads((run / 'result.json').read_text())['configuration_saved'] is False
    else:
        assert metadata['status'] == 'rejected' and metadata['reason']


@pytest.mark.parametrize('drift_phase', ['preflight', 'reference'])
def test_offline_replay_cannot_bypass_recorded_drift_even_when_fit_is_good(tmp_path, monkeypatch, drift_phase):
    path = tmp_path / 'config.yaml'
    original = (Path(__file__).resolve().parents[1] / 'config.yaml').read_bytes()
    path.write_bytes(original)
    records = synthetic_records()
    center = deepcopy(records[:20])
    if drift_phase == 'reference':
        extra = center
        for index, record in enumerate(extra):
            record.update(split='reference', pose_id=1002,
                          host_monotonic=200.+index*.02, robot_timestamp=200.+index*.02)
            record['raw_wrench'][2] += .2
        records.extend(extra)
    else:
        extra = []
        for index in range(301):
            record = deepcopy(center[index % 20])
            record.update(split='preflight', pose_id=-1,
                          host_monotonic=20.+index*.1, robot_timestamp=20.+index*.1)
            record['raw_wrench'][2] += .003*index*.1
            extra.append(record)
        records = extra + records
    source = tmp_path / 'drift.jsonl'
    source.write_text(''.join(json.dumps(record)+'\n' for record in records))

    def forbidden(*args, **kwargs):
        pytest.fail('recorded drift must be rejected before fitting or connecting devices')

    monkeypatch.setattr(cli, 'calibrate', forbidden)
    monkeypatch.setattr(hardware_module, 'Hardware', forbidden)
    monkeypatch.setattr(cli, 'OperatorKeyboard', forbidden)
    root = tmp_path / 'replay_data'
    assert cli.main(['--offline', str(source), '--config', str(path), '--data-dir', str(root)]) == 2
    run, = root.iterdir()
    metadata = json.loads((run / 'metadata.json').read_text())
    assert metadata['status'] == 'rejected' and '漂移' in metadata['reason']
    assert metadata['diagnostics']['recorded_stability']
    assert metadata['diagnostics']['failure_analysis']['analysis_policy']['diagnostic_only']
    assert not (run / 'result.json').exists() and not (run / 'config_backup').exists()
    assert path.read_bytes() == original


def test_failure_analysis_runs_after_close_and_durable_logs_then_reports_evidence(workflow, monkeypatch, capsys):
    from tools.sensor_mount_calibration.solver import CalibrationError

    primary = CalibrationError('同姿态原始力/矩漂移', dict(force_delta_sensor_N=[.02, -.04, .237],
                               position_delta_m=.000013, orientation_delta_deg=.0011))
    finish = cli.RunStore.finish_logging

    def finalized(store, timeout_sec=5.):
        assert workflow.hardware.closed
        finish(store, timeout_sec)
        assert store.logging_diagnostics['durable']
        workflow.events.append('durable_logs')

    def failed_fit(records, *args):
        raise primary

    def analyzed(records, diagnostics):
        assert workflow.hardware.closed
        assert 'durable_logs' in workflow.events
        assert [r for r in records if r['split'] in ('fit', 'validation')] == workflow.records
        assert diagnostics is primary.diagnostics
        workflow.events.append('analysis')
        return dict(classification='abrupt_force_change', summary='原始力突变候选',
                    change_candidates=[dict(utc_time='2026-10-03T09:24:30.350000+00:00',
                                            force_delta_sensor_N=[-.02, -.05, .27],
                                            torque_delta_sensor_Nm=[0., -.006, 0.])])

    monkeypatch.setattr(cli.RunStore, 'finish_logging', finalized)
    monkeypatch.setattr(cli, 'calibrate', failed_fit)
    monkeypatch.setattr(cli, 'analyze_failure_records', analyzed)
    assert workflow.run() == 2
    assert workflow.events.index('closed') < workflow.events.index('durable_logs') < workflow.events.index('analysis')
    metadata = json.loads((one_run(workflow) / 'metadata.json').read_text())
    assert metadata['reason'] == 'CalibrationError: 同姿态原始力/矩漂移'
    assert metadata['diagnostics']['failure_analysis']['classification'] == 'abrupt_force_change'
    assert len(workflow.prompts) == 1 and workflow.path.read_bytes() == workflow.original
    output = capsys.readouterr().out
    assert '突变候选时刻' in output and 'ΔF=' in output and 'ΔM=' in output
    assert '不能仅归因于预热不足' in output and '仅凭此项检查' not in output


def test_optional_failure_analysis_error_preserves_primary_rejection_and_config(workflow, monkeypatch):
    from tools.sensor_mount_calibration.solver import CalibrationError

    primary = CalibrationError('original fixed-bias model rejection', {'force_signal_N': 1.})

    def failed_fit(records, *args):
        raise primary

    def failed_analysis(*args):
        raise RuntimeError('optional diagnostic unavailable')

    monkeypatch.setattr(cli, 'calibrate', failed_fit)
    monkeypatch.setattr(cli, 'analyze_failure_records', failed_analysis)
    assert workflow.run() == 2
    run = one_run(workflow)
    metadata = json.loads((run / 'metadata.json').read_text())
    assert metadata['reason'] == 'CalibrationError: original fixed-bias model rejection'
    assert metadata['diagnostics'] == {'force_signal_N': 1.}
    assert 'optional diagnostic unavailable' in '\n'.join(primary.__notes__)
    assert not (run / 'result.json').exists() and not (run / 'config_backup').exists()
    assert workflow.path.read_bytes() == workflow.original


def test_no_extra_numerical_analysis_when_device_close_fails(workflow, monkeypatch):
    from tools.sensor_mount_calibration.solver import CalibrationError

    def failed_acquisition():
        def failed_close():
            raise OSError('device close failed')
        workflow.hardware.close = failed_close
        raise CalibrationError('original quality rejection')

    def forbidden_analysis(*args):
        pytest.fail('optional analysis must not run before device shutdown succeeds')

    workflow.before_acquisition = failed_acquisition
    monkeypatch.setattr(cli, 'analyze_failure_records', forbidden_analysis)
    assert workflow.run() == 2
    metadata = json.loads((one_run(workflow)/'metadata.json').read_text())
    assert metadata['reason'] == 'CalibrationError: original quality rejection'
    assert any('device close failed' in reason for reason in metadata['cleanup_errors'])
    assert workflow.path.read_bytes() == workflow.original


def test_wide_preview_displays_new_bounds_without_constructing_devices(tmp_path, monkeypatch, capsys):
    def forbidden(*args, **kwargs):
        pytest.fail('preview must never construct a device or keyboard')
    monkeypatch.setattr(hardware_module, 'Hardware', forbidden)
    monkeypatch.setattr(cli, 'OperatorKeyboard', forbidden)
    output_dir = tmp_path/'unused'
    assert cli.main(['--profile', 'wide', '--data-dir', str(output_dir)]) == 0
    output = capsys.readouterr().out
    assert '20°/45°' in output and '5°' in output and '46°' in output
    assert '197.2 mm' in output and '1.5rad' in output
    assert not output_dir.exists()


def test_wide_execution_saves_profile_and_angle_estimate_with_rotation_only(workflow, capsys):
    from tools.sensor_mount_calibration.profiles import get_profile
    profile = get_profile('wide')
    workflow.records = synthetic_records(profile.motion)
    assert workflow.run('--profile', 'wide') == 0
    run = one_run(workflow)
    result = json.loads((run/'result.json').read_text())
    metadata = json.loads((run/'metadata.json').read_text())
    assert metadata['calibration_profile'] == profile.as_dict()
    assert result['calibration_profile'] == profile.as_dict()
    assert metadata['motion_limits'] == profile.motion.as_dict()
    assert result['estimated_rotation_95_bound_deg'] < 5.
    assert max(p['tilt_deg'] for p in metadata['plan']) == 45.
    assert len(workflow.prompts) == 2
    before, after = yaml.safe_load(workflow.original), yaml.safe_load(workflow.path.read_bytes())
    before['preprocessing']['coordinate_transform']['rotation_sensor_to_tool'] = result['rotation_sensor_to_tool']
    assert before == after
    assert Path(result['configuration_backup']).read_bytes() == workflow.original
    output = capsys.readouterr().out
    assert output.index('20°/45°') < output.index('正在连接 PX6D')
    assert output.index('局部 95% 安装角不确定度估计') < output.index('正在备份原配置')


def test_offline_replay_uses_saved_wide_profile_without_new_required_files(tmp_path, monkeypatch):
    from tools.sensor_mount_calibration.profiles import get_profile
    profile = get_profile('wide')
    source = tmp_path/'source'
    source.mkdir()
    records = synthetic_records(profile.motion)
    raw = source/'raw.jsonl'
    raw.write_text(''.join(json.dumps(r)+'\n' for r in records))
    (source/'metadata.json').write_text(json.dumps({'calibration_profile': profile.as_dict()}))

    def forbidden(*args, **kwargs):
        pytest.fail('offline must not open devices or request confirmation')
    monkeypatch.setattr(hardware_module, 'Hardware', forbidden)
    monkeypatch.setattr(cli, 'OperatorKeyboard', forbidden)
    root = tmp_path/'replay'
    assert cli.main(['--offline', str(raw), '--data-dir', str(root)]) == 0
    run, = root.iterdir()
    result = json.loads((run/'result.json').read_text())
    assert result['calibration_profile'] == profile.as_dict()
    assert result['configuration_saved'] is False
    assert result['estimated_rotation_95_bound_deg'] < 5.


def test_wide_does_not_relabel_small_angle_records_as_large_coverage(tmp_path):
    raw = tmp_path/'raw.jsonl'
    raw.write_text(''.join(json.dumps(r)+'\n' for r in synthetic_records()))
    root = tmp_path/'replay'
    assert cli.main(['--offline', str(raw), '--profile', 'wide', '--data-dir', str(root)]) == 2
    run, = root.iterdir()
    assert not (run/'result.json').exists()
    metadata = json.loads((run/'metadata.json').read_text())
    assert 'coverage' in metadata['reason']
