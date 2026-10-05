import json
import pytest
from experiment_logging.paths import create_run, run_root, read_metadata, PROJECT_ROOT


def test_beijing_creation_names_and_utc_local_fields_use_same_instant(tmp_path, monkeypatch):
    from datetime import datetime, timezone
    from experiment_logging import paths
    fixed = datetime(2026, 10, 5, 9, 33, 5, 896939, tzinfo=timezone.utc)
    class Clock(datetime):
        @classmethod
        def now(cls, tz=None):
            return fixed.astimezone(tz)
    monkeypatch.setattr(paths, 'datetime', Clock)
    directory = paths.create_run('real', 'continuous', PROJECT_ROOT/'config.yaml', data_root=tmp_path)
    assert directory.name == 'run_2026-10-05_17-33-05_896939'
    metadata = paths.read_metadata(directory)
    assert metadata['timestamp'] == fixed.isoformat()
    assert metadata['timestamp_local'] == '2026-10-05T17:33:05.896939+08:00'
    assert metadata['timezone'] == 'Asia/Shanghai'
    assert metadata['timestamp_meaning'] == 'run_directory_created_at'
    assert datetime.fromisoformat(metadata['timestamp_local']) == fixed
    assert '北京时间 (Asia/Shanghai)' in paths.local_time_text(metadata['timestamp'])


def test_sorting_prefers_metadata_and_accepts_old_and_new_names(tmp_path):
    import json
    from experiment_logging.paths import run_sort_key
    old = tmp_path/'run_20261005_093305_896939'
    new = tmp_path/'run_2026-10-05_17-34-00_000000'
    for path in (old, new):
        path.mkdir()
    assert sorted([new, old], key=run_sort_key) == [old, new]
    (new/'metadata.json').write_text(json.dumps(dict(schema_version=1, mode='real', strategy='continuous',
        timestamp='2026-10-05T08:00:00+00:00', git={}, config_source=None)))
    assert sorted([old, new], key=run_sort_key) == [new, old]
    assert read_metadata(new)['schema_version'] == 1
from experiment_logging.data_logger import ExperimentLogger
from experiment_logging.termination import TerminationRecorder
from simulation.simulator import ContourSimulator, load_simulation_config


@pytest.mark.parametrize('mode,strategy',[('real','continuous'),('real','discrete'),('real','return'),('simulation','continuous'),('simulation','discrete')])
def test_identity_paths_and_metadata(mode,strategy,tmp_path):
    path=create_run(mode,strategy,PROJECT_ROOT/'config.yaml',data_root=tmp_path)
    assert path.parent==tmp_path/mode/strategy
    assert path.name.startswith('run_')
    data=read_metadata(path)
    assert data['mode']==mode and data['strategy']==strategy
    assert set(data['git'])=={'branch','commit','dirty'}
    assert data['config_source']==str(PROJECT_ROOT/'config.yaml')
    assert data['timestamp']


def test_no_header_only_optional_tables(config,tmp_path):
    with ExperimentLogger(tmp_path,config,mode='simulation',strategy='continuous',workspace_logging=False) as logger:
        path=logger.run_dir
        logger.write_summary('STOP','cancelled',0)
    for name in ('policy_waypoints.csv','boundary_points.csv','boundary_recovery_rays.csv'):
        assert not (path/name).exists()


@pytest.mark.parametrize('asynchronous', [False, True])
def test_sample_frame_reaches_csv_and_termination_without_overwriting_measurements(config,tmp_path,asynchronous):
    import csv
    import numpy as np
    from core.models import Wrench, RobotState, PolicyCommand
    from experiment_logging.continuous_writer import ContinuousLogWriter
    config['force_display'] = {'frame': 'Base'}
    logger = ExperimentLogger(tmp_path, config, mode='real', strategy='continuous', workspace_logging=False)
    recorder = TerminationRecorder()
    logger.termination = recorder
    writer = ContinuousLogWriter(logger) if asynchronous else logger
    pose = np.zeros(6)
    raw = Wrench(1., 2., 3., 0., 0., 0.)
    robot = RobotState(1., pose, pose.copy())
    command = PolicyCommand('RETURN_TO_START', False, np.zeros(2), 0., pose, False, False)
    try:
        writer.log_sample(1., raw, raw, robot, command, None, None,
                          extra={'processed_force_frame': 'Sensor'})
        recorder.set_stop_reason(detail='test stopped', source='test')
        assert recorder.record['force_frame'] == 'Sensor'
        writer.flush()
        with (logger.run_dir/'samples.csv').open() as stream:
            row, = csv.DictReader(stream)
        assert row['processed_force_frame'] == 'Sensor'
        assert float(row['dfx']) == 1.
        if not asynchronous:
            with pytest.raises(ValueError, match='overwrite standard'):
                logger.log_sample(2., raw, raw, robot, command, None, None, extra={'dfx': 99.})
    finally:
        writer.close()


def test_failed_startup_has_same_classification(tmp_path):
    recorder=TerminationRecorder(output_root=tmp_path,mode='real',strategy='continuous',config_source=PROJECT_ROOT/'config.yaml')
    recorder.set_stop_reason(detail='bad configuration',source='test')
    path=recorder.flush()
    assert path.parent==tmp_path/'real/continuous'
    assert read_metadata(path)['mode']=='real'


def test_discrete_simulation_output(tmp_path):
    config=load_simulation_config(PROJECT_ROOT/'simulation/scene_square.yaml')
    config['_data_root']=str(tmp_path)
    sim=ContourSimulator(config);sim.run_headless(max_steps=10);path=sim.finalize()
    assert path.parent==tmp_path/'simulation/discrete'
    assert read_metadata(path)['mode']=='simulation'
    assert (path/'simulation_log.csv').exists()
    assert json.loads((path/'summary.json').read_text())['strategy']=='discrete'


def test_preview_and_replay_read_metadata_without_waypoints(config,tmp_path):
    import matplotlib.pyplot as plt
    from simulation.continuous_preview import PreviewRun,ContinuousPreview
    from tools.visualize_continuous_run import read_run
    scene=load_simulation_config(PROJECT_ROOT/'simulation/scene_continuous.yaml')
    model=PreviewRun(config,scene,tmp_path)
    view=ContinuousPreview(model)
    model.start(0.);model.tick(.04,work_budget_sec=1.);view.draw();model.stop()
    path=model.last_run_dir
    assert path.parent==tmp_path/'simulation/continuous'
    assert read_metadata(path)['mode']=='simulation'
    data,_,_=read_run(path)
    assert len(data['time'])>0
    plt.close(view.figure)
