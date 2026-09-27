import ast
from pathlib import Path
import pytest
import run_project
from experiment_logging.paths import PROJECT_ROOT,create_run


def test_all_twenty_steps_keep_semantics():
    assert set(run_project.STEPS)=={str(n) for n in range(1,21)}
    assert '--execute' in run_project.STEPS['12'][1] and 'main.py' in run_project.STEPS['12'][1]
    assert '--execute' in run_project.STEPS['18'][1] and 'run_continuous_tracking.py' in run_project.STEPS['18'][1]
    assert 'return_to_reset.py' in run_project.STEPS['11'][1]
    for _,command in run_project.STEPS.values():
        for arg in command:
            if arg.endswith('.py'): assert (PROJECT_ROOT/arg).is_file()


def test_replay_uses_explicit_metadata(tmp_path):
    path=create_run('real','continuous',PROJECT_ROOT/'config.yaml',data_root=tmp_path)
    assert run_project.describe_run(path)==('连续','真机')
    (path/'metadata.json').unlink()
    with pytest.raises(FileNotFoundError): run_project.describe_run(path)


def test_wrappers_are_thin():
    for name in ('main.py','run_continuous_tracking.py','return_to_reset.py','return_to_start.py'):
        assert len((PROJECT_ROOT/name).read_text().splitlines())<10


def test_only_owner_constructs_control():
    for p in PROJECT_ROOT.rglob('*.py'):
        if 'tests' in p.parts:continue
        text=p.read_text()
        if 'RTDEControlInterface' in text:
            assert p==PROJECT_ROOT/'robot/rtde_controller.py'
