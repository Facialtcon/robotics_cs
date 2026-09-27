from pathlib import Path
import hashlib
import pytest
from config.loader import load_config
from experiment_logging.paths import PROJECT_ROOT
from app.configuration import prepare_real


def test_algorithm_parameters_preserved(config):
    c=config['continuous_tracking']
    assert c['search_speed']==.018 and config['policy']['contact_threshold']==1
    assert c['tangential_speed']==.001 and c['normal_speed_limit']==.0005
    assert c['force_reference']==1.5 and c['reacquire_enabled'] is False
    assert config['policy']['force_rate_limit']==30


def test_saved_calibrations_resolve_inside_clean(config):
    robot,start=prepare_real(config,PROJECT_ROOT/'config.yaml')
    for path in config['continuous_calibration_sources'].values():
        assert Path(path).is_relative_to(PROJECT_ROOT)
    assert len(start)==6 and robot['continuous_require_watchdog']


def test_missing_config_fails(tmp_path):
    path=tmp_path/'config.yaml';path.write_text('robot: {}')
    with pytest.raises(ValueError): load_config(path)
