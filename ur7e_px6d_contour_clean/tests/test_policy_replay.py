"""Recorded-input replay must separate startup return from the scan policy."""
import csv
import json

import pytest
import yaml

from tools.replay_continuous_policy import replay


def write_run(tmp_path, config, rows):
    (tmp_path/'config_snapshot.yaml').write_text(yaml.safe_dump(config))
    (tmp_path/'termination.json').write_text(json.dumps(dict(reason='STOP_USER_REQUEST', detail='test')))
    with (tmp_path/'samples.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def scan_sample():
    return dict(current_state='TARGET_SEARCH', monotonic_sec='10', tcp_read_end='10',
        raw_fx=0, raw_fy=0, raw_fz=0, raw_tx=0, raw_ty=0, raw_tz=0,
        dfx=0, dfy=0, dfz=0, dtx=0, dty=0, dtz=0,
        tcp_x=0, tcp_y=0, tcp_z=0, tcp_rx=0, tcp_ry=0, tcp_rz=0,
        tcp_vx=0, tcp_vy=0, tcp_vz=0, tcp_vrx=0, tcp_vry=0, tcp_vrz=0,
        runtime_stop_state='STOPPED', command_vx=0, command_vy=0,
        reacquire_path_length=0, reason='')


def test_replay_skips_return_rows_without_inventing_scan_inputs(tmp_path, config):
    scan = scan_sample()
    startup = {key: '' for key in scan}
    startup.update(current_state='RETURN_TO_START', reason='VERTICAL_RETREAT')
    write_run(tmp_path, config, [startup, scan])
    result = replay(tmp_path)
    case = result['cases']['saved_configuration']
    assert result['sample_count'] == 2 and result['skipped_return_samples'] == 1
    assert case['replayed_samples'] == 1
    assert case['first_changed_output'] is None
    assert case['final_state'] == 'TARGET_SEARCH'


def test_replay_does_not_hide_missing_scan_observation(tmp_path, config):
    scan = scan_sample()
    scan['tcp_read_end'] = ''
    write_run(tmp_path, config, [scan])
    with pytest.raises(ValueError):
        replay(tmp_path)


def test_replay_reports_first_recorded_stop_comparison(tmp_path, config):
    scan = scan_sample()
    scan.update(current_state='STOP', reason='local reacquire path budget exhausted',
                reacquire_path_length=.0035)
    write_run(tmp_path, config, [scan])
    comparison = replay(tmp_path)['cases']['saved_configuration']['recorded_stop_comparison']
    assert comparison['csv_line'] == 2
    assert comparison['old_reacquire_path_length_m'] == .0035
    assert comparison['new_reacquire_path_length_m'] == 0.
    assert comparison['new_state'] == 'TARGET_SEARCH'
