"""Replay must never rotate or relabel unknown sensor data as Base vectors."""
import csv
import json

import matplotlib.pyplot as plt
import numpy as np
import pytest
import yaml

from simulation.continuous_view import VectorDisplay, display_label, force_demonstration
from tools.visualize_continuous_run import read_run, make_figure


def make_run(tmp_path, config, *, frame_column=True):
    run = tmp_path/'run'
    run.mkdir()
    (run/'metadata.json').write_text(json.dumps(dict(mode='real', strategy='continuous',
        timestamp='2026-10-03T00:00:00Z', git={}, config_source='offline_fixture')))
    (run/'config_snapshot.yaml').write_text(yaml.safe_dump(config))
    common = dict(tcp_x=.1, tcp_y=.1, dfx=1., dfy=.2, fxy=1.02,
        tangent_x=0., tangent_y=1., contact_direction_x=1., contact_direction_y=0.,
        force_reference=1., direction_valid=1., stop_requested=0.,
        raw_fx=1., raw_fy=.2, tcp_vx=0., tcp_vy=0., tcp_vz=0.,
        command_vx=0., command_vy=0., v_t=0., v_n=0.)
    first = dict(common, monotonic_sec=1., current_state='RETURN_TO_START',
                 force_reference='', tangent_x='', tangent_y='',
                 contact_direction_x='', contact_direction_y='')
    second = dict(common, monotonic_sec=2., current_state='CONTINUOUS_TRACKING')
    if frame_column:
        first['processed_force_frame'] = 'Sensor'
        second['processed_force_frame'] = 'Base'
    with (run/'samples.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(first))
        writer.writeheader()
        writer.writerows([first, second])
    return run


def test_complete_log_keeps_return_policy_values_missing_and_preserves_each_frame(tmp_path, config):
    data, _, _ = read_run(make_run(tmp_path, config))
    assert np.isnan(data['force_reference'][0])
    assert np.isnan(data['contact_direction_x'][0])
    assert data['force_reference'][1] == 1.
    assert data['processed_force_frame'] == ['Sensor', 'Base']


@pytest.mark.parametrize('configured_frame', [None, 'Base', 'sensor_uncalibrated'])
def test_legacy_frame_uses_only_explicit_metadata(tmp_path, config, configured_frame):
    config.pop('force_display', None)
    if configured_frame:
        config['force_display'] = dict(frame=configured_frame)
    data, _, _ = read_run(make_run(tmp_path, config, frame_column=False))
    assert data['processed_force_frame'] == [configured_frame or 'unknown frame']*2


@pytest.mark.parametrize('frame', ['Sensor', 'sensor_uncalibrated', 'unknown frame'])
def test_sensor_components_are_not_drawn_on_base_position_axes(frame):
    fig, axis = plt.subplots()
    try:
        display = VectorDisplay(axis)
        display.draw([0., 0.], [1., 2.], [1., 0.], [0., 0.], [0., 1.], [1., 0.],
                     valid=True, debug=True, simulated=False, force_frame=frame)
        assert not display.arrows['measured'].get_visible()
        assert display.arrows['command'].get_visible()
        measured = display.legend.get_texts()[display.legend_keys.index('measured')].get_text()
        assert frame in measured and 'processed Base' not in measured
        display.draw([0., 0.], [1., 2.], [1., 0.], [0., 0.], [0., 1.], [1., 0.],
                     valid=True, debug=True, simulated=False, force_frame='Base')
        assert display.arrows['measured'].get_visible()
    finally:
        plt.close(fig)


def test_replay_mixed_frame_rows_change_arrows_without_base_label_for_sensor_curve(tmp_path, config):
    data, snapshot, events = read_run(make_run(tmp_path, config))
    fig, update = make_figure(data, snapshot, events, debug=True)
    try:
        assert not fig.continuous_vectors.arrows['measured'].get_visible()
        assert 'mixed frames' in fig.axes[1].get_ylabel()
        assert 'processed Base' not in fig.axes[1].lines[0].get_label()
        assert any('Processed force frame: Sensor' in text.get_text() for text in fig.texts)
        update(1)
        assert fig.continuous_vectors.arrows['measured'].get_visible()
        assert any('Processed force frame: Base' in text.get_text() for text in fig.texts)
    finally:
        plt.close(fig)


def test_return_sensor_row_does_not_inherit_confirmed_base_physical_force():
    metadata = dict(schema_version=1, frame='Base', estimate_method='quasistatic_planar_balance',
        force_convention='environment_on_probe', force_source='processed_wrench',
        physical_sign_confirmed=True, base_frame_confirmed=True, calibration_reference='fixture')
    row = dict(processed_force_frame='Sensor', dfx=1., dfy=2.)
    result = force_demonstration(row, metadata, simulated=False)
    assert result['blue'] is None and result['red'] is None
    row['processed_force_frame'] = 'Base'
    assert force_demonstration(row, metadata, simulated=False)['blue'] is not None


def test_real_force_label_requires_an_explicit_frame():
    assert 'unknown frame' in display_label('measured', simulated=False)
    assert 'processed Base' in display_label('measured', simulated=False, force_frame='Base')
    assert '仿真合成' in display_label('measured', simulated=True)
