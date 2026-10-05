"""Offline TCP plots preserve phases and use each saved recovery configuration."""
import csv
from copy import deepcopy

import numpy as np
import pytest
import yaml

from experiment_logging.paths import create_run, PROJECT_ROOT
from tools.visualize_continuous_run import (read_run, recovery_windows, recovery_reference,
    contact_trajectory_figures, render_contact_trajectory)


def trace():
    states = ['TARGET_SEARCH', 'FIRST_CONTACT', 'CONTINUOUS_TRACKING', 'CONTINUOUS_TRACKING',
        'CONTACT_LOST', 'LOCAL_REACQUIRE', 'LOCAL_REACQUIRE', 'CONTINUOUS_TRACKING',
        'CONTINUOUS_TRACKING', 'CONTACT_LOST', 'LOCAL_REACQUIRE', 'LOCAL_REACQUIRE', 'STOP']
    count = len(states)
    data = dict(monotonic_sec=np.arange(count, dtype=float), time=np.arange(count, dtype=float), state=states,
        tcp_x=.65-np.arange(count)*.0005, tcp_y=.14-np.arange(count)*.0001)
    for key in ('dfx', 'dfy', 'fxy', 'force_reference', 'tangent_x', 'tangent_y', 'contact_direction_x', 'contact_direction_y'):
        data[key] = np.ones(count)
    for key in ('reacquire_origin_x', 'reacquire_origin_y', 'reacquire_normal_x', 'reacquire_normal_y',
                'reacquire_tangent_x', 'reacquire_tangent_y'):
        data[key] = np.full(count, np.nan)
    for start, end in ((5, 10), (10, 13)):
        values = (data['tcp_x'][start], data['tcp_y'][start], 0., -1., -1., 0.)
        for key, value in zip(('reacquire_origin_x', 'reacquire_origin_y', 'reacquire_normal_x',
                              'reacquire_normal_y', 'reacquire_tangent_x', 'reacquire_tangent_y'), values):
            data[key][start:end] = value
    events = [dict(timestamp=str(index), state=states[index], event_type=name,
                   x=str(data['tcp_x'][index]), y=str(data['tcp_y'][index])) for index, name in
        ((2, 'FIRST_CONTACT'), (4, 'CONTACT_LOST'), (5, 'LOCAL_REACQUIRE'), (7, 'REACQUIRED'),
         (9, 'CONTACT_LOST'), (10, 'LOCAL_REACQUIRE'), (12, 'BUDGET_STOP'))]
    return data, events


def test_reference_uses_saved_sixty_degrees_and_omits_unknown_directions(config):
    data, events = trace()
    config = deepcopy(config)
    config['continuous_tracking']['reacquire_max_angle_deg'] = 60.
    windows = recovery_windows(data, events)
    assert [w['round'] for w in windows] == [1, 2]
    reference = recovery_reference(data, config, windows[0])
    origin = np.array([data['reacquire_origin_x'][5], data['reacquire_origin_y'][5]])*1000
    expected = origin+2.*np.array([1.-np.cos(np.pi/3), -np.sin(np.pi/3)])
    np.testing.assert_allclose(reference[-1], expected)
    data['reacquire_normal_x'][:] = np.nan
    assert recovery_reference(data, config, windows[0]) is None


def test_post_contact_plots_do_not_join_tracking_across_recovery(config):
    import matplotlib.pyplot as plt
    data, events = trace()
    figures = contact_trajectory_figures(data, config, events)
    try:
        assert [name for name, _ in figures] == ['continuous_contact_trajectory.png',
            'continuous_reacquire_1.png', 'continuous_reacquire_2.png']
        axis = figures[0][1].axes[0]
        assert axis.get_aspect() == 1.
        tracking, = [c for c in axis.collections if c.get_label() == '连续跟踪阶段']
        segments = tracking.get_segments()
        assert len(segments) == 2  # 2->3 and 7->8, never 3->7 or pre-contact search.
        for segment, indices in zip(segments, ((2, 3), (7, 8))):
            np.testing.assert_allclose(segment[:, 0], data['tcp_x'][list(indices)]*1000)
        assert len(axis.patches) == 2  # Boxes identify the two enlarged windows.
        assert '第 1 次' in figures[1][1].axes[0].get_title()
        assert '第 2 次' in figures[2][1].axes[0].get_title()
        assert '物体真实外形' in axis.get_title()
    finally:
        for _, figure in figures:
            plt.close(figure)


@pytest.mark.parametrize('old', [False, True])
def test_old_and_new_named_runs_replay_without_overwriting_logs(tmp_path, config, old):
    data, events = trace()
    directory = create_run('real', 'continuous', PROJECT_ROOT/'config.yaml', data_root=tmp_path)
    if old:
        legacy = directory.with_name('run_20261005_093305_896939')
        directory.rename(legacy)  # Only this synthetic test directory.
        directory = legacy
    saved = deepcopy(config)
    saved['continuous_tracking']['reacquire_max_angle_deg'] = 60. if old else 90.
    (directory/'config_snapshot.yaml').write_text(yaml.safe_dump(saved))
    columns = [key for key in data if key not in ('state', 'time')]+['current_state']
    with (directory/'samples.csv').open('w') as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        for i, state in enumerate(data['state']):
            writer.writerow({**{key: data[key][i] for key in columns[:-1]}, 'current_state': state})
    with (directory/'policy_waypoints.csv').open('w') as handle:
        writer = csv.DictWriter(handle, fieldnames=events[0])
        writer.writeheader();writer.writerows(events)
    before = {p.name: p.read_bytes() for p in directory.iterdir()}
    read_data, read_config, read_events = read_run(directory)
    assert read_config['continuous_tracking']['reacquire_max_angle_deg'] == (60. if old else 90.)
    assert len(recovery_windows(read_data, read_events)) == 2
    artifacts = render_contact_trajectory(directory, output_dir=tmp_path/'figures')
    assert len(artifacts) == 3 and all(p.stat().st_size > 1000 for p in artifacts)
    assert before == {p.name: p.read_bytes() for p in directory.iterdir()}
