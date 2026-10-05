"""Run-only tangent selection and unchanged motion/safety behavior, offline."""
from copy import deepcopy
import csv
import json

import numpy as np
import pytest
import yaml

from app import continuous_runtime as runtime
from core.models import RobotState, Wrench
from doubles import Devices, Sensor, Keyboard
from experiment_logging.paths import PROJECT_ROOT
from policy.continuous_tracking import ContinuousTrackingPolicy, State
from test_continuous_runtime import args, prepare


@pytest.mark.parametrize('answer,multiplier', [('', 1.), ('3', 3.), ('2.5', 2.5), ('30', 30.)])
def test_choice_uses_original_base_and_displays_mm_per_second(answer, multiplier):
    messages = []
    selection = runtime.choose_tracking_speed(.001, .030, read_text=lambda _: answer,
                                              emit=lambda message, **unused: messages.append(message))
    assert selection == dict(tracking_base_speed_mps=.001, tracking_speed_multiplier=multiplier,
                             tracking_nominal_speed_mps=.001*multiplier)
    assert '1 mm/s' in messages[-1]
    assert f'{multiplier:g} 倍' in messages[-1]
    assert f'{multiplier:g} mm/s' in messages[-1]
    assert '受力减速' in messages[-1] and '限幅' in messages[-1]
    # A later startup's default is the original base, never the previous choice.
    again = runtime.choose_tracking_speed(.001, .030, read_text=lambda _: '', emit=lambda *a, **k: None)
    assert again['tracking_nominal_speed_mps'] == .001


@pytest.mark.parametrize('invalid', ['0', '-1', 'nan', 'inf', '-inf', '1e309', '31', 'bad', '1e-323'])
def test_invalid_or_over_limit_choice_reprompts(invalid):
    answers, prompts, messages = iter([invalid, '3']), [], []
    def read(prompt):
        prompts.append(prompt)
        return next(answers)
    selected = runtime.choose_tracking_speed(.001, .030, read_text=read,
                                             emit=lambda message, **unused: messages.append(message))
    assert len(prompts) == 2
    assert selected['tracking_speed_multiplier'] == 3.
    assert selected['tracking_nominal_speed_mps'] == .003
    assert '重新输入' in messages[0] or '有限正数' in messages[0]


def sample(policy, now, force):
    wrench = Wrench(force, 0., 0., 0., 0., 0.)
    return policy.update(now, wrench, wrench, RobotState(now, np.zeros(6), np.zeros(6)))


def policies(config, multiplier):
    original, selected = deepcopy(config), deepcopy(config)
    selected['continuous_tracking']['tangential_speed'] *= multiplier
    return ContinuousTrackingPolicy(original), ContinuousTrackingPolicy(selected)


@pytest.mark.parametrize('force', [1.1, 1.9])
def test_multiplier_only_changes_tangent_with_force_slowing_and_same_normal_feedback(config, force):
    base, scaled = policies(config, 3.)
    for i in range(100):
        sample(base, i*.01, 1.5)
        sample(scaled, i*.01, 1.5)
    for i, f in enumerate(np.linspace(1.5, force, 21)[1:], 100):
        before, after = sample(base, i*.01, f), sample(scaled, i*.01, f)
    # Compare after the unchanged acceleration limiter has settled; a larger
    # tangential target can take longer to reach during a force transition.
    for i in range(120, 150):
        before, after = sample(base, i*.01, force), sample(scaled, i*.01, force)
    assert base.state == scaled.state == State.CONTINUOUS_TRACKING
    assert scaled.v_t == pytest.approx(3.*base.v_t)
    assert scaled.v_n == pytest.approx(base.v_n)
    assert abs(scaled.v_n) > 0.
    # Compare actual velocity projections, not a multiplied final vector.
    assert (after.direction_xy*after.speed) @ scaled.contact_direction == pytest.approx(
        (before.direction_xy*before.speed) @ base.contact_direction)
    for key in ('search_speed', 'reacquire_speed', 'normal_speed_limit', 'force_gain', 'command_acceleration'):
        assert scaled.c[key] == base.c[key]


def test_search_and_lost_contact_exploration_speeds_do_not_scale(config):
    base, scaled = policies(config, 3.)
    for i in range(200):
        before, after = sample(base, i*.01, 0.), sample(scaled, i*.01, 0.)
        assert after.speed == pytest.approx(before.speed)
        np.testing.assert_allclose(after.direction_xy, before.direction_xy)
    assert before.speed == pytest.approx(config['continuous_tracking']['search_speed'])
    for i in range(200, 300):
        sample(base, i*.01, 1.5)
        sample(scaled, i*.01, 1.5)
    for i in range(300, 340):
        before, after = sample(base, i*.01, 0.), sample(scaled, i*.01, 0.)
        if base.state == State.LOCAL_REACQUIRE and before.move:
            break
    assert base.state == scaled.state == State.LOCAL_REACQUIRE and before.move
    assert after.speed == pytest.approx(before.speed)
    np.testing.assert_allclose(after.direction_xy, before.direction_xy)


def test_combined_cap_acceleration_limit_and_stop_remain_effective(config):
    _, scaled = policies(config, config['robot']['max_tcp_speed']/config['continuous_tracking']['tangential_speed'])
    previous = np.zeros(2)
    for i in range(500):
        command = sample(scaled, i*.01, 1.1)
        velocity = command.direction_xy*command.speed
        assert command.speed <= config['robot']['max_tcp_speed'] + 1e-12
        assert np.linalg.norm(velocity-previous) <= scaled.c['command_acceleration']*.01 + 1e-12
        previous = velocity
    assert command.speed == pytest.approx(config['robot']['max_tcp_speed'])
    assert abs(scaled.v_n) <= config['continuous_tracking']['normal_speed_limit']
    scaled.request_stop(5., np.zeros(6), 'offline stop')
    assert not sample(scaled, 5.01, 1.1).move


@pytest.mark.parametrize('multiplier', [1., 3.])
def test_real_entry_selects_before_scan_and_records_effective_settings_without_source_write(
        config, monkeypatch, tmp_path, capsys, multiplier):
    source_before = (PROJECT_ROOT/'config.yaml').read_bytes()
    target = prepare(config, monkeypatch)
    loaded_config = runtime.load_config(PROJECT_ROOT/'config.yaml')
    devices = Devices(config, target)
    prompts, observations = [], []
    def read_text(self, prompt):
        prompts.append(prompt)
        return str(multiplier)
    def read_line(self, prompt):
        prompts.append(prompt)
        return ''
    monkeypatch.setattr(Keyboard, 'read_text', read_text)
    monkeypatch.setattr(Keyboard, 'read_line', read_line)
    update = runtime.ContinuousTrackingPolicy.update
    def observe(policy, *a, **k):
        observations.append(policy.c['tangential_speed'])
        return update(policy, *a, **k)
    monkeypatch.setattr(runtime.ContinuousTrackingPolicy, 'update', observe)
    assert runtime.run(args(tmp_path), controller_factory=devices.controller, reader_factory=Sensor) == 0
    choice, = [i for i, prompt in enumerate(prompts) if '贴边扫描速度倍率' in prompt]
    start, = [i for i, prompt in enumerate(prompts) if '沿保存方向开始扫描' in prompt]
    assert choice < start
    nominal = config['continuous_tracking']['tangential_speed']*multiplier
    assert observations and all(value == nominal for value in observations)
    run_dir, = (tmp_path/'real/continuous').glob('run_*')
    snapshot = yaml.safe_load((run_dir/'config_snapshot.yaml').read_text())
    selection = json.loads((run_dir/'tracking_speed_selection.json').read_text())
    assert selection == snapshot['continuous_speed_selection']
    assert selection == dict(tracking_base_speed_mps=.001, tracking_speed_multiplier=multiplier,
                             tracking_nominal_speed_mps=nominal)
    assert snapshot['continuous_tracking']['tangential_speed'] == nominal
    for key in ('search_speed', 'reacquire_speed', 'normal_speed_limit', 'force_gain', 'command_acceleration'):
        assert snapshot['continuous_tracking'][key] == config['continuous_tracking'][key]
    assert snapshot['safe_return'] == loaded_config['safe_return']
    assert devices.owner.config['max_tcp_speed'] == config['robot']['max_tcp_speed']
    assert devices.owner.config['speed_acceleration'] == config['robot']['speed_acceleration']
    assert devices.owner.config['continuous_speed_limits']['CONTINUOUS_TRACKING'] == pytest.approx(
        np.hypot(config['continuous_tracking']['tangential_speed'], config['continuous_tracking']['normal_speed_limit']))
    with (run_dir/'samples.csv').open(newline='') as handle:
        rows = list(csv.DictReader(handle))
    assert rows
    for row in rows:
        if row['current_state'] == 'RETURN_TO_START':
            continue  # Selection occurs above P0 after the initial lift/transit.
        for key, value in selection.items():
            assert float(row[key]) == value
    output = capsys.readouterr().out
    assert '基准速度：1 mm/s' in output and f'所选倍率：{multiplier:g} 倍' in output
    assert f'本次名义贴边速度：{nominal*1000:g} mm/s' in output
    assert (PROJECT_ROOT/'config.yaml').read_bytes() == source_before


def test_cancel_choice_never_starts_scan(config, monkeypatch, tmp_path):
    target = prepare(config, monkeypatch)
    devices = Devices(config, target)
    monkeypatch.setattr(Keyboard, 'read_text', lambda *args: 'q')
    def forbidden(*args, **kwargs):
        pytest.fail('cancelled choice cannot reach scan policy')
    monkeypatch.setattr(runtime.ContinuousTrackingPolicy, 'update', forbidden)
    assert runtime.run(args(tmp_path), controller_factory=devices.controller, reader_factory=Sensor) == 0
    assert not any(call[0] == 'speedL' and np.any(call[1]) for call in devices.control.calls)
