"""Real rotation and persistent Kalman math, without robot/sensor hardware."""
import numpy as np
import pytest

from core.models import Wrench
from sensor.force_preprocess import IndependentForceKalman, WrenchPreprocessor, _tool_rotation
from test_force_transform_pose import preprocessing, wrench, RX, RY, RZ


def processor(*, orientation=(0., 0., 0.), kalman=None, **transform):
    config = preprocessing(rotation_sensor_to_tool=np.eye(3).tolist(), **transform)
    if kalman is not None:
        config['kalman'] = kalman
    return WrenchPreprocessor.from_config(config, tool_orientation=orientation)


def test_identity_preserves_all_axes_and_first_actual_measurement():
    p = processor()
    np.testing.assert_array_equal(p.process(wrench([1., -2., 3.])).force, [1., -2., 3.])
    np.testing.assert_array_equal(p.force_base, p.filtered_force_base)


@pytest.mark.parametrize('rotvec,rotation', [([np.pi/2, 0., 0.], RX),
    ([0., np.pi/2, 0.], RY), ([0., 0., np.pi/2], RZ)])
def test_ninety_degree_rotations_map_every_sensor_axis(rotvec, rotation):
    p = processor(orientation=rotvec, kalman={'enabled': False})
    for axis in np.eye(3):
        np.testing.assert_allclose(p.process(wrench(axis)).force, rotation @ axis, atol=1e-14)


def test_mixed_axis_rtde_rotvec_is_axis_angle_not_euler():
    # Pi about (X+Y)/sqrt(2) swaps X/Y and negates Z, not two Euler rotations.
    rotvec = np.array([1., 1., 0.])*np.pi/np.sqrt(2)
    expected = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
    np.testing.assert_allclose(_tool_rotation(rotvec), expected, atol=1e-14)
    p = processor(orientation=rotvec)
    np.testing.assert_allclose(p.process(wrench([1., 2., 3.])).force, [2., 1., -3.], atol=1e-14)


def test_bias_gravity_and_base_baseline_precede_persistent_base_kalman():
    config = preprocessing(rotation_sensor_to_tool=np.eye(3).tolist())
    config['gravity_wrench_sensor'] = [1., 0., 0., 0., 0., 0.]
    config['granular_baseline_output'] = [0., 1., 0., 0., 0., 0.]
    config['kalman'] = dict(process_noise=1., measurement_noise=3., initial_covariance=2.)
    p = WrenchPreprocessor.from_config(config, tool_orientation=[0., 0., 0.])
    p.set_zero_bias([wrench([4., 0., 0.])])
    np.testing.assert_allclose(p.process(wrench([6., 0., 0.])).force, [1., -1., 0.])
    p.set_tool_orientation([0., 0., np.pi/2])
    # Same corrected Sensor +X, now Base +Y. P'=2+1, K=3/(3+3)=1/2.
    result = p.process(wrench([6., 0., 0.]))
    np.testing.assert_allclose(p.force_base, [0., 0., 0.], atol=1e-14)
    np.testing.assert_allclose(result.force, [.5, -.5, 0.], atol=1e-14)
    np.testing.assert_allclose(p.force_kalman.covariance, [1.5]*3)
    np.testing.assert_allclose(p.filtered_force_base, result.force)
    np.testing.assert_allclose(p.raw_sensor_wrench, [6., 0., 0., 0., 0., 0.])


def test_kalman_suppresses_seeded_static_noise_on_all_axes():
    rng = np.random.default_rng(26)
    true_force = np.array([1., -2., 3.])
    measurements = true_force + rng.normal(0., .5, (1000, 3))
    k = IndependentForceKalman()
    filtered = np.array([k.update(row) for row in measurements])
    assert np.all(np.mean((filtered[100:]-true_force)**2, axis=0)
                  < .25*np.mean((measurements[100:]-true_force)**2, axis=0))


@pytest.mark.parametrize('invalid', [None, [1., 2.], [0., np.nan, 0.], [np.inf, 0., 0.]])
def test_missing_or_nonfinite_measurements_cannot_change_state_or_covariance(invalid):
    k = IndependentForceKalman()
    k.update([1., 2., 3.])
    before_x, before_p = k.state.copy(), k.covariance.copy()
    with pytest.raises(ValueError, match='measurement'):
        k.update(invalid)
    np.testing.assert_array_equal(k.state, before_x)
    np.testing.assert_array_equal(k.covariance, before_p)


@pytest.mark.parametrize('invalid', [None, Wrench(np.nan, 0., 0., 0., 0., 0.),
                                    Wrench(0., 0., 0., 0., np.inf, 0.)])
def test_invalid_wrench_does_not_update_kalman_or_publish_fresh_force(invalid):
    p = processor()
    p.process(wrench([1., 2., 3.]))
    before_x, before_p = p.force_kalman.state.copy(), p.force_kalman.covariance.copy()
    with pytest.raises(ValueError):
        p.process(invalid)
    np.testing.assert_array_equal(p.force_kalman.state, before_x)
    np.testing.assert_array_equal(p.force_kalman.covariance, before_p)
    np.testing.assert_array_equal(p.filtered_force_base, before_x)


def test_disabled_kalman_is_base_pass_through_even_with_legacy_alpha():
    p = processor(orientation=[0., 0., np.pi/2], kalman={'enabled': False})
    p.filter_alpha = .99  # compatibility field cannot revive sensor EMA
    for force in ([1., 2., 3.], [4., 5., 6.], [0., 0., 0.]):
        np.testing.assert_allclose(p.process(wrench(force)).force, RZ @ force, atol=1e-14)
    assert p.force_kalman.state is None


def test_legacy_configuration_gets_kalman_defaults_without_alpha():
    config = preprocessing(rotation_sensor_to_tool=np.eye(3).tolist())
    del config['filter_alpha']
    p = WrenchPreprocessor.from_config(config, tool_orientation=[0., 0., 0.])
    assert p.force_kalman.enabled
    np.testing.assert_allclose(p.force_kalman.process_noise, [.01]*3)
    p.process(wrench([1., 0., 0.]))
    assert 1. < p.process(wrench([2., 0., 0.])).fx < 2.


def test_zero_bias_reset_and_invalid_bias_preserve_filter_state():
    p = processor()
    p.process(wrench([1., 2., 3.]))
    with pytest.raises(ValueError, match='bias sample'):
        p.set_zero_bias([Wrench(np.nan, 0., 0., 0., 0., 0.)])
    np.testing.assert_array_equal(p.force_kalman.state, [1., 2., 3.])
    p.set_zero_bias([wrench([1., 2., 3.])])
    assert p.force_kalman.state is None
    np.testing.assert_array_equal(p.process(wrench([1., 2., 3.])).force, [0.]*3)


def test_axis_noise_configuration_is_independent_and_validated():
    k = IndependentForceKalman(dict(process_noise=[1., 2., 3.],
                                   measurement_noise=[3., 2., 1.], initial_covariance=1.))
    k.update([0., 0., 0.])
    np.testing.assert_allclose(k.update([1., 1., 1.]), [2/5, 3/5, 4/5])
    for config in ({'enabled': 'false'}, {'process_noise': 0.}, {'measurement_noise': np.nan},
                   {'initial_covariance': [1., 2.]}):
        with pytest.raises(ValueError):
            IndependentForceKalman(config)


def test_unknown_lever_arm_rotates_torque_at_sensor_origin_only():
    p = processor(orientation=[0., 0., np.pi/2])
    np.testing.assert_allclose(p.process(Wrench(1., 0., 0., 2., 3., 4.)).torque,
                               [-3., 2., 4.], atol=1e-14)
    status = p.force_transform_status
    assert not status['wrench_reference_point_transform_complete']
    assert status['torque_reference_point'] == 'sensor_origin_rotation_only'


def test_measured_tool_lever_arm_uses_unfiltered_force_for_reference_shift():
    p = processor(orientation=[0., 0., np.pi/2], sensor_origin_in_tool_m=[1., 0., 0.])
    p.process(wrench([0., 0., 0.]))
    output = p.process(wrench([0., 2., 0.]))
    assert p.force_transform_status['wrench_reference_point_transform_complete']
    np.testing.assert_allclose(output.torque, [0., 0., 2.], atol=1e-14)
    assert not np.isclose(output.fx, -2.)  # force is Kalman filtered; torque shift is physical raw force


def test_filter_settling_bound_and_disabled_behavior():
    assert .2 < IndependentForceKalman().settling_time(100.) < .3
    assert IndependentForceKalman({'enabled': False}).settling_time(100.) == 0.


def test_return_monitor_uses_pose_after_serial_acquisition(config):
    from types import SimpleNamespace
    from core.models import RobotState
    from safety.safe_return import PX6DForceMonitor
    p = processor(kalman={'enabled': False})
    actual = RobotState(1., np.array([0., 0., 0., 0., 0., np.pi/2]), np.zeros(6))
    order = []
    def read_raw():
        order.append('sensor')
        return wrench([.1, .2, .3])
    def read_state():
        order.append('rtde')
        return actual
    monitor = PX6DForceMonitor(config, SimpleNamespace(read_wrench=read_raw), p)
    assert monitor.sample(read_state=read_state) is actual
    assert order == ['sensor', 'rtde']
    np.testing.assert_allclose(monitor.processed.force, [-.2, .1, .3], atol=1e-14)
    assert monitor.force_frame == 'Base'


@pytest.mark.parametrize('orientation,rotation_base_tool', [([0., 0., 0.], np.eye(3)),
                                                         ([np.pi/2, 0., 0.], RX)])
def test_configured_mount_force_axes_and_positive_reference_point_shift(config, orientation, rotation_base_tool):
    from copy import deepcopy
    settings = deepcopy(config['preprocessing'])
    settings['kalman']['enabled'] = False
    p = WrenchPreprocessor.from_config(settings, tool_orientation=orientation)
    a = .70710678
    expected_rotation = np.array([[a, -a, 0.], [a, a, 0.], [0., 0., 1.]])
    np.testing.assert_array_equal(settings['coordinate_transform']['rotation_sensor_to_tool'], expected_rotation)
    np.testing.assert_array_equal(settings['coordinate_transform']['sensor_origin_in_tool_m'], [0., 0., .024])
    # Unit Sensor X/Y/Z: r_T=[0,0,.024] gives (-.024*Fy, +.024*Fx, 0).
    expected_forces_tool = ([a, a, 0.], [-a, a, 0.], [0., 0., 1.])
    expected_torques_tool = ([-.024*a, .024*a, 0.], [-.024*a, -.024*a, 0.], [0., 0., 0.])
    for axis, force_tool, torque_tool in zip(np.eye(3), expected_forces_tool, expected_torques_tool):
        result = p.process(wrench(axis))
        np.testing.assert_allclose(result.force, rotation_base_tool @ force_tool, atol=1e-14)
        np.testing.assert_allclose(result.torque, rotation_base_tool @ torque_tool, atol=1e-14)
    # Also rotate a nonzero intrinsic Sensor moment before adding the lever arm.
    result = p.process(Wrench(1., 0., 0., 0., 1., 0.))
    np.testing.assert_allclose(result.torque,
        rotation_base_tool @ np.array([-a-.024*a, a+.024*a, 0.]), atol=1e-14)
