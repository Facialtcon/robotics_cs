"""Offline transform checks; supplied matrices represent synthetic known mounts."""
import copy

import numpy as np
import pytest

from core.models import Wrench
from sensor.force_preprocess import WrenchPreprocessor
from sensor.force_direction import control_directions


RX = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
RY = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
RZ = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])


def preprocessing(**transform):
    return dict(filter_alpha=0., gravity_wrench_sensor=[0.]*6,
        granular_baseline_output=[0.]*6, coordinate_transform=dict(
            rotation_sensor_to_base=np.eye(3).tolist(),
            sensor_origin_in_base_m=[0.]*3, **transform))


def wrench(force):
    return Wrench.from_sequence([*force, 0., 0., 0.])


def test_unknown_identity_does_not_become_calibrated_by_tcp_pose():
    config = preprocessing()
    before = copy.deepcopy(config)
    processor = WrenchPreprocessor.from_config(config, tool_orientation=[0., 0., np.pi/2])
    status = processor.force_transform_status
    assert status['available'] is False
    assert status['output_frame'] == 'sensor_uncalibrated'
    assert status['rotation_sensor_to_base'] is None
    np.testing.assert_allclose(processor.process(wrench([1., 2., 3.])).force, [1., 2., 3.])
    processor.set_tool_orientation([0., np.pi/2, 0.])
    assert processor.force_transform_status['available'] is False
    assert config == before


def test_unknown_mount_preserves_sensor_bias_and_filter_but_ignores_base_baseline():
    config = preprocessing()
    config['coordinate_transform']['rotation_sensor_to_base'] = RZ.tolist()
    config['coordinate_transform']['sensor_origin_in_base_m'] = [1., 2., 3.]
    config['granular_baseline_output'] = [50.]*6
    config['filter_alpha'] = .5
    processor = WrenchPreprocessor.from_config(config)
    processor.set_zero_bias([wrench([5., 0., 0.])])
    np.testing.assert_allclose(processor.process(wrench([7., 0., 0.])).array(), [2., 0., 0., 0., 0., 0.])
    np.testing.assert_allclose(processor.process(wrench([9., 0., 0.])).force, [3., 0., 0.])


def test_explicit_sensor_to_tool_composes_in_correct_order_and_updates():
    processor = WrenchPreprocessor.from_config(
        preprocessing(rotation_sensor_to_tool=RX.tolist()), tool_orientation=[0., 0., np.pi/2])
    np.testing.assert_allclose(processor.rotation_sensor_to_output, RZ @ RX, atol=1e-14)
    np.testing.assert_allclose(processor.process(wrench([0., 1., 0.])).force, [0., 0., 1.], atol=1e-14)
    processor.set_tool_orientation([0., np.pi/2, 0.])
    np.testing.assert_allclose(processor.rotation_sensor_to_output, RY @ RX, atol=1e-14)
    status = processor.force_transform_status
    assert status['available'] is True and status['output_frame'] == 'Base'
    assert status['tool_orientation'] == [0., np.pi/2, 0.]
    np.testing.assert_allclose(status['rotation_sensor_to_base'], RY @ RX, atol=1e-14)
    np.testing.assert_allclose(status['rotation_sensor_to_tool'], RX, atol=1e-14)


def test_mount_requires_actual_pose_and_can_be_resolved_later():
    processor = WrenchPreprocessor.from_config(preprocessing(rotation_sensor_to_tool=RX.tolist()))
    assert processor.force_transform_status['available'] is False
    assert processor.force_transform_status['reason'] == 'actual tool orientation is unavailable'
    processor.set_tool_orientation([0., 0., 0.])
    assert processor.force_transform_status['available'] is True
    np.testing.assert_allclose(processor.rotation_sensor_to_output, RX)


def test_known_reference_pose_maintains_mount_instead_of_treating_old_base_matrix_as_mount():
    config = preprocessing(reference_tool_orientation=[0., 0., np.pi/2])
    config['coordinate_transform']['rotation_sensor_to_base'] = (RZ @ RX).tolist()
    processor = WrenchPreprocessor.from_config(config, tool_orientation=[0., np.pi/2, 0.])
    np.testing.assert_allclose(processor.rotation_sensor_to_output, RY @ RX, atol=1e-14)
    assert processor.force_transform_status['source'] == 'configured_base_at_reference_pose'
    np.testing.assert_allclose(processor.force_transform_status['rotation_sensor_to_tool'], RX, atol=1e-14)
    assert processor.force_transform_status['reference_tool_orientation'] == [0., 0., np.pi/2]


def test_direct_installation_does_not_require_a_fixed_pose_base_matrix():
    config = preprocessing(rotation_sensor_to_tool=RX.tolist())
    config['coordinate_transform']['rotation_sensor_to_base'] = None
    processor = WrenchPreprocessor.from_config(config, tool_orientation=[0., 0., np.pi/2])
    np.testing.assert_allclose(processor.rotation_sensor_to_output, RZ @ RX, atol=1e-14)
    assert processor.force_transform_status['available'] is True


def test_null_reference_matrix_cannot_be_promoted_to_identity_calibration():
    config = preprocessing(reference_tool_orientation=[0., 0., 0.])
    config['coordinate_transform']['rotation_sensor_to_base'] = None
    with pytest.raises(ValueError, match='requires rotation_sensor_to_base'):
        WrenchPreprocessor.from_config(config, tool_orientation=[0., 0., 0.])


def test_known_reference_origin_rotates_with_tool():
    config = preprocessing(reference_tool_orientation=[0., 0., 0.])
    config['coordinate_transform']['sensor_origin_in_base_m'] = [1., 0., 0.]
    processor = WrenchPreprocessor.from_config(config, tool_orientation=[0., 0., np.pi/2])
    np.testing.assert_allclose(processor.sensor_origin_in_output_m, [0., 1., 0.], atol=1e-14)
    np.testing.assert_allclose(processor.process(wrench([0., 1., 0.])).array(),
                               [-1., 0., 0., 0., 0., 1.], atol=1e-14)


def test_synthetic_config_is_explicit_and_keeps_old_simulation_transform():
    config = preprocessing()
    config['coordinate_transform']['rotation_sensor_to_base'] = RZ.tolist()
    processor = WrenchPreprocessor.from_config(config, synthetic=True)
    assert processor.force_transform_status['source'] == 'synthetic_config'
    np.testing.assert_allclose(processor.process(wrench([1., 0., 0.])).force, [0., 1., 0.])


def test_moving_known_mount_does_not_keep_an_unreferenced_nonzero_base_lever_arm():
    config = preprocessing(rotation_sensor_to_tool=np.eye(3).tolist())
    config['coordinate_transform']['sensor_origin_in_base_m'] = [1., 0., 0.]
    with pytest.raises(ValueError, match='nonzero sensor_origin_in_base_m'):
        WrenchPreprocessor.from_config(config, tool_orientation=[0., 0., np.pi/2])
    config['coordinate_transform']['sensor_origin_in_tool_m'] = [1., 0., 0.]
    processor = WrenchPreprocessor.from_config(config, tool_orientation=[0., 0., np.pi/2])
    np.testing.assert_allclose(processor.sensor_origin_in_output_m, [0., 1., 0.], atol=1e-14)


@pytest.mark.parametrize('orientation', [[0., 0.], [0., 0., np.nan], [0., np.inf, 0.]])
def test_bad_orientation_is_not_accepted(orientation):
    with pytest.raises(ValueError, match='rotation-vector'):
        WrenchPreprocessor.from_config(preprocessing(), tool_orientation=orientation)


@pytest.mark.parametrize('rotation', [[[1., 0., 0.], [0., 1., 0.], [0., 0., -1.]],
                                     [[2., 0., 0.], [0., 1., 0.], [0., 0., 1.]]])
def test_invalid_installation_rotation_is_rejected(rotation):
    with pytest.raises(ValueError, match='proper'):
        WrenchPreprocessor.from_config(preprocessing(rotation_sensor_to_tool=rotation))


def test_reference_pose_without_base_matrix_does_not_invent_identity_calibration():
    config = preprocessing(reference_tool_orientation=[0., 0., 0.])
    del config['coordinate_transform']['rotation_sensor_to_base']
    with pytest.raises(ValueError, match='requires rotation_sensor_to_base'):
        WrenchPreprocessor.from_config(config)


def test_transformed_normal_tangent_and_unload_use_same_base_frame():
    processor = WrenchPreprocessor.from_config(preprocessing(rotation_sensor_to_tool=np.eye(3).tolist()),
                                             tool_orientation=[0., 0., np.pi/2])
    force = processor.process(wrench([1., 0., 0.])).force
    cw = control_directions(force, 1, 'CLOCKWISE')
    ccw = control_directions(force, 1, 'COUNTERCLOCKWISE')
    np.testing.assert_allclose(cw[0], [0., 1.], atol=1e-14)
    np.testing.assert_allclose(cw[2], [0., -1.], atol=1e-14)
    np.testing.assert_allclose(cw[1], -ccw[1], atol=1e-14)
    np.testing.assert_allclose(cw[0], ccw[0], atol=1e-14)
