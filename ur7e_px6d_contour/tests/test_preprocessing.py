import numpy as np

from sensor.force_preprocess import WrenchPreprocessor
from core.models import Wrench


def make_preprocessor(alpha=0.0, rotation=None, origin=(0, 0, 0)):
    return WrenchPreprocessor(
        filter_alpha=alpha,
        rotation_sensor_to_output=np.eye(3) if rotation is None else rotation,
        sensor_origin_in_output_m=np.asarray(origin),
        gravity_wrench_sensor=np.zeros(6),
        granular_baseline_output=np.zeros(6),
    )


def test_bias_is_subtracted_without_losing_raw_sample():
    raw = Wrench(2, 3, 4, 5, 6, 7)
    processor = make_preprocessor()
    processor.set_zero_bias([Wrench(1, 1, 1, 1, 1, 1)])
    assert np.allclose(processor.process(raw).array(), [1, 2, 3, 4, 5, 6])
    assert np.allclose(raw.array(), [2, 3, 4, 5, 6, 7])


def test_ema_weights_previous_sample():
    processor = make_preprocessor(alpha=0.75)
    processor.process(Wrench(0, 0, 0, 0, 0, 0))
    result = processor.process(Wrench(4, 0, 0, 0, 0, 0))
    assert np.isclose(result.fx, 1.0)


def test_force_and_torque_transform_includes_moment_arm():
    rotation = np.asarray([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
    processor = make_preprocessor(rotation=rotation, origin=(0, 0, 1))
    result = processor.process(Wrench(1, 0, 0, 0, 0, 0))
    assert np.allclose(result.force, [0, 1, 0])
    assert np.allclose(result.torque, [-1, 0, 0])

