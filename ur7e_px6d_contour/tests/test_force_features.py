import math

import numpy as np

from sensor.force_features import angle_between, extract_force_features, perpendicular
from core.models import Wrench


def test_xy_features():
    features = extract_force_features(Wrench(3, 4, 9, 0, 0, 0))
    assert features.fxy == 5
    assert math.isclose(features.force_angle, math.atan2(4, 3))
    assert np.allclose(features.interaction_direction, [0.6, 0.8])


def test_tangent_and_angle():
    assert np.allclose(perpendicular([1, 0], "LEFT"), [0, 1])
    assert np.allclose(perpendicular([1, 0], "RIGHT"), [0, -1])
    assert math.isclose(angle_between([1, 0], [0, 1]), math.pi / 2)

