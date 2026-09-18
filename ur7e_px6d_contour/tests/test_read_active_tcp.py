import pytest

from tools.read_active_tcp import validate_tcp_offset


def test_validate_tcp_offset_accepts_six_finite_values():
    assert validate_tcp_offset([0, 0, 0.25, 0, 0, 0]) == [0.0, 0.0, 0.25, 0.0, 0.0, 0.0]


@pytest.mark.parametrize("value", ([0, 0, 0], [0, 0, 0, 0, 0, float("nan")]))
def test_validate_tcp_offset_rejects_invalid_values(value):
    with pytest.raises(ValueError):
        validate_tcp_offset(value)
