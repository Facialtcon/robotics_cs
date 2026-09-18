import math

import pytest

from sensor.px6d_reader import (
    GET_FRAME_COMMAND,
    GET_VERSION_COMMAND,
    ProtocolError,
    parse_version_packet,
    parse_wrench_packet,
)


def test_official_commands_match_manual():
    assert GET_FRAME_COMMAND.hex(" ") == "aa 55 7f 05 01 fa"
    assert GET_VERSION_COMMAND.hex(" ") == "aa 55 7f 07 01 d0"


def test_measured_wrench_packet():
    packet = bytes.fromhex(
        "aa 55 7f 03 04 2f c1 3d c9 85 41 3e 8c 73 22 3d "
        "ea 6a f4 bb d8 4d 06 3c a4 1b 69 3b 17"
    )
    actual = parse_wrench_packet(packet).array()
    expected = (0.0943279564, 0.1889869124, 0.0396609753, -0.0074590342, 0.0081972703, 0.0035569454)
    assert all(math.isclose(a, e, rel_tol=1e-6) for a, e in zip(actual, expected))


def test_measured_version_packet():
    packet = bytes.fromhex("aa 55 7f 07 00 76 31 2e 30 2e 31 00 64")
    assert parse_version_packet(packet) == "v1.0.1"


def test_bad_crc_rejected():
    packet = bytearray.fromhex(
        "aa 55 7f 03 04 2f c1 3d c9 85 41 3e 8c 73 22 3d "
        "ea 6a f4 bb d8 4d 06 3c a4 1b 69 3b 00"
    )
    with pytest.raises(ProtocolError, match="CRC"):
        parse_wrench_packet(bytes(packet))

