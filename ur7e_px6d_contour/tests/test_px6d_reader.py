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


def test_stationary_resynchronization_discards_old_bytes_and_checks_version():
    from sensor.px6d_reader import PX6DReader
    version = bytes.fromhex('aa 55 7f 07 00 76 31 2e 30 2e 31 00 64')
    class Port:
        data = b'old partial packet'
        writes = []
        def reset_input_buffer(self): self.data = b''
        def write(self, command):
            self.writes.append(command); self.data += version
        def flush(self): pass
        @property
        def in_waiting(self): return len(self.data)
        def read(self, n):
            data, self.data = self.data[:n], self.data[n:]; return data
    r = PX6DReader('fake'); r._port = Port(); r._buffer.extend(b'old software buffer')
    r.resynchronize_after_timeout()
    assert r._port.writes == [GET_VERSION_COMMAND]
    assert r.firmware == 'v1.0.1' and not r._buffer


def test_read_timeout_keeps_original_base_exception_contract():
    from types import SimpleNamespace
    from sensor.px6d_reader import PX6DReader, PX6DError, PX6DTimeout
    r = PX6DReader('fake'); r._port = SimpleNamespace(in_waiting=0)
    with pytest.raises(PX6DTimeout) as exc:
        r._read_packet(29, .001)
    assert isinstance(exc.value, PX6DError)
