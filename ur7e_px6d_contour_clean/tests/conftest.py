"""All tests are hardware-free, including accidental construction paths."""
import os
from pathlib import Path
import socket
import pytest

os.environ.setdefault('MPLBACKEND', 'Agg')
os.environ.setdefault('MPLCONFIGDIR', '/tmp/contour-clean-test-mpl')
ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(autouse=True)
def no_hardware(monkeypatch, tmp_path):
    from experiment_logging import paths
    monkeypatch.setattr(paths, 'DATA_ROOT', tmp_path/'data')
    def forbidden(*args, **kwargs):
        raise AssertionError('real hardware/network is forbidden in clean tests')
    monkeypatch.setattr(socket, 'create_connection', forbidden)
    import rtde_control, rtde_receive, serial
    monkeypatch.setattr(rtde_control, 'RTDEControlInterface', forbidden)
    monkeypatch.setattr(rtde_receive, 'RTDEReceiveInterface', forbidden)
    monkeypatch.setattr(serial, 'Serial', forbidden)
    # Contract inspection is performed separately against the installed SDK, never connected.
    from robot.rtde_controller import URRTDEController
    monkeypatch.setattr(URRTDEController, 'verified_watchdog_contract', staticmethod(lambda: {'version': 'fake'}))


@pytest.fixture
def config():
    from config.loader import load_config
    return load_config(ROOT/'config.yaml')
