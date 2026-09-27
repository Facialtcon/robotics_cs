import pathlib
import sys
import importlib
import importlib.util
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


@pytest.fixture(autouse=True)
def forbid_real_hardware(monkeypatch):
    """All project tests are offline; SDK docs remain inspectable, devices do not."""
    def forbidden(*args, **kwargs):
        pytest.fail('real RTDE/serial interface construction is forbidden in offline tests')
    for module_name, class_name in (('rtde_control', 'RTDEControlInterface'),
                                    ('rtde_receive', 'RTDEReceiveInterface'), ('serial', 'Serial')):
        if importlib.util.find_spec(module_name) is not None:
            cls = getattr(importlib.import_module(module_name), class_name)
            monkeypatch.setattr(cls, '__init__', forbidden)
