import pytest

from tools.read_active_tcp import validate_tcp_offset


def test_validate_tcp_offset_accepts_six_finite_values():
    assert validate_tcp_offset([0, 0, 0.25, 0, 0, 0]) == [0.0, 0.0, 0.25, 0.0, 0.0, 0.0]


@pytest.mark.parametrize("value", ([0, 0, 0], [0, 0, 0, 0, 0, float("nan")]))
def test_validate_tcp_offset_rejects_invalid_values(value):
    with pytest.raises(ValueError):
        validate_tcp_offset(value)


@pytest.fixture
def tcp_tool(tmp_path, monkeypatch):
    from pathlib import Path
    from types import SimpleNamespace
    import sys
    import yaml
    from tools import read_active_tcp as tool

    source = Path(__file__).resolve().parents[1] / 'config.yaml'
    path = tmp_path / 'config with spaces.yaml'
    path.write_bytes(source.read_bytes())
    original = path.read_bytes()
    old = yaml.safe_load(original)['tcp']['offset']
    new = list(old)
    new[0] += .001
    events = []

    class ReadOnlyControl:
        def __init__(self, address):
            events.append('construct_stub')
        def getTCPOffset(self):
            events.append('read')
            return new
        def disconnect(self):
            events.append('disconnect')
    monkeypatch.setitem(sys.modules, 'rtde_control', SimpleNamespace(RTDEControlInterface=ReadOnlyControl))
    monkeypatch.setattr(sys, 'argv', ['read_active_tcp.py', '--config', str(path), '--write-config'])
    return tool, path, original, new, events, ReadOnlyControl


def test_explicit_save_updates_only_tcp_and_backs_up_original(tcp_tool, capsys):
    import yaml
    tool, path, original, new, events, _ = tcp_tool
    assert tool.main() == 0
    expected = yaml.safe_load(original)
    expected['tcp']['offset'] = new
    assert yaml.safe_load(path.read_bytes()) == expected
    backups = list(path.parent.glob('*.backup.yaml'))
    assert len(backups) == 1 and backups[0].read_bytes() == original
    assert events == ['construct_stub', 'read', 'disconnect']
    output = capsys.readouterr().out
    assert str(path) in output and str(backups[0]) in output and 'TCP 已写入' in output
    # Repeat read does not rewrite or create a second backup when exactly equal.
    after = path.read_bytes()
    assert tool.main() == 0
    assert path.read_bytes() == after and len(list(path.parent.glob('*.backup.yaml'))) == 1


def test_direct_command_remains_print_only(tcp_tool, monkeypatch, capsys):
    import sys
    tool, path, original, _, _, _ = tcp_tool
    monkeypatch.setattr(sys, 'argv', ['read_active_tcp.py', '--config', str(path)])
    assert tool.main() == 0
    assert path.read_bytes() == original
    assert not list(path.parent.glob('*.backup.yaml'))
    assert 'TCP 已写入' not in capsys.readouterr().out


@pytest.mark.parametrize('failure', ['read', 'invalid', 'disconnect'])
def test_device_failure_never_changes_config(tcp_tool, monkeypatch, failure):
    tool, path, original, _, _, control = tcp_tool
    def fail(*args):
        raise RuntimeError('injected device error')
    if failure == 'invalid':
        monkeypatch.setattr(control, 'getTCPOffset', lambda _: [float('nan')] * 6)
    else:
        monkeypatch.setattr(control, 'getTCPOffset' if failure == 'read' else 'disconnect', fail)
    with pytest.raises((RuntimeError, ValueError)):
        tool.main()
    assert path.read_bytes() == original
    assert not list(path.parent.glob('*.backup.yaml'))


def test_concurrent_config_edit_is_not_overwritten(tcp_tool, monkeypatch):
    tool, path, original, new, _, control = tcp_tool
    edited = original + b'\n# operator edit\n'
    def read(_):
        path.write_bytes(edited)
        return new
    monkeypatch.setattr(control, 'getTCPOffset', read)
    with pytest.raises(RuntimeError, match='changed'):
        tool.main()
    assert path.read_bytes() == edited
    assert not list(path.parent.glob('*.backup.yaml'))


def test_failed_atomic_replace_leaves_original_and_backup(tcp_tool, monkeypatch):
    tool, path, original, _, _, _ = tcp_tool
    def fail(*args):
        raise OSError('injected write error')
    monkeypatch.setattr(tool.os, 'replace', fail)
    with pytest.raises(OSError, match='write error'):
        tool.main()
    assert path.read_bytes() == original
    assert next(path.parent.glob('*.backup.yaml')).read_bytes() == original
    assert not list(path.parent.glob('.config*'))


@pytest.mark.parametrize('offset', [
    '[0, 0, 0, 0, 0, 0] # keep inline comment',
    '\n    - 0 # keep item comment\n    - 0\n    - 0\n    - 0\n    - 0\n    - 0',
])
def test_preserves_comments_and_unrelated_yaml(tmp_path, offset):
    from tools.read_active_tcp import save_tcp_offset
    original = ('# keep header\ntcp:\n  offset: ' + offset + '\n  offset_tolerance: 0.0001\nother: [1, 2]\n').encode()
    path = tmp_path / 'config.yaml'
    path.write_bytes(original)
    save_tcp_offset(path, original, [1, 2, 3, 4, 5, 6])
    after = path.read_text()
    assert '# keep header' in after and 'other: [1, 2]\n' in after
    assert '# keep inline comment' in after or '# keep item comment' in after
    assert '  offset_tolerance: 0.0001\n' in after
