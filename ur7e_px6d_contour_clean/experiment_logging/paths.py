"""One place for experiment identity, provenance and all run directories."""
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DATA_ROOT = PROJECT_ROOT / 'data'


def run_root(mode, strategy, data_root=None):
    if (mode, strategy) not in {('real', 'continuous'), ('real', 'discrete'),
                               ('real', 'return'), ('simulation', 'continuous'),
                               ('simulation', 'discrete')}:
        raise ValueError(f'invalid experiment type: {mode}/{strategy}')
    return Path(data_root or DATA_ROOT).expanduser().resolve() / mode / strategy


def git_provenance():
    def read(*args):
        return subprocess.check_output(['git', *args], cwd=PROJECT_ROOT,
                                       text=True, stderr=subprocess.DEVNULL).strip()
    try:
        return dict(branch=read('branch', '--show-current'), commit=read('rev-parse', 'HEAD'),
                    dirty=bool(read('status', '--porcelain')))
    except (OSError, subprocess.SubprocessError):
        return dict(branch=None, commit=None, dirty=None)


def create_run(mode, strategy, config_source, *, data_root=None, **extra):
    root = run_root(mode, strategy, data_root)
    now = datetime.now(timezone.utc)
    path = root / ('run_' + now.strftime('%Y%m%d_%H%M%S_%f'))
    path.mkdir(parents=True, exist_ok=False)
    metadata = dict(schema_version=1, mode=mode, strategy=strategy,
                    timestamp=now.isoformat(), git=git_provenance(),
                    config_source=str(Path(config_source).expanduser().resolve())
                    if config_source else None, **extra)
    (path / 'metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    return path


def read_metadata(path):
    data = json.loads((Path(path) / 'metadata.json').read_text(encoding='utf-8'))
    run_root(data['mode'], data['strategy'])  # validate the identity, never guess it
    for key in ('timestamp', 'git', 'config_source'):
        if key not in data:
            raise ValueError(f'missing metadata field: {key}')
    return data


def annotate_run(path, **fields):
    """Add device/scene context without changing the run's authoritative identity."""
    if set(fields) & {'mode', 'strategy', 'timestamp', 'git', 'config_source', 'schema_version'}:
        raise ValueError('experiment identity cannot be overwritten')
    data = read_metadata(path)
    data.update(fields)
    (Path(path)/'metadata.json').write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
