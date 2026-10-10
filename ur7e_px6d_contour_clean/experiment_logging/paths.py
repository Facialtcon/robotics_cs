"""One place for experiment identity, provenance and all run directories."""
from datetime import datetime, timezone
from zoneinfo import ZoneInfo
import json
from pathlib import Path
import subprocess

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DATA_ROOT = PROJECT_ROOT / 'data'
LOCAL_TIMEZONE = 'Asia/Shanghai'


def wall_time_fields(now=None, *, utc_key='timestamp_utc'):
    """UTC and Beijing wall time from one aware instant; never a control clock."""
    now = datetime.now(timezone.utc) if now is None else now
    if now.tzinfo is None:
        raise ValueError('wall time must be timezone aware')
    return {utc_key: now.astimezone(timezone.utc).isoformat(),
            'timestamp_local': now.astimezone(ZoneInfo(LOCAL_TIMEZONE)).isoformat(),
            'timezone': LOCAL_TIMEZONE}


def local_time_text(timestamp):
    now = datetime.fromisoformat(timestamp)
    return f'{now.astimezone(ZoneInfo(LOCAL_TIMEZONE)):%Y-%m-%d %H:%M:%S.%f} 北京时间 ({LOCAL_TIMEZONE})'


def run_sort_key(path):
    """Metadata creation time takes precedence over either directory spelling."""
    path = Path(path)
    try:
        stamp = datetime.fromisoformat(read_metadata(path)['timestamp'])
        if stamp.tzinfo is None:
            stamp = stamp.replace(tzinfo=timezone.utc)  # Historical timestamp is UTC.
    except (OSError, ValueError, KeyError):
        stamp = None
        for pattern, zone in (('run_%Y-%m-%d_%H-%M-%S_%f', ZoneInfo(LOCAL_TIMEZONE)),
                              ('run_%Y%m%d_%H%M%S_%f', timezone.utc)):
            try:
                stamp = datetime.strptime(path.name, pattern).replace(tzinfo=zone)
                break
            except ValueError:
                pass
    return (float('inf') if stamp is None else stamp.timestamp(), str(path))


def run_root(mode, strategy, data_root=None):
    if (mode, strategy) not in {('real', 'continuous'), ('real', 'discrete'),
                               ('real', 'return'), ('simulation', 'continuous'),
                               ('simulation', 'discrete'), ('real', 'single_point'),
                               ('simulation', 'single_point')}:
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
    path = root / ('run_' + now.astimezone(ZoneInfo(LOCAL_TIMEZONE)).strftime('%Y-%m-%d_%H-%M-%S_%f'))
    path.mkdir(parents=True, exist_ok=False)
    metadata = dict(schema_version=2, mode=mode, strategy=strategy,
                    **wall_time_fields(now, utc_key='timestamp'),
                    timestamp_meaning='run_directory_created_at', git=git_provenance(),
                    config_source=str(Path(config_source).expanduser().resolve())
                    if config_source else None, **extra)
    (path / 'metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print('记录目录创建时间：'+local_time_text(metadata['timestamp']), flush=True)
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
    if set(fields) & {'mode', 'strategy', 'timestamp', 'timestamp_local', 'timezone',
                      'timestamp_meaning', 'git', 'config_source', 'schema_version'}:
        raise ValueError('experiment identity cannot be overwritten')
    data = read_metadata(path)
    data.update(fields)
    (Path(path)/'metadata.json').write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
