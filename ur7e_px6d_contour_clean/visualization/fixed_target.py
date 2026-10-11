"""Offline-only target overlay attached after devices and log files are closed."""
from pathlib import Path

from calibration.target_geometry import load_geometry, save_geometry, outline_mm


def recorded_outline(run_dir):
    path = Path(run_dir)/'target_geometry_calibration.yaml'
    if not path.exists():
        return None
    try:
        return outline_mm(load_geometry(path))
    except (ValueError, KeyError, TypeError, OSError) as exc:
        print(f'可选目标轮廓无效，保留原 TCP 回放：{exc}', flush=True)
        return None


def generate_fixed_visualization(run_dir, geometry_path=None):
    from visualization.run_plots import generate_run_visualization
    # No lookup occurs during control. Historical playback uses the run copy,
    # so later reteaching cannot move a target in an already recorded experiment.
    if geometry_path is not None and Path(geometry_path).is_file():
        try:
            save_geometry(Path(run_dir)/'target_geometry_calibration.yaml', load_geometry(geometry_path))
        except (ValueError, KeyError, TypeError, OSError) as exc:
            print(f'可选目标轮廓无效，保留原 TCP 回放：{exc}', flush=True)
    return generate_run_visualization(run_dir)
