"""Equal-scale Base XY preview, optionally overlaid with the existing actual TCP log."""
from pathlib import Path
import json
import subprocess
import sys

import numpy as np

from visualization.xy_grid import square_xy_ticks


def plot_plan(plan, *, actual_xy=None, contacts=None, interactive=False):
    if interactive:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(12, 8))
    else:
        # Saving a PNG must not create a GUI in the robot/terminal process.
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        fig = Figure(figsize=(12, 8))
        FigureCanvasAgg(fig)
        ax = fig.subplots()
    fig.subplots_adjust(right=.72, bottom=.12)
    for key, label, style in (
        ('raw_corners_xy', 'Taught corners C0-C3', 'o:'),
        ('rectified_corners_xy', 'Rectified rectangle', '--'),
        ('effective_corners_xy', 'Effective scan region', '-')):
        xy = np.asarray(plan[key])*1000
        ax.plot(*np.vstack((xy,xy[0])).T, style, label=label)
    route = np.asarray(plan['coverage_xy'])*1000
    ax.plot(*route.T, color='tab:blue', linewidth=1, label='Zigzag coverage')
    # Sparse arrows keep large plans readable; always include the first leg.
    for i in np.unique(np.linspace(0, len(route)-2, min(12,len(route)-1), dtype=int)):
        a,b = route[i],route[i+1]
        ax.annotate('', xy=a+.65*(b-a), xytext=a+.35*(b-a),
                    arrowprops=dict(arrowstyle='->', color='tab:blue'))
    entry = np.asarray(plan['entry_xy'])*1000
    ax.plot(*entry.T, color='tab:orange', linestyle='-.', label='P0 entry (raised if outside)')
    ax.scatter(*entry[0], marker='*', s=120, color='tab:orange', label='P0')
    ax.scatter(*route[0], marker='s', s=40, color='green', label='Coverage start')
    ax.scatter(*route[-1], marker='X', s=70, color='red', label='Coverage end')
    if actual_xy is not None and len(actual_xy):
        ax.plot(*(np.asarray(actual_xy)*1000).T, color='black', linewidth=.8, alpha=.65, label='Actual TCP')
    if contacts:
        ax.scatter(*(np.asarray(contacts)*1000).T, marker='*', s=150, color='magenta', label='First confirmed contact')
    ax.set(xlabel='Base X [mm]', ylabel='Base Y [mm]', title='Fixed-start sandbox coverage')
    ax.set_aspect('equal', adjustable='box')
    square_xy_ticks(ax)
    ax.grid(True, alpha=.3)
    ax.legend(loc='upper left', bbox_to_anchor=(1.02,1.), fontsize=9)
    text = (f"Scan Z: {plan['scan_z']*1000:g} mm\nP0 taught Z: {plan['p0_tcp_pose'][2]*1000:g} mm\n"
        f"Line spacing <= {plan['line_spacing_m']*1000:g} mm\nActual spacing: {plan['actual_line_spacing_m']*1000:.3f} mm\n"
        f"Search speed: {plan['search_speed_mps']*1000:g} mm/s\n"
        f"Tracking base: {plan['tracking_base_speed_mps']*1000:g} mm/s\n"
        f"Boundary inset: {plan['boundary_margin_m']*1000:.3f} mm\n"
        f"Lines: {plan['line_count']}\nCoverage: {plan['coverage_length_m']:.3f} m\nEntry: {plan['entry_kind']}")
    fig.text(.74, .40, text, va='top', fontsize=9)
    return fig


def save_preview(plan, directory, *, show=False):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    plan_path = directory/'sandbox_scan_plan.json'
    plan_path.write_text(json.dumps(plan, ensure_ascii=False, indent=2), encoding='utf-8')
    fig = plot_plan(plan)
    path = directory/'sandbox_scan_preview.png'
    fig.savefig(path, dpi=150)
    if show:
        # Tk/Qt event loops cannot share the thread blocked on terminal input
        # or running robot control. The viewer has no hardware/control imports.
        subprocess.Popen([sys.executable, '-m', 'visualization.sandbox_scan',
                          '--view-plan', str(plan_path.resolve())],
                         cwd=Path(__file__).resolve().parents[1],
                         stdin=subprocess.DEVNULL, start_new_session=True)
    return path


def show_preview(plan_path):
    """Read-only GUI process: keep its event loop alive until the window closes."""
    import matplotlib.pyplot as plt
    plan = json.loads(Path(plan_path).read_text(encoding='utf-8'))
    fig = plot_plan(plan, interactive=True)
    fig.canvas.manager.set_window_title('Sandbox coverage preview')
    plt.show(block=True)


def generate_run_visualization(run_dir):
    from visualization.run_plots import generate_run_visualization as original
    paths = list(original(run_dir))
    run_dir = Path(run_dir)
    plan = json.loads((run_dir/'sandbox_scan_plan.json').read_text(encoding='utf-8'))
    # Same exact Base TCP columns used by the original plotter.
    from visualization.run_plots import load_trace
    trace = load_trace(run_dir)
    contacts = []
    summary = json.loads((run_dir/'summary.json').read_text(encoding='utf-8'))
    if summary.get('initial_contact') is not None:
        contacts = [summary['initial_contact'][:2]]
    fig = plot_plan(plan, actual_xy=trace.xy_mm/1000, contacts=contacts)
    path = run_dir/'sandbox_scan_actual.png'
    fig.savefig(path, dpi=150)
    import matplotlib.pyplot as plt
    plt.close(fig)
    return paths+[path]


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Read-only sandbox coverage preview')
    parser.add_argument('--view-plan', type=Path, required=True)
    show_preview(parser.parse_args().view_plan)
