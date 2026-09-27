#!/usr/bin/env python3
"""Offline synchronized XY / force replay. Never imported by the control loop."""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys
import textwrap
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import yaml


def read_run(run_dir):
    run_dir = Path(run_dir)
    with (run_dir / "samples.csv").open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError("run contains no control samples")
    required = ("monotonic_sec", "tcp_x", "tcp_y", "dfx", "dfy", "fxy", "force_reference",
                "tangent_x", "tangent_y", "contact_direction_x", "contact_direction_y")
    data = {key: np.asarray([float(row[key]) for row in rows]) for key in required}
    if not np.all(np.isfinite(data["monotonic_sec"])) or np.any(np.diff(data["monotonic_sec"]) < 0):
        raise ValueError("sample timestamps must be finite and ordered")
    for key in ('direction_valid', 'command_vx', 'command_vy', 'reacquire_origin_x', 'reacquire_origin_y',
                'reacquire_reference_x', 'reacquire_reference_y'):
        data[key] = np.array([float(row.get(key) or ('0' if key=='direction_valid' else 'nan')) for row in rows])
    for key in ('tcp_vx','tcp_vy','tcp_vz','v_t','v_n','stop_requested','raw_fx','raw_fy',
                'sim_components_available','sim_object_fx','sim_object_fy','sim_friction_fx','sim_friction_fy',
                'sim_background_fx','sim_background_fy','sim_noise_fx','sim_noise_fy',
                'measurement_jump_deg','estimate_residual_deg','physical_force_available',
                'sim_normal_physical_fx','sim_normal_physical_fy',
                'sim_boundary_physical_fx','sim_boundary_physical_fy','sim_robot_estimate_fx','sim_robot_estimate_fy'):
        data[key] = np.array([float(row.get(key) or 'nan') for row in rows])
    data['reason'] = [row.get('reason','') for row in rows]
    data['direction_phase'] = [row.get('direction_phase','unavailable') for row in rows]
    data["time"] = data["monotonic_sec"] - data["monotonic_sec"][0]
    data["state"] = [row["current_state"] for row in rows]
    with (run_dir / "config_snapshot.yaml").open(encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    from experiment_logging.paths import read_metadata
    if read_metadata(run_dir)['strategy'] != 'continuous':
        raise ValueError('selected run is not a continuous experiment')
    events = []
    if (run_dir / 'policy_waypoints.csv').exists():
        with (run_dir / 'policy_waypoints.csv').open(newline='', encoding='utf-8') as handle:
            events = list(csv.DictReader(handle))
    return data, config, events


def frame_indices(times, fps, max_frames=1200):
    if not np.isfinite(fps) or fps <= 0:
        raise ValueError("fps must be finite and positive")
    # Pick the latest sample at each playback tick, not every Nth row: works
    # with real control jitter as well as exact 100 Hz synthetic logs.
    if not isinstance(max_frames, int) or max_frames < 2:
        raise ValueError('max_frames must be an integer >= 2')
    count = int(np.ceil((times[-1]-times[0])*fps)) + 1
    ticks = (np.linspace(times[0], times[-1], max_frames-1, endpoint=False) if count > max_frames
             else np.arange(times[0], times[-1], 1/fps))
    indices = np.searchsorted(times, ticks, side="right") - 1
    return np.r_[indices, len(times)-1].astype(int)


def make_figure(data, config, events, *, local_xy=False, view=None, components=False, debug=False, true_tangent=False):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Rectangle, Circle
    from simulation.continuous_view import (XYViewport, VectorDisplay, force_demonstration, CJK_FONT,
        FORCE_CURVES, SPEED_CURVES, line_style, component_note)
    from simulation.continuous_preview import extrema_indices

    fig=plt.figure(figsize=(14,8))
    grid=fig.add_gridspec(2,2,width_ratios=[3,1],hspace=.5,wspace=.18)
    xy=fig.add_subplot(grid[:,0]);force=fig.add_subplot(grid[0,1]);velocity=fig.add_subplot(grid[1,1])
    fig.subplots_adjust(left=.055,right=.97,bottom=.15,top=.88)
    xy.set_aspect('equal',adjustable='box')
    xy.set(xlabel='Base X [mm]',ylabel='Base Y [mm]')
    scene=config.get('continuous_simulation');outline=None
    points=np.column_stack((data['tcp_x'],data['tcp_y']))*1000
    bounds=np.array([points.min(axis=0)-20,points.max(axis=0)+20])
    if scene:
        if 'target_boundary_xy' in scene:outline=np.array(scene['target_boundary_xy'])*1000
        else:
            from simulation.geometry import create_target
            outline=create_target(scene).boundary_points()*1000
        xy.add_patch(Polygon(outline,closed=True,color='.8',label='simulation target truth'))
        b=scene['container'];bounds=np.array([[b['x_min'],b['y_min']],[b['x_max'],b['y_max']]])*1000
    else:
        calibrated=config.get('continuous_workspace_calibration',{}).get('rectified_points',{})
        if calibrated:
            xy.add_patch(Polygon([np.asarray(calibrated[k][:2])*1000 for k in ('R0','R1','R2','R3')],fill=False,color='gray',label='calibrated container'))
        elif config['workspace'].get('enabled',True):
            b=config['workspace']['limits'];bounds=np.array([[b['x_min'],b['y_min']],[b['x_max'],b['y_max']]])*1000
    xy.add_patch(Rectangle(bounds[0],*(bounds[1]-bounds[0]),fill=False,color='gray',label='container / view envelope'))
    executed,=xy.plot([],[],lw=1.3,**line_style('executed'))
    envelope=list(xy.patches)[-1]
    trajectory,=xy.plot([],[],color='tab:blue',lw=1,label='reliable contact TCP path')
    other_path,=xy.plot([],[],color='tab:orange',lw=1,label='search / lost / recovery TCP path')
    reference_path,=xy.plot([],[],'--',color='purple',lw=1,label='recovery position reference')
    recovery_range=Circle((0,0),float(config['continuous_tracking'].get('reacquire_max_distance',.004))*1000,
                           fill=False,ls=':',color='purple',label='recovery displacement limit')
    xy.add_patch(recovery_range);recovery_range.set_visible(False)
    probe,=xy.plot([],[],'ko',ms=4,zorder=10,label='TCP position symbol')
    viewport=XYViewport(xy,bounds,None if local_xy else outline)
    mode=(view or 'Target').title()
    viewport.select(mode,points[0],path=points)
    viewport.follow=mode=='Probe'
    fig.canvas.mpl_connect('scroll_event',viewport.scroll)
    vectors=VectorDisplay(xy)
    contact_mask=((np.array(data['state'])=='CONTINUOUS_TRACKING') & (data['direction_valid']>.5) &
                  (np.nan_to_num(data['stop_requested'])<.5) &
                  (data['fxy']>=float(config['continuous_tracking']['contact_lost_threshold'])))
    markers=[]
    for name,marker,color in [('FIRST_CONTACT','*','green'),('CONTACT_LOST','x','red'),('DIRECTION_STOP_REQUEST','x','orange'),
                              ('DIRECTION_CONFIRMED','s','purple'),('DIRECTION_RESUME_VERIFIED','+','green')]:
        selected=[e for e in events if e['event_type']==name]
        if selected:markers.append((xy.scatter([],[],marker=marker,color=color,s=30,label=name),selected))
    force_values=np.column_stack((data['dfx'],data['dfy'],data['fxy'],data['force_reference'],
                                  data['force_reference']-data['fxy'],np.hypot(data['raw_fx'],data['raw_fy'])))
    speeds=np.column_stack((np.hypot(data['command_vx'],data['command_vy']),
                            np.sqrt(data['tcp_vx']**2+data['tcp_vy']**2+data['tcp_vz']**2),data['v_t'],data['v_n']))*1000
    indices=extrema_indices(np.nan_to_num(force_values),buckets=100)
    for col,key in enumerate((*FORCE_CURVES, 'raw')):
        force.plot(data['time'][indices],force_values[indices,col],lw=.9,**line_style(key,simulated=bool(scene)))
    cursor=force.axvline(0,color='k',ls='--',lw=.8)
    indices=extrema_indices(np.nan_to_num(speeds),buckets=100)
    for col,key in enumerate(SPEED_CURVES):
        velocity.plot(data['time'][indices],speeds[indices,col],lw=.9,**line_style(key,simulated=bool(scene)))
    velocity_cursor=velocity.axvline(0,color='k',ls='--',lw=.8)
    force.set(xlabel='Elapsed time [s]',ylabel='Processed Base / raw force [N]')
    velocity.set(xlabel='Elapsed time [s]',ylabel='Velocity [mm/s]')
    for axis in (force,velocity):axis.legend(prop={'family': CJK_FONT, 'size': 6},ncol=2,loc='upper right');axis.grid(alpha=.2)
    title=fig.suptitle('',fontsize=10)
    note=fig.text(.055,.08,'',fontsize=8,va='bottom',fontfamily=['Noto Sans CJK JP', 'DejaVu Sans'])

    # Exposed only for offline tests/interactive notebook inspection.
    fig.continuous_viewport=viewport;fig.continuous_vectors=vectors
    fig.continuous_debug=bool(debug or components or true_tangent)
    fig.continuous_components=bool(components)
    fig.continuous_true_tangent=bool(true_tangent and scene)
    current_index=[0]
    from matplotlib.widgets import Button
    debug_button=Button(fig.add_axes([.86,.025,.10,.04]),'Debug')
    fig.continuous_debug_button=debug_button
    def toggle(event):
        fig.continuous_debug=not fig.continuous_debug
        update(current_index[0]);fig.canvas.draw_idle()
    debug_button.on_clicked(toggle)
    component_button=Button(fig.add_axes([.72,.025,.12,.04]),'Components')
    truth_button=Button(fig.add_axes([.45,.025,.25,.04]),'真实切向对照（仅仿真）：关')
    truth_button.label.set_fontfamily(CJK_FONT)
    truth_button.label.set_fontsize(8)
    fig.continuous_component_button=component_button;fig.continuous_truth_button=truth_button
    def toggle_components(event):
        fig.continuous_components=not fig.continuous_components
        update(current_index[0]);fig.canvas.draw_idle()
    def toggle_truth(event):
        fig.continuous_true_tangent=bool(scene) and not fig.continuous_true_tangent
        update(current_index[0]);fig.canvas.draw_idle()
    component_button.on_clicked(toggle_components);truth_button.on_clicked(toggle_truth)
    force.legend(handles=[force.lines[2],force.lines[3]],prop={'family': CJK_FONT, 'size': 7})
    last_mode=[None]

    def update(index):
        current_index[0]=index
        debug=fig.continuous_debug
        components=fig.continuous_components
        component_button.ax.set_visible(debug)
        truth_button.ax.set_visible(debug and bool(scene))
        truth_button.label.set_text('真实切向对照（仅仿真）：'+('开' if fig.continuous_true_tangent else '关'))
        if last_mode[0]!=debug:
            for artist in (trajectory,other_path,reference_path,envelope):artist.set_visible(False)
            velocity.set_visible(debug)
            if not scene:
                for patch in xy.patches:patch.set_visible(False)
            visible_values=force_values if debug else force_values[:,[2,3]]
            finite=visible_values[np.isfinite(visible_values)]
            if len(finite):force.set_ylim(min(0,finite.min())-.1,max(.1,finite.max())+.1)
            executed.set_visible(True)
            force.set_xlabel('' if debug else 'Elapsed time [s]')
            force.tick_params(axis='x', labelbottom=not debug)
            for i,line in enumerate(force.lines[:-1]):line.set_visible(debug or i in (2,3))
            force.legend(handles=[line for line in force.lines[:-1] if line.get_visible()],prop={'family': CJK_FONT, 'size': 7},ncol=1)
            last_mode[0]=debug
        point=points[index];mask=contact_mask[:index+1]
        # Bounded path display, preserve endpoints and keep gaps between modes.
        keep=np.unique(np.r_[np.arange(0,index+1,max(1,(index+1)//4000)),index])
        for line,selected in ((trajectory,mask),(other_path,~mask)):
            line.set_data(np.where(selected[keep],points[keep,0],np.nan),np.where(selected[keep],points[keep,1],np.nan))
        executed.set_data(points[keep,0],points[keep,1])
        reference_path.set_data(data['reacquire_reference_x'][keep]*1000,data['reacquire_reference_y'][keep]*1000)
        origin=np.array([data['reacquire_origin_x'][index],data['reacquire_origin_y'][index]])*1000
        recovery_range.set_visible(False)
        if np.isfinite(origin).all():recovery_range.center=origin
        probe.set_data([point[0]],[point[1]])
        viewport.update(point,running=True)
        available=bool(scene) and data['sim_components_available'][index]==1
        comp=({key:np.array([data['sim_'+key+'_fx'][index],data['sim_'+key+'_fy'][index]])
               for key in ('object','friction','background','noise')} if debug and components and available else None)
        row={key:values[index] for key,values in data.items()}
        row['current_state']=data['state'][index]
        physical=force_demonstration(row,config.get('force_display',{}),simulated=bool(scene),
                      components_visible=comp is not None,
                      previous_velocity=np.array([data['tcp_vx'][index-1],data['tcp_vy'][index-1]]) if index else None)
        vectors.draw(point,np.array([data['dfx'][index],data['dfy'][index]]),
                     np.array([data['command_vx'][index],data['command_vy'][index]])*1000,
                     np.array([data['tcp_vx'][index],data['tcp_vy'][index]])*1000,
                     np.array([data['tangent_x'][index],data['tangent_y'][index]]),
                     np.array([data['contact_direction_x'][index],data['contact_direction_y'][index]]),
                     valid=bool(contact_mask[index]),components=comp,physical=physical,debug=debug,
                     components_enabled=components, true_tangent=fig.continuous_true_tangent, simulated=bool(scene))
        for artist,selected in markers:
            artist.set_visible(False)
            visible=[[float(e['x'])*1000,float(e['y'])*1000] for e in selected if float(e['timestamp'])<=data['monotonic_sec'][index]+1e-8]
            artist.set_offsets(np.asarray(visible).reshape(-1,2))
        for cur in (cursor,velocity_cursor):cur.set_xdata([data['time'][index]]*2)
        f=force_values[index];v=speeds[index]
        force.set_title(f'Fx {f[0]:.2f}, Fy {f[1]:.2f}, Fxy {f[2]:.2f} N\nF_ref {f[3]:.2f}, error {f[4]:+.2f} N' if debug else 'Control feedback load [N]',fontsize=8)
        velocity.set_title(f'cmd {v[0]:.3f}, actual {v[1]:.3f} mm/s\nv_t {v[2]:+.3f}, v_n {v[3]:+.3f} mm/s',fontsize=8)
        title.set_text(f"{data['time'][index]:.2f}s | {data['state'][index]} / {data['direction_phase'][index]} | {data['reason'][index]}")
        note.set_text('\n'.join(part for part in (physical['note'],
            component_note(simulated=bool(scene),enabled=debug and components),vectors.truth_note) if part))
        width,height=fig.get_size_inches()
        note.set_text('\n'.join(textwrap.fill(line,width=max(30,int(width*72*.915/4.8)))
                                for line in note.get_text().splitlines()))
        fig.subplots_adjust(bottom=.08+((note.get_text().count('\n')+1)*12+40)/(height*72))
        vectors.layout_legend()
        force.set_ylabel('Control feedback [N]' if not debug else 'Processed Base / raw force [N]')

        return trajectory,other_path,reference_path,probe,cursor,velocity_cursor,title
    fig.canvas.mpl_connect('resize_event',lambda event:update(current_index[0]))
    update(0)
    return fig,update


def render(run_dir, *, fps=None, output_format="none", local_xy=False, view=None, components=False, debug=False, true_tangent=False):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FFMpegWriter, PillowWriter

    data, config, events = read_run(run_dir)
    fps = float(config["continuous_tracking"]["visualization_fps"] if fps is None else fps)
    if output_format == 'auto':
        output_format = 'mp4' if FFMpegWriter.isAvailable() else 'gif'
    max_frames = 240 if output_format == 'gif' else 1200
    indices = frame_indices(data["time"], fps, max_frames=max_frames)
    if data['time'][-1]*fps+1 > max_frames:
        fps = (len(indices)-1)/max(data['time'][-1], 1/fps)
        print(f'Bounded replay: {len(indices)} frames, {fps:.2f} fps')
    fig, update = make_figure(data, config, events, local_xy=local_xy, view=view, components=components, debug=debug, true_tangent=true_tangent)
    output = Path(run_dir)
    suffix = "_local" if local_xy else ("_"+view if view else "")
    summary = output / f"continuous_summary{suffix}.png"
    if summary.exists():
        from datetime import datetime
        summary=summary.with_name(summary.stem+datetime.now().strftime('_%Y%m%d_%H%M%S_%f')+'.png')
    update(len(data["time"])-1)
    fig.savefig(summary, dpi=140)
    artifacts = [summary]
    if output_format == "auto":
        output_format = "mp4" if FFMpegWriter.isAvailable() else "gif"
    try:
        if output_format != "none":
            if output_format == "mp4":
                writer = FFMpegWriter(fps=fps, bitrate=600)
            else:
                # Pillow retains frames in memory. Bound fallback memory to at
                # most 240 low-resolution frames; preserve elapsed replay time.
                if len(indices) > 240:
                    indices = indices[np.linspace(0, len(indices)-1, 240).astype(int)]
                    fps = len(indices) / max(data["time"][-1], 1/fps)
                    print(f"GIF fallback: reduced to {fps:.2f} fps (240-frame memory limit)")
                writer = PillowWriter(fps=fps)
            movie = output / f"continuous_replay{suffix}.{output_format}"
            if movie.exists():
                from datetime import datetime
                movie=movie.with_name(movie.stem+datetime.now().strftime('_%Y%m%d_%H%M%S_%f')+movie.suffix)
            # Direct frame streaming, no intermediate PNG sequence.
            with writer.saving(fig, str(movie), dpi=80):
                for index in indices:
                    update(index)
                    writer.grab_frame()
            artifacts.append(movie)
    except Exception as exc:
        print(f"Animation unavailable ({type(exc).__name__}: {exc}); static summary saved.")
    finally:
        plt.close(fig)
    return artifacts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--fps", type=float)
    parser.add_argument("--local-xy", action="store_true", help="crop XY around actual path, preserving equal scale")
    parser.add_argument("--format", choices=("auto", "mp4", "gif", "none"), default="none")
    parser.add_argument("--view", choices=("global", "target", "probe"), help="equal XY framing; unknown targets use trajectory")
    parser.add_argument("--components", action="store_true", help="show recorded RAW simulation components if available")
    parser.add_argument("--debug", action="store_true", help="show shared arrow legend, velocity and internal diagnostics")
    parser.add_argument('--true-tangent', action='store_true', help='display-only model tangent comparison (simulation only)')
    args = parser.parse_args()
    for path in render(args.run_dir, fps=args.fps, output_format=args.format, local_xy=args.local_xy, view=args.view, components=args.components, debug=args.debug, true_tangent=args.true_tangent):
        print(path)


if __name__ == "__main__":
    main()
