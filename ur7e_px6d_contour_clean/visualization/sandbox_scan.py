"""Equal-scale Base XY preview, optionally overlaid with the existing actual TCP log."""
from pathlib import Path
import json
import subprocess
import sys

import numpy as np

from visualization.xy_grid import square_xy_ticks


def direction_text(delta):
    """Base XY direction of an actual ordered segment, including its tilt."""
    x, y = np.asarray(delta, dtype=float)
    if abs(y) >= abs(x):
        main, minor = ('向上' if y >= 0 else '向下'), ('右' if x >= 0 else '左')
        return main+('偏'+minor if abs(x) > .02*abs(y) else '')
    main, minor = ('向右' if x >= 0 else '向左'), ('上' if y >= 0 else '下')
    return main+('偏'+minor if abs(y) > .02*abs(x) else '')


def _inside_polygon(point, polygon):
    vertices = np.asarray(polygon)
    edges = np.roll(vertices,-1,axis=0)-vertices
    area = np.sum(vertices[:,0]*np.roll(vertices[:,1],-1)-vertices[:,1]*np.roll(vertices[:,0],-1))
    offsets = np.asarray(point)-vertices
    return bool(np.all(np.sign(area)*(edges[:,0]*offsets[:,1]-edges[:,1]*offsets[:,0]) >= -1e-12))


def describe_route(plan):
    """Read execution order directly from coverage_xy; never regenerate/reorder it."""
    route = np.asarray(plan['coverage_xy'], dtype=float)
    segments = []
    for i,(a,b) in enumerate(zip(route,route[1:])):
        scan = i % 2 == 0
        kind = 'scan' if scan else 'connector'
        number = i+1
        prefix = chr(0x2460+i) if i < 7 else str(number)
        action = f'扫描{i//2+1}' if scan else '横移换行'
        segments.append(dict(index=i,kind=kind,start_xy=a.tolist(),end_xy=b.tolist(),
            direction=direction_text(b-a),label=f'{prefix} {action}：{direction_text(b-a)}'))
    rectangle = np.asarray(plan['effective_corners_xy'])
    axes = np.array([rectangle[1]-rectangle[0],rectangle[3]-rectangle[0]])
    lengths = np.linalg.norm(axes,axis=1)
    scan_axis = int(np.argmax(np.abs((axes/lengths[:,None]) @ (route[1]-route[0]))))
    side = ('等长边' if np.isclose(*lengths,rtol=1e-9,atol=1e-12) else
            '长边' if lengths[scan_axis] > lengths[1-scan_axis] else '短边')
    progression = route[2]-route[1]
    spacing = float(np.linalg.norm(progression))
    count = len(route)//2
    if not _inside_polygon(plan['p0_tcp_pose'][:2],plan['raw_corners_xy']):
        location,text = 'outside_raw','原始沙箱外'
    elif not _inside_polygon(plan['p0_tcp_pose'][:2],rectangle):
        location,text = 'inside_raw_outside_effective','沙箱内，但在有效扫描区域外'
    else:
        location,text = 'inside_effective','有效扫描区域内'
    summary = f'沿矩形{side}往返，{direction_text(progression)}逐条推进，共 {count} 条线，实际间距 {spacing*1000:.3f} 毫米。'
    return dict(summary=summary,scan_side=side,progression=direction_text(progression),
        line_count=count,actual_spacing_m=spacing,p0_location=location,p0_location_text=text,
        segments=segments,first_steps=segments[:7])


def _preview_font():
    from matplotlib import font_manager
    available = {f.name for f in font_manager.fontManager.ttflist}
    names = ('Noto Sans CJK SC','Noto Sans CJK JP','Microsoft YaHei','SimHei',
             'WenQuanYi Micro Hei','Droid Sans Fallback','DejaVu Sans')
    return font_manager.FontProperties(family=next(n for n in names if n in available))


def _arrow(ax,a,b,color,*,width=1.2,head=11):
    ax.annotate('',xy=b,xytext=a,zorder=8,
        arrowprops=dict(arrowstyle='-|>',color=color,lw=width,mutation_scale=head))


def _draw_geometry(ax,plan):
    fitted_label = ('四角最小二乘拟合' if plan.get('geometry',{}).get(
        'fitted_workspace',{}).get('fit_method') == 'least_squares' else '拟合矩形')
    for key,label,color,style in (
        ('raw_corners_xy','原始四角 / 四边形','#777777',':'),
        ('rectified_corners_xy',fitted_label,'#ae96bd','--'),
        ('effective_corners_xy','有效扫描边界','#7f9f7f','--')):
        xy = np.asarray(plan[key])*1000
        ax.plot(*np.vstack((xy,xy[0])).T,color=color,ls=style,lw=1.1,zorder=1,
                marker='o' if key=='raw_corners_xy' else None,ms=3,label=label)
    ax.set_aspect('equal',adjustable='box')
    square_xy_ticks(ax)
    ax.grid(True,alpha=.16)
    ax.set_xlabel('Base X [mm]');ax.set_ylabel('Base Y [mm]')


def plot_plan(plan, *, actual_xy=None, contacts=None, interactive=False):
    import textwrap
    from matplotlib.transforms import offset_copy
    if interactive:
        import matplotlib.pyplot as plt
        fig = plt.figure(figsize=(16,10),layout='constrained')
    else:
        # The robot/terminal process only renders offscreen. GUI owns its loop.
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        fig = Figure(figsize=(16,10),layout='constrained')
        FigureCanvasAgg(fig)
    font = _preview_font()
    grid = fig.add_gridspec(3,2,width_ratios=(1.25,1.),height_ratios=(1.35,1.,.43))
    ax = fig.add_subplot(grid[:,0])
    information = fig.add_subplot(grid[0,1]);information.set_axis_off()
    zoom = fig.add_subplot(grid[1,1])
    legend = fig.add_subplot(grid[2,1]);legend.set_axis_off()
    info = describe_route(plan)
    route = np.asarray(plan['coverage_xy'])*1000
    colors = ('#d73027','#7b3294','#008577','#b65c00')
    connector_color = '#e88714'
    _draw_geometry(ax,plan)
    ax.set_title('固定起点沙箱覆盖：数字为实际执行顺序',fontproperties=font,fontsize=14,pad=12)
    for segment in info['segments']:
        i = segment['index'];a,b = route[i],route[i+1]
        scan = segment['kind']=='scan'
        highlighted = scan and i//2 < 4
        color = colors[i//2] if highlighted else '#47799e' if scan else connector_color
        ax.plot(*np.array([a,b]).T,color=color,lw=2.2 if highlighted else .85 if scan else 1.8,
                alpha=1 if highlighted or not scan else .65,zorder=5 if scan else 6,
                label=('扫描段（前四条彩色突出）' if i==0 else '换行连接段' if i==1 else None))
        # Every scan gets a centered arrow, including adjacent reverse rows.
        _arrow(ax,a+.40*(b-a),a+.60*(b-a),color,width=1.5 if highlighted else 1.,head=12 if highlighted else 9)
        if i < 7:
            fraction = .2+.17*(i//2) if scan else .5
            point = a+fraction*(b-a)
            ax.annotate(chr(0x2460+i),point,xytext=(7,3 if scan else -16),textcoords='offset points',
                fontproperties=font,fontsize=12,color=color,zorder=10,
                bbox=dict(facecolor='white',edgecolor='none',alpha=.9,pad=1))
    entry = np.asarray(plan['entry_xy'])*1000
    ax.plot(*entry.T,color='#a30089',ls='-.',lw=1.8,zorder=7,label='P0 → 覆盖起点（进入路线）')
    if np.linalg.norm(entry[1]-entry[0])>1e-9:
        _arrow(ax,entry[0]+.25*(entry[1]-entry[0]),entry[0]+.65*(entry[1]-entry[0]),'#a30089',head=13)
    for xy,label,marker,color,offset in (
        (entry[0],'P0','*','#a30089',(-8,-24)),
        (route[0],'覆盖起点','s','#199b31',(12,18)),
        (route[-1],'覆盖终点','X','#c21818',(12,-22))):
        ax.scatter(*xy,marker=marker,s=110,color=color,zorder=10,label=label)
        ax.annotate(label,xy,xytext=offset,textcoords='offset points',fontproperties=font,
            fontsize=11,color=color,zorder=10,
            bbox=dict(facecolor='white',edgecolor='none',alpha=.9,pad=2),
            arrowprops=dict(arrowstyle='-',color=color,lw=.8))
    if actual_xy is not None and len(actual_xy):
        ax.plot(*(np.asarray(actual_xy)*1000).T,color='black',lw=.9,alpha=.65,label='实际 TCP')
    if contacts:
        ax.scatter(*(np.asarray(contacts)*1000).T,marker='*',s=150,color='magenta',label='首次确认接触')

    # Right-side text lives in its own layout-managed axes, with explicit wraps.
    entry_text = ('抬升 → 经 P0 上方平移 → 下降至覆盖起点' if plan['entry_kind']=='lift_translate_descend' else
                  '下降至 P0 扫描高度 → 平面进入覆盖起点')
    lines = ['本次路线']+textwrap.wrap(info['summary'],width=30)+[
        'P0 位置：'+info['p0_location_text'],entry_text,
        f"起点 XY：({route[0,0]:.3f}, {route[0,1]:.3f}) mm",
        f"终点 XY：({route[-1,0]:.3f}, {route[-1,1]:.3f}) mm",
        f"扫描高度 {plan['scan_z']*1000:.3f} mm；P0 示教高度 {plan['p0_tcp_pose'][2]*1000:.3f} mm",
        f"搜索 {plan['search_speed_mps']*1000:g} mm/s；贴边基准 {plan['tracking_base_speed_mps']*1000:g} mm/s",
        f"边界余量 {plan['boundary_margin_m']*1000:.4f} mm；覆盖路程 {plan['coverage_length_m']:.3f} m"]
    geometry = plan.get('geometry')
    if geometry:
        fit = geometry['fitted_workspace']['diagnostics']
        lines.append(f"拟合 RMS / 最大偏移：{fit['fit_rms_xy_m']*1000:.3f} / {fit['fit_max_xy_m']*1000:.3f} mm")
    lines.append('前四条扫描线的实际顺序：')
    # Point-based line offsets keep rows legible when the window is resized.
    # Constrained layout reserves the full text extent, including any part
    # below this axes, instead of compressing text into overlapping rows.
    def row_transform(row,scale=1.):
        return offset_copy(information.transAxes,fig=fig,y=-row*14*scale,units='points')
    for row,text in enumerate(lines):
        artist = information.text(0,1,text,transform=row_transform(row),va='top',
            fontproperties=font,fontsize=10.8,color='#222222')
        artist.set_gid('preview-info')
    for row,segment in enumerate(info['first_steps'],start=len(lines)):
        i=segment['index'];color=colors[i//2] if segment['kind']=='scan' else connector_color
        artist = information.text(0,1,segment['label'],transform=row_transform(row),va='top',
            fontproperties=font,fontsize=11,color=color)
        artist.set_gid('preview-order')

    # First actual turn: keep its exact approach, connector, and reverse leg.
    _draw_geometry(zoom,plan)
    for i in range(3):
        a,b=route[i],route[i+1]
        color=colors[i//2] if i%2==0 else connector_color
        zoom.plot(*np.array([a,b]).T,color=color,lw=2.5,zorder=5)
    b,c=route[1],route[2]
    padding=max(1.5*np.linalg.norm(c-b),.04*np.linalg.norm(route[1]-route[0]))
    zoom.set_xlim(min(b[0],c[0])-padding,max(b[0],c[0])+padding)
    zoom.set_ylim(min(b[1],c[1])-padding,max(b[1],c[1])+padding)
    approach=(b-route[0])/np.linalg.norm(b-route[0]);reverse=(route[3]-c)/np.linalg.norm(route[3]-c)
    for i,(a,z) in enumerate(((b-approach*padding*.85,b-approach*padding*.1),
                             (b+.1*(c-b),b+.9*(c-b)),
                             (c+reverse*padding*.1,c+reverse*padding*.85))):
        color=colors[i//2] if i%2==0 else connector_color
        _arrow(zoom,a,z,color,width=2,head=14)
        zoom.annotate(chr(0x2460+i),(a+z)/2,xytext=(-16,3) if i==0 else (5,-18) if i==1 else (10,3),
                      textcoords='offset points',fontproperties=font,fontsize=12,color=color,
                      bbox=dict(facecolor='white',edgecolor='none',alpha=.85,pad=1))
    zoom.scatter(*np.array([b,c]).T,s=30,color=connector_color,zorder=8)
    zoom.set_title('首个折返处放大\n①到达端点 → ②横移 → ③反向扫描',fontproperties=font,fontsize=12)
    # Geometry at full extent would otherwise dominate inset autoscaling.
    zoom.set_aspect('equal',adjustable='box')
    handles,labels=ax.get_legend_handles_labels()
    def draw_legend(scale):
        legend_font=font.copy();legend_font.set_size(9*scale)
        legend.legend(handles,labels,loc='center left',ncol=2,prop=legend_font,frameon=False,
                      columnspacing=1.4,handlelength=2.4)
    draw_legend(1.)
    # Resize text together with the canvas, rather than retaining desktop-size
    # fonts/styles while the panels become smaller, including HiDPI Tk canvases.
    from matplotlib.text import Text, Annotation
    text_sizes=[(text,text.get_fontsize()) for text in fig.findobj(Text)]
    annotations=fig.findobj(Annotation)
    offsets=[(text,np.asarray(text.xyann)) for text in annotations if text.anncoords=='offset points']
    arrow_sizes=[(text,text.arrowprops.get('mutation_scale',text.get_fontsize()),text.arrow_patch.get_linewidth())
                 for text in annotations if text.arrow_patch is not None]
    line_sizes=[(line,line.get_linewidth(),line.get_markersize()) for axes in (ax,zoom) for line in axes.lines]
    marker_sizes=[(collection,collection.get_sizes().copy()) for axes in (ax,zoom) for collection in axes.collections]
    def resize(_):
        scale=min(fig.get_size_inches()/[16,10])
        for text,size in text_sizes:
            text.set_fontsize(size*scale)
        for row,text in enumerate(information.texts):
            text.set_transform(row_transform(row,scale))
        for text,offset in offsets:
            text.xyann=offset*scale
        for text,size,width in arrow_sizes:
            text.arrowprops['mutation_scale']=size*scale
            text.arrow_patch.set_linewidth(width*scale)
        for line,width,marker in line_sizes:
            line.set_linewidth(width*scale);line.set_markersize(marker*scale)
        for collection,sizes in marker_sizes:
            collection.set_sizes(sizes*scale**2)
        for axes in (ax,zoom):
            axes.tick_params(labelsize=10*scale,width=.8*scale,length=3.5*scale,
                             pad=3.5*scale,grid_linewidth=.8*scale)
            axes.xaxis.labelpad=axes.yaxis.labelpad=4*scale
            for spine in axes.spines.values():
                spine.set_linewidth(.8*scale)
        draw_legend(scale)
        fig.get_layout_engine().set(h_pad=.04167*scale,w_pad=.04167*scale)
    fig.canvas.mpl_connect('resize_event',resize)
    return fig


def save_preview(plan, directory, *, show=False):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    plan_path = directory/'sandbox_scan_plan.json'
    plan_path.write_text(json.dumps(plan, ensure_ascii=False, indent=2), encoding='utf-8')
    if plan.get('geometry'):
        (directory/'sandbox_scan_geometry.json').write_text(
            json.dumps(plan['geometry'], ensure_ascii=False, indent=2), encoding='utf-8')
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
