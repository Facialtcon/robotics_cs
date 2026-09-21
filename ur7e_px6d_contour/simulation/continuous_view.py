"""Display-only scales and equal-XY viewport; never called by a control policy."""
import numpy as np

FORCE_MM_PER_N = 8.0
VELOCITY_MM_PER_MM_S = 12.0
DIRECTION_LENGTH_MM = 8.0
NORMAL_REACTION_LABEL = '法向反力 / Normal reaction'
NORMAL_REACTION_NOTE = '法向反力示意；摩擦分量未绘制'

CJK_FONT = ['Noto Sans CJK JP', 'DejaVu Sans']
# One source for arrows, curve/trajectory lines, proxies and diagnostic names.
DISPLAY_STYLES = {
    'boundary': dict(color='#1762d2', linestyle='-', name=NORMAL_REACTION_LABEL, unit='N'),
    'robot_estimate': dict(color='#d32f2f', linestyle='--', name='机器人作用力估计', unit='N'),
    'measured': dict(color='#e07800', linestyle='-', name='处理后力信号', unit='N'),
    'tangent': dict(color='#008855', linestyle='-', name='估计切向', unit='单位方向'),
    'inward': dict(color='#00a6b2', linestyle='-', name='估计朝内方向', unit='单位方向'),
    'command': dict(color='black', linestyle='-', name='命令速度', unit='mm/s'),
    'actual': dict(color='#9933aa', linestyle='-', name='实际TCP速度', unit='mm/s'),
    'executed': dict(color='#444444', linestyle='-', name='已执行轨迹', unit='mm'),
    'true_tangent': dict(color='#888888', linestyle='--', name='真实切向对照（仅仿真）', unit='局部直线'),
    'object': dict(color='#967000', linestyle=':', name='模型法向合成分量（朝内）', unit='N'),
    'friction': dict(color='#885522', linestyle='--', name='表面摩擦分量', unit='N'),
    'background': dict(color='#596e77', linestyle='-.', name='颗粒背景分量', unit='N'),
    'noise': dict(color='#dd55aa', linestyle=':', name='测量噪声分量', unit='N'),
    'fx': dict(color='#e07800', linestyle='--', name='处理后 Fx', unit='N'),
    'fy': dict(color='#e07800', linestyle=':', name='处理后 Fy', unit='N'),
    'fxy': dict(color='#e07800', linestyle='-', name='处理后 Fxy', unit='N'),
    'reference': dict(color='#555555', linestyle='--', name='F_ref', unit='N'),
    'error': dict(color='#b38921', linestyle='-.', name='F_error', unit='N'),
    'raw': dict(color='#596e77', linestyle=':', name='Raw Fxy', unit='N'),
    'vt': dict(color='#008855', linestyle='--', name='v_t', unit='mm/s'),
    'vn': dict(color='#00a6b2', linestyle=':', name='v_n', unit='mm/s'),
}
COMPONENT_KEYS = ('object', 'friction', 'background', 'noise')
ARROW_KEYS = ('boundary', 'robot_estimate', 'measured', 'tangent', 'inward', 'command', 'actual')
FORCE_CURVES = ('fx', 'fy', 'fxy', 'reference', 'error')
SPEED_CURVES = ('command', 'actual', 'vt', 'vn')


def display_label(key, *, simulated=True, units=True):
    style = DISPLAY_STYLES[key]
    name = style['name']
    if key == 'boundary' and not simulated:
        name = 'Measured environment resultant'
    if key in ('measured', 'fx', 'fy', 'fxy'):
        name += '（仿真合成）' if simulated else '（processed Base）'
    if key == 'raw':
        name += '（模型Base）' if simulated else '（传感器）'
    return name + (f" [{style['unit']}]" if units else '')


def line_style(key, *, simulated=True):
    style = DISPLAY_STYLES[key]
    return dict(color=style['color'], linestyle=style['linestyle'],
                label=display_label(key, simulated=simulated))


def component_note(*, simulated, enabled):
    if not enabled:
        return ''
    if not simulated:
        return 'Components: 不可用（没有可分离的仿真分量）'
    return 'RAW = ' + ' + '.join(DISPLAY_STYLES[k]['name'] for k in COMPONENT_KEYS) + ' [N]'


class XYViewport:
    def __init__(self, axis, bounds, target=None):
        self.axis, self.bounds, self.target = axis, bounds, target
        self.mode, self.follow, self._changing = 'Target', False, False
        axis.set_aspect('equal', adjustable='box')
        axis.callbacks.connect('xlim_changed', self._manual)
        axis.callbacks.connect('ylim_changed', self._manual)

    def _manual(self, axis):
        if not self._changing:
            self.follow = False

    def fit(self, points, padding=5.):
        points=np.asarray(points).reshape(-1,2)
        points=points[np.isfinite(points).all(axis=1)]
        if not len(points): return
        lo,hi=points.min(axis=0),points.max(axis=0)
        span=np.maximum(hi-lo,10.)
        pad=np.maximum(span*.12,padding)
        # Fill the allocated wide XY panel while keeping identical X/Y units.
        box=self.axis.get_position(original=True); width,height=self.axis.figure.get_size_inches()
        ratio=(box.width*width)/(box.height*height)
        half=(hi-lo)/2+pad
        half=np.maximum(half,[half[1]*ratio,half[0]/ratio])
        center=(lo+hi)/2
        self._changing=True
        try:
            self.axis.set_xlim(center[0]-half[0],center[0]+half[0])
            self.axis.set_ylim(center[1]-half[1],center[1]+half[1])
        finally:self._changing=False

    def select(self, mode, xy, path=None):
        self.mode=mode
        if mode=='Global': self.fit(self.bounds,padding=0.)
        elif mode=='Target':
            self.fit(self.target if self.target is not None else (path if path is not None and len(path) else [xy]))
        elif mode=='Probe':self.fit([np.asarray(xy)-8,np.asarray(xy)+8],padding=4.)
        else:raise ValueError('view must be Global, Target or Probe')

    def update(self, xy, *, running):
        if running and self.follow and self.mode=='Probe':self.select('Probe',xy)

    def scroll(self,event):
        if event.inaxes is not self.axis:return
        xlim,ylim=self.axis.get_xlim(),self.axis.get_ylim()
        center=np.array([np.mean(xlim) if event.xdata is None else event.xdata,
                         np.mean(ylim) if event.ydata is None else event.ydata])
        factor=.8 if event.button=='up' else 1.25
        self.axis.set_xlim(center[0]+(np.asarray(xlim)-center[0])*factor)
        self.axis.set_ylim(center[1]+(np.asarray(ylim)-center[1])*factor)


class VectorDisplay:
    """Shared display and cached proxy legend, with no sensor or policy calls."""
    colors = {key: DISPLAY_STYLES[key]['color'] for key in (*ARROW_KEYS, *COMPONENT_KEYS)}

    def __init__(self, axis):
        self.axis = axis
        self.arrows = {key: axis.annotate('', xy=(0, 0), xytext=(0, 0),
            arrowprops=dict(arrowstyle='->', color=self.colors[key],
                            linestyle=DISPLAY_STYLES[key]['linestyle'], lw=1.5)) for key in self.colors}
        self.boundary_text = axis.text(.02, .97, '', transform=axis.transAxes, va='top', fontsize=9,
            color=self.colors['boundary'], fontfamily=CJK_FONT)
        self.robot_text = axis.text(.02, .93, '', transform=axis.transAxes, va='top', fontsize=9,
            color=self.colors['robot_estimate'], fontfamily=CJK_FONT)
        self.force_bar, = axis.plot([], [], color=self.colors['boundary'], lw=2)
        self.speed_bar, = axis.plot([], [], color=self.colors['command'], lw=2)
        self.force_label = axis.text(0, 0, '1 N', fontsize=7, color=self.colors['boundary'])
        self.speed_label = axis.text(0, 0, '1 mm/s', fontsize=7)
        self.true_line, = axis.plot([], [], lw=1.2, **line_style('true_tangent'))
        self.true_line.set_visible(False)
        self.truth_angle, self.truth_note = None, ''
        self.legend, self.legend_keys, self._legend_signature = None, (), None

    def _legend(self, keys, states, *, debug, simulated):
        if not debug:
            if self.legend is not None: self.legend.set_visible(False)
            return
        from matplotlib.lines import Line2D
        from matplotlib.patches import FancyArrowPatch
        from matplotlib.legend_handler import HandlerPatch
        def proxy_arrow(legend, orig_handle, xdescent, ydescent, width, height, fontsize):
            return FancyArrowPatch((0, height/2), (width, height/2), arrowstyle='->',
                mutation_scale=fontsize, color=orig_handle.get_edgecolor(),
                linestyle=orig_handle.get_linestyle(), linewidth=1.5)
        signature = (tuple(keys), simulated)
        if self._legend_signature != signature:
            if self.legend is not None: self.legend.remove()
            handles = []
            for key in keys:
                style = DISPLAY_STYLES[key]
                kw = dict(color=style['color'], linestyle=style['linestyle'], linewidth=1.3)
                handles.append(Line2D([], [], **kw) if key in ('executed', 'true_tangent') else
                               FancyArrowPatch((0, 0), (1, 0), arrowstyle='->', **kw))
            self.legend = self.axis.legend(handles, [display_label(k, simulated=simulated) for k in keys],
                handler_map={FancyArrowPatch: HandlerPatch(patch_func=proxy_arrow)},
                loc='upper left', ncol=3, mode='expand', borderaxespad=0,
                prop={'family': CJK_FONT, 'size': 7}, handlelength=2.4,
                labelspacing=.5, columnspacing=.8, frameon=False)
            self.legend_keys, self._legend_signature = tuple(keys), signature
        self.legend.set_visible(True)
        for key, text in zip(keys, self.legend.get_texts()):
            text.set_text(display_label(key, simulated=simulated) + states.get(key, ''))

    def layout_legend(self):
        """Call after subplot layout. Reserve a band above XY, outside all data."""
        box = self.axis.get_position(original=True)
        fig = self.axis.figure
        if self.legend is not None and self.legend.get_visible():
            # Fixed number of columns; only enabled membership rebuilds proxies.
            rows = (len(self.legend_keys)+2)//3
            reserve = (rows*15+16)/(fig.get_figheight()*72)
            self.legend.set_bbox_to_anchor((box.x0, box.y1-reserve, box.width, reserve), transform=fig.transFigure)
            self.axis.set_position([box.x0, box.y0, box.width, max(.03, box.height-reserve-.012)])
        height_pt = self.axis.get_position().height*fig.get_figheight()*72
        self.boundary_text.set_position((.02, 1-4/max(height_pt,1)))
        self.robot_text.set_position((.02, 1-20/max(height_pt,1)))

    def draw(self, xy, force, command, actual, tangent, inward, *, valid,
             components=None, physical=None, debug=False, simulated=True,
             components_enabled=None, true_tangent=False):
        if components_enabled is None: components_enabled = components is not None
        physical = physical or dict(blue=None, red=None, blue_label=display_label('boundary', simulated=simulated, units=False))
        vectors = dict(measured=np.asarray(force)*FORCE_MM_PER_N,
                       command=np.asarray(command)*VELOCITY_MM_PER_MM_S,
                       actual=np.asarray(actual)*VELOCITY_MM_PER_MM_S)
        for name, direction in [('tangent', tangent), ('inward', inward)]:
            direction = np.asarray(direction)
            length = np.linalg.norm(direction)
            vectors[name] = direction/length*DIRECTION_LENGTH_MM if valid and np.isfinite(length) and length>0 else np.full(2,np.nan)
        for key in COMPONENT_KEYS:
            vectors[key] = np.asarray((components or {}).get(key, [np.nan,np.nan]))*FORCE_MM_PER_N
        for name, key in [('boundary', 'blue'), ('robot_estimate', 'red')]:
            value = physical.get(key)
            vectors[name] = np.asarray(value)*FORCE_MM_PER_N if value is not None else np.full(2,np.nan)
        def magnitude(value):
            return 'unavailable' if value is None or not np.isfinite(value).all() else f'{np.linalg.norm(value):.2f} N'
        self.boundary_text.set_text(physical['blue_label']+': '+magnitude(physical.get('blue')))
        self.robot_text.set_text(display_label('robot_estimate', units=False)+': '+magnitude(physical.get('red')))
        keys = list(ARROW_KEYS)+['executed'] if debug else ['boundary','robot_estimate']
        if debug and components_enabled: keys.extend(COMPONENT_KEYS)
        states = {}
        for key, vector in vectors.items():
            finite = np.isfinite(vector).all()
            nonzero = finite and np.linalg.norm(vector)>1e-10
            states[key] = '' if nonzero else ('（零）' if finite else '（不可用）')
            arrow = self.arrows[key]
            arrow.set_visible(key in keys and bool(nonzero))
            arrow.set_position(xy); arrow.xy = np.asarray(xy)+np.nan_to_num(vector)
        self.truth_angle, self.truth_note = None, ''
        self.true_line.set_visible(False)
        if debug and not simulated:
            self.truth_note = '真实切向对照不可用（无仿真几何真值）'
        if debug and true_tangent and simulated:
            keys.append('true_tangent')
            # Nonzero recorded physical normal establishes valid model contact.
            # Never pass this truth into tangent/command/force feedback above.
            normal = physical.get('blue')
            if normal is not None and np.isfinite(normal).all() and np.linalg.norm(normal)>1e-10:
                normal = np.asarray(normal)/np.linalg.norm(normal)
                truth = np.array([-normal[1], normal[0]])
                points = np.asarray(xy)+np.array([-1.,1.])[:,None]*truth*4
                self.true_line.set_data(points[:,0], points[:,1]); self.true_line.set_visible(True)
                estimate = np.asarray(tangent)
                if valid and np.isfinite(estimate).all() and np.linalg.norm(estimate)>1e-10:
                    self.truth_angle = float(np.degrees(np.arccos(np.clip(abs(np.dot(estimate/np.linalg.norm(estimate),truth)),0,1))))
                angle = '不可用' if self.truth_angle is None else f'{self.truth_angle:.1f}°'
                self.truth_note = f'估计切向与真实切线夹角：{angle}（无方向直线 0–90°，不判断倒退）'
            else:
                states['true_tangent'] = '（不可用）'
                self.truth_note = '真实切向对照不可用（无有效模型接触/法向）'
        self._legend(keys, states, debug=debug, simulated=simulated)
        self.speed_bar.set_visible(debug); self.speed_label.set_visible(debug)
        lo = np.array([self.axis.get_xlim()[0], self.axis.get_ylim()[0]])
        span = np.array([np.ptp(self.axis.get_xlim()), np.ptp(self.axis.get_ylim())])
        for i,(bar,label,length) in enumerate([(self.force_bar,self.force_label,FORCE_MM_PER_N),
                                             (self.speed_bar,self.speed_label,VELOCITY_MM_PER_MM_S)]):
            origin = lo+span*[.04,.05+i*.065]
            bar.set_data([origin[0],origin[0]+length],[origin[1],origin[1]])
            label.set_position(origin+[length+span[0]*.01,0])


def force_demonstration(row, metadata, *, simulated, previous_velocity=None, components_visible=False):
    """Resolve recorded physical diagnostics; never infer a sign from control hand/sign.

    Older/unidentified logs remain readable but cannot claim a physical force.
    Real confirmation is a separate recorded physical convention, not the existing
    targetward force_direction_sign used by the continuous controller.
    """
    label=display_label('boundary', simulated=simulated, units=False)
    prefix=NORMAL_REACTION_NOTE+'\n' if simulated and not components_visible else ''
    result=dict(blue=None,red=None,blue_label=label,note=prefix+'Physical force unavailable: missing/unconfirmed convention')
    def vector(x,y):
        try:
            value=np.array([float(row.get(x,np.nan)),float(row.get(y,np.nan))])
            return value if np.isfinite(value).all() else None
        except (ValueError,TypeError):return None
    if metadata.get('schema_version')!=1 or metadata.get('frame')!='Base':return result
    if metadata.get('estimate_method')!='quasistatic_planar_balance':return result
    if simulated:
        if (metadata.get('force_convention')!='legacy_inward_normal_to_outward_physical_v1' or
                metadata.get('force_source')!='synthetic_contact_and_drag_model' or
                str(row.get('physical_force_available')) not in ('1','1.0', 'True')):return result
        # Recorded outward normal * actual normal load. Never project the
        # boundary resultant: its magnitude also includes surface friction.
        result['blue']=vector('sim_normal_physical_fx','sim_normal_physical_fy')
        # Still balances the FULL boundary + background; not merely -blue.
        result['red']=vector('sim_robot_estimate_fx','sim_robot_estimate_fy')
        note='Quasi-static estimate; inertia omitted'
    else:
        if not (metadata.get('force_convention')=='environment_on_probe' and
                metadata.get('force_source')=='processed_wrench' and
                metadata.get('physical_sign_confirmed') is True and metadata.get('base_frame_confirmed') is True and
                metadata.get('calibration_reference')):return result
        result['blue']=vector('dfx','dfy')
        result['red']=None if result['blue'] is None else -result['blue']
        note='Quasi-static estimate; measured noise / other-force uncertainty'
    velocity=vector('tcp_vx','tcp_vy')
    transient=(row.get('current_state')!='CONTINUOUS_TRACKING' or
               row.get('direction_phase')!='TRACK' or velocity is None or previous_velocity is None or
               not np.isfinite(previous_velocity).all() or np.linalg.norm(velocity-previous_velocity)>1e-6)
    result['note']=prefix+('Transient: estimate approximate. ' if transient else '')+note
    if result['blue'] is None or result['red'] is None:result['note']+='; unavailable data'
    return result
