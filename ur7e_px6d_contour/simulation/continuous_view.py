"""Display-only scales and equal-XY viewport; never called by a control policy."""
import numpy as np

FORCE_MM_PER_N = 8.0
VELOCITY_MM_PER_MM_S = 12.0
DIRECTION_LENGTH_MM = 8.0


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
    """Force and speed lengths use separate FIXED data scales, never normalized."""
    colors={'command':'black','actual':'#9933aa','measured':'#1762d2','tangent':'#008855','inward':'#b51d3c',
            'boundary':'#1762d2','robot_estimate':'#d32f2f','object':'#d87900','friction':'#885522','background':'#777777','noise':'#dd55aa'}
    def __init__(self,axis):
        self.axis=axis
        self.arrows={key:axis.annotate('',xy=(0,0),xytext=(0,0),arrowprops=dict(arrowstyle='->',color=color,lw=1.5))
                     for key,color in self.colors.items()}
        self.arrows['robot_estimate'].arrow_patch.set_linestyle('--')
        self.boundary_text=axis.text(.02,.97,'',transform=axis.transAxes,va='top',fontsize=9,color='#1762d2')
        self.robot_text=axis.text(.02,.93,'',transform=axis.transAxes,va='top',fontsize=9,color='#d32f2f')
        self.force_bar,=axis.plot([],[],color=self.colors['measured'],lw=2)
        self.speed_bar,=axis.plot([],[],color=self.colors['command'],lw=2)
        self.force_label=axis.text(0,0,'1 N',fontsize=7,color=self.colors['measured'])
        self.speed_label=axis.text(0,0,'1 mm/s',fontsize=7)

    def draw(self,xy,force,command,actual,tangent,inward,*,valid,components=None,physical=None,debug=False):
        vectors=dict(measured=np.asarray(force)*FORCE_MM_PER_N,command=np.asarray(command)*VELOCITY_MM_PER_MM_S,
                     actual=np.asarray(actual)*VELOCITY_MM_PER_MM_S)
        for name,direction in [('tangent',tangent),('inward',inward)]:
            length=np.linalg.norm(direction)
            vectors[name]=np.asarray(direction)/length*DIRECTION_LENGTH_MM if valid and length>0 else np.zeros(2)
        for key in ('object','friction','background','noise'):
            vectors[key]=np.asarray((components or {}).get(key,[np.nan,np.nan]))*FORCE_MM_PER_N
        for key in list(vectors):
            if not debug:vectors[key]=np.zeros(2)
        physical=physical or dict(blue=None,red=None,blue_label='Boundary contact (model)')
        for name,key in [('boundary','blue'),('robot_estimate','red')]:
            value=physical.get(key)
            vectors[name]=np.asarray(value)*FORCE_MM_PER_N if value is not None else np.full(2,np.nan)
        def magnitude(value):
            return 'unavailable' if value is None or not np.isfinite(value).all() else f'{np.linalg.norm(value):.2f} N'
        self.boundary_text.set_text(physical['blue_label']+': '+magnitude(physical.get('blue')))
        self.robot_text.set_text('Robot force (estimate): '+magnitude(physical.get('red')))
        self.speed_bar.set_visible(debug);self.speed_label.set_visible(debug)
        for key,vector in vectors.items():
            arrow=self.arrows[key]
            arrow.set_visible(bool(np.isfinite(vector).all() and np.linalg.norm(vector)>1e-10))
            arrow.set_position(xy);arrow.xy=np.asarray(xy)+np.nan_to_num(vector)
        lo=np.array([self.axis.get_xlim()[0],self.axis.get_ylim()[0]])
        span=np.array([np.ptp(self.axis.get_xlim()),np.ptp(self.axis.get_ylim())])
        for i,(bar,label,length) in enumerate([(self.force_bar,self.force_label,FORCE_MM_PER_N),
                                             (self.speed_bar,self.speed_label,VELOCITY_MM_PER_MM_S)]):
            origin=lo+span*[.04,.05+i*.065]
            bar.set_data([origin[0],origin[0]+length],[origin[1],origin[1]])
            label.set_position(origin+[length+span[0]*.01,0])


def force_demonstration(row, metadata, *, simulated, previous_velocity=None):
    """Resolve recorded physical diagnostics; never infer a sign from control hand/sign.

    Older/unidentified logs remain readable but cannot claim a physical force.
    Real confirmation is a separate recorded physical convention, not the existing
    targetward force_direction_sign used by the continuous controller.
    """
    label='Boundary contact (model)' if simulated else 'Measured environment resultant'
    result=dict(blue=None,red=None,blue_label=label,note='Physical force unavailable: missing/unconfirmed convention')
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
        result['blue']=vector('sim_boundary_physical_fx','sim_boundary_physical_fy')
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
    result['note']=('Transient: estimate approximate. ' if transient else '')+note
    if result['blue'] is None or result['red'] is None:result['note']+='; unavailable data'
    return result
