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
            'object':'#d87900','friction':'#885522','background':'#777777','noise':'#dd55aa'}
    def __init__(self,axis):
        self.axis=axis
        self.arrows={key:axis.annotate('',xy=(0,0),xytext=(0,0),arrowprops=dict(arrowstyle='->',color=color,lw=1.5))
                     for key,color in self.colors.items()}
        self.force_bar,=axis.plot([],[],color=self.colors['measured'],lw=2)
        self.speed_bar,=axis.plot([],[],color=self.colors['command'],lw=2)
        self.force_label=axis.text(0,0,'1 N',fontsize=7,color=self.colors['measured'])
        self.speed_label=axis.text(0,0,'1 mm/s',fontsize=7)

    def draw(self,xy,force,command,actual,tangent,inward,*,valid,components=None):
        vectors=dict(measured=np.asarray(force)*FORCE_MM_PER_N,command=np.asarray(command)*VELOCITY_MM_PER_MM_S,
                     actual=np.asarray(actual)*VELOCITY_MM_PER_MM_S)
        for name,direction in [('tangent',tangent),('inward',inward)]:
            length=np.linalg.norm(direction)
            vectors[name]=np.asarray(direction)/length*DIRECTION_LENGTH_MM if valid and length>0 else np.zeros(2)
        for key in ('object','friction','background','noise'):
            vectors[key]=np.asarray((components or {}).get(key,[np.nan,np.nan]))*FORCE_MM_PER_N
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
