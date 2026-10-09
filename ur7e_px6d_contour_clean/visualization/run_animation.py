#!/usr/bin/env python3
"""On-demand CSV playback. No video export, device connection or motion code."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from visualization.run_plots import (CUTOFF_RUN, PHASES, COLORS, configure_xy,
                                     display_indices, enabled_for_run, load_trace)


class PlaybackClock:
    """Wall-monotonic playback time; CSV timestamps decide which data is visible."""
    def __init__(self, duration, *, clock=time.monotonic):
        self.duration = float(duration)
        self.clock = clock
        self.position = 0.
        self.speed = 1.
        self.playing = self.duration > 0.
        self.last_wall = clock()

    def advance(self):
        now = self.clock()
        elapsed = max(0., now-self.last_wall)
        self.last_wall = now
        if self.playing:
            self.position = min(self.duration, self.position+elapsed*self.speed)
            if self.position >= self.duration:
                self.playing = False
        return self.position

    def toggle(self):
        self.advance()
        if self.position < self.duration:
            self.playing = not self.playing

    def restart(self):
        self.position = 0.
        self.last_wall = self.clock()
        self.playing = self.duration > 0.

    def set_speed(self, speed):
        if speed not in (.5, 1., 2., 4.):
            raise ValueError('Playback speed must be 0.5, 1, 2 or 4.')
        self.advance()
        self.speed = float(speed)


class RunAnimation:
    def __init__(self, trace, *, clock=time.monotonic, fps=30, animate=True):
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from matplotlib.widgets import Button, Slider, RadioButtons

        self.trace = trace
        self.clock = PlaybackClock(trace.duration, clock=clock)
        self.indices = display_indices(trace)
        self.figure = plt.figure(figsize=(18, 9.4), facecolor='white')
        self.xy = self.figure.add_axes([.06, .43, .34, .46])
        self.force_axis = self.figure.add_axes([.57, .65, .36, .22])
        self.velocity_axis = self.figure.add_axes([.57, .31, .36, .22], sharex=self.force_axis)
        bounds = trace.xy_mm if trace.target_mm is None else np.r_[trace.xy_mm, trace.target_mm]
        configure_xy(self.xy, bounds, 'Recorded TCP trajectory')
        if trace.target_mm is not None:
            self.xy.plot(*trace.target_mm.T, '--', color='.25', label='Known cube')
        prefix = 'SYNTHETIC / illustrative — ' if trace.config.get('experiment', {}).get('kind') == 'synthetic_demo' else ''
        self.figure.suptitle(prefix+trace.run_dir.name, fontsize=17, y=.98)
        self.complete_path, = self.xy.plot([], [], color='#cbd1d6', lw=1.)
        self.path_lines = [self.xy.plot([], [], color=color, lw=2.2, label=name)[0]
                           for name, color in zip(PHASES, COLORS)]
        self.xy.plot(*trace.xy_mm[0], 'o', color='#25364a', ms=7)
        self.xy.annotate('Start', trace.xy_mm[0], xytext=(-28, 22), textcoords='offset points', fontsize=10)
        self.current_tcp, = self.xy.plot([], [], 'o', color='#ffd84c', mec='#182633', mew=1.3, ms=9, zorder=6)
        self.first_contact, = self.xy.plot([], [], '*', color='#9b287b', ms=12, zorder=5)
        self.final_stop, = self.xy.plot([], [], 'X', color='#b72a33', ms=10, zorder=5)
        self.first_label = self.xy.annotate('Contact threshold' if trace.contact_source.startswith('threshold') else 'Contact confirmed', (0, 0), xytext=(10, 9),
                                            textcoords='offset points', fontsize=11, color='#9b287b')
        self.stop_label = self.xy.annotate('Final stop', (0, 0), xytext=(10, -17),
                                           textcoords='offset points', fontsize=11, color='#b72a33')
        self.xy.legend(loc='upper left', bbox_to_anchor=(1.01, 1.), frameon=False, fontsize=8)
        self.force_lines = [self.force_axis.plot([], [], color=color, lw=1.2, label=label)[0]
                            for label, color in zip(('Fx', 'Fy', 'Fxy'), ('#3975ae', '#c65050', '#16836b'))]
        values = np.r_[trace.force.ravel(), trace.unfiltered_force.ravel(),
                       trace.config.get('policy', {}).get('contact_threshold', np.nan)]
        values = values[np.isfinite(values)]
        low, high = (float(values.min()), float(values.max())) if len(values) else (-1., 1.)
        pad = max((high-low)*.18, .2)
        self.force_axis.set(xlim=(0., max(trace.duration, 1.)), ylim=(low-pad, high+pad),
                            ylabel='Force [N]', title='Base horizontal force')
        self.force_axis.set_autoscale_on(False)
        self.force_axis.tick_params(labelsize=12)
        self.force_axis.xaxis.label.set_size(14)
        self.force_axis.yaxis.label.set_size(14)
        self.force_axis.title.set_size(18)
        self.force_axis.grid(alpha=.25)
        self.force_axis.legend(loc='upper right', framealpha=.8, fontsize=11)
        self.cursor = self.force_axis.axvline(0., color='#25364a', lw=1., ls='--')
        from visualization.contact_report import (collision_text, command_at, event_lines, EVENT_STYLE,
            planar_speed, SIGNAL_COLORS)
        self.planar_speed = planar_speed
        self.command_at = command_at
        self.actual_speeds = planar_speed(trace.actual_velocity)
        self.sent_speeds = planar_speed(trace.command_velocity)
        # The faint complete curves make future moments available to click while playing.
        for i, color in enumerate(SIGNAL_COLORS):
            self.force_axis.plot(trace.time[self.indices], trace.force[self.indices, i], color=color, alpha=.16, lw=.8)
        if np.isfinite(trace.unfiltered_force).any():
            self.force_axis.plot(trace.time[self.indices], trace.unfiltered_force[self.indices, 2],
                                 color='.55', alpha=.5, lw=.7, label='Pre-filter Fxy')
        threshold = trace.config.get('policy', {}).get('contact_threshold')
        if threshold is not None:
            self.force_axis.axhline(float(threshold), color='#9b287b', ls='--', label=f'Contact {threshold:g} N')
        lost = trace.config.get('continuous_tracking', {}).get('contact_lost_threshold')
        if lost is not None:
            self.force_axis.axhline(float(lost), color='#e68621', ls='-.', label=f'Lost {lost:g} N')
        self.force_axis.legend(loc='lower left', bbox_to_anchor=(0,1.02), ncol=3, fontsize=8, frameon=False)
        event_lines(self.force_axis, trace); event_lines(self.velocity_axis, trace)
        self.velocity_lines = []
        for i, (component, color) in enumerate(zip(('vx','vy','XY'), SIGNAL_COLORS)):
            actual, = self.velocity_axis.plot(trace.actual_time[self.indices], self.actual_speeds[self.indices, i],
                color=color, lw=1.2, label=f'Actual {component}')
            sent, = self.velocity_axis.step(trace.command_time, self.sent_speeds[:, i], where='post',
                color=color, ls='--', lw=1., label=f'Sent {component}')
            self.velocity_lines.append((actual, sent))
        self.velocity_axis.set(ylabel='Base speed [mm/s]', xlabel='Elapsed time [s]')
        self.velocity_axis.grid(alpha=.25)
        self.velocity_axis.legend(handles=[line for pair in self.velocity_lines for line in pair],
                                  loc='lower left', bbox_to_anchor=(0,1.02), ncol=3, fontsize=8, frameon=False)
        self.velocity_cursor = self.velocity_axis.axvline(0., color='#25364a', lw=1., ls='--')
        self.force_axis.set_title('Base force [N]', fontsize=13, y=1.31, pad=0)
        self.velocity_axis.set_title('Actual (solid) / sent command (dashed)', fontsize=13, y=1.31, pad=0)
        self.event_markers = []
        for kind in ('CONTACT_LOST', 'REACQUIRED'):
            style = EVENT_STYLE[kind]
            points = [e for e in trace.events if e['kind'] == kind and np.isfinite(e['xy_mm']).all()]
            artist = self.xy.scatter([], [], color=style[1], marker=style[2], s=55, zorder=6, label=style[0])
            self.event_markers.append((artist, points))
        self.xy.legend(loc='upper left', bbox_to_anchor=(1.01, 1.), frameon=False, fontsize=8)
        # Fixed independent scales; force and actual velocity are never added together.
        max_f = np.nanmax(trace.force[:,2]) if np.isfinite(trace.force[:,2]).any() else 1.
        max_v = np.nanmax(self.actual_speeds[:,2]) if np.isfinite(self.actual_speeds[:,2]).any() else 1.
        span = min(np.diff(self.xy.get_xlim())[0], np.diff(self.xy.get_ylim())[0])*.16
        self.force_scale = span/max(max_f, .1)  # display mm per N
        self.velocity_scale = span/max(max_v, .1)  # display mm per mm/s
        self.force_arrow = self.xy.quiver([0],[0],[0],[0], color='#9b287b', angles='xy', scale_units='xy', scale=1, zorder=7)
        self.velocity_arrow = self.xy.quiver([0],[0],[0],[0], color='#1664be', angles='xy', scale_units='xy', scale=1, zorder=7)
        self.vector_axis = self.figure.add_axes([.06,.198,.11,.06])
        self.vector_axis.set(xlim=(-1.2,1.2), ylim=(-1.2,1.2), title='Base XY directions')
        self.vector_axis.set_aspect('equal'); self.vector_axis.axhline(0,color='.85'); self.vector_axis.axvline(0,color='.85')
        self.vector_axis.set_xticks([]); self.vector_axis.set_yticks([])
        self.vector_axis.text(.95,.05,'+X', fontsize=7, ha='right', transform=self.vector_axis.transAxes)
        self.vector_axis.text(.05,.95,'+Y', fontsize=7, va='top', transform=self.vector_axis.transAxes)
        self.vector_axis.plot([0],[0], 'ko', ms=4)
        self.vector_force = self.vector_axis.quiver([0],[0],[0],[0], color='#9b287b', angles='xy', scale_units='xy', scale=1)
        self.vector_velocity = self.vector_axis.quiver([0],[0],[0],[0], color='#1664be', angles='xy', scale_units='xy', scale=1)
        self.inset_force_scale = 1/max(max_f,.1); self.inset_velocity_scale = 1/max(max_v,.1)
        value_axis = self.figure.add_axes([.19,.253,.34,.035], frameon=False)
        value_axis.set_axis_off()
        self.current_values = value_axis.text(0,1,'',fontsize=8, va='top', family='monospace')
        self.figure.text(.19,.212, f'Purple force: {self.force_scale:.3g} mm/N\nBlue actual speed: {self.velocity_scale:.3g} mm/(mm/s)\nIndependent fixed arrow scales', fontsize=8)
        self.figure.text(.06,.395, collision_text(trace, compact=True), fontsize=8, va='top', family='monospace')
        status_axis = self.figure.add_axes([.06,.162,.48,.022], frameon=False)
        status_axis.set_axis_off()
        self.status = status_axis.text(0,.5,'',fontsize=11)
        self.figure.text(.57,.19, 'Force: '+', '.join(trace.force_fields)+'; Base only\nSequential host reads; no hardware synchronization.', fontsize=8)
        self.slider = Slider(self.figure.add_axes([.10,.13,.82,.022]), 'Time [s]', 0., max(trace.duration,1e-9), valinit=0.)
        self.slider.on_changed(self.seek)
        self.component_selector = RadioButtons(self.figure.add_axes([.78,.045,.07,.065]), ['All','vx','vy','XY'])
        self.component_selector.on_clicked(self.select_component)
        self.figure.canvas.mpl_connect('button_press_event', self.click_time)
        self.buttons = {}
        for key, label, left, width in (('play', 'Pause', .06, .07), ('restart', 'Restart', .14, .07)):
            self.buttons[key] = Button(self.figure.add_axes([left, .06, width, .05]), label)
        self.buttons['play'].on_clicked(self.toggle)
        self.buttons['restart'].on_clicked(self.restart)
        self.figure.text(.23, .078, 'Speed', fontsize=12)
        for i, speed in enumerate((.5, 1., 2., 4.)):
            button = Button(self.figure.add_axes([.28+i*.065, .06, .055, .05]), f'{speed:g}×')
            self.buttons[f'speed_{speed:g}'] = button
            button.on_clicked(lambda event, value=speed: self.set_speed(value))
        for key, label, left in (('contact_zoom','Contact zoom',.57),('lost_zoom','Lost zoom',.68),('full','Full time',.86)):
            button = Button(self.figure.add_axes([left,.055,.095,.04]), label)
            self.buttons[key] = button
            button.on_clicked(lambda event, mode=key: self.zoom(mode))
        self._lost_zoom_index = 0
        self._highlight_speed()
        self.animation = None
        self.draw_at(0.)
        if animate:
            self.animation = FuncAnimation(self.figure, self.tick, init_func=lambda: self.draw_at(self.clock.position),
                interval=1000./fps, blit=True, cache_frame_data=False)
        self.clock.last_wall = self.clock.clock()  # UI construction is not playback time.

    def _highlight_speed(self):
        for speed in (.5, 1., 2., 4.):
            button = self.buttons[f'speed_{speed:g}']
            button.ax.set_facecolor('#d3eadf' if self.clock.speed == speed else '.92')

    def draw_at(self, elapsed):
        """Both panels use the same last recorded index <= elapsed; no interpolation."""
        trace = self.trace
        current = trace.sample_index(elapsed)
        visible = self.indices[:np.searchsorted(self.indices, current, side='right')]
        if not len(visible) or visible[-1] != current:
            visible = np.r_[visible, current]
        points = trace.xy_mm[visible]
        self.complete_path.set_data(*points.T)
        for phase, line in enumerate(self.path_lines):
            colored = points.copy()
            colored[trace.phase[visible] != phase] = np.nan
            line.set_data(*colored.T)
        self.current_tcp.set_data([trace.xy_mm[current, 0]], [trace.xy_mm[current, 1]])
        for component, line in enumerate(self.force_lines):
            line.set_data(trace.time[visible], trace.force[visible, component])
        self.cursor.set_xdata([trace.time[current], trace.time[current]])
        self.velocity_cursor.set_xdata([trace.time[current], trace.time[current]])
        for artist, events in self.event_markers:
            artist.set_offsets(np.asarray([e['xy_mm'] for e in events if e['time'] <= elapsed]).reshape(-1,2))
        f = trace.force[current,:2]
        v = self.actual_speeds[current,:2]
        for arrow, vector, scale in ((self.force_arrow, f, self.force_scale), (self.velocity_arrow, v, self.velocity_scale)):
            arrow.set_offsets(trace.xy_mm[current:current+1]); arrow.set_visible(np.isfinite(vector).all())
            arrow.set_UVC(*(np.nan_to_num(vector)*scale))
        for arrow, vector, scale in ((self.vector_force, f, self.inset_force_scale), (self.vector_velocity, v, self.inset_velocity_scale)):
            arrow.set_visible(np.isfinite(vector).all()); arrow.set_UVC(*(np.nan_to_num(vector)*scale))
        sent = self.planar_speed(self.command_at(trace, trace.actual_time[current])[None,:])[0]
        def values(vector):
            return ', '.join(f'{x:+.2f}' if np.isfinite(x) else 'N/A' for x in vector)
        self.current_values.set_text('Fx, Fy, Fxy [N]: '+values(trace.force[current])+'\nActual vx, vy, XY [mm/s]: '+values(self.actual_speeds[current])+\
                                     '\nSent vx, vy, XY [mm/s]: '+values(sent))
        self.slider.eventson = False
        self.slider.drawon = False
        self.slider.set_val(float(trace.time[current]))
        self.slider.eventson = True
        self.slider.drawon = True
        first = trace.first_contact_index
        confirmed = first is not None and current >= first
        self.first_contact.set_visible(confirmed)
        self.first_label.set_visible(confirmed)
        if confirmed:
            self.first_contact.set_data([trace.xy_mm[first, 0]], [trace.xy_mm[first, 1]])
            self.first_label.xy = trace.xy_mm[first]
        finished = elapsed >= trace.duration
        self.final_stop.set_visible(finished)
        self.stop_label.set_visible(finished)
        self.stop_label.set_text('Confirmed stop' if trace.stop_position_mm is not None else 'Final sample')
        if finished:
            point = trace.xy_mm[-1] if trace.stop_position_mm is None else trace.stop_position_mm
            self.final_stop.set_data([point[0]], [point[1]])
            self.stop_label.xy = point
        state = 'Stopped' if trace.state[current] == 'STOP' else PHASES[trace.phase[current]]
        if trace.state[current] == 'FIRST_CONTACT':
            state = 'First contact confirmation'
        self.status.set_text(f'{trace.time[current]:.2f} / {trace.duration:.2f} s    {state}    {self.clock.speed:g}×')
        self.current_index = current
        artists = (self.complete_path, *self.path_lines, self.current_tcp, *self.force_lines,
                self.cursor, self.first_contact, self.final_stop, self.first_label, self.stop_label, self.status, self.velocity_cursor, self.force_arrow, self.velocity_arrow,
                self.vector_force, self.vector_velocity, self.current_values,
                self.slider.poly, self.slider.valtext, self.slider._handle, *(a for a,_ in self.event_markers))
        self._dynamic_artists = artists
        return artists

    def seek(self, elapsed):
        self.clock.position = float(np.clip(elapsed, 0, self.trace.duration))
        self.clock.playing = False
        self.clock.last_wall = self.clock.clock()
        self.draw_at(self.clock.position)
        self._update_timer()
        self.figure.canvas.draw_idle()

    def click_time(self, event):
        if event.inaxes in (self.force_axis, self.velocity_axis) and event.xdata is not None:
            self.seek(event.xdata)

    def select_component(self, selected):
        for component, pair in zip(('vx','vy','XY'), self.velocity_lines):
            for line in pair:
                line.set_visible(selected == 'All' or selected == component)
        if self.animation is not None:
            self.animation._blit_cache.clear()
        self.figure.canvas.draw_idle()

    def zoom(self, mode):
        events = [e for e in self.trace.events if e['kind'] == 'CONTACT_LOST']
        if mode == 'contact_zoom' and self.trace.first_contact_index is not None:
            t = self.trace.time[self.trace.first_contact_index]
        elif mode == 'lost_zoom' and events:
            t = events[self._lost_zoom_index % len(events)]['time']
            self._lost_zoom_index += 1
        else:
            self.force_axis.set_xlim(0, max(self.trace.duration,1.))
            self.figure.canvas.draw_idle()
            return
        self.force_axis.set_xlim(max(0,t-1.), min(self.trace.duration,t+2.))
        self.seek(t)

    def _update_timer(self):
        # Paused artists must take part in ordinary canvas redraws triggered by
        # sliders/buttons. Otherwise blitting hides them while the timer is stopped.
        if self.animation is not None:
            changed = False
            for artist in self._dynamic_artists:
                changed |= artist.get_animated() != self.clock.playing
                artist.set_animated(self.clock.playing)
            if changed:
                self.animation._blit_cache.clear()
        label = 'Pause' if self.clock.playing else 'Play'
        if self.buttons['play'].label.get_text() != label:
            self.buttons['play'].label.set_text(label)
            self.figure.canvas.draw_idle()
        if self.animation is not None:
            if self.clock.playing:
                self.animation.event_source.start()
            else:
                self.animation.event_source.stop()

    def tick(self, frame=None):
        artists = self.draw_at(self.clock.advance())
        self._update_timer()
        return artists

    def toggle(self, event=None):
        self.clock.toggle()
        self.draw_at(self.clock.position)
        self._update_timer()
        self.figure.canvas.draw_idle()

    def restart(self, event=None):
        self.clock.restart()
        self.draw_at(0.)
        self._update_timer()
        self.figure.canvas.draw_idle()

    def set_speed(self, speed):
        self.clock.set_speed(speed)
        self._highlight_speed()
        self.draw_at(self.clock.position)
        self._update_timer()
        self.figure.canvas.draw_idle()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    args = parser.parse_args()
    import matplotlib
    # Separate launcher process: automatic static plotting never selects the
    # interactive backend or opens a window during experiment completion.
    try:
        matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
        try:
            trace = load_trace(args.run_dir)
        except ValueError as exc:
            parser.error(str(exc))
        application = RunAnimation(trace)
        plt.show()  # Stay open at the final frame; only the user closes the window.
    except (ImportError, RuntimeError) as exc:
        parser.exit(1, f'Animation needs a graphical desktop with Tk: {exc}\n')
    return application


if __name__ == '__main__':
    main()
