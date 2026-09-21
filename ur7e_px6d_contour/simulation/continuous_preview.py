"""Offline scene editor and bounded live view of the shared continuous session.

Matplotlib is imported only when constructing the explicitly requested view.
No geometry is used here to plan commands. All motion comes from session.step().
"""
from collections import deque
from copy import deepcopy
from datetime import datetime
from decimal import Decimal
from pathlib import Path
import time
import textwrap

import numpy as np
import yaml

from experiment_logging.data_logger import ExperimentLogger
from experiment_logging.termination import TerminationReason, classify_stop_reason
from policy.continuous_tracking import EXTRA_SAMPLE_FIELDS, State
from simulation.continuous_session import SimulationSession, validate_scene, SIMULATION_SAMPLE_FIELDS
from simulation.simulator import load_simulation_config
from simulation.continuous_view import (XYViewport, VectorDisplay, force_demonstration, NORMAL_REACTION_NOTE,
    CJK_FONT, FORCE_CURVES, SPEED_CURVES, line_style, component_note)


def surface_friction(value):
    """Parse an editable coefficient without changing any live session."""
    try:
        if isinstance(value, bool):raise ValueError
        value = float(value)
        if not np.isfinite(value) or value < 0:raise ValueError
    except (ValueError, TypeError, OverflowError):
        raise ValueError('请输入非负有限数值，例如 0.03 或 0.20') from None
    return value


def friction_text(value):
    text = repr(float(value))
    if 'e' not in text.lower():
        whole, _, fraction = text.partition('.')
        text = whole + '.' + fraction.ljust(2, '0')
    return text


class PreviewRun:
    """GUI-independent run lifecycle, scene copies, and fixed-step playback clock."""
    max_steps_per_tick = 100
    max_path_points = 4096

    def __init__(self, config, scene, output):
        self.base_config = deepcopy(config)
        self.output = Path(output)
        self.scene = validate_scene(scene)
        self.session = SimulationSession(self.base_config, self.scene)
        self.scene = deepcopy(self.session.scene)
        self.status, self.message = 'READY', 'Ready: choose / drag target, then Start'
        self.history = deque(maxlen=max(2, int(30/self.session.dt)+1))
        self.events = deque(maxlen=200)
        self.path_history = []
        self.logger = None
        self.run_dir = None
        self.last_run_dir = None
        self.speed, self.effective_speed = 1, 0.
        self._wall = None
        self._credit = 0.

    def start(self, wall=None):
        if self.status == 'ENDED':
            self.message = 'Run ended. Reset or edit scene before Start.'
            return
        if self.status == 'READY':
            validate_scene(self.scene)
            self.logger = ExperimentLogger(self.output, self.session.config,
                                          extra_sample_fields=EXTRA_SAMPLE_FIELDS + SIMULATION_SAMPLE_FIELDS, workspace_logging=False)
            self.logger.termination.observe(policy=self.session.policy, processed_force_frame='Base')
            self.run_dir = self.logger.run_dir
            self.last_run_dir = self.run_dir
            print(f'Continuous preview run_dir: {self.run_dir}', flush=True)
            try:
                self.session.capture_bias(observe=self.logger.termination.observe)
            except KeyboardInterrupt:
                self.stop('keyboard interrupt', event='USER_STOP', code=TerminationReason.STOP_USER_REQUEST)
                raise
            except BaseException as exc:
                self.stop(str(exc), event='BIAS_FAILURE', code=TerminationReason.STOP_SENSOR_ERROR)
                raise
        self.status, self.message = 'RUNNING', 'Running synthetic closed loop'
        self._wall = time.monotonic() if wall is None else wall
        self._credit = 0.

    def pause(self, wall=None):
        if self.status == 'RUNNING':
            self.status, self.message = 'PAUSED', 'Paused: simulation time and all control memory frozen'
        self._wall = time.monotonic() if wall is None else wall
        self._credit = 0.

    def set_speed(self, speed, wall=None):
        if speed not in (1, 5, 10):
            raise ValueError('playback speed must be 1, 5 or 10')
        self.speed = speed
        self._wall = time.monotonic() if wall is None else wall
        self._credit = 0.

    def _record(self, sample):
        self.history.append(sample)
        self.path_history.append((sample.time, *sample.robot.pose[:2], segment_kind(sample)))
        if len(self.path_history) > self.max_path_points:
            # Keep the whole run in bounded display memory. A merged segment
            # crossing a state boundary is conservatively marked UNCERTAIN.
            old = self.path_history
            self.path_history = [old[0]] + [
                (*old[i][:3], old[i][3] if old[i-1][3] == old[i][3] else 'UNCERTAIN')
                for i in range(2, len(old), 2)]
        self.events.extend(sample.events)
        self.logger.termination.observe(policy=self.session.policy, timestamp=sample.time,
                                        phase='SIMULATION_STOP' if sample.command.state == 'STOP' else 'SIMULATION_STEP')
        self.logger.log_sample(sample.time, sample.raw, sample.processed, sample.robot,
                               sample.command, sample.inward, sample.tangent,
                               extra={**sample.telemetry, **sample.simulation_telemetry})
        for event in sample.events:
            self.logger.log_waypoint(event)
        self.session.policy.events.clear()

    def tick(self, wall=None, *, work_budget_sec=.025):
        wall = time.monotonic() if wall is None else wall
        elapsed = 0. if self._wall is None else max(0., wall-self._wall)
        self._wall = wall
        if self.status != 'RUNNING':
            return 0
        # Discard excess WALL-clock demand when overloaded, never simulation
        # steps. Actual simulation runs slower; dt and control sequence are fixed.
        self._credit = min(self._credit + elapsed*self.speed,
                           self.max_steps_per_tick*self.session.dt)
        count, started = 0, time.monotonic()
        try:
            while self._credit + 1e-12 >= self.session.dt and count < self.max_steps_per_tick:
                if time.monotonic()-started >= work_budget_sec:
                    break
                self._record(self.session.step())
                self._credit = max(0., self._credit-self.session.dt)
                count += 1
                if self.session.policy.state == State.STOP:
                    self.stop()
                    break
        except Exception as exc:
            self.stop(str(exc), event='PREVIEW_ERROR', code=TerminationReason.STOP_UNKNOWN_REASON)
            raise
        self.effective_speed = count*self.session.dt/elapsed if elapsed > 0 else 0.
        if self.status == 'RUNNING':
            self.message = ('Playback throttled: bounded callback; no control steps skipped'
                            if self.effective_speed < .85*self.speed else 'Running synthetic closed loop')
        return count

    def stop(self, reason='operator stop', *, event='USER_STOP', code=TerminationReason.STOP_USER_REQUEST):
        self._credit = 0.
        if self.logger is None:
            return
        session, logger = self.session, self.logger
        try:
            if session.policy.state != State.STOP:
                session.policy.request_stop(session.robot.time, session.robot.pose, reason, event=event, code=code)
            # A partially completed/legacy stop may have no code. Classify its
            # original detail, never relabel it as the later user stop request.
            if session.policy.stop_reason is None:
                session.policy.stop_reason = classify_stop_reason(session.policy.reason)
            self.status = 'ENDED'
            logger.termination.set_stop_reason(session.policy.stop_reason, session.policy.reason, source='continuous.preview')
            display_code = getattr(session.policy.stop_reason, 'value', session.policy.stop_reason)
            self.message = f'Ended: {display_code or TerminationReason.STOP_UNKNOWN_REASON.value}: {session.policy.reason}'
            # The same simulated execution adapter integrates its braking tail.
            # Record fresh force/pose samples during the tail, with STOP commands.
            for _ in range(int(session.policy.c['confirmation_timeout_sec']/session.dt)+1):
                self._record(session.step())
                if np.linalg.norm(session.robot.tcp_speed[:3]) <= float(session.policy.c['settle_speed_mps']):
                    break
            robot = session.robot.read_state()
            last = self.history[-1]
            info = dict(source='post_stop_simulation', host_age_sec=0,
                        standstill_confirmed=bool(np.linalg.norm(robot.tcp_speed[:3]) <= float(session.policy.c['settle_speed_mps'])))
            logger.write_stop_snapshot(robot, last.raw, last.processed, session.policy.state.value,
                                       extra=dict(stop_observation=info, wrench_is_last_valid_sample=True,
                                                  wrench_age_sec=robot.timestamp-last.time))
            logger.write_summary(session.policy.state.value, session.policy.reason,
                                 int(session.policy.initial_contact is not None),
                                 initial_contact=None if session.policy.initial_contact is None else session.policy.initial_contact.tolist(),
                                 last_contact_pose=None if session.policy.last_contact_pose is None else session.policy.last_contact_pose.tolist(),
                                 first_threshold_pose=None if session.policy.first_threshold_pose is None else session.policy.first_threshold_pose.tolist(),
                                 mode='simulation_preview', stop_observation=info,
                                 provenance=session.config.get('continuous_provenance'))
        finally:
            self.logger = None
            try:
                logger.close()
            finally:
                logger.termination.flush(emit=True)

    def _new_scene(self, candidate, event, *, reason=None):
        candidate = validate_scene(candidate)
        # Build/validate before replacing any state; invalid edits are rejected.
        fresh = SimulationSession(self.base_config, candidate)
        self.stop(reason or ('scene changed' if event == 'SCENE_CHANGED' else 'preview reset'), event=event)
        self.session, self.scene = fresh, deepcopy(fresh.scene)
        self.history.clear()
        self.path_history.clear()
        self.events.clear()
        self.run_dir = None
        self.status, self.message = 'READY', f'{event}: fresh scene ready; press Start'
        self.effective_speed, self._credit, self._wall = 0., 0., None

    def edit(self, **changes):
        candidate = deepcopy(self.scene)
        candidate.update(deepcopy(changes))
        self._new_scene(candidate, 'SCENE_CHANGED')

    def set_surface_friction(self, value):
        value = surface_friction(value)
        old = float(self.scene['force_model']['friction_coefficient'])
        candidate = deepcopy(self.scene)
        candidate['force_model']['friction_coefficient'] = value
        self._new_scene(candidate, 'FRICTION_CHANGED',
                        reason=f'friction_coefficient changed: {old!r} -> {value!r}')

    def reset(self):
        self._new_scene(self.scene, 'USER_RESET')

    def choose_shape(self, shape):
        if shape not in ('square', 'circle', 'triangle'):
            raise ValueError('choose square, circle or triangle')
        changes = dict(target_shape='rectangle' if shape == 'square' else shape)
        if shape == 'square':
            # The square uses the scene's existing width, explicitly shown in UI.
            changes.update(target_height=self.scene['target_width'])
        self.edit(**changes)

    def rotate(self, degrees):
        self.edit(target_rotation_deg=float(self.scene.get('target_rotation_deg', 0))+degrees)

    def save_scene(self, path):
        path = Path(path).expanduser().resolve()
        if path.suffix.lower() not in ('.yaml', '.yml'):
            raise ValueError('independent scene filename must end in .yaml or .yml')
        scene = validate_scene(self.scene)
        scene.pop('base_config', None)
        scene['scene_name'] = 'continuous_preview_independent'
        scene['continuous_preview_scene_version'] = 1
        # Exclusive creation protects ALL existing configuration, calibration,
        # attestation and experiment files, including symlink destinations.
        if path.exists():
            raise FileExistsError(f'will not overwrite existing file: {path}')
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('x', encoding='utf-8') as handle:
            yaml.safe_dump(scene, handle, allow_unicode=True, sort_keys=False)
        self.message = f'Saved independent scene: {path}'
        print(self.message, flush=True)
        return path

    def load_scene(self, path):
        self._new_scene(load_simulation_config(path), 'SCENE_CHANGED')


def extrema_indices(values, buckets=100):
    """Bound display work, retaining minima/maxima of EACH displayed force."""
    values = np.asarray(values)
    if len(values) <= 2+2*values.shape[1]*buckets:
        return np.arange(len(values))
    indices = {0, len(values)-1}
    for chunk in np.array_split(np.arange(len(values)), buckets):
        for column in range(values.shape[1]):
            indices.add(int(chunk[np.argmin(values[chunk, column])]))
            indices.add(int(chunk[np.argmax(values[chunk, column])]))
    return np.array(sorted(indices))


def segment_kind(sample):
    if sample.command.state == 'TARGET_SEARCH':
        return 'SEARCH'
    if sample.command.state in ('CONTACT_LOST', 'LOCAL_REACQUIRE'):
        return 'RECOVERY'
    if (sample.command.state == 'CONTINUOUS_TRACKING' and sample.telemetry['direction_valid']
            and not sample.telemetry['stop_requested']):
        return 'RELIABLE'
    return 'UNCERTAIN'


class ContinuousPreview:
    """Matplotlib artists persist; callbacks can be tested without plt.show()."""
    def __init__(self, model, *, fps=10):
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, Polygon
        from matplotlib.collections import LineCollection
        from matplotlib.widgets import Button, TextBox
        if not np.isfinite(fps) or not 1 <= fps <= 60:
            raise ValueError('preview fps must be between 1 and 60')
        self.model, self.fps = model, fps
        self._drag = None
        self.figure = plt.figure(figsize=(16, 9))
        grid = self.figure.add_gridspec(2, 2, width_ratios=[3, 1], hspace=.4, wspace=.18)
        self.xy = self.figure.add_subplot(grid[:, 0])
        self.force = self.figure.add_subplot(grid[0, 1])
        self.velocity = self.figure.add_subplot(grid[1, 1])
        self.figure.subplots_adjust(left=.065, right=.98, bottom=.17, top=.88)
        self.show_components = False
        self.show_true_tangent = False
        self.debug = self.settings = False
        self.friction_open = False
        self.friction_error = ''
        self._friction_session = None
        self.figure.suptitle('SIMULATION — force demonstration', fontsize=13)
        self.figure.canvas.manager.set_window_title('SIMULATION / SYNTHETIC FORCE')
        self.xy.set(xlabel='X [mm]', ylabel='Y [mm]')
        self.xy.set_aspect('equal', adjustable='box')
        self.force.set(xlabel='Simulation time [s]', ylabel='Processed synthetic force [N]')
        self.force.grid(alpha=.2)
        self.target = Polygon(np.zeros((3, 2)), closed=True, color='.8', label='Known synthetic target (drag)')
        self.xy.add_patch(self.target)
        self.drag_outline, = self.xy.plot([], [], '--', color='.4', lw=1, visible=False)
        self.container, = self.xy.plot([], [], 'k-', lw=1, label='Container')
        self.search, = self.xy.plot([], [], 'k:', lw=1, label='Fixed search line')
        self.points, = self.xy.plot([], [], 'ks', ms=4)
        self.point_labels = [self.xy.text(0, 0, name, fontsize=8) for name in ('P0', 'P1', 'Start')]
        self.start_marker, = self.xy.plot([], [], 'k*', ms=8)
        colors = {'SEARCH': '#5277a3', 'RELIABLE': '#008855', 'UNCERTAIN': '#da8800', 'RECOVERY': '#aa4499'}
        self.paths = {}
        for kind, color in colors.items():
            collection = LineCollection([], colors=color, linewidths=2, label=f'Executed: {kind}')
            self.xy.add_collection(collection)
            self.paths[kind] = collection
        self.probe = Circle((0, 0), radius=0, fill=False, color='red', label='Synthetic probe radius (to scale)')
        self.xy.add_patch(self.probe)
        self.executed, = self.xy.plot([], [], lw=1.3, **line_style('executed'))
        self.tcp, = self.xy.plot([], [], 'ko', ms=4, zorder=10)
        self.event_points, = self.xy.plot([], [], 'kx', ms=5, label='Policy events')
        self.vectors = VectorDisplay(self.xy)
        self.arrows = self.vectors.arrows
        self.force_note = self.figure.text(.045, .10, '', fontsize=8,
                                           fontfamily=['Noto Sans CJK JP', 'DejaVu Sans'])
        self.geometry_text = self.figure.text(.045, .245, '', fontsize=8)
        self.status_text = self.figure.text(.045, .945, '', fontsize=9, va='top',
                                           fontfamily=['Noto Sans CJK JP', 'DejaVu Sans'])
        self.event_text = self.figure.text(.53, .245, '', fontsize=8)
        self.message_text = self.figure.text(.045, .225, '', fontsize=8, color='#8b3510')
        self.force_lines = [self.force.plot([], [], **line_style(key))[0] for key in FORCE_CURVES]
        self.cursor = self.force.axvline(0, color='k', lw=.8)
        self.velocity_lines = [self.velocity.plot([], [], **line_style(key))[0] for key in SPEED_CURVES]
        self.velocity_cursor = self.velocity.axvline(0, color='k', lw=.8)
        self.velocity.set(xlabel='Simulation time [s]', ylabel='Velocity [mm/s]')
        self.velocity.legend(prop={'family': CJK_FONT, 'size': 6}, loc='upper right', ncol=2)
        self.velocity.grid(alpha=.2)
        self.viewport = XYViewport(self.xy, [[-180,-130],[180,130]])
        self.force.legend(loc='upper right', prop={'family': CJK_FONT, 'size': 7})
        self.buttons = []
        self.settings_axes = []
        def add_button(label,action,position,setting=False):
            button=Button(self.figure.add_axes(position),label)
            button.label.set_fontsize(8)
            button.on_clicked(lambda event: self.invoke(action))
            self.buttons.append(button)
            if setting:self.settings_axes.append(button.ax)
            return button
        actions=[('Start',self.start),('Pause/Continue',self.pause),('Stop',lambda:model.stop()),
                 ('Reset',lambda:model.reset()),('Settings',self.toggle_settings),('Debug',self.toggle_debug),
                 ('Target',lambda:self.select_view('Target')),('Probe',lambda:self.select_view('Probe')),('Save PNG',self.save_png)]
        for i,(label,action) in enumerate(actions):add_button(label,action,[.025+i*.0875,.025,.082,.035])
        cjk = ['Noto Sans CJK JP', 'DejaVu Sans']
        self.friction_button=add_button('摩擦设置',self.toggle_friction,[.815,.025,.09,.035])
        self.friction_button.label.set_fontfamily(cjk)
        self.friction_current=self.figure.text(.918,.042,'',fontsize=8,va='center')
        actions=[('1 Square',lambda:model.choose_shape('square')),('2 Circle',lambda:model.choose_shape('circle')),
                 ('3 Triangle',lambda:model.choose_shape('triangle')),('[ -15 deg',lambda:model.rotate(-15)),
                 ('] +15 deg',lambda:model.rotate(15))]+[(f'{speed}x',lambda speed=speed:model.set_speed(speed)) for speed in (1,5,10)]
        for i,(label,action) in enumerate(actions):add_button(label,action,[.025+i*.118,.12,.11,.033],True)
        actions=[('Save scene',self.save_scene),('Load scene',self.load_scene),('Global',lambda:self.select_view('Global')),
                 ('Follow',self.toggle_follow),('Components',self.toggle_components)]
        for i,(label,action) in enumerate(actions):add_button(label,action,[.025+i*.16,.075,.15,.033],True)
        self.truth_button=add_button('真实切向对照（仅仿真）：关',self.toggle_true_tangent,[.825,.075,.15,.033])
        self.truth_button.label.set_fontfamily(CJK_FONT)
        self.truth_button.label.set_fontsize(7)
        self.truth_button.ax.set_visible(False)
        default_path=Path('simulation_scenes')/f'continuous_{datetime.now():%Y%m%d_%H%M%S_%f}.yaml'
        self.path_box=TextBox(self.figure.add_axes([.11,.175,.865,.033]),'Scene YAML: ',initial=str(default_path))
        self.settings_axes.append(self.path_box.ax)
        for axis in self.settings_axes:axis.set_visible(False)
        # Reuse the otherwise empty lower signal area; never shrink XY for this editor.
        self.friction_box=TextBox(self.figure.add_axes([0,0,.1,.035]),'μ ',initial='')
        self.friction_box.on_text_change(lambda text:setattr(self,'friction_error',''))
        self.friction_minus=add_button('−',lambda:self.adjust_friction(-1),[0,0,.04,.035])
        self.friction_plus=add_button('+',lambda:self.adjust_friction(1),[0,0,.04,.035])
        self.friction_apply=add_button('应用并重置本轮',self.apply_friction,[0,0,.2,.035])
        self.friction_apply.label.set_fontfamily(cjk)
        self.friction_axes=[self.friction_box.ax,self.friction_minus.ax,self.friction_plus.ax,self.friction_apply.ax]
        self.friction_caption=self.figure.text(0,0,'表面摩擦 μ（待应用）',fontsize=8,va='top',fontfamily=cjk)
        self.friction_status=self.figure.text(0,0,'',fontsize=8,va='top',fontfamily=cjk)
        for artist in [*self.friction_axes,self.friction_caption,self.friction_status]:artist.set_visible(False)
        self.timer = self.figure.canvas.new_timer(interval=round(1000/fps))
        self.timer.add_callback(self.on_timer)
        for event, callback in [('key_press_event', self.on_key), ('button_press_event', self.on_press),
                                ('scroll_event', self.viewport.scroll), ('resize_event', lambda event: self.draw()),
                                ('motion_notify_event', self.on_motion), ('button_release_event', self.on_release), ('close_event', self.on_close)]:
            self.figure.canvas.mpl_connect(event, callback)
        self.refresh_scene()
        self.draw()

    def invoke(self, action):
        old_session = self.model.session
        try:
            action()
        except Exception as exc:
            self.model.message = f'Rejected / stopped: {type(exc).__name__}: {exc}'
            print(self.model.message, flush=True)
        if old_session is not self.model.session:
            self.refresh_scene()
        self.draw()
        self.figure.canvas.draw_idle()

    def start(self):
        self.model.start()

    def pause(self):
        if self.model.status == 'RUNNING':
            self.model.pause()
        elif self.model.status == 'PAUSED':
            self.model.start()

    def save_scene(self):
        self.model.save_scene(self.path_box.text)

    def load_scene(self):
        self.model.load_scene(self.path_box.text)

    def save_png(self):
        if self.model.run_dir is None or self.model.status != 'ENDED':
            raise ValueError('Stop this run before Save PNG')
        self.draw()
        path = self.model.run_dir/'continuous_preview.png'
        if path.exists():
            path = path.with_name(f'continuous_preview_{datetime.now():%H%M%S_%f}.png')
        self.figure.savefig(path, dpi=140)
        self.model.message = f'Saved synchronized PNG: {path}'
        print(self.model.message, flush=True)

    def refresh_scene(self):
        scene = self.model.scene
        target = self.model.session.target
        self.target.set_xy(target.boundary_points()*1000)
        b = scene['container']
        x0, x1, y0, y1 = (b[k]*1000 for k in ('x_min', 'x_max', 'y_min', 'y_max'))
        self.container.set_data([x0, x1, x1, x0, x0], [y0, y0, y1, y1, y0])
        self.viewport.bounds = [[x0,y0],[x1,y1]]
        self.viewport.target = target.boundary_points()*1000
        self.viewport.select(self.viewport.mode, self.model.session.robot.pose[:2]*1000)
        points = np.asarray([scene['calibration_point_0'], scene['calibration_point_1']])*1000
        start = np.asarray(scene['start_point'])*1000
        end = start+np.asarray(scene['scan_direction_xy'])*self.model.session.policy.c['search_max_distance']*1000
        self.search.set_data([start[0], end[0]], [start[1], end[1]])
        self.points.set_data(points[:, 0], points[:, 1])
        self.start_marker.set_data([start[0]], [start[1]])
        for label, point, offset in zip(self.point_labels, [*points, start], [(2, 3), (2, 3), (2, -6)]):
            label.set_position(point+offset)
        self.probe.set_radius(float(scene['force_model'].get('probe_tip_radius', 0))*1000)
        center = np.asarray(scene['target_center'])*1000
        shape = scene['target_shape']
        size = (f"width {scene['target_width']*1000:g}, height {scene['target_height']*1000:g} mm"
                if shape in ('rectangle', 'rotated_rectangle') else
                f"{'circumradius' if shape == 'triangle' else 'radius'} {scene['target_radius']*1000:g} mm")
        self.geometry_text.set_text(f'{shape}: center ({center[0]:.1f}, {center[1]:.1f}) mm; {size}\n'
                                    f"rotation {scene.get('target_rotation_deg', 0):g} deg; synthetic probe radius {self.probe.radius:g} mm")

    def select_view(self, mode):
        self.viewport.select(mode, self.model.session.robot.pose[:2]*1000)

    def toggle_follow(self):
        self.viewport.follow = not self.viewport.follow
        if self.viewport.follow:
            self.viewport.select('Probe', self.model.session.robot.pose[:2]*1000)

    def toggle_components(self):
        self.show_components = not self.show_components
        if self.show_components:self.debug=True

    def toggle_true_tangent(self):
        self.show_true_tangent = not self.show_true_tangent
        if self.show_true_tangent: self.debug = True

    def toggle_debug(self):
        self.debug = not self.debug

    def toggle_settings(self):
        self.settings = not self.settings
        for axis in self.settings_axes:axis.set_visible(self.settings)

    def toggle_friction(self):
        self.friction_open = not self.friction_open
        for artist in [*self.friction_axes,self.friction_caption,self.friction_status]:artist.set_visible(self.friction_open)

    def adjust_friction(self, steps):
        try:
            surface_friction(self.friction_box.text)
            draft = Decimal(self.friction_box.text.strip()) + Decimal(steps)*Decimal('0.01')
            value = surface_friction(draft)
        except ValueError as exc:
            self.friction_error = str(exc)
            return
        self.friction_box.set_val(friction_text(value))

    def apply_friction(self):
        try:
            value = surface_friction(self.friction_box.text)
        except ValueError as exc:
            self.friction_error = str(exc)
            return
        self.model.set_surface_friction(value)
        self.refresh_scene()

    def _draw_friction(self):
        value = self.model.scene['force_model']['friction_coefficient']
        if self._friction_session is not self.model.session:
            self._friction_session = self.model.session
            self.friction_box.set_val(friction_text(value))
            self.friction_error = ''
        text = friction_text(value)
        self.friction_current.set_text('μ='+text if len(text)<=8 else 'μ≈'+format(float(value),'.3g'))
        box = self.velocity.get_position(original=True)
        x,w,top = box.x0,box.width,box.y1
        self.friction_caption.set_position((x,top-.008))
        self.friction_box.ax.set_position([x+.02,top-.060,w*.46,.035])
        self.friction_minus.ax.set_position([x+w*.60,top-.060,w*.17,.035])
        self.friction_plus.ax.set_position([x+w*.83,top-.060,w*.17,.035])
        self.friction_apply.ax.set_position([x,top-.108,w,.036])
        self.friction_status.set_position((x,top-.123))
        self.friction_status.set_text(self.friction_error or '仅编辑不生效；应用后回到 READY。\n点击 Start 开始新一轮。')
        self.friction_status.set_color('#a22' if self.friction_error else '.25')

    def _layout_footer(self, note):
        # One persistent text block, with measured line spacing reserved below
        # the axes. Never stack independent notes over the Settings controls.
        width, height = self.figure.get_size_inches()
        columns = max(30, int(width*72*.935/4.8))
        lines = [note]
        if self.debug:
            lines += [self.geometry_text.get_text(), self.event_text.get_text(), self.message_text.get_text()]
        elif self.model.message.startswith(('Rejected', 'Saved')):
            lines += [self.model.message]
        wrapped = '\n'.join(textwrap.fill(line, width=columns) for block in lines for line in block.splitlines() if line)
        self.force_note.set_text(wrapped)
        base = .225 if self.settings else .077
        self.force_note.set_position((.045, base))
        self.force_note.set_verticalalignment('bottom')
        footer_height = (wrapped.count('\n')+1)*12/(height*72)
        header_height = (self.status_text.get_text().count('\n')+1)*14/(height*72)
        self.figure.subplots_adjust(bottom=base+footer_height+40/(height*72),
                                    top=min(.88,.945-header_height-.012))

    def _display_mode(self):
        self.truth_button.ax.set_visible(self.debug and self.settings)
        self.truth_button.label.set_text("真实切向对照（仅仿真）："+("开" if self.show_true_tangent else "关"))
        mode=(self.debug,self.friction_open)
        if getattr(self,'_last_display_debug',None)==mode:return
        self._last_display_debug=mode
        for artist in [self.container,self.search,self.points,self.start_marker,self.probe,self.event_points,
                       *self.point_labels,*self.paths.values()]:
            artist.set_visible(False)
        self.executed.set_visible(True)
        velocity_visible=self.debug and not self.friction_open
        self.velocity.set_visible(velocity_visible)
        # In Debug both right panels share time; label the lower one only.
        self.force.set_xlabel('' if velocity_visible else 'Simulation time [s]')
        self.force.tick_params(axis='x', labelbottom=not velocity_visible)
        for i,line in enumerate(self.force_lines):line.set_visible(self.debug or i in (2,3))
        self.force.legend(handles=[line for line in self.force_lines if line.get_visible()],prop={'family': CJK_FONT, 'size': 7})
        for text in (self.geometry_text,self.event_text,self.message_text):text.set_visible(False)


    def draw(self):
        model = self.model
        history = list(model.history)
        self._display_mode()
        components = None
        physical = None
        valid = False
        if history:
            last = history[-1]
            self.display_time = last.time
            xy = last.robot.pose[:2]*1000
            force = last.processed.force[:2]
            command = last.command.direction_xy*last.command.speed*1000
            actual = last.robot.tcp_speed[:2]*1000
            tangent, inward = last.tangent, last.inward
            valid = bool(last.telemetry['direction_valid']) and not last.telemetry['stop_requested'] and last.command.state == 'CONTINUOUS_TRACKING'
            values = np.array([[s.processed.fx, s.processed.fy, np.hypot(s.processed.fx,s.processed.fy),
                                s.telemetry['force_reference'], s.telemetry['force_reference']-np.hypot(s.processed.fx,s.processed.fy)] for s in history])
            velocities = np.array([[s.command.speed, np.linalg.norm(s.robot.tcp_speed[:3]), s.telemetry['v_t'], s.telemetry['v_n']] for s in history])*1000
            times = np.array([s.time for s in history])
            for axis, lines, rows in [(self.force,self.force_lines,values),(self.velocity,self.velocity_lines,velocities)]:
                indices = extrema_indices(rows, buckets=60)
                for i,line in enumerate(lines): line.set_data(times[indices],rows[indices,i])
                axis.set_xlim(max(0,last.time-30),max(.1,last.time))
                visible_rows=rows[:,[2,3]] if axis is self.force and not self.debug else rows
                lo,hi=min(0,visible_rows.min()),max(0,visible_rows.max());pad=max(.1,(hi-lo)*.1)
                axis.set_ylim(lo-pad,hi+pad)
            segments = {kind: [] for kind in self.paths}
            for a,b in zip(model.path_history,model.path_history[1:]):
                segments[b[3]].append(np.asarray([a[1:3],b[1:3]])*1000)
            for kind,artist in self.paths.items():artist.set_segments(segments[kind])
            if self.show_components and self.debug:
                components={key:last.diagnostics[key+'_force'] for key in ('object','friction','background','noise')}
            row={**last.simulation_telemetry,**last.telemetry,'current_state':last.command.state,
                 'tcp_vx':last.robot.tcp_speed[0],'tcp_vy':last.robot.tcp_speed[1]}
            physical=force_demonstration(row,model.session.config.get('force_display',{}),simulated=True,
                         components_visible=components is not None,
                         previous_velocity=history[-2].robot.tcp_speed[:2] if len(history)>1 else None)
            self.executed.set_data([p[1]*1000 for p in model.path_history],[p[2]*1000 for p in model.path_history])
            current = f'Fx {values[-1,0]:.2f}, Fy {values[-1,1]:.2f}, Fxy {values[-1,2]:.2f} N\nF_ref {values[-1,3]:.2f}, error {values[-1,4]:+.2f} N'
            motion = f'cmd {velocities[-1,0]:.3f}, actual {velocities[-1,1]:.3f} mm/s\nv_t {velocities[-1,2]:+.3f}, v_n {velocities[-1,3]:+.3f} mm/s'
            policy_state = last.command.state+' / '+last.telemetry['direction_phase']
        else:
            self.display_time=0.
            xy=model.session.robot.pose[:2]*1000
            force=command=actual=tangent=inward=np.zeros(2)
            for line in self.force_lines+self.velocity_lines:line.set_data([],[])
            for artist in self.paths.values():artist.set_segments([])
            self.executed.set_data([],[])
            current,motion,policy_state='No force samples yet','No velocity samples yet','READY'
            for axis in (self.force,self.velocity):axis.set_xlim(0,1);axis.set_ylim(-.2,2)
        self.tcp.set_data([xy[0]],[xy[1]])
        self.probe.center=xy
        self.viewport.update(xy,running=model.status=='RUNNING')
        self.vectors.draw(xy,force,command,actual,tangent,inward,valid=valid,components=components,physical=physical,debug=self.debug,
                          components_enabled=self.show_components, true_tangent=self.show_true_tangent, simulated=True)
        for cursor in (self.cursor,self.velocity_cursor):cursor.set_xdata([self.display_time]*2)
        event_xy=np.array([e.pose[:2]*1000 for e in model.events]).reshape(-1,2)
        self.event_points.set_data(event_xy[:,0],event_xy[:,1])
        self.event_text.set_text('\n'.join(f'{e.timestamp:.2f}s {e.event_type}' for e in list(model.events)[-2:]))
        self.xy.set_title('')
        self.force.set_title(current if self.debug else 'Control feedback load [N]',fontsize=9)
        self.force.set_ylabel('Control feedback [N]')
        self.velocity.set_title(motion,fontsize=8)
        budget = model.session.policy.c['max_runtime_sec']
        mode = '手动停止模式' if budget is None else f'限时 {float(budget):g} 秒'
        code = getattr(model.session.policy.stop_reason, 'value', model.session.policy.stop_reason)
        detail = f'{model.status} | {policy_state}'
        if model.session.policy.reason:detail += f' | {code or ""}: {model.session.policy.reason}'
        self.status_text.set_text(f'仿真时间：{self.display_time:.2f}秒｜{mode}\n'+
                                 textwrap.fill(detail,width=max(40,int(self.figure.get_figwidth()*12))))
        diagnostic = component_note(simulated=True, enabled=self.debug and self.show_components)
        self.message_text.set_text(model.message)
        note = (physical or {}).get('note',NORMAL_REACTION_NOTE+'\nPhysical force unavailable: no sample')
        self._layout_footer('\n'.join(part for part in (note, diagnostic, self.vectors.truth_note) if part))
        self.vectors.layout_legend()
        self._draw_friction()

    def on_timer(self):
        self.invoke(self.model.tick)

    def on_key(self, event):
        key = (event.key or '').lower()
        if key in ('q', 'escape'):
            self.invoke(self.model.stop)
            return
        # Ignore shortcuts while typing a scene filename.
        if ((self.settings and self.path_box.capturekeystrokes) or
                (self.friction_open and self.friction_box.capturekeystrokes)):
            return
        actions = {'1': lambda: self.model.choose_shape('square'), '2': lambda: self.model.choose_shape('circle'),
                   '3': lambda: self.model.choose_shape('triangle'), '[': lambda: self.model.rotate(-15),
                   ']': lambda: self.model.rotate(15), ' ': self.pause, 'enter': self.start,
                   'r': self.model.reset, 'q': self.model.stop, 'escape': self.model.stop}
        if key in actions:
            self.invoke(actions[key])

    def on_press(self, event):
        toolbar = getattr(self.figure.canvas.manager, 'toolbar', None)
        if toolbar is not None and toolbar.mode:
            return
        if event.inaxes is self.xy and event.button == 1 and event.xdata is not None:
            point = np.array([event.xdata, event.ydata])/1000
            if self.model.session.target.signed_distance_and_outward_normal(point)[0] <= 0:
                # Freeze playback during a drag; commit the validated scene on
                # release, then terminate old state with SCENE_CHANGED.
                self.model.pause()
                self._drag = (point, np.asarray(self.model.scene['target_center']))

    def on_motion(self, event):
        if self._drag is not None and event.inaxes is self.xy and event.xdata is not None:
            point, center = self._drag
            delta = np.array([event.xdata, event.ydata])/1000-point
            boundary = (self.model.session.target.boundary_points()+delta)*1000
            self.drag_outline.set_data(boundary[:, 0], boundary[:, 1])
            self.drag_outline.set_visible(True)
            self.figure.canvas.draw_idle()

    def on_release(self, event):
        if self._drag is None:
            return
        point, center = self._drag
        self.drag_outline.set_visible(False)
        self._drag = None
        if event.inaxes is self.xy and event.xdata is not None:
            new_center = center+np.array([event.xdata, event.ydata])/1000-point
            self.invoke(lambda: self.model.edit(target_center=new_center.tolist()))

    def on_close(self, event):
        self.timer.stop()
        self.model.stop('window closed', event='WINDOW_CLOSED')

    def show(self):
        import matplotlib.pyplot as plt
        self.timer.start()
        try:
            plt.show()
        except KeyboardInterrupt:
            self.model.stop('keyboard interrupt', event='USER_STOP')
        finally:
            self.on_close(None)


def launch_preview(args, config, provenance):
    """Explicit GUI entry. No hardware objects, including when setup fails."""
    from calibration.scan_calibration import resolve_calibration_path
    config = deepcopy(config)
    config['continuous_provenance'] = provenance
    if getattr(args, 'enable_reacquire', False):
        config['continuous_tracking']['reacquire_enabled'] = True
    if args.duration is not None and (not np.isfinite(args.duration) or args.duration <= 0):
        raise ValueError('--duration must be finite and positive')
    # Override this simulation copy only. None is explicit manual-stop mode;
    # finite offline horizons may exceed the unchanged, reviewed real budget.
    config['continuous_tracking']['max_runtime_sec'] = args.duration
    output = args.output or resolve_calibration_path(args.config, config['logging']['output_root'])
    model = PreviewRun(config, load_simulation_config(args.scene), output)
    import matplotlib
    from matplotlib.backends.registry import backend_registry
    _, gui = backend_registry.resolve_backend(matplotlib.get_backend())
    if gui is None:
        raise RuntimeError('No interactive Matplotlib backend; use --dry-run, or run --preview from a graphical desktop')
    view = ContinuousPreview(model, fps=float(config['continuous_tracking']['visualization_fps']))
    view.show()
    return 0
