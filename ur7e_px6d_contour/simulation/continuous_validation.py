"""Offline evidence only: actual integrated motion -> existing geometric force sensor.

Recovery-only cases explicitly seed a lost-contact state and historical local
memory; they are not claims that an entire approach/loss sequence succeeded.
Target geometry is available to sensor/scorer, never to the policy.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime
import json
from pathlib import Path

import numpy as np

from config.loader import load_config
from experiment_logging.data_logger import ExperimentLogger
from policy.continuous_tracking import ContinuousTrackingPolicy, EXTRA_SAMPLE_FIELDS, State, arc_reference
from sensor.force_preprocess import WrenchPreprocessor
from simulation.geometry import create_target
from simulation.simulated_force_sensor import SimulatedForceSensor
from simulation.simulated_robot import SimulatedRobot

ROOT = Path(__file__).resolve().parents[1]
BOUNDS = dict(x_min=-.2, x_max=.2, y_min=-.2, y_max=.2)
FORCE = dict(random_seed=7, granular_drag_force=0., noise_std=0., contact_stiffness=1800.,
             probe_tip_radius=.001, friction_coefficient=0.)


def scene_definition(name):
    rectangle = dict(target_shape='rectangle', target_center=[.02, 0.], target_width=.04, target_height=.04)
    circle = dict(target_shape='circle', target_center=[.02, 0.], target_radius=.02)
    endpoint = dict(target_shape='rectangle', target_center=[.02, .002], target_width=.04, target_height=.004)
    return {
        'track_straight': (rectangle, [-.003, .006], False, 12.),
        'track_circle': (circle, [-.003, 0.], False, 15.),
        'track_endpoint': (endpoint, [-.003, .0015], False, 12.),
        'recover_straight': (rectangle, [-.0015, 0.], True, 8.),
        'recover_arc': (circle, [-.0015, 0.], True, 8.),
        'recover_endpoint': (endpoint, [-.0008, -.0005], True, 8.),
        'recover_empty': (dict(target_shape='circle', target_center=[.1,0], target_radius=.02), [-.0015,0.], True, 8.),
    }[name]


def run_case(name, *, output_root=None, rotation_deg=0., mirror=False, hand='COUNTERCLOCKWISE', delay_steps=2,
             acceleration_limit=.005):
    cfg = deepcopy(load_config(ROOT/'config.yaml'))
    cfg['continuous_tracking']['reacquire_enabled'] = True
    cfg['policy']['follow_hand'] = hand
    geometry, start, recovery_only, duration = scene_definition(name)
    angle = np.deg2rad(rotation_deg)
    transform = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]) @ np.diag([1, -1 if mirror else 1])
    # Orthogonal transform of environment and observations. Geometry remains
    # exclusively inside this sensor/scorer wrapper.
    base_target = create_target(geometry)
    class TransformedTarget:
        def signed_distance_and_outward_normal(self, xy):
            distance, normal = base_target.signed_distance_and_outward_normal(transform.T@xy)
            return distance, transform@normal
    target = TransformedTarget()
    start = transform@start
    cfg['policy']['search_direction_xy'] = (transform@np.array([1.,0.])).tolist()
    pre = WrenchPreprocessor.from_config(cfg['preprocessing'])
    # No sensor zero recapture in contact; synthetic no-bias sensor needs none.
    robot = SimulatedRobot(start, .01, BOUNDS, acceleration_limit=acceleration_limit, delay_steps=delay_steps)
    sensor = SimulatedForceSensor(target, FORCE)
    policy = ContinuousTrackingPolicy(cfg)
    if recovery_only:
        # Synthetic HISTORY, explicitly labelled in every report. Force after
        # t=0 always comes from actual XY geometry, never a timed injection.
        policy.state = State.CONTACT_LOST
        policy._started = policy._last_time = -.01
        policy._start_pose = robot.pose.copy()
        policy._confirm_started = 0.
        n = transform@np.array([1.,0.])
        from policy.boundary_estimation import handed_tangent
        t = handed_tangent(n, hand)
        policy.contact_direction, policy.tangent = n.copy(), t.copy()
        policy.direction_confidence = 1.
        policy._remember(-.01, robot.pose)
        policy.loss_detection_pose = robot.pose.copy()
    cfg['continuous_simulation'] = dict(**geometry, container=BOUNDS, target_boundary_xy=(base_target.boundary_points()@transform.T).tolist())
    cfg['continuous_validation_case'] = dict(name=name, seeded_lost_state=recovery_only,
                                           delay_steps=delay_steps, acceleration_limit=acceleration_limit,
                                           rotation_deg=rotation_deg, mirror=mirror, hand=hand)
    if output_root is not None:
        from run_continuous_tracking import git_provenance
        cfg['continuous_provenance'] = git_provenance()
    logger = None if output_root is None else ExperimentLogger(output_root, cfg, extra_sample_fields=EXTRA_SAMPLE_FIELDS, workspace_logging=False)
    trajectory, references, forces, states, penetrations, compressions, errors = [], [], [], [], [], [], []
    events = []
    path = 0.
    previous = robot.pose[:2].copy()
    for _ in range(int(duration/.01)+1):
        state = robot.read_state()
        raw, meta = sensor.read_wrench(state.pose[:2], state.tcp_speed[:2])
        processed = pre.process(raw)
        command = policy.update(state.timestamp, raw, processed, state)
        trajectory.append(state.pose[:2].copy())
        path += float(np.linalg.norm(state.pose[:2]-previous))
        previous = state.pose[:2].copy()
        references.append([float('nan')]*2 if policy.reacquire_reference is None else policy.reacquire_reference.copy())
        forces.append(float(np.linalg.norm(raw.force[:2])))
        states.append(policy.state.value)
        penetrations.append(meta['penetration'])
        compressions.append(meta['tip_compression'])
        errors.append(policy.reacquire_tracking_error)
        events.extend(e.event_type for e in policy.events)
        if logger:
            logger.log_sample(state.timestamp, raw, processed, state, command, policy.contact_direction, policy.tangent,
                              extra=policy.telemetry(command))
            for event in policy.events: logger.log_waypoint(event)
        policy.events.clear()
        robot.apply_command(command.direction_xy, command.speed, command.move)
        if policy.state == State.STOP or (recovery_only and 'REACQUIRED' in events):
            break
    # Include actual braking tail in max force/penetration/path evaluation.
    for _ in range(100):
        raw, meta = sensor.read_wrench(robot.pose[:2], robot.tcp_speed[:2])
        forces.append(float(np.linalg.norm(raw.force[:2])))
        penetrations.append(meta['penetration']); compressions.append(meta['tip_compression'])
        path += float(np.linalg.norm(robot.pose[:2]-previous)); previous=robot.pose[:2].copy()
        if np.linalg.norm(robot.tcp_speed[:2]) < 1e-10: break
        robot.apply_command([0,0],0,False)
    track_mask = np.array(states)==State.CONTINUOUS_TRACKING.value
    track_forces = np.asarray(forces[:len(states)])[track_mask]
    report = dict(scene=name, evidence='closed_loop_recovery_with_seeded_memory' if recovery_only else 'closed_loop_from_search',
                  recovery_success='REACQUIRED' in events, final_state=policy.state.value, stop_reason=policy.reason,
                  events=events, actual_path_length_m=path, recovery_path_length_m=policy.reacquire_path_length,
                  max_tracking_error_m=max(errors), max_penetration_m=max(penetrations), max_tip_compression_m=max(compressions),
                  max_force_N=max(forces), tracking_contact_fraction=None if not len(track_forces) else float(np.mean(track_forces>=.5)),
                  force_error_rms_N=None if not len(track_forces) else float(np.sqrt(np.mean((track_forces-1.5)**2))),
                  rotation_deg=rotation_deg, mirror=mirror, follow_hand=hand,
                  delay_steps=delay_steps, acceleration_limit_mps2=acceleration_limit)
    if logger:
        report['run_dir'] = str(logger.run_dir)
        logger.write_summary(policy.state.value, policy.reason or 'validation horizon reached', int(policy.initial_contact is not None), validation=report)
        with (logger.run_dir/'geometry_report.json').open('w') as handle: json.dump(report,handle,indent=2)
        logger.close()
    return report, np.asarray(trajectory), np.asarray(references)


def run_direction_case(name, *, output_root, duration=180., rotation_deg=0., translation=(0.,0.)):
    """Full search->motion->sensor loop; only the OFFLINE duration is extended.

    Truth below is an independent scorer; never supplied to the policy.
    Passing a first-turn metric is not a full-contour or hardware claim.
    """
    import csv
    from simulation.simulator import load_simulation_config
    from simulation.continuous_preview import PreviewRun
    from policy.continuous_tracking import angle_between
    from policy.boundary_estimation import handed_tangent
    from run_continuous_tracking import git_provenance
    cfg=load_config(ROOT/'config.yaml')
    cfg['continuous_tracking']['max_runtime_sec']=float(duration)
    cfg['continuous_provenance']=git_provenance()
    path=ROOT/'simulation'/('scene_direction_triangle.yaml' if name=='triangle' else 'scene_continuous.yaml')
    scene=load_simulation_config(path)
    if name=='square':scene.update(target_shape='rectangle',target_width=.12,target_height=.12,target_rotation_deg=0.)
    elif name=='empty':scene.update(target_shape='circle',target_center=[0,.08],target_radius=.02)
    elif name not in ('circle','triangle'):raise ValueError(name)
    theta=np.deg2rad(rotation_deg);matrix=np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
    for key in ('target_center','start_point','calibration_point_0','calibration_point_1'):
        scene[key]=(matrix@np.asarray(scene[key])+translation).tolist()
    scene['target_rotation_deg']=scene.get('target_rotation_deg',0.)+rotation_deg
    cfg['continuous_direction_validation']=dict(case=name,offline_duration_sec=duration,
        rotation_deg=rotation_deg,translation=list(translation),only_duration_extended=True)
    model=PreviewRun(cfg,scene,output_root)
    model.start(0);model.set_speed(10,0);wall=0.
    try:
        while model.status=='RUNNING':
            wall+=.1;model.tick(wall,work_budget_sec=10)
    finally:model.stop('validation interrupted')
    with (model.run_dir/'samples.csv').open() as f:rows=list(csv.DictReader(f))
    xy=np.array([[float(r['tcp_x']),float(r['tcp_y'])] for r in rows])
    force=np.array([float(r['fxy']) for r in rows])
    track=np.array([r['current_state']=='CONTINUOUS_TRACKING' for r in rows])
    target=model.session.target
    normals=np.array([target.signed_distance_and_outward_normal(p)[1] for p in xy])
    indices=np.flatnonzero(track)
    turn=None;progress=path_after=0.
    threshold=115 if name=='triangle' else 85 if name=='square' else 90
    if len(indices):
        first=indices[0]
        angles=np.unwrap(np.arctan2(normals[first:,1],normals[first:,0]))
        reached=np.flatnonzero(np.abs(angles-angles[0])>=np.deg2rad(threshold))
        if len(reached):
            turn=first+int(reached[0])
            tangent=handed_tangent(-normals[turn],cfg['policy']['follow_hand'])
            progress=float(np.dot(xy[-1]-xy[turn],tangent))
            path_after=float(np.linalg.norm(np.diff(xy[turn:],axis=0),axis=1).sum())
    penetration=max(float(r['sim_penetration_m']) for r in rows)
    crossings=sum(int(r['sim_segment_penetration']) for r in rows)
    compression=max(float(r['sim_compression_m']) for r in rows)
    abnormal=compression>float(scene['force_model']['probe_tip_radius'])+1e-9
    events=[e.event_type for e in model.events]
    report=dict(case=name,run_dir=str(model.run_dir),offline_duration_sec=duration,
        first_turn_normal_change_deg=threshold,first_turn_time=None if turn is None else float(rows[turn]['monotonic_sec']),
        post_turn_net_progress_m=progress,post_turn_path_m=path_after,
        first_turn_validated=bool(progress>=.005 and penetration<=1e-9 and crossings==0 and not abnormal),
        contour_complete=False,termination_reason=model.session.policy.stop_reason.value,stop_detail=model.session.policy.reason,
        direction_pause_count=events.count('DIRECTION_STOP_REQUEST'),direction_confirmed_count=events.count('DIRECTION_CONFIRMED'),
        resume_verified_count=events.count('DIRECTION_RESUME_VERIFIED'),
        max_penetration_m=penetration,segment_penetration_count=crossings,max_compression_m=compression,
        abnormal_compression=bool(abnormal),processed_force_range_N=[float(force.min()),float(force.max())],
        tracking_force_range_N=None if not len(indices) else [float(force[track].min()),float(force[track].max())],
        rotation_deg=rotation_deg,translation=list(translation),sample_count=len(rows))
    (model.run_dir/'direction_geometry_report.json').write_text(json.dumps(report,indent=2))
    return report


def direction_suite(output_root):
    output=Path(output_root)/datetime.now().strftime('directions_%Y%m%d_%H%M%S_%f')
    reports=[run_direction_case(name,output_root=output/name) for name in ('circle','square','triangle','empty')]
    reports.append(run_direction_case('square',output_root=output/'square_transformed',rotation_deg=30.,translation=(.01,.015)))
    (output/'direction_reports.json').write_text(json.dumps(reports,indent=2))
    print(json.dumps(reports,indent=2));print(output)
    return reports


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'simulation_outputs'/'continuous_revision')
    parser.add_argument('--directions',action='store_true',help='extended offline first-turn audit; no speed increase')
    args=parser.parse_args()
    if args.directions:
        direction_suite(args.output)
        return
    output=args.output/datetime.now().strftime('validation_%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True,exist_ok=False)
    reports=[run_case(name,output_root=output)[0] for name in ('track_straight','track_circle','track_endpoint',
                                                             'recover_straight','recover_arc','recover_endpoint','recover_empty')]
    reports.append(run_case('track_endpoint',output_root=output,delay_steps=30)[0])
    (output/'reports.json').write_text(json.dumps(reports,indent=2))
    print(json.dumps(reports,indent=2))
    print(output)


if __name__=='__main__': main()
