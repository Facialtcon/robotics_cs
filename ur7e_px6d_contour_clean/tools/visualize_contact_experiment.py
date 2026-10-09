#!/usr/bin/env python3
"""Reports for explicit runs, or a labelled synthetic demo. No hardware."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def make_demo(output):
    import numpy as np
    from config.loader import load_config
    from core.models import Wrench, RobotState, PolicyCommand, PolicyWaypoint
    from experiment_logging.data_logger import ExperimentLogger
    from sensor.force_preprocess import WrenchPreprocessor
    config = load_config(ROOT/'config.yaml')
    config['experiment'] = dict(kind='synthetic_demo', note='Illustrative signals, not a physical robot simulation',
        target=dict(center_base_mm=[32.,20.], side_mm=40., rotation_deg=15.))
    config['force_display'] = dict(frame='Base')
    config['preprocessing']['kalman']['enabled'] = False
    transform = config['preprocessing']['coordinate_transform']
    transform['rotation_sensor_to_tool'] = np.eye(3).tolist()
    transform['sensor_origin_in_tool_m'] = [0.,0.,0.]
    config['preprocessing']['granular_baseline_output'] = [0.]*6
    config['preprocessing']['zero_bias_sensor'] = [0.]*6
    config['preprocessing']['gravity_wrench_sensor'] = [0.]*6
    prep = WrenchPreprocessor.from_config(config['preprocessing'], tool_orientation=[0.,0.,np.pi/4])
    logger = ExperimentLogger(output, config, mode='simulation', strategy='continuous', workspace_logging=False)
    pose = np.array([0.,0.,0.,0.,0.,np.pi/4])
    events = {150:'FIRST_THRESHOLD_STOP_REQUEST', 190:'FIRST_CONTACT', 200:'TRACKING_ENTERED',
              400:'CONTACT_LOST', 460:'REACQUIRED', 600:'EXECUTION_STOP_CONFIRMED'}
    try:
        for i in range(601):
            t = i*.01
            state = ('PRECONTACT' if t<1 else 'TARGET_SEARCH' if t<1.5 else 'FIRST_CONTACT' if t<2 else
                     'CONTINUOUS_TRACKING' if t<4 else 'LOCAL_REACQUIRE' if t<4.6 else
                     'CONTINUOUS_TRACKING' if t<5.8 else 'STOP')
            sent = np.zeros(6); actual = np.zeros(6)
            if 1 <= t < 1.5:
                sent[0]=.008; actual[0]=min(.006,(t-1)*.04)
            elif 1.5<=t<1.7:
                actual[0]=max(0.,.006-(t-1.5)*.03)
            elif 2 <= t < 4 or 4.6<=t<5.8:
                sent[1]=.004; actual[1]=.0035+.0002*np.sin(t*20)
            elif 4<=t<4.6:
                sent[0]=.0005; actual[0]=.00035
            actual[3:]=[.001,-.002,.003]
            pose[:3] += actual[:3]*.01
            magnitude = (0. if t<1.5 or 4<=t<4.6 else 1.4+.12*np.sin(t*30))
            base = np.array([-magnitude, .2*magnitude, .05, .01,.02,.03])
            sensor = Wrench.from_sequence(np.r_[prep.rotation_sensor_to_output.T @ base[:3],
                                               prep.rotation_sensor_to_output.T @ base[3:]])
            processed = prep.process(sensor)
            logger.log_commands([dict(sequence=i+1, kind='simulation', velocity=sent.tolist(),
                host_monotonic=t+.0005, return_monotonic=t+.0006, accepted=True, source='synthetic_demo')])
            logger.log_sample(t, sensor, processed, RobotState(t, pose.copy(), actual),
                PolicyCommand(state, bool(np.linalg.norm(sent[:3])), np.array([1.,0.]),
                    float(np.linalg.norm(sent[:3])), pose.copy(), magnitude>=1, False), None, None,
                extra={**prep.force_log_fields, 'processed_force_frame':'Base'})
            if i in events:
                logger.log_waypoint(PolicyWaypoint(t,state,events[i],pose.copy()))
        logger.write_summary('STOP','synthetic demo completed',0)
    finally:
        logger.close()
    return logger.run_dir


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dirs', nargs='*', type=Path)
    parser.add_argument('--output', type=Path, help='write into separate run-named subdirectories')
    parser.add_argument('--demo', action='store_true', help='synthetic labelled data; no devices')
    args=parser.parse_args()
    if args.demo:
        if args.run_dirs:
            parser.error('--demo cannot be combined with existing run directories')
        args.run_dirs=[make_demo(args.output or ROOT/'data/analysis/contact_demo')]
        args.output=None
    if not args.run_dirs:
        parser.error('provide run directories or --demo')
    from visualization.run_plots import generate_run_visualization, load_trace
    from visualization.contact_report import report_data, json_finite
    comparisons=[]
    for directory in args.run_dirs:
        out=args.output/directory.name if args.output else None
        for path in generate_run_visualization(directory, force=True, output_dir=out):
            print(path)
        trace=load_trace(directory)
        comparisons.append(dict(run=str(directory.resolve()), **report_data(trace)))
    if args.output and len(comparisons)>1:
        path=args.output/'experiment_comparison.json'
        path.write_text(json.dumps(json_finite(comparisons),ensure_ascii=False,indent=2,allow_nan=False))
        print(path)


if __name__=='__main__':
    main()
