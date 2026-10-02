#!/usr/bin/env python3
"""Offline recorded-input replay; no devices and no post-log extrapolation."""
import argparse
from copy import deepcopy
import csv
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import yaml

from core.models import RobotState, Wrench
from policy.continuous_tracking import ContinuousTrackingPolicy, State, UNLOAD_DEFAULTS


def replay(run_dir):
    directory = Path(run_dir)
    config = yaml.safe_load((directory/'config_snapshot.yaml').read_text())
    with (directory/'samples.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    original = json.loads((directory/'termination.json').read_text())
    result = dict(source=str(directory.resolve()), sample_count=len(rows),
        skipped_return_samples=sum(row['current_state'] == 'RETURN_TO_START' for row in rows),
        original_termination_reason=original['reason'], original_termination_detail=original['detail'],
        original_configuration_sha=config.get('continuous_provenance', {}).get('commit'),
        new_unload_defaults=UNLOAD_DEFAULTS,
        interpretation='Recorded inputs, not a closed-loop prediction after changed commands; no physical direction verification and no extrapolation beyond last valid frame.',
        cases={})
    for case in ['saved_configuration']:
        policy = ContinuousTrackingPolicy(deepcopy(config))
        max_error, first_change, state_differences = 0., None, 0
        first_limits = {}
        replayed_samples, recorded_stop_comparison = 0, None
        for index, row in enumerate(rows):
            # Startup return has its own executor and no policy read intervals.
            # Feeding it to this policy also invents a pre-scan search history.
            if row['current_state'] == 'RETURN_TO_START':
                continue
            replayed_samples += 1
            now = float(row['monotonic_sec'])
            raw = Wrench.from_sequence([float(row[k]) for k in ('raw_fx','raw_fy','raw_fz','raw_tx','raw_ty','raw_tz')])
            processed = Wrench.from_sequence([float(row[k]) for k in ('dfx','dfy','dfz','dtx','dty','dtz')])
            robot = RobotState(float(row['tcp_read_end']),
                np.array([float(row[k]) for k in ('tcp_x','tcp_y','tcp_z','tcp_rx','tcp_ry','tcp_rz')]),
                np.array([float(row[k]) for k in ('tcp_vx','tcp_vy','tcp_vz','tcp_vrx','tcp_vry','tcp_vrz')]))
            command = policy.update(now, raw, processed, robot, execution_settled=row['runtime_stop_state']=='STOPPED')
            error = float(np.linalg.norm(command.direction_xy*command.speed-
                         [float(row['command_vx']), float(row['command_vy'])]))
            state_differences += command.state != row['current_state']
            max_error = max(max_error, error)
            if first_change is None and (error > 1e-10 or command.state != row['current_state']):
                first_change = dict(csv_line=index+2, host_monotonic=now,
                                    old_state=row['current_state'], new_state=command.state)
            if policy.tangent_limit_reason:
                first_limits.setdefault(policy.tangent_limit_reason, now)
            if row['current_state'] == 'STOP' and recorded_stop_comparison is None:
                telemetry = policy.telemetry(command)
                recorded_stop_comparison = dict(csv_line=index+2, host_monotonic=now,
                    old_reason=row['reason'], new_state=command.state, new_reason=command.reason,
                    old_reacquire_path_length_m=float(row['reacquire_path_length']),
                    new_reacquire_path_length_m=policy.reacquire_path_length,
                    new_path_diagnostics={key: value for key, value in telemetry.items()
                        if key in ('reacquire_raw_pose_path_length', 'reacquire_speed_path_length',
                                   'reacquire_max_displacement')})
        result['cases'][case] = dict(final_state=policy.state.value,
            replayed_samples=replayed_samples, recorded_stop_comparison=recorded_stop_comparison,
            stop_reason=None if policy.stop_reason is None else policy.stop_reason.value,
            state_differences=state_differences, max_command_difference_mps=max_error,
            first_changed_output=first_change, current_limit_first_seen=first_limits,
            unload_elapsed_sec=policy.unload_elapsed, unload_displacement_m=policy.unload_displacement,
            events=[dict(name=e.event_type, host_monotonic=e.timestamp) for e in policy.events])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    text = json.dumps(replay(args.run_dir), ensure_ascii=False, indent=2)+'\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding='utf-8')
    else:
        print(text, end='')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
