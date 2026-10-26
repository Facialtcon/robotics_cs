"""Waypoint search only; contact confirmation and all post-contact behavior stay in the policy."""
import numpy as np

from experiment_logging.termination import TerminationReason


class CoverageSearch:
    def __init__(self, plan, start, position_tolerance):
        self.plan = plan
        self.points = np.asarray(plan['coverage_xy'], dtype=float)
        self.index = 0
        self.origin = np.asarray(start, dtype=float).copy()
        self.tolerance = float(position_tolerance)
        self.completed = False
        self._stopping_at_waypoint = False

    def observe(self, policy, robot):
        # The original first-contact sign check needs this leg's origin and
        # approach, including contacts on the short row connectors or entry.
        # Keep the last approach during braking/confirmation.
        if not self._stopping_at_waypoint:
            velocity = robot.tcp_speed[:2]
            if np.linalg.norm(velocity) > policy.c['settle_speed_mps']:
                policy.search_direction = velocity/np.linalg.norm(velocity)
            elif self.index < len(self.points):
                delta = self.points[self.index]-robot.pose[:2]
                if np.linalg.norm(delta) > self.tolerance:
                    policy.search_direction = delta/np.linalg.norm(delta)
        policy._start_pose = self.origin.copy()

    def command(self, policy, now, robot):
        pose = robot.pose
        target = self.points[self.index]
        distance = float(np.linalg.norm(target-pose[:2]))
        if distance <= self.tolerance or self._stopping_at_waypoint:
            self._stopping_at_waypoint = True
            policy.stop_requested = True
            settled = (np.linalg.norm(robot.tcp_speed[:3]) <= policy.c['settle_speed_mps'] and
                       (policy.execution_settled is None or policy.execution_settled))
            if not settled:
                return policy._command(pose)
            # Revisit the same endpoint if braking displaced the TCP out of tolerance.
            self._stopping_at_waypoint = False
            if distance > self.tolerance:
                return policy._command(pose)
            policy._event(now, pose, 'COVERAGE_WAYPOINT_REACHED')
            self.index += 1
            self.origin = pose.copy()
            if self.index == len(self.points):
                self.completed = True
                policy.request_stop(now, pose, '区域扫描完成，未检测到目标',
                    code=TerminationReason.SUCCESS, event='COVERAGE_COMPLETE')
            return policy._command(pose)
        policy.stop_requested = False
        # Approach each endpoint gently and stop before reversing. Existing
        # _command retains its acceleration and global speed limiting.
        nominal = float(policy.c['search_speed'])
        approach_time = max(1., 2*nominal/float(policy.c['command_acceleration']))
        speed = min(nominal, distance/approach_time,
                    float(np.sqrt(2*policy.c['command_acceleration']*distance)))
        return policy._command(pose, (target-pose[:2])/distance*speed)

    def summary(self, policy):
        return dict(completed=self.completed, reached_waypoints=self.index,
            planned_waypoints=len(self.points), exit_reason=policy.reason,
            coverage_exit_reason=('first_contact_confirmed' if policy.initial_contact is not None else
                                  'coverage_complete_no_contact' if self.completed else 'run_stopped_before_contact'),
            first_contact_state='CONTINUOUS_TRACKING' if policy.initial_contact is not None else None,
            first_contact_pose=None if policy.initial_contact is None else policy.initial_contact.tolist())
