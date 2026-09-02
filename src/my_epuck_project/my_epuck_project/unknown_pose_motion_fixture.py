"""Validation-only motion fixture for unknown-pose map overlap.

This node publishes ordinary per-robot velocity commands.  It is deliberately
not part of the production cooperative launch and does not read or publish
ground-truth pose, TF, or odometry.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rosgraph_msgs.msg import Clock

from my_epuck_interfaces.msg import DistributedExplorationStatus
from std_msgs.msg import Bool


class UnknownPoseMotionFixture(Node):
    def __init__(self):
        super().__init__('unknown_pose_motion_fixture')
        self.declare_parameter('start_delay_s', 20.0)
        self.declare_parameter('turn_duration_s', 3.2)
        self.declare_parameter('drive_duration_s', 12.0)
        self.declare_parameter('cycles', 1)
        self.declare_parameter('mirror_turns', True)
        self.declare_parameter('robot2_static', False)
        self.declare_parameter('robot2_static_after_first_cycle', False)
        self.declare_parameter('linear_speed', 0.10)
        self.declare_parameter('robot2_linear_scale', 1.0)
        self.declare_parameter('angular_speed', 0.45)
        self.declare_parameter('pending_timeout_s', 90.0)
        # Full-map unknown-pose registration does not use the legacy
        # evidence-acquisition lease.  The synchronized traffic fixture may
        # opt into the same bounded maneuver after both status streams are
        # present, but only through an explicit test-only launch combination.
        self.declare_parameter('synchronized_traffic_test', False)
        self.declare_parameter('hold_prehandoff_motion', False)
        self.start_delay = float(self.get_parameter('start_delay_s').value)
        self.turn_duration = float(self.get_parameter('turn_duration_s').value)
        self.drive_duration = float(self.get_parameter('drive_duration_s').value)
        self.cycles = max(1, int(self.get_parameter('cycles').value))
        self.mirror_turns = bool(self.get_parameter('mirror_turns').value)
        self.robot2_static = bool(self.get_parameter('robot2_static').value)
        self.robot2_static_after_first_cycle = bool(
            self.get_parameter('robot2_static_after_first_cycle').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.robot2_linear_scale = float(
            self.get_parameter('robot2_linear_scale').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.pending_timeout_s = max(
            1.0, float(self.get_parameter('pending_timeout_s').value))
        self.synchronized_traffic_test = bool(
            self.get_parameter('synchronized_traffic_test').value)
        self.hold_prehandoff_motion = bool(
            self.get_parameter('hold_prehandoff_motion').value)
        self.clock_s = None
        self.started_s = None
        self.ready = False
        self._odom_seen = {'robot1': False, 'robot2': False}
        self._odom_pose = {'robot1': None, 'robot2': None}
        self._odom_last_stamp_s = {'robot1': None, 'robot2': None}
        self._odom_update_count = {'robot1': 0, 'robot2': 0}
        self._observation_pose = {'robot1': None, 'robot2': None}
        self._drive_pose = {'robot1': None, 'robot2': None}
        self._turn_confirmed = {'robot1': False, 'robot2': False}
        self._command_counts = {'robot1': 0, 'robot2': 0}
        self._evidence_active = {'robot1': False, 'robot2': False}
        self._navigation_active = {'robot1': None, 'robot2': None}
        self._status_seen = {'robot1': False, 'robot2': False}
        self._observation_started = False
        self._maneuver_started = False
        self._pending_since_s = None
        self._last_pending_log_s = None
        self._post_goal_odom_counts = None
        self._maneuver_odom_counts = {'robot1': 0, 'robot2': 0}
        self._drive_odom_counts = {'robot1': 0, 'robot2': 0}
        self._motion_start_s = None
        self._expected_turn_sign = {'robot1': -1.0, 'robot2': 1.0}
        self._first_nonzero_command_s = None
        self._last_maneuver_freshness_check_s = None
        self._observation_finished = False
        self._stop_published = False
        self.finished = False
        # The fixture is a validation-only local motion source.  Feed the
        # dedicated evidence input consumed by the existing twist stamper so
        # it remains the sole publisher of the final controller topic.  This
        # avoids racing both Nav2/collision-monitor output and the final
        # stamped command topic.
        self.command_publishers = {
            'robot1': self.create_publisher(
                Twist, '/robot1/evidence_cmd_vel', 10),
            'robot2': self.create_publisher(
                Twist, '/robot2/evidence_cmd_vel', 10),
        }
        for robot in ('robot1', 'robot2'):
            self.create_subscription(
                Odometry, f'/{robot}/odom',
                lambda message, robot_id=robot: self._odom_callback(
                    robot_id, message), 10)
        evidence_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        status_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for robot in ('robot1', 'robot2'):
            self.create_subscription(
                Bool,
                f'/cslam/relative_pose/{robot}/'
                'evidence_acquisition_active',
                lambda message, robot_id=robot:
                self._evidence_active.__setitem__(
                    robot_id, bool(message.data)), evidence_qos)
            self.create_subscription(
                DistributedExplorationStatus,
                f'/{robot}/distributed_status',
                lambda message, robot_id=robot: self._status_callback(
                    robot_id, message), status_qos)
        # Webots publishes /clock with sensor-data (best-effort) QoS.  A
        # reliable subscription silently receives nothing and leaves the
        # evidence maneuver permanently idle after readiness.
        self.create_subscription(
            Clock, '/clock', self.clock_callback, qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(
            'Unknown-pose validation motion fixture enabled; '
            'no ground-truth interfaces are used')

    def clock_callback(self, message):
        self.clock_s = message.clock.sec + message.clock.nanosec / 1.0e9

    def _check_readiness(self):
        """Hold the fixture until both wheel controllers are active.

        The fixture is diagnostic-only.  Its commands enter through the
        established per-robot twist-stamper input, so waiting for every Nav2
        lifecycle service would unnecessarily serialize acquisition with the
        much slower navigation bring-up under WSL.  The runner records local
        Nav2 readiness separately.
        """
        if self.ready:
            return
        if not all(self._odom_seen.values()):
            return
        self.ready = True
        if self.clock_s is not None:
            self.started_s = self.clock_s
        self.get_logger().info(
            'Unknown-pose validation motion fixture readiness confirmed: '
            'both odom topics publishing')

    @staticmethod
    def _yaw_from_quaternion(orientation):
        return math.atan2(
            2.0 * (orientation.w * orientation.z +
                   orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y +
                         orientation.z * orientation.z))

    def _odom_callback(self, robot, message):
        pose = message.pose.pose
        self._odom_seen[robot] = True
        stamp = message.header.stamp
        self._odom_last_stamp_s[robot] = (
            float(stamp.sec) + float(stamp.nanosec) / 1.0e9)
        self._odom_update_count[robot] += 1
        self._odom_pose[robot] = (
            float(pose.position.x),
            float(pose.position.y),
            self._yaw_from_quaternion(pose.orientation))

    def publish(self, robot, linear=0.0, angular=0.0):
        nonzero = abs(linear) > 1.0e-6 or abs(angular) > 1.0e-6
        if (nonzero and self._maneuver_started and
                self._first_nonzero_command_s is None):
            self._first_nonzero_command_s = self.clock_s
            self.get_logger().info(
                'EVIDENCE_OBSERVATION_FIRST_NONZERO_COMMAND '
                'sim_time_s=%.6f robot=%s topic=/%s/evidence_cmd_vel '
                'linear_mps=%.6f angular_radps=%.6f' % (
                    self.clock_s, robot, robot, linear, angular))
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.command_publishers[robot].publish(message)
        if nonzero:
            self._command_counts[robot] += 1

    def _status_callback(self, robot, message):
        self._navigation_active[robot] = bool(message.local_nav_goal_active)
        self._status_seen[robot] = True

    def _publish_stop_once(self):
        if self._stop_published:
            return
        self.publish('robot1')
        self.publish('robot2')
        self._stop_published = True

    @staticmethod
    def _pose_delta(start, current):
        if start is None or current is None:
            return None
        dx = current[0] - start[0]
        dy = current[1] - start[1]
        dyaw = (current[2] - start[2] + math.pi) % (2.0 * math.pi) - math.pi
        return math.hypot(dx, dy), abs(dyaw), dx, dy, dyaw

    def _motion_failure(self, robot, phase, delta, minimum):
        if not self._odom_is_fresh(robot, phase):
            detail = 'odom_stale_or_not_updated'
        elif delta is None:
            detail = 'odom_unavailable'
        else:
            detail = (
                'translation_m=%.6f yaw_rad=%.6f required=%.6f' %
                (delta[0], delta[1], minimum))
        self.get_logger().error(
            'EVIDENCE_OBSERVATION_MOTION_FAILED robot=%s phase=%s %s' %
            (robot, phase, detail))

    def _odom_is_fresh(self, robot, phase):
        baseline = (self._maneuver_odom_counts if phase == 'TURN'
                    else self._drive_odom_counts)[robot]
        stamp = self._odom_last_stamp_s[robot]
        return (
            self.clock_s is not None and stamp is not None and
            self._odom_update_count[robot] > baseline and
            0.0 <= self.clock_s - stamp <= 0.5)

    def _odom_is_fresh_after(self, robot, counts):
        stamp = self._odom_last_stamp_s[robot]
        return (
            self.clock_s is not None and stamp is not None and
            self._odom_update_count[robot] > counts[robot] and
            0.0 <= self.clock_s - stamp <= 0.5)

    def _confirm_turn_motion(self):
        minimum = max(0.20, 0.20 * abs(self.angular_speed) *
                      max(0.001, self.turn_duration))
        failed = False
        for robot in ('robot1', 'robot2'):
            if robot == 'robot2' and self.robot2_static:
                self._turn_confirmed[robot] = True
                continue
            delta = self._pose_delta(
                self._observation_pose[robot], self._odom_pose[robot])
            if (not self._odom_is_fresh(robot, 'TURN') or
                    delta is None or
                    self._expected_turn_sign[robot] * delta[4] < minimum):
                self._motion_failure(robot, 'TURN', delta, minimum)
                failed = True
            else:
                self._turn_confirmed[robot] = True
        return not failed

    def _confirm_drive_motion(self):
        minimum = max(0.10, 0.20 * abs(self.linear_speed) *
                      max(0.001, self.drive_duration))
        failed = False
        for robot in ('robot1', 'robot2'):
            if robot == 'robot2' and self.robot2_static:
                continue
            delta = self._pose_delta(self._drive_pose[robot], self._odom_pose[robot])
            if (not self._odom_is_fresh(robot, 'DRIVE') or
                    delta is None or delta[0] < minimum):
                self._motion_failure(robot, 'DRIVE', delta, minimum)
                failed = True
        return not failed

    def _abort_motion(self):
        self._publish_stop_once()
        self._observation_finished = True
        self.finished = True
        self.get_logger().error(
            'EVIDENCE_OBSERVATION_ABORTED reason=motion_confirmation_failed '
            'active_goals_untouched=true')

    def _observation_can_start(self):
        """Require a live overlap signal and both status streams."""
        if not all(self._status_seen.values()):
            return False
        if self.synchronized_traffic_test and self.hold_prehandoff_motion:
            return True
        return any(self._evidence_active.values())

    def _finish_deferred(self, reason):
        self._publish_stop_once()
        self._observation_finished = True
        self.finished = True
        self.get_logger().warning(
            '%s active_goals_untouched=true lease_release=normal' % reason)

    def _wait_for_inactive_goals(self):
        """Keep the evidence phase pending without competing with Nav2."""
        waited = self.clock_s - self._pending_since_s
        if waited >= self.pending_timeout_s:
            self._finish_deferred(
                'EVIDENCE_OBSERVATION_DEFERRED_ACTIVE_GOAL_TIMEOUT')
            return True
        if (self._last_pending_log_s is None or
                self.clock_s - self._last_pending_log_s >= 1.0):
            active = ','.join(
                robot for robot in ('robot1', 'robot2')
                if self._navigation_active[robot] is True)
            self.get_logger().info(
                'EVIDENCE_OBSERVATION_PENDING active_robots=%s '
                'waited_s=%.3f deadline_s=%.3f' % (
                    active or 'none', waited, self.pending_timeout_s))
            self._last_pending_log_s = self.clock_s
        return False

    def tick(self):
        self._check_readiness()
        if not self.ready or self.clock_s is None:
            return
        if self._observation_finished:
            return
        if not self._observation_started:
            if not self._observation_can_start():
                return
            self._observation_started = True
            self._pending_since_s = self.clock_s
            self.get_logger().info(
                'EVIDENCE_OBSERVATION_STARTED trigger=peer_overlap '
                'pending_until_inactive=true active_goals_untouched=true '
                'symmetric=true')
        if any(active is True for active in self._navigation_active.values()):
            if self._maneuver_started:
                # A goal accepted concurrently with the lease must not be
                # cancelled or competed with.  Stop the evidence input and
                # return to pending; take a fresh odometry baseline only after
                # both coordinators report inactive again.
                self._publish_stop_once()
                self._maneuver_started = False
                self._post_goal_odom_counts = None
                self._observation_pose = {'robot1': None, 'robot2': None}
                self._drive_pose = {'robot1': None, 'robot2': None}
                self._turn_confirmed = {'robot1': False, 'robot2': False}
                self.get_logger().warning(
                    'EVIDENCE_OBSERVATION_PENDING active_goal_detected '
                    'evidence_commands_stopped=true active_goals_untouched=true')
            self._wait_for_inactive_goals()
            return
        if self._post_goal_odom_counts is None:
            # Status transitions and odometry callbacks are independent.  Do
            # not use the last odometry sample as the maneuver baseline: first
            # require a new sample after both Nav2 goals became inactive.
            self._post_goal_odom_counts = dict(self._odom_update_count)
            self.get_logger().info(
                'EVIDENCE_OBSERVATION_WAITING_FOR_FRESH_ODOM '
                'active_goals=false')
            return
        if not all(self._odom_is_fresh_after(robot, self._post_goal_odom_counts)
                   for robot in ('robot1', 'robot2')):
            return
        if not self._maneuver_started:
            self._maneuver_started = True
            self.started_s = self.clock_s
            self._motion_start_s = self.clock_s
            self._observation_pose = dict(self._odom_pose)
            self._drive_pose = {'robot1': None, 'robot2': None}
            self._turn_confirmed = {'robot1': False, 'robot2': False}
            self._maneuver_odom_counts = dict(self._odom_update_count)
            self._drive_odom_counts = dict(self._odom_update_count)
            self._stop_published = False
            self._first_nonzero_command_s = None
            self._last_maneuver_freshness_check_s = self.clock_s
            turn_sign = -1.0
            self._expected_turn_sign = {
                'robot1': turn_sign,
                'robot2': (-turn_sign if self.mirror_turns else turn_sign),
            }
            self.get_logger().info(
                'EVIDENCE_OBSERVATION_MOTION_STARTED pending_wait_s=%.3f '
                'active_goals=false symmetric=true sim_time_s=%.6f '
                'baseline_robot1=(%.6f,%.6f,%.6f) '
                'baseline_robot2=(%.6f,%.6f,%.6f)' % (
                    self.clock_s - self._pending_since_s, self.clock_s,
                    self._observation_pose['robot1'][0],
                    self._observation_pose['robot1'][1],
                    self._observation_pose['robot1'][2],
                    self._observation_pose['robot2'][0],
                    self._observation_pose['robot2'][1],
                    self._observation_pose['robot2'][2]))
        elapsed = self.clock_s - self.started_s
        if (elapsed > 0.05 and
                not all(self._odom_is_fresh(robot, 'TURN')
                        for robot in ('robot1', 'robot2'))):
            stale = next(
                robot for robot in ('robot1', 'robot2')
                if not self._odom_is_fresh(robot, 'TURN'))
            self._motion_failure(stale, 'MANEUVER', None, 0.0)
            self._abort_motion()
            return
        cycle_duration = self.turn_duration + self.drive_duration
        cycle = int(elapsed // max(0.001, cycle_duration))
        if cycle >= self.cycles:
            if not self._confirm_drive_motion():
                self._abort_motion()
                return
            self.get_logger().info(
                'MOTION_CONFIRMED sim_time_s=%.6f '
                'fresh_odom_robot1=true fresh_odom_robot2=true' % self.clock_s)
            self._publish_stop_once()
            self._observation_finished = True
            self.finished = True
            self.get_logger().info(
                'EVIDENCE_OBSERVATION_FINISHED cycles=%d '
                'commands_robot1=%d commands_robot2=%d '
                'motion_confirmed=true' % (
                    self.cycles, self._command_counts['robot1'],
                    self._command_counts['robot2']))
            return
        phase_elapsed = elapsed - cycle * cycle_duration
        robot2_static = self.robot2_static or (
            self.robot2_static_after_first_cycle and cycle >= 1)
        turn_sign = -1.0 if cycle % 2 == 0 else 1.0
        if phase_elapsed < self.turn_duration:
            self.publish('robot1', angular=turn_sign * self.angular_speed)
            self.publish('robot2', angular=(
                0.0 if robot2_static else
                (-turn_sign if self.mirror_turns else turn_sign) *
                self.angular_speed))
        else:
            if cycle == 0 and not all(self._turn_confirmed.values()):
                if not self._confirm_turn_motion():
                    self._abort_motion()
                    return
                self._drive_pose = dict(self._odom_pose)
                self._drive_odom_counts = dict(self._odom_update_count)
            self.publish('robot1', linear=self.linear_speed)
            self.publish('robot2', linear=(
                0.0 if robot2_static else
                self.linear_speed * self.robot2_linear_scale))


def main(args=None):
    rclpy.init(args=args)
    node = UnknownPoseMotionFixture()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
