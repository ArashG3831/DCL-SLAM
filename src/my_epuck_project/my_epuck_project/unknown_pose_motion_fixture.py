"""Validation-only motion fixture for unknown-pose map overlap.

This node publishes ordinary per-robot velocity commands.  It is deliberately
not part of the production cooperative launch and does not read or publish
ground-truth pose, TF, or odometry.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


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
        self.clock_s = None
        self.started_s = None
        self.ready = False
        self._odom_seen = {'robot1': False, 'robot2': False}
        self.finished = False
        # The fixture is a validation-only local motion source.  Nav2's
        # command topics are continuously populated with zeroes while no goal
        # is active, so publishing on cmd_vel_nav or cmd_vel_smoothed races
        # those outputs and can suppress the entire course.  Publish a
        # stamped command directly to each robot's final ros2_control input;
        # this does not use peer pose/control or Supervisor state and leaves
        # the production launch and allocator unchanged.
        self.command_publishers = {
            'robot1': self.create_publisher(TwistStamped, '/robot1/cmd_vel', 10),
            'robot2': self.create_publisher(TwistStamped, '/robot2/cmd_vel', 10),
        }
        for robot in ('robot1', 'robot2'):
            self.create_subscription(
                Odometry, f'/{robot}/odom',
                lambda _message, robot_id=robot: self._odom_seen.__setitem__(
                    robot_id, True), 10)
        self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(
            'Unknown-pose validation motion fixture enabled; '
            'no ground-truth interfaces are used')

    def clock_callback(self, message):
        self.clock_s = message.clock.sec + message.clock.nanosec / 1.0e9

    def _check_readiness(self):
        """Hold the fixture until both wheel controllers are active.

        The fixture is diagnostic-only.  Its commands are local final
        controller inputs, so waiting for every Nav2 lifecycle service would
        unnecessarily serialize acquisition with the much slower navigation
        bring-up under WSL.  The runner records local Nav2 readiness separately.
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

    def publish(self, robot, linear=0.0, angular=0.0):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = f'{robot}/base_link'
        message.twist.linear.x = linear
        message.twist.angular.z = angular
        self.command_publishers[robot].publish(message)

    def tick(self):
        self._check_readiness()
        if not self.ready or self.clock_s is None:
            return
        if self.started_s is None:
            self.started_s = self.clock_s
            return
        elapsed = self.clock_s - self.started_s
        if elapsed < self.start_delay:
            phase = 'WAIT'
        else:
            active_elapsed = elapsed - self.start_delay
            cycle_duration = self.turn_duration + self.drive_duration
            cycle = int(active_elapsed // cycle_duration)
            if cycle >= self.cycles:
                phase = 'DONE'
            elif active_elapsed - cycle * cycle_duration < self.turn_duration:
                phase = 'TURN'
            else:
                phase = 'DRIVE'
        cycle = 0 if elapsed < self.start_delay else int(
            (elapsed - self.start_delay) /
            max(0.001, self.turn_duration + self.drive_duration))
        robot2_static = self.robot2_static or (
            self.robot2_static_after_first_cycle and cycle >= 1)
        turn_sign = -1.0 if cycle % 2 == 0 else 1.0
        if phase == 'TURN':
            self.publish('robot1', angular=turn_sign * self.angular_speed)
            self.publish('robot2', angular=(
                0.0 if robot2_static else
                (-turn_sign if self.mirror_turns else turn_sign) *
                self.angular_speed))
        elif phase == 'DRIVE':
            self.publish('robot1', linear=self.linear_speed)
            self.publish('robot2', linear=(
                0.0 if robot2_static else
                self.linear_speed * self.robot2_linear_scale))
        else:
            self.publish('robot1')
            self.publish('robot2')
        if phase == 'DONE' and not self.finished:
            self.finished = True
            self.get_logger().info(
                'Unknown-pose validation motion fixture complete cycles=%d' %
                self.cycles)


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
