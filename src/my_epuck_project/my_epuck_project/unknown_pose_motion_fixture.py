"""Validation-only motion fixture for unknown-pose map overlap.

This node publishes ordinary per-robot velocity commands.  It is deliberately
not part of the production cooperative launch and does not read or publish
ground-truth pose, TF, or odometry.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class UnknownPoseMotionFixture(Node):
    def __init__(self):
        super().__init__('unknown_pose_motion_fixture')
        self.declare_parameter('start_delay_s', 20.0)
        self.declare_parameter('turn_duration_s', 3.2)
        self.declare_parameter('drive_duration_s', 12.0)
        self.declare_parameter('linear_speed', 0.10)
        self.declare_parameter('angular_speed', 0.45)
        self.start_delay = float(self.get_parameter('start_delay_s').value)
        self.turn_duration = float(self.get_parameter('turn_duration_s').value)
        self.drive_duration = float(self.get_parameter('drive_duration_s').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.clock_s = None
        self.started_s = None
        self.finished = False
        self.command_publishers = {
            'robot1': self.create_publisher(Twist, '/robot1/cmd_vel_unstamped', 10),
            'robot2': self.create_publisher(Twist, '/robot2/cmd_vel_unstamped', 10),
        }
        self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(
            'Unknown-pose validation motion fixture enabled; '
            'no ground-truth interfaces are used')

    def clock_callback(self, message):
        self.clock_s = message.clock.sec + message.clock.nanosec / 1.0e9
        if self.started_s is None and self.clock_s > 0.0:
            self.started_s = self.clock_s

    def publish(self, robot, linear=0.0, angular=0.0):
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.command_publishers[robot].publish(message)

    def tick(self):
        if self.started_s is None:
            return
        elapsed = self.clock_s - self.started_s
        if elapsed < self.start_delay:
            phase = 'WAIT'
        elif elapsed < self.start_delay + self.turn_duration:
            phase = 'TURN'
        elif elapsed < self.start_delay + self.turn_duration + self.drive_duration:
            phase = 'DRIVE'
        else:
            phase = 'DONE'
        if phase == 'TURN':
            self.publish('robot1', angular=-self.angular_speed)
            self.publish('robot2', angular=self.angular_speed)
        elif phase == 'DRIVE':
            self.publish('robot1', linear=self.linear_speed)
            self.publish('robot2', linear=self.linear_speed)
        else:
            self.publish('robot1')
            self.publish('robot2')
        if phase == 'DONE' and not self.finished:
            self.finished = True
            self.get_logger().info('Unknown-pose validation motion fixture complete')


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
