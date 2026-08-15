#!/usr/bin/env python3
"""Passive high-rate autonomous turn diagnostics.

This executable never publishes commands or state.  It observes an existing
robot's command stages, wheel JointState, odometry, Nav2 feedback and
Collision Monitor state, and writes a bounded simulation-time CSV.  Supervisor
truth is intentionally recorded by the separate external observer.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from nav2_msgs.msg import CollisionMonitorState


def stamp_s(msg, fallback):
    header = getattr(msg, 'header', None)
    if header is None:
        return fallback
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TurnMotionDiagnostic(Node):
    def __init__(self, robot, output, max_rows, duration_s, geometry, flush_every=25):
        super().__init__('turn_motion_diagnostics')
        self.robot = robot
        self.output = os.path.abspath(output)
        self.max_rows = int(max_rows)
        self.duration_s = float(duration_s)
        self.radius = float(geometry['wheel_radius_m'])
        self.separation = float(geometry['wheel_separation_m'])
        self.multiplier = float(geometry['wheel_separation_multiplier'])
        self.effective_separation = self.separation * self.multiplier
        os.makedirs(os.path.dirname(self.output), exist_ok=True)
        self.clock_s = None
        self.first_s = None
        self.rows = 0
        self.dropped = 0
        self.flush_every = max(1, int(flush_every))
        self.latest = {
            'controller_raw_vx': None, 'controller_raw_wz': None,
            'smoother_output_vx': None, 'smoother_output_wz': None,
            'collision_output_vx': None, 'collision_output_wz': None,
            'final_command_vx': None, 'final_command_wz': None,
            'odom_x': None, 'odom_y': None, 'odom_yaw': None,
            'odom_vx': None, 'odom_wz': None,
            'left_position': None, 'right_position': None,
            'left_velocity': None, 'right_velocity': None,
            'joint_stamp_s': None, 'odom_stamp_s': None,
            'nav_recoveries': None, 'nav_distance_remaining_m': None,
            'collision_action_type': None, 'collision_polygon': None,
        }
        self.fields = [
            'sim_time_s', 'wall_elapsed_s', 'robot_id',
            'controller_raw_vx', 'controller_raw_wz',
            'smoother_output_vx', 'smoother_output_wz',
            'collision_output_vx', 'collision_output_wz',
            'final_command_vx', 'final_command_wz',
            'left_position_rad', 'right_position_rad',
            'left_velocity_radps', 'right_velocity_radps',
            'wheel_predicted_vx_mps', 'wheel_predicted_wz_controller_radps',
            'wheel_predicted_wz_physical_radps',
            'odom_x_m', 'odom_y_m', 'odom_yaw_rad',
            'odom_vx_mps', 'odom_wz_radps',
            'odom_stamp_s', 'joint_stamp_s',
            'nav_recoveries', 'nav_distance_remaining_m',
            'collision_action_type', 'collision_polygon',
        ]
        self.stream = open(self.output, 'w', newline='', encoding='utf-8')
        self.writer = csv.DictWriter(self.stream, fieldnames=self.fields)
        self.writer.writeheader()
        self.stream.flush()
        self.wall_start = time.monotonic()
        self.stop_requested = False
        prefix = f'/{robot}'
        qos = 20
        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self.create_subscription(Twist, f'{prefix}/cmd_vel_nav',
                                 lambda m: self._twist('controller_raw', m), qos)
        self.create_subscription(Twist, f'{prefix}/cmd_vel_smoothed',
                                 lambda m: self._twist('smoother_output', m), qos)
        self.create_subscription(Twist, f'{prefix}/cmd_vel_unstamped',
                                 lambda m: self._twist('collision_output', m), qos)
        self.create_subscription(TwistStamped, f'{prefix}/cmd_vel',
                                 lambda m: self._stamped_command(m), qos)
        self.create_subscription(JointState, f'{prefix}/joint_states',
                                 self._joints, qos)
        self.create_subscription(Odometry, f'{prefix}/odom', self._odom, qos)
        self.create_subscription(
            NavigateToPose_FeedbackMessage,
            f'{prefix}/navigate_to_pose/_action/feedback', self._feedback, 10)
        self.create_subscription(
            CollisionMonitorState, f'{prefix}/collision_monitor_state',
            self._collision, 10)
        # JointState is the highest-rate stable observation.  Persist one row
        # for every wheel-state callback, bounded by max_rows.

    def _on_clock(self, msg):
        self.clock_s = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9
        if self.first_s is None and self.clock_s > 0.0:
            self.first_s = self.clock_s
        if self.first_s is not None and self.clock_s - self.first_s >= self.duration_s:
            self.stop_requested = True

    def _twist(self, stage, msg):
        self.latest[f'{stage}_vx'] = float(msg.linear.x)
        self.latest[f'{stage}_wz'] = float(msg.angular.z)

    def _stamped_command(self, msg):
        self.latest['final_command_vx'] = float(msg.twist.linear.x)
        self.latest['final_command_wz'] = float(msg.twist.angular.z)

    def _joints(self, msg):
        try:
            li = msg.name.index('left wheel motor')
            ri = msg.name.index('right wheel motor')
        except ValueError:
            return
        if li >= len(msg.position) or ri >= len(msg.position):
            return
        self.latest['left_position'] = float(msg.position[li])
        self.latest['right_position'] = float(msg.position[ri])
        if li < len(msg.velocity) and ri < len(msg.velocity):
            self.latest['left_velocity'] = float(msg.velocity[li])
            self.latest['right_velocity'] = float(msg.velocity[ri])
        self.latest['joint_stamp_s'] = stamp_s(msg, self.clock_s or 0.0)
        self._write_row(self.latest['joint_stamp_s'])

    def _odom(self, msg):
        self.latest['odom_x'] = float(msg.pose.pose.position.x)
        self.latest['odom_y'] = float(msg.pose.pose.position.y)
        self.latest['odom_yaw'] = yaw_from_quaternion(msg.pose.pose.orientation)
        self.latest['odom_vx'] = float(msg.twist.twist.linear.x)
        self.latest['odom_wz'] = float(msg.twist.twist.angular.z)
        self.latest['odom_stamp_s'] = stamp_s(msg, self.clock_s or 0.0)

    def _feedback(self, msg):
        feedback = msg.feedback
        self.latest['nav_recoveries'] = int(feedback.number_of_recoveries)
        self.latest['nav_distance_remaining_m'] = float(feedback.distance_remaining)

    def _collision(self, msg):
        self.latest['collision_action_type'] = int(msg.action_type)
        self.latest['collision_polygon'] = str(msg.polygon_name)

    def _write_row(self, t):
        if self.stop_requested or self.first_s is None:
            return
        if self.rows >= self.max_rows:
            self.dropped += 1
            return
        now = self.clock_s if self.clock_s is not None else t
        left = self.latest['left_velocity']
        right = self.latest['right_velocity']
        wheel_vx = ''
        wheel_wz_controller = ''
        wheel_wz_physical = ''
        if left is not None and right is not None:
            wheel_vx = self.radius * (right + left) / 2.0
            wheel_wz_controller = self.radius * (right - left) / self.effective_separation
            wheel_wz_physical = self.radius * (right - left) / self.separation
        data = {
            'sim_time_s': now, 'wall_elapsed_s': time.monotonic() - self.wall_start,
            'robot_id': self.robot,
            'controller_raw_vx': self.latest['controller_raw_vx'],
            'controller_raw_wz': self.latest['controller_raw_wz'],
            'smoother_output_vx': self.latest['smoother_output_vx'],
            'smoother_output_wz': self.latest['smoother_output_wz'],
            'collision_output_vx': self.latest['collision_output_vx'],
            'collision_output_wz': self.latest['collision_output_wz'],
            'final_command_vx': self.latest['final_command_vx'],
            'final_command_wz': self.latest['final_command_wz'],
            'left_position_rad': self.latest['left_position'],
            'right_position_rad': self.latest['right_position'],
            'left_velocity_radps': left, 'right_velocity_radps': right,
            'wheel_predicted_vx_mps': wheel_vx,
            'wheel_predicted_wz_controller_radps': wheel_wz_controller,
            'wheel_predicted_wz_physical_radps': wheel_wz_physical,
            'odom_x_m': self.latest['odom_x'], 'odom_y_m': self.latest['odom_y'],
            'odom_yaw_rad': self.latest['odom_yaw'],
            'odom_vx_mps': self.latest['odom_vx'], 'odom_wz_radps': self.latest['odom_wz'],
            'odom_stamp_s': self.latest['odom_stamp_s'],
            'joint_stamp_s': self.latest['joint_stamp_s'],
            'nav_recoveries': self.latest['nav_recoveries'],
            'nav_distance_remaining_m': self.latest['nav_distance_remaining_m'],
            'collision_action_type': self.latest['collision_action_type'],
            'collision_polygon': self.latest['collision_polygon'],
        }
        self.writer.writerow(data)
        self.rows += 1
        if self.rows % self.flush_every == 0:
            self.stream.flush()

    def close(self):
        if self.stream.closed:
            return
        self.stream.flush()
        self.stream.close()
        metadata = {
            'robot_id': self.robot, 'rows': self.rows, 'dropped_rows': self.dropped,
            'duration_s': self.duration_s,
            'wheel_radius_m': self.radius,
            'wheel_separation_m': self.separation,
            'wheel_separation_multiplier': self.multiplier,
            'effective_wheel_separation_m': self.effective_separation,
            'source': 'diagnostic-only passive ROS observer',
            'supervisor_source': 'external Webots Supervisor CSV joined offline',
        }
        with open(self.output + '.json', 'w', encoding='utf-8') as fp:
            json.dump(metadata, fp, indent=2)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', default='robot1')
    parser.add_argument('--output', required=True)
    parser.add_argument('--duration-s', type=float, default=300.0)
    parser.add_argument('--max-rows', type=int, default=200000)
    parser.add_argument('--wheel-radius-m', type=float, default=0.02)
    parser.add_argument('--wheel-separation-m', type=float, default=0.052)
    parser.add_argument('--wheel-separation-multiplier', type=float, default=1.095)
    parser.add_argument('--flush-every', type=int, default=25)
    args = parser.parse_args(argv)
    rclpy.init(args=None)
    node = TurnMotionDiagnostic(args.robot, args.output, args.max_rows,
                                args.duration_s, vars(args), args.flush_every)
    def stop(_sig, _frame):
        node.stop_requested = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f'TURN_DIAGNOSTICS_COMPLETE output={args.output} rows={node.rows} dropped={node.dropped}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
