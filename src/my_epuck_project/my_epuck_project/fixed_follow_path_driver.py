"""Diagnostic fixed-route FollowPath action driver.

This node is deliberately small: it sends the same deterministic sequence of
nav2_msgs/FollowPath goals for every controller variant.  It never consumes
Supervisor data and never publishes velocity commands.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.node import Node


def _pose(x, y, yaw, frame):
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def _line(x0, y0, x1, y1, n=24):
    yaw = math.atan2(y1 - y0, x1 - x0)
    return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n, yaw)
            for i in range(n + 1)]


def _arc(cx, cy, radius, a0, a1, n=14):
    sign = 1.0 if a1 >= a0 else -1.0
    return [(cx + radius * math.cos(a0 + (a1 - a0) * i / n),
             cy + radius * math.sin(a0 + (a1 - a0) * i / n),
             a0 + (a1 - a0) * i / n + sign * math.pi / 2.0)
            for i in range(n + 1)]


def _local_route():
    """Two identical rounded rectangular laps, about 21 m total."""
    segments = []
    def add(points, label):
        # Remove the duplicated first point at a segment join while retaining
        # the exact route definition in the metadata.
        segments.append((label, points))

    lap = [
        ('line_east', _line(0.0, 0.0, 1.5, 0.0)),
        ('arc_left_bottom', _arc(1.5, 0.3, 0.3, -math.pi / 2, 0.0)),
        ('line_north', _line(1.8, 0.3, 1.8, 1.5)),
        ('arc_left_top', _arc(1.5, 1.5, 0.3, 0.0, math.pi / 2)),
        ('line_west', _line(1.5, 1.8, -1.5, 1.8)),
        ('arc_left_top_west', _arc(-1.5, 1.5, 0.3, math.pi / 2, math.pi)),
        ('line_south', _line(-1.8, 1.5, -1.8, 0.3)),
        ('arc_left_bottom_west', _arc(-1.5, 0.3, 0.3, math.pi, 3 * math.pi / 2)),
        ('line_finish', _line(-1.5, 0.0, 0.0, 0.0)),
    ]
    for lap_index in range(2):
        # One full lap plus a deterministic half-lap gives about 15.5 m,
        # enough curvature coverage without approaching the arena walls.
        selected = lap if lap_index == 0 else lap[:5]
        for label, points in selected:
            add(points, f'lap{lap_index + 1}_{label}')
    return segments


def _mirror_route(segments):
    """Reflect the validated route across the x axis, reversing turn sign."""
    return [(label.replace('left', 'right'),
             [(x, -y, -yaw) for x, y, yaw in points])
            for label, points in segments]


class FixedFollowPathDriver(Node):
    def __init__(self, output_root, variant, route_mode='original', frame='robot1/odom'):
        super().__init__('fixed_follow_path_driver')
        self.output_root = os.path.abspath(output_root)
        os.makedirs(self.output_root, exist_ok=True)
        self.variant = variant
        self.route_mode = route_mode
        self.frame = frame
        self.client = ActionClient(self, FollowPath, '/robot1/follow_path')
        self.odom = None
        self.segments = _local_route()
        if self.route_mode == 'mirrored':
            self.segments = _mirror_route(self.segments)
        self.index = 0
        self.initial = None
        self.goal_handle = None
        self.sent = False
        self.active = False
        self.state_request_pending = False
        self.rows = []
        self.done = False
        self.create_subscription(Odometry, '/robot1/odom', self._odom, 10)
        self.state_client = self.create_client(GetState, '/robot1/controller_server/get_state')
        self.timer = self.create_timer(0.2, self._tick)
        self._write_route()

    def _odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        if self.initial is None:
            self.initial = self.odom

    def _transform(self, p):
        x0, y0, th = self.initial
        c, s = math.cos(th), math.sin(th)
        x, y, yaw = p
        return (x0 + c * x - s * y, y0 + s * x + c * y, th + yaw)

    def _write_route(self):
        with open(os.path.join(self.output_root, 'route_definition.json'), 'w', encoding='utf-8') as f:
            json.dump({'variant': self.variant, 'route_mode': self.route_mode, 'frame': self.frame,
                       'segments': [{'label': label, 'points': points}
                                    for label, points in self.segments]}, f, indent=2)
        with open(os.path.join(self.output_root, 'route_status.csv'), 'w', newline='', encoding='utf-8') as f:
            csv.writer(f).writerow(['segment_index', 'label', 'event', 'sim_time_s', 'status'])

    def _status(self, label, event, status=''):
        with open(os.path.join(self.output_root, 'route_status.csv'), 'a', newline='', encoding='utf-8') as f:
            csv.writer(f).writerow([self.index, label, event,
                                    self.get_clock().now().nanoseconds * 1e-9, status])

    def _tick(self):
        if self.done or self.initial is None or self.index >= len(self.segments):
            if not self.done and self.initial is not None:
                self.done = True
                self._status('', 'route_complete', 'SUCCEEDED')
                rclpy.shutdown()
            return
        if not self.active:
            if not self.state_client.wait_for_service(timeout_sec=0.0):
                return
            if not self.state_request_pending:
                self.state_request_pending = True
                future = self.state_client.call_async(GetState.Request())
                future.add_done_callback(self._state_response)
            return
        label, points = self.segments[self.index]
        if self.goal_handle is not None:
            return
        if not self.client.wait_for_server(timeout_sec=0.0):
            return
        path = Path()
        path.header.frame_id = self.frame
        path.header.stamp = self.get_clock().now().to_msg()
        transformed = [self._transform(p) for p in points]
        path.poses = [_pose(x, y, yaw, self.frame) for x, y, yaw in transformed]
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'general_goal_checker'
        goal.progress_checker_id = 'progress_checker'
        self._status(label, 'goal_sent')
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)
        self.sent = True

    def _state_response(self, future):
        self.state_request_pending = False
        try:
            # lifecycle_msgs PRIMARY_STATE_ACTIVE is 3.
            self.active = int(future.result().current_state.id) == 3
        except Exception:
            self.active = False

    def _goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self._status(self.segments[self.index][0], 'goal_error', str(exc))
            self._abort()
            return
        if not handle.accepted:
            self._status(self.segments[self.index][0], 'goal_rejected', 'REJECTED')
            self._abort()
            return
        self.goal_handle = handle
        self._status(self.segments[self.index][0], 'goal_accepted', 'ACCEPTED')
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result)

    def _result(self, future):
        status = getattr(future.result(), 'status', '')
        self._status(self.segments[self.index][0], 'goal_result', str(status))
        if status != 4:  # action_msgs GoalStatus.STATUS_SUCCEEDED
            self._abort()
            return
        self.goal_handle = None
        self.sent = False
        self.index += 1

    def _abort(self):
        self.done = True
        self._status(self.segments[self.index][0] if self.index < len(self.segments) else '',
                     'route_failure', 'ABORTED')
        rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-root', required=True)
    parser.add_argument('--variant', required=True)
    parser.add_argument('--route-mode', choices=['original', 'mirrored'], default='original')
    args, _unknown = parser.parse_known_args(argv)
    rclpy.init(args=None)
    node = FixedFollowPathDriver(args.output_root, args.variant, args.route_mode)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
