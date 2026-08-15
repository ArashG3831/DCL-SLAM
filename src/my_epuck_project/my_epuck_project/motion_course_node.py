"""Distance-locked single-robot motion-course driver and telemetry collector."""

from __future__ import annotations

import csv
import json
import math
import os
import tempfile

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan

from .motion_course_spec import course_for_name


def _atomic_json(path, payload):
    directory = os.path.dirname(os.path.abspath(path))
    fd, temporary = tempfile.mkstemp(prefix='.course_command_', dir=directory)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            json.dump(payload, stream, sort_keys=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _read_json(path):
    try:
        with open(path, encoding='utf-8') as stream:
            return json.load(stream)
    except (FileNotFoundError, OSError, ValueError, TypeError):
        return None


class MotionCourseNode(Node):
    """Drive the same geometric course at several requested speeds."""

    def __init__(self):
        super().__init__('motion_course_node')
        self.declare_parameter('output_root', '/tmp/motion_characterization')
        self.declare_parameter('profile', 'course_speed_0p10')
        self.declare_parameter('target_speed', 0.10)
        self.declare_parameter('acceleration', 0.20)
        self.declare_parameter('angular_speed', 0.30)
        self.declare_parameter('angular_acceleration', 1.20)
        self.declare_parameter('wheel_radius', 0.02)
        self.declare_parameter('wheel_separation', 0.052)
        self.declare_parameter('gate_file', '')
        self.declare_parameter('command_file', '')
        self.declare_parameter('course_name', 'short')
        root = os.path.abspath(str(self.get_parameter('output_root').value))
        os.makedirs(root, exist_ok=True)
        profile = str(self.get_parameter('profile').value)
        self.output_path = os.path.join(root, f'{profile}.csv')
        self.scan_path = os.path.join(root, 'scan_timestamps.csv')
        self.gate_file = str(self.get_parameter('gate_file').value) or os.path.join(root, 'course_gate.json')
        self.command_file = str(self.get_parameter('command_file').value) or os.path.join(root, 'course_command.json')
        self.target_speed = abs(float(self.get_parameter('target_speed').value))
        self.accel = abs(float(self.get_parameter('acceleration').value))
        self.angular_speed = abs(float(self.get_parameter('angular_speed').value))
        self.angular_accel = abs(float(self.get_parameter('angular_acceleration').value))
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.course = course_for_name(self.get_parameter('course_name').value)

        # The course is a vehicle-physics diagnostic.  Publish stamped
        # commands directly to the simulation controller so a delayed
        # diagnostic stamper cannot change segment boundaries.  The later
        # production/Nav2 experiment uses the normal smoother/stamper chain.
        self.pub = self.create_publisher(TwistStamped, '/robot1/cmd_vel', 10)
        self.odom = None
        self.joints = None
        self.final_cmd = None
        self.clock_s = None
        self.last_clock_s = None
        self.t0_s = None
        self.last_s = None
        self.current_segment = 0
        self.completed_segment = -1
        self.linear_command = 0.0
        self.angular_command = 0.0
        self.done = False
        self.scan_counts = {'raw': 0, 'fixed': 0, 'slam': 0}

        self.motion_file = open(self.output_path, 'w', newline='', encoding='utf-8')
        self.motion_writer = csv.DictWriter(self.motion_file, fieldnames=[
            'wall_s', 'sim_elapsed_s', 'segment_index', 'segment_label', 'segment_kind',
            'cmd_vx', 'cmd_wz', 'final_cmd_vx', 'final_cmd_wz',
            'odom_x', 'odom_y', 'odom_yaw', 'odom_vx', 'odom_wz',
            'left_position', 'right_position', 'left_velocity', 'right_velocity',
            'wheel_predicted_vx', 'wheel_predicted_wz', 'ros_time_s',
            'raw_scan_count', 'fixed_scan_count', 'slam_scan_count',
            'gate_completed_segment'])
        self.motion_writer.writeheader()
        self.motion_file.flush()
        self.scan_file = open(self.scan_path, 'w', newline='', encoding='utf-8')
        self.scan_writer = csv.DictWriter(self.scan_file, fieldnames=[
            'topic', 'sim_time_s', 'header_stamp_s', 'count'])
        self.scan_writer.writeheader()
        self.scan_file.flush()

        self.create_subscription(Clock, '/clock', self._clock_cb, 10)
        self.create_subscription(Odometry, '/robot1/odom', self._odom, 20)
        self.create_subscription(JointState, '/robot1/joint_states', self._joints, 20)
        self.create_subscription(TwistStamped, '/robot1/cmd_vel', self._final_cmd, 20)
        self.create_subscription(LaserScan, '/robot1/scan_d500', lambda msg: self._scan('raw', msg), 10)
        self.create_subscription(LaserScan, '/robot1/scan_d500_fixed', lambda msg: self._scan('fixed', msg), 10)
        self.create_subscription(LaserScan, '/robot1/scan_d500_slam', lambda msg: self._scan('slam', msg), 10)
        self.timer = self.create_timer(0.02, self._tick)
        self.t0_wall = self.get_clock().now().nanoseconds / 1.0e9

    def _clock_cb(self, msg):
        self.clock_s = msg.clock.sec + msg.clock.nanosec / 1.0e9

    def _odom(self, msg):
        self.odom = msg

    def _joints(self, msg):
        self.joints = msg

    def _final_cmd(self, msg):
        self.final_cmd = msg

    def _scan(self, topic, msg):
        if self.scan_file.closed:
            return
        self.scan_counts[topic] += 1
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1.0e9
        sim = self.clock_s if self.clock_s is not None else stamp
        self.scan_writer.writerow({
            'topic': topic, 'sim_time_s': sim, 'header_stamp_s': stamp,
            'count': self.scan_counts[topic]})
        self.scan_file.flush()

    def _write_command(self, done=False):
        _atomic_json(self.command_file, {
            'segment_index': self.current_segment,
            'done': bool(done),
            'sim_time_s': self.clock_s,
        })

    def _publish(self, twist):
        stamped = TwistStamped()
        stamped.header.stamp.sec = int(self.clock_s or 0.0)
        stamped.header.stamp.nanosec = int(max(0.0, (self.clock_s or 0.0) % 1.0) * 1.0e9)
        stamped.header.frame_id = 'base_link'
        stamped.twist = twist
        self.pub.publish(stamped)

    def _read_gate(self):
        gate = _read_json(self.gate_file) or {}
        try:
            self.completed_segment = max(self.completed_segment, int(gate.get('completed_segment', -1)))
        except (TypeError, ValueError):
            pass

    def _ramp(self, current, goal, limit, dt):
        if goal > current:
            return min(goal, current + limit * dt)
        return max(goal, current - limit * dt)

    def _joint(self, msg, name, mode):
        try:
            values = msg.position if mode == 0 else msg.velocity
            return values[msg.name.index(name)]
        except (ValueError, IndexError):
            return ''

    @staticmethod
    def _yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _tick(self):
        if self.done or self.clock_s is None or self.clock_s <= 0.0:
            self._publish(Twist())
            return
        if self.t0_s is None:
            self.t0_s = self.clock_s
            self.last_s = self.clock_s
            self._write_command()
        dt = min(0.1, max(1.0e-4, self.clock_s - self.last_s))
        self.last_s = self.clock_s
        self._read_gate()
        if self.completed_segment >= self.current_segment:
            self.current_segment += 1
            self.linear_command = 0.0
            self.angular_command = 0.0
            if self.current_segment >= len(self.course):
                self.done = True
                self._write_command(done=True)
                self._publish(Twist())
                self._close()
                return
            self._write_command()

        segment = self.course[self.current_segment]
        goal_v = self.target_speed if segment['kind'] == 'straight' else 0.0
        goal_w = (segment['direction'] * self.angular_speed) if segment['kind'] == 'turn' else 0.0
        self.linear_command = self._ramp(self.linear_command, goal_v, self.accel, dt)
        self.angular_command = self._ramp(self.angular_command, goal_w, self.angular_accel, dt)
        command = Twist()
        command.linear.x = self.linear_command
        command.angular.z = self.angular_command
        self._publish(command)

        odom, joints, final_cmd = self.odom, self.joints, self.final_cmd
        left_v = '' if joints is None else self._joint(joints, 'left wheel motor', 1)
        right_v = '' if joints is None else self._joint(joints, 'right wheel motor', 1)
        wheel_vx = ''
        wheel_wz = ''
        if isinstance(left_v, (int, float)) and isinstance(right_v, (int, float)):
            wheel_vx = self.wheel_radius * (left_v + right_v) / 2.0
            wheel_wz = self.wheel_radius * (right_v - left_v) / self.wheel_separation
        self.motion_writer.writerow({
            'wall_s': self.get_clock().now().nanoseconds / 1.0e9 - self.t0_wall,
            'sim_elapsed_s': self.clock_s - self.t0_s,
            'segment_index': self.current_segment,
            'segment_label': segment['label'], 'segment_kind': segment['kind'],
            'cmd_vx': self.linear_command, 'cmd_wz': self.angular_command,
            'final_cmd_vx': '' if final_cmd is None else final_cmd.twist.linear.x,
            'final_cmd_wz': '' if final_cmd is None else final_cmd.twist.angular.z,
            'odom_x': '' if odom is None else odom.pose.pose.position.x,
            'odom_y': '' if odom is None else odom.pose.pose.position.y,
            'odom_yaw': '' if odom is None else self._yaw(odom.pose.pose.orientation),
            'odom_vx': '' if odom is None else odom.twist.twist.linear.x,
            'odom_wz': '' if odom is None else odom.twist.twist.angular.z,
            'left_position': '' if joints is None else self._joint(joints, 'left wheel motor', 0),
            'right_position': '' if joints is None else self._joint(joints, 'right wheel motor', 0),
            'left_velocity': left_v, 'right_velocity': right_v,
            'wheel_predicted_vx': wheel_vx, 'wheel_predicted_wz': wheel_wz,
            'ros_time_s': self.clock_s,
            'raw_scan_count': self.scan_counts['raw'],
            'fixed_scan_count': self.scan_counts['fixed'],
            'slam_scan_count': self.scan_counts['slam'],
            'gate_completed_segment': self.completed_segment,
        })
        self.motion_file.flush()

    def _close(self):
        if not self.motion_file.closed:
            self.motion_file.flush()
            self.motion_file.close()
        if not self.scan_file.closed:
            self.scan_file.flush()
            self.scan_file.close()
        self.timer.cancel()
        self.get_logger().info(f'MOTION_COURSE_COMPLETE path={self.output_path}')
        self.create_timer(0.2, self._shutdown)

    def _shutdown(self):
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MotionCourseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.motion_file.closed:
            node.motion_file.close()
        if not node.scan_file.closed:
            node.scan_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
