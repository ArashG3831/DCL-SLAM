"""Bounded single-robot motion characterization collector."""

import csv
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState


class MotionCharacterizationNode(Node):
    """Drive repeatable phases and record command, odom, and wheel data."""

    def __init__(self):
        super().__init__('motion_characterization_node')
        self.declare_parameter('output_root', '/tmp/motion_characterization')
        self.declare_parameter('profile', 'speed_0p10')
        self.declare_parameter('target_speed', 0.10)
        self.declare_parameter('acceleration', 0.20)
        self.declare_parameter('wheel_radius', 0.02)
        self.declare_parameter('wheel_separation', 0.052)
        root = str(self.get_parameter('output_root').value)
        profile = str(self.get_parameter('profile').value)
        os.makedirs(root, exist_ok=True)
        self.path = os.path.join(root, f'{profile}.csv')
        self.target = abs(float(self.get_parameter('target_speed').value))
        self.accel = abs(float(self.get_parameter('acceleration').value))
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(
            self.get_parameter('wheel_separation').value)
        self.pub = self.create_publisher(Twist, '/robot1/cmd_vel_unstamped', 10)
        self.odom = None
        self.joints = None
        self.final_cmd = None
        self.t0_wall = time.monotonic()
        self.t0_ros = None
        self.last_ros = None
        self.done = False
        self.commanded = 0.0
        self.writer_file = open(self.path, 'w', newline='', encoding='utf-8')
        self.writer = csv.DictWriter(self.writer_file, fieldnames=[
            'wall_s', 'sim_elapsed_s', 'phase', 'cmd_vx', 'cmd_wz',
            'odom_x', 'odom_y', 'odom_yaw', 'odom_vx',
            'odom_wz', 'left_position', 'right_position', 'left_velocity',
            'right_velocity', 'wheel_predicted_vx', 'wheel_predicted_wz',
            'final_cmd_vx', 'final_cmd_wz', 'ros_time_s', 'scan_count'])
        self.writer.writeheader()
        self.scan_count = 0
        self.create_subscription(Odometry, '/robot1/odom', self._odom, 20)
        self.create_subscription(JointState, '/robot1/joint_states', self._joints, 20)
        self.create_subscription(TwistStamped, '/robot1/cmd_vel',
                                 self._final_cmd, 20)
        # A scan callback is intentionally topic-only; the payload is not stored.
        from sensor_msgs.msg import LaserScan
        self.create_subscription(LaserScan, '/robot1/scan_d500_fixed', self._scan, 10)
        self.timer = self.create_timer(0.02, self._tick)

    def _odom(self, msg): self.odom = msg
    def _joints(self, msg): self.joints = msg
    def _final_cmd(self, msg): self.final_cmd = msg
    def _scan(self, _): self.scan_count += 1

    def _phase(self, t):
        # 1 s settle, 10 s forward, 4 s stop, 3 s gentle arc, 4 s rotate,
        # 2 s stop.  The same geometry is used for every speed profile.
        if t < 1.0: return 'settle', 0.0, 0.0
        if t < 11.0: return 'straight', self.target, 0.0
        if t < 15.0: return 'brake', 0.0, 0.0
        if t < 18.0: return 'curve', min(self.target, 0.08), 0.20
        if t < 22.0: return 'rotate', 0.0, 0.30
        if t < 24.0: return 'finish', 0.0, 0.0
        return 'done', 0.0, 0.0

    def _tick(self):
        if self.done:
            return
        now_wall = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds / 1e9
        # Motion phases use simulation time, so fast mode does not alter the
        # commanded trajectory or acceleration profile.
        if self.t0_ros is None:
            if ros_now <= 0.0:
                self.pub.publish(Twist())
                return
            self.t0_ros = ros_now
            self.last_ros = ros_now
        t = ros_now - self.t0_ros
        phase, goal_v, goal_w = self._phase(t)
        dt = min(0.1, max(1e-4, ros_now - self.last_ros))
        self.last_ros = ros_now
        if goal_v > self.commanded:
            self.commanded = min(goal_v, self.commanded + self.accel * dt)
        else:
            self.commanded = max(goal_v, self.commanded - self.accel * dt)
        msg = Twist(); msg.linear.x = self.commanded; msg.angular.z = goal_w
        self.pub.publish(msg)
        odom = self.odom; joints = self.joints
        final_cmd = self.final_cmd
        left_velocity = '' if joints is None else self._joint(
            joints, 'left wheel motor', 1)
        right_velocity = '' if joints is None else self._joint(
            joints, 'right wheel motor', 1)
        wheel_vx = ''
        wheel_wz = ''
        if isinstance(left_velocity, (float, int)) and isinstance(
                right_velocity, (float, int)):
            wheel_vx = self.wheel_radius * (left_velocity + right_velocity) / 2.0
            wheel_wz = self.wheel_radius * (right_velocity - left_velocity) / self.wheel_separation
        row = {
            'wall_s': now_wall - self.t0_wall, 'sim_elapsed_s': t,
            'phase': phase, 'cmd_vx': self.commanded,
            'cmd_wz': goal_w,
            'odom_x': '' if odom is None else odom.pose.pose.position.x,
            'odom_y': '' if odom is None else odom.pose.pose.position.y,
            'odom_yaw': '' if odom is None else self._yaw(odom.pose.pose.orientation),
            'odom_vx': '' if odom is None else odom.twist.twist.linear.x,
            'odom_wz': '' if odom is None else odom.twist.twist.angular.z,
            'left_position': '' if joints is None else self._joint(joints, 'left wheel motor', 0),
            'right_position': '' if joints is None else self._joint(joints, 'right wheel motor', 0),
            'left_velocity': left_velocity,
            'right_velocity': right_velocity,
            'wheel_predicted_vx': wheel_vx,
            'wheel_predicted_wz': wheel_wz,
            'final_cmd_vx': '' if final_cmd is None else final_cmd.twist.linear.x,
            'final_cmd_wz': '' if final_cmd is None else final_cmd.twist.angular.z,
            'ros_time_s': ros_now,
            'scan_count': self.scan_count,
        }
        self.writer.writerow(row); self.writer_file.flush()
        if phase == 'done' and not self.done:
            self.done = True; self.pub.publish(Twist())
            self.timer.cancel()
            self.writer_file.flush(); self.writer_file.close()
            self.get_logger().info(f'MOTION_CHARACTERIZATION_COMPLETE path={self.path}')
            self.create_timer(0.2, self._shutdown_once)

    @staticmethod
    def _joint(msg, name, mode):
        try: return (msg.position if mode == 0 else msg.velocity)[msg.name.index(name)]
        except (ValueError, IndexError): return ''

    @staticmethod
    def _yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))

    def _shutdown_once(self):
        if rclpy.ok(): rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MotionCharacterizationNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if node.writer_file and not node.writer_file.closed: node.writer_file.close()
        node.destroy_node()


if __name__ == '__main__': main()
