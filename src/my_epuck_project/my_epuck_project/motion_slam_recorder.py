"""Bounded, read-only SLAM telemetry for motion characterization.

This node is diagnostic-only.  It records the SLAM trajectory relative to the
Webots Supervisor trajectory and compact map-growth statistics.  It never
publishes TF, commands, or navigation data.
"""

import csv
import math
import os

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener


class MotionSlamRecorder(Node):
    def __init__(self):
        super().__init__('motion_slam_recorder')
        self.declare_parameter('output_root', '/tmp/motion_characterization')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.declare_parameter('map_topic', '/robot1/map')
        self.declare_parameter('map_frame', 'robot1/map')
        self.declare_parameter('odom_frame', 'robot1/odom')
        self.declare_parameter('base_frame', 'robot1/base_footprint')
        root = str(self.get_parameter('output_root').value)
        os.makedirs(root, exist_ok=True)
        self.pose_path = os.path.join(root, 'slam_trajectory.csv')
        self.map_path = os.path.join(root, 'slam_map_updates.csv')
        self.map_odom_path = os.path.join(root, 'map_odom.csv')
        self.final_map_path = os.path.join(root, 'slam_final_map.pgm')
        self.pose_file = open(self.pose_path, 'w', newline='', encoding='utf-8')
        self.map_file = open(self.map_path, 'w', newline='', encoding='utf-8')
        self.map_odom_file = open(self.map_odom_path, 'w', newline='', encoding='utf-8')
        self.pose_writer = csv.DictWriter(self.pose_file, fieldnames=[
            'sim_time_s', 'x_m', 'y_m', 'yaw_rad', 'lookup_ok'])
        self.map_writer = csv.DictWriter(self.map_file, fieldnames=[
            'sim_time_s', 'width', 'height', 'resolution_m', 'origin_x_m',
            'origin_y_m', 'known_cells', 'free_cells', 'occupied_cells',
            'unknown_cells'])
        self.map_odom_writer = csv.DictWriter(self.map_odom_file, fieldnames=[
            'sim_time_s', 'x_m', 'y_m', 'yaw_rad', 'lookup_ok'])
        self.pose_writer.writeheader()
        self.map_writer.writeheader()
        self.map_odom_writer.writeheader()
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        # Do not infer the time domain from Node.get_clock(): a recorder that
        # starts before /clock can otherwise silently write a wall-clock epoch
        # while Supervisor and odometry use simulation seconds.  The explicit
        # /clock stream is the authority for this diagnostic.
        self.clock_s = None
        self.create_subscription(Clock, '/clock', self._clock_cb, 10)
        self.latest_map = None
        self.last_map_stamp = None
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter('map_topic').value),
            self._map, 10)
        self.create_timer(0.05, self._sample_pose)

    def _clock_cb(self, msg):
        self.clock_s = msg.clock.sec + msg.clock.nanosec / 1.0e9

    def _sim_time(self):
        return self.clock_s

    def _sample_pose(self):
        now = self._sim_time()
        if now is None or now <= 0.0:
            self.pose_writer.writerow({
                'sim_time_s': '', 'x_m': '', 'y_m': '', 'yaw_rad': '',
                'lookup_ok': 0})
            self.pose_file.flush()
            return
        self._sample_map_odom(now)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
            t = transform.transform.translation
            q = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.pose_writer.writerow({
                'sim_time_s': now, 'x_m': t.x, 'y_m': t.y,
                'yaw_rad': yaw, 'lookup_ok': 1})
        except Exception:
            self.pose_writer.writerow({
                'sim_time_s': now, 'x_m': '', 'y_m': '', 'yaw_rad': '',
                'lookup_ok': 0})
        self.pose_file.flush()

    def _sample_map_odom(self, now):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.odom_frame, rclpy.time.Time())
            t = transform.transform.translation
            q = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.map_odom_writer.writerow({
                'sim_time_s': now, 'x_m': t.x, 'y_m': t.y,
                'yaw_rad': yaw, 'lookup_ok': 1})
        except Exception:
            self.map_odom_writer.writerow({
                'sim_time_s': now, 'x_m': '', 'y_m': '', 'yaw_rad': '',
                'lookup_ok': 0})
        self.map_odom_file.flush()

    def _map(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1.0e9
        if self.last_map_stamp is not None and stamp <= self.last_map_stamp:
            return
        self.last_map_stamp = stamp
        data = list(msg.data)
        known = sum(value >= 0 for value in data)
        free = sum(value == 0 for value in data)
        occupied = sum(value >= 65 for value in data)
        unknown = len(data) - known
        self.map_writer.writerow({
            'sim_time_s': self._sim_time() if self._sim_time() is not None else stamp,
            'width': msg.info.width,
            'height': msg.info.height, 'resolution_m': msg.info.resolution,
            'origin_x_m': msg.info.origin.position.x,
            'origin_y_m': msg.info.origin.position.y,
            'known_cells': known, 'free_cells': free,
            'occupied_cells': occupied, 'unknown_cells': unknown})
        self.map_file.flush()
        self.latest_map = msg

    def close(self):
        if self.latest_map is not None:
            self._write_pgm(self.latest_map, self.final_map_path)
        if not self.pose_file.closed:
            self.pose_file.close()
        if not self.map_file.closed:
            self.map_file.close()
        if not self.map_odom_file.closed:
            self.map_odom_file.close()

    @staticmethod
    def _write_pgm(msg, path):
        # PGM uses white for free, black for occupied, and mid-gray for unknown.
        with open(path, 'wb') as stream:
            stream.write(f'P5\n{msg.info.width} {msg.info.height}\n255\n'.encode())
            for y in range(msg.info.height - 1, -1, -1):
                start = y * msg.info.width
                for value in msg.data[start:start + msg.info.width]:
                    if value < 0:
                        pixel = 205
                    elif value >= 65:
                        pixel = 0
                    else:
                        pixel = 255
                    stream.write(bytes((pixel,)))


def main(args=None):
    rclpy.init(args=args)
    node = MotionSlamRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
