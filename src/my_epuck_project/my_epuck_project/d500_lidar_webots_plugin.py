"""Bounded Webots D500 lidar publisher for accelerated WSL runs.

The stock webots_ros2_driver LaserScan publisher uses a reliable writer.  In
accelerated Windows-Webots/WSL runs that writer can block after one large
720-beam sample.  This plugin reads the same Webots lidar device and publishes
the unchanged scan with sensor-data (best-effort) QoS at a bounded rate.
"""

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class D500LidarWebotsPlugin:
    def __init__(self):
        self.robot = None
        self.node = None
        self.publisher = None
        self.lidar = None
        self.period = 0.1
        self.last_publish = -1.0
        self.publish_count = 0
        self.ready = False

    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node('d500_lidar_webots_plugin')
        self.period = 1.0 / max(0.1, float(properties.get('updateRate', '10.0')))
        robot_name = self.robot.getName()
        self.publisher = self.node.create_publisher(
            LaserScan, f'/{robot_name}/scan_d500', qos_profile_sensor_data)
        self.lidar = self.robot.getDevice('d500_lidar')
        if self.lidar is None:
            self.node.get_logger().error("Webots device 'd500_lidar' was not found.")
            return
        self.lidar.enable(max(int(self.robot.getBasicTimeStep()),
                              int(round(self.period * 1000.0))))
        self.ready = True
        self.node.get_logger().info(
            f'D500 Webots lidar plugin started: /{robot_name}/scan_d500 '
            f'at {1.0 / self.period:.1f} Hz')

    def step(self):
        if not self.ready:
            return
        rclpy.spin_once(self.node, timeout_sec=0.0)
        now = float(self.robot.getTime())
        if self.last_publish >= 0.0 and now - self.last_publish < self.period:
            return
        ranges = self.lidar.getLayerRangeImage(0)
        if not ranges:
            return
        resolution = self.lidar.getHorizontalResolution()
        message = LaserScan()
        sec = int(now)
        message.header.stamp.sec = sec
        message.header.stamp.nanosec = int((now - sec) * 1_000_000_000)
        message.header.frame_id = f'{self.robot.getName()}/d500_lidar'
        message.angle_min = self.lidar.getFov() / 2.0
        message.angle_max = -self.lidar.getFov() / 2.0
        message.angle_increment = -self.lidar.getFov() / (resolution - 1)
        message.time_increment = self.period / resolution
        message.scan_time = self.period
        message.range_min = self.lidar.getMinRange()
        message.range_max = self.lidar.getMaxRange()
        message.ranges = list(ranges)
        self.publisher.publish(message)
        # Flush the embedded rclpy publisher before the next accelerated
        # Webots step; this is especially important for large LaserScan
        # samples when the driver also hosts the C++ ROS executor.
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.last_publish = now
        self.publish_count += 1
        if self.publish_count == 1 or self.publish_count % 100 == 0:
            self.node.get_logger().info(
                f'D500 Webots lidar counts published={self.publish_count} '
                f'stamp={now:.3f}')
