import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class D500ScanFix(Node):
    def __init__(self):
        super().__init__('d500_scan_fix')

        self.declare_parameter('input_topic', '/scan_d500')
        self.declare_parameter('output_topic', '/scan_d500_fixed')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.sub = self.create_subscription(
            LaserScan,
            input_topic,
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            LaserScan,
            output_topic,
            10
        )

        self.get_logger().info(
            f'D500 scan fixer started: {input_topic} -> {output_topic}'
        )

    def callback(self, msg: LaserScan):
        n = len(msg.ranges)
        if n < 2:
            return

        fixed = LaserScan()
        fixed.header = msg.header
        fixed.header.frame_id = msg.header.frame_id

        fixed.angle_min = -math.pi
        fixed.angle_max = math.pi
        fixed.angle_increment = (fixed.angle_max - fixed.angle_min) / (n - 1)

        fixed.time_increment = msg.time_increment
        fixed.scan_time = msg.scan_time
        fixed.range_min = msg.range_min
        fixed.range_max = msg.range_max

        fixed.ranges = list(reversed(msg.ranges))
        fixed.intensities = list(reversed(msg.intensities)) if msg.intensities else []

        self.pub.publish(fixed)


def main(args=None):
    rclpy.init(args=args)
    node = D500ScanFix()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
