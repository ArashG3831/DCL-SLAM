"""Diagnostic-only fixed-scan to SLAM-branch relay for a single robot."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class MotionScanBranchRelay(Node):
    def __init__(self):
        super().__init__('motion_scan_branch_relay')
        self.publisher = self.create_publisher(LaserScan, '/robot1/scan_d500_slam', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/robot1/scan_d500_fixed', self._scan, 10)

    def _scan(self, msg):
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotionScanBranchRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
