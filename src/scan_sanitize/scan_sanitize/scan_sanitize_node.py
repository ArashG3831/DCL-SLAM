#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanSanitize(Node):
    def __init__(self):
        super().__init__('scan_sanitize')

        # allow launch/CLI control

        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, 10)
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', 10)

    def cb(self, msg: LaserScan):
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.intensities = list(msg.intensities)

        rr = []
        for r in msg.ranges:
            invalid = (
                r == 0.0 or
                math.isnan(r) or
                r < msg.range_min or
                r > msg.range_max
            )
            rr.append(float('inf') if invalid else r)
        out.ranges = rr
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(ScanSanitize())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
