"""Diagnostic-only odometry-to-TF bridge for the motion harness.

The production controller normally owns this transform. The isolated motion
stack publishes odometry but may not publish TF in every Webots driver build,
so this bridge exposes the same measured odometry as TF for diagnostic SLAM.
It never reads Supervisor ground truth.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class MotionOdomTfBridge(Node):
    def __init__(self):
        super().__init__('motion_odom_tf_bridge')
        self.declare_parameter('odom_topic', '/robot1/odom')
        self.declare_parameter('parent_frame', 'robot1/odom')
        self.declare_parameter('child_frame', 'base_footprint')
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self._odom, 20)

    def _odom(self, msg):
        transform = TransformStamped()
        transform.header = msg.header
        transform.header.frame_id = str(self.get_parameter('parent_frame').value)
        transform.child_frame_id = str(self.get_parameter('child_frame').value)
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = MotionOdomTfBridge()
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
