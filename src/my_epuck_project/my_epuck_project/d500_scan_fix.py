import math
import signal

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan


class D500ScanFix(Node):
    def __init__(self, **node_kwargs):
        super().__init__('d500_scan_fix', **node_kwargs)

        self.declare_parameter('input_topic', '/scan_d500')
        self.declare_parameter('output_topic', '/scan_d500_fixed')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.sub = self.create_subscription(
            LaserScan,
            input_topic,
            self.callback,
            qos_profile_sensor_data,
        )

        self.pub = self.create_publisher(
            LaserScan,
            output_topic,
            # The Webots input is sensor-data (best effort), but the fixed
            # scan is consumed by the teammate filter and SLAM with the
            # project's reliable subscription profile.  Publishing the
            # corrected stream as best effort makes CycloneDDS reject those
            # subscribers due to incompatible reliability.
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
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

        if rclpy.ok():
            self.pub.publish(fixed)


def shutdown_node(node, executor=None):
    """Stop ROS entities in dependency order and remain idempotent."""
    if executor is not None and node is not None:
        executor.remove_node(node)
        executor.shutdown()
    if node is not None and node.context.ok():
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    executor = None
    stopping = {'value': False}
    previous_handlers = {}

    def request_shutdown(signum, frame):
        del signum, frame
        stopping['value'] = True
        if executor is not None:
            executor.wake()

    try:
        node = D500ScanFix()
        executor = SingleThreadedExecutor(context=node.context)
        executor.add_node(node)
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, request_shutdown)
        while not stopping['value'] and rclpy.ok():
            executor.spin_once(timeout_sec=0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        shutdown_node(node, executor)


if __name__ == '__main__':
    main()
