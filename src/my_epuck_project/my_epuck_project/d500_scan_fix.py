import math
import signal

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan


class D500ScanFix(Node):
    def __init__(self, **node_kwargs):
        super().__init__('d500_scan_fix', **node_kwargs)

        self.declare_parameter('input_topic', '/scan_d500')
        self.declare_parameter('output_topic', '/scan_d500_fixed')
        self.declare_parameter('minimum_time_interval', 0.0)
        self.declare_parameter('input_reliability', 'reliable')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.minimum_time_interval = max(
            0.0, float(self.get_parameter('minimum_time_interval').value))
        input_reliability = str(
            self.get_parameter('input_reliability').value).lower()
        self._last_published_stamp = None
        self._received_count = 0
        self._published_count = 0

        # Keep a bounded but deeper queue than the upstream driver.  In fast
        # WSL runs the reliable writer can publish several simulation scans
        # between executor wakeups; depth 10 otherwise leaves the bridge
        # permanently draining an obsolete queue.
        input_qos = 100
        self.sub = self.create_subscription(
            LaserScan,
            input_topic,
            self.callback,
            # The Webots ROS driver advertises the raw D500 stream with the
            # project's default reliable profile.  Keep this subscription
            # reliable; using sensor-data best-effort here silently leaves
            # the bridge without samples under CycloneDDS.
            input_qos,
        )

        self.pub = self.create_publisher(
            LaserScan,
            output_topic,
            100,
        )

        self.get_logger().info(
            f'D500 scan fixer started: {input_topic} -> {output_topic}'
        )

    def callback(self, msg: LaserScan):
        self._received_count += 1
        n = len(msg.ranges)
        if n < 2:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if (self.minimum_time_interval > 0.0 and
                self._last_published_stamp is not None and
                stamp - self._last_published_stamp <
                self.minimum_time_interval):
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
            self._last_published_stamp = stamp
            self._published_count += 1
            if self._published_count == 1 or self._published_count % 100 == 0:
                self.get_logger().info(
                    f'D500 scan fixer counts received={self._received_count} '
                    f'published={self._published_count} stamp={stamp:.3f}')


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
