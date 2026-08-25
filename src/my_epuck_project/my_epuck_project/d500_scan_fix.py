import math
import signal

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan


# The corrected stream stays reliable and retains a bounded history so the
# full scan can be acknowledged by Slam Toolbox and Nav2 over WSL loopback.
# Subscriber clock/QoS overrides, rather than an undersized reliable writer
# history, prevent stale samples from becoming a transport deadlock.
CORRECTED_SCAN_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
LATEST_SCAN_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
LATEST_NAV_SCAN_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
RAW_SCAN_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
RAW_SCAN_BEST_EFFORT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    # A raw sensor stream must not accumulate an unbounded backlog while the
    # bridge is being used for transport isolation diagnostics.
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def raw_scan_qos(input_reliability):
    """Return the explicitly requested QoS for the raw D500 input.

    The launch argument is intentionally handled here rather than relying on
    a graph-wide override.  That makes the transport A/B reproducible and
    prevents a best-effort diagnostic from silently remaining reliable.
    """
    value = str(input_reliability).lower()
    if value == 'reliable':
        return RAW_SCAN_QOS
    if value == 'best_effort':
        return RAW_SCAN_BEST_EFFORT_QOS
    raise ValueError(
        f"input_reliability must be 'reliable' or 'best_effort', got {value!r}")


class D500ScanFix(Node):
    def __init__(self, **node_kwargs):
        super().__init__('d500_scan_fix', **node_kwargs)

        self.declare_parameter('input_topic', '/scan_d500')
        self.declare_parameter('output_topic', '/scan_d500_fixed')
        self.declare_parameter('minimum_time_interval', 0.0)
        self.declare_parameter('input_reliability', 'reliable')
        self.declare_parameter('output_depth', 100)
        self.declare_parameter('output_reliability', 'reliable')
        self.declare_parameter('output_sample_count', 0)
        # One raw subscription can feed both corrected outputs.  This avoids
        # duplicated raw DDS readers while keeping Slam and Nav2 QoS separate.
        self.declare_parameter('secondary_output_topic', '')
        self.declare_parameter('secondary_output_depth', 1)
        self.declare_parameter('secondary_output_reliability', 'best_effort')
        self.declare_parameter('secondary_output_sample_count', 0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.minimum_time_interval = max(
            0.0, float(self.get_parameter('minimum_time_interval').value))
        output_depth = int(self.get_parameter('output_depth').value)
        output_reliability = str(
            self.get_parameter('output_reliability').value).lower()
        self.output_sample_count = int(
            self.get_parameter('output_sample_count').value)
        self.secondary_output_topic = str(
            self.get_parameter('secondary_output_topic').value).strip()
        self.secondary_output_sample_count = int(
            self.get_parameter('secondary_output_sample_count').value)
        if output_depth <= 1 and output_reliability == 'best_effort':
            output_qos = LATEST_NAV_SCAN_QOS
        elif output_depth <= 1:
            output_qos = LATEST_SCAN_QOS
        else:
            output_qos = CORRECTED_SCAN_QOS
        input_reliability = str(
            self.get_parameter('input_reliability').value).lower()
        input_qos = raw_scan_qos(input_reliability)
        self._last_published_stamp = None
        self._received_count = 0
        self._published_count = 0

        self.sub = self.create_subscription(
            LaserScan,
            input_topic,
            self.callback,
            input_qos,
        )

        self.pub = self.create_publisher(
            LaserScan,
            output_topic,
            output_qos,
        )
        self.secondary_pub = None
        if self.secondary_output_topic:
            secondary_depth = int(
                self.get_parameter('secondary_output_depth').value)
            secondary_reliability = str(
                self.get_parameter('secondary_output_reliability').value).lower()
            if secondary_depth <= 1 and secondary_reliability == 'best_effort':
                secondary_qos = LATEST_NAV_SCAN_QOS
            elif secondary_depth <= 1:
                secondary_qos = LATEST_SCAN_QOS
            else:
                secondary_qos = CORRECTED_SCAN_QOS
            self.secondary_pub = self.create_publisher(
                LaserScan, self.secondary_output_topic, secondary_qos)

        self.get_logger().info(
            f'D500 scan fixer started: {input_topic} -> {output_topic}'
        )

    def callback(self, msg: LaserScan):
        self._received_count += 1
        source_count = len(msg.ranges)
        if source_count < 2:
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

        target_count = source_count
        if 1 < self.output_sample_count < source_count:
            target_count = self.output_sample_count
        fixed.angle_min = -math.pi
        fixed.angle_max = math.pi
        fixed.angle_increment = (fixed.angle_max - fixed.angle_min) / (target_count - 1)

        fixed.time_increment = msg.time_increment
        fixed.scan_time = msg.scan_time
        fixed.range_min = msg.range_min
        fixed.range_max = msg.range_max

        ranges = list(reversed(msg.ranges))
        intensities = list(reversed(msg.intensities)) if msg.intensities else []
        if target_count != source_count:
            # Uniformly retain the corrected angular support.  The Nav2
            # safety stream may use fewer beams; Slam's full-resolution
            # stream never requests this path.
            indices = [round(i * (source_count - 1) / (target_count - 1))
                       for i in range(target_count)]
            ranges = [ranges[i] for i in indices]
            if intensities:
                intensities = [intensities[i] for i in indices]
        fixed.ranges = ranges
        fixed.intensities = intensities

        if rclpy.ok():
            self.pub.publish(fixed)
            if self.secondary_pub is not None:
                fixed_count = len(fixed.ranges)
                secondary_count = fixed_count
                if (1 < self.secondary_output_sample_count < fixed_count):
                    secondary_count = self.secondary_output_sample_count
                secondary = fixed
                if secondary_count != fixed_count:
                    secondary = LaserScan()
                    secondary.header = fixed.header
                    secondary.angle_min = fixed.angle_min
                    secondary.angle_max = fixed.angle_max
                    secondary.angle_increment = (
                        (fixed.angle_max - fixed.angle_min) /
                        (secondary_count - 1))
                    secondary.time_increment = fixed.time_increment
                    secondary.scan_time = fixed.scan_time
                    secondary.range_min = fixed.range_min
                    secondary.range_max = fixed.range_max
                    indices = [
                        round(i * (fixed_count - 1) /
                              (secondary_count - 1))
                        for i in range(secondary_count)]
                    secondary.ranges = [fixed.ranges[i] for i in indices]
                    if fixed.intensities:
                        secondary.intensities = [
                            fixed.intensities[i] for i in indices]
                self.secondary_pub.publish(secondary)
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
