import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry


class TwistStamper(Node):
    def __init__(self, **node_kwargs):
        super().__init__('twist_stamper', **node_kwargs)

        self.telemetry_timeout_s = max(0.1, float(self.declare_parameter(
            'telemetry_timeout_s', 2.0).value))
        self.override_cmd_topic = str(self.declare_parameter(
            'override_cmd_topic', '').value).strip()
        self.watchdog_period_s = max(0.02, float(self.declare_parameter(
            'watchdog_period_s', 0.1).value))
        self._last_odom_wall_s = None
        self._last_command_wall_s = None
        self._last_override_wall_s = None
        self._override_active = False
        self._command_active = False
        self._watchdog_stopped = False

        # Subscribe to un-stamped velocity commands
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel_unstamped',
            self.cmd_callback,
            10
        )

        # Publish stamped velocity commands for the controller
        self.pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.override_sub = None
        if self.override_cmd_topic:
            self.override_sub = self.create_subscription(
                Twist, self.override_cmd_topic,
                self.override_callback, 10)
        self.watchdog = self.create_timer(
            self.watchdog_period_s, self.watchdog_callback)

        self.get_logger().info('TwistStamper node started: /cmd_vel_unstamped -> /cmd_vel (TwistStamped)')

    def cmd_callback(self, msg: Twist):
        if self._override_active:
            return
        self._last_command_wall_s = time.monotonic()
        self._command_active = (
            abs(float(msg.linear.x)) > 1.0e-3 or
            abs(float(msg.angular.z)) > 1.0e-3)
        self._watchdog_stopped = False
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_link'  # or '' if you prefer
        stamped.twist = msg
        if rclpy.ok():
            self.pub.publish(stamped)

    def override_callback(self, msg: Twist):
        """Publish a bounded validation command without racing Nav2 output."""
        now = time.monotonic()
        active = (
            abs(float(msg.linear.x)) > 1.0e-3 or
            abs(float(msg.angular.z)) > 1.0e-3)
        if active and not self._override_active:
            self.get_logger().info(
                'EVIDENCE_CMD_OVERRIDE_ACTIVE topic=%s' %
                self.override_cmd_topic)
        elif not active and self._override_active:
            self.get_logger().info('EVIDENCE_CMD_OVERRIDE_CLEARED')
        self._last_override_wall_s = now
        self._override_active = active
        self._last_command_wall_s = now
        self._command_active = active
        self._watchdog_stopped = False
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_link'
        stamped.twist = msg
        if rclpy.ok():
            self.pub.publish(stamped)

    def odom_callback(self, _msg: Odometry):
        """Record transport liveness using wall time, not simulation time."""
        self._last_odom_wall_s = time.monotonic()

    def watchdog_callback(self):
        """Clear a nonzero command when odometry stops arriving."""
        now = time.monotonic()
        if (self._override_active and
                self._last_override_wall_s is not None and
                now - self._last_override_wall_s >
                max(0.25, 3.0 * self.watchdog_period_s)):
            zero = TwistStamped()
            zero.header.stamp = self.get_clock().now().to_msg()
            zero.header.frame_id = 'base_link'
            if rclpy.ok():
                self.pub.publish(zero)
            self._override_active = False
            self._command_active = False
            self._watchdog_stopped = True
            self.get_logger().error(
                'SAFETY_STOP_EVIDENCE_OVERRIDE_TIMEOUT timeout_s=%.3f' %
                max(0.25, 3.0 * self.watchdog_period_s))
            return
        if not self._command_active or self._last_command_wall_s is None:
            return
        odom_age = (float('inf') if self._last_odom_wall_s is None else
                    now - self._last_odom_wall_s)
        if (odom_age <= self.telemetry_timeout_s or
                self._watchdog_stopped):
            return
        zero = TwistStamped()
        zero.header.stamp = self.get_clock().now().to_msg()
        zero.header.frame_id = 'base_link'
        if rclpy.ok():
            self.pub.publish(zero)
        self._command_active = False
        self._watchdog_stopped = True
        self.get_logger().error(
            'SAFETY_STOP_STALE_ODOM odom_age_s=%.3f timeout_s=%.3f' %
            (odom_age, self.telemetry_timeout_s))


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
        node = TwistStamper()
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
