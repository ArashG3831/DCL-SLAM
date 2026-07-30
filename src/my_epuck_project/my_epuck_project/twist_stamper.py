import signal

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist, TwistStamped


class TwistStamper(Node):
    def __init__(self, **node_kwargs):
        super().__init__('twist_stamper', **node_kwargs)

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

        self.get_logger().info('TwistStamper node started: /cmd_vel_unstamped -> /cmd_vel (TwistStamped)')

    def cmd_callback(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_link'  # or '' if you prefer
        stamped.twist = msg
        if rclpy.ok():
            self.pub.publish(stamped)


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
