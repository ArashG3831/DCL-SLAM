"""Webots supervisor with bounded accelerated-simulation clock traffic.

Webots still advances one physics step on every supervisor callback.  In
``--mode=fast`` that can be thousands of callbacks per wall second, and the
stock ROS 2 supervisor publishes every one of those clock messages.  The
resulting reliable DDS traffic can starve sensor and lifecycle callbacks.

This wrapper keeps the stock supervisor step/service behavior and only
coalesces clock publication to a bounded simulation-time cadence.  It does
not expose Webots poses or alter any estimator input.
"""

import os
import time

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

from webots_ros2_driver.ros2_supervisor import Ros2Supervisor


class _ClockPublisherProxy:
    """Forward clock messages at most once per simulation-time interval."""

    def __init__(self, publisher, interval_seconds=0.1,
                 step_sleep_seconds=0.001):
        self._publisher = publisher
        self._interval_seconds = float(interval_seconds)
        self._step_sleep_seconds = max(0.0, float(step_sleep_seconds))
        self._last_seconds = None

    def publish(self, message):
        # A tiny wall-time yield keeps the external Webots ROS drivers
        # schedulable while the simulator remains in --mode=fast.  Without
        # it, accelerated controller steps can outrun the driver's DDS
        # publication path and lidar only arrives when Webots shuts down.
        if self._step_sleep_seconds:
            time.sleep(self._step_sleep_seconds)
        stamp = message.clock
        seconds = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if (self._last_seconds is None or
                seconds - self._last_seconds >= self._interval_seconds):
            self._last_seconds = seconds
            self._publisher.publish(message)


class PacedRos2Supervisor(Ros2Supervisor):
    """Stock supervisor with coalesced accelerated-mode ``/clock`` output."""

    def __init__(self):
        super().__init__()
        private_name = '_Ros2Supervisor__clock_publisher'
        # The stock Ros2Supervisor creates ``clock`` with the default reliable
        # QoS.  In Webots fast mode that writer can block the supervisor
        # callback when the post-handoff ROS graph grows, preventing the next
        # Webots physics step and freezing simulation time.  The clock is a
        # disposable telemetry stream; best-effort depth one keeps the
        # simulator stepping even when a subscriber is temporarily behind.
        stock_publisher = getattr(self, private_name)
        self.destroy_publisher(stock_publisher)
        clock_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        clock_publisher = self.create_publisher(Clock, 'clock', clock_qos)
        setattr(self, private_name, _ClockPublisherProxy(
            clock_publisher,
            step_sleep_seconds=os.environ.get(
                'MY_EPUCK_FAST_STEP_SLEEP_SECONDS', '0.001')))


def main(args=None):
    rclpy.init(args=args)
    supervisor = PacedRos2Supervisor()
    try:
        rclpy.spin(supervisor)
    finally:
        supervisor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
