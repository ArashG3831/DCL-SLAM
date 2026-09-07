"""Test-only simulated-time dispatch barrier for synchronized traffic trials.

This node is never part of the normal production graph.  It only waits for
both shared allocators to advertise an identical agreed round, then publishes
one release on the next received simulated-clock sample.  It does not choose
tasks, bids, paths, or traffic priority.
"""

from __future__ import annotations

import json

import rclpy
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import String


READY_TOPIC = '/cslam/traffic_test/dispatch_ready'
RELEASE_TOPIC = '/cslam/traffic_test/dispatch_release'


def ready_key(message: String) -> tuple[str, str] | None:
    """Return the agreed round identity carried by an allocator readiness message."""
    try:
        payload = json.loads(str(message.data))
    except (TypeError, ValueError, json.JSONDecodeError):
        return None
    round_id = str(payload.get('round_id', ''))
    decision_hash = str(payload.get('decision_hash', ''))
    if not round_id or not decision_hash:
        return None
    return round_id, decision_hash


class TrafficTestBarrier(Node):
    """Release both test dispatchers on one common simulated-clock boundary."""

    def __init__(self) -> None:
        super().__init__('traffic_test_dispatch_barrier')
        self._ready: dict[str, tuple[str, str]] = {}
        self._clock_seen = False
        self._sim_time_s = 0.0
        self._released_keys: set[tuple[str, str]] = set()
        self._pending_release_key: tuple[str, str] | None = None
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._release_publisher = self.create_publisher(String, RELEASE_TOPIC, qos)
        self.create_subscription(
            String, f'{READY_TOPIC}/robot1',
            lambda message: self._ready_callback('robot1', message), qos,
        )
        self.create_subscription(
            String, f'{READY_TOPIC}/robot2',
            lambda message: self._ready_callback('robot2', message), qos,
        )
        # /clock is published with the ROS simulated-time sensor-data QoS;
        # using the transient-local readiness QoS prevents clock matching.
        self.create_subscription(
            Clock, '/clock', self._clock_callback, qos_profile_sensor_data,
        )

    def _ready_callback(self, robot_id: str, message: String) -> None:
        key = ready_key(message)
        if key is None:
            return
        self._ready[robot_id] = key
        self._maybe_release()

    def _clock_callback(self, message: Clock) -> None:
        self._sim_time_s = (
            float(message.clock.sec) + float(message.clock.nanosec) / 1e9
        )
        self._clock_seen = True
        self._maybe_release()

    def _maybe_release(self) -> None:
        if not self._clock_seen:
            return
        # /clock normally arrives before either allocator is ready.  Do not
        # index the readiness map until both robot keys exist; the barrier is
        # deliberately a passive test aid and must never crash the trial.
        if 'robot1' not in self._ready or 'robot2' not in self._ready:
            return
        if self._ready['robot1'] != self._ready['robot2']:
            return
        key = self._ready['robot1']
        if key in self._released_keys:
            return
        # Publish a future simulated-time boundary.  Both allocators receive
        # the same boundary and independently wait for /clock to reach it;
        # callback delivery order therefore cannot create a robot-dependent
        # release time.  The barrier does not choose a task or priority.
        release_at_s = self._sim_time_s + 0.20
        release = String()
        round_id, decision_hash = key
        release.data = json.dumps({
            'round_id': round_id,
            'decision_hash': decision_hash,
            'release_at_sim_time_s': release_at_s,
        }, sort_keys=True, separators=(',', ':'))
        self._release_publisher.publish(release)
        self._released_keys.add(key)
        self.get_logger().info(
            'TRAFFIC_TEST_DISPATCH_RELEASE round=%s decision_hash=%s '
            'release_at_sim_time_s=%.6f sim_clock_boundary=true' %
            (round_id, decision_hash, release_at_s),
        )


def main(args=None) -> int:
    rclpy.init(args=args)
    node = TrafficTestBarrier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
