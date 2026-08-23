"""Per-robot lifecycle switch from local exploration to shared exploration."""

from __future__ import annotations

import time

import rclpy
from my_epuck_interfaces.msg import RelativePoseHypothesis
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class UnknownPosePhaseManager(Node):
    """Switch only this robot's Nav2 phase after mutual frontend acceptance.

    This node is orchestration, not allocation: it has no map, goal, pose, or
    ground-truth input.  Both robots independently observe the same accepted
    hypothesis and perform the same local lifecycle transition.
    """

    def __init__(self):
        super().__init__('unknown_pose_phase_manager')
        self.robot_id = str(self.declare_parameter('robot_id', '').value)
        local_service = str(self.declare_parameter(
            'local_manager_service', '').value)
        shared_service = str(self.declare_parameter(
            'shared_manager_service', '').value)
        if not self.robot_id or not local_service or not shared_service:
            raise ValueError('robot_id and lifecycle manager services are required')
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            # The frontend publishes accepted hypotheses with volatile
            # durability.  Matching it is required for the phase transition
            # subscriber to receive the one-shot handoff announcement.
            durability=DurabilityPolicy.VOLATILE,
        )
        self._local_client = self.create_client(
            ManageLifecycleNodes, local_service)
        self._shared_client = self.create_client(
            ManageLifecycleNodes, shared_service)
        self._accepted = False
        self._transition = 'WAITING_FOR_HANDOFF'
        self._request_in_flight = False
        self._transition_started = 0.0
        # A rejected lifecycle request leaves Nav2 in a partially transitioning
        # state for a short period.  Do not hammer the manager at 10 Hz: that
        # creates entity/log churn and can starve the action servers needed by
        # the next attempt.  A one-second bounded backoff preserves retry
        # behavior while allowing lifecycle cleanup/advertisement to settle.
        self._next_retry_at = 0.0
        self._retry_interval_s = 1.0
        self._hypothesis_sub = self.create_subscription(
            RelativePoseHypothesis, '/cslam/relative_pose/hypotheses',
            self._hypothesis_callback, qos)
        self._timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(
            'UNKNOWN_POSE_PHASE robot=%s phase=PRE_HANDOFF '
            'local_nav2_active=true shared_nav2_waiting=true' % self.robot_id)

    def _hypothesis_callback(self, message: RelativePoseHypothesis) -> None:
        if not bool(message.accepted) or str(message.status) != 'ACCEPTED':
            return
        if not self._accepted:
            self._accepted = True
            self._transition = 'SHUTTING_DOWN_LOCAL'
            self.get_logger().info(
                'UNKNOWN_POSE_PHASE robot=%s accepted_handoff=true '
                'transition=SHUTTING_DOWN_LOCAL' % self.robot_id)

    def _send(self, client, command, next_state: str) -> None:
        if (self._request_in_flight or
                time.monotonic() < self._next_retry_at or
                not client.service_is_ready()):
            return
        request = ManageLifecycleNodes.Request()
        request.command = command
        self._request_in_flight = True
        self._transition_started = time.monotonic()
        future = client.call_async(request)

        def done(result):
            self._request_in_flight = False
            try:
                response = result.result()
            except Exception as error:  # noqa: B902
                self._next_retry_at = time.monotonic() + self._retry_interval_s
                self.get_logger().error(
                    'UNKNOWN_POSE_PHASE robot=%s transition=%s exception=%s' %
                    (self.robot_id, self._transition, error))
                return
            if not bool(response.success):
                self._next_retry_at = time.monotonic() + self._retry_interval_s
                self.get_logger().error(
                    'UNKNOWN_POSE_PHASE robot=%s transition=%s rejected' %
                    (self.robot_id, self._transition))
                return
            self._transition = next_state
            self.get_logger().info(
                'UNKNOWN_POSE_PHASE robot=%s transition_complete=%s' %
                (self.robot_id, next_state))

        future.add_done_callback(done)

    def _tick(self) -> None:
        if self._transition == 'SHUTTING_DOWN_LOCAL':
            self._send(
                self._local_client, ManageLifecycleNodes.Request.SHUTDOWN,
                'STARTING_SHARED')
        elif self._transition == 'STARTING_SHARED':
            self._send(
                self._shared_client, ManageLifecycleNodes.Request.STARTUP,
                'POST_HANDOFF_SHARED')
        elif self._transition == 'POST_HANDOFF_SHARED':
            return


def main(args=None):
    rclpy.init(args=args)
    node = UnknownPosePhaseManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
