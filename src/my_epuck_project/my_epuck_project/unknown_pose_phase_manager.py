"""Per-robot lifecycle switch from local exploration to shared exploration."""

from __future__ import annotations

import os
import signal
import time
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from my_epuck_interfaces.msg import RelativePoseHypothesis
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster


class UnknownPosePhaseManager(Node):
    """Switch only this robot's Nav2 phase after mutual frontend acceptance.

    This node is orchestration, not allocation: it has no map, goal, pose, or
    ground-truth input.  Both robots independently observe the same accepted
    hypothesis and perform the same local lifecycle transition.
    """

    def __init__(self):
        super().__init__('unknown_pose_phase_manager')
        self.robot_id = str(self.declare_parameter('robot_id', '').value)
        self.peer_robot_id = str(self.declare_parameter(
            'peer_robot_id', '').value)
        if not self.peer_robot_id and self.robot_id in ('robot1', 'robot2'):
            self.peer_robot_id = (
                'robot2' if self.robot_id == 'robot1' else 'robot1')
        self.shared_frame = str(self.declare_parameter(
            'shared_frame', 'shared_map').value)
        local_service = str(self.declare_parameter(
            'local_manager_service', '').value)
        shared_service = str(self.declare_parameter(
            'shared_manager_service', '').value)
        self._handoff_marker_path = str(self.declare_parameter(
            'handoff_marker_path', '').value)
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
        # Keep the mutually accepted alignment alive after the frontend is
        # intentionally torn down before shared Nav2 starts.  This relay only
        # republishes the accepted protocol message; it never reads truth or
        # estimates a transform.
        self._tf_broadcaster = StaticTransformBroadcaster(self)
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

    def _write_handoff_marker(self) -> None:
        """Record that local-child termination is an expected phase switch.

        The launch graph treats an unexpected frontend exit as fatal.  During
        the accepted handoff, however, the phase manager intentionally tears
        down the pre-handoff frontend before starting the shared stack.  A
        small per-robot marker lets that launch exit handler distinguish this
        bounded, expected teardown from an active-runtime crash without
        weakening fail-fast behavior for any other exit.
        """
        if not self._handoff_marker_path:
            return
        try:
            parent = os.path.dirname(self._handoff_marker_path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            temporary = self._handoff_marker_path + '.tmp'
            with open(temporary, 'w', encoding='utf-8') as stream:
                stream.write('accepted_handoff=true\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self._handoff_marker_path)
        except OSError as error:
            self.get_logger().error(
                'UNKNOWN_POSE_PHASE robot=%s handoff_marker_write_failed=%s' %
                (self.robot_id, error))

    def _local_process_pids(self):
        """Return only this robot's pre-handoff process PIDs.

        Lifecycle ``SHUTDOWN`` deactivates managed Nav2 nodes but does not
        terminate their processes.  The post-handoff launch intentionally
        creates the shared-map Nav2 stack with different node names, so
        leaving the local processes resident doubles the controller/costmap
        footprint.  The accepted frontend is deliberately retained: it is
        the bounded post-handoff local-evidence relay that keeps publishing
        ``PeerMap`` samples after the phase manager tears down local Nav2.
        Inspecting the exact ROS namespace and local node names keeps teardown
        scoped to this campaign robot; no broad ROS process matching or global
        kill is used.
        """
        selected = []
        for entry in os.listdir('/proc'):
            if not entry.isdigit():
                continue
            pid = int(entry)
            if pid == os.getpid():
                continue
            try:
                raw = open('/proc/%s/cmdline' % pid, 'rb').read()
            except (FileNotFoundError, PermissionError, OSError):
                continue
            argv = [part.decode(errors='replace') for part in raw.split(b'\0')
                    if part]
            if not argv:
                continue
            namespace = any(arg == '-r' for arg in argv)
            # Match the exact remap pair emitted by launch_ros, rather than
            # accepting a robot name anywhere in an executable or parameter.
            ns_match = any(
                argv[index + 1] == '__ns:=/%s' % self.robot_id
                for index, arg in enumerate(argv[:-1])
                if arg == '-r')
            if not namespace or not ns_match:
                continue
            node_names = [
                argv[index + 1][len('__node:='):]
                for index, arg in enumerate(argv[:-1])
                if arg == '-r' and argv[index + 1].startswith('__node:=')
            ]
            if any(name.startswith('local_') for name in node_names):
                selected.append(pid)
        return sorted(set(selected))

    def _terminate_local_processes(self) -> None:
        """Terminate this robot's deactivated pre-handoff process set."""
        pids = self._local_process_pids()
        if not pids:
            self.get_logger().info(
                'UNKNOWN_POSE_PHASE robot=%s local_process_teardown=none' %
                self.robot_id)
            return
        self.get_logger().info(
            'UNKNOWN_POSE_PHASE robot=%s local_process_teardown=term pids=%s' %
            (self.robot_id, ','.join(str(pid) for pid in pids)))
        for pid in pids:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

        def live(pid):
            """Return whether a PID still owns a running process.

            A launch child can remain as a zombie until its launch parent
            reaps it.  Treating the mere presence of ``/proc/<pid>`` as live
            made the phase switch escalate an already-terminated child to
            SIGKILL, which launch correctly reported as an unexpected
            frontend exit.  This is teardown bookkeeping only; it does not
            broaden which processes are selected.
            """
            try:
                fields = open('/proc/%s/stat' % pid, encoding='utf-8').read().split()
                return len(fields) < 3 or fields[2] not in ('Z', 'X')
            except (FileNotFoundError, PermissionError, OSError):
                return False

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            remaining = [pid for pid in pids if live(pid)]
            if not remaining:
                break
            time.sleep(0.05)
        remaining = [pid for pid in pids if live(pid)]
        for pid in remaining:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        self.get_logger().info(
            'UNKNOWN_POSE_PHASE robot=%s local_process_teardown=complete '
            'remaining=%s' % (self.robot_id, len(remaining)))

    def _hypothesis_callback(self, message: RelativePoseHypothesis) -> None:
        if not bool(message.accepted) or str(message.status) != 'ACCEPTED':
            return
        if not self._accepted:
            self._accepted = True
            self._publish_accepted_tf(message)
            self._write_handoff_marker()
            self._transition = 'SHUTTING_DOWN_LOCAL'
            self.get_logger().info(
                'UNKNOWN_POSE_PHASE robot=%s accepted_handoff=true '
                'transition=SHUTTING_DOWN_LOCAL' % self.robot_id)

    @staticmethod
    def _yaw_from_quaternion(rotation) -> float:
        return math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z))

    def _publish_accepted_tf(self, message: RelativePoseHypothesis) -> None:
        """Relay the canonical accepted alignment through phase teardown."""
        if (not self.peer_robot_id or
                self.robot_id != min(self.robot_id, self.peer_robot_id)):
            return
        tx = float(message.source_to_target.translation.x)
        ty = float(message.source_to_target.translation.y)
        yaw = self._yaw_from_quaternion(message.source_to_target.rotation)
        # The protocol transform maps source crop points into target crop
        # coordinates.  TF needs the target child pose in the shared parent,
        # hence one SE(2) inverse at this boundary.
        inverse_yaw = -yaw
        cos_yaw = math.cos(inverse_yaw)
        sin_yaw = math.sin(inverse_yaw)
        inverse_x = -(cos_yaw * tx - sin_yaw * ty)
        inverse_y = -(sin_yaw * tx + cos_yaw * ty)
        now = self.get_clock().now().to_msg()
        identity = TransformStamped()
        identity.header.stamp = now
        identity.header.frame_id = self.shared_frame
        identity.child_frame_id = f'{self.robot_id}/local_world'
        identity.transform.rotation.w = 1.0
        target = TransformStamped()
        target.header.stamp = now
        target.header.frame_id = self.shared_frame
        target.child_frame_id = f'{self.peer_robot_id}/local_world'
        target.transform.translation.x = inverse_x
        target.transform.translation.y = inverse_y
        target.transform.rotation.z = math.sin(inverse_yaw / 2.0)
        target.transform.rotation.w = math.cos(inverse_yaw / 2.0)
        self._tf_broadcaster.sendTransform([identity, target])
        self.get_logger().info(
            'UNKNOWN_POSE_PHASE robot=%s accepted_tf_relay=true '
            'parent=%s child=%s evidence_set_hash=%s' % (
                self.robot_id, self.shared_frame, target.child_frame_id,
                str(getattr(message, 'evidence_set_hash', ''))))

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
            if (self._transition == 'SHUTTING_DOWN_LOCAL' and
                    next_state == 'STARTING_SHARED'):
                self._terminate_local_processes()
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
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
