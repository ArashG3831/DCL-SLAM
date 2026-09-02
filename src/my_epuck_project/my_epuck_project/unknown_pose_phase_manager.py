"""Per-robot lifecycle switch from local exploration to shared exploration."""

from __future__ import annotations

import os
import signal
import time
import math
import json
import threading

import rclpy
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import GetState
from my_epuck_interfaces.msg import RelativePoseHypothesis
from nav2_msgs.srv import ManageLifecycleNodes
from std_msgs.msg import Bool
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
        self._cleanup_required = bool(self.declare_parameter(
            'historical_cleanup_required', True).value)
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
        # These read-only clients are retained for startup diagnostics.  The
        # lifecycle manager owns the managed-node ordering and is the only
        # service required to request STARTUP.  Waiting for every node's
        # get_state service here needlessly serializes discovery after the
        # manager is already able to perform its own dependency-aware start.
        shared_nodes = (
            'controller_server', 'smoother_server', 'planner_server',
            'route_server', 'behavior_server', 'velocity_smoother',
            'collision_monitor', 'bt_navigator', 'waypoint_follower',
        )
        self._shared_lifecycle_node_names = shared_nodes
        # Create the read-only shared lifecycle clients while handoff evidence
        # is accumulating.  The clients do not issue a transition before the
        # local SHUTDOWN safety boundary completes, but early creation lets DDS
        # discovery overlap the unavoidable unknown-pose wait and local
        # teardown.  This removes a post-handoff service-discovery stall
        # without allowing two Nav2 stacks to command the same robot.
        self._shared_lifecycle_clients = {
            name: self.create_client(
                GetState, f'/{self.robot_id}/{name}/get_state')
            for name in self._shared_lifecycle_node_names
        }
        self._last_shared_readiness_log = 0.0
        # Keep the mutually accepted alignment alive after the frontend is
        # intentionally torn down before shared Nav2 starts.  This relay only
        # republishes the accepted protocol message; it never reads truth or
        # estimates a transform.
        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self._accepted = False
        self._transition = 'WAITING_FOR_HANDOFF'
        self._request_in_flight = False
        self._transition_started = 0.0
        # ``PAUSE`` is the Nav2 lifecycle operation that removes local
        # controller authority while retaining the process long enough for a
        # bounded, scoped teardown.  Shared Nav2 may only start after this
        # request succeeds.  A conservative SHUTDOWN fallback remains for a
        # platform that rejects PAUSE; it never starts two active stacks.
        self._local_control_release_uses_shutdown = False
        self._local_process_teardown_started = False
        self._local_process_teardown_complete = False
        self._local_process_teardown_thread = None
        # A rejected lifecycle request leaves Nav2 in a partially transitioning
        # state for a short period.  Do not hammer the manager at 10 Hz: that
        # creates entity/log churn and can starve the action servers needed by
        # the next attempt.  A one-second bounded backoff preserves retry
        # behavior while allowing lifecycle cleanup/advertisement to settle.
        self._next_retry_at = 0.0
        self._retry_interval_s = 1.0
        self._cleanup_ready = {'robot1': False, 'robot2': False}
        self._shared_ready_publisher = self.create_publisher(
            Bool,
            f'/cslam/unknown_pose/{self.robot_id}/shared_nav2_ready',
            QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._hypothesis_sub = self.create_subscription(
            RelativePoseHypothesis, '/cslam/relative_pose/hypotheses',
            self._hypothesis_callback, qos)
        if self._cleanup_required:
            for robot in ('robot1', 'robot2'):
                self.create_subscription(
                    Bool,
                    f'/cslam/unknown_pose/{robot}/historical_cleanup_ready',
                    lambda message, item=robot: self._cleanup_callback(
                        item, message),
                    QoSProfile(
                        depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL),
                )
        self._timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(
            'UNKNOWN_POSE_PHASE robot=%s phase=PRE_HANDOFF '
            'local_nav2_active=true shared_nav2_waiting=true' % self.robot_id)

    def _sim_time_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _timeline(self, event: str, **fields) -> None:
        """Emit compact, parseable startup timing evidence.

        This is diagnostic-only.  It neither participates in estimation nor
        changes lifecycle decisions.  Including both clock domains makes
        accelerated Webots runs auditable without treating wall time as a
        simulation-time safety condition.
        """
        payload = {
            'event': event,
            'robot_id': self.robot_id,
            'sim_time_s': round(self._sim_time_s(), 6),
            'wall_monotonic_s': round(time.monotonic(), 6),
            **fields,
        }
        self.get_logger().info(
            'STARTUP_TIMELINE %s' % json.dumps(
                payload, sort_keys=True, separators=(',', ':')))

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
        self._timeline('LOCAL_NAV2_PROCESSES_TEARDOWN_STARTED')
        pids = self._local_process_pids()
        if not pids:
            self.get_logger().info(
                'UNKNOWN_POSE_PHASE robot=%s local_process_teardown=none' %
                self.robot_id)
            self._local_process_teardown_complete = True
            self._timeline('LOCAL_NAV2_PROCESSES_TEARDOWN_COMPLETE',
                           remaining=0)
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
        self._local_process_teardown_complete = True
        self._timeline('LOCAL_NAV2_PROCESSES_TEARDOWN_COMPLETE',
                       remaining=len(remaining))

    def _begin_local_process_teardown(self) -> None:
        """Retire inactive local processes without blocking shared startup.

        The caller has already received a successful lifecycle PAUSE (or the
        conservative SHUTDOWN fallback), so this thread cannot overlap two
        control authorities.  It only waits on this robot's exact local
        process set; shared lifecycle startup remains on the executor thread.
        """
        if self._local_process_teardown_started:
            return
        self._local_process_teardown_started = True
        self._local_process_teardown_thread = threading.Thread(
            target=self._terminate_local_processes,
            name='%s-local-nav2-teardown' % self.robot_id,
            daemon=True,
        )
        self._local_process_teardown_thread.start()

    def _hypothesis_callback(self, message: RelativePoseHypothesis) -> None:
        if not bool(message.accepted) or str(message.status) != 'ACCEPTED':
            return
        if not self._accepted:
            self._accepted = True
            self._publish_accepted_tf(message)
            self._write_handoff_marker()
            self._timeline('HANDOFF_LOCKED')
            self.get_logger().info(
                'UNKNOWN_POSE_PHASE robot=%s accepted_handoff=true '
                'cleanup_wait=%s' % (self.robot_id, self._cleanup_required))
            self._maybe_start_transition()

    def _cleanup_callback(self, robot, message):
        if not bool(message.data):
            return
        self._cleanup_ready[robot] = True
        self.get_logger().info(
            'UNKNOWN_POSE_PHASE robot=%s historical_cleanup_ready=%s' %
            (self.robot_id, robot))
        self._maybe_start_transition()

    def _maybe_start_transition(self):
        if not self._accepted or self._transition != 'WAITING_FOR_HANDOFF':
            return
        if self._cleanup_required and not all(self._cleanup_ready.values()):
            return
        self._transition = 'SHUTTING_DOWN_LOCAL'
        self._timeline('LOCAL_NAV_GOAL_CANCEL_REQUESTED',
                       reason='accepted_handoff_cleanup_complete')
        self.get_logger().info(
            'UNKNOWN_POSE_PHASE robot=%s transition=SHUTTING_DOWN_LOCAL' %
            self.robot_id)

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

    def _send(self, client, command, next_state: str, operation: str) -> None:
        if (self._request_in_flight or
                time.monotonic() < self._next_retry_at or
                not client.service_is_ready()):
            return
        request = ManageLifecycleNodes.Request()
        request.command = command
        if operation == 'LOCAL_CONTROL_RELEASE':
            self._timeline(
                'LOCAL_NAV2_LIFECYCLE_SHUTDOWN_REQUESTED',
                command=('SHUTDOWN' if self._local_control_release_uses_shutdown
                         else 'PAUSE'),
            )
        elif operation == 'SHARED_NAV2_STARTUP':
            self._timeline('SHARED_NAV2_LIFECYCLE_STARTUP_REQUESTED')
        self._request_in_flight = True
        self._transition_started = time.monotonic()
        future = client.call_async(request)

        def done(result):
            self._request_in_flight = False
            try:
                response = result.result()
            except Exception as error:  # noqa: B902
                if (operation == 'LOCAL_CONTROL_RELEASE' and
                        not self._local_control_release_uses_shutdown):
                    self._local_control_release_uses_shutdown = True
                    self._timeline(
                        'LOCAL_NAV2_PAUSE_FALLBACK_TO_SHUTDOWN',
                        reason='service_exception', error=str(error))
                self._next_retry_at = time.monotonic() + self._retry_interval_s
                self.get_logger().error(
                    'UNKNOWN_POSE_PHASE robot=%s transition=%s exception=%s' %
                    (self.robot_id, self._transition, error))
                return
            if not bool(response.success):
                if (operation == 'LOCAL_CONTROL_RELEASE' and
                        not self._local_control_release_uses_shutdown):
                    self._local_control_release_uses_shutdown = True
                    self._timeline(
                        'LOCAL_NAV2_PAUSE_FALLBACK_TO_SHUTDOWN',
                        reason='request_rejected')
                self._next_retry_at = time.monotonic() + self._retry_interval_s
                self.get_logger().error(
                    'UNKNOWN_POSE_PHASE robot=%s transition=%s rejected' %
                    (self.robot_id, self._transition))
                return
            if operation == 'LOCAL_CONTROL_RELEASE':
                release_command = (
                    'SHUTDOWN' if self._local_control_release_uses_shutdown
                    else 'PAUSE')
                self._timeline('LOCAL_NAV2_LIFECYCLE_INACTIVE',
                               command=release_command)
                # PAUSE completes only after local controller_server and the
                # action stack have deactivated.  It is the control-safety
                # boundary; process exit below is bookkeeping, not a reason
                # to delay inactive shared-stack activation.
                self._timeline('LOCAL_NAV_GOAL_TERMINAL',
                               terminal_by_lifecycle_release=True,
                               command=release_command)
                self._begin_local_process_teardown()
            self._transition = next_state
            if next_state == 'POST_HANDOFF_SHARED':
                ready = Bool()
                ready.data = True
                self._shared_ready_publisher.publish(ready)
                self.get_logger().info(
                    'UNKNOWN_POSE_PHASE robot=%s shared_nav2_ready=true' %
                    self.robot_id)
                self._timeline('SHARED_NAV2_LIFECYCLE_ACTIVE')
                self._log_shared_lifecycle_states()
            self.get_logger().info(
                'UNKNOWN_POSE_PHASE robot=%s transition_complete=%s' %
                (self.robot_id, next_state))

        future.add_done_callback(done)

    def _log_shared_lifecycle_states(self) -> None:
        """Record the critical shared nodes' reported lifecycle state."""
        for node_name, event in (
                ('planner_server', 'SHARED_PLANNER_ACTIVE'),
                ('controller_server', 'SHARED_CONTROLLER_ACTIVE')):
            client = self._shared_lifecycle_clients.get(node_name)
            if client is None or not client.service_is_ready():
                continue
            future = client.call_async(GetState.Request())

            def done(result, item=node_name, event_name=event):
                try:
                    response = result.result()
                    state = response.current_state
                    self._timeline(
                        event_name,
                        lifecycle_id=int(state.id),
                        lifecycle_label=str(state.label),
                        node=item,
                    )
                except Exception as error:  # noqa: B902
                    self._timeline(
                        'SHARED_LIFECYCLE_STATE_QUERY_FAILED', node=item,
                        error=str(error))

            future.add_done_callback(done)

    def _ensure_shared_lifecycle_clients(self) -> None:
        if self._shared_lifecycle_clients:
            return
        self._shared_lifecycle_clients = {
            name: self.create_client(
                GetState, f'/{self.robot_id}/{name}/get_state')
            for name in self._shared_lifecycle_node_names
        }

    def _tick(self) -> None:
        if self._transition == 'SHUTTING_DOWN_LOCAL':
            self._send(
                # PAUSE is the narrow safety boundary: it deactivates the
                # local controller and action stack before shared Nav2 can be
                # activated.  Full process retirement continues in parallel
                # afterward.  If PAUSE is rejected, the explicit SHUTDOWN
                # fallback preserves the former conservative behavior.
                self._local_client,
                (ManageLifecycleNodes.Request.SHUTDOWN
                 if self._local_control_release_uses_shutdown
                 else ManageLifecycleNodes.Request.PAUSE),
                'STARTING_SHARED', 'LOCAL_CONTROL_RELEASE')
        elif self._transition == 'STARTING_SHARED':
            self._ensure_shared_lifecycle_clients()
            if not self._shared_client.service_is_ready():
                now = time.monotonic()
                if now - self._last_shared_readiness_log >= 2.0:
                    self._last_shared_readiness_log = now
                    self.get_logger().info(
                        'UNKNOWN_POSE_PHASE robot=%s '
                        'shared_nav2_waiting_for_lifecycle_manager=true' %
                        self.robot_id)
                return
            self._send(
                self._shared_client, ManageLifecycleNodes.Request.STARTUP,
                'POST_HANDOFF_SHARED', 'SHARED_NAV2_STARTUP')
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
