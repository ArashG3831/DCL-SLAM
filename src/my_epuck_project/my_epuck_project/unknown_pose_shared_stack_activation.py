"""One-shot post-handoff launcher for the shared exploration stack.

This process is orchestration only.  It never estimates a pose, publishes a
transform, reads Supervisor state, or commands Nav2.  The frontends remain the
only runtime source of the accepted relative-pose handoff.
"""

from __future__ import annotations

import os
import shutil
import signal
import subprocess
import time
import math
import json

import rclpy
from my_epuck_interfaces.msg import RelativePoseHypothesis
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


LOCAL_PATH_GATE_MODES = ('MODE_A', 'MODE_B')


class UnknownPoseSharedStackActivation(Node):
    """Launch shared fusion/frontiers/allocation exactly once after acceptance."""

    def __init__(self):
        super().__init__('unknown_pose_shared_stack_activation')
        self._process = None
        self._activated = False
        self._accepted_message = None
        self._cleanup_ready = {'robot1': False, 'robot2': False}
        self._shared_nav2_ready = {'robot1': False, 'robot2': False}
        self._shared_costmap_seen = {'robot1': False, 'robot2': False}
        self._shared_local_costmap_seen = {'robot1': False, 'robot2': False}
        self._shared_nav2_barrier_published = False
        self._cooperative_start_ready = {'robot1': False, 'robot2': False}
        self._start_release_published = False
        self._shutdown_requested = False
        self._ros2 = shutil.which('ros2') or 'ros2'
        boolean_parameters = {
            'webots_gui',
            'use_sim_time',
            'use_scan_matching',
            'do_loop_closing',
            'nav2_autostart',
            'diagnostic_mode',
            'diagnostic_frontier_capture',
            'traffic_scheduler_enabled',
            'synchronized_traffic_test',
            'traffic_test_force_conflict_pair',
            'prelaunch_shared_nav2',
            'enable_mission_timeout',
            'common_start_release_required',
            'event_driven_costing',
        }
        parameter_defaults = (
                ('world_profile', 'large_unknown_pose_16m'),
                ('world_path', ''),
                ('webots_mode', 'fast'),
                ('webots_gui', False),
                ('use_sim_time', True),
                ('use_scan_matching', False),
                ('do_loop_closing', False),
                # The phase manager owns the one explicit shared Nav2 STARTUP
                # transition after local controller authority is released.
                # Autostarting here races that transition and makes a correct
                # STARTUP request fail because the nodes are already active.
                ('nav2_autostart', False),
                ('sensor_profile', 'full'),
                ('scan_input_reliability', 'reliable'),
                ('diagnostic_mode', False),
                ('diagnostic_frontier_capture', False),
                ('fusion_process_nice', 0),
                ('fusion_cpu_quota_percent', 30),
                ('fusion_rebuild_period_s', 1.0),
                ('controller_variant', 'rpp'),
                ('assignment_strategy', 'frontier_mrtsp'),
                # The parent launch must provide this explicitly.  An empty
                # default makes a standalone/miswired activation fail closed
                # instead of silently selecting the nested launch default.
                ('local_path_gate_mode', ''),
                ('burgard_beta', 1.0),
                ('traffic_scheduler_enabled', False),
                ('synchronized_traffic_test', False),
                ('traffic_test_force_conflict_pair', False),
                # Opt-in only: keeping four Nav2 graphs resident during
                # unknown-pose startup can starve Webots/DDS on a loaded host.
                # The normal profile launches this graph at accepted handoff.
                ('prelaunch_shared_nav2', False),
                ('common_start_release_required', False),
                ('event_driven_costing', False),
                ('enable_mission_timeout', False),
                ('mission_timeout_s', 600.0),
                ('terminal_small_frontier_length_m', 0.20),
                ('slam_tf_publish_probe_library', ''),
                ('slam_tf_publish_probe_log', ''),
                ('slam_tf_publication_mode', ''),
                ('webots_port', 23000),
        )
        self._parameters = {}
        for name, default in parameter_defaults:
            value = self.declare_parameter(name, default).value
            if name in boolean_parameters:
                self._parameters[name] = str(bool(value)).lower()
            else:
                self._parameters[name] = str(value)
        local_path_gate_mode = self._parameters.get(
            'local_path_gate_mode', '').strip().upper()
        if local_path_gate_mode not in LOCAL_PATH_GATE_MODES:
            raise ValueError(
                'local_path_gate_mode must be explicitly provided as '
                'MODE_A or MODE_B')
        self._parameters['local_path_gate_mode'] = local_path_gate_mode
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._hypothesis_sub = self.create_subscription(
            RelativePoseHypothesis,
            '/cslam/relative_pose/hypotheses',
            self._hypothesis_callback,
            qos,
        )
        self._shared_nav2_ready_publisher = self.create_publisher(
            Bool,
            '/cslam/unknown_pose/shared_nav2_ready',
            QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._start_release_publisher = self.create_publisher(
            String,
            '/cslam/unknown_pose/start_release',
            QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
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
            self.create_subscription(
                String,
                f'/cslam/unknown_pose/cooperative_start_ready/{robot}',
                lambda message, item=robot: self._start_ready_callback(
                    item, message),
                QoSProfile(
                    depth=1, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
            self.create_subscription(
                Bool,
                f'/cslam/unknown_pose/{robot}/shared_nav2_ready',
                lambda message, item=robot: self._shared_nav2_ready_callback(
                    item, message),
                QoSProfile(
                    depth=1, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
            self.create_subscription(
                OccupancyGrid,
                f'/{robot}/global_costmap/costmap',
                lambda message, item=robot: self._shared_costmap_callback(
                    item, message),
                QoSProfile(
                    depth=1, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
            self.create_subscription(
                OccupancyGrid,
                f'/{robot}/local_costmap/costmap',
                lambda message, item=robot: self._shared_local_costmap_callback(
                    item, message),
                QoSProfile(
                    depth=1, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
        self._poll_timer = self.create_timer(0.5, self._poll_child)
        self.get_logger().info(
            'UNKNOWN_POSE_SHARED_ACTIVATION waiting_for_accepted_handoff=true')

    def _timeline(self, event: str, **fields) -> None:
        """Emit parseable phase-readiness timing evidence only."""
        payload = {
            'event': event,
            'sim_time_s': round(self.get_clock().now().nanoseconds / 1e9, 6),
            'wall_monotonic_s': round(time.monotonic(), 6),
            **fields,
        }
        self.get_logger().info(
            'STARTUP_TIMELINE %s' % json.dumps(
                payload, sort_keys=True, separators=(',', ':')))

    def _hypothesis_callback(self, message: RelativePoseHypothesis) -> None:
        """Start only for a canonical accepted hypothesis."""
        if self._activated or self._shutdown_requested:
            return
        if not bool(message.accepted) or str(message.status) != 'ACCEPTED':
            return
        self._accepted_message = message
        self._maybe_start_shared_stack()

    def _cleanup_callback(self, robot, message):
        if bool(message.data):
            self._cleanup_ready[robot] = True
            self._maybe_start_shared_stack()

    def _shared_nav2_ready_callback(self, robot, message):
        if bool(message.data):
            # Before handoff, the inert activation process already subscribes
            # to these topics and can see a retained *local* costmap sample.
            # Resetting at the successful shared lifecycle transition makes
            # the barrier require a fresh map emitted by the shared stack.
            if not self._shared_nav2_ready[robot]:
                self._shared_costmap_seen[robot] = False
                self._shared_local_costmap_seen[robot] = False
            self._shared_nav2_ready[robot] = True
            self.get_logger().info(
                'UNKNOWN_POSE_SHARED_ACTIVATION robot=%s '
                'shared_nav2_ready=true' % robot)
            self._maybe_publish_shared_nav2_ready()

    def _shared_costmap_callback(self, robot, _message):
        if not self._shared_costmap_seen[robot]:
            self._shared_costmap_seen[robot] = True
            self.get_logger().info(
                'UNKNOWN_POSE_SHARED_ACTIVATION robot=%s '
                'shared_costmap_seen=true' % robot)
            self._timeline('SHARED_GLOBAL_COSTMAP_ACTIVE', robot_id=robot)
            self._maybe_publish_shared_nav2_ready()

    def _shared_local_costmap_callback(self, robot, _message):
        if not self._shared_local_costmap_seen[robot]:
            self._shared_local_costmap_seen[robot] = True
            self.get_logger().info(
                'UNKNOWN_POSE_SHARED_ACTIVATION robot=%s '
                'shared_local_costmap_seen=true' % robot)
            self._timeline('SHARED_LOCAL_COSTMAP_ACTIVE', robot_id=robot)
            self._maybe_publish_shared_nav2_ready()

    def _maybe_publish_shared_nav2_ready(self):
        if self._shared_nav2_barrier_published:
            return
        if not (all(self._shared_nav2_ready.values()) and
                all(self._shared_costmap_seen.values()) and
                all(self._shared_local_costmap_seen.values())):
            return
        self._shared_nav2_barrier_published = True
        ready = Bool()
        ready.data = True
        self._shared_nav2_ready_publisher.publish(ready)
        self.get_logger().info(
            'UNKNOWN_POSE_SHARED_ACTIVATION shared_nav2_barrier_released=true')
        self._timeline('SHARED_NAV2_COSTMAP_BARRIER_RELEASED')
        self._maybe_publish_start_release()

    def _start_ready_callback(self, robot, message):
        try:
            payload = json.loads(str(message.data))
            if (str(payload.get('event', '')) !=
                    'COOPERATIVE_START_STATE_READY'):
                return
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        self._cooperative_start_ready[robot] = True
        self._timeline('COOPERATIVE_START_STATE_READY_OBSERVED',
                       robot_id=robot)
        self._maybe_publish_start_release()

    def _maybe_publish_start_release(self):
        if (self._start_release_published or
                not self._shared_nav2_barrier_published or
                not all(self._cooperative_start_ready.values()) or
                self._accepted_message is None):
            return
        # The traffic scheduler is disabled in the current authoritative C
        # profile; that is a ready/no-gate state, not a reason to delay the
        # common release.  If enabled, the existing scheduler remains the
        # authority for later dispatch conflict decisions.
        release_time = self.get_clock().now().nanoseconds / 1e9
        message = String()
        message.data = json.dumps({
            'event': 'START_RELEASE',
            'release_sim_time_s': release_time,
            'traffic_scheduler_ready': True,
            'shared_nav2_ready': True,
            'accepted_handoff': True,
            'ready_robots': ['robot1', 'robot2'],
        }, sort_keys=True, separators=(',', ':'))
        self._start_release_publisher.publish(message)
        self._start_release_published = True
        self.get_logger().info(
            'START_RELEASE t=%.6f traffic_scheduler_ready=true' %
            release_time)
        self._timeline('START_RELEASE', release_sim_time_s=release_time,
                       traffic_scheduler_ready=True)

    def _maybe_start_shared_stack(self):
        if (self._activated or self._shutdown_requested or
                self._accepted_message is None):
            return
        # The launched shared stack is explicitly phase-gated and Nav2 is
        # explicitly non-autostarting.  Starting those inactive processes at
        # handoff is therefore safe and lets process/DDS discovery overlap the
        # existing historical-cleanup work.  The phase managers still own the
        # real lifecycle STARTUP transition and the shared Nav2 readiness
        # barrier; cleanup semantics and the activation barrier are unchanged.
        if not all(self._cleanup_ready.values()):
            self.get_logger().info(
                'UNKNOWN_POSE_SHARED_ACTIVATION starting_phase_gated_stack='
                'true cleanup_ready=%s' % self._cleanup_ready)
        self._activated = True
        message = self._accepted_message
        transform = message.source_to_target
        quaternion = transform.rotation
        yaw = math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2),
        )
        self._start_shared_stack(
            float(transform.translation.x),
            float(transform.translation.y),
            yaw,
            str(getattr(message, 'evidence_set_hash', '')),
        )

    def _start_shared_stack(self, x, y, yaw, evidence_hash):
        args = [
            self._ros2, 'launch', 'my_epuck_project',
            'two_robots_distributed_assignment_launch.py',
        ]
        values = dict(self._parameters)
        values.update({
            'dispatch_enabled': 'true',
            'unknown_initial_pose': 'true',
            'phase_already_aligned': 'true',
            # The persistent pre-handoff minimal allocator owns the single
            # robot-local allocation process across this phase switch.
            'launch_allocator': 'false',
            'launch_mapping': 'false',
            'local_path_gate_mode': self._parameters['local_path_gate_mode'],
            # Initial unknown-pose startup may already have launched the
            # shared Nav2 processes inactive and phase-gated.  In that case
            # launch only the distributed frontier/assignment layer now.
            'launch_shared_stack': (
                'false' if self._parameters.get('prelaunch_shared_nav2')
                == 'true' else 'true'),
            'prelaunch_shared_nav2': 'false',
            # The fusion pair is already resident in an inert, handoff-gated
            # state from the initial launch.  Do not start duplicate nodes.
            'launch_shared_fusion': 'false',
            'handoff_transform_x': f'{x:.12g}',
            'handoff_transform_y': f'{y:.12g}',
            'handoff_transform_yaw': f'{yaw:.12g}',
            'handoff_evidence_set_hash': evidence_hash,
            # Keep shared assignment peers inert until both Nav2 lifecycle
            # managers are active and both shared costmaps have produced a
            # sample.  This gates readiness only; task policy is unchanged.
            'phase_gated': 'true',
            'shared_nav2_ready_topic':
                '/cslam/unknown_pose/shared_nav2_ready',
        })
        # This activation launches the distributed-assignment launch file,
        # whose public arguments intentionally do not include the
        # activation-only prelaunch switch.
        values.pop('prelaunch_shared_nav2', None)
        # Optional probe parameters are intentionally empty when no probe is
        # configured.  Do not emit malformed ``name:=`` launch tokens: ROS 2
        # rejects those before the post-handoff stack can start.
        args.extend(
            f'{key}:={value}' for key, value in values.items()
            if value != '')
        self.get_logger().info(
            'UNKNOWN_POSE_SHARED_ACTIVATION starting=true evidence_set_hash=%s '
            'argv=%s' % (evidence_hash, ' '.join(args)))
        self._timeline('SHARED_NAV2_PROCESSES_LAUNCH_STARTED',
                       evidence_set_hash=evidence_hash)
        try:
            self._process = subprocess.Popen(
                args,
                env=os.environ.copy(),
                start_new_session=True,
                close_fds=True,
            )
            self._timeline('SHARED_NAV2_PROCESSES_PRESENT',
                           process_pid=int(self._process.pid))
        except Exception as error:  # surface a deterministic campaign failure
            self.get_logger().error(
                'UNKNOWN_POSE_SHARED_ACTIVATION_START_FAILURE error=%s' % error)
            self._process = None

    def _poll_child(self) -> None:
        if self._process is None:
            return
        return_code = self._process.poll()
        if return_code is None:
            return
        self.get_logger().error(
            'UNKNOWN_POSE_SHARED_ACTIVATION_EXIT return_code=%s' % return_code)
        self._process = None

    def shutdown_child(self) -> None:
        """Terminate only the process group owned by this activation."""
        self._shutdown_requested = True
        process = self._process
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGTERM)
            deadline = time.monotonic() + 8.0
            while process.poll() is None and time.monotonic() < deadline:
                time.sleep(0.1)
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = UnknownPoseSharedStackActivation()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.shutdown_child()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
