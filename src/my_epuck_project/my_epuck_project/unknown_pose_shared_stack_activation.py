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

import rclpy
from my_epuck_interfaces.msg import RelativePoseHypothesis
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class UnknownPoseSharedStackActivation(Node):
    """Launch shared fusion/frontiers/allocation exactly once after acceptance."""

    def __init__(self):
        super().__init__('unknown_pose_shared_stack_activation')
        self._process = None
        self._activated = False
        self._shutdown_requested = False
        self._ros2 = shutil.which('ros2') or 'ros2'
        boolean_parameters = {
            'webots_gui',
            'use_sim_time',
            'use_scan_matching',
            'do_loop_closing',
            'diagnostic_mode',
            'diagnostic_frontier_capture',
            'traffic_scheduler_enabled',
            'enable_mission_timeout',
        }
        parameter_defaults = (
                ('world_profile', 'large_unknown_pose_16m'),
                ('world_path', ''),
                ('webots_mode', 'fast'),
                ('webots_gui', False),
                ('use_sim_time', True),
                ('use_scan_matching', False),
                ('do_loop_closing', False),
                ('sensor_profile', 'full'),
                ('diagnostic_mode', False),
                ('diagnostic_frontier_capture', False),
                ('fusion_process_nice', 0),
                ('fusion_cpu_quota_percent', 30),
                ('fusion_rebuild_period_s', 1.0),
                ('controller_variant', 'rpp'),
                ('assignment_strategy', 'burgard'),
                ('burgard_beta', 1.0),
                ('traffic_scheduler_enabled', False),
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
        self._poll_timer = self.create_timer(0.5, self._poll_child)
        self.get_logger().info(
            'UNKNOWN_POSE_SHARED_ACTIVATION waiting_for_accepted_handoff=true')

    def _hypothesis_callback(self, message: RelativePoseHypothesis) -> None:
        """Start only for a canonical accepted hypothesis."""
        if self._activated or self._shutdown_requested:
            return
        if not bool(message.accepted) or str(message.status) != 'ACCEPTED':
            return
        self._activated = True
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
            'launch_mapping': 'false',
            'launch_shared_stack': 'true',
            'handoff_transform_x': f'{x:.12g}',
            'handoff_transform_y': f'{y:.12g}',
            'handoff_transform_yaw': f'{yaw:.12g}',
            'handoff_evidence_set_hash': evidence_hash,
        })
        # Optional probe parameters are intentionally empty when no probe is
        # configured.  Do not emit malformed ``name:=`` launch tokens: ROS 2
        # rejects those before the post-handoff stack can start.
        args.extend(
            f'{key}:={value}' for key, value in values.items()
            if value != '')
        self.get_logger().info(
            'UNKNOWN_POSE_SHARED_ACTIVATION starting=true evidence_set_hash=%s '
            'argv=%s' % (evidence_hash, ' '.join(args)))
        try:
            self._process = subprocess.Popen(
                args,
                env=os.environ.copy(),
                start_new_session=True,
                close_fds=True,
            )
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
    finally:
        node.shutdown_child()
        node.destroy_node()
        rclpy.shutdown()
