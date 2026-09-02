"""Visualization-only aliases for independent robot occupancy maps.

This node deliberately creates no relationship between the real robot map
frames.  It copies maps onto separate ``viz/*`` topics and publishes only
``viz/*`` static transforms so RViz can show independent maps in one window.
The bridge is optional and must not be used by estimation or control nodes.
"""

from copy import deepcopy
import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
from my_epuck_interfaces.msg import RelativePoseHypothesis
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker


VIZ_WORLD = 'viz/world'


def alias_map(message: OccupancyGrid, alias_frame: str) -> OccupancyGrid:
    """Copy a map while changing only its visualization frame ID."""
    result = deepcopy(message)
    result.header.frame_id = alias_frame
    return result


def alias_transform(
        child_frame: str, offset_x_m: float, offset_y_m: float,
        offset_z_m: float, offset_yaw_rad: float) -> TransformStamped:
    """Build a visualization-only transform with no real-frame names."""
    if not child_frame.startswith('viz/'):
        raise ValueError('visualization alias must start with viz/')
    transform = TransformStamped()
    transform.header.frame_id = VIZ_WORLD
    transform.child_frame_id = child_frame
    transform.transform.translation.x = float(offset_x_m)
    transform.transform.translation.y = float(offset_y_m)
    transform.transform.translation.z = float(offset_z_m)
    half = float(offset_yaw_rad) / 2.0
    transform.transform.rotation.z = math.sin(half)
    transform.transform.rotation.w = math.cos(half)
    return transform


class RvizMapOverlay(Node):
    """Relay maps and publish visualization-only poses, paths, and status."""

    def __init__(self):
        super().__init__('rviz_map_overlay')
        self.declare_parameter('viz_world_frame', VIZ_WORLD)
        self.declare_parameter('robot1_map_topic', '/robot1/map')
        self.declare_parameter('robot2_map_topic', '/robot2/map')
        self.declare_parameter('robot1_output_topic', '/viz/robot1_map')
        self.declare_parameter('robot2_output_topic', '/viz/robot2_map')
        self.declare_parameter('robot1_alias_frame', 'viz/robot1_map')
        self.declare_parameter('robot2_alias_frame', 'viz/robot2_map')
        self.declare_parameter('robot1_shared_map_topic', '/robot1/shared_map_visualization')
        self.declare_parameter('robot2_shared_map_topic', '/robot2/shared_map_visualization')
        self.declare_parameter('shared_output_topic', '/viz/shared_map')
        self.declare_parameter('handoff_topic', '/cslam/relative_pose/hypotheses')
        self.declare_parameter('handoff_status_topic', '/viz/handoff_status')
        self.declare_parameter('robot1_path_topic', '/viz/robot1_traveled_path')
        self.declare_parameter('robot2_path_topic', '/viz/robot2_traveled_path')
        for robot, defaults in (
                ('robot1', (0.0, 0.0, 0.0, 0.0)),
                ('robot2', (10.0, 0.0, 0.0, 0.0))):
            self.declare_parameter(f'{robot}_offset_x_m', defaults[0])
            self.declare_parameter(f'{robot}_offset_y_m', defaults[1])
            self.declare_parameter(f'{robot}_offset_z_m', defaults[2])
            self.declare_parameter(f'{robot}_offset_yaw_rad', defaults[3])

        self.viz_world = str(self.get_parameter('viz_world_frame').value)
        if not self.viz_world.startswith('viz/'):
            raise ValueError('viz_world_frame must start with viz/')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_publishers = {}
        self._aliases = {}
        self._map_offsets = {}
        for robot in ('robot1', 'robot2'):
            source_topic = str(self.get_parameter(
                f'{robot}_map_topic').value)
            output_topic = str(self.get_parameter(
                f'{robot}_output_topic').value)
            alias_frame = str(self.get_parameter(
                f'{robot}_alias_frame').value)
            if not alias_frame.startswith('viz/'):
                raise ValueError('map alias frames must start with viz/')
            self._aliases[robot] = alias_frame
            self._map_offsets[robot] = (
                float(self.get_parameter(f'{robot}_offset_x_m').value),
                float(self.get_parameter(f'{robot}_offset_y_m').value),
                float(self.get_parameter(f'{robot}_offset_yaw_rad').value))
            publisher = self.create_publisher(OccupancyGrid, output_topic, qos)
            self._map_publishers[robot] = publisher
            self.create_subscription(
                OccupancyGrid, source_topic,
                lambda message, name=robot: self._relay(name, message), qos)

        self._shared_publisher = self.create_publisher(
            OccupancyGrid, str(self.get_parameter('shared_output_topic').value), qos)
        shared_topic_qos = qos
        for robot in ('robot1', 'robot2'):
            self.create_subscription(
                OccupancyGrid,
                str(self.get_parameter(f'{robot}_shared_map_topic').value),
                lambda message, name=robot: self._relay_shared(name, message),
                shared_topic_qos)

        self._path_publishers = {
            robot: self.create_publisher(
                Path, str(self.get_parameter(f'{robot}_path_topic').value), qos)
            for robot in ('robot1', 'robot2')}
        self._paths = {robot: Path() for robot in ('robot1', 'robot2')}
        self._last_path_xy = {robot: None for robot in ('robot1', 'robot2')}
        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._pose_tf = TransformBroadcaster(self)
        self._handoff_publisher = self.create_publisher(
            Marker, str(self.get_parameter('handoff_status_topic').value), qos)
        self._handoff_accepted = False
        self._handoff_time = None
        # RelativePoseHypothesis is published by the frontends with volatile
        # durability.  Keep the visualization subscriber compatible with that
        # runtime QoS; the marker itself is transient-local for RViz.
        handoff_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            RelativePoseHypothesis,
            str(self.get_parameter('handoff_topic').value),
            self._handoff_callback, handoff_qos)

        self._tf = StaticTransformBroadcaster(self)
        transforms = []
        for robot in ('robot1', 'robot2'):
            transforms.append(alias_transform(
                self._aliases[robot],
                float(self.get_parameter(f'{robot}_offset_x_m').value),
                float(self.get_parameter(f'{robot}_offset_y_m').value),
                float(self.get_parameter(f'{robot}_offset_z_m').value),
                float(self.get_parameter(f'{robot}_offset_yaw_rad').value)))
        transforms.append(alias_transform('viz/shared_map', 0.0, 0.0, 0.0, 0.0))
        for transform in transforms:
            transform.header.frame_id = self.viz_world
        self._tf.sendTransform(transforms)
        self._publish_handoff_marker()
        self.create_timer(0.10, self._update_visual_poses)
        self.get_logger().info(
            'VISUALIZATION_ONLY_OVERLAY active; real map/TF frames are untouched')

    def _relay(self, robot: str, message: OccupancyGrid):
        self._map_publishers[robot].publish(alias_map(
            message, self._aliases[robot]))

    def _relay_shared(self, _robot: str, message: OccupancyGrid):
        """Expose one shared-map copy in the visualization-only frame."""
        # The accepted-handoff hypothesis is intentionally volatile because
        # it is a runtime protocol event. A manually started viewer can miss
        # it. In unknown-pose mode a shared-map publication is the durable,
        # phase-gated evidence that the handoff boundary was crossed; use it
        # only to repair this visualization node's local display state. It
        # never feeds estimation or control.
        if not self._handoff_accepted:
            self._accept_handoff('shared_map_observed')
        self._shared_publisher.publish(alias_map(message, 'viz/shared_map'))

    @staticmethod
    def _planar(transform):
        q = transform.transform.rotation
        return (float(transform.transform.translation.x),
                float(transform.transform.translation.y),
                math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    @staticmethod
    def _compose(first, second):
        c, s = math.cos(first[2]), math.sin(first[2])
        return (first[0] + c * second[0] - s * second[1],
                first[1] + s * second[0] + c * second[1],
                first[2] + second[2])

    @staticmethod
    def _quaternion(yaw):
        return math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def _lookup_pose(self, robot):
        base = f'{robot}/base_footprint'
        if self._handoff_accepted:
            # Once handoff is accepted, the shared frame is the only valid
            # visualization frame for the merged map.  Do not fall back to
            # the pre-handoff map alias, whose deliberate offset would make a
            # robot appear to float away from the shared map.
            candidates = (('shared_map', (0.0, 0.0, 0.0)),)
        else:
            candidates = (
                (f'{robot}/map', self._map_offsets[robot]),
                ('shared_map', (0.0, 0.0, 0.0)),
            )
        for parent, offset in candidates:
            try:
                tf = self._tf_buffer.lookup_transform(
                    parent, base, Time(), timeout=Duration(seconds=0.01))
            except Exception:
                continue
            return self._compose(offset, self._planar(tf))
        return None

    def _update_visual_poses(self):
        now = self.get_clock().now().to_msg()
        for robot in ('robot1', 'robot2'):
            pose = self._lookup_pose(robot)
            if pose is None:
                continue
            qz, qw = self._quaternion(pose[2])
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self.viz_world
            tf.child_frame_id = f'viz/{robot}/base_footprint'
            tf.transform.translation.x = pose[0]
            tf.transform.translation.y = pose[1]
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self._pose_tf.sendTransform(tf)
            previous = self._last_path_xy[robot]
            if previous is not None and math.hypot(
                    pose[0] - previous[0], pose[1] - previous[1]) < 0.02:
                continue
            self._last_path_xy[robot] = pose[:2]
            path = self._paths[robot]
            path.header.stamp = now
            path.header.frame_id = self.viz_world
            item = PoseStamped()
            item.header = path.header
            item.pose.position.x = pose[0]
            item.pose.position.y = pose[1]
            item.pose.orientation.z = qz
            item.pose.orientation.w = qw
            path.poses.append(item)
            if len(path.poses) > 5000:
                path.poses = path.poses[-5000:]
            self._path_publishers[robot].publish(path)

    def _handoff_callback(self, message):
        if not bool(message.accepted) or str(message.status) != 'ACCEPTED':
            return
        if self._handoff_accepted:
            return
        self._accept_handoff('accepted_handoff')

    def _accept_handoff(self, reason):
        """Switch visualization into the common frame exactly once."""
        if self._handoff_accepted:
            return
        self._handoff_accepted = True
        # The pre-handoff paths are expressed in deliberately separated map
        # aliases.  Start a fresh shared-frame path at the handoff boundary so
        # RViz never draws a misleading 25 m cross-frame segment.
        self._paths = {robot: Path() for robot in ('robot1', 'robot2')}
        self._last_path_xy = {robot: None for robot in ('robot1', 'robot2')}
        # The hypothesis header carries the evidence/map timestamp, which can
        # precede the actual acceptance by many seconds.  The visualization
        # indicator must show when handoff was accepted, so use this node's
        # synchronized ROS clock at receipt rather than the evidence stamp.
        now = self.get_clock().now()
        self._handoff_time = float(now.nanoseconds) * 1e-9
        # Clear the latched pre-handoff path immediately. Otherwise RViz can
        # retain that old, intentionally offset path until the next motion
        # sample arrives.
        stamp = now.to_msg()
        for robot in ('robot1', 'robot2'):
            empty = Path()
            empty.header.frame_id = self.viz_world
            empty.header.stamp = stamp
            self._path_publishers[robot].publish(empty)
        self._publish_handoff_marker()
        self.get_logger().info(
            'VISUALIZATION_HANDOFF_ACCEPTED sim_time=%.3f reason=%s' %
            (self._handoff_time, reason))

    def _publish_handoff_marker(self):
        marker = Marker()
        marker.header.frame_id = self.viz_world
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'handoff'
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = -1.0
        marker.pose.position.z = 0.5
        marker.scale.z = 0.22
        marker.color.a = 1.0
        marker.color.r = 0.1 if self._handoff_accepted else 1.0
        marker.color.g = 1.0 if self._handoff_accepted else 0.2
        marker.text = ('HANDOFF: ACCEPTED t=%.2fs' % self._handoff_time
                       if self._handoff_accepted else 'HANDOFF: PENDING')
        self._handoff_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = RvizMapOverlay()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        # Normal executor termination after a process-group shutdown.
        pass
    finally:
        node.destroy_node()
        # A SIGTERM can cause rclpy's signal handler to shut the context down
        # before spin() unwinds.  The visualization bridge must not turn that
        # normal shutdown race into a process error.
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except RuntimeError:
                pass
