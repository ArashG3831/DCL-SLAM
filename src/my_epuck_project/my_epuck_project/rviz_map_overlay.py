"""Visualization-only aliases for independent robot occupancy maps.

This node deliberately creates no relationship between the real robot map
frames.  It copies maps onto separate ``viz/*`` topics and publishes only
``viz/*`` static transforms so RViz can show independent maps in one window.
The bridge is optional and must not be used by estimation or control nodes.
"""

from copy import deepcopy
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


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
    """Relay two local maps into independent, explicitly fake viz frames."""

    def __init__(self):
        super().__init__('rviz_map_overlay')
        self.declare_parameter('viz_world_frame', VIZ_WORLD)
        self.declare_parameter('robot1_map_topic', '/robot1/map')
        self.declare_parameter('robot2_map_topic', '/robot2/map')
        self.declare_parameter('robot1_output_topic', '/viz/robot1_map')
        self.declare_parameter('robot2_output_topic', '/viz/robot2_map')
        self.declare_parameter('robot1_alias_frame', 'viz/robot1_map')
        self.declare_parameter('robot2_alias_frame', 'viz/robot2_map')
        for robot, defaults in (
                ('robot1', (0.0, 0.0, 0.0, 0.0)),
                ('robot2', (25.0, 0.0, 0.0, 0.0))):
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
            publisher = self.create_publisher(OccupancyGrid, output_topic, qos)
            self._map_publishers[robot] = publisher
            self.create_subscription(
                OccupancyGrid, source_topic,
                lambda message, name=robot: self._relay(name, message), qos)

        self._tf = StaticTransformBroadcaster(self)
        transforms = []
        for robot in ('robot1', 'robot2'):
            transforms.append(alias_transform(
                self._aliases[robot],
                float(self.get_parameter(f'{robot}_offset_x_m').value),
                float(self.get_parameter(f'{robot}_offset_y_m').value),
                float(self.get_parameter(f'{robot}_offset_z_m').value),
                float(self.get_parameter(f'{robot}_offset_yaw_rad').value)))
        for transform in transforms:
            transform.header.frame_id = self.viz_world
        self._tf.sendTransform(transforms)
        self.get_logger().info(
            'VISUALIZATION_ONLY_OVERLAY active; real map/TF frames are untouched')

    def _relay(self, robot: str, message: OccupancyGrid):
        self._map_publishers[robot].publish(alias_map(
            message, self._aliases[robot]))


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
