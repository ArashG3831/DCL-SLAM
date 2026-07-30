import math

import rclpy
from my_epuck_interfaces.msg import PeerMap
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class SourceAwareMapFusion(Node):
    """Fuse own local SLAM evidence with one validated remote peer map."""

    def __init__(self):
        super().__init__('map_fusion')
        self.declare_parameter('local_map_topic', 'map')
        self.declare_parameter('remote_peer_topic', '/cslam/remote/local_map')
        self.declare_parameter('expected_remote_source', '')
        self.declare_parameter('output_topic', 'shared_map')
        self.declare_parameter('metadata_topic', 'shared_map_metadata')
        self.declare_parameter('output_frame', 'shared_map')
        self.declare_parameter('resolution', 0.01)

        local_topic = self.get_parameter('local_map_topic').value
        remote_topic = self.get_parameter('remote_peer_topic').value
        self.expected_source = self.get_parameter('expected_remote_source').value
        output_topic = self.get_parameter('output_topic').value
        metadata_topic = self.get_parameter('metadata_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.resolution = float(self.get_parameter('resolution').value)
        if not self.expected_source:
            raise ValueError('expected_remote_source must not be empty')
        if self.resolution <= 0.0:
            raise ValueError('resolution must be positive')

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_publisher = self.create_publisher(OccupancyGrid, output_topic, qos)
        self.metadata_publisher = self.create_publisher(MapMetaData, metadata_topic, qos)
        self.local_subscription = self.create_subscription(
            OccupancyGrid, local_topic, self.local_callback, qos
        )
        self.peer_subscription = self.create_subscription(
            PeerMap, remote_topic, self.peer_callback, qos
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.local_map = None
        self.remote_map = None
        self.last_remote_revision = 0
        self.retry_timer = self.create_timer(0.5, self.try_fuse)
        self.get_logger().info(
            f'Local {self.resolve_topic_name(local_topic)} + remote-only '
            f'{self.resolve_topic_name(remote_topic)} from {self.expected_source} '
            f'-> {self.resolve_topic_name(output_topic)}'
        )

    def local_callback(self, message):
        self.local_map = message
        self.try_fuse()

    def reject(self, reason):
        self.get_logger().warning(reason)

    def peer_callback(self, message):
        if message.source_robot_id != self.expected_source:
            self.reject(
                f'Rejected peer source {message.source_robot_id!r}; '
                f'expected {self.expected_source!r}'
            )
            return
        if not message.local_evidence_only:
            self.reject(
                f'Rejected revision {message.revision}: remote map is not '
                'marked local_evidence_only'
            )
            return
        if message.revision <= self.last_remote_revision:
            self.reject(
                f'Ignored stale/duplicate revision {message.revision}; '
                f'last accepted is {self.last_remote_revision}'
            )
            return
        self.last_remote_revision = message.revision
        self.remote_map = message.occupancy_grid
        self.try_fuse()

    @staticmethod
    def yaw(quaternion):
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
        )

    @staticmethod
    def transform_point(x, y, transform):
        tx, ty, heading = transform
        cosine, sine = math.cos(heading), math.sin(heading)
        return tx + cosine*x - sine*y, ty + sine*x + cosine*y

    def map_transform(self, message):
        transform = self.tf_buffer.lookup_transform(
            self.output_frame, message.header.frame_id, Time(),
            timeout=Duration(seconds=0.05)
        )
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            self.yaw(transform.transform.rotation),
        )

    def point_in_output(self, message, transform, x, y):
        heading = self.yaw(message.info.origin.orientation)
        cosine, sine = math.cos(heading), math.sin(heading)
        map_x = message.info.origin.position.x + cosine*x - sine*y
        map_y = message.info.origin.position.y + sine*x + cosine*y
        return self.transform_point(map_x, map_y, transform)

    def corners(self, message, transform):
        width = message.info.width * message.info.resolution
        height = message.info.height * message.info.resolution
        return [
            self.point_in_output(message, transform, x, y)
            for x, y in ((0.0, 0.0), (width, 0.0), (0.0, height), (width, height))
        ]

    def try_fuse(self):
        if self.local_map is None or self.remote_map is None:
            return
        messages = [self.local_map, self.remote_map]
        try:
            transforms = [self.map_transform(message) for message in messages]
        except TransformException as error:
            self.get_logger().warning(
                f'Waiting for local-map transforms: {error}',
                throttle_duration_sec=5.0,
            )
            return

        corners = [
            point for message, transform in zip(messages, transforms)
            for point in self.corners(message, transform)
        ]
        minimum_x = math.floor(min(p[0] for p in corners)/self.resolution)*self.resolution
        minimum_y = math.floor(min(p[1] for p in corners)/self.resolution)*self.resolution
        maximum_x = math.ceil(max(p[0] for p in corners)/self.resolution)*self.resolution
        maximum_y = math.ceil(max(p[1] for p in corners)/self.resolution)*self.resolution
        width = max(1, int(round((maximum_x-minimum_x)/self.resolution)))
        height = max(1, int(round((maximum_y-minimum_y)/self.resolution)))
        fused_data = [-1] * (width*height)

        for message, transform in zip(messages, transforms):
            source_resolution = message.info.resolution
            for index, value in enumerate(message.data):
                if value < 0:
                    continue
                column = index % message.info.width
                row = index // message.info.width
                x, y = self.point_in_output(
                    message, transform,
                    (column+0.5)*source_resolution,
                    (row+0.5)*source_resolution,
                )
                output_column = int(math.floor((x-minimum_x)/self.resolution))
                output_row = int(math.floor((y-minimum_y)/self.resolution))
                if 0 <= output_column < width and 0 <= output_row < height:
                    target = output_row*width + output_column
                    current = fused_data[target]
                    fused_data[target] = value if current < 0 else max(current, value)

        stamp = max(
            (message.header.stamp for message in messages),
            key=lambda value: (value.sec, value.nanosec),
        )
        fused = OccupancyGrid()
        fused.header.stamp = stamp
        fused.header.frame_id = self.output_frame
        fused.info.map_load_time = stamp
        fused.info.resolution = self.resolution
        fused.info.width = width
        fused.info.height = height
        fused.info.origin.position.x = minimum_x
        fused.info.origin.position.y = minimum_y
        fused.info.origin.orientation.w = 1.0
        fused.data = fused_data
        if rclpy.ok():
            self.map_publisher.publish(fused)
            self.metadata_publisher.publish(fused.info)


def main(args=None):
    rclpy.init(args=args)
    node = SourceAwareMapFusion()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.context.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
