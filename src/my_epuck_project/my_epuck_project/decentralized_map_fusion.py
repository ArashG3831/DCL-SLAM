import math

import rclpy
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class DecentralizedMapFusion(Node):
    """Fuse original local occupancy grids into one known-pose shared grid."""

    def __init__(self):
        super().__init__('map_fusion')

        self.declare_parameter('input_topics', ['/robot1/map', '/robot2/map'])
        self.declare_parameter('output_topic', 'shared_map')
        self.declare_parameter('metadata_topic', 'shared_map_metadata')
        self.declare_parameter('output_frame', 'shared_map')
        self.declare_parameter('resolution', 0.01)

        self.input_topics = list(self.get_parameter('input_topics').value)
        self.output_topic = self.get_parameter('output_topic').value
        self.metadata_topic = self.get_parameter('metadata_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.resolution = float(self.get_parameter('resolution').value)

        if len(self.input_topics) != 2:
            raise ValueError('input_topics must contain exactly two local map topics')
        if self.resolution <= 0.0:
            raise ValueError('resolution must be positive')

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_publisher = self.create_publisher(
            OccupancyGrid, self.output_topic, map_qos
        )
        self.metadata_publisher = self.create_publisher(
            MapMetaData, self.metadata_topic, map_qos
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.maps = {}
        self.map_subscriptions = [
            self.create_subscription(
                OccupancyGrid,
                topic,
                lambda message, source_topic=topic: self.map_callback(
                    source_topic, message
                ),
                map_qos,
            )
            for topic in self.input_topics
        ]
        self.retry_timer = self.create_timer(0.5, self.try_fuse)

        self.get_logger().info(
            f'Fusing only {self.input_topics} -> '
            f'{self.resolve_topic_name(self.output_topic)} '
            f'in frame {self.output_frame} at {self.resolution:.3f} m'
        )

    def map_callback(self, source_topic, message):
        self.maps[source_topic] = message
        self.try_fuse()

    @staticmethod
    def yaw_from_quaternion(quaternion):
        return math.atan2(
            2.0 * (
                quaternion.w * quaternion.z
                + quaternion.x * quaternion.y
            ),
            1.0 - 2.0 * (
                quaternion.y * quaternion.y
                + quaternion.z * quaternion.z
            ),
        )

    @staticmethod
    def transform_point(x, y, transform):
        translation_x, translation_y, heading = transform
        cosine = math.cos(heading)
        sine = math.sin(heading)
        return (
            translation_x + cosine * x - sine * y,
            translation_y + sine * x + cosine * y,
        )

    def map_transform(self, message):
        transform = self.tf_buffer.lookup_transform(
            self.output_frame,
            message.header.frame_id,
            Time(),
            timeout=Duration(seconds=0.05),
        )
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            self.yaw_from_quaternion(transform.transform.rotation),
        )

    def grid_point_in_output(self, message, map_transform, x, y):
        origin_heading = self.yaw_from_quaternion(
            message.info.origin.orientation
        )
        origin_cosine = math.cos(origin_heading)
        origin_sine = math.sin(origin_heading)
        map_x = (
            message.info.origin.position.x
            + origin_cosine * x
            - origin_sine * y
        )
        map_y = (
            message.info.origin.position.y
            + origin_sine * x
            + origin_cosine * y
        )
        return self.transform_point(map_x, map_y, map_transform)

    def transformed_bounds(self, message, map_transform):
        width = message.info.width * message.info.resolution
        height = message.info.height * message.info.resolution
        return [
            self.grid_point_in_output(message, map_transform, x, y)
            for x, y in (
                (0.0, 0.0),
                (width, 0.0),
                (0.0, height),
                (width, height),
            )
        ]

    @staticmethod
    def latest_stamp(messages):
        return max(
            (message.header.stamp for message in messages),
            key=lambda stamp: (stamp.sec, stamp.nanosec),
        )

    def try_fuse(self):
        if any(topic not in self.maps for topic in self.input_topics):
            return

        messages = [self.maps[topic] for topic in self.input_topics]
        try:
            transforms = [
                self.map_transform(message) for message in messages
            ]
        except TransformException as error:
            self.get_logger().warning(
                f'Waiting for local-map transforms: {error}',
                throttle_duration_sec=5.0,
            )
            return

        corners = [
            point
            for message, transform in zip(messages, transforms)
            for point in self.transformed_bounds(message, transform)
        ]
        minimum_x = (
            math.floor(min(point[0] for point in corners) / self.resolution)
            * self.resolution
        )
        minimum_y = (
            math.floor(min(point[1] for point in corners) / self.resolution)
            * self.resolution
        )
        maximum_x = (
            math.ceil(max(point[0] for point in corners) / self.resolution)
            * self.resolution
        )
        maximum_y = (
            math.ceil(max(point[1] for point in corners) / self.resolution)
            * self.resolution
        )
        width = max(1, int(round((maximum_x - minimum_x) / self.resolution)))
        height = max(1, int(round((maximum_y - minimum_y) / self.resolution)))
        fused_data = [-1] * (width * height)

        for message, transform in zip(messages, transforms):
            source_resolution = message.info.resolution
            for index, value in enumerate(message.data):
                if value < 0:
                    continue
                column = index % message.info.width
                row = index // message.info.width
                output_x, output_y = self.grid_point_in_output(
                    message,
                    transform,
                    (column + 0.5) * source_resolution,
                    (row + 0.5) * source_resolution,
                )
                output_column = int(
                    math.floor((output_x - minimum_x) / self.resolution)
                )
                output_row = int(
                    math.floor((output_y - minimum_y) / self.resolution)
                )
                if not (
                    0 <= output_column < width
                    and 0 <= output_row < height
                ):
                    continue
                output_index = output_row * width + output_column
                current = fused_data[output_index]
                fused_data[output_index] = (
                    value if current < 0 else max(current, value)
                )

        stamp = self.latest_stamp(messages)
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

        self.map_publisher.publish(fused)
        self.metadata_publisher.publish(fused.info)


def main(args=None):
    rclpy.init(args=args)
    node = DecentralizedMapFusion()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
