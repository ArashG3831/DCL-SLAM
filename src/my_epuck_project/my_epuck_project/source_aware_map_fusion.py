import math

from my_epuck_interfaces.msg import PeerMap
from my_epuck_project.live_map_sanitizer import sanitize_shared_map
from nav_msgs.msg import MapMetaData, OccupancyGrid
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def message_key(messages, local_revision, remote_revision):
    return tuple(
        (
            message.header.frame_id,
            message.header.stamp.sec,
            message.header.stamp.nanosec,
            message.info.width,
            message.info.height,
            message.info.resolution,
        )
        for message in messages
    ) + (local_revision, remote_revision)


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
        self.declare_parameter(
            'live_robot_frames',
            ['robot1/base_footprint', 'robot2/base_footprint'])
        self.declare_parameter('live_footprint_radius_m', 0.037)
        self.declare_parameter('live_footprint_uncertainty_cells', 1)
        self.declare_parameter('live_pose_max_age_s', 0.5)
        self.declare_parameter('publish_on_callback', True)
        self.declare_parameter('sanitize_live_footprints', False)

        local_topic = self.get_parameter('local_map_topic').value
        remote_topic = self.get_parameter('remote_peer_topic').value
        self.expected_source = self.get_parameter('expected_remote_source').value
        output_topic = self.get_parameter('output_topic').value
        metadata_topic = self.get_parameter('metadata_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.resolution = float(self.get_parameter('resolution').value)
        self.live_robot_frames = list(
            self.get_parameter('live_robot_frames').value)
        namespace = self.get_namespace().strip('/')
        self.own_robot_id = namespace or self.live_robot_frames[0].split('/')[0]
        self.peer_robot_id = next(
            (frame.split('/')[0] for frame in self.live_robot_frames
             if frame.split('/')[0] != self.own_robot_id),
            None)
        self.live_footprint_radius = float(
            self.get_parameter('live_footprint_radius_m').value)
        self.live_footprint_uncertainty_cells = int(
            self.get_parameter('live_footprint_uncertainty_cells').value)
        self.live_pose_max_age_s = float(
            self.get_parameter('live_pose_max_age_s').value)
        self.publish_on_callback = bool(
            self.get_parameter('publish_on_callback').value)
        self.sanitize_live_footprints = bool(
            self.get_parameter('sanitize_live_footprints').value)
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
        self.local_revision = 0
        self.last_remote_revision = 0
        self.map_revision = 0
        self.live_pose_cache = {}
        self.last_sanitized_key = None
        self.retry_timer = self.create_timer(0.5, self.try_fuse)
        self.get_logger().info(
            f'Local {self.resolve_topic_name(local_topic)} + remote-only '
            f'{self.resolve_topic_name(remote_topic)} from {self.expected_source} '
            f'-> {self.resolve_topic_name(output_topic)}'
        )

    def local_callback(self, message):
        self.local_map = message
        self.local_revision += 1
        if self.publish_on_callback:
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
        if self.publish_on_callback:
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

    def live_footprints(self):
        """Return independent own/peer footprints and freshness telemetry.

        Own clearing is deliberately independent of peer TF availability. A
        stale peer suppresses only that peer's footprint; it must never make
        the local robot's own live footprint stale as a side effect.
        """
        footprints = []
        key = []
        for frame in self.live_robot_frames:
            role = 'own' if frame.split('/')[0] == self.own_robot_id else 'peer'
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.output_frame, frame, Time(),
                    timeout=Duration(seconds=0.05))
                stamp = Time.from_msg(transform.header.stamp)
                now = self.get_clock().now()
                age = max(0.0, (now - stamp).nanoseconds / 1e9)
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                heading = self.yaw(transform.transform.rotation)
                footprints.append({
                    'robot_frame': frame, 'x': x, 'y': y,
                    'radius_m': self.live_footprint_radius,
                    'role': role,
                    'pose_age_s': age,
                })
                self.live_pose_cache[frame] = (x, y, stamp)
                key.append((frame, True, round(x, 3), round(y, 3),
                            round(heading, 3), age <= self.live_pose_max_age_s))
            except TransformException as error:
                cached = self.live_pose_cache.get(frame)
                now = self.get_clock().now()
                cached_age = None
                if cached is not None:
                    cached_age = max(
                        0.0, (now - cached[2]).nanoseconds / 1e9)
                if cached is not None and cached_age <= self.live_pose_max_age_s:
                    footprints.append({
                        'robot_frame': frame, 'x': cached[0], 'y': cached[1],
                        'radius_m': self.live_footprint_radius,
                        'role': role,
                        'pose_age_s': cached_age,
                    })
                    key.append((frame, True, round(cached[0], 3),
                                round(cached[1], 3), round(cached_age, 2),
                                True))
                else:
                    self.get_logger().warning(
                        f'Live footprint unavailable for {frame}: {error}',
                        throttle_duration_sec=5.0)
                    key.append((frame, False))
        telemetry = {}
        for role in ('own', 'peer'):
            matches = [item for item in footprints if item['role'] == role]
            if matches:
                telemetry[f'{role}_pose_age_s'] = min(
                    item['pose_age_s'] for item in matches)
                telemetry[f'{role}_pose_available'] = True
                telemetry[f'{role}_clear_skipped_reason'] = (
                    'POSE_STALE' if telemetry[f'{role}_pose_age_s']
                    > self.live_pose_max_age_s else 'NONE')
            else:
                telemetry[f'{role}_pose_age_s'] = None
                telemetry[f'{role}_pose_available'] = False
                telemetry[f'{role}_clear_skipped_reason'] = 'TF_UNAVAILABLE'
        return footprints, tuple(key), telemetry

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

        footprints = []
        if self.sanitize_live_footprints:
            footprints, pose_key, pose_telemetry = self.live_footprints()
            fusion_key = (
                message_key(messages, self.local_revision,
                            self.last_remote_revision),
                tuple(round(value, 4) for transform in transforms
                      for value in transform),
                pose_key,
            )
            if fusion_key == self.last_sanitized_key:
                return
            self.last_sanitized_key = fusion_key
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
        if not self.sanitize_live_footprints:
            if rclpy.ok():
                self.map_publisher.publish(fused)
                self.metadata_publisher.publish(fused.info)
            return
        fused, sanitization = sanitize_shared_map(
            fused, footprints,
            uncertainty_cells=self.live_footprint_uncertainty_cells,
            max_pose_age_s=self.live_pose_max_age_s)
        self.map_revision += 1
        self.get_logger().info(
            'MAP_SANITIZE ' + ' '.join((
                f'revision={self.map_revision}',
                f'cleared_cell_count={sanitization["cleared_cell_count"]}',
                f'stale_pose_count={sanitization["stale_pose_count"]}',
                f'own_pose_age_s={pose_telemetry["own_pose_age_s"]}',
                f'peer_pose_age_s={pose_telemetry["peer_pose_age_s"]}',
                f'own_cells_cleared={sanitization["cleared_by_role"]["own"]}',
                f'peer_cells_cleared={sanitization["cleared_by_role"]["peer"]}',
                f'own_clear_skipped_reason={pose_telemetry["own_clear_skipped_reason"]}',
                f'peer_clear_skipped_reason={pose_telemetry["peer_clear_skipped_reason"]}',
                f'footprint_radius_m={self.live_footprint_radius}',
                f'uncertainty_cells={self.live_footprint_uncertainty_cells}',
            )))
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
