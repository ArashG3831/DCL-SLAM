import math
import time

from my_epuck_interfaces.msg import PeerMap
from my_epuck_project.live_map_sanitizer import (
    apply_incremental_patch,
    footprint_cell_indices,
)
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
        self.remote_content_revision = 0
        self.map_revision = 0
        self.live_pose_cache = {}
        self.base_grid = None
        self.base_key = None
        self.output_grid = None
        self.last_footprint_cells = set()
        self.last_pose_key = None
        self.profile_window = {
            'invocations': 0, 'full_rebuilds': 0, 'pose_updates': 0,
            'cells_inspected': 0, 'cells_copied': 0, 'cells_modified': 0,
            'publications': 0, 'wall_s': 0.0, 'cpu_s': 0.0,
            'last_log_wall': time.monotonic(),
        }
        self.retry_timer = self.create_timer(0.5, self.try_fuse)
        self.get_logger().info(
            f'Local {self.resolve_topic_name(local_topic)} + remote-only '
            f'{self.resolve_topic_name(remote_topic)} from {self.expected_source} '
            f'-> {self.resolve_topic_name(output_topic)}'
        )

    def local_callback(self, message):
        changed = self.local_map is None or (
            self.local_map.header.stamp != message.header.stamp or
            self.local_map.info.width != message.info.width or
            self.local_map.info.height != message.info.height or
            self.local_map.info.resolution != message.info.resolution or
            list(self.local_map.data) != list(message.data))
        self.local_map = message
        if changed:
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
        changed = self.remote_map is None or (
            self.remote_map.header.stamp != message.occupancy_grid.header.stamp or
            self.remote_map.info.width != message.occupancy_grid.info.width or
            self.remote_map.info.height != message.occupancy_grid.info.height or
            self.remote_map.info.resolution != message.occupancy_grid.info.resolution or
            list(self.remote_map.data) != list(message.occupancy_grid.data))
        self.last_remote_revision = message.revision
        self.remote_map = message.occupancy_grid
        if changed:
            self.remote_content_revision += 1
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

    def _make_base_grid(self, messages, transforms, minimum_x, minimum_y,
                        width, height):
        """Build the unsanitized fused base once per map revision."""
        fused_data = [-1] * (width * height)
        inspected = 0
        for message, transform in zip(messages, transforms):
            source_resolution = message.info.resolution
            inspected += len(message.data)
            for index, value in enumerate(message.data):
                if value < 0:
                    continue
                column = index % message.info.width
                row = index // message.info.width
                x, y = self.point_in_output(
                    message, transform,
                    (column + 0.5) * source_resolution,
                    (row + 0.5) * source_resolution,
                )
                output_column = int(math.floor(
                    (x - minimum_x) / self.resolution))
                output_row = int(math.floor(
                    (y - minimum_y) / self.resolution))
                if 0 <= output_column < width and 0 <= output_row < height:
                    target = output_row * width + output_column
                    current = fused_data[target]
                    fused_data[target] = (
                        value if current < 0 else max(current, value))
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
        return fused, inspected

    def _fresh_footprint_cells(self, grid, footprints):
        cells = set()
        for footprint in footprints:
            age = footprint.get('pose_age_s')
            if age is None or age < 0.0 or age > self.live_pose_max_age_s:
                continue
            cells.update(footprint_cell_indices(
                grid, footprint,
                uncertainty_cells=self.live_footprint_uncertainty_cells))
        return cells

    def _apply_pose_cells(self, cells):
        """Restore old patches and clear new patches without map-sized copies."""
        if self.base_grid is None or self.output_grid is None:
            return 0
        modified = apply_incremental_patch(
            self.base_grid.data, self.output_grid.data,
            self.last_footprint_cells, cells)
        self.last_footprint_cells = set(cells)
        return modified

    def _profile(self, *, mode, map_changed, dimensions, cells_inspected,
                 cells_copied, cells_modified, published, wall_s, cpu_s):
        stats = self.profile_window
        stats['invocations'] += 1
        stats['full_rebuilds'] += int(mode == 'FULL_REBUILD')
        stats['pose_updates'] += int(mode == 'POSE_PATCH')
        stats['cells_inspected'] += cells_inspected
        stats['cells_copied'] += cells_copied
        stats['cells_modified'] += cells_modified
        stats['publications'] += int(published)
        stats['wall_s'] += wall_s
        stats['cpu_s'] += cpu_s
        now = time.monotonic()
        if now - stats['last_log_wall'] < 1.0:
            return
        elapsed = max(1e-9, now - stats['last_log_wall'])
        self.get_logger().info(
            'FUSION_PROFILE ' + ' '.join((
                f'mode={mode}', f'map_changed={map_changed}',
                f'dimensions={dimensions[0]}x{dimensions[1]}',
                f'invocations={stats["invocations"]}',
                f'full_rebuilds={stats["full_rebuilds"]}',
                f'pose_updates={stats["pose_updates"]}',
                f'cells_inspected={stats["cells_inspected"]}',
                f'cells_copied={stats["cells_copied"]}',
                f'cells_modified={stats["cells_modified"]}',
                f'publications={stats["publications"]}',
                f'wall_duration_s={stats["wall_s"]:.6f}',
                f'cpu_duration_s={stats["cpu_s"]:.6f}',
                f'window_wall_s={elapsed:.3f}',
            )))
        stats.update({
            'invocations': 0, 'full_rebuilds': 0, 'pose_updates': 0,
            'cells_inspected': 0, 'cells_copied': 0, 'cells_modified': 0,
            'publications': 0, 'wall_s': 0.0, 'cpu_s': 0.0,
            'last_log_wall': now,
        })

    def try_fuse(self):
        started_wall = time.perf_counter()
        started_cpu = time.process_time()
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
        pose_key = None
        pose_telemetry = {}
        if self.sanitize_live_footprints:
            footprints, pose_key, pose_telemetry = self.live_footprints()
        if not self.sanitize_live_footprints:
            fused, _ = self._make_base_grid(
                messages, transforms, minimum_x, minimum_y, width, height)
            if rclpy.ok():
                self.map_publisher.publish(fused)
                self.metadata_publisher.publish(fused.info)
            return
        base_key = (
            self.local_revision, self.remote_content_revision,
            tuple(round(value, 4) for transform in transforms
                  for value in transform),
            minimum_x, minimum_y, width, height,
        )
        map_changed = base_key != self.base_key
        if map_changed:
            base, inspected = self._make_base_grid(
                messages, transforms, minimum_x, minimum_y, width, height)
            self.base_grid = base
            self.base_key = base_key
            self.output_grid = OccupancyGrid()
            self.output_grid.header = base.header
            self.output_grid.info = base.info
            self.output_grid.data = list(base.data)
            self.last_footprint_cells = set()
            self.map_revision += 1
            cells = self._fresh_footprint_cells(base, footprints)
            modified = self._apply_pose_cells(cells)
            mode = 'FULL_REBUILD'
            copied = len(base.data)
            published = True
        else:
            if pose_key == self.last_pose_key:
                self._profile(
                    mode='NOOP', map_changed=False,
                    dimensions=(width, height), cells_inspected=0,
                    cells_copied=0, cells_modified=0, published=False,
                    wall_s=time.perf_counter() - started_wall,
                    cpu_s=time.process_time() - started_cpu)
                return
            inspected = 0
            cells = self._fresh_footprint_cells(self.base_grid, footprints)
            modified = self._apply_pose_cells(cells)
            mode = 'POSE_PATCH'
            copied = 0
            published = modified > 0
        self.last_pose_key = pose_key
        if published and rclpy.ok():
            self.map_publisher.publish(self.output_grid)
            self.metadata_publisher.publish(self.output_grid.info)
        self._profile(
            mode=mode, map_changed=map_changed,
            dimensions=(width, height), cells_inspected=inspected,
            cells_copied=copied, cells_modified=modified,
            published=published, wall_s=time.perf_counter() - started_wall,
            cpu_s=time.process_time() - started_cpu)
        if map_changed:
            stale_count = sum(
                1 for item in footprints
                if item.get('pose_age_s') is None
                or item.get('pose_age_s') > self.live_pose_max_age_s)
            own_fresh = sum(
                1 for item in footprints
                if item.get('role') == 'own'
                and item.get('pose_age_s') is not None
                and item.get('pose_age_s') <= self.live_pose_max_age_s)
            peer_fresh = sum(
                1 for item in footprints
                if item.get('role') == 'peer'
                and item.get('pose_age_s') is not None
                and item.get('pose_age_s') <= self.live_pose_max_age_s)
            self.get_logger().info(
                'MAP_SANITIZE ' + ' '.join((
                    f'revision={self.map_revision}',
                    f'cleared_cell_count={modified}',
                    f'stale_pose_count={stale_count}',
                    f'own_pose_age_s={pose_telemetry["own_pose_age_s"]}',
                    f'peer_pose_age_s={pose_telemetry["peer_pose_age_s"]}',
                    f'own_cells_cleared={own_fresh}',
                    f'peer_cells_cleared={peer_fresh}',
                    f'own_clear_skipped_reason={pose_telemetry["own_clear_skipped_reason"]}',
                    f'peer_clear_skipped_reason={pose_telemetry["peer_clear_skipped_reason"]}',
                    f'footprint_radius_m={self.live_footprint_radius}',
                    f'uncertainty_cells={self.live_footprint_uncertainty_cells}',
                )))


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
