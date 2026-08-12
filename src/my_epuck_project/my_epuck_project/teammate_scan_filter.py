"""Remove observed teammate lidar returns from the simulation SLAM branch."""

from collections import deque
import copy
from dataclasses import dataclass
import math
import time

from my_epuck_project.slam_range_policy import (
    complete_natural_no_returns,
    FREE_SPACE_CAP,
)

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import LaserScan

from tf2_ros import Buffer, TransformException, TransformListener


# Measured from the Webots E-puck v2/Pi-puck/D500 model used by this package.
# E-puck.proto body bounding cylinder: radius 0.037 m (1145-1155), and
# Pi-puck.proto board surfaces: radius 0.035 m (102-143), but both terminate
# below the D500 horizontal ray.  The world places the D500 housing at z=.055
# with radius .025 m (large world 253-273); this is the measured scan-plane
# silhouette.  One mm covers the mesh/grid boundary.  The historical
# peer_radius_m remains unchanged; this is the selected verified model.
VERIFIED_SILHOUETTE_RADIUS_M = 0.026
# Exact-time odometry still leaves a bounded scan-endpoint discrepancy while a
# teammate is moving.  Live runs measured endpoint radii up to 0.0597 m about
# the peer centre.  This 60 mm envelope is therefore an observation-exclusion
# envelope, not a physical robot-radius estimate.
VERIFIED_EXCLUSION_RADIUS_M = 0.060
VERIFIED_GEOMETRY_MODEL = 'epuck_v2_pi_puck_d500_conservative_circle'


@dataclass(frozen=True)
class Transform2D:
    """Planar rigid transform."""

    x: float
    y: float
    yaw: float


def normalize_angle(angle):
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def compose_transform(first, second):
    """Compose T_a_b and T_b_c to produce T_a_c."""
    cosine, sine = math.cos(first.yaw), math.sin(first.yaw)
    return Transform2D(
        first.x + cosine * second.x - sine * second.y,
        first.y + sine * second.x + cosine * second.y,
        normalize_angle(first.yaw + second.yaw),
    )


def transform_message_2d(transform):
    """Project a TransformStamped onto the horizontal plane."""
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    yaw = math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )
    return Transform2D(translation.x, translation.y, yaw)


def circle_first_intersection(center_x, center_y, radius, beam_angle):
    """Return the nearest nonnegative ray-circle intersection, if any."""
    if radius <= 0.0:
        raise ValueError('radius must be positive')
    direction_x, direction_y = math.cos(beam_angle), math.sin(beam_angle)
    projection = center_x * direction_x + center_y * direction_y
    discriminant = projection * projection - (
        center_x * center_x + center_y * center_y - radius * radius)
    if discriminant < -1e-12:
        return None
    half_chord = math.sqrt(max(0.0, discriminant))
    intersections = (
        distance for distance in (projection - half_chord,
                                  projection + half_chord)
        if distance >= -1e-12
    )
    return min((max(0.0, distance) for distance in intersections),
               default=None)


def mask_teammate_returns(scan, peer_x, peer_y, peer_radius_m,
                          range_tolerance_m, geometry_radius_m=None):
    """Replace finite returns inside the conservative teammate envelope."""
    if range_tolerance_m < 0.0:
        raise ValueError('range_tolerance_m must not be negative')
    geometry_radius = (peer_radius_m if geometry_radius_m is None
                       else float(geometry_radius_m))
    if geometry_radius <= 0.0:
        raise ValueError('geometry_radius_m must be positive')
    output = copy.copy(scan)
    output.ranges = list(scan.ranges)
    masked_indices = []
    for index, measured in enumerate(scan.ranges):
        if not (math.isfinite(measured)
                and measured > scan.range_min
                and measured < scan.range_max):
            continue
        beam_angle = scan.angle_min + index * scan.angle_increment
        expected_near = circle_first_intersection(
            peer_x, peer_y, geometry_radius, beam_angle)
        endpoint_x = measured * math.cos(beam_angle)
        endpoint_y = measured * math.sin(beam_angle)
        endpoint_in_envelope = math.hypot(
            endpoint_x - peer_x, endpoint_y - peer_y) <= peer_radius_m
        if (expected_near is not None
                and (abs(measured - expected_near) <= range_tolerance_m
                     or endpoint_in_envelope)):
            output.ranges[index] = math.nan
            masked_indices.append(index)
    return output, masked_indices


def prepare_slam_scan(scan, peer_x, peer_y, peer_radius_m,
                      range_tolerance_m, free_space_cap,
                      geometry_radius_m=None):
    """Mask finite teammate hits, then complete simulation no-returns."""
    masked_scan, masked_indices = mask_teammate_returns(
        scan, peer_x, peer_y, peer_radius_m, range_tolerance_m,
        geometry_radius_m=geometry_radius_m)
    completed_ranges, completion_stats = complete_natural_no_returns(
        masked_scan.ranges,
        free_space_cap,
        masked_scan.range_min,
        masked_scan.range_max,
        enabled=True,
    )
    output = copy.copy(masked_scan)
    output.ranges = completed_ranges
    return output, masked_indices, completion_stats


class PendingScanQueue:
    """Small FIFO used while exact-time odometry transforms arrive."""

    MAXIMUM_DEPTH = 4

    def __init__(self, maximum_depth=MAXIMUM_DEPTH):
        """Create a FIFO with an enforced four-scan upper bound."""
        if not 1 <= maximum_depth <= self.MAXIMUM_DEPTH:
            raise ValueError('pending queue depth must be between 1 and 4')
        self.maximum_depth = maximum_depth
        self.items = deque()

    def enqueue(self, scan, arrival_time):
        """Append a scan and return the oldest scan if the FIFO overflows."""
        self.items.append((scan, arrival_time))
        if len(self.items) > self.maximum_depth:
            return self.items.popleft()
        return None

    def take(self, now, maximum_latency, pose_available):
        """Wait, expire, or remove the oldest item for filtered publication."""
        if not self.items:
            return 'idle', None, 0.0
        item = self.items[0]
        waited = now - item[1]
        if waited > maximum_latency:
            return 'expired', self.items.popleft(), waited
        if not pose_available:
            return 'wait', item, waited
        return 'publish', self.items.popleft(), waited


class TeammateScanFilter(Node):
    """Publish a teammate-masked, simulation-completed scan only for SLAM."""

    def __init__(self):
        """Configure the simulation-only filter and exact-time TF queue."""
        super().__init__('teammate_scan_filter')
        defaults = {
            'input_topic': 'scan_d500_fixed', 'output_topic': 'scan_d500_slam',
            'peer_base_frame': '', 'expected_lidar_frame': '',
            'own_odom_frame': '', 'peer_odom_frame': '',
            'own_odom_to_peer_odom': [0.0, 0.0, 0.0],
            'peer_radius_m': VERIFIED_EXCLUSION_RADIUS_M,
            'teammate_geometry_radius_m': 0.035,
            'maximum_processing_latency': 0.20,
            'pending_queue_depth': 4, 'transform_retry_period': 0.02,
            'warning_interval': 2.0,
            'simulation_free_space_completion': True,
            'free_space_cap': FREE_SPACE_CAP,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        def value(name):
            return self.get_parameter(name).value
        self.peer_frame = str(value('peer_base_frame'))
        self.expected_frame = str(value('expected_lidar_frame'))
        self.own_odom_frame = str(value('own_odom_frame'))
        self.peer_odom_frame = str(value('peer_odom_frame'))
        fixed = list(value('own_odom_to_peer_odom'))
        if len(fixed) != 3:
            raise ValueError('own_odom_to_peer_odom must contain x, y, yaw')
        self.fixed_odom_transform = Transform2D(*map(float, fixed))
        self.peer_radius = float(value('peer_radius_m'))
        self.range_tolerance = float(value('range_tolerance_m'))
        self.geometry_radius = float(value('teammate_geometry_radius_m'))
        self.max_latency = float(value('maximum_processing_latency'))
        self.retry_period = float(value('transform_retry_period'))
        self.warning_interval = float(value('warning_interval'))
        self.free_space_cap = float(value('free_space_cap'))
        if not bool(value('simulation_free_space_completion')):
            raise ValueError(
                'simulation free-space completion must remain enabled')
        if any(not frame for frame in (
                self.peer_frame, self.expected_frame, self.own_odom_frame,
                self.peer_odom_frame)):
            raise ValueError(
                'peer/lidar/odom frame parameters must not be empty')
        if (self.peer_radius <= 0.0 or self.range_tolerance < 0.0
                or self.geometry_radius <= 0.0):
            raise ValueError(
                'peer radius must be positive and tolerance nonnegative')
        if not 0.0 < self.max_latency <= 0.20 or self.retry_period <= 0.0:
            raise ValueError(
                'latency must be in (0, 0.20] and retry period positive')

        self.publisher = self.create_publisher(
            LaserScan, value('output_topic'), qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            LaserScan, value('input_topic'), self.scan_callback,
            qos_profile_sensor_data)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pending = PendingScanQueue(int(value('pending_queue_depth')))
        self.retry_timer = self.create_timer(self.retry_period,
                                             self._process_pending)
        self.metrics_timer = self.create_timer(10.0, self._log_metrics)
        self.counts = dict.fromkeys((
            'received', 'published', 'masked', 'dropped_tf_timeout',
            'dropped_overflow', 'dropped_invalid_frame',
            'dropped_nonmonotonic', 'raw_positive_infinity',
            'converted_free_cap'), 0)
        self.teammate_window = dict.fromkeys((
            'finite_returns', 'masked_returns', 'suspected_missed_returns',
            'outside_model_radius', 'range_tolerance_miss',
            'noncircular_geometry', 'environment_return'), 0)
        self.maximum_teammate_residual_m = 0.0
        self.sum_teammate_residual_m = 0.0
        self.sum_teammate_radial_m = 0.0
        self.maximum_teammate_radial_m = 0.0
        self.teammate_residual_samples = 0
        self.last_peer_pose_age_s = None
        self.exact_tf_lookups = 0
        self.exact_tf_failures = 0
        self.maximum_pending_depth = 0
        self.longest_output_gap_s = 0.0
        self.last_published_stamp = None
        self.last_output_monotonic = None
        self.last_warning_monotonic = 0.0
        self.get_logger().info(
            f'{self.resolve_topic_name(value("input_topic"))} -> '
            f'{self.resolve_topic_name(value("output_topic"))}; '
            f'odom-only exact-time pose; radius={self.peer_radius:.3f}m '
            f'tolerance={self.range_tolerance:.3f}m; zero passthrough')
        self.get_logger().info(
            f'geometry_model={VERIFIED_GEOMETRY_MODEL}; '
            f'visible_radius={self.geometry_radius:.3f}m')

    @staticmethod
    def _stamp_key(scan):
        return scan.header.stamp.sec, scan.header.stamp.nanosec

    def _warn(self, message):
        now = time.monotonic()
        if now - self.last_warning_monotonic >= self.warning_interval:
            self.last_warning_monotonic = now
            self.get_logger().warning(message)

    def scan_callback(self, scan):
        """Validate and enqueue one fixed scan without passthrough."""
        self.counts['received'] += 1
        if scan.header.frame_id != self.expected_frame:
            self.counts['dropped_invalid_frame'] += 1
            self._warn(
                f'Dropped frame {scan.header.frame_id!r}; '
                f'expected {self.expected_frame!r}')
            return
        if (self.last_published_stamp is not None
                and self._stamp_key(scan) <= self.last_published_stamp):
            self.counts['dropped_nonmonotonic'] += 1
            self._warn('Dropped non-monotonic input scan')
            return
        dropped = self.pending.enqueue(scan, time.monotonic())
        if dropped is not None:
            self.counts['dropped_overflow'] += 1
            self._warn('Pending queue overflow: dropped oldest scan')
        self.maximum_pending_depth = max(
            self.maximum_pending_depth, len(self.pending.items))
        self._process_pending()

    def _lookup_exact(self, target, source, stamp):
        self.exact_tf_lookups += 1
        if not self.tf_buffer.can_transform(
                target, source, stamp, timeout=Duration()):
            self.exact_tf_failures += 1
            raise TransformException(
                f'exact transform {target} <- {source} unavailable')
        try:
            return self.tf_buffer.lookup_transform(
                target, source, stamp, timeout=Duration())
        except TransformException:
            self.exact_tf_failures += 1
            raise

    def _odom_peer_pose(self, stamp):
        """Compute own_lidar <- own_odom <- peer_odom <- peer_base."""
        lidar_from_own_odom = transform_message_2d(
            self._lookup_exact(
                self.expected_frame, self.own_odom_frame, stamp))
        peer_odom_from_peer_base = transform_message_2d(
            self._lookup_exact(self.peer_odom_frame, self.peer_frame, stamp))
        return compose_transform(
            compose_transform(lidar_from_own_odom, self.fixed_odom_transform),
            peer_odom_from_peer_base,
        )

    def _process_pending(self):
        while self.pending.items and rclpy.ok():
            action, item, _ = self.pending.take(
                time.monotonic(), self.max_latency, pose_available=False)
            if action == 'expired':
                self.counts['dropped_tf_timeout'] += 1
                self._warn('Dropped scan after exact-transform deadline')
                continue
            scan = item[0]
            if (self.last_published_stamp is not None
                    and self._stamp_key(scan) <= self.last_published_stamp):
                self.pending.items.popleft()
                self.counts['dropped_nonmonotonic'] += 1
                self._warn('Dropped non-monotonic queued scan')
                continue
            try:
                pose = self._odom_peer_pose(Time.from_msg(scan.header.stamp))
            except TransformException:
                return
            try:
                age = (self.get_clock().now()
                       - Time.from_msg(scan.header.stamp)).nanoseconds / 1e9
                self.last_peer_pose_age_s = max(0.0, age)
            except (TypeError, ValueError):
                self.last_peer_pose_age_s = None
            action, item, _ = self.pending.take(
                time.monotonic(), self.max_latency, pose_available=True)
            if action == 'expired':
                self.counts['dropped_tf_timeout'] += 1
                self._warn('Dropped scan after exact-transform deadline')
                continue
            self._publish_filtered(item[0], pose)

    def _publish_filtered(self, scan, pose):
        output, indices, completion = prepare_slam_scan(
            scan, pose.x, pose.y, self.peer_radius,
            self.range_tolerance, self.free_space_cap,
            geometry_radius_m=self.geometry_radius)
        self._record_teammate_window(scan, pose, indices)
        if not rclpy.ok():
            return
        self.publisher.publish(output)
        now = time.monotonic()
        if self.last_output_monotonic is not None:
            self.longest_output_gap_s = max(
                self.longest_output_gap_s,
                now - self.last_output_monotonic)
        self.last_output_monotonic = now
        self.last_published_stamp = self._stamp_key(scan)
        self.counts['published'] += 1
        self.counts['masked'] += len(indices)
        self.counts['raw_positive_infinity'] += (
            completion.raw_positive_infinity)
        self.counts['converted_free_cap'] += completion.converted_free_cap

    def _record_teammate_window(self, scan, pose, masked_indices):
        """Aggregate finite-return diagnostics around the peer silhouette."""
        masked = set(masked_indices)
        window = self.geometry_radius + self.range_tolerance + 0.02
        for index, measured in enumerate(scan.ranges):
            if not (math.isfinite(measured)
                    and measured > scan.range_min
                    and measured < scan.range_max):
                continue
            angle = scan.angle_min + index * scan.angle_increment
            endpoint_x = measured * math.cos(angle)
            endpoint_y = measured * math.sin(angle)
            radial = math.hypot(endpoint_x - pose.x, endpoint_y - pose.y)
            if radial > window:
                continue
            self.teammate_window['finite_returns'] += 1
            predicted = circle_first_intersection(
                pose.x, pose.y, self.geometry_radius, angle)
            residual = (abs(measured - predicted)
                        if predicted is not None else math.inf)
            if math.isfinite(residual):
                self.maximum_teammate_residual_m = max(
                    self.maximum_teammate_residual_m, residual)
                self.sum_teammate_residual_m += residual
                self.teammate_residual_samples += 1
            self.sum_teammate_radial_m += radial
            self.maximum_teammate_radial_m = max(
                self.maximum_teammate_radial_m, radial)
            if index in masked:
                self.teammate_window['masked_returns'] += 1
                continue
            self.teammate_window['suspected_missed_returns'] += 1
            if radial > self.geometry_radius + self.range_tolerance:
                self.teammate_window['environment_return'] += 1
            elif predicted is None:
                self.teammate_window['noncircular_geometry'] += 1
            elif residual > self.range_tolerance:
                self.teammate_window['range_tolerance_miss'] += 1
            elif radial > self.geometry_radius:
                self.teammate_window['outside_model_radius'] += 1

    def _metrics_snapshot(self):
        return {
            **self.counts,
            **{f'teammate_window_{key}': value
               for key, value in self.teammate_window.items()},
            'teammate_window_max_residual_m': round(
                self.maximum_teammate_residual_m, 6),
            'teammate_window_mean_residual_m': round(
                self.sum_teammate_residual_m / max(
                    1, self.teammate_residual_samples), 6),
            'teammate_window_mean_radial_m': round(
                self.sum_teammate_radial_m / max(
                    1, self.teammate_window['finite_returns']), 6),
            'teammate_window_max_radial_m': round(
                self.maximum_teammate_radial_m, 6),
            'geometry_radius_m': self.geometry_radius,
            'geometry_model': VERIFIED_GEOMETRY_MODEL,
            'peer_pose_age_s': self.last_peer_pose_age_s,
            'exact_tf_lookups': self.exact_tf_lookups,
            'exact_tf_failures': self.exact_tf_failures,
            'pending_depth': len(self.pending.items),
            'maximum_pending_depth': self.maximum_pending_depth,
            'longest_output_gap_s': round(self.longest_output_gap_s, 6),
        }

    def _log_metrics(self):
        self.get_logger().info('FILTER_METRICS ' + ' '.join(
            f'{key}={value}'
            for key, value in self._metrics_snapshot().items()))


def main(args=None):
    """Run the teammate scan filter with explicit executor teardown."""
    rclpy.init(args=args)
    node = TeammateScanFilter()
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.pending.items.clear()
        executor.remove_node(node)
        executor.shutdown()
        if node.context.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
