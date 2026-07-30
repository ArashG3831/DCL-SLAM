import copy
from collections import deque
from dataclasses import dataclass
import math
import statistics
import threading
import time

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from my_epuck_project.slam_range_policy import (
    FREE_SPACE_CAP,
    complete_natural_no_returns,
)


@dataclass(frozen=True)
class Transform2D:
    x: float
    y: float
    yaw: float


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compose_transform(first, second):
    """Compose T_a_b and T_b_c to produce T_a_c."""
    cosine, sine = math.cos(first.yaw), math.sin(first.yaw)
    return Transform2D(
        first.x + cosine * second.x - sine * second.y,
        first.y + sine * second.x + cosine * second.y,
        normalize_angle(first.yaw + second.yaw),
    )


def inverse_transform(transform):
    cosine, sine = math.cos(transform.yaw), math.sin(transform.yaw)
    return Transform2D(
        -cosine * transform.x - sine * transform.y,
        sine * transform.x - cosine * transform.y,
        normalize_angle(-transform.yaw),
    )


def transform_message_2d(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    yaw = math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )
    return Transform2D(translation.x, translation.y, yaw)


def pose_difference(first, second):
    return math.hypot(first.x - second.x, first.y - second.y), abs(
        normalize_angle(first.yaw - second.yaw)
    )


def circle_ray_interval(center_x, center_y, radius, angle):
    dx, dy = math.cos(angle), math.sin(angle)
    projection = center_x * dx + center_y * dy
    perpendicular_squared = center_x**2 + center_y**2 - projection**2
    if perpendicular_squared > radius**2 + 1e-7:
        return None
    half_chord = math.sqrt(max(0.0, radius**2 - perpendicular_squared))
    near, far = projection - half_chord, projection + half_chord
    if far < 0.0:
        return None
    return max(0.0, near), far


def beam_directions(scan):
    return [
        (math.cos(scan.angle_min + index * scan.angle_increment),
         math.sin(scan.angle_min + index * scan.angle_increment))
        for index in range(len(scan.ranges))
    ]


def selected_indices_cached(scan, center_x, center_y, radius,
                            range_tolerance, directions,
                            natural_no_return_indices=()):
    selected, intervals = [], {}
    distance = math.hypot(center_x, center_y)
    if distance <= radius:
        candidates = range(len(scan.ranges))
    else:
        center_angle = math.atan2(center_y, center_x)
        half_angle = math.asin(min(1.0, radius / distance))
        candidates = []
        for index, (dx, dy) in enumerate(directions):
            beam_angle = math.atan2(dy, dx)
            if abs(normalize_angle(beam_angle - center_angle)) <= half_angle + 1e-7:
                candidates.append(index)
    radius_squared = radius * radius
    for index in candidates:
        measured_range = scan.ranges[index]
        if not math.isfinite(measured_range):
            continue
        if measured_range < scan.range_min or measured_range > scan.range_max:
            continue
        dx, dy = directions[index]
        projection = center_x * dx + center_y * dy
        perpendicular_squared = center_x**2 + center_y**2 - projection**2
        if perpendicular_squared > radius_squared + 1e-7:
            continue
        half_chord = math.sqrt(max(0.0, radius_squared - perpendicular_squared))
        near, far = max(0.0, projection - half_chord), projection + half_chord
        if far < 0.0:
            continue
        intervals[index] = (near, far)
        natural_no_return = index in natural_no_return_indices
        if (natural_no_return and near <= scan.range_max + 1e-7) or (
                near - range_tolerance - 1e-7 <= measured_range <=
                far + range_tolerance + 1e-7):
            selected.append(index)
    return selected, intervals


def selected_indices(scan, center_x, center_y, radius, range_tolerance):
    return selected_indices_cached(
        scan, center_x, center_y, radius, range_tolerance, beam_directions(scan)
    )


def filtered_scan(scan, center_x, center_y, radius, range_tolerance,
                  directions=None, natural_no_return_indices=()):
    output = copy.copy(scan)
    output.ranges = list(scan.ranges)
    output.intensities = list(scan.intensities)
    directions = directions if directions is not None else beam_directions(scan)
    indices, intervals = selected_indices_cached(
        scan, center_x, center_y, radius, range_tolerance, directions,
        natural_no_return_indices,
    )
    for index in indices:
        output.ranges[index] = math.nan
    return output, indices, intervals


class PendingScanQueue:
    def __init__(self, maximum_depth):
        self.maximum_depth = maximum_depth
        self.items = deque()
        self.overflow_drops = 0

    @staticmethod
    def key(item):
        stamp = item[0].header.stamp
        return stamp.sec, stamp.nanosec

    def enqueue(self, scan, arrival_time):
        items = list(self.items)
        items.append((scan, arrival_time))
        items.sort(key=self.key)
        dropped = None
        if len(items) > self.maximum_depth:
            dropped = items.pop(0)
            self.overflow_drops += 1
        self.items = deque(items)
        return dropped

    def peek(self):
        return self.items[0] if self.items else None

    def take(self, now, maximum_latency, pose_available):
        item = self.peek()
        if item is None:
            return 'idle', None, 0.0
        waited = now - item[1]
        if waited > maximum_latency:
            return 'expired', self.items.popleft(), waited
        if not pose_available:
            return 'wait', item, waited
        return 'publish', self.items.popleft(), waited

    def clear(self):
        self.items.clear()


class PoseSourceTransition:
    def __init__(self, required_matches, position_tolerance, yaw_tolerance):
        self.state = 'waiting_for_odom'
        self.required_matches = required_matches
        self.position_tolerance = position_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.consecutive_matches = 0
        self.rejected_transitions = 0
        self.last_difference = None

    def odom_available(self):
        if self.state == 'waiting_for_odom':
            self.state = 'odom_bootstrap'

    def compare_shared(self, odom_pose, shared_pose):
        if self.state == 'shared_map_tf_active':
            return True
        position, yaw = pose_difference(odom_pose, shared_pose)
        self.last_difference = (position, yaw)
        if position > self.position_tolerance or yaw > self.yaw_tolerance:
            self.consecutive_matches = 0
            self.rejected_transitions += 1
            return False
        self.consecutive_matches += 1
        if self.consecutive_matches >= self.required_matches:
            self.state = 'shared_map_tf_active'
            return True
        return False


class TeammateScanFilter(Node):
    def __init__(self):
        super().__init__('teammate_scan_filter')
        defaults = {
            'input_topic': 'scan_d500_fixed', 'output_topic': 'scan_d500_slam',
            'peer_base_frame': '', 'expected_lidar_frame': '',
            'own_odom_frame': '', 'peer_odom_frame': '',
            'own_odom_to_peer_odom': [0.0, 0.0, 0.0],
            'peer_shape_type': 'circle', 'peer_shape_dimensions': [0.050],
            'static_safety_margin': 0.003, 'dynamic_motion_margin': 0.007,
            'range_matching_tolerance': 0.012,
            'maximum_processing_latency': 0.2, 'pending_queue_depth': 4,
            'transform_retry_period': 0.02,
            'queue_overflow_policy': 'drop_oldest',
            'allow_latest_transform_fallback': False,
            'shared_tf_confirmation_scans': 5,
            'shared_tf_position_tolerance': 0.05,
            'shared_tf_yaw_tolerance': 0.15,
            'warning_interval': 2.0,
            'mode': 'simulation',
            'simulation_free_space_completion': True,
            'physical_free_space_completion': False,
            'free_space_cap': FREE_SPACE_CAP,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        value = lambda name: self.get_parameter(name).value
        input_topic, output_topic = value('input_topic'), value('output_topic')
        self.peer_frame = value('peer_base_frame')
        self.expected_frame = value('expected_lidar_frame')
        self.own_odom_frame = value('own_odom_frame')
        self.peer_odom_frame = value('peer_odom_frame')
        fixed = list(value('own_odom_to_peer_odom'))
        self.fixed_odom_transform = Transform2D(*map(float, fixed))
        dimensions = list(value('peer_shape_dimensions'))
        self.static_margin = float(value('static_safety_margin'))
        self.dynamic_margin = float(value('dynamic_motion_margin'))
        self.range_tolerance = float(value('range_matching_tolerance'))
        self.max_latency = float(value('maximum_processing_latency'))
        self.queue_depth_limit = int(value('pending_queue_depth'))
        self.retry_period = float(value('transform_retry_period'))
        self.warning_interval = float(value('warning_interval'))
        self.mode = str(value('mode')).strip().lower()
        if self.mode not in ('simulation', 'physical'):
            raise ValueError("mode must be 'simulation' or 'physical'")
        self.free_space_completion = bool(
            value('simulation_free_space_completion') if self.mode == 'simulation'
            else value('physical_free_space_completion'))
        self.free_space_cap = float(value('free_space_cap'))
        required_frames = (self.peer_frame, self.expected_frame,
                           self.own_odom_frame, self.peer_odom_frame)
        if any(not frame for frame in required_frames):
            raise ValueError('peer/lidar/odom frame parameters must not be empty')
        if value('peer_shape_type') != 'circle' or len(dimensions) != 1:
            raise ValueError('peer shape must be a circle with one diameter')
        if value('queue_overflow_policy') != 'drop_oldest':
            raise ValueError('only drop_oldest overflow is supported')
        if bool(value('allow_latest_transform_fallback')):
            raise ValueError('latest-transform fallback is intentionally unsupported')
        self.body_radius = float(dimensions[0]) / 2.0
        self.effective_radius = self.body_radius + self.static_margin + self.dynamic_margin
        self.pose_transition = PoseSourceTransition(
            int(value('shared_tf_confirmation_scans')),
            float(value('shared_tf_position_tolerance')),
            float(value('shared_tf_yaw_tolerance')),
        )
        self.publisher = self.create_publisher(LaserScan, output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            LaserScan, input_topic, self.scan_callback, qos_profile_sensor_data)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.state_lock = threading.Lock()
        self.pending = PendingScanQueue(self.queue_depth_limit)
        self.received_scan_count = self.enqueued_scan_count = 0
        self.published_filtered_count = self.masked_beam_count = 0
        self.latency_expired_drop_count = self.queue_overflow_drop_count = 0
        self.invalid_frame_drop_count = self.stale_input_drop_count = 0
        self.maximum_observed_queue_depth = 0
        self.pose_source_counts = {'waiting_for_odom': 0, 'odom_bootstrap': 0,
                                   'shared_map_tf_active': 0}
        self.transform_wait_samples = deque(maxlen=2048)
        self.last_published_stamp = None
        self.last_warning_monotonic = 0.0
        self.direction_key = None
        self.directions = None
        self.last_masked_indices = []
        self.last_intervals = {}
        self.last_peer_position = None
        self.last_scan_metrics = {}
        self.scan_metric_totals = {
            'raw_positive_infinity': 0, 'converted_free_cap': 0,
            'teammate_masked': 0, 'nan': 0, 'negative_infinity': 0,
            'finite_obstacle': 0, 'exact_range_max': 0, 'invalid': 0,
        }
        self.logged_scan_contract = False
        self.retry_timer = self.create_timer(self.retry_period, self.process_pending)
        self.metrics_timer = self.create_timer(10.0, self.log_metrics)
        self.get_logger().info(
            f'{self.resolve_topic_name(input_topic)} -> {self.resolve_topic_name(output_topic)}; '
            f'fixed_odom={self.fixed_odom_transform}; zero passthrough; '
            f'queue={self.queue_depth_limit} retry={self.retry_period:.3f}s exact-time-only; '
            f'mode={self.mode} free_space_cap={self.free_space_cap:.6f}m '
            f'completion_enabled={self.free_space_completion} '
            f'masking_after_conversion=True')

    def warn(self, message):
        now = time.monotonic()
        with self.state_lock:
            allowed = now - self.last_warning_monotonic >= self.warning_interval
            if allowed:
                self.last_warning_monotonic = now
        if allowed:
            self.get_logger().warning(message)

    @staticmethod
    def stamp_seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    @staticmethod
    def stamp_key(scan):
        return scan.header.stamp.sec, scan.header.stamp.nanosec

    def scan_callback(self, scan):
        arrival = time.monotonic()
        with self.state_lock:
            self.received_scan_count += 1
        if not scan.header.frame_id or scan.header.frame_id != self.expected_frame:
            with self.state_lock:
                self.invalid_frame_drop_count += 1
            self.warn(f'Dropped frame {scan.header.frame_id!r}; expected {self.expected_frame!r}')
            return
        age = self.get_clock().now().nanoseconds * 1e-9 - self.stamp_seconds(scan.header.stamp)
        if age > self.max_latency:
            with self.state_lock:
                self.stale_input_drop_count += 1
            self.warn(f'Dropped stale input aged {age:.3f}s')
            return
        with self.state_lock:
            dropped = self.pending.enqueue(scan, arrival)
            self.enqueued_scan_count += 1
            if dropped is not None:
                self.queue_overflow_drop_count += 1
            depth = len(self.pending.items)
            self.maximum_observed_queue_depth = max(self.maximum_observed_queue_depth, depth)
        if dropped is not None:
            self.warn('Pending scan queue overflow: dropped oldest scan')

    def lookup_exact(self, target, source, stamp):
        if not self.tf_buffer.can_transform(target, source, stamp, timeout=Duration()):
            raise TransformException(f'exact transform {target} <- {source} unavailable')
        return self.tf_buffer.lookup_transform(target, source, stamp, timeout=Duration())

    def odom_peer_pose(self, stamp):
        lidar_from_own_odom = transform_message_2d(
            self.lookup_exact(self.expected_frame, self.own_odom_frame, stamp))
        peer_odom_from_peer_base = transform_message_2d(
            self.lookup_exact(self.peer_odom_frame, self.peer_frame, stamp))
        return compose_transform(
            compose_transform(lidar_from_own_odom, self.fixed_odom_transform),
            peer_odom_from_peer_base,
        )

    def shared_peer_pose(self, stamp):
        return transform_message_2d(
            self.lookup_exact(self.expected_frame, self.peer_frame, stamp))

    def resolve_peer_pose(self, scan):
        stamp = Time.from_msg(scan.header.stamp)
        with self.state_lock:
            state = self.pose_transition.state
        if state == 'shared_map_tf_active':
            return self.shared_peer_pose(stamp), state
        odom_pose = self.odom_peer_pose(stamp)
        with self.state_lock:
            self.pose_transition.odom_available()
            state = self.pose_transition.state
        try:
            shared_pose = self.shared_peer_pose(stamp)
        except TransformException:
            return odom_pose, state
        with self.state_lock:
            switched = self.pose_transition.compare_shared(odom_pose, shared_pose)
            state = self.pose_transition.state
            difference = self.pose_transition.last_difference
        if switched and state == 'shared_map_tf_active':
            self.get_logger().info(
                f'POSE_SOURCE_SHARED_MAP_ACTIVE position_difference={difference[0]:.6f} '
                f'yaw_difference={difference[1]:.6f}')
            return shared_pose, state
        return odom_pose, state

    def process_pending(self):
        while rclpy.ok():
            now = time.monotonic()
            with self.state_lock:
                item = self.pending.peek()
            if item is None:
                return
            scan = item[0]
            try:
                pose, source = self.resolve_peer_pose(scan)
                available = True
            except TransformException:
                pose, source, available = None, 'waiting_for_odom', False
            with self.state_lock:
                action, selected, waited = self.pending.take(now, self.max_latency, available)
                if action == 'expired':
                    self.latency_expired_drop_count += 1
            if action in ('idle', 'wait'):
                return
            if action == 'expired':
                self.warn(f'Dropped startup/queued scan after {waited:.3f}s pose wait')
                continue
            scan = selected[0]
            key = self.stamp_key(scan)
            with self.state_lock:
                stale = self.last_published_stamp is not None and key <= self.last_published_stamp
                if stale:
                    self.stale_input_drop_count += 1
                else:
                    self.transform_wait_samples.append(waited)
                    self.pose_source_counts[source] += 1
            if stale:
                self.warn('Dropped non-monotonic queued scan')
                continue
            self.publish_filtered(scan, pose)

    def scan_directions(self, scan):
        key = (len(scan.ranges), scan.angle_min, scan.angle_increment)
        if key != self.direction_key:
            self.directions = beam_directions(scan)
            self.direction_key = key
        return self.directions

    def publish_filtered(self, scan, pose):
        if not self.logged_scan_contract:
            self.get_logger().info(
                f'SCAN_CONTRACT mode={self.mode} original_range_max='
                f'{scan.range_max:.6f} free_space_cap={self.free_space_cap:.6f} '
                f'completion_enabled={self.free_space_completion} '
                f'masking_after_conversion=True')
            self.logged_scan_contract = True
        completed_ranges, completion = complete_natural_no_returns(
            scan.ranges, self.free_space_cap, scan.range_min, scan.range_max,
            self.free_space_completion)
        completed = copy.copy(scan)
        completed.ranges = completed_ranges
        natural_no_return_indices = {
            index for index, value in enumerate(scan.ranges)
            if math.isinf(value) and value > 0.0
        }
        output, indices, intervals = filtered_scan(
            completed, pose.x, pose.y, self.effective_radius,
            self.range_tolerance, self.scan_directions(completed),
            natural_no_return_indices)
        if not rclpy.ok():
            return
        self.publisher.publish(output)
        with self.state_lock:
            self.last_published_stamp = self.stamp_key(scan)
            self.last_peer_position = (pose.x, pose.y, pose.yaw)
            self.last_masked_indices = indices
            self.last_intervals = intervals
            self.last_scan_metrics = {
                'raw_positive_infinity': completion.raw_positive_infinity,
                'converted_free_cap': completion.converted_free_cap,
                'teammate_masked': len(indices),
                'nan': completion.nan + len(indices),
                'negative_infinity': completion.negative_infinity,
                'finite_obstacle': completion.finite_obstacle,
                'exact_range_max': completion.exact_range_max,
                'invalid': completion.invalid,
            }
            for key, value in self.last_scan_metrics.items():
                self.scan_metric_totals[key] += value
            self.published_filtered_count += 1
            self.masked_beam_count += len(indices)

    def metrics_snapshot(self):
        with self.state_lock:
            waits = list(self.transform_wait_samples)
            values = dict(received=self.received_scan_count,
                          enqueued=self.enqueued_scan_count,
                          published=self.published_filtered_count,
                          masked=self.masked_beam_count,
                          expired=self.latency_expired_drop_count,
                          overflow=self.queue_overflow_drop_count,
                          invalid_frame=self.invalid_frame_drop_count,
                          stale=self.stale_input_drop_count,
                          depth=len(self.pending.items),
                          max_depth=self.maximum_observed_queue_depth,
                          pose_state=self.pose_transition.state,
                          pose_counts=dict(self.pose_source_counts),
                          transition_rejections=self.pose_transition.rejected_transitions,
                          transition_difference=self.pose_transition.last_difference,
                          scan_metrics=dict(self.scan_metric_totals))
        if waits:
            ordered = sorted(waits)
            p95 = ordered[min(len(ordered)-1, math.ceil(.95*len(ordered))-1)]
            values.update(wait_min=min(waits), wait_mean=statistics.fmean(waits),
                          wait_median=statistics.median(waits), wait_p95=p95,
                          wait_max=max(waits), wait_samples=len(waits))
        else:
            values.update(wait_min=0.0, wait_mean=0.0, wait_median=0.0,
                          wait_p95=0.0, wait_max=0.0, wait_samples=0)
        return values

    def log_metrics(self):
        metrics = self.metrics_snapshot()
        self.get_logger().info('FILTER_METRICS ' + ' '.join(
            f'{key}={value}' for key, value in metrics.items()))

    def clear_pending(self):
        with self.state_lock:
            self.pending.clear()


def main(args=None):
    rclpy.init(args=args)
    node = TeammateScanFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.clear_pending()
        if node.context.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
