"""Small namespace-local Nav2 path evaluation and execution boundary."""

from dataclasses import dataclass
from collections import deque
import fcntl
import hashlib
import json
import math
import time
from typing import Callable, Optional

from action_msgs.msg import GoalStatus

from lifecycle_msgs.srv import GetState

from nav2_msgs.action import ComputePathToPose, NavigateToPose

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from tf2_ros import Buffer, TransformException, TransformListener

from .failures import classify_failure
from .models import FailureClass, FailureEvidence, PhysicalTask, Point, TravelDistance


# NavigateToPose does not declare child-controller result constants, but the
# Nav2 BT propagates FollowPath's result code in the observed terminal result.
# Keep this mapping local and explicit so a controller abort becomes hard
# evidence for task suppression without changing navigation behavior.
FOLLOW_PATH_TF_FAILURE_CODES = frozenset({102})
FOLLOW_PATH_CONTROLLER_FAILURE_CODES = frozenset({104, 105, 106, 107})

FOLLOW_PATH_CONTROLLER_ERROR_NAMES = {
    102: 'TF_ERROR',
    104: 'PATIENCE_EXCEEDED',
    105: 'FAILED_TO_MAKE_PROGRESS',
    106: 'NO_VALID_CONTROL',
    107: 'CONTROLLER_TIMED_OUT',
}


def classify_follow_path_controller_error(error_code: int) -> FailureClass:
    """Classify propagated FollowPath failures without hiding TF faults."""
    if int(error_code) in FOLLOW_PATH_TF_FAILURE_CODES:
        return FailureClass.TF_OR_LIFECYCLE
    if int(error_code) in FOLLOW_PATH_CONTROLLER_FAILURE_CODES:
        return FailureClass.CONTROLLER_NO_PROGRESS
    return FailureClass.UNKNOWN


def follow_path_controller_error_name(error_code: int) -> str:
    """Return the installed Nav2 FollowPath meaning without flattening it."""
    return FOLLOW_PATH_CONTROLLER_ERROR_NAMES.get(int(error_code), '')


@dataclass(frozen=True)
class PathEvaluation:
    """Observable result of one local ComputePathToPose request."""

    valid: bool
    length_m: float
    samples: tuple[Point, ...]
    query_ros_ns: int
    error_code: int
    error_message: str
    failure_class: FailureClass
    query_started_ros_ns: int = 0
    duration_s: float = 0.0
    caller: str = 'UNKNOWN'
    task_signature: str = ''
    map_stamp_ns: int = 0
    costmap_stamp_ns: int = 0


@dataclass(frozen=True)
class DispatchPreconditions:
    """Auditable final local dispatch preconditions."""

    action_server_ready: bool
    lifecycle_active: bool
    transform_available: bool
    transform_age_s: Optional[float]
    goal_inside_map: bool
    goal_inside_costmap: bool
    goal_map_value: Optional[int]
    goal_costmap_value: Optional[int]
    no_local_goal_active: bool
    final_path_valid: bool
    reason: str

    @property
    def ready(self) -> bool:
        """Return whether every required dispatch condition passed."""
        return (
            self.action_server_ready and self.lifecycle_active and
            self.transform_available and self.goal_inside_map and
            self.goal_inside_costmap and self.no_local_goal_active and
            self.final_path_valid and not self.reason
        )


@dataclass(frozen=True)
class NavigationOutcome:
    """Terminal local NavigateToPose evidence and measured motion."""

    accepted: bool
    status: int
    error_code: int
    error_message: str
    failure_class: FailureClass
    duration_s: float
    travelled_distance_m: float
    recoveries: int
    nav2_error_name: str = ''
    follow_path_error_code: int = 0
    follow_path_error_name: str = ''
    controller_failure_family: str = ''
    deepest_failure_classification: str = ''
    deepest_failure_timestamp_ros_ns: int = 0
    diagnostic_snapshot_json: str = ''


UPSTREAM_POINT_BLOCK_THRESHOLD = 50


def upstream_point_validation(
        grid: Optional[OccupancyGrid], point: Point,
        threshold: int = UPSTREAM_POINT_BLOCK_THRESHOLD) -> dict:
    """Mirror v1.6.0 ``world_point_cost``/``is_world_point_blocked``.

    The upstream public point helper treats absent and out-of-bounds points as
    not blocked, and blocks only a present occupancy value strictly greater
    than ``OCC_THRESHOLD``.  Keep the raw value and the reason separate so
    this diagnostic cannot be mistaken for a dispatch gate.
    """
    if grid is None:
        return {'status': 'NO_DATA', 'cost': None, 'in_bounds': False}
    value = occupancy_value(grid, point)
    if value is None:
        return {'status': 'OUT_OF_BOUNDS', 'cost': None, 'in_bounds': False}
    if int(value) < 0:
        return {'status': 'UNKNOWN', 'cost': int(value), 'in_bounds': True}
    return {
        'status': 'BLOCKED' if int(value) > int(threshold) else 'FREE_OR_ACCEPTED',
        'cost': int(value),
        'in_bounds': True,
    }


def _grid_cell(grid: OccupancyGrid, point: tuple[float, float]) -> Optional[tuple[int, int]]:
    """Return a world point's integer cell without exposing an unbounded grid."""
    resolution = float(grid.info.resolution)
    if resolution <= 0.0:
        return None
    origin = grid.info.origin
    quaternion = origin.orientation
    yaw = math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2),
    )
    dx = point[0] - origin.position.x
    dy = point[1] - origin.position.y
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return math.floor(local_x / resolution), math.floor(local_y / resolution)


def bounded_grid_crop(
        grid: Optional[OccupancyGrid], center: Optional[tuple[float, float]],
        radius_cells: int = 10) -> dict:
    """Return a bounded row-major occupancy crop for failure diagnostics."""
    if grid is None or center is None:
        return {}
    cell = _grid_cell(grid, center)
    if cell is None:
        return {}
    cx, cy = cell
    radius = max(1, min(int(radius_cells), 20))
    values = []
    for y in range(cy - radius, cy + radius + 1):
        for x in range(cx - radius, cx + radius + 1):
            if x < 0 or y < 0 or x >= grid.info.width or y >= grid.info.height:
                values.append(None)
            else:
                index = y * grid.info.width + x
                values.append(int(grid.data[index]) if index < len(grid.data) else None)
    return {
        'center_cell': [cx, cy],
        'radius_cells': radius,
        'resolution_m': float(grid.info.resolution),
        'width': 2 * radius + 1,
        'height': 2 * radius + 1,
        'values': values,
    }


def _transform_translation(tf_buffer, target_frame: str, source_frame: str):
    """Return the latest source-frame origin expressed in target_frame."""
    try:
        transform = tf_buffer.lookup_transform(
            target_frame, source_frame, Time(),
            timeout=Duration(seconds=0.0),
        )
    except TransformException:
        return None
    return (
        float(transform.transform.translation.x),
        float(transform.transform.translation.y),
    )


def _transform_point(tf_buffer, target_frame: str, source_frame: str,
                     point: Point):
    """Return a source-frame point expressed in target_frame, if available."""
    try:
        transform = tf_buffer.lookup_transform(
            target_frame, source_frame, Time(),
            timeout=Duration(seconds=0.0),
        )
    except TransformException:
        return None
    rotation = transform.transform.rotation
    yaw = math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2),
    )
    translation = transform.transform.translation
    return (
        math.cos(yaw) * point[0] - math.sin(yaw) * point[1] + translation.x,
        math.sin(yaw) * point[0] + math.cos(yaw) * point[1] + translation.y,
    ), _stamp_ns_static(transform.header.stamp)


def _stamp_ns_static(stamp) -> int:
    """Convert a ROS builtin time stamp without requiring a node instance."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def execution_geometry_signature(
        target: Optional[tuple[float, float]], costmap_crop: dict) -> str:
    """Hash execution-relevant local geometry, excluding timestamps."""
    payload = {
        'target': None if target is None else [round(target[0], 3), round(target[1], 3)],
        'resolution_m': costmap_crop.get('resolution_m'),
        'width': costmap_crop.get('width'),
        'height': costmap_crop.get('height'),
        'values': costmap_crop.get('values', []),
    }
    return hashlib.sha256(
        json.dumps(payload, sort_keys=True, separators=(',', ':')).encode('utf-8'),
    ).hexdigest()[:20]


def path_length(points: tuple[Point, ...]) -> float:
    """Measure a path polyline in its declared frame."""
    return sum(math.dist(first, second) for first, second in zip(points, points[1:]))


def downsample_path(points: tuple[Point, ...], maximum_samples: int) -> tuple[Point, ...]:
    """Keep deterministic endpoints and bounded evenly spaced path samples."""
    if len(points) <= maximum_samples:
        return points
    if maximum_samples < 2:
        return points[:maximum_samples]
    indices = {
        round(index * (len(points) - 1) / (maximum_samples - 1))
        for index in range(maximum_samples)
    }
    return tuple(points[index] for index in sorted(indices))


def occupancy_value(grid: OccupancyGrid, point: Point) -> Optional[int]:
    """Read a world point from a grid with a possibly rotated origin."""
    resolution = grid.info.resolution
    if resolution <= 0.0:
        return None
    origin = grid.info.origin
    quaternion = origin.orientation
    yaw = math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2),
    )
    dx = point[0] - origin.position.x
    dy = point[1] - origin.position.y
    cosine, sine = math.cos(yaw), math.sin(yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    cell_x = math.floor(local_x / resolution)
    cell_y = math.floor(local_y / resolution)
    if (cell_x < 0 or cell_y < 0 or
            cell_x >= grid.info.width or cell_y >= grid.info.height):
        return None
    index = cell_y * grid.info.width + cell_x
    if index >= len(grid.data):
        return None
    return int(grid.data[index])


class LocalNav2:
    """Own only this node namespace's planner, navigator, state, TF, and odometry."""

    def __init__(self, node: Node, *, phase_gated: bool = False):
        """Create relative interfaces that resolve inside the local robot namespace."""
        self._node = node
        self._phase_inputs_active = not phase_gated
        self._robot_id = node.get_namespace().strip('/') or 'root'
        self._global_frame = node.declare_parameter('global_frame', 'shared_map').value
        self._base_frame = node.declare_parameter('robot_base_frame', 'base_footprint').value
        self._nav2_node_prefix = str(node.declare_parameter(
            'nav2_node_prefix', '').value)
        self._planner_id = node.declare_parameter('planner_id', 'GridBased').value
        self._path_timeout_s = float(node.declare_parameter('path_query_timeout_s', 1.5).value)
        self._navigation_timeout_s = float(
            node.declare_parameter('navigation_timeout_s', 240.0).value,
        )
        self._navigation_no_progress_timeout_s = float(
            node.declare_parameter(
                'navigation_no_progress_timeout_s', 30.0,
            ).value,
        )
        self._navigation_min_progress_m = float(
            node.declare_parameter('navigation_min_progress_m', 0.05).value,
        )
        self._maximum_path_samples = int(
            node.declare_parameter('maximum_path_samples', 32).value,
        )
        self._maximum_tf_age_s = float(
            node.declare_parameter('maximum_tf_age_s', 1.0).value,
        )
        self._costmap_lethal_threshold = int(
            node.declare_parameter('costmap_lethal_threshold', 253).value,
        )
        namespace = node.get_namespace().strip('/') or 'root'
        self._path_query_lock_path = str(node.declare_parameter(
            'path_query_lock_path',
            '/tmp/my_epuck_%s_compute_path.lock' % namespace,
        ).value)
        self._path_query_lock_file = None
        self._compute_client = ActionClient(
            node, ComputePathToPose, 'compute_path_to_pose',
        )
        self._navigate_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self._lifecycle_clients = {
            name: node.create_client(
                GetState, f'{self._nav2_node_prefix}{name}/get_state')
            for name in ('planner_server', 'controller_server', 'bt_navigator')
        }
        transient_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_topic = str(node.declare_parameter('map_topic', 'shared_map').value)
        self._phase_subscriptions = []
        # Scan freshness is diagnostic-only.  The production Nav2/SLAM stack
        # publishes the corrected scan as scan_d500_fixed.  The allocator
        # intentionally does not create a cmd_vel interface; command capture
        # belongs to the observer node so exploration remains a local Nav2
        # action boundary.
        self._scan_topic = str(node.declare_parameter(
            'diagnostic_scan_topic', 'scan_d500_fixed').value)
        self._tf_buffer = Buffer(node=node)
        self._tf_listener = None
        self._map: Optional[OccupancyGrid] = None
        self._costmap: Optional[OccupancyGrid] = None
        self._local_costmap: Optional[OccupancyGrid] = None
        self._distance = TravelDistance(maximum_step_m=0.5)
        self._active_path_request = 0
        self._path_deadline_steady_s = 0.0
        self._path_callback: Optional[Callable[[PathEvaluation], None]] = None
        self._path_started_steady_s = 0.0
        self._path_started_ros_ns = 0
        self._path_caller = 'UNKNOWN'
        self._path_task_signature = ''
        self._path_map_stamp_ns = 0
        self._path_costmap_stamp_ns = 0
        self._path_goal_handle = None
        self._navigation_goal_handle = None
        self._navigation_send_pending = False
        self._navigation_callback: Optional[Callable[[NavigationOutcome], None]] = None
        self._navigation_started_steady_s = 0.0
        self._navigation_start_distance_m = 0.0
        self._navigation_last_progress_distance_m = 0.0
        self._navigation_last_progress_steady_s = 0.0
        self._navigation_recoveries = 0
        self._navigation_timeout_requested = False
        self._navigation_no_progress_requested = False
        self._navigation_no_progress_requested_ros_ns = 0
        self._navigation_cancel_requested = False
        self._navigation_target: Optional[tuple[float, float]] = None
        self._navigation_physical_signature = ''
        self._navigation_diagnostic_path: tuple[tuple[float, float], ...] = ()
        self._navigation_goal_number = 0
        self._navigation_point_validation: Optional[dict] = None
        self._navigation_point_last_sample_steady_s = 0.0
        self._navigation_point_sample_period_s = 0.5
        self._last_odom_stamp_ns = 0
        self._last_odom_linear = (0.0, 0.0)
        self._last_odom_angular_z = 0.0
        self._last_scan_stamp_ns = 0
        self._last_scan_min_range: Optional[float] = None
        self._recent_odom_samples = deque(maxlen=32)
        self._last_lifecycle_active: Optional[bool] = None
        self._lifecycle_health_pending = False
        self._timer = None
        if self._phase_inputs_active:
            self._activate_phase_inputs(transient_qos)

    def _activate_phase_inputs(self, transient_qos=None) -> None:
        """Start map/health callbacks for a post-handoff shared phase."""
        if self._phase_inputs_active and self._timer is not None:
            return
        if transient_qos is None:
            transient_qos = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
        self._phase_inputs_active = True
        if self._tf_listener is None:
            self._tf_listener = TransformListener(
                self._tf_buffer, self._node, spin_thread=False)
        self._phase_subscriptions.extend([
            self._node.create_subscription(
                OccupancyGrid, self._map_topic, self._on_map, transient_qos),
            self._node.create_subscription(
                OccupancyGrid, 'global_costmap/costmap', self._on_costmap,
                transient_qos),
            self._node.create_subscription(
                OccupancyGrid, 'local_costmap/costmap', self._on_local_costmap,
                transient_qos),
            self._node.create_subscription(Odometry, 'odom', self._on_odom, 20),
            self._node.create_subscription(LaserScan, self._scan_topic,
                                            self._on_scan, 20),
        ])
        self._timer = self._node.create_timer(0.1, self._check_timeouts)

    @property
    def local_goal_active(self) -> bool:
        """Return whether this wrapper owns an unresolved local navigation goal."""
        return (
            self._navigation_goal_handle is not None or
            self._navigation_send_pending
        )

    @property
    def shared_map(self) -> Optional[OccupancyGrid]:
        """Expose the local shared-map replica for allocator LOS evaluation."""
        return self._map

    @property
    def travelled_distance_m(self) -> float:
        """Return measured cumulative local odometry displacement."""
        return self._distance.distance_m

    def interface_names(self) -> dict[str, str]:
        """Expose resolved local names for static integration auditing."""
        namespace = self._node.get_namespace().rstrip('/')
        return {
            'compute_path': f'{namespace}/compute_path_to_pose',
            'navigate': f'{namespace}/navigate_to_pose',
            'odom': f'{namespace}/odom',
        }

    def refresh_health(self) -> None:
        """Refresh managed-node state without blocking the coordinator timer."""
        if self._lifecycle_health_pending:
            return
        if any(not client.service_is_ready() for client in self._lifecycle_clients.values()):
            self._last_lifecycle_active = False
            return
        self._lifecycle_health_pending = True
        states = {}

        def completed(name, future):
            try:
                states[name] = future.result().current_state.label
            except Exception:  # noqa: B902
                states[name] = 'error'
            if len(states) == len(self._lifecycle_clients):
                self._last_lifecycle_active = all(
                    value == 'active' for value in states.values()
                )
                self._lifecycle_health_pending = False

        for name, client in self._lifecycle_clients.items():
            future = client.call_async(GetState.Request())
            future.add_done_callback(lambda result, key=name: completed(key, result))

    def health_flags(self) -> tuple[bool, bool]:
        """Return conservative current Nav2 and required-transform health flags."""
        nav2_healthy = (
            self._compute_client.server_is_ready() and
            self._navigate_client.server_is_ready() and
            self._last_lifecycle_active is True and
            self._map is not None and self._costmap is not None
        )
        tf_healthy = False
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame, self._base_frame, Time(),
                timeout=Duration(seconds=0.0),
            )
            stamp = transform.header.stamp
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            age_s = 0.0 if stamp_ns == 0 else max(
                0.0, (self._node.get_clock().now().nanoseconds - stamp_ns) / 1e9,
            )
            tf_healthy = age_s <= self._maximum_tf_age_s
        except TransformException:
            pass
        return nav2_healthy, tf_healthy

    def _on_map(self, message: OccupancyGrid) -> None:
        self._map = message

    def _on_costmap(self, message: OccupancyGrid) -> None:
        self._costmap = message

    def _on_local_costmap(self, message: OccupancyGrid) -> None:
        """Retain only the latest rolling costmap for failure diagnostics."""
        self._local_costmap = message

    @staticmethod
    def _grid_stamp(message: Optional[OccupancyGrid]) -> int:
        """Return the source timestamp used for bounded path freshness."""
        if message is None:
            return 0
        return int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec)

    def path_context_matches(self, evaluation: PathEvaluation) -> bool:
        """Reject bid reuse when either local planning input has advanced."""
        if evaluation.map_stamp_ns == 0 or evaluation.costmap_stamp_ns == 0:
            return False
        return (
            self._grid_stamp(self._map) == evaluation.map_stamp_ns and
            self._grid_stamp(self._costmap) == evaluation.costmap_stamp_ns
        )

    def _on_odom(self, message: Odometry) -> None:
        self._last_odom_stamp_ns = self._stamp_ns(message.header.stamp)
        self._last_odom_linear = (
            float(message.twist.twist.linear.x),
            float(message.twist.twist.linear.y),
        )
        self._last_odom_angular_z = float(message.twist.twist.angular.z)
        self._recent_odom_samples.append({
            'stamp_ns': self._last_odom_stamp_ns,
            'linear_x_mps': self._last_odom_linear[0],
            'angular_z_radps': self._last_odom_angular_z,
        })
        position = message.pose.pose.position
        distance = self._distance.observe((position.x, position.y))
        if (self._navigation_goal_handle is not None and
                distance - self._navigation_last_progress_distance_m >=
                self._navigation_min_progress_m):
            self._navigation_last_progress_distance_m = distance
            self._navigation_last_progress_steady_s = time.monotonic()

    @staticmethod
    def _stamp_ns(stamp) -> int:
        """Convert a ROS builtin time stamp to nanoseconds."""
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _on_scan(self, message: LaserScan) -> None:
        self._last_scan_stamp_ns = self._stamp_ns(message.header.stamp)
        finite = [float(value) for value in message.ranges if math.isfinite(value)]
        self._last_scan_min_range = min(finite) if finite else None

    def _age_s(self, stamp_ns: int, now_ns: int) -> Optional[float]:
        """Return a non-negative source age, or None for missing time."""
        if not stamp_ns:
            return None
        return max(0.0, (now_ns - stamp_ns) / 1e9)

    def _point_validation_sample(self) -> dict:
        """Capture point-level upstream validation without affecting dispatch."""
        now_ns = self._node.get_clock().now().nanoseconds
        target = self._navigation_target
        global_result = upstream_point_validation(self._costmap, target) if target else {
            'status': 'NO_DATA', 'cost': None, 'in_bounds': False,
        }
        local_result = {'status': 'NO_DATA', 'cost': None, 'in_bounds': False}
        local_point = None
        local_tf_stamp_ns = 0
        local_frame = '' if self._local_costmap is None else self._local_costmap.header.frame_id
        if self._local_costmap is not None and target is not None:
            transformed = _transform_point(
                self._tf_buffer, local_frame, self._global_frame, target,
            )
            if transformed is None:
                local_result = {
                    'status': 'LOCAL_TF_UNAVAILABLE', 'cost': None, 'in_bounds': False,
                }
            else:
                local_point, local_tf_stamp_ns = transformed
                local_result = upstream_point_validation(self._local_costmap, local_point)
                if local_result['status'] == 'OUT_OF_BOUNDS':
                    local_result = {
                        **local_result, 'status': 'OUT_OF_LOCAL_WINDOW',
                    }
        return {
            'sample_ros_ns': int(now_ns),
            'global_costmap_frame': '' if self._costmap is None else self._costmap.header.frame_id,
            'global_costmap_stamp_ns': self._grid_stamp(self._costmap),
            'global_costmap_age_s': self._age_s(self._grid_stamp(self._costmap), now_ns),
            'global_point_status': global_result['status'],
            'global_point_cost': global_result['cost'],
            'global_point_in_bounds': bool(global_result['in_bounds']),
            'local_costmap_frame': local_frame,
            'local_costmap_stamp_ns': self._grid_stamp(self._local_costmap),
            'local_costmap_age_s': self._age_s(self._grid_stamp(self._local_costmap), now_ns),
            'local_point_frame': local_frame,
            'local_point_xy': local_point,
            'local_tf_stamp_ns': int(local_tf_stamp_ns),
            'local_tf_age_s': self._age_s(local_tf_stamp_ns, now_ns),
            'local_point_status': local_result['status'],
            'local_point_cost': local_result['cost'],
            'local_point_in_bounds': bool(local_result['in_bounds']),
            'tf_stamp_ns': self._last_tf_stamp_ns(),
            'tf_age_s': self._age_s(self._last_tf_stamp_ns(), now_ns),
            'odom_stamp_ns': self._last_odom_stamp_ns,
            'odom_age_s': self._age_s(self._last_odom_stamp_ns, now_ns),
            'scan_stamp_ns': self._last_scan_stamp_ns,
            'scan_age_s': self._age_s(self._last_scan_stamp_ns, now_ns),
        }

    def _last_tf_stamp_ns(self) -> int:
        """Read the latest shared-map-to-base transform stamp for telemetry."""
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame, self._base_frame, Time(),
                timeout=Duration(seconds=0.0),
            )
        except TransformException:
            return 0
        return _stamp_ns_static(transform.header.stamp)

    def _record_point_validation_sample(self, event: str, force: bool = False) -> None:
        """Append only meaningful bounded target-cost transitions."""
        record = self._navigation_point_validation
        if record is None:
            return
        sample = self._point_validation_sample()
        previous = record.get('_last_sample')
        changed = previous is None or any(
            sample.get(key) != previous.get(key)
            for key in (
                'global_point_status', 'global_point_cost', 'local_point_status',
                'local_point_cost', 'local_point_in_bounds',
            )
        )
        if not force and not changed:
            return
        if previous is not None and previous.get('local_point_status') == 'OUT_OF_LOCAL_WINDOW' \
                and sample.get('local_point_in_bounds'):
            event = 'TARGET_ENTERED_LOCAL_WINDOW'
        transition = {**sample, 'event': event}
        transitions = record.setdefault('transitions', [])
        if len(transitions) < 128:
            transitions.append(transition)
        record['_last_sample'] = sample

    def _emit_point_validation(self, phase: str, result: Optional[NavigationOutcome] = None) -> None:
        """Publish compact diagnostic-only dispatch/transition lifecycle data."""
        record = self._navigation_point_validation
        if record is None:
            return
        payload = {
            'schema_version': 'dispatch_point_validation.v1',
            'phase': phase,
            'robot': self._robot_id,
            'goal_number': record['goal_number'],
            'physical_task_signature': record['physical_task_signature'],
            'target': record['target'],
            'path_length_m': record['path_length_m'],
            'dispatch': record['dispatch'],
            'transitions': record.get('transitions', []),
        }
        if result is not None:
            payload['result'] = {
                'accepted': bool(result.accepted),
                'status': int(result.status),
                'error_code': int(result.error_code),
                'error_message': result.error_message,
                'failure_class': result.failure_class.value,
                'duration_s': result.duration_s,
                'travelled_distance_m': result.travelled_distance_m,
                'recoveries': result.recoveries,
                'follow_path_error_code': result.follow_path_error_code,
                'follow_path_error_name': result.follow_path_error_name,
                'controller_failure_family': result.controller_failure_family,
                'deepest_failure_classification': result.deepest_failure_classification,
            }
        self._node.get_logger().info(
            'NAVIGATION_POINT_VALIDATION %s' % json.dumps(
                payload, sort_keys=True, separators=(',', ':')),
        )

    def _failure_snapshot(self, error_code: int, error_message: str,
                          failure_class: FailureClass) -> str:
        """Serialize one bounded synchronized failure snapshot."""
        now_ns = self._node.get_clock().now().nanoseconds
        tf_stamp_ns = 0
        tf_age_s = None
        tf_error = ''
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame, self._base_frame, Time(),
                timeout=Duration(seconds=0.0),
            )
            tf_stamp_ns = self._stamp_ns(transform.header.stamp)
            if tf_stamp_ns:
                tf_age_s = max(0.0, (now_ns - tf_stamp_ns) / 1e9)
        except TransformException as error:
            tf_error = str(error)
        global_costmap_crop = bounded_grid_crop(
            self._costmap, self._navigation_target,
        )
        local_costmap_target_crop = bounded_grid_crop(
            self._local_costmap, self._navigation_target,
        )
        local_costmap_robot_pose = None
        local_costmap_robot_pose_error = ''
        local_costmap_robot_crop = {}
        if self._local_costmap is not None:
            local_frame = self._local_costmap.header.frame_id
            local_costmap_robot_pose = _transform_translation(
                self._tf_buffer, local_frame, self._base_frame)
            if local_costmap_robot_pose is None:
                local_costmap_robot_pose_error = (
                    f'robot pose unavailable in local costmap frame {local_frame!r}')
            local_costmap_robot_crop = bounded_grid_crop(
                self._local_costmap, local_costmap_robot_pose,
            )
        # Keep the legacy local_costmap_crop key robot-centered: it is the
        # rolling navigation window. The target-centered local crop is retained
        # separately because it is normally outside that window by design.
        local_costmap_crop = local_costmap_robot_crop
        signature_crop = local_costmap_robot_crop or local_costmap_target_crop or global_costmap_crop
        snapshot = {
            'schema_version': 'navigation_failure_snapshot.v1',
            'stamp_ros_ns': now_ns,
            'physical_task_signature': self._navigation_physical_signature,
            'target': self._navigation_target,
            'failure_class': failure_class.value,
            'navigate_to_pose_error_code': int(error_code),
            'navigate_to_pose_error_text': error_message,
            'map_stamp_ns': self._grid_stamp(self._map),
            'costmap_stamp_ns': self._grid_stamp(self._costmap),
            'global_costmap_stamp_ns': self._grid_stamp(self._costmap),
            'local_costmap_stamp_ns': self._grid_stamp(self._local_costmap),
            'tf_stamp_ns': tf_stamp_ns,
            'tf_age_s': tf_age_s,
            'tf_error': tf_error,
            'odom_stamp_ns': self._last_odom_stamp_ns,
            'odom_age_s': (
                None if not self._last_odom_stamp_ns else
                max(0.0, (now_ns - self._last_odom_stamp_ns) / 1e9)
            ),
            'odom_linear_x_mps': self._last_odom_linear[0],
            'odom_angular_z_radps': self._last_odom_angular_z,
            'scan_stamp_ns': self._last_scan_stamp_ns,
            'scan_age_s': (
                None if not self._last_scan_stamp_ns else
                max(0.0, (now_ns - self._last_scan_stamp_ns) / 1e9)
            ),
            'scan_min_range_m': self._last_scan_min_range,
            'cmd_capture': 'observer_only; allocator creates no cmd_vel interface',
            'recent_odom_samples': list(self._recent_odom_samples),
            'global_costmap_crop': global_costmap_crop,
            'local_costmap_crop': local_costmap_crop,
            'local_costmap_robot_pose': local_costmap_robot_pose,
            'local_costmap_robot_pose_frame': (
                self._local_costmap.header.frame_id
                if self._local_costmap is not None else ''),
            'local_costmap_robot_pose_error': local_costmap_robot_pose_error,
            'local_costmap_robot_crop': local_costmap_robot_crop,
            'local_costmap_target_crop': local_costmap_target_crop,
            'costmap_source_for_signature': (
                'local_costmap_robot_centered' if local_costmap_robot_crop else
                'local_costmap_target_fallback' if local_costmap_target_crop else
                'global_costmap_fallback' if global_costmap_crop else 'unavailable'
            ),
            # Keep the legacy key for readers of earlier diagnostic bundles.
            'costmap_crop': signature_crop,
            'validated_path_samples': [
                {'x': float(point[0]), 'y': float(point[1])}
                for point in self._navigation_diagnostic_path[:32]
            ],
            'validated_path_sample_count': len(self._navigation_diagnostic_path),
            'footprint_capture': 'Nav2 footprint configuration is external to allocator diagnostics',
        }
        snapshot['execution_geometry_signature'] = execution_geometry_signature(
            self._navigation_target, signature_crop,
        )
        return json.dumps(snapshot, sort_keys=True, separators=(',', ':'))

    def _pose(self, task: PhysicalTask):
        from geometry_msgs.msg import PoseStamped

        pose = PoseStamped()
        pose.header.frame_id = self._global_frame
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = task.approach
        pose.pose.orientation.z = math.sin(task.approach_yaw / 2.0)
        pose.pose.orientation.w = math.cos(task.approach_yaw / 2.0)
        return pose

    def evaluate_path(
            self, task: PhysicalTask,
            callback: Callable[[PathEvaluation], None],
            caller: str = 'ALLOCATOR_BID') -> bool:
        """Start one bounded local path request; return false if busy/unavailable."""
        if (self._path_callback is not None or
                not self._compute_client.server_is_ready() or
                not self._acquire_path_query_lock()):
            return False
        self._active_path_request += 1
        generation = self._active_path_request
        self._path_callback = callback
        self._path_deadline_steady_s = time.monotonic() + self._path_timeout_s
        self._path_started_steady_s = time.monotonic()
        self._path_started_ros_ns = self._node.get_clock().now().nanoseconds
        self._path_caller = str(caller)
        self._path_task_signature = task.physical_signature
        self._path_map_stamp_ns = self._grid_stamp(self._map)
        self._path_costmap_stamp_ns = self._grid_stamp(self._costmap)
        goal = ComputePathToPose.Goal()
        goal.goal = self._pose(task)
        goal.planner_id = self._planner_id
        goal.use_start = False
        future = self._compute_client.send_goal_async(goal)
        future.add_done_callback(
            lambda result: self._path_goal_response(generation, result),
        )
        return True

    def _path_goal_response(self, generation: int, future) -> None:
        if generation != self._active_path_request or self._path_callback is None:
            return
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._finish_path(PathEvaluation(
                False, 0.0, (), self._node.get_clock().now().nanoseconds,
                0, 'ComputePathToPose goal rejected', FailureClass.ACTION_REJECTION,
            ))
            return
        self._path_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._path_result(generation, result),
        )

    def _path_result(self, generation: int, future) -> None:
        if generation != self._active_path_request or self._path_callback is None:
            return
        wrapped = future.result()
        result = wrapped.result
        points = tuple(
            (pose.pose.position.x, pose.pose.position.y)
            for pose in result.path.poses
        ) if result is not None else ()
        valid = (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED and result is not None and
            result.error_code == ComputePathToPose.Result.NONE and bool(points)
        )
        error_code = 0 if result is None else result.error_code
        error_message = 'missing action result' if result is None else result.error_msg
        evidence = FailureEvidence()
        if error_code in (
                ComputePathToPose.Result.GOAL_OCCUPIED,
                ComputePathToPose.Result.GOAL_OUTSIDE_MAP,
                ComputePathToPose.Result.NO_VALID_PATH):
            evidence = FailureEvidence(compute_path_error='HARD_UNREACHABLE')
        elif error_code == ComputePathToPose.Result.TF_ERROR:
            evidence = FailureEvidence(tf_unavailable=True)
        elif error_code == ComputePathToPose.Result.TIMEOUT:
            evidence = FailureEvidence(timed_out=True)
        elif not valid and error_code != 0:
            evidence = FailureEvidence(compute_path_error='PLANNER_FAILURE')
        self._finish_path(PathEvaluation(
            valid=valid,
            length_m=path_length(points) if valid else 0.0,
            samples=downsample_path(points, self._maximum_path_samples),
            query_ros_ns=self._node.get_clock().now().nanoseconds,
            error_code=error_code,
            error_message=error_message,
            failure_class=FailureClass.UNKNOWN if valid else classify_failure(evidence),
            query_started_ros_ns=self._path_started_ros_ns,
            duration_s=max(0.0, time.monotonic() - self._path_started_steady_s),
            caller=self._path_caller,
            task_signature=self._path_task_signature,
        ))

    def _finish_path(self, result: PathEvaluation) -> None:
        if self._path_started_steady_s:
            result = PathEvaluation(
                **{**result.__dict__,
                   'query_started_ros_ns': self._path_started_ros_ns,
                   'duration_s': (
                       result.duration_s if result.duration_s > 0.0 else
                       max(0.0, time.monotonic() - self._path_started_steady_s)
                   ),
                   'caller': self._path_caller,
                   'task_signature': self._path_task_signature,
                   'map_stamp_ns': self._path_map_stamp_ns,
                   'costmap_stamp_ns': self._path_costmap_stamp_ns},
            )
        self._node.get_logger().info(
            'COMPUTE_PATH_RESULT source=%s task=%s valid=%s error_code=%d '
            'failure_class=%s duration_s=%.3f error=%r' % (
                result.caller, result.task_signature, result.valid,
                result.error_code, result.failure_class.value,
                result.duration_s, result.error_message,
            )
        )
        callback, self._path_callback = self._path_callback, None
        self._path_goal_handle = None
        self._release_path_query_lock()
        if callback is not None:
            callback(result)

    def _acquire_path_query_lock(self) -> bool:
        """Serialize this robot's planner action with the C++ candidate node."""
        if self._path_query_lock_file is not None:
            return True
        try:
            handle = open(self._path_query_lock_path, 'a+')
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (OSError, IOError):
            try:
                handle.close()
            except UnboundLocalError:
                pass
            return False
        self._path_query_lock_file = handle
        return True

    def _release_path_query_lock(self) -> None:
        """Release the bounded per-robot planner lease, if held."""
        handle, self._path_query_lock_file = self._path_query_lock_file, None
        if handle is None:
            return
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        finally:
            handle.close()

    def check_dispatch_preconditions(
            self, task: PhysicalTask, final_path_valid: bool,
            callback: Callable[[DispatchPreconditions], None]) -> None:
        """Asynchronously confirm local lifecycle plus map, costmap, and TF context."""
        base = self._basic_preconditions(task, final_path_valid)
        if base.reason:
            callback(base)
            return
        unavailable = [name for name, client in self._lifecycle_clients.items()
                       if not client.service_is_ready()]
        if unavailable:
            callback(DispatchPreconditions(
                **{**base.__dict__, 'lifecycle_active': False,
                   'reason': 'lifecycle services unavailable: ' + ','.join(unavailable)},
            ))
            return
        states = {}

        def completed(name, future):
            try:
                states[name] = future.result().current_state.label
            except Exception as error:  # noqa: B902
                states[name] = 'error:' + str(error)
            if len(states) != len(self._lifecycle_clients):
                return
            active = all(value == 'active' for value in states.values())
            self._last_lifecycle_active = active
            callback(DispatchPreconditions(
                **{**base.__dict__, 'lifecycle_active': active,
                   'reason': '' if active else 'inactive lifecycle nodes: ' + str(states)},
            ))

        for name, client in self._lifecycle_clients.items():
            future = client.call_async(GetState.Request())
            future.add_done_callback(lambda result, key=name: completed(key, result))

    def _basic_preconditions(
            self, task: PhysicalTask, final_path_valid: bool) -> DispatchPreconditions:
        map_value = None if self._map is None else occupancy_value(self._map, task.approach)
        cost_value = (
            None if self._costmap is None else occupancy_value(self._costmap, task.approach)
        )
        inside_map = map_value is not None
        inside_costmap = cost_value is not None
        reason = ''
        transform_available = False
        transform_age_s = None
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame, self._base_frame, Time(), timeout=Duration(seconds=0.1),
            )
            transform_available = True
            stamp = transform.header.stamp
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            if stamp_ns > 0:
                transform_age_s = max(
                    0.0, (self._node.get_clock().now().nanoseconds - stamp_ns) / 1e9,
                )
                if transform_age_s > self._maximum_tf_age_s:
                    transform_available = False
                    reason = 'shared_map to base transform is stale'
        except TransformException as error:
            reason = 'required transform unavailable: ' + str(error)
        if not inside_map:
            reason = reason or 'goal lies outside current shared map'
        elif map_value >= 50:
            reason = reason or 'goal occupancy-map cell is not free'
        if not inside_costmap:
            reason = reason or 'goal lies outside current global costmap'
        elif cost_value < 0 or cost_value >= self._costmap_lethal_threshold:
            reason = reason or 'goal costmap cell is unknown or lethal'
        if self.local_goal_active:
            reason = reason or 'another local navigation goal is active'
        if not final_path_valid:
            reason = reason or 'final local ComputePathToPose validation failed'
        if not self._navigate_client.server_is_ready():
            reason = reason or 'local NavigateToPose action server unavailable'
        return DispatchPreconditions(
            action_server_ready=self._navigate_client.server_is_ready(),
            lifecycle_active=False,
            transform_available=transform_available,
            transform_age_s=transform_age_s,
            goal_inside_map=inside_map,
            goal_inside_costmap=inside_costmap,
            goal_map_value=map_value,
            goal_costmap_value=cost_value,
            no_local_goal_active=not self.local_goal_active,
            final_path_valid=final_path_valid,
            reason=reason,
        )

    def send_navigation(
            self, task: PhysicalTask,
            callback: Callable[[NavigationOutcome], None],
            diagnostic_path: tuple[tuple[float, float], ...] = ()) -> bool:
        """Send one already validated goal to this namespace's navigator only."""
        if self.local_goal_active or not self._navigate_client.server_is_ready():
            return False
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(task)
        self._navigation_callback = callback
        self._navigation_started_steady_s = time.monotonic()
        self._navigation_start_distance_m = self.travelled_distance_m
        self._navigation_last_progress_distance_m = self.travelled_distance_m
        self._navigation_last_progress_steady_s = self._navigation_started_steady_s
        self._navigation_recoveries = 0
        self._navigation_timeout_requested = False
        self._navigation_no_progress_requested = False
        self._navigation_no_progress_requested_ros_ns = 0
        self._navigation_cancel_requested = False
        self._navigation_target = task.approach
        self._navigation_physical_signature = task.physical_signature
        self._navigation_diagnostic_path = tuple(diagnostic_path)
        self._navigation_goal_number += 1
        self._navigation_point_validation = {
            'goal_number': self._navigation_goal_number,
            'physical_task_signature': task.physical_signature,
            'target': [float(task.approach[0]), float(task.approach[1])],
            'path_length_m': path_length(tuple(diagnostic_path)),
            'dispatch': self._point_validation_sample(),
            'transitions': [],
        }
        self._navigation_point_validation['_last_sample'] = (
            self._navigation_point_validation['dispatch'])
        self._navigation_point_last_sample_steady_s = time.monotonic()
        self._emit_point_validation('DISPATCH')
        self._navigation_send_pending = True
        future = self._navigate_client.send_goal_async(
            goal, feedback_callback=self._navigation_feedback,
        )
        future.add_done_callback(self._navigation_goal_response)
        return True

    def _navigation_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # noqa: B902
            self._navigation_send_pending = False
            self._finish_navigation(NavigationOutcome(
                False, GoalStatus.STATUS_UNKNOWN, 0,
                'NavigateToPose goal response exception: ' + str(error),
                FailureClass.UNKNOWN,
                time.monotonic() - self._navigation_started_steady_s,
                self.travelled_distance_m - self._navigation_start_distance_m,
                self._navigation_recoveries,
            ))
            return
        self._navigation_send_pending = False
        if goal_handle is None or not goal_handle.accepted:
            self._finish_navigation(NavigationOutcome(
                False, GoalStatus.STATUS_UNKNOWN, 0, 'NavigateToPose goal rejected',
                FailureClass.ACTION_REJECTION,
                time.monotonic() - self._navigation_started_steady_s,
                self.travelled_distance_m - self._navigation_start_distance_m,
                self._navigation_recoveries,
            ))
            return
        self._navigation_goal_handle = goal_handle
        if self._navigation_cancel_requested:
            goal_handle.cancel_goal_async()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._navigation_result)

    def _navigation_feedback(self, feedback) -> None:
        self._navigation_recoveries = max(
            self._navigation_recoveries,
            int(feedback.feedback.number_of_recoveries),
        )

    def cancel_navigation(self) -> bool:
        """Request explicit cancellation of this namespace's active goal."""
        if not self.local_goal_active:
            return False
        self._navigation_cancel_requested = True
        if self._navigation_goal_handle is not None:
            self._navigation_goal_handle.cancel_goal_async()
        return True

    def _navigation_result(self, future) -> None:
        try:
            wrapped = future.result()
        except Exception as error:  # noqa: B902
            self._finish_navigation(NavigationOutcome(
                True, GoalStatus.STATUS_UNKNOWN, 0,
                'NavigateToPose result exception: ' + str(error),
                FailureClass.UNKNOWN,
                time.monotonic() - self._navigation_started_steady_s,
                max(0.0, self.travelled_distance_m - self._navigation_start_distance_m),
                self._navigation_recoveries,
            ))
            return
        result = wrapped.result
        error_code = 0 if result is None else result.error_code
        error_message = 'missing action result' if result is None else result.error_msg
        succeeded = (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED and
            result is not None and error_code == NavigateToPose.Result.NONE
        )
        evidence = FailureEvidence()
        if self._navigation_timeout_requested:
            evidence = FailureEvidence(timed_out=True)
        elif self._navigation_no_progress_requested:
            evidence = FailureEvidence(controller_no_progress=True)
        elif self._navigation_cancel_requested or wrapped.status == GoalStatus.STATUS_CANCELED:
            evidence = FailureEvidence(explicitly_cancelled=True)
        elif classify_follow_path_controller_error(error_code) == FailureClass.TF_OR_LIFECYCLE:
            evidence = FailureEvidence(tf_unavailable=True)
        elif classify_follow_path_controller_error(error_code) == FailureClass.CONTROLLER_NO_PROGRESS:
            evidence = FailureEvidence(controller_no_progress=True)
        error_name = follow_path_controller_error_name(error_code)
        if self._navigation_no_progress_requested and not error_message:
            error_message = 'local controller no-progress timeout'
        failure_class = FailureClass.UNKNOWN if succeeded else classify_failure(evidence)
        deepest_classification = ''
        controller_failure_family = ''
        deepest_stamp = 0
        if not succeeded:
            deepest_stamp = self._navigation_no_progress_requested_ros_ns or (
                self._node.get_clock().now().nanoseconds)
            if self._navigation_no_progress_requested:
                deepest_classification = 'CONTROLLER_EXECUTION_NO_PROGRESS'
                controller_failure_family = 'CONTROLLER_EXECUTION_NO_PROGRESS'
            elif error_code in FOLLOW_PATH_TF_FAILURE_CODES:
                deepest_classification = error_name or 'TF_ERROR'
                controller_failure_family = 'NAV_INFRASTRUCTURE_TF'
            elif error_code in FOLLOW_PATH_CONTROLLER_FAILURE_CODES:
                deepest_classification = error_name or 'FOLLOW_PATH_CONTROLLER_FAILURE'
                controller_failure_family = 'CONTROLLER_EXECUTION'
            else:
                deepest_classification = 'NAV2_ABORT_CAUSE_UNAVAILABLE'
        snapshot_json = ''
        if not succeeded:
            snapshot_json = self._failure_snapshot(error_code, error_message, failure_class)
        self._finish_navigation(NavigationOutcome(
            True, wrapped.status, error_code, error_message,
            failure_class,
            time.monotonic() - self._navigation_started_steady_s,
            max(0.0, self.travelled_distance_m - self._navigation_start_distance_m),
            self._navigation_recoveries,
            error_name,
            error_code if error_code in (
                FOLLOW_PATH_TF_FAILURE_CODES | FOLLOW_PATH_CONTROLLER_FAILURE_CODES
            ) else 0,
            error_name if error_code in (
                FOLLOW_PATH_TF_FAILURE_CODES | FOLLOW_PATH_CONTROLLER_FAILURE_CODES
            ) else '',
            controller_failure_family,
            deepest_classification,
            deepest_stamp,
            snapshot_json,
        ))

    def _finish_navigation(self, result: NavigationOutcome) -> None:
        self._record_point_validation_sample('FAILURE_OR_SUCCESS', force=True)
        self._emit_point_validation('FINAL', result)
        callback, self._navigation_callback = self._navigation_callback, None
        self._navigation_goal_handle = None
        self._navigation_send_pending = False
        self._navigation_target = None
        self._navigation_physical_signature = ''
        self._navigation_diagnostic_path = ()
        self._navigation_point_validation = None
        self._navigation_point_last_sample_steady_s = 0.0
        if callback is not None:
            callback(result)

    def _check_timeouts(self) -> None:
        now = time.monotonic()
        if (self._navigation_point_validation is not None and
                now - self._navigation_point_last_sample_steady_s >=
                self._navigation_point_sample_period_s):
            self._record_point_validation_sample('POINT_STATUS_CHANGE')
            self._navigation_point_last_sample_steady_s = now
        if self._path_callback is not None and now > self._path_deadline_steady_s:
            if self._path_goal_handle is not None:
                self._path_goal_handle.cancel_goal_async()
            self._active_path_request += 1
            self._finish_path(PathEvaluation(
                False, 0.0, (), self._node.get_clock().now().nanoseconds,
                ComputePathToPose.Result.TIMEOUT, 'local path query timeout',
                FailureClass.TIMEOUT,
            ))
        if (self._navigation_goal_handle is not None and
                not self._navigation_timeout_requested and
                now - self._navigation_started_steady_s > self._navigation_timeout_s):
            self._navigation_timeout_requested = True
            self._navigation_goal_handle.cancel_goal_async()
        if (self._navigation_goal_handle is not None and
                not self._navigation_no_progress_requested and
                now - self._navigation_last_progress_steady_s >
                self._navigation_no_progress_timeout_s):
            self._navigation_no_progress_requested = True
            self._navigation_no_progress_requested_ros_ns = (
                self._node.get_clock().now().nanoseconds
            )
            self._navigation_goal_handle.cancel_goal_async()
