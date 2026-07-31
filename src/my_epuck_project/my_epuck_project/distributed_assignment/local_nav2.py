"""Small namespace-local Nav2 path evaluation and execution boundary."""

import math
import time
from dataclasses import dataclass
from typing import Callable, Optional

from action_msgs.msg import GoalStatus

from lifecycle_msgs.srv import GetState

from nav2_msgs.action import ComputePathToPose, NavigateToPose

from nav_msgs.msg import OccupancyGrid, Odometry

from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from tf2_ros import Buffer, TransformException, TransformListener

from .failures import classify_failure
from .models import FailureClass, FailureEvidence, PhysicalTask, Point, TravelDistance


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

    def __init__(self, node: Node):
        """Create relative interfaces that resolve inside the local robot namespace."""
        self._node = node
        self._global_frame = node.declare_parameter('global_frame', 'shared_map').value
        self._base_frame = node.declare_parameter('robot_base_frame', 'base_footprint').value
        self._planner_id = node.declare_parameter('planner_id', 'GridBased').value
        self._path_timeout_s = float(node.declare_parameter('path_query_timeout_s', 1.5).value)
        self._navigation_timeout_s = float(
            node.declare_parameter('navigation_timeout_s', 240.0).value,
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
        self._compute_client = ActionClient(
            node, ComputePathToPose, 'compute_path_to_pose',
        )
        self._navigate_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self._lifecycle_clients = {
            name: node.create_client(GetState, f'{name}/get_state')
            for name in ('planner_server', 'controller_server', 'bt_navigator')
        }
        transient_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(
            OccupancyGrid, 'shared_map', self._on_map, transient_qos,
        )
        node.create_subscription(
            OccupancyGrid, 'global_costmap/costmap', self._on_costmap, transient_qos,
        )
        node.create_subscription(Odometry, 'odom', self._on_odom, 20)
        self._tf_buffer = Buffer(node=node)
        self._tf_listener = TransformListener(self._tf_buffer, node, spin_thread=False)
        self._map: Optional[OccupancyGrid] = None
        self._costmap: Optional[OccupancyGrid] = None
        self._distance = TravelDistance(maximum_step_m=0.5)
        self._active_path_request = 0
        self._path_deadline_steady_s = 0.0
        self._path_callback: Optional[Callable[[PathEvaluation], None]] = None
        self._path_goal_handle = None
        self._navigation_goal_handle = None
        self._navigation_callback: Optional[Callable[[NavigationOutcome], None]] = None
        self._navigation_started_steady_s = 0.0
        self._navigation_start_distance_m = 0.0
        self._navigation_recoveries = 0
        self._navigation_timeout_requested = False
        self._timer = node.create_timer(0.1, self._check_timeouts)

    @property
    def local_goal_active(self) -> bool:
        """Return whether this wrapper owns an unresolved local navigation goal."""
        return self._navigation_goal_handle is not None

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

    def _on_map(self, message: OccupancyGrid) -> None:
        self._map = message

    def _on_costmap(self, message: OccupancyGrid) -> None:
        self._costmap = message

    def _on_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self._distance.observe((position.x, position.y))

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
            callback: Callable[[PathEvaluation], None]) -> bool:
        """Start one bounded local path request; return false if busy/unavailable."""
        if self._path_callback is not None or not self._compute_client.server_is_ready():
            return False
        self._active_path_request += 1
        generation = self._active_path_request
        self._path_callback = callback
        self._path_deadline_steady_s = time.monotonic() + self._path_timeout_s
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
        ))

    def _finish_path(self, result: PathEvaluation) -> None:
        callback, self._path_callback = self._path_callback, None
        self._path_goal_handle = None
        if callback is not None:
            callback(result)

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
            callback: Callable[[NavigationOutcome], None]) -> bool:
        """Send one already validated goal to this namespace's navigator only."""
        if self.local_goal_active or not self._navigate_client.server_is_ready():
            return False
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(task)
        self._navigation_callback = callback
        self._navigation_started_steady_s = time.monotonic()
        self._navigation_start_distance_m = self.travelled_distance_m
        self._navigation_recoveries = 0
        self._navigation_timeout_requested = False
        future = self._navigate_client.send_goal_async(
            goal, feedback_callback=self._navigation_feedback,
        )
        future.add_done_callback(self._navigation_goal_response)
        return True

    def _navigation_goal_response(self, future) -> None:
        goal_handle = future.result()
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
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._navigation_result)

    def _navigation_feedback(self, feedback) -> None:
        self._navigation_recoveries = max(
            self._navigation_recoveries,
            int(feedback.feedback.number_of_recoveries),
        )

    def _navigation_result(self, future) -> None:
        wrapped = future.result()
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
        elif wrapped.status == GoalStatus.STATUS_CANCELED:
            evidence = FailureEvidence(explicitly_cancelled=True)
        self._finish_navigation(NavigationOutcome(
            True, wrapped.status, error_code, error_message,
            FailureClass.UNKNOWN if succeeded else classify_failure(evidence),
            time.monotonic() - self._navigation_started_steady_s,
            max(0.0, self.travelled_distance_m - self._navigation_start_distance_m),
            self._navigation_recoveries,
        ))

    def _finish_navigation(self, result: NavigationOutcome) -> None:
        callback, self._navigation_callback = self._navigation_callback, None
        self._navigation_goal_handle = None
        if callback is not None:
            callback(result)

    def _check_timeouts(self) -> None:
        now = time.monotonic()
        if self._path_callback is not None and now > self._path_deadline_steady_s:
            if self._path_goal_handle is not None:
                self._path_goal_handle.cancel_goal_async()
            self._active_path_request += 1
            self._finish_path(PathEvaluation(
                False, 0.0, (), self._node.get_clock().now().nanoseconds,
                ComputePathToPose.Result.TIMEOUT, 'local path query timeout',
                FailureClass.TIMEOUT,
            ))
        if (self.local_goal_active and not self._navigation_timeout_requested and
                now - self._navigation_started_steady_s > self._navigation_timeout_s):
            self._navigation_timeout_requested = True
            self._navigation_goal_handle.cancel_goal_async()
