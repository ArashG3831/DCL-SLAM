"""
Lean allocator-free Nav2 and frontier handoff diagnostic.

The ROS node in this module owns every NavigateToPose request made by the
diagnostic launch.  It deliberately consumes frontier candidate messages as
data and never imports the distributed allocator or its scoring utilities.
The source-tree runner lives here as well so it can share bounded cleanup and
argument validation without changing the established regression runners.
"""

from __future__ import annotations

import argparse
from collections import Counter, deque
import csv
from dataclasses import dataclass, field
from datetime import datetime, timezone
import fcntl
from functools import partial
import json
import math
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from typing import Callable, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from my_epuck_interfaces.msg import FrontierCandidateArray
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
import psutil
from rcl_interfaces.msg import Log
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock as RclClock, ClockType
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from .cooperative_profiles import profile


WORKSPACE = Path('/home/arash/webots_ws')
PACKAGE = 'my_epuck_project'
LAUNCH_FILE = 'two_robots_nav2_frontier_diagnostic_launch.py'
ARTIFACT_NAMES = frozenset({
    'launch.log', 'diagnostic_events.jsonl',
    'diagnostic_timeseries.csv', 'diagnostic_summary.json',
    'effective_command.txt', 'handoff_rejections.csv',
})
ROBOTS = ('robot1', 'robot2')
NAV2_NODES = (
    'controller_server', 'smoother_server', 'planner_server',
    'route_server', 'behavior_server', 'velocity_smoother',
    'collision_monitor', 'bt_navigator', 'waypoint_follower',
)
MISSION_PHASES = (
    (0.0, 60.0, 'FILTER_AND_MAPPING'),
    (60.0, 300.0, 'NAV2_ONLY'),
    (300.0, 450.0, 'FRONTIER_GENERATION_ONLY'),
    (450.0, 600.0, 'FRONTIER_TO_NAV2'),
)
TIME_STATES = (
    'startup/readiness',
    'intentionally stationary filter test',
    'waiting for candidates',
    'waiting for planner',
    'waiting for goal acceptance',
    'active navigation with nonzero cmd_vel',
    'active goal with zero cmd_vel',
    'recovery',
    'post-goal settling',
    'idle despite valid candidates',
    'map/TF unavailable',
)
WAIT_CLASSIFICATIONS = (
    'WAITING_FOR_ACTION_SERVER', 'WAITING_FOR_PLANNER',
    'WAITING_FOR_CONTROLLER', 'ZERO_COMMAND_WITH_ACTIVE_GOAL',
    'RECOVERY', 'TF_UNAVAILABLE', 'COSTMAP_UNAVAILABLE',
    'GOAL_REJECTED', 'NAVIGATING', 'SETTLING',
)


def utc_now() -> str:
    """Return a stable UTC timestamp for artifacts."""
    return datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')


def stamp_seconds(stamp) -> float:
    """Convert a builtin_interfaces Time-like message to seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def duration_seconds(duration) -> float:
    """Convert a builtin_interfaces Duration-like message to seconds."""
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def path_length(path: NavPath) -> float:
    """Measure a finite planar ROS path, returning infinity if malformed."""
    if not path.poses:
        return math.inf
    total = 0.0
    previous = path.poses[0].pose.position
    if not math.isfinite(previous.x) or not math.isfinite(previous.y):
        return math.inf
    for stamped in path.poses[1:]:
        point = stamped.pose.position
        if not math.isfinite(point.x) or not math.isfinite(point.y):
            return math.inf
        total += math.hypot(point.x - previous.x, point.y - previous.y)
        previous = point
    return total


PHASE_BOUNDARIES = {
    'full': (0.0, 60.0, 300.0, 450.0, 600.0),
    'short': (0.0, 20.0, 100.0, 140.0, 180.0),
}


def phase_at(elapsed: float, mission_duration: float = 600.0,
             profile: str = 'full') -> str:
    """Return the named phase for readiness-relative simulation time."""
    del mission_duration
    try:
        _, filter_end, nav_end, candidate_end, _ = PHASE_BOUNDARIES[profile]
    except KeyError as error:
        raise ValueError(f'unknown phase profile: {profile}') from error
    if elapsed < filter_end:
        return 'FILTER_AND_MAPPING'
    if elapsed < nav_end:
        return 'NAV2_ONLY'
    if elapsed < candidate_end:
        return 'FRONTIER_GENERATION_ONLY'
    return 'FRONTIER_TO_NAV2'


def deterministic_candidate_key(candidate) -> tuple:
    """Rank only by valid path, gain, path length, then task ID."""
    reachable = (
        candidate.reachability_state == candidate.REACHABLE
        and math.isfinite(candidate.path_length_m)
        and candidate.path_length_m >= 0.0
    )
    gain = candidate.information_gain
    if not math.isfinite(gain) or gain < 0.0:
        gain = -math.inf
    length = candidate.path_length_m
    if not math.isfinite(length):
        length = math.inf
    return (not reachable, -gain, length, int(candidate.frontier_id))


def grid_cell(grid: OccupancyGrid, x: float, y: float):
    """Return (index, mx, my) for a world point in a possibly rotated grid."""
    origin = grid.info.origin
    quaternion = origin.orientation
    yaw = math.atan2(
        2.0 * (quaternion.w * quaternion.z
               + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y
                     + quaternion.z * quaternion.z),
    )
    dx, dy = x - origin.position.x, y - origin.position.y
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    if grid.info.resolution <= 0.0:
        return None
    mx = int(math.floor(local_x / grid.info.resolution))
    my = int(math.floor(local_y / grid.info.resolution))
    if mx < 0 or my < 0 or mx >= grid.info.width or my >= grid.info.height:
        return None
    index = my * grid.info.width + mx
    if index >= len(grid.data):
        return None
    return index, mx, my


def grid_value(grid: Optional[OccupancyGrid], x: float, y: float):
    """Return an occupancy/cost value or None outside the current grid."""
    if grid is None:
        return None
    cell = grid_cell(grid, x, y)
    return None if cell is None else int(grid.data[cell[0]])


def classify_start_cell(costmap: Optional[OccupancyGrid], x: float, y: float):
    """Classify a robot start for the bounded planner gate."""
    if costmap is None or grid_cell(costmap, x, y) is None:
        return 'OUTSIDE', None
    value = grid_value(costmap, x, y)
    if value is None or value < 0:
        return 'UNKNOWN', value
    if value == 0:
        return 'FREE', value
    if value >= 100:
        return 'LETHAL', value
    if value >= 99:
        return 'INSCRIBED', value
    return 'INFLATED', value


def known_cell_count(grid: Optional[OccupancyGrid]) -> int:
    """Count known cells in an OccupancyGrid."""
    return 0 if grid is None else sum(value >= 0 for value in grid.data)


def nearest_blocked_distance(
        grid: OccupancyGrid, x: float, y: float,
        maximum: float = 0.5, blocked_threshold: int = 1) -> float:
    """Measure local clearance from blocked or unknown costmap cells."""
    cell = grid_cell(grid, x, y)
    if cell is None:
        return 0.0
    _, cx, cy = cell
    resolution = grid.info.resolution
    radius = max(1, int(math.ceil(maximum / resolution)))
    best = maximum
    for my in range(max(0, cy - radius), min(grid.info.height, cy + radius + 1)):
        for mx in range(max(0, cx - radius), min(grid.info.width, cx + radius + 1)):
            value = int(grid.data[my * grid.info.width + mx])
            if value < 0 or value >= blocked_threshold:
                best = min(best, math.hypot(mx - cx, my - cy) * resolution)
    return best


def nearest_occupied_distance(
        grid: OccupancyGrid, x: float, y: float,
        maximum: float = 0.5, occupied_threshold: int = 50) -> float:
    """Measure clearance from known occupied cells, ignoring unknown space."""
    cell = grid_cell(grid, x, y)
    if cell is None:
        return 0.0
    _, cx, cy = cell
    resolution = grid.info.resolution
    radius = max(1, int(math.ceil(maximum / resolution)))
    best = maximum
    for my in range(max(0, cy - radius),
                    min(grid.info.height, cy + radius + 1)):
        for mx in range(max(0, cx - radius),
                        min(grid.info.width, cx + radius + 1)):
            value = int(grid.data[my * grid.info.width + mx])
            if value >= occupied_threshold:
                best = min(
                    best, math.hypot(mx - cx, my - cy) * resolution)
    return best


def nearest_occupied_cell(grid: Optional[OccupancyGrid], x: float, y: float,
                          maximum: float = 0.5,
                          occupied_threshold: int = 50):
    """Return the nearest occupied cell coordinates and distance."""
    if grid is None:
        return None
    cell = grid_cell(grid, x, y)
    if cell is None:
        return None
    _, cx, cy = cell
    resolution = grid.info.resolution
    radius = max(1, int(math.ceil(maximum / resolution)))
    best = None
    for my in range(max(0, cy - radius),
                    min(grid.info.height, cy + radius + 1)):
        for mx in range(max(0, cx - radius),
                        min(grid.info.width, cx + radius + 1)):
            value = int(grid.data[my * grid.info.width + mx])
            if value < occupied_threshold:
                continue
            distance = math.hypot(mx - cx, my - cy) * resolution
            if best is None or distance < best['distance_m']:
                best = {
                    'column': mx, 'row': my, 'value': value,
                    'distance_m': distance,
                    'x_m': grid.info.origin.position.x
                    + (mx + 0.5) * resolution,
                    'y_m': grid.info.origin.position.y
                    + (my + 0.5) * resolution,
                }
    return best


def line_point_distance(ax, ay, bx, by, px, py) -> float:
    """Return distance from a point to a finite line segment."""
    dx, dy = bx - ax, by - ay
    denominator = dx * dx + dy * dy
    if denominator <= 1e-12:
        return math.hypot(px - ax, py - ay)
    fraction = max(
        0.0, min(1.0,
                 ((px - ax) * dx + (py - ay) * dy) / denominator))
    return math.hypot(px - (ax + fraction * dx),
                      py - (ay + fraction * dy))


def candidate_physical_signature(candidate) -> str:
    """Stable geometric identity; frontier IDs can change between snapshots."""
    pose = candidate.approach_pose.pose.position
    centroid = candidate.centroid
    # Three centimetres is below a costmap cell in this setup, while avoiding
    # meaningless identity churn from floating-point serialization.
    return ':'.join(str(round(value, 2)) for value in (
        centroid.x, centroid.y, pose.x, pose.y,
        candidate.bounding_box_min.x, candidate.bounding_box_min.y,
        candidate.bounding_box_max.x, candidate.bounding_box_max.y))


def region_fingerprint(grid: Optional[OccupancyGrid], points,
                       margin_m: float = 0.12):
    """Return the cells relevant to a path/goal, not a whole-map revision."""
    if grid is None or grid.info.resolution <= 0.0:
        return None
    step = grid.info.resolution
    offsets = (-margin_m, 0.0, margin_m)
    samples = []
    for x, y in points:
        for dx in offsets:
            for dy in offsets:
                cell = grid_cell(grid, x + dx, y + dy)
                samples.append(None if cell is None else int(grid.data[cell[0]]))
    return tuple(samples)


def map_change_classification(source_revision, current_revision,
                              source_region, current_region) -> str:
    """Classify revision churn without treating remote mapping as unsafe."""
    if source_revision == current_revision:
        return 'MAP_REVISION_UNCHANGED'
    if source_region is not None and source_region == current_region:
        return 'REVISION_CHANGED_BUT_PATH_REGION_UNCHANGED'
    return 'RELEVANT_MAP_CHANGE'


@dataclass
class PlannerRequest:
    """One serialized ComputePathToPose request."""

    purpose: str
    pose: PoseStamped
    requested_sim: float
    requested_wall: float
    metadata: dict
    callback: Callable
    lock_file: object
    goal_handle: object = None


@dataclass
class NavigationRun:
    """High-rate internal metrics for one NavigateToPose request."""

    robot: str
    kind: str
    label: str
    pose: PoseStamped
    path_length_m: float
    path_latency_s: float
    request_sim: float
    request_wall: float
    metadata: dict
    start_pose: tuple
    start_cost: Optional[int]
    goal_cost: Optional[int]
    clearance_m: float
    map_age_s: Optional[float]
    costmap_age_s: Optional[float]
    tf_age_s: Optional[float]
    accepted_sim: Optional[float] = None
    first_motion_sim: Optional[float] = None
    terminal_sim: Optional[float] = None
    goal_handle: object = None
    cmd_count: int = 0
    nonzero_cmd_count: int = 0
    zero_feedback_intervals: int = 0
    feedback_intervals: int = 0
    recovery_count: int = 0
    maximum_recovery_count: int = 0
    odom_distance_m: float = 0.0
    last_odom: Optional[tuple] = None
    result_status: int = GoalStatus.STATUS_UNKNOWN
    result_code: int = -1
    result_message: str = ''
    final_pose_error_m: Optional[float] = None
    coverage_start_cells: int = 0
    coverage_end_cells: Optional[int] = None
    next_candidate_delay_s: Optional[float] = None
    cancel_requested: bool = False
    classifications: Counter = field(default_factory=Counter)


class DiagnosticNode(Node):
    """Readiness gate, four-phase scheduler, and lightweight recorder."""

    FILTER_MIN_HZ = 4.0
    FRESH_SCAN_S = 1.0
    FRESH_MAP_S = 5.0
    FRESH_COSTMAP_S = 3.0
    PLANNER_HIGH_S = 2.0
    CONTROLLER_MIN_HZ = 5.0
    ZERO_COMMAND_WARN_S = 5.0
    FRONTIER_STALE_S = 5.0
    FRONTIER_SLOW_S = 5.0

    def __init__(self, **kwargs):
        """Create the diagnostic with optional standard rclpy node options."""
        super().__init__('nav2_frontier_diagnostic', **kwargs)
        self.declare_parameter('output_directory', '')
        self.declare_parameter('mission_duration_s', 600.0)
        self.declare_parameter('startup_timeout_s', 300.0)
        self.declare_parameter('world_profile', 'large')
        self.declare_parameter('phase_profile', 'full')
        output = self.get_parameter('output_directory').value
        if not output:
            raise ValueError('output_directory must be configured')
        self.output = Path(output).resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        self.mission_duration = float(
            self.get_parameter('mission_duration_s').value)
        self.startup_timeout = float(
            self.get_parameter('startup_timeout_s').value)
        self.world_profile = str(self.get_parameter('world_profile').value)
        self.phase_profile = str(self.get_parameter('phase_profile').value)
        if self.phase_profile not in PHASE_BOUNDARIES:
            raise ValueError(f'unknown phase profile: {self.phase_profile}')
        self.started_wall = time.monotonic()
        self.started_utc = utc_now()
        self.ready_sim = None
        self.ready_wall = None
        self.ready_utc = None
        self.completed = False
        self.exit_code = 0
        self.last_phase = 'READINESS'
        self.clock_samples = deque(maxlen=5)
        self.latest_clock = 0.0
        self.last_received = {robot: {} for robot in ROBOTS}
        self.messages = {robot: Counter() for robot in ROBOTS}
        self.phase_counts = {robot: Counter() for robot in ROBOTS}
        self.maps = {robot: None for robot in ROBOTS}
        self.shared_maps = {robot: None for robot in ROBOTS}
        self.costmaps = {robot: None for robot in ROBOTS}
        self.local_costmaps = {robot: None for robot in ROBOTS}
        self.odom = {robot: None for robot in ROBOTS}
        self.poses = {robot: None for robot in ROBOTS}
        self.cmd = {robot: {'nonzero': False, 'last': 0.0} for robot in ROBOTS}
        self.candidates = {robot: None for robot in ROBOTS}
        self.candidate_received_sim = {robot: None for robot in ROBOTS}
        self.candidate_stats = {robot: Counter() for robot in ROBOTS}
        self.candidate_intervals = {robot: [] for robot in ROBOTS}
        self.candidate_last_signature = {robot: None for robot in ROBOTS}
        self.candidate_generation = {robot: [] for robot in ROBOTS}
        self.candidate_cycle_started = {robot: None for robot in ROBOTS}
        self.candidate_cycle_durations = {robot: [] for robot in ROBOTS}
        self.candidate_gain_records = {robot: [] for robot in ROBOTS}
        self.candidate_invalid_reasons = {
            robot: Counter() for robot in ROBOTS}
        self.candidate_cpu_start = {robot: None for robot in ROBOTS}
        self.candidate_cpu_end = {robot: None for robot in ROBOTS}
        self.frontier_pid = {robot: None for robot in ROBOTS}
        self.filter_stats = {robot: Counter() for robot in ROBOTS}
        self.filter_observations = {robot: Counter() for robot in ROBOTS}
        self.fixed_scans = {robot: {} for robot in ROBOTS}
        self.scan_times = {robot: {'fixed': [], 'slam': []} for robot in ROBOTS}
        self.scan_latencies = {robot: [] for robot in ROBOTS}
        self.map_times = {robot: [] for robot in ROBOTS}
        self.stationary_known_start = {robot: None for robot in ROBOTS}
        self.stationary_known_end = {robot: None for robot in ROBOTS}
        self.lifecycle = {
            robot: {name: False for name in NAV2_NODES} for robot in ROBOTS}
        self.lifecycle_pending = set()
        self.readiness_report = {}
        self.tf_failures = Counter()
        self.planner_requests = {robot: None for robot in ROBOTS}
        self.planner_callback_active = {robot: False for robot in ROBOTS}
        self.planner_failures = {robot: 0 for robot in ROBOTS}
        self.candidate_failure_cycles = {robot: 0 for robot in ROBOTS}
        self.planner_failure_storm = {robot: False for robot in ROBOTS}
        self.planner_busy_since = {robot: None for robot in ROBOTS}
        self.nav_runs = []
        self.active_nav = {robot: None for robot in ROBOTS}
        self.last_terminal_sim = {robot: None for robot in ROBOTS}
        self.manual_steps = (
            [
                ('robot1', 'short', False), ('robot2', 'short', False),
            ] if self.phase_profile == 'short' else [
                ('robot1', 'short', False), ('robot2', 'short', False),
                ('robot1', 'medium', False), ('robot2', 'medium', False),
                ('both', 'simultaneous', True),
            ])
        self.manual_index = 0
        self.manual_prepared = {}
        self.frontier_steps = [
            ('robot1', False), ('robot2', False), ('both', True)]
        self.frontier_index = 0
        self.frontier_prepared = {}
        self.selection_started = {robot: None for robot in ROBOTS}
        self.goal_search = {robot: None for robot in ROBOTS}
        self.failure_events = Counter()
        self.emitted_throttles = {}
        self.start_occupied = {robot: False for robot in ROBOTS}
        self.last_costmap_start_value = {robot: None for robot in ROBOTS}
        self.costmap_transition_emitted = {robot: False for robot in ROBOTS}
        self.start_gate_next_retry = {robot: 0.0 for robot in ROBOTS}
        self.start_gate_backoff = {robot: 0.25 for robot in ROBOTS}
        self.start_gate_prevented_requests = {robot: 0 for robot in ROBOTS}
        self.start_cell_stats = {
            robot: {
                source: {
                    'latest': None, 'minimum': None, 'maximum': None,
                    'samples': 0, 'inscribed_or_lethal_samples': 0,
                }
                for source in ('own_local_map', 'peer_local_map',
                               'shared_map', 'global_costmap')
            }
            for robot in ROBOTS
        }
        self.start_clearance_stats = {
            robot: {
                source: {'latest_m': None, 'minimum_m': None}
                for source in ('own_local_map', 'peer_local_map',
                               'shared_map', 'global_costmap')
            }
            for robot in ROBOTS
        }
        self.latest_start_clearance = {
            robot: {source: None for source in self.start_clearance_stats[robot]}
            for robot in ROBOTS
        }
        # Clearance is diagnostic telemetry, not a control input. Cache the
        # bounded neighbourhood scan until the map stamp or robot cell changes.
        self._start_clearance_cache = {}
        self.time_state_seconds = {
            robot: Counter({state: 0.0 for state in TIME_STATES})
            for robot in ROBOTS
        }
        self.last_sample_sim = None
        self.phase_coverage = {
            name: {robot: {'start': None, 'end': None} for robot in ROBOTS}
            for name in ('FILTER_AND_MAPPING', 'NAV2_ONLY',
                         'FRONTIER_GENERATION_ONLY', 'FRONTIER_TO_NAV2')
        }
        self.tf_buffer = Buffer(cache_time=Duration(seconds=60.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.compute_clients = {
            robot: ActionClient(
                self, ComputePathToPose,
                f'/{robot}/compute_path_to_pose') for robot in ROBOTS
        }
        self.navigate_clients = {
            robot: ActionClient(
                self, NavigateToPose,
                f'/{robot}/navigate_to_pose') for robot in ROBOTS
        }
        self.lifecycle_clients = {}
        for robot in ROBOTS:
            for name in NAV2_NODES:
                key = (robot, name)
                self.lifecycle_clients[key] = self.create_client(
                    GetState, f'/{robot}/{name}/get_state')
        map_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Clock, '/clock', self._on_clock, qos_profile_sensor_data)
        self.create_subscription(Log, '/rosout', self._rosout, 100)
        for robot in ROBOTS:
            self.create_subscription(
                LaserScan, f'/{robot}/scan_d500_fixed',
                partial(self._scan, robot, 'fixed'), qos_profile_sensor_data)
            self.create_subscription(
                LaserScan, f'/{robot}/scan_d500_slam',
                partial(self._scan, robot, 'slam'), qos_profile_sensor_data)
            self.create_subscription(
                Odometry, f'/{robot}/odom', partial(self._odom, robot),
                qos_profile_sensor_data)
            self.create_subscription(
                OccupancyGrid, f'/{robot}/map',
                partial(self._map, robot, 'local'), map_qos)
            self.create_subscription(
                OccupancyGrid, f'/{robot}/shared_map',
                partial(self._map, robot, 'shared'), map_qos)
            self.create_subscription(
                OccupancyGrid, f'/{robot}/global_costmap/costmap',
                partial(self._map, robot, 'costmap'), map_qos)
            self.create_subscription(
                OccupancyGrid, f'/{robot}/local_costmap/costmap',
                partial(self._map, robot, 'local_costmap'), map_qos)
            self.create_subscription(
                FrontierCandidateArray, f'/{robot}/frontier_candidates',
                partial(self._candidate, robot), 10)
            self.create_subscription(
                Twist, f'/{robot}/cmd_vel_nav',
                partial(self._cmd_vel, robot), qos_profile_sensor_data)
            self.create_subscription(
                TwistStamped, f'/{robot}/cmd_vel',
                partial(self._cmd_vel_stamped, robot), qos_profile_sensor_data)
        self.events_file = (self.output / 'diagnostic_events.jsonl').open(
            'w', encoding='utf-8', buffering=1)
        self.handoff_file = (self.output / 'handoff_rejections.csv').open(
            'w', encoding='utf-8', newline='', buffering=1)
        self.handoff_csv = csv.DictWriter(self.handoff_file, fieldnames=[
            'robot', 'canonical_task_id', 'physical_signature',
            'snapshot_generation_time_s', 'dispatch_attempt_time_s',
            'candidate_age_s', 'source_map_revision', 'current_map_revision',
            'source_map_timestamp_s', 'current_map_timestamp_s',
            'source_costmap_timestamp_s', 'current_costmap_timestamp_s',
            'map_costmap_skew_s', 'approach_x_m', 'approach_y_m',
            'robot_x_m', 'robot_y_m', 'original_path_length_m',
            'final_path_length_m', 'start_cell_cost', 'goal_cell_cost',
            'minimum_path_clearance_m', 'original_planner_result',
            'final_planner_result', 'rejection_subreason',
            'map_change_classification', 'retry_count', 'wait_duration_s',
        ])
        self.handoff_csv.writeheader()
        self.handoff_rejections = Counter()
        self.handoff_records = []
        self.handoff_seen = set()
        self.candidate_context = {robot: {} for robot in ROBOTS}
        self.costmap_wait = {robot: None for robot in ROBOTS}
        self.timeseries_file = (self.output / 'diagnostic_timeseries.csv').open(
            'w', encoding='utf-8', newline='', buffering=1)
        self.timeseries = csv.DictWriter(
            self.timeseries_file,
            fieldnames=[
                'sim_time_s', 'mission_time_s', 'phase', 'robot', 'state',
                'x_m', 'y_m', 'cmd_nonzero', 'active_goal',
                'fixed_scan_age_s', 'slam_scan_age_s', 'local_map_age_s',
                'shared_map_age_s', 'costmap_age_s', 'candidate_age_s',
                'candidate_count', 'known_local_cells', 'known_shared_cells',
                'own_local_map_start', 'peer_local_map_start',
                'shared_map_start', 'start_cost',
                'start_classification',
                'start_gate_prevented_path_requests',
                'own_local_clearance_m', 'peer_local_clearance_m',
                'shared_map_clearance_m', 'costmap_clearance_m',
                'tf_available',
            ])
        self.timeseries.writeheader()
        self.tick_timer = self.create_timer(0.1, self._tick)
        self.sample_timer = self.create_timer(0.5, self._sample)
        self.lifecycle_timer = self.create_timer(1.0, self._poll_lifecycle)
        self.process_timer = self.create_timer(10.0, self._sample_processes)
        self.wall_watchdog_timer = self.create_timer(
            0.5, self._wall_watchdog,
            clock=RclClock(clock_type=ClockType.STEADY_TIME))
        self._event('DIAGNOSTIC_STARTED', severity='INFO',
                    mission_duration_s=self.mission_duration,
                    launch_chain=[
                        LAUNCH_FILE,
                        'two_robots_frontier_candidates_launch.py',
                        'two_robots_teammate_filtered_stack_launch.py',
                        'two_robots_teammate_filtered_dual_slam_launch.py'])

    def now_sim(self) -> float:
        """Return the latest authoritative simulation clock value."""
        return self.latest_clock

    def mission_elapsed(self) -> Optional[float]:
        """Return readiness-relative simulation time."""
        if self.ready_sim is None:
            return None
        return max(0.0, self.now_sim() - self.ready_sim)

    def _event(self, event: str, robot: Optional[str] = None,
               severity: str = 'INFO', **values):
        record = {
            'event': event, 'severity': severity, 'robot': robot,
            'wall_time': utc_now(), 'sim_time_s': round(self.now_sim(), 6),
            'mission_time_s': (
                None if self.mission_elapsed() is None
                else round(self.mission_elapsed(), 6)),
            **values,
        }
        self.events_file.write(json.dumps(record, sort_keys=True) + '\n')
        encoded = 'DIAGNOSTIC_EVENT ' + json.dumps(record, sort_keys=True)
        # rclpy caches severity per Python call site, so each severity needs a
        # distinct static logger invocation.
        if severity == 'ERROR':
            self.get_logger().error(encoded)
        elif severity == 'WARN':
            self.get_logger().warning(encoded)
        else:
            self.get_logger().info(encoded)

    def _trigger(self, event: str, robot: Optional[str] = None,
                 throttle_s: float = 10.0, **values):
        key = (event, robot)
        now = self.now_sim()
        if now - self.emitted_throttles.get(key, -math.inf) < throttle_s:
            return
        self.emitted_throttles[key] = now
        self.failure_events[event] += 1
        self._event(event, robot, severity='WARN', **values)

    def _on_clock(self, message: Clock):
        value = stamp_seconds(message.clock)
        self.latest_clock = value
        if not self.clock_samples or value != self.clock_samples[-1]:
            self.clock_samples.append(value)

    def _scan(self, robot: str, stream: str, message: LaserScan):
        now_wall = time.monotonic()
        now_sim = self.now_sim()
        stamp = stamp_seconds(message.header.stamp)
        self.last_received[robot][stream + '_scan'] = now_sim
        self.messages[robot][stream + '_scan'] += 1
        if self.ready_sim is not None and self.mission_elapsed() < 60.0:
            self.scan_times[robot][stream].append(now_sim)
        key = (message.header.stamp.sec, message.header.stamp.nanosec)
        if stream == 'fixed':
            self.fixed_scans[robot][key] = (
                now_wall, tuple(message.ranges), message.range_min,
                message.range_max)
            while len(self.fixed_scans[robot]) > 32:
                self.fixed_scans[robot].pop(next(iter(self.fixed_scans[robot])))
            return
        del stamp
        raw = self.fixed_scans[robot].pop(key, None)
        output_nan = sum(math.isnan(value) for value in message.ranges)
        output_inf = sum(math.isinf(value) and value > 0
                         for value in message.ranges)
        self.filter_observations[robot]['output_nan'] += output_nan
        self.filter_observations[robot]['output_positive_inf'] += output_inf
        if raw is None:
            self.filter_observations[robot]['unmatched_output'] += 1
            return
        arrival, ranges, _, _ = raw
        self.scan_latencies[robot].append(max(0.0, now_wall - arrival))
        input_nan = sum(math.isnan(value) for value in ranges)
        input_inf = sum(math.isinf(value) and value > 0 for value in ranges)
        completed = sum(
            math.isinf(before) and before > 0
            and index < len(message.ranges)
            and math.isclose(message.ranges[index], 11.98,
                             rel_tol=0.0, abs_tol=1e-6)
            for index, before in enumerate(ranges))
        finite_masked = sum(
            math.isfinite(before) and index < len(message.ranges)
            and math.isnan(message.ranges[index])
            for index, before in enumerate(ranges))
        self.filter_observations[robot]['matched_output'] += 1
        self.filter_observations[robot]['input_nan'] += input_nan
        self.filter_observations[robot]['input_positive_inf'] += input_inf
        self.filter_observations[robot]['completed_positive_inf'] += completed
        self.filter_observations[robot]['finite_masked'] += finite_masked
        if output_inf:
            self.filter_observations[robot]['unsafe_output'] += 1
        if self.filter_observations[robot]['matched_output'] == 1:
            self._event(
                'FIRST_SLAM_SCAN_VALIDATED', robot,
                output_positive_inf=output_inf,
                finite_returns_masked=finite_masked,
                natural_positive_inf_completed=completed,
                safe=output_inf == 0)

    def _odom(self, robot: str, message: Odometry):
        self.last_received[robot]['odom'] = self.now_sim()
        point = message.pose.pose.position
        current = (point.x, point.y)
        self.odom[robot] = message
        run = self.active_nav[robot]
        if run is not None:
            if run.last_odom is not None:
                run.odom_distance_m += math.dist(run.last_odom, current)
            run.last_odom = current

    def _map(self, robot: str, kind: str, message: OccupancyGrid):
        now = self.now_sim()
        self.last_received[robot][kind + '_map'] = now
        self.messages[robot][kind + '_map'] += 1
        if kind == 'local':
            self.maps[robot] = message
            if self.ready_sim is not None and self.mission_elapsed() < 60.0:
                self.map_times[robot].append(now)
        elif kind == 'shared':
            self.shared_maps[robot] = message
        elif kind == 'costmap':
            self.costmaps[robot] = message
        else:
            self.local_costmaps[robot] = message

    def _candidate(self, robot: str, message: FrontierCandidateArray):
        now = self.now_sim()
        previous = self.candidate_received_sim[robot]
        if previous is not None and now >= previous:
            self.candidate_intervals[robot].append(now - previous)
        self.candidate_received_sim[robot] = now
        self.last_received[robot]['candidate'] = now
        self.candidates[robot] = message
        source_stamp = stamp_seconds(message.map_stamp)
        for item in message.candidates:
            signature = candidate_physical_signature(item)
            points = self._candidate_points(item)
            # The start footprint is part of final path validity too.
            if self.poses[robot] is not None:
                points.append(self.poses[robot])
            self.candidate_context[robot][(signature, int(message.map_revision))] = {
                'snapshot_generation_time_s': now,
                'source_map_revision': int(message.map_revision),
                'source_map_timestamp_s': source_stamp,
                'source_costmap_timestamp_s': self._grid_stamp(
                    self.costmaps[robot]),
                'original_path_length_m': float(item.path_length_m),
                'source_region': region_fingerprint(
                    self.shared_maps[robot], points),
                'points': points,
                'frontier_id': int(item.frontier_id),
            }
        stats = self.candidate_stats[robot]
        stats['snapshots'] += 1
        stats['approaches'] += len(message.candidates)
        stats['valid_paths'] += sum(
            item.reachability_state == item.REACHABLE
            and math.isfinite(item.path_length_m)
            for item in message.candidates)
        stats['visible_gain_finite_nonnegative'] += sum(
            math.isfinite(item.information_gain)
            and item.information_gain >= 0.0 for item in message.candidates)
        stats['reused_local_paths'] += sum(
            bool(item.local_path_samples) for item in message.candidates)
        for item in message.candidates:
            if len(self.candidate_gain_records[robot]) >= 2000:
                break
            self.candidate_gain_records[robot].append({
                'snapshot_stamp_s': stamp_seconds(message.header.stamp),
                'map_revision': int(message.map_revision),
                'frontier_id': int(item.frontier_id),
                'visible_reveal_gain': float(item.information_gain),
                'path_length_m': float(item.path_length_m),
                'reachability_state': int(item.reachability_state),
            })
        signature = (message.map_revision, tuple(
            (item.frontier_id, round(item.path_length_m, 3))
            for item in message.candidates))
        if signature == self.candidate_last_signature[robot]:
            stats['unchanged_snapshots'] += 1
        self.candidate_last_signature[robot] = signature
        for run in reversed(self.nav_runs):
            if run.robot != robot or run.next_candidate_delay_s is not None:
                continue
            if run.terminal_sim is not None and message.candidates:
                run.next_candidate_delay_s = max(
                    0.0, now - run.terminal_sim)
                self._event(
                    'POST_GOAL_CANDIDATE_READY', robot, label=run.label,
                    delay_s=run.next_candidate_delay_s,
                    map_revision=int(message.map_revision),
                    candidate_count=len(message.candidates))
            break

    def _cmd_vel_stamped(self, robot: str, message: TwistStamped):
        self._record_cmd(robot, message.twist)

    def _cmd_vel(self, robot: str, message: Twist):
        self._record_cmd(robot, message)

    def _record_cmd(self, robot: str, twist: Twist):
        nonzero = (
            abs(twist.linear.x) > 1e-4 or abs(twist.linear.y) > 1e-4
            or abs(twist.angular.z) > 1e-4)
        self.cmd[robot] = {'nonzero': nonzero, 'last': self.now_sim()}
        self.messages[robot]['cmd'] += 1
        run = self.active_nav[robot]
        if run is not None and run.accepted_sim is not None:
            run.cmd_count += 1
            if nonzero:
                run.nonzero_cmd_count += 1
                if run.first_motion_sim is None:
                    run.first_motion_sim = self.now_sim()
                    self._event('FIRST_NONZERO_CMD_VEL', robot,
                                label=run.label,
                                latency_s=run.first_motion_sim
                                - run.accepted_sim)

    def _rosout(self, message: Log):
        name = message.name
        text = message.msg
        robot = next((item for item in ROBOTS if f'/{item}/' in name), None)
        if robot is None:
            robot = next((item for item in ROBOTS if item in name), None)
        if text.startswith('FILTER_METRICS') and robot:
            for key, value in re.findall(r'(\w+)=([0-9.]+)', text):
                self.filter_stats[robot][key] = float(value)
        if text.startswith('CANDIDATE_METRICS') and robot:
            values = dict(re.findall(r'(\w+)=([0-9.]+)', text))
            converted = {key: float(value) for key, value in values.items()}
            self.candidate_generation[robot].append(converted)
            started = self.candidate_cycle_started[robot]
            cycle_duration = 0.0
            if started is not None:
                cycle_duration = max(0.0, self.now_sim() - started)
                self.candidate_cycle_durations[robot].append(cycle_duration)
                self.candidate_cycle_started[robot] = None
            duration = max(cycle_duration,
                           converted.get('extraction_ms', 0.0) / 1000.0)
            if duration > self.FRONTIER_SLOW_S:
                self._trigger('FRONTIER_GENERATION_SLOW', robot,
                              duration_s=duration)
            queries = int(converted.get('queries', 0))
            reachable = int(converted.get('reachable', 0))
            if queries >= 3 and reachable == 0:
                self.candidate_failure_cycles[robot] += 1
                if self.candidate_failure_cycles[robot] >= 3:
                    failed_cycles = self.candidate_failure_cycles[robot]
                    self.planner_failure_storm[robot] = True
                    self._trigger(
                        'PLANNER_FAILURE_STORM', robot,
                        source='frontier_candidate_generator',
                        consecutive_failed_cycles=failed_cycles,
                        queries_in_cycle=queries,
                        reachable_in_cycle=reachable)
            else:
                self.candidate_failure_cycles[robot] = 0
        if text.startswith('CANDIDATE_PATH_RESULT') and robot:
            self.candidate_stats[robot]['path_results'] += 1
            if 'ok=false' in text:
                self.candidate_stats[robot]['unreachable'] += 1
        if text.startswith('CANDIDATE_REJECT') and robot:
            self.candidate_stats[robot]['suppressed'] += 1
            match = re.search(r'reason=([A-Z0-9_]+)', text)
            reason = match.group(1) if match else 'UNSPECIFIED_REJECTION'
            self.candidate_invalid_reasons[robot][reason] += 1
            self._event('FRONTIER_CANDIDATE_INVALID', robot,
                        severity='WARN', reason=reason,
                        source='frontier_candidate_generator')
        if text.startswith('CANDIDATE_CYCLE_CONTEXT') and robot:
            self.candidate_cycle_started[robot] = self.now_sim()

    def _poll_lifecycle(self):
        if self.completed:
            return
        for key, client in self.lifecycle_clients.items():
            if key in self.lifecycle_pending or self.lifecycle[key[0]][key[1]]:
                continue
            if not client.service_is_ready():
                continue
            self.lifecycle_pending.add(key)
            future = client.call_async(GetState.Request())
            future.add_done_callback(partial(self._lifecycle_result, key))

    def _lifecycle_result(self, key, future):
        self.lifecycle_pending.discard(key)
        try:
            response = future.result()
            self.lifecycle[key[0]][key[1]] = (
                response.current_state.id == State.PRIMARY_STATE_ACTIVE)
        except Exception:
            self.lifecycle[key[0]][key[1]] = False

    def _age(self, robot: str, key: str) -> Optional[float]:
        stamp = self.last_received[robot].get(key)
        if stamp is None:
            return None
        return max(0.0, self.now_sim() - stamp)

    def _clock_advancing(self) -> bool:
        return len(self.clock_samples) >= 2 and any(
            right > left for left, right in zip(
                self.clock_samples, list(self.clock_samples)[1:]))

    @staticmethod
    def _usable(grid: Optional[OccupancyGrid]) -> bool:
        return bool(grid and grid.info.width and grid.info.height
                    and len(grid.data) == grid.info.width * grid.info.height
                    and any(value >= 0 for value in grid.data))

    def _lookup_pose(self, robot: str):
        try:
            transform = self.tf_buffer.lookup_transform(
                'shared_map', f'{robot}/base_footprint', Time(),
                timeout=Duration(seconds=0.0))
        except TransformException:
            self.tf_failures[robot] += 1
            return None
        point = transform.transform.translation
        self.poses[robot] = (point.x, point.y)
        return transform

    def _point_from_shared(self, grid: Optional[OccupancyGrid],
                           x: float, y: float) -> Optional[tuple[float, float]]:
        if grid is None:
            return None
        frame = grid.header.frame_id.lstrip('/')
        if frame == 'shared_map':
            return x, y
        try:
            transform = self.tf_buffer.lookup_transform(
                frame, 'shared_map', Time(),
                timeout=Duration(seconds=0.0)).transform
        except TransformException:
            return None
        q = transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        local_x = cosine * x - sine * y + transform.translation.x
        local_y = sine * x + cosine * y + transform.translation.y
        return local_x, local_y

    def _grid_patch_from_shared(self, grid: Optional[OccupancyGrid],
                                pose, radius_m=0.5):
        """Serialize a bounded square patch for costmap provenance events."""
        if grid is None or pose is None:
            return None
        point = self._point_from_shared(grid, *pose)
        if point is None or grid.info.resolution <= 0.0:
            return None
        _, cx, cy = grid_cell(grid, *point)
        cells = max(1, int(math.ceil(radius_m / grid.info.resolution)))
        rows = []
        occupied = []
        for row in range(max(0, cy - cells),
                         min(grid.info.height, cy + cells + 1)):
            values = []
            for column in range(max(0, cx - cells),
                                min(grid.info.width, cx + cells + 1)):
                value = int(grid.data[row * grid.info.width + column])
                values.append(value)
                if value >= 50:
                    occupied.append({
                        'column': column, 'row': row, 'value': value,
                        'distance_m': math.hypot(column - cx, row - cy)
                        * grid.info.resolution,
                    })
            rows.append(values)
        occupied.sort(key=lambda item: item['distance_m'])
        return {
            'frame_id': grid.header.frame_id,
            'stamp_s': stamp_seconds(grid.header.stamp),
            'resolution_m': grid.info.resolution,
            'origin_x_m': grid.info.origin.position.x,
            'origin_y_m': grid.info.origin.position.y,
            'centre_cell': grid_value(grid, *point),
            'centre_column': cx, 'centre_row': cy,
            'rows': rows,
            'nearest_occupied': occupied[0] if occupied else None,
        }

    def _grid_value_from_shared(self, grid: Optional[OccupancyGrid],
                                x: float, y: float) -> Optional[int]:
        point = self._point_from_shared(grid, x, y)
        return None if point is None else grid_value(grid, *point)

    def _grid_clearance_from_shared(self, grid: Optional[OccupancyGrid],
                                    x: float, y: float,
                                    threshold: int) -> Optional[float]:
        point = self._point_from_shared(grid, x, y)
        if grid is None or point is None:
            return None
        return nearest_occupied_distance(
            grid, *point, occupied_threshold=threshold)

    def _start_cell_provenance(self, robot: str) -> dict:
        pose = self.poses[robot]
        if pose is None:
            return {source: None for source in self.start_cell_stats[robot]}
        peer = 'robot2' if robot == 'robot1' else 'robot1'
        values = {
            'own_local_map': self._grid_value_from_shared(
                self.maps[robot], *pose),
            'peer_local_map': self._grid_value_from_shared(
                self.maps[peer], *pose),
            'shared_map': grid_value(self.shared_maps[robot], *pose),
            'global_costmap': grid_value(self.costmaps[robot], *pose),
        }
        def grid_key(grid):
            if grid is None:
                return None
            stamp = grid.header.stamp
            return (id(grid), stamp.sec, stamp.nanosec, grid.info.width,
                    grid.info.height, grid.info.resolution)

        cell_key = tuple(round(value / 0.01) for value in pose)
        cache_key = (
            robot, cell_key, grid_key(self.maps[robot]),
            grid_key(self.maps[peer]), grid_key(self.shared_maps[robot]),
            grid_key(self.costmaps[robot]),
        )
        clearances = self._start_clearance_cache.get(cache_key)
        if clearances is None:
            clearances = {
                'own_local_map': self._grid_clearance_from_shared(
                    self.maps[robot], *pose, threshold=50),
                'peer_local_map': self._grid_clearance_from_shared(
                    self.maps[peer], *pose, threshold=50),
                'shared_map': nearest_occupied_distance(
                    self.shared_maps[robot], *pose, occupied_threshold=50)
                if self.shared_maps[robot] is not None else None,
                'global_costmap': nearest_occupied_distance(
                    self.costmaps[robot], *pose, occupied_threshold=99)
                if self.costmaps[robot] is not None else None,
            }
            self._start_clearance_cache[cache_key] = clearances
            if len(self._start_clearance_cache) > 512:
                self._start_clearance_cache.pop(
                    next(iter(self._start_clearance_cache)))
        self.latest_start_clearance[robot] = clearances
        for source, clearance in clearances.items():
            if clearance is None:
                continue
            stats = self.start_clearance_stats[robot][source]
            stats['latest_m'] = clearance
            stats['minimum_m'] = clearance if stats['minimum_m'] is None \
                else min(stats['minimum_m'], clearance)
        for source, value in values.items():
            if value is None:
                continue
            stats = self.start_cell_stats[robot][source]
            stats['latest'] = value
            stats['minimum'] = value if stats['minimum'] is None else min(
                stats['minimum'], value)
            stats['maximum'] = value if stats['maximum'] is None else max(
                stats['maximum'], value)
            stats['samples'] += 1
            threshold = 99 if source == 'global_costmap' else 100
            if value >= threshold:
                stats['inscribed_or_lethal_samples'] += 1
        return values

    def _tf_ready(self, robot: str) -> bool:
        for frame in (
                f'{robot}/map', f'{robot}/odom',
                f'{robot}/base_footprint', f'{robot}/d500_lidar'):
            if not self.tf_buffer.can_transform(
                    'shared_map', frame, Time(),
                    timeout=Duration(seconds=0.0)):
                self.tf_failures[robot] += 1
                return False
        return self._lookup_pose(robot) is not None

    def _publisher_safe(self, robot: str) -> bool:
        infos = self.get_publishers_info_by_topic(
            f'/{robot}/scan_d500_slam')
        return len(infos) == 1 and infos[0].node_name == 'teammate_scan_filter'

    def _readiness(self) -> tuple[bool, dict]:
        details = {'clock_advances': self._clock_advancing()}
        for robot in ROBOTS:
            details[robot] = {
                'fixed_scan_fresh': (
                    self._age(robot, 'fixed_scan') is not None
                    and self._age(robot, 'fixed_scan') <= self.FRESH_SCAN_S),
                'slam_scan_fresh': (
                    self._age(robot, 'slam_scan') is not None
                    and self._age(robot, 'slam_scan') <= self.FRESH_SCAN_S),
                'odom_fresh': (
                    self._age(robot, 'odom') is not None
                    and self._age(robot, 'odom') <= self.FRESH_SCAN_S),
                'local_map_usable': self._usable(self.maps[robot]),
                'shared_map_usable': self._usable(self.shared_maps[robot]),
                'costmap_usable': self._usable(self.costmaps[robot]),
                'tf_ready': self._tf_ready(robot),
                'nav2_active': all(self.lifecycle[robot].values()),
                'compute_path_available':
                    self.compute_clients[robot].server_is_ready(),
                'navigate_available':
                    self.navigate_clients[robot].server_is_ready(),
                'frontier_publishing': self.candidates[robot] is not None,
                'slam_scan_single_safe_publisher': self._publisher_safe(robot),
            }
        ready = details['clock_advances'] and all(
            all(details[robot].values()) for robot in ROBOTS)
        return ready, details

    def _tick(self):
        if self.completed:
            return
        if self.ready_sim is None:
            ready, details = self._readiness()
            self.readiness_report = details
            if ready:
                self.ready_sim = self.now_sim()
                self.ready_wall = time.monotonic()
                self.ready_utc = utc_now()
                for robot in ROBOTS:
                    known = known_cell_count(self.maps[robot])
                    self.stationary_known_start[robot] = known
                    self.phase_coverage['FILTER_AND_MAPPING'][robot]['start'] = (
                        known_cell_count(self.shared_maps[robot]))
                self._event(
                    'FULL_READINESS', startup_wall_s=self.ready_wall
                    - self.started_wall, readiness=details)
                self.get_logger().info('NAV2_FRONTIER_DIAGNOSTIC_READY')
            elif time.monotonic() - self.started_wall > self.startup_timeout:
                self.exit_code = 2
                self._event('READINESS_TIMEOUT', severity='ERROR',
                            readiness=details,
                            startup_wall_s=time.monotonic() - self.started_wall)
                self._finish()
            return
        elapsed = self.mission_elapsed()
        phase = phase_at(elapsed, self.mission_duration, self.phase_profile)
        if phase != self.last_phase:
            self._transition_phase(self.last_phase, phase)
            self.last_phase = phase
        if elapsed >= self.mission_duration:
            self._finish()
            return
        if phase == 'NAV2_ONLY':
            self._drive_manual()
        elif phase == 'FRONTIER_GENERATION_ONLY':
            self._cancel_all('phase3 candidate-only isolation')
        elif phase == 'FRONTIER_TO_NAV2':
            self._drive_frontiers()
        self._check_active_navigation()

    def _wall_watchdog(self):
        if (not self.completed and self.ready_sim is None
                and time.monotonic() - self.started_wall > self.startup_timeout):
            self.exit_code = 2
            _, details = self._readiness()
            self.readiness_report = details
            self._event('READINESS_TIMEOUT', severity='ERROR',
                        readiness=details,
                        startup_wall_s=time.monotonic() - self.started_wall)
            self._finish()

    def _transition_phase(self, old: str, new: str):
        elapsed = self.mission_elapsed()
        if old in self.phase_coverage:
            for robot in ROBOTS:
                self.phase_coverage[old][robot]['end'] = known_cell_count(
                    self.shared_maps[robot])
        if new in self.phase_coverage:
            for robot in ROBOTS:
                self.phase_coverage[new][robot]['start'] = known_cell_count(
                    self.shared_maps[robot])
        self._event('PHASE_TRANSITION', from_phase=old, to_phase=new,
                    elapsed_s=elapsed)
        if new == 'NAV2_ONLY':
            for robot in ROBOTS:
                self.stationary_known_end[robot] = known_cell_count(
                    self.maps[robot])
            self._validate_filter_phase()
        elif new == 'FRONTIER_GENERATION_ONLY':
            self._cancel_all('entering candidate generation only')
            self.manual_prepared.clear()
            self._sample_processes()
            for robot in ROBOTS:
                self.candidate_cpu_start[robot] = self._process_cpu(robot)
        elif new == 'FRONTIER_TO_NAV2':
            self._cancel_all('entering frontier handoff')
            for robot in ROBOTS:
                self.candidate_cpu_end[robot] = self._process_cpu(robot)

    def _validate_filter_phase(self):
        for robot in ROBOTS:
            fixed_hz = self._frequency(self.scan_times[robot]['fixed'])
            slam_hz = self._frequency(self.scan_times[robot]['slam'])
            raw = dict(self.filter_observations[robot])
            adequate = slam_hz >= self.FILTER_MIN_HZ
            if not adequate:
                self._trigger('FILTER_OUTPUT_TOO_SLOW', robot,
                              output_hz=slam_hz,
                              required_hz=self.FILTER_MIN_HZ)
            received = self.filter_stats[robot].get('received', 0.0)
            drops = self.filter_stats[robot].get('dropped_tf_timeout', 0.0)
            if received and drops / received > 0.2:
                self._trigger('FILTER_TF_DROP_STORM', robot,
                              dropped_tf_timeout=drops, received=received,
                              ratio=drops / received)
            self._event(
                'FILTER_PHASE_RESULT', robot, fixed_scan_hz=fixed_hz,
                slam_scan_hz=slam_hz, sufficient_for_slam_toolbox=adequate,
                local_map_growth_cells=(self.stationary_known_end[robot]
                                        - self.stationary_known_start[robot]),
                runtime_filter_metrics=dict(self.filter_stats[robot]),
                scan_observations=raw,
                contract={
                    'pose_source': 'independent odometry transforms only',
                    'shared_map_or_slam_tf_used': False,
                    'finite_first_surface_only': True,
                    'peer_radius_m': 0.035,
                    'range_tolerance_m': 0.005,
                    'selected_geometry_model': (
                        raw.get('geometry_model',
                                'epuck_v2_pi_puck_d500_conservative_circle')),
                    'teammate_geometry_radius_m': raw.get(
                        'geometry_radius_m', 0.026),
                    'teammate_window_finite_returns': raw.get(
                        'teammate_window_finite_returns', 0),
                    'teammate_window_masked_returns': raw.get(
                        'teammate_window_masked_returns', 0),
                    'suspected_missed_teammate_returns': raw.get(
                        'teammate_window_suspected_missed_returns', 0),
                    'maximum_teammate_residual_m': raw.get(
                        'teammate_window_max_residual_m', 0.0),
                    'mean_teammate_residual_m': raw.get(
                        'teammate_window_mean_residual_m', 0.0),
                    'mean_teammate_radial_m': raw.get(
                        'teammate_window_mean_radial_m', 0.0),
                    'maximum_teammate_radial_m': raw.get(
                        'teammate_window_max_radial_m', 0.0),
                    'positive_inf_masked_as_teammate': False,
                    'degraded_passthrough_possible': False,
                    'exact_tf_queue_bound': 4,
                    'exact_tf_drop_deadline_s': 0.2,
                    'first_output_safely_filtered': (
                        raw.get('unsafe_output', 0) == 0
                        and raw.get('matched_output', 0) > 0),
                })

    @staticmethod
    def _frequency(times: list[float]) -> float:
        if len(times) < 2 or times[-1] <= times[0]:
            return 0.0
        return (len(times) - 1) / (times[-1] - times[0])

    def _sample(self):
        if self.completed:
            return
        if self.ready_sim is None:
            return
        elapsed = min(self.mission_elapsed(), self.mission_duration)
        delta = 0.0 if self.last_sample_sim is None else max(
            0.0, elapsed - self.last_sample_sim)
        self.last_sample_sim = elapsed
        phase = phase_at(elapsed, self.mission_duration, self.phase_profile)
        for robot in ROBOTS:
            transform = self._lookup_pose(robot)
            pose = self.poses[robot]
            state, classification = self._classify(robot, phase)
            self.time_state_seconds[robot][state] += delta
            run = self.active_nav[robot]
            if run is not None:
                run.classifications[classification] += delta
                run.feedback_intervals += 1
                if not self.cmd[robot]['nonzero']:
                    run.zero_feedback_intervals += 1
            provenance = self._start_cell_provenance(robot)
            clearances = self.latest_start_clearance[robot]
            start_cost = provenance['global_costmap']
            start_classification, _ = classify_start_cell(
                self.costmaps[robot], *(pose or (0.0, 0.0)))
            previous_cost = self.last_costmap_start_value[robot]
            if (robot == 'robot2' and pose is not None
                    and previous_cost is not None
                    and previous_cost < 99 and start_cost is not None
                    and start_cost >= 99
                    and not self.costmap_transition_emitted[robot]):
                peer = 'robot1'
                shared_nearest = nearest_occupied_cell(
                    self.shared_maps[robot], *pose, occupied_threshold=50)
                peer_nearest = self._grid_clearance_from_shared(
                    self.maps[peer], *pose, threshold=50)
                cost_nearest = nearest_occupied_cell(
                    self.costmaps[robot], *pose, occupied_threshold=99)
                self._event(
                    'COSTMAP_99_TRANSITION', robot,
                    pose=pose, previous_cost=previous_cost,
                    current_cost=start_cost,
                    peer_pose=self.poses[peer],
                    inter_robot_distance=(None if self.poses[peer] is None
                                          else math.dist(pose, self.poses[peer])),
                    map_revisions={
                        'robot1_local_messages': self.messages['robot1']['local_map'],
                        'robot2_local_messages': self.messages['robot2']['local_map'],
                        'robot1_shared_messages': self.messages['robot1']['shared_map'],
                        'robot2_shared_messages': self.messages['robot2']['shared_map'],
                        'robot2_global_costmap_messages': self.messages['robot2']['costmap'],
                        'robot2_local_costmap_messages': self.messages['robot2']['local_costmap'],
                    },
                    map_timestamps={
                        name: (None if value is None else stamp_seconds(value.header.stamp))
                        for name, value in {
                            'robot1_local': self.maps['robot1'],
                            'robot2_local': self.maps['robot2'],
                            'robot1_shared': self.shared_maps['robot1'],
                            'robot2_shared': self.shared_maps['robot2'],
                            'robot2_sanitized_shared': self.shared_maps['robot2'],
                            'robot2_global_costmap': self.costmaps['robot2'],
                            'robot2_local_costmap': self.local_costmaps['robot2'],
                        }.items()},
                    centre_values={
                        'robot1_local': self._grid_value_from_shared(
                            self.maps['robot1'], *pose),
                        'robot2_local': self._grid_value_from_shared(
                            self.maps['robot2'], *pose),
                        'robot1_shared': grid_value(
                            self.shared_maps['robot1'], *pose),
                        'robot2_shared_sanitized': grid_value(
                            self.shared_maps['robot2'], *pose),
                        'robot2_global_costmap': start_cost,
                        'robot2_local_costmap': self._grid_value_from_shared(
                            self.local_costmaps['robot2'], *pose),
                    },
                    nearest_cells={
                        'robot2_shared': shared_nearest,
                        'robot1_local_at_robot2': peer_nearest,
                        'robot2_global_costmap': cost_nearest,
                    },
                    patches={
                        'robot1_local': self._grid_patch_from_shared(
                            self.maps['robot1'], pose),
                        'robot2_local': self._grid_patch_from_shared(
                            self.maps['robot2'], pose),
                        'robot1_shared': self._grid_patch_from_shared(
                            self.shared_maps['robot1'], pose),
                        'robot2_shared_sanitized': self._grid_patch_from_shared(
                            self.shared_maps['robot2'], pose),
                        'robot2_global_costmap': self._grid_patch_from_shared(
                            self.costmaps['robot2'], pose),
                        'robot2_local_costmap': self._grid_patch_from_shared(
                            self.local_costmaps['robot2'], pose),
                    },
                    sanitizer_radius_m=0.067,
                    global_costmap_layers={
                        'plugins': ['static_layer', 'inflation_layer'],
                        'static_map_topic': '/robot2/shared_map',
                        'obstacle_layer_in_plugins': False,
                        'configured_scan_topic': '/robot2/scan_d500_fixed',
                        'inflation_radius_m': 0.11,
                    },
                    source_classification=(
                        'STATIC_LAYER_OR_INFLATION_FROM_STATIC'),
                )
                self.costmap_transition_emitted[robot] = True
            self.last_costmap_start_value[robot] = start_cost
            if pose:
                if start_cost is None or start_cost < 0:
                    self._trigger('COSTMAP_STALE', robot,
                                  reason='START_CELL_UNAVAILABLE',
                                  costmap_value=start_cost)
                elif start_cost >= 99:
                    self.start_occupied[robot] = True
                    self._trigger('ROBOT_START_OCCUPIED', robot,
                                  costmap_value=start_cost,
                                  own_local_map_value=provenance[
                                      'own_local_map'],
                                  peer_local_map_value=provenance[
                                      'peer_local_map'],
                                  shared_map_value=provenance['shared_map'])
            tf_available = transform is not None
            row = {
                'sim_time_s': f'{self.now_sim():.6f}',
                'mission_time_s': f'{elapsed:.6f}', 'phase': phase,
                'robot': robot, 'state': state,
                'x_m': '' if pose is None else f'{pose[0]:.5f}',
                'y_m': '' if pose is None else f'{pose[1]:.5f}',
                'cmd_nonzero': int(self.cmd[robot]['nonzero']),
                'active_goal': int(run is not None),
                'fixed_scan_age_s': self._format_age(
                    self._age(robot, 'fixed_scan')),
                'slam_scan_age_s': self._format_age(
                    self._age(robot, 'slam_scan')),
                'local_map_age_s': self._format_age(
                    self._age(robot, 'local_map')),
                'shared_map_age_s': self._format_age(
                    self._age(robot, 'shared_map')),
                'costmap_age_s': self._format_age(
                    self._age(robot, 'costmap_map')),
                'candidate_age_s': self._format_age(
                    self._age(robot, 'candidate')),
                'candidate_count': 0 if self.candidates[robot] is None
                else len(self.candidates[robot].candidates),
                'known_local_cells': known_cell_count(self.maps[robot]),
                'known_shared_cells': known_cell_count(
                    self.shared_maps[robot]),
                'own_local_map_start': provenance['own_local_map'],
                'peer_local_map_start': provenance['peer_local_map'],
                'shared_map_start': provenance['shared_map'],
                'start_cost': '' if start_cost is None else start_cost,
                'start_classification': start_classification,
                'start_gate_prevented_path_requests': (
                    self.start_gate_prevented_requests[robot]),
                'own_local_clearance_m': clearances['own_local_map'],
                'peer_local_clearance_m': clearances['peer_local_map'],
                'shared_map_clearance_m': clearances['shared_map'],
                'costmap_clearance_m': clearances['global_costmap'],
                'tf_available': int(tf_available),
            }
            self.timeseries.writerow(row)
            self._automatic_triggers(robot, phase)

    @staticmethod
    def _format_age(value):
        return '' if value is None else f'{value:.4f}'

    def _classify(self, robot: str, phase: str):
        if not self._tf_ready(robot) or not self._usable(self.shared_maps[robot]):
            return 'map/TF unavailable', 'TF_UNAVAILABLE'
        if not self._usable(self.costmaps[robot]):
            return 'map/TF unavailable', 'COSTMAP_UNAVAILABLE'
        if phase == 'FILTER_AND_MAPPING':
            return ('intentionally stationary filter test', 'SETTLING')
        request = self.planner_requests[robot]
        if request is not None:
            return 'waiting for planner', 'WAITING_FOR_PLANNER'
        run = self.active_nav[robot]
        if run is not None:
            if run.accepted_sim is None:
                return 'waiting for goal acceptance', 'WAITING_FOR_ACTION_SERVER'
            if run.recovery_count > 0:
                return 'recovery', 'RECOVERY'
            if self.cmd[robot]['nonzero']:
                return ('active navigation with nonzero cmd_vel', 'NAVIGATING')
            return ('active goal with zero cmd_vel',
                    'ZERO_COMMAND_WITH_ACTIVE_GOAL')
        terminal = self.last_terminal_sim[robot]
        if terminal is not None and self.now_sim() - terminal < 2.0:
            return 'post-goal settling', 'SETTLING'
        batch = self.candidates[robot]
        valid = batch is not None and any(
            deterministic_candidate_key(item)[0] is False
            for item in batch.candidates)
        if valid:
            return 'idle despite valid candidates', 'WAITING_FOR_CONTROLLER'
        return 'waiting for candidates', 'WAITING_FOR_CONTROLLER'

    def _automatic_triggers(self, robot: str, phase: str):
        scan_age = self._age(robot, 'slam_scan')
        if scan_age is None or scan_age > self.FRESH_SCAN_S:
            self._trigger('SLAM_SCAN_STALE', robot, age_s=scan_age)
        map_age = self._age(robot, 'shared_map')
        if map_age is None or map_age > self.FRESH_MAP_S:
            self._trigger('MAP_STALE', robot, age_s=map_age)
        cost_age = self._age(robot, 'costmap_map')
        if cost_age is None or cost_age > self.FRESH_COSTMAP_S:
            self._trigger('COSTMAP_STALE', robot, age_s=cost_age)
        candidate_age = self._age(robot, 'candidate')
        if phase in ('FRONTIER_GENERATION_ONLY', 'FRONTIER_TO_NAV2'):
            if (not self.compute_clients[robot].server_is_ready()
                    or not self.navigate_clients[robot].server_is_ready()):
                self.candidate_stats[robot][
                    'nav2_service_unavailable_samples'] += 1
            if candidate_age is None or candidate_age > self.FRONTIER_STALE_S:
                self._trigger('FRONTIER_SNAPSHOT_STALE', robot,
                              age_s=candidate_age)
            batch = self.candidates[robot]
            if batch is not None and not batch.candidates:
                self._trigger('NO_REACHABLE_FRONTIERS', robot,
                              snapshot_age_s=candidate_age)
        run = self.active_nav[robot]
        if run is not None and run.accepted_sim is not None:
            active_for = self.now_sim() - run.accepted_sim
            first = run.first_motion_sim
            if first is None and active_for > self.ZERO_COMMAND_WARN_S:
                self._trigger('ZERO_COMMAND_ACTIVE_GOAL', robot,
                              active_duration_s=active_for,
                              cmd_count=run.cmd_count)
            recent_cmds = self.messages[robot]['cmd']
            if active_for > 5.0 and recent_cmds / max(active_for, 1e-9) \
                    < self.CONTROLLER_MIN_HZ:
                self._trigger('CONTROLLER_RATE_LOW', robot,
                              measured_hz=recent_cmds / active_for,
                              threshold_hz=self.CONTROLLER_MIN_HZ)
            if active_for > 10.0 and run.odom_distance_m < 0.01:
                self._trigger('ODOM_NO_PROGRESS', robot,
                              active_duration_s=active_for,
                              odom_distance_m=run.odom_distance_m)

    def _pose(self, x: float, y: float, yaw: float = 0.0) -> PoseStamped:
        result = PoseStamped()
        result.header.frame_id = 'shared_map'
        result.header.stamp = self.get_clock().now().to_msg()
        result.pose.position.x = x
        result.pose.position.y = y
        result.pose.orientation.z = math.sin(yaw / 2.0)
        result.pose.orientation.w = math.cos(yaw / 2.0)
        return result

    def _start_gate_allows(self, robot: str) -> bool:
        """Prevent diagnostic path-query storms from an occupied start."""
        pose = self.poses[robot]
        if pose is None:
            return False
        category, value = classify_start_cell(
            self.costmaps[robot], *pose)
        if category in ('INSCRIBED', 'LETHAL'):
            self.start_occupied[robot] = True
            self.start_gate_prevented_requests[robot] += 1
            now = self.now_sim()
            if now >= self.start_gate_next_retry[robot]:
                self._trigger(
                    'ROBOT_START_OCCUPIED', robot,
                    start_classification=category, costmap_value=value,
                    prevented_path_requests=(
                        self.start_gate_prevented_requests[robot]),
                    retry_backoff_s=self.start_gate_backoff[robot])
                self.start_gate_next_retry[robot] = (
                    now + self.start_gate_backoff[robot])
                self.start_gate_backoff[robot] = min(
                    5.0, self.start_gate_backoff[robot] * 1.5)
            return False
        if category in ('FREE', 'INFLATED'):
            self.start_occupied[robot] = False
            self.start_gate_backoff[robot] = 0.25
            return True
        return False

    @staticmethod
    def _grid_stamp(grid: Optional[OccupancyGrid]):
        return None if grid is None else stamp_seconds(grid.header.stamp)

    @staticmethod
    def _candidate_points(candidate):
        """Bounded corridor sample used only for revision relevance telemetry."""
        points = [(candidate.approach_pose.pose.position.x,
                   candidate.approach_pose.pose.position.y),
                  (candidate.centroid.x, candidate.centroid.y)]
        for sample in list(candidate.local_path_samples)[:64]:
            points.append((sample.x, sample.y))
        return points

    def _handoff_context(self, robot: str, candidate, batch: FrontierCandidateArray,
                         source_context=None):
        signature = candidate_physical_signature(candidate)
        context = source_context or self.candidate_context[robot].get(
            (signature, int(batch.map_revision)), {})
        points = context.get('points', self._candidate_points(candidate))
        current_map = self.shared_maps[robot]
        current_revision = int(batch.map_revision) if batch is not None else None
        classification = map_change_classification(
            context.get('source_map_revision', int(batch.map_revision)),
            current_revision, context.get('source_region'),
            region_fingerprint(current_map, points))
        return signature, context, classification

    def _record_handoff(self, robot: str, candidate, batch, reason: str,
                        safe=None, final_length=None, final_result=None,
                        retry_count=0, wait_duration=0.0,
                        map_classification=None, source_context=None):
        """Write one machine-readable record for each distinct rejection."""
        signature, context, classification = self._handoff_context(
            robot, candidate, batch, source_context)
        if map_classification is None:
            map_classification = classification
        key = (robot, signature, reason,
               context.get('source_map_revision'),
               int(batch.map_revision) if batch is not None else None)
        if key in self.handoff_seen:
            return
        self.handoff_seen.add(key)
        point = candidate.approach_pose.pose.position
        pose = self.poses[robot]
        current_costmap_stamp = self._grid_stamp(self.costmaps[robot])
        current_map_stamp = self._grid_stamp(self.shared_maps[robot])
        source_costmap_stamp = context.get('source_costmap_timestamp_s')
        row = {
            'robot': robot,
            'canonical_task_id': int(candidate.frontier_id),
            'physical_signature': signature,
            'snapshot_generation_time_s': context.get(
                'snapshot_generation_time_s'),
            'dispatch_attempt_time_s': self.now_sim(),
            'candidate_age_s': self._age(robot, 'candidate'),
            'source_map_revision': context.get('source_map_revision'),
            'current_map_revision': int(batch.map_revision)
            if batch is not None else None,
            'source_map_timestamp_s': context.get('source_map_timestamp_s'),
            'current_map_timestamp_s': current_map_stamp,
            'source_costmap_timestamp_s': source_costmap_stamp,
            'current_costmap_timestamp_s': current_costmap_stamp,
            'map_costmap_skew_s': None if current_map_stamp is None
            or current_costmap_stamp is None else current_map_stamp - current_costmap_stamp,
            'approach_x_m': point.x, 'approach_y_m': point.y,
            'robot_x_m': None if pose is None else pose[0],
            'robot_y_m': None if pose is None else pose[1],
            'original_path_length_m': context.get('original_path_length_m'),
            'final_path_length_m': final_length,
            'start_cell_cost': None if safe is None else safe.get('start_cost'),
            'goal_cell_cost': None if safe is None else safe.get('goal_cost'),
            'minimum_path_clearance_m': None if safe is None
            else safe.get('nearest_obstacle_distance_m'),
            'original_planner_result': 'REACHABLE' if (
                candidate.reachability_state == candidate.REACHABLE) else 'UNREACHABLE',
            'final_planner_result': final_result,
            'rejection_subreason': reason,
            'map_change_classification': map_classification,
            'retry_count': retry_count, 'wait_duration_s': wait_duration,
        }
        self.handoff_csv.writerow(row)
        self.handoff_rejections[reason] += 1
        self.handoff_records.append(row)
        self._event(
            'FRONTIER_HANDOFF_REJECTED', robot, severity='WARN',
            **{key: value for key, value in row.items() if key != 'robot'})

    def _handoff_safe_goal(self, robot: str, candidate, batch):
        """Give a precise rejection reason; never hide it as provenance."""
        age = self._age(robot, 'candidate')
        if age is None or age > self.FRONTIER_STALE_S:
            return None, 'CANDIDATE_TOO_OLD'
        if self._age(robot, 'odom') is None or self._age(robot, 'odom') > self.FRESH_SCAN_S:
            return None, 'TF_TOO_OLD'
        map_stamp = self._grid_stamp(self.shared_maps[robot])
        costmap_stamp = self._grid_stamp(self.costmaps[robot])
        if (map_stamp is not None and costmap_stamp is not None
                and costmap_stamp + 0.01 < map_stamp):
            return None, 'WAITING_FOR_COSTMAP'
        pose = self.poses[robot]
        if pose is None or not self._start_gate_allows(robot):
            return None, 'START_NOT_TRAVERSABLE'
        point = candidate.approach_pose.pose.position
        raw = {
            'start_cost': grid_value(self.costmaps[robot], *pose),
            'goal_cost': grid_value(self.costmaps[robot], point.x, point.y),
            'nearest_obstacle_distance_m': nearest_blocked_distance(
                self.costmaps[robot], point.x, point.y),
        }
        safe = self._safe_goal(robot, point.x, point.y)
        if safe is not None:
            return safe, None
        cost = raw['goal_cost']
        shared = grid_value(self.shared_maps[robot], point.x, point.y)
        if cost is None or shared is None:
            return raw, 'GOAL_NOT_TRAVERSABLE'
        if cost != 0 or shared < 0 or shared >= 50:
            return raw, 'GOAL_NOT_TRAVERSABLE'
        peer = self.poses['robot2' if robot == 'robot1' else 'robot1']
        if (raw['nearest_obstacle_distance_m'] < 0.12
                or peer is None
                or math.dist((point.x, point.y), peer) < 0.18
                or line_point_distance(*pose, point.x, point.y, *peer) < 0.18):
            return raw, 'GOAL_NOT_TRAVERSABLE'
        return raw, 'OTHER'

    def _safe_goal(self, robot: str, x: float, y: float) -> Optional[dict]:
        costmap = self.costmaps[robot]
        shared = self.shared_maps[robot]
        pose = self.poses[robot]
        peer = self.poses['robot2' if robot == 'robot1' else 'robot1']
        if costmap is None or shared is None or pose is None or peer is None:
            return None
        if not self._start_gate_allows(robot):
            return None
        cost = grid_value(costmap, x, y)
        shared_value = grid_value(shared, x, y)
        start_cost = grid_value(costmap, *pose)
        clearance = nearest_blocked_distance(costmap, x, y)
        if (cost != 0 or shared_value is None or shared_value < 0
                or shared_value >= 50 or start_cost is None
                or start_cost < 0 or start_cost >= 99
                or clearance < 0.12 or math.dist((x, y), peer) < 0.18
                or line_point_distance(*pose, x, y, *peer) < 0.18):
            return None
        return {
            'start_cost': start_cost, 'goal_cost': cost,
            'nearest_obstacle_distance_m': clearance,
        }

    def _manual_candidates(self, robot: str, band: str):
        pose = self.poses[robot]
        if pose is None:
            return []
        low, high = ((0.8, 1.5) if band == 'short' else (2.0, 3.5))
        distances = [low + (high - low) * fraction
                     for fraction in (0.5, 0.25, 0.75, 0.05, 0.95)]
        options = []
        for distance in distances:
            for degrees in range(0, 360, 15):
                angle = math.radians(degrees)
                x = pose[0] + distance * math.cos(angle)
                y = pose[1] + distance * math.sin(angle)
                safe = self._safe_goal(robot, x, y)
                if safe:
                    options.append((self._pose(x, y, angle), safe))
        return options

    def _acquire_planner_lock(self, robot: str):
        path = f'/tmp/my_epuck_{robot}_compute_path.lock'
        lock_file = open(path, 'a+', encoding='utf-8')
        try:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            lock_file.close()
            if self.planner_busy_since[robot] is None:
                self.planner_busy_since[robot] = self.now_sim()
            busy = self.now_sim() - self.planner_busy_since[robot]
            if busy > 2.0:
                self._trigger('PATH_QUERY_BUSY', robot, busy_duration_s=busy)
            return None
        self.planner_busy_since[robot] = None
        return lock_file

    @staticmethod
    def _release_planner_lock(lock_file):
        if lock_file is None:
            return
        try:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
        finally:
            lock_file.close()

    def _request_path(self, robot: str, pose: PoseStamped, purpose: str,
                      metadata: dict, callback: Callable) -> bool:
        if self.planner_requests[robot] is not None:
            return False
        if not self._start_gate_allows(robot):
            return False
        client = self.compute_clients[robot]
        if not client.server_is_ready():
            self._event('WAITING_FOR_ACTION_SERVER', robot,
                        action='ComputePathToPose', purpose=purpose)
            return False
        lock_file = self._acquire_planner_lock(robot)
        if lock_file is None:
            return False
        request = PlannerRequest(
            purpose, pose, self.now_sim(), time.monotonic(), metadata,
            callback, lock_file)
        self.planner_requests[robot] = request
        goal = ComputePathToPose.Goal()
        goal.goal = pose
        goal.planner_id = 'GridBased'
        goal.use_start = False
        self._event('COMPUTE_PATH_REQUEST', robot, purpose=purpose,
                    x=pose.pose.position.x, y=pose.pose.position.y)
        future = client.send_goal_async(goal)
        future.add_done_callback(partial(self._path_accepted, robot, request))
        return True

    def _path_accepted(self, robot: str, request: PlannerRequest, future):
        if self.planner_requests[robot] is not request:
            self._release_planner_lock(request.lock_file)
            return
        try:
            handle = future.result()
        except Exception as error:
            self._complete_path(robot, request, False, None, -1, str(error))
            return
        if not handle.accepted:
            self._complete_path(
                robot, request, False, None, -1, 'GOAL_REJECTED')
            return
        request.goal_handle = handle
        result = handle.get_result_async()
        result.add_done_callback(partial(self._path_result, robot, request))

    def _path_result(self, robot: str, request: PlannerRequest, future):
        try:
            wrapped = future.result()
            result = wrapped.result
            success = (
                wrapped.status == GoalStatus.STATUS_SUCCEEDED
                and result.error_code == ComputePathToPose.Result.NONE
                and bool(result.path.poses))
            self._complete_path(
                robot, request, success, result.path,
                int(result.error_code), result.error_msg)
        except Exception as error:
            self._complete_path(robot, request, False, None, -1, str(error))

    def _complete_path(self, robot: str, request: PlannerRequest,
                       success: bool, path: Optional[NavPath],
                       error_code: int, message: str):
        self.planner_callback_active[robot] = True
        if self.planner_requests[robot] is request:
            self.planner_requests[robot] = None
        self._release_planner_lock(request.lock_file)
        latency = max(0.0, self.now_sim() - request.requested_sim)
        length = path_length(path) if path is not None else math.inf
        if latency > self.PLANNER_HIGH_S:
            self._trigger('PLANNER_LATENCY_HIGH', robot, latency_s=latency,
                          purpose=request.purpose, raw_path_length_m=length)
        if not success:
            self.planner_failures[robot] += 1
            if self.planner_failures[robot] >= 3:
                self.planner_failure_storm[robot] = True
                self._trigger('PLANNER_FAILURE_STORM', robot,
                              consecutive_failures=self.planner_failures[robot],
                              error_code=error_code, message=message)
        else:
            self.planner_failures[robot] = 0
        self._event(
            'COMPUTE_PATH_RESULT', robot,
            purpose=request.purpose, success=success, latency_s=latency,
            path_length_m=None if not math.isfinite(length) else length,
            estimated_travel_cost=None if not math.isfinite(length)
            else length, error_code=error_code, message=message)
        try:
            request.callback(
                success, path, length, latency, error_code, message)
        finally:
            self.planner_callback_active[robot] = False

    def _drive_manual(self):
        if (any(self.active_nav.values()) or any(self.planner_requests.values())
                or any(self.planner_callback_active.values())):
            return
        if self.manual_index >= len(self.manual_steps):
            return
        robot, band, simultaneous = self.manual_steps[self.manual_index]
        if not simultaneous:
            if self.goal_search[robot] is None:
                options = self._manual_candidates(robot, band)
                self.goal_search[robot] = deque(options[:30])
                self.selection_started[robot] = self.now_sim()
                self._event('MANUAL_GOAL_SELECTION_STARTED', robot,
                            band=band, safe_costmap_options=len(options))
            self._try_manual_option(robot, band, simultaneous=False)
            return
        for item in ROBOTS:
            if item in self.manual_prepared:
                continue
            if self.goal_search[item] is None:
                options = self._manual_candidates(item, 'short')
                if item == 'robot2' and 'robot1' in self.manual_prepared:
                    first = self.manual_prepared['robot1']['pose'].pose.position
                    options = [option for option in options if math.hypot(
                        option[0].pose.position.x - first.x,
                        option[0].pose.position.y - first.y) >= 0.8]
                self.goal_search[item] = deque(options[:30])
                self.selection_started[item] = self.now_sim()
            self._try_manual_option(item, 'short', simultaneous=True)
            return
        first = self.manual_prepared['robot1']
        second = self.manual_prepared['robot2']
        one = first['pose'].pose.position
        two = second['pose'].pose.position
        if math.hypot(one.x - two.x, one.y - two.y) < 0.8:
            self.manual_prepared.pop('robot2')
            self.goal_search['robot2'] = None
            return
        self._dispatch_prepared(self.manual_prepared, 'manual_simultaneous')
        self.manual_prepared = {}
        self.manual_index += 1

    def _try_manual_option(self, robot: str, band: str, simultaneous: bool):
        options = self.goal_search[robot]
        if not options:
            self._event('MANUAL_GOAL_SELECTION_FAILED', robot,
                        severity='WARN', band=band)
            self.goal_search[robot] = None
            self.manual_index += 1
            return
        pose, safe = options.popleft()
        low, high = ((0.8, 1.5) if band == 'short' else (2.0, 3.5))

        def validated(success, path, length, latency, code, message):
            del code, message
            peer = self.poses['robot2' if robot == 'robot1' else 'robot1']
            avoids_peer = success and peer is not None and all(
                math.hypot(stamped.pose.position.x - peer[0],
                           stamped.pose.position.y - peer[1]) >= 0.18
                for stamped in path.poses)
            if success and low <= length <= high and avoids_peer:
                prepared = {
                    'pose': pose, 'path': path, 'path_length': length,
                    'path_latency': latency, 'safe': safe,
                    'candidate_age': None, 'selection_time':
                        self.now_sim() - self.selection_started[robot],
                }
                self.goal_search[robot] = None
                if simultaneous:
                    self.manual_prepared[robot] = prepared
                else:
                    self._dispatch_navigation(
                        robot, prepared, 'manual', f'manual_{band}')
                    self.manual_index += 1
            else:
                self._try_manual_option(robot, band, simultaneous)

        self._request_path(robot, pose, f'manual_{band}', safe, validated)

    def _dispatch_prepared(self, prepared: dict, label: str):
        for robot, details in prepared.items():
            self._dispatch_navigation(robot, details, 'manual', label)

    def _dispatch_navigation(self, robot: str, prepared: dict,
                             kind: str, label: str):
        if self.active_nav[robot] is not None:
            return
        pose = prepared['pose']
        current = self.poses[robot]
        if current is None:
            return
        map_age = self._age(robot, 'shared_map')
        cost_age = self._age(robot, 'costmap_map')
        transform = self._lookup_pose(robot)
        tf_age = None
        if transform is not None:
            stamped = stamp_seconds(transform.header.stamp)
            tf_age = 0.0 if stamped == 0.0 else max(
                0.0, self.now_sim() - stamped)
        safe = prepared['safe']
        run = NavigationRun(
            robot=robot, kind=kind, label=label, pose=pose,
            path_length_m=prepared['path_length'],
            path_latency_s=prepared['path_latency'],
            request_sim=self.now_sim(), request_wall=time.monotonic(),
            metadata={
                **safe,
                'selection_time_s': prepared.get('selection_time'),
                'candidate_age_s': prepared.get('candidate_age'),
                'map_revision': prepared.get('map_revision'),
                'frontier_id': prepared.get('frontier_id'),
            }, start_pose=current, start_cost=safe.get('start_cost'),
            goal_cost=safe.get('goal_cost'),
            clearance_m=safe.get('nearest_obstacle_distance_m', 0.0),
            map_age_s=map_age, costmap_age_s=cost_age, tf_age_s=tf_age,
            last_odom=(self.odom[robot].pose.pose.position.x,
                       self.odom[robot].pose.pose.position.y)
            if self.odom[robot] is not None else None,
            coverage_start_cells=known_cell_count(self.shared_maps[robot]))
        self.active_nav[robot] = run
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._event(
            'NAVIGATION_REQUEST', robot, kind=kind, label=label,
            request_time_s=run.request_sim, path_length_m=run.path_length_m,
            estimated_travel_cost=run.path_length_m,
            start_cost=run.start_cost, goal_cost=run.goal_cost,
            nearest_obstacle_distance_m=run.clearance_m,
            tf_age_s=tf_age, map_age_s=map_age, costmap_age_s=cost_age,
            **{key: value for key, value in run.metadata.items()
               if key not in ('start_cost', 'goal_cost',
                              'nearest_obstacle_distance_m')})
        future = self.navigate_clients[robot].send_goal_async(
            goal, feedback_callback=partial(self._nav_feedback, robot, run))
        future.add_done_callback(partial(self._nav_accepted, robot, run))

    def _nav_accepted(self, robot: str, run: NavigationRun, future):
        if self.active_nav[robot] is not run:
            return
        try:
            handle = future.result()
        except Exception as error:
            self._nav_terminal(robot, run, GoalStatus.STATUS_UNKNOWN,
                               -1, str(error))
            return
        if not handle.accepted:
            self._event('GOAL_REJECTED', robot, severity='WARN', label=run.label)
            self._nav_terminal(robot, run, GoalStatus.STATUS_ABORTED,
                               -1, 'GOAL_REJECTED')
            return
        run.goal_handle = handle
        run.accepted_sim = self.now_sim()
        self._event('NAVIGATION_ACCEPTED', robot, label=run.label,
                    acceptance_latency_s=run.accepted_sim - run.request_sim)
        result = handle.get_result_async()
        result.add_done_callback(partial(self._nav_result, robot, run))

    def _nav_feedback(self, robot: str, run: NavigationRun, message):
        if self.active_nav[robot] is not run:
            return
        feedback = message.feedback
        run.recovery_count = int(feedback.number_of_recoveries)
        run.maximum_recovery_count = max(
            run.maximum_recovery_count, run.recovery_count)

    def _nav_result(self, robot: str, run: NavigationRun, future):
        try:
            wrapped = future.result()
            self._nav_terminal(
                robot, run, int(wrapped.status),
                int(wrapped.result.error_code), wrapped.result.error_msg)
        except Exception as error:
            self._nav_terminal(robot, run, GoalStatus.STATUS_UNKNOWN,
                               -1, str(error))

    def _nav_terminal(self, robot: str, run: NavigationRun, status: int,
                      result_code: int, message: str):
        if run.terminal_sim is not None:
            return
        run.terminal_sim = self.now_sim()
        run.result_status = status
        run.result_code = result_code
        run.result_message = message
        pose = self.poses[robot]
        goal = run.pose.pose.position
        run.final_pose_error_m = None if pose is None else math.hypot(
            pose[0] - goal.x, pose[1] - goal.y)
        run.coverage_end_cells = known_cell_count(self.shared_maps[robot])
        if self.active_nav[robot] is run:
            self.active_nav[robot] = None
        self.last_terminal_sim[robot] = self.now_sim()
        self.nav_runs.append(run)
        zero_percent = (100.0 * run.zero_feedback_intervals
                        / max(1, run.feedback_intervals))
        self._event(
            'NAVIGATION_TERMINAL', robot, kind=run.kind, label=run.label,
            result_status=status, result_code=result_code, message=message,
            success=(status == GoalStatus.STATUS_SUCCEEDED
                     and result_code == NavigateToPose.Result.NONE),
            execution_duration_s=(run.terminal_sim - run.accepted_sim)
            if run.accepted_sim is not None else None,
            first_nonzero_cmd_vel_latency_s=(run.first_motion_sim
                                             - run.accepted_sim)
            if run.first_motion_sim is not None
            and run.accepted_sim is not None else None,
            controller_update_hz=(run.cmd_count / max(
                1e-9, run.terminal_sim - run.accepted_sim))
            if run.accepted_sim is not None else 0.0,
            zero_cmd_feedback_percent=zero_percent,
            odometry_distance_m=run.odom_distance_m,
            recovery_count=run.maximum_recovery_count,
            final_pose_error_m=run.final_pose_error_m,
            classifications=dict(run.classifications))

    def _check_active_navigation(self):
        for robot, run in self.active_nav.items():
            if run is None or run.accepted_sim is None:
                continue
            limit = 80.0 if 'medium' in run.label else 60.0
            if self.now_sim() - run.accepted_sim > limit and run.goal_handle:
                if run.cancel_requested:
                    continue
                run.cancel_requested = True
                run.goal_handle.cancel_goal_async()
                self._event('NAVIGATION_TIMEOUT_CANCEL', robot,
                            severity='WARN', label=run.label,
                            limit_s=limit)

    def _cancel_all(self, reason: str):
        for robot, run in self.active_nav.items():
            if run is not None and run.goal_handle is not None:
                if run.cancel_requested:
                    continue
                run.cancel_requested = True
                run.goal_handle.cancel_goal_async()
                self._event('NAVIGATION_CANCEL_REQUEST', robot,
                            label=run.label, reason=reason)

    def _ranked_frontiers(self, robot: str):
        batch = self.candidates[robot]
        if batch is None:
            return []
        age = self._age(robot, 'candidate')
        if age is None or age > self.FRONTIER_STALE_S:
            return []
        ranked = []
        waiting_for_costmap = False
        for candidate in sorted(batch.candidates,
                                key=deterministic_candidate_key):
            if deterministic_candidate_key(candidate)[0]:
                continue
            point = candidate.approach_pose.pose.position
            safe, reason = self._handoff_safe_goal(robot, candidate, batch)
            if reason is not None:
                if reason == 'WAITING_FOR_COSTMAP':
                    waiting_for_costmap = True
                    continue
                self._record_handoff(robot, candidate, batch, reason)
                continue
            ranked.append((candidate, safe, batch, age))
        if waiting_for_costmap and not ranked:
            wait = self.costmap_wait[robot]
            now = self.now_sim()
            if wait is None:
                self.costmap_wait[robot] = {'started': now, 'retries': 0}
                self._event('WAITING_FOR_COSTMAP', robot,
                            reason='STATIC_LAYER_BEHIND_SANITIZED_MAP')
            else:
                wait['retries'] += 1
                if now - wait['started'] > 2.0:
                    for candidate in batch.candidates:
                        if deterministic_candidate_key(candidate)[0]:
                            continue
                        self._record_handoff(
                            robot, candidate, batch, 'COSTMAP_BEHIND_MAP',
                            retry_count=wait['retries'],
                            wait_duration=now - wait['started'])
                    self.costmap_wait[robot] = None
            return []
        self.costmap_wait[robot] = None
        return ranked

    def _drive_frontiers(self):
        if (any(self.active_nav.values()) or any(self.planner_requests.values())
                or any(self.planner_callback_active.values())):
            return
        if self.frontier_index >= len(self.frontier_steps):
            return
        target, simultaneous = self.frontier_steps[self.frontier_index]
        robots = ROBOTS if simultaneous else (target,)
        for robot in robots:
            if robot in self.frontier_prepared:
                continue
            ranked = self._ranked_frontiers(robot)
            if (simultaneous and robot == 'robot2'
                    and 'robot1' in self.frontier_prepared):
                first = self.frontier_prepared['robot1']
                first_point = first['pose'].pose.position
                ranked = [entry for entry in ranked
                          if int(entry[0].frontier_id) != first['frontier_id']
                          and math.hypot(
                              entry[0].approach_pose.pose.position.x
                              - first_point.x,
                              entry[0].approach_pose.pose.position.y
                              - first_point.y) >= 0.8]
            if not ranked:
                return
            candidate, safe, batch, age = ranked[0]
            started = self.now_sim()
            pose = candidate.approach_pose
            pose.header.stamp = self.get_clock().now().to_msg()
            _, source_context, _ = self._handoff_context(robot, candidate, batch)

            def validated(success, path, length, latency, code, message,
                          robot=robot, candidate=candidate, safe=safe,
                          batch=batch, age=age, started=started,
                          source_context=source_context):
                current_age = self._age(robot, 'candidate')
                current = self.candidates[robot]
                signature = candidate_physical_signature(candidate)
                current_match = None if current is None else next(
                    (item for item in current.candidates
                     if candidate_physical_signature(item) == signature), None)
                if current_age is None or current_age > self.FRONTIER_STALE_S:
                    self._record_handoff(
                        robot, candidate, current or batch,
                        'CANDIDATE_TOO_OLD', safe, length,
                        f'ERROR_{code}:{message}',
                        source_context=source_context)
                    return
                if current_match is None:
                    self._record_handoff(
                        robot, candidate, current or batch,
                        'TASK_NO_LONGER_PRESENT', safe, length,
                        f'ERROR_{code}:{message}',
                        source_context=source_context)
                    return
                final_safe, reason = self._handoff_safe_goal(
                    robot, current_match, current)
                if reason is not None:
                    self._record_handoff(
                        robot, current_match, current, reason, final_safe,
                        length, f'ERROR_{code}:{message}',
                        source_context=source_context)
                    return
                if not success:
                    self._record_handoff(
                        robot, current_match, current, 'FINAL_PATH_FAILED',
                        final_safe, length, f'ERROR_{code}:{message}',
                        source_context=source_context)
                    return
                _, context, map_change = self._handoff_context(
                    robot, current_match, current, source_context)
                # The successfully replanned path is now the provenance.  A
                # remote map revision must not suppress a safe current path.
                prepared = {
                    'pose': pose, 'path': path, 'path_length': length,
                    'path_latency': latency, 'safe': final_safe,
                    'candidate_age': current_age, 'selection_time':
                        self.now_sim() - started,
                    'map_revision': int(current.map_revision),
                    'frontier_id': int(current_match.frontier_id),
                    'physical_signature': signature,
                    'path_provenance': {
                        'map_revision': int(current.map_revision),
                        'map_stamp_s': self._grid_stamp(self.shared_maps[robot]),
                        'costmap_stamp_s': self._grid_stamp(self.costmaps[robot]),
                        'map_change_classification': map_change,
                        'source_map_revision': context.get('source_map_revision'),
                    },
                }
                self.frontier_prepared[robot] = prepared
                self._event(
                    'FRONTIER_SELECTED_AND_VALIDATED', robot,
                    frontier_id=int(current_match.frontier_id),
                    physical_signature=signature,
                    map_revision=int(current.map_revision),
                    map_change_classification=map_change,
                    visible_reveal_gain=float(current_match.information_gain),
                    path_length_m=length, candidate_age_s=current_age,
                    selection_time_s=prepared['selection_time'],
                    final_path_validation_latency_s=latency)

            self._request_path(
                robot, pose, 'frontier_final_validation',
                {'frontier_id': int(candidate.frontier_id)}, validated)
            return
        if simultaneous:
            first = self.frontier_prepared['robot1']
            second = self.frontier_prepared['robot2']
            one = first['pose'].pose.position
            two = second['pose'].pose.position
            same_id = first['frontier_id'] == second['frontier_id']
            separation = math.hypot(one.x - two.x, one.y - two.y)
            if same_id or separation < 0.8:
                self.frontier_prepared.pop('robot2')
                self._event(
                    'FRONTIER_PAIR_REJECTED', severity='WARN',
                    reason='TASK_SIGNATURE_OR_SEPARATION',
                    same_task_id=same_id, separation_m=separation)
                return
        for robot, details in list(self.frontier_prepared.items()):
            handoff = self.now_sim() - (
                self.candidate_received_sim[robot] or self.now_sim())
            if handoff > 5.0:
                self._trigger('NAV2_FRONTIER_HANDOFF_DELAY', robot,
                              delay_s=handoff,
                              frontier_id=details['frontier_id'])
            label = ('frontier_simultaneous' if simultaneous
                     else f'frontier_{robot}')
            self._dispatch_navigation(robot, details, 'frontier', label)
        self.frontier_prepared = {}
        self.frontier_index += 1

    def _sample_processes(self):
        try:
            for process in psutil.process_iter(['pid', 'cmdline']):
                command = ' '.join(process.info.get('cmdline') or [])
                if 'frontier_candidate_generator' not in command:
                    continue
                for robot in ROBOTS:
                    if f'__ns:=/{robot}' in command or f'/{robot}' in command:
                        self.frontier_pid[robot] = process.info['pid']
        except (psutil.Error, OSError):
            pass

    def _process_cpu(self, robot: str):
        pid = self.frontier_pid[robot]
        if pid is None:
            return None
        try:
            times = psutil.Process(pid).cpu_times()
            return times.user + times.system
        except psutil.Error:
            return None

    def _finish(self):
        if self.completed:
            return
        self.completed = True
        self._cancel_all('mission complete')
        for phase in self.phase_coverage:
            for robot in ROBOTS:
                if self.phase_coverage[phase][robot]['end'] is None:
                    self.phase_coverage[phase][robot]['end'] = known_cell_count(
                        self.shared_maps[robot])
        for robot in ROBOTS:
            if self.candidate_cpu_end[robot] is None:
                self.candidate_cpu_end[robot] = self._process_cpu(robot)
        summary = self._summary()
        (self.output / 'diagnostic_summary.json').write_text(
            json.dumps(summary, indent=2, sort_keys=True) + '\n',
            encoding='utf-8')
        self._event('DIAGNOSTIC_COMPLETE', passed=summary['passed'],
                    acceptance=summary['acceptance'])
        self.events_file.flush()
        self.timeseries_file.flush()
        self.handoff_file.flush()
        self.get_logger().info(
            'NAV2_FRONTIER_DIAGNOSTIC_COMPLETE passed=%s output=%s'
            % (str(summary['passed']).lower(), self.output))
        self.tick_timer.cancel()
        self.sample_timer.cancel()
        self.lifecycle_timer.cancel()
        self.process_timer.cancel()
        self.wall_watchdog_timer.cancel()
        # Let the final event reach rosout, then main observes completed.

    def _summary(self):
        elapsed = 0.0 if self.ready_sim is None else min(
            self.mission_duration, self.mission_elapsed())
        filter_table = {}
        frontier_table = {}
        time_breakdown = {}
        for robot in ROBOTS:
            fixed_hz = self._frequency(self.scan_times[robot]['fixed'])
            slam_hz = self._frequency(self.scan_times[robot]['slam'])
            observations = dict(self.filter_observations[robot])
            latencies = self.scan_latencies[robot]
            filter_table[robot] = {
                'fixed_scan_hz': fixed_hz,
                'slam_scan_hz': slam_hz,
                'input_to_output_latency_mean_s': (
                    sum(latencies) / len(latencies) if latencies else None),
                'input_to_output_latency_max_s': max(latencies, default=None),
                'map_publication_hz': self._frequency(self.map_times[robot]),
                'stationary_map_growth_cells': (
                    None if self.stationary_known_end[robot] is None
                    else self.stationary_known_end[robot]
                    - self.stationary_known_start[robot]),
                'runtime_metrics': dict(self.filter_stats[robot]),
                'scan_observations': observations,
                'adequate_for_slam_toolbox': slam_hz >= self.FILTER_MIN_HZ,
                'zero_degraded_or_unfiltered_outputs': (
                    observations.get('unsafe_output', 0) == 0
                    and self._publisher_safe(robot)),
            }
            intervals = self.candidate_intervals[robot]
            generations = self.candidate_generation[robot]
            stats = dict(self.candidate_stats[robot])
            cpu_start = self.candidate_cpu_start[robot]
            cpu_end = self.candidate_cpu_end[robot]
            frontier_table[robot] = {
                'snapshot_hz': (1.0 / (sum(intervals) / len(intervals)))
                if intervals and sum(intervals) > 0 else 0.0,
                'snapshot_age_s': self._age(robot, 'candidate'),
                'snapshot_count': stats.get('snapshots', 0),
                'frontier_region_count': sum(
                    item.get('regions', 0) for item in generations),
                'generated_approach_count': stats.get('approaches', 0),
                'valid_path_count': stats.get('valid_paths', 0),
                'unreachable_count': stats.get('unreachable', 0),
                'stale_result_count': max(
                    [item.get('stale_results', 0) for item in generations]
                    or [0]),
                'suppressed_count': stats.get('suppressed', 0),
                'compute_path_query_count': sum(
                    item.get('queries', 0) for item in generations),
                'reused_local_path_count': stats.get('reused_local_paths', 0),
                'candidate_generation_duration_max_s': max(
                    self.candidate_cycle_durations[robot] or [0.0]),
                'extraction_duration_max_s': max(
                    [item.get('extraction_ms', 0) / 1000.0
                     for item in generations] or [0.0]),
                'path_query_latency_mean_s': (
                    sum(self.candidate_cycle_durations[robot])
                    / max(1, sum(item.get('queries', 0)
                                 for item in generations))),
                'unchanged_snapshot_count': stats.get(
                    'unchanged_snapshots', 0),
                'cpu_time_s': (
                    cpu_end - cpu_start if cpu_start is not None
                    and cpu_end is not None else None),
                'path_queries_serialized': True,
                'planner_busy_churn': self.failure_events['PATH_QUERY_BUSY'],
                'finite_nonnegative_gain_count': stats.get(
                    'visible_gain_finite_nonnegative', 0),
                'visible_gain_records': self.candidate_gain_records[robot],
                'invalid_candidate_reasons': dict(
                    self.candidate_invalid_reasons[robot]),
                'nav2_service_unavailable_samples': stats.get(
                    'nav2_service_unavailable_samples', 0),
            }
            seconds = self.time_state_seconds[robot]
            time_breakdown[robot] = {
                state: {
                    'seconds': seconds[state],
                    'percent': (100.0 * seconds[state] / elapsed)
                    if elapsed > 0.0 else 0.0,
                } for state in TIME_STATES
            }
        navigation_table = [self._run_summary(run) for run in self.nav_runs]
        acceptance = self._acceptance(
            filter_table, frontier_table, navigation_table)
        return {
            'schema_version': 1,
            'launch_file': LAUNCH_FILE,
            'launch_chain': [
                LAUNCH_FILE, 'two_robots_frontier_candidates_launch.py',
                'two_robots_teammate_filtered_stack_launch.py',
                'two_robots_teammate_filtered_dual_slam_launch.py',
                'Webots + corrected scans + rewritten teammate filters + '
                'Slam Toolbox + map export/fusion + Nav2 + frontier generators',
            ],
            'excluded_components': [
                'distributed_pair_assignment',
                'distributed assignment protocols', 'task snapshots',
                'assignment agreement', 'allocator goal dispatch',
                'old collector', 'old observer'],
            'start_time': self.started_utc,
            'ready_time': self.ready_utc,
            'startup_duration_wall_s': None if self.ready_wall is None
            else self.ready_wall - self.started_wall,
            'mission_duration_requested_s': self.mission_duration,
            'phase_profile': self.phase_profile,
            'phase_boundaries_s': dict(zip(
                ('readiness', 'filter_end', 'nav2_end', 'candidate_end',
                 'integration_end'), PHASE_BOUNDARIES[self.phase_profile])),
            'mission_duration_observed_s': elapsed,
            'world_profile': self.world_profile,
            'readiness': self.readiness_report,
            'phases': self.phase_coverage,
            'filter_performance': filter_table,
            'navigation_requests': navigation_table,
            'frontier_performance': frontier_table,
            'handoff_rejection_counts': dict(self.handoff_rejections),
            'handoff_rejection_record_count': len(self.handoff_records),
            'handoff_rejection_csv': 'handoff_rejections.csv',
            'time_breakdown': time_breakdown,
            'failure_triggers': dict(self.failure_events),
            'planner_failure_storm': self.planner_failure_storm,
            'robot_start_occupied': self.start_occupied,
            'start_gate_prevented_path_requests':
                self.start_gate_prevented_requests,
            'start_cell_provenance': self.start_cell_stats,
            'start_obstacle_clearance': self.start_clearance_stats,
            'acceptance': acceptance,
            'passed': all(acceptance.values()),
            'diagnostic_completed': self.ready_sim is not None
            and elapsed >= self.mission_duration - 0.11,
            'selector_contract': [
                'valid current path', 'visible reveal gain descending',
                'shorter path length', 'deterministic task ID tie-break'],
            'wait_classifications': list(WAIT_CLASSIFICATIONS),
            'thresholds': {
                'filter_min_hz': self.FILTER_MIN_HZ,
                'scan_fresh_s': self.FRESH_SCAN_S,
                'map_fresh_s': self.FRESH_MAP_S,
                'costmap_fresh_s': self.FRESH_COSTMAP_S,
                'planner_latency_high_s': self.PLANNER_HIGH_S,
                'controller_min_hz': self.CONTROLLER_MIN_HZ,
                'zero_command_warn_s': self.ZERO_COMMAND_WARN_S,
                'frontier_stale_s': self.FRONTIER_STALE_S,
            },
        }

    @staticmethod
    def _run_summary(run: NavigationRun):
        duration = None if run.terminal_sim is None \
            or run.accepted_sim is None else run.terminal_sim - run.accepted_sim
        return {
            'robot': run.robot, 'kind': run.kind, 'label': run.label,
            'request_time_s': run.request_sim,
            'acceptance_latency_s': None if run.accepted_sim is None
            else run.accepted_sim - run.request_sim,
            'compute_path_latency_s': run.path_latency_s,
            'path_length_m': run.path_length_m,
            'estimated_travel_cost': run.path_length_m,
            'first_nonzero_cmd_vel_latency_s': None
            if run.first_motion_sim is None or run.accepted_sim is None
            else run.first_motion_sim - run.accepted_sim,
            'controller_update_hz': run.cmd_count / max(duration, 1e-9)
            if duration is not None else 0.0,
            'zero_cmd_feedback_percent': 100.0 * run.zero_feedback_intervals
            / max(1, run.feedback_intervals),
            'odometry_distance_m': run.odom_distance_m,
            'execution_duration_s': duration,
            'recovery_count': run.maximum_recovery_count,
            'result_status': run.result_status,
            'result_code': run.result_code,
            'result_message': run.result_message,
            'success': run.result_status == GoalStatus.STATUS_SUCCEEDED
            and run.result_code == NavigateToPose.Result.NONE,
            'final_pose_error_m': run.final_pose_error_m,
            'costmap_value_at_start': run.start_cost,
            'costmap_value_at_goal': run.goal_cost,
            'nearest_obstacle_distance_m': run.clearance_m,
            'tf_age_s': run.tf_age_s, 'map_age_s': run.map_age_s,
            'costmap_age_s': run.costmap_age_s,
            'classifications': dict(run.classifications),
            'coverage_start_cells': run.coverage_start_cells,
            'coverage_end_cells': run.coverage_end_cells,
            'coverage_growth_cells': None if run.coverage_end_cells is None
            else run.coverage_end_cells - run.coverage_start_cells,
            'new_frontier_snapshot_after_completion':
                run.next_candidate_delay_s is not None,
            'terminal_to_next_usable_candidate_s':
                run.next_candidate_delay_s,
            **run.metadata,
        }

    def _acceptance(self, filters, frontiers, navigation):
        def success(robot, kind, label_contains=None):
            return any(
                item['robot'] == robot and item['kind'] == kind
                and item['success']
                and (label_contains is None
                     or label_contains in item['label'])
                for item in navigation)

        robot2_runs = [item for item in navigation
                       if item['robot'] == 'robot2']
        return {
            'both_filters_publish_safe': all(
                filters[robot]['zero_degraded_or_unfiltered_outputs']
                and filters[robot]['scan_observations'].get(
                    'matched_output', 0) > 0 for robot in ROBOTS),
            'slam_scan_frequency_adequate': all(
                filters[robot]['adequate_for_slam_toolbox']
                for robot in ROBOTS),
            'both_maps_grow_and_remain_fresh': all(
                self.phase_coverage['FILTER_AND_MAPPING'][robot]['start']
                is not None
                and known_cell_count(self.shared_maps[robot])
                > self.phase_coverage['FILTER_AND_MAPPING'][robot]['start']
                and self._age(robot, 'shared_map') is not None
                and self._age(robot, 'shared_map') <= self.FRESH_MAP_S
                for robot in ROBOTS),
            'both_nav2_short_manual_success': all(
                success(robot, 'manual', 'short') for robot in ROBOTS),
            'both_frontier_goal_success': all(
                success(robot, 'frontier') for robot in ROBOTS),
            'robot2_motion_observed': any(
                item['odometry_distance_m'] > 0.01
                and item['first_nonzero_cmd_vel_latency_s'] is not None
                for item in robot2_runs),
            'accepted_goals_not_mostly_zero_cmd': all(
                item['zero_cmd_feedback_percent'] < 50.0
                for item in navigation if item['acceptance_latency_s'] is not None),
            'no_planner_failure_storm': not any(
                self.planner_failure_storm.values()),
            'no_start_cell_occupied': not any(self.start_occupied.values()),
            'frontier_generation_responsive': all(
                frontiers[robot]['snapshot_hz'] >= 0.15
                and frontiers[robot]['candidate_generation_duration_max_s']
                <= self.FRONTIER_SLOW_S for robot in ROBOTS),
            'path_queries_serialized': all(
                frontiers[robot]['path_queries_serialized'] for robot in ROBOTS),
            'no_stale_frontier_dispatch':
                self.failure_events['STALE_PATH_BEFORE_DISPATCH'] == 0,
            'candidate_generation_did_not_starve_controller': all(
                item['controller_update_hz'] >= self.CONTROLLER_MIN_HZ
                for item in navigation if item['kind'] == 'frontier'
                and item['execution_duration_s'] is not None),
            'time_breakdown_complete': all(
                set(self.time_state_seconds[robot]) == set(TIME_STATES)
                for robot in ROBOTS),
        }

    def close_artifacts(self):
        """Close streaming artifacts after the executor stops."""
        if not self.events_file.closed:
            self.events_file.close()
        if not self.timeseries_file.closed:
            self.timeseries_file.close()
        if not self.handoff_file.closed:
            self.handoff_file.close()


def main(args=None):
    """Run the ROS diagnostic node."""
    rclpy.init(args=args)
    node = DiagnosticNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.completed:
            executor.spin_once(timeout_sec=0.1)
        # Flush final publisher/log callbacks without extending sim mission time.
        deadline = time.monotonic() + 0.5
        while rclpy.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_artifacts()
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


class RunnerError(RuntimeError):
    """Bounded runner preflight or execution failure."""


def boolean(value: str | bool) -> bool:
    """Parse explicit true/false CLI values."""
    if isinstance(value, bool):
        return value
    lowered = value.lower()
    if lowered in ('true', '1', 'yes', 'on'):
        return True
    if lowered in ('false', '0', 'no', 'off'):
        return False
    raise argparse.ArgumentTypeError('expected true or false')


def runner_parser() -> argparse.ArgumentParser:
    """Build the lean diagnostic runner CLI."""
    result = argparse.ArgumentParser(
        description='Run the allocator-free Nav2/frontier diagnostic')
    result.add_argument('--world-profile', choices=('small', 'large'),
                        default='large')
    result.add_argument('--world-path', default='')
    result.add_argument('--execution-profile',
                        choices=('headless', 'visual'), default='headless')
    result.add_argument('--sensor-profile', choices=('full', 'throughput'),
                        default='full')
    result.add_argument('--time-mode', choices=('sim',), default='sim')
    result.add_argument('--fast-mode', type=boolean, default=True)
    result.add_argument('--rendering', type=boolean)
    result.add_argument('--rviz', type=boolean)
    result.add_argument('--mission-timeout', type=float, default=600.0)
    result.add_argument('--phase-profile', choices=('short', 'full'),
                        default='full')
    result.add_argument('--startup-timeout', type=float, default=300.0)
    result.add_argument('--emergency-wall-runtime', type=float, default=1200.0)
    result.add_argument('--ros-domain-id', type=int, default=232)
    result.add_argument('--webots-port', type=int, default=23667)
    result.add_argument(
        '--fusion-cpu-quota-percent', type=float, default=30.0,
        help=('systemd CPUQuota for fusion processes; 0 disables it '
              '(default: 30 for host protection)'))
    result.add_argument(
        '--cpu-core-limit', type=int, default=4,
        help=('limit the diagnostic process tree to this many CPU cores; '
              '0 disables the affinity limit (default: 4)'))
    result.add_argument(
        '--results-directory',
        default='results/nav2_frontier_diagnostic')
    return result


def resolve_world(args: argparse.Namespace) -> Path:
    """Resolve an explicit or profile-selected source world."""
    if args.world_path:
        world = Path(args.world_path).expanduser().resolve()
        if not world.is_file():
            raise RunnerError(f'world path does not exist: {world}')
        return world
    selected = profile(
        args.world_profile, WORKSPACE / 'src' / PACKAGE / 'worlds')
    return Path(selected['world_path']).resolve()


def package_prefix() -> Path:
    """Require the intended workspace package installation."""
    ros2 = shutil.which('ros2')
    if ros2 is None:
        raise RunnerError('ros2 is not available in PATH')
    result = subprocess.run(
        [ros2, 'pkg', 'prefix', PACKAGE], check=False,
        capture_output=True, text=True, timeout=10)
    prefix = Path(result.stdout.strip()) if result.stdout.strip() else None
    expected = WORKSPACE / 'install' / PACKAGE
    if result.returncode != 0 or prefix is None:
        raise RunnerError('ROS package lookup failed: ' + result.stderr.strip())
    if prefix.resolve() != expected.resolve():
        raise RunnerError(
            f'{PACKAGE} resolves to {prefix}, expected {expected}; '
            'build and source this workspace first')
    return prefix


def port_is_free(port: int) -> bool:
    """Check the requested external-controller TCP endpoint."""
    if not 1024 <= port <= 65535:
        raise RunnerError('--webots-port must be between 1024 and 65535')
    with socket.socket() as stream:
        stream.settimeout(0.25)
        return stream.connect_ex(('127.0.0.1', port)) != 0


def launch_command(args: argparse.Namespace, world: Path,
                   attempt: Path) -> list[str]:
    """Return the exact allocator-free launch command."""
    rendering = args.rendering
    if rendering is None:
        rendering = args.execution_profile == 'visual'
    return [
        'ros2', 'launch', PACKAGE, LAUNCH_FILE,
        f'world_profile:={args.world_profile}', f'world_path:={world}',
        f'webots_port:={args.webots_port}',
        f'webots_mode:={"fast" if args.fast_mode else "realtime"}',
        f'webots_gui:={str(rendering).lower()}',
        f'sensor_profile:={args.sensor_profile}',
        f'output_directory:={attempt}',
        f'mission_duration_s:={args.mission_timeout}',
        f'startup_timeout_s:={args.startup_timeout}',
        f'phase_profile:={args.phase_profile}',
        f'fusion_cpu_quota_percent:={args.fusion_cpu_quota_percent:g}',
    ]


def rviz_command() -> list[str]:
    """Return the existing cooperative visualization command."""
    config = (WORKSPACE / 'install' / PACKAGE / 'share' / PACKAGE
              / 'resource' / 'cooperative_manual_exploration.rviz')
    return ['rviz2', '-d', str(config), '--ros-args',
            '-p', 'use_sim_time:=true']


def _pump(stream, log, lock):
    try:
        for line in iter(stream.readline, ''):
            with lock:
                log.write(line)
                log.flush()
                print(line, end='', flush=True)
    finally:
        stream.close()


def _stop(process, sig):
    if process is not None and process.poll() is None:
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass


def _wait(process, timeout):
    if process is None:
        return True
    try:
        process.wait(timeout=timeout)
        return True
    except subprocess.TimeoutExpired:
        return False


def _cleanup(launch, rviz, owned_pids):
    _stop(launch, signal.SIGINT)
    _stop(rviz, signal.SIGINT)
    if not _wait(launch, 20.0):
        _stop(launch, signal.SIGTERM)
        if not _wait(launch, 10.0):
            _stop(launch, signal.SIGKILL)
            _wait(launch, 5.0)
    if not _wait(rviz, 5.0):
        _stop(rviz, signal.SIGTERM)
        if not _wait(rviz, 5.0):
            _stop(rviz, signal.SIGKILL)
    # The Webots controller wrapper can exit before its Linux driver child,
    # reparenting that child outside ros2 launch's process group.  Only reap
    # PIDs previously observed as descendants of this exact attempt.
    survivors = []
    for pid in owned_pids:
        try:
            process = psutil.Process(pid)
            if process.is_running() and process.status() != psutil.STATUS_ZOMBIE:
                process.terminate()
                survivors.append(process)
        except psutil.Error:
            pass
    _, alive = psutil.wait_procs(survivors, timeout=5.0)
    for process in alive:
        try:
            process.kill()
        except psutil.Error:
            pass
    psutil.wait_procs(alive, timeout=2.0)


def _affinity_preexec(core_limit):
    """Return a child hook that bounds the complete diagnostic process tree."""
    if core_limit <= 0 or not hasattr(os, 'sched_setaffinity'):
        return None
    allowed = sorted(os.sched_getaffinity(0))
    selected = set(allowed[:min(core_limit, len(allowed))])
    if not selected:
        return None

    def set_affinity():
        os.sched_setaffinity(0, selected)

    return set_affinity


def runner_run(args: argparse.Namespace) -> int:
    """Run one diagnostic attempt and produce exactly five artifacts."""
    package_prefix()
    world = resolve_world(args)
    if not port_is_free(args.webots_port):
        raise RunnerError(f'Webots port is already in use: {args.webots_port}')
    root = Path(args.results_directory).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    attempt = root / f'{args.execution_profile}_{stamp}'
    attempt.mkdir()
    command = launch_command(args, world, attempt)
    show_rviz = args.rviz
    if show_rviz is None:
        show_rviz = args.execution_profile == 'visual'
    effective = (
        f'ROS_DOMAIN_ID={args.ros_domain_id} '
        + ' '.join(shlex.quote(item) for item in command) + '\n')
    if show_rviz:
        effective += (
            f'ROS_DOMAIN_ID={args.ros_domain_id} '
            + ' '.join(shlex.quote(item) for item in rviz_command()) + '\n')
    (attempt / 'effective_command.txt').write_text(
        effective, encoding='utf-8')
    print(f'launch_file={LAUNCH_FILE}', flush=True)
    print(f'launch_chain={LAUNCH_FILE} -> '
          'two_robots_frontier_candidates_launch.py -> '
          'two_robots_teammate_filtered_stack_launch.py -> '
          'two_robots_teammate_filtered_dual_slam_launch.py', flush=True)
    print(f'world_path={world}', flush=True)
    print(f'results_directory={attempt}', flush=True)
    environment = os.environ.copy()
    environment['ROS_DOMAIN_ID'] = str(args.ros_domain_id)
    environment['PYTHONUNBUFFERED'] = '1'
    launch = None
    rviz = None
    threads = []
    owned_pids = set()
    lock = threading.Lock()
    started = time.monotonic()
    preexec = _affinity_preexec(args.cpu_core_limit)
    with (attempt / 'launch.log').open('w', encoding='utf-8') as log:
        try:
            launch = subprocess.Popen(
                command, env=environment, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, bufsize=1,
                start_new_session=True, preexec_fn=preexec)
            threads.append(threading.Thread(
                target=_pump, args=(launch.stdout, log, lock), daemon=True))
            threads[-1].start()
            if show_rviz:
                rviz = subprocess.Popen(
                    rviz_command(), env=environment, stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT, text=True, bufsize=1,
                    start_new_session=True, preexec_fn=preexec)
                threads.append(threading.Thread(
                    target=_pump, args=(rviz.stdout, log, lock), daemon=True))
                threads[-1].start()
            while launch.poll() is None:
                try:
                    owned_pids.update(
                        child.pid for child in psutil.Process(
                            launch.pid).children(recursive=True))
                except psutil.Error:
                    pass
                if time.monotonic() - started > args.emergency_wall_runtime:
                    raise RunnerError(
                        'emergency wall runtime exceeded: '
                        f'{args.emergency_wall_runtime}s')
                time.sleep(0.2)
        finally:
            _cleanup(launch, rviz, owned_pids)
            for thread in threads:
                thread.join(timeout=2.0)
    required = ARTIFACT_NAMES
    actual = {path.name for path in attempt.iterdir() if path.is_file()}
    missing = required - actual
    extras = actual - required
    if missing:
        raise RunnerError(f'missing diagnostic artifacts: {sorted(missing)}')
    if extras:
        raise RunnerError(f'unexpected diagnostic artifacts: {sorted(extras)}')
    summary = json.loads(
        (attempt / 'diagnostic_summary.json').read_text(encoding='utf-8'))
    print('diagnostic_passed=' + str(summary.get('passed', False)).lower(),
          flush=True)
    # A short requested smoke run validates runtime completion, not the
    # 600-second acceptance criteria.  Full runs return nonzero on evidence
    # failure so automation cannot mistake a completed launch for a pass.
    if args.mission_timeout < 600.0:
        return 0 if summary.get('diagnostic_completed') else 1
    return 0 if summary.get('passed') else 1


def runner_main(argv=None) -> int:
    """CLI entry point for the source-tree runner."""
    args = runner_parser().parse_args(argv)
    if args.ros_domain_id < 0 or args.ros_domain_id > 232:
        raise SystemExit('--ros-domain-id must be between 0 and 232')
    if args.mission_timeout <= 0.0:
        raise SystemExit('--mission-timeout must be positive')
    if args.startup_timeout <= 0.0:
        raise SystemExit('--startup-timeout must be positive')
    if args.emergency_wall_runtime <= args.startup_timeout:
        raise SystemExit(
            '--emergency-wall-runtime must exceed --startup-timeout')
    if args.cpu_core_limit < 0:
        raise SystemExit('--cpu-core-limit must be zero or positive')
    try:
        return runner_run(args)
    except (RunnerError, OSError, subprocess.SubprocessError) as error:
        print(f'NAV2_FRONTIER_DIAGNOSTIC_FAILURE reason={error}',
              file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    main()
