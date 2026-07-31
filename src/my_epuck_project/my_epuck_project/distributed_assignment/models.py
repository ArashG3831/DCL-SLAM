"""Transport-independent models for exactly two assignment peers."""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple


Point = Tuple[float, float]


@dataclass(frozen=True)
class Bounds:
    """Axis-aligned world bounds for compact frontier geometry."""

    minimum: Point
    maximum: Point


@dataclass(frozen=True)
class PhysicalTask:
    """One source robot's physical observation-task proposal."""

    source_robot_id: str
    source_session_id: str
    source_snapshot_epoch: int
    source_map_revision: int
    physical_signature: str
    local_frontier_id: int
    centroid: Point
    bounds: Bounds
    approach: Point
    approach_yaw: float = 0.0
    frontier_geometry: Tuple[Point, ...] = ()
    visible_cells: Tuple[Point, ...] = ()
    visible_bounds: Optional[Bounds] = None
    visible_reveal_gain: float = 0.0
    local_ordering_score: float = 0.0
    local_path_valid: bool = False
    generation_ros_ns: int = 0


@dataclass(frozen=True)
class TaskSnapshot:
    """Bounded task snapshot with ROS provenance and sender TTL."""

    source_robot_id: str
    source_session_id: str
    epoch: int
    map_revision: int
    map_fingerprint: str
    generation_ros_ns: int
    validity_s: float
    tasks: Tuple[PhysicalTask, ...]


@dataclass(frozen=True)
class CanonicalTask:
    """Deterministic union task produced from equivalent source proposals."""

    canonical_id: str
    members: Tuple[PhysicalTask, ...]
    centroid: Point
    bounds: Bounds
    approach: Point
    approach_yaw: float
    frontier_geometry: Tuple[Point, ...]
    visible_cells: Tuple[Point, ...]
    visible_bounds: Optional[Bounds]
    visible_reveal_gain: float


@dataclass(frozen=True)
class CanonicalUnion:
    """Canonical ordered physical task set for one coordination round."""

    tasks: Tuple[CanonicalTask, ...]
    union_hash: str


@dataclass(frozen=True)
class Bid:
    """One robot's bounded local Nav2 evaluation of a canonical task."""

    canonical_task_id: str
    path_valid: bool
    path_length_m: float
    estimated_travel_cost: float
    heading_cost: float = 0.0
    own_utility_contribution: float = 0.0
    task_generation_ros_ns: int = 0
    path_query_ros_ns: int = 0
    path: Tuple[Point, ...] = ()


@dataclass(frozen=True)
class BidBatch:
    """Round-bound bid array from one robot."""

    round_id: str
    union_hash: str
    source_robot_id: str
    source_session_id: str
    source_snapshot_epoch: int
    validity_s: float
    bids: Tuple[Bid, ...]


@dataclass(frozen=True)
class AssignmentScore:
    """Bounded, individually auditable pair-score terms."""

    team_visible_gain: float = 0.0
    combined_path_cost: float = 0.0
    nearby_goal_penalty: float = 0.0
    route_overlap_penalty: float = 0.0
    hard_failure_penalty: float = 0.0
    sensing_overlap_penalty: float = 0.0
    workload_imbalance_penalty: float = 0.0
    total: float = 0.0


@dataclass(frozen=True)
class PairDecision:
    """Complete replicated assignment for Robot 1 and Robot 2."""

    round_id: str
    union_hash: str
    robot1_task_id: str
    robot2_task_id: str
    robot1_bid_fingerprint: str
    robot2_bid_fingerprint: str
    score: AssignmentScore
    decision_hash: str
    combined_path_length_m: float
    maximum_path_length_m: float


class FailureClass(str, Enum):
    """Evidence-conservative navigation failure classes."""

    HARD_UNREACHABLE = 'HARD_UNREACHABLE'
    PLANNER_FAILURE = 'PLANNER_FAILURE'
    CONTROLLER_NO_PROGRESS = 'CONTROLLER_NO_PROGRESS'
    DYNAMIC_BLOCKAGE = 'DYNAMIC_BLOCKAGE'
    TF_OR_LIFECYCLE = 'TF_OR_LIFECYCLE'
    ACTION_REJECTION = 'ACTION_REJECTION'
    TIMEOUT = 'TIMEOUT'
    EXPLICIT_CANCELLATION = 'EXPLICIT_CANCELLATION'
    UNKNOWN = 'UNKNOWN'


@dataclass(frozen=True)
class FailureEvidence:
    """Only directly observed evidence used for classification."""

    compute_path_error: Optional[str] = None
    nav2_error_code: Optional[int] = None
    nav2_error_message: str = ''
    controller_no_progress: bool = False
    dynamic_obstacle_confirmed: bool = False
    tf_unavailable: bool = False
    lifecycle_inactive: bool = False
    action_rejected: bool = False
    timed_out: bool = False
    explicitly_cancelled: bool = False


@dataclass(frozen=True)
class FailureRecord:
    """Bounded team-visible failure evidence for one physical task."""

    source_robot_id: str
    source_session_id: str
    round_id: str
    canonical_task_id: str
    physical_signature: str
    approach: Point
    failure_class: FailureClass
    retry_count: int
    validity_s: float
    alternative_approach: bool = False


class CoordinatorState(str, Enum):
    """Small public coordinator lifecycle."""

    WAITING_FOR_INPUTS = 'WAITING_FOR_INPUTS'
    BIDDING = 'BIDDING'
    WAITING_FOR_MATCHING_DECISION = 'WAITING_FOR_MATCHING_DECISION'
    NAVIGATING = 'NAVIGATING'
    DEGRADED_SOLO = 'DEGRADED_SOLO'
    COMPLETE = 'COMPLETE'
    BLOCKED = 'BLOCKED'


@dataclass(frozen=True)
class CompletionInputs:
    """Health and stability evidence required for operational completion."""

    both_snapshots_fresh: bool
    both_statuses_fresh: bool
    robot1_has_valid_task: bool
    robot2_has_valid_task: bool
    valid_pair_exists: bool
    shared_maps_stable: bool
    active_assignment: bool
    local_nav_goal_active: bool
    peer_nav_goal_active: bool
    tf_healthy: bool
    nav2_healthy: bool
    candidates_healthy: bool
    communication_healthy: bool
    peer_completion_matches: bool
    condition_duration_s: float
    confirmation_interval_s: float


@dataclass
class TravelDistance:
    """Accumulate odometric distance while rejecting discontinuous jumps."""

    maximum_step_m: float = 1.0
    distance_m: float = 0.0
    _previous: Optional[Point] = field(default=None, repr=False)

    def observe(self, point: Point) -> float:
        """Add one valid odometry displacement and return the total."""
        import math

        if self._previous is not None:
            step = math.dist(self._previous, point)
            if 0.0 <= step <= self.maximum_step_m:
                self.distance_m += step
        self._previous = point
        return self.distance_m
