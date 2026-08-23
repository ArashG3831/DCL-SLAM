"""Replicated peer-to-peer two-robot frontier assignment ROS node."""

from dataclasses import dataclass, field, replace
import hashlib
import json
import math
import os
import time
from typing import Optional

from my_epuck_interfaces.msg import (
    DistributedExplorationEvent,
    DistributedExplorationStatus,
    ExplorationFailure,
    FrontierCandidateArray,
    PairDecision as PairDecisionMsg,
    TaskBidArray as TaskBidArrayMsg,
    TaskSnapshot as TaskSnapshotMsg,
    RelativePoseHypothesis,
)

import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .distributed_assignment.canonical import (
    build_canonical_union,
    canonical_round_id,
    TaskIdentity,
)
from .distributed_assignment.failures import (
    HARD_FAILURES, bounded_suppression_duration,
)
from .distributed_assignment.local_nav2 import (
    classify_dispatch_precondition_failure,
    DispatchPreconditions,
    LocalNav2,
    NavigationOutcome,
    PathEvaluation,
)
from .distributed_assignment.models import (
    Bid,
    BidBatch,
    CanonicalTask,
    CanonicalUnion,
    CoordinatorState,
    FailureClass,
    PairDecision,
    PhysicalTask,
    TaskSnapshot,
)
from .distributed_assignment.protocol import (
    bid_batch_valid,
    CommittedRound,
    PeerLiveness,
    receive,
    Received,
    SnapshotLedger,
)
from .distributed_assignment.ros_conversion import (
    bid_batch_from_msg,
    bid_batch_to_msg,
    decision_to_msg,
    duration_to_seconds,
    seconds_to_duration,
    snapshot_from_msg,
    text_to_uuid,
    uuid_to_text,
)
from .distributed_assignment.scoring import (
    AssignmentWeights,
    choose_pair_assignment,
)
from .distributed_assignment.burgard_assignment import choose_burgard_assignment
from .distributed_assignment.traffic_scheduler import TrafficDecision, schedule_traffic
from .round_lifecycle import RoundGeneration
from .mission_termination import (
    CandidateEvidence,
    FrontierRegionEvidence,
    TerminalReason,
    classify_empty_frontiers,
    summarize_frontier_regions,
    terminal_reason_is_success,
)


@dataclass
class RoundWork:
    """Mutable bounded work for one uncommitted canonical round."""

    round_id: str
    union: CanonicalUnion
    snapshots: tuple[TaskSnapshot, TaskSnapshot]
    query_tasks: tuple[CanonicalTask, ...]
    content_fingerprint: str = ''
    query_index: int = 0
    bids: tuple[Bid, ...] = ()
    local_path_evaluations: dict[str, PathEvaluation] = field(default_factory=dict)
    local_batch: Optional[BidBatch] = None
    decision: Optional[PairDecision] = None
    traffic: Optional[TrafficDecision] = None
    decision_published: bool = False


def round_pass_is_current(current_round, expected_round) -> bool:
    """Return whether a threaded tick still owns its round snapshot."""
    return current_round is expected_round


@dataclass
class TrafficHold:
    """A local deferred dispatch bound to one agreed traffic reservation."""

    round_id: str
    decision_hash: str
    winner_robot_id: str
    snapshot_epochs: tuple[int, int]
    created_steady_s: float
    winner_observed_active: bool = False


STATE_TO_MESSAGE = {
    CoordinatorState.WAITING_FOR_INPUTS: DistributedExplorationStatus.WAITING_FOR_INPUTS,
    CoordinatorState.BIDDING: DistributedExplorationStatus.BIDDING,
    CoordinatorState.WAITING_FOR_MATCHING_DECISION:
        DistributedExplorationStatus.WAITING_FOR_MATCHING_DECISION,
    CoordinatorState.WAITING_FOR_TRAFFIC:
        DistributedExplorationStatus.WAITING_FOR_TRAFFIC,
    CoordinatorState.NAVIGATING: DistributedExplorationStatus.NAVIGATING,
    CoordinatorState.DEGRADED_SOLO: DistributedExplorationStatus.DEGRADED_SOLO,
    CoordinatorState.COMPLETE: DistributedExplorationStatus.COMPLETE,
    CoordinatorState.BLOCKED: DistributedExplorationStatus.BLOCKED,
}


FAILURE_TO_MESSAGE = {
    FailureClass.HARD_UNREACHABLE: ExplorationFailure.HARD_UNREACHABLE,
    FailureClass.PLANNER_FAILURE: ExplorationFailure.PLANNER_FAILURE,
    FailureClass.CONTROLLER_NO_PROGRESS: ExplorationFailure.CONTROLLER_NO_PROGRESS,
    FailureClass.DYNAMIC_BLOCKAGE: ExplorationFailure.DYNAMIC_BLOCKAGE,
    FailureClass.TF_OR_LIFECYCLE: ExplorationFailure.TF_OR_LIFECYCLE,
    FailureClass.ACTION_REJECTION: ExplorationFailure.ACTION_REJECTION,
    FailureClass.TIMEOUT: ExplorationFailure.TIMEOUT,
    FailureClass.EXPLICIT_CANCELLATION: ExplorationFailure.EXPLICIT_CANCELLATION,
    FailureClass.UNKNOWN: ExplorationFailure.UNKNOWN,
}


def eligible_solo_tasks(
        tasks: tuple[PhysicalTask, ...],
        hard_failure_signatures: set[str],
        completed_signatures: set[str],
        minimum_visible_gain_m: float,
        minimum_ordering_score: float,
        maximum_path_m: float) -> tuple[PhysicalTask, ...]:
    """Return locally dispatchable tasks after bounded physical suppression."""
    return tuple(
        task for task in tasks
        if task.physical_signature not in hard_failure_signatures
        and task.physical_signature not in completed_signatures
        and task.visible_reveal_gain >= minimum_visible_gain_m
        and task.local_ordering_score >= minimum_ordering_score
        and (
            task.local_path_length_m <= 0.0 or
            task.local_path_length_m <= maximum_path_m
        )
    )


class DistributedFrontierAssignment(Node):
    """Compute a complete pair decision independently and dispatch only locally."""

    def __init__(self):
        """Configure one equal peer with identity-bound topic and Nav2 interfaces."""
        super().__init__('distributed_frontier_assignment')
        self._robot_id = self.declare_parameter('robot_id', '').value
        if self._robot_id not in ('robot1', 'robot2'):
            raise ValueError('robot_id must be exactly robot1 or robot2')
        self._peer_id = 'robot2' if self._robot_id == 'robot1' else 'robot1'
        self._local_only = bool(self.declare_parameter('local_only', False).value)
        self._handoff_gated = bool(self.declare_parameter(
            'handoff_gated', False).value)
        self._stop_after_handoff = bool(self.declare_parameter(
            'stop_after_handoff', False).value)
        self._phase_gated = bool(self.declare_parameter(
            'phase_gated', False).value)
        self._handoff_complete = False
        self._dispatch_enabled = bool(
            self.declare_parameter('dispatch_enabled', False).value,
        )
        self._mission_timeout_enabled = bool(
            self.declare_parameter('enable_mission_timeout', False).value,
        )
        self._mission_timeout_s = float(
            self.declare_parameter('mission_timeout_s', 0.0).value,
        )
        self._planner_failure_confirmation_s = float(
            self.declare_parameter('planner_failure_confirmation_s', 30.0).value,
        )
        self._mission_started_steady_s = time.monotonic()
        self._synthetic_bids = bool(
            self.declare_parameter('synthetic_bids', False).value,
        )
        self._synthetic_origin = (
            float(self.declare_parameter('synthetic_origin_x', 0.0).value),
            float(self.declare_parameter('synthetic_origin_y', 0.0).value),
        )
        self._maximum_union_tasks = int(
            # Each peer advertises at most K tasks.  The canonical union must
            # therefore admit the full two-peer bound 2K before equivalence
            # clustering (K=5 in the final launch).
            self.declare_parameter('maximum_union_tasks', 10).value,
        )
        self._maximum_path_queries = int(
            self.declare_parameter('maximum_path_queries', 8).value,
        )
        self._snapshot_maximum_tasks = int(
            self.declare_parameter('maximum_tasks_per_source', 5).value,
        )
        self._bid_validity_s = float(self.declare_parameter('bid_validity_s', 3.0).value)
        self._decision_validity_s = float(
            self.declare_parameter('decision_validity_s', 3.0).value,
        )
        peer_timeout_s = float(self.declare_parameter('peer_timeout_s', 6.0).value)
        self._minimum_solo_visible_gain_m = float(
            self.declare_parameter('minimum_solo_visible_gain_m', 0.05).value,
        )
        self._minimum_solo_ordering_score = float(
            self.declare_parameter('minimum_solo_ordering_score', 0.0).value,
        )
        self._maximum_solo_path_m = float(
            self.declare_parameter('maximum_solo_path_m', 18.0).value,
        )
        self._post_goal_settle_s = float(
            self.declare_parameter('post_goal_settle_s', 1.0).value,
        )
        self._map_stability_grace_s = float(
            self.declare_parameter('map_stability_grace_s', 5.0).value,
        )
        self._completion_confirmation_s = float(
            self.declare_parameter('completion_confirmation_s', 4.0).value,
        )
        self._terminal_small_frontier_length_m = float(
            self.declare_parameter(
                'terminal_small_frontier_length_m', 0.20,
            ).value,
        )
        if self._terminal_small_frontier_length_m <= 0.0:
            raise ValueError('terminal_small_frontier_length_m must be positive')
        self._assignment_strategy = str(
            self.declare_parameter('assignment_strategy', 'burgard').value,
        )
        if self._assignment_strategy not in ('burgard', 'legacy_weighted'):
            raise ValueError('assignment_strategy must be burgard or legacy_weighted')
        self._burgard_beta = float(self.declare_parameter('burgard_beta', 1.0).value)
        self._burgard_sensor_max_range_m = float(self.declare_parameter(
            'burgard_sensor_max_range_m', 11.98,
        ).value)
        self._burgard_occupied_threshold = int(self.declare_parameter(
            'burgard_occupied_threshold', 50,
        ).value)
        self._burgard_allow_missing_map_for_test = bool(self.declare_parameter(
            'burgard_allow_missing_map_for_test', False,
        ).value)
        self._traffic_scheduler_enabled = bool(self.declare_parameter(
            'traffic_scheduler_enabled', False,
        ).value)
        self._traffic_robot1_safe_radius_m = float(self.declare_parameter(
            'traffic_robot1_safe_radius_m', 0.08,
        ).value)
        self._traffic_robot2_safe_radius_m = float(self.declare_parameter(
            'traffic_robot2_safe_radius_m', 0.08,
        ).value)
        self._traffic_reference_speed_mps = float(self.declare_parameter(
            'traffic_reference_speed_mps', 0.13,
        ).value)
        self._traffic_eta_tie_s = float(self.declare_parameter(
            'traffic_eta_tie_s', 0.05,
        ).value)
        self._traffic_dispatch_grace_s = float(self.declare_parameter(
            'traffic_dispatch_grace_s', 8.0,
        ).value)
        self._executor_threads = max(1, int(self.declare_parameter(
            'executor_threads', 4).value))
        if (self._burgard_beta < 0.0 or self._burgard_sensor_max_range_m <= 0.0 or
                self._traffic_robot1_safe_radius_m <= 0.0 or
                self._traffic_robot2_safe_radius_m <= 0.0 or
                self._traffic_reference_speed_mps <= 0.0):
            raise ValueError('Burgard and traffic parameters must be positive')
        self._weights = AssignmentWeights(
            gain=float(self.declare_parameter('weight_gain', 3.0).value),
            path=float(self.declare_parameter('weight_path', 1.0).value),
            nearby_goal=float(self.declare_parameter('weight_nearby_goal', 1.5).value),
            route_overlap=float(self.declare_parameter('weight_route_overlap', 3.0).value),
            hard_failure=float(self.declare_parameter('weight_hard_failure', 5.0).value),
            sensing_overlap=float(
                self.declare_parameter('weight_sensing_overlap', 2.0).value,
            ),
            workload_imbalance=float(
                self.declare_parameter('weight_workload_imbalance', 0.35).value,
            ),
            visible_gain_scale=float(
                self.declare_parameter('visible_gain_scale', 5.0).value,
            ),
            path_cost_scale_m=float(
                self.declare_parameter('path_cost_scale_m', 12.0).value,
            ),
            nearby_goal_distance_m=float(
                self.declare_parameter('nearby_goal_distance_m', 0.6).value,
            ),
            route_corridor_radius_m=float(
                self.declare_parameter('route_corridor_radius_m', 0.16).value,
            ),
            minimum_visible_gain_m=self._minimum_solo_visible_gain_m,
            maximum_path_length_m=self._maximum_solo_path_m,
        )
        self._ledger = SnapshotLedger(self._snapshot_maximum_tasks)
        self._snapshots: dict[str, Received[TaskSnapshot]] = {}
        self._bid_batches: dict[str, Received[BidBatch]] = {}
        self._peer_decision: Optional[Received[PairDecisionMsg]] = None
        self._peer_status: Optional[Received[DistributedExplorationStatus]] = None
        self._hard_failure_signatures: dict[str, float] = {}
        # Keep bounded, evidence-based local/peer failure history so a hard
        # failure cannot immediately re-enter the next auction under the
        # same physical signature.  The duration escalates for repeated hard
        # evidence, while transient classes never enter this table.
        self._hard_failure_counts: dict[str, int] = {}
        # A successfully completed local-only frontier remains suppressed
        # while the generator continues to publish the same physical region.
        # Without this bounded set, resetting the semantic snapshot key after
        # every success can immediately redispatch a tiny residual frontier.
        self._completed_solo_physical_signatures: set[str] = set()
        # Diagnostics-only counters.  These do not alter eligibility or
        # suppression; they separate structural controller failures from
        # transient TF/infrastructure failures for forensic replay.
        self._failure_task_diagnostics: dict[str, dict[str, object]] = {}
        self._peer_liveness = PeerLiveness(peer_timeout_s)
        self._round: Optional[RoundWork] = None
        self._round_lifecycle = RoundGeneration()
        self._round_created_count = 0
        self._round_completed_count = 0
        self._round_replaced_count = 0
        self._stale_tick_discard_count = 0
        self._dispatch_count = 0
        self._last_tick_steady_s = time.monotonic()
        self._last_round_completion_steady_s = 0.0
        self._last_tick_log_key = None
        self._coordinator_alive = True
        self._committed = CommittedRound()
        self._state = CoordinatorState.WAITING_FOR_INPUTS
        self._state_reason = 'startup'
        self._dispatch_in_progress = False
        self._settle_until_steady_s = 0.0
        self._active_task: Optional[CanonicalTask] = None
        self._active_round_id = ''
        self._active_decision_hash = ''
        self._traffic_hold: Optional[TrafficHold] = None
        # Local-only work is keyed by semantic task content, not heartbeat
        # epoch.  The key is cleared on a terminal result or failure so a
        # fresh path/action attempt still occurs when the previous attempt
        # actually ended.
        self._last_solo_snapshot_key: Optional[tuple[str, str]] = None
        self._map_versions: dict[str, tuple[str, int, str]] = {}
        self._maps_stable_since_steady_s = time.monotonic()
        self._completion_candidate_since_steady_s: Optional[float] = None
        self._completion_candidate_reason = ''
        self._planner_failure_candidate_since_steady_s: Optional[float] = None
        self._terminal = False
        self._terminal_success = False
        self._terminal_reason = ''
        self._terminal_epoch = 0
        self._terminal_finalization_attempt_count = 0
        self._terminal_commit_count = 0
        self._last_semantic_fingerprint = ''
        self._candidate_evidence = {
            robot: CandidateEvidence() for robot in ('robot1', 'robot2')
        }
        self._candidate_region_snapshots: dict[
            str, tuple[FrontierRegionEvidence, ...]] = {}
        self._candidate_evidence_seen = set()
        self._nav2 = LocalNav2(self, phase_gated=self._phase_gated)
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # The C++ candidate generator publishes volatile data.  Keep the
        # evidence subscription compatible with that live stream; task
        # snapshots/bids/decisions remain transient-local protocol data.
        candidate_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        hypothesis_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            # Accepted hypotheses are one-shot volatile announcements from
            # the frontend; do not request transient-local durability.
            durability=DurabilityPolicy.VOLATILE,
        )
        task_snapshot_topic = str(self.declare_parameter(
            'task_snapshot_topic', 'task_snapshot').value)
        candidate_topic = str(self.declare_parameter(
            'candidate_topic', 'frontier_candidates').value)
        robot_ids = (self._robot_id,) if self._local_only else ('robot1', 'robot2')
        self._phase_subscriptions = []
        self._phase_protocol = {
            'qos': qos,
            'candidate_qos': candidate_qos,
            'task_snapshot_topic': task_snapshot_topic,
            'candidate_topic': candidate_topic,
            'robot_ids': robot_ids,
        }
        if not self._phase_gated:
            self._activate_protocol_inputs()
        if self._handoff_gated or self._stop_after_handoff:
            self.create_subscription(
                RelativePoseHypothesis, '/cslam/relative_pose/hypotheses',
                self._handoff_callback, hypothesis_qos,
            )
        self._bid_publisher = self.create_publisher(TaskBidArrayMsg, 'task_bids', qos)
        self._decision_publisher = self.create_publisher(
            PairDecisionMsg, 'pair_decision', qos,
        )
        self._status_publisher = self.create_publisher(
            DistributedExplorationStatus, 'distributed_status', qos,
        )
        self._failure_publisher = self.create_publisher(
            ExplorationFailure, 'exploration_failure', qos,
        )
        self._event_publisher = self.create_publisher(
            DistributedExplorationEvent, 'distributed_event', 50,
        )
        self._tick_timer = None
        self._status_timer = None
        if not self._phase_gated:
            self._activate_assignment_timers()
        interfaces = self._nav2.interface_names()
        self.get_logger().info(
            'DISTRIBUTED_ASSIGNMENT robot=%s peer=%s dispatch=%s strategy=%s '
            'beta=%.3f path_limit=%.3f sensor_range=%.3f traffic=%s '
            'terminal_small_frontier_length_m=%.3f '
            'traffic_radii=(%.3f,%.3f) traffic_speed=%.3f compute=%s navigate=%s '
            'local_only=%s no_peer_clients=%s no_cmd_vel=true' % (
                self._robot_id, self._peer_id, self._dispatch_enabled,
                self._assignment_strategy, self._burgard_beta,
                self._maximum_solo_path_m, self._burgard_sensor_max_range_m,
                self._traffic_scheduler_enabled,
                self._terminal_small_frontier_length_m,
                self._traffic_robot1_safe_radius_m,
                self._traffic_robot2_safe_radius_m, self._traffic_reference_speed_mps,
                interfaces['compute_path'], interfaces['navigate'],
                self._local_only, self._local_only,
            )
        )

    def _activate_protocol_inputs(self) -> None:
        """Subscribe to task/bid traffic only in the active shared phase."""
        if self._phase_subscriptions:
            return
        qos = self._phase_protocol['qos']
        candidate_qos = self._phase_protocol['candidate_qos']
        task_snapshot_topic = self._phase_protocol['task_snapshot_topic']
        candidate_topic = self._phase_protocol['candidate_topic']
        for robot_id in self._phase_protocol['robot_ids']:
            self._phase_subscriptions.extend([
                self.create_subscription(
                    FrontierCandidateArray,
                    candidate_topic if self._local_only else
                    f'/{robot_id}/frontier_candidates',
                    self._candidate_callback, candidate_qos),
                self.create_subscription(
                    TaskSnapshotMsg,
                    task_snapshot_topic if self._local_only else
                    f'/{robot_id}/task_snapshot',
                    self._snapshot_callback, qos),
            ])
        if not self._local_only:
            for robot_id in ('robot1', 'robot2'):
                self._phase_subscriptions.extend([
                    self.create_subscription(
                        TaskBidArrayMsg, f'/{robot_id}/task_bids',
                        self._bid_callback, qos),
                    self.create_subscription(
                        PairDecisionMsg, f'/{robot_id}/pair_decision',
                        self._decision_callback, qos),
                    self.create_subscription(
                        DistributedExplorationStatus,
                        f'/{robot_id}/distributed_status',
                        self._status_callback, qos),
                    self.create_subscription(
                        ExplorationFailure,
                        f'/{robot_id}/exploration_failure',
                        self._failure_callback, qos),
                ])

    def _activate_assignment_timers(self) -> None:
        """Start assignment work only after the canonical handoff."""
        if self._tick_timer is None:
            tick_period_s = max(0.05, float(os.environ.get(
                'MY_EPUCK_ASSIGNMENT_TICK_PERIOD_S', '0.1')))
            self._tick_timer = self.create_timer(tick_period_s, self._tick)
        if self._status_timer is None:
            self._status_timer = self.create_timer(1.0, self._publish_status)

    def _activate_shared_phase(self) -> None:
        if not self._phase_gated:
            return
        self._phase_gated = False
        self._activate_protocol_inputs()
        self._nav2._activate_phase_inputs()
        self._activate_assignment_timers()
        self.get_logger().info(
            'UNKNOWN_POSE_PHASE shared_assignment_active=true '
            'protocol_inputs=true timers=true')

    def _handoff_callback(self, message: RelativePoseHypothesis) -> None:
        """Switch local-only dispatch off only after canonical acceptance."""
        if not bool(message.accepted) or str(message.status) != 'ACCEPTED':
            return
        if self._handoff_complete:
            return
        self._handoff_complete = True
        self._activate_shared_phase()
        self._dispatch_enabled = False if self._local_only else True
        if self._local_only and self._nav2.local_goal_active:
            self._nav2.cancel_navigation()
        if self._local_only and self._stop_after_handoff:
            self._stop_local_phase()
        self.get_logger().info(
            'UNKNOWN_POSE_PHASE robot=%s phase=POST_HANDOFF dispatch=%s' %
            (self._robot_id, self._dispatch_enabled))

    def _stop_local_phase(self) -> None:
        """Stop pre-handoff allocation work after canonical handoff."""
        for subscription in self._phase_subscriptions:
            self.destroy_subscription(subscription)
        self._phase_subscriptions = []
        if self._tick_timer is not None:
            self._tick_timer.cancel()
            self._tick_timer = None
        if self._status_timer is not None:
            self._status_timer.cancel()
            self._status_timer = None
        self._round = None
        self._dispatch_in_progress = False
        self._state = CoordinatorState.WAITING_FOR_INPUTS
        self._state_reason = 'pre-handoff local phase stopped'
        self.get_logger().info(
            'FRONTIER_PHASE pre_handoff_stopped=true reason=ACCEPTED_HANDOFF')

    def _candidate_callback(self, message: FrontierCandidateArray) -> None:
        """Keep bounded generator evidence separate from reachable task bids."""
        if message.source_robot_id not in self._candidate_evidence:
            return
        self._candidate_evidence_seen.add(message.source_robot_id)
        fallback = CandidateEvidence(
            detected=int(message.detected_frontier_count),
            small=int(message.small_frontier_count),
            reachable=sum(
                1 for candidate in message.candidates
                if candidate.reachability_state == candidate.REACHABLE
            ),
            out_of_range=int(message.out_of_range_frontier_count),
            unreachable=int(message.unreachable_frontier_count),
            planner_failures=int(message.planner_failure_count),
            unclassified=int(message.unclassified_frontier_count),
            detected_not_queried=int(
                getattr(message, 'detected_not_queried_count',
                        message.unclassified_frontier_count)),
            below_minimum_gain=sum(
                1 for candidate in message.candidates
                if candidate.reachability_state == candidate.REACHABLE and
                candidate.information_gain < self._minimum_solo_visible_gain_m
            ),
            actionable_reachable=sum(
                1 for candidate in message.candidates
                if candidate.reachability_state == candidate.REACHABLE and
                candidate.information_gain >= self._minimum_solo_visible_gain_m
            ),
        )
        raw_regions = str(getattr(
            message, 'terminal_frontier_regions_json', '') or '')
        if not raw_regions:
            raw_regions = str(getattr(
                message, 'diagnostic_regions_json', '') or '')
        regions = self._decode_frontier_regions(raw_regions)
        if regions:
            self._candidate_region_snapshots[message.source_robot_id] = regions
            if set(self._candidate_region_snapshots) == {'robot1', 'robot2'}:
                combined = summarize_frontier_regions(
                    self._candidate_region_snapshots['robot1'] +
                    self._candidate_region_snapshots['robot2'],
                    self._terminal_small_frontier_length_m,
                    self._minimum_solo_visible_gain_m,
                )
                # Store the unique physical union once.  This keeps status
                # reporting and terminal classification from summing peer
                # replicas as if they were separate frontiers.
                self._candidate_evidence['robot1'] = combined
                self._candidate_evidence['robot2'] = CandidateEvidence()
                return
        self._candidate_evidence[message.source_robot_id] = fallback

    @staticmethod
    def _decode_frontier_regions(value: str) -> tuple[FrontierRegionEvidence, ...]:
        """Decode compact diagnostic region geometry; malformed data is ignored."""
        if not value:
            return ()
        try:
            payload = json.loads(value)
        except (TypeError, ValueError):
            return ()
        if isinstance(payload, dict):
            payload = payload.get('regions', [])
        if not isinstance(payload, list):
            return ()
        output = []
        for item in payload:
            if not isinstance(item, dict):
                continue
            physical_id = str(item.get('physical_id', item.get('id', '')))
            if not physical_id:
                continue
            try:
                size_m = float(item.get('size_m', 0.0))
                if size_m <= 0.0:
                    # Backward-compatible decoding for older diagnostic
                    # captures, whose schema used the production 0.03 m map.
                    size_m = float(item.get('cell_count', 0.0)) * 0.03
                gain_value = item.get('visible_reveal_gain')
                visible_reveal_gain = (
                    float(gain_value) if gain_value is not None else None)
            except (TypeError, ValueError):
                continue
            output.append(FrontierRegionEvidence(
                physical_id=physical_id,
                size_m=max(0.0, size_m),
                status=str(item.get('status', 'UNCLASSIFIED')),
                visible_reveal_gain=visible_reveal_gain,
            ))
        return tuple(output)
    def _snapshot_callback(self, message: TaskSnapshotMsg) -> None:
        """Accept only bounded monotonic source-local snapshot provenance."""
        try:
            snapshot = snapshot_from_msg(message)
        except (ValueError, TypeError) as error:
            self.get_logger().error('TASK_SNAPSHOT_REJECTED decode=%s' % error)
            return
        now = time.monotonic()
        previous = self._snapshots.get(snapshot.source_robot_id)
        if not self._ledger.accept(snapshot):
            # An exact immutable retransmission is a heartbeat, not a new epoch.
            if previous is not None and previous.value == snapshot:
                self._snapshots[snapshot.source_robot_id] = receive(
                    snapshot, snapshot.validity_s, now,
                )
                if snapshot.source_robot_id == self._peer_id:
                    self._peer_liveness.observe(snapshot.source_session_id, now)
            return
        map_version = (
            snapshot.source_session_id, snapshot.map_revision,
            snapshot.map_fingerprint,
        )
        if self._map_versions.get(snapshot.source_robot_id) != map_version:
            self._map_versions[snapshot.source_robot_id] = map_version
            self._maps_stable_since_steady_s = now
            self._completion_candidate_since_steady_s = None
            self._completion_candidate_reason = ''
            self._planner_failure_candidate_since_steady_s = None
        self._snapshots[snapshot.source_robot_id] = receive(
            snapshot, snapshot.validity_s, now,
        )
        if snapshot.source_robot_id == self._peer_id:
            previous_session = (
                '' if previous is None else previous.value.source_session_id
            )
            if previous_session and previous_session != snapshot.source_session_id:
                self._emit_event(
                    'PEER_SESSION_RESTART',
                    'fresh peer session superseded prior session',
                )
                if self._nav2.local_goal_active:
                    self._nav2.cancel_navigation()
                elif self._dispatch_in_progress:
                    self._invalidate_round(
                        FailureClass.EXPLICIT_CANCELLATION,
                        'peer session restarted before local dispatch',
                    )
            previous_state = self._peer_liveness.state
            self._peer_liveness.observe(snapshot.source_session_id, now)
            if (previous_state == CoordinatorState.DEGRADED_SOLO and
                    self._peer_liveness.state != CoordinatorState.DEGRADED_SOLO):
                self._reset_round('fresh peer session handshake')

    def _bid_callback(self, message: TaskBidArrayMsg) -> None:
        try:
            batch = bid_batch_from_msg(message)
        except (ValueError, TypeError) as error:
            self.get_logger().error('BID_REJECTED decode=%s' % error)
            return
        if batch.source_robot_id not in ('robot1', 'robot2'):
            return
        self._bid_batches[batch.source_robot_id] = receive(
            batch, batch.validity_s, time.monotonic(),
        )

    def _decision_callback(self, message: PairDecisionMsg) -> None:
        if message.source_robot_id != self._peer_id:
            return
        self._peer_decision = receive(
            message, duration_to_seconds(message.validity), time.monotonic(),
        )

    def _status_callback(self, message: DistributedExplorationStatus) -> None:
        if message.source_robot_id != self._peer_id:
            return
        peer_session = uuid_to_text(message.source_session_id)
        if peer_session:
            # Status is the peer liveness heartbeat.  Candidate snapshots are
            # proposals and may legitimately stop while the peer navigates.
            self._peer_liveness.observe(peer_session, time.monotonic())
        self._peer_status = receive(
            message, duration_to_seconds(message.validity), time.monotonic(),
        )

    def _failure_callback(self, message: ExplorationFailure) -> None:
        if message.source_robot_id == self._robot_id:
            return
        hard_values = {FAILURE_TO_MESSAGE[item] for item in HARD_FAILURES}
        if (message.failure_class in hard_values and
                message.physical_task_signature):
            self._record_hard_failure(
                message.physical_task_signature,
                duration_to_seconds(message.validity),
            )

    def _record_hard_failure(self, signature: str, requested_ttl_s: float) -> None:
        """Record bounded suppression for directly observed hard evidence."""
        if not signature:
            return
        now = time.monotonic()
        count = self._hard_failure_counts.get(signature, 0) + 1
        self._hard_failure_counts[signature] = count
        ttl = bounded_suppression_duration(count, requested_ttl_s)
        self._hard_failure_signatures[signature] = max(
            self._hard_failure_signatures.get(signature, 0.0), now + ttl,
        )
        if len(self._hard_failure_counts) > 128:
            expired = [
                key for key in self._hard_failure_counts
                if key not in self._hard_failure_signatures
            ]
            if expired:
                self._hard_failure_counts.pop(expired[0], None)
        # Keep this diagnostic bounded and auditable without making failure
        # suppression dependent on logger timing.
        self.get_logger().info(
            'HARD_FAILURE_SUPPRESSION signature=%s count=%d ttl_s=%.3f' %
            (signature, count, ttl),
        )

    def _fresh_snapshot(self, robot_id: str, now: float) -> Optional[TaskSnapshot]:
        received = self._snapshots.get(robot_id)
        return received.value if received is not None and received.fresh(now) else None

    @staticmethod
    def _snapshot_content_fingerprint(
            first: TaskSnapshot, second: TaskSnapshot) -> str:
        """Fingerprint task content while ignoring epoch-only heartbeats."""
        payload = []
        for snapshot in (first, second):
            tasks = []
            for task in sorted(snapshot.tasks, key=lambda item: item.physical_signature):
                tasks.append((
                    task.physical_signature,
                    tuple(round(value, 3) for value in task.approach),
                    round(task.approach_yaw, 3),
                    tuple(round(value, 3) for value in task.bounds.minimum),
                    tuple(round(value, 3) for value in task.bounds.maximum),
                    round(task.visible_reveal_gain, 4),
                    round(task.local_ordering_score, 4),
                    bool(task.local_path_valid),
                    round(task.local_path_length_m, 3),
                ))
            payload.append((snapshot.source_robot_id, tuple(tasks)))
        return hashlib.sha256(repr(tuple(payload)).encode('utf-8')).hexdigest()

    def _round_is_current(self, round_work: RoundWork, generation: int) -> bool:
        """Check both object identity and generation before committing work."""
        return (
            self._round is round_work and
            self._round_lifecycle.generation == generation and
            self._round_lifecycle.active_round_id == round_work.round_id
        )

    def _discard_stale_tick(
            self, round_work: Optional[RoundWork], captured_generation: int,
            reason: str) -> None:
        """Account for asynchronous work that no longer owns the round."""
        self._stale_tick_discard_count += 1
        current_round = self._round
        self.get_logger().warning(
            'ALLOCATOR_TICK_STALE_DISCARDED robot=%s round_id=%s '
            'captured_generation=%d current_round_id=%s current_generation=%d '
            'reason=%s' % (
                self._robot_id,
                '' if round_work is None else round_work.round_id,
                captured_generation,
                '' if current_round is None else current_round.round_id,
                self._round_lifecycle.generation,
                reason,
            ),
        )

    def _log_round_lifecycle(
            self, event: str, round_work: Optional[RoundWork] = None,
            generation: Optional[int] = None, reason: str = '') -> None:
        """Emit compact lifecycle telemetry for post-run race auditing."""
        current = self._round if round_work is None else round_work
        active_generation = self._round_lifecycle.generation
        logged_generation = active_generation if generation is None else generation
        self.get_logger().info(
            '%s robot=%s round_id=%s generation=%d current_generation=%d '
            'timestamp=%.6f reason=%s' % (
                event, self._robot_id,
                '' if current is None else current.round_id,
                logged_generation, active_generation, time.monotonic(), reason,
            ),
        )

    def _activate_round(self, round_work: RoundWork, reason: str) -> int:
        """Install a round and return its generation token."""
        previous = self._round
        lease = self._round_lifecycle.activate(round_work.round_id)
        self._round = round_work
        self._round_created_count += 1
        if previous is None:
            self._log_round_lifecycle(
                'ALLOCATOR_ROUND_CREATED', round_work, lease.generation, reason,
            )
        else:
            self._round_replaced_count += 1
            self._log_round_lifecycle(
                'ALLOCATOR_ROUND_REPLACED', round_work, lease.generation, reason,
            )
        return lease.generation

    def _publish_health_diagnostic(self) -> None:
        """Publish liveness counters without changing allocator behavior."""
        now = time.monotonic()
        current = self._round
        last_tick_age = max(0.0, now - self._last_tick_steady_s)
        completion_age = (
            -1.0 if self._last_round_completion_steady_s <= 0.0 else
            max(0.0, now - self._last_round_completion_steady_s)
        )
        self.get_logger().info(
            'ALLOCATOR_HEALTH robot=%s coordinator_alive=%s current_round_id=%s '
            'current_generation=%d last_tick_age_s=%.3f '
            'last_round_completion_age_s=%.3f stale_tick_discard_count=%d '
            'rounds_created=%d rounds_completed=%d rounds_replaced=%d '
            'dispatch_count=%d' % (
                self._robot_id, self._coordinator_alive,
                '' if current is None else current.round_id,
                self._round_lifecycle.generation, last_tick_age, completion_age,
                self._stale_tick_discard_count, self._round_created_count,
                self._round_completed_count, self._round_replaced_count,
                self._dispatch_count,
            ),
        )

    def _tick(self) -> None:
        now = time.monotonic()
        self._last_tick_steady_s = now
        tick_round = self._round
        tick_generation = self._round_lifecycle.generation
        tick_key = (
            None if tick_round is None else tick_round.round_id,
            tick_generation,
        )
        if tick_key != self._last_tick_log_key:
            self._last_tick_log_key = tick_key
            self._log_round_lifecycle(
                'ALLOCATOR_TICK_STARTED', tick_round, tick_generation,
                'timer callback',
            )
        self._expire_failures(now)
        if self._local_only:
            if self._handoff_complete:
                return
            if self._nav2.local_goal_active or self._dispatch_in_progress:
                return
            local = self._fresh_snapshot(self._robot_id, now)
            if local is not None:
                self._continue_degraded_solo(local)
            return
        if self._terminal or self._state == CoordinatorState.COMPLETE:
            return
        peer_status = self._peer_status
        if (peer_status is not None and peer_status.fresh(now) and
                getattr(peer_status.value, 'terminal', False) and
                not self._nav2.local_goal_active and not self._dispatch_in_progress):
            peer_terminal_reason = str(
                getattr(peer_status.value, 'terminal_reason', ''),
            )
            if peer_terminal_reason.startswith('MISSION_ABORT_'):
                self._set_terminal(peer_terminal_reason, success=False)
                return
            if (terminal_reason_is_success(peer_terminal_reason) and
                    self._candidate_evidence_seen == {'robot1', 'robot2'}):
                local_reason = classify_empty_frontiers(
                    self._candidate_evidence['robot1'],
                    self._candidate_evidence['robot2'],
                )
                if (local_reason is not None and
                        local_reason.value == peer_terminal_reason):
                    self._set_terminal(peer_terminal_reason, success=True)
                    return
        if (self._mission_timeout_enabled and self._mission_timeout_s > 0.0 and
                now - self._mission_started_steady_s >= self._mission_timeout_s):
            if self._nav2.local_goal_active:
                self._nav2.cancel_navigation()
            self._set_terminal(TerminalReason.TIMEOUT.value, success=False)
            return
        if self._traffic_hold is not None:
            self._continue_traffic_hold(now)
            return
        if self._nav2.local_goal_active or self._dispatch_in_progress:
            return
        # A healthy peer goal is an already-issued local Nav2 commitment, not
        # a new candidate to be re-arbitrated.  Conservatively wait for its
        # terminal result and fresh proposals instead of issuing a conflicting
        # new goal or cancelling the peer's local action.  This is the active
        # robot priority rule; both replicas observe the same peer status.
        peer_status = self._peer_status
        if (peer_status is not None and peer_status.fresh(now) and
                (peer_status.value.local_nav_goal_active or
                 peer_status.value.state == DistributedExplorationStatus.NAVIGATING)):
            self._transition(
                CoordinatorState.WAITING_FOR_INPUTS,
                'peer active local NavigateToPose retains traffic priority',
            )
            return
        if now < self._settle_until_steady_s:
            return
        first = self._fresh_snapshot('robot1', now)
        second = self._fresh_snapshot('robot2', now)
        if first is None or second is None:
            peer_status = self._peer_status
            peer_status_fresh = peer_status is not None and peer_status.fresh(now)
            peer_navigating = bool(
                peer_status_fresh and
                (peer_status.value.local_nav_goal_active or
                 (peer_status.value.state == DistributedExplorationStatus.NAVIGATING and
                  bool(peer_status.value.active_canonical_task_id)))
            )
            if peer_navigating:
                self._transition(
                    CoordinatorState.WAITING_FOR_INPUTS,
                    'peer active assignment heartbeat; awaiting fresh proposal',
                )
                return
            if self._peer_liveness.evaluate(now) == CoordinatorState.DEGRADED_SOLO:
                self._transition(CoordinatorState.DEGRADED_SOLO, 'peer snapshot timeout')
                local = first if self._robot_id == 'robot1' else second
                if local is not None:
                    self._continue_degraded_solo(local)
            else:
                self._transition(
                    CoordinatorState.WAITING_FOR_INPUTS,
                    'fresh task snapshots from both source sessions required',
                )
            return
        # Terminal significance is evaluated from the unique physical
        # frontier evidence, not from whether tiny regions happened to become
        # allocator tasks.  This prevents a 0.06--0.18 m residual fragment
        # from being dispatched merely because it has a valid approach pose.
        if self._candidate_evidence_seen == {'robot1', 'robot2'}:
            terminal_candidate = classify_empty_frontiers(
                self._candidate_evidence['robot1'],
                self._candidate_evidence['robot2'],
            )
            if terminal_candidate is not None:
                if self._consider_completion(now):
                    return
                return
        round_id = canonical_round_id(
            TaskIdentity('robot1', first.source_session_id, first.epoch),
            TaskIdentity('robot2', second.source_session_id, second.epoch),
        )
        content_fingerprint = self._snapshot_content_fingerprint(first, second)
        current_round = self._round
        if (
                current_round is not None and current_round.decision is not None and
                not current_round.decision.robot1_task_id and
                not current_round.decision.robot2_task_id and
                current_round.content_fingerprint == content_fingerprint):
            # Heartbeat epochs can advance while the semantic task set stays
            # empty.  Revisit the persistent completion gate on every tick;
            # otherwise the first empty round could never reach confirmation.
            if self._consider_completion(now):
                return
            return
        if (self._round is None and self._last_semantic_fingerprint ==
                content_fingerprint):
            self._transition(
                CoordinatorState.WAITING_FOR_INPUTS,
                'unchanged semantic task content; no new planner round',
            )
            return
        if (
                current_round is not None and current_round.decision is not None and
                current_round.round_id != round_id and
                not current_round.decision.robot1_task_id and
                not current_round.decision.robot2_task_id and
                current_round.content_fingerprint == content_fingerprint):
            self._transition(
                CoordinatorState.WAITING_FOR_INPUTS,
                'unchanged IDLE task content; waiting for meaningful proposal change',
            )
            return
        if current_round is None or current_round.round_id != round_id:
            union = build_canonical_union(
                first.tasks, second.tasks, self._maximum_union_tasks,
            )
            new_round = RoundWork(
                round_id=round_id,
                union=union,
                snapshots=(first, second),
                query_tasks=union.tasks[:self._maximum_path_queries],
                content_fingerprint=content_fingerprint,
            )
            generation = self._activate_round(new_round, 'new canonical round')
            self._bid_batches.clear()
            self._peer_decision = None
            self._committed = CommittedRound()
            self._transition(CoordinatorState.BIDDING, 'new canonical round')
            self._log_union(new_round)
        # This node uses a MultiThreadedExecutor.  A navigation-terminal
        # callback may reset self._round while this timer callback is still
        # unwinding.  Keep the round object local for this pass and abandon
        # the stale pass if another callback replaced it.
        round_work = self._round
        generation = self._round_lifecycle.generation
        if round_work is None:
            return
        if round_work.union.tasks:
            self._completion_candidate_since_steady_s = None
        if round_work.local_batch is None:
            self._continue_bidding(round_work, generation)
            return
        if not self._round_is_current(round_work, generation):
            self._discard_stale_tick(round_work, generation, 'before bid validation')
            return
        if not self._both_bid_batches_valid(now, round_work):
            self._transition(CoordinatorState.BIDDING, 'waiting for valid peer bids')
            return
        if round_work.decision is None:
            first_batch = self._bid_batches['robot1'].value
            second_batch = self._bid_batches['robot2'].value
            hard_ids = self._hard_failed_task_ids(round_work.union)
            if self._assignment_strategy == 'burgard':
                if self._nav2.shared_map is None:
                    self.get_logger().warning(
                        'BURGARD_LOS_MAP_UNAVAILABLE round=%s; retaining range '
                        'utility reduction gate with zero reduction until a map exists',
                        round_work.round_id,
                    )
                decision = choose_burgard_assignment(
                    round_work.round_id, round_work.union,
                    first_batch, second_batch,
                    beta=self._burgard_beta,
                    maximum_path_length_m=self._maximum_solo_path_m,
                    minimum_visible_gain_m=self._minimum_solo_visible_gain_m,
                    sensor_max_range_m=self._burgard_sensor_max_range_m,
                    occupied_threshold=self._burgard_occupied_threshold,
                    shared_map=self._nav2.shared_map,
                    hard_failed_tasks=hard_ids,
                )
            else:
                decision = choose_pair_assignment(
                    round_work.round_id, round_work.union,
                    first_batch, second_batch, hard_ids, self._weights,
                )
            traffic = self._traffic_for_decision(decision, first_batch, second_batch)
            # Snapshot cardinalities are transport/provenance facts rather
            # than solver inputs.  Attach them here so the decision telemetry
            # can distinguish an empty source snapshot from a source whose
            # tasks were all rejected by local path validation.
            if not self._round_is_current(round_work, generation):
                self._discard_stale_tick(round_work, generation, 'before decision commit')
                return
            round_work.decision = replace(
                decision,
                diagnostics=replace(
                    decision.diagnostics,
                    robot1_snapshot_task_count=len(
                        round_work.snapshots[0].tasks,
                    ),
                    robot2_snapshot_task_count=len(round_work.snapshots[1].tasks),
                    traffic=traffic.as_dict(),
                ),
            )
            round_work.traffic = traffic
            self._publish_decision(round_work, generation)
            if not self._round_is_current(round_work, generation):
                self._discard_stale_tick(round_work, generation, 'after decision publication')
                return
            self._log_round_lifecycle(
                'ALLOCATOR_TICK_COMMITTED', round_work, generation,
                'decision published',
            )
            self._transition(
                CoordinatorState.WAITING_FOR_MATCHING_DECISION,
                'local complete pair decision published',
            )
        if not self._round_is_current(round_work, generation):
            self._discard_stale_tick(round_work, generation, 'before peer match')
            return
        if not self._matching_peer_decision(round_work, generation, now):
            return
        if not self._round_is_current(round_work, generation):
            self._discard_stale_tick(round_work, generation, 'after peer match')
            return
        if self._committed.decision is None:
            self._committed.commit(round_work.decision)
            self._last_semantic_fingerprint = round_work.content_fingerprint
            self._emit_event('DECISION_AGREED', 'positive replicated decision match')
        if (not round_work.decision.robot1_task_id and
                not round_work.decision.robot2_task_id and
                self._consider_completion(now)):
            return
        if not self._dispatch_enabled:
            self._transition(
                CoordinatorState.WAITING_FOR_MATCHING_DECISION,
                'decision agreed; dispatch disabled',
            )
            return
        if not self._round_is_current(round_work, generation):
            self._discard_stale_tick(round_work, generation, 'before local dispatch')
            return
        self._start_local_dispatch(round_work, generation)

    def _traffic_for_decision(
            self, decision: PairDecision, robot1_bids: BidBatch,
            robot2_bids: BidBatch) -> TrafficDecision:
        """Derive the same bounded traffic result from agreed bid geometry."""
        required = self._traffic_robot1_safe_radius_m + self._traffic_robot2_safe_radius_m
        if not self._traffic_scheduler_enabled:
            return TrafficDecision(required_separation_m=required, reason='DISABLED')
        if not decision.robot1_task_id or not decision.robot2_task_id:
            return TrafficDecision(required_separation_m=required, reason='SINGLE_ACTIVE_OR_IDLE')
        first = {bid.canonical_task_id: bid for bid in robot1_bids.bids}.get(
            decision.robot1_task_id,
        )
        second = {bid.canonical_task_id: bid for bid in robot2_bids.bids}.get(
            decision.robot2_task_id,
        )
        if first is None or second is None:
            return TrafficDecision(required_separation_m=required, reason='SELECTED_BID_MISSING')
        return schedule_traffic(
            first.path, second.path,
            robot1_safe_radius_m=self._traffic_robot1_safe_radius_m,
            robot2_safe_radius_m=self._traffic_robot2_safe_radius_m,
            reference_speed_mps=self._traffic_reference_speed_mps,
            eta_tie_s=self._traffic_eta_tie_s,
            # A matched round contains only new undispatched goals.  Keeping
            # asynchronous status observations out of this calculation makes
            # traffic evidence bit-for-bit reproducible at both replicas.
            active_robots=frozenset(),
        )

    def _traffic_reason(self, traffic: TrafficDecision) -> str:
        """Keep event/status evidence compact, structured, and deterministic."""
        return json.dumps(traffic.as_dict(), sort_keys=True, separators=(',', ':'))

    def _continue_traffic_hold(self, now: float) -> None:
        """Wait for winner terminal evidence, then require newer proposals."""
        hold = self._traffic_hold
        if hold is None:
            return
        peer = self._peer_status
        peer_active = bool(
            peer is not None and peer.fresh(now) and
            (peer.value.local_nav_goal_active or
             peer.value.state == DistributedExplorationStatus.NAVIGATING)
        )
        if peer_active:
            hold.winner_observed_active = True
            self._transition(
                CoordinatorState.WAITING_FOR_TRAFFIC,
                'traffic reservation held by active %s' % hold.winner_robot_id,
            )
            return
        if not hold.winner_observed_active and (
                now - hold.created_steady_s < self._traffic_dispatch_grace_s):
            self._transition(
                CoordinatorState.WAITING_FOR_TRAFFIC,
                'waiting for traffic winner %s dispatch heartbeat' % hold.winner_robot_id,
            )
            return
        first = self._fresh_snapshot('robot1', now)
        second = self._fresh_snapshot('robot2', now)
        if first is None or second is None:
            self._transition(
                CoordinatorState.WAITING_FOR_TRAFFIC,
                'traffic winner released; waiting for fresh task snapshots',
            )
            return
        if (first.epoch <= hold.snapshot_epochs[0] and
                second.epoch <= hold.snapshot_epochs[1]):
            self._transition(
                CoordinatorState.WAITING_FOR_TRAFFIC,
                'traffic winner released; stale deferred goal discarded pending newer proposals',
            )
            return
        waited = now - hold.created_steady_s
        self._emit_event(
            'TRAFFIC_RELEASED_FRESH_REALLOCATION',
            'winner terminal/released; stale deferred task will not dispatch',
            duration=waited,
        )
        self._traffic_hold = None
        self._reset_round('traffic reservation released; rebuilding from fresh proposals')

    def _begin_traffic_wait(self, traffic: TrafficDecision) -> None:
        """Defer only this new local goal; never inject a Nav2 velocity hold."""
        round_work = self._round
        if round_work is None or round_work.decision is None:
            return
        if self._traffic_hold is None:
            self._traffic_hold = TrafficHold(
                round_id=round_work.round_id,
                decision_hash=round_work.decision.decision_hash,
                winner_robot_id=traffic.winner_robot_id,
                snapshot_epochs=(round_work.snapshots[0].epoch,
                                 round_work.snapshots[1].epoch),
                created_steady_s=time.monotonic(),
            )
            self._emit_event('TRAFFIC_WAITING', self._traffic_reason(traffic))
        self._transition(
            CoordinatorState.WAITING_FOR_TRAFFIC,
            'traffic deferred local dispatch behind %s' % traffic.winner_robot_id,
        )

    def _consider_completion(self, now: float) -> bool:
        """Require matching healthy empty-round persistence before COMPLETE."""
        nav2_healthy, tf_healthy = self._nav2.health_flags()
        peer = self._peer_status
        peer_fresh = peer is not None and peer.fresh(now)
        peer_healthy = bool(
            peer_fresh and peer.value.nav2_healthy and peer.value.tf_healthy and
            peer.value.candidate_source_healthy and
            peer.value.peer_communication_healthy and
            not peer.value.local_nav_goal_active
        )
        local_snapshot_fresh = self._fresh_snapshot(self._robot_id, now) is not None
        candidate_reason = None
        if self._candidate_evidence_seen == {'robot1', 'robot2'}:
            candidate_reason = classify_empty_frontiers(
                self._candidate_evidence['robot1'],
                self._candidate_evidence['robot2'],
            )
        peer_reason = '' if not peer_fresh else str(peer.value.reason)
        if peer_reason.startswith('COMPLETION_CANDIDATE:'):
            peer_reason = peer_reason.split(':', 1)[1]
        peer_candidate = bool(
            peer_fresh and candidate_reason is not None and
            peer_reason == candidate_reason.value
        )
        local_planner_failures = sum(
            item.planner_failures for item in self._candidate_evidence.values()
        )
        peer_planner_failures = int(
            getattr(peer.value, 'planner_failure_count', 0)
        ) if peer_fresh else 0
        planner_failure_candidate = bool(
            self._candidate_evidence_seen == {'robot1', 'robot2'} and
            local_planner_failures > 0 and peer_planner_failures > 0 and
            all(item.unclassified == 0 for item in self._candidate_evidence.values())
        )
        if planner_failure_candidate:
            if self._planner_failure_candidate_since_steady_s is None:
                self._planner_failure_candidate_since_steady_s = now
            if (peer_fresh and peer_reason == 'PLANNER_FAILURE_CANDIDATE' and
                    now - self._planner_failure_candidate_since_steady_s >=
                    self._planner_failure_confirmation_s):
                self._set_terminal(
                    TerminalReason.PLANNER_INFRASTRUCTURE.value, success=False,
                )
                return True
            self._transition(
                CoordinatorState.BLOCKED,
                'PLANNER_FAILURE_CANDIDATE',
            )
            return False
        self._planner_failure_candidate_since_steady_s = None
        healthy = (
            nav2_healthy and tf_healthy and peer_healthy and local_snapshot_fresh and
            not self._nav2.local_goal_active and not self._dispatch_in_progress
        )
        if not healthy or candidate_reason is None:
            self._completion_candidate_since_steady_s = None
            self._transition(
                CoordinatorState.BLOCKED,
                'empty task union but completion evidence is incomplete',
            )
            return False
        if self._completion_candidate_reason != candidate_reason.value:
            self._completion_candidate_reason = candidate_reason.value
            self._completion_candidate_since_steady_s = now
        if self._completion_candidate_since_steady_s is None:
            self._completion_candidate_since_steady_s = now
        stable_duration = now - self._maps_stable_since_steady_s
        candidate_duration = now - self._completion_candidate_since_steady_s
        if (stable_duration >= self._map_stability_grace_s and peer_candidate and
                candidate_duration >= self._completion_confirmation_s):
            self._set_terminal(candidate_reason.value, success=True)
            return True
        self._transition(
            CoordinatorState.WAITING_FOR_INPUTS,
            'COMPLETION_CANDIDATE:' + candidate_reason.value,
        )
        return False

    def _set_terminal(self, reason: str, success: bool) -> None:
        """Freeze local dispatch after a replicated terminal semantic result."""
        self._terminal_finalization_attempt_count += 1
        self.get_logger().info(
            'TERMINAL_FINALIZATION_ATTEMPT robot=%s reason=%s success=%s '
            'attempt=%d' % (
                self._robot_id, reason, bool(success),
                self._terminal_finalization_attempt_count,
            ),
        )
        if self._terminal:
            # Multiple callbacks may observe the same peer terminal state.  A
            # committed terminal result is immutable; duplicate observations
            # must be harmless and must not publish a second result.
            if (self._terminal_success != bool(success) or
                    self._terminal_reason != str(reason)):
                self.get_logger().warning(
                    'TERMINAL_FINALIZATION_CONFLICT robot=%s existing=%s '
                    'requested=%s' % (
                        self._robot_id, self._terminal_reason, str(reason),
                    ),
                )
            else:
                self.get_logger().info(
                    'TERMINAL_FINALIZATION_DUPLICATE robot=%s reason=%s' % (
                        self._robot_id, self._terminal_reason,
                    ),
                )
            return
        self._terminal = True
        self._terminal_success = bool(success)
        self._terminal_reason = str(reason)
        # _snapshots stores Received[TaskSnapshot].  The receipt wrapper owns
        # freshness metadata; epoch belongs to its immutable TaskSnapshot
        # payload and must be read through .value.
        self._terminal_epoch = max(
            (received.value.epoch for received in self._snapshots.values()),
            default=0,
        )
        self._terminal_commit_count += 1
        self.get_logger().info(
            'TERMINAL_FINALIZATION_COMMITTED robot=%s reason=%s success=%s '
            'epoch=%d commit_count=%d' % (
                self._robot_id, self._terminal_reason, self._terminal_success,
                self._terminal_epoch, self._terminal_commit_count,
            ),
        )
        state = CoordinatorState.COMPLETE if success else CoordinatorState.BLOCKED
        self._transition(state, self._terminal_reason)
        self._emit_event(
            'MISSION_COMPLETE' if success else 'MISSION_ABORTED',
            self._terminal_reason,
        )

    def _continue_degraded_solo(self, snapshot: TaskSnapshot) -> None:
        """Dispatch at most one locally proposed task per epoch without team claims."""
        if not self._dispatch_enabled or self._dispatch_in_progress:
            return
        # Proposal epochs may advance for heartbeats or unchanged reachability
        # metadata.  Reusing the existing semantic fingerprint avoids
        # repeating an identical local path/action decision while preserving
        # all behavior when task content changes.
        key = (
            snapshot.source_session_id,
            self._snapshot_content_fingerprint(snapshot, snapshot),
        )
        if self._last_solo_snapshot_key == key:
            return
        live_signatures = {
            item.physical_signature for item in snapshot.tasks
            if item.physical_signature
        }
        self._completed_solo_physical_signatures.intersection_update(
            live_signatures
        )
        candidates = eligible_solo_tasks(
            snapshot.tasks,
            self._hard_failure_signatures,
            self._completed_solo_physical_signatures,
            self._minimum_solo_visible_gain_m,
            self._minimum_solo_ordering_score,
            self._maximum_solo_path_m,
        )
        self._last_solo_snapshot_key = key
        if not candidates:
            return
        union = build_canonical_union(
            candidates, (), min(self._maximum_union_tasks, len(candidates)),
        )
        task = union.tasks[0]
        self._dispatch_in_progress = True
        self._active_task = task
        self._active_round_id = 'degraded:%s:%s:%d' % (
            self._robot_id, snapshot.source_session_id, snapshot.epoch,
        )
        self._active_decision_hash = 'DEGRADED_SOLO'
        round_id = self._active_round_id

        def final_path(result: PathEvaluation):
            if self._active_round_id != round_id:
                return
            if not result.valid:
                self._invalidate_round(
                    result.failure_class,
                    'degraded solo ComputePathToPose failed: %s' % result.error_message,
                    result,
                )
                return
            self._nav2.check_dispatch_preconditions(
                task.members[0], True,
                lambda checks: self._dispatch_after_checks(task, result, checks),
            )

        if not self._nav2.evaluate_path(
                task.members[0], final_path, caller='DEGRADED_SOLO_DISPATCH'):
            self._invalidate_round(
                FailureClass.TF_OR_LIFECYCLE,
                'degraded solo local path action unavailable',
            )

    def _continue_bidding(self, round_work: RoundWork, generation: int) -> None:
        if not self._round_is_current(round_work, generation):
            self._discard_stale_tick(round_work, generation, 'bid continuation entry')
            return
        if round_work.query_index >= len(round_work.query_tasks):
            self._finish_bids(round_work, generation)
            return
        task = round_work.query_tasks[round_work.query_index]
        if self._synthetic_bids:
            distance = math.dist(self._synthetic_origin, task.approach)
            samples = tuple((
                self._synthetic_origin[0] + step / 6.0 * (
                    task.approach[0] - self._synthetic_origin[0]
                ),
                self._synthetic_origin[1] + step / 6.0 * (
                    task.approach[1] - self._synthetic_origin[1]
                ),
            ) for step in range(7))
            self._append_bid(round_work, generation, task, PathEvaluation(
                True, distance, samples, self.get_clock().now().nanoseconds,
                0, '', FailureClass.UNKNOWN,
            ))
            return
        round_id = round_work.round_id

        local_member = next(
            (member for member in task.members
             if member.source_robot_id == self._robot_id),
            None,
        )
        if (local_member is not None and local_member.local_path_valid and
                math.isfinite(local_member.local_path_length_m) and
                local_member.local_path_length_m >= 0.0):
            self._append_bid(round_work, generation, task, PathEvaluation(
                True, local_member.local_path_length_m,
                tuple(local_member.local_path),
                self.get_clock().now().nanoseconds, 0, 'reused local candidate path',
                FailureClass.UNKNOWN,
            ))
            return

        def completed(result: PathEvaluation):
            if not self._round_is_current(round_work, generation):
                self._discard_stale_tick(
                    round_work, generation, 'bid completion callback',
                )
                return
            self._append_bid(round_work, generation, task, result)

        if not self._nav2.evaluate_path(
                task.members[0], completed, caller='ALLOCATOR_BID'):
            # The per-robot planner lease may be held briefly by the
            # candidate generator.  Keep the round in bidding and retry on
            # the next coordinator tick instead of manufacturing a blocked
            # round or discarding the current task set.
            self._transition(
                CoordinatorState.BIDDING,
                'waiting for local ComputePathToPose query lease',
            )

    def _append_bid(
            self, round_work: RoundWork, generation: int,
            task: CanonicalTask, result: PathEvaluation) -> None:
        if not self._round_is_current(round_work, generation):
            self._discard_stale_tick(round_work, generation, 'append bid')
            return
        source_stamp = max(member.generation_ros_ns for member in task.members)
        bid = Bid(
            canonical_task_id=task.canonical_id,
            path_valid=result.valid,
            path_length_m=result.length_m,
            estimated_travel_cost=result.length_m,
            own_utility_contribution=task.visible_reveal_gain - result.length_m,
            task_generation_ros_ns=source_stamp,
            path_query_ros_ns=result.query_ros_ns,
            path=result.samples,
        )
        round_work.bids = round_work.bids + (bid,)
        if (result.valid and result.caller != 'UNKNOWN' and
                result.map_stamp_ns and result.costmap_stamp_ns):
            round_work.local_path_evaluations[task.canonical_id] = result
        round_work.query_index += 1
        self._continue_bidding(round_work, generation)

    def _finish_bids(self, round_work: RoundWork, generation: int) -> None:
        if (not self._round_is_current(round_work, generation) or
                round_work.local_batch is not None):
            if not self._round_is_current(round_work, generation):
                self._discard_stale_tick(round_work, generation, 'finish bids')
            return
        local_snapshot = round_work.snapshots[0 if self._robot_id == 'robot1' else 1]
        batch = BidBatch(
            round_id=round_work.round_id,
            union_hash=round_work.union.union_hash,
            source_robot_id=self._robot_id,
            source_session_id=local_snapshot.source_session_id,
            source_snapshot_epoch=local_snapshot.epoch,
            validity_s=self._bid_validity_s,
            bids=round_work.bids,
        )
        round_work.local_batch = batch
        self._publish_local_bid_batch(round_work, generation, log_batch=True)

    def _publish_local_bid_batch(
            self, round_work: Optional[RoundWork] = None,
            generation: Optional[int] = None, log_batch: bool = False) -> None:
        """Refresh the local bid heartbeat without changing round semantics."""
        round_work = self._round if round_work is None else round_work
        if round_work is None or round_work.local_batch is None:
            return
        if generation is not None and not self._round_is_current(round_work, generation):
            self._discard_stale_tick(round_work, generation, 'publish bid batch')
            return
        batch = round_work.local_batch
        message = bid_batch_to_msg(batch, self.get_clock().now().to_msg())
        self._bid_publisher.publish(message)
        self._bid_batches[self._robot_id] = receive(
            batch, batch.validity_s, time.monotonic(),
        )
        if log_batch:
            self.get_logger().info(
                'BID_ARRAY robot=%s round=%s union=%s bids=%d' % (
                    self._robot_id, batch.round_id, batch.union_hash, len(batch.bids),
                )
            )

    def _both_bid_batches_valid(
            self, now: float, round_work: Optional[RoundWork] = None) -> bool:
        round_work = self._round if round_work is None else round_work
        if round_work is None:
            return False
        for index, robot_id in enumerate(('robot1', 'robot2')):
            snapshot = round_work.snapshots[index]
            received = self._bid_batches.get(robot_id)
            if received is None or not bid_batch_valid(
                    received, now, robot_id, snapshot.source_session_id,
                    snapshot.epoch, round_work.round_id,
                    round_work.union.union_hash, True):
                return False
        return True

    def _publish_decision(
            self, round_work: Optional[RoundWork] = None,
            generation: Optional[int] = None,
            log_decision: bool = True) -> None:
        round_work = self._round if round_work is None else round_work
        if round_work is None or round_work.decision is None:
            return
        if generation is not None and not self._round_is_current(round_work, generation):
            self._discard_stale_tick(round_work, generation, 'publish decision')
            return
        local_snapshot = round_work.snapshots[0 if self._robot_id == 'robot1' else 1]
        message = decision_to_msg(
            round_work.decision, self._robot_id, local_snapshot.source_session_id,
            round_work.snapshots[0].epoch, round_work.snapshots[1].epoch,
            self._state.value, self._decision_validity_s,
            self.get_clock().now().to_msg(),
        )
        if generation is not None and not self._round_is_current(round_work, generation):
            self._discard_stale_tick(round_work, generation, 'before decision publish')
            return
        self._decision_publisher.publish(message)
        if generation is not None and not self._round_is_current(round_work, generation):
            self._discard_stale_tick(round_work, generation, 'after decision publish')
            return
        round_work.decision_published = True
        if not log_decision:
            return
        score = round_work.decision.score
        diagnostics = round_work.decision.diagnostics
        if diagnostics.strategy == 'burgard':
            self.get_logger().info(
                'BURGARD_DECISION robot=%s round=%s union=%s hash=%s '
                'r1=%s r2=%s beta=%.6f path_limit=%.6f sensor_range=%.6f '
                'selection_total=%.6f trace=%s traffic=%s' % (
                    self._robot_id, round_work.round_id,
                    round_work.union.union_hash,
                    round_work.decision.decision_hash,
                    round_work.decision.robot1_task_id or 'IDLE',
                    round_work.decision.robot2_task_id or 'IDLE',
                    diagnostics.beta, diagnostics.feasible_path_limit_m,
                    diagnostics.sensor_max_range_m, score.total,
                    json.dumps(diagnostics.burgard_trace, sort_keys=True,
                               separators=(',', ':')),
                    json.dumps(diagnostics.traffic, sort_keys=True,
                               separators=(',', ':')),
                )
            )
            return
        self.get_logger().info(
            'PAIR_DECISION robot=%s round=%s union=%s hash=%s r1=%s r2=%s '
            'total=%.6f gain=%.6f path=%.6f nearby=%.6f route=%.6f hard=%.6f '
            'sensing=%.6f imbalance=%.6f' % (
                self._robot_id, round_work.round_id, round_work.union.union_hash,
                round_work.decision.decision_hash,
                round_work.decision.robot1_task_id or 'IDLE',
                round_work.decision.robot2_task_id or 'IDLE', score.total,
                score.team_visible_gain, score.combined_path_cost,
                score.nearby_goal_penalty, score.route_overlap_penalty,
                score.hard_failure_penalty, score.sensing_overlap_penalty,
                score.workload_imbalance_penalty,
            )
        )

    def _matching_peer_decision(
            self, round_work: RoundWork, generation: int, now: float) -> bool:
        if (not self._round_is_current(round_work, generation) or
                round_work.decision is None or
                self._peer_decision is None or not self._peer_decision.fresh(now)):
            return False
        peer = self._peer_decision.value
        local = round_work.decision
        expected_session = round_work.snapshots[
            1 if self._peer_id == 'robot2' else 0
        ].source_session_id
        return (
            uuid_to_text(peer.source_session_id) == expected_session and
            peer.round_id == local.round_id and
            peer.union_hash == local.union_hash and
            peer.robot1_snapshot_epoch == round_work.snapshots[0].epoch and
            peer.robot2_snapshot_epoch == round_work.snapshots[1].epoch and
            peer.robot1_bid_fingerprint == local.robot1_bid_fingerprint and
            peer.robot2_bid_fingerprint == local.robot2_bid_fingerprint and
            peer.robot1_canonical_task_id == local.robot1_task_id and
            peer.robot2_canonical_task_id == local.robot2_task_id and
            peer.decision_hash == local.decision_hash
        )

    def _start_local_dispatch(self, round_work: RoundWork, generation: int) -> None:
        if not self._round_is_current(round_work, generation):
            self._discard_stale_tick(round_work, generation, 'start local dispatch')
            return
        if round_work.decision is None:
            return
        traffic = round_work.traffic
        if (self._traffic_scheduler_enabled and traffic is not None and
                traffic.conflict):
            if traffic.waiting_robot_id == self._robot_id:
                self._begin_traffic_wait(traffic)
                return
            if traffic.winner_robot_id == self._robot_id:
                self._emit_event('TRAFFIC_PRIORITY_GRANTED', self._traffic_reason(traffic))
            elif traffic.reason == 'BOTH_ALREADY_ACTIVE_MONITOR_ONLY':
                self._emit_event('TRAFFIC_MONITOR_ONLY', self._traffic_reason(traffic))
        task_id = (
            round_work.decision.robot1_task_id if self._robot_id == 'robot1'
            else round_work.decision.robot2_task_id
        )
        if not task_id:
            self._transition(CoordinatorState.WAITING_FOR_INPUTS, 'local assignment is IDLE')
            return
        tasks = {task.canonical_id: task for task in round_work.union.tasks}
        task = tasks.get(task_id)
        if task is None:
            self._invalidate_round(FailureClass.UNKNOWN, 'agreed task missing from union')
            return
        self._dispatch_in_progress = True
        self._active_task = task
        self._active_round_id = round_work.round_id
        self._active_decision_hash = round_work.decision.decision_hash
        round_id = self._active_round_id

        def final_path(result: PathEvaluation):
            if (self._active_round_id != round_id or
                    not self._round_is_current(round_work, generation)):
                self._discard_stale_tick(round_work, generation, 'final path callback')
                return
            if not result.valid:
                self._invalidate_round(
                    result.failure_class,
                    'final ComputePathToPose failed: %s' % result.error_message,
                    result,
                )
                return
            self._nav2.check_dispatch_preconditions(
                task.members[0], True,
                lambda checks: self._dispatch_after_checks(
                    task, result, checks, round_work, generation,
                ),
            )

        local_evaluation = round_work.local_path_evaluations.get(task_id)
        if (local_evaluation is not None and
                self._nav2.path_context_matches(local_evaluation)):
            # The local bid was computed for this exact immutable round.  The
            # dispatch gate still rechecks TF, map, costmap, lifecycle, and
            # active-goal state, so reusing the path does not make stale
            # planner output authoritative.
            self.get_logger().info(
                'COMPUTE_PATH_REUSED source=ALLOCATOR_BID task=%s round=%s '
                'path_length_m=%.3f' % (
                    task_id, round_work.round_id, local_evaluation.length_m,
                )
            )
            final_path(local_evaluation)
            return
        if not self._nav2.evaluate_path(
                task.members[0], final_path,
                caller='FINAL_DISPATCH_VALIDATION'):
            # A local path query can be serialized behind the candidate
            # generator.  Preserve the agreed round and retry dispatch; this
            # is not evidence that the task is unreachable.
            self._dispatch_in_progress = False
            self._active_task = None
            self._active_round_id = ''
            self._active_decision_hash = ''
            self._transition(
                CoordinatorState.WAITING_FOR_MATCHING_DECISION,
                'waiting for local ComputePathToPose query lease',
            )

    def _dispatch_after_checks(
            self, task: CanonicalTask, final_path: PathEvaluation,
            checks: DispatchPreconditions,
            round_work: Optional[RoundWork] = None,
            generation: Optional[int] = None) -> None:
        if (round_work is not None and generation is not None and
                not self._round_is_current(round_work, generation)):
            self._discard_stale_tick(round_work, generation, 'dispatch checks callback')
            self._dispatch_in_progress = False
            return
        self.get_logger().info(
            'DISPATCH_PRECONDITIONS robot=%s round=%s task=%s ready=%s '
            'action=%s lifecycle=%s tf=%s tf_age=%s map_inside=%s map_value=%s '
            'costmap_inside=%s costmap_value=%s local_goal_clear=%s path=%s reason=%s' % (
                self._robot_id, self._active_round_id, task.canonical_id, checks.ready,
                checks.action_server_ready, checks.lifecycle_active,
                checks.transform_available, checks.transform_age_s,
                checks.goal_inside_map, checks.goal_map_value,
                checks.goal_inside_costmap, checks.goal_costmap_value,
                checks.no_local_goal_active, checks.final_path_valid, checks.reason,
            )
        )
        if not checks.ready:
            failure = classify_dispatch_precondition_failure(checks)
            self._invalidate_round(failure, checks.reason, final_path)
            return
        if not self._nav2.send_navigation(
                task.members[0], self._navigation_finished,
                diagnostic_path=final_path.samples):
            self._invalidate_round(
                FailureClass.ACTION_REJECTION,
                'local NavigateToPose send precondition changed', final_path,
            )
            return
        self._dispatch_count += 1
        self._dispatch_in_progress = False
        self._transition(CoordinatorState.NAVIGATING, 'local agreed goal accepted for send')
        self._emit_event(
            'NAV_GOAL_SENT', 'local-only NavigateToPose dispatch',
            path_length=final_path.length_m,
        )

    def _navigation_finished(self, outcome: NavigationOutcome) -> None:
        result = 'SUCCEEDED' if (
            outcome.status == 4 and outcome.error_code == 0
        ) else 'FAILED'
        if result != 'SUCCEEDED':
            self.get_logger().warning(
                'NAV2_TERMINAL_RESULT robot=%s status=%s accepted=%s '
                'error_code=%s error_message=%r failure_class=%s recoveries=%s '
                'duration_s=%.3f travelled_m=%.3f follow_path_code=%s '
                'follow_path_name=%s failure_family=%s deepest_failure=%s' % (
                    self._robot_id, outcome.status, outcome.accepted,
                    outcome.error_code, outcome.error_message,
                    outcome.failure_class.value, outcome.recoveries,
                    outcome.duration_s, outcome.travelled_distance_m,
                    outcome.follow_path_error_code,
                    outcome.follow_path_error_name,
                    outcome.controller_failure_family,
                    outcome.deepest_failure_classification))
            physical_signature = (
                '' if self._active_task is None else
                self._active_task.members[0].physical_signature
            )
            task_diag = self._failure_task_diagnostics.setdefault(
                physical_signature or '__unknown__', {
                    'controller_failure_count': 0,
                    'tf_failure_count': 0,
                    'last_failure_type': '',
                    'last_failure_time_s': 0.0,
                    'last_failure_geometry_signature': '',
                    'consecutive_structural_failure_count': 0,
                },
            )
            if outcome.controller_failure_family.startswith('CONTROLLER'):
                task_diag['controller_failure_count'] = int(
                    task_diag['controller_failure_count']) + 1
                task_diag['consecutive_structural_failure_count'] = int(
                    task_diag['consecutive_structural_failure_count']) + 1
            elif outcome.controller_failure_family == 'NAV_INFRASTRUCTURE_TF':
                task_diag['tf_failure_count'] = int(task_diag['tf_failure_count']) + 1
                task_diag['consecutive_structural_failure_count'] = 0
            else:
                task_diag['consecutive_structural_failure_count'] = 0
            task_diag['last_failure_type'] = outcome.deepest_failure_classification
            task_diag['last_failure_time_s'] = self.get_clock().now().nanoseconds / 1e9
            if outcome.diagnostic_snapshot_json:
                try:
                    task_diag['last_failure_geometry_signature'] = json.loads(
                        outcome.diagnostic_snapshot_json,
                    ).get('execution_geometry_signature', '')
                except (TypeError, ValueError):
                    pass
            self.get_logger().warning(
                'NAVIGATION_TASK_FAILURE_COUNTS physical_signature=%s data=%s' %
                (physical_signature, json.dumps(task_diag, sort_keys=True)),
            )
            if outcome.diagnostic_snapshot_json:
                self.get_logger().warning(
                    'NAVIGATION_FAILURE_SNAPSHOT %s' % outcome.diagnostic_snapshot_json,
                )
            self.get_logger().warning(
                'NAVIGATION_FAILURE_PROPAGATION robot=%s physical_signature=%s '
                'navigate_to_pose_error_code=%s navigate_to_pose_error_text=%r '
                'follow_path_error_code=%s follow_path_error_name=%s '
                'failure_family=%s deepest_failure=%s '
                'deepest_failure_timestamp_ros_ns=%s' % (
                    self._robot_id, physical_signature, outcome.error_code,
                    outcome.error_message, outcome.follow_path_error_code,
                    outcome.follow_path_error_name, outcome.controller_failure_family,
                    outcome.deepest_failure_classification,
                    outcome.deepest_failure_timestamp_ros_ns,
                )
            )
        self._emit_event(
            'NAVIGATION_' + result, outcome.error_message or result,
            travelled=outcome.travelled_distance_m,
            duration=outcome.duration_s, recoveries=outcome.recoveries,
            failure=outcome.failure_class,
            nav2_error_code=outcome.error_code,
            nav2_error_message=outcome.error_message,
            nav2_error_name=outcome.nav2_error_name,
        )
        if result != 'SUCCEEDED':
            self._publish_failure(
                outcome.failure_class, outcome.error_message,
                nav2_error_code=outcome.error_code,
                nav2_error_message=outcome.error_message,
                nav2_error_name=outcome.nav2_error_name,
            )
        elif self._active_task is not None:
            # Suppress only the exact physical region that just succeeded.
            # A later disappearance from the snapshot permits it to be
            # reconsidered; unchanged residual fragments cannot churn goals.
            self._completed_solo_physical_signatures.update(
                member.physical_signature
                for member in self._active_task.members
                if member.physical_signature
            )
        self._active_task = None
        self._active_round_id = ''
        self._active_decision_hash = ''
        self._dispatch_in_progress = False
        self._settle_until_steady_s = time.monotonic() + self._post_goal_settle_s
        # Robot availability is part of the semantic trigger.  A completed
        # goal must permit a fresh auction even when the task-set fingerprint
        # is unchanged.
        self._last_semantic_fingerprint = ''
        self._last_solo_snapshot_key = None
        self._reset_round('navigation terminal result')

    def _invalidate_round(
            self, failure: FailureClass, reason: str,
            path: Optional[PathEvaluation] = None) -> None:
        self._publish_failure(failure, reason, path)
        self._emit_event('ROUND_INVALIDATED', reason, failure=failure)
        self._dispatch_in_progress = False
        self._active_task = None
        self._active_round_id = ''
        self._active_decision_hash = ''
        # A failure changes the effective feasible-pair set even when the
        # published task geometry is unchanged.  Permit exactly one fresh
        # semantic round so failure suppression can take effect.
        self._last_semantic_fingerprint = ''
        self._last_solo_snapshot_key = None
        self._settle_until_steady_s = time.monotonic() + self._post_goal_settle_s
        self._reset_round('round invalidated')

    def _publish_failure(
            self, failure: FailureClass, reason: str,
            path: Optional[PathEvaluation] = None,
            nav2_error_code: int = 0, nav2_error_message: str = '',
            nav2_error_name: str = '') -> None:
        if self._active_task is None:
            return
        member = self._active_task.members[0]
        if failure in HARD_FAILURES:
            # Record local suppression before consulting snapshot provenance;
            # a stale snapshot must not make the failing robot immediately
            # reselect the same physical task.
            self._record_hard_failure(member.physical_signature, 15.0)
        local_snapshot = self._fresh_snapshot(self._robot_id, time.monotonic())
        if local_snapshot is None:
            return
        message = ExplorationFailure()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'shared_map'
        message.source_robot_id = self._robot_id
        message.source_session_id = text_to_uuid(local_snapshot.source_session_id)
        message.round_id = self._active_round_id
        message.canonical_task_id = self._active_task.canonical_id
        message.physical_task_signature = self._active_task.members[0].physical_signature
        message.approach_pose.header = message.header
        message.approach_pose.pose.position.x, message.approach_pose.pose.position.y = (
            member.approach
        )
        message.approach_pose.pose.orientation.z = math.sin(member.approach_yaw / 2.0)
        message.approach_pose.pose.orientation.w = math.cos(member.approach_yaw / 2.0)
        message.failure_class = FAILURE_TO_MESSAGE[failure]
        if path is not None:
            message.path_length_m = path.length_m
            for x, y in path.samples:
                from geometry_msgs.msg import Point

                message.path_samples.append(Point(x=x, y=y))
        message.retry_count = 1
        message.nav2_error_code = int(max(0, nav2_error_code))
        message.nav2_error_message = nav2_error_message
        message.nav2_error_name = nav2_error_name or {
            102: 'TF_ERROR',
            104: 'PATIENCE_EXCEEDED',
            105: 'FAILED_TO_MAKE_PROGRESS',
            106: 'NO_VALID_CONTROL',
            107: 'CONTROLLER_TIMED_OUT',
        }.get(int(nav2_error_code), '')
        message.validity = seconds_to_duration(15.0 if failure in HARD_FAILURES else 2.0)
        message.evidence = reason
        self._failure_publisher.publish(message)

    def _hard_failed_task_ids(self, union: CanonicalUnion) -> frozenset[str]:
        signatures = set(self._hard_failure_signatures)
        return frozenset(
            task.canonical_id for task in union.tasks
            if any(member.physical_signature in signatures for member in task.members)
        )

    def _expire_failures(self, now: float) -> None:
        self._hard_failure_signatures = {
            signature: expiry for signature, expiry in self._hard_failure_signatures.items()
            if expiry > now
        }

    def _reset_round(self, reason: str) -> None:
        previous = self._round
        new_generation = self._round_lifecycle.invalidate()
        if previous is not None:
            completed = reason == 'navigation terminal result'
            if completed:
                self._round_completed_count += 1
                self._last_round_completion_steady_s = time.monotonic()
                self._log_round_lifecycle(
                    'ALLOCATOR_ROUND_COMPLETED', previous, new_generation, reason,
                )
            else:
                self._log_round_lifecycle(
                    'ALLOCATOR_ROUND_CLEARED', previous, new_generation, reason,
                )
        self._round = None
        self._bid_batches.clear()
        self._peer_decision = None
        self._committed = CommittedRound()
        self._transition(CoordinatorState.WAITING_FOR_INPUTS, reason)

    def _transition(self, state: CoordinatorState, reason: str) -> None:
        if state == self._state and reason == self._state_reason:
            return
        previous = self._state
        self._state, self._state_reason = state, reason
        now = time.monotonic()
        ages = {}
        for robot_id, received_snapshot in self._snapshots.items():
            ages[robot_id] = max(0.0, now - received_snapshot.receipt_steady_s)
        peer_bid = self._bid_batches.get(self._peer_id)
        self.get_logger().info(
            'STATE_TRANSITION robot=%s session=%s round=%s previous=%s next=%s '
            'reason=%s task=%s peer_state=%s local_snapshot_age=%s peer_snapshot_age=%s '
            'local_bid_age=%s peer_bid_age=%s decision=%s nav_active=%s' % (
                self._robot_id, self._local_session_text(), self._current_round_id(),
                previous.value, state.value, reason,
                '' if self._active_task is None else self._active_task.canonical_id,
                self._peer_state_text(), ages.get(self._robot_id), ages.get(self._peer_id),
                self._receipt_age(self._bid_batches.get(self._robot_id), now),
                self._receipt_age(peer_bid, now), self._active_decision_hash,
                self._nav2.local_goal_active,
            )
        )
        self._emit_event(
            'STATE_TRANSITION', reason,
            previous=previous.value, next_state=state.value,
        )

    @staticmethod
    def _receipt_age(received_value, now: float):
        return None if received_value is None else now - received_value.receipt_steady_s

    def _local_session_text(self) -> str:
        item = self._snapshots.get(self._robot_id)
        return '' if item is None else item.value.source_session_id

    def _current_round_id(self) -> str:
        round_work = self._round
        return '' if round_work is None else round_work.round_id

    def _peer_state_text(self) -> str:
        if self._peer_status is None:
            return 'UNKNOWN'
        return str(self._peer_status.value.state)

    def _publish_status(self) -> None:
        self._nav2.refresh_health()
        round_work = self._round
        generation = self._round_lifecycle.generation
        self._publish_local_bid_batch(round_work, generation)
        if round_work is not None and round_work.decision is not None:
            self._publish_decision(round_work, generation, log_decision=False)
        self._publish_health_diagnostic()
        message = DistributedExplorationStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'shared_map'
        message.source_robot_id = self._robot_id
        session = self._local_session_text()
        if session:
            message.source_session_id = text_to_uuid(session)
        message.state = STATE_TO_MESSAGE[self._state]
        message.round_id = '' if round_work is None else round_work.round_id
        message.union_hash = '' if round_work is None else round_work.union.union_hash
        decision = None if round_work is None else round_work.decision
        message.decision_hash = '' if decision is None else decision.decision_hash
        message.active_canonical_task_id = (
            '' if self._active_task is None else self._active_task.canonical_id
        )
        message.local_nav_goal_active = self._nav2.local_goal_active
        message.nav2_healthy, message.tf_healthy = self._nav2.health_flags()
        message.candidate_source_healthy = self._fresh_snapshot(
            self._robot_id, time.monotonic(),
        ) is not None
        message.peer_communication_healthy = self._peer_liveness.state != (
            CoordinatorState.DEGRADED_SOLO
        )
        message.terminal = self._terminal
        message.terminal_reason = self._terminal_reason
        message.terminal_epoch = self._terminal_epoch
        message.terminal_map_revision = max(
            (received.value.map_revision
             for received in self._snapshots.values()),
            default=0,
        )
        evidence = tuple(self._candidate_evidence.values())
        message.remaining_frontier_count = sum(item.detected for item in evidence)
        message.remaining_small_frontier_count = sum(item.small for item in evidence)
        message.remaining_out_of_range_count = sum(item.out_of_range for item in evidence)
        message.remaining_unreachable_count = sum(item.unreachable for item in evidence)
        message.planner_failure_count = sum(item.planner_failures for item in evidence)
        message.detected_not_queried_count = sum(
            item.detected_not_queried for item in evidence)
        message.below_minimum_gain_count = sum(
            item.below_minimum_gain for item in evidence)
        message.actionable_reachable_count = sum(
            (item.actionable_reachable if item.actionable_reachable is not None
             else item.reachable) for item in evidence)
        message.validity = seconds_to_duration(2.5)
        message.reason = self._state_reason
        self._status_publisher.publish(message)

    def _emit_event(
            self, event_type: str, reason: str, previous: str = '',
            next_state: str = '', path_length: float = 0.0,
            travelled: float = 0.0, duration: float = 0.0,
            recoveries: int = 0, failure: FailureClass = FailureClass.UNKNOWN,
            nav2_error_code: int = 0, nav2_error_message: str = '',
            nav2_error_name: str = '') -> None:
        message = DistributedExplorationEvent()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'shared_map'
        message.source_robot_id = self._robot_id
        session = self._local_session_text()
        if session:
            message.source_session_id = text_to_uuid(session)
        message.event_type = event_type
        round_work = self._round
        message.round_id = self._active_round_id or (
            '' if round_work is None else round_work.round_id
        )
        message.union_hash = '' if round_work is None else round_work.union.union_hash
        # Agreement and state-transition events can be emitted before a local
        # task is promoted to ``_active_task``.  Preserve the round's decision
        # fingerprint in that interval instead of emitting an empty hash.
        message.decision_hash = self._active_decision_hash
        if (not message.decision_hash and round_work is not None and
                round_work.decision is not None):
            message.decision_hash = round_work.decision.decision_hash
        if self._active_task is not None:
            message.canonical_task_id = self._active_task.canonical_id
            message.physical_task_signature = self._active_task.members[0].physical_signature
        message.previous_state = previous
        message.next_state = next_state
        message.reason = reason
        message.path_length_m = path_length
        message.travelled_distance_m = travelled
        message.navigation_duration_s = duration
        message.result = event_type
        message.failure_class = FAILURE_TO_MESSAGE[failure]
        message.recoveries = recoveries
        message.nav2_error_code = int(max(0, nav2_error_code))
        message.nav2_error_message = nav2_error_message
        message.nav2_error_name = nav2_error_name or {
            102: 'TF_ERROR',
            104: 'PATIENCE_EXCEEDED',
            105: 'FAILED_TO_MAKE_PROGRESS',
            106: 'NO_VALID_CONTROL',
            107: 'CONTROLLER_TIMED_OUT',
        }.get(int(nav2_error_code), '')
        if round_work is not None and round_work.decision is not None:
            message.route_overlap_score = round_work.decision.score.route_overlap_penalty
            message.sensing_overlap_estimate = (
                round_work.decision.score.sensing_overlap_penalty
            )
        self._event_publisher.publish(message)

    def _log_union(self, round_work: RoundWork) -> None:
        self.get_logger().info(
            'CANONICAL_UNION robot=%s round=%s union=%s tasks=%s' % (
                self._robot_id, round_work.round_id, round_work.union.union_hash,
                ','.join(task.canonical_id for task in round_work.union.tasks),
            )
        )


def main(args=None):
    """Run one namespaced replicated assignment peer."""
    rclpy.init(args=args)
    node = DistributedFrontierAssignment()
    executor_override = os.environ.get('MY_EPUCK_ASSIGNMENT_EXECUTOR_THREADS')
    executor_threads = max(1, int(executor_override or node._executor_threads))
    if executor_threads == 1:
        executor = SingleThreadedExecutor()
    else:
        executor = MultiThreadedExecutor(num_threads=executor_threads)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
