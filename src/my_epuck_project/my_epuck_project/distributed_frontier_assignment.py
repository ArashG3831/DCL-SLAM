"""Replicated peer-to-peer two-robot frontier assignment ROS node."""

import math
import time
from dataclasses import dataclass
from typing import Optional

from my_epuck_interfaces.msg import (
    DistributedExplorationEvent,
    DistributedExplorationStatus,
    ExplorationFailure,
    PairDecision as PairDecisionMsg,
    TaskBidArray as TaskBidArrayMsg,
    TaskSnapshot as TaskSnapshotMsg,
)

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .distributed_assignment.canonical import (
    TaskIdentity,
    build_canonical_union,
    canonical_round_id,
)
from .distributed_assignment.failures import HARD_FAILURES
from .distributed_assignment.local_nav2 import (
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
    TaskSnapshot,
)
from .distributed_assignment.protocol import (
    CommittedRound,
    PeerLiveness,
    Received,
    SnapshotLedger,
    bid_batch_valid,
    receive,
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


@dataclass
class RoundWork:
    """Mutable bounded work for one uncommitted canonical round."""

    round_id: str
    union: CanonicalUnion
    snapshots: tuple[TaskSnapshot, TaskSnapshot]
    query_tasks: tuple[CanonicalTask, ...]
    query_index: int = 0
    bids: tuple[Bid, ...] = ()
    local_batch: Optional[BidBatch] = None
    decision: Optional[PairDecision] = None
    decision_published: bool = False


STATE_TO_MESSAGE = {
    CoordinatorState.WAITING_FOR_INPUTS: DistributedExplorationStatus.WAITING_FOR_INPUTS,
    CoordinatorState.BIDDING: DistributedExplorationStatus.BIDDING,
    CoordinatorState.WAITING_FOR_MATCHING_DECISION:
        DistributedExplorationStatus.WAITING_FOR_MATCHING_DECISION,
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


class DistributedFrontierAssignment(Node):
    """Compute a complete pair decision independently and dispatch only locally."""

    def __init__(self):
        """Configure one equal peer with identity-bound topic and Nav2 interfaces."""
        super().__init__('distributed_frontier_assignment')
        self._robot_id = self.declare_parameter('robot_id', '').value
        if self._robot_id not in ('robot1', 'robot2'):
            raise ValueError('robot_id must be exactly robot1 or robot2')
        self._peer_id = 'robot2' if self._robot_id == 'robot1' else 'robot1'
        self._dispatch_enabled = bool(
            self.declare_parameter('dispatch_enabled', False).value,
        )
        self._synthetic_bids = bool(
            self.declare_parameter('synthetic_bids', False).value,
        )
        self._synthetic_origin = (
            float(self.declare_parameter('synthetic_origin_x', 0.0).value),
            float(self.declare_parameter('synthetic_origin_y', 0.0).value),
        )
        self._maximum_union_tasks = int(
            self.declare_parameter('maximum_union_tasks', 8).value,
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
        self._post_goal_settle_s = float(
            self.declare_parameter('post_goal_settle_s', 1.0).value,
        )
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
        )
        self._ledger = SnapshotLedger(self._snapshot_maximum_tasks)
        self._snapshots: dict[str, Received[TaskSnapshot]] = {}
        self._bid_batches: dict[str, Received[BidBatch]] = {}
        self._peer_decision: Optional[Received[PairDecisionMsg]] = None
        self._peer_status: Optional[Received[DistributedExplorationStatus]] = None
        self._hard_failure_signatures: dict[str, float] = {}
        self._peer_liveness = PeerLiveness(peer_timeout_s)
        self._round: Optional[RoundWork] = None
        self._committed = CommittedRound()
        self._state = CoordinatorState.WAITING_FOR_INPUTS
        self._state_reason = 'startup'
        self._dispatch_in_progress = False
        self._settle_until_steady_s = 0.0
        self._active_task: Optional[CanonicalTask] = None
        self._active_round_id = ''
        self._active_decision_hash = ''
        self._nav2 = LocalNav2(self)
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        for robot_id in ('robot1', 'robot2'):
            self.create_subscription(
                TaskSnapshotMsg, f'/{robot_id}/task_snapshot',
                self._snapshot_callback, qos,
            )
            self.create_subscription(
                TaskBidArrayMsg, f'/{robot_id}/task_bids',
                self._bid_callback, qos,
            )
            self.create_subscription(
                PairDecisionMsg, f'/{robot_id}/pair_decision',
                self._decision_callback, qos,
            )
            self.create_subscription(
                DistributedExplorationStatus, f'/{robot_id}/distributed_status',
                self._status_callback, qos,
            )
            self.create_subscription(
                ExplorationFailure, f'/{robot_id}/exploration_failure',
                self._failure_callback, qos,
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
        self._tick_timer = self.create_timer(0.1, self._tick)
        self._status_timer = self.create_timer(1.0, self._publish_status)
        interfaces = self._nav2.interface_names()
        self.get_logger().info(
            'DISTRIBUTED_ASSIGNMENT robot=%s peer=%s dispatch=%s '
            'compute=%s navigate=%s no_peer_clients=true no_cmd_vel=true' % (
                self._robot_id, self._peer_id, self._dispatch_enabled,
                interfaces['compute_path'], interfaces['navigate'],
            )
        )

    def _snapshot_callback(self, message: TaskSnapshotMsg) -> None:
        """Accept only bounded monotonic source-local snapshot provenance."""
        try:
            snapshot = snapshot_from_msg(message)
        except (ValueError, TypeError) as error:
            self.get_logger().error('TASK_SNAPSHOT_REJECTED decode=%s' % error)
            return
        if not self._ledger.accept(snapshot):
            return
        now = time.monotonic()
        self._snapshots[snapshot.source_robot_id] = receive(
            snapshot, snapshot.validity_s, now,
        )
        if snapshot.source_robot_id == self._peer_id:
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
        self._peer_status = receive(
            message, duration_to_seconds(message.validity), time.monotonic(),
        )

    def _failure_callback(self, message: ExplorationFailure) -> None:
        if message.source_robot_id == self._robot_id:
            return
        hard_values = {FAILURE_TO_MESSAGE[item] for item in HARD_FAILURES}
        if message.failure_class in hard_values:
            ttl = max(0.1, min(120.0, duration_to_seconds(message.validity)))
            self._hard_failure_signatures[message.physical_task_signature] = (
                time.monotonic() + ttl
            )

    def _fresh_snapshot(self, robot_id: str, now: float) -> Optional[TaskSnapshot]:
        received = self._snapshots.get(robot_id)
        return received.value if received is not None and received.fresh(now) else None

    def _tick(self) -> None:
        now = time.monotonic()
        self._expire_failures(now)
        if self._nav2.local_goal_active or self._dispatch_in_progress:
            return
        if now < self._settle_until_steady_s:
            return
        first = self._fresh_snapshot('robot1', now)
        second = self._fresh_snapshot('robot2', now)
        if first is None or second is None:
            if self._peer_liveness.evaluate(now) == CoordinatorState.DEGRADED_SOLO:
                self._transition(CoordinatorState.DEGRADED_SOLO, 'peer snapshot timeout')
            else:
                self._transition(
                    CoordinatorState.WAITING_FOR_INPUTS,
                    'fresh task snapshots from both source sessions required',
                )
            return
        round_id = canonical_round_id(
            TaskIdentity('robot1', first.source_session_id, first.epoch),
            TaskIdentity('robot2', second.source_session_id, second.epoch),
        )
        if self._round is None or self._round.round_id != round_id:
            union = build_canonical_union(
                first.tasks, second.tasks, self._maximum_union_tasks,
            )
            self._round = RoundWork(
                round_id=round_id,
                union=union,
                snapshots=(first, second),
                query_tasks=union.tasks[:self._maximum_path_queries],
            )
            self._bid_batches.clear()
            self._peer_decision = None
            self._committed = CommittedRound()
            self._transition(CoordinatorState.BIDDING, 'new canonical round')
            self._log_union(self._round)
        if self._round.local_batch is None:
            self._continue_bidding()
            return
        if not self._both_bid_batches_valid(now):
            self._transition(CoordinatorState.BIDDING, 'waiting for valid peer bids')
            return
        if self._round.decision is None:
            first_batch = self._bid_batches['robot1'].value
            second_batch = self._bid_batches['robot2'].value
            hard_ids = self._hard_failed_task_ids(self._round.union)
            self._round.decision = choose_pair_assignment(
                self._round.round_id, self._round.union,
                first_batch, second_batch, hard_ids, self._weights,
            )
            self._publish_decision()
            self._transition(
                CoordinatorState.WAITING_FOR_MATCHING_DECISION,
                'local complete pair decision published',
            )
        if not self._matching_peer_decision(now):
            return
        if self._committed.decision is None:
            self._committed.commit(self._round.decision)
            self._emit_event('DECISION_AGREED', 'positive replicated decision match')
        if not self._dispatch_enabled:
            self._transition(
                CoordinatorState.WAITING_FOR_MATCHING_DECISION,
                'decision agreed; dispatch disabled',
            )
            return
        self._start_local_dispatch()

    def _continue_bidding(self) -> None:
        if self._round is None:
            return
        if self._round.query_index >= len(self._round.query_tasks):
            self._finish_bids()
            return
        task = self._round.query_tasks[self._round.query_index]
        if self._synthetic_bids:
            distance = math.dist(self._synthetic_origin, task.approach)
            samples = (self._synthetic_origin, task.approach)
            self._append_bid(task, PathEvaluation(
                True, distance, samples, self.get_clock().now().nanoseconds,
                0, '', FailureClass.UNKNOWN,
            ))
            return
        round_id = self._round.round_id

        def completed(result: PathEvaluation):
            if self._round is None or self._round.round_id != round_id:
                return
            self._append_bid(task, result)

        if not self._nav2.evaluate_path(task.members[0], completed):
            self._transition(
                CoordinatorState.BLOCKED,
                'local ComputePathToPose server unavailable or request busy',
            )

    def _append_bid(self, task: CanonicalTask, result: PathEvaluation) -> None:
        if self._round is None:
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
        self._round.bids = self._round.bids + (bid,)
        self._round.query_index += 1
        self._continue_bidding()

    def _finish_bids(self) -> None:
        if self._round is None or self._round.local_batch is not None:
            return
        local_snapshot = self._round.snapshots[0 if self._robot_id == 'robot1' else 1]
        batch = BidBatch(
            round_id=self._round.round_id,
            union_hash=self._round.union.union_hash,
            source_robot_id=self._robot_id,
            source_session_id=local_snapshot.source_session_id,
            source_snapshot_epoch=local_snapshot.epoch,
            validity_s=self._bid_validity_s,
            bids=self._round.bids,
        )
        self._round.local_batch = batch
        message = bid_batch_to_msg(batch, self.get_clock().now().to_msg())
        self._bid_publisher.publish(message)
        self._bid_batches[self._robot_id] = receive(
            batch, batch.validity_s, time.monotonic(),
        )
        self.get_logger().info(
            'BID_ARRAY robot=%s round=%s union=%s bids=%d' % (
                self._robot_id, batch.round_id, batch.union_hash, len(batch.bids),
            )
        )

    def _both_bid_batches_valid(self, now: float) -> bool:
        if self._round is None:
            return False
        for index, robot_id in enumerate(('robot1', 'robot2')):
            snapshot = self._round.snapshots[index]
            received = self._bid_batches.get(robot_id)
            if received is None or not bid_batch_valid(
                    received, now, robot_id, snapshot.source_session_id,
                    snapshot.epoch, self._round.round_id,
                    self._round.union.union_hash, True):
                return False
        return True

    def _publish_decision(self) -> None:
        if self._round is None or self._round.decision is None:
            return
        local_snapshot = self._round.snapshots[0 if self._robot_id == 'robot1' else 1]
        message = decision_to_msg(
            self._round.decision, self._robot_id, local_snapshot.source_session_id,
            self._round.snapshots[0].epoch, self._round.snapshots[1].epoch,
            self._state.value, self._decision_validity_s,
            self.get_clock().now().to_msg(),
        )
        self._decision_publisher.publish(message)
        self._round.decision_published = True
        score = self._round.decision.score
        self.get_logger().info(
            'PAIR_DECISION robot=%s round=%s union=%s hash=%s r1=%s r2=%s '
            'total=%.6f gain=%.6f path=%.6f nearby=%.6f route=%.6f hard=%.6f '
            'sensing=%.6f imbalance=%.6f' % (
                self._robot_id, self._round.round_id, self._round.union.union_hash,
                self._round.decision.decision_hash,
                self._round.decision.robot1_task_id or 'IDLE',
                self._round.decision.robot2_task_id or 'IDLE', score.total,
                score.team_visible_gain, score.combined_path_cost,
                score.nearby_goal_penalty, score.route_overlap_penalty,
                score.hard_failure_penalty, score.sensing_overlap_penalty,
                score.workload_imbalance_penalty,
            )
        )

    def _matching_peer_decision(self, now: float) -> bool:
        if (self._round is None or self._round.decision is None or
                self._peer_decision is None or not self._peer_decision.fresh(now)):
            return False
        peer = self._peer_decision.value
        local = self._round.decision
        expected_session = self._round.snapshots[
            1 if self._peer_id == 'robot2' else 0
        ].source_session_id
        return (
            uuid_to_text(peer.source_session_id) == expected_session and
            peer.round_id == local.round_id and
            peer.union_hash == local.union_hash and
            peer.robot1_snapshot_epoch == self._round.snapshots[0].epoch and
            peer.robot2_snapshot_epoch == self._round.snapshots[1].epoch and
            peer.robot1_bid_fingerprint == local.robot1_bid_fingerprint and
            peer.robot2_bid_fingerprint == local.robot2_bid_fingerprint and
            peer.robot1_canonical_task_id == local.robot1_task_id and
            peer.robot2_canonical_task_id == local.robot2_task_id and
            peer.decision_hash == local.decision_hash
        )

    def _start_local_dispatch(self) -> None:
        if self._round is None or self._round.decision is None:
            return
        task_id = (
            self._round.decision.robot1_task_id if self._robot_id == 'robot1'
            else self._round.decision.robot2_task_id
        )
        if not task_id:
            self._transition(CoordinatorState.WAITING_FOR_INPUTS, 'local assignment is IDLE')
            return
        tasks = {task.canonical_id: task for task in self._round.union.tasks}
        task = tasks.get(task_id)
        if task is None:
            self._invalidate_round(FailureClass.UNKNOWN, 'agreed task missing from union')
            return
        self._dispatch_in_progress = True
        self._active_task = task
        self._active_round_id = self._round.round_id
        self._active_decision_hash = self._round.decision.decision_hash
        round_id = self._active_round_id

        def final_path(result: PathEvaluation):
            if self._active_round_id != round_id:
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
                lambda checks: self._dispatch_after_checks(task, result, checks),
            )

        if not self._nav2.evaluate_path(task.members[0], final_path):
            self._invalidate_round(
                FailureClass.TF_OR_LIFECYCLE,
                'final path action server unavailable',
            )

    def _dispatch_after_checks(
            self, task: CanonicalTask, final_path: PathEvaluation,
            checks: DispatchPreconditions) -> None:
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
            failure = (
                FailureClass.TF_OR_LIFECYCLE if
                (not checks.lifecycle_active or not checks.transform_available)
                else FailureClass.HARD_UNREACHABLE
            )
            self._invalidate_round(failure, checks.reason, final_path)
            return
        if not self._nav2.send_navigation(task.members[0], self._navigation_finished):
            self._invalidate_round(
                FailureClass.ACTION_REJECTION,
                'local NavigateToPose send precondition changed', final_path,
            )
            return
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
        self._emit_event(
            'NAVIGATION_' + result, outcome.error_message or result,
            travelled=outcome.travelled_distance_m,
            duration=outcome.duration_s, recoveries=outcome.recoveries,
            failure=outcome.failure_class,
        )
        if result != 'SUCCEEDED':
            self._publish_failure(outcome.failure_class, outcome.error_message)
        self._active_task = None
        self._active_round_id = ''
        self._active_decision_hash = ''
        self._dispatch_in_progress = False
        self._settle_until_steady_s = time.monotonic() + self._post_goal_settle_s
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
        self._settle_until_steady_s = time.monotonic() + self._post_goal_settle_s
        self._reset_round('round invalidated')

    def _publish_failure(
            self, failure: FailureClass, reason: str,
            path: Optional[PathEvaluation] = None) -> None:
        if self._active_task is None:
            return
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
        member = self._active_task.members[0]
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
        return '' if self._round is None else self._round.round_id

    def _peer_state_text(self) -> str:
        if self._peer_status is None:
            return 'UNKNOWN'
        return str(self._peer_status.value.state)

    def _publish_status(self) -> None:
        message = DistributedExplorationStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'shared_map'
        message.source_robot_id = self._robot_id
        session = self._local_session_text()
        if session:
            message.source_session_id = text_to_uuid(session)
        message.state = STATE_TO_MESSAGE[self._state]
        message.round_id = self._current_round_id()
        message.union_hash = '' if self._round is None else self._round.union.union_hash
        decision = None if self._round is None else self._round.decision
        message.decision_hash = '' if decision is None else decision.decision_hash
        message.active_canonical_task_id = (
            '' if self._active_task is None else self._active_task.canonical_id
        )
        message.local_nav_goal_active = self._nav2.local_goal_active
        message.nav2_healthy = self._state != CoordinatorState.BLOCKED
        message.tf_healthy = self._state != CoordinatorState.BLOCKED
        message.candidate_source_healthy = self._fresh_snapshot(
            self._robot_id, time.monotonic(),
        ) is not None
        message.peer_communication_healthy = self._peer_liveness.state != (
            CoordinatorState.DEGRADED_SOLO
        )
        message.validity = seconds_to_duration(2.5)
        message.reason = self._state_reason
        self._status_publisher.publish(message)

    def _emit_event(
            self, event_type: str, reason: str, previous: str = '',
            next_state: str = '', path_length: float = 0.0,
            travelled: float = 0.0, duration: float = 0.0,
            recoveries: int = 0, failure: FailureClass = FailureClass.UNKNOWN) -> None:
        message = DistributedExplorationEvent()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'shared_map'
        message.source_robot_id = self._robot_id
        session = self._local_session_text()
        if session:
            message.source_session_id = text_to_uuid(session)
        message.event_type = event_type
        message.round_id = self._active_round_id or self._current_round_id()
        message.union_hash = '' if self._round is None else self._round.union.union_hash
        message.decision_hash = self._active_decision_hash
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
        if self._round is not None and self._round.decision is not None:
            message.route_overlap_score = self._round.decision.score.route_overlap_penalty
            message.sensing_overlap_estimate = (
                self._round.decision.score.sensing_overlap_penalty
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
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
