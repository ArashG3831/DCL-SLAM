"""Compose the six primitives of the minimal decentralized coordinator.

This module owns ROS I/O and the small current evaluation/navigation state.
It does not detect frontiers, plan paths, implement traffic geometry, retain
history, run certificates or continuation rounds, or recover infrastructure.
"""

from __future__ import annotations

import json
from dataclasses import replace

from action_msgs.msg import GoalStatus
from my_epuck_interfaces.msg import (
    DistributedExplorationStatus,
    DistributedExplorationEvent,
    FrontierCandidateArray,
    TaskBidArray,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from . import minimal_frontier_navigation as navigation
from . import minimal_frontier_protocol as protocol
from . import minimal_frontier_selection as selection
from . import minimal_frontier_sets as frontier_sets
from . import minimal_frontier_termination as termination
from . import minimal_frontier_traffic as traffic
from .distributed_assignment.local_nav2 import (
    DispatchPreconditions,
    LocalNav2,
    NavigationOutcome,
)
from .mission_termination import CandidateEvidence


class MinimalFrontierAllocator:
    """Robot-local composition object; the caller supplies the ROS node."""

    IDLE = 'IDLE'
    EVALUATING = 'EVALUATING'
    WAITING_TRAFFIC = 'WAITING_TRAFFIC'
    GOAL_PENDING = 'GOAL_PENDING'
    NAVIGATING = 'NAVIGATING'

    def __init__(
        self, node: Node, robot_id: str | None = None, *,
            configure_io: bool = True) -> None:
        self._node = node
        if robot_id is None:
            configured_id = node.declare_parameter('robot_id', '').value
            self._robot_id = str(configured_id)
        else:
            self._robot_id = str(robot_id)
        if self._robot_id not in ('robot1', 'robot2'):
            raise ValueError('robot_id must be robot1 or robot2')
        self._peer_id = 'robot2' if self._robot_id == 'robot1' else 'robot1'
        self._source_session_id = '0' * 32
        self._nav: LocalNav2 | None = None
        self._bid_publisher = None
        self._status_publisher = None
        self._event_publisher = None
        self._start_ready_publisher = None
        self._start_ready_published = False
        self._initialize_state()
        if configure_io:
            self._configure_io()

    def _initialize_state(self) -> None:
        self._candidates = {'robot1': None, 'robot2': None}
        self._evidence = {
            'robot1': CandidateEvidence(),
            'robot2': CandidateEvidence(),
        }
        self._union = None
        self._local_batch = None
        self._local_batch_from_costing = False
        self._peer_batch = None
        self._peer_active_goal: str | None = None
        self._peer_status_union_hash: str | None = None
        self._peer_status_state: int | None = None
        self._peer_status_current = False
        self._peer_bid_after_terminal = True
        self._peer_terminal = False
        self._peer_terminal_reason = ''
        self._peer_status_after_terminal = True
        self._active_goal_id: str | None = None
        self._active_goal_union_hash: str | None = None
        self._goal_token = 0
        self._state = self.IDLE
        self._completed_ids: set[str] = set()
        self._local_completed_ids: set[str] = set()
        self._traffic_decision = None
        self._terminal_reason: str | None = None
        self._allocation_reason = 'initial allocation'
        self._stable_since_s = self._now_s()
        self._released = True
        self._start_release_required = False
        self._stability_grace_s = 5.0
        self._bid_validity_s = 0.0
        self._minimum_visible_gain_m = 0.05
        self._safe_radius = 0.08
        self._reference_speed = 0.13
        self._eta_tie_s = 0.05

    def _configure_io(self) -> None:
        declare = self._node.declare_parameter
        self._start_release_required = bool(
            declare('common_start_release_required', False).value)
        self._released = not self._start_release_required
        self._stability_grace_s = float(
            declare('map_stability_grace_s', 5.0).value)
        self._bid_validity_s = float(declare('bid_validity_s', 0.0).value)
        self._minimum_visible_gain_m = float(
            declare('minimum_solo_visible_gain_m', 0.05).value)
        self._safe_radius = float(
            declare('traffic_safe_radius_m', 0.08).value)
        self._reference_speed = float(
            declare('traffic_reference_speed_mps', 0.13).value)
        self._eta_tie_s = float(declare('traffic_eta_tie_s', 0.05).value)
        self._nav = LocalNav2(self._node, phase_gated=False)

        reliable = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        volatile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        candidate_topic = str(
            declare('candidate_topic', 'frontier_candidates').value).lstrip('/')
        bid_topic = f'/{self._robot_id}/task_bids'
        status_topic = f'/{self._robot_id}/distributed_status'
        self._bid_publisher = self._node.create_publisher(
            TaskBidArray, bid_topic, reliable)
        self._status_publisher = self._node.create_publisher(
            DistributedExplorationStatus, status_topic, reliable)
        event_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._event_publisher = self._node.create_publisher(
            DistributedExplorationEvent,
            f'/{self._robot_id}/distributed_event',
            event_qos,
        )
        if bool(declare('publish_cooperative_start_ready', False).value):
            self._start_ready_publisher = self._node.create_publisher(
                String,
                f'/cslam/unknown_pose/cooperative_start_ready/{self._robot_id}',
                QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            )
        for robot in ('robot1', 'robot2'):
            topic = f'/{robot}/{candidate_topic}'
            self._node.create_subscription(
                FrontierCandidateArray, topic, self.on_candidate_array, volatile)
        self._node.create_subscription(
            TaskBidArray,
            f'/{self._peer_id}/task_bids',
            self.on_peer_bid,
            reliable,
        )
        self._node.create_subscription(
            DistributedExplorationStatus,
            f'/{self._peer_id}/distributed_status',
            self.on_peer_status,
            reliable,
        )
        self._node.create_subscription(
            DistributedExplorationEvent,
            f'/{self._peer_id}/distributed_event',
            self.on_peer_event,
            event_qos,
        )
        if self._start_release_required:
            self._node.create_subscription(
                String,
                '/cslam/unknown_pose/start_release',
                self.on_start_release,
                reliable,
            )
        self._node.create_timer(0.5, self._tick)

    def _now_s(self) -> float:
        return self._node.get_clock().now().nanoseconds / 1e9

    def _log_info(self, message: str) -> None:
        get_logger = getattr(self._node, 'get_logger', None)
        if get_logger is None:
            return
        logger = get_logger()
        logger.info(message)

    @staticmethod
    def _message_stamp_ns(message) -> int:
        stamp = message.header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _make_evidence(self, message: FrontierCandidateArray) -> CandidateEvidence:
        reachable = sum(
            candidate.reachability_state == candidate.REACHABLE
            for candidate in message.candidates
        )
        actionable = sum(
            candidate.reachability_state == candidate.REACHABLE and
            candidate.information_gain >= self._minimum_visible_gain_m
            for candidate in message.candidates
        )
        return CandidateEvidence(
            detected=int(message.detected_frontier_count),
            small=int(message.small_frontier_count),
            reachable=reachable,
            out_of_range=int(message.out_of_range_frontier_count),
            unreachable=int(message.unreachable_frontier_count),
            planner_failures=int(message.planner_failure_count),
            unclassified=int(message.unclassified_frontier_count),
            detected_not_queried=int(message.detected_not_queried_count),
            below_minimum_gain=reachable - actionable,
            actionable_reachable=actionable,
        )

    def _publish_status(self) -> None:
        if self._status_publisher is None:
            return
        message = DistributedExplorationStatus()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.header.frame_id = 'shared_map'
        message.source_robot_id = self._robot_id
        message.active_canonical_task_id = self._active_goal_id or ''
        message.local_nav_goal_active = bool(self._active_goal_id)
        states = {
            self.IDLE: message.WAITING_FOR_INPUTS,
            self.EVALUATING: message.BIDDING,
            self.WAITING_TRAFFIC: message.WAITING_FOR_TRAFFIC,
            self.GOAL_PENDING: message.BIDDING,
            self.NAVIGATING: message.NAVIGATING,
        }
        message.state = states[self._state]
        message.union_hash = self._union.union_hash if self._union else ''
        message.round_id = message.union_hash
        message.terminal = self._terminal_reason is not None
        message.terminal_reason = self._terminal_reason or ''
        message.reason = self._allocation_reason or self._state
        self._status_publisher.publish(message)
        self._allocation_reason = ''

    def _invalidate_pending_goal(self) -> None:
        if self._state != self.GOAL_PENDING:
            return
        self._goal_token += 1
        self._active_goal_id = None
        self._traffic_decision = None
        self._state = self.EVALUATING if self._released else self.IDLE

    def _local_batch_for(self, union):
        current_ids = {task.canonical_id for task in union.tasks}
        self._completed_ids.intersection_update(current_ids)
        self._local_completed_ids.intersection_update(current_ids)
        array = self._candidates[self._robot_id]
        bids = frontier_sets.materialize_bids(array, union)
        bids = tuple(
            replace(
                bid,
                path_valid=False,
                path_length_m=0.0,
                estimated_travel_cost=0.0,
                heading_cost=0.0,
                path=(),
            )
            if bid.canonical_task_id in self._completed_ids else bid
            for bid in bids
        )
        return protocol.make_batch(
            self._robot_id,
            self._source_session_id,
            int(array.candidate_generation_id),
            union,
            bids,
            self._bid_validity_s,
        )

    @staticmethod
    def _has_cost_evidence(message) -> bool:
        return any(
            candidate.reachability_state == candidate.REACHABLE
            for candidate in message.candidates
        )

    @staticmethod
    def _is_geometry_only(message) -> bool:
        return (
            bool(message.candidates) and
            not MinimalFrontierAllocator._has_cost_evidence(message) and
            bool(message.detected_not_queried_count)
        )

    def _passive_batch_for(self, union):
        previous = {
            bid.canonical_task_id: bid
            for bid in (self._local_batch.bids if self._local_batch else ())
        }
        batch = self._local_batch_for(union)
        active_id = self._active_goal_id
        active_bid = previous.get(active_id)
        if not active_id:
            return batch
        if active_bid is None:
            active_bid = next(
                (bid for bid in batch.bids if bid.canonical_task_id == active_id),
                None,
            )
        unavailable = tuple(
            replace(
                bid,
                path_valid=False,
                path_length_m=0.0,
                estimated_travel_cost=0.0,
                heading_cost=0.0,
                path=(),
            )
            if bid.canonical_task_id != active_id or active_bid is None else active_bid
            for bid in batch.bids
        )
        return replace(batch, bids=unavailable)

    def _costing_pending(self) -> bool:
        local = self._candidates[self._robot_id]
        if self._is_geometry_only(local):
            return True
        peer = self._candidates[self._peer_id]
        return self._peer_active_goal is None and self._is_geometry_only(peer)

    def _publish_batch(self) -> None:
        if self._local_batch is None:
            return
        if self._bid_publisher is not None:
            stamp = self._node.get_clock().now().to_msg()
            self._bid_publisher.publish(protocol.to_msg(self._local_batch, stamp))
        self._publish_completion_events()

    def _publish_completion_events(
            self, task_ids: tuple[str, ...] | None = None,
            union_hash: str | None = None) -> None:
        if self._event_publisher is None or self._union is None:
            return
        union_hash = union_hash or self._union.union_hash
        task_ids = task_ids or tuple(sorted(self._local_completed_ids))
        for task_id in task_ids:
            event = DistributedExplorationEvent()
            event.header.stamp = self._node.get_clock().now().to_msg()
            event.header.frame_id = 'shared_map'
            event.source_robot_id = self._robot_id
            event.event_type = 'NAVIGATION_SUCCEEDED'
            event.round_id = union_hash
            event.union_hash = union_hash
            event.canonical_task_id = task_id
            event.physical_task_signature = task_id
            event.result = event.event_type
            self._event_publisher.publish(event)

    def on_candidate_array(self, message: FrontierCandidateArray) -> None:
        source = str(message.source_robot_id)
        if source not in self._candidates:
            return
        evidence = self._make_evidence(message)
        if evidence != self._evidence[source]:
            self._stable_since_s = self._now_s()
        self._candidates[source] = message
        self._evidence[source] = evidence
        first = self._candidates['robot1']
        second = self._candidates['robot2']
        if first is None or second is None:
            return
        self._publish_start_ready()

        union = frontier_sets.build_union(first, second)
        if union is None:
            self._invalidate_pending_goal()
            self._union = None
            self._local_batch = None
            self._peer_batch = None
            self._peer_status_union_hash = None
            self._peer_status_state = None
            self._peer_status_current = False
            self._peer_terminal = False
            self._peer_terminal_reason = ''
            self._terminal_reason = None
            if self._active_goal_id is None:
                self._state = self.EVALUATING if self._released else self.IDLE
            self._publish_status()
            return

        changed = self._union is None or union.union_hash != self._union.union_hash
        if changed:
            self._invalidate_pending_goal()
            self._stable_since_s = self._now_s()
            self._peer_batch = None
            self._peer_status_union_hash = None
            self._peer_status_state = None
            self._peer_status_current = False
            self._peer_terminal = False
            self._peer_terminal_reason = ''
            self._terminal_reason = None
        self._union = union
        if self._active_goal_id is not None:
            if changed or self._local_batch is None:
                self._local_batch = self._passive_batch_for(union)
            self._publish_batch()
            self._publish_status()
            return
        geometry_only = self._is_geometry_only(message)
        same_batch_union = (
            self._local_batch is not None and
            self._local_batch.union_hash == union.union_hash
        )
        keep_cost_batch = (
            geometry_only and same_batch_union and
            self._local_batch_from_costing
        )
        if not keep_cost_batch:
            self._local_batch = self._local_batch_for(union)
            self._local_batch_from_costing = not geometry_only
            self._publish_batch()
        else:
            self._publish_completion_events()
        if self._active_goal_id is None and self._state != self.GOAL_PENDING:
            self._state = self.EVALUATING if self._released else self.IDLE
        self._publish_status()
        self._maybe_terminal()
        self._try_allocate()

    def _publish_start_ready(self) -> None:
        if self._start_ready_publisher is None or self._start_ready_published:
            return
        message = String()
        message.data = json.dumps({
            'event': 'COOPERATIVE_START_STATE_READY',
            'robot_id': self._robot_id,
            'traffic_scheduler_ready': True,
        }, sort_keys=True, separators=(',', ':'))
        self._start_ready_publisher.publish(message)
        self._start_ready_published = True

    def on_peer_bid(self, message: TaskBidArray) -> None:
        if (not self._peer_bid_after_terminal and
                self._message_stamp_ns(message) <
                int(self._stable_since_s * 1e9)):
            return
        try:
            batch = protocol.from_msg(message)
        except (TypeError, ValueError):
            return
        if batch.source_robot_id != self._peer_id:
            return
        if self._union is None or batch.union_hash != self._union.union_hash:
            return
        self._peer_batch = batch
        self._peer_bid_after_terminal = True
        self._publish_completion_events()
        self._try_allocate()

    def on_peer_status(self, message: DistributedExplorationStatus) -> None:
        if message.source_robot_id != self._peer_id:
            return
        if (not self._peer_status_after_terminal and
                self._message_stamp_ns(message) <
                int(self._stable_since_s * 1e9)):
            return
        if self._union is None:
            return
        if (str(message.union_hash) != self._union.union_hash or
                str(message.round_id) != self._union.union_hash):
            return
        if message.local_nav_goal_active and not message.terminal:
            task_id = str(message.active_canonical_task_id)
            self._peer_active_goal = task_id or None
        else:
            self._peer_active_goal = None
        self._peer_status_union_hash = self._union.union_hash
        self._peer_status_state = int(message.state)
        self._peer_terminal = bool(message.terminal)
        self._peer_terminal_reason = str(message.terminal_reason)
        self._peer_status_after_terminal = True
        self._peer_status_current = True
        self._maybe_terminal()
        self._try_allocate()

    def on_peer_event(self, message: DistributedExplorationEvent) -> None:
        if message.source_robot_id != self._peer_id:
            return
        if message.event_type != 'NAVIGATION_SUCCEEDED':
            return
        if self._union is None or message.union_hash != self._union.union_hash:
            return
        current_ids = {task.canonical_id for task in self._union.tasks}
        task_id = str(message.canonical_task_id)
        if task_id not in current_ids:
            return
        self._completed_ids.add(task_id)
        if self._active_goal_id is not None:
            self._local_batch = self._passive_batch_for(self._union)
        else:
            self._local_batch = self._local_batch_for(self._union)
            self._local_batch_from_costing = self._has_cost_evidence(
                self._candidates[self._robot_id]
            )
        self._publish_batch()
        self._publish_status()
        self._try_allocate()

    def on_start_release(self, message: String) -> None:
        try:
            released = json.loads(str(message.data)).get('event') == 'START_RELEASE'
        except (TypeError, ValueError, json.JSONDecodeError):
            released = False
        if released:
            self._released = True
            self._try_allocate()

    @staticmethod
    def _bid_path(batch, task_id):
        if batch is None or not task_id:
            return ()
        for bid in batch.bids:
            if bid.canonical_task_id == task_id:
                return bid.path
        return ()

    def _try_allocate(self) -> None:
        if not self._released:
            return
        if self._terminal_reason is not None:
            return
        if self._union is None or self._local_batch is None:
            return
        if self._peer_batch is None or self._active_goal_id is not None:
            return
        if not self._peer_status_after_terminal:
            return
        if not self._peer_status_current:
            return
        if not self._peer_bid_after_terminal:
            return
        if self._state == self.GOAL_PENDING:
            return

        if self._robot_id == 'robot1':
            robot1_batch = self._local_batch
            robot2_batch = self._peer_batch
        else:
            robot1_batch = self._peer_batch
            robot2_batch = self._local_batch
        if not protocol.complete_pair(self._union, robot1_batch, robot2_batch):
            return
        if self._costing_pending():
            self._state = self.EVALUATING
            return

        known_ids = {task.canonical_id for task in self._union.tasks}
        peer_active = self._peer_active_goal
        peer_active_in_union = peer_active in known_ids
        if not peer_active_in_union:
            peer_active = ''
        peer_active_is_valid = any(
            bid.canonical_task_id == peer_active and bid.path_valid
            for bid in self._peer_batch.bids
        )
        if peer_active_in_union and not peer_active_is_valid:
            return
        selector_peer_active = peer_active if peer_active_is_valid else ''
        if self._robot_id == 'robot1':
            active1, active2 = '', selector_peer_active
        else:
            active1, active2 = selector_peer_active, ''
        self._state = self.EVALUATING
        assignment = selection.choose_assignment(
            self._union,
            robot1_batch,
            robot2_batch,
            active_robot1_id=active1,
            active_robot2_id=active2,
        )
        if assignment is None:
            self._state = self.IDLE
            self._publish_status()
            return

        if self._robot_id == 'robot1':
            local_id = assignment[0]
        else:
            local_id = assignment[1]
        if not local_id or local_id in self._completed_ids:
            self._state = self.IDLE
            self._publish_status()
            self._maybe_terminal()
            return
        if local_id == peer_active:
            self._state = self.IDLE
            self._publish_status()
            return

        path1 = self._bid_path(robot1_batch, assignment[0])
        path2 = self._bid_path(robot2_batch, assignment[1])
        active_robots = frozenset(
            robot
            for robot, task_id in (
                ('robot1', active1),
                ('robot2', active2),
            )
            if task_id
        )
        self._traffic_decision = traffic.decide(
            path1,
            path2,
            robot1_safe_radius_m=self._safe_radius,
            robot2_safe_radius_m=self._safe_radius,
            reference_speed_mps=self._reference_speed,
            eta_tie_s=self._eta_tie_s,
            active_robots=active_robots,
        )
        if traffic.should_wait(self._robot_id, self._traffic_decision):
            self._state = self.WAITING_TRAFFIC
            self._publish_status()
            return

        candidate = next(
            (
                item
                for item in self._candidates[self._robot_id].candidates
                if str(item.frontier_id) == local_id
            ),
            None,
        )
        if candidate is not None:
            path = path1 if self._robot_id == 'robot1' else path2
            self._dispatch(candidate, path)

    def _dispatch(self, candidate, path) -> None:
        if self._nav is None or self._active_goal_id is not None:
            return
        task = navigation.to_physical_task(candidate, self._robot_id)
        pending_union_hash = self._union.union_hash if self._union else ''
        pending_task_id = str(candidate.frontier_id)
        self._goal_token += 1
        token = self._goal_token
        self._active_goal_id = str(candidate.frontier_id)
        self._active_goal_union_hash = pending_union_hash
        self._state = self.GOAL_PENDING
        self._local_batch = self._passive_batch_for(self._union)
        self._publish_batch()
        self._publish_status()
        send_started = False

        def precondition_result(result: DispatchPreconditions) -> None:
            nonlocal send_started
            if token != self._goal_token or self._state != self.GOAL_PENDING:
                return
            if (self._union is None or
                    self._union.union_hash != pending_union_hash or
                    self._active_goal_id != pending_task_id):
                self._clear_goal(token)
                return
            if self._peer_active_goal == pending_task_id:
                self._clear_goal(token)
                return
            if not result.ready or send_started:
                self._clear_goal(token)
                return
            send_started = True
            self._state = self.NAVIGATING
            self._publish_status()
            sent = navigation.send(
                self._nav,
                task,
                lambda outcome: self.on_navigation_outcome(token, outcome),
                tuple(path),
            )
            if sent:
                self._log_info(
                    'MINIMAL_ALLOCATOR_NAVIGATION_DISPATCH '
                    f'robot={self._robot_id} task_id={pending_task_id} '
                    f'union_hash={pending_union_hash} sim_time={self._now_s():.6f}'
                )
            if not sent:
                self._clear_goal(token)

        navigation.check_preconditions(
            self._nav,
            task,
            bool(task.local_path_valid),
            precondition_result,
            tuple(path),
            'shared_map',
        )

    def on_navigation_outcome(
            self, token: int, outcome: NavigationOutcome) -> None:
        if token != self._goal_token or self._active_goal_id is None:
            return
        if outcome.accepted and outcome.status == GoalStatus.STATUS_SUCCEEDED:
            completed_id = self._active_goal_id
            self._completed_ids.add(completed_id)
            self._local_completed_ids.add(completed_id)
            self._publish_completion_events(
                (completed_id,), self._active_goal_union_hash,
            )
        if outcome.status == GoalStatus.STATUS_SUCCEEDED:
            self._allocation_reason = 'goal success'
        elif outcome.status == GoalStatus.STATUS_CANCELED:
            self._allocation_reason = 'goal cancel/watchdog'
        else:
            self._allocation_reason = 'goal failure'
        self._log_info(
            'MINIMAL_ALLOCATOR_NAVIGATION_TERMINAL '
            f'robot={self._robot_id} status={outcome.status} '
            f'sim_time={self._now_s():.6f}'
        )
        self._peer_status_after_terminal = False
        self._peer_status_current = False
        self._peer_bid_after_terminal = False
        self._stable_since_s = self._now_s()
        self._clear_goal(token)

    def _clear_goal(self, token: int) -> None:
        if token != self._goal_token:
            return
        self._goal_token += 1
        self._active_goal_id = None
        self._active_goal_union_hash = None
        self._traffic_decision = None
        self._state = self.EVALUATING if self._released else self.IDLE
        self._local_batch = None
        self._local_batch_from_costing = False
        self._publish_status()

    def cancel_active(self) -> bool:
        if self._nav is None or self._active_goal_id is None:
            return False
        return navigation.cancel(self._nav)

    def _maybe_terminal(self) -> None:
        reason = None
        if (self._active_goal_id is None and
                self._state not in (self.GOAL_PENDING, self.WAITING_TRAFFIC) and
                self._union is not None and
                self._candidates['robot1'] is not None and
                self._candidates['robot2'] is not None and
                self._peer_status_union_hash == self._union.union_hash and
                self._peer_active_goal is None and
                self._peer_state_allows_terminal() and
                self._local_and_peer_batches_complete()):
            classified = termination.classify(
                self._evidence['robot1'],
                self._evidence['robot2'],
                stable_for_s=max(0.0, self._now_s() - self._stable_since_s),
                stability_grace_s=self._stability_grace_s,
            )
            if classified is not None:
                local_reason = str(getattr(classified, 'value', classified))
                reason = termination.matching(
                    True, local_reason, self._peer_terminal,
                    self._peer_terminal_reason,
                )
        if reason != self._terminal_reason:
            self._terminal_reason = reason
            self._publish_status()

    def _peer_state_allows_terminal(self) -> bool:
        return self._peer_status_state in (
            DistributedExplorationStatus.WAITING_FOR_INPUTS,
            DistributedExplorationStatus.COMPLETE,
        )

    def _local_and_peer_batches_complete(self) -> bool:
        if self._local_batch is None or self._peer_batch is None:
            return False
        if self._robot_id == 'robot1':
            robot1_batch, robot2_batch = self._local_batch, self._peer_batch
        else:
            robot1_batch, robot2_batch = self._peer_batch, self._local_batch
        return protocol.complete_pair(self._union, robot1_batch, robot2_batch)

    def _tick(self) -> None:
        self._maybe_terminal()
        if self._state in (self.EVALUATING, self.WAITING_TRAFFIC):
            self._try_allocate()


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = Node('minimal_frontier_allocator')
    MinimalFrontierAllocator(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
