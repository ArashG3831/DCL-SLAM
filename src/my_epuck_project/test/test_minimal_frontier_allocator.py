"""Offline behavior tests for the composed minimal coordinator."""

import json
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from my_epuck_interfaces.msg import (
    DistributedExplorationEvent,
    DistributedExplorationStatus,
    FrontierCandidate,
    FrontierCandidateArray,
)
from std_msgs.msg import String

from my_epuck_project import minimal_frontier_allocator as composition
from my_epuck_project.distributed_assignment.local_nav2 import (
    DispatchPreconditions, FailureClass, NavigationOutcome,
)
from my_epuck_project.distributed_assignment.traffic_scheduler import TrafficDecision
from my_epuck_project.minimal_frontier_protocol import to_msg


class _Clock:
    def __init__(self):
        self.seconds = 0.0

    def now(self):
        return SimpleNamespace(
            nanoseconds=int(self.seconds * 1e9),
            to_msg=lambda: SimpleNamespace(sec=int(self.seconds), nanosec=0),
        )


class _Node:
    def __init__(self, clock=None):
        self.clock = clock or _Clock()

    def get_clock(self):
        return self.clock


class _Nav:
    def __init__(self, *, automatic_preconditions=True):
        self.automatic_preconditions = automatic_preconditions
        self.precondition_callbacks = []
        self.send_calls = []
        self.cancel_calls = 0
        self.ready = DispatchPreconditions(
            True, True, True, None, True, True, None, None, True, True,
            True, '')

    def check_dispatch_preconditions(
            self, task, final_path_valid, callback, path_samples, path_frame_id):
        self.precondition_callbacks.append(callback)
        if self.automatic_preconditions:
            callback(self.ready)

    def send_navigation(self, task, callback, diagnostic_path):
        self.send_calls.append((task, callback, diagnostic_path))
        return True

    def cancel_navigation(self):
        self.cancel_calls += 1
        return True

    def release_preconditions(self):
        callback, self.precondition_callbacks = self.precondition_callbacks[0], self.precondition_callbacks[1:]
        callback(self.ready)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _point(x, y):
    return Point(x=float(x), y=float(y), z=0.0)


def _candidate(frontier_id, *, x=None, reachable=True, path=None):
    x = float(frontier_id if x is None else x)
    path = path or ((x, 0.0), (x + 0.5, 0.0))
    candidate = FrontierCandidate()
    candidate.frontier_id = frontier_id
    candidate.centroid = _point(x, 0.0)
    candidate.bounding_box_min = _point(x - 0.1, -0.1)
    candidate.bounding_box_max = _point(x + 0.1, 0.1)
    candidate.approach_pose.pose.position = _point(x, 0.0)
    candidate.approach_pose.pose.orientation.w = 1.0
    candidate.information_gain = 1.0
    candidate.score = 1.0
    candidate.path_length_m = 0.5
    candidate.local_path_length_m = 0.5
    candidate.heading_change_rad = 0.0
    candidate.local_path_samples = [_point(px, py) for px, py in path]
    candidate.reachability_state = (
        FrontierCandidate.REACHABLE if reachable else FrontierCandidate.UNKNOWN)
    return candidate


def _array(robot, ids=(1, 2), *, candidates=None, map_revision=1,
           detected=None, unreachable=0, planner_failures=0,
           detected_not_queried=0):
    message = FrontierCandidateArray()
    message.header.frame_id = 'shared_map'
    message.source_robot_id = robot
    message.map_revision = map_revision
    message.costmap_revision = 2
    message.candidate_generation_id = map_revision
    message.candidates = list(candidates) if candidates is not None else [
        _candidate(item) for item in ids]
    message.detected_frontier_count = len(ids) if detected is None else detected
    message.unreachable_frontier_count = unreachable
    message.planner_failure_count = planner_failures
    message.detected_not_queried_count = detected_not_queried
    return message


def _allocator(robot, clock=None, *, nav=None):
    instance = composition.MinimalFrontierAllocator(
        _Node(clock), robot, configure_io=False)
    instance._nav = nav or _Nav()
    instance._event_publisher = _Publisher()
    return instance


def _feed_pair(first, second, array1, array2):
    for instance in (first, second):
        instance.on_candidate_array(array1)
        instance.on_candidate_array(array2)


def _exchange(first, second):
    first.on_peer_bid(to_msg(second._local_batch, SimpleNamespace(sec=0, nanosec=0)))
    second.on_peer_bid(to_msg(first._local_batch, SimpleNamespace(sec=0, nanosec=0)))


def _ready_pair(ids=(1, 2), *, nav1=None, nav2=None):
    first = _allocator('robot1', nav=nav1)
    second = _allocator('robot2', nav=nav2)
    array1 = _array('robot1', ids)
    array2 = _array('robot2', ids)
    _feed_pair(first, second, array1, array2)
    _exchange(first, second)
    return first, second, array1, array2


def _success():
    return NavigationOutcome(
        True, GoalStatus.STATUS_SUCCEEDED, 0, '', FailureClass.UNKNOWN,
        1.0, 0.5, 0)


def _failure():
    return NavigationOutcome(
        True, GoalStatus.STATUS_ABORTED, 1, 'failed',
        FailureClass.CONTROLLER_NO_PROGRESS, 1.0, 0.1, 0)


def test_start_release_gate_blocks_then_allows_dispatch():
    first, second = _allocator('robot1'), _allocator('robot2')
    first._start_release_required = second._start_release_required = True
    first._released = second._released = False
    array1, array2 = _array('robot1'), _array('robot2')
    _feed_pair(first, second, array1, array2)
    _exchange(first, second)
    assert not first._nav.send_calls and not second._nav.send_calls
    release = String(data=json.dumps({'event': 'START_RELEASE'}))
    first.on_start_release(release)
    second.on_start_release(release)
    assert first._nav.send_calls or second._nav.send_calls


def test_basic_cooperative_pair_uses_distinct_deterministic_assignments():
    first, second, _, _ = _ready_pair()
    assert first._active_goal_id and second._active_goal_id
    assert first._active_goal_id != second._active_goal_id


def test_different_raw_candidate_sets_use_union_and_unavailable_entries():
    first, second = _allocator('robot1'), _allocator('robot2')
    array1, array2 = _array('robot1', (1, 2, 3)), _array('robot2', (1, 2, 4))
    _feed_pair(first, second, array1, array2)
    assert tuple(t.canonical_id for t in first._union.tasks) == ('1', '2', '3', '4')
    assert any(not b.path_valid for b in first._local_batch.bids if b.canonical_task_id == '4')
    assert any(not b.path_valid for b in second._local_batch.bids if b.canonical_task_id == '3')
    _exchange(first, second)
    assert first._active_goal_id != second._active_goal_id


def test_wrong_peer_union_never_dispatches():
    first, second = _allocator('robot1'), _allocator('robot2')
    old1, old2 = _array('robot1'), _array('robot2')
    _feed_pair(first, second, old1, old2)
    stale = to_msg(second._local_batch, SimpleNamespace(sec=0, nanosec=0))
    new1, new2 = _array('robot1', (9,)), _array('robot2', (9,))
    _feed_pair(first, second, new1, new2)
    first.on_peer_bid(stale)
    assert not first._nav.send_calls and first._peer_batch is None


def test_new_union_replaces_evaluation_without_retaining_old_context():
    first, second = _allocator('robot1'), _allocator('robot2')
    _feed_pair(first, second, _array('robot1'), _array('robot2'))
    old_hash = first._union.union_hash
    _feed_pair(first, second, _array('robot1', (7,)), _array('robot2', (8,)))
    assert first._union.union_hash != old_hash
    assert first._peer_batch is None


def test_map_churn_does_not_cancel_accepted_navigation():
    first, second, _, _ = _ready_pair()
    active = first if first._active_goal_id else second
    old_goal = active._active_goal_id
    _feed_pair(first, second, _array('robot1', (3, 4)), _array('robot2', (3, 4)))
    assert active._active_goal_id == old_goal
    assert active._nav.cancel_calls == 0


def test_one_busy_robot_leaves_only_idle_robot_for_new_assignment():
    first, second = _allocator('robot1'), _allocator('robot2')
    first._active_goal_id, first._state = '1', first.NAVIGATING
    arrays = (_array('robot1'), _array('robot2'))
    _feed_pair(first, second, *arrays)
    status = DistributedExplorationStatus()
    status.source_robot_id = 'robot1'; status.local_nav_goal_active = True; status.active_canonical_task_id = '1'
    second.on_peer_status(status)
    _exchange(first, second)
    assert first._active_goal_id == '1'
    assert not first._nav.send_calls
    assert second._active_goal_id and second._active_goal_id != '1'


def test_both_busy_robots_receive_no_new_assignment():
    first, second = _allocator('robot1'), _allocator('robot2')
    first._active_goal_id, first._state = '1', first.NAVIGATING
    second._active_goal_id, second._state = '2', second.NAVIGATING
    _feed_pair(first, second, _array('robot1'), _array('robot2'))
    for instance, peer, task in ((first, 'robot2', '2'), (second, 'robot1', '1')):
        status = DistributedExplorationStatus(); status.source_robot_id = peer
        status.local_nav_goal_active = True; status.active_canonical_task_id = task
        instance.on_peer_status(status)
    _exchange(first, second)
    assert not first._nav.send_calls and not second._nav.send_calls


def test_traffic_waiter_sends_only_after_current_recomputation(monkeypatch):
    first, second = _allocator('robot1'), _allocator('robot2')
    monkeypatch.setattr(composition.traffic, 'decide', lambda *a, **k: TrafficDecision(conflict=True, waiting_robot_id='robot1'))
    _feed_pair(first, second, _array('robot1'), _array('robot2'))
    _exchange(first, second)
    assert first._state == first.WAITING_TRAFFIC and not first._nav.send_calls
    monkeypatch.setattr(composition.traffic, 'decide', lambda *a, **k: TrafficDecision())
    first._tick()
    assert first._nav.send_calls


def test_goal_pending_has_exactly_one_send_attempt():
    nav = _Nav(automatic_preconditions=False)
    first, second, _, _ = _ready_pair(nav1=nav)
    pending = first if first._state == first.GOAL_PENDING else second
    assert pending._state == pending.GOAL_PENDING
    pending._tick()
    assert len(nav.precondition_callbacks) == 1 and not nav.send_calls
    nav.release_preconditions()
    assert len(nav.send_calls) == 1


def test_success_suppresses_delayed_stale_candidate():
    first, second, array1, array2 = _ready_pair(ids=(1,))
    winner = first if first._active_goal_id else second
    completed = winner._active_goal_id
    winner.on_navigation_outcome(winner._goal_token, _success())
    assert completed in winner._completed_ids and winner._active_goal_id is None
    _feed_pair(first, second, array1, array2)
    _exchange(first, second)
    assert winner._active_goal_id is None


def test_basic_peer_completion_event_suppresses_frontier():
    first, second, _, _ = _ready_pair(ids=(1,))
    winner = first if first._active_goal_id else second
    loser = second if winner is first else first
    winner.on_navigation_outcome(winner._goal_token, _success())
    event = winner._event_publisher.messages[-1]
    assert event.event_type == 'NAVIGATION_SUCCEEDED'
    assert event.canonical_task_id == '1'
    assert event.union_hash == winner._union.union_hash
    loser.on_peer_event(event)
    assert '1' in loser._completed_ids
    assert not any(
        bid.canonical_task_id == '1' and bid.path_valid
        for bid in loser._local_batch.bids
    )


def test_delayed_completion_is_repeated_for_union_churn():
    first, second, _, _ = _ready_pair(ids=(1,))
    winner = first if first._active_goal_id else second
    loser = second if winner is first else first
    winner.on_navigation_outcome(winner._goal_token, _success())
    initial_event = winner._event_publisher.messages[-1]
    changed1 = _array('robot1', (1, 2), map_revision=2)
    changed2 = _array('robot2', (1, 2), map_revision=2)
    _feed_pair(first, second, changed1, changed2)
    current_event = winner._event_publisher.messages[-1]
    assert current_event.canonical_task_id == '1'
    assert current_event.union_hash != initial_event.union_hash
    loser.on_peer_event(current_event)
    _exchange(first, second)
    assert '1' in loser._completed_ids
    assert loser._active_goal_id != '1'


def test_old_completion_event_is_rejected_after_retirement():
    first, second, _, _ = _ready_pair(ids=(1,))
    winner = first if first._active_goal_id else second
    loser = second if winner is first else first
    winner.on_navigation_outcome(winner._goal_token, _success())
    old_event = winner._event_publisher.messages[-1]
    _feed_pair(
        first,
        second,
        _array('robot1', (), map_revision=2),
        _array('robot2', (), map_revision=2),
    )
    assert '1' not in loser._completed_ids
    _feed_pair(
        first,
        second,
        _array('robot1', (1,), map_revision=3),
        _array('robot2', (1,), map_revision=3),
    )
    loser.on_peer_event(old_event)
    assert '1' not in loser._completed_ids


def test_duplicate_completion_event_is_idempotent():
    first, second, _, _ = _ready_pair(ids=(1,))
    winner = first if first._active_goal_id else second
    loser = second if winner is first else first
    winner.on_navigation_outcome(winner._goal_token, _success())
    event = winner._event_publisher.messages[-1]
    loser.on_peer_event(event)
    first_state = set(loser._completed_ids)
    loser.on_peer_event(event)
    assert loser._completed_ids == first_state


def test_navigation_failure_clears_ownership_and_recomputes():
    first, second, _, _ = _ready_pair()
    winner = first if first._active_goal_id else second
    winner.on_navigation_outcome(winner._goal_token, _failure())
    assert winner._active_goal_id is None
    assert winner._state == winner.EVALUATING


def test_stale_navigation_callback_cannot_clear_newer_goal():
    first, second, _, _ = _ready_pair()
    winner = first if first._active_goal_id else second
    old_token = winner._goal_token
    candidate = winner._candidates[winner._robot_id].candidates[-1]
    winner._clear_goal(old_token)
    winner._dispatch(candidate, tuple((p.x, p.y) for p in candidate.local_path_samples))
    newer = winner._active_goal_id
    winner.on_navigation_outcome(old_token, _success())
    assert winner._active_goal_id == newer


def test_reachable_work_does_not_terminate_and_unresolved_work_blocks():
    first, second = _allocator('robot1'), _allocator('robot2')
    _feed_pair(first, second, _array('robot1'), _array('robot2'))
    first._node.clock.seconds = second._node.clock.seconds = 6.0
    first._tick(); second._tick()
    assert first._terminal_reason is None and second._terminal_reason is None
    _feed_pair(first, second, _array('robot1', (), detected=1, planner_failures=1), _array('robot2', ()))
    first._node.clock.seconds = second._node.clock.seconds = 12.0
    first._tick(); second._tick()
    assert first._terminal_reason is None and second._terminal_reason is None


def test_stable_unreachable_and_empty_evidence_follow_existing_terminal_rules():
    clock1, clock2 = _Clock(), _Clock()
    first, second = _allocator('robot1', clock1), _allocator('robot2', clock2)
    unreachable1 = _array('robot1', (), detected=1, unreachable=1)
    unreachable2 = _array('robot2', (), detected=1, unreachable=1)
    _feed_pair(first, second, unreachable1, unreachable2)
    clock1.seconds = clock2.seconds = 6.0; first._tick(); second._tick()
    assert first._terminal_reason == 'MISSION_COMPLETE_NO_REACHABLE_FRONTIERS'
    empty1, empty2 = _array('robot1', ()), _array('robot2', ())
    _feed_pair(first, second, empty1, empty2)
    clock1.seconds = clock2.seconds = 12.0; first._tick(); second._tick()
    assert first._terminal_reason == 'MISSION_COMPLETE_NO_FRONTIERS'


def test_composition_has_no_legacy_lifecycle_surface():
    source = open(composition.__file__, encoding='utf-8').read()
    forbidden_imports = (
        'distributed_frontier_assignment',
        'round_work',
    )
    assert 'from . import minimal_frontier_termination as termination' in source
    assert 'from . import minimal_frontier_traffic as traffic' in source
    assert 'from . import minimal_frontier_protocol as protocol' in source
    for imported_name in forbidden_imports:
        assert f'import {imported_name}' not in source
