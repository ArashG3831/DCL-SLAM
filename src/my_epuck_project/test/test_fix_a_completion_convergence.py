"""Focused regressions for Fix A completion convergence and mismatch recovery."""

import json
from dataclasses import replace
from types import SimpleNamespace

from my_epuck_project.distributed_assignment.canonical import build_canonical_union
from my_epuck_project.distributed_assignment.models import (
    AssignmentDiagnostics,
    AssignmentScore,
    Bid,
    BidBatch,
    Bounds,
    PairDecision,
    PhysicalTask,
    TaskSnapshot,
)
from my_epuck_project.distributed_assignment.protocol import receive
from my_epuck_project.distributed_assignment.ros_conversion import text_to_uuid
from my_epuck_project.distributed_assignment.scoring import choose_pair_assignment
from my_epuck_project.distributed_frontier_assignment import (
    ActiveCommitment,
    ActiveNavigationAction,
    DistributedFrontierAssignment,
    RoundWork,
    normal_round_requires_replacement,
    selector_feasibility_identity,
)
from my_epuck_project.round_lifecycle import RoundGeneration


R1_SESSION = '11' * 16
R2_SESSION = '22' * 16


def _task(robot, signature, x, local_id=1):
    session = R1_SESSION if robot == 'robot1' else R2_SESSION
    return PhysicalTask(
        robot, session, 1, 1, signature, local_id,
        (x, 0.0), Bounds((x - 0.1, -0.1), (x + 0.1, 0.1)),
        (x, 0.0), frontier_geometry=((x, 0.0),),
        visible_reveal_gain=1.0, local_path_valid=True,
        local_path_length_m=abs(x), local_path=((0.0, 0.0), (x, 0.0)),
    )


def _snapshot(robot, task, epoch=1):
    session = R1_SESSION if robot == 'robot1' else R2_SESSION
    return TaskSnapshot(
        robot, session, epoch, epoch, 'map-%s-%d' % (robot, epoch),
        100 + epoch, 10.0, (task,),
    )


def _batch(robot, union, entries):
    session = R1_SESSION if robot == 'robot1' else R2_SESSION
    return BidBatch(
        'round', union.union_hash, robot, session, 1, 10.0,
        tuple(Bid(task_id, True, length, length, path=path)
              for task_id, length, path in entries),
    )


def test_selector_feasibility_identity_is_order_independent_and_complete():
    first = selector_feasibility_identity(
        ('a', 'b', 'c'), {'b'}, {'a'}, {'c'},
    )
    second = selector_feasibility_identity(
        ('c', 'b', 'a'), {'b'}, {'a'}, {'c'},
    )
    changed = selector_feasibility_identity(
        ('a', 'b', 'c'), {'b'}, {'a', 'c'}, set(),
    )

    assert first == second
    assert first[0] != changed[0]
    assert changed[1]['completed'] == ('a', 'c')
    assert changed[1]['peer_reservations'] == ()


def test_observed_suppression_difference_changes_pair_selector_input():
    r1_a = _task('robot1', 'r1-a', 1.0)
    r1_b = _task('robot1', 'r1-b', 4.0, local_id=2)
    r2_a = _task('robot2', 'r2-a', 2.0)
    r2_b = _task('robot2', 'r2-b', 5.0, local_id=2)
    union = build_canonical_union((r1_a, r1_b), (r2_a, r2_b))
    ids = {
        member.physical_signature: task.canonical_id
        for task in union.tasks for member in task.members
    }
    first = _batch('robot1', union, [
        (ids['r1-a'], 1.0, ((0.0, 0.0), (1.0, 0.0))),
        (ids['r1-b'], 4.0, ((0.0, 0.0), (4.0, 0.0))),
        (ids['r2-a'], 1.0, ((0.0, 0.0), (1.0, 0.0))),
        (ids['r2-b'], 4.0, ((0.0, 0.0), (4.0, 0.0))),
    ])
    second = _batch('robot2', union, [
        (ids['r1-a'], 1.0, ((0.0, 0.0), (1.0, 0.0))),
        (ids['r1-b'], 4.0, ((0.0, 0.0), (4.0, 0.0))),
        (ids['r2-a'], 1.0, ((0.0, 0.0), (1.0, 0.0))),
        (ids['r2-b'], 4.0, ((0.0, 0.0), (4.0, 0.0))),
    ])
    selected = choose_pair_assignment(
        'round', union, first, second, hard_failed_tasks=frozenset(),
        scoring_mode='frontier_cost_only',
    )
    reordered = choose_pair_assignment(
        'round', union,
        replace(first, bids=tuple(reversed(first.bids))),
        replace(second, bids=tuple(reversed(second.bids))),
        hard_failed_tasks=frozenset(), scoring_mode='frontier_cost_only',
    )
    suppressed = choose_pair_assignment(
        'round', union, first, second,
        hard_failed_tasks=frozenset({ids['r1-a']}),
        scoring_mode='frontier_cost_only',
    )

    assert selected.robot1_task_id != suppressed.robot1_task_id or \
        selected.robot2_task_id != suppressed.robot2_task_id
    assert (selected.robot1_task_id, selected.robot2_task_id,
            selected.decision_hash) == (
        reordered.robot1_task_id, reordered.robot2_task_id,
        reordered.decision_hash,
    )


def _mismatch_fixture():
    r1 = _task('robot1', 'r1', 1.0)
    r2 = _task('robot2', 'r2', 2.0)
    union = build_canonical_union((r1,), (r2,))
    snapshots = (_snapshot('robot1', r1), _snapshot('robot2', r2))
    diagnostics = AssignmentDiagnostics(
        selector_feasibility_fingerprint='selector-a',
    )
    local = PairDecision(
        'round', union.union_hash, union.tasks[0].canonical_id, '',
        'r1-bids', 'r2-bids', AssignmentScore(), 'local-hash',
        1.0, 1.0, diagnostics,
    )
    peer = SimpleNamespace(
        source_session_id=text_to_uuid(R2_SESSION),
        round_id='round', union_hash=union.union_hash,
        robot1_snapshot_epoch=1, robot2_snapshot_epoch=1,
        robot1_bid_fingerprint='r1-bids',
        robot2_bid_fingerprint='r2-bids',
        robot1_canonical_task_id='',
        robot2_canonical_task_id=union.tasks[0].canonical_id,
        decision_hash='peer-hash',
        diagnostics_json=json.dumps({
            'selector_feasibility_fingerprint': 'selector-b',
        }),
    )
    round_work = RoundWork(
        'round', union, snapshots, union.tasks, 'semantic', decision=local,
    )
    node = DistributedFrontierAssignment.__new__(DistributedFrontierAssignment)
    node._robot_id = 'robot1'
    node._peer_id = 'robot2'
    node._peer_decision = receive(peer, 10.0, 0.0)
    node._round = round_work
    node._snapshots = {
        'robot1': receive(snapshots[0], 10.0, 0.0),
        'robot2': receive(snapshots[1], 10.0, 0.0),
    }
    node._round_lifecycle = RoundGeneration()
    node._round_lifecycle.activate('round')
    node._active_navigation_action = object()
    node._events = []
    node._reset_reasons = []
    node._reset_round = lambda reason: node._reset_reasons.append(reason)
    node._emit_event = lambda *args, **kwargs: node._events.append((args, kwargs))
    node.get_logger = lambda: SimpleNamespace(warning=lambda *a, **k: None)
    return node, round_work, snapshots


def test_matching_peer_decision_requires_selector_identity():
    node, round_work, _ = _mismatch_fixture()
    peer = node._peer_decision.value
    peer.robot1_canonical_task_id = round_work.decision.robot1_task_id
    peer.robot2_canonical_task_id = round_work.decision.robot2_task_id
    peer.decision_hash = round_work.decision.decision_hash
    peer.diagnostics_json = json.dumps({
        'selector_feasibility_fingerprint': 'selector-a',
    })
    assert node._matching_peer_decision(round_work, 1, 0.1) is True
    peer.diagnostics_json = json.dumps({
        'selector_feasibility_fingerprint': 'selector-other',
    })
    assert node._matching_peer_decision(round_work, 1, 0.1) is False


def test_selector_state_divergence_is_fail_closed_without_round_reset():
    node, round_work, _ = _mismatch_fixture()
    assert node._matching_peer_decision(round_work, 1, 0.1) is False
    assert node._reset_reasons == []
    assert node._active_navigation_action is not None
    assert any(
        args and args[0] == 'SELECTOR_FEASIBILITY_DIVERGENCE'
        for args, _ in node._events
    )


def test_equal_selector_inputs_with_different_outputs_emit_invariant_violation():
    node, round_work, _ = _mismatch_fixture()
    peer = node._peer_decision.value
    peer.diagnostics_json = json.dumps({
        'selector_feasibility_fingerprint': 'selector-a',
    })
    assert node._matching_peer_decision(round_work, 1, 0.1) is False
    assert node._reset_reasons == []
    assert not hasattr(node, '_decision_mismatch_retry_token')
    assert any(
        args and args[0] == 'PAIR_DECISION_INVARIANT_VIOLATION'
        for args, _ in node._events
    )


def test_selector_state_change_invalidates_uncommitted_decision():
    node, round_work, _ = _mismatch_fixture()
    node._committed = SimpleNamespace(decision=None)
    node._peer_decision = receive(
        node._peer_decision.value, 10.0, 0.0,
    )
    selector_inputs = {
        'completed': ('completed-task',),
        'hard_failed': (),
        'peer_reservations': (),
    }
    assert node._selector_decision_requires_recompute(
        round_work, 'selector-new', selector_inputs,
    ) is True
    assert round_work.decision is None
    assert node._peer_decision is None
    assert any(
        args and args[0] == 'SELECTOR_FEASIBILITY_CHANGED_RECOMPUTE'
        for args, _ in node._events
    )


def test_epoch_only_update_does_not_require_semantic_round_replacement():
    node, round_work, snapshots = _mismatch_fixture()
    fresh = (
        _snapshot('robot1', snapshots[0].tasks[0], 2),
        _snapshot('robot2', snapshots[1].tasks[0], 2),
    )
    assert normal_round_requires_replacement(
        round_work, fresh[0], fresh[1], round_work.content_fingerprint,
    ) is False


def test_observed_free_robot_continuation_completion_converges_across_wrappers():
    """Reproduce the 5a6... topology: R1 busy, R2 newly assigned and free."""
    busy = _task('robot1', 'busy-task', 3.0)
    free = _task(
        'robot2', '5a6b551622c0e565ca3011e4', 2.0,
    )
    union = build_canonical_union((busy,), (free,))
    tasks = {task.canonical_id: task for task in union.tasks}
    task_by_signature = {
        member.physical_signature: task
        for task in union.tasks for member in task.members
    }
    busy_id = task_by_signature[busy.physical_signature].canonical_id
    free_id = task_by_signature[free.physical_signature].canonical_id
    snapshots = (_snapshot('robot1', busy), _snapshot('robot2', free))
    busy_commitment = ActiveCommitment(
        robot_id='robot1',
        source_session_id=R1_SESSION,
        source_snapshot_epoch=1,
        canonical_id=busy_id,
        task=tasks[busy_id],
        decision_round_id='original-agreement-round',
        decision_hash='original-agreement-decision',
        path=((0.0, 0.0), (3.0, 0.0)),
        path_length_m=3.0,
        commitment_id='immutable-busy-lineage',
    )

    def make_round(round_id, decision_hash):
        decision = PairDecision(
            round_id, union.union_hash, busy_id, free_id,
            'robot1-bids', 'robot2-bids', AssignmentScore(), decision_hash,
            2.0, 3.0, AssignmentDiagnostics(),
        )
        return RoundWork(
            round_id, union, snapshots, union.tasks, 'continuation-semantic',
            mode='continuation', continuation_free_robot_id='robot2',
            continuation_busy_robot_id='robot1',
            continuation_commitment_id=busy_commitment.commitment_id,
            decision=decision,
        )

    def make_node(round_work):
        node = DistributedFrontierAssignment.__new__(DistributedFrontierAssignment)
        node._active_commitments = {'robot1': busy_commitment}
        node._bid_batches = {
            'robot1': receive(
                replace(_batch('robot1', union, [
                    (busy_id, 3.0, ((0.0, 0.0), (3.0, 0.0))),
                    (free_id, 2.0, ((0.0, 0.0), (2.0, 0.0))),
                ]), round_id=round_work.round_id), 10.0, 0.0,
            ),
            'robot2': receive(
                replace(_batch('robot2', union, [
                    (busy_id, 3.0, ((0.0, 0.0), (3.0, 0.0))),
                    (free_id, 2.0, ((0.0, 0.0), (2.0, 0.0))),
                ]), round_id=round_work.round_id), 10.0, 0.0,
            ),
        }
        node._round = round_work
        node._robot_id = 'robot1'
        node._peer_id = 'robot2'
        node._completed_shared_canonical_ids = set()
        node._clear_active_commitment = lambda robot_id, reason: (
            node._active_commitments.pop(robot_id, None))
        node.get_logger = lambda: SimpleNamespace(
            warning=lambda *a, **k: None,
            info=lambda *a, **k: None,
        )
        return node

    # The two local continuation wrappers intentionally differ, as in the
    # observed runtime.  The physical assignment is otherwise identical.
    robot1_round = make_round('continuation-wrapper-r1', 'decision-wrapper-r1')
    robot2_round = make_round('continuation-wrapper-r2', 'decision-wrapper-r2')
    robot1 = make_node(robot1_round)
    robot2 = make_node(robot2_round)
    robot1._remember_active_commitments(robot1_round)
    robot2._remember_active_commitments(robot2_round)

    robot1_free = robot1._active_commitments['robot2']
    robot2_free = robot2._active_commitments['robot2']
    assert robot1_free.commitment_id == robot2_free.commitment_id
    assert robot1_free.decision_round_id == robot2_free.decision_round_id
    assert robot1_free.decision_hash == robot2_free.decision_hash

    robot2_action = ActiveNavigationAction(
        'robot2:1:%s' % free_id,
        tasks[free_id], free_id, free.physical_signature,
        robot2_free.decision_round_id, robot2_free.decision_hash, 1,
        commitment_id=robot2_free.commitment_id,
    )
    assert robot2_action.commitment_id == robot1_free.commitment_id

    robot2._completed_shared_canonical_ids.add(free_id)
    terminal = SimpleNamespace(
        source_robot_id='robot2',
        event_type='NAVIGATION_SUCCEEDED',
        source_session_id=text_to_uuid(R2_SESSION),
        canonical_task_id=free_id,
        physical_task_signature=free.physical_signature,
        round_id=robot2_action.round_id,
        decision_hash=robot2_action.decision_hash,
    )
    robot1._peer_event_callback(terminal)

    assert free_id in robot1._completed_shared_canonical_ids
    assert robot1._completed_shared_canonical_ids == \
        robot2._completed_shared_canonical_ids


def test_asynchronous_completion_state_converges_without_retry_protocol():
    """A state update changes fingerprints, then convergence permits recompute."""
    node, round_work, snapshots = _mismatch_fixture()
    task_ids = tuple(task.canonical_id for task in round_work.union.tasks)
    before, _ = selector_feasibility_identity(
        task_ids, set(), {task_ids[0]}, set(),
    )
    after, inputs = selector_feasibility_identity(
        task_ids, set(), set(), set(),
    )
    assert before != after
    assert inputs['completed'] == ()
    node._committed = SimpleNamespace(decision=None)
    node._peer_decision = None
    round_work.decision = replace(
        round_work.decision,
        diagnostics=replace(
            round_work.decision.diagnostics,
            selector_feasibility_fingerprint=before,
        ),
    )
    assert node._selector_decision_requires_recompute(
        round_work, after, inputs,
    ) is True

    # The underlying completion has now converged; a fresh deterministic
    # selector input can be installed without a retry token or round reset.
    round_work.decision = PairDecision(
        round_work.round_id, round_work.union.union_hash, '', '',
        'r1-bids', 'r2-bids', AssignmentScore(), 'new-decision',
        0.0, 0.0,
        replace(
            AssignmentDiagnostics(),
            selector_feasibility_fingerprint=after,
        ),
    )
    assert node._selector_decision_requires_recompute(
        round_work, after, inputs,
    ) is False
    assert not hasattr(node, '_decision_mismatch_retry_token')
