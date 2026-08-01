"""Tests for source-local provenance, TTL, rounds, and completion."""

from my_epuck_project.distributed_assignment.models import (
    Bid,
    BidBatch,
    Bounds,
    CompletionInputs,
    CoordinatorState,
    PhysicalTask,
    TaskSnapshot,
)
from my_epuck_project.distributed_assignment.canonical import build_canonical_union
from my_epuck_project.distributed_assignment.scoring import choose_pair_assignment
from my_epuck_project.distributed_assignment.protocol import (
    bid_batch_valid,
    CommittedRound,
    completion_state,
    PeerLiveness,
    receive,
    SnapshotLedger,
)


def snapshot(robot='robot1', session='s1', epoch=1, revision=1):
    """Create a source-consistent single-task snapshot."""
    proposal = PhysicalTask(
        robot, session, epoch, revision, 'sig', 1, (0.0, 0.0),
        Bounds((-0.1, -0.1), (0.1, 0.1)), (0.0, -0.2),
    )
    return TaskSnapshot(robot, session, epoch, revision, 'map', 100, 2.0, (proposal,))


def bid_batch(robot='robot1', session='s1', epoch=1):
    """Create one valid round-bound bid batch."""
    return BidBatch(
        'round', 'union', robot, session, epoch, 2.0,
        (Bid('task', True, 1.0, 1.0),),
    )


def test_stale_epoch_and_duplicate_rejected():
    """Reject duplicate and older source-local epochs."""
    ledger = SnapshotLedger()
    assert ledger.accept(snapshot(epoch=2, revision=4))
    assert not ledger.accept(snapshot(epoch=2, revision=4))
    assert not ledger.accept(snapshot(epoch=1, revision=5))


def test_source_session_restart_resets_revision_interpretation():
    """A new source session starts a fresh revision sequence."""
    ledger = SnapshotLedger()
    assert ledger.accept(snapshot(session='old', epoch=8, revision=30))
    assert ledger.accept(snapshot(session='new', epoch=1, revision=1))


def test_delayed_snapshot_from_retired_session_is_rejected():
    """A delayed old session cannot masquerade as a second source restart."""
    ledger = SnapshotLedger()
    assert ledger.accept(snapshot(session='old', epoch=8, revision=30))
    assert ledger.accept(snapshot(session='new', epoch=1, revision=1))
    assert not ledger.accept(snapshot(session='old', epoch=9, revision=31))


def test_same_source_revision_regression_rejected():
    """Reject a lower map revision within one source session."""
    ledger = SnapshotLedger()
    assert ledger.accept(snapshot(epoch=1, revision=8))
    assert not ledger.accept(snapshot(epoch=2, revision=7))


def test_cross_source_revision_comparison_is_invalid():
    """Never order independent robots by numeric map revision."""
    ledger = SnapshotLedger()
    try:
        ledger.compare_revisions('robot1', 'robot2')
    except ValueError:
        pass
    else:
        raise AssertionError('numeric revisions from different sources are incomparable')


def test_ttl_uses_receiver_local_steady_time_and_clamps_sender_value():
    """Expire from receipt time without comparing sender clocks."""
    item = receive('payload', sender_ttl_s=999.0, now_steady_s=50.0)
    assert item.fresh(59.9)
    assert not item.fresh(60.1)
    assert not item.fresh(49.0)


def test_bid_must_bind_round_union_identity_epoch_and_fresh_peer():
    """Reject stale or incompletely bound bid arrays."""
    received = receive(bid_batch(), 2.0, 10.0)
    assert bid_batch_valid(
        received, 11.0, 'robot1', 's1', 1, 'round', 'union', True,
    )
    assert not bid_batch_valid(
        received, 13.0, 'robot1', 's1', 1, 'round', 'union', True,
    )
    assert not bid_batch_valid(
        received, 11.0, 'robot1', 's1', 1, 'old-round', 'union', True,
    )
    assert not bid_batch_valid(
        received, 11.0, 'robot1', 's1', 1, 'round', 'union', False,
    )


def test_useful_partial_assignment_beats_idle_when_peer_has_no_valid_bid():
    """One robot may continue useful work when the peer has no reachable bid."""
    first = PhysicalTask(
        'robot1', 's1', 1, 1, 'north', 1, (0.0, 1.0),
        Bounds((-0.1, 0.9), (0.1, 1.1)), (0.0, 0.8),
        visible_reveal_gain=4.0,
    )
    second = PhysicalTask(
        'robot2', 's2', 1, 1, 'east', 2, (2.0, 0.0),
        Bounds((1.9, -0.1), (2.1, 0.1)), (2.0, 0.0),
        visible_reveal_gain=4.0,
    )
    union = build_canonical_union((first,), (second,))
    selected = choose_pair_assignment(
        'round', union,
        BidBatch('round', union.union_hash, 'robot1', 's1', 1, 5.0, (
            Bid(union.tasks[0].canonical_id, True, 0.8, 0.8),
        )),
        BidBatch('round', union.union_hash, 'robot2', 's2', 1, 5.0, (
            Bid(union.tasks[1].canonical_id, False, 0.0, 0.0),
        )),
    )
    assert selected.robot1_task_id == union.tasks[0].canonical_id
    assert selected.robot2_task_id == ''
    assert selected.diagnostics.availability_reason == 'ONLY_ROBOT1_REACHABLE'


def test_idle_diagnostics_identify_no_feasible_task():
    """IDLE is explainable when the only proposal has zero visible gain."""
    first = PhysicalTask(
        'robot1', 's1', 1, 1, 'north', 1, (0.0, 1.0),
        Bounds((-0.1, 0.9), (0.1, 1.1)), (0.0, 1.0),
        visible_reveal_gain=0.0,
    )
    union = build_canonical_union((first,), ())
    bid = Bid(union.tasks[0].canonical_id, True, 1.0, 1.0)
    selected = choose_pair_assignment(
        'round', union,
        BidBatch('round', union.union_hash, 'robot1', 's1', 1, 5.0, (bid,)),
        BidBatch('round', union.union_hash, 'robot2', 's2', 1, 5.0, ()),
    )
    assert selected.robot1_task_id == ''
    assert selected.robot2_task_id == ''
    assert selected.diagnostics.idle_reason == 'BELOW_GAIN_THRESHOLD'


def test_delayed_old_message_cannot_cancel_committed_navigation():
    """Old rounds do not alter committed navigation."""
    guard = CommittedRound()
    decision = type('Decision', (), {'round_id': 'committed'})()
    guard.commit(decision)
    assert not guard.delayed_message_can_cancel('older')
    assert not guard.delayed_message_can_cancel('committed')
    assert guard.explicit_reauction_allowed('GOAL_SUCCEEDED')
    assert not guard.explicit_reauction_allowed('NEWER_SNAPSHOT_ONLY')


def test_peer_timeout_enters_solo_and_requires_fresh_session_handshake():
    """Recover from degraded solo only through a new peer session."""
    liveness = PeerLiveness(timeout_s=3.0)
    liveness.observe('peer-old', 10.0)
    assert liveness.evaluate(14.0) == CoordinatorState.DEGRADED_SOLO
    assert liveness.observe('peer-old', 15.0) == CoordinatorState.DEGRADED_SOLO
    assert liveness.observe('peer-new', 16.0) == CoordinatorState.WAITING_FOR_INPUTS


def test_peer_status_heartbeat_prevents_snapshot_expiry_from_being_peer_loss():
    """A live navigating peer can refresh liveness without new proposals."""
    liveness = PeerLiveness(timeout_s=3.0)
    liveness.observe('peer-session', 10.0)
    liveness.observe('peer-session', 12.0)
    assert liveness.evaluate(14.5) == CoordinatorState.WAITING_FOR_INPUTS


def complete_inputs(**changes):
    """Create a healthy operational completion fixture."""
    values = {
        'both_snapshots_fresh': True, 'both_statuses_fresh': True,
        'robot1_has_valid_task': False, 'robot2_has_valid_task': False,
        'valid_pair_exists': False, 'shared_maps_stable': True,
        'active_assignment': False, 'local_nav_goal_active': False,
        'peer_nav_goal_active': False, 'tf_healthy': True,
        'nav2_healthy': True, 'candidates_healthy': True,
        'communication_healthy': True, 'peer_completion_matches': True,
        'condition_duration_s': 5.0, 'confirmation_interval_s': 4.0,
    }
    values.update(changes)
    return CompletionInputs(**values)


def test_operational_completion_requires_health_and_persistence():
    """Do not report completion during degradation or transient exhaustion."""
    assert completion_state(complete_inputs())[0] == CoordinatorState.COMPLETE
    assert completion_state(
        complete_inputs(nav2_healthy=False),
    )[0] == CoordinatorState.BLOCKED
    assert completion_state(
        complete_inputs(condition_duration_s=1.0),
    )[0] != CoordinatorState.COMPLETE
    assert completion_state(
        complete_inputs(robot2_has_valid_task=True),
    )[0] != CoordinatorState.COMPLETE
