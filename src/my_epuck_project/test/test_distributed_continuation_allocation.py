"""Behavioral tests for one-busy/one-free continuation allocation."""

from dataclasses import replace
from types import SimpleNamespace

from my_epuck_interfaces.msg import DistributedExplorationStatus

from my_epuck_project.distributed_assignment.canonical import (
    build_canonical_union,
)
from my_epuck_project.distributed_assignment.models import (
    Bid,
    BidBatch,
    Bounds,
    CoordinatorState,
    PhysicalTask,
    TaskSnapshot,
)
from my_epuck_project.distributed_assignment.protocol import receive
from my_epuck_project.distributed_assignment.scoring import choose_pair_assignment
from my_epuck_project.distributed_assignment.traffic_scheduler import schedule_traffic
from my_epuck_project.distributed_assignment.traffic_scheduler import TrafficDecision
from my_epuck_project.distributed_frontier_assignment import (
    ActiveCommitment,
    DistributedFrontierAssignment,
)
from my_epuck_project.distributed_assignment.ros_conversion import text_to_uuid
from my_epuck_project.round_lifecycle import RoundGeneration


NOW = 100.0
R1_SESSION = '11' * 16
R2_SESSION = '22' * 16


def _physical(robot, session, signature, x, route_rank=0):
    """Create a source-consistent, non-equivalent frontier proposal."""
    return PhysicalTask(
        robot, session, 1, 1, signature, 1,
        (x, 0.0), Bounds((x - 0.1, -0.1), (x + 0.1, 0.1)),
        (x, 0.0), frontier_geometry=((x, 0.0),),
        visible_reveal_gain=1.0, local_ordering_score=1.0,
        mrtsp_route_rank=route_rank, mrtsp_route_generation=1,
        mrtsp_solver='dp', local_path_valid=True, local_path_length_m=abs(x),
        local_path=((0.0, 0.0), (x, 0.0)), generation_ros_ns=100,
    )


def _snapshot(task, session, robot):
    return TaskSnapshot(
        robot, session, 1, 1, 'map-%s' % robot, 100, 10.0, (task,),
    )


def _fake_node(local_robot='robot1', local_active=False, free_x=-2.0,
               busy_x=2.0, peer_status_fresh=True):
    """Build a ROS-free node shell sufficient for continuation helpers."""
    free_robot = local_robot if not local_active else (
        'robot2' if local_robot == 'robot1' else 'robot1')
    busy_robot = 'robot2' if free_robot == 'robot1' else 'robot1'
    free_session = R1_SESSION if free_robot == 'robot1' else R2_SESSION
    busy_session = R2_SESSION if busy_robot == 'robot2' else R1_SESSION
    free_task = _physical(free_robot, free_session, 'free-task', free_x)
    busy_task = _physical(busy_robot, busy_session, 'busy-task', busy_x)
    free_source = _snapshot(free_task, free_session, free_robot)
    busy_union = build_canonical_union(
        (busy_task,) if busy_robot == 'robot1' else (),
        (busy_task,) if busy_robot == 'robot2' else (),
    )
    busy_canonical = busy_union.tasks[0]
    commitment = ActiveCommitment(
        robot_id=busy_robot,
        source_session_id=busy_session,
        source_snapshot_epoch=1,
        canonical_id=busy_canonical.canonical_id,
        task=busy_canonical,
        decision_round_id='normal-round',
        decision_hash='normal-decision',
        path=tuple(busy_task.local_path),
        path_length_m=busy_task.local_path_length_m,
        commitment_id='commitment-%s' % busy_robot,
    )
    node = DistributedFrontierAssignment.__new__(DistributedFrontierAssignment)
    node._robot_id = local_robot
    node._peer_id = 'robot2' if local_robot == 'robot1' else 'robot1'
    peer_robot = node._peer_id
    peer_active = not local_active
    peer_session = R1_SESSION if peer_robot == 'robot1' else R2_SESSION
    peer_status = SimpleNamespace(
        local_nav_goal_active=peer_active,
        state=(DistributedExplorationStatus.NAVIGATING if peer_active else
               DistributedExplorationStatus.WAITING_FOR_INPUTS),
        active_canonical_task_id=(
            busy_canonical.canonical_id if peer_active else ''),
        source_session_id=text_to_uuid(peer_session),
    )
    node._nav2 = SimpleNamespace(local_goal_active=local_active)
    node._state = (CoordinatorState.NAVIGATING if local_active else
                   CoordinatorState.WAITING_FOR_INPUTS)
    node._active_task = commitment.task if local_active else None
    node._active_commitments = {busy_robot: commitment}
    node._snapshots = {free_robot: receive(free_source, 10.0, NOW)}
    node._peer_status = receive(
        peer_status, 10.0, NOW if peer_status_fresh else NOW - 20.0,
    )
    node._maximum_union_tasks = 10
    node._bid_validity_s = 10.0
    node._round = None
    node._round_lifecycle = RoundGeneration()
    node._round_created_count = 0
    node._round_replaced_count = 0
    node._bid_batches = {}
    node._peer_decision = None
    node._committed = SimpleNamespace()
    node._transition_log = []
    node._events = []
    node._log_round_lifecycle = lambda *args, **kwargs: None
    node._log_union = lambda *args, **kwargs: None
    node._transition = lambda state, reason: node._transition_log.append(
        (state, reason))
    node._emit_event = lambda *args, **kwargs: node._events.append(args)
    return node, commitment, free_source, free_task, busy_canonical


def _install_free_bid(node, free_task, round_work):
    """Install the free robot's one fresh exchanged bid for the round."""
    member = free_task.members[0]
    bid = Bid(
        free_task.canonical_id, True, member.local_path_length_m,
        member.local_path_length_m, path=tuple(member.local_path),
    )
    batch = BidBatch(
        round_work.round_id, round_work.union.union_hash,
        round_work.continuation_free_robot_id,
        R1_SESSION if round_work.continuation_free_robot_id == 'robot1'
        else R2_SESSION,
        1, 10.0, (bid,),
    )
    node._bid_batches[round_work.continuation_free_robot_id] = receive(
        batch, 10.0, NOW,
    )


def _continuation_setup(local_robot='robot1', local_active=False,
                        peer_status_fresh=True):
    node, commitment, source, free_task, busy_task = _fake_node(
        local_robot, local_active, peer_status_fresh=peer_status_fresh,
    )
    context = node._continuation_context(NOW)
    assert context is not None
    assert context.free_robot_id == (node._peer_id if local_active else local_robot)
    assert context.busy_robot_id == (local_robot if local_active else node._peer_id)
    assert node._activate_continuation_round(context)
    assert node._round is not None
    _install_free_bid(node, context.free_tasks[0], node._round)
    batches = node._continuation_bid_batches(NOW, node._round)
    assert batches is not None
    return node, context, batches


def _select_continuation(node, context, batches):
    """Run the existing pair solver with only the busy task fixed."""
    fixed = {
        ('fixed_robot1_task_id' if context.busy_robot_id == 'robot1'
         else 'fixed_robot2_task_id'): context.commitment.canonical_id,
    }
    return choose_pair_assignment(
        node._round.round_id, node._round.union, batches[0], batches[1],
        **fixed,
    )


def test_free_robot_gets_continuation_assignment_while_peer_stays_busy():
    node, context, batches = _continuation_setup()
    decision = _select_continuation(node, context, batches)
    assert decision.robot1_task_id == context.free_tasks[0].canonical_id
    assert decision.robot2_task_id == context.commitment.canonical_id
    assert node._round.mode == 'continuation'


def test_continuation_is_symmetric_when_robot2_is_free():
    node, context, batches = _continuation_setup(
        local_robot='robot2', local_active=False,
    )
    decision = _select_continuation(node, context, batches)
    assert decision.robot2_task_id == context.free_tasks[0].canonical_id
    assert decision.robot1_task_id == context.commitment.canonical_id


def test_busy_peer_candidate_snapshot_is_not_required_when_status_is_fresh():
    node, context, _ = _continuation_setup()
    busy_id = context.busy_robot_id
    assert busy_id not in node._snapshots
    assert node._continuation_context(NOW) is not None


def test_active_peer_without_task_identity_keeps_continuation_commitment():
    node, _, _, _, _ = _fake_node()
    context = node._continuation_context(NOW)
    assert context is not None
    node._peer_status.value.active_canonical_task_id = ''
    assert context.busy_robot_id not in node._snapshots
    assert node._continuation_context(NOW) is not None


def test_both_free_stale_peer_snapshot_does_not_form_pair_inputs():
    node, _, free_source, free_task, _ = _fake_node()
    node._active_commitments = {}
    node._peer_status.value.local_nav_goal_active = False
    node._peer_status.value.state = DistributedExplorationStatus.WAITING_FOR_INPUTS
    node._snapshots['robot2'] = receive(
        _snapshot(_physical('robot2', R2_SESSION, 'peer-task', 2.0),
                  R2_SESSION, 'robot2'),
        10.0, NOW - 20.0,
    )
    assert node._continuation_context(NOW) is None
    assert node._fresh_snapshot('robot1', NOW) is not None
    assert node._fresh_snapshot('robot2', NOW) is None


def test_busy_continuation_requires_fresh_free_snapshot():
    node, _, _, _, _ = _fake_node()
    context = node._continuation_context(NOW)
    assert context is not None
    free_robot = context.free_robot_id
    free_task = context.free_snapshot.tasks[0]
    node._snapshots[free_robot] = receive(
        _snapshot(free_task, free_task.source_session_id, free_robot),
        10.0, NOW - 20.0,
    )
    assert node._continuation_context(NOW) is None


def test_busy_peer_staleness_blocks_continuation_but_not_freshness_bypass():
    node, _, _, _, _ = _fake_node(peer_status_fresh=False)
    assert node._continuation_context(NOW) is None


def test_busy_peer_session_change_invalidates_old_commitment():
    node, _, _, _, _ = _fake_node()
    node._peer_status.value.source_session_id = text_to_uuid('33' * 16)
    assert node._continuation_context(NOW) is None


def test_both_busy_does_not_enter_continuation_mode():
    node, _, _, _, _ = _fake_node()
    node._nav2.local_goal_active = True
    node._peer_status.value.local_nav_goal_active = True
    node._peer_status.value.state = DistributedExplorationStatus.NAVIGATING
    assert node._continuation_context(NOW) is None


def test_duplicate_busy_commitment_is_removed_from_free_candidates():
    node, _, free_source, _, _ = _fake_node()
    duplicate = _physical('robot1', R1_SESSION, 'duplicate', 2.0)
    node._snapshots['robot1'] = receive(
        _snapshot(duplicate, R1_SESSION, 'robot1'), 10.0, NOW,
    )
    context = node._continuation_context(NOW)
    assert context is not None
    assert not context.free_tasks


def test_distinct_continuation_task_still_passes_existing_traffic_scheduler():
    node, context, batches = _continuation_setup()
    decision = _select_continuation(node, context, batches)
    busy_bid = next(
        bid for bid in batches[1].bids
        if bid.canonical_task_id == context.commitment.canonical_id
    )
    free_bid = next(
        bid for bid in batches[0].bids
        if bid.canonical_task_id == context.free_tasks[0].canonical_id
    )
    traffic = schedule_traffic(
        free_bid.path, busy_bid.path,
        robot1_safe_radius_m=0.08, robot2_safe_radius_m=0.08,
        reference_speed_mps=0.13, eta_tie_s=0.25,
    )
    assert decision.robot1_task_id == context.free_tasks[0].canonical_id
    assert traffic.conflict is True
    assert traffic.waiting_robot_id in ('robot1', 'robot2')


def test_continuation_traffic_hold_retains_busy_commitment_path():
    node, context, batches = _continuation_setup()
    decision = _select_continuation(node, context, batches)
    node._round.decision = decision
    node._traffic_hold = None
    node._traffic_reallocation_after_clear = False
    node._released_traffic_winner_robot_id = ''
    node._traffic_conflict_clearance_m = 0.05
    node._begin_traffic_wait(TrafficDecision(
        conflict=True,
        minimum_separation_m=0.0,
        required_separation_m=0.16,
        robot1_last_conflict_distance_m=0.5,
        robot2_last_conflict_distance_m=0.5,
        winner_robot_id=context.commitment.robot_id,
        waiting_robot_id=context.free_robot_id,
        reason='ACTIVE_COMMITMENT_PRIORITY',
    ))
    assert node._traffic_hold is not None
    assert node._traffic_hold.winner_path == context.commitment.path


def test_continuation_round_id_ignores_busy_candidate_epoch_changes():
    node, _, _, _, _ = _fake_node()
    context = node._continuation_context(NOW)
    assert context is not None
    first = node._continuation_round_id(context, 'content')
    changed = context.commitment.task
    same_commitment = context.commitment.__class__(
        **{**context.commitment.__dict__, 'task': changed},
    )
    second_context = context.__class__(
        context.free_robot_id, context.busy_robot_id, context.free_snapshot,
        same_commitment, context.free_tasks,
    )
    assert node._continuation_round_id(second_context, 'content') == first


def test_continuation_free_epoch_update_keeps_active_round_generation():
    node, _, _, _, _ = _fake_node()
    context = node._continuation_context(NOW)
    assert context is not None
    assert node._activate_continuation_round(context)
    active_round = node._round
    created = node._round_created_count
    replaced = node._round_replaced_count

    refreshed_free = replace(
        context.free_snapshot,
        epoch=context.free_snapshot.epoch + 1,
        map_revision=context.free_snapshot.map_revision + 1,
        map_fingerprint='fresh-map',
        lower_bound_context_fingerprint='fresh-bounds',
        candidate_generation_id=2,
    )
    node._snapshots[context.free_robot_id] = receive(
        refreshed_free, 10.0, NOW,
    )
    refreshed_context = node._continuation_context(NOW)
    assert refreshed_context is not None

    assert node._activate_continuation_round(refreshed_context)
    assert node._round is active_round
    assert node._round_created_count == created
    assert node._round_replaced_count == replaced


def test_canonical_continuation_canonical_oscillation_keeps_round_on_stale_context():
    """Temporary context loss must not clear continuation auction evidence."""
    node, context, _ = _continuation_setup()
    active_round = node._round
    created = node._round_created_count
    replaced = node._round_replaced_count

    # The peer heartbeat briefly expires.  The old tick path treated this as
    # a hard invalidation, allowing the next tick to form a canonical round.
    node._peer_status = receive(
        node._peer_status.value, 10.0, NOW - 20.0,
    )
    assert node._continuation_context(NOW) is None
    assert not node._continuation_round_requires_invalidation(NOW)

    # Exercise the exact tick gate that used to reset the continuation round.
    node._terminal = False
    node._last_tick_log_key = None
    node._local_only = False
    node._handoff_complete = False
    node._mission_timeout_enabled = False
    node._traffic_hold = None
    node._traffic_reallocation_after_clear = False
    node._released_traffic_winner_robot_id = ''
    node._settle_until_steady_s = 0.0
    node._dispatch_in_progress = False
    node._expire_failures = lambda now: None
    node._consume_local_fallback_trigger = lambda: False
    node._allocator_timing_timed_call = (
        lambda section, callback, *args, **kwargs: callback(*args, **kwargs)
    )
    reset_reasons = []
    node._reset_round = lambda reason: reset_reasons.append(reason)
    DistributedFrontierAssignment._tick_impl(node)

    assert reset_reasons == []
    assert node._round is active_round
    assert node._round_created_count == created
    assert node._round_replaced_count == replaced

    # Once the same peer context is fresh again, activation retains the
    # original continuation generation instead of canonicalizing/restarting.
    node._peer_status = receive(
        node._peer_status.value, 10.0, NOW,
    )
    restored = node._continuation_context(NOW)
    assert restored is not None
    assert node._activate_continuation_round(restored)
    assert node._round is active_round
    assert node._round_created_count == created
    assert node._round_replaced_count == replaced


def test_continuation_context_identity_or_safety_change_still_invalidates():
    node, context, _ = _continuation_setup()

    node._peer_status.value.source_session_id = text_to_uuid('33' * 16)
    assert node._continuation_round_requires_invalidation(NOW)

    node, context, _ = _continuation_setup()
    node._active_commitments.pop(context.busy_robot_id)
    assert node._continuation_round_requires_invalidation(NOW)


def test_continuation_task_path_change_replaces_active_round():
    node, context, _ = _continuation_setup()
    active_round = node._round
    changed_task = replace(
        context.free_snapshot.tasks[0],
        local_path_length_m=3.0,
        local_path=((0.0, 0.0), (-3.0, 0.0)),
    )
    changed_snapshot = replace(
        context.free_snapshot,
        epoch=context.free_snapshot.epoch + 1,
        tasks=(changed_task,),
    )
    node._snapshots[context.free_robot_id] = receive(
        changed_snapshot, 10.0, NOW,
    )
    changed_context = node._continuation_context(NOW)
    assert changed_context is not None

    assert node._activate_continuation_round(changed_context)
    assert node._round is not active_round
    assert node._round_replaced_count == 1


def test_continuation_identity_still_changes_for_semantic_session_or_commitment():
    node, _, _, _, _ = _fake_node()
    context = node._continuation_context(NOW)
    assert context is not None
    baseline = node._continuation_round_id(context, 'content')

    assert node._continuation_round_id(context, 'changed-path') != baseline

    changed_session = replace(
        context.free_snapshot, source_session_id='new-free-session',
    )
    changed_session_context = replace(
        context, free_snapshot=changed_session,
    )
    assert node._continuation_round_id(
        changed_session_context, 'content',
    ) != baseline

    changed_commitment = replace(
        context.commitment, commitment_id='new-commitment',
    )
    changed_commitment_context = replace(
        context, commitment=changed_commitment,
    )
    assert node._continuation_round_id(
        changed_commitment_context, 'content',
    ) != baseline


def test_busy_goal_termination_invalidates_continuation_round():
    node, context, _ = _continuation_setup()
    assert node._round is not None
    node._clear_active_commitment(context.busy_robot_id, 'busy goal terminated')
    assert node._round is None
    assert context.busy_robot_id not in node._active_commitments


def test_busy_robot_retains_goal_instead_of_dispatching_continuation_copy():
    node, context, batches = _continuation_setup(
        local_robot='robot2', local_active=True,
    )
    decision = _select_continuation(node, context, batches)
    node._round.decision = decision
    node._nav2.local_goal_active = True
    invalidations = []
    node._invalidate_round = lambda *args: invalidations.append(args)
    node._start_local_dispatch(node._round, node._round_lifecycle.generation)
    assert not invalidations
    assert any('retaining active goal' in reason
               for _, reason in node._transition_log)


def test_continuation_free_commitment_is_persisted_for_the_next_symmetric_round():
    node, context, batches = _continuation_setup()
    decision = _select_continuation(node, context, batches)
    node._round.decision = decision
    busy_commitment = node._active_commitments[context.busy_robot_id]
    node._remember_active_commitments(node._round)
    free_id = context.free_robot_id
    assert free_id in node._active_commitments
    assert node._active_commitments[free_id].canonical_id == (
        context.free_tasks[0].canonical_id)
    assert node._active_commitments[context.busy_robot_id].canonical_id == (
        context.commitment.canonical_id)
    assert node._active_commitments[context.busy_robot_id] is busy_commitment
    assert node._active_commitments[context.busy_robot_id].decision_round_id == (
        context.commitment.decision_round_id)
    assert node._active_commitments[context.busy_robot_id].decision_hash == (
        context.commitment.decision_hash)
    assert node._active_commitments[context.busy_robot_id].commitment_id == (
        context.commitment.commitment_id)


def test_normal_pair_round_still_requires_both_bid_batches():
    node, _, _, _, _ = _fake_node()
    # The continuation helpers are not allowed to relax the existing normal
    # protocol predicate; a normal round with one received batch is invalid.
    assert node._both_bid_batches_valid(NOW, None) is False


def test_degraded_solo_status_is_a_temporary_peer_reservation():
    node, _, _, _, _ = _fake_node()
    node._peer_status = receive(SimpleNamespace(
        active_canonical_task_id='temporary-task',
        decision_hash='DEGRADED_SOLO',
        state=DistributedExplorationStatus.DEGRADED_SOLO,
        source_session_id=text_to_uuid(R2_SESSION),
    ), 10.0, NOW)
    assert node._temporary_peer_reservation_ids(NOW) == {'temporary-task'}


def test_normal_cooperative_status_is_not_treated_as_degraded_reservation():
    node, _, _, _, _ = _fake_node()
    node._peer_status.value.decision_hash = 'normal-decision'
    assert node._temporary_peer_reservation_ids(NOW) == set()
