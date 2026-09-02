"""Regressions for exhaustive deterministic two-robot pair assignment."""

import math

from my_epuck_project.distributed_assignment.canonical import build_canonical_union
from my_epuck_project.distributed_assignment.models import (
    Bid,
    BidBatch,
    Bounds,
    PhysicalTask,
)
from my_epuck_project.distributed_assignment.scoring import (
    AssignmentWeights,
    choose_mrtsp_route_assignment,
    choose_pair_assignment,
    decisions_match,
    nearby_goal_penalty,
    nominal_motion_cost_s,
    rank_solo_tasks,
    route_overlap,
)
from my_epuck_project.distributed_assignment.traffic_scheduler import schedule_traffic
from dataclasses import replace


def make_task(robot, signature, approach, centroid, bounds, gain=5.0, local_id=1):
    """Create a physical proposal fixture."""
    return PhysicalTask(
        robot, 's1' if robot == 'robot1' else 's2', 1, 1,
        signature, local_id, centroid, bounds, approach,
        visible_reveal_gain=gain, local_path_valid=True,
    )


def test_frontier_gain_mode_consumes_transported_generator_score():
    """Gain-aware mode ranks the exact source score without a new formula."""
    high = replace(make_task(
        'robot1', 'high', (1.0, 0.0), (1.0, 0.2),
        Bounds((0.8, 0.0), (1.2, 0.4)), gain=1.0,
    ), local_ordering_score=0.80)
    low = replace(make_task(
        'robot1', 'low', (0.1, 0.0), (0.1, 0.2),
        Bounds((-0.1, 0.0), (0.3, 0.4)), gain=1.0, local_id=2,
    ), local_ordering_score=0.10)
    union = build_canonical_union([high, low], [])
    ids = {member.physical_signature: task.canonical_id
           for task in union.tasks for member in task.members}
    first = batch('robot1', union.union_hash, [
        bid(ids['high'], 2.0, [(0.0, 0.0), (1.0, 0.0)]),
        bid(ids['low'], 0.2, [(0.0, 0.0), (0.1, 0.0)]),
    ])
    second = batch('robot2', union.union_hash, [])
    decision = choose_pair_assignment(
        'round', union, first, second, scoring_mode='frontier_gain',
    )
    assert decision.robot1_task_id == ids['high']
    assert decision.robot2_task_id == ''
    assert decision.diagnostics.strategy == 'frontier_gain'
    assert decision.score.team_local_ordering_score == 0.80


def test_frontier_gain_mode_preserves_canonical_duplicate_exclusion():
    """Changing utility mode cannot make one physical task win twice."""
    first_task = make_task(
        'robot1', 'opening-a', (0.0, 0.0), (0.0, 0.2),
        Bounds((-0.2, 0.0), (0.2, 0.4)), gain=5.0,
    )
    second_task = make_task(
        'robot2', 'opening-b', (0.21, 0.0), (0.05, 0.2),
        Bounds((-0.15, 0.0), (0.25, 0.4)), gain=5.0,
    )
    union = build_canonical_union([first_task], [second_task])
    only = union.tasks[0].canonical_id
    decision = choose_pair_assignment(
        'round', union,
        batch('robot1', union.union_hash, [bid(only, 1.0, [(0, 0), (0, 1)])]),
        batch('robot2', union.union_hash, [bid(only, 1.0, [(1, 0), (1, 1)])]),
        scoring_mode='frontier_gain',
    )
    assert not (decision.robot1_task_id == only and decision.robot2_task_id == only)


def test_mrtsp_route_mode_selects_distinct_top_ranked_tasks_without_scalar_utility():
    """Corrected mode uses upstream ordinal ranks, never min-max score values."""
    west = replace(make_task(
        'robot1', 'west', (-1.0, 0.0), (-1.0, 0.2),
        Bounds((-1.2, 0.0), (-0.8, 0.4)), gain=1.0,
    ), mrtsp_route_rank=0, mrtsp_solver='dp', local_ordering_score=-99.0)
    east = replace(make_task(
        'robot2', 'east', (1.0, 0.0), (1.0, 0.2),
        Bounds((0.8, 0.0), (1.2, 0.4)), gain=1.0,
    ), mrtsp_route_rank=0, mrtsp_solver='dp', local_ordering_score=99.0)
    union = build_canonical_union([west], [east])
    ids = {member.physical_signature: task.canonical_id
           for task in union.tasks for member in task.members}
    first = batch('robot1', union.union_hash, [
        bid(ids['west'], 1.0, [(0.0, 0.0), (-1.0, 0.0)]),
        bid(ids['east'], 2.0, [(0.0, 0.0), (1.0, 0.0)]),
    ])
    second = batch('robot2', union.union_hash, [
        bid(ids['west'], 2.0, [(0.0, 0.0), (-1.0, 0.0)]),
        bid(ids['east'], 1.0, [(0.0, 0.0), (1.0, 0.0)]),
    ])
    decision = choose_mrtsp_route_assignment('round', union, first, second)
    assert decision.robot1_task_id == ids['west']
    assert decision.robot2_task_id == ids['east']
    assert decision.score.team_local_ordering_score == 0.0
    assert decision.diagnostics.strategy == 'frontier_mrtsp'


def test_mrtsp_shared_top_uses_raw_path_then_robot2_tie_break():
    """A shared first task goes to lower path cost; a real tie goes to Robot 2."""
    shared_r1 = replace(make_task(
        'robot1', 'shared-r1', (0.0, 1.0), (0.0, 1.2),
        Bounds((-0.2, 1.0), (0.2, 1.4)), gain=1.0,
    ), mrtsp_route_rank=0, mrtsp_solver='dp')
    fallback_r1 = replace(make_task(
        'robot1', 'fallback-r1', (-1.0, 0.0), (-1.0, 0.2),
        Bounds((-1.2, 0.0), (-0.8, 0.4)), gain=1.0, local_id=2,
    ), mrtsp_route_rank=1, mrtsp_solver='dp')
    shared_r2 = replace(make_task(
        'robot2', 'shared-r2', (0.01, 1.0), (0.01, 1.2),
        Bounds((-0.19, 1.0), (0.21, 1.4)), gain=1.0,
    ), mrtsp_route_rank=0, mrtsp_solver='dp')
    fallback_r2 = replace(make_task(
        'robot2', 'fallback-r2', (1.0, 0.0), (1.0, 0.2),
        Bounds((0.8, 0.0), (1.2, 0.4)), gain=1.0, local_id=2,
    ), mrtsp_route_rank=1, mrtsp_solver='dp')
    union = build_canonical_union([shared_r1, fallback_r1], [shared_r2, fallback_r2])
    ids = {member.physical_signature: task.canonical_id
           for task in union.tasks for member in task.members}
    shared = ids['shared-r1']
    first = batch('robot1', union.union_hash, [
        bid(shared, 1.0, [(0, 0), (0, 1)]),
        bid(ids['fallback-r1'], 2.0, [(0, 0), (-1, 0)]),
        bid(ids['fallback-r2'], 2.0, [(0, 0), (1, 0)]),
    ])
    second = batch('robot2', union.union_hash, [
        bid(shared, 1.0, [(0, 0), (0, 1)]),
        bid(ids['fallback-r1'], 2.0, [(0, 0), (-1, 0)]),
        bid(ids['fallback-r2'], 2.0, [(0, 0), (1, 0)]),
    ])
    decision = choose_mrtsp_route_assignment('round', union, first, second)
    assert decision.robot2_task_id == shared
    assert decision.robot1_task_id == ids['fallback-r1']
    assert decision.robot1_task_id != decision.robot2_task_id
    assert decision.diagnostics.burgard_trace[0]['duplicate_resolution'] == (
        'SHARED_TOP_RAW_PATH_TIE_ROBOT2')


def test_mrtsp_completed_task_is_skipped_until_its_physical_frontier_evolves():
    """A terminal task cannot become an immediate residual redispatch."""
    completed = replace(make_task(
        'robot1', 'completed', (0.2, 0.0), (0.2, 0.2),
        Bounds((0.0, 0.0), (0.4, 0.4)), gain=1.0,
    ), mrtsp_route_rank=0, mrtsp_solver='dp')
    next_task = replace(make_task(
        'robot1', 'next', (1.0, 0.0), (1.0, 0.2),
        Bounds((0.8, 0.0), (1.2, 0.4)), gain=1.0, local_id=2,
    ), mrtsp_route_rank=1, mrtsp_solver='dp')
    union = build_canonical_union([completed, next_task], [])
    ids = {member.physical_signature: task.canonical_id
           for task in union.tasks for member in task.members}
    decision = choose_mrtsp_route_assignment(
        'round', union,
        batch('robot1', union.union_hash, [
            bid(ids['completed'], 0.06, [(0.0, 0.0), (0.06, 0.0)]),
            bid(ids['next'], 1.0, [(0.0, 0.0), (1.0, 0.0)]),
        ]),
        batch('robot2', union.union_hash, []),
        hard_failed_tasks=frozenset({ids['completed']}),
    )
    assert decision.robot1_task_id == ids['next']
    assert decision.robot2_task_id == ''


def batch(robot, union_hash, bids):
    """Create a correctly round-bound bid batch."""
    return BidBatch(
        'round', union_hash, robot,
        's1' if robot == 'robot1' else 's2', 1, 2.0, tuple(bids),
    )


def bid(task_id, length, path, valid=True, heading=0.0):
    """Create a bounded path bid fixture."""
    return Bid(task_id, valid, length, length, heading_cost=heading,
               path=tuple(path))


def traffic_compatible_with_robot1_commitment(
        first_id, first_bid, second_id, second_bid):
    """Use the production scheduler as a pure selection-side oracle."""
    if first_bid is None or second_bid is None:
        return True
    result = schedule_traffic(
        first_bid.path, second_bid.path,
        robot1_safe_radius_m=0.08,
        robot2_safe_radius_m=0.08,
        reference_speed_mps=0.13,
        eta_tie_s=0.05,
        active_robots=frozenset({'robot1'}),
    )
    return not result.conflict


def _mrtsp_busy_free_fixture(free_paths):
    """Build the 558 s forensic shape: fixed R1 plus ordered R2 options."""
    busy = replace(make_task(
        'robot1', 'busy-commitment', (4.0, 0.0), (4.0, 0.2),
        Bounds((3.8, -0.2), (4.2, 0.4)), gain=1.0,
    ), mrtsp_route_rank=0, mrtsp_solver='dp')
    free_tasks = [replace(make_task(
        'robot2', 'free-rank-%d' % index, (1.0 + index, 1.0),
        (1.0 + index, 1.2),
        Bounds((0.8 + index, 0.8), (1.2 + index, 1.4)), gain=1.0,
        local_id=index + 1,
    ), mrtsp_route_rank=index, mrtsp_solver='dp')
                   for index in range(len(free_paths))]
    union = build_canonical_union([busy], free_tasks)
    ids = {
        member.physical_signature: task.canonical_id
        for task in union.tasks for member in task.members
    }
    first = batch('robot1', union.union_hash, [
        bid(ids['busy-commitment'], 4.0, [(0.0, 0.0), (4.0, 0.0)]),
    ])
    second = batch('robot2', union.union_hash, [
        bid(ids['free-rank-%d' % index],
            sum(math.dist(a, b) for a, b in zip(path, path[1:])), path)
        for index, path in enumerate(free_paths)
    ])
    return union, ids, first, second


def test_traffic_aware_mrtsp_continuation_reproduces_forensic_fallback():
    """A conflicting rank 0 must not serialize a clear rank 1 continuation."""
    union, ids, first, second = _mrtsp_busy_free_fixture([
        [(2.0, -1.0), (2.0, 1.0)],
        [(0.0, 3.0), (0.0, 4.0)],
    ])
    baseline = choose_mrtsp_route_assignment(
        'round', union, first, second,
        fixed_robot1_task_id=ids['busy-commitment'],
    )
    selected = choose_mrtsp_route_assignment(
        'round', union, first, second,
        fixed_robot1_task_id=ids['busy-commitment'],
        traffic_compatibility=traffic_compatible_with_robot1_commitment,
    )
    assert baseline.robot2_task_id == ids['free-rank-0']
    assert selected.robot2_task_id == ids['free-rank-1']
    assert selected.robot1_task_id == baseline.robot1_task_id == ids['busy-commitment']
    assert selected.diagnostics.burgard_trace[0]['traffic_aware_fallback'] is True
    assert traffic_compatible_with_robot1_commitment(
        selected.robot1_task_id,
        next(item for item in first.bids if item.canonical_task_id == selected.robot1_task_id),
        selected.robot2_task_id,
        next(item for item in second.bids if item.canonical_task_id == selected.robot2_task_id),
    )


def test_traffic_aware_mrtsp_keeps_rank_zero_when_clear():
    union, ids, first, second = _mrtsp_busy_free_fixture([
        [(0.0, 3.0), (0.0, 4.0)],
        [(2.0, -1.0), (2.0, 1.0)],
    ])
    selected = choose_mrtsp_route_assignment(
        'round', union, first, second,
        fixed_robot1_task_id=ids['busy-commitment'],
        traffic_compatibility=traffic_compatible_with_robot1_commitment,
    )
    assert selected.robot2_task_id == ids['free-rank-0']
    assert selected.diagnostics.burgard_trace[0]['traffic_aware_fallback'] is False


def test_traffic_aware_mrtsp_advances_until_rank_two_when_needed():
    union, ids, first, second = _mrtsp_busy_free_fixture([
        [(2.0, -1.0), (2.0, 1.0)],
        [(3.0, -1.0), (3.0, 1.0)],
        [(0.0, 3.0), (0.0, 4.0)],
    ])
    selected = choose_mrtsp_route_assignment(
        'round', union, first, second,
        fixed_robot1_task_id=ids['busy-commitment'],
        traffic_compatibility=traffic_compatible_with_robot1_commitment,
    )
    assert selected.robot2_task_id == ids['free-rank-2']


def test_traffic_aware_mrtsp_preserves_serialization_when_all_options_conflict():
    union, ids, first, second = _mrtsp_busy_free_fixture([
        [(2.0, -1.0), (2.0, 1.0)],
        [(3.0, -1.0), (3.0, 1.0)],
    ])
    selected = choose_mrtsp_route_assignment(
        'round', union, first, second,
        fixed_robot1_task_id=ids['busy-commitment'],
        traffic_compatibility=traffic_compatible_with_robot1_commitment,
    )
    assert selected.robot2_task_id == ids['free-rank-0']
    assert selected.diagnostics.burgard_trace[0]['traffic_aware_fallback'] is False


def test_traffic_aware_cost_only_skips_conflict_but_keeps_cost_order():
    busy = cost_task('busy', (4.0, 0.0), 4.0, 0.0, local_id=1)
    conflict = cost_task('conflict', (2.0, 0.0), 1.0, 0.0, local_id=2)
    clear = cost_task('clear', (0.0, 4.0), 3.0, 0.0, local_id=3)
    union = build_canonical_union([busy], [conflict, clear])
    ids = {
        member.physical_signature: task.canonical_id
        for task in union.tasks for member in task.members
    }
    first = batch('robot1', union.union_hash, [
        bid(ids['busy'], 4.0, [(0.0, 0.0), (4.0, 0.0)]),
    ])
    second = batch('robot2', union.union_hash, [
        bid(ids['conflict'], 1.0, [(2.0, -1.0), (2.0, 1.0)]),
        bid(ids['clear'], 3.0, [(0.0, 3.0), (0.0, 4.0)]),
    ])
    selected = choose_pair_assignment(
        'round', union, first, second,
        scoring_mode='frontier_cost_only',
        fixed_robot1_task_id=ids['busy'],
        traffic_compatibility=traffic_compatible_with_robot1_commitment,
    )
    assert selected.robot1_task_id == ids['busy']
    assert selected.robot2_task_id == ids['clear']


def test_traffic_aware_both_free_prefers_clear_pair_in_existing_mrtsp_order():
    r1_top = replace(make_task(
        'robot1', 'r1-top', (4.0, 0.0), (4.0, 0.2),
        Bounds((3.8, -0.2), (4.2, 0.4)), gain=1.0,
    ), mrtsp_route_rank=0, mrtsp_solver='dp')
    r1_fallback = replace(make_task(
        'robot1', 'r1-fallback', (0.0, 3.0), (0.0, 3.2),
        Bounds((-0.2, 2.8), (0.2, 3.4)), gain=1.0, local_id=2,
    ), mrtsp_route_rank=1, mrtsp_solver='dp')
    r2_top = replace(make_task(
        'robot2', 'r2-top', (2.0, 0.0), (2.0, 0.2),
        Bounds((1.8, -0.2), (2.2, 0.4)), gain=1.0,
    ), mrtsp_route_rank=0, mrtsp_solver='dp')
    r2_fallback = replace(make_task(
        'robot2', 'r2-fallback', (0.0, 4.0), (0.0, 4.2),
        Bounds((-0.2, 3.8), (0.2, 4.4)), gain=1.0, local_id=2,
    ), mrtsp_route_rank=1, mrtsp_solver='dp')
    union = build_canonical_union([r1_top, r1_fallback], [r2_top, r2_fallback])
    ids = {
        member.physical_signature: task.canonical_id
        for task in union.tasks for member in task.members
    }
    first = batch('robot1', union.union_hash, [
        bid(ids['r1-top'], 4.0, [(0.0, 0.0), (4.0, 0.0)]),
        bid(ids['r1-fallback'], 3.0, [(0.0, 3.0), (0.0, 4.0)]),
    ])
    second = batch('robot2', union.union_hash, [
        bid(ids['r2-top'], 2.0, [(2.0, -1.0), (2.0, 1.0)]),
        bid(ids['r2-fallback'], 4.0, [(0.0, 4.0), (0.0, 5.0)]),
    ])
    def compatible(first_id, first_bid, second_id, second_bid):
        if first_bid is None or second_bid is None:
            return True
        return not schedule_traffic(
            first_bid.path, second_bid.path,
            robot1_safe_radius_m=0.08, robot2_safe_radius_m=0.08,
            reference_speed_mps=0.13, eta_tie_s=0.05,
        ).conflict
    selected = choose_mrtsp_route_assignment(
        'round', union, first, second, traffic_compatibility=compatible,
    )
    assert (selected.robot1_task_id, selected.robot2_task_id) != (
        ids['r1-top'], ids['r2-top'])
    assert selected.robot1_task_id == ids['r1-top']
    assert selected.robot2_task_id == ids['r2-fallback']


def cost_task(signature, approach, path_length, heading, gain=1.0, local_id=1):
    """Create a task carrying the cost-only candidate primitives."""
    return replace(
        make_task(
            'robot1', signature, approach, approach,
            Bounds((approach[0] - 0.1, approach[1] - 0.1),
                   (approach[0] + 0.1, approach[1] + 0.1)),
            gain=gain, local_id=local_id,
        ),
        local_path_valid=True,
        local_path_length_m=path_length,
        path_heading_cost_rad=heading,
    )


def test_cost_only_prefers_initially_aligned_path_when_path_cost_ties():
    """Equal travel cost is resolved by the actual path's initial heading."""
    aligned = cost_task('aligned', (1.0, 0.0), 2.0, 0.0, local_id=1)
    turning = cost_task('turning', (0.0, 1.0), 2.0, 1.5, local_id=2)
    union = build_canonical_union([aligned, turning], [])
    ids = {member.physical_signature: task.canonical_id
           for task in union.tasks for member in task.members}
    decision = choose_pair_assignment(
        'round', union,
        batch('robot1', union.union_hash, [
            bid(ids['aligned'], 2.0, [(0, 0), (2, 0)], heading=0.0),
            bid(ids['turning'], 2.0, [(0, 0), (0, 2)], heading=1.5),
        ]),
        batch('robot2', union.union_hash, []),
        scoring_mode='frontier_cost_only',
    )
    assert decision.robot1_task_id == ids['aligned']
    assert decision.diagnostics.strategy == 'frontier_cost_only'


def test_nominal_motion_cost_heading_order_is_physical():
    """For equal path length, no turn beats 90 degrees and 180 degrees."""
    costs = [nominal_motion_cost_s(4.0, heading)
             for heading in (0.0, math.pi / 2.0, math.pi)]
    assert costs[0] < costs[1] < costs[2]


def test_nominal_motion_cost_180_degree_penalty_is_equivalent_to_1_1667_m():
    """The configured references produce the specified physical tradeoff."""
    penalty_s = nominal_motion_cost_s(0.0, math.pi)
    penalty_m = 0.13 * math.pi / 0.35
    assert math.isclose(penalty_s, math.pi / 0.35, rel_tol=0.0, abs_tol=1e-12)
    assert math.isclose(penalty_m, 1.1667, rel_tol=0.0, abs_tol=2e-4)


def _select_cost_only_pair(path_a, heading_a, path_b, heading_b):
    first = cost_task('physical-a', (1.0, 0.0), path_a, heading_a, local_id=1)
    second = cost_task('physical-b', (3.0, 0.0), path_b, heading_b, local_id=2)
    union = build_canonical_union([first, second], [])
    ids = {member.physical_signature: task.canonical_id
           for task in union.tasks for member in task.members}
    decision = choose_pair_assignment(
        'round', union,
        batch('robot1', union.union_hash, [
            bid(ids['physical-a'], path_a, [(0.0, 0.0), (path_a, 0.0)],
                heading=heading_a),
            bid(ids['physical-b'], path_b, [(0.0, 0.0), (path_b, 0.0)],
                heading=heading_b),
        ]),
        batch('robot2', union.union_hash, []),
        scoring_mode='frontier_cost_only',
    )
    return decision, ids


def test_cost_only_physical_tradeoff_4m_zero_beats_3_5m_180deg():
    decision, ids = _select_cost_only_pair(4.0, 0.0, 3.5, math.pi)
    assert decision.robot1_task_id == ids['physical-a']


def test_cost_only_physical_tradeoff_2_5m_180deg_beats_4m_zero():
    decision, ids = _select_cost_only_pair(4.0, 0.0, 2.5, math.pi)
    assert decision.robot1_task_id == ids['physical-b']


def test_cost_only_pairwise_tradeoff_is_independent_of_other_batch_members():
    """Adding an unrelated candidate cannot change the fixed A/B ordering."""
    first = cost_task('fixed-a', (1.0, 0.0), 4.0, 0.0, local_id=1)
    second = cost_task('fixed-b', (3.0, 0.0), 3.5, math.pi, local_id=2)
    extra = cost_task('extra', (20.0, 0.0), 0.1, math.pi, local_id=3)
    without_extra = rank_solo_tasks(
        [first, second], scoring_mode='frontier_cost_only')
    with_extra = rank_solo_tasks(
        [first, second, extra], scoring_mode='frontier_cost_only')
    order_without = [task.physical_signature for task in without_extra]
    order_with = [task.physical_signature for task in with_extra]
    assert order_without == ['fixed-a', 'fixed-b']
    assert order_with.index('fixed-a') < order_with.index('fixed-b')


def test_cost_only_order_is_independent_of_information_gain():
    """Changing gain alone cannot change the cost-only choice or ordering."""
    near = cost_task('near', (1.0, 0.0), 1.0, 0.0, gain=0.01, local_id=1)
    far = cost_task('far', (4.0, 0.0), 3.0, 0.0, gain=100.0, local_id=2)
    union = build_canonical_union([near, far], [])
    ids = {member.physical_signature: task.canonical_id
           for task in union.tasks for member in task.members}
    def select(first, second):
        return choose_pair_assignment(
            'round', union,
            batch('robot1', union.union_hash, [
                bid(ids['near'], 1.0, [(0, 0), (1, 0)]),
                bid(ids['far'], 3.0, [(0, 0), (3, 0)]),
            ]),
            batch('robot2', union.union_hash, []),
            scoring_mode='frontier_cost_only',
        ).robot1_task_id
    first_choice = select(0.01, 100.0)
    changed_union = build_canonical_union([
        replace(near, visible_reveal_gain=100.0),
        replace(far, visible_reveal_gain=0.01),
    ], [])
    changed_ids = {member.physical_signature: task.canonical_id
                   for task in changed_union.tasks for member in task.members}
    changed_choice = choose_pair_assignment(
        'round', changed_union,
        batch('robot1', changed_union.union_hash, [
            bid(changed_ids['near'], 1.0, [(0, 0), (1, 0)]),
            bid(changed_ids['far'], 3.0, [(0, 0), (3, 0)]),
        ]),
        batch('robot2', changed_union.union_hash, []),
        scoring_mode='frontier_cost_only',
    ).robot1_task_id
    assert first_choice == ids['near']
    assert changed_choice == changed_ids['near']


def test_cost_only_gain_does_not_change_score_identity():
    """Ignored gain cannot alter cost-only score/hash agreement evidence."""
    first = cost_task('hash-a', (1.0, 0.0), 2.0, 0.0, gain=0.1, local_id=1)
    second = cost_task('hash-b', (3.0, 0.0), 3.0, 0.4, gain=0.2, local_id=2)
    union = build_canonical_union([first, second], [])
    ids = {member.physical_signature: task.canonical_id
           for task in union.tasks for member in task.members}
    def decide(tasks):
        local_union = build_canonical_union(tasks, [])
        task_ids = {member.physical_signature: task.canonical_id
                    for task in local_union.tasks for member in task.members}
        return choose_pair_assignment(
            'round', local_union,
            batch('robot1', local_union.union_hash, [
                bid(task_ids['hash-a'], 2.0, [(0.0, 0.0), (2.0, 0.0)]),
                bid(task_ids['hash-b'], 3.0, [(0.0, 0.0), (3.0, 0.0)],
                    heading=0.4),
            ]),
            batch('robot2', local_union.union_hash, []),
            scoring_mode='frontier_cost_only',
        )
    original = decide([first, second])
    changed = decide([
        replace(first, visible_reveal_gain=99.0),
        replace(second, visible_reveal_gain=0.001),
    ])
    assert original.robot1_task_id == changed.robot1_task_id == ids['hash-a']
    assert original.decision_hash == changed.decision_hash
    assert original.score.team_visible_gain == 0.0
    assert changed.score.team_visible_gain == 0.0


def test_cost_only_uses_nav2_path_cost_not_euclidean_distance():
    """A longer Euclidean target can win when its valid path is shorter."""
    near_detour = cost_task('near-detour', (1.0, 0.0), 8.0, 0.0, local_id=1)
    far_direct = cost_task('far-direct', (3.0, 0.0), 2.0, 0.0, local_id=2)
    union = build_canonical_union([near_detour, far_direct], [])
    ids = {member.physical_signature: task.canonical_id
           for task in union.tasks for member in task.members}
    decision = choose_pair_assignment(
        'round', union,
        batch('robot1', union.union_hash, [
            bid(ids['near-detour'], 8.0, [(0, 0), (8, 0)]),
            bid(ids['far-direct'], 2.0, [(0, 0), (2, 0)]),
        ]),
        batch('robot2', union.union_hash, []),
        scoring_mode='frontier_cost_only',
    )
    assert decision.robot1_task_id == ids['far-direct']
    assert decision.score.combined_path_cost == 2.0
    assert math.isclose(
        decision.score.combined_heading_cost, 0.0, rel_tol=0.0, abs_tol=1e-12)


def test_cost_only_solo_order_has_no_gain_leakage():
    """Local cost-only ordering also ignores gain and generator score."""
    first = cost_task('first', (1.0, 0.0), 1.0, 0.1, gain=0.1, local_id=1)
    second = cost_task('second', (2.0, 0.0), 2.0, 0.2, gain=99.0, local_id=2)
    ordered = rank_solo_tasks([first, second], scoring_mode='frontier_cost_only')
    assert [task.physical_signature for task in ordered] == ['first', 'second']
    changed = [replace(first, visible_reveal_gain=99.0,
                       local_ordering_score=100.0),
               replace(second, visible_reveal_gain=0.1,
                       local_ordering_score=-100.0)]
    ordered_changed = rank_solo_tasks(
        changed, scoring_mode='frontier_cost_only')
    assert [task.physical_signature for task in ordered_changed] == [
        'first', 'second']


def test_route_overlap_and_nearby_penalties_are_bounded():
    """Keep assignment geometry terms within zero and one."""
    assert route_overlap([(0, 0), (0, 1)], [(0.1, 0), (0.1, 1)], 0.1) == 1.0
    assert route_overlap([(0, 0)], [(5, 5)], 0.1) == 0.0
    assert nearby_goal_penalty((0, 0), (0.2, 0), 0.5) > 0.5


def test_valid_long_paths_remain_feasible_and_retain_distance_cost():
    """18 m is not a hidden eligibility boundary in distributed scoring."""
    task = make_task(
        'robot1', 'long', (25.0, 0.0), (25.0, 0.2),
        Bounds((24.8, 0.0), (25.2, 0.4)), gain=1.0,
    )
    union = build_canonical_union([task], [])
    task_id = union.tasks[0].canonical_id
    for length in (17.9, 18.0, 18.1, 25.0, 50.0):
        decision = choose_pair_assignment(
            'round', union,
            batch('robot1', union.union_hash, [
                bid(task_id, length, [(0.0, 0.0), (length, 0.0)])]),
            batch('robot2', union.union_hash, []),
        )
        assert decision.robot1_task_id == task_id
        assert decision.diagnostics.rejected_path_threshold_count == 0


def test_invalid_or_nonfinite_paths_remain_ineligible():
    """No-path and malformed path data are still rejected safely."""
    task = make_task(
        'robot1', 'invalid', (1.0, 0.0), (1.0, 0.2),
        Bounds((0.8, 0.0), (1.2, 0.4)), gain=1.0,
    )
    union = build_canonical_union([task], [])
    task_id = union.tasks[0].canonical_id
    for length, valid, path in (
            (0.0, False, []),
            (1.0, True, [(0.0, 0.0), (float('nan'), 0.0)]),
            (1.0, True, [(0.0, 0.0), (float('inf'), 0.0)])):
        decision = choose_pair_assignment(
            'round', union,
            batch('robot1', union.union_hash, [
                bid(task_id, length, path, valid)]),
            batch('robot2', union.union_hash, []),
        )
        assert decision.robot1_task_id == ''


def test_hallway_following_regression_selects_alternate_branch():
    """Robot 2 must avoid a strongly overlapping northern route."""
    north1 = make_task(
        'robot1', 'north-near', (0.0, 5.0), (0.0, 5.2),
        Bounds((-0.2, 5.0), (0.2, 5.4)), gain=5.0,
    )
    north2 = make_task(
        'robot2', 'north-far', (0.9, 5.0), (0.9, 5.2),
        Bounds((0.7, 5.0), (1.1, 5.4)), gain=5.0,
    )
    alternate = make_task(
        'robot2', 'east-branch', (5.0, 0.0), (5.2, 0.0),
        Bounds((5.0, -0.2), (5.4, 0.2)), gain=4.5, local_id=2,
    )
    union = build_canonical_union([north1], [north2, alternate])
    ids = {member.physical_signature: item.canonical_id
           for item in union.tasks for member in item.members}
    north_path1 = [(0.0, 0.0), (0.0, 2.0), (0.0, 4.0), (0.0, 5.0)]
    north_path2 = [(0.1, 0.0), (0.1, 2.0), (0.1, 4.0), (0.9, 5.0)]
    east_path = [(0.1, 0.0), (2.0, 0.0), (4.0, 0.0), (5.0, 0.0)]
    first = batch('robot1', union.union_hash, [
        bid(ids['north-near'], 5.0, north_path1),
        bid(ids['north-far'], 5.2, north_path2),
        bid(ids['east-branch'], 5.4, east_path),
    ])
    second = batch('robot2', union.union_hash, [
        bid(ids['north-near'], 5.1, north_path1),
        bid(ids['north-far'], 5.0, north_path2),
        bid(ids['east-branch'], 5.0, east_path),
    ])
    decision = choose_pair_assignment('round', union, first, second)
    assert decision.robot1_task_id == ids['north-near']
    assert decision.robot2_task_id == ids['east-branch']
    assert decision.score.route_overlap_penalty < 0.5


def test_near_duplicate_tasks_cannot_both_win():
    """Equivalent tasks collapse before ordered-pair enumeration."""
    first_task = make_task(
        'robot1', 'opening-a', (0.0, 0.0), (0.0, 0.2),
        Bounds((-0.2, 0.0), (0.2, 0.4)),
    )
    second_task = make_task(
        'robot2', 'opening-b', (0.21, 0.0), (0.05, 0.2),
        Bounds((-0.15, 0.0), (0.25, 0.4)),
    )
    union = build_canonical_union([first_task], [second_task])
    assert len(union.tasks) == 1
    only = union.tasks[0].canonical_id
    first = batch('robot1', union.union_hash, [bid(only, 1.0, [(0, 0), (0, 1)])])
    second = batch('robot2', union.union_hash, [bid(only, 1.0, [(1, 0), (1, 1)])])
    decision = choose_pair_assignment('round', union, first, second)
    assert not (decision.robot1_task_id == only and decision.robot2_task_id == only)


def test_invalid_or_timed_out_bid_cannot_win():
    """Exclude a bid whose local path result is invalid."""
    proposal = make_task(
        'robot1', 'task', (1.0, 0.0), (1.0, 0.2),
        Bounds((0.8, 0.0), (1.2, 0.4)),
    )
    union = build_canonical_union([proposal], [])
    task_id = union.tasks[0].canonical_id
    first = batch('robot1', union.union_hash, [bid(task_id, 1.0, [], valid=False)])
    second = batch('robot2', union.union_hash, [])
    decision = choose_pair_assignment('round', union, first, second)
    assert decision.robot1_task_id == ''


def test_hard_failed_task_not_immediately_repeated_by_peer():
    """A team hard-failure penalty keeps the pair away from task X."""
    failed = make_task(
        'robot1', 'physical-x', (1.0, 0.0), (1.0, 0.2),
        Bounds((0.8, 0.0), (1.2, 0.4)), gain=5.0,
    )
    useful = make_task(
        'robot2', 'physical-y', (0.0, 2.0), (0.0, 2.2),
        Bounds((-0.2, 2.0), (0.2, 2.4)), gain=4.0,
    )
    union = build_canonical_union([failed], [useful])
    ids = {member.physical_signature: item.canonical_id
           for item in union.tasks for member in item.members}
    first = batch('robot1', union.union_hash, [
        bid(ids['physical-x'], 1.0, [(0, 0), (1, 0)]),
        bid(ids['physical-y'], 2.0, [(0, 0), (0, 2)]),
    ])
    second = batch('robot2', union.union_hash, [
        bid(ids['physical-x'], 1.0, [(0, 0), (1, 0)]),
        bid(ids['physical-y'], 2.0, [(0, 0), (0, 2)]),
    ])
    decision = choose_pair_assignment(
        'round', union, first, second,
        hard_failed_tasks=frozenset({ids['physical-x']}),
    )
    assert ids['physical-x'] not in (
        decision.robot1_task_id, decision.robot2_task_id,
    )


def test_decision_hash_and_tie_break_ignore_bid_arrival_order():
    """Stable fingerprints and ties do not depend on message ordering."""
    west = make_task(
        'robot1', 'west', (-2.0, 0.0), (-2.0, 0.2),
        Bounds((-2.2, 0.0), (-1.8, 0.4)),
    )
    east = make_task(
        'robot2', 'east', (2.0, 0.0), (2.0, 0.2),
        Bounds((1.8, 0.0), (2.2, 0.4)),
    )
    union = build_canonical_union([west], [east])
    ids = [item.canonical_id for item in union.tasks]
    bids1 = [bid(ids[0], 2.0, []), bid(ids[1], 2.0, [])]
    bids2 = [bid(ids[0], 2.0, []), bid(ids[1], 2.0, [])]
    a = choose_pair_assignment(
        'round', union, batch('robot1', union.union_hash, bids1),
        batch('robot2', union.union_hash, list(reversed(bids2))),
        weights=AssignmentWeights(route_overlap=0.0, sensing_overlap=0.0),
    )
    b = choose_pair_assignment(
        'round', union, batch('robot1', union.union_hash, list(reversed(bids1))),
        batch('robot2', union.union_hash, bids2),
        weights=AssignmentWeights(route_overlap=0.0, sensing_overlap=0.0),
    )
    assert decisions_match(a, b)


def test_feasible_negative_soft_utility_still_assigns_one_active_robot():
    """Feasibility, not zero-valued IDLE, controls useful work."""
    task = make_task(
        'robot1', 'small-useful', (10.0, 0.0), (10.0, 0.2),
        Bounds((9.8, 0.0), (10.2, 0.4)), gain=0.05,
    )
    union = build_canonical_union([task], [])
    task_id = union.tasks[0].canonical_id
    decision = choose_pair_assignment(
        'round', union, batch('robot1', union.union_hash, [
            bid(task_id, 10.0, [(0.0, 0.0), (10.0, 0.0)]),
        ]), batch('robot2', union.union_hash, []),
    )
    assert decision.robot1_task_id == task_id
    assert decision.robot2_task_id == ''
    assert decision.score.total < 0.0
    assert decision.diagnostics.idle_idle_permitted is False


def test_two_feasible_negative_tasks_beat_idle_idle():
    """Two feasible tasks remain eligible even when absolute score is negative."""
    first_task = make_task(
        'robot1', 'negative-a', (10.0, 0.0), (10.0, 0.2),
        Bounds((9.8, 0.0), (10.2, 0.4)), gain=0.05,
    )
    second_task = make_task(
        'robot2', 'negative-b', (0.0, 10.0), (0.0, 10.2),
        Bounds((-0.2, 9.8), (0.2, 10.2)), gain=0.05,
    )
    union = build_canonical_union([first_task], [second_task])
    ids = {m.physical_signature: t.canonical_id
           for t in union.tasks for m in t.members}
    decision = choose_pair_assignment(
        'round', union,
        batch('robot1', union.union_hash, [
            bid(ids['negative-a'], 10.0, [(0, 0), (10, 0)]),
            bid(ids['negative-b'], 10.0, [(0, 10), (0, 20)]),
        ]),
        batch('robot2', union.union_hash, [
            bid(ids['negative-a'], 10.0, [(0, 0), (10, 0)]),
            bid(ids['negative-b'], 10.0, [(0, 10), (0, 20)]),
        ]),
    )
    assert decision.robot1_task_id or decision.robot2_task_id
    assert decision.diagnostics.idle_idle_permitted is False
    assert decision.score.total < 0.0


def test_zero_gain_is_explicitly_infeasible():
    """Zero visible reveal gain cannot be dispatched."""
    task = make_task(
        'robot1', 'zero-gain', (1.0, 0.0), (1.0, 0.2),
        Bounds((0.8, 0.0), (1.2, 0.4)), gain=0.0,
    )
    union = build_canonical_union([task], [])
    task_id = union.tasks[0].canonical_id
    decision = choose_pair_assignment(
        'round', union, batch('robot1', union.union_hash, [
            bid(task_id, 1.0, [(0, 0), (1, 0)]),
        ]), batch('robot2', union.union_hash, []),
    )
    assert decision.robot1_task_id == ''
    assert decision.diagnostics.idle_reason == 'BELOW_GAIN_THRESHOLD'


def test_long_path_is_expensive_but_not_infeasible():
    """A valid 19 m path remains eligible and retains its true distance."""
    task = make_task(
        'robot1', 'backtrack', (19.0, 0.0), (19.0, 0.2),
        Bounds((18.8, 0.0), (19.2, 0.4)), gain=5.0,
    )
    union = build_canonical_union([task], [])
    task_id = union.tasks[0].canonical_id
    decision = choose_pair_assignment(
        'round', union, batch('robot1', union.union_hash, [
            bid(task_id, 19.0, [(0, 0), (19, 0)]),
        ]), batch('robot2', union.union_hash, []),
    )
    assert decision.robot1_task_id == task_id
    assert decision.combined_path_length_m == 19.0
    assert decision.diagnostics.rejected_path_threshold_count == 0


def test_hard_failed_equivalent_task_does_not_block_useful_task():
    """A hard-failed physical task is filtered while another task proceeds."""
    failed = make_task(
        'robot1', 'failed', (1.0, 0.0), (1.0, 0.2),
        Bounds((0.8, 0.0), (1.2, 0.4)), gain=5.0,
    )
    useful = make_task(
        'robot2', 'useful', (0.0, 2.0), (0.0, 2.2),
        Bounds((-0.2, 2.0), (0.2, 2.4)), gain=0.05,
    )
    union = build_canonical_union([failed], [useful])
    ids = {m.physical_signature: t.canonical_id
           for t in union.tasks for m in t.members}
    decision = choose_pair_assignment(
        'round', union,
        batch('robot1', union.union_hash, [
            bid(ids['failed'], 1.0, [(0, 0), (1, 0)]),
            bid(ids['useful'], 10.0, [(0, 0), (0, 2)]),
        ]),
        batch('robot2', union.union_hash, [
            bid(ids['failed'], 1.0, [(0, 0), (1, 0)]),
            bid(ids['useful'], 10.0, [(0, 0), (0, 2)]),
        ]),
        hard_failed_tasks=frozenset({ids['failed']}),
    )
    assert ids['failed'] not in (decision.robot1_task_id, decision.robot2_task_id)
    assert ids['useful'] in (decision.robot1_task_id, decision.robot2_task_id)


def test_one_active_can_beat_conflicting_two_active_pair():
    """A strongly overlapping second task does not force a pair assignment."""
    first_task = make_task(
        'robot1', 'primary', (2.0, 0.0), (2.0, 0.2),
        Bounds((1.8, 0.0), (2.2, 0.4)), gain=2.0,
    )
    second_task = make_task(
        'robot2', 'nearby', (2.8, 0.0), (2.8, 0.2),
        Bounds((2.6, 0.0), (3.0, 0.4)), gain=0.05,
    )
    union = build_canonical_union([first_task], [second_task])
    ids = {m.physical_signature: t.canonical_id
           for t in union.tasks for m in t.members}
    same_path = [(0, 0), (1, 0), (2, 0)]
    decision = choose_pair_assignment(
        'round', union,
        batch('robot1', union.union_hash, [
            bid(ids['primary'], 2.0, same_path),
            bid(ids['nearby'], 2.0, same_path),
        ]),
        batch('robot2', union.union_hash, [
            bid(ids['primary'], 2.0, same_path),
            bid(ids['nearby'], 2.0, same_path),
        ]),
    )
    assert (decision.robot1_task_id, decision.robot2_task_id) in {
        (ids['primary'], ''), ('', ids['primary']),
    }


def test_conflict_free_pair_beats_numerically_better_solo_assignment():
    """Use both robots when the second useful task has no conflict signal."""
    first_task = make_task(
        'robot1', 'first', (5.0, 0.0), (5.0, 0.2),
        Bounds((4.8, 0.0), (5.2, 0.4)), gain=0.05,
    )
    second_task = make_task(
        'robot2', 'second', (0.0, 5.0), (0.0, 5.2),
        Bounds((-0.2, 4.8), (0.2, 5.2)), gain=0.05,
    )
    union = build_canonical_union([first_task], [second_task])
    ids = {m.physical_signature: t.canonical_id
           for t in union.tasks for m in t.members}
    decision = choose_pair_assignment(
        'round', union,
        batch('robot1', union.union_hash, [
            bid(ids['first'], 5.0, [(0, 0), (5, 0)]),
        ]),
        batch('robot2', union.union_hash, [
            bid(ids['second'], 5.0, [(0, 1), (0, 5)]),
        ]),
    )
    assert (decision.robot1_task_id, decision.robot2_task_id) == (
        ids['first'], ids['second'],
    )
    assert decision.score.total < 0.0
    assert decision.score.route_overlap_penalty == 0.0
    assert decision.score.sensing_overlap_penalty == 0.0


def test_best_conflict_free_pair_beats_higher_scoring_conflicting_pair():
    """Filter conflict before ranking, rather than inspecting only raw best."""
    first_task = make_task(
        'robot1', 'first', (2.0, 0.0), (2.0, 0.2),
        Bounds((1.8, 0.0), (2.2, 0.4)), gain=1.0,
    )
    independent = make_task(
        'robot2', 'independent', (0.0, 4.0), (0.0, 4.2),
        Bounds((-0.2, 3.8), (0.2, 4.2)), gain=0.05,
    )
    conflicting = make_task(
        'robot2', 'conflicting', (2.8, 0.0), (2.8, 0.2),
        Bounds((2.6, 0.0), (3.0, 0.4)), gain=5.0, local_id=2,
    )
    union = build_canonical_union([first_task], [independent, conflicting])
    ids = {m.physical_signature: t.canonical_id
           for t in union.tasks for m in t.members}
    decision = choose_pair_assignment(
        'round', union,
        batch('robot1', union.union_hash, [
            bid(ids['first'], 2.0, [(0, 0), (2, 0)]),
        ]),
        batch('robot2', union.union_hash, [
            bid(ids['independent'], 2.0, [(0, 1), (0, 4)]),
            bid(ids['conflicting'], 2.0, [(0, 0), (2, 0)]),
        ]),
    )
    assert (decision.robot1_task_id, decision.robot2_task_id) == (
        ids['first'], ids['independent'],
    )
    assert decision.score.nearby_goal_penalty == 0.0
    assert decision.score.route_overlap_penalty == 0.0
    assert decision.score.sensing_overlap_penalty == 0.0


def test_existing_rank_selects_best_of_multiple_conflict_free_pairs():
    """Conflict-free selection preserves the original utility ranking."""
    first_task = make_task(
        'robot1', 'first', (4.0, 0.0), (4.0, 0.2),
        Bounds((3.8, 0.0), (4.2, 0.4)), gain=1.0,
    )
    lower = make_task(
        'robot2', 'lower', (0.0, 4.0), (0.0, 4.2),
        Bounds((-0.2, 3.8), (0.2, 4.2)), gain=0.1,
    )
    higher = make_task(
        'robot2', 'higher', (-4.0, 0.0), (-4.0, 0.2),
        Bounds((-4.2, 0.0), (-3.8, 0.4)), gain=2.0, local_id=2,
    )
    union = build_canonical_union([first_task], [lower, higher])
    ids = {m.physical_signature: t.canonical_id
           for t in union.tasks for m in t.members}
    decision = choose_pair_assignment(
        'round', union,
        batch('robot1', union.union_hash, [
            bid(ids['first'], 2.0, [(0, 0), (4, 0)]),
        ]),
        batch('robot2', union.union_hash, [
            bid(ids['lower'], 2.0, [(0, 1), (0, 4)]),
            bid(ids['higher'], 2.0, [(0, -1), (-4, 0)]),
        ]),
    )
    assert (decision.robot1_task_id, decision.robot2_task_id) == (
        ids['first'], ids['higher'],
    )
