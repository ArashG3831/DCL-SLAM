"""Regressions for exhaustive deterministic two-robot pair assignment."""

import json
import math
from types import SimpleNamespace

from my_epuck_project.distributed_assignment.canonical import build_canonical_union
from my_epuck_project.distributed_assignment.models import (
    Bid,
    BidBatch,
    Bounds,
    PhysicalTask,
    TaskSnapshot,
)
from my_epuck_project.distributed_assignment.scoring import (
    AssignmentWeights,
    cost_only_dispatch_certificate,
    choose_mrtsp_route_assignment,
    choose_pair_assignment,
    decisions_match,
    nearby_goal_penalty,
    nominal_motion_cost_s,
    rank_solo_tasks,
    route_overlap,
)
from my_epuck_project.distributed_assignment.traffic_scheduler import schedule_traffic
from my_epuck_project.distributed_frontier_assignment import (
    DistributedFrontierAssignment,
    classify_lower_bound_evidence,
    completion_evidence_matches_snapshots,
    lower_bound_context_matches,
)
from my_epuck_project.mission_termination import CandidateEvidence
from my_epuck_interfaces.msg import FrontierCandidateArray
from dataclasses import replace


def test_completion_evidence_requires_current_candidate_provenance():
    snapshots = (
        TaskSnapshot('robot1', 's1', 2, 11, 'm1', 0, 2.0, (), '', 4, 7),
        TaskSnapshot('robot2', 's2', 3, 12, 'm2', 0, 2.0, (), '', 5, 9),
    )
    metadata = {
        'robot1': {'map_revision': 11, 'costmap_revision': 4,
                   'candidate_generation_id': 7},
        'robot2': {'map_revision': 12, 'costmap_revision': 5,
                   'candidate_generation_id': 9},
    }
    assert completion_evidence_matches_snapshots(metadata, snapshots)

    changed_map = snapshots[0].__class__(
        'robot1', 's1', 2, 13, 'm1', 0, 2.0, (), '', 4, 7,
    )
    assert not completion_evidence_matches_snapshots(
        metadata, (changed_map, snapshots[1]),
    )

    changed_generation = snapshots[1].__class__(
        'robot2', 's2', 3, 12, 'm2', 0, 2.0, (), '', 5, 10,
    )
    assert not completion_evidence_matches_snapshots(
        metadata, (snapshots[0], changed_generation),
    )

    assert not completion_evidence_matches_snapshots(
        {'robot1': metadata['robot1']}, snapshots,
    )


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


def batch(robot, union_hash, bids, round_id='round'):
    """Create a correctly round-bound bid batch."""
    return BidBatch(
        round_id, union_hash, robot,
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


def test_two_active_cardinality_precedes_conflicting_pair_score():
    """A valid pair wins before the existing conflict-aware score ranking."""
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
    assert decision.robot1_task_id and decision.robot2_task_id
    assert decision.robot1_task_id != decision.robot2_task_id
    assert decision.diagnostics.valid_two_active_pair_count == 2


def _independent_cost_pair_fixture(round_id, first_path, second_path):
    """Build a two-robot cost-only pair resembling a recorded C round."""
    first = cost_task(
        '%s-robot1' % round_id, (4.0, 0.0), first_path, 0.0, local_id=1,
    )
    second = replace(cost_task(
        '%s-robot2' % round_id, (0.0, 4.0), second_path, 0.0, local_id=2,
    ), source_robot_id='robot2', source_session_id='s2')
    union = build_canonical_union([first], [second])
    ids = {
        member.physical_signature: task.canonical_id
        for task in union.tasks for member in task.members
    }
    first_id = ids['%s-robot1' % round_id]
    second_id = ids['%s-robot2' % round_id]
    first_batch = batch('robot1', union.union_hash, [
        bid(first_id, first_path, [(0.0, 0.0), (first_path, 0.0)]),
    ], round_id=round_id)
    second_batch = batch('robot2', union.union_hash, [
        bid(second_id, second_path, [(0.0, 1.0), (0.0, second_path + 1.0)]),
    ], round_id=round_id)
    return union, first_batch, second_batch, first_id, second_id


def test_cost_only_recorded_63_20_round_prefers_two_active_cardinality():
    """The preserved 63.20 s shape cannot regress to robot1-only IDLE."""
    union, first, second, first_id, second_id = _independent_cost_pair_fixture(
        'c-63.20', 2.623618, 2.747824,
    )
    decision = choose_pair_assignment(
        'c-63.20', union, first, second,
        scoring_mode='frontier_cost_only',
    )
    assert (decision.robot1_task_id, decision.robot2_task_id) == (
        first_id, second_id,
    )


def test_cost_only_recorded_96_52_round_prefers_two_active_cardinality():
    """The preserved 96.52 s shape cannot regress to robot1-only IDLE."""
    union, first, second, first_id, second_id = _independent_cost_pair_fixture(
        'c-96.52', 1.965506, 3.517690,
    )
    decision = choose_pair_assignment(
        'c-96.52', union, first, second,
        scoring_mode='frontier_cost_only',
    )
    assert (decision.robot1_task_id, decision.robot2_task_id) == (
        first_id, second_id,
    )


def test_cost_only_one_active_survives_when_other_robot_has_no_feasible_bid():
    """One-active remains the fallback when no valid pair can be formed."""
    first = cost_task('only-robot1', (1.0, 0.0), 1.0, 0.0)
    union = build_canonical_union([first], [])
    first_id = union.tasks[0].canonical_id
    decision = choose_pair_assignment(
        'round-one-active', union,
        batch('robot1', union.union_hash, [
            bid(first_id, 1.0, [(0.0, 0.0), (1.0, 0.0)]),
        ], round_id='round-one-active'),
        batch('robot2', union.union_hash, [], round_id='round-one-active'),
        scoring_mode='frontier_cost_only',
    )
    assert (decision.robot1_task_id, decision.robot2_task_id) == (first_id, '')
    assert decision.diagnostics.valid_two_active_pair_count == 0


def test_cost_only_shared_single_task_assigns_one_robot_and_leaves_one_idle():
    """A shared one-task union is selectable without duplicate dispatch."""
    robot1_task = make_task(
        'robot1', 'same-physical-frontier-r1', (1.0, 0.0), (1.0, 0.0),
        Bounds((0.9, -0.1), (1.1, 0.1)), gain=1.0,
    )
    robot2_task = make_task(
        'robot2', 'same-physical-frontier-r2', (1.0, 0.0), (1.0, 0.0),
        Bounds((0.9, -0.1), (1.1, 0.1)), gain=1.0,
    )
    union = build_canonical_union([robot1_task], [robot2_task])
    assert len(union.tasks) == 1
    task_id = union.tasks[0].canonical_id

    decision = choose_pair_assignment(
        'shared-single-task', union,
        batch('robot1', union.union_hash, [
            bid(task_id, 1.0, [(0.0, 0.0), (1.0, 0.0)]),
        ], round_id='shared-single-task'),
        batch('robot2', union.union_hash, [
            bid(task_id, 2.0, [(0.0, 0.0), (1.0, 0.0)]),
        ], round_id='shared-single-task'),
        scoring_mode='frontier_cost_only',
    )

    assert {decision.robot1_task_id, decision.robot2_task_id} == {
        task_id, '',
    }
    assert decision.robot1_task_id != decision.robot2_task_id
    assert decision.diagnostics.valid_two_active_pair_count == 0
    assert decision.diagnostics.valid_one_active_assignment_count == 2


def test_cost_only_idle_remains_possible_when_nothing_is_feasible():
    """IDLE/IDLE remains the explicit result when every bid is invalid."""
    task = cost_task('invalid-both', (1.0, 0.0), 1.0, 0.0)
    union = build_canonical_union([task], [])
    task_id = union.tasks[0].canonical_id
    invalid = bid(task_id, 1.0, [(0.0, 0.0), (1.0, 0.0)], valid=False)
    decision = choose_pair_assignment(
        'round-no-work', union,
        batch('robot1', union.union_hash, [invalid], round_id='round-no-work'),
        batch('robot2', union.union_hash, [invalid], round_id='round-no-work'),
        scoring_mode='frontier_cost_only',
    )
    assert (decision.robot1_task_id, decision.robot2_task_id) == ('', '')
    assert decision.diagnostics.idle_idle_permitted is True


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


def _cost_only_certificate_fixture(evaluated_path_m):
    """One evaluated option plus an optional lower-bound-only option."""
    known = cost_task('evaluated', (1.0, 0.0), evaluated_path_m, 0.0)
    union = build_canonical_union([known], [])
    task_id = union.tasks[0].canonical_id
    first = batch('robot1', union.union_hash, [
        bid(task_id, evaluated_path_m, [(0.0, 0.0),
                                        (evaluated_path_m, 0.0)]),
    ])
    second = batch('robot2', union.union_hash, [])
    decision = choose_pair_assignment(
        'round', union, first, second,
        scoring_mode='frontier_cost_only',
    )
    return decision, first, second


def test_cost_only_certificate_blocks_near_unqueried_candidate():
    """A 4 m lower bound must block dispatch of a 16 m evaluated option."""
    decision, first, second = _cost_only_certificate_fixture(16.0)
    certified, blocking, optimistic, reason = cost_only_dispatch_certificate(
        decision, first, second, [4.0 / 0.13], [], AssignmentWeights(),
    )
    assert not certified
    assert blocking >= 1
    assert optimistic > decision.score.total
    assert reason == 'UNQUERIED_OPTION_CAN_BEAT_EVALUATED_ASSIGNMENT'


def test_cost_only_certificate_allows_dominated_unqueried_candidate():
    """A provably slower lower bound need not consume every query slot."""
    decision, first, second = _cost_only_certificate_fixture(16.0)
    certified, blocking, _optimistic, reason = cost_only_dispatch_certificate(
        decision, first, second, [200.0], [], AssignmentWeights(),
    )
    assert certified
    assert blocking == 0
    assert reason == 'ALL_UNQUERIED_OPTIONS_DOMINATED'


def test_cost_only_certificate_checks_unknown_pair_in_both_free_case():
    """An unknown/evaluated pair remains a possible winning assignment."""
    r1 = cost_task('r1-known', (1.0, 0.0), 16.0, 0.0)
    r2 = cost_task('r2-known', (2.0, 0.0), 16.0, 0.0)
    union = build_canonical_union([r1], [r2])
    ids = {m.physical_signature: t.canonical_id
           for t in union.tasks for m in t.members}
    first = batch('robot1', union.union_hash, [
        bid(ids['r1-known'], 16.0, [(0.0, 0.0), (16.0, 0.0)]),
    ])
    second = batch('robot2', union.union_hash, [
        bid(ids['r2-known'], 16.0, [(0.0, 0.0), (16.0, 0.0)]),
    ])
    decision = choose_pair_assignment(
        'round', union, first, second,
        scoring_mode='frontier_cost_only',
    )
    certified, _blocking, _optimistic, _reason = cost_only_dispatch_certificate(
        decision, first, second, [4.0 / 0.13], [4.0 / 0.13],
        AssignmentWeights(),
    )
    assert not certified


def test_terminal_bounds_reach_allocator_when_diagnostic_capture_is_off():
    """The production compact summary is independent of verbose diagnostics."""
    def ingest(diagnostic_payload):
        node = DistributedFrontierAssignment.__new__(DistributedFrontierAssignment)
        node._candidate_evidence = {
            'robot1': CandidateEvidence(), 'robot2': CandidateEvidence(),
        }
        node._candidate_evidence_seen = set()
        node._candidate_region_snapshots = {}
        node._unqueried_cost_bounds = {'robot1': None, 'robot2': None}
        node._unqueried_cost_bound_provenance = {}
        node._unqueried_cost_bounds_received = {}
        node._terminal_small_frontier_length_m = 0.2
        node._minimum_solo_visible_gain_m = 0.05
        message = FrontierCandidateArray()
        message.source_robot_id = 'robot1'
        message.candidate_generation_id = 7
        message.detected_frontier_count = 1
        message.detected_not_queried_count = 1
        message.unclassified_frontier_count = 1
        message.lower_bound_context_fingerprint = 'context-7'
        message.terminal_frontier_regions_json = json.dumps({
            'regions': [{
                'physical_id': 7, 'status': 'DETECTED_NOT_QUERIED',
                'size_m': 0.6, 'visible_reveal_gain': 0.2,
                'optimistic_cost_lower_bound_s': 30.0,
            }],
        })
        message.diagnostic_regions_json = diagnostic_payload
        node._candidate_callback(message)
        return node._unqueried_cost_bounds['robot1']

    assert ingest('') == (30.0,)
    assert ingest('{"regions": []}') == (30.0,)


def test_lower_bound_context_provenance_matches_snapshot_without_receipt_ttl():
    """An unchanged context remains valid after the former eight-second lease."""
    snapshot = TaskSnapshot(
        'robot1', 'session-1', 4, 27, 'map', 0, 2.0, (), 'context-27', 0, 7,
    )
    assert lower_bound_context_matches((27, 'context-27', 7), snapshot)
    assert not lower_bound_context_matches((26, 'context-27', 7), snapshot)
    assert not lower_bound_context_matches((27, 'context-old', 7), snapshot)
    assert not lower_bound_context_matches((27, 'context-27', 6), snapshot)


def test_cost_only_certificate_uses_matching_context_not_bound_receipt_age():
    """A matching lower-bound snapshot is accepted independent of wall age."""
    decision, first, second = _cost_only_certificate_fixture(16.0)
    node = DistributedFrontierAssignment.__new__(DistributedFrontierAssignment)
    node._assignment_strategy = 'frontier_cost_only'
    node._unqueried_cost_bounds = {
        'robot1': (4.0 / 0.13,), 'robot2': (),
    }
    node._unqueried_cost_bound_provenance = {
        'robot1': (1, 'r1-context', 7), 'robot2': (1, 'r2-context', 7),
    }
    node._unqueried_cost_bounds_received = {'robot1': 0.0, 'robot2': 0.0}
    node._candidate_evidence = {
        'robot1': CandidateEvidence(detected_not_queried=1),
        'robot2': CandidateEvidence(),
    }
    node._candidate_lower_bound_metadata = {
        'robot1': {
            'fingerprint': 'r1-context', 'map_revision': 1,
            'costmap_revision': 0, 'source_session_id': None,
            'source_epoch': None, 'generation_ros_ns': 0,
            'candidate_generation_id': 7, 'bound_entry_count': 1,
            'detected_not_queried_count': 1, 'all_bounds_finite': True,
            'bound_state': 'OK',
        },
        'robot2': {
            'fingerprint': 'r2-context', 'map_revision': 1,
            'costmap_revision': 0, 'source_session_id': None,
            'source_epoch': None, 'generation_ros_ns': 0,
            'candidate_generation_id': 7, 'bound_entry_count': 0,
            'detected_not_queried_count': 0, 'all_bounds_finite': True,
            'bound_state': 'OK',
        },
    }
    node._weights = AssignmentWeights()
    node._last_cost_only_certificate_key = None
    node._emit_event = lambda *_args, **_kwargs: None
    round_work = SimpleNamespace(
        round_id='round',
        mode='normal',
        snapshots=(
            TaskSnapshot('robot1', 's1', 1, 1, 'map', 0, 2.0, (), 'r1-context', 0, 7),
            TaskSnapshot('robot2', 's2', 1, 1, 'map', 0, 2.0, (), 'r2-context', 0, 7),
        ),
    )
    certified, blocking, _optimistic, reason = (
        node._cost_only_dispatch_certificate(
            round_work, decision, first, second,
        )
    )
    assert not certified
    assert blocking >= 1
    assert reason == 'UNQUERIED_OPTION_CAN_BEAT_EVALUATED_ASSIGNMENT'


def test_cost_only_certificate_blocks_context_mismatch_conservatively():
    """A changed context cannot reuse an otherwise well-formed old bound."""
    decision, first, second = _cost_only_certificate_fixture(16.0)
    node = DistributedFrontierAssignment.__new__(DistributedFrontierAssignment)
    node._assignment_strategy = 'frontier_cost_only'
    node._unqueried_cost_bounds = {'robot1': (200.0,), 'robot2': ()}
    node._unqueried_cost_bound_provenance = {
        'robot1': (1, 'old-context', 7), 'robot2': (1, 'r2-context', 7),
    }
    node._candidate_evidence = {
        'robot1': CandidateEvidence(detected_not_queried=1),
        'robot2': CandidateEvidence(),
    }
    node._candidate_lower_bound_metadata = {
        'robot1': {
            'fingerprint': 'old-context', 'map_revision': 1,
            'costmap_revision': 0, 'source_session_id': None,
            'source_epoch': None, 'generation_ros_ns': 0,
            'candidate_generation_id': 7, 'bound_entry_count': 1,
            'detected_not_queried_count': 1, 'all_bounds_finite': True,
            'bound_state': 'OK',
        },
        'robot2': {
            'fingerprint': 'r2-context', 'map_revision': 1,
            'costmap_revision': 0, 'source_session_id': None,
            'source_epoch': None, 'generation_ros_ns': 0,
            'candidate_generation_id': 7, 'bound_entry_count': 0,
            'detected_not_queried_count': 0, 'all_bounds_finite': True,
            'bound_state': 'OK',
        },
    }
    node._weights = AssignmentWeights()
    node._last_cost_only_certificate_key = None
    node._emit_event = lambda *_args, **_kwargs: None
    round_work = SimpleNamespace(
        round_id='round',
        mode='normal',
        snapshots=(
            TaskSnapshot('robot1', 's1', 1, 1, 'map', 0, 2.0, (), 'new-context', 0, 7),
            TaskSnapshot('robot2', 's2', 1, 1, 'map', 0, 2.0, (), 'r2-context', 0, 7),
        ),
    )
    certified, blocking, optimistic, reason = (
        node._cost_only_dispatch_certificate(
            round_work, decision, first, second,
        )
    )
    assert not certified
    assert blocking == 0
    assert optimistic == float('-inf')
    assert reason == 'MISSING_UNQUERIED_LOWER_BOUNDS'


def test_cost_only_certificate_requires_missing_bounds_conservatively():
    """Missing genuinely required production evidence still blocks dispatch."""
    decision, first, second = _cost_only_certificate_fixture(16.0)
    certified, blocking, optimistic, reason = cost_only_dispatch_certificate(
        decision, first, second, None, [], AssignmentWeights(),
    )
    assert not certified
    assert blocking == 0
    assert optimistic == float('-inf')
    assert reason == 'MISSING_UNQUERIED_LOWER_BOUNDS'


def _bound_diag_fixture(**overrides):
    meta = {
        'fingerprint': 'fp', 'map_revision': 7, 'costmap_revision': 9,
        'source_session_id': None, 'source_epoch': None,
        'generation_ros_ns': 100, 'bound_entry_count': 1,
        'candidate_generation_id': 7,
        'detected_not_queried_count': 1, 'all_bounds_finite': True,
        'bound_state': 'OK',
    }
    meta.update(overrides)
    snapshot = TaskSnapshot(
        'robot1', 'session-1', 3, 7, 'map', 100, 2.0, (), 'fp', 9, 7,
    )
    return meta, snapshot


def test_certificate_telemetry_classifies_missing_and_empty_summary():
    meta, snapshot = _bound_diag_fixture(
        bound_state='NO_CANDIDATE_BOUND_SUMMARY', bound_entry_count=0,
        all_bounds_finite=False,
    )
    reason, _ = classify_lower_bound_evidence(meta, snapshot, None, 1)
    assert reason == 'NO_CANDIDATE_BOUND_SUMMARY'
    reason, _ = classify_lower_bound_evidence(None, snapshot, None, 1)
    assert reason == 'NO_CANDIDATE_BOUND_SUMMARY'
    meta['bound_state'] = 'EMPTY_BOUND_SUMMARY'
    meta['detected_not_queried_count'] = 0
    reason, _ = classify_lower_bound_evidence(meta, snapshot, None, 0)
    assert reason == 'EMPTY_BOUND_SUMMARY'


def test_certificate_telemetry_classifies_nonfinite_and_incomplete_bounds():
    meta, snapshot = _bound_diag_fixture(
        bound_state='NONFINITE_BOUND', all_bounds_finite=False,
    )
    reason, _ = classify_lower_bound_evidence(meta, snapshot, (float('nan'),), 1)
    assert reason == 'NONFINITE_BOUND'
    meta['bound_state'] = 'INCOMPLETE_BOUND_SET'
    meta['bound_entry_count'] = 0
    reason, _ = classify_lower_bound_evidence(meta, snapshot, None, 1)
    assert reason == 'INCOMPLETE_BOUND_SET'


def test_certificate_telemetry_classifies_provenance_mismatches():
    meta, snapshot = _bound_diag_fixture(fingerprint='other')
    reason, _ = classify_lower_bound_evidence(meta, snapshot, (1.0,), 1)
    assert reason == 'FINGERPRINT_MISMATCH'
    meta, snapshot = _bound_diag_fixture(map_revision=8)
    reason, _ = classify_lower_bound_evidence(meta, snapshot, (1.0,), 1)
    assert reason == 'MAP_REVISION_MISMATCH'
    meta, snapshot = _bound_diag_fixture(costmap_revision=8)
    reason, _ = classify_lower_bound_evidence(meta, snapshot, (1.0,), 1)
    assert reason == 'COSTMAP_REVISION_MISMATCH'
    meta, snapshot = _bound_diag_fixture(source_session_id='other')
    reason, _ = classify_lower_bound_evidence(meta, snapshot, (1.0,), 1)
    assert reason == 'SESSION_EPOCH_MISMATCH'


def test_certificate_telemetry_reports_matching_evidence_as_ok():
    meta, snapshot = _bound_diag_fixture()
    reason, comparison = classify_lower_bound_evidence(
        meta, snapshot, (1.0,), 1,
    )
    assert reason == 'OK'
    assert comparison['fingerprints_equal']
    assert comparison['revisions_equal']
    assert comparison['bound_set_complete']


def test_one_busy_continuation_only_requires_free_robot_bounds():
    """A busy commitment is represented by an empty bound set."""
    decision, first, second = _cost_only_certificate_fixture(16.0)
    certified, blocking, _optimistic, reason = cost_only_dispatch_certificate(
        decision, first, second, [200.0], [], AssignmentWeights(),
    )
    assert certified
    assert blocking == 0
    assert reason == 'ALL_UNQUERIED_OPTIONS_DOMINATED'


def _certificate_node_with_provenance(candidate_generation_id=7,
                                      task_generation_id=7):
    """Build a small ROS-free certificate fixture with two source records."""
    decision, first, second = _cost_only_certificate_fixture(16.0)
    node = DistributedFrontierAssignment.__new__(DistributedFrontierAssignment)
    node._assignment_strategy = 'frontier_cost_only'
    node._unqueried_cost_bounds = {
        'robot1': (4.0 / 0.13,), 'robot2': (),
    }
    node._unqueried_cost_bound_provenance = {
        'robot1': (1, 'r1-context', candidate_generation_id),
        'robot2': (1, 'r2-context', 7),
    }
    node._candidate_source_local_evidence = {
        'robot1': CandidateEvidence(detected_not_queried=1),
        'robot2': CandidateEvidence(),
    }
    node._candidate_evidence = dict(node._candidate_source_local_evidence)
    node._candidate_lower_bound_metadata = {
        'robot1': {
            'fingerprint': 'r1-context', 'map_revision': 1,
            'costmap_revision': 0, 'source_session_id': None,
            'source_epoch': None, 'generation_ros_ns': 100,
            'candidate_generation_id': candidate_generation_id,
            'bound_entry_count': 1, 'detected_not_queried_count': 1,
            'all_bounds_finite': True, 'bound_state': 'OK',
        },
        'robot2': {
            'fingerprint': 'r2-context', 'map_revision': 1,
            'costmap_revision': 0, 'source_session_id': None,
            'source_epoch': None, 'generation_ros_ns': 100,
            'candidate_generation_id': 7,
            'bound_entry_count': 0, 'detected_not_queried_count': 0,
            'all_bounds_finite': True, 'bound_state': 'OK',
        },
    }
    node._weights = AssignmentWeights()
    node._last_cost_only_certificate_key = None
    node._emit_event = lambda *_args, **_kwargs: None
    snapshots = (
        TaskSnapshot('robot1', 's1', 1, 1, 'map', 0, 2.0, (), 'r1-context', 0,
                     task_generation_id),
        TaskSnapshot('robot2', 's2', 1, 1, 'map', 0, 2.0, (), 'r2-context', 0,
                     7),
    )
    return node, decision, first, second, SimpleNamespace(
        round_id='round', mode='normal', snapshots=snapshots,
    )


def test_source_local_certificate_does_not_use_merged_peer_count():
    """R1's 76 bounds are checked against R1's 76 DNU items only."""
    node = DistributedFrontierAssignment.__new__(DistributedFrontierAssignment)
    node._candidate_evidence = {
        'robot1': CandidateEvidence(), 'robot2': CandidateEvidence(),
    }
    node._candidate_source_local_evidence = {
        'robot1': CandidateEvidence(), 'robot2': CandidateEvidence(),
    }
    node._candidate_evidence_seen = set()
    node._candidate_region_snapshots = {}
    node._unqueried_cost_bounds = {'robot1': None, 'robot2': None}
    node._unqueried_cost_bound_provenance = {}
    node._unqueried_cost_bounds_received = {}
    node._candidate_lower_bound_metadata = {}
    node._terminal_small_frontier_length_m = 0.2
    node._minimum_solo_visible_gain_m = 0.05

    def message(robot, generation, count, fingerprint):
        value = FrontierCandidateArray()
        value.source_robot_id = robot
        value.candidate_generation_id = generation
        value.map_revision = 1
        value.costmap_revision = 1
        value.detected_frontier_count = count
        value.detected_not_queried_count = count
        value.unclassified_frontier_count = count
        value.lower_bound_context_fingerprint = fingerprint
        value.terminal_frontier_regions_json = json.dumps({
            'regions': [
                {'physical_id': index, 'status': 'DETECTED_NOT_QUERIED',
                 'size_m': 0.6, 'visible_reveal_gain': 0.2,
                 'optimistic_cost_lower_bound_s': 20.0}
                for index in range(
                    (0 if robot == 'robot1' else 1000),
                    (0 if robot == 'robot1' else 1000) + count,
                )
            ],
        })
        return value

    node._candidate_callback(message('robot1', 1, 76, 'r1'))
    node._candidate_callback(message('robot2', 1, 39, 'r2'))
    assert node._candidate_source_local_evidence['robot1'].detected_not_queried == 76
    assert node._candidate_source_local_evidence['robot2'].detected_not_queried == 39
    assert node._candidate_evidence['robot1'].detected_not_queried == 115
    assert node._unqueried_cost_bounds['robot1'] == (20.0,) * 76
    emitted = []
    decision, first, second = _cost_only_certificate_fixture(16.0)
    node._assignment_strategy = 'frontier_cost_only'
    node._weights = AssignmentWeights()
    node._last_cost_only_certificate_key = None
    node._emit_event = lambda event_type, message, **_kwargs: emitted.append(
        (event_type, json.loads(message)))
    round_work = SimpleNamespace(
        round_id='source-local-round', mode='normal', snapshots=(
            TaskSnapshot('robot1', 's1', 1, 1, 'map', 0, 2.0, (), 'r1', 1, 1),
            TaskSnapshot('robot2', 's2', 1, 1, 'map', 0, 2.0, (), 'r2', 1, 1),
        ),
    )
    node._cost_only_dispatch_certificate(round_work, decision, first, second)
    payload = emitted[-1][1]
    comparison = payload['provenance_comparison']
    assert comparison['robot1']['candidate']['bound_entry_count'] == 76
    assert comparison['robot1']['candidate']['expected_bound_entry_count'] == 76
    assert comparison['robot1']['bound_set_complete']
    assert comparison['robot2']['candidate']['expected_bound_entry_count'] == 39
    assert comparison['robot2']['bound_set_complete']


def test_candidate_generation_mismatch_defers_then_matching_snapshot_rechecks():
    """A newer candidate cannot be joined to an older task snapshot."""
    node, decision, first, second, round_work = (
        _certificate_node_with_provenance(candidate_generation_id=8,
                                           task_generation_id=7)
    )
    emitted = []
    node._emit_event = lambda event_type, message, **_kwargs: emitted.append(
        (event_type, json.loads(message)))
    certified, blocking, _optimistic, reason = node._cost_only_dispatch_certificate(
        round_work, decision, first, second,
    )
    assert not certified
    assert blocking == 0
    assert reason == 'MISSING_UNQUERIED_LOWER_BOUNDS'
    assert emitted[-1][1]['evidence_reason'] == (
        'TASK_CANDIDATE_GENERATION_MISMATCH')
    round_work.snapshots = (
        replace(round_work.snapshots[0], candidate_generation_id=8),
        round_work.snapshots[1],
    )
    certified, blocking, _optimistic, reason = node._cost_only_dispatch_certificate(
        round_work, decision, first, second,
    )
    assert not certified
    assert blocking >= 1
    assert reason == 'UNQUERIED_OPTION_CAN_BEAT_EVALUATED_ASSIGNMENT'


def test_missing_candidate_generation_id_blocks_conservatively():
    meta, snapshot = _bound_diag_fixture(
        candidate_generation_id=0,
    )
    snapshot = replace(snapshot, candidate_generation_id=0)
    reason, comparison = classify_lower_bound_evidence(
        meta, snapshot, (1.0,), 1,
    )
    assert reason == 'TASK_CANDIDATE_GENERATION_MISMATCH'
    assert comparison['bound_set_complete']


def test_adapter_copies_candidate_generation_id_unchanged():
    """The adapter must not reconstruct or alter the source generation ID."""
    from my_epuck_project.frontier_proposal_adapter import FrontierProposalAdapter

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    node = FrontierProposalAdapter.__new__(FrontierProposalAdapter)
    node._robot_id = 'robot1'
    node._stopped_after_handoff = False
    node._epoch = 0
    node._session_text = '11' * 16
    from my_epuck_project.distributed_assignment.ros_conversion import text_to_uuid
    node._session_uuid = text_to_uuid(node._session_text)
    node._maximum_tasks = 5
    node._validity_s = 8.0
    node._signature_quantum_m = 0.05
    node._publisher = Publisher()
    node.get_logger = lambda: SimpleNamespace(info=lambda *_args: None)
    message = FrontierCandidateArray()
    message.source_robot_id = 'robot1'
    message.candidate_generation_id = 42
    node._on_candidates(message)
    assert node._publisher.messages[0].candidate_generation_id == 42
    from my_epuck_project.distributed_assignment.ros_conversion import snapshot_from_msg
    snapshot = snapshot_from_msg(node._publisher.messages[0])
    assert snapshot.candidate_generation_id == 42
