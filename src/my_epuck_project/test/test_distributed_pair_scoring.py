"""Regressions for exhaustive deterministic two-robot pair assignment."""

from my_epuck_project.distributed_assignment.canonical import build_canonical_union
from my_epuck_project.distributed_assignment.models import (
    Bid,
    BidBatch,
    Bounds,
    PhysicalTask,
)
from my_epuck_project.distributed_assignment.scoring import (
    AssignmentWeights,
    choose_pair_assignment,
    decisions_match,
    nearby_goal_penalty,
    route_overlap,
)


def make_task(robot, signature, approach, centroid, bounds, gain=5.0, local_id=1):
    """Create a physical proposal fixture."""
    return PhysicalTask(
        robot, 's1' if robot == 'robot1' else 's2', 1, 1,
        signature, local_id, centroid, bounds, approach,
        visible_reveal_gain=gain, local_path_valid=True,
    )


def batch(robot, union_hash, bids):
    """Create a correctly round-bound bid batch."""
    return BidBatch(
        'round', union_hash, robot,
        's1' if robot == 'robot1' else 's2', 1, 2.0, tuple(bids),
    )


def bid(task_id, length, path, valid=True):
    """Create a bounded path bid fixture."""
    return Bid(task_id, valid, length, length, path=tuple(path))


def test_route_overlap_and_nearby_penalties_are_bounded():
    """Keep assignment geometry terms within zero and one."""
    assert route_overlap([(0, 0), (0, 1)], [(0.1, 0), (0.1, 1)], 0.1) == 1.0
    assert route_overlap([(0, 0)], [(5, 5)], 0.1) == 0.0
    assert nearby_goal_penalty((0, 0), (0.2, 0), 0.5) > 0.5


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
