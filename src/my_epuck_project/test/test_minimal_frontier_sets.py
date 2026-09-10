"""Focused tests for the minimal coordinator's current frontier-set stage."""

from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path

import pytest
from geometry_msgs.msg import Point
from my_epuck_interfaces.msg import FrontierCandidate, FrontierCandidateArray

from my_epuck_project.minimal_frontier_sets import (
    build_union,
    compatible_context,
    materialize_bids,
    materialize_costs,
)


def _candidate(
        frontier_id, *, reachable=True, path_length=1.0,
        heading=0.25, path=((0.0, 0.0), (1.0, 0.0))):
    candidate = FrontierCandidate()
    candidate.frontier_id = frontier_id
    candidate.centroid = Point(x=float(frontier_id), y=1.0, z=0.0)
    candidate.bounding_box_min = Point(x=0.0, y=0.0, z=0.0)
    candidate.bounding_box_max = Point(x=1.0, y=1.0, z=0.0)
    candidate.approach_pose.pose.position = Point(
        x=float(frontier_id), y=0.5, z=0.0)
    candidate.information_gain = 2.0
    candidate.score = 3.0
    candidate.reachability_state = (
        FrontierCandidate.REACHABLE if reachable else FrontierCandidate.UNKNOWN)
    candidate.path_length_m = path_length
    candidate.heading_change_rad = heading
    candidate.local_path_samples = [Point(x=x, y=y, z=0.0) for x, y in path]
    return candidate


def _array(source, ids=(), *, frame='map', map_revision=7,
           costmap_revision=1, generation=10, candidates=None):
    message = FrontierCandidateArray()
    message.header.frame_id = frame
    message.source_robot_id = source
    message.map_revision = map_revision
    message.costmap_revision = costmap_revision
    message.candidate_generation_id = generation
    message.candidates = list(candidates) if candidates is not None else [
        _candidate(frontier_id) for frontier_id in ids]
    return message


def _task_ids(union):
    return tuple(task.canonical_id for task in union.tasks)


def test_compatible_context_accepts_matching_context_with_different_raw_metadata():
    robot1 = _array('robot1', (30, 2), costmap_revision=11, generation=101)
    robot2 = _array('robot2', (10,), costmap_revision=29, generation=909)

    assert compatible_context(robot1, robot2)
    union = build_union(robot1, robot2)
    assert union is not None
    assert _task_ids(union) == ('2', '10', '30')


def test_cost_only_wire_uses_shared_content_identity_not_local_counter():
    source = (Path(__file__).parents[3] /
              'src/my_epuck_frontier_candidates/src/frontier_candidate_generator.cpp').read_text(
                  encoding='utf-8')
    assert 'selection_policy_ == "frontier_cost_only"' in source
    assert 'map_checksum(*cycle_map_)' in source


def test_shared_content_identity_accepts_local_metadata_skew():
    robot1 = _array('robot1', (1,), map_revision=0x1234,
                    costmap_revision=3, generation=11)
    robot2 = _array('robot2', (2,), map_revision=0x1234,
                    costmap_revision=9, generation=27)
    robot1.header.stamp.sec = 10
    robot2.header.stamp.sec = 11
    robot1.map_stamp.sec = 20
    robot2.map_stamp.sec = 21

    assert compatible_context(robot1, robot2)
    assert _task_ids(build_union(robot1, robot2)) == ('1', '2')


@pytest.mark.parametrize(
    ('robot1_changes', 'robot2_changes'),
    [
        ({'frame': 'map_a'}, {'frame': 'map_b'}),
        ({'map_revision': 7}, {'map_revision': 8}),
    ],
)
def test_incompatible_frame_or_map_context_is_rejected(
        robot1_changes, robot2_changes):
    robot1 = _array('robot1', (1,), **robot1_changes)
    robot2 = _array('robot2', (2,), **robot2_changes)

    assert not compatible_context(robot1, robot2)
    assert build_union(robot1, robot2) is None


def test_union_contains_all_ids_and_uses_specified_order_and_digest():
    robot1 = _array('robot1', (30, 2))
    robot2 = _array('robot2', (10, 30))

    union = build_union(robot1, robot2)

    assert union is not None
    assert _task_ids(union) == ('2', '10', '30')
    assert len(union.tasks[2].members) == 2
    expected_payload = {
        'frame_id': 'map',
        'map_revision': 7,
        'union_ids': [2, 10, 30],
    }
    expected_hash = hashlib.sha256(
        json.dumps(
            expected_payload, sort_keys=True, separators=(',', ':'),
            allow_nan=False).encode('utf-8')).hexdigest()
    assert union.union_hash == expected_hash


def test_same_inputs_have_same_hash_and_union_change_changes_hash():
    robot1 = _array('robot1', (1, 4), candidates=[
        _candidate(4), _candidate(1)])
    robot2 = _array('robot2', (3,))

    first = build_union(robot1, robot2)
    repeated = build_union(robot1, robot2)
    changed = build_union(robot1, _array('robot2', (3, 5)))

    assert first is not None and repeated is not None and changed is not None
    assert first.union_hash == repeated.union_hash
    assert first.union_hash != changed.union_hash
    assert _task_ids(changed) == ('1', '3', '4', '5')


def test_materialization_uses_union_and_marks_missing_local_candidates_unavailable():
    robot1 = _array('robot1', candidates=[_candidate(2, path_length=2.0)])
    robot2 = _array('robot2', candidates=[_candidate(3, path_length=3.0)])
    union = build_union(robot1, robot2)
    assert union is not None

    costs = materialize_costs(robot1, union)
    bids = {bid.canonical_task_id: bid for bid in materialize_bids(robot1, union)}

    assert costs == {'2': 2.0, '3': math.inf}
    assert bids['2'].path_valid
    assert bids['2'].path_length_m == 2.0
    assert not bids['3'].path_valid
    assert bids['3'].path_length_m == 0.0
    assert bids['3'].estimated_travel_cost == 0.0
    assert bids['3'].heading_cost == 0.0
    assert bids['3'].path == ()


def test_unreachable_invalid_and_nonfinite_local_paths_are_unavailable():
    robot1 = _array('robot1', candidates=[
        _candidate(1, reachable=False, path_length=1.0),
        _candidate(2, path_length=-1.0),
        _candidate(3, path_length=math.nan),
        _candidate(4, path_length=math.inf),
    ])
    robot2 = _array('robot2', candidates=[])
    union = build_union(robot1, robot2)
    assert union is not None

    costs = materialize_costs(robot1, union)
    bids = materialize_bids(robot1, union)

    assert all(math.isinf(costs[str(frontier_id)]) for frontier_id in range(1, 5))
    assert tuple(bid.canonical_task_id for bid in bids) == ('1', '2', '3', '4')
    assert all(not bid.path_valid for bid in bids)
    assert all(bid.path_length_m == 0.0 for bid in bids)
    assert all(bid.estimated_travel_cost == 0.0 for bid in bids)
    assert all(math.isfinite(bid.heading_cost) for bid in bids)
    assert all(bid.path == () for bid in bids)


def test_duplicate_frontier_ids_within_one_source_are_rejected():
    robot1 = _array('robot1', candidates=[_candidate(8), _candidate(8)])
    robot2 = _array('robot2', candidates=[])

    with pytest.raises(ValueError, match='duplicate frontier_id'):
        build_union(robot1, robot2)


def test_ambiguous_or_duplicate_canonical_ids_are_rejected():
    robot1 = _array('robot1', (1,))
    robot2 = _array('robot2', candidates=[])
    union = build_union(robot1, robot2)
    assert union is not None

    duplicate = replace(union, tasks=(union.tasks[0], union.tasks[0]))
    ambiguous = replace(
        union, tasks=(replace(union.tasks[0], canonical_id='01'),))

    with pytest.raises(ValueError, match='duplicate or ambiguous'):
        materialize_costs(robot1, duplicate)
    with pytest.raises(ValueError, match='duplicate or ambiguous'):
        materialize_costs(robot1, ambiguous)


def test_current_union_does_not_retain_tasks_or_hash_from_prior_call():
    old_robot1 = _array('robot1', (1, 2))
    old_robot2 = _array('robot2', (3,))
    new_robot1 = _array('robot1', (9,))
    new_robot2 = _array('robot2', (10,))

    old_union = build_union(old_robot1, old_robot2)
    new_union = build_union(new_robot1, new_robot2)
    old_again = build_union(old_robot1, old_robot2)

    assert old_union is not None and new_union is not None and old_again is not None
    assert _task_ids(new_union) == ('9', '10')
    assert new_union.union_hash != old_union.union_hash
    assert old_again == old_union
