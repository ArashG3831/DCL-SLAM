"""Focused tests for exact local planner-result reuse across rounds."""

from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from my_epuck_project.distributed_assignment.canonical import (
    build_canonical_union,
)
from my_epuck_project.distributed_assignment.local_nav2 import PathEvaluation
from my_epuck_project.distributed_assignment.models import (
    Bid,
    BidBatch,
    Bounds,
    FailureClass,
    PhysicalTask,
    TaskSnapshot,
)
from my_epuck_project.distributed_assignment.protocol import receive
from my_epuck_project.distributed_assignment.scoring import AssignmentWeights
from my_epuck_project.distributed_frontier_assignment import (
    DistributedFrontierAssignment,
    RoundWork,
    LOCAL_PATH_EVALUATION_CACHE_MAX_ENTRIES,
)
from my_epuck_project.round_lifecycle import RoundGeneration


SOURCE = Path(__file__).parents[1] / 'my_epuck_project' / (
    'distributed_frontier_assignment.py'
)


def _physical(
        signature='frontier', *, epoch=1, map_revision=1,
        approach=(1.0, 0.0), approach_yaw=0.0) -> PhysicalTask:
    """Create one source-consistent task without an embedded path."""
    return PhysicalTask(
        'robot1', 'session-1', epoch, map_revision, signature, 1,
        approach, Bounds(
            (approach[0] - 0.1, approach[1] - 0.1),
            (approach[0] + 0.1, approach[1] + 0.1),
        ), approach,
        approach_yaw=approach_yaw,
        frontier_geometry=(approach,),
        visible_reveal_gain=1.0,
        local_ordering_score=1.0,
        generation_ros_ns=100,
    )


def _round_for(
        task: PhysicalTask, *, round_id='round-1', candidate_generation_id=7,
        costmap_revision=3) -> tuple[RoundWork, object]:
    """Build a normal round carrying the task's source provenance."""
    union = build_canonical_union((task,), ())
    snapshot = TaskSnapshot(
        'robot1', task.source_session_id, task.source_snapshot_epoch,
        task.source_map_revision, 'map-%d' % task.source_map_revision,
        task.generation_ros_ns, 8.0, (task,), 'bounds-context',
        costmap_revision, candidate_generation_id,
    )
    peer_snapshot = TaskSnapshot(
        'robot2', 'session-2', 1, 1, 'peer-map', 100, 8.0, (),
    )
    return RoundWork(
        round_id=round_id,
        union=union,
        snapshots=(snapshot, peer_snapshot),
        query_tasks=union.tasks,
    ), union.tasks[0]


def _evaluation(task, *, valid=True, length=1.0, error_code=0,
                map_stamp_ns=10, costmap_stamp_ns=20,
                heading_cost=0.1) -> PathEvaluation:
    """Create an allocator-tagged result suitable for cache tests."""
    return PathEvaluation(
        valid=valid,
        length_m=length,
        samples=((0.0, 0.0), (length if valid else 0.0, 0.0)),
        query_ros_ns=123,
        error_code=error_code,
        error_message='' if valid else 'planner failure',
        failure_class=FailureClass.UNKNOWN if valid else FailureClass.ACTION_REJECTION,
        caller='ALLOCATOR_BID',
        task_signature=task.members[0].physical_signature,
        map_stamp_ns=map_stamp_ns,
        costmap_stamp_ns=costmap_stamp_ns,
        heading_cost=heading_cost,
    )


def _node(*, path_context_matches=True):
    """Build the narrow ROS-free shell used by the allocator helpers."""
    node = DistributedFrontierAssignment.__new__(DistributedFrontierAssignment)
    node._local_path_evaluation_cache = {}
    node._local_path_cache_hits = 0
    node._local_path_cache_misses = 0
    node._local_path_cache_invalidations = 0
    node._local_path_cache_evictions = 0
    node._nav2 = SimpleNamespace(
        path_context_matches=lambda _result: path_context_matches,
        evaluate_path=lambda *_args, **_kwargs: pytest.fail(
            'cache hit unexpectedly started a planner query'),
    )
    node._robot_id = 'robot1'
    node._synthetic_bids = False
    node._assignment_strategy = 'frontier_cost_only'
    node._weights = AssignmentWeights()
    node._continue_bidding = lambda *_args, **_kwargs: None
    node._bid_validity_s = 3.0
    node._publish_local_bid_batch = lambda *_args, **_kwargs: None
    return node


def test_exact_task_and_context_hit_creates_a_new_round_bid():
    """A cache hit supplies a path, while the new RoundWork gets the bid."""
    task = _physical()
    old_round, canonical = _round_for(task, round_id='old-round')
    node = _node()
    node._remember_local_path_evaluation(
        old_round, canonical, _evaluation(canonical),
    )

    new_round, new_canonical = _round_for(task, round_id='new-round')
    node._round_lifecycle = RoundGeneration()
    generation = node._round_lifecycle.activate(new_round.round_id).generation
    node._round = new_round
    node._continue_bidding_impl(new_round, generation)

    assert node._local_path_cache_hits == 1
    assert node._local_path_cache_misses == 0
    assert old_round.bids == ()
    assert len(new_round.bids) == 1
    assert new_round.bids[0].canonical_task_id == new_canonical.canonical_id
    assert new_round.local_path_evaluations[new_canonical.canonical_id].query_ros_ns == 123
    node._finish_bids(new_round, generation)
    assert new_round.local_batch.round_id == 'new-round'
    assert new_round.local_batch.union_hash == new_round.union.union_hash
    assert new_round.local_batch.source_snapshot_epoch == 1


@pytest.mark.parametrize('change', ('geometry', 'orientation', 'epoch',
                                    'candidate_generation', 'map_revision'))
def test_changed_execution_or_provenance_key_misses(change):
    """Any path/provenance change prevents exact cache reuse."""
    base_task = _physical()
    base_round, base_canonical = _round_for(base_task)
    node = _node()
    node._remember_local_path_evaluation(
        base_round, base_canonical, _evaluation(base_canonical),
    )

    if change == 'geometry':
        changed_task = replace(base_task, approach=(2.0, 0.0),
                               centroid=(2.0, 0.0),
                               bounds=Bounds((1.9, -0.1), (2.1, 0.1)),
                               frontier_geometry=((2.0, 0.0),))
        changed_round, changed_canonical = _round_for(changed_task)
    elif change == 'orientation':
        changed_task = replace(base_task, approach_yaw=0.7)
        changed_round, changed_canonical = _round_for(changed_task)
    elif change == 'epoch':
        changed_task = replace(base_task, source_snapshot_epoch=2)
        changed_round, changed_canonical = _round_for(changed_task)
    elif change == 'candidate_generation':
        changed_round, changed_canonical = _round_for(
            base_task, candidate_generation_id=8,
        )
    else:
        changed_task = replace(base_task, source_map_revision=2)
        changed_round, changed_canonical = _round_for(changed_task)

    assert node._local_path_execution_key(
        changed_round, changed_canonical,
    ) != node._local_path_execution_key(base_round, base_canonical)
    assert node._cached_local_path_evaluation(
        changed_round, changed_canonical,
    ) is None
    assert node._local_path_cache_hits == 0
    assert node._local_path_cache_misses == 1


def test_changed_costmap_context_invalidates_cached_path():
    """A stale current map/costmap context removes the cached result."""
    task = _physical()
    round_work, canonical = _round_for(task)
    node = _node(path_context_matches=False)
    node._remember_local_path_evaluation(
        round_work, canonical, _evaluation(canonical),
    )

    assert node._cached_local_path_evaluation(round_work, canonical) is None
    assert node._local_path_cache_invalidations == 1
    assert node._local_path_evaluation_cache == {}


@pytest.mark.parametrize('result', [
    _evaluation(_round_for(_physical())[1], valid=False, error_code=1),
    _evaluation(_round_for(_physical())[1], length=float('nan')),
    _evaluation(_round_for(_physical())[1], length=float('inf')),
    _evaluation(_round_for(_physical())[1], heading_cost=float('nan')),
    _evaluation(_round_for(_physical())[1], map_stamp_ns=0),
])
def test_invalid_or_nonfinite_planner_result_is_never_stored(result):
    """Failed, aborted, nonfinite, or contextless results are not cached."""
    task = _physical()
    round_work, canonical = _round_for(task)
    node = _node()
    node._remember_local_path_evaluation(round_work, canonical, result)
    assert node._local_path_evaluation_cache == {}


def test_round_replacement_does_not_reuse_old_peer_batch_or_decision():
    """Cache reuse rebuilds only the local bid for the current round."""
    task = _physical()
    old_round, old_canonical = _round_for(task, round_id='old-round')
    new_round, new_canonical = _round_for(task, round_id='new-round')
    node = _node()
    node._remember_local_path_evaluation(
        old_round, old_canonical, _evaluation(old_canonical),
    )
    old_peer_batch = BidBatch(
        'old-round', old_round.union.union_hash, 'robot2', 'session-2', 1,
        3.0, (Bid('old-task', True, 1.0, 1.0),),
    )
    node._bid_batches = {'robot2': receive(old_peer_batch, 3.0, 100.0)}
    node._peer_decision = SimpleNamespace(round_id='old-round')
    node._round_lifecycle = RoundGeneration()
    generation = node._round_lifecycle.activate(new_round.round_id).generation
    node._round = new_round
    node._continue_bidding_impl(new_round, generation)

    assert len(new_round.bids) == 1
    assert new_round.round_id == 'new-round'
    assert new_round.local_batch is None
    assert node._bid_batches['robot2'].value.round_id == 'old-round'
    assert node._peer_decision.round_id == 'old-round'


def test_cache_is_bounded_with_deterministic_fifo_eviction():
    """The oldest insertion is evicted at the fixed cache bound."""
    node = _node()
    rounds = []
    for index in range(LOCAL_PATH_EVALUATION_CACHE_MAX_ENTRIES + 1):
        task = _physical(signature='frontier-%d' % index)
        round_work, canonical = _round_for(
            task, candidate_generation_id=index + 1,
        )
        node._remember_local_path_evaluation(
            round_work, canonical, _evaluation(canonical),
        )
        rounds.append((round_work, canonical))

    assert len(node._local_path_evaluation_cache) == (
        LOCAL_PATH_EVALUATION_CACHE_MAX_ENTRIES)
    assert node._local_path_cache_evictions == 1
    assert node._cached_local_path_evaluation(*rounds[0]) is None
    assert node._cached_local_path_evaluation(*rounds[-1]) is not None


def test_existing_final_dispatch_safety_checks_remain_in_place():
    """The cache does not bypass the established final dispatch gates."""
    source = SOURCE.read_text(encoding='utf-8')
    dispatch = source[source.index('    def _start_local_dispatch'):
                      source.index('    def _dispatch_after_checks')]
    assert 'check_dispatch_preconditions' in dispatch
    assert 'path_context_matches' in dispatch
    assert 'if not path_is_valid_finite(final_path):' in source
    assert 'send_navigation' in source
