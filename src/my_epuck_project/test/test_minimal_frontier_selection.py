"""Focused tests for the minimal coordinator's pure selection boundary."""

import math
from types import SimpleNamespace

import pytest

from my_epuck_project import minimal_frontier_selection as selection
from my_epuck_project.distributed_assignment.models import (
    Bid,
    BidBatch,
    Bounds,
    CanonicalTask,
    CanonicalUnion,
    PhysicalTask,
)


UNION_HASH = 'minimal-union'


def _task(task_id, x, *, visible_gain=0.0):
    """Create a canonical task without ROS or coordinator state."""
    point = (float(x), 0.0)
    bounds = Bounds((float(x) - 0.1, -0.1), (float(x) + 0.1, 0.1))
    member = PhysicalTask(
        source_robot_id='robot1',
        source_session_id='',
        source_snapshot_epoch=1,
        source_map_revision=1,
        physical_signature=task_id,
        local_frontier_id=1,
        centroid=point,
        bounds=bounds,
        approach=point,
        visible_reveal_gain=visible_gain,
        local_path_valid=True,
    )
    return CanonicalTask(
        canonical_id=task_id,
        members=(member,),
        centroid=point,
        bounds=bounds,
        approach=point,
        approach_yaw=0.0,
        frontier_geometry=(),
        visible_cells=(),
        visible_bounds=None,
        visible_reveal_gain=visible_gain,
    )


def _union(*task_specs):
    """Build a deterministic union from ``(task_id, x[, gain])`` specs."""
    return CanonicalUnion(
        tasks=tuple(
            _task(task_id, x, visible_gain=gain[0] if gain else 0.0)
            for task_id, x, *gain in task_specs
        ),
        union_hash=UNION_HASH,
    )


def _bid(
        task_id, length=1.0, *, valid=True, travel=None, heading=0.0,
        path=None):
    """Create a bid with finite, nonempty path evidence by default."""
    if path is None:
        path = ((0.0, 0.0), (0.1, 0.0))
    return Bid(
        canonical_task_id=task_id,
        path_valid=valid,
        path_length_m=length,
        estimated_travel_cost=length if travel is None else travel,
        heading_cost=heading,
        path=tuple(path),
    )


def _batch(robot_id, union, *bids):
    """Bind fixture bids to the selector's expected round and robot."""
    return BidBatch(
        round_id=union.union_hash,
        union_hash=union.union_hash,
        source_robot_id=robot_id,
        source_session_id=f'{robot_id}-session',
        source_snapshot_epoch=1,
        validity_s=5.0,
        bids=tuple(bids),
    )


def test_pair_selection_is_deterministic_and_assigns_distinct_tasks():
    """Both idle robots receive the same deterministic pair on reordered bids."""
    union = _union(('alpha', 0.0), ('bravo', 5.0))
    first = selection.choose_assignment(
        union,
        _batch(
            'robot1', union,
            _bid('bravo', 2.0, path=((5.0, 0.0), (5.1, 0.0))),
            _bid('alpha', 1.0),
        ),
        _batch(
            'robot2', union,
            _bid('alpha', 3.0),
            _bid('bravo', 0.5, path=((5.0, 0.0), (5.1, 0.0))),
        ),
    )
    second = selection.choose_assignment(
        union,
        _batch(
            'robot1', union,
            _bid('alpha', 1.0),
            _bid('bravo', 2.0, path=((5.0, 0.0), (5.1, 0.0))),
        ),
        _batch(
            'robot2', union,
            _bid('bravo', 0.5, path=((5.0, 0.0), (5.1, 0.0))),
            _bid('alpha', 3.0),
        ),
    )

    assert first == second == ('alpha', 'bravo')
    assert first[0] != first[1]


def test_solo_selection_assigns_the_only_reachable_task():
    """A single valid bid is preserved as a one-robot assignment."""
    union = _union(('solo', 0.0))

    result = selection.choose_assignment(
        union,
        _batch('robot1', union, _bid('solo', 1.5)),
        _batch('robot2', union),
    )

    assert result == ('solo', '')


def test_robot1_busy_leaves_robot2_to_select_new_work():
    """A busy robot is normalized to idle while retaining its active goal."""
    union = _union(('active-r1', 0.0), ('new-r2', 5.0))

    result = selection.choose_assignment(
        union,
        _batch('robot1', union, _bid('active-r1', 2.0)),
        _batch(
            'robot2', union,
            _bid('active-r1', 0.1),
            _bid('new-r2', 1.5, path=((5.0, 0.0), (5.1, 0.0))),
        ),
        active_robot1_id='active-r1',
    )

    assert result == ('', 'new-r2')


def test_robot2_busy_is_symmetric():
    """The same busy/free rule works with Robot 2 as the active robot."""
    union = _union(('new-r1', 0.0), ('active-r2', 5.0))

    result = selection.choose_assignment(
        union,
        _batch(
            'robot1', union,
            _bid('new-r1', 1.5),
            _bid('active-r2', 0.1, path=((5.0, 0.0), (5.1, 0.0))),
        ),
        _batch('robot2', union, _bid('active-r2', 2.0)),
        active_robot2_id='active-r2',
    )

    assert result == ('new-r1', '')


def test_active_frontier_is_excluded_from_the_free_robot():
    """The free robot cannot select the frontier active on its peer."""
    union = _union(('active', 0.0), ('replacement', 5.0))

    result = selection.choose_assignment(
        union,
        _batch('robot1', union, _bid('active', 1.0)),
        _batch(
            'robot2', union,
            _bid('active', 0.01),
            _bid('replacement', 2.0, path=((5.0, 0.0), (5.1, 0.0))),
        ),
        active_robot1_id='active',
    )

    assert result == ('', 'replacement')
    assert 'active' not in result


def test_both_busy_returns_no_allocation():
    """Two active goals make the minimal selector a no-op."""
    union = _union(('active-r1', 0.0), ('active-r2', 5.0))

    result = selection.choose_assignment(
        union,
        _batch('robot1', union, _bid('active-r1')),
        _batch('robot2', union, _bid('active-r2')),
        active_robot1_id='active-r1',
        active_robot2_id='active-r2',
    )

    assert result is None


def test_positive_infinity_bid_is_unavailable():
    """A positive-infinity path or travel cost cannot win over finite work."""
    union = _union(('infinite-path', 0.0), ('infinite-travel', 5.0),
                   ('finite', 10.0))

    result = selection.choose_assignment(
        union,
        _batch(
            'robot1', union,
            _bid('infinite-path', math.inf),
            _bid('infinite-travel', 1.0, travel=math.inf),
            _bid('finite', 2.0, path=((10.0, 0.0), (10.1, 0.0))),
        ),
        _batch('robot2', union),
    )

    assert result == ('finite', '')


def test_duplicate_task_cannot_be_selected_for_both_robots():
    """One canonical frontier is assigned to at most one robot."""
    union = _union(('only', 0.0))

    result = selection.choose_assignment(
        union,
        _batch('robot1', union, _bid('only')),
        _batch('robot2', union, _bid('only')),
    )

    assert result is not None
    assert sum(bool(task_id) for task_id in result) == 1
    assert result[0] != result[1]


def test_duplicate_bid_ids_in_one_batch_are_rejected():
    """The scorer rejects an ambiguous duplicate within one bid vector."""
    union = _union(('only', 0.0))

    with pytest.raises(ValueError, match='duplicate canonical task'):
        selection.choose_assignment(
            union,
            _batch('robot1', union, _bid('only'), _bid('only')),
            _batch('robot2', union),
        )


def test_frontier_cost_only_prefers_motion_cost_over_information_gain():
    """The adapter's cost-only mode ignores a task's visible gain."""
    union = _union(
        ('high-gain-long', 0.0, 100.0),
        ('low-gain-short', 5.0, 0.0),
    )

    result = selection.choose_assignment(
        union,
        _batch(
            'robot1', union,
            _bid('high-gain-long', 10.0),
            _bid('low-gain-short', 1.0, path=((5.0, 0.0), (5.1, 0.0))),
        ),
        _batch('robot2', union),
    )

    assert result == ('low-gain-short', '')


def test_adapter_binds_union_hash_and_frontier_cost_only(monkeypatch):
    """The adapter passes the specified pure-selector contract."""
    union = _union(('a', 0.0), ('b', 5.0))
    calls = []

    def fake_scorer(**kwargs):
        calls.append(kwargs)
        return SimpleNamespace(robot1_task_id='a', robot2_task_id='b')

    monkeypatch.setattr(selection, 'choose_pair_assignment', fake_scorer)

    assert selection.choose_assignment(
        union,
        _batch('robot1', union, _bid('a')),
        _batch('robot2', union, _bid('b', path=((5.0, 0.0), (5.1, 0.0)))),
    ) == ('a', 'b')
    assert len(calls) == 1
    assert calls[0]['round_id'] == union.union_hash
    assert calls[0]['union'] is union
    assert calls[0]['scoring_mode'] == 'frontier_cost_only'
    assert calls[0]['hard_failed_tasks'] == frozenset()


def test_selection_boundary_has_no_lifecycle_or_pair_decision_surface():
    """The minimal boundary returns IDs only and owns no lifecycle state."""
    union = _union(('a', 0.0))

    result = selection.choose_assignment(
        union,
        _batch('robot1', union, _bid('a')),
        _batch('robot2', union),
    )

    assert type(result) is tuple
    assert result == ('a', '')
    assert not hasattr(selection, 'PairDecision')
    assert not hasattr(selection, 'CoordinatorState')
    assert not hasattr(selection, 'RoundWork')


def test_active_ids_are_normalized_to_strings(monkeypatch):
    """Busy-ID normalization is pure and does not add lifecycle machinery."""
    union = _union(('active', 0.0), ('new', 5.0))
    calls = []

    def fake_scorer(**kwargs):
        calls.append(kwargs)
        return SimpleNamespace(robot1_task_id=7, robot2_task_id=8)

    monkeypatch.setattr(selection, 'choose_pair_assignment', fake_scorer)
    result = selection.choose_assignment(
        union,
        _batch('robot1', union, _bid('active')),
        _batch('robot2', union, _bid('new', path=((5.0, 0.0), (5.1, 0.0)))),
        active_robot1_id=7,
    )

    assert result == ('', '8')
    assert calls[0]['fixed_robot1_task_id'] == '7'
