"""Tests for deterministic physical task matching and union construction."""

import math

from my_epuck_project.distributed_assignment.canonical import (
    TaskIdentity,
    build_canonical_union,
    canonical_round_id,
    equivalent_tasks,
    world_from_rotated_grid_cell,
)
from my_epuck_project.distributed_assignment.models import Bounds, PhysicalTask


def task(
        robot='robot1', session='s1', epoch=1, revision=1, local_id=1,
        approach=(0.0, 0.0), centroid=(0.0, 0.2),
        bounds=Bounds((-0.2, 0.0), (0.2, 0.4)), geometry=(),
        signature='opening', gain=4.0):
    """Create a compact physical task fixture."""
    return PhysicalTask(
        source_robot_id=robot,
        source_session_id=session,
        source_snapshot_epoch=epoch,
        source_map_revision=revision,
        physical_signature=signature,
        local_frontier_id=local_id,
        centroid=centroid,
        bounds=bounds,
        approach=approach,
        frontier_geometry=geometry,
        visible_reveal_gain=gain,
        local_path_valid=True,
    )


def test_different_ids_and_near_approaches_match_same_opening():
    """The 0.21 m near-duplicate regression must collapse to one task."""
    first = task(local_id=11, approach=(0.0, 0.0))
    second = task(
        robot='robot2', session='s2', local_id=999,
        approach=(0.21, 0.0), signature='different-transient-id',
    )
    assert equivalent_tasks(first, second)
    union = build_canonical_union([first], [second])
    assert len(union.tasks) == 1
    assert len(union.tasks[0].members) == 2


def test_centroid_distance_alone_does_not_merge_tasks():
    """Nearby centroids without bounds, geometry, or viewpoints are insufficient."""
    first = task(
        approach=(-1.0, 0.0), bounds=Bounds((-1.0, -1.0), (-0.8, -0.8)),
    )
    second = task(
        robot='robot2', session='s2', approach=(1.0, 0.0),
        centroid=(0.1, 0.2), bounds=Bounds((0.8, 0.8), (1.0, 1.0)),
    )
    assert not equivalent_tasks(first, second)


def test_overlapping_bounds_match_split_and_merged_frontiers():
    """Strongly overlapping bounds identify split/merged representations."""
    merged = task(
        bounds=Bounds((0.0, 0.0), (1.0, 0.4)), centroid=(0.5, 0.2),
        approach=(0.5, -0.2),
    )
    split = task(
        robot='robot2', session='s2', local_id=2,
        bounds=Bounds((0.2, 0.0), (0.9, 0.4)), centroid=(0.55, 0.2),
        approach=(0.8, -0.1),
    )
    assert equivalent_tasks(merged, split)


def test_geometry_samples_match_same_practical_observation_task():
    """Substantially shared frontier samples compensate for changed IDs."""
    first = task(
        approach=(0.0, 0.0), bounds=Bounds((-0.1, 0.9), (0.3, 1.1)),
        geometry=((0.0, 1.0), (0.1, 1.0), (0.2, 1.0)),
    )
    second = task(
        robot='robot2', session='s2', approach=(0.35, 0.0),
        bounds=Bounds((-0.1, 0.9), (0.3, 1.1)),
        geometry=((0.02, 1.01), (0.12, 1.0), (0.22, 0.99)),
    )
    assert equivalent_tasks(first, second)


def test_union_and_ids_ignore_input_arrival_order():
    """Canonical union ordering and hash do not follow DDS arrival order."""
    first = task(approach=(-2.0, 0.0), centroid=(-2.0, 1.0), signature='west')
    second = task(
        robot='robot2', session='s2', approach=(2.0, 0.0),
        centroid=(2.0, 1.0), bounds=Bounds((1.8, 0.8), (2.2, 1.2)),
        signature='east',
    )
    a = build_canonical_union([first], [second])
    b = build_canonical_union([], [second, first])
    assert [item.canonical_id for item in a.tasks] == [
        item.canonical_id for item in b.tasks
    ]
    assert a.union_hash == b.union_hash


def test_round_hash_always_orders_robot1_before_robot2():
    """Round hashing enforces canonical source order."""
    identity1 = TaskIdentity('robot1', 'session-a', 4)
    identity2 = TaskIdentity('robot2', 'session-b', 9)
    assert canonical_round_id(identity1, identity2) == canonical_round_id(
        identity1, identity2,
    )
    try:
        canonical_round_id(identity2, identity1)
    except ValueError:
        pass
    else:
        raise AssertionError('reversed source ordering must be rejected')


def test_rotated_occupancy_origin_cell_conversion():
    """A 90-degree map origin rotation is applied around the origin."""
    point = world_from_rotated_grid_cell((1.0, 2.0), math.pi / 2, 1.0, 0, 1)
    assert math.isclose(point[0], -0.5, abs_tol=1e-12)
    assert math.isclose(point[1], 2.5, abs_tol=1e-12)
