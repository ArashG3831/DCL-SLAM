"""Contract tests for the minimal coordinator's Stage B bid protocol."""

from dataclasses import replace

from builtin_interfaces.msg import Time

from my_epuck_project.distributed_assignment.models import (
    Bid,
    Bounds,
    CanonicalTask,
    CanonicalUnion,
)
from my_epuck_project.minimal_frontier_protocol import (
    complete_pair,
    from_msg,
    make_batch,
    to_msg,
)


SESSION_ONE = '11' * 16
SESSION_TWO = '22' * 16


def _union() -> CanonicalUnion:
    """Build a small canonical union with a stable, explicit token."""
    task_kwargs = dict(
        members=(),
        centroid=(0.0, 0.0),
        bounds=Bounds((-1.0, -1.0), (1.0, 1.0)),
        approach=(0.5, 0.0),
        approach_yaw=0.0,
        frontier_geometry=(),
        visible_cells=(),
        visible_bounds=None,
        visible_reveal_gain=1.0,
    )
    return CanonicalUnion(
        tasks=(
            CanonicalTask('101', **task_kwargs),
            CanonicalTask('202', **task_kwargs),
        ),
        union_hash='union-token-101-202',
    )


def _bids(union: CanonicalUnion, *, unavailable=False) -> tuple[Bid, ...]:
    """Return one complete vector, including an explicit unavailable bid."""
    return (
        Bid(
            union.tasks[0].canonical_id,
            not unavailable,
            1.25 if not unavailable else 0.0,
            1.25 if not unavailable else 0.0,
            heading_cost=0.2 if not unavailable else 0.0,
            task_generation_ros_ns=1_000_000_007,
            path_query_ros_ns=2_000_000_011,
            path=((0.0, 0.0), (0.5, 0.0)) if not unavailable else (),
        ),
        Bid(
            union.tasks[1].canonical_id,
            False,
            0.0,
            0.0,
            path=(),
        ),
    )


def _batch(
        union: CanonicalUnion, robot: str, session: str, epoch: int,
        *, validity_s=2.5, bids=None):
    """Create a batch while leaving protocol metadata caller-controlled."""
    return make_batch(
        robot,
        session,
        epoch,
        union,
        _bids(union) if bids is None else bids,
        validity_s,
    )


def test_make_batch_binds_the_union_token_to_round_and_hash():
    """The current union hash is the only Stage B round token."""
    union = _union()

    batch = _batch(union, 'robot1', SESSION_ONE, 7)

    assert batch.round_id == union.union_hash
    assert batch.union_hash == union.union_hash
    assert batch.source_robot_id == 'robot1'
    assert batch.source_session_id == SESSION_ONE
    assert batch.source_snapshot_epoch == 7
    assert batch.validity_s == 2.5
    assert tuple(bid.canonical_task_id for bid in batch.bids) == ('101', '202')


def test_conversion_round_trip_preserves_available_and_unavailable_bids():
    """ROS conversion preserves the full vector and its union binding."""
    union = _union()
    original = _batch(union, 'robot1', SESSION_ONE, 7)

    message = to_msg(original, Time(sec=12, nanosec=34))
    restored = from_msg(message)

    assert message.header.frame_id == 'shared_map'
    assert message.round_id == union.union_hash
    assert message.union_hash == union.union_hash
    assert message.source_robot_id == 'robot1'
    assert restored == original
    assert restored.bids[0].path_valid
    assert restored.bids[0].path == ((0.0, 0.0), (0.5, 0.0))
    assert not restored.bids[1].path_valid
    assert restored.bids[1].path_length_m == 0.0
    assert restored.bids[1].estimated_travel_cost == 0.0
    assert restored.bids[1].path == ()


def test_unavailable_bid_is_a_full_wire_entry_not_an_omitted_id():
    """An unavailable task still occupies its canonical vector position."""
    union = _union()
    batch = _batch(
        union, 'robot1', SESSION_ONE, 7,
        bids=_bids(union, unavailable=True),
    )

    assert len(batch.bids) == len(union.tasks)
    assert tuple(bid.canonical_task_id for bid in batch.bids) == ('101', '202')
    assert all(not bid.path_valid for bid in batch.bids)
    assert complete_pair(union, batch, _batch(union, 'robot2', SESSION_TWO, 9))


def test_matching_full_pair_is_accepted_for_the_same_union():
    """Two complete vectors with the same union token form a matching pair."""
    union = _union()

    robot1 = _batch(union, 'robot1', SESSION_ONE, 7)
    robot2 = _batch(union, 'robot2', SESSION_TWO, 9)

    assert complete_pair(union, robot1, robot2)


def test_hash_mismatch_rejects_the_pair_even_when_ids_are_complete():
    """A complete vector for another current union cannot participate."""
    union = _union()
    robot1 = _batch(union, 'robot1', SESSION_ONE, 7)
    robot2 = replace(
        _batch(union, 'robot2', SESSION_TWO, 9),
        union_hash='different-union-token',
    )

    assert not complete_pair(union, robot1, robot2)


def test_incomplete_or_wrong_id_vectors_reject_the_pair():
    """Both arrays must contain exactly the canonical union ID set."""
    union = _union()
    robot1 = _batch(union, 'robot1', SESSION_ONE, 7)
    missing = replace(robot1, bids=robot1.bids[:1])
    wrong_id = replace(
        robot1,
        bids=(
            robot1.bids[0],
            replace(robot1.bids[1], canonical_task_id='999'),
        ),
    )
    robot2 = _batch(union, 'robot2', SESSION_TWO, 9)

    assert not complete_pair(union, missing, robot2)
    assert not complete_pair(union, wrong_id, robot2)


def test_pairing_does_not_infer_ttl_or_session_freshness():
    """Completeness uses token and IDs, not arrival, TTL, or session guesses."""
    union = _union()
    robot1 = _batch(union, 'robot1', SESSION_ONE, 1, validity_s=0.0)
    robot2 = _batch(union, 'robot2', SESSION_TWO, 999, validity_s=999.0)

    assert complete_pair(union, robot1, robot2)
    assert robot1.source_session_id != robot2.source_session_id
    assert robot1.source_snapshot_epoch != robot2.source_snapshot_epoch
    assert robot1.validity_s != robot2.validity_s

