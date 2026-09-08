"""Deterministic tests for allocator round ownership and liveness triggers."""

from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

from my_epuck_project.round_lifecycle import RoundGeneration
from my_epuck_project.distributed_assignment.models import (
    BidBatch,
    Bounds,
    PhysicalTask,
    TaskSnapshot,
)
from my_epuck_project.distributed_assignment.protocol import (
    bid_batch_valid,
    receive,
)
from my_epuck_project.distributed_frontier_assignment import (
    DistributedFrontierAssignment,
    normal_round_requires_replacement,
)


SOURCE = Path(__file__).parents[1] / 'my_epuck_project' / (
    'distributed_frontier_assignment.py'
)


def test_clear_during_tick_invalidates_captured_lease():
    lifecycle = RoundGeneration()
    round_a = lifecycle.activate('A')
    assert lifecycle.is_current(round_a)

    lifecycle.invalidate()

    assert not lifecycle.is_current(round_a)
    assert lifecycle.active_round_id is None


def test_replace_during_tick_invalidates_old_generation_but_keeps_new_round():
    lifecycle = RoundGeneration()
    round_a = lifecycle.activate('A')
    round_b = lifecycle.activate('B')

    assert not lifecycle.is_current(round_a)
    assert lifecycle.is_current(round_b)
    assert round_b.generation > round_a.generation


def test_busy_to_idle_requires_a_new_allocation_trigger():
    text = SOURCE.read_text(encoding='utf-8')
    navigation_finished = text[text.index('def _navigation_finished'):]
    assert "self._last_semantic_fingerprint = ''" in navigation_finished
    assert "self._reset_round('navigation terminal result')" in navigation_finished


def test_reset_round_rearms_normal_round_creation_after_continuation_clear():
    """Invalidating a continuation must not retain the semantic suppression."""
    node = DistributedFrontierAssignment.__new__(DistributedFrontierAssignment)
    node._round_lifecycle = RoundGeneration()
    node._round = SimpleNamespace(
        mode='continuation',
        round_id='continuation-round',
    )
    node._last_semantic_fingerprint = 'previous-round-fingerprint'
    node._bid_batches = {}
    node._peer_decision = object()
    node._transition = lambda *_args, **_kwargs: None
    node._log_round_lifecycle = lambda *_args, **_kwargs: None

    DistributedFrontierAssignment._reset_round(
        node, 'peer active commitment ended or changed',
    )

    assert node._round is None
    assert node._last_semantic_fingerprint == ''
    assert node._bid_batches == {}
    assert node._peer_decision is None


def _active_normal_round(fingerprint='same', session1='r1', session2='r2',
                         epoch1=1, epoch2=1):
    return SimpleNamespace(
        mode='normal',
        content_fingerprint=fingerprint,
        snapshots=(
            SimpleNamespace(source_session_id=session1, epoch=epoch1),
            SimpleNamespace(source_session_id=session2, epoch=epoch2),
        ),
    )


def _snapshot_identity(robot_id, session, epoch):
    return SimpleNamespace(
        source_robot_id=robot_id,
        source_session_id=session,
        epoch=epoch,
    )


def test_epoch_only_snapshot_update_keeps_active_normal_round():
    current = _active_normal_round(epoch1=10, epoch2=11)
    first = _snapshot_identity('robot1', 'r1', 12)
    second = _snapshot_identity('robot2', 'r2', 13)

    assert normal_round_requires_replacement(
        current, first, second, 'same',
    ) is False


def test_allocation_fingerprint_ignores_freshness_provenance_only():
    first = TaskSnapshot(
        'robot1', 'session', 10, 20, 'map-a', 100, 5.0, (),
        lower_bound_context_fingerprint='bounds-a', costmap_revision=30,
        candidate_generation_id=40,
    )
    refreshed = replace(
        first,
        epoch=11,
        map_revision=21,
        map_fingerprint='map-b',
        generation_ros_ns=101,
        lower_bound_context_fingerprint='bounds-b',
        costmap_revision=31,
        candidate_generation_id=41,
    )

    assert DistributedFrontierAssignment._allocation_semantic_fingerprint(
        first, first,
    ) == DistributedFrontierAssignment._allocation_semantic_fingerprint(
        refreshed, refreshed,
    )


def test_allocation_fingerprint_changes_when_path_semantics_change():
    task = PhysicalTask(
        'robot1', 'session', 1, 1, 'task', 1,
        (1.0, 0.0), Bounds((0.9, -0.1), (1.1, 0.1)), (1.0, 0.0),
        local_path_valid=True, local_path_length_m=1.0,
        local_path=((0.0, 0.0), (1.0, 0.0)),
    )
    changed_task = replace(task, local_path_length_m=2.0)
    first = TaskSnapshot(
        'robot1', 'session', 1, 1, 'map', 1, 5.0, (task,),
    )
    changed = replace(first, tasks=(changed_task,))

    assert DistributedFrontierAssignment._allocation_semantic_fingerprint(
        first, first,
    ) != DistributedFrontierAssignment._allocation_semantic_fingerprint(
        changed, changed,
    )


def test_provenance_only_update_preserves_active_normal_round():
    first = TaskSnapshot(
        'robot1', 'r1', 1, 1, 'map-1', 1, 5.0, (),
        lower_bound_context_fingerprint='bounds-1',
    )
    second = TaskSnapshot(
        'robot2', 'r2', 1, 1, 'map-2', 1, 5.0, (),
        lower_bound_context_fingerprint='bounds-2',
    )
    refreshed_first = replace(
        first, epoch=2, map_revision=2, map_fingerprint='map-1-new',
        lower_bound_context_fingerprint='bounds-1-new',
    )
    refreshed_second = replace(
        second, epoch=2, map_revision=2, map_fingerprint='map-2-new',
        lower_bound_context_fingerprint='bounds-2-new',
    )
    fingerprint = DistributedFrontierAssignment._allocation_semantic_fingerprint(
        first, second,
    )
    current = _active_normal_round(
        fingerprint=fingerprint, session1='r1', session2='r2',
    )
    refreshed_fingerprint = (
        DistributedFrontierAssignment._allocation_semantic_fingerprint(
            refreshed_first, refreshed_second,
        )
    )

    assert normal_round_requires_replacement(
        current, refreshed_first, refreshed_second, refreshed_fingerprint,
    ) is False


def test_semantic_task_or_path_change_replaces_active_normal_round():
    current = _active_normal_round(fingerprint='old')
    first = _snapshot_identity('robot1', 'r1', 2)
    second = _snapshot_identity('robot2', 'r2', 2)

    assert normal_round_requires_replacement(
        current, first, second, 'new-task-or-path',
    ) is True


def test_source_session_change_replaces_active_normal_round():
    current = _active_normal_round()
    first = _snapshot_identity('robot1', 'new-session', 2)
    second = _snapshot_identity('robot2', 'r2', 2)

    assert normal_round_requires_replacement(
        current, first, second, 'same',
    ) is True


def test_epoch_mismatched_bid_remains_rejected():
    batch = BidBatch(
        round_id='round', union_hash='union', source_robot_id='robot2',
        source_session_id='session', source_snapshot_epoch=4,
        validity_s=5.0, bids=(),
    )
    received = receive(batch, 5.0, 10.0)

    assert bid_batch_valid(
        received, 10.1, 'robot2', 'session', 5, 'round', 'union', True,
    ) is False


def test_stale_round_cannot_be_committed_by_source_guard():
    text = SOURCE.read_text(encoding='utf-8')
    assert 'self._round_lifecycle.generation == generation' in text
    assert 'self._round is round_work' in text
    assert 'ALLOCATOR_TICK_STALE_DISCARDED' in text
    assert "'ALLOCATOR_TICK_COMMITTED'" in text


def test_deterministic_interleaving_stress_has_no_live_old_lease():
    """Exercise thousands of create/clear/replace orderings without sleeps."""
    lifecycle = RoundGeneration()
    stale_leases = []
    for index in range(5000):
        lease = lifecycle.activate('round-%d' % index)
        stale_leases.append(lease)
        if index % 3 == 0:
            lifecycle.invalidate()
        else:
            replacement = lifecycle.activate('replacement-%d' % index)
            assert lifecycle.is_current(replacement)
            assert not lifecycle.is_current(lease)

    assert all(not lifecycle.is_current(lease)
               for lease in stale_leases[:-1])


def _snapshot(robot_id: str, epoch: int) -> TaskSnapshot:
    return TaskSnapshot(
        source_robot_id=robot_id,
        source_session_id=robot_id + '-session',
        epoch=epoch,
        map_revision=epoch,
        map_fingerprint='fingerprint-%d' % epoch,
        generation_ros_ns=epoch,
        validity_s=5.0,
        tasks=(),
    )


class _TerminalFake:
    """Minimal node surface for exercising the real terminal method."""

    def __init__(self):
        self._robot_id = 'robot1'
        self._snapshots = {
            'robot1': receive(_snapshot('robot1', 4), 5.0, 10.0),
            'robot2': receive(_snapshot('robot2', 7), 5.0, 10.0),
        }
        self._terminal = False
        self._terminal_success = False
        self._terminal_reason = ''
        self._terminal_epoch = 0
        self._terminal_finalization_attempt_count = 0
        self._terminal_commit_count = 0
        self.transitions = []
        self.events = []
        self.get_logger = lambda: SimpleNamespace(
            info=lambda *_args, **_kwargs: None,
            warning=lambda *_args, **_kwargs: None,
        )

    def _transition(self, state, reason):
        self.transitions.append((state, reason))

    def _emit_event(self, event_type, reason):
        self.events.append((event_type, reason))


def test_terminal_finalization_reads_wrapped_snapshot_payload_epochs():
    fake = _TerminalFake()

    DistributedFrontierAssignment._set_terminal(
        fake, 'MISSION_COMPLETE_NO_ACTIONABLE_FRONTIERS', success=True,
    )

    assert fake._terminal is True
    assert fake._terminal_epoch == 7
    assert fake._terminal_commit_count == 1
    assert fake.events == [(
        'MISSION_COMPLETE', 'MISSION_COMPLETE_NO_ACTIONABLE_FRONTIERS',
    )]


def test_terminal_finalization_is_idempotent_for_duplicate_observations():
    fake = _TerminalFake()
    method = DistributedFrontierAssignment._set_terminal

    method(fake, 'MISSION_COMPLETE_NO_FRONTIERS', success=True)
    method(fake, 'MISSION_COMPLETE_NO_FRONTIERS', success=True)

    assert fake._terminal_commit_count == 1
    assert len(fake.events) == 1
    assert fake._terminal_finalization_attempt_count == 2


def test_terminal_finalization_handles_missing_snapshot_wrappers():
    fake = _TerminalFake()
    fake._snapshots = {}

    DistributedFrontierAssignment._set_terminal(
        fake, 'MISSION_ABORT_TIMEOUT', success=False,
    )

    assert fake._terminal is True
    assert fake._terminal_epoch == 0
    assert fake._terminal_commit_count == 1


def test_fresh_snapshot_accessor_rejects_stale_wrapper_without_payload_mixup():
    fake = _TerminalFake()
    fake._snapshots['robot2'] = receive(_snapshot('robot2', 8), 1.0, 0.0)

    assert DistributedFrontierAssignment._fresh_snapshot(
        fake, 'robot1', 10.5,
    ).epoch == 4
    assert DistributedFrontierAssignment._fresh_snapshot(
        fake, 'robot2', 10.5,
    ) is None
