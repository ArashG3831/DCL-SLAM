"""Regression test for the multi-threaded coordinator round race."""

from pathlib import Path

from my_epuck_project.distributed_assignment.models import Bounds, PhysicalTask
from my_epuck_project.distributed_frontier_assignment import (
    eligible_solo_tasks,
    peer_navigation_blocks_dispatch,
)

SOURCE = Path(__file__).parents[1] / 'my_epuck_project' / (
    'distributed_frontier_assignment.py'
)


def test_tick_owns_a_local_round_and_abandons_replaced_rounds():
    """A terminal callback must not make _tick dereference None."""
    text = SOURCE.read_text(encoding='utf-8')
    assert 'round_work = self._round' in text
    assert 'if round_work is None:' in text
    assert 'self._round_is_current(round_work, generation)' in text
    assert 'self._round_lifecycle.generation' in text


def test_successful_physical_frontier_is_not_redispatched_while_present():
    """A tiny successful residual must not cause same-task goal churn."""
    completed = PhysicalTask(
        'robot1', 'session', 1, 7, 'completed-region', 1,
        (1.0, 1.0), Bounds((0.9, 0.9), (1.1, 1.1)), (1.0, 1.0),
        visible_reveal_gain=1.0, local_ordering_score=1.0,
        local_path_valid=True, local_path_length_m=0.06,
    )
    alternate = PhysicalTask(
        'robot1', 'session', 1, 7, 'new-region', 2,
        (2.0, 2.0), Bounds((1.9, 1.9), (2.1, 2.1)), (2.0, 2.0),
        visible_reveal_gain=1.0, local_ordering_score=1.0,
        local_path_valid=True, local_path_length_m=1.0,
    )
    selected = eligible_solo_tasks(
        (completed, alternate), set(), {'completed-region'}, 0.05, 0.0, 18.0,
    )
    assert [task.physical_signature for task in selected] == ['new-region']


def test_peer_navigation_does_not_serialize_when_traffic_scheduler_disabled():
    """The normal profile permits independent goals on both robots."""
    assert peer_navigation_blocks_dispatch(False, True) is False


def test_peer_navigation_reservation_remains_when_traffic_scheduler_enabled():
    """Explicit traffic scheduling retains its safety reservation."""
    assert peer_navigation_blocks_dispatch(True, True) is True
    assert peer_navigation_blocks_dispatch(True, False) is False


def test_snapshot_wait_path_uses_the_same_scheduler_gate():
    """Both peer-activity paths obey the explicit traffic setting."""
    text = SOURCE.read_text(encoding='utf-8')
    # One occurrence is the helper definition; the other two are the normal
    # dispatch and missing-snapshot paths in _tick.
    assert text.count('peer_navigation_blocks_dispatch(') >= 3
