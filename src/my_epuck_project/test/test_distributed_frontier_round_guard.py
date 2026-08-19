"""Regression test for the multi-threaded coordinator round race."""

from pathlib import Path


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
