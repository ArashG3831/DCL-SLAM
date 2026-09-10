"""Focused tests for the minimal coordinator's traffic adapter."""

import inspect

from my_epuck_project import minimal_frontier_traffic as traffic
from my_epuck_project.distributed_assignment.traffic_scheduler import (
    TrafficDecision,
)


def _decide(first, second, **kwargs):
    """Use one fixed scheduler configuration for geometry assertions."""
    return traffic.decide(
        first,
        second,
        robot1_safe_radius_m=0.08,
        robot2_safe_radius_m=0.08,
        reference_speed_mps=1.0,
        **kwargs,
    )


def test_decide_delegates_and_returns_the_scheduler_result(monkeypatch):
    expected = object()
    calls = []

    def fake_schedule(first, second, **kwargs):
        calls.append((first, second, kwargs))
        return expected

    monkeypatch.setattr(
        traffic.traffic_scheduler, 'schedule_traffic', fake_schedule,
    )

    result = traffic.decide(
        [(0.0, 0.0)],
        [(1.0, 1.0)],
        robot1_safe_radius_m=0.11,
        robot2_safe_radius_m=0.12,
        reference_speed_mps=0.23,
        eta_tie_s=0.07,
        active_robots=frozenset({'robot2'}),
    )

    assert result is expected
    assert len(calls) == 1
    assert calls[0][2] == {
        'robot1_safe_radius_m': 0.11,
        'robot2_safe_radius_m': 0.12,
        'reference_speed_mps': 0.23,
        'eta_tie_s': 0.07,
        'active_robots': frozenset({'robot2'}),
    }


def test_decide_preserves_path_and_control_inputs_verbatim(monkeypatch):
    first = [(0.0, 0.0), (1.0, 0.0)]
    second = ((0.0, 1.0), (1.0, 1.0))
    active = frozenset({'robot1', 'robot2'})
    captured = {}

    def fake_schedule(first_path, second_path, **kwargs):
        captured['first'] = first_path
        captured['second'] = second_path
        captured['kwargs'] = kwargs
        return TrafficDecision()

    monkeypatch.setattr(
        traffic.traffic_scheduler, 'schedule_traffic', fake_schedule,
    )

    traffic.decide(
        first,
        second,
        robot1_safe_radius_m=0.081,
        robot2_safe_radius_m=0.093,
        reference_speed_mps=0.137,
        eta_tie_s=0.051,
        active_robots=active,
    )

    assert captured['first'] is first
    assert captured['second'] is second
    assert captured['kwargs']['active_robots'] is active
    assert captured['kwargs']['robot1_safe_radius_m'] == 0.081
    assert captured['kwargs']['robot2_safe_radius_m'] == 0.093
    assert captured['kwargs']['reference_speed_mps'] == 0.137
    assert captured['kwargs']['eta_tie_s'] == 0.051


def test_non_conflicting_paths_proceed_for_both_robots():
    decision = _decide(
        [(0.0, 0.0), (3.0, 0.0)],
        [(0.0, 0.3), (3.0, 0.3)],
    )

    assert not decision.conflict
    assert decision.winner_robot_id == ''
    assert decision.waiting_robot_id == ''
    assert not traffic.should_wait('robot1', decision)
    assert not traffic.should_wait('robot2', decision)


def test_conflict_winner_and_waiter_are_deterministic_on_eta_tie():
    first = _decide(
        [(0.0, 0.0), (2.0, 0.0)],
        [(1.0, -1.0), (1.0, 1.0)],
    )
    repeated = _decide(
        [(0.0, 0.0), (2.0, 0.0)],
        [(1.0, -1.0), (1.0, 1.0)],
    )

    assert first == repeated
    assert first.conflict
    assert first.winner_robot_id == 'robot2'
    assert first.waiting_robot_id == 'robot1'
    assert first.reason == 'ETA_TIE_ROBOT_ID'


def test_should_wait_matches_only_the_designated_waiter():
    decision = TrafficDecision(
        conflict=True,
        winner_robot_id='robot1',
        waiting_robot_id='robot2',
        reason='LOWER_ETA',
    )

    assert traffic.should_wait('robot2', decision) is True
    assert traffic.should_wait('robot1', decision) is False
    assert traffic.should_wait('robot3', decision) is False
    assert traffic.should_wait('', decision) is False


def test_adapter_functions_have_no_algorithm_lifecycle_history_or_replay_state():
    source = '\n'.join(
        inspect.getsource(function)
        for function in (traffic.decide, traffic.should_wait)
    ).lower()

    assert source.count('schedule_traffic') == 1
    assert not any(
        marker in source
        for marker in (
            'round_id', 'history', 'replay', 'lifecycle', 'reservation',
            'journal', 'continuation', 'certificate',
        )
    )

