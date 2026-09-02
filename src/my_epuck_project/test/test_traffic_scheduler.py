"""Pure geometry and priority regressions for pre-dispatch traffic gating."""

from my_epuck_project.distributed_assignment.traffic_scheduler import (
    detect_path_conflict,
    detect_path_conflict_interval,
    project_path_progress,
    schedule_traffic,
)
from my_epuck_project.traffic_test_barrier import ready_key
from std_msgs.msg import String


def _schedule(first, second, **kwargs):
    return schedule_traffic(
        first, second, robot1_safe_radius_m=0.08,
        robot2_safe_radius_m=0.08, reference_speed_mps=0.13, **kwargs,
    )


def test_separated_parallel_paths_do_not_conflict():
    decision = _schedule([(0.0, 0.0), (3.0, 0.0)], [(0.0, 0.3), (3.0, 0.3)])
    assert not decision.conflict
    assert decision.minimum_separation_m == 0.3


def test_identical_and_opposite_corridor_paths_conflict():
    identical = _schedule([(0.0, 0.0), (3.0, 0.0)], [(0.0, 0.0), (3.0, 0.0)])
    opposite = _schedule([(0.0, 0.0), (3.0, 0.0)], [(3.0, 0.0), (0.0, 0.0)])
    assert identical.conflict and opposite.conflict
    assert identical.required_separation_m == 0.16


def test_perpendicular_crossing_conflicts_continuously_between_samples():
    conflict, minimum, first_distance, second_distance = detect_path_conflict(
        [(0.0, 0.0), (4.0, 0.0)], [(2.0, -2.0), (2.0, 2.0)], 0.16,
    )
    assert conflict and minimum == 0.0
    assert first_distance == 2.0 and second_distance == 2.0


def test_conflict_interval_retains_last_conflicting_route_location():
    """Sparse continuous segments expose both ends of a conservative interval."""
    result = detect_path_conflict_interval(
        [(0.0, 0.0), (4.0, 0.0), (8.0, 0.0)],
        [(2.0, -2.0), (2.0, 2.0), (6.0, 2.0), (6.0, -2.0)],
        0.16,
    )
    conflict, minimum, first1, first2, last1, last2 = result
    assert conflict and minimum == 0.0
    assert first1 == 2.0 and first2 == 2.0
    assert last1 >= first1 and last2 >= first2


def test_path_progress_projection_is_continuous_between_vertices():
    """A pose between path samples is usable for event-driven release."""
    progress = project_path_progress(
        [(0.0, 0.0), (1.0, 0.0), (1.0, 2.0)], (1.0, 1.25),
    )
    assert progress == (2.25, 0.0)


def test_lower_eta_wins_and_id_only_breaks_near_tie():
    lower_eta = _schedule(
        [(0.0, 0.0), (4.0, 0.0)],
        [(1.0, -3.0), (1.0, 3.0)],
    )
    tie = _schedule([(0.0, 0.0), (2.0, 0.0)], [(1.0, -1.0), (1.0, 1.0)])
    assert lower_eta.winner_robot_id == 'robot1'
    assert lower_eta.reason == 'LOWER_ETA'
    assert tie.winner_robot_id == 'robot2'
    assert tie.reason == 'ETA_TIE_ROBOT_ID'


def test_active_robot_wins_and_loser_is_explicitly_deferred():
    decision = _schedule(
        [(0.0, 0.0), (2.0, 0.0)], [(0.0, 0.0), (2.0, 0.0)],
        active_robots=frozenset({'robot2'}),
    )
    assert decision.winner_robot_id == 'robot2'
    assert decision.waiting_robot_id == 'robot1'
    assert decision.reason == 'ALREADY_ACTIVE'


def test_both_active_is_monitor_only_not_a_cancellation_order():
    decision = _schedule(
        [(0.0, 0.0), (2.0, 0.0)], [(0.0, 0.0), (2.0, 0.0)],
        active_robots=frozenset({'robot1', 'robot2'}),
    )
    assert decision.conflict
    assert decision.winner_robot_id == ''
    assert decision.reason == 'BOTH_ALREADY_ACTIVE_MONITOR_ONLY'


def test_allocator_uses_waiting_state_not_cmd_vel_traffic_hack():
    source = open(
        'src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py',
        encoding='utf-8',
    ).read()
    assert 'CoordinatorState.WAITING_FOR_TRAFFIC' in source
    assert "'cmd_vel'" not in source
    assert '_begin_traffic_wait' in source
    assert 'stale deferred task will not dispatch' in source


def test_test_barrier_accepts_only_complete_matching_round_identity():
    valid = String()
    valid.data = '{"decision_hash":"hash","round_id":"round"}'
    incomplete = String()
    incomplete.data = '{"round_id":"round"}'
    assert ready_key(valid) == ('round', 'hash')
    assert ready_key(incomplete) is None
