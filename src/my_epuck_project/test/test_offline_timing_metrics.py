from my_epuck_project.offline_timing_metrics import (
    analyze_event_records, analyze_protocol, protocol_snappiness,
)


def _event(robot, time_s, kind, **fields):
    return {
        'robot_id': robot,
        'event_type': kind,
        'sim_time_s': time_s,
        'elapsed_s': time_s,
        **fields,
    }


def test_exact_idle_state_machine_preserves_interval_semantics():
    events = [
        _event('robot1', 1.0, 'CANDIDATE_BATCH_RECEIVED', candidate_count=2),
        _event('robot1', 1.0, 'DISTRIBUTED_STATUS', nav2_healthy=True,
               tf_healthy=True, local_nav_goal_active=False),
        _event('robot1', 3.0, 'NAV_GOAL_SENT'),
        _event('robot1', 5.0, 'NAVIGATION_SUCCEEDED'),
    ]
    result = analyze_event_records(events, ready_sim=1.0, end_sim=8.0,
                                   robots=('robot1',))
    metrics = result['robots']['robot1']
    assert metrics['feasible_work_seconds'] == 7.0
    # This is the established legacy state-machine behavior: adjacent
    # FEASIBLE_WORK_AVAILABLE segments merge even when goal_active changes.
    assert metrics['avoidable_idle_seconds'] == 7.0
    assert metrics['productive_engagement_seconds'] == 0.0
    assert metrics['longest_avoidable_idle_s'] == 7.0
    assert metrics['feasible_terminal_to_next_useful_dispatch_s']['count'] == 0


def test_missing_health_is_not_reclassified_as_feasible_work():
    events = [_event('robot1', 1.0, 'CANDIDATE_BATCH_RECEIVED',
                     candidate_count=1)]
    result = analyze_event_records(events, ready_sim=0.0, end_sim=4.0,
                                   robots=('robot1',))
    metrics = result['robots']['robot1']
    assert metrics['feasible_work_seconds'] == 0.0
    assert metrics['avoidable_idle_seconds'] == 0.0
    assert metrics['work_unavailable_seconds'] == 4.0


def test_protocol_snappiness_joins_use_simulation_time_and_identity():
    protocol = {
        'generation_records': [
            {'robot': 'robot1', 'kind': 'candidate',
             'candidate_generation_id': 7, 'sim_time_s': 1.0},
            {'robot': 'robot1', 'kind': 'task_snapshot',
             'candidate_generation_id': 7, 'source_snapshot_epoch': 3,
             'sim_time_s': 2.0},
        ],
        'bid_records': [{'robot': 'robot1', 'source_snapshot_epoch': 3,
                         'sim_time_s': 3.0}],
        'cooperation_events': [
            {'robot': 'robot1', 'event_type': 'DECISION_AGREED',
             'round_id': 'r1', 'sim_time_s': 4.0},
            {'robot': 'robot1', 'event_type': 'NAV_GOAL_SENT',
             'round_id': 'r1', 'task_id': 'task1', 'sim_time_s': 5.0},
            {'robot': 'robot1', 'event_type': 'NAVIGATION_SUCCEEDED',
             'round_id': 'r1', 'task_id': 'task1', 'sim_time_s': 8.0},
        ],
        'dispatches': [{'robot': 'robot1', 'round_id': 'r1',
                        'task_id': 'task1', 'sim_time_s': 5.0}],
        'navigation_terminals': [{'robot': 'robot1', 'round_id': 'r1',
                                  'task_id': 'task1', 'sim_time_s': 8.0}],
        'start_release_records': [
            {'event': 'START_RELEASE', 'accepted_handoff': True,
             'release_sim_time_s': 1.0}],
    }
    result = protocol_snappiness(protocol)
    assert result['dispatch_to_terminal']['values_s'] == [3.0]
    assert result['agreement_to_nav_goal_sent']['values_s'] == [1.0]
    assert result['handoff_to_first_shared_goal']['values_s'] == [4.0]
    assert result['candidate_generation']['values_s'] == [1.0]
    assert result['bid']['values_s'] == [1.0]


def test_analyze_protocol_exposes_item_one_outputs():
    protocol = {
        'generation_records': [], 'bid_records': [], 'cooperation_events': [],
        'dispatches': [], 'navigation_terminals': [],
        'status_records': [{'robot': 'robot1', 'sim_time_s': 1.0,
                            'feasible_work_available': True,
                            'actionable_work_available': False}],
        'start_release_records': [],
    }
    result = analyze_protocol(protocol, ready_sim=0.0, end_sim=2.0)
    assert 'avoidable_idle' in result
    assert 'work_availability' in result
    assert result['work_availability']['available'] is True


def test_protocol_snappiness_handles_equal_timestamp_dispatches():
    protocol = {
        'cooperation_events': [],
        'dispatches': [
            {'sim_time_s': 1.0, 'robot': 'robot1', 'task_id': 'a'},
            {'sim_time_s': 1.0, 'robot': 'robot2', 'task_id': 'b'},
        ],
        'navigation_terminals': [],
        'generation_records': [],
        'certificate_records': [],
    }
    result = protocol_snappiness(protocol)
    assert result['dispatch_to_terminal']['count'] == 0
