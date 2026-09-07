from my_epuck_project.offline_protocol_replay import (
    _certificate_status, protocol_summary,
)


def _protocol():
    return {
        'available': True,
        'robots': {
            'robot1': {'bid_batches': 1, 'failure_classes': {}},
            'robot2': {'bid_batches': 0, 'failure_classes': {}},
        },
        'generation_records': [
            {'kind': 'candidate', 'robot': 'robot1',
             'candidate_generation_id': 7, 'sim_time_s': 1.0,
             'detected_not_queried_count': 2},
            {'kind': 'task_snapshot', 'robot': 'robot1',
             'candidate_generation_id': 7, 'sim_time_s': 2.0},
        ],
        'bid_records': [{'robot': 'robot1', 'round_id': 'r1',
                         'path_valid': True}],
        'pair_decisions': [],
        'cooperation_events': [
            {'event_type': 'DECISION_AGREED', 'round_id': 'r1',
             'robot': 'robot1'},
            {'event_type': 'NAV_GOAL_SENT', 'round_id': 'r1',
             'robot': 'robot1', 'task_id': 't1'},
        ],
        'dispatches': [{'event_type': 'NAV_GOAL_SENT', 'round_id': 'r1',
                        'robot': 'robot1', 'task_id': 't1'}],
        'navigation_terminals': [],
        'failure_records': [],
        'certificate_records': [],
        'certificate_status': {'state': 'NOT_INVOKED',
                               'payload_complete': False},
        'status_records': [{'robot': 'robot1', 'sim_time_s': 3.0,
                            'detected_not_queried_count': 2,
                            'actionable_reachable_count': 1}],
    }


def test_protocol_summary_preserves_counts_and_explicit_zeroes():
    result = protocol_summary(_protocol())
    assert result['candidate_batches']['total'] == 1
    assert result['task_snapshots']['total'] == 1
    assert result['bids']['records'] == 1
    assert result['pair_decisions']['records'] == 0
    assert result['agreements']['publications'] == 1
    assert result['assignments']['dispatches'] == 1
    assert result['dnu']['observations'] == 2
    assert result['certificate']['status']['state'] == 'NOT_INVOKED'


def test_incomplete_certificate_payload_fails_closed():
    status = _certificate_status([{'sim_time_s': 1.0}], [], [], [])
    assert status['state'] == 'OBSERVED_INCOMPLETE'
    assert status['payload_complete'] is False
    assert 'blocking_unqueried_candidates' in status['missing_fields']
