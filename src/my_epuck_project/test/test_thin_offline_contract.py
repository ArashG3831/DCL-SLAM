"""Parity/contract tests for the thin offline evidence boundary."""

from my_epuck_project.offline_evidence_replay import _dedupe_series
from my_epuck_project.raw_evidence_contract import (
    condition_semantic_requirements,
    semantic_contract_status,
)


def _c_counts():
    return {
        '/robot1/navigate_to_pose/_action/status': 1,
        '/robot2/navigate_to_pose/_action/status': 1,
        '/robot1/distributed_status': 1,
        '/robot2/distributed_status': 1,
        '/robot1/distributed_event': 1,
        '/robot2/distributed_event': 1,
        '/robot1/task_snapshot': 1,
        '/robot2/task_snapshot': 1,
        '/robot2/task_bids': 1,
    }


def test_condition_c_requires_cooperation_semantics_and_handoff():
    requirements = condition_semantic_requirements(
        'C', ('robot1', 'robot2'), unknown_initial_pose=True)
    assert 'cooperation_status' in requirements
    assert 'structured_handoff' in requirements
    missing = semantic_contract_status(
        _c_counts(), 'C', ('robot1', 'robot2'), unknown_initial_pose=True,
        semantic_artifacts={'handoff_structured': False})
    assert not missing['complete']
    assert 'structured_handoff' in missing['failed_requirements']


def test_condition_c_accepts_replicated_status_and_bid_fallback():
    status = semantic_contract_status(
        _c_counts(), 'C', ('robot1', 'robot2'), unknown_initial_pose=True,
        semantic_artifacts={'handoff_structured': True})
    assert status['complete']
    assert status['failed_requirements'] == []


def test_cost_only_contract_requires_certificate_event_stream():
    status = semantic_contract_status(
        _c_counts(), 'C', ('robot1', 'robot2'), unknown_initial_pose=False,
        assignment_strategy='frontier_cost_only')
    assert 'certificate_event_stream' in status['checks']
    assert status['checks']['certificate_event_stream']['complete']


def test_not_invoked_cost_only_certificate_is_a_valid_contract_state():
    from my_epuck_project.offline_protocol_replay import _certificate_status

    certificate = _certificate_status(
        [], [{'robot': 'robot1'}], [],
        [{'event_type': 'DEGRADED_SOLO_COMMITMENT'}])
    assert certificate['state'] == 'NOT_INVOKED'
    assert certificate['payload_complete'] is False


def test_generic_contract_remains_topic_only():
    status = semantic_contract_status({}, None, ('robot1', 'robot2'))
    assert status['complete']
    assert status['checks'] == {}


def test_map_series_deduplication_preserves_stream_identity():
    series = _dedupe_series([
        {'sim_time_s': 1.0, 'known_cells': 10, 'occupied_cells': 2,
         'unknown_cells': 8, 'resolution': .05, 'topic': '/robot1/map'},
        {'sim_time_s': 1.0, 'known_cells': 10, 'occupied_cells': 2,
         'unknown_cells': 8, 'resolution': .05, 'topic': '/robot1/map'},
        {'sim_time_s': 2.0, 'known_cells': 12, 'occupied_cells': 2,
         'unknown_cells': 6, 'resolution': .05, 'topic': '/robot1/map'},
    ])
    assert len(series) == 2
    assert all(item['topic'] == '/robot1/map' for item in series)


def test_status_contract_exposes_explicit_work_availability_fields():
    from my_epuck_interfaces.msg import DistributedExplorationStatus

    message = DistributedExplorationStatus()
    message.feasible_work_available = True
    message.actionable_work_available = False
    message.work_availability_reason = 'REACHABLE_BELOW_GAIN'
    assert message.feasible_work_available is True
    assert message.actionable_work_available is False
    assert message.work_availability_reason == 'REACHABLE_BELOW_GAIN'
