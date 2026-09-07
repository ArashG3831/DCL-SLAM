"""Focused tests for native cooperative payload decoding boundaries."""

from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message


def test_frontier_and_task_snapshot_round_trip_current_interface():
    from my_epuck_interfaces.msg import FrontierCandidateArray, TaskSnapshot

    candidate = FrontierCandidateArray()
    candidate.source_robot_id = 'robot1'
    candidate.map_revision = 10
    candidate.costmap_revision = 11
    candidate.detected_not_queried_count = 1
    candidate.lower_bound_context_fingerprint = 'ctx-a'
    candidate.candidate_generation_id = 7
    candidate.terminal_frontier_regions_json = '[]'
    candidate_bytes = serialize_message(candidate)
    decoded_candidate = deserialize_message(
        candidate_bytes, get_message(
            'my_epuck_interfaces/msg/FrontierCandidateArray'))
    assert decoded_candidate.candidate_generation_id == 7
    assert decoded_candidate.lower_bound_context_fingerprint == 'ctx-a'

    snapshot = TaskSnapshot()
    snapshot.source_robot_id = 'robot1'
    snapshot.source_map_revision = 10
    snapshot.source_costmap_revision = 11
    snapshot.lower_bound_context_fingerprint = 'ctx-a'
    snapshot.candidate_generation_id = 7
    snapshot_bytes = serialize_message(snapshot)
    decoded_snapshot = deserialize_message(
        snapshot_bytes, get_message('my_epuck_interfaces/msg/TaskSnapshot'))
    assert decoded_snapshot.candidate_generation_id == 7
    assert decoded_snapshot.lower_bound_context_fingerprint == 'ctx-a'


def test_dispatch_latency_uses_distinct_authoritative_event_streams():
    from my_epuck_project.offline_protocol_replay import _dispatch_latencies

    dispatch = [{'robot': 'robot1', 'task_id': 'task-a', 'sim_time_s': 4.0}]
    terminal = [{'robot': 'robot1', 'task_id': 'task-a', 'sim_time_s': 9.5}]
    result = _dispatch_latencies(dispatch, terminal)
    assert result['count'] == 1
    assert result['values_s'] == [5.5]


def test_missing_certificate_payload_is_not_synthesized():
    from my_epuck_project.offline_protocol_replay import _certificate_status

    status = _certificate_status(
        [], [], [],
        [{'event_type': 'DEGRADED_SOLO_COMMITMENT'}])
    assert status['state'] == 'NOT_INVOKED'
    assert status['payload_complete'] is False


def test_degraded_bid_array_without_pair_round_is_not_certificate_invocation():
    from my_epuck_project.offline_protocol_replay import _certificate_status

    status = _certificate_status(
        [], [{'robot': 'robot1'}], [],
        [{'event_type': 'DEGRADED_SOLO_COMMITMENT'}])
    assert status['state'] == 'NOT_INVOKED'
    assert status['payload_complete'] is False


def test_certificate_payload_without_pair_or_bid_is_not_complete():
    from my_epuck_project.offline_protocol_replay import _certificate_status

    status = _certificate_status(
        [], [], [],
        [{'event_type': 'STATE_TRANSITION'}])
    assert status['state'] == 'NO_EVIDENCE'
    assert status['payload_complete'] is False


def test_certificate_payload_is_complete_only_when_observed():
    from my_epuck_project.offline_protocol_replay import _certificate_status

    status = _certificate_status(
        [{
            'sim_time_s': 1.0,
            'evaluated_candidate_count': 2,
            'detected_not_queried_count': 1,
            'blocking_unqueried_candidates': 1,
            'current_evaluated_assignment_score': 3.0,
            'best_optimistic_unqueried_score': 2.0,
            'dispatch_certified': True,
            'reason': 'OK',
        }], [], [], [])
    assert status['state'] == 'OBSERVED'
    assert status['payload_complete'] is True
