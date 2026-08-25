import json
import math
from pathlib import Path

from action_msgs.msg import GoalStatus, GoalStatusArray
from my_epuck_interfaces.msg import ExplorationClaim, ExplorationStatus
from my_epuck_project.cooperative_trial_collector import (
    add_settled_age,
    atomic_save_map,
    claim_dict,
    completion_receipt_age,
    map_metadata,
    status_dict,
    validate_map_message,
)
from nav_msgs.msg import OccupancyGrid
import numpy as np


def test_collector_accepts_local_map_before_handoff_and_reads_shared_maps():
    source = (Path(__file__).resolve().parents[1] / 'my_epuck_project' /
              'cooperative_trial_collector.py').read_text(encoding='utf-8')
    assert 'shared_map_reader = QoSProfile' in source
    assert "and self.messages[robot]['local_map'] is None" in source


def occupancy(width=3, height=2):
    message = OccupancyGrid()
    message.header.frame_id = 'shared_map'
    message.info.width = width
    message.info.height = height
    message.info.resolution = 0.01
    message.info.origin.orientation.w = 1.0
    message.data = [-1, 0, 25, 50, 65, 100]
    return message


def test_full_final_status_storage_and_completion_detection():
    message = ExplorationStatus()
    message.source_robot_id = 'robot1'
    message.source_session_id.uuid = list(range(16))
    message.message_revision = 12
    message.state = ExplorationStatus.COMPLETE
    message.reason = 'decentralized_completion_consensus'
    message.candidate_count = 3
    message.eligible_candidate_count = 0
    message.active_claim_id = 8
    message.active_frontier_id = 9
    message.local_map_revision = 10
    message.shared_map_checksum = 11
    message.status_ttl.sec = 4
    result = status_dict(message, 10.0, 11.0)
    assert result['state'] == 'MISSION_COMPLETE'
    assert result['source_robot_id'] == 'robot1'
    assert result['source_session_id'] == bytes(range(16)).hex()
    assert result['age_at_write_s'] == 1.0
    assert result['active_claim_id'] == 8


def test_full_claim_storage_and_active_reservation():
    message = ExplorationClaim()
    message.source_robot_id = 'robot2'
    message.source_session_id.uuid = [1] * 16
    message.message_revision = 2
    message.claim_id = 3
    message.state = ExplorationClaim.NAVIGATING
    message.frontier_id = 4
    message.map_revision = 5
    message.path_length_m = 1.25
    message.frontier_centroid.x = 0.2
    message.approach_pose.pose.orientation.w = 1.0
    result = claim_dict(message, 20.0, 20.5)
    assert result['state'] == 'NAVIGATING'
    assert result['reserving'] is True
    assert result['terminal'] is False
    assert result['frontier_id'] == 4
    message.state = ExplorationClaim.RELEASED
    assert claim_dict(message, 20.0, 21.0)['terminal'] is True


def test_map_validation_and_atomic_lossless_save(tmp_path):
    message = occupancy()
    assert validate_map_message(message) == []
    metadata = map_metadata(
        message, '/robot1/shared_map', 'run', 'robot1', 10.0)
    path = tmp_path / 'robot1_final_shared_map.npz'
    atomic_save_map(path, message, metadata)
    with np.load(path, allow_pickle=False) as archive:
        assert archive['occupancy'].dtype == np.int8
        assert archive['occupancy'].reshape(-1).tolist() == list(message.data)
        assert json.loads(archive['metadata_json'].item())['run_id'] == 'run'
    assert not list(tmp_path.glob('*.tmp'))


def test_map_validation_rejects_malformed_geometry():
    message = occupancy()
    message.info.resolution = float('nan')
    message.info.origin.orientation.w = 0.0
    message.header.frame_id = 'map'
    message.data.pop()
    errors = validate_map_message(message)
    assert 'data_length_mismatch' in errors
    assert 'invalid_resolution' in errors
    assert 'invalid_origin_quaternion' in errors
    assert 'unexpected_frame' in errors


def test_goal_status_active_state_definition():
    array = GoalStatusArray()
    status = GoalStatus()
    status.status = GoalStatus.STATUS_EXECUTING
    array.status_list.append(status)
    assert any(item.status in (
        GoalStatus.STATUS_ACCEPTED,
        GoalStatus.STATUS_EXECUTING,
        GoalStatus.STATUS_CANCELING,
    ) for item in array.status_list)


def test_stale_status_age_is_recorded_not_hidden():
    message = ExplorationStatus()
    message.state = ExplorationStatus.NAVIGATING
    result = status_dict(message, 1.0, 10.0)
    assert result['state'] == 'NAVIGATING'
    assert math.isclose(result['age_at_write_s'], 9.0)


def test_distributed_completion_receipt_is_used_without_legacy_status():
    received = {'distributed_status': 10.0}
    assert completion_receipt_age(received, 11.5) == 1.5


def test_legacy_completion_receipt_remains_supported():
    received = {'status': 20.0}
    assert completion_receipt_age(received, 21.0) == 1.0


def test_settled_age_is_attached_to_distributed_document():
    document = {'status': None, 'distributed_status': {}}
    add_settled_age(document, 0.25)
    assert document['distributed_status']['age_at_collection_s'] == 0.25


def test_missing_completion_receipt_is_safe():
    assert completion_receipt_age({}, 12.0) is None
