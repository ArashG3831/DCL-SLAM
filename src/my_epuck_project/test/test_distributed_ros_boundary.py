"""Static and construction tests for equal local-only assignment peers."""

from pathlib import Path
from types import SimpleNamespace

from my_epuck_project.distributed_frontier_assignment import (
    DistributedFrontierAssignment,
)

import rclpy


SOURCE = Path(__file__).parents[1] / 'my_epuck_project' / (
    'distributed_frontier_assignment.py'
)
NAV_SOURCE = Path(__file__).parents[1] / 'my_epuck_project' / (
    'distributed_assignment/local_nav2.py'
)


def test_explorer_source_has_no_velocity_or_peer_action_interface():
    """The explorer never creates raw velocity or absolute peer action clients."""
    text = SOURCE.read_text() + NAV_SOURCE.read_text()
    assert "'cmd_vel'" not in text
    assert '"cmd_vel"' not in text
    assert "ActionClient(" in text
    assert "NavigateToPose, 'navigate_to_pose'" in text
    assert "ComputePathToPose, 'compute_path_to_pose'" in text
    assert "'/robot1/navigate_to_pose'" not in text
    assert "'/robot2/navigate_to_pose'" not in text
    assert "'/robot1/compute_path_to_pose'" not in text
    assert "'/robot2/compute_path_to_pose'" not in text


def test_robot1_peer_resolves_only_robot1_nav2_clients():
    """Relative action names resolve inside the configured local namespace."""
    rclpy.init(args=[
        '--ros-args', '-r', '__ns:=/robot1', '-p', 'robot_id:=robot1',
        '-p', 'dispatch_enabled:=false',
    ])
    node = DistributedFrontierAssignment()
    try:
        names = node._nav2.interface_names()
        assert names['compute_path'] == '/robot1/compute_path_to_pose'
        assert names['navigate'] == '/robot1/navigate_to_pose'
        assert all('/robot2/' not in value for value in names.values())
        publishers = node.get_publishers_info_by_topic(
            '/robot1/task_bids',
        )
        assert len(publishers) == 1
        assert publishers[0].node_namespace == '/robot1'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_local_solo_work_key_ignores_epoch_only_heartbeat():
    """Unchanged local task content must not repeat path work per heartbeat."""
    task = SimpleNamespace(
        physical_signature='task-a',
        approach=(1.0, 2.0),
        approach_yaw=0.0,
        bounds=SimpleNamespace(minimum=(0.0, 0.0), maximum=(2.0, 3.0)),
        visible_reveal_gain=0.5,
        local_ordering_score=0.2,
        local_path_valid=True,
        local_path_length_m=1.5,
    )
    first = SimpleNamespace(
        source_robot_id='robot1', source_session_id='session', epoch=1,
        tasks=(task,),
    )
    heartbeat = SimpleNamespace(
        source_robot_id='robot1', source_session_id='session', epoch=2,
        tasks=(task,),
    )
    first_key = (
        first.source_session_id,
        DistributedFrontierAssignment._snapshot_content_fingerprint(first, first),
    )
    heartbeat_key = (
        heartbeat.source_session_id,
        DistributedFrontierAssignment._snapshot_content_fingerprint(
            heartbeat, heartbeat),
    )
    assert first_key == heartbeat_key

    changed = SimpleNamespace(**{
        **task.__dict__, 'local_path_length_m': 2.0,
    })
    changed_snapshot = SimpleNamespace(
        source_robot_id='robot1', source_session_id='session', epoch=3,
        tasks=(changed,),
    )
    changed_key = (
        changed_snapshot.source_session_id,
        DistributedFrontierAssignment._snapshot_content_fingerprint(
            changed_snapshot, changed_snapshot),
    )
    assert changed_key != first_key
