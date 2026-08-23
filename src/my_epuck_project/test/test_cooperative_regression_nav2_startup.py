from my_epuck_project.cooperative_regression import (
    abnormal_ros_exit_evidence,
    collector_clock_readiness,
    lifecycle_startup_action,
    MAX_CAMPAIGN_GRACEFUL_SHUTDOWN_S,
    NAV2_MANAGER_STARTUP_CALL_SLICE_S,
    nav2_startup_preflight_mode,
    nav2_manager_startup_call_timeout,
    mission_infrastructure_ready,
    nav2_manager_name,
    nav2_node_names,
    tf_readiness_requirements,
)


def test_campaign_cleanup_grace_period_is_bounded():
    assert MAX_CAMPAIGN_GRACEFUL_SHUTDOWN_S <= 20.0


def test_abnormal_ros_exit_evidence_detects_dds_abort_without_false_positive(
        tmp_path):
    path = tmp_path / 'launch.log'
    path.write_text('normal\n', encoding='utf-8')
    offset, evidence = abnormal_ros_exit_evidence(path)
    assert not evidence
    path.write_text(
        'normal\n'
        'frontier: ddsi_entity_index.c:271: Assertion failed\n'
        'process has died [pid 7, exit code -6]\n', encoding='utf-8')
    offset, evidence = abnormal_ros_exit_evidence(path, offset)
    assert len(evidence) == 2
    assert any('ddsi_entity_index' in line for line in evidence)
    assert any('exit code -6' in line for line in evidence)


def test_abnormal_ros_exit_evidence_detects_dds_segfault(tmp_path):
    path = tmp_path / 'launch.log'
    path.write_text(
        'process has died [pid 9, exit code -11]\n'
        'potentially unexpected fatal signal 11\n', encoding='utf-8')
    _, evidence = abnormal_ros_exit_evidence(path)
    assert len(evidence) == 2


NODES = [
    '/robot1/distributed_frontier_assignment',
    '/robot2/distributed_frontier_assignment',
    '/robot1/map_fusion',
    '/robot2/map_fusion',
]


def test_mission_readiness_requires_successful_nav2_activation():
    assert not mission_infrastructure_ready(True, True, False, NODES)
    assert mission_infrastructure_ready(True, True, True, NODES)


def test_mission_readiness_keeps_existing_clock_tf_and_graph_gates():
    assert not mission_infrastructure_ready(False, True, True, NODES)
    assert not mission_infrastructure_ready(True, False, True, NODES)
    assert not mission_infrastructure_ready(True, True, True, NODES[:-1])


def test_unknown_pose_readiness_does_not_require_prehandoff_fusion():
    nodes = [
        '/robot1/local_distributed_frontier_assignment',
        '/robot2/local_distributed_frontier_assignment',
        '/robot1/unknown_pose_frontend',
        '/robot2/unknown_pose_frontend',
    ]
    assert mission_infrastructure_ready(True, True, True, nodes, True)
    assert not mission_infrastructure_ready(True, True, True, NODES, True)


def test_unknown_pose_tf_readiness_requires_local_map_and_odom_chains():
    requirements = tf_readiness_requirements(unknown_initial_pose=True)
    assert requirements == [
        ('robot1/base_footprint', 'robot1/odom', 'odom_to_base'),
        ('robot1/map', 'robot1/base_footprint', 'local_map_to_base'),
        ('robot2/base_footprint', 'robot2/odom', 'odom_to_base'),
        ('robot2/map', 'robot2/base_footprint', 'local_map_to_base'),
    ]
    assert len(tf_readiness_requirements()) == 4


def test_active_manager_is_observed_without_reissuing_startup():
    assert lifecycle_startup_action(True, None) == 'ALREADY_ACTIVE'


def test_pending_startup_is_not_reentered_after_partial_pair_start():
    assert lifecycle_startup_action(False, 'STARTING') == 'WAIT_FOR_PRIOR_ATTEMPT'
    assert lifecycle_startup_action(False, 'ACTIVE') == 'WAIT_FOR_PRIOR_ATTEMPT'


def test_unattempted_manager_receives_the_only_startup_request():
    assert lifecycle_startup_action(False, None) == 'SEND_STARTUP'


def test_unknown_pose_nav2_readiness_uses_local_phase_names():
    assert nav2_node_names(True)[0] == 'local_controller_server'
    assert nav2_node_names(True)[-1] == 'local_waypoint_follower'
    assert nav2_manager_name(True) == 'local_lifecycle_manager_navigation'


def test_known_pose_nav2_readiness_keeps_existing_names():
    assert nav2_node_names(False)[0] == 'controller_server'
    assert nav2_manager_name(False) == 'lifecycle_manager_navigation'


def test_unknown_pose_nav2_uses_manager_ack_before_plugin_state_queries():
    assert nav2_startup_preflight_mode(True) == 'manager_ack'
    assert nav2_startup_preflight_mode(False) == 'state_then_manager'


def test_manager_startup_wait_is_bounded_for_readiness_retry():
    assert nav2_manager_startup_call_timeout(300.0) == \
        NAV2_MANAGER_STARTUP_CALL_SLICE_S
    assert nav2_manager_startup_call_timeout(2.0) == 2.0


def test_collector_clock_fallback_requires_two_increasing_observations():
    assert not collector_clock_readiness([{
        'sim_time_seconds': 1.0, 'wall_elapsed_s': 2.0,
    }])[0]
    ready, details = collector_clock_readiness([
        {'sim_time_seconds': 1.0, 'wall_elapsed_s': 2.0},
        {'sim_time_seconds': 1.5, 'wall_elapsed_s': 2.5},
    ])
    assert ready
    assert details['source'] == 'collector_status.json'
    assert collector_clock_readiness([
        {'sim_time_seconds': 1.5, 'wall_elapsed_s': 2.0},
        {'sim_time_seconds': 1.5, 'wall_elapsed_s': 2.5},
    ])[0] is False
