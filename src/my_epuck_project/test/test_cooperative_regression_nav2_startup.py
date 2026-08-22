from my_epuck_project.cooperative_regression import (
    lifecycle_startup_action,
    mission_infrastructure_ready,
    nav2_manager_name,
    nav2_node_names,
    tf_readiness_requirements,
)


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


def test_unknown_pose_tf_readiness_requires_only_local_odom_chain():
    requirements = tf_readiness_requirements(unknown_initial_pose=True)
    assert requirements == [
        ('robot1/base_footprint', 'robot1/odom', 'odom_to_base'),
        ('robot2/base_footprint', 'robot2/odom', 'odom_to_base'),
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
