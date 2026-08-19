from my_epuck_project.cooperative_regression import (
    lifecycle_startup_action,
    mission_infrastructure_ready,
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


def test_active_manager_is_observed_without_reissuing_startup():
    assert lifecycle_startup_action(True, None) == 'ALREADY_ACTIVE'


def test_pending_startup_is_not_reentered_after_partial_pair_start():
    assert lifecycle_startup_action(False, 'STARTING') == 'WAIT_FOR_PRIOR_ATTEMPT'
    assert lifecycle_startup_action(False, 'ACTIVE') == 'WAIT_FOR_PRIOR_ATTEMPT'


def test_unattempted_manager_receives_the_only_startup_request():
    assert lifecycle_startup_action(False, None) == 'SEND_STARTUP'
