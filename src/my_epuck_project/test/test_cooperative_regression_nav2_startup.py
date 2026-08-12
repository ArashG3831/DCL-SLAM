from my_epuck_project.cooperative_regression import mission_infrastructure_ready


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
