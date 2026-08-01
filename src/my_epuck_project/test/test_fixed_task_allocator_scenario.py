"""End-to-end pure regression for the ROS fixed-task scenario data."""

import math

from my_epuck_project.distributed_assignment.canonical import build_canonical_union
from my_epuck_project.distributed_assignment.models import Bid, BidBatch
from my_epuck_project.distributed_assignment.ros_conversion import task_from_msg
from my_epuck_project.distributed_assignment.scoring import (
    choose_pair_assignment,
    decisions_match,
)
from my_epuck_project.fixed_task_snapshot_source import (
    alternate_hallway_tasks,
    FixedTaskSnapshotSource,
    SESSIONS,
)

import rclpy


def synthetic_bid(task, origin):
    """Reproduce the node's deterministic straight-line synthetic path."""
    samples = tuple((
        origin[0] + step / 6.0 * (task.approach[0] - origin[0]),
        origin[1] + step / 6.0 * (task.approach[1] - origin[1]),
    ) for step in range(7))
    distance = math.dist(origin, task.approach)
    return Bid(task.canonical_id, True, distance, distance, path=samples)


def test_both_peers_choose_matching_alternate_hallway_decision():
    """Two independent evaluations separate north and east work."""
    first_tasks = tuple(task_from_msg(item) for item in alternate_hallway_tasks(
        'robot1', SESSIONS['robot1'], 1,
    ))
    second_tasks = tuple(task_from_msg(item) for item in alternate_hallway_tasks(
        'robot2', SESSIONS['robot2'], 1,
    ))
    union = build_canonical_union(first_tasks, second_tasks, max_union_tasks=8)
    signatures = {
        member.physical_signature: task.canonical_id
        for task in union.tasks for member in task.members
    }
    first = BidBatch(
        'round', union.union_hash, 'robot1', SESSIONS['robot1'], 1, 3.0,
        tuple(synthetic_bid(task, (0.0, 0.0)) for task in union.tasks),
    )
    second = BidBatch(
        'round', union.union_hash, 'robot2', SESSIONS['robot2'], 1, 3.0,
        tuple(synthetic_bid(task, (0.1, 0.0)) for task in union.tasks),
    )
    robot1_decision = choose_pair_assignment('round', union, first, second)
    robot2_decision = choose_pair_assignment('round', union, first, second)
    assert decisions_match(robot1_decision, robot2_decision)
    assert robot1_decision.robot1_task_id == signatures['north-near']
    assert robot1_decision.robot2_task_id == signatures['east-branch']


def test_fixed_source_republishes_exact_immutable_snapshot():
    """Heartbeat retransmission must not mutate one epoch's provenance fields."""
    rclpy.init(args=[
        '--ros-args', '-r', '__ns:=/robot1', '-p', 'robot_id:=robot1',
    ])
    node = FixedTaskSnapshotSource()

    class Recorder:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    recorder = Recorder()
    node._publisher = recorder
    try:
        node._publish_snapshot()
        node._publish_snapshot()
        assert len(recorder.messages) == 2
        assert recorder.messages[0] is recorder.messages[1]
    finally:
        node.destroy_node()
        rclpy.shutdown()
