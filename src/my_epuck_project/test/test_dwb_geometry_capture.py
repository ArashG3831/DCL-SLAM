"""Focused tests for the bounded geometry-capture policy and serializers."""

import math

from dwb_msgs.msg import LocalPlanEvaluation, TrajectoryScore
from geometry_msgs.msg import Pose2D
from nav_2d_msgs.msg import Twist2D
from nav_msgs.msg import OccupancyGrid, Path

from my_epuck_project.dwb_geometry_capture import (
    GeometryCapturePolicy,
    serialize_costmap_message,
    serialize_path_message,
    select_sequence_indices,
    trajectory_sequence_record,
)


def test_first_frontier_trigger_and_bounded_zero_window():
    policy = GeometryCapturePolicy()
    key = ('robot1', 'frontier_robot1', 10.0)
    assert not policy.observe_trigger(12.0, key, 'manual', 'ANGULAR_ONLY_OVER_2S')
    assert policy.observe_trigger(12.0, key, 'frontier', 'ANGULAR_ONLY_OVER_2S')
    assert policy.window_start() == 10.0
    assert policy.observe_state(13.0, key, 'ANGULAR_ONLY') == 'ACTIVE'
    assert policy.observe_state(14.0, key, 'ZERO') == 'ACTIVE'
    assert policy.zero_started_sim_s == 14.0
    assert policy.observe_state(18.99, key, 'ZERO') == 'ACTIVE'
    assert policy.observe_state(19.0, key, 'ZERO') == 'END'


def test_policy_does_not_select_a_second_frontier_goal():
    policy = GeometryCapturePolicy()
    assert not policy.observe_trigger(1.0, ('r1', 'first', 1), 'frontier', 'OTHER')
    assert not policy.observe_trigger(2.0, ('r1', 'second', 2), 'frontier', 'ANGULAR_ONLY_OVER_2S')


def test_path_serialization_preserves_order_and_yaw():
    message = Path()
    message.header.frame_id = 'shared_map'
    for x in (1.0, 2.0):
        pose = message.poses.add() if hasattr(message.poses, 'add') else None
        if pose is None:
            from geometry_msgs.msg import PoseStamped
            pose = PoseStamped()
            message.poses.append(pose)
        pose.pose.position.x = x
        pose.pose.orientation.w = 1.0
    result = serialize_path_message(message, 3, 'transformed')
    assert [item['index'] for item in result['points']] == [0, 1]
    assert result['points'][0]['x_m'] == 1.0
    assert math.isclose(result['length_m'], 1.0)


def _score(index, vx, wz, total):
    score = TrajectoryScore()
    score.traj.velocity = Twist2D(x=vx, y=0.0, theta=wz)
    score.traj.poses = [Pose2D(x=0.0, y=0.0, theta=0.0),
                        Pose2D(x=vx, y=0.0, theta=wz)]
    score.total = total
    return score


def test_sequence_selection_is_bounded_and_keeps_selected_order():
    message = LocalPlanEvaluation()
    message.best_index = 2
    message.twists = [
        _score(0, 0.0, 0.2, 4.0), _score(1, 0.026, 0.2, 3.0),
        _score(2, 0.0, 0.0, 2.0), _score(3, 0.052, 0.1, 2.5),
        _score(4, 0.078, 0.1, 2.8), _score(5, 0.104, 0.1, 2.9),
    ]
    roles = select_sequence_indices(message)
    assert roles[0] == (2, 'selected')
    assert len(roles) <= 6
    assert len({index for index, _ in roles}) == len(roles)
    record = trajectory_sequence_record(message.twists[1], 1, 'best_forward')
    assert [pose['index'] for pose in record['poses']] == [0, 1]


def test_costmap_serialization_keeps_raw_values_and_metadata():
    grid = OccupancyGrid()
    grid.header.frame_id = 'robot1/local_costmap'
    grid.info.width = 2
    grid.info.height = 1
    grid.info.resolution = 0.005
    grid.data = [0, 100]
    result = serialize_costmap_message(grid, 'TRIGGER', {'column': 0, 'row': 0})
    assert result['data'] == [0, 100]
    assert result['width'] == 2
    assert result['capture_reason'] == 'TRIGGER'
