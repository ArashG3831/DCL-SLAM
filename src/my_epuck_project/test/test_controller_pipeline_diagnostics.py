"""Deterministic tests for command-stage reasoning and bounded captures."""

from pathlib import Path

import rclpy
import pytest
from rclpy.parameter import Parameter
from dwb_msgs.msg import LocalPlanEvaluation, TrajectoryScore, CriticScore
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan

from my_epuck_project.controller_pipeline_diagnostics import (
    Incident,
    PipelineDiagnosticNode,
    RollingCapture,
    Sample,
    shutdown_node,
    DWB_TOP_K,
    command_reason,
    scan_statistics,
    summarize_dwb_evaluation,
)


def test_node_constructor_creates_bounded_two_robot_observer(tmp_path):
    rclpy.init()
    node = PipelineDiagnosticNode(parameter_overrides=[
        Parameter('output_root', Parameter.Type.STRING, str(tmp_path)),
    ])
    try:
        assert isinstance(node._subscriptions, list)
        assert not callable(node._subscriptions)
        topics = {
            subscription.topic_name
            for subscription in node._subscriptions
        }
        assert '/robot1/cmd_vel_nav' in topics
        assert '/robot2/cmd_vel_nav' in topics
        assert '/robot1/cmd_vel' in topics
        assert '/robot2/cmd_vel' in topics
        assert '/robot1/scan_d500' in topics
        assert '/robot1/scan_d500_fixed' in topics
        assert '/robot1/scan_d500_slam' in topics
        assert '/robot2/scan_d500_slam' in topics
        assert not hasattr(PipelineDiagnosticNode, '_subscriptions')
        node.finalize()
        files = list(Path(node.root).glob('*'))
        assert files
        assert max(path.stat().st_size for path in files) < 1_000_000
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_external_ros_shutdown_flushes_and_destroys_cleanly(tmp_path):
    rclpy.init()
    node = PipelineDiagnosticNode(parameter_overrides=[
        Parameter('output_root', Parameter.Type.STRING, str(tmp_path)),
    ])
    rclpy.shutdown()
    shutdown_node(node)
    assert list(tmp_path.glob('*'))


def sample(t, active=True, distance=1.0, x=0.0, commands=None, state=None):
    return Sample(t, t, pose_x=x, active_goal=active,
                  distance_remaining=distance, commands=commands or {},
                  collision_state=state or {})


def test_stage_reasoning_distinguishes_controller_smoother_collision_and_base():
    assert command_reason([sample(1, commands={'controller_raw': (0.0, 0.0, 1.0)})])[0] == 'DWB_ZERO_COMMAND'
    assert command_reason([sample(1, commands={'controller_raw': (0.04, 0.0, 1.0), 'smoother_output': (0.0, 0.0, 1.0)})])[0] == 'VELOCITY_SMOOTHER_SUPPRESSION'
    assert command_reason([sample(1, commands={'controller_raw': (0.04, 0.0, 1.0), 'smoother_output': (0.04, 0.0, 1.0), 'collision_output': (0.0, 0.0, 1.0)}, state={'action_type': 1})])[0] == 'COLLISION_MONITOR_STOP'
    assert command_reason([sample(1, commands={'controller_raw': (0.04, 0.0, 1.0), 'smoother_output': (0.04, 0.0, 1.0), 'collision_output': (0.04, 0.0, 1.0), 'final_base_command': (0.0, 0.0, 1.0)})])[0] == 'FINAL_COMMAND_NOT_DELIVERED'
    assert command_reason([sample(1, commands={
        'controller_raw': (0.04, 0.0, 1.0),
        'smoother_output': (0.04, 0.0, 1.0),
        'collision_output': (0.04, 0.0, 1.0),
        'final_base_command': (0.04, 0.0, 1.0),
    })])[0] == 'BASE_LINEAR_NOT_RESPONDING'


def test_base_response_is_axis_specific_and_windowed():
    common = {
        'controller_raw': (0.04, 0.0, 3.0),
        'smoother_output': (0.04, 0.0, 3.0),
        'collision_output': (0.04, 0.0, 3.0),
        'final_base_command': (0.04, 0.0, 3.0),
    }
    assert command_reason([sample(0, commands=common, x=0),
                           sample(3, commands=common, x=0.1)])[0] == 'UNKNOWN_COMMAND_STALL'
    assert command_reason([sample(0, commands=common, x=0),
                           sample(3, commands=common, x=0)])[0] == 'BASE_LINEAR_NOT_RESPONDING'
    angular = {key: (0.0, 0.3, 3.0) for key in common}
    assert command_reason([sample(0, commands=angular),
                           Sample(3, 3, pose_yaw=0.5, odom_wz=0.3,
                                  active_goal=True, distance_remaining=1.0,
                                  commands=angular)])[0] == 'UNKNOWN_COMMAND_STALL'
    assert command_reason([sample(0, commands=angular),
                           Sample(3, 3, pose_yaw=0.0, odom_wz=0.0,
                                  active_goal=True, distance_remaining=1.0,
                                  commands=angular)])[0] == 'BASE_ANGULAR_NOT_RESPONDING'
    combined = {key: (0.04, 0.3, 3.0) for key in common}
    assert command_reason([Sample(0, 0, pose_x=0.0, pose_yaw=0.0,
                                  commands=combined),
                           Sample(3, 3, pose_x=0.1, pose_yaw=0.5,
                                  odom_vx=0.04, odom_wz=0.3,
                                  active_goal=True, distance_remaining=1.0,
                                  commands=combined)])[0] == 'UNKNOWN_COMMAND_STALL'
    delayed = [Sample(0, 0, commands=common),
               Sample(1, 1, commands=common),
               Sample(3, 3, odom_vx=0.04, active_goal=True,
                      distance_remaining=1.0, commands=common)]
    assert command_reason(delayed)[0] == 'UNKNOWN_COMMAND_STALL'
    noisy = [Sample(0, 0, odom_vx=0.002, commands=common),
             Sample(3, 3, odom_vx=-0.002, active_goal=True,
                    distance_remaining=1.0, commands=common)]
    assert command_reason(noisy)[0] == 'BASE_LINEAR_NOT_RESPONDING'


def test_stale_controller_zero_is_not_dwb_zero_command():
    stages = {key: (0.0, 0.0, 0.0) for key in (
        'controller_raw', 'smoother_output', 'collision_output',
        'final_base_command')}
    result = command_reason([sample(3, commands=stages)])
    assert result[0] == 'COMMAND_STAGE_STALE'


def test_fresh_controller_zero_is_dwb_zero_command():
    stages = {key: (0.0, 0.0, 3.0) for key in (
        'controller_raw', 'smoother_output', 'collision_output',
        'final_base_command')}
    result = command_reason([sample(3, commands=stages)])
    assert result[0] == 'DWB_ZERO_COMMAND'


def test_diagnostic_launch_is_opt_in_and_dwb_publications_are_gated():
    launch_dir = Path(__file__).resolve().parents[1] / 'launch'
    observed = (launch_dir / 'two_robots_observed_continuous_exploration_launch.py').read_text()
    stack = (launch_dir / 'two_robots_teammate_filtered_stack_launch.py').read_text()
    assert "DeclareLaunchArgument('diagnostic_mode', default_value='false'" in observed
    assert "condition=IfCondition(LaunchConfiguration('diagnostic_mode'))" in observed
    for parameter in (
            'publish_evaluation', 'publish_local_plan',
            'publish_global_plan', 'publish_transformed_global_plan',
            'publish_cost_grid_pc'):
        assert f"FollowPath.{parameter}'" in stack
        assert "LaunchConfiguration('diagnostic_mode')" in stack
    assert "DeclareLaunchArgument('diagnostic_mode', default_value='false'" in stack


def test_rolling_capture_keeps_history_post_trigger_and_one_incident():
    capture = RollingCapture(pre_s=5, post_s=3, max_samples=6, max_incidents=1)
    for t in range(6):
        item = sample(float(t), x=0.0)
        capture.add(item)
    incident = capture.trigger('robot1', sample(5, x=0.0))
    assert incident.incident_id == 'robot1-001'
    assert incident.start_sim_s == 0
    for t in (6, 7, 8):
        capture.add(sample(float(t), x=0.0))
    assert capture.active is None
    assert len(capture.incidents) == 1
    assert len(capture.incidents[0].samples) >= 4
    assert capture.trigger('robot1', sample(9)) is None


def test_legitimate_turn_and_goal_tolerance_do_not_trigger_by_themselves():
    turning = sample(4, distance=1.0, commands={'controller_raw': (0.0, 0.3, 4.0)})
    assert command_reason([turning])[0] == 'DWB_ROTATION_ONLY'
    capture = RollingCapture()
    for t in range(4):
        capture.add(sample(float(t), active=True, distance=0.1))
    capture.clear(4)
    assert not capture.incidents


def test_incident_schema_has_unique_identity_and_unknown_missing_evidence():
    incident = Incident('robot2-001', 'robot2', 1.0, 6.0, True, 0.8)
    value = incident.as_dict()
    assert value['incident_id'] == 'robot2-001'
    reason = command_reason([sample(1, commands={})])
    assert reason[0] == 'UNKNOWN_COMMAND_STALL'
    assert reason[3]


def _trajectory(vx, total, raw, theta=0.0):
    score = TrajectoryScore()
    score.traj.velocity.x = vx
    score.traj.velocity.theta = theta
    score.traj.poses = [Pose2D(x=0.0, y=0.0, theta=0.0),
                        Pose2D(x=vx, y=0.0, theta=theta)]
    score.scores = [CriticScore(name=name, raw_score=value, scale=scale)
                    for name, value, scale in raw]
    score.total = total
    return score


def test_jazzy_dwb_validity_selected_and_best_forward_semantics():
    evaluation = LocalPlanEvaluation()
    evaluation.twists = [
        _trajectory(0.0, 1.0, [('PathAlign', 1.0, 1.0),
                               ('GoalAlign', 0.2, 1.0)], theta=0.3),
        _trajectory(0.05, 2.8, [('PathAlign', 2.0, 1.0),
                                ('GoalAlign', 0.8, 1.0)]),
        _trajectory(0.04, -1.0, [('BaseObstacle', -1.0, 1.0)]),
    ]
    evaluation.best_index = 0
    result = summarize_dwb_evaluation(evaluation)
    assert result['valid_count'] == 2
    assert result['forward_valid_count'] == 1
    assert result['selected']['trajectory_index'] == 0
    assert result['best_valid_forward']['trajectory_index'] == 1
    assert result['score_difference_forward_minus_selected'] == pytest.approx(1.8)
    assert result['best_valid_forward']['critics'][0]['weighted_contribution'] == 2.0
    assert result['dominant_selected_advantage'][0]['name'] == 'PathAlign'
    assert result['invalid_count'] == 1
    assert result['top_overall'][0]['trajectory_index'] == 0


def test_jazzy_dwb_invalid_best_index_and_no_forward_are_explicit():
    evaluation = LocalPlanEvaluation()
    evaluation.twists = [_trajectory(0.001, 1.0, [('GoalAlign', 1.0, 1.0)])]
    evaluation.best_index = 99
    result = summarize_dwb_evaluation(evaluation)
    assert result['selected'] is None
    assert result['best_valid_forward'] is None
    assert result['semantics']['best_index_valid'] is False
    assert result['forward_valid'] is False


def test_jazzy_dwb_negative_best_score_is_not_a_selected_valid_trajectory():
    evaluation = LocalPlanEvaluation()
    evaluation.twists = [_trajectory(0.0, -1.0, [('BaseObstacle', -1.0, 1.0)])]
    evaluation.best_index = 0
    result = summarize_dwb_evaluation(evaluation)
    assert result['selected'] is None
    assert result['semantics']['best_index_valid'] is False
    assert result['invalid_count'] == 1


def test_dwb_top_k_is_bounded_and_nonfinite_is_not_valid():
    evaluation = LocalPlanEvaluation()
    evaluation.twists = [_trajectory(0.01 + i * 0.001, float(i), [], theta=0.0)
                         for i in range(DWB_TOP_K + 3)]
    evaluation.twists[-1].total = float('nan')
    evaluation.best_index = 0
    result = summarize_dwb_evaluation(evaluation, top_k=DWB_TOP_K)
    assert len(result['top_overall']) == DWB_TOP_K
    assert len(result['top_valid_forward']) == DWB_TOP_K
    assert result['invalid_count'] == 1


def test_scan_statistics_distinguishes_no_return_values_and_sectors():
    scan = LaserScan()
    scan.header.frame_id = 'robot1/d500_lidar'
    scan.range_min = 0.05
    scan.range_max = 12.0
    scan.angle_min = -3.141592653589793
    scan.angle_increment = 3.141592653589793 / 4.0
    scan.angle_max = scan.angle_min + 7 * scan.angle_increment
    scan.ranges = [float('inf'), float('-inf'), float('nan'), 0.0,
                   12.0, 11.995, 1.0, 2.0]
    result = scan_statistics(scan)
    assert result['beam_count'] == 8
    assert result['positive_infinite_count'] == 1
    assert result['negative_infinite_count'] == 1
    assert result['nan_count'] == 1
    assert result['zero_or_negative_count'] == 1
    assert result['exact_range_max_count'] == 1
    assert result['near_range_max_count'] == 2
    assert result['sectors']['front']['beam_count'] >= 1
