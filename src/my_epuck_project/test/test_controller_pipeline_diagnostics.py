"""Deterministic tests for command-stage reasoning and bounded captures."""

from pathlib import Path
import json
import time

import rclpy
import pytest
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from dwb_msgs.msg import LocalPlanEvaluation, TrajectoryScore, CriticScore
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan

from my_epuck_project.controller_pipeline_diagnostics import (
    Incident,
    PipelineDiagnosticNode,
    RollingCapture,
    Sample,
    shutdown_node,
    is_shutdown_conversion_error,
    DWB_TOP_K,
    command_reason,
    costmap_command_contract_evidence,
    scan_statistics,
    FullCandidateCapturePolicy,
    serialize_full_dwb_evaluation,
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


def test_long_run_rows_and_finalization_remain_bounded(tmp_path):
    rclpy.init()
    node = PipelineDiagnosticNode(parameter_overrides=[
        Parameter('output_root', Parameter.Type.STRING, str(tmp_path)),
        Parameter('max_rows_per_robot', Parameter.Type.INTEGER, 128),
    ])
    try:
        rows = []
        for index in range(5000):
            item = sample(float(index), x=float(index) * 0.001).as_dict()
            item.update({
                'commands': {}, 'command_meta': {}, 'collision_state': {},
                'dwb': {}, 'scan_stats': {}, 'geometry': {}, 'costmap': {},
            })
            rows.append(item)
        for robot in node.robots:
            node.rows[robot].extend(rows)
            node.dropped_rows[robot] += 5000 - len(node.rows[robot])
        started = time.monotonic()
        node.finalize()
        elapsed = time.monotonic() - started
        assert elapsed < 5.0
        assert all(len(values) == 128 for values in node.rows.values())
        assert all(value == 4872 for value in node.dropped_rows.values())
        assert node.shutdown_timing['total_shutdown_ms'] < 5000.0
        assert (tmp_path / 'shutdown_timing.json').is_file()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_byte_budget_caps_large_diagnostic_rows(tmp_path):
    budget = 1 * 1024 * 1024
    rclpy.init()
    node = PipelineDiagnosticNode(parameter_overrides=[
        Parameter('output_root', Parameter.Type.STRING, str(tmp_path)),
        Parameter('max_rows_per_robot', Parameter.Type.INTEGER, 4096),
        Parameter('max_artifact_bytes_per_robot', Parameter.Type.INTEGER, budget),
    ])
    try:
        for index in range(4096):
            item = sample(float(index)).as_dict(compact=True)
            item.update({
                'commands': {}, 'command_meta': {}, 'collision_state': {},
                'dwb': {}, 'scan_stats': {},
                'geometry': {'representative': 'x' * 4096},
                'costmap': {'representative': 'y' * 4096},
            })
            for robot in node.robots:
                node.rows[robot].append(item)
        node.finalize()
        for robot in node.robots:
            assert node.shutdown_timing['artifact_bytes'][robot] <= budget
        assert any(node.shutdown_timing['dropped_by_byte_budget'].values())
        assert 0 < node.shutdown_timing['retained_rows']['robot1'] < 4096
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_total_budget_covers_csv_jsonl_and_report(tmp_path):
    budget = 256 * 1024
    rclpy.init()
    node = PipelineDiagnosticNode(parameter_overrides=[
        Parameter('output_root', Parameter.Type.STRING, str(tmp_path)),
        Parameter('max_rows_per_robot', Parameter.Type.INTEGER, 1024),
        Parameter('max_artifact_bytes_per_robot', Parameter.Type.INTEGER, budget),
    ])
    try:
        for robot in node.robots:
            node.rows[robot].extend(
                sample(float(index), commands={
                    'controller_raw': (0.01, 0.0, float(index)),
                }).as_dict() for index in range(1024))
            incident = node.captures[robot].trigger(robot, sample(1023))
            incident.samples = [{'payload': 'x' * 20000} for _ in range(64)]
        node.finalize()
        for robot in node.robots:
            total = sum(path.stat().st_size for path in tmp_path.glob(
                f'{robot}_controller_*'))
            assert total <= budget
        json.loads((tmp_path / 'robot1_controller_stall_report.json').read_text())
        assert 'configured_total_budget_bytes' in node.shutdown_timing
        assert node.shutdown_timing['written_bytes_by_artifact']
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_long_duration_equivalent_input_is_hard_bounded(tmp_path):
    """Production writers stay bounded when attempted output exceeds 42 MiB."""
    budget = 10 * 1024 * 1024
    rclpy.init()
    node = PipelineDiagnosticNode(parameter_overrides=[
        Parameter('output_root', Parameter.Type.STRING, str(tmp_path)),
        Parameter('max_rows_per_robot', Parameter.Type.INTEGER, 1024),
        Parameter('max_artifact_bytes_per_robot', Parameter.Type.INTEGER, budget),
    ])
    try:
        large = sample(0.0).as_dict()
        large['geometry'] = {'representative': 'x' * 50000}
        large['costmap'] = {'representative': 'y' * 50000}
        for robot in node.robots:
            node.rows[robot].extend(dict(large, sim_s=float(index))
                                     for index in range(1024))
        started = time.monotonic()
        node.finalize()
        elapsed = time.monotonic() - started
        assert elapsed < 2.0
        for robot in node.robots:
            total = sum(path.stat().st_size for path in tmp_path.glob(
                f'{robot}_controller_*'))
            assert total <= budget
        assert all(value > 0 for value in node.shutdown_timing[
            'dropped_by_byte_budget'].values())
        assert (tmp_path / 'shutdown_timing.json').is_file()
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


def test_explicit_executor_shutdown_is_idempotent(tmp_path):
    """The diagnostic executable uses an explicit stable executor lifecycle."""
    rclpy.init()
    node = PipelineDiagnosticNode(parameter_overrides=[
        Parameter('output_root', Parameter.Type.STRING, str(tmp_path)),
    ])
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    shutdown_node(node, executor)
    shutdown_node(node, executor)
    assert list(tmp_path.glob('*'))


def test_shutdown_conversion_error_is_only_accepted_after_context_shutdown():
    """The known pybind error is not suppressed during normal operation."""
    error = RuntimeError('Unable to convert call argument')
    assert is_shutdown_conversion_error(error, True, False)
    assert not is_shutdown_conversion_error(error, False, False)
    assert not is_shutdown_conversion_error(error, True, True)
    assert not is_shutdown_conversion_error(
        RuntimeError('application callback failure'), True, False)


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
    assert result['invalid_rejection_counts'] == {'BaseObstacle': 1}
    assert result['valid_critic_extrema']['PathAlign'][
        'minimum_weighted_contribution'] == pytest.approx(1.0)
    assert result['valid_critic_extrema']['PathAlign'][
        'maximum_weighted_contribution'] == pytest.approx(2.0)
    assert result['top_overall'][0]['trajectory_index'] == 0


def test_full_candidate_serializer_keeps_every_jazzy_trajectory_and_limitations():
    evaluation = LocalPlanEvaluation()
    evaluation.header.frame_id = 'robot1/odom'
    evaluation.twists = [_trajectory(0.0, 1.0, [('GoalDist', 1.0, 24.0)]),
                         _trajectory(0.026, 2.0, [('PathDist', 1.0, 24.0)]),
                         _trajectory(0.052, -1.0, [('BaseObstacle', -1.0, 0.2)])]
    evaluation.best_index = 1
    serialized = serialize_full_dwb_evaluation(evaluation)
    assert len(serialized['trajectories']) == 3
    assert serialized['selected_index'] == 1
    assert serialized['trajectories'][1]['selected'] is True
    assert serialized['trajectories'][2]['valid'] is False
    assert 'explicit_invalidation_reason' in serialized['unavailable_fields']


def test_full_candidate_policy_is_triggered_deduplicated_and_hard_bounded():
    policy = FullCandidateCapturePolicy(limit=1)
    context = {'goal': {'label': 'frontier_robot1'}, 'robot_state': {'shared_map_pose': {'x_m': 0, 'y_m': 0}},
               'path_revisions': {'global': 1, 'local': 1}, 'local_costmap': {'stamp_s': 1.0}}
    evaluation = {'trajectories': [{'selected': True, 'valid': True,
                                    'velocity': {'linear_x_mps': 0.0, 'angular_z_radps': 0.2},
                                    'total_score': 1.0, 'critics': []}]}
    goal = ('robot1', 'frontier_robot1', 1.0)
    assert policy.observe_stall(0.0, goal, 'ANGULAR_ONLY', context, evaluation) == []
    captures = policy.observe_stall(2.0, goal, 'ANGULAR_ONLY', context, evaluation)
    assert captures[0][0] == 'ANGULAR_ONLY_OVER_2S'
    assert policy.observe_stall(3.0, goal, 'ZERO', context, evaluation) == []
    assert len(policy.captured_stall) == 1


def test_full_candidate_policy_samples_diverse_usable_healthy_forward_only():
    policy = FullCandidateCapturePolicy()
    base = {'robot_state': {}, 'goal': {'source': 'manual', 'distance_to_goal_m': 2.0}}
    forward = {'trajectories': [{'selected': True, 'valid': True,
                                 'velocity': {'linear_x_mps': 0.026, 'angular_z_radps': 0.0}}]}
    assert policy.observe_healthy(base, forward) == 'HEALTHY_MANUAL_FAR_STRAIGHT'
    assert policy.observe_healthy(base, forward) is None


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


def test_costmap_evidence_distinguishes_clear_inflated_lethal_and_unknown():
    """Bounded trajectory evidence retains cost and collision semantics."""
    costmap = {
        'frame': 'robot1/local_costmap', 'stamp_sec': 10,
        'stamp_nanosec': 0, 'resolution': 0.1,
        'origin': {'x': -1.0, 'y': -1.0}, 'robot_cell': {'x': 10, 'y': 10},
        'cells': [{'x': x, 'y': 10, 'value': 0} for x in range(10, 21)] +
                 [{'x': 15, 'y': 11, 'value': 100},
                  {'x': 16, 'y': 11, 'value': 254},
                  {'x': 17, 'y': 11, 'value': -1}],
    }
    dwb = {
        'selected': {'endpoint': {'x': 0.4, 'y': 0.0}},
        'best_valid_forward': {'endpoint': {'x': 0.2, 'y': 0.0}},
    }
    result = costmap_command_contract_evidence(dwb, costmap, (0.0, 0.0, 0.0))
    assert result['selected']['available']
    assert not result['selected']['lethal_crossed']
    assert result['selected']['maximum_cost'] == 0
    assert result['best_executable_forward']['minimum_clearance_m'] is not None


def test_costmap_evidence_reports_missing_or_stale_input():
    """Missing crops are explicit rather than silently treated as safe."""
    result = costmap_command_contract_evidence(
        {'selected': {}, 'best_valid_forward': {}}, {}, (0.0, 0.0, 0.0))
    assert result['selected']['available'] is False
    assert result['best_executable_forward']['available'] is False
