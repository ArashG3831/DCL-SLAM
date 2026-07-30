"""Deterministic tests for command-stage reasoning and bounded captures."""

from pathlib import Path

import rclpy
from rclpy.parameter import Parameter

from my_epuck_project.controller_pipeline_diagnostics import (
    Incident,
    PipelineDiagnosticNode,
    RollingCapture,
    Sample,
    shutdown_node,
    command_reason,
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
