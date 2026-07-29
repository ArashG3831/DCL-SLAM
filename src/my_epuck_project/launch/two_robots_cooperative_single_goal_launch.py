#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def coordinator(robot, peer, test_rank, test_frontier_id):
    return Node(
        package='my_epuck_cooperative_exploration',
        executable='cooperative_frontier_coordinator',
        name='cooperative_frontier_coordinator',
        namespace=robot,
        output='screen',
        parameters=[{
            'robot_id': robot,
            'peer_robot_id': peer,
            'own_candidate_topic': f'/{robot}/frontier_candidates',
            'peer_claim_topic': f'/cslam/{peer}/exploration_claim',
            'own_claim_topic': f'/cslam/{robot}/exploration_claim',
            'navigate_to_pose_action': f'/{robot}/navigate_to_pose',
            'global_frame': 'shared_map',
            'arbitration_window_s': 1.0,
            'claim_heartbeat_rate_hz': 2.0,
            'claim_ttl_s': 3.0,
            'minimum_peer_ttl_s': 0.5,
            'maximum_peer_ttl_s': 10.0,
            'terminal_broadcast_duration_s': 1.0,
            'terminal_broadcast_rate_hz': 4.0,
            'equivalent_frontier_centroid_tolerance_m': 0.15,
            'equivalent_frontier_bbox_margin_m': 0.05,
            'path_cost_tie_tolerance_m': 0.02,
            'maximum_proposals_per_round': 3,
            'maximum_round_exclusion_records': 16,
            'maximum_retired_session_records': 32,
            'require_fresh_candidate_age_s': 5.0,
            'action_server_wait_timeout_s': 10.0,
            'navigation_goal_timeout_s': 90.0,
            'cancellation_ack_timeout_s': 5.0,
            'autostart': True,
            'one_goal_only': True,
            # Verification-only selector. -1 preserves normal ranked selection.
            'test_candidate_rank': test_rank,
            'test_frontier_id': test_frontier_id,
        }],
    )


def generate_launch_description():
    project = get_package_share_directory('my_epuck_project')
    stack = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        project, 'launch', 'two_robots_frontier_candidates_launch.py')))
    robot1_rank = LaunchConfiguration('robot1_test_candidate_rank')
    robot2_rank = LaunchConfiguration('robot2_test_candidate_rank')
    robot1_frontier = LaunchConfiguration('robot1_test_frontier_id')
    robot2_frontier = LaunchConfiguration('robot2_test_frontier_id')
    return LaunchDescription([
        DeclareLaunchArgument('robot1_test_candidate_rank', default_value='-1'),
        DeclareLaunchArgument('robot2_test_candidate_rank', default_value='-1'),
        DeclareLaunchArgument('robot1_test_frontier_id', default_value=''),
        DeclareLaunchArgument('robot2_test_frontier_id', default_value=''),
        stack,
        coordinator('robot1', 'robot2', robot1_rank, robot1_frontier),
        coordinator('robot2', 'robot1', robot2_rank, robot2_frontier),
    ])
