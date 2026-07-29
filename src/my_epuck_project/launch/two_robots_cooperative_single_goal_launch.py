#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from my_epuck_project.cooperative_profiles import profile


def coordinator(
        robot, peer, test_rank, test_frontier_id,
        minimum_known_cell_gain):
    return Node(
        package='my_epuck_cooperative_exploration',
        executable='cooperative_frontier_coordinator',
        name='cooperative_frontier_coordinator',
        namespace=robot,
        output='screen',
        parameters=[{
            'robot_id': robot,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'peer_robot_id': peer,
            'operating_mode': LaunchConfiguration('coordinator_mode'),
            'one_goal_only': LaunchConfiguration('one_goal_only'),
            'own_candidate_topic': f'/{robot}/frontier_candidates',
            'own_shared_map_topic': f'/{robot}/shared_map',
            'peer_claim_topic': f'/cslam/{peer}/exploration_claim',
            'own_claim_topic': f'/cslam/{robot}/exploration_claim',
            'peer_status_topic': f'/cslam/{peer}/exploration_status',
            'own_status_topic': f'/cslam/{robot}/exploration_status',
            'own_event_topic': f'/cslam/{robot}/exploration_event',
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
            'cycle_cooldown_s': LaunchConfiguration('cycle_cooldown_s'),
            'success_region_cooldown_s':
                LaunchConfiguration('success_region_cooldown_s'),
            'first_failure_suppression_s':
                LaunchConfiguration('first_failure_suppression_s'),
            'second_failure_suppression_s':
                LaunchConfiguration('second_failure_suppression_s'),
            'maximum_failure_suppression_s':
                LaunchConfiguration('maximum_failure_suppression_s'),
            'no_candidate_grace_s':
                LaunchConfiguration('no_candidate_grace_s'),
            'map_stability_window_s':
                LaunchConfiguration('map_stability_window_s'),
            'completion_consensus_grace_s':
                LaunchConfiguration('completion_consensus_grace_s'),
            'minimum_known_cell_gain_for_activity':
                minimum_known_cell_gain,
            'autostart': LaunchConfiguration('coordinator_autostart'),
            # Verification-only selector. -1 preserves normal ranked selection.
            'test_candidate_rank': test_rank,
            'test_frontier_id': test_frontier_id,
        }],
    )


def launch_setup(context):
    project = get_package_share_directory('my_epuck_project')
    selected = profile(
        LaunchConfiguration('world_profile').perform(context),
        os.path.join(project, 'worlds'),
    )
    stack = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        project, 'launch', 'two_robots_frontier_candidates_launch.py')),
        launch_arguments={
            'world_profile': selected['name'],
            'webots_port': LaunchConfiguration('webots_port'),
            'webots_mode': LaunchConfiguration('webots_mode'),
            'webots_gui': LaunchConfiguration('webots_gui'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'sensor_profile': LaunchConfiguration('sensor_profile'),
        }.items(),
    )
    robot1_rank = LaunchConfiguration('robot1_test_candidate_rank')
    robot2_rank = LaunchConfiguration('robot2_test_candidate_rank')
    robot1_frontier = LaunchConfiguration('robot1_test_frontier_id')
    robot2_frontier = LaunchConfiguration('robot2_test_frontier_id')
    minimum_gain = selected['minimum_known_cell_gain_for_activity']
    return [
        stack,
        coordinator(
            'robot1', 'robot2', robot1_rank, robot1_frontier, minimum_gain),
        coordinator(
            'robot2', 'robot1', robot2_rank, robot2_frontier, minimum_gain),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_profile',
            default_value='small',
            choices=['large', 'small'],
        ),
        DeclareLaunchArgument('coordinator_mode', default_value='single_goal'),
        DeclareLaunchArgument('webots_port', default_value='23000'),
        DeclareLaunchArgument('webots_mode', default_value='realtime'),
        DeclareLaunchArgument('webots_gui', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('sensor_profile', default_value='full',
                              choices=['full', 'throughput']),
        DeclareLaunchArgument('one_goal_only', default_value='true'),
        DeclareLaunchArgument('coordinator_autostart', default_value='true'),
        DeclareLaunchArgument('cycle_cooldown_s', default_value='2.5'),
        DeclareLaunchArgument('success_region_cooldown_s', default_value='25.0'),
        DeclareLaunchArgument('first_failure_suppression_s', default_value='20.0'),
        DeclareLaunchArgument('second_failure_suppression_s', default_value='60.0'),
        DeclareLaunchArgument('maximum_failure_suppression_s', default_value='180.0'),
        DeclareLaunchArgument('no_candidate_grace_s', default_value='18.0'),
        DeclareLaunchArgument('map_stability_window_s', default_value='15.0'),
        DeclareLaunchArgument('completion_consensus_grace_s', default_value='8.0'),
        DeclareLaunchArgument('robot1_test_candidate_rank', default_value='-1'),
        DeclareLaunchArgument('robot2_test_candidate_rank', default_value='-1'),
        DeclareLaunchArgument('robot1_test_frontier_id', default_value=''),
        DeclareLaunchArgument('robot2_test_frontier_id', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
