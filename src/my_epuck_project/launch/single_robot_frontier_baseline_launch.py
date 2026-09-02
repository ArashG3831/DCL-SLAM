#!/usr/bin/env python3
"""Upstream single autonomous explorer on Robot 1; Robot 2 remains passive.

This is a diagnostic-only launch.  It deliberately starts the proven local
unknown-pose mapping/Nav2 infrastructure, but no handoff, fusion, shared map,
distributed allocator, or Robot 2 explorer.  The upstream node therefore
sees exactly Robot 1's local SLAM map and drives Robot 1's local Nav2 action.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Start the two-robot infrastructure and only Robot 1's upstream explorer."""
    package_dir = get_package_share_directory('my_epuck_project')
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_dir, 'launch', 'two_robots_teammate_filtered_stack_launch.py',
        )),
        launch_arguments={
            'world_profile': LaunchConfiguration('world_profile'),
            'webots_port': LaunchConfiguration('webots_port'),
            'webots_controller_port': LaunchConfiguration(
                'webots_controller_port'),
            'world_path': LaunchConfiguration('world_path'),
            'webots_mode': LaunchConfiguration('webots_mode'),
            'webots_gui': LaunchConfiguration('webots_gui'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_scan_matching': LaunchConfiguration('use_scan_matching'),
            'do_loop_closing': 'false',
            'unknown_initial_pose': 'true',
            'launch_mapping': 'true',
            'launch_shared_stack': 'false',
            'launch_shared_fusion': 'false',
            'phase_already_aligned': 'false',
            'nav2_autostart': 'true',
        }.items(),
    )
    explorer = Node(
        package='frontier_exploration_ros2', executable='frontier_explorer',
        name='frontier_explorer', namespace='robot1', output='screen',
        remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
        parameters=[{
            # Local pre-handoff Nav2 is intentionally retained by the stack
            # under the normal namespaced action and costmap topics.  It uses
            # ``robot1/map`` as its global frame, so this control cannot
            # accidentally consume Robot 2 or a fused map.
            'map_topic': '/robot1/map',
            'costmap_topic': '/robot1/global_costmap/costmap',
            'local_costmap_topic': '/robot1/local_costmap/costmap',
            'navigate_to_pose_action_name': 'navigate_to_pose',
            'global_frame': 'robot1/map',
            'robot_base_frame': 'robot1/base_footprint',
            # Match the upstream bounded-DP route model, rather than the
            # project adapter's scalar-only candidate publication path.
            'mrtsp_solver': 'dp',
            'dp_solver_candidate_limit': 12,
            'dp_planning_horizon': 8,
            # The upstream core's declared default is non-preemptive.  Keep
            # that lifecycle for this bounded control: map updates may revise
            # an MRTSP route, but must not cancel a still-active Nav2 goal on
            # every SLAM update.
            'goal_preemption_enabled': False,
            'goal_skip_on_blocked_goal': True,
            'post_goal_settle_enabled': False,
            'frontier_suppression_enabled': False,
            'completion_event_enabled': True,
            'return_to_start_on_complete': False,
            'sensor_effective_range_m': 1.5,
            'weight_distance_wd': 1.0,
            'weight_gain_ws': 1.0,
            'max_linear_speed_vmax': 0.50,
            'max_angular_speed_wmax': 1.0,
            'frontier_selection_min_distance': 0.8,
            'frontier_candidate_min_goal_distance_m': 0.8,
            'frontier_visit_tolerance': 0.40,
            'min_frontier_size_cells': 5,
            'goal_preemption_lidar_range_m': 12.0,
            # The upstream node is deliberately started through its existing
            # control service by the diagnostic runner after the local Nav2
            # costmap has received a stable map.  This avoids treating
            # startup-time costmap resize failures as frontier behavior.
            'autostart': LaunchConfiguration('explorer_autostart'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_profile', default_value='large_unknown_pose_close_start_20ms_scan_matching',
            choices=['large_unknown_pose_close_start_20ms_scan_matching',
                     'large_unknown_pose_close_start_20ms', 'large']),
        DeclareLaunchArgument('webots_port', default_value='23000'),
        DeclareLaunchArgument('webots_controller_port', default_value=''),
        DeclareLaunchArgument('world_path', default_value=''),
        DeclareLaunchArgument('webots_mode', default_value='realtime'),
        DeclareLaunchArgument('webots_gui', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_scan_matching', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('explorer_autostart', default_value='false',
                              choices=['true', 'false']),
        stack, explorer,
    ])
