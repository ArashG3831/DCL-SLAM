#!/usr/bin/env python3
"""Upstream single autonomous explorer on Robot 1; Robot 2 remains passive."""

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
            'world_path': LaunchConfiguration('world_path'),
            'webots_mode': LaunchConfiguration('webots_mode'),
            'webots_gui': LaunchConfiguration('webots_gui'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )
    explorer = Node(
        package='frontier_exploration_ros2', executable='frontier_explorer',
        name='frontier_explorer', namespace='robot1', output='screen',
        remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
        parameters=[{
            'map_topic': '/robot1/shared_map',
            'costmap_topic': '/robot1/global_costmap/costmap',
            'local_costmap_topic': '/robot1/local_costmap/costmap',
            'navigate_to_pose_action_name': 'navigate_to_pose',
            'global_frame': 'shared_map',
            'robot_base_frame': 'robot1/base_footprint',
            'goal_preemption_lidar_range_m': 11.98,
            'autostart': True,
            'return_to_start_on_complete': False,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('world_profile', default_value='large',
                              choices=['large', 'small']),
        DeclareLaunchArgument('webots_port', default_value='23000'),
        DeclareLaunchArgument('world_path', default_value=''),
        DeclareLaunchArgument('webots_mode', default_value='realtime'),
        DeclareLaunchArgument('webots_gui', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        stack, explorer,
    ])
