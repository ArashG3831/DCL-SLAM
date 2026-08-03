#!/usr/bin/env python3
"""Validated two-robot mapping and per-robot Nav2 without an explorer."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Expose the validated mapping/Nav2 stack under a stable launch name."""
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
            'diagnostic_mode': LaunchConfiguration('diagnostic_mode'),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('world_profile', default_value='large',
                              choices=['large', 'small']),
        DeclareLaunchArgument('webots_port', default_value='23000'),
        DeclareLaunchArgument(
            'webots_controller_port', default_value=''),
        DeclareLaunchArgument('world_path', default_value=''),
        DeclareLaunchArgument('webots_mode', default_value='realtime'),
        DeclareLaunchArgument('webots_gui', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('diagnostic_mode', default_value='false'),
        stack,
    ])
