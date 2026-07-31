#!/usr/bin/env python3
"""Retained legacy claim-only decentralized exploration baseline."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Expose the retained claim-only baseline under an explicit name."""
    package_dir = get_package_share_directory('my_epuck_project')
    baseline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_dir, 'launch',
            'two_robots_observed_continuous_exploration_launch.py',
        )),
        launch_arguments={
            'world_profile': LaunchConfiguration('world_profile'),
            'source_world_path': LaunchConfiguration('source_world_path'),
            'webots_port': LaunchConfiguration('webots_port'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'launch_rviz': 'false',
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('world_profile', default_value='large',
                              choices=['large', 'small']),
        DeclareLaunchArgument('source_world_path', default_value=''),
        DeclareLaunchArgument('webots_port', default_value='23000'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        baseline,
    ])
