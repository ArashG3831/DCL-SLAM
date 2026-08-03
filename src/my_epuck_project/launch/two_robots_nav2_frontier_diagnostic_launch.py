#!/usr/bin/env python3
"""Allocator-free two-robot Nav2/frontier diagnostic launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start the candidate stack and one diagnostic-only goal dispatcher."""
    project = get_package_share_directory('my_epuck_project')
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            project, 'launch', 'two_robots_frontier_candidates_launch.py')),
        launch_arguments={
            'world_profile': LaunchConfiguration('world_profile'),
            'world_path': LaunchConfiguration('world_path'),
            'webots_port': LaunchConfiguration('webots_port'),
            'webots_controller_port': LaunchConfiguration(
                'webots_controller_port'),
            'webots_mode': LaunchConfiguration('webots_mode'),
            'webots_gui': LaunchConfiguration('webots_gui'),
            'sensor_profile': LaunchConfiguration('sensor_profile'),
            'use_sim_time': 'true',
            'diagnostic_mode': 'true',
            'fusion_cpu_quota_percent': LaunchConfiguration(
                'fusion_cpu_quota_percent'),
            'fusion_rebuild_period_s': LaunchConfiguration(
                'fusion_rebuild_period_s'),
            'nav2_autostart': 'true',
        }.items(),
    )
    diagnostic = Node(
        package='my_epuck_project',
        executable='nav2_frontier_diagnostic',
        name='nav2_frontier_diagnostic',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'output_directory': LaunchConfiguration('output_directory'),
            'mission_duration_s': LaunchConfiguration('mission_duration_s'),
            'startup_timeout_s': LaunchConfiguration('startup_timeout_s'),
            'phase_profile': LaunchConfiguration('phase_profile'),
            'world_profile': LaunchConfiguration('world_profile'),
        }],
    )
    stop_when_finished = RegisterEventHandler(OnProcessExit(
        target_action=diagnostic,
        on_exit=[EmitEvent(event=Shutdown(
            reason='Nav2/frontier diagnostic completed'))],
    ))
    return LaunchDescription([
        DeclareLaunchArgument('world_profile', default_value='large',
                              choices=['large', 'small']),
        DeclareLaunchArgument('world_path', default_value=''),
        DeclareLaunchArgument('webots_port', default_value='23000'),
        DeclareLaunchArgument(
            'webots_controller_port', default_value=''),
        DeclareLaunchArgument('webots_mode', default_value='fast',
                              choices=['pause', 'realtime', 'fast']),
        DeclareLaunchArgument('webots_gui', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('sensor_profile', default_value='full',
                              choices=['full', 'throughput']),
        DeclareLaunchArgument('output_directory'),
        DeclareLaunchArgument('mission_duration_s', default_value='600.0'),
        DeclareLaunchArgument('startup_timeout_s', default_value='300.0'),
        DeclareLaunchArgument('phase_profile', default_value='full',
                              choices=['full', 'short']),
        # Protective default for interactive runs. Pass 0 explicitly for the
        # unthrottled comparison required by the diagnostic protocol.
        DeclareLaunchArgument('fusion_cpu_quota_percent', default_value='30'),
        DeclareLaunchArgument('fusion_rebuild_period_s', default_value='1.0'),
        stack,
        diagnostic,
        stop_when_finished,
    ])
