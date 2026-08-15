#!/usr/bin/env python3
"""Validated mapping/Nav2/frontiers plus two equal replicated assignment peers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def _proposal_adapter(robot):
    return Node(
        package='my_epuck_project',
        executable='frontier_proposal_adapter',
        name='frontier_proposal_adapter',
        namespace=robot,
        output='screen',
        parameters=[{
            'robot_id': robot,
            'maximum_tasks': 5,
            # A snapshot is a bounded proposal lease, not a heartbeat.  Keep
            # it alive long enough for the bounded peer-path bid round.
            'validity_s': 8.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )


def _assignment_peer(robot):
    return Node(
        package='my_epuck_project',
        executable='distributed_frontier_assignment',
        name='distributed_frontier_assignment',
        namespace=robot,
        output='screen',
        remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
        parameters=[{
            'robot_id': robot,
            'robot_base_frame': f'{robot}/base_footprint',
            'global_frame': 'shared_map',
            'dispatch_enabled': LaunchConfiguration('dispatch_enabled'),
            'synthetic_bids': False,
            'maximum_tasks_per_source': 5,
            'maximum_union_tasks': 10,
            'maximum_path_queries': 8,
            'minimum_solo_visible_gain_m': 0.05,
            'minimum_solo_ordering_score': 0.0,
            'maximum_solo_path_m': 18.0,
            'bid_validity_s': 8.0,
            'decision_validity_s': 8.0,
            'peer_timeout_s': 12.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )


def generate_launch_description():
    """Compose the existing frontier stack with two equal assignment peers."""
    package_dir = get_package_share_directory('my_epuck_project')
    frontier_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_dir, 'launch', 'two_robots_frontier_candidates_launch.py',
        )),
        launch_arguments={
            'world_profile': LaunchConfiguration('world_profile'),
            'webots_port': LaunchConfiguration('webots_port'),
            'world_path': LaunchConfiguration('world_path'),
            'webots_mode': LaunchConfiguration('webots_mode'),
            'webots_gui': LaunchConfiguration('webots_gui'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'sensor_profile': LaunchConfiguration('sensor_profile'),
            'diagnostic_mode': LaunchConfiguration('diagnostic_mode'),
            'nav2_autostart': LaunchConfiguration('nav2_autostart'),
            'controller_variant': LaunchConfiguration('controller_variant'),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('world_profile', default_value='large',
                              choices=['large', 'small']),
        DeclareLaunchArgument('webots_port', default_value='23000'),
        DeclareLaunchArgument('world_path', default_value=''),
        DeclareLaunchArgument('webots_mode', default_value='realtime'),
        DeclareLaunchArgument('webots_gui', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('sensor_profile', default_value='full',
                              choices=['full', 'throughput']),
        DeclareLaunchArgument('diagnostic_mode', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('nav2_autostart', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('controller_variant', default_value='rpp',
                              choices=['dwb', 'rotation_shim_dwb', 'rpp']),
        DeclareLaunchArgument('dispatch_enabled', default_value='false',
                              choices=['true', 'false']),
        frontier_stack,
        _proposal_adapter('robot1'),
        _proposal_adapter('robot2'),
        _assignment_peer('robot1'),
        _assignment_peer('robot2'),
    ])
