#!/usr/bin/env python3
"""Two independent Slam Toolbox mappers plus source-aware peer map fusion."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from my_epuck_project.cooperative_profiles import profile


def launch_setup(context):
    """Reuse the validated mapping graph without constructing Nav2 processes."""
    package_dir = get_package_share_directory('my_epuck_project')
    world_path = LaunchConfiguration('world_path').perform(context)
    selected = profile(
        LaunchConfiguration('world_profile').perform(context),
        os.path.dirname(world_path) if world_path else os.path.join(package_dir, 'worlds'),
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_dir, 'launch',
            'two_robots_teammate_filtered_dual_slam_launch.py',
        )),
        launch_arguments={
            'world_profile': selected['name'],
            'webots_port': LaunchConfiguration('webots_port'),
            'world_path': world_path,
            'webots_mode': LaunchConfiguration('webots_mode'),
            'webots_gui': LaunchConfiguration('webots_gui'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'sensor_profile': LaunchConfiguration('sensor_profile'),
        }.items(),
    )
    relative = selected['world_metadata']['relative_transform']
    alignment = [
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='shared_to_robot1_map', output='screen',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0', '--yaw', '0.0',
                '--frame-id', 'shared_map', '--child-frame-id', 'robot1/map',
            ],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='shared_to_robot2_map', output='screen',
            arguments=[
                '--x', str(relative[0]), '--y', str(relative[1]),
                '--z', '0.0', '--yaw', str(relative[2]),
                '--frame-id', 'shared_map', '--child-frame-id', 'robot2/map',
            ],
        ),
    ]
    exchange = []
    for robot, peer in (('robot1', 'robot2'), ('robot2', 'robot1')):
        exchange.extend([
            Node(
                package='my_epuck_project', executable='map_exporter',
                name='map_exporter', namespace=robot, output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'source_robot_id': robot,
                    'input_topic': f'/{robot}/map',
                    'output_topic': f'/cslam/{robot}/local_map',
                    'export_rate_hz': 1.0,
                }],
            ),
            Node(
                package='my_epuck_project', executable='source_aware_map_fusion',
                name='map_fusion', namespace=robot, output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'local_map_topic': f'/{robot}/map',
                    'remote_peer_topic': f'/cslam/{peer}/local_map',
                    'expected_remote_source': peer,
                    'output_topic': 'shared_map',
                    'metadata_topic': 'shared_map_metadata',
                    'output_frame': 'shared_map',
                    'resolution': selected['fusion_resolution'],
                }],
            ),
        ])
    return [slam, *alignment, *exchange]


def generate_launch_description():
    """Declare mapping-only world and transport controls."""
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
        OpaqueFunction(function=launch_setup),
    ])
