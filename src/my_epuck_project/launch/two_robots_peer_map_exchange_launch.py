#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('my_epuck_project')
    alignment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_dir, 'launch', 'two_robots_known_map_alignment_launch.py'
        )),
        launch_arguments={'launch_rviz': 'false'}.items(),
    )

    nodes = []
    for robot, peer in (('robot1', 'robot2'), ('robot2', 'robot1')):
        nodes.extend([
            Node(
                package='my_epuck_project',
                executable='map_exporter',
                name='map_exporter',
                namespace=robot,
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'source_robot_id': robot,
                    'input_topic': f'/{robot}/map',
                    'output_topic': f'/cslam/{robot}/local_map',
                    'export_rate_hz': 1.0,
                }],
            ),
            Node(
                package='my_epuck_project',
                executable='source_aware_map_fusion',
                name='map_fusion',
                namespace=robot,
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'local_map_topic': f'/{robot}/map',
                    'remote_peer_topic': f'/cslam/{peer}/local_map',
                    'expected_remote_source': peer,
                    'output_topic': 'shared_map',
                    'metadata_topic': 'shared_map_metadata',
                    'output_frame': 'shared_map',
                    'resolution': 0.01,
                }],
            ),
        ])

    return LaunchDescription([alignment, *nodes])
