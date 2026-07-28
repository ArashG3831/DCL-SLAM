#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('my_epuck_project')

    known_alignment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_dir,
                'launch',
                'two_robots_known_map_alignment_launch.py',
            )
        ),
        launch_arguments={
            'launch_rviz': 'false',
        }.items(),
    )

    fusion_nodes = [
        Node(
            package='my_epuck_project',
            executable='decentralized_map_fusion',
            name='map_fusion',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'input_topics': ['/robot1/map', '/robot2/map'],
                'output_topic': 'shared_map',
                'metadata_topic': 'shared_map_metadata',
                'output_frame': 'shared_map',
                'resolution': 0.01,
            }],
        )
        for robot_name in ('robot1', 'robot2')
    ]

    return LaunchDescription([
        known_alignment,
        *fusion_nodes,
    ])
