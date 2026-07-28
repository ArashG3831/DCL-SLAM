#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('my_epuck_project')
    launch_rviz = LaunchConfiguration('launch_rviz')

    dual_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'two_robots_dual_slam_launch.py')
        )
    )

    # shared_map coincides with robot1's initial map and odometry origin.
    shared_to_robot1_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='shared_to_robot1_map',
        output='screen',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'shared_map',
            '--child-frame-id', 'robot1/map',
        ],
    )

    # T_shared_robot2 = inverse(T_world_robot1) * T_world_robot2.
    shared_to_robot2_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='shared_to_robot2_map',
        output='screen',
        arguments=[
            '--x', '-0.299999998712',
            '--y', '-0.000027796077',
            '--z', '0.0',
            '--yaw', '-3.1415',
            '--frame-id', 'shared_map',
            '--child-frame-id', 'robot2/map',
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='known_map_alignment_rviz',
        output='screen',
        condition=IfCondition(launch_rviz),
        arguments=[
            '-d',
            os.path.join(
                package_dir,
                'resource',
                'two_robots_known_map_alignment.rviz',
            ),
        ],
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Display both independent maps in the shared_map frame.',
        ),
        dual_slam,
        shared_to_robot1_map,
        shared_to_robot2_map,
        rviz,
    ])
