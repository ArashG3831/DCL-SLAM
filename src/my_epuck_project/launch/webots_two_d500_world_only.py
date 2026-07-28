#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('my_epuck_project')

    world = LaunchConfiguration('world')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([
            TextSubstitution(text=package_dir),
            'worlds',
            world,
        ]),
        ros2_supervisor=False,
        port='23000',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='epuck_d500_two_world.wbt',
            description='Two custom D500 e-pucks world file',
        ),
        webots,
    ])
