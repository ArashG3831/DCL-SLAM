#!/usr/bin/env python3
"""Committed cooperative stack plus one strictly passive experiment observer."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    project=get_package_share_directory('my_epuck_project')
    stack=IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(project,'launch','two_robots_cooperative_single_goal_launch.py')))
    arguments=[
        DeclareLaunchArgument('run_id',default_value=''),
        DeclareLaunchArgument('output_root',default_value='/home/arash/webots_ws/results'),
        DeclareLaunchArgument('enable_console_status',default_value='true'),
        DeclareLaunchArgument('enable_coverage_attribution',default_value='true'),
        DeclareLaunchArgument('enable_trajectory_overlap',default_value='true'),
    ]
    observer=Node(package='my_epuck_project',executable='cooperative_experiment_logger',name='cooperative_experiment_logger',namespace='',output='screen',parameters=[{
        'run_id':LaunchConfiguration('run_id'),'output_root':LaunchConfiguration('output_root'),
        'enable_console_status':LaunchConfiguration('enable_console_status'),
        'enable_coverage_attribution':LaunchConfiguration('enable_coverage_attribution'),
        'enable_trajectory_overlap':LaunchConfiguration('enable_trajectory_overlap'),
    }])
    return LaunchDescription([*arguments,stack,observer])
