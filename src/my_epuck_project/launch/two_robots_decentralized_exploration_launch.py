#!/usr/bin/env python3
"""Final two-robot decentralized mapping, pair assignment, and local Nav2 launch."""

import os
import json

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from my_epuck_project.cooperative_profiles import profile, profile_summary


def launch_setup(context):
    """Resolve the exact world once for both control and passive metadata."""
    package_dir = get_package_share_directory('my_epuck_project')
    world_path = LaunchConfiguration('world_path').perform(context)
    selected = profile(
        LaunchConfiguration('world_profile').perform(context),
        os.path.dirname(world_path) if world_path else os.path.join(package_dir, 'worlds'),
    )
    summary = profile_summary(selected)
    assignment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_dir, 'launch', 'two_robots_distributed_assignment_launch.py',
        )),
        launch_arguments={
            'world_profile': selected['name'],
            'webots_port': LaunchConfiguration('webots_port'),
            'world_path': world_path,
            'webots_mode': LaunchConfiguration('webots_mode'),
            'webots_gui': LaunchConfiguration('webots_gui'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'diagnostic_mode': LaunchConfiguration('diagnostic_mode'),
            'dispatch_enabled': 'true',
        }.items(),
    )
    observer = Node(
        package='my_epuck_project', executable='cooperative_experiment_logger',
        name='cooperative_experiment_logger', output='screen',
        condition=IfCondition(LaunchConfiguration('enable_observer')),
        parameters=[{
            'run_id': LaunchConfiguration('run_id'),
            'output_root': LaunchConfiguration('output_root'),
            'launch_file': 'two_robots_decentralized_exploration_launch.py',
            'world_profile': selected['name'],
            'source_world_path': selected['world_path'],
            'installed_world_path': selected['world_path'],
            'world_dimensions': list(selected['world_metadata']['dimensions']),
            'robot_start_poses_json': json.dumps(
                summary['robot_start_poses'], sort_keys=True,
            ),
            'known_relative_transform': list(
                selected['world_metadata']['relative_transform'],
            ),
            'transform_source': 'WORLD_DERIVED',
            'slam_resolution': selected['slam_resolution'],
            'fusion_resolution': selected['fusion_resolution'],
            'global_costmap_resolution': selected['global_costmap_resolution'],
            'local_costmap_resolution': selected['local_costmap_resolution'],
            'lidar_maximum_range': summary['lidar_maximum_range'],
            'world_sha256': selected['world_metadata']['sha256'],
            'coverage_attribution_resolution':
                selected['coverage_attribution_resolution'],
            'initial_configuration_json': json.dumps({
                'frontier_engine': 'frontier_exploration_ros2 public core',
                'maximum_tasks_per_source': 5,
                'maximum_union_tasks': 8,
                'maximum_path_queries': 8,
                'dispatch_enabled': True,
                'map_fusion_resolution': selected['fusion_resolution'],
            }, sort_keys=True),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    return [assignment, observer]


def generate_launch_description():
    """Launch final control graph and its removable passive evaluator."""
    return LaunchDescription([
        DeclareLaunchArgument('world_profile', default_value='large',
                              choices=['large', 'small']),
        DeclareLaunchArgument('webots_port', default_value='23000'),
        DeclareLaunchArgument('world_path', default_value=''),
        DeclareLaunchArgument('webots_mode', default_value='realtime'),
        DeclareLaunchArgument('webots_gui', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('diagnostic_mode', default_value='false'),
        DeclareLaunchArgument('enable_observer', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('run_id', default_value=''),
        DeclareLaunchArgument('output_root', default_value='/home/arash/webots_ws/results'),
        OpaqueFunction(function=launch_setup),
    ])
