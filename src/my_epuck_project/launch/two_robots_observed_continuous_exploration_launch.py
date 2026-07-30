#!/usr/bin/env python3
"""Continuous decentralized two-robot exploration with a passive observer."""

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from my_epuck_project.cooperative_profiles import (
    manual_rviz_path,
    profile,
    profile_summary,
)


def runtime_actions(context):
    project = get_package_share_directory('my_epuck_project')
    source_world_path = LaunchConfiguration('source_world_path').perform(context)
    selected = profile(
        LaunchConfiguration('world_profile').perform(context),
        os.path.dirname(source_world_path) if source_world_path else os.path.join(project, 'worlds'),
    )
    summary = profile_summary(selected)
    source_world_path = source_world_path or selected['world_path']
    continuous_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            project, 'launch',
            'two_robots_cooperative_single_goal_launch.py',
        )),
        launch_arguments={
            'world_profile': selected['name'],
            'coordinator_mode': 'continuous',
            'one_goal_only': 'false',
            'coordinator_autostart':
                LaunchConfiguration('coordinator_autostart'),
            'cycle_cooldown_s': LaunchConfiguration('cycle_cooldown_s'),
            'success_region_cooldown_s':
                LaunchConfiguration('success_region_cooldown_s'),
            'first_failure_suppression_s':
                LaunchConfiguration('first_failure_suppression_s'),
            'second_failure_suppression_s':
                LaunchConfiguration('second_failure_suppression_s'),
            'maximum_failure_suppression_s':
                LaunchConfiguration('maximum_failure_suppression_s'),
            'no_candidate_grace_s':
                LaunchConfiguration('no_candidate_grace_s'),
            'map_stability_window_s':
                LaunchConfiguration('map_stability_window_s'),
            'completion_consensus_grace_s':
                LaunchConfiguration('completion_consensus_grace_s'),
            'webots_port': LaunchConfiguration('webots_port'),
            'webots_mode': LaunchConfiguration('webots_mode'),
            'webots_gui': LaunchConfiguration('webots_gui'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'sensor_profile': LaunchConfiguration('sensor_profile'),
            'world_path': source_world_path,
        }.items(),
    )
    observer = Node(
        package='my_epuck_project',
        executable='cooperative_experiment_logger',
        name='cooperative_experiment_logger',
        output='screen',
        parameters=[{
            'run_id': LaunchConfiguration('run_id'),
            'output_root': LaunchConfiguration('output_root'),
            'launch_file':
                'two_robots_observed_continuous_exploration_launch.py',
            'enable_console_status': LaunchConfiguration('logger_console_status'),
            'world_profile': selected['name'],
            'source_world_path': source_world_path,
            'installed_world_path': selected['world_path'],
            'world_dimensions': list(
                selected['world_metadata']['dimensions']),
            'robot_start_poses_json': json.dumps(
                summary['robot_start_poses'], sort_keys=True),
            'known_relative_transform':
                list(selected['world_metadata']['relative_transform']),
            'transform_source': 'WORLD_DERIVED',
            'slam_resolution': selected['slam_resolution'],
            'fusion_resolution': selected['fusion_resolution'],
            'global_costmap_resolution':
                selected['global_costmap_resolution'],
            'local_costmap_resolution':
                selected['local_costmap_resolution'],
            'lidar_maximum_range': summary['lidar_maximum_range'],
            'initial_configuration_json': json.dumps({
                'minimum_frontier_cells':
                    selected['minimum_frontier_cells'],
                'minimum_known_cell_gain_for_activity':
                    selected['minimum_known_cell_gain_for_activity'],
                'coverage_attribution_resolution':
                    selected['coverage_attribution_resolution'],
            }, sort_keys=True),
            'world_sha256': selected['world_metadata']['sha256'],
            'coverage_attribution_resolution':
                selected['coverage_attribution_resolution'],
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    diagnostic = Node(
        package='my_epuck_project',
        executable='controller_pipeline_diagnostics',
        name='controller_pipeline_diagnostics',
        output='screen',
        condition=IfCondition(LaunchConfiguration('diagnostic_mode')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'output_root': os.path.join(
                LaunchConfiguration('output_root').perform(context),
                'controller_diagnostics'),
        }],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='cooperative_manual_rviz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        arguments=[
            '-d',
            manual_rviz_path(selected, os.path.join(project, 'resource')),
        ],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': 'true'},
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    metadata = selected['world_metadata']
    return [
        LogInfo(msg=(
            f'WORLD_DERIVED selected_world={source_world_path} '
            f'robot1={metadata["robots"]["robot1"].translation},'
            f'yaw={metadata["robots"]["robot1"].planar_yaw:.9f} '
            f'robot2={metadata["robots"]["robot2"].translation},'
            f'yaw={metadata["robots"]["robot2"].planar_yaw:.9f} '
            f'separation_m={metadata["initial_separation_m"]:.6f} '
            f'robot2_in_robot1={metadata["relative_transform"]} '
            f'validation={metadata["validation"]}')),
        continuous_stack, observer, diagnostic, rviz,
    ]


def generate_launch_description():
    timeout = TimerAction(
        period=LaunchConfiguration('mission_timeout_s'),
        condition=IfCondition(LaunchConfiguration('enable_mission_timeout')),
        actions=[EmitEvent(event=Shutdown(reason='bounded mission timeout'))],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_profile',
            default_value='large',
            choices=['large', 'small'],
            description='Reusable continuous-exploration world profile.',
        ),
        DeclareLaunchArgument(
            'source_world_path',
            default_value='',
            description='Optional source path recorded as experiment metadata.',
        ),
        DeclareLaunchArgument('run_id', default_value=''),
        DeclareLaunchArgument('webots_port', default_value='23000'),
        DeclareLaunchArgument('webots_mode', default_value='realtime'),
        DeclareLaunchArgument('webots_gui', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('sensor_profile', default_value='full',
                              choices=['full', 'throughput']),
        DeclareLaunchArgument('diagnostic_mode', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Open the passive cooperative-exploration RViz view.',
        ),
        DeclareLaunchArgument(
            'output_root', default_value='/home/arash/webots_ws/results'),
        DeclareLaunchArgument('mission_timeout_s', default_value='600.0'),
        DeclareLaunchArgument(
            'enable_mission_timeout',
            default_value='true',
            description='Enable the bounded launch shutdown timer.'),
        DeclareLaunchArgument('coordinator_autostart', default_value='true'),
        DeclareLaunchArgument('cycle_cooldown_s', default_value='2.5'),
        DeclareLaunchArgument('success_region_cooldown_s', default_value='25.0'),
        DeclareLaunchArgument('first_failure_suppression_s', default_value='20.0'),
        DeclareLaunchArgument('second_failure_suppression_s', default_value='60.0'),
        DeclareLaunchArgument('maximum_failure_suppression_s', default_value='180.0'),
        DeclareLaunchArgument('no_candidate_grace_s', default_value='18.0'),
        DeclareLaunchArgument('map_stability_window_s', default_value='15.0'),
        DeclareLaunchArgument('completion_consensus_grace_s', default_value='8.0'),
        DeclareLaunchArgument('logger_console_status', default_value='true'),
        OpaqueFunction(function=runtime_actions),
        timeout,
    ])
