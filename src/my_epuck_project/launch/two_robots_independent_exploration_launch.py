#!/usr/bin/env python3
"""Two independent local WFD explorers; no cooperative target-selection path."""

import json
import os
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from my_epuck_project.cooperative_profiles import profile_for_world, profile_summary


def launch_setup(context):
    package_dir = get_package_share_directory('my_epuck_project')
    source_world_path = LaunchConfiguration('world_path').perform(context)
    source_selected = profile_for_world(
        LaunchConfiguration('world_profile').perform(context),
        os.path.dirname(source_world_path) if source_world_path else os.path.join(
            package_dir, 'worlds'),
        explicit_world_path=source_world_path,
    )
    launch_world_path = source_world_path or source_selected['world_path']
    forensic_enabled = LaunchConfiguration(
        'enable_forensic_capture').perform(context).lower() == 'true'
    contact_enabled = LaunchConfiguration(
        'enable_contact_capture').perform(context).lower() == 'true'
    if forensic_enabled or contact_enabled:
        # Webots requires an external Supervisor Robot for the passive ground
        # truth recorder.  Stage only that diagnostic node in a normal project
        # layout; the independent explorers still consume only local maps.
        forensic_root = tempfile.mkdtemp(prefix='my_epuck_b_forensic_')
        forensic_worlds = os.path.join(forensic_root, 'worlds')
        os.makedirs(forensic_worlds, exist_ok=True)
        source_project = os.path.dirname(
            os.path.dirname(os.path.abspath(launch_world_path)))
        source_protos = os.path.join(source_project, 'protos')
        if os.path.isdir(source_protos):
            shutil.copytree(source_protos, os.path.join(forensic_root, 'protos'))
        staged_world = os.path.join(
            forensic_worlds, os.path.basename(launch_world_path))
        shutil.copyfile(launch_world_path, staged_world)
        with open(staged_world, 'a', encoding='utf-8') as stream:
            stream.write(
                '\nRobot {\n'
                '  name "ForensicGroundTruthSupervisor"\n'
                '  controller "<extern>"\n'
                '  supervisor TRUE\n'
                '}\n')
        launch_world_path = staged_world
    _staged_selected = profile_for_world(
        LaunchConfiguration('world_profile').perform(context),
        os.path.dirname(launch_world_path),
        explicit_world_path=launch_world_path,
    )
    summary = profile_summary(source_selected)
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_dir, 'launch', 'two_robots_teammate_filtered_stack_launch.py')),
        launch_arguments={
            'world_profile': source_selected['name'],
            'world_path': launch_world_path,
            'webots_port': LaunchConfiguration('webots_port'),
            'webots_mode': LaunchConfiguration('webots_mode'),
            'webots_gui': LaunchConfiguration('webots_gui'),
            'use_sim_time': 'true',
            'unknown_initial_pose': 'false',
            'launch_mapping': 'true',
            'launch_shared_stack': 'false',
            'launch_shared_fusion': 'false',
            'independent_local_maps': 'true',
            'active_robots': 'robot1,robot2',
        }.items(),
    )
    explorers = []
    for robot in ('robot1', 'robot2'):
        explorers.append(Node(
            package='frontier_exploration_ros2', executable='frontier_explorer',
            namespace=robot, name='frontier_explorer', output='screen',
            parameters=[{
                'use_sim_time': True,
                'map_topic': f'/{robot}/map',
                'costmap_topic': f'/{robot}/global_costmap/costmap',
                'local_costmap_topic': f'/{robot}/local_costmap/costmap',
                'global_frame': f'{robot}/map',
                'robot_base_frame': f'{robot}/base_footprint',
                'navigate_to_pose_action_name': 'navigate_to_pose',
                'autostart': True, 'frontier_suppression_enabled': False,
                'goal_preemption_enabled': False,
            }],
        ))
    observer = Node(
        package='my_epuck_project', executable='cooperative_experiment_logger',
        name='cooperative_experiment_logger', output='screen',
        condition=IfCondition(LaunchConfiguration('enable_observer')),
        parameters=[{
            'run_id': LaunchConfiguration('run_id'),
            'output_root': LaunchConfiguration('output_root'),
            'launch_file': 'two_robots_independent_exploration_launch.py',
            'experiment_condition': LaunchConfiguration('experiment_condition'),
            'seed_provenance_json': ParameterValue(
                LaunchConfiguration('seed_provenance_json'), value_type=str),
            'world_profile': source_selected['name'],
            'source_world_path': source_selected['world_path'],
            'installed_world_path': launch_world_path,
            'world_dimensions': list(source_selected['world_metadata']['dimensions']),
            'robot_start_poses_json': json.dumps(
                summary['robot_start_poses'], sort_keys=True),
            'known_relative_transform': list(
                source_selected['world_metadata']['relative_transform']),
            'transform_source': 'WORLD_DERIVED',
            'slam_resolution': source_selected['slam_resolution'],
            'fusion_resolution': source_selected['fusion_resolution'],
            'global_costmap_resolution': source_selected['global_costmap_resolution'],
            'local_costmap_resolution': source_selected['local_costmap_resolution'],
            'lidar_maximum_range': summary['lidar_maximum_range'],
            'world_sha256': source_selected['world_metadata']['sha256'],
            'coverage_attribution_resolution':
                source_selected['coverage_attribution_resolution'],
            'coverage_source': 'local_map_union',
            'webots_port': LaunchConfiguration('webots_port'),
            'robot_ids': ['robot1', 'robot2'], 'global_frame': 'robot1/map',
            'use_sim_time': True,
            'enable_forensic_capture': LaunchConfiguration('enable_forensic_capture'),
            'enable_contact_capture': LaunchConfiguration('enable_contact_capture'),
        }],
    )
    return [stack, *explorers, observer]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world_profile', default_value=
                              'large_unknown_pose_close_start_20ms_scan_matching'),
        DeclareLaunchArgument('world_path', default_value=''),
        DeclareLaunchArgument('webots_port', default_value='23000'),
        DeclareLaunchArgument('webots_mode', default_value='fast'),
        DeclareLaunchArgument('webots_gui', default_value='false'),
        DeclareLaunchArgument('output_root', default_value=''),
        DeclareLaunchArgument('run_id', default_value=''),
        DeclareLaunchArgument('experiment_condition', default_value='B'),
        DeclareLaunchArgument('seed_provenance_json', default_value='{}'),
        DeclareLaunchArgument('enable_observer', default_value='true'),
        DeclareLaunchArgument('enable_forensic_capture', default_value='false'),
        DeclareLaunchArgument('enable_contact_capture', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
