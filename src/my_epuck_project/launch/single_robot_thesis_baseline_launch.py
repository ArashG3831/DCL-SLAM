#!/usr/bin/env python3
"""True one-robot thesis baseline: local SLAM, Nav2, and WFD explorer."""

import os
import json
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             LogInfo, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from my_epuck_project.thesis_baseline_topology import (
    add_forensic_supervisor, materialize_single_robot_world, sha256_file)


def launch_setup(context):
    package_dir = get_package_share_directory('my_epuck_project')
    source_world = LaunchConfiguration('world_path').perform(context)
    if not source_world:
        source_world = os.path.join(
            package_dir, 'worlds',
            'epuck_d500_two_world_unknown_pose_close_start_dynamic_low_slip_20ms_finite.wbt')
    derived = materialize_single_robot_world(source_world, tempfile.mkdtemp(
        prefix='my_epuck_thesis_A_'))
    forensic_enabled = (
        LaunchConfiguration('enable_forensic_capture').perform(context).lower()
        == 'true')
    contact_enabled = (
        LaunchConfiguration('enable_contact_capture').perform(context).lower()
        == 'true')
    if forensic_enabled or contact_enabled:
        # The external observer is read-only, but Webots requires its
        # Supervisor controller to be represented by a temporary Robot node.
        # This is not a second e-puck and does not change measured geometry.
        add_forensic_supervisor(derived)
    seed_provenance = json.loads(
        LaunchConfiguration('seed_provenance_json').perform(context) or '{}')
    seed_provenance['derived_run_world_path'] = str(derived)
    seed_provenance['derived_run_world_sha256'] = sha256_file(derived)
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_dir, 'launch', 'two_robots_teammate_filtered_stack_launch.py')),
        launch_arguments={
            'world_profile': LaunchConfiguration('world_profile'),
            'world_path': str(derived),
            'webots_port': LaunchConfiguration('webots_port'),
            'webots_mode': LaunchConfiguration('webots_mode'),
            'webots_gui': LaunchConfiguration('webots_gui'),
            'use_sim_time': 'true',
            'unknown_initial_pose': 'false',
            'launch_mapping': 'true',
            'launch_shared_stack': 'false',
            'launch_shared_fusion': 'false',
            'independent_local_maps': 'true',
            'active_robots': 'robot1',
        }.items(),
    )
    explorer = Node(
        package='frontier_exploration_ros2', executable='frontier_explorer',
        namespace='robot1', name='frontier_explorer', output='screen',
        parameters=[{
            'use_sim_time': True, 'map_topic': '/robot1/map',
            'costmap_topic': '/robot1/global_costmap/costmap',
            'local_costmap_topic': '/robot1/local_costmap/costmap',
            'global_frame': 'robot1/map',
            'robot_base_frame': 'robot1/base_footprint',
            'navigate_to_pose_action_name': 'navigate_to_pose',
            'autostart': True, 'frontier_suppression_enabled': False,
            'goal_preemption_enabled': False,
        }],
    )
    observer = Node(
        package='my_epuck_project', executable='cooperative_experiment_logger',
        name='cooperative_experiment_logger', output='screen',
        condition=IfCondition(LaunchConfiguration('enable_observer')),
        parameters=[{
            'run_id': LaunchConfiguration('run_id'),
            'output_root': LaunchConfiguration('output_root'),
            'launch_file': 'single_robot_thesis_baseline_launch.py',
            'experiment_condition': LaunchConfiguration('experiment_condition'),
            'seed_provenance_json': ParameterValue(
                json.dumps(seed_provenance, sort_keys=True, separators=(',', ':')),
                value_type=str),
            'world_profile': LaunchConfiguration('world_profile'),
            'robot_ids': ['robot1'], 'global_frame': 'robot1/map',
            'use_sim_time': True,
            'enable_forensic_capture': LaunchConfiguration('enable_forensic_capture'),
            'enable_contact_capture': LaunchConfiguration('enable_contact_capture'),
            'webots_port': LaunchConfiguration('webots_port'),
        }],
    )
    return [LogInfo(msg=(
        f'THESIS_A_WORLD source={source_world} derived={derived} '
        'robot2_removed=true environment_geometry_preserved=true')),
            stack, explorer, observer]


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
        DeclareLaunchArgument('experiment_condition', default_value='A'),
        DeclareLaunchArgument('seed_provenance_json', default_value='{}'),
        DeclareLaunchArgument('enable_observer', default_value='true'),
        DeclareLaunchArgument('enable_forensic_capture', default_value='false'),
        DeclareLaunchArgument('enable_contact_capture', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
