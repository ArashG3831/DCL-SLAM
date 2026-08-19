#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from my_epuck_project.cooperative_profiles import profile

def generator(robot, minimum_frontier_cells, approach_clearance,
              forensic_clearance_cells, log_level):
    return Node(
        package='my_epuck_frontier_candidates', executable='frontier_candidate_generator',
        name='frontier_candidate_generator', namespace=robot, output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
        parameters=[{
            'robot_id': robot,
            'map_topic': f'/{robot}/shared_map',
            'global_costmap_topic': f'/{robot}/global_costmap/costmap',
            'global_frame': 'shared_map',
            'robot_base_frame': f'{robot}/base_footprint',
            'compute_path_action': f'/{robot}/compute_path_to_pose',
            'candidate_topic': f'/{robot}/frontier_candidates',
            'marker_topic': f'/{robot}/frontier_candidate_markers',
            'processing_rate_hz': 0.5,
            'minimum_frontier_cells': minimum_frontier_cells,
            'minimum_frontier_length_m': 0.05,
            'stable_id_quantization_m': 0.05,
            'approach_clearance_m': approach_clearance,
            'minimum_robot_distance_m': 0.08,
            'maximum_candidates_before_path_check': 8,
            'maximum_path_queries_per_cycle': 5,
            'path_query_timeout_s': 1.0,
            'planner_id': 'GridBased',
            'occupied_threshold': 50,
            'visible_gain_range_m': 11.98,
            'forensic_clearance_cells': forensic_clearance_cells,
            'diagnostic_frontier_capture': LaunchConfiguration(
                'diagnostic_frontier_capture'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

def launch_setup(context):
    project = get_package_share_directory('my_epuck_project')
    world_path = LaunchConfiguration('world_path').perform(context)
    selected = profile(
        LaunchConfiguration('world_profile').perform(context),
        os.path.dirname(world_path) if world_path else os.path.join(project, 'worlds'),
    )
    diagnostic_capture = (
        LaunchConfiguration('diagnostic_frontier_capture').perform(context)
        .lower() == 'true'
    )
    frontier_log_level = 'INFO' if diagnostic_capture else 'WARN'
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            project, 'launch', 'two_robots_teammate_filtered_stack_launch.py')),
        launch_arguments={
            'world_profile': selected['name'],
            'webots_port': LaunchConfiguration('webots_port'),
            'webots_controller_port': LaunchConfiguration(
                'webots_controller_port'),
            'webots_mode': LaunchConfiguration('webots_mode'),
            'webots_gui': LaunchConfiguration('webots_gui'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_scan_matching': LaunchConfiguration('use_scan_matching'),
            'do_loop_closing': LaunchConfiguration('do_loop_closing'),
            'slam_tf_publish_probe_library': LaunchConfiguration(
                'slam_tf_publish_probe_library'),
            'slam_tf_publish_probe_log': LaunchConfiguration(
                'slam_tf_publish_probe_log'),
            'slam_tf_publication_mode': LaunchConfiguration(
                'slam_tf_publication_mode'),
            'sensor_profile': LaunchConfiguration('sensor_profile'),
            'world_path': world_path,
            'diagnostic_mode': LaunchConfiguration('diagnostic_mode'),
            'fusion_cpu_quota_percent': LaunchConfiguration(
                'fusion_cpu_quota_percent'),
            'fusion_process_nice': LaunchConfiguration(
                'fusion_process_nice'),
            'fusion_rebuild_period_s': LaunchConfiguration(
                'fusion_rebuild_period_s'),
            'nav2_autostart': LaunchConfiguration('nav2_autostart'),
            'controller_variant': LaunchConfiguration('controller_variant'),
            'unknown_initial_pose': LaunchConfiguration(
                'unknown_initial_pose'),
        }.items(),
    )
    return [
        stack,
        generator('robot1', selected['minimum_frontier_cells'],
                  0.15 if selected['name'] == 'large' else 0.06,
                  LaunchConfiguration('forensic_clearance_cells'),
                  frontier_log_level),
        generator('robot2', selected['minimum_frontier_cells'],
                  0.15 if selected['name'] == 'large' else 0.06,
                  LaunchConfiguration('forensic_clearance_cells'),
                  frontier_log_level),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_profile',
            default_value='small',
            choices=['large', 'small', 'large_unknown_pose'],
        ),
        DeclareLaunchArgument('webots_port', default_value='23000'),
        DeclareLaunchArgument(
            'webots_controller_port', default_value=''),
        DeclareLaunchArgument('world_path', default_value=''),
        DeclareLaunchArgument('webots_mode', default_value='realtime'),
        DeclareLaunchArgument('webots_gui', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_scan_matching', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('do_loop_closing', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('slam_tf_publish_probe_library', default_value=''),
        DeclareLaunchArgument('slam_tf_publish_probe_log', default_value=''),
        DeclareLaunchArgument('slam_tf_publication_mode', default_value='',
                              choices=['', 'SYNCHRONOUS', 'ASYNCHRONOUS']),
        DeclareLaunchArgument('sensor_profile', default_value='full',
                              choices=['full', 'throughput']),
        DeclareLaunchArgument('diagnostic_mode', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('diagnostic_frontier_capture', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('fusion_cpu_quota_percent', default_value='30'),
        DeclareLaunchArgument('fusion_process_nice', default_value='0'),
        DeclareLaunchArgument('fusion_rebuild_period_s', default_value='0.0'),
        DeclareLaunchArgument('forensic_clearance_cells', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('nav2_autostart', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('controller_variant', default_value='rpp',
                              choices=['dwb', 'rotation_shim_dwb', 'rpp']),
        DeclareLaunchArgument('unknown_initial_pose', default_value='false',
                              choices=['true', 'false']),
        OpaqueFunction(function=launch_setup),
    ])
