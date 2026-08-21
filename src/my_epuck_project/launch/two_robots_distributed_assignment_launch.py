#!/usr/bin/env python3
"""Validated mapping/Nav2/frontiers plus two equal replicated assignment peers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def _proposal_adapter(robot):
    return Node(
        package='my_epuck_project',
        executable='frontier_proposal_adapter',
        name='frontier_proposal_adapter',
        namespace=robot,
        output='screen',
        parameters=[{
            'robot_id': robot,
            'maximum_tasks': 5,
            # A snapshot is a bounded proposal lease, not a heartbeat.  Keep
            # it alive long enough for the bounded peer-path bid round.
            'validity_s': 8.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )


def _assignment_peer(robot):
    return Node(
        package='my_epuck_project',
        executable='distributed_frontier_assignment',
        name='distributed_frontier_assignment',
        namespace=robot,
        output='screen',
        remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
        parameters=[{
            'robot_id': robot,
            'robot_base_frame': f'{robot}/base_footprint',
            'global_frame': 'shared_map',
            'dispatch_enabled': LaunchConfiguration('dispatch_enabled'),
            'handoff_gated': LaunchConfiguration('handoff_gated'),
            'phase_gated': LaunchConfiguration('phase_gated'),
            'synthetic_bids': False,
            'maximum_tasks_per_source': 5,
            'maximum_union_tasks': 10,
            'maximum_path_queries': 8,
            'minimum_solo_visible_gain_m': 0.05,
            'minimum_solo_ordering_score': 0.0,
            'maximum_solo_path_m': 18.0,
            # Production default: task-level adaptation of Burgard et al.
            # Algorithm 1.  The 18 m denominator is the pre-existing path
            # feasibility ceiling, not a second tuned score scale.
            'assignment_strategy': LaunchConfiguration('assignment_strategy'),
            'burgard_beta': LaunchConfiguration('burgard_beta'),
            'burgard_sensor_max_range_m': 11.98,
            'burgard_occupied_threshold': 50,
            # Derived from the frozen production Nav2 profiles: stop circle
            # 0.08 m dominates robot_radius 0.055 m; RPP desired speed 0.13.
            'traffic_scheduler_enabled': LaunchConfiguration(
                'traffic_scheduler_enabled'),
            'traffic_robot1_safe_radius_m': 0.08,
            'traffic_robot2_safe_radius_m': 0.08,
            'traffic_reference_speed_mps': 0.13,
            'traffic_eta_tie_s': 0.05,
            'bid_validity_s': 8.0,
            'decision_validity_s': 8.0,
            'peer_timeout_s': 12.0,
            'enable_mission_timeout': LaunchConfiguration(
                'enable_mission_timeout'),
            'mission_timeout_s': LaunchConfiguration('mission_timeout_s'),
            # Termination-only significance threshold.  The frontier
            # detector's 0.05 m minimum remains unchanged.
            'terminal_small_frontier_length_m': LaunchConfiguration(
                'terminal_small_frontier_length_m'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )


def generate_launch_description():
    """Compose the existing frontier stack with two equal assignment peers."""
    package_dir = get_package_share_directory('my_epuck_project')
    frontier_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_dir, 'launch', 'two_robots_frontier_candidates_launch.py',
        )),
        launch_arguments={
            'world_profile': LaunchConfiguration('world_profile'),
            'webots_port': LaunchConfiguration('webots_port'),
            'world_path': LaunchConfiguration('world_path'),
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
            'diagnostic_mode': LaunchConfiguration('diagnostic_mode'),
            'diagnostic_frontier_capture': LaunchConfiguration(
                'diagnostic_frontier_capture'),
            'fusion_process_nice': LaunchConfiguration(
                'fusion_process_nice'),
            'fusion_cpu_quota_percent': LaunchConfiguration(
                'fusion_cpu_quota_percent'),
            'fusion_rebuild_period_s': LaunchConfiguration(
                'fusion_rebuild_period_s'),
            'nav2_autostart': LaunchConfiguration('nav2_autostart'),
            'controller_variant': LaunchConfiguration('controller_variant'),
            'unknown_initial_pose': LaunchConfiguration(
                'unknown_initial_pose'),
            'launch_mapping': LaunchConfiguration('launch_mapping'),
            'launch_shared_stack': LaunchConfiguration('launch_shared_stack'),
            'phase_already_aligned': LaunchConfiguration(
                'phase_already_aligned'),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('world_profile', default_value='large',
                              choices=['large', 'small', 'large_unknown_pose',
                                       'large_unknown_pose_16m',
                                       'large_unknown_pose_close_start',
                                       'large_unknown_pose_close_start_20ms',
                                       'large_unknown_pose_close_start_20ms_scan_matching']),
        DeclareLaunchArgument('webots_port', default_value='23000'),
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
        DeclareLaunchArgument('fusion_process_nice', default_value='0'),
        DeclareLaunchArgument('fusion_cpu_quota_percent', default_value='30'),
        DeclareLaunchArgument('fusion_rebuild_period_s', default_value='1.0'),
        DeclareLaunchArgument('nav2_autostart', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('controller_variant', default_value='rpp',
                              choices=['dwb', 'rotation_shim_dwb', 'rpp']),
        DeclareLaunchArgument('unknown_initial_pose', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('launch_mapping', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('launch_shared_stack', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('phase_already_aligned', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('handoff_gated', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('phase_gated', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('handoff_transform_x', default_value='0.0'),
        DeclareLaunchArgument('handoff_transform_y', default_value='0.0'),
        DeclareLaunchArgument('handoff_transform_yaw', default_value='0.0'),
        DeclareLaunchArgument('handoff_evidence_set_hash', default_value=''),
        DeclareLaunchArgument('dispatch_enabled', default_value='false',
                              choices=['true', 'false']),
        # Defaults are the production literature-backed allocator.  These
        # switches are deliberately explicit so historical weighted rounds
        # remain reproducible without changing normal launches.
        DeclareLaunchArgument('assignment_strategy', default_value='burgard',
                              choices=['burgard', 'legacy_weighted']),
        DeclareLaunchArgument('burgard_beta', default_value='1.0'),
        DeclareLaunchArgument('traffic_scheduler_enabled', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('enable_mission_timeout', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('mission_timeout_s', default_value='1800.0'),
        DeclareLaunchArgument('terminal_small_frontier_length_m',
                              default_value='0.20'),
        frontier_stack,
        _proposal_adapter('robot1'),
        _proposal_adapter('robot2'),
        _assignment_peer('robot1'),
        _assignment_peer('robot2'),
    ])
