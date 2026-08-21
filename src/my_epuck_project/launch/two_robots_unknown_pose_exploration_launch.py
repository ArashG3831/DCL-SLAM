#!/usr/bin/env python3
"""Authoritative full exploration entry point for unknown initial pose.

This is an explicit wrapper around the validated decentralized exploration
stack.  It selects the reviewed far-start world and enables the existing
unknown-pose frontend; the underlying stack remains responsible for SLAM,
fusion, frontiers, allocation, and per-robot Nav2 ownership.

Before handoff, the frontend withholds the peer ``PeerMap`` evidence and the
stack withholds cross-map alignment.  After mutual multi-keyframe acceptance,
the existing frontend publishes the gated evidence and canonical TF handoff,
allowing the unchanged fusion/frontier/allocation path to proceed.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _arg(name, default, choices=None):
    kwargs = {'default_value': default}
    if choices is not None:
        kwargs['choices'] = choices
    return DeclareLaunchArgument(name, **kwargs)


def generate_launch_description():
    """Compose the complete stack with the unknown-pose contract enabled."""
    package_dir = get_package_share_directory('my_epuck_project')
    full_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_dir, 'launch',
            'two_robots_decentralized_exploration_launch.py')),
        launch_arguments={
            # This wrapper deliberately selects the committed reviewed
            # fixture.  An explicit world_path is still supported for a
            # campaign-owned staged copy of that same fixture.
            'world_profile': LaunchConfiguration('world_profile'),
            'world_path': LaunchConfiguration('world_path'),
            'webots_port': LaunchConfiguration('webots_port'),
            'webots_mode': LaunchConfiguration('webots_mode'),
            'webots_gui': LaunchConfiguration('webots_gui'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'ideal_encoder_sensing': 'true',
            'diagnostic_mode': LaunchConfiguration('diagnostic_mode'),
            'diagnostic_frontier_capture': LaunchConfiguration(
                'diagnostic_frontier_capture'),
            'fusion_process_nice': '0',
            'sensor_profile': 'full',
            'nav2_autostart': 'true',
            'dispatch_enabled': 'true',
            'unknown_initial_pose': 'true',
            'unknown_pose_diagnostic_output': LaunchConfiguration(
                'unknown_pose_diagnostic_output'),
            'max_verification_batches': LaunchConfiguration(
                'max_verification_batches'),
            'verification_lifetime_s': LaunchConfiguration(
                'verification_lifetime_s'),
            'assignment_strategy': 'burgard',
            'burgard_beta': '1.0',
            'traffic_scheduler_enabled': 'false',
            'enable_observer': LaunchConfiguration('enable_observer'),
            'run_id': LaunchConfiguration('run_id'),
            'output_root': LaunchConfiguration('output_root'),
            'mission_timeout_s': LaunchConfiguration('mission_timeout_s'),
            'terminal_small_frontier_length_m': LaunchConfiguration(
                'terminal_small_frontier_length_m'),
            'enable_mission_timeout': LaunchConfiguration(
                'enable_mission_timeout'),
            'launch_rviz': 'false',
            'logger_console_status': LaunchConfiguration(
                'logger_console_status'),
            'enable_rosout_collection': LaunchConfiguration(
                'enable_rosout_collection'),
            'enable_coverage_attribution': LaunchConfiguration(
                'enable_coverage_attribution'),
            'enable_trajectory_overlap': LaunchConfiguration(
                'enable_trajectory_overlap'),
            'enable_forensic_capture': LaunchConfiguration(
                'enable_forensic_capture'),
            'enable_contact_capture': LaunchConfiguration(
                'enable_contact_capture'),
            'contact_sampling_period_ms': LaunchConfiguration(
                'contact_sampling_period_ms'),
            'controller_variant': 'rpp',
            'forensic_snapshot_interval_s': LaunchConfiguration(
                'forensic_snapshot_interval_s'),
        }.items(),
    )

    return LaunchDescription([
        _arg('world_profile', 'large_unknown_pose_close_start',
             ['large_unknown_pose', 'large_unknown_pose_16m',
              'large_unknown_pose_close_start',
              'large_unknown_pose_close_start_20ms']),
        _arg('world_path', ''),
        _arg('webots_port', '23000'),
        _arg('webots_mode', 'realtime'),
        _arg('webots_gui', 'false', ['true', 'false']),
        _arg('use_sim_time', 'true', ['true', 'false']),
        _arg('diagnostic_mode', 'false', ['true', 'false']),
        _arg('diagnostic_frontier_capture', 'false', ['true', 'false']),
        _arg('unknown_pose_diagnostic_output', ''),
        _arg('max_verification_batches', '64'),
        _arg('verification_lifetime_s', '1100.0'),
        _arg('enable_observer', 'true', ['true', 'false']),
        _arg('run_id', ''),
        _arg('output_root', '/home/arash/webots_ws/results'),
        _arg('mission_timeout_s', '600.0'),
        _arg('terminal_small_frontier_length_m', '0.20'),
        _arg('enable_mission_timeout', 'false', ['true', 'false']),
        _arg('logger_console_status', 'true', ['true', 'false']),
        _arg('enable_rosout_collection', 'true', ['true', 'false']),
        _arg('enable_coverage_attribution', 'true', ['true', 'false']),
        _arg('enable_trajectory_overlap', 'true', ['true', 'false']),
        _arg('enable_forensic_capture', 'true', ['true', 'false']),
        _arg('enable_contact_capture', 'false', ['true', 'false']),
        _arg('contact_sampling_period_ms', '20'),
        _arg('forensic_snapshot_interval_s', '15.0'),
        full_launch,
    ])
