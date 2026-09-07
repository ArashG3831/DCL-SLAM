#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from my_epuck_project.cooperative_profiles import profile_for_world
from my_epuck_project.slam_range_policy import FREE_SPACE_CAP
from my_epuck_project.thesis_baseline_topology import parse_active_robots


def slam_actions(package_dir, robot, slam_resolution, tf_probe_library,
                 tf_probe_log, tf_publication_mode, unknown_initial_pose,
                 slam_runtime_parameters, scan_topic=None):
    probe_env = {}
    if tf_probe_library:
        probe_env = {
            'LD_PRELOAD': tf_probe_library,
            'SLAM_TF_PUBLISH_PROBE_LOG': tf_probe_log,
        }
        if tf_publication_mode:
            probe_env['RMW_FASTRTPS_PUBLICATION_MODE'] = tf_publication_mode
    runtime_parameters = {
        'use_lifecycle_manager': False,
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'resolution': slam_resolution,
        'scan_topic': scan_topic or f'/{robot}/scan_d500_slam',
    }
    if slam_runtime_parameters:
        # Raw Webots transport QoS belongs to the namespaced driver/relay
        # launch. Do not pass it as an undeclared Slam Toolbox parameter.
        runtime_parameters.update({
            key: value for key, value in slam_runtime_parameters.items()
            if key != 'scan_input_reliability'
        })
    else:
        # Existing profiles retain their launch-argument override path.
        runtime_parameters.update({
            'use_scan_matching': LaunchConfiguration('use_scan_matching'),
            'do_loop_closing': LaunchConfiguration('do_loop_closing'),
        })
    slam = LifecycleNode(
        # The validation wrapper preserves upstream Slam Toolbox behavior but
        # replaces only its hard-coded sensor-data QoS with reliable QoS for
        # the full 720-reading corrected scan on WSL loopback.
        package='reliable_slam_toolbox_wrapper',
        executable='reliable_async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=robot,
        output='screen',
        additional_env=probe_env,
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
            ('/map', f'/{robot}/map'),
            ('/map_metadata', f'/{robot}/map_metadata'),
        ],
        parameters=[
            os.path.join(
                package_dir, 'resource',
                f'slam_toolbox_{robot}_teammate_filtered.yaml',
            ),
            runtime_parameters,
        ],
    )
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam,
        start_state='configuring',
        goal_state='inactive',
        entities=[
            LogInfo(msg=f'{robot} filtered-scan SLAM is activating.'),
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(slam),
                transition_id=Transition.TRANSITION_ACTIVATE,
            )),
        ],
    ))
    return [slam, configure, activate]


def launch_setup(context):
    package_dir = get_package_share_directory('my_epuck_project')
    world_path = LaunchConfiguration('world_path').perform(context)
    tf_probe_library = LaunchConfiguration(
        'slam_tf_publish_probe_library').perform(context)
    tf_probe_log = LaunchConfiguration(
        'slam_tf_publish_probe_log').perform(context)
    tf_publication_mode = LaunchConfiguration(
        'slam_tf_publication_mode').perform(context)
    selected = profile_for_world(
        LaunchConfiguration('world_profile').perform(context),
        os.path.dirname(world_path) if world_path else os.path.join(package_dir, 'worlds'),
        explicit_world_path=world_path,
    )
    unknown_initial_pose = (
        LaunchConfiguration('unknown_initial_pose').perform(context).lower()
        == 'true')
    launch_mapping = (
        LaunchConfiguration('launch_mapping').perform(context).lower()
        == 'true')
    phase_already_aligned = (
        LaunchConfiguration('phase_already_aligned').perform(context).lower()
        == 'true')
    active_robots = parse_active_robots(
        LaunchConfiguration('active_robots').perform(context))
    independent_local_maps = (
        LaunchConfiguration('independent_local_maps').perform(context).lower()
        == 'true')
    slam_runtime_parameters = dict(
        selected.get('slam_runtime_parameters', {}))
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_dir, 'launch', 'two_robots_namespaced_launch.py'
        )),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'sensor_profile': LaunchConfiguration('sensor_profile'),
            'world': selected['world'],
            'world_path': world_path,
            'webots_port': LaunchConfiguration('webots_port'),
            'webots_controller_port': LaunchConfiguration(
                'webots_controller_port'),
            'webots_mode': LaunchConfiguration('webots_mode'),
            'webots_gui': LaunchConfiguration('webots_gui'),
            'scan_publish_period': str(
                slam_runtime_parameters.get('minimum_time_interval', 0.0)),
            'lidar_update_rate': str(
                slam_runtime_parameters.get('lidar_update_rate', 0.0)),
            'scan_input_reliability': LaunchConfiguration(
                'scan_input_reliability'),
            'scan_transport': LaunchConfiguration('scan_transport'),
            'corrected_scan_reliability': LaunchConfiguration(
                'corrected_scan_reliability'),
            'corrected_scan_depth': LaunchConfiguration(
                'corrected_scan_depth'),
            'active_robots': LaunchConfiguration('active_robots'),
            'independent_local_maps': LaunchConfiguration('independent_local_maps'),
        }.items(),
    )
    filters = []
    fixed_odom = {
        'robot1': list(selected['world_metadata']['relative_transform']),
        'robot2': list(
            selected['world_metadata']['reverse_relative_transform']),
    }
    for robot, peer in (('robot1', 'robot2'), ('robot2', 'robot1')):
        if (independent_local_maps or robot not in active_robots
                or len(active_robots) < 2):
            continue
        filters.append(Node(
                package='my_epuck_project',
                executable='teammate_scan_filter',
                name='teammate_scan_filter',
                namespace=robot,
                output='screen',
                remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'input_topic': f'/{robot}/scan_d500_fixed',
                    'output_topic': f'/{robot}/scan_d500_slam',
                    'peer_base_frame': f'{peer}/base_footprint',
                    'expected_lidar_frame': f'{robot}/d500_lidar',
                    'own_odom_frame': f'{robot}/odom',
                    'peer_odom_frame': f'{peer}/odom',
                    'own_odom_to_peer_odom': fixed_odom[robot],
                    'robot_id': robot,
                    'map_frame': f'{robot}/map',
                    'accepted_hypothesis_topic':
                        '/cslam/relative_pose/hypotheses',
                    'active_at_start': not unknown_initial_pose,
                    'require_accepted_handoff': unknown_initial_pose,
                    'record_prehandoff_path': unknown_initial_pose,
                    'pre_handoff_path_topic':
                        f'/cslam/unknown_pose/{robot}/pre_handoff_path',
                    'trajectory_min_spacing_m': 0.03,
                    'trajectory_max_samples': 2048,
                    'peer_radius_m': 0.060,
                    'range_tolerance_m': 0.005,
                    'teammate_geometry_radius_m': LaunchConfiguration(
                        'teammate_geometry_radius_m'),
                    'maximum_processing_latency': 0.2,
                    'pending_queue_depth': 4,
                    'transform_retry_period': 0.02,
                    'warning_interval': 2.0,
                    'simulation_free_space_completion': True,
                    'free_space_cap': FREE_SPACE_CAP,
                }],
                ))
    local_frame_anchors = []
    if unknown_initial_pose and not phase_already_aligned:
        for robot in ('robot1', 'robot2'):
            local_frame_anchors.append(Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='local_map_frame_anchor',
                namespace=robot,
                output='screen',
                arguments=[
                    '--x', '0.0', '--y', '0.0', '--z', '0.0', '--yaw', '0.0',
                    '--frame-id', f'{robot}/local_world',
                    '--child-frame-id', f'{robot}/map',
                ],
            ))
    mapping_actions = []
    if launch_mapping:
        mapping_actions = [
            base,
            *filters,
            *local_frame_anchors,
            *sum((slam_actions(
                package_dir, robot, selected['slam_resolution'],
                tf_probe_library, tf_probe_log, tf_publication_mode,
                unknown_initial_pose, slam_runtime_parameters,
                scan_topic=(f'/{robot}/scan_d500_fixed'
                            if independent_local_maps or len(active_robots) == 1
                            else None))
                  for robot in active_robots), []),
        ]
    return mapping_actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_profile',
            default_value='small',
            choices=['large', 'small', 'large_unknown_pose',
                     'large_unknown_pose_16m',
                     'large_unknown_pose_close_start',
                     'large_unknown_pose_close_start_20ms',
                     'large_unknown_pose_close_start_20ms_scan_matching',
                     'large_unknown_pose_far_start_20ms_scan_matching'],
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
        DeclareLaunchArgument('sensor_profile', default_value='full',
                              choices=['full', 'throughput']),
        DeclareLaunchArgument('scan_input_reliability', default_value='reliable',
                              choices=['reliable', 'best_effort']),
        DeclareLaunchArgument('scan_transport', default_value='chunked',
                              choices=['chunked', 'laser_scan']),
        DeclareLaunchArgument(
            'corrected_scan_reliability', default_value='reliable',
            choices=['reliable', 'best_effort']),
        DeclareLaunchArgument('corrected_scan_depth', default_value='100'),
        DeclareLaunchArgument('active_robots', default_value='robot1,robot2'),
        DeclareLaunchArgument('independent_local_maps', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('slam_tf_publish_probe_library', default_value=''),
        DeclareLaunchArgument('slam_tf_publish_probe_log', default_value=''),
        DeclareLaunchArgument('slam_tf_publication_mode', default_value='',
                              choices=['', 'SYNCHRONOUS', 'ASYNCHRONOUS']),
        DeclareLaunchArgument(
            'teammate_geometry_radius_m', default_value='0.026'),
        DeclareLaunchArgument(
            'unknown_initial_pose', default_value='false',
            choices=['true', 'false']),
        DeclareLaunchArgument('launch_mapping', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('phase_already_aligned', default_value='false',
                              choices=['true', 'false']),
        OpaqueFunction(function=launch_setup),
    ])
