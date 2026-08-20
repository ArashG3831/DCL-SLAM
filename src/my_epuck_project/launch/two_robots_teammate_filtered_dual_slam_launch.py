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


def slam_actions(package_dir, robot, slam_resolution, tf_probe_library,
                 tf_probe_log, tf_publication_mode, unknown_initial_pose):
    probe_env = {}
    if tf_probe_library:
        probe_env = {
            'LD_PRELOAD': tf_probe_library,
            'SLAM_TF_PUBLISH_PROBE_LOG': tf_probe_log,
            'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        }
        if tf_publication_mode:
            probe_env['RMW_FASTRTPS_PUBLICATION_MODE'] = tf_publication_mode
    slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
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
            {
                'use_lifecycle_manager': False,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'resolution': slam_resolution,
                'scan_topic': (
                    f'/{robot}/scan_d500_fixed'
                    if unknown_initial_pose else
                    f'/{robot}/scan_d500_slam'),
                # Runtime-only diagnostic overrides.  The checked-in YAML
                # remains the production source of truth (false/false).
                'use_scan_matching': LaunchConfiguration('use_scan_matching'),
                'do_loop_closing': LaunchConfiguration('do_loop_closing'),
            },
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
        }.items(),
    )
    filters = []
    fixed_odom = {
        'robot1': list(selected['world_metadata']['relative_transform']),
        'robot2': list(
            selected['world_metadata']['reverse_relative_transform']),
    }
    for robot, peer in (('robot1', 'robot2'), ('robot2', 'robot1')):
        if not unknown_initial_pose:
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
    if unknown_initial_pose:
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
    return [
        base,
        *filters,
        *local_frame_anchors,
        *slam_actions(package_dir, 'robot1', selected['slam_resolution'],
                      tf_probe_library, tf_probe_log, tf_publication_mode,
                      unknown_initial_pose),
        *slam_actions(package_dir, 'robot2', selected['slam_resolution'],
                      tf_probe_library, tf_probe_log, tf_publication_mode,
                      unknown_initial_pose),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_profile',
            default_value='small',
            choices=['large', 'small', 'large_unknown_pose',
                     'large_unknown_pose_16m'],
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
        DeclareLaunchArgument('slam_tf_publish_probe_library', default_value=''),
        DeclareLaunchArgument('slam_tf_publish_probe_log', default_value=''),
        DeclareLaunchArgument('slam_tf_publication_mode', default_value='',
                              choices=['', 'SYNCHRONOUS', 'ASYNCHRONOUS']),
        DeclareLaunchArgument(
            'teammate_geometry_radius_m', default_value='0.026'),
        DeclareLaunchArgument(
            'unknown_initial_pose', default_value='false',
            choices=['true', 'false']),
        OpaqueFunction(function=launch_setup),
    ])
