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
from my_epuck_project.cooperative_profiles import profile


def slam_actions(package_dir, robot, slam_resolution):
    slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=robot,
        output='screen',
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
    selected = profile(
        LaunchConfiguration('world_profile').perform(context),
        os.path.join(package_dir, 'worlds'),
    )
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_dir, 'launch', 'two_robots_namespaced_launch.py'
        )),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'sensor_profile': LaunchConfiguration('sensor_profile'),
            'world': selected['world'],
            'webots_port': LaunchConfiguration('webots_port'),
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
                'peer_shape_type': 'circle',
                'own_odom_frame': f'{robot}/odom',
                'peer_odom_frame': f'{peer}/odom',
                'own_odom_to_peer_odom': fixed_odom[robot],
                'peer_shape_dimensions': [0.050],
                'static_safety_margin': 0.003,
                'dynamic_motion_margin': 0.007,
                'range_matching_tolerance': 0.012,
                'maximum_processing_latency': 0.2,
                'pending_queue_depth': 4,
                'transform_retry_period': 0.02,
                'queue_overflow_policy': 'drop_oldest',
                'allow_latest_transform_fallback': False,
                'shared_tf_confirmation_scans': 5,
                'shared_tf_position_tolerance': 0.05,
                'shared_tf_yaw_tolerance': 0.15,
                'warning_interval': 2.0,
            }],
        ))
    return [
        base,
        *filters,
        *slam_actions(package_dir, 'robot1', selected['slam_resolution']),
        *slam_actions(package_dir, 'robot2', selected['slam_resolution']),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_profile',
            default_value='small',
            choices=['large', 'small'],
        ),
        DeclareLaunchArgument('webots_port', default_value='23000'),
        DeclareLaunchArgument('webots_mode', default_value='realtime'),
        DeclareLaunchArgument('webots_gui', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('sensor_profile', default_value='full',
                              choices=['full', 'throughput']),
        OpaqueFunction(function=launch_setup),
    ])
