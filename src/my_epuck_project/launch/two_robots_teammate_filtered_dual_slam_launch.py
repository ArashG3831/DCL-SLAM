#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def slam_actions(package_dir, robot):
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
            {'use_lifecycle_manager': False, 'use_sim_time': False},
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


def generate_launch_description():
    package_dir = get_package_share_directory('my_epuck_project')
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_dir, 'launch', 'two_robots_namespaced_launch.py'
        )),
        launch_arguments={
            'use_sim_time': 'false',
            'world': 'epuck_d500_two_world_teammate_visible.wbt',
        }.items(),
    )
    filters = []
    fixed_odom = {
        'robot1': [-0.299999998712, -0.000027796077, -3.1415],
        'robot2': [-0.299999999999703, -0.000000000000102, 3.1415],
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
                'use_sim_time': False,
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
    return LaunchDescription([
        base,
        *filters,
        *slam_actions(package_dir, 'robot1'),
        *slam_actions(package_dir, 'robot2'),
    ])
