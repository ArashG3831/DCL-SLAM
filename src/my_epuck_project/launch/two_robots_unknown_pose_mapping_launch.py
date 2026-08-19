#!/usr/bin/env python3

"""Controlled unknown-relative-pose mapping validation launch.

This launch intentionally omits the known-map alignment and the teammate scan
filter that requires a supplied inter-robot transform.  It is a mapping-only
front-end validation profile; the normal cooperative launch is unchanged.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from lifecycle_msgs.msg import Transition
from my_epuck_project.cooperative_profiles import profile


def slam_node(package_dir, robot, use_sim_time):
    node = LifecycleNode(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', namespace=robot, output='screen',
        remappings=[
            ('tf', '/tf'), ('tf_static', '/tf_static'),
            ('/map', f'/{robot}/map'),
            ('/map_metadata', f'/{robot}/map_metadata'),
        ],
        parameters=[
            os.path.join(
                package_dir, 'resource',
                f'slam_toolbox_{robot}_two_robots.yaml'),
            {
                'use_lifecycle_manager': False,
                'use_sim_time': use_sim_time,
                'use_scan_matching': False,
                'do_loop_closing': False,
            },
        ],
    )
    return [
        node,
        EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE)),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=node,
            start_state='configuring', goal_state='inactive',
            entities=[
                LogInfo(msg=f'{robot} unknown-pose SLAM activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(node),
                    transition_id=Transition.TRANSITION_ACTIVATE)),
            ])),
    ]


def launch_setup(context):
    package_dir = get_package_share_directory('my_epuck_project')
    selected = profile(
        LaunchConfiguration('world_profile').perform(context),
        os.path.join(package_dir, 'worlds'))
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_dir, 'launch', 'two_robots_namespaced_launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'sensor_profile': 'full', 'world': selected['world'],
            'webots_port': LaunchConfiguration('webots_port'),
            'webots_controller_port': LaunchConfiguration(
                'webots_controller_port'),
            'webots_mode': LaunchConfiguration('webots_mode'),
            'webots_gui': LaunchConfiguration('webots_gui'),
        }.items())
    nodes = []
    for robot, peer in (('robot1', 'robot2'), ('robot2', 'robot1')):
        nodes.extend(slam_node(package_dir, robot,
                                LaunchConfiguration('use_sim_time')))
        # Establish each local map under a robot-private frame only.  This is
        # not an inter-robot transform and carries no relative-pose estimate;
        # the shared_map edge is still absent until mutual registration.
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='local_map_frame_anchor', namespace=robot, output='screen',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0', '--yaw', '0.0',
                '--frame-id', f'{robot}/local_world',
                '--child-frame-id', f'{robot}/map',
            ]))
        nodes.append(Node(
            package='my_epuck_project', executable='unknown_pose_frontend',
            name='unknown_pose_frontend', namespace=robot, output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_id': robot, 'peer_robot_id': peer,
                'map_topic': f'/{robot}/map',
                'peer_map_topic': f'/cslam/unknown_pose/{robot}/local_map',
                'shared_frame': 'shared_map',
                'diagnostic_output': LaunchConfiguration('diagnostic_output'),
            }]))
        nodes.append(Node(
            package='my_epuck_project', executable='source_aware_map_fusion',
            name='map_fusion', namespace=robot, output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'local_map_topic': f'/{robot}/map',
                'remote_peer_topic': f'/cslam/unknown_pose/{peer}/local_map',
                'expected_remote_source': peer,
                'output_topic': 'shared_map',
                'metadata_topic': 'shared_map_metadata',
                'output_frame': 'shared_map', 'resolution': 0.01,
                'publish_on_callback': False,
                'min_fusion_rebuild_period_s': 1.0,
            }]))
    if LaunchConfiguration('enable_motion_fixture').perform(context).lower() == 'true':
        nodes.append(Node(
            package='my_epuck_project', executable='unknown_pose_motion_fixture',
            name='unknown_pose_motion_fixture', output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]))
    return [base, *nodes]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world_profile', default_value='large'),
        DeclareLaunchArgument('webots_port', default_value='23000'),
        DeclareLaunchArgument('webots_controller_port', default_value=''),
        DeclareLaunchArgument('webots_mode', default_value='realtime'),
        DeclareLaunchArgument('webots_gui', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('diagnostic_output', default_value=''),
        DeclareLaunchArgument('enable_motion_fixture', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
