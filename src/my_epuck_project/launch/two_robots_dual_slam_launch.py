#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def make_slam_actions(package_dir, robot_name):
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=robot_name,
        output='screen',
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
            ('/map', f'/{robot_name}/map'),
            ('/map_metadata', f'/{robot_name}/map_metadata'),
        ],
        parameters=[
            os.path.join(
                package_dir,
                'resource',
                f'slam_toolbox_{robot_name}_two_robots.yaml',
            ),
            {
                'use_lifecycle_manager': False,
                'use_sim_time': False,
            },
        ],
    )

    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg=f'{robot_name.capitalize()} SLAM Toolbox is activating.'),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return [slam_node, configure_slam, activate_slam]


def generate_launch_description():
    package_dir = get_package_share_directory('my_epuck_project')

    two_robots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'two_robots_namespaced_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([
        two_robots,
        *make_slam_actions(package_dir, 'robot1'),
        *make_slam_actions(package_dir, 'robot2'),
    ])
