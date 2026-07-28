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

    robot2_slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='robot2',
        output='screen',
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
            ('/map', '/robot2/map'),
            ('/map_metadata', '/robot2/map_metadata'),
        ],
        parameters=[
            os.path.join(
                package_dir,
                'resource',
                'slam_toolbox_robot2_two_robots.yaml',
            ),
            {
                'use_lifecycle_manager': False,
                'use_sim_time': False,
            },
        ],
    )

    configure_robot2_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(robot2_slam),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_robot2_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=robot2_slam,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='Robot2 SLAM Toolbox is activating.'),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(robot2_slam),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription([
        two_robots,
        robot2_slam,
        configure_robot2_slam,
        activate_robot2_slam,
    ])
