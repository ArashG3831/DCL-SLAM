#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


LIFECYCLE_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'route_server',
    'behavior_server',
    'velocity_smoother',
    'collision_monitor',
    'bt_navigator',
    'waypoint_follower',
]


def nav2_nodes(robot, parameters):
    configured_parameters = ParameterFile(
        RewrittenYaml(
            source_file=parameters,
            root_key=robot,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )
    common = {
        'namespace': robot,
        'output': 'screen',
        'parameters': [configured_parameters],
        'remappings': [('tf', '/tf'), ('tf_static', '/tf_static')],
    }
    specifications = [
        ('nav2_controller', 'controller_server', 'controller_server',
         [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_smoother', 'smoother_server', 'smoother_server', []),
        ('nav2_planner', 'planner_server', 'planner_server', []),
        ('nav2_route', 'route_server', 'route_server', []),
        ('nav2_behaviors', 'behavior_server', 'behavior_server',
         [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
         [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_collision_monitor', 'collision_monitor', 'collision_monitor', []),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', []),
        ('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower', []),
    ]
    nodes = []
    for package, executable, name, extra_remaps in specifications:
        nodes.append(Node(
            package=package,
            executable=executable,
            name=name,
            namespace=common['namespace'],
            output=common['output'],
            parameters=common['parameters'],
            remappings=common['remappings'] + extra_remaps,
        ))
    nodes.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace=robot,
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': LIFECYCLE_NODES,
        }],
    ))
    return nodes


def generate_launch_description():
    package_dir = get_package_share_directory('my_epuck_project')
    peer_exchange = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_dir, 'launch', 'two_robots_peer_map_exchange_launch.py'
        ))
    )
    actions = [peer_exchange]
    for robot in ('robot1', 'robot2'):
        parameters = os.path.join(
            package_dir, 'resource', f'nav2_{robot}_shared_map.yaml'
        )
        actions.extend(nav2_nodes(robot, parameters))
    return LaunchDescription(actions)
