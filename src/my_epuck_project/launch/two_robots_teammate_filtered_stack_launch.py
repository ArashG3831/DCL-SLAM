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
    'controller_server', 'smoother_server', 'planner_server', 'route_server',
    'behavior_server', 'velocity_smoother', 'collision_monitor',
    'bt_navigator', 'waypoint_follower',
]


def nav2_nodes(package_dir, robot):
    source = os.path.join(
        package_dir, 'resource', f'nav2_{robot}_shared_map.yaml'
    )
    parameters = ParameterFile(
        RewrittenYaml(
            source_file=source,
            root_key=robot,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )
    common_remaps = [('tf', '/tf'), ('tf_static', '/tf_static')]
    specifications = [
        ('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_smoother', 'smoother_server', []),
        ('nav2_planner', 'planner_server', []),
        ('nav2_route', 'route_server', []),
        ('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_velocity_smoother', 'velocity_smoother',
         [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_collision_monitor', 'collision_monitor', []),
        ('nav2_bt_navigator', 'bt_navigator', []),
        ('nav2_waypoint_follower', 'waypoint_follower', []),
    ]
    nodes = [
        Node(
            package=package,
            executable=executable,
            name=executable,
            namespace=robot,
            output='screen',
            parameters=[parameters],
            remappings=common_remaps + extra_remaps,
        )
        for package, executable, extra_remaps in specifications
    ]
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
    filtered_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_dir, 'launch',
            'two_robots_teammate_filtered_dual_slam_launch.py',
        ))
    )
    alignment = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='shared_to_robot1_map',
            output='screen',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0', '--yaw', '0.0',
                '--frame-id', 'shared_map', '--child-frame-id', 'robot1/map',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='shared_to_robot2_map',
            output='screen',
            arguments=[
                '--x', '-0.299999998712', '--y', '-0.000027796077',
                '--z', '0.0', '--yaw', '-3.1415',
                '--frame-id', 'shared_map', '--child-frame-id', 'robot2/map',
            ],
        ),
    ]
    exchange = []
    for robot, peer in (('robot1', 'robot2'), ('robot2', 'robot1')):
        exchange.extend([
            Node(
                package='my_epuck_project',
                executable='map_exporter',
                name='map_exporter',
                namespace=robot,
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'source_robot_id': robot,
                    'input_topic': f'/{robot}/map',
                    'output_topic': f'/cslam/{robot}/local_map',
                    'export_rate_hz': 1.0,
                }],
            ),
            Node(
                package='my_epuck_project',
                executable='source_aware_map_fusion',
                name='map_fusion',
                namespace=robot,
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'local_map_topic': f'/{robot}/map',
                    'remote_peer_topic': f'/cslam/{peer}/local_map',
                    'expected_remote_source': peer,
                    'output_topic': 'shared_map',
                    'metadata_topic': 'shared_map_metadata',
                    'output_frame': 'shared_map',
                    'resolution': 0.01,
                }],
            ),
        ])
    return LaunchDescription([
        filtered_slam,
        *alignment,
        *exchange,
        *nav2_nodes(package_dir, 'robot1'),
        *nav2_nodes(package_dir, 'robot2'),
    ])
