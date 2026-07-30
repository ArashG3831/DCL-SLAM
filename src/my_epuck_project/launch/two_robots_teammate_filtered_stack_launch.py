#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from my_epuck_project.cooperative_profiles import profile


LIFECYCLE_NODES = [
    'controller_server', 'smoother_server', 'planner_server', 'route_server',
    'behavior_server', 'velocity_smoother', 'collision_monitor',
    'bt_navigator', 'waypoint_follower',
]


def nav2_nodes(package_dir, robot, selected):
    source = os.path.join(
        package_dir, 'resource', f'nav2_{robot}_shared_map.yaml'
    )
    parameters = ParameterFile(
        RewrittenYaml(
            source_file=source,
            root_key=robot,
            param_rewrites={
                'local_costmap.local_costmap.ros__parameters.resolution':
                    str(selected['local_costmap_resolution']),
                'global_costmap.global_costmap.ros__parameters.resolution':
                    str(selected['global_costmap_resolution']),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'controller_server.ros__parameters.progress_checker.required_movement_radius':
                    '0.08' if selected['name'] == 'large' else '0.5',
                'controller_server.ros__parameters.progress_checker.movement_time_allowance':
                    '18.0' if selected['name'] == 'large' else '10.0',
            },
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
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': LIFECYCLE_NODES,
        }],
    ))
    return nodes


def launch_setup(context):
    package_dir = get_package_share_directory('my_epuck_project')
    world_path = LaunchConfiguration('world_path').perform(context)
    selected = profile(
        LaunchConfiguration('world_profile').perform(context),
        os.path.dirname(world_path) if world_path else os.path.join(package_dir, 'worlds'),
    )
    filtered_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_dir, 'launch',
            'two_robots_teammate_filtered_dual_slam_launch.py',
        )),
        launch_arguments={
            'world_profile': selected['name'],
            'webots_port': LaunchConfiguration('webots_port'),
            'webots_mode': LaunchConfiguration('webots_mode'),
            'webots_gui': LaunchConfiguration('webots_gui'),
            'sensor_profile': LaunchConfiguration('sensor_profile'),
            'world_path': world_path,
        }.items(),
    )
    relative = selected['world_metadata']['relative_transform']
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
                '--x', str(relative[0]), '--y', str(relative[1]),
                '--z', '0.0', '--yaw', str(relative[2]),
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
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
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
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'local_map_topic': f'/{robot}/map',
                    'remote_peer_topic': f'/cslam/{peer}/local_map',
                    'expected_remote_source': peer,
                    'output_topic': 'shared_map',
                    'metadata_topic': 'shared_map_metadata',
                    'output_frame': 'shared_map',
                    'resolution': selected['fusion_resolution'],
                }],
            ),
        ])
    return [
        filtered_slam,
        *alignment,
        *exchange,
        *nav2_nodes(package_dir, 'robot1', selected),
        *nav2_nodes(package_dir, 'robot2', selected),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_profile',
            default_value='small',
            choices=['large', 'small'],
        ),
        DeclareLaunchArgument('webots_port', default_value='23000'),
        DeclareLaunchArgument('world_path', default_value=''),
        DeclareLaunchArgument('webots_mode', default_value='realtime'),
        DeclareLaunchArgument('webots_gui', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('sensor_profile', default_value='full',
                              choices=['full', 'throughput']),
        OpaqueFunction(function=launch_setup),
    ])
