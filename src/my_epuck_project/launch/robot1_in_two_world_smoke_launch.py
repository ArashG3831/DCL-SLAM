#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import webots_ros2_driver.webots_controller as webots_controller_module
import webots_ros2_driver.webots_launcher as webots_launcher_module
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    # WSL/Windows fix:
    # Webots is reachable from WSL at 127.0.0.1:23000.
    # The default webots_ros2 helper incorrectly picked another IP.
    webots_controller_module.controller_ip_address = lambda: '127.0.0.1'
    webots_launcher_module.controller_url_prefix = lambda port='1234': f'tcp://127.0.0.1:{port}/'

    package_dir = get_package_share_directory('my_epuck_project')

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    robot_description_path = os.path.join(package_dir, 'resource', 'epuck_d500_webots.urdf')
    with open(robot_description_path, 'r') as f:
        robot_description = f.read()

    webots_port = '23000'

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    webots = WebotsLauncher(
        world=PathJoinSubstitution([
            TextSubstitution(text=package_dir),
            'worlds',
            world,
        ]),
        ros2_supervisor=False,
        port=webots_port,
    )

    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ros_control_spawners = [
        diffdrive_controller_spawner,
        joint_state_broadcaster_spawner,
    ]

    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control.yml')

    mappings = [
        ('/diffdrive_controller/cmd_vel', '/cmd_vel'),
        ('/diffdrive_controller/odom', '/odom'),
    ]

    robot1_driver = WebotsController(
        robot_name='robot1',
        port=webots_port,
        parameters=[
            {
                'robot_description': robot_description_path,
                'use_sim_time': use_sim_time,
                'set_robot_state_publisher': True,
            },
            ros2_control_params,
        ],
        remappings=mappings,
        respawn=False,
    )

    epuck_process = Node(
        package='webots_ros2_epuck',
        executable='epuck_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    twist_stamper_node = Node(
        package='my_epuck_project',
        executable='twist_stamper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    waiting_nodes = WaitForControllerConnection(
        target_driver=robot1_driver,
        nodes_to_start=ros_control_spawners,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='epuck_d500_two_world_robot1_smoke.wbt',
            description='Two D500 e-pucks world; robot1 external, robot2 void for smoke test.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Temporary smoke test without /clock.',
        ),

        webots,
        robot_state_publisher,
        robot1_driver,
        epuck_process,
        twist_stamper_node,
        waiting_nodes,
    ])
