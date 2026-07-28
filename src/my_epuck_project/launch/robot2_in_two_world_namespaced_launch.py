#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
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

    robot_description_path = os.path.join(package_dir, 'resource', 'epuck_d500_webots_robot2.urdf')
    with open(robot_description_path, 'r') as f:
        robot_description = f.read()

    webots_port = '23000'

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        remappings=[
            ('/joint_states', '/robot2/joint_states'),
        ],
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
            'frame_prefix': 'robot2/',
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
        arguments=[
            'diffdrive_controller',
        ] + controller_manager_timeout + [
            '--controller-ros-args=--ros-args --remap /diffdrive_controller/odom:=/robot2/odom --remap /diffdrive_controller/cmd_vel:=/robot2/cmd_vel --param tf_frame_prefix:=robot2/',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=[
            'joint_state_broadcaster',
        ] + controller_manager_timeout + [
            '--controller-ros-args=--ros-args --remap /joint_states:=/robot2/joint_states --remap /dynamic_joint_states:=/robot2/dynamic_joint_states',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ros2_control_source = os.path.join(package_dir, 'resource', 'ros2_control.yml')
    with open(ros2_control_source, 'r') as f:
        robot2_control_config = f.read()

    robot2_control_config = robot2_control_config.replace(
        '    base_frame_id: base_link',
        '    base_frame_id: base_footprint',
    )
    ros2_control_params = '/tmp/my_epuck_project_robot2_ros2_control.yml'
    with open(ros2_control_params, 'w') as f:
        f.write(robot2_control_config)

    mappings = [
        ('/scan_d500', '/robot2/scan_d500'),
    ]

    robot2_driver = WebotsController(
        robot_name='robot2',
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

    twist_stamper_node = Node(
        package='my_epuck_project',
        executable='twist_stamper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel_unstamped', '/robot2/cmd_vel_unstamped'),
            ('/cmd_vel', '/robot2/cmd_vel'),
        ],
    )

    waiting_nodes = WaitForControllerConnection(
        target_driver=robot2_driver,
        nodes_to_start=[diffdrive_controller_spawner],
    )

    start_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=diffdrive_controller_spawner,
            on_exit=lambda event, context: (
                [joint_state_broadcaster_spawner]
                if event.returncode == 0
                else []
            ),
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='epuck_d500_two_world_robot2_active.wbt',
            description='Two D500 e-pucks world; robot2 external, robot1 void for single-active-robot test.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Temporary robot2 active test without /clock.',
        ),

        webots,
        robot_state_publisher,
        robot2_driver,
        twist_stamper_node,
        start_joint_state_broadcaster,
        waiting_nodes,
    ])
