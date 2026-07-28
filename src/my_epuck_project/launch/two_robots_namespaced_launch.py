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
    # WSL reaches Windows Webots through this fixed external-controller endpoint.
    webots_controller_module.controller_ip_address = lambda: '127.0.0.1'
    webots_launcher_module.controller_url_prefix = lambda port='1234': f'tcp://127.0.0.1:{port}/'

    package_dir = get_package_share_directory('my_epuck_project')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    webots_port = '23000'

    base_urdf_path = os.path.join(package_dir, 'resource', 'epuck_d500_webots.urdf')
    base_control_path = os.path.join(package_dir, 'resource', 'ros2_control.yml')
    with open(base_urdf_path, 'r') as f:
        base_urdf = f.read()
    with open(base_control_path, 'r') as f:
        base_control = f.read()

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

    robot_actions = []
    for robot_name in ('robot1', 'robot2'):
        robot_urdf = base_urdf.replace(
            '<topicName>/scan_d500</topicName>',
            '<topicName>/scan_d500</topicName>\n'
            f'                <frameName>{robot_name}/d500_lidar</frameName>',
        )
        robot_urdf_path = f'/tmp/my_epuck_project_{robot_name}.urdf'
        with open(robot_urdf_path, 'w') as f:
            f.write(robot_urdf)

        robot_control = base_control.replace(
            '    base_frame_id: base_link',
            '    base_frame_id: base_footprint',
        )
        robot_control = robot_control.replace(
            '    enable_odom_tf: false',
            '    enable_odom_tf: true',
        )
        robot_control = robot_control.replace(
            'controller_manager:\n',
            f'/{robot_name}/controller_manager:\n',
            1,
        )
        robot_control = robot_control.replace(
            '\ndiffdrive_controller:\n',
            f'\n/{robot_name}/diffdrive_controller:\n',
        )
        robot_control = robot_control.replace(
            '\njoint_state_broadcaster:\n',
            f'\n/{robot_name}/joint_state_broadcaster:\n',
        )
        robot_control_path = f'/tmp/my_epuck_project_{robot_name}_ros2_control.yml'
        with open(robot_control_path, 'w') as f:
            f.write(robot_control)

        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_name,
            output='screen',
            remappings=[
                ('/joint_states', f'/{robot_name}/joint_states'),
            ],
            parameters=[{
                'robot_description': robot_urdf,
                'use_sim_time': use_sim_time,
                'frame_prefix': f'{robot_name}/',
            }],
        )

        diffdrive_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            namespace=robot_name,
            output='screen',
            prefix=controller_manager_prefix,
            arguments=[
                'diffdrive_controller',
                '--controller-manager', f'/{robot_name}/controller_manager',
            ] + controller_manager_timeout + [
                '--controller-ros-args=--ros-args '
                f'--remap /{robot_name}/diffdrive_controller/odom:=/{robot_name}/odom '
                f'--remap /{robot_name}/diffdrive_controller/cmd_vel:=/{robot_name}/cmd_vel '
                f'--param tf_frame_prefix:={robot_name}/',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        )

        joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            namespace=robot_name,
            output='screen',
            prefix=controller_manager_prefix,
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', f'/{robot_name}/controller_manager',
            ] + controller_manager_timeout + [
                '--controller-ros-args=--ros-args '
                f'--remap /joint_states:=/{robot_name}/joint_states '
                f'--remap /dynamic_joint_states:=/{robot_name}/dynamic_joint_states',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        )

        robot_driver = WebotsController(
            robot_name=robot_name,
            namespace=robot_name,
            port=webots_port,
            parameters=[
                {
                    'robot_description': robot_urdf_path,
                    'use_sim_time': use_sim_time,
                    'set_robot_state_publisher': True,
                },
                robot_control_path,
            ],
            remappings=[
                ('/scan_d500', f'/{robot_name}/scan_d500'),
            ],
            respawn=False,
        )

        twist_stamper_node = Node(
            package='my_epuck_project',
            executable='twist_stamper',
            namespace=robot_name,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('/cmd_vel_unstamped', f'/{robot_name}/cmd_vel_unstamped'),
                ('/cmd_vel', f'/{robot_name}/cmd_vel'),
            ],
        )

        scan_fix_node = Node(
            package='my_epuck_project',
            executable='d500_scan_fix',
            name=f'{robot_name}_d500_scan_fix',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': 'scan_d500',
                'output_topic': 'scan_d500_fixed',
            }],
        )

        waiting_nodes = WaitForControllerConnection(
            target_driver=robot_driver,
            nodes_to_start=[diffdrive_controller_spawner],
        )

        start_joint_state_broadcaster = RegisterEventHandler(
            OnProcessExit(
                target_action=diffdrive_controller_spawner,
                on_exit=lambda event, context, joint_spawner=joint_state_broadcaster_spawner: (
                    [joint_spawner] if event.returncode == 0 else []
                ),
            )
        )

        robot_actions.extend([
            robot_state_publisher,
            robot_driver,
            twist_stamper_node,
            scan_fix_node,
            start_joint_state_broadcaster,
            waiting_nodes,
        ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='epuck_d500_two_world_both_active.wbt',
            description='Two active D500 e-pucks with independent ROS namespaces.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Run against the external Windows Webots clock behavior.',
        ),
        webots,
    ] + robot_actions)
