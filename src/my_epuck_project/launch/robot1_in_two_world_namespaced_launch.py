#!/usr/bin/env python3

import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import webots_ros2_driver.webots_controller as webots_controller_module
import webots_ros2_driver.webots_launcher as webots_launcher_module
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def _default_route_gateway():
    try:
        result = subprocess.run(
            ['ip', 'route', 'show', 'default'], check=True,
            capture_output=True, text=True, timeout=1.0)
    except (OSError, subprocess.SubprocessError) as exc:
        raise RuntimeError('Unable to resolve the WSL NAT gateway') from exc
    for line in result.stdout.splitlines():
        fields = line.split()
        if fields and fields[0] == 'default' and 'via' in fields:
            return fields[fields.index('via') + 1]
    raise RuntimeError('No default-route gateway found for Windows Webots')


def _resolve_controller_host():
    explicit = os.environ.get('MY_EPUCK_WEBOTS_NETWORK_MODE', '').strip().lower()
    if explicit in ('mirrored', 'loopback'):
        return '127.0.0.1'
    if explicit in ('nat', 'subnet'):
        return _default_route_gateway()
    localhost_only = os.environ.get('ROS_LOCALHOST_ONLY', '').strip().lower()
    discovery = os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE', '').strip().upper()
    if localhost_only in ('1', 'true', 'yes') or discovery != 'SUBNET':
        return '127.0.0.1'
    return _default_route_gateway()


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='epuck_d500_two_world_robot1_active.wbt',
            description='Two D500 e-pucks world; robot1 external, robot2 void for single-active-robot test.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Temporary robot1 active test without /clock.',
        ),
        DeclareLaunchArgument(
            'webots_port', default_value='23000',
            description='Webots TCP port for the single-robot fixture.',
        ),
        DeclareLaunchArgument(
            'webots_gui', default_value='false',
            description='Enable the Webots GUI for interactive fixture use.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context):
    controller_host = _resolve_controller_host()
    webots_controller_module.controller_ip_address = lambda: controller_host
    webots_launcher_module.controller_url_prefix = lambda port='1234': f'tcp://{controller_host}:{port}/'

    package_dir = get_package_share_directory('my_epuck_project')

    world = LaunchConfiguration('world').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    robot_description_path = os.path.join(package_dir, 'resource', 'epuck_d500_webots.urdf')
    with open(robot_description_path, 'r') as f:
        robot_description = f.read()

    webots_port = LaunchConfiguration('webots_port').perform(context)
    webots_gui = LaunchConfiguration('webots_gui').perform(context).lower() == 'true'

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot1',
        output='screen',
        remappings=[
            ('/joint_states', '/robot1/joint_states'),
        ],
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
        gui=webots_gui,
    )

    diffdrive_controller_spawner = Node(
        package='my_epuck_project',
        executable='controller_startup_guard',
        namespace='robot1',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'controller_name': 'diffdrive_controller',
            'controller_manager': '/robot1/controller_manager',
            'controller_param_file': '/tmp/my_epuck_project_robot1_ros2_control.yml',
            'switch_timeout_s': 10.0,
            'controller_ros_args': (
                '--ros-args --remap /robot1/diffdrive_controller/odom:=/robot1/odom '
                '--remap /robot1/diffdrive_controller/cmd_vel:=/robot1/cmd_vel '
                '--param tf_frame_prefix:=robot1/'),
        }],
    )

    joint_state_broadcaster_spawner = Node(
        package='my_epuck_project',
        executable='controller_startup_guard',
        namespace='robot1',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'controller_name': 'joint_state_broadcaster',
            'controller_manager': '/robot1/controller_manager',
            'controller_param_file': '/tmp/my_epuck_project_robot1_ros2_control.yml',
            'switch_timeout_s': 10.0,
            'controller_ros_args': (
                '--ros-args --remap /joint_states:=/robot1/joint_states '
                '--remap /dynamic_joint_states:=/robot1/dynamic_joint_states'),
        }],
    )

    ros2_control_source = os.path.join(package_dir, 'resource', 'ros2_control.yml')
    with open(ros2_control_source, 'r') as f:
        robot1_control_config = f.read()
    robot1_control_config = robot1_control_config.replace(
        '    base_frame_id: base_link',
        '    base_frame_id: base_footprint',
    ).replace(
        '    enable_odom_tf: false',
        '    enable_odom_tf: true',
    ).replace(
        'controller_manager:\n',
        '/robot1/controller_manager:\n', 1,
    ).replace(
        '\ndiffdrive_controller:\n',
        '\n/robot1/diffdrive_controller:\n', 1,
    ).replace(
        '\njoint_state_broadcaster:\n',
        '\n/robot1/joint_state_broadcaster:\n', 1,
    )
    ros2_control_params = '/tmp/my_epuck_project_robot1_ros2_control.yml'
    with open(ros2_control_params, 'w') as f:
        f.write(robot1_control_config)

    mappings = [
        ('/scan_d500', '/robot1/scan_d500'),
    ]

    robot1_driver = WebotsController(
        robot_name='robot1',
        namespace='robot1',
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
        remappings=[
            ('/cmd_vel_unstamped', '/robot1/cmd_vel_unstamped'),
            ('/cmd_vel', '/robot1/cmd_vel'),
            ('/odom', '/robot1/odom'),
        ],
    )

    waiting_nodes = WaitForControllerConnection(
        target_driver=robot1_driver,
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

    return [
        webots,
        robot_state_publisher,
        robot1_driver,
        epuck_process,
        twist_stamper_node,
        start_joint_state_broadcaster,
        waiting_nodes,
    ]
