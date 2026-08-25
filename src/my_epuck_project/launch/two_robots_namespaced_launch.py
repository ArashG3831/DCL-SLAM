#!/usr/bin/env python3

import os
import re
import tempfile

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import (
    get_package_prefix, get_package_share_directory,
)

import webots_ros2_driver.webots_controller as webots_controller_module
import webots_ros2_driver.webots_launcher as webots_launcher_module
from webots_ros2_driver.webots_launcher import (
    Ros2SupervisorLauncher, WebotsLauncher,
)
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def launch_setup(context):
    # WSL reaches Windows Webots through this fixed external-controller endpoint.
    webots_controller_module.controller_ip_address = lambda: '127.0.0.1'
    webots_launcher_module.controller_url_prefix = lambda port='1234': f'tcp://127.0.0.1:{port}/'

    package_dir = get_package_share_directory('my_epuck_project')
    world = LaunchConfiguration('world').perform(context)
    world_path = LaunchConfiguration('world_path').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    webots_port = LaunchConfiguration('webots_port').perform(context)
    controller_port = LaunchConfiguration(
        'webots_controller_port').perform(context) or webots_port
    webots_mode = LaunchConfiguration('webots_mode').perform(context)
    webots_gui = (
        LaunchConfiguration('webots_gui').perform(context).lower() == 'true'
    )
    sensor_profile = LaunchConfiguration('sensor_profile').perform(context)
    lidar_update_rate = LaunchConfiguration('lidar_update_rate').perform(context)
    scan_input_reliability = LaunchConfiguration(
        'scan_input_reliability').perform(context)
    scan_transport = LaunchConfiguration('scan_transport').perform(context)
    # ROS parameter typing is strict. LaunchConfiguration values are strings;
    # convert the scan period before passing it to rclpy's DOUBLE parameter.
    scan_publish_period = float(
        LaunchConfiguration('scan_publish_period').perform(context))

    base_urdf_path = os.path.join(package_dir, 'resource', 'epuck_d500_webots.urdf')
    base_control_path = os.path.join(package_dir, 'resource', 'ros2_control.yml')
    with open(base_urdf_path, 'r') as f:
        base_urdf = f.read()
    if sensor_profile == 'throughput':
        # The campaign audit found no project subscriber or controller use for
        # these devices. Keep lidar, odometry, and IMU unchanged.
        for device in ('ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7', 'tof'):
            base_urdf = base_urdf.replace(
                '<enabled>true</enabled>', '<enabled>false</enabled>', 1)
        base_urdf = base_urdf.replace(
            '<topicName>/camera</topicName>',
            '<topicName>/camera</topicName>\n'
            '                <enabled>false</enabled>', 1)
    with open(base_control_path, 'r') as f:
        base_control = f.read()

    webots = WebotsLauncher(
        world=(world_path or PathJoinSubstitution([
            TextSubstitution(text=package_dir), 'worlds', world,
        ])),
        # The Webots ROS 2 supervisor is the authoritative publisher of the
        # simulation /clock.  Robot controllers consume that one clock.
        ros2_supervisor=True,
        port=webots_port,
        mode=webots_mode,
        gui=webots_gui,
    )
    # The launcher modifies the world to add Ros2Supervisor when
    # ros2_supervisor=True, but its internal supervisor client uses the same
    # requested listen port.  Under this WSL/Webots build the server may
    # redirect by two ports; replace only that client with the confirmed
    # actual port while preserving the official world augmentation.
    if webots_mode == 'fast':
        # The stock supervisor publishes /clock on every 20 ms physics step.
        # In accelerated mode that becomes an unbounded reliable DDS stream
        # which starves lidar and lifecycle callbacks.  The project wrapper
        # keeps the same Supervisor step/services but coalesces /clock.
        webots._supervisor = Node(
            package='my_epuck_project',
            executable='paced_ros2_supervisor',
            namespace='Ros2Supervisor',
            remappings=[('/Ros2Supervisor/clock', '/clock')],
            output='screen',
            additional_env={
                'WEBOTS_CONTROLLER_URL': (
                    f'tcp://127.0.0.1:{controller_port}/Ros2Supervisor'),
                'WEBOTS_HOME': get_package_prefix('webots_ros2_driver'),
            },
            respawn=False,
        )
    else:
        webots._supervisor = Ros2SupervisorLauncher(port=controller_port)

    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    robot_actions = []
    for robot_name in ('robot1', 'robot2'):
        if scan_transport == 'chunked':
            # The stock Ros2Lidar publisher serializes a 720-beam LaserScan
            # as one DDS sample.  That sample is dropped by the verified WSL
            # best-effort path before d500_scan_fix can see it.  Disable only
            # that publisher and let the in-process Webots plugin publish
            # bounded 180-beam chunks from the same lidar device.
            # Remove the stock Ros2Lidar device block entirely.  An
            # ``enabled=false`` property still leaves a publisher endpoint
            # in this driver version, so retaining it would create an
            # unconsumed full-size LaserScan writer alongside the chunk
            # publisher.
            robot_urdf_base = re.sub(
                r'\s*<device reference="d500_lidar" type="Lidar">.*?</device>\s*',
                '\n', base_urdf, count=1, flags=re.DOTALL)
            lidar_properties = None
        else:
            lidar_properties = (
                '<topicName>/scan_d500</topicName>\n'
                f'                <frameName>{robot_name}/d500_lidar</frameName>'
            )
        if lidar_properties is not None and float(lidar_update_rate) > 0.0:
            lidar_properties += (
                f'\n                <updateRate>{lidar_update_rate}</updateRate>')
        robot_urdf = (robot_urdf_base if scan_transport == 'chunked' else
                      base_urdf.replace(
                          '<topicName>/scan_d500</topicName>', lidar_properties, 1))
        if scan_transport == 'chunked':
            plugin = (
                '        <plugin type="my_epuck_frontier_candidates::ChunkedLidarPlugin">\n'
                f'            <robot_id>{robot_name}</robot_id>\n'
                '            <topic>scan_d500_chunks</topic>\n'
                f'            <frame_id>{robot_name}/d500_lidar</frame_id>\n'
                '            <chunk_beams>180</chunk_beams>\n'
                f'            <update_rate>{lidar_update_rate}</update_rate>\n'
                '            <lidar_name>d500_lidar</lidar_name>\n'
                '        </plugin>\n'
            )
            robot_urdf = robot_urdf.replace('    </webots>', plugin + '    </webots>', 1)
        robot_urdf_path = os.path.join(
            tempfile.gettempdir(), f'my_epuck_project_{robot_name}.urdf')
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
        robot_control_path = os.path.join(
            tempfile.gettempdir(),
            f'my_epuck_project_{robot_name}_ros2_control.yml',
        )
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

        manager_name = f'/{robot_name}/controller_manager'
        diffdrive_controller_spawner = Node(
            package='my_epuck_project',
            executable='controller_startup_guard',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'controller_name': 'diffdrive_controller',
                'controller_manager': manager_name,
                'controller_param_file': robot_control_path,
                'controller_ros_args': (
                    '--ros-args '
                    f'--remap /{robot_name}/diffdrive_controller/odom:=/{robot_name}/odom '
                    f'--remap /{robot_name}/diffdrive_controller/cmd_vel:=/{robot_name}/cmd_vel '
                    f'--param tf_frame_prefix:={robot_name}/'),
            }],
        )

        joint_state_broadcaster_spawner = Node(
            package='my_epuck_project',
            executable='controller_startup_guard',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'controller_name': 'joint_state_broadcaster',
                'controller_manager': manager_name,
                'controller_param_file': robot_control_path,
                'controller_ros_args': (
                    '--ros-args '
                    f'--remap /joint_states:=/{robot_name}/joint_states '
                    f'--remap /dynamic_joint_states:=/{robot_name}/dynamic_joint_states'),
            }],
        )

        robot_driver = WebotsController(
            robot_name=robot_name,
            namespace=robot_name,
            port=controller_port,
            parameters=[
                {
                    'robot_description': robot_urdf_path,
                    'use_sim_time': use_sim_time,
                    'set_robot_state_publisher': True,
                    'qos_overrides./scan_d500.publisher.reliability':
                        scan_input_reliability,
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
            # This bridge stamps the outgoing command from the incoming
            # message and does not need a ROS-time clock subscription.  In
            # Webots fast mode /clock can run at thousands of callbacks per
            # wall second; opting this stateless bridge out prevents it from
            # starving the sensor and command callbacks.
            parameters=[{'use_sim_time': False}],
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
                # Scan headers already carry Webots simulation timestamps;
                # the fixer has no timers.  Avoid subscribing it to the very
                # high-rate /clock stream in fast mode so raw scans continue
                # to reach the corrected output.
                'use_sim_time': False,
                'input_topic': (
                    'scan_d500_chunks' if scan_transport == 'chunked'
                    else 'scan_d500'),
                'input_mode': scan_transport,
                'source_robot_id': robot_name,
                'max_pending_scans': 4,
                # Four reliable chunks are published back-to-back, but the
                # full Nav2/frontier graph can delay one callback for more
                # than a quarter second.  Keep assembly bounded while
                # allowing one complete 1 Hz scan to arrive atomically.  The
                # timeout is still finite and the pending-scan cap bounds
                # memory even if a peer disappears.
                'assembly_timeout_s': 2.0,
                'max_chunks': 8,
                'output_topic': 'scan_d500_fixed',
                'input_reliability': scan_input_reliability,
                'minimum_time_interval': scan_publish_period,
                # One raw reader publishes both corrected streams.  Slam
                # receives the full reliable output; Nav2 receives a bounded
                # latest-sample best-effort output.
                'secondary_output_topic': 'scan_d500_nav',
                'secondary_output_depth': 1,
                'secondary_output_reliability': 'best_effort',
                'secondary_output_sample_count': 180,
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

    # WebotsLauncher creates the official Ros2Supervisor action separately;
    # include it so its single /clock publisher is actually launched.
    return [webots, webots._supervisor] + robot_actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='epuck_d500_two_world_both_active.wbt',
            description='Two active D500 e-pucks with independent ROS namespaces.',
        ),
        DeclareLaunchArgument(
            'world_path', default_value='',
            description='Exact saved world path; overrides installed profile copy.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Run against the external Windows Webots clock behavior.',
        ),
        DeclareLaunchArgument(
            'webots_port',
            default_value='23000',
            description='Requested Webots server listen port.',
        ),
        DeclareLaunchArgument(
            'webots_controller_port', default_value='',
            description='Confirmed actual Webots port used by controllers.',
        ),
        DeclareLaunchArgument(
            'webots_mode',
            default_value='realtime',
            choices=['pause', 'realtime', 'fast'],
            description='Webots startup simulation mode.',
        ),
        DeclareLaunchArgument(
            'webots_gui',
            default_value='true',
            choices=['true', 'false'],
            description='Enable Webots rendering and its normal window.',
        ),
        DeclareLaunchArgument('sensor_profile', default_value='full',
                              choices=['full', 'throughput']),
        DeclareLaunchArgument(
            'scan_publish_period', default_value='0.0',
            description='Minimum simulation-time interval for corrected scans.'),
        DeclareLaunchArgument(
            'lidar_update_rate', default_value='0.0',
            description='Optional Webots ROS lidar publication rate in Hz.'),
        DeclareLaunchArgument(
            'scan_input_reliability', default_value='reliable',
            choices=['reliable', 'best_effort'],
            description='Reliability for the raw Webots lidar stream.'),
        DeclareLaunchArgument(
            'scan_transport', default_value='chunked',
            choices=['chunked', 'laser_scan'],
            description='Raw Webots scan transport; chunked avoids DDS fragmentation.'),
        OpaqueFunction(function=launch_setup),
    ])
