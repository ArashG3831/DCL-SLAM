#!/usr/bin/env python3
"""Single e-puck motion/SLAM characterization launch.

This is a diagnostic-only launch. Ground truth comes from a separate,
read-only Webots Supervisor observer; no diagnostic GPS or Compass devices are
added to the robot.
"""

import os
import sys
import tempfile

from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown, matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from webots_ros2_driver.webots_launcher import WebotsLauncher
import webots_ros2_driver.webots_controller as webots_controller_module
import webots_ros2_driver.webots_launcher as webots_launcher_module
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_path',
            default_value=os.path.join(
                get_package_share_directory('my_epuck_project'), 'worlds',
                'epuck_motion_characterization.wbt')),
        DeclareLaunchArgument(
            'urdf_path',
            default_value=os.path.join(
                get_package_share_directory('my_epuck_project'), 'resource',
                'epuck_motion_characterization.urdf')),
        DeclareLaunchArgument('webots_port', default_value='24160'),
        DeclareLaunchArgument('webots_controller_port', default_value=''),
        DeclareLaunchArgument('webots_mode', default_value='realtime'),
        DeclareLaunchArgument('webots_gui', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('output_root', default_value='/tmp/motion_characterization'),
        DeclareLaunchArgument('ground_truth_output', default_value=''),
        DeclareLaunchArgument('enable_slam', default_value='false'),
        DeclareLaunchArgument(
            'slam_params',
            default_value=os.path.join(
                get_package_share_directory('my_epuck_project'), 'resource',
                'slam_toolbox_robot1_two_robots.yaml')),
        DeclareLaunchArgument('profile', default_value='speed_0p10'),
        DeclareLaunchArgument('target_speed', default_value='0.10'),
        DeclareLaunchArgument('acceleration', default_value='0.20'),
        DeclareLaunchArgument('course_mode', default_value='false'),
        DeclareLaunchArgument('course_name', default_value='short'),
        DeclareLaunchArgument('course_max_sim_time', default_value='180.0'),
        DeclareLaunchArgument('course_gate_file', default_value=''),
        DeclareLaunchArgument('course_command_file', default_value=''),
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context):
    # WSL/Webots uses the Windows host endpoint.
    webots_controller_module.controller_ip_address = lambda: '127.0.0.1'
    webots_launcher_module.controller_url_prefix = lambda port='1234': f'tcp://127.0.0.1:{port}/'

    package_dir = get_package_share_directory('my_epuck_project')
    driver_prefix = get_package_prefix('webots_ros2_driver')
    world_path = LaunchConfiguration('world_path').perform(context)
    urdf_path = LaunchConfiguration('urdf_path').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    webots_port = LaunchConfiguration('webots_port').perform(context)
    controller_port = LaunchConfiguration('webots_controller_port').perform(context)
    controller_port = controller_port or webots_port
    if controller_port != webots_port:
        raise RuntimeError(
            'motion characterization requires the Webots and controller ports '
            f'to match; got webots_port={webots_port} '
            f'webots_controller_port={controller_port}')
    webots_mode = LaunchConfiguration('webots_mode').perform(context)
    webots_gui = LaunchConfiguration('webots_gui').perform(context).lower() == 'true'
    output_root = LaunchConfiguration('output_root').perform(context)
    ground_truth_output = LaunchConfiguration('ground_truth_output').perform(context)
    if not ground_truth_output:
        ground_truth_output = os.path.join(output_root, 'supervisor_ground_truth.csv')
    robot = 'robot1'
    enable_slam = LaunchConfiguration('enable_slam').perform(context).lower() == 'true'
    course_mode = LaunchConfiguration('course_mode').perform(context).lower() == 'true'
    course_gate_file = LaunchConfiguration('course_gate_file').perform(context)
    course_command_file = LaunchConfiguration('course_command_file').perform(context)
    if course_mode:
        course_gate_file = course_gate_file or os.path.join(output_root, 'course_gate.json')
        course_command_file = course_command_file or os.path.join(output_root, 'course_command.json')

    webots = WebotsLauncher(
        world=world_path,
        ros2_supervisor=True,
        port=webots_port,
        mode=webots_mode,
        gui=webots_gui,
    )

    with open(os.path.join(package_dir, 'resource', 'ros2_control.yml'), encoding='utf-8') as f:
        control = f.read()
    control = control.replace('    base_frame_id: base_link', '    base_frame_id: base_footprint')
    control = control.replace('    enable_odom_tf: false', '    enable_odom_tf: true')
    control = control.replace(
        '    enable_odom_tf: true',
        '    enable_odom_tf: true\n    odom_frame_id: odom')
    control = control.replace('controller_manager:\n', f'/{robot}/controller_manager:\n', 1)
    control = control.replace('\ndiffdrive_controller:\n', f'\n/{robot}/diffdrive_controller:\n', 1)
    control = control.replace('\njoint_state_broadcaster:\n', f'\n/{robot}/joint_state_broadcaster:\n', 1)
    control_path = os.path.join(
        tempfile.gettempdir(), 'my_epuck_motion_characterization_ros2_control.yml')
    with open(control_path, 'w', encoding='utf-8') as f:
        f.write(control)

    # Run the native driver directly instead of the legacy webots-controller
    # wrapper. The wrapper execs a second child (`ros2 run ... driver`) which
    # can outlive launch shutdown; a direct Node is tracked and reaped by ROS 2
    # launch, keeping repeated diagnostic runs bounded.
    driver_site = os.path.join(
        driver_prefix, 'lib', f'python{sys.version_info.major}.{sys.version_info.minor}',
        'site-packages')
    inherited_ld = os.environ.get('LD_LIBRARY_PATH', '')
    inherited_pythonpath = os.environ.get('PYTHONPATH', '')
    driver_env = {
        'WEBOTS_HOME': driver_prefix,
        'WEBOTS_CONTROLLER_URL':
            f'tcp://127.0.0.1:{controller_port}/{robot}',
        'LD_LIBRARY_PATH': os.pathsep.join(
            p for p in (os.path.join(driver_prefix, 'lib', 'controller'), inherited_ld) if p),
        'PYTHONPATH': os.pathsep.join(
            p for p in (os.path.join(driver_prefix, 'lib', 'controller', 'python'),
                        driver_site, inherited_pythonpath) if p),
    }
    driver = Node(
        package='webots_ros2_driver', executable='driver', namespace=robot,
        output='screen', additional_env=driver_env,
        parameters=[
            {'robot_description': urdf_path, 'use_sim_time': use_sim_time,
             'set_robot_state_publisher': False},
            control_path,
        ],
        remappings=[('/scan_d500', f'/{robot}/scan_d500')],
    )

    with open(urdf_path, encoding='utf-8') as f:
        robot_description = f.read()
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace=robot, output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}],
    )

    diffdrive = Node(
        package='controller_manager', executable='spawner', namespace=robot,
        output='screen', prefix='python.exe' if os.name == 'nt' else '',
        arguments=['diffdrive_controller', '--controller-manager', f'/{robot}/controller_manager',
                   '--controller-manager-timeout', '50',
                   '--controller-ros-args=--ros-args '
                   f'--remap /{robot}/diffdrive_controller/odom:=/{robot}/odom '
                   f'--remap /{robot}/diffdrive_controller/cmd_vel:=/{robot}/cmd_vel '
                   ],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    joints = Node(
        package='controller_manager', executable='spawner', namespace=robot,
        output='screen', prefix='python.exe' if os.name == 'nt' else '',
        arguments=['joint_state_broadcaster', '--controller-manager', f'/{robot}/controller_manager',
                   '--controller-manager-timeout', '50',
                   '--controller-ros-args=--ros-args '
                   f'--remap /joint_states:=/{robot}/joint_states '
                   f'--remap /dynamic_joint_states:=/{robot}/dynamic_joint_states'],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    stamper = Node(
        package='my_epuck_project', executable='twist_stamper', namespace=robot,
        name='motion_twist_stamper', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_unstamped', f'/{robot}/cmd_vel_unstamped'),
                    ('/cmd_vel', f'/{robot}/cmd_vel')],
    )
    scan_fix = Node(
        package='my_epuck_project', executable='d500_scan_fix', namespace=robot,
        name='motion_d500_scan_fix', output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'input_topic': f'/{robot}/scan_d500',
                     'output_topic': f'/{robot}/scan_d500_fixed'}],
    )
    odom_tf_bridge = ExecuteProcess(
        cmd=[sys.executable, '-m', 'my_epuck_project.motion_odom_tf_bridge',
             '--ros-args', '-p', 'use_sim_time:=true',
             '-p', f'odom_topic:={robot}/odom',
             '-p', f'parent_frame:={robot}/odom',
             '-p', 'child_frame:=base_footprint'],
        name='motion_odom_tf_bridge', output='screen', respawn=False,
    )
    if course_mode:
        motion_node = Node(
            package='my_epuck_project', executable='motion_course_node',
            name='motion_course_node', output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'output_root': LaunchConfiguration('output_root'),
                         'profile': LaunchConfiguration('profile'),
                         'target_speed': LaunchConfiguration('target_speed'),
                         'acceleration': LaunchConfiguration('acceleration'),
                         'course_name': LaunchConfiguration('course_name'),
                         'gate_file': course_gate_file,
                         'command_file': course_command_file}],
        )
    else:
        motion_node = Node(
            package='my_epuck_project', executable='motion_characterization_node',
            name='motion_characterization_node', output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'output_root': LaunchConfiguration('output_root'),
                         'profile': LaunchConfiguration('profile'),
                         'target_speed': LaunchConfiguration('target_speed'),
                         'acceleration': LaunchConfiguration('acceleration')}],
        )

    # This process talks directly to the diagnostic Supervisor robot. It is
    # intentionally outside ROS and only records world pose/velocity.
    python_version = f'python{sys.version_info.major}.{sys.version_info.minor}'
    observer_python = os.path.join(
        driver_prefix, 'lib', python_version, 'site-packages')
    inherited_pythonpath = os.environ.get('PYTHONPATH', '')
    observer_pythonpath = os.pathsep.join(
        p for p in (observer_python, inherited_pythonpath) if p)
    observer_module = ('my_epuck_project.motion_course_supervisor'
                       if course_mode else 'my_epuck_project.motion_supervisor_observer')
    observer_cmd = [sys.executable, '-m', observer_module,
                    '--output', ground_truth_output, '--robot-def', 'ROBOT1']
    if course_mode:
        observer_cmd += ['--command-file', course_command_file,
                          '--gate-file', course_gate_file,
                          '--max-sim-time', LaunchConfiguration(
                              'course_max_sim_time').perform(context),
                          '--course-name', LaunchConfiguration(
                              'course_name').perform(context)]
    else:
        observer_cmd += ['--max-sim-time', '40.0']
    observer = ExecuteProcess(
        cmd=observer_cmd,
        name='motion_supervisor_observer', output='screen', respawn=False,
        additional_env={
            'WEBOTS_HOME': driver_prefix,
            'WEBOTS_CONTROLLER_URL':
                f'tcp://127.0.0.1:{webots_port}/MotionGroundTruthSupervisor',
            'PYTHONPATH': observer_pythonpath,
            'PYTHONUNBUFFERED': '1',
        },
    )
    start_observer = RegisterEventHandler(OnProcessStart(
        target_action=webots, on_start=[observer]))
    stop_launch_on_observer_complete = RegisterEventHandler(OnProcessExit(
        target_action=observer,
        on_exit=[EmitEvent(event=Shutdown(reason='motion ground truth complete'))],
    ))

    wait = WaitForControllerConnection(target_driver=driver, nodes_to_start=[diffdrive])
    start_joints = RegisterEventHandler(OnProcessExit(
        target_action=diffdrive,
        on_exit=lambda event, context: [joints] if event.returncode == 0 else [],
    ))
    # A failed controller spawner must terminate the diagnostic immediately;
    # otherwise the external Supervisor keeps Webots alive until the full
    # course timeout and the artifact can be mistaken for a motion result.
    stop_on_diffdrive_failure = RegisterEventHandler(OnProcessExit(
        target_action=diffdrive,
        on_exit=lambda event, context: (
            [EmitEvent(event=Shutdown(reason='motion diffdrive spawner failed'))]
            if event.returncode != 0 else [])))
    # Do not start the motion profile while ros2_control is still waiting for
    # its robot description or controller services. Otherwise the initial
    # command step is silently dropped and acceleration measurements are false.
    start_motion = RegisterEventHandler(OnProcessExit(
        target_action=joints,
        on_exit=lambda event, context: [motion_node] if event.returncode == 0 else [],
    ))
    stop_on_joints_failure = RegisterEventHandler(OnProcessExit(
        target_action=joints,
        on_exit=lambda event, context: (
            [EmitEvent(event=Shutdown(reason='motion joint-state spawner failed'))]
            if event.returncode != 0 else [])))

    slam_scan_topic = (f'/{robot}/scan_d500_slam' if course_mode
                       else f'/{robot}/scan_d500_fixed')
    slam = LifecycleNode(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', namespace=robot, output='screen',
        remappings=[
            ('tf', '/tf'), ('tf_static', '/tf_static'),
            ('/map', f'/{robot}/map'),
            ('/map_metadata', f'/{robot}/map_metadata'),
        ],
        parameters=[LaunchConfiguration('slam_params'), {
            'use_lifecycle_manager': False,
            'use_sim_time': use_sim_time,
            # The diagnostic URDF intentionally has unprefixed physical link
            # names.  Only the odometry/map frames carry the robot namespace;
            # these overrides keep the diagnostic scan TF chain valid without
            # changing the production two-robot SLAM parameter files.
            'odom_frame': f'{robot}/odom',
            'map_frame': f'{robot}/map',
            'base_frame': 'base_footprint',
            'scan_topic': slam_scan_topic,
        }],
    )
    scan_branch_relay = Node(
        package='my_epuck_project', executable='motion_scan_branch_relay',
        name='motion_scan_branch_relay', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    configure_slam = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam),
        transition_id=Transition.TRANSITION_CONFIGURE))
    activate_slam = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam,
        start_state='configuring', goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_ACTIVATE))]))
    slam_recorder = ExecuteProcess(
        cmd=[sys.executable, '-m', 'my_epuck_project.motion_slam_recorder',
             '--ros-args', '-p', 'use_sim_time:=true',
             '-p', f'output_root:={output_root}'],
        name='motion_slam_recorder', output='screen', respawn=False)
    start_slam = RegisterEventHandler(OnProcessExit(
        target_action=joints,
        on_exit=lambda event, context: (
            ([scan_branch_relay, slam, configure_slam, activate_slam, slam_recorder]
             if course_mode else [slam, configure_slam, activate_slam, slam_recorder])
            if event.returncode == 0 else [])))

    actions = [
        webots, webots._supervisor, robot_state_publisher, driver,
        scan_fix, odom_tf_bridge, wait, start_joints, start_motion,
        stop_on_diffdrive_failure, stop_on_joints_failure,
        start_observer, stop_launch_on_observer_complete,
    ]
    if not course_mode:
        actions.insert(4, stamper)
    if enable_slam:
        actions.append(start_slam)
    return actions
