#!/usr/bin/env python3
"""Lightweight, one-robot Nav2 FollowPath residual comparison launch.

This is diagnostic-only.  It starts the real Webots ros2_control chain and a
minimal Nav2 controller/local-costmap stack, then sends a fixed path through
FollowPath.  Production YAML files and launches are never modified.
"""
import os, sys, tempfile
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
import webots_ros2_driver.webots_controller as webots_controller_module
import webots_ros2_driver.webots_launcher as webots_launcher_module
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def _controller_params(variant):
    controller = {
        'use_sim_time': True, 'controller_frequency': 20.0,
        'min_x_velocity_threshold': 0.001, 'min_y_velocity_threshold': 0.5,
        'min_theta_velocity_threshold': 0.001,
        'failure_tolerance': 0.3,
        'progress_checker_plugin': 'progress_checker',
        'goal_checker_plugins': ['general_goal_checker'],
        'controller_plugins': ['FollowPath'],
        'progress_checker': {'plugin': 'nav2_controller::SimpleProgressChecker',
                             'required_movement_radius': 0.25,
                             'movement_time_allowance': 20.0},
        'general_goal_checker': {'plugin': 'nav2_controller::SimpleGoalChecker',
                                 'xy_goal_tolerance': 0.06,
                                 'yaw_goal_tolerance': 6.28, 'stateful': True},
    }
    dwb = {
        'plugin': 'dwb_core::DWBLocalPlanner', 'debug_trajectory_details': False,
        'min_vel_x': 0.0, 'min_vel_y': 0.0, 'max_vel_x': 0.13, 'max_vel_y': 0.0,
        'max_vel_theta': 0.35, 'min_speed_xy': 0.0, 'max_speed_xy': 0.13,
        'min_speed_theta': 0.0, 'acc_lim_x': 0.20, 'acc_lim_y': 0.0,
        'acc_lim_theta': 1.2, 'decel_lim_x': -0.20, 'decel_lim_y': 0.0,
        'decel_lim_theta': -1.2, 'vx_samples': 6, 'vy_samples': 5,
        'vtheta_samples': 21, 'sim_time': 1.7, 'linear_granularity': 0.01,
        'angular_granularity': 0.025, 'transform_tolerance': 1.5,
        'xy_goal_tolerance': 0.06, 'trans_stopped_velocity': 0.04,
        'short_circuit_trajectory_evaluation': True, 'stateful': True,
        'critics': ['RotateToGoal', 'Oscillation', 'BaseObstacle', 'GoalAlign',
                    'PathAlign', 'PathDist', 'GoalDist'],
        'BaseObstacle.scale': 0.20, 'PathAlign.scale': 8.0,
        'PathAlign.forward_point_distance': 0.0025, 'GoalAlign.scale': 0.0,
        'GoalAlign.forward_point_distance': 0.0025, 'PathDist.scale': 24.0,
        'GoalDist.scale': 24.0, 'RotateToGoal.scale': 0.0,
        'RotateToGoal.slowing_factor': 5.0, 'RotateToGoal.lookahead_time': -1.0,
    }
    if variant == 'rpp':
        controller['FollowPath'] = {
            'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
            'desired_linear_vel': 0.13, 'lookahead_dist': 0.20,
            'min_lookahead_dist': 0.12, 'max_lookahead_dist': 0.30,
            'lookahead_time': 1.5, 'use_velocity_scaled_lookahead_dist': True,
            'use_regulated_linear_velocity_scaling': True,
            'regulated_linear_scaling_min_radius': 0.50,
            'regulated_linear_scaling_min_speed': 0.026,
            'use_rotate_to_heading': True, 'rotate_to_heading_min_angle': 0.785,
            'rotate_to_heading_angular_vel': 0.35, 'max_angular_accel': 0.20,
            'allow_reversing': False, 'use_collision_detection': True,
            'stateful': True,
        }
    else:
        controller['FollowPath'] = dwb
    return {
        'controller_server': {'ros__parameters': controller},
        'local_costmap': {'local_costmap': {'ros__parameters': {
            'use_sim_time': True, 'update_frequency': 5.0,
            'publish_frequency': 2.0, 'global_frame': 'robot1/odom',
            'robot_base_frame': 'base_footprint', 'rolling_window': True,
            'width': 4, 'height': 4, 'resolution': 0.01,
            'robot_radius': 0.055, 'plugins': ['inflation_layer'],
            'inflation_layer': {'plugin': 'nav2_costmap_2d::InflationLayer',
                                'inflation_radius': 0.11,
                                'cost_scaling_factor': 3.5},
        }}},
        'velocity_smoother': {'ros__parameters': {
            'use_sim_time': True, 'smoothing_frequency': 20.0,
            'scale_velocities': False, 'feedback': 'OPEN_LOOP',
            'max_velocity': [0.13, 0.0, 0.35], 'min_velocity': [0.0, 0.0, -0.35],
            'max_accel': [0.20, 0.0, 1.2], 'max_decel': [-0.20, 0.0, -1.2],
            'odom_topic': '/robot1/odom', 'odom_duration': 0.1,
            'velocity_timeout': 1.0}},
        'collision_monitor': {'ros__parameters': {
            'use_sim_time': True, 'base_frame_id': 'robot1/base_footprint',
            'odom_frame_id': 'robot1/odom', 'cmd_vel_in_topic': 'cmd_vel_smoothed',
            'cmd_vel_out_topic': 'cmd_vel_unstamped', 'state_topic': 'collision_monitor_state',
            'transform_tolerance': 0.2, 'source_timeout': 2.0,
            'stop_pub_timeout': 2.0, 'observation_sources': ['scan'],
            'scan': {'type': 'scan', 'topic': '/robot1/scan_d500_fixed', 'enabled': True},
            'polygons': ['StopCircle'], 'StopCircle': {'type': 'circle', 'radius': 0.08,
                'action_type': 'stop', 'min_points': 4, 'visualize': False}}},
        'lifecycle_manager_navigation': {'ros__parameters': {
            'use_sim_time': True, 'autostart': True,
            'node_names': ['controller_server', 'velocity_smoother', 'collision_monitor']}}
    }


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('variant', default_value='dwb', choices=['dwb', 'rpp']),
        DeclareLaunchArgument('route_mode', default_value='original', choices=['original', 'mirrored']),
        DeclareLaunchArgument('output_root', default_value='/tmp/controller_residual_test'),
        DeclareLaunchArgument('world_path', default_value=''),
        DeclareLaunchArgument('urdf_path', default_value=''),
        DeclareLaunchArgument('webots_port', default_value='23000'),
        DeclareLaunchArgument('webots_mode', default_value='fast'),
        DeclareLaunchArgument('webots_gui', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('max_sim_time', default_value='240.0'),
        OpaqueFunction(function=_setup),
    ])


def _setup(context):
    webots_controller_module.controller_ip_address = lambda: '127.0.0.1'
    webots_launcher_module.controller_url_prefix = lambda port='1234': f'tcp://127.0.0.1:{port}/'
    pkg = get_package_share_directory('my_epuck_project')
    prefix = get_package_prefix('webots_ros2_driver')
    world = LaunchConfiguration('world_path').perform(context) or os.path.join(pkg, 'worlds', 'epuck_motion_characterization.wbt')
    urdf = LaunchConfiguration('urdf_path').perform(context) or os.path.join(pkg, 'resource', 'epuck_motion_characterization.urdf')
    port = LaunchConfiguration('webots_port').perform(context)
    out = LaunchConfiguration('output_root').perform(context)
    variant = LaunchConfiguration('variant').perform(context)
    route_mode = LaunchConfiguration('route_mode').perform(context)
    os.makedirs(out, exist_ok=True)
    param_path = os.path.join(out, f'controller_params_{variant}.yaml')
    # Directly namespaced launch nodes require the namespace root in a ROS 2
    # parameter file (the normal Nav2 bringup RewrittenYaml adds this root).
    with open(param_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump({'robot1': _controller_params(variant)}, f, sort_keys=False)
    lifecycle_path = os.path.join(out, 'lifecycle_manager.yaml')
    with open(lifecycle_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump({'/**': {'ros__parameters': {
            'use_sim_time': True, 'autostart': True,
            'node_names': ['controller_server', 'velocity_smoother',
                           'collision_monitor']}}}, f, sort_keys=False)
    control = open(os.path.join(pkg, 'resource', 'ros2_control.yml'), encoding='utf-8').read()
    control = control.replace('    base_frame_id: base_link', '    base_frame_id: base_footprint')
    control = control.replace('    enable_odom_tf: false', '    enable_odom_tf: true\n    odom_frame_id: odom')
    control = control.replace('controller_manager:\n', '/robot1/controller_manager:\n', 1)
    control = control.replace('\ndiffdrive_controller:\n', '\n/robot1/diffdrive_controller:\n', 1)
    control = control.replace('\njoint_state_broadcaster:\n', '\n/robot1/joint_state_broadcaster:\n', 1)
    control_path = os.path.join(out, 'ros2_control_diagnostic.yaml'); open(control_path, 'w', encoding='utf-8').write(control)
    driver_site = os.path.join(prefix, 'lib', f'python{sys.version_info.major}.{sys.version_info.minor}', 'site-packages')
    inherited_ld = os.environ.get('LD_LIBRARY_PATH', '')
    env = {'WEBOTS_HOME': prefix, 'WEBOTS_CONTROLLER_URL': f'tcp://127.0.0.1:{port}/robot1',
           'LD_LIBRARY_PATH': os.pathsep.join(p for p in (os.path.join(prefix, 'lib', 'controller'),
                                                            os.path.join(prefix, 'lib'), inherited_ld) if p),
           'PYTHONPATH': os.pathsep.join([os.path.join(prefix, 'lib', 'controller', 'python'), driver_site])}
    webots = WebotsLauncher(world=world, ros2_supervisor=True, port=port,
                            mode=LaunchConfiguration('webots_mode').perform(context),
                            gui=LaunchConfiguration('webots_gui').perform(context).lower() == 'true')
    driver = Node(package='webots_ros2_driver', executable='driver', namespace='robot1', output='screen',
                  additional_env=env, parameters=[{'robot_description': open(urdf, encoding='utf-8').read(),
                  'use_sim_time': True, 'set_robot_state_publisher': False}, control_path],
                  remappings=[('/scan_d500', '/robot1/scan_d500')])
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', namespace='robot1', output='screen',
               parameters=[{'robot_description': open(urdf, encoding='utf-8').read(), 'use_sim_time': True}])
    diff = Node(package='controller_manager', executable='spawner', namespace='robot1', output='screen',
                arguments=['diffdrive_controller', '--controller-manager', '/robot1/controller_manager', '--controller-manager-timeout', '50',
                           '--controller-ros-args=--ros-args --remap /robot1/diffdrive_controller/odom:=/robot1/odom --remap /robot1/diffdrive_controller/cmd_vel:=/robot1/cmd_vel'])
    joints = Node(package='controller_manager', executable='spawner', namespace='robot1', output='screen',
                  arguments=['joint_state_broadcaster', '--controller-manager', '/robot1/controller_manager', '--controller-manager-timeout', '50',
                             '--controller-ros-args=--ros-args --remap /joint_states:=/robot1/joint_states --remap /dynamic_joint_states:=/robot1/dynamic_joint_states'])
    scan = Node(package='my_epuck_project', executable='d500_scan_fix', namespace='robot1', name='diagnostic_scan_fix', output='screen',
                parameters=[{'use_sim_time': True, 'input_topic': '/robot1/scan_d500', 'output_topic': '/robot1/scan_d500_fixed'}])
    bridge = ExecuteProcess(cmd=[sys.executable, '-m', 'my_epuck_project.motion_odom_tf_bridge', '--ros-args', '-p', 'use_sim_time:=true', '-p', 'odom_topic:=robot1/odom', '-p', 'parent_frame:=robot1/odom', '-p', 'child_frame:=base_footprint'], output='screen')
    stamper = Node(package='my_epuck_project', executable='twist_stamper', namespace='robot1', name='diagnostic_twist_stamper', output='screen',
                   parameters=[{'use_sim_time': True}], remappings=[('/cmd_vel_unstamped','/robot1/cmd_vel_unstamped'),('/cmd_vel','/robot1/cmd_vel')])
    ctrl = Node(package='nav2_controller', executable='controller_server', namespace='robot1', name='controller_server', output='screen',
                parameters=[param_path], remappings=[('/tf','/tf'),('/tf_static','/tf_static'),('cmd_vel','cmd_vel_nav')])
    smoother = Node(package='nav2_velocity_smoother', executable='velocity_smoother', namespace='robot1', name='velocity_smoother', output='screen',
                    parameters=[param_path], remappings=[('/tf','/tf'),('/tf_static','/tf_static'),('cmd_vel','cmd_vel_nav')])
    collision = Node(package='nav2_collision_monitor', executable='collision_monitor', namespace='robot1', name='collision_monitor', output='screen',
                     parameters=[param_path], remappings=[('/tf','/tf'),('/tf_static','/tf_static')])
    lifecycle = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', namespace='robot1', name='lifecycle_manager_navigation', output='screen',
                     parameters=[lifecycle_path])
    observer = ExecuteProcess(cmd=[sys.executable, '-m', 'my_epuck_project.motion_course_supervisor',
                                  '--output', os.path.join(out, 'supervisor_ground_truth.csv'),
                                  '--command-file', os.path.join(out, 'unused_command.json'),
                                  '--gate-file', os.path.join(out, 'unused_gate.json'),
                                  '--robot-def', 'ROBOT1', '--max-sim-time', LaunchConfiguration('max_sim_time').perform(context), '--course-name', 'short'],
                       name='controller_residual_supervisor', output='screen', additional_env={'WEBOTS_HOME': prefix, 'WEBOTS_CONTROLLER_URL': f'tcp://127.0.0.1:{port}/MotionGroundTruthSupervisor', 'PYTHONPATH': os.pathsep.join([driver_site, os.environ.get('PYTHONPATH','')]), 'PYTHONUNBUFFERED':'1'})
    logger_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'tools', 'turn_motion_diagnostics.py'))
    if not os.path.exists(logger_path):
        logger_path = '/home/arash/webots_ws/src/my_epuck_project/tools/turn_motion_diagnostics.py'
    logger = ExecuteProcess(cmd=[sys.executable, logger_path, '--robot','robot1','--output',os.path.join(out,'telemetry.csv'),'--duration-s',LaunchConfiguration('max_sim_time').perform(context),'--max-rows','250000','--flush-every','50'], name='controller_residual_logger', output='screen')
    route = Node(package='my_epuck_project', executable='fixed_follow_path_driver', output='screen', parameters=[{'use_sim_time': True}], arguments=['--output-root', out, '--variant', variant, '--route-mode', route_mode])
    wait = WaitForControllerConnection(target_driver=driver, nodes_to_start=[diff])
    start_joints = RegisterEventHandler(OnProcessExit(target_action=diff, on_exit=lambda e,c: [joints] if e.returncode == 0 else [EmitEvent(event=Shutdown(reason='diffdrive failed'))]))
    start_stack = RegisterEventHandler(OnProcessExit(target_action=joints, on_exit=lambda e,c: [ctrl, smoother, collision, lifecycle, stamper, logger, route] if e.returncode == 0 else [EmitEvent(event=Shutdown(reason='joint broadcaster failed'))]))
    start_observer = RegisterEventHandler(OnProcessStart(target_action=webots, on_start=[observer]))
    stop_route = RegisterEventHandler(OnProcessExit(target_action=route, on_exit=[EmitEvent(event=Shutdown(reason='fixed route complete'))]))
    return [webots, webots._supervisor, rsp, driver, scan, bridge, wait, start_joints, start_stack, start_observer, stop_route]
