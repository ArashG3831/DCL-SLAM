#!/usr/bin/env python3

"""Controlled unknown-relative-pose mapping validation launch.

This launch intentionally omits the known-map alignment and the teammate scan
filter that requires a supplied inter-robot transform.  It is a mapping-only
front-end validation profile; the normal cooperative launch is unchanged.
"""

import os
import shutil
import tempfile
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from lifecycle_msgs.msg import Transition
from my_epuck_project.cooperative_profiles import profile_for_world


def stage_forensic_world(source_world, staging_directory=''):
    """Create a self-contained validation world with the passive Supervisor."""
    source = Path(source_world).resolve()
    if not source.is_file():
        raise RuntimeError(f'validation world does not exist: {source}')
    root = Path(staging_directory).resolve() if staging_directory else Path(
        tempfile.mkdtemp(prefix='my_epuck_unknown_pose_forensic_'))
    worlds = root / 'worlds'
    worlds.mkdir(parents=True, exist_ok=True)
    source_protos = source.parent.parent / 'protos'
    staged_protos = root / 'protos'
    if not source_protos.is_dir():
        raise RuntimeError(f'project-local proto directory missing: {source_protos}')
    if not staged_protos.exists():
        shutil.copytree(source_protos, staged_protos)
    staged_world = worlds / source.name
    shutil.copyfile(source, staged_world)
    content = staged_world.read_text(encoding='utf-8')
    if 'name "ForensicGroundTruthSupervisor"' not in content or \
            'supervisor TRUE' not in content:
        with staged_world.open('a', encoding='utf-8') as stream:
            stream.write(
                '\nRobot {\n'
                '  name "ForensicGroundTruthSupervisor"\n'
                '  controller "<extern>"\n'
                '  supervisor TRUE\n'
                '}\n')
    content = staged_world.read_text(encoding='utf-8')
    if 'name "ForensicGroundTruthSupervisor"' not in content or \
            'controller "<extern>"' not in content or \
            'supervisor TRUE' not in content:
        raise RuntimeError(
            'staged validation world lacks ForensicGroundTruthSupervisor')
    return str(staged_world)


def slam_node(package_dir, robot, use_sim_time):
    node = LifecycleNode(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', namespace=robot, output='screen',
        remappings=[
            ('tf', '/tf'), ('tf_static', '/tf_static'),
            ('/map', f'/{robot}/map'),
            ('/map_metadata', f'/{robot}/map_metadata'),
        ],
        parameters=[
            os.path.join(
                package_dir, 'resource',
                f'slam_toolbox_{robot}_two_robots.yaml'),
            {
                'use_lifecycle_manager': False,
                'use_sim_time': use_sim_time,
                'use_scan_matching': False,
                'do_loop_closing': False,
            },
        ],
    )
    return [
        node,
        EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE)),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=node,
            start_state='configuring', goal_state='inactive',
            entities=[
                LogInfo(msg=f'{robot} unknown-pose SLAM activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(node),
                    transition_id=Transition.TRANSITION_ACTIVATE)),
            ])),
    ]


def launch_setup(context):
    package_dir = get_package_share_directory('my_epuck_project')
    launch_world_path = LaunchConfiguration('world_path').perform(context)
    selected = profile_for_world(
        LaunchConfiguration('world_profile').perform(context),
        os.path.dirname(launch_world_path)
        if launch_world_path else os.path.join(package_dir, 'worlds'),
        explicit_world_path=launch_world_path,
    )
    forensic_enabled = LaunchConfiguration(
        'enable_forensic_capture').perform(context).lower() == 'true'
    if forensic_enabled:
        launch_world_path = stage_forensic_world(
            launch_world_path or selected['world_path'],
            LaunchConfiguration('staging_directory').perform(context))
        forensic_writer = importlib.util.find_spec(
            'my_epuck_project.forensic_evidence')
        if forensic_writer is None or not forensic_writer.origin or \
                'forensic_evidence.py' not in forensic_writer.origin or \
                'maps_dir' not in Path(forensic_writer.origin).read_text(
                    encoding='utf-8'):
            raise RuntimeError(
                'required forensic/maps export path is unavailable')
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_dir, 'launch', 'two_robots_namespaced_launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'sensor_profile': 'full', 'world': selected['world'],
            'world_path': launch_world_path,
            'webots_port': LaunchConfiguration('webots_port'),
            'webots_controller_port': LaunchConfiguration(
                'webots_controller_port'),
            'webots_mode': LaunchConfiguration('webots_mode'),
            'webots_gui': LaunchConfiguration('webots_gui'),
        }.items())
    nodes = []
    for robot, peer in (('robot1', 'robot2'), ('robot2', 'robot1')):
        nodes.extend(slam_node(package_dir, robot,
                                LaunchConfiguration('use_sim_time')))
        # Establish each local map under a robot-private frame only.  This is
        # not an inter-robot transform and carries no relative-pose estimate;
        # the shared_map edge is still absent until mutual registration.
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='local_map_frame_anchor', namespace=robot, output='screen',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0', '--yaw', '0.0',
                '--frame-id', f'{robot}/local_world',
                '--child-frame-id', f'{robot}/map',
            ]))
        nodes.append(Node(
            package='my_epuck_project', executable='unknown_pose_frontend',
            name='unknown_pose_frontend', namespace=robot, output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_id': robot, 'peer_robot_id': peer,
                'map_topic': f'/{robot}/map',
                'peer_map_topic': f'/cslam/unknown_pose/{robot}/local_map',
                'shared_frame': 'shared_map',
                'diagnostic_output': LaunchConfiguration('diagnostic_output'),
            }]))
        nodes.append(Node(
            package='my_epuck_project', executable='source_aware_map_fusion',
            name='map_fusion', namespace=robot, output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'local_map_topic': f'/{robot}/map',
                'remote_peer_topic': f'/cslam/unknown_pose/{peer}/local_map',
                'expected_remote_source': peer,
                'output_topic': 'shared_map',
                'metadata_topic': 'shared_map_metadata',
                'output_frame': 'shared_map', 'resolution': 0.01,
                'publish_on_callback': False,
                'min_fusion_rebuild_period_s': 1.0,
            }]))
    readiness_gate = Node(
        package='my_epuck_project', executable='controller_readiness_gate',
        name='controller_readiness_gate', output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_ids': ['robot1', 'robot2'],
            'timeout_s': 120.0,
        }])
    launch_after_readiness = []
    if LaunchConfiguration('enable_motion_fixture').perform(context).lower() == 'true':
        launch_after_readiness.append(Node(
            package='my_epuck_project', executable='unknown_pose_motion_fixture',
            name='unknown_pose_motion_fixture', output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'start_delay_s': LaunchConfiguration(
                    'motion_fixture_start_delay_s'),
                'turn_duration_s': LaunchConfiguration(
                    'motion_fixture_turn_duration_s'),
                'drive_duration_s': LaunchConfiguration(
                    'motion_fixture_drive_duration_s'),
                'linear_speed': LaunchConfiguration(
                    'motion_fixture_linear_speed'),
                'angular_speed': LaunchConfiguration(
                    'motion_fixture_angular_speed'),
            }]))
    readiness_handler = RegisterEventHandler(OnProcessExit(
        target_action=readiness_gate,
        on_exit=lambda event, context: (
            launch_after_readiness if event.returncode == 0 else [])))
    return [base, *nodes, readiness_gate, readiness_handler]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world_profile', default_value='large'),
        DeclareLaunchArgument('world_path', default_value=''),
        DeclareLaunchArgument('webots_port', default_value='23000'),
        DeclareLaunchArgument('webots_controller_port', default_value=''),
        DeclareLaunchArgument('webots_mode', default_value='realtime'),
        DeclareLaunchArgument('webots_gui', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('diagnostic_output', default_value=''),
        DeclareLaunchArgument('enable_forensic_capture', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('staging_directory', default_value=''),
        DeclareLaunchArgument('enable_motion_fixture', default_value='false'),
        # Validation-only motion controls.  Defaults preserve the historical
        # short fixture; an explicit run may extend the drive to create
        # spatially separated overlapping keyframes without changing any
        # estimator or production navigation behavior.
        DeclareLaunchArgument('motion_fixture_start_delay_s', default_value='20.0'),
        DeclareLaunchArgument('motion_fixture_turn_duration_s', default_value='3.2'),
        DeclareLaunchArgument('motion_fixture_drive_duration_s', default_value='12.0'),
        DeclareLaunchArgument('motion_fixture_linear_speed', default_value='0.10'),
        DeclareLaunchArgument('motion_fixture_angular_speed', default_value='0.45'),
        OpaqueFunction(function=launch_setup),
    ])
