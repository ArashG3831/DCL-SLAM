#!/usr/bin/env python3
"""Final two-robot decentralized mapping, pair assignment, and local Nav2 launch."""

import json
import hashlib
import os
import shutil
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo,
    OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from my_epuck_project.cooperative_profiles import (
    FRONTIER_DECISION_MAP_PARAMETERS,
    profile_for_world,
    profile_summary,
)


def _frontend_exit_handler(robot, handoff_marker_path=''):
    def on_exit(event, context):
        expected_handoff_teardown = bool(
            handoff_marker_path and os.path.isfile(handoff_marker_path))
        # The frontend is intentionally terminated after it writes the
        # accepted-handoff marker.  Depending on whether the signal is
        # observed by ros2's executable wrapper or by launch directly, a
        # SIGTERM/SIGINT may be normalized to exit code 1 instead of the
        # negative signal number.  Once the marker exists, code 1 is the
        # same expected phase transition; without the marker it remains a
        # fail-fast abnormal frontend exit.
        if event.returncode in (0, -2, -15) or (
                expected_handoff_teardown and event.returncode in (-9, 1)):
            return []
        return [EmitEvent(event=Shutdown(
            reason=(f'{robot} unknown-pose frontend exited with '
                    f'code {event.returncode}')))]
    return on_exit


def launch_setup(context):
    """Resolve the exact world once for both control and passive metadata."""
    package_dir = get_package_share_directory('my_epuck_project')
    world_path = LaunchConfiguration('world_path').perform(context)
    selected = profile_for_world(
        LaunchConfiguration('world_profile').perform(context),
        os.path.dirname(world_path) if world_path else os.path.join(package_dir, 'worlds'),
        explicit_world_path=world_path,
        ideal_encoder_sensing=(
            LaunchConfiguration('ideal_encoder_sensing').perform(context).lower()
            == 'true'),
    )
    summary = profile_summary(selected)
    source_world_sha256 = selected['world_metadata']['sha256']
    staged_source_world_sha256 = source_world_sha256
    staged_world_sha256 = source_world_sha256
    forensic_enabled = (
        LaunchConfiguration('enable_forensic_capture').perform(context).lower()
        == 'true')
    contact_enabled = (
        LaunchConfiguration('enable_contact_capture').perform(context).lower()
        == 'true')
    unknown_initial_pose = (
        LaunchConfiguration('unknown_initial_pose').perform(context).lower()
        == 'true')
    assignment_strategy = LaunchConfiguration('assignment_strategy').perform(context)
    visualization_overlay_nodes = []
    if (LaunchConfiguration('launch_visualization_overlay').perform(context)
            .lower() == 'true'):
        visualization_overlay_nodes.append(Node(
            package='my_epuck_project',
            executable='rviz_map_overlay',
            name='rviz_map_overlay',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'viz_world_frame': 'viz/world',
                'robot1_map_topic': '/robot1/map',
                'robot2_map_topic': '/robot2/map',
                'robot1_output_topic': '/viz/robot1_map',
                'robot2_output_topic': '/viz/robot2_map',
                'robot1_alias_frame': 'viz/robot1_map',
                'robot2_alias_frame': 'viz/robot2_map',
                'robot1_offset_x_m': LaunchConfiguration(
                    'visualization_robot1_offset_x_m'),
                'robot1_offset_y_m': LaunchConfiguration(
                    'visualization_robot1_offset_y_m'),
                'robot1_offset_z_m': LaunchConfiguration(
                    'visualization_robot1_offset_z_m'),
                'robot1_offset_yaw_rad': LaunchConfiguration(
                    'visualization_robot1_offset_yaw_rad'),
                'robot2_offset_x_m': LaunchConfiguration(
                    'visualization_robot2_offset_x_m'),
                'robot2_offset_y_m': LaunchConfiguration(
                    'visualization_robot2_offset_y_m'),
                'robot2_offset_z_m': LaunchConfiguration(
                    'visualization_robot2_offset_z_m'),
                'robot2_offset_yaw_rad': LaunchConfiguration(
                    'visualization_robot2_offset_yaw_rad'),
            }],
        ))
    profile_scan_parameters = dict(
        selected.get('slam_runtime_parameters', {}))
    # IncludeLaunchDescription arguments are textual launch substitutions, but
    # the activation controller is an rclpy node whose declared parameters
    # must retain their native types.  Keep these paths separate so a profile
    # bool is not serialized as the string ``"true"`` for that node.
    activation_scan_matching = profile_scan_parameters.get(
        'use_scan_matching', False)
    activation_loop_closing = profile_scan_parameters.get(
        'do_loop_closing', False)
    effective_scan_matching = (
        'true' if profile_scan_parameters.get('use_scan_matching') is True
        else LaunchConfiguration('use_scan_matching'))
    effective_loop_closing = (
        'false' if 'do_loop_closing' in profile_scan_parameters
        else LaunchConfiguration('do_loop_closing'))
    launch_world_path = world_path or selected['world_path']
    integrated_thin_capture = os.environ.get(
        'MY_EPUCK_THIN_INTEGRATED_GT', '').lower() in ('1', 'true', 'yes')
    if (forensic_enabled or contact_enabled) and not integrated_thin_capture:
        # Keep the source/production world untouched.  Webots requires every
        # external controller to have a corresponding Robot node, so the
        # read-only Supervisor gets one temporary diagnostic-only node.
        source_for_copy = launch_world_path
        forensic_world_dir = tempfile.mkdtemp(
            prefix=f'my_epuck_forensic_{os.getpid()}_')
        # Preserve the source world's ``worlds/`` level and its project-local
        # ``protos/`` sibling.  The production worlds use relative imports
        # such as ``../protos/e-puck/E-puck.proto``; copying only the WBT to a
        # flat temporary directory makes Webots reject those imports before
        # any robot controller can connect.
        forensic_worlds_dir = os.path.join(forensic_world_dir, 'worlds')
        os.makedirs(forensic_worlds_dir, exist_ok=True)
        source_package_dir = os.path.dirname(
            os.path.dirname(os.path.abspath(source_for_copy)))
        source_protos_dir = os.path.join(source_package_dir, 'protos')
        forensic_protos_dir = os.path.join(forensic_world_dir, 'protos')
        if os.path.isdir(source_protos_dir):
            shutil.copytree(source_protos_dir, forensic_protos_dir)
        forensic_world = os.path.join(
            forensic_worlds_dir, os.path.basename(source_for_copy))
        shutil.copyfile(source_for_copy, forensic_world)
        staged_source_world_sha256 = hashlib.sha256(
            Path(forensic_world).read_bytes()).hexdigest()
        with open(forensic_world, 'a', encoding='utf-8') as stream:
            stream.write(
                '\nRobot {\n'
                '  name "ForensicGroundTruthSupervisor"\n'
                '  controller "<extern>"\n'
                '  supervisor TRUE\n'
                '}\n')
        staged_world_sha256 = hashlib.sha256(
            Path(forensic_world).read_bytes()).hexdigest()
        launch_world_path = forensic_world
    dispatch_enabled = (
        LaunchConfiguration('dispatch_enabled').perform(context).lower()
        == 'true'
    )
    synchronized_traffic_test = (
        LaunchConfiguration('synchronized_traffic_test').perform(context).lower()
        == 'true'
    )
    synchronized_traffic_hold_prehandoff_motion = (
        LaunchConfiguration('synchronized_traffic_hold_prehandoff_motion')
        .perform(context).lower() == 'true'
    )
    # The known-pose launch keeps the existing shared assignment include.  In
    # unknown-pose mode it is deliberately omitted from the returned action
    # graph; a one-shot activation node launches it after acceptance.
    shared_dispatch_enabled = False if unknown_initial_pose else dispatch_enabled
    assignment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_dir, 'launch', 'two_robots_distributed_assignment_launch.py',
        )),
        launch_arguments={
            'world_profile': selected['name'],
            'webots_port': LaunchConfiguration('webots_port'),
            'world_path': launch_world_path,
            'webots_mode': LaunchConfiguration('webots_mode'),
            'webots_gui': LaunchConfiguration('webots_gui'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_scan_matching': effective_scan_matching,
            'do_loop_closing': effective_loop_closing,
            'slam_tf_publish_probe_library': LaunchConfiguration(
                'slam_tf_publish_probe_library'),
            'slam_tf_publish_probe_log': LaunchConfiguration(
                'slam_tf_publish_probe_log'),
            'slam_tf_publication_mode': LaunchConfiguration(
                'slam_tf_publication_mode'),
            'diagnostic_mode': LaunchConfiguration('diagnostic_mode'),
            'diagnostic_frontier_capture': LaunchConfiguration(
                'diagnostic_frontier_capture'),
            'fusion_process_nice': LaunchConfiguration(
                'fusion_process_nice'),
            'sensor_profile': LaunchConfiguration('sensor_profile'),
            'scan_input_reliability': LaunchConfiguration(
                'scan_input_reliability'),
            'nav2_autostart': LaunchConfiguration('nav2_autostart'),
            'dispatch_enabled': str(shared_dispatch_enabled).lower(),
            'controller_variant': LaunchConfiguration('controller_variant'),
            'assignment_strategy': LaunchConfiguration('assignment_strategy'),
            'burgard_beta': LaunchConfiguration('burgard_beta'),
            'traffic_scheduler_enabled': LaunchConfiguration(
                'traffic_scheduler_enabled'),
            'synchronized_traffic_test': LaunchConfiguration(
                'synchronized_traffic_test'),
            'traffic_test_force_conflict_pair': LaunchConfiguration(
                'traffic_test_force_conflict_pair'),
            'enable_mission_timeout': LaunchConfiguration(
                'enable_mission_timeout'),
            'mission_timeout_s': LaunchConfiguration('mission_timeout_s'),
            'terminal_small_frontier_length_m': LaunchConfiguration(
                'terminal_small_frontier_length_m'),
            'unknown_initial_pose': LaunchConfiguration(
                'unknown_initial_pose'),
        }.items(),
    )
    unknown_local_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_dir, 'launch',
            'two_robots_teammate_filtered_stack_launch.py',
        )),
        launch_arguments={
            'world_profile': selected['name'],
            'webots_port': LaunchConfiguration('webots_port'),
            'world_path': launch_world_path,
            'webots_mode': LaunchConfiguration('webots_mode'),
            'webots_gui': LaunchConfiguration('webots_gui'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_scan_matching': effective_scan_matching,
            'do_loop_closing': effective_loop_closing,
            'slam_tf_publish_probe_library': LaunchConfiguration(
                'slam_tf_publish_probe_library'),
            'slam_tf_publish_probe_log': LaunchConfiguration(
                'slam_tf_publish_probe_log'),
            'slam_tf_publication_mode': LaunchConfiguration(
                'slam_tf_publication_mode'),
            'sensor_profile': LaunchConfiguration('sensor_profile'),
            'scan_input_reliability': LaunchConfiguration(
                'scan_input_reliability'),
            'diagnostic_mode': LaunchConfiguration('diagnostic_mode'),
            'fusion_cpu_quota_percent': LaunchConfiguration(
                'fusion_cpu_quota_percent'),
            'fusion_process_nice': LaunchConfiguration('fusion_process_nice'),
            'fusion_rebuild_period_s': LaunchConfiguration(
                'fusion_rebuild_period_s'),
            'nav2_autostart': LaunchConfiguration('nav2_autostart'),
            'controller_variant': LaunchConfiguration('controller_variant'),
            'unknown_initial_pose': 'true',
            'launch_mapping': 'true',
            # Keep shared Nav2 phase-gated and inactive, but start its
            # processes early so startup/DDS/service discovery overlaps the
            # unknown-pose and cleanup work.
            'launch_shared_stack': LaunchConfiguration(
                'prelaunch_shared_nav2'),
            'launch_shared_fusion': 'true',
            'prelaunch_shared_nav2': LaunchConfiguration(
                'prelaunch_shared_nav2'),
            'phase_already_aligned': 'false',
        }.items(),
    )
    unknown_pose_frontends = []
    local_phase_nodes = []
    motion_fixture_nodes = []
    frontend_watchdogs = []
    shared_activation = None
    if unknown_initial_pose:
        diagnostic_output = LaunchConfiguration(
            'unknown_pose_diagnostic_output').perform(context)
        registration_capture_output = LaunchConfiguration(
            'unknown_pose_registration_capture_output').perform(context)
        for robot, peer in (('robot1', 'robot2'), ('robot2', 'robot1')):
            profile_prefix = os.environ.get(
                'MY_EPUCK_UNKNOWN_POSE_CPROFILE_PREFIX', '')
            if profile_prefix:
                profile_prefix = profile_prefix.format(robot=robot)
            unknown_pose_frontends.append(Node(
                package='my_epuck_project',
                executable='unknown_pose_frontend',
                name='unknown_pose_frontend',
                namespace=robot,
                output='screen',
                prefix=profile_prefix,
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'robot_id': robot,
                    'peer_robot_id': peer,
                    'map_topic': f'/{robot}/map',
                    'peer_map_topic': (
                        f'/cslam/unknown_pose/{robot}/local_map'),
                    'peer_map_publish_period_s': LaunchConfiguration(
                        'peer_map_publish_period_s'),
                    'shared_frame': 'shared_map',
                    'diagnostic_output': diagnostic_output,
                    'registration_input_capture_output':
                        registration_capture_output,
                    'registration_input_capture_max_pairs': 64,
                    'full_map_registration': LaunchConfiguration(
                        'full_map_registration'),
                    # Latest useful full-map pair cadence for startup-only
                    # discovery; the matcher remains serialized per robot.
                    'full_map_registration_period_s': 1.0,
                    # Keep a bounded exact-ID history long enough for a
                    # startup proposal's confirmation snapshots to remain
                    # recoverable while DDS delivery and peer verification
                    # complete. This is transport/cache capacity only; it
                    # does not change map registration or acceptance rules.
                    'full_map_max_snapshots': 64,
                    # Keep the production MRPT path explicitly pinned to the
                    # configuration validated offline.  These are estimator
                    # parameters, not GT or navigation inputs.
                    # This close-start validation uses the existing custom
                    # full-map matcher.  MRPT remains an explicit optional
                    # backend for prior experiments, but is not part of this
                    # production path.
                    'registration_backend': 'legacy',
                    'max_verification_batches': LaunchConfiguration(
                        'max_verification_batches'),
                    'verification_lifetime_s': LaunchConfiguration(
                        'verification_lifetime_s'),
                    'evidence_keyframe_translation_threshold_m':
                        LaunchConfiguration(
                            'evidence_keyframe_translation_threshold_m'),
                }],
            ))
            frontend_watchdogs.append(RegisterEventHandler(
                OnProcessExit(
                    target_action=unknown_pose_frontends[-1],
                    on_exit=_frontend_exit_handler(
                        robot, os.path.join(
                            diagnostic_output,
                            f'{robot}_accepted_handoff.marker')),
                )))
            local_phase_nodes.extend([
                Node(
                    package='my_epuck_frontier_candidates',
                    executable='frontier_candidate_generator',
                    name='local_frontier_candidate_generator',
                    namespace=robot,
                    output='screen',
                    remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
                    parameters=[{
                        'robot_id': robot,
                        'map_topic': f'/{robot}/map',
                        'global_costmap_topic': f'/{robot}/global_costmap/costmap',
                        'global_frame': f'{robot}/map',
                        'robot_base_frame': f'{robot}/base_footprint',
                        'compute_path_action': f'/{robot}/compute_path_to_pose',
                        'candidate_topic': f'/{robot}/frontier_candidates',
                        'marker_topic': f'/{robot}/local_frontier_candidate_markers',
                        'processing_rate_hz': 0.5,
                        # Use the same private decision-map configuration as
                        # the shared/post-handoff generator.  Keeping this
                        # mapping centralized prevents the pre-handoff path
                        # from silently falling back to the C++ default.
                        **FRONTIER_DECISION_MAP_PARAMETERS,
                        'minimum_frontier_cells': selected['minimum_frontier_cells'],
                        'minimum_frontier_length_m': 0.05,
                        'stable_id_quantization_m': 0.05,
                        'approach_clearance_m': 0.15,
                        'minimum_robot_distance_m': 0.08,
                        'maximum_candidates_before_path_check': 10000,
                        'maximum_path_queries_per_cycle': 10000,
                        'path_query_timeout_s': 1.0,
                        # Mirrors nav2_robot{1,2}_shared_map.yaml RPP motion
                        # references; not controller tuning or an ETA model.
                        'cost_only_reference_linear_speed_mps': 0.13,
                        'cost_only_reference_angular_speed_radps': 0.35,
                        'planner_id': 'GridBased',
                        'occupied_threshold': 50,
                        'visible_gain_range_m': 11.98,
                        'selection_policy': assignment_strategy,
                        'event_driven_costing': True,
                        'stop_after_handoff': True,
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                    }],
                ),
                Node(
                    package='my_epuck_project',
                    executable='frontier_proposal_adapter',
                    name='local_frontier_proposal_adapter',
                    namespace=robot,
                    output='screen',
                    parameters=[{
                        'robot_id': robot,
                        'candidate_topic': f'/{robot}/frontier_candidates',
                        'task_snapshot_topic': f'/{robot}/local_task_snapshot',
                        'maximum_tasks': 10000,
                        'validity_s': 8.0,
                        'stop_after_handoff': True,
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                    }],
                ),
                Node(
                    package='my_epuck_project',
                    executable='minimal_frontier_allocator',
                    name='minimal_frontier_allocator',
                    namespace=robot,
                    output='screen',
                    prefix=os.environ.get(
                        'MY_EPUCK_LOCAL_ASSIGNMENT_CPROFILE_PREFIX',
                        '').format(robot=robot),
                    remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
                    parameters=[{
                        'robot_id': robot,
                        'robot_base_frame': f'{robot}/base_footprint',
                        'global_frame': 'shared_map',
                        'map_topic': f'/{robot}/shared_map',
                        'nav2_node_prefix': '',
                        'candidate_topic': 'frontier_candidates',
                        'common_start_release_required': LaunchConfiguration(
                            'common_start_release_required'),
                        'publish_cooperative_start_ready': True,
                        'local_path_gate_mode': LaunchConfiguration(
                            'local_path_gate_mode'),
                        'map_stability_grace_s': 5.0,
                        'bid_validity_s': 8.0,
                        'traffic_safe_radius_m': 0.08,
                        'traffic_reference_speed_mps': 0.13,
                        'traffic_eta_tie_s': 0.05,
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                    }],
                ),
                Node(
                    package='my_epuck_project',
                    executable='unknown_pose_phase_manager',
                    name='unknown_pose_phase_manager',
                    namespace=robot,
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'robot_id': robot,
                        'shared_frame': 'shared_map',
                        'local_manager_service':
                            f'/{robot}/local_lifecycle_manager_navigation/manage_nodes',
                        'shared_manager_service':
                            f'/{robot}/lifecycle_manager_navigation/manage_nodes',
                        'handoff_marker_path': os.path.join(
                            diagnostic_output,
                            f'{robot}_accepted_handoff.marker'),
                'historical_cleanup_required': True,
                    }],
                ),
            ])
        if (LaunchConfiguration('enable_motion_fixture').perform(context)
                .lower() == 'true'):
            motion_fixture_nodes.append(Node(
                package='my_epuck_project',
                executable='unknown_pose_motion_fixture',
                name='unknown_pose_motion_fixture', output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'start_delay_s': LaunchConfiguration(
                        'motion_fixture_start_delay_s'),
                    'turn_duration_s': LaunchConfiguration(
                        'motion_fixture_turn_duration_s'),
                    'drive_duration_s': LaunchConfiguration(
                        'motion_fixture_drive_duration_s'),
                    'cycles': LaunchConfiguration('motion_fixture_cycles'),
                    'mirror_turns': LaunchConfiguration(
                        'motion_fixture_mirror_turns'),
                    'robot2_static': LaunchConfiguration(
                        'motion_fixture_robot2_static'),
                    'robot2_static_after_first_cycle': LaunchConfiguration(
                        'motion_fixture_robot2_static_after_first_cycle'),
                    'linear_speed': LaunchConfiguration(
                        'motion_fixture_linear_speed'),
                    'robot2_linear_scale': LaunchConfiguration(
                        'motion_fixture_robot2_linear_scale'),
                    'angular_speed': LaunchConfiguration(
                        'motion_fixture_angular_speed'),
                    'synchronized_traffic_test': LaunchConfiguration(
                        'synchronized_traffic_test'),
                    'hold_prehandoff_motion': LaunchConfiguration(
                        'synchronized_traffic_hold_prehandoff_motion'),
                }],
            ))
        shared_activation = Node(
            package='my_epuck_project',
            executable='unknown_pose_shared_stack_activation',
            name='unknown_pose_shared_stack_activation',
            output='screen',
            parameters=[{
                'world_profile': selected['name'],
                'world_path': launch_world_path,
                'webots_mode': LaunchConfiguration('webots_mode'),
                'webots_gui': LaunchConfiguration('webots_gui'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_scan_matching': activation_scan_matching,
                'do_loop_closing': activation_loop_closing,
                'sensor_profile': LaunchConfiguration('sensor_profile'),
                'scan_input_reliability': LaunchConfiguration(
                    'scan_input_reliability'),
                'diagnostic_mode': LaunchConfiguration('diagnostic_mode'),
                'diagnostic_frontier_capture': LaunchConfiguration(
                    'diagnostic_frontier_capture'),
                'fusion_process_nice': LaunchConfiguration(
                    'fusion_process_nice'),
                'fusion_cpu_quota_percent': LaunchConfiguration(
                    'fusion_cpu_quota_percent'),
                'fusion_rebuild_period_s': LaunchConfiguration(
                    'fusion_rebuild_period_s'),
                'controller_variant': LaunchConfiguration('controller_variant'),
                'assignment_strategy': LaunchConfiguration('assignment_strategy'),
                'event_driven_costing': True,
                'local_path_gate_mode': LaunchConfiguration(
                    'local_path_gate_mode'),
                'common_start_release_required': LaunchConfiguration(
                    'common_start_release_required'),
                'burgard_beta': LaunchConfiguration('burgard_beta'),
                'traffic_scheduler_enabled': LaunchConfiguration(
                    'traffic_scheduler_enabled'),
                'synchronized_traffic_test': LaunchConfiguration(
                    'synchronized_traffic_test'),
                'traffic_test_force_conflict_pair': LaunchConfiguration(
                    'traffic_test_force_conflict_pair'),
                'prelaunch_shared_nav2': LaunchConfiguration(
                    'prelaunch_shared_nav2'),
                'enable_mission_timeout': LaunchConfiguration(
                    'enable_mission_timeout'),
                'mission_timeout_s': LaunchConfiguration('mission_timeout_s'),
                'terminal_small_frontier_length_m': LaunchConfiguration(
                    'terminal_small_frontier_length_m'),
                'slam_tf_publish_probe_library': LaunchConfiguration(
                    'slam_tf_publish_probe_library'),
                'slam_tf_publish_probe_log': LaunchConfiguration(
                    'slam_tf_publish_probe_log'),
                'slam_tf_publication_mode': LaunchConfiguration(
                    'slam_tf_publication_mode'),
                'webots_port': LaunchConfiguration('webots_port'),
            }],
        )
    traffic_test_barrier = Node(
        package='my_epuck_project',
        executable='traffic_test_barrier',
        name='traffic_test_dispatch_barrier',
        output='screen',
        condition=IfCondition(LaunchConfiguration('synchronized_traffic_test')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    observer = Node(
        package='my_epuck_project', executable='cooperative_experiment_logger',
        name='cooperative_experiment_logger', output='screen',
        # The observer closes lossless forensic JSONL streams before the
        # launch process exits.  The default launch child timeout is too short
        # for a bounded campaign's final evidence flush.
        sigterm_timeout='120.0',
        sigkill_timeout='30.0',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('enable_observer'),
            "' == 'true' and '",
            LaunchConfiguration('observer_architecture'), "' == 'legacy'",
        ])),
        parameters=[{
            'run_id': LaunchConfiguration('run_id'),
            'output_root': LaunchConfiguration('output_root'),
            'launch_file': 'two_robots_decentralized_exploration_launch.py',
            'experiment_condition': LaunchConfiguration('experiment_condition'),
            'seed_provenance_json': ParameterValue(
                LaunchConfiguration('seed_provenance_json'), value_type=str),
            'world_profile': selected['name'],
            'source_world_path': selected['world_path'],
            'installed_world_path': selected['world_path'],
            'world_dimensions': list(selected['world_metadata']['dimensions']),
            'robot_start_poses_json': json.dumps(
                summary['robot_start_poses'], sort_keys=True,
            ),
            'known_relative_transform': list(
                selected['world_metadata']['relative_transform'],
            ),
            'transform_source': 'WORLD_DERIVED',
            'slam_resolution': selected['slam_resolution'],
            'fusion_resolution': selected['fusion_resolution'],
            'global_costmap_resolution': selected['global_costmap_resolution'],
            'local_costmap_resolution': selected['local_costmap_resolution'],
            'lidar_maximum_range': summary['lidar_maximum_range'],
            'world_sha256': selected['world_metadata']['sha256'],
            'coverage_attribution_resolution':
                selected['coverage_attribution_resolution'],
            'terminal_small_frontier_length_m': LaunchConfiguration(
                'terminal_small_frontier_length_m'),
            'initial_configuration_json': json.dumps({
                'frontier_engine': 'frontier_exploration_ros2 public core',
                'maximum_tasks_per_source': 10000,
                'maximum_union_tasks': 10000,
                'maximum_path_queries': 10000,
                'terminal_small_frontier_length_m': float(
                    LaunchConfiguration('terminal_small_frontier_length_m')
                    .perform(context)),
                'dispatch_enabled': dispatch_enabled,
                'assignment_strategy': LaunchConfiguration(
                    'assignment_strategy').perform(context),
                'local_path_gate_mode': LaunchConfiguration(
                    'local_path_gate_mode').perform(context),
                'burgard_beta': float(LaunchConfiguration(
                    'burgard_beta').perform(context)),
                'use_scan_matching': profile_scan_parameters.get(
                    'use_scan_matching',
                    LaunchConfiguration('use_scan_matching').perform(
                        context).lower() == 'true'),
                'do_loop_closing': profile_scan_parameters.get(
                    'do_loop_closing',
                    LaunchConfiguration('do_loop_closing').perform(
                    context).lower() == 'true'),
                # Preserve the complete effective SLAM override in the
                # passive manifest.  This is also the source of truth for the
                # scan-pipeline diagnostic; both namespaced Slam Toolbox
                # instances receive this same dictionary below.
                'slam_runtime_parameters': profile_scan_parameters,
                'ideal_encoder_sensing': LaunchConfiguration(
                    'ideal_encoder_sensing').perform(context).lower() == 'true',
                'encoder_profile': selected['encoder_profile'],
                'traffic_scheduler_enabled': LaunchConfiguration(
                    'traffic_scheduler_enabled').perform(context).lower()
                    == 'true',
                'map_fusion_resolution': selected['fusion_resolution'],
                'unknown_initial_pose': unknown_initial_pose,
            }, sort_keys=True),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'enable_console_status': LaunchConfiguration('logger_console_status'),
            'enable_rosout_collection': LaunchConfiguration(
                'enable_rosout_collection'),
            'enable_coverage_attribution': LaunchConfiguration(
                'enable_coverage_attribution'),
            'enable_trajectory_overlap': LaunchConfiguration(
                'enable_trajectory_overlap'),
            # Diagnostic-only, passive forensic capture.  It is disabled for
            # the normal production launch and does not alter the control
            # graph, motion limits, SLAM, fusion, or allocation.
            'enable_forensic_capture': LaunchConfiguration(
                'enable_forensic_capture'),
            'diagnostic_frontier_capture': LaunchConfiguration(
                'diagnostic_frontier_capture'),
            'enable_contact_capture': LaunchConfiguration(
                'enable_contact_capture'),
            'enable_passive_rosbag': LaunchConfiguration(
                'enable_passive_rosbag'),
            'enable_scientific_raw_capture': LaunchConfiguration(
                'enable_scientific_raw_capture'),
            'contact_sampling_period_ms': LaunchConfiguration(
                'contact_sampling_period_ms'),
            'forensic_snapshot_interval_s': LaunchConfiguration(
                'forensic_snapshot_interval_s'),
            'forensic_ground_truth_sample_period_s': LaunchConfiguration(
                'forensic_ground_truth_sample_period_s'),
            'webots_port': LaunchConfiguration('webots_port'),
        }],
    )
    profile_log = LogInfo(msg=(
        'WORLD_PROFILE_SELECTED '
        f'profile={selected["name"]} '
        f'source_world={selected["world_path"]} '
        f'launch_world={launch_world_path} '
        f'source_sha256={source_world_sha256} '
        f'staged_source_sha256={staged_source_world_sha256} '
        f'staged_sha256={staged_world_sha256}'))
    if unknown_initial_pose:
        return [profile_log, unknown_local_mapping, *unknown_pose_frontends,
                *local_phase_nodes, *motion_fixture_nodes,
                *visualization_overlay_nodes,
                traffic_test_barrier, shared_activation, *frontend_watchdogs,
                observer]
    return [profile_log, assignment, *visualization_overlay_nodes, observer]


def generate_launch_description():
    """Launch final control graph and its removable passive evaluator."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_profile', default_value='large',
            choices=['large', 'small', 'large_unknown_pose',
                     'large_unknown_pose_16m',
                     'large_unknown_pose_close_start',
                     'large_unknown_pose_close_start_20ms',
                     'large_unknown_pose_close_start_20ms_scan_matching',
                     'large_unknown_pose_far_start_20ms_scan_matching']),
        DeclareLaunchArgument('webots_port', default_value='23000'),
        DeclareLaunchArgument('world_path', default_value=''),
        DeclareLaunchArgument('webots_mode', default_value='realtime'),
        DeclareLaunchArgument('webots_gui', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Production YAML remains false/false.  These are explicit diagnostic
        # runtime overrides for controlled Slam Toolbox experiments.
        DeclareLaunchArgument('use_scan_matching', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('do_loop_closing', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('slam_tf_publish_probe_library', default_value=''),
        DeclareLaunchArgument('slam_tf_publish_probe_log', default_value=''),
        DeclareLaunchArgument('slam_tf_publication_mode', default_value='',
                              choices=['', 'SYNCHRONOUS', 'ASYNCHRONOUS']),
        # The active Webots path already uses un-noised PositionSensor
        # measurements.  This explicit flag records that thesis-simulation
        # assumption; it never substitutes Supervisor pose for /odom.
        DeclareLaunchArgument('ideal_encoder_sensing', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('diagnostic_mode', default_value='false'),
        DeclareLaunchArgument('diagnostic_frontier_capture', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('fusion_process_nice', default_value='0'),
        DeclareLaunchArgument('fusion_cpu_quota_percent', default_value='30'),
        DeclareLaunchArgument('fusion_rebuild_period_s', default_value='1.0'),
        DeclareLaunchArgument('peer_map_publish_period_s', default_value='5.0'),
        DeclareLaunchArgument('sensor_profile', default_value='full',
                              choices=['full', 'throughput']),
        DeclareLaunchArgument('scan_input_reliability', default_value='reliable',
                              choices=['reliable', 'best_effort']),
        DeclareLaunchArgument('nav2_autostart', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('dispatch_enabled', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('common_start_release_required',
                              default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('prehandoff_dispatch_delay_s',
                              default_value='0.0'),
        DeclareLaunchArgument('unknown_initial_pose', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('unknown_pose_diagnostic_output',
                              default_value=''),
        DeclareLaunchArgument('unknown_pose_registration_capture_output',
                              default_value=''),
        DeclareLaunchArgument('full_map_registration', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument(
            # Keep the resource-heavy inactive shared Nav2 graph opt-in.  The
            # normal unknown-pose profile starts it at accepted handoff; an
            # earlier prelaunch experiment could starve Webots before /clock
            # became available on a loaded host.
            'prelaunch_shared_nav2', default_value='false',
            choices=['true', 'false']),
        # Validation-only motion for unknown-pose evidence acquisition.  The
        # default is disabled so production launches remain unchanged.
        DeclareLaunchArgument('enable_motion_fixture', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('motion_fixture_start_delay_s',
                              default_value='20.0'),
        DeclareLaunchArgument('motion_fixture_turn_duration_s',
                              default_value='3.2'),
        DeclareLaunchArgument('motion_fixture_drive_duration_s',
                              default_value='12.0'),
        DeclareLaunchArgument('motion_fixture_cycles', default_value='1'),
        DeclareLaunchArgument('motion_fixture_mirror_turns',
                              default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('motion_fixture_robot2_static',
                              default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('motion_fixture_robot2_static_after_first_cycle',
                              default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('motion_fixture_linear_speed',
                              default_value='0.10'),
        DeclareLaunchArgument('motion_fixture_robot2_linear_scale',
                              default_value='1.0'),
        DeclareLaunchArgument('motion_fixture_angular_speed',
                              default_value='0.45'),
        DeclareLaunchArgument('max_verification_batches', default_value='64'),
        DeclareLaunchArgument('verification_lifetime_s',
                              default_value='1100.0'),
        DeclareLaunchArgument(
            'evidence_keyframe_translation_threshold_m',
            default_value='0.80'),
        DeclareLaunchArgument(
            'assignment_strategy', default_value='frontier_mrtsp',
            choices=['frontier_cost_only', 'frontier_mrtsp']),
        DeclareLaunchArgument(
            'local_path_gate_mode', default_value='MODE_A',
            choices=['MODE_A', 'MODE_B']),
        DeclareLaunchArgument('burgard_beta', default_value='1.0'),
        DeclareLaunchArgument('traffic_scheduler_enabled', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('synchronized_traffic_test', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument(
            'traffic_test_force_conflict_pair', default_value='false',
            choices=['true', 'false']),
        DeclareLaunchArgument(
            'synchronized_traffic_hold_prehandoff_motion',
            default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_observer', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('observer_architecture', default_value='legacy',
                              choices=['legacy', 'thin']),
        DeclareLaunchArgument('run_id', default_value=''),
        DeclareLaunchArgument('experiment_condition', default_value='C'),
        DeclareLaunchArgument('seed_provenance_json', default_value='{}'),
        DeclareLaunchArgument('output_root', default_value='/home/arash/webots_ws/results'),
        DeclareLaunchArgument('mission_timeout_s', default_value='600.0'),
        DeclareLaunchArgument('terminal_small_frontier_length_m',
                              default_value='0.20'),
        DeclareLaunchArgument('enable_mission_timeout', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('launch_rviz', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('launch_visualization_overlay',
                              default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('visualization_robot1_offset_x_m',
                              default_value='0.0'),
        DeclareLaunchArgument('visualization_robot1_offset_y_m',
                              default_value='0.0'),
        DeclareLaunchArgument('visualization_robot1_offset_z_m',
                              default_value='0.0'),
        DeclareLaunchArgument('visualization_robot1_offset_yaw_rad',
                              default_value='0.0'),
        DeclareLaunchArgument('visualization_robot2_offset_x_m',
                              default_value='10.0'),
        DeclareLaunchArgument('visualization_robot2_offset_y_m',
                              default_value='0.0'),
        DeclareLaunchArgument('visualization_robot2_offset_z_m',
                              default_value='0.0'),
        DeclareLaunchArgument('visualization_robot2_offset_yaw_rad',
                              default_value='0.0'),
        DeclareLaunchArgument('logger_console_status', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('enable_rosout_collection', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('enable_coverage_attribution', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('enable_trajectory_overlap', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('enable_forensic_capture', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('enable_contact_capture', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('enable_passive_rosbag', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument(
            'enable_scientific_raw_capture', default_value='false',
            choices=['true', 'false']),
        DeclareLaunchArgument('contact_sampling_period_ms', default_value='20'),
        DeclareLaunchArgument('controller_variant', default_value='rpp',
                              choices=['dwb', 'rotation_shim_dwb', 'rpp']),
        DeclareLaunchArgument('forensic_snapshot_interval_s', default_value='5.0'),
        # Preserve the historical 50 Hz GT evidence rate until thin-mode
        # scientific parity explicitly proves a lower rate equivalent.
        DeclareLaunchArgument(
            'forensic_ground_truth_sample_period_s', default_value='0.02'),
        OpaqueFunction(function=launch_setup),
    ])
