"""Regression coverage for the unknown-pose observer world override."""

from pathlib import Path


LAUNCH = Path(__file__).resolve().parents[1] / 'launch'


def test_unknown_pose_launch_passes_optional_world_path_to_namespaced_stack():
    source = (LAUNCH / 'two_robots_unknown_pose_mapping_launch.py').read_text(
        encoding='utf-8')
    assert "DeclareLaunchArgument('world_path', default_value='')" in source
    assert "'world_path': launch_world_path" in source


def test_unknown_pose_forensic_launch_stages_supervisor_and_map_export():
    source = (LAUNCH / 'two_robots_unknown_pose_mapping_launch.py').read_text(
        encoding='utf-8')
    assert 'stage_forensic_world' in source
    assert 'ForensicGroundTruthSupervisor' in source
    assert "DeclareLaunchArgument('enable_forensic_capture'" in source
    assert "DeclareLaunchArgument('staging_directory'" in source
    assert 'required forensic/maps export path is unavailable' in source


def test_unknown_pose_frontend_keeps_bounded_requestable_keyframe_history():
    source = (LAUNCH.parent / 'my_epuck_project' /
              'unknown_pose_frontend.py').read_text(
        encoding='utf-8')
    assert "declare_parameter('max_keyframes', 64)" in source
    assert 'does not make the history unbounded' in source


def test_ground_truth_observer_has_bounded_clean_finalization_mode():
    source = (LAUNCH.parent / 'my_epuck_project' /
              'cooperative_ground_truth_observer.py').read_text(
                  encoding='utf-8')
    assert "'--max-runtime-s'" in source
    assert 'FORENSIC_SUPERVISOR_COMPLETE' in source


def test_ground_truth_observer_requires_explicit_controller_url_and_runtime_dir():
    source = (LAUNCH.parent / 'my_epuck_project' /
              'cooperative_ground_truth_observer.py').read_text(
                  encoding='utf-8')
    logger = (LAUNCH.parent / 'my_epuck_project' /
              'cooperative_experiment_logger.py').read_text(
                  encoding='utf-8')
    assert "'--controller-url'" in source
    assert 'OBSERVER_CONTROLLER_URL_REQUIRED' in source
    assert "'--runtime-directory'" in source
    assert "'--controller-url', environment['WEBOTS_CONTROLLER_URL']" in logger
    assert "'--runtime-directory', str(forensic_dir / 'runtime')" in logger


def test_unknown_pose_frontend_persists_bounded_shutdown_diagnostics():
    source = (LAUNCH.parent / 'my_epuck_project' /
              'unknown_pose_frontend.py').read_text(encoding='utf-8')
    assert "'DESCRIPTOR_GATE_SURVIVED'" in source
    assert "'CANDIDATE_SELECTION'" in source
    assert "'CROP_REQUEST_PUBLISHED'" in source
    assert "'descriptor_gate_rejection_reason_counts'" in source
    assert "'callback_timing'" in source
    assert "'cpu_samples'" in source
    assert 'node.finalize()' in source


def test_unknown_pose_frontend_records_each_descriptor_gate_reason():
    source = (LAUNCH.parent / 'my_epuck_project' /
              'unknown_pose_frontend.py').read_text(encoding='utf-8')
    for reason in (
            'SIMILARITY_BELOW_GATE', 'MARGIN_BELOW_GATE',
            'KNOWN_FRACTION_BELOW_GATE', 'INSUFFICIENT_CONFIRMATIONS'):
        assert reason in source


def test_unknown_pose_frontend_rechecks_cached_temporal_support_from_timer():
    source = (LAUNCH.parent / 'my_epuck_project' /
              'unknown_pose_frontend.py').read_text(encoding='utf-8')
    assert 'self._compare_peer_descriptors()' in source
    assert 'new_peer_key=key' in source
    assert 'confirmation_window_for_cadence' in source


def test_full_exploration_launch_starts_both_unknown_pose_frontends():
    source = (LAUNCH / 'two_robots_decentralized_exploration_launch.py')
    text = source.read_text(encoding='utf-8')
    assert "DeclareLaunchArgument('unknown_initial_pose'" in text
    assert "executable='unknown_pose_frontend'" in text
    assert "for robot, peer in (('robot1', 'robot2'), ('robot2', 'robot1'))" in text
    assert "'robot_id': robot" in text
    assert "'peer_robot_id': peer" in text


def test_explicit_unknown_pose_exploration_entrypoint_selects_full_stack():
    source = (LAUNCH / 'two_robots_unknown_pose_exploration_launch.py')
    text = source.read_text(encoding='utf-8')
    assert 'two_robots_decentralized_exploration_launch.py' in text
    assert "'world_profile': LaunchConfiguration('world_profile')" in text
    assert "'large_unknown_pose_16m'" in text
    assert "'unknown_initial_pose': 'true'" in text
    assert "'ideal_encoder_sensing': 'true'" in text
    assert "'sensor_profile': 'full'" in text
    assert "'controller_variant': 'rpp'" in text
    assert "'enable_forensic_capture': LaunchConfiguration(" in text
    assert "'launch_rviz': 'false'" in text


def test_explicit_unknown_pose_entrypoint_keeps_runtime_ground_truth_out():
    source = (LAUNCH / 'two_robots_unknown_pose_exploration_launch.py')
    text = source.read_text(encoding='utf-8')
    assert 'ground_truth' not in text
    assert 'Supervisor' not in text
    assert 'initial_relative' not in text
    assert 'known_relative_transform' not in text


def test_unknown_pose_pre_handoff_gate_is_data_dependent_and_post_handoff_is_gated():
    stack = (LAUNCH / 'two_robots_teammate_filtered_stack_launch.py')
    stack_text = stack.read_text(encoding='utf-8')
    frontend = (LAUNCH.parent / 'my_epuck_project' /
                'unknown_pose_frontend.py').read_text(encoding='utf-8')
    fusion = (LAUNCH.parent / 'my_epuck_project' /
              'source_aware_map_fusion.py').read_text(encoding='utf-8')
    assert 'alignment = [] if unknown_initial_pose else' in stack_text
    assert "f'/cslam/unknown_pose/{peer}/local_map'" in stack_text
    assert 'if self.latest_map is None or self.accepted is None:' in frontend
    assert 'message.local_evidence_only = True' in frontend
    assert 'self.map_transform(message)' in fusion
    assert 'self.local_map is None or self.remote_map is None' in fusion


def test_unknown_pose_mode_gates_known_alignment_and_raw_map_export():
    stack = (LAUNCH / 'two_robots_teammate_filtered_stack_launch.py')
    text = stack.read_text(encoding='utf-8')
    slam = (LAUNCH / 'two_robots_teammate_filtered_dual_slam_launch.py')
    slam_text = slam.read_text(encoding='utf-8')
    assert 'alignment = [] if unknown_initial_pose else' in text
    assert 'if not unknown_initial_pose:' in text
    assert "f'/cslam/unknown_pose/{peer}/local_map'" in text
    assert "f'/{robot}/scan_d500_fixed'" in slam_text
    assert "if not unknown_initial_pose:" in slam_text
    assert "'--frame-id', f'{robot}/local_world'" in slam_text


def test_unknown_pose_frontend_handoff_is_the_only_peer_map_enablement():
    full = (LAUNCH / 'two_robots_decentralized_exploration_launch.py')
    stack = (LAUNCH / 'two_robots_teammate_filtered_stack_launch.py')
    full_text = full.read_text(encoding='utf-8')
    stack_text = stack.read_text(encoding='utf-8')
    assert "'peer_map_topic': (" in full_text
    assert "f'/cslam/unknown_pose/{robot}/local_map'" in full_text
    assert "f'/cslam/unknown_pose/{peer}/local_map'" in stack_text
    assert "'output_topic': 'shared_map'" in stack_text
    assert "'global_frame': 'shared_map'" in (
        LAUNCH / 'two_robots_frontier_candidates_launch.py').read_text(
            encoding='utf-8')


def test_full_unknown_pose_launch_keeps_observer_artifact_finalization():
    source = (LAUNCH / 'two_robots_decentralized_exploration_launch.py')
    text = source.read_text(encoding='utf-8')
    assert "executable='cooperative_experiment_logger'" in text
    assert "DeclareLaunchArgument('enable_forensic_capture'" in text
    assert "'enable_forensic_capture': LaunchConfiguration(" in text
    assert "'enable_trajectory_overlap': LaunchConfiguration(" in text
    assert "'unknown_pose_diagnostic_output'" in text


def test_far_start_campaign_fixture_uses_frozen_physics_and_two_robots():
    world = (LAUNCH.parent / 'worlds' /
             'epuck_d500_two_world_large_unknown_pose_dynamic_low_slip_4ms_finite.wbt')
    text = world.read_text(encoding='utf-8')
    assert text.count('name "robot1"') == 1
    assert text.count('name "robot2"') == 1
    assert 'translation 16 0 0.001' in text
    assert 'translation -18.74 0 0.001' in text
    assert 'basicTimeStep 4' in text
    assert 'coulombFriction 10' in text
    assert 'optimalThreadCount 1' in text
    assert 'randomSeed 20260818' in text
    assert text.count('noise 0') == 2
    assert text.count('resolution -1') == 2


def test_unknown_pose_runtime_has_no_supervisor_pose_input_or_peer_nav2():
    frontend = (LAUNCH.parent / 'my_epuck_project' /
                'unknown_pose_frontend.py').read_text(encoding='utf-8')
    full = (LAUNCH / 'two_robots_decentralized_exploration_launch.py').read_text(
        encoding='utf-8')
    assert 'Supervisor' not in frontend
    assert 'ground_truth' not in frontend
    assert 'NavigateToPose' not in frontend
    assert 'cmd_vel' not in frontend
    assert "'robot_id': robot" in full
