"""Static integration guards for the unknown-pose two-phase launch contract."""

from pathlib import Path
import importlib.util

import yaml


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / 'launch'
PY = ROOT / 'my_epuck_project'


def _load_nav_launch():
    source = LAUNCH / 'two_robots_teammate_filtered_stack_launch.py'
    spec = importlib.util.spec_from_file_location('teammate_nav_launch', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_local_phase_uses_each_robot_local_map_and_local_nav2():
    stack = (LAUNCH / 'two_robots_teammate_filtered_stack_launch.py').read_text()
    full = (LAUNCH / 'two_robots_decentralized_exploration_launch.py').read_text()
    assignment = (PY / 'distributed_frontier_assignment.py').read_text()
    assert "global_frame=f'{robot}/map'" in stack
    assert "map_topic=f'/{robot}/map'" in stack
    assert "'map_topic': f'/{robot}/map'" in full
    assert "'local_only': True" in full
    assert "if self._local_only:" in assignment


def test_shared_stack_is_inert_until_accepted_handoff():
    stack = (LAUNCH / 'two_robots_teammate_filtered_stack_launch.py').read_text()
    full = (LAUNCH / 'two_robots_decentralized_exploration_launch.py').read_text()
    assignment_launch = (LAUNCH / 'two_robots_distributed_assignment_launch.py').read_text()
    frontier_launch = (LAUNCH / 'two_robots_frontier_candidates_launch.py').read_text()
    assignment = (PY / 'distributed_frontier_assignment.py').read_text()
    fusion = (PY / 'source_aware_map_fusion.py').read_text()
    generator = (ROOT.parent / 'my_epuck_frontier_candidates' / 'src' /
                 'frontier_candidate_generator.cpp').read_text()
    phase = (PY / 'unknown_pose_phase_manager.py').read_text()
    assert "autostart=False" in stack
    assert "shared_dispatch_enabled = False if unknown_initial_pose" in full
    assert "'launch_shared_stack': 'false'" in full
    assert "'launch_mapping': 'true'" in full
    assert "'handoff_gated': handoff_gated" in stack
    assert "'stop_after_handoff': True" in full
    assert "'phase_gated': LaunchConfiguration('phase_gated')" in assignment_launch
    assert "'handoff_gated': LaunchConfiguration('handoff_gated')" in frontier_launch
    assert "if not self._phase_gated:" in assignment
    assert "self._tick_timer = None" in assignment
    assert "self._activate_shared_phase()" in assignment
    assert "FUSION_PHASE pre_handoff=true map_inputs=false timer=false" in fusion
    assert "if not self.phase_active:" in fusion
    assert "handoff_gated_" in generator
    assert "if (handoff_gated_ && !processing_active_)" in generator
    assert "stop_after_handoff_" in generator
    assert "pre_handoff_stopped=true" in generator
    assert "FRONTIER_PHASE post_handoff=true processing_active=true" in generator
    assert "ManageLifecycleNodes.Request.STARTUP" in phase
    assert "ManageLifecycleNodes.Request.SHUTDOWN" in phase
    assert "message.accepted" in phase


def test_handoff_teardown_does_not_escalate_zombie_children():
    phase = (PY / 'unknown_pose_phase_manager.py').read_text()
    assert "fields = open('/proc/%s/stat' % pid" in phase
    assert "fields[2] not in ('Z', 'X')" in phase
    assert "remaining = [pid for pid in pids if live(pid)]" in phase


def test_handoff_node_shutdown_paths_are_idempotent():
    for name in (
            'unknown_pose_frontend.py', 'frontier_proposal_adapter.py',
            'distributed_frontier_assignment.py',
            'unknown_pose_phase_manager.py',
            'unknown_pose_shared_stack_activation.py'):
        source = (PY / name).read_text()
        assert 'ExternalShutdownException' in source
        assert 'if rclpy.ok()' in source


def test_shared_inputs_are_created_once_after_handoff():
    assignment = (PY / 'distributed_frontier_assignment.py').read_text()
    fusion = (PY / 'source_aware_map_fusion.py').read_text()
    generator = (ROOT.parent / 'my_epuck_frontier_candidates' / 'src' /
                 'frontier_candidate_generator.cpp').read_text()
    assert assignment.count('def _activate_shared_phase') == 1
    assert assignment.count('self._activate_protocol_inputs()') == 2
    assert fusion.count('def _activate_fusion_phase') == 1
    assert fusion.count('self._activate_fusion_phase(') == 2
    assert generator.count('processing_active_ = true;') == 3


def test_shared_stack_is_started_only_by_one_shot_handoff_activation():
    full = (LAUNCH / 'two_robots_decentralized_exploration_launch.py').read_text()
    activation = (PY / 'unknown_pose_shared_stack_activation.py').read_text()
    assert "executable='unknown_pose_shared_stack_activation'" in full
    assert "two_robots_distributed_assignment_launch.py" in activation
    assert "'phase_already_aligned': 'true'" in activation
    assert "'launch_mapping': 'false'" in activation
    assert "'launch_shared_stack': 'true'" in activation
    assert 'start_new_session=True' in activation
    assert 'shell=True' not in activation


def test_shared_activation_omits_empty_optional_launch_arguments():
    activation = (PY / 'unknown_pose_shared_stack_activation.py').read_text()
    assert "if value != ''" in activation
    assert "f'{key}:={value}'" in activation


def test_shared_activation_declares_boolean_launch_parameters_with_boolean_types():
    activation = (PY / 'unknown_pose_shared_stack_activation.py').read_text()
    assert "('webots_gui', False)" in activation
    assert "('use_sim_time', True)" in activation
    assert "('use_scan_matching', False)" in activation
    assert "('do_loop_closing', False)" in activation
    assert "('nav2_autostart', False)" in activation
    assert "'nav2_autostart'" in activation
    assert "('fusion_process_nice', 0)" in activation
    assert "('fusion_cpu_quota_percent', 30)" in activation
    assert "('fusion_rebuild_period_s', 1.0)" in activation
    assert "('webots_port', 23000)" in activation
    assert "self._parameters[name] = str(bool(value)).lower()" in activation
    assert "Trying to set parameter 'webots_gui'" not in activation


def test_rejected_or_missing_handoff_keeps_shared_work_disabled():
    assignment = (PY / 'distributed_frontier_assignment.py').read_text()
    fusion = (PY / 'source_aware_map_fusion.py').read_text()
    generator = (ROOT.parent / 'my_epuck_frontier_candidates' / 'src' /
                 'frontier_candidate_generator.cpp').read_text()
    assert "if not bool(message.accepted) or str(message.status) != 'ACCEPTED'" in assignment
    assert "if not bool(message.accepted) or str(message.status) != 'ACCEPTED'" in fusion
    assert 'if (!message->accepted || message->status != "ACCEPTED"' in generator
    assert 'world_derived' not in (PY / 'unknown_pose_phase_manager.py').read_text()


def test_unknown_mode_does_not_include_shared_assignment_before_handoff():
    full = (LAUNCH / 'two_robots_decentralized_exploration_launch.py').read_text()
    assert "return [profile_log, unknown_local_mapping" in full
    assert "return [profile_log, assignment," in full
    assert "*visualization_overlay_nodes" in full
    assert "observer]" in full
    assert "'launch_shared_stack': 'false'" in full


def test_unknown_stack_requires_aligned_phase_before_shared_components():
    stack = (LAUNCH / 'two_robots_teammate_filtered_stack_launch.py').read_text()
    assert 'requested_shared_stack and (' in stack
    assert 'not unknown_initial_pose or phase_already_aligned' in stack


def test_frontend_diagnostic_collision_is_merged_before_write():
    frontend = (PY / 'unknown_pose_frontend.py').read_text()
    assert 'accepted_metadata = dict(request_metadata or {})' in frontend
    assert 'accepted_metadata.update({' in frontend
    accepted = frontend[
        frontend.index("'CROP_RESPONSE_ACCEPTED'"):
        frontend.index('result = self._verify_candidate_crop(')]
    assert '**accepted_metadata' in accepted
    assert '**request_metadata' not in accepted


def test_diagnostic_failures_are_counted_without_frontend_shutdown():
    frontend = (PY / 'unknown_pose_frontend.py').read_text()
    assert "'diagnostic_write_failures': 0" in frontend
    assert 'diagnostics must never kill estimation' in frontend
    assert 'UNKNOWN_POSE_DIAGNOSTIC_WRITE_FAILURE' in frontend


def test_candidate_diagnostics_have_compact_selection_representation():
    frontend = (PY / 'unknown_pose_frontend.py').read_text()
    assert 'def _candidate_diagnostic(self, candidate, status=None, reason=None,' in frontend
    assert "if compact:" in frontend
    assert "'own_keyframe_creation_timestamp_ns'" in frontend
    assert "status='PENDING', compact=True" in frontend
    assert 'def _candidate_diagnostic_reference(' in frontend
    assert 'self._diagnosed_physical_candidates' in frontend
    assert "candidate_pool=[self._candidate_diagnostic_reference(" in frontend


def test_frontend_exit_watchdog_shuts_down_on_unexpected_exit():
    full = (LAUNCH / 'two_robots_decentralized_exploration_launch.py').read_text()
    assert 'RegisterEventHandler' in full
    assert 'OnProcessExit' in full
    assert 'unknown-pose frontend exited with' in full
    assert 'accepted_handoff.marker' in full
    assert 'expected_handoff_teardown and event.returncode in (-9, 1)' in full


def test_expected_handoff_frontend_exit_code_one_is_not_a_campaign_failure():
    """ros2 wrappers may normalize expected signal teardown to exit code 1."""
    full = (LAUNCH / 'two_robots_decentralized_exploration_launch.py').read_text()
    assert 'without the marker it remains a' in full
    assert 'event.returncode in (-9, 1)' in full


def test_expected_handoff_teardown_is_marked_before_local_children_stop():
    phase = (PY / 'unknown_pose_phase_manager.py').read_text()
    assert "'handoff_marker_path'" in phase
    assert 'self._write_handoff_marker()' in phase
    assert 'os.replace(temporary, self._handoff_marker_path)' in phase


def test_phase_manager_relays_accepted_tf_after_frontend_teardown():
    phase = (PY / 'unknown_pose_phase_manager.py').read_text()
    full = (LAUNCH / 'two_robots_decentralized_exploration_launch.py').read_text()
    assert 'StaticTransformBroadcaster' in phase
    assert 'self._publish_accepted_tf(message)' in phase
    assert 'accepted_tf_relay=true' in phase
    assert 'self.peer_robot_id' in phase
    assert "'shared_frame': 'shared_map'" in full


def test_source_ack_can_finalize_after_proposal_keyframes_are_evicted():
    """An in-flight canonical proposal owns its immutable protocol envelope."""
    frontend = (PY / 'unknown_pose_frontend.py').read_text()
    assert 'self.pending_proposal_messages = {}' in frontend
    assert 'self.pending_proposal_messages[key] = copy.deepcopy(proposal)' in frontend
    assert "HYPOTHESIS_ACK_FINALIZED_FROM_RETAINED_PROPOSAL" in frontend
    assert 'proposal_message is not None' in frontend
    # The missing-keyframe path remains only as a defensive fallback for
    # legacy proposals, rather than rejecting every delayed ACK.
    ack_start = frontend.index('        proposal = self.pending_proposals.get(')
    ack_end = frontend.index('    def _ack_message(', ack_start)
    ack = frontend[ack_start:ack_end]
    assert ack.count('HYPOTHESIS_ACK_IGNORED_MISSING_KEYFRAME') == 1
    assert 'pending_proposal_messages' in ack


def test_peer_summary_verifies_when_crops_arrived_before_summary():
    frontend = (PY / 'unknown_pose_frontend.py').read_text()
    assert 'source_id in self.received_peer_crops' in frontend
    assert 'self.peer_summary_source_ids.clear()' in frontend
    assert 'self._verify_peer_hypothesis_summary()' in frontend


def test_peer_summary_verification_is_symmetric_and_canonical_publish_is_not():
    """Both peers verify exchanged evidence; only one publishes the proposal."""
    frontend = (PY / 'unknown_pose_frontend.py').read_text()
    verify_start = frontend.index('    def _verify_peer_hypothesis_summary(')
    verify_end = frontend.index('    def _publish_canonical_proposal(',
                                verify_start)
    verify = frontend[verify_start:verify_end]
    callback_start = frontend.index('    def hypothesis_callback(')
    callback_end = frontend.index('    def _ack_message(', callback_start)
    callback = frontend[callback_start:callback_end]
    assert "self.robot_id != min(" not in verify
    assert "message.keyframe_id in self.peer_summary_source_ids" in frontend
    assert "self._verified_peer_summary_hash" in frontend
    # Canonical publication remains deterministic and single-owner.
    publish_start = frontend.index('    def _publish_canonical_proposal(')
    publish_end = frontend.index('    def _publish_local_evidence_crops(',
                                 publish_start)
    publish = frontend[publish_start:publish_end]
    assert "self.robot_id != min(" in publish
    assert "self._publish_canonical_proposal(" in callback


def test_proposal_confirmation_runs_from_cached_evidence_without_waiting_for_crop():
    frontend = (PY / 'unknown_pose_frontend.py').read_text()
    proposed_start = frontend.index(
        "if message.status == 'PROPOSED' and self.robot_id == message.target_robot_id:")
    proposed_end = frontend.index("if message.status == 'REJECTED':", proposed_start)
    proposed = frontend[proposed_start:proposed_end]
    crop_start = frontend.index('    def crop_callback(self, message):')
    crop_end = frontend.index('    def _try_confirm_pending_proposal(', crop_start)
    crop = frontend[crop_start:crop_end]
    helper_start = frontend.index('    def _try_confirm_pending_proposal(')
    helper_end = frontend.index('    def _record_physical_worker_result(', helper_start)
    helper = frontend[helper_start:helper_end]
    assert 'self._request_source_for_confirmation(message)' in proposed
    assert 'self._try_confirm_pending_proposal(message)' in proposed
    assert 'self._try_confirm_pending_proposal(proposal, message)' in crop
    assert "'PROPOSAL_CONFIRMATION_WAITING'" in helper
    assert "'PROPOSAL_ACK_PUBLISHED'" in helper
    assert 'proposal.target_keyframe_id' in helper
    assert 'proposal.keyframe_id' not in helper


def test_single_constraint_evidence_is_relayed_for_peer_reverification():
    """Accepted evidence is exchanged without becoming a handoff."""
    frontend = (PY / 'unknown_pose_frontend.py').read_text()
    assert "status='EVIDENCE'" in frontend
    assert 'self._publish_evidence_announcement(' in frontend
    assert 'self._queue_peer_evidence_reverification' in frontend
    assert "if message.status == 'EVIDENCE':" in frontend
    # The relay is explicitly non-accepting; only the normal CANDIDATE /
    # PROPOSED / ACCEPTED path may install TF or activate fusion.
    relay_start = frontend.index('    def _publish_evidence_announcement(')
    relay_end = frontend.index('    def _queue_peer_evidence_reverification(',
                               relay_start)
    relay = frontend[relay_start:relay_end]
    assert 'accepted=False' in relay
    assert "message.constraint_count = 1" in relay


def test_phase_manager_hypothesis_qos_matches_frontend_publisher():
    frontend = (PY / 'unknown_pose_frontend.py').read_text()
    phase = (PY / 'unknown_pose_phase_manager.py').read_text()
    assignment = (PY / 'distributed_frontier_assignment.py').read_text()
    assert 'durability=DurabilityPolicy.VOLATILE' in frontend
    assert 'durability=DurabilityPolicy.VOLATILE' in phase
    assert 'hypothesis_qos' in assignment
    assert 'self._handoff_callback, hypothesis_qos' in assignment
    assert 'durability=DurabilityPolicy.TRANSIENT_LOCAL' not in phase


def test_local_and_shared_goal_owners_are_phase_exclusive():
    full = (LAUNCH / 'two_robots_decentralized_exploration_launch.py').read_text()
    assignment = (PY / 'distributed_frontier_assignment.py').read_text()
    assert "name='local_distributed_frontier_assignment'" in full
    assert "name='unknown_pose_phase_manager'" in full
    assert "self._dispatch_enabled = False if self._local_only else True" in assignment
    assert "self._nav2.cancel_navigation()" in assignment
    assert "'stop_after_handoff': True" in full
    assert "def _stop_local_phase" in assignment


def test_pre_handoff_frontend_outputs_stop_at_canonical_handoff():
    full = (LAUNCH / 'two_robots_decentralized_exploration_launch.py').read_text()
    adapter = (PY / 'frontier_proposal_adapter.py').read_text()
    assert full.count("'stop_after_handoff': True") >= 3
    assert "RelativePoseHypothesis" in adapter
    assert "_stopped_after_handoff" in adapter
    assert "if self._stopped_after_handoff" in adapter


def test_local_frontiers_and_adapter_are_not_peer_tasks():
    full = (LAUNCH / 'two_robots_decentralized_exploration_launch.py').read_text()
    assert "local_frontier_candidates" in full
    assert "local_task_snapshot" in full
    assert "'peer_robot_id'" not in full[full.find("local_distributed_frontier_assignment"):]


def test_frontend_is_the_only_runtime_alignment_input_and_no_ground_truth_path():
    frontend = (PY / 'unknown_pose_frontend.py').read_text()
    phase = (PY / 'unknown_pose_phase_manager.py').read_text()
    assert 'Supervisor' not in frontend
    assert 'ground_truth' not in frontend
    assert 'NavigateToPose' not in frontend
    assert 'cmd_vel' not in frontend
    assert 'RelativePoseHypothesis' in phase
    assert 'OccupancyGrid' not in phase


def test_each_local_assignment_uses_own_nav2_namespace():
    full = (LAUNCH / 'two_robots_decentralized_exploration_launch.py').read_text()
    local_nav = (PY / 'distributed_assignment' / 'local_nav2.py').read_text()
    assert "'nav2_node_prefix': 'local_'" in full
    assert "f'{self._nav2_node_prefix}{name}/get_state'" in local_nav
    assert "'compute_path_action': f'/{robot}/compute_path_to_pose'" in full


def test_provenance_is_not_hard_coded_to_original_dirty_checkout():
    logger = (PY / 'cooperative_experiment_logger.py').read_text()
    assert 'def runtime_worktree' in logger
    assert "'runtime_worktree'" in logger
    assert "'git_branch'" in logger
    assert "cwd='/home/arash/webots_ws'" not in logger


def test_far_start_world_and_frozen_slam_settings_remain_selected():
    world = (ROOT / 'worlds' /
             'epuck_d500_two_world_large_unknown_pose_dynamic_low_slip_4ms_finite.wbt').read_text()
    slam = (LAUNCH / 'two_robots_teammate_filtered_dual_slam_launch.py').read_text()
    assert 'translation 16 0 0.001' in world
    assert 'translation -18.74 0 0.001' in world
    assert 'use_scan_matching' in slam
    assert 'do_loop_closing' in slam


def test_feasible_unknown_pose_world_is_16m_and_retains_physics_profile():
    world = (ROOT / 'worlds' /
             'epuck_d500_two_world_unknown_pose_16m_dynamic_low_slip_4ms_finite.wbt').read_text()
    assert 'translation 16 0 0.001' in world
    assert 'translation -1 0 0.001' in world
    assert 'basicTimeStep 4' in world
    assert 'coulombFriction 10' in world
    assert 'optimalThreadCount 1' in world
    assert 'randomSeed 20260818' in world


def test_known_relative_pose_path_remains_separate():
    stack = (LAUNCH / 'two_robots_teammate_filtered_stack_launch.py').read_text()
    assert 'alignment = [] if unknown_initial_pose else' in stack
    assert 'if not unknown_initial_pose:' in stack
    assert "f'/cslam/unknown_pose/{peer}/local_map'" in stack


def test_prefixed_local_nav2_nodes_receive_frozen_rpp_parameters():
    module = _load_nav_launch()
    source = ROOT / 'resource' / 'nav2_robot1_shared_map.yaml'
    generated = module._diagnostic_params(
        str(source), 'robot1', 'rpp', node_prefix='local_')
    try:
        document = yaml.safe_load(Path(generated).read_text(encoding='utf-8'))
    finally:
        Path(generated).unlink(missing_ok=True)
    params = document['local_controller_server']['ros__parameters']
    assert params['FollowPath']['plugin'] == (
        'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController')
    assert 'controller_server' not in document


def test_unknown_pose_large_profiles_use_large_progress_checker_contract():
    module = _load_nav_launch()
    assert module._is_large_world_profile({
        'name': 'large_unknown_pose_close_start_20ms_scan_matching'})
    assert module._is_large_world_profile({
        'name': 'large_unknown_pose_far_start_20ms_scan_matching'})
    assert not module._is_large_world_profile({'name': 'small'})
    assert module._progress_movement_time_allowance({
        'name': 'large_unknown_pose_close_start_20ms_scan_matching'}) == '30.0'
    assert module._progress_movement_time_allowance({'name': 'large'}) == '18.0'
    assert module._progress_movement_time_allowance({'name': 'small'}) == '10.0'


def test_recovery_behaviors_use_namespaced_odom_frame():
    for robot in ('robot1', 'robot2'):
        params = (ROOT / 'resource' /
                  f'nav2_{robot}_shared_map.yaml').read_text(encoding='utf-8')
        assert f'local_frame: {robot}/odom' in params


def test_prefixed_local_nav2_rewrites_local_map_frame_and_topic():
    module = _load_nav_launch()
    source = ROOT / 'resource' / 'nav2_robot1_shared_map.yaml'
    generated = module._diagnostic_params(
        str(source), 'robot1', 'rpp', node_prefix='local_')
    try:
        document = yaml.safe_load(Path(generated).read_text(encoding='utf-8'))
    finally:
        Path(generated).unlink(missing_ok=True)
    assert document['local_bt_navigator']['ros__parameters']['global_frame'] == (
        'shared_map')
    # The launch-level RewrittenYaml supplies the phase-specific local frame;
    # this assertion guards that the prefixed node key is the one rewritten.
    launch_text = (LAUNCH / 'two_robots_teammate_filtered_stack_launch.py').read_text()
    assert "f'{parameter_node(\"bt_navigator\")}.ros__parameters.global_frame'" in launch_text
    assert "'global_costmap.global_costmap.ros__parameters.static_layer.map_topic'" in launch_text


def test_shared_nav2_bt_waits_for_post_handoff_action_servers():
    for robot in ('robot1', 'robot2'):
        params = yaml.safe_load((ROOT / 'resource' /
                                f'nav2_{robot}_shared_map.yaml').read_text())
        bt = params['bt_navigator']['ros__parameters']
        assert bt['wait_for_service_timeout'] == 5000
        assert bt['default_server_timeout'] == 500


def test_phase_manager_backoff_prevents_lifecycle_retry_spin():
    phase = (PY / 'unknown_pose_phase_manager.py').read_text()
    assert 'self._next_retry_at = 0.0' in phase
    assert 'self._retry_interval_s = 1.0' in phase
    assert 'time.monotonic() < self._next_retry_at' in phase
    assert 'self._next_retry_at = time.monotonic() + self._retry_interval_s' in phase


def test_phase_manager_terminates_only_deactivated_local_processes_before_shared_start():
    phase = (PY / 'unknown_pose_phase_manager.py').read_text()
    assert 'def _local_process_pids' in phase
    assert "argv[index + 1] == '__ns:=/%s' % self.robot_id" in phase
    assert "name.startswith('local_')" in phase
    assert "name == 'unknown_pose_frontend'" in phase
    assert 'os.kill(pid, signal.SIGTERM)' in phase
    assert 'os.kill(pid, signal.SIGKILL)' in phase
    assert "self._transition == 'SHUTTING_DOWN_LOCAL'" in phase
    assert 'self._terminate_local_processes()' in phase


def test_phase_manager_does_not_use_broad_ros_process_cleanup():
    phase = (PY / 'unknown_pose_phase_manager.py').read_text()
    assert 'pkill' not in phase
    assert 'killall' not in phase
    assert 'ros2 node list' not in phase
    assert "os.listdir('/proc')" in phase


def test_fast_runner_uses_remaining_startup_deadline_for_nav2():
    runner = (PY / 'cooperative_trial_fast.py').read_text()
    assert 'nav2_deadline = readiness_deadline' in runner
    assert 'phase_start + 60.0' not in runner


def test_artifact_observer_remains_in_full_launch():
    full = (LAUNCH / 'two_robots_decentralized_exploration_launch.py').read_text()
    assert "executable='cooperative_experiment_logger'" in full
    assert "'enable_forensic_capture': LaunchConfiguration(" in full
    assert "'unknown_pose_diagnostic_output'" in full


def test_full_campaign_observer_records_joint_state_health():
    logger = (PY / 'cooperative_experiment_logger.py').read_text()
    assert "JointState" in logger
    assert "f'/{r}/joint_states'" in logger
    assert "'joint_states':self.p['odom_stale_s']" in logger


def test_local_prehandoff_assignment_uses_single_executor_without_changing_shared_default():
    full = (LAUNCH / 'two_robots_decentralized_exploration_launch.py').read_text()
    assignment = (PY / 'distributed_frontier_assignment.py').read_text()
    assert "'executor_threads': 1" in full
    assert "'executor_threads', 4" in assignment
    assert 'SingleThreadedExecutor' in assignment


def test_no_path_exchange_or_ghost_cleanup_was_added():
    frontend = (PY / 'unknown_pose_frontend.py').read_text()
    full = (LAUNCH / 'two_robots_decentralized_exploration_launch.py').read_text()
    combined = frontend + full
    assert 'path_exchange' not in combined.lower()
    assert 'ghost_cleanup' not in combined.lower()
    assert 'ground_truth' not in frontend
