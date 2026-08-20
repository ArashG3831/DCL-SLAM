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
    phase = (PY / 'unknown_pose_phase_manager.py').read_text()
    assert "autostart=False" in stack
    assert "shared_dispatch_enabled = False if unknown_initial_pose" in full
    assert "ManageLifecycleNodes.Request.STARTUP" in phase
    assert "ManageLifecycleNodes.Request.SHUTDOWN" in phase
    assert "message.accepted" in phase


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


def test_fast_runner_uses_remaining_startup_deadline_for_nav2():
    runner = (PY / 'cooperative_trial_fast.py').read_text()
    assert 'nav2_deadline = readiness_deadline' in runner
    assert 'phase_start + 60.0' not in runner


def test_artifact_observer_remains_in_full_launch():
    full = (LAUNCH / 'two_robots_decentralized_exploration_launch.py').read_text()
    assert "executable='cooperative_experiment_logger'" in full
    assert "'enable_forensic_capture': LaunchConfiguration(" in full
    assert "'unknown_pose_diagnostic_output'" in full


def test_no_path_exchange_or_ghost_cleanup_was_added():
    frontend = (PY / 'unknown_pose_frontend.py').read_text()
    full = (LAUNCH / 'two_robots_decentralized_exploration_launch.py').read_text()
    combined = frontend + full
    assert 'path_exchange' not in combined.lower()
    assert 'ghost_cleanup' not in combined.lower()
    assert 'ground_truth' not in frontend
