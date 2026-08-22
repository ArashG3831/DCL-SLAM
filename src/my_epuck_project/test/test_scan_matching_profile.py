"""Static/profile regression coverage for the conservative scan-match A/B profile."""

from pathlib import Path

from my_epuck_project.cooperative_profiles import profile


PACKAGE = Path(__file__).resolve().parents[1]
WORLDS = PACKAGE / 'worlds'
RESOURCE = PACKAGE / 'resource'
LAUNCH = PACKAGE / 'launch'


def selected(name):
    return profile(name, WORLDS)


def test_scan_matching_profile_is_separate_and_keeps_dynamic_20ms_world():
    value = selected('large_unknown_pose_close_start_20ms_scan_matching')
    baseline = selected('large_unknown_pose_close_start_20ms')
    assert value['world'] == baseline['world']
    assert value['world_metadata']['sha256'] == baseline['world_metadata']['sha256']
    assert value['physics_profile'] == 'dynamic_low_slip_20ms_finite'
    assert value['unknown_initial_pose'] is True
    assert 'kinematic' not in Path(value['world_path']).read_text(
        encoding='utf-8').lower()


def test_existing_dynamic_profile_has_no_scan_matching_runtime_override():
    value = selected('large_unknown_pose_close_start_20ms')
    assert value['slam_runtime_parameters'] == {}
    for robot in ('robot1', 'robot2'):
        params = (RESOURCE / f'slam_toolbox_{robot}_teammate_filtered.yaml').read_text(
            encoding='utf-8')
        assert 'use_scan_matching: false' in params
        assert 'do_loop_closing: false' in params


def test_conservative_scan_matching_values_are_supported_and_symmetric():
    parameters = selected(
        'large_unknown_pose_close_start_20ms_scan_matching'
    )['slam_runtime_parameters']
    expected = {
        'use_scan_matching': True,
        'do_loop_closing': False,
        'correlation_search_space_dimension': 0.12,
        'correlation_search_space_resolution': 0.01,
        'correlation_search_space_smear_deviation': 0.015,
        'distance_variance_penalty': 0.05,
        'angle_variance_penalty': 0.05235987755982989,
        'minimum_distance_penalty': 0.15,
        'minimum_angle_penalty': 0.70,
        'coarse_search_angle_offset': 0.0523596583,
        'coarse_angle_resolution': 0.0174532925,
        'fine_search_angle_offset': 0.0034906585,
        'use_response_expansion': False,
        'map_update_interval': 1.0,
    }
    assert parameters == expected
    slam = (LAUNCH / 'two_robots_teammate_filtered_dual_slam_launch.py').read_text(
        encoding='utf-8')
    assert 'slam_runtime_parameters' in slam
    assert slam.count('slam_runtime_parameters)') >= 2
    assert 'if slam_runtime_parameters:' in slam


def test_all_nested_launch_layers_accept_scan_matching_profile():
    names = (
        'two_robots_unknown_pose_exploration_launch.py',
        'two_robots_decentralized_exploration_launch.py',
        'two_robots_frontier_candidates_launch.py',
        'two_robots_teammate_filtered_stack_launch.py',
        'two_robots_teammate_filtered_dual_slam_launch.py',
        'two_robots_distributed_assignment_launch.py',
    )
    for name in names:
        text = (LAUNCH / name).read_text(encoding='utf-8')
        assert 'large_unknown_pose_close_start_20ms_scan_matching' in text


def test_scan_matching_campaign_uses_unknown_pose_and_sim_watchdog():
    runner = (PACKAGE / 'my_epuck_project' /
              'cooperative_regression.py').read_text(encoding='utf-8')
    assert '--unknown-initial-pose' in runner
    assert 'unknown_initial_pose:=' in runner
    assert "args.time_mode == 'sim'" in runner
    assert "shutdown_reason = 'simulated_mission_timeout'" in runner
    assert 'large_unknown_pose_close_start_20ms_scan_matching' in runner


def test_shared_activation_receives_native_boolean_scan_matching_parameters():
    launch = (LAUNCH / 'two_robots_decentralized_exploration_launch.py').read_text(
        encoding='utf-8')
    assert "activation_scan_matching = profile_scan_parameters.get(" in launch
    assert "'use_scan_matching': activation_scan_matching" in launch
    assert "'do_loop_closing': activation_loop_closing" in launch


def test_scan_matching_profile_preserves_sensor_and_physics_contract():
    value = selected('large_unknown_pose_close_start_20ms_scan_matching')
    content = Path(value['world_path']).read_text(encoding='utf-8')
    assert 'basicTimeStep 20' in content
    assert content.count('noise 0') == 2
    assert content.count('resolution -1') == 2
    assert 'kinematic TRUE' not in content
    assert 'supervisor TRUE' not in content
