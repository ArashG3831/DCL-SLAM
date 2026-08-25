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
        'throttle_scans': 1,
        # The corrected fixed-scan input is approximately 1 Hz, so every
        # corrected scan must be available to Slam Toolbox.
        'minimum_time_interval': 1.0,
        'lidar_update_rate': 1.0,
        'scan_input_reliability': 'reliable',
        'map_update_interval': 1.0,
    }
    assert parameters == expected
    slam = (LAUNCH / 'two_robots_teammate_filtered_dual_slam_launch.py').read_text(
        encoding='utf-8')
    assert 'slam_runtime_parameters' in slam
    assert slam.count('slam_runtime_parameters)') >= 2
    assert 'if slam_runtime_parameters:' in slam
    assert 'scan_output_reliability' not in slam
    assert 'scan_output_sample_count' not in slam


def test_scan_matching_overlays_are_identical_and_normal_profile_is_unchanged():
    close = selected('large_unknown_pose_close_start_20ms_scan_matching')
    far = selected('large_unknown_pose_far_start_20ms_scan_matching')
    assert close['slam_runtime_parameters'] == far['slam_runtime_parameters']
    assert close['slam_runtime_parameters']['throttle_scans'] == 1
    normal = selected('large_unknown_pose_close_start_20ms')
    assert normal['slam_runtime_parameters'] == {}
    for robot in ('robot1', 'robot2'):
        yaml = (RESOURCE / f'slam_toolbox_{robot}_teammate_filtered.yaml').read_text(
            encoding='utf-8')
        assert 'throttle_scans: 3' in yaml


def test_generated_runtime_configuration_carries_throttle_to_both_robots():
    launch = (LAUNCH / 'two_robots_teammate_filtered_dual_slam_launch.py').read_text(
        encoding='utf-8')
    # The launch creates one parameter override dictionary for each namespaced
    # Slam Toolbox node.  Keep this assertion textual so it remains a fast,
    # source-only regression check.
    assert launch.count(
        'unknown_initial_pose, slam_runtime_parameters)') >= 2
    source = (LAUNCH / 'two_robots_decentralized_exploration_launch.py').read_text(
        encoding='utf-8')
    assert "'slam_runtime_parameters': profile_scan_parameters" in source


def test_installed_profile_matches_source_profile():
    value = selected('large_unknown_pose_close_start_20ms_scan_matching')
    assert value['slam_runtime_parameters']['throttle_scans'] == 1
    # The installed launch is the generated artifact consumed by ros2 launch.
    # It must retain the same runtime-override path as the source launch; the
    # two profile worlds intentionally have different hashes (close vs far).
    installed = PACKAGE.parent / 'install' / 'my_epuck_project' / 'share' / \
        'my_epuck_project' / 'launch' / \
        'two_robots_teammate_filtered_dual_slam_launch.py'
    if installed.is_file():
        text = installed.read_text(encoding='utf-8')
        assert 'slam_runtime_parameters' in text
        assert 'runtime_parameters.update(slam_runtime_parameters)' in text


def test_scan_pipeline_diagnostic_records_effective_params_and_rates():
    logger = (PACKAGE / 'my_epuck_project' /
              'cooperative_experiment_logger.py').read_text(encoding='utf-8')
    assert 'scan_pipeline_diagnostic.json' in logger
    assert 'corrected_scan_rate_hz' in logger
    assert 'map_update_rate_hz' in logger
    assert 'scan_correction_records' in logger
    assert 'FAIL_SCAN_THROTTLE_MISMATCH' in logger
    assert 'configured_throttle_scans' in logger


def test_nav2_uses_single_raw_relay_secondary_output():
    launch = (LAUNCH / 'two_robots_namespaced_launch.py').read_text(
        encoding='utf-8')
    assert "'secondary_output_topic': 'scan_d500_nav'" in launch
    assert "'secondary_output_depth': 1" in launch
    assert "'secondary_output_sample_count': 180" in launch
    assert "name=f'{robot_name}_d500_nav_scan_fix'" not in launch
    for robot in ('robot1', 'robot2'):
        params = (RESOURCE / f'nav2_{robot}_shared_map.yaml').read_text(
            encoding='utf-8')
        assert f'scan_topic: /{robot}/scan_d500_nav' in params
        assert f'qos_overrides./{robot}/scan_d500_nav.subscription.reliability: best_effort' in params
        assert f'/{robot}/scan_d500_fixed' not in params
        assert 'source_timeout: 3.0' in params


def test_wsl_full_resolution_transport_uses_chunked_single_reader_path():
    """The supported WSL path must not silently select raw 720-beam DDS."""
    launch = (LAUNCH / 'two_robots_namespaced_launch.py').read_text(
        encoding='utf-8')
    assert "'scan_transport', default_value='chunked'" in launch
    assert "choices=['chunked', 'laser_scan']" in launch
    assert "'scan_d500_chunks' if scan_transport == 'chunked'" in launch
    assert "'input_mode': scan_transport" in launch
    assert "webots_controller_module.controller_ip_address = lambda: '127.0.0.1'" in launch
    assert "webots_launcher_module.controller_url_prefix = lambda port='1234': f'tcp://127.0.0.1:{port}/'" in launch
    # Chunked mode removes the stock 720-beam Ros2Lidar device instead of
    # leaving a second raw publisher beside the fragment-safe plugin.
    assert "re.sub(" in launch
    assert "reference=\"d500_lidar\" type=\"Lidar\"" in launch


def test_nested_stack_forwards_fragment_safe_scan_transport():
    launch = (LAUNCH / 'two_robots_teammate_filtered_stack_launch.py').read_text(
        encoding='utf-8')
    assert "'scan_transport': LaunchConfiguration('scan_transport')" in launch
    assert "DeclareLaunchArgument('scan_transport', default_value='chunked'" in launch


def test_scan_publish_period_is_converted_to_ros_double():
    launch = (LAUNCH / 'two_robots_namespaced_launch.py').read_text(
        encoding='utf-8')
    assert "scan_publish_period = float(" in launch
    assert launch.count("'minimum_time_interval': scan_publish_period") == 1


def test_collision_monitor_uses_sim_clock_for_relay_scan_stamps():
    """Safety timestamps must share the Webots clock with LaserScan headers."""
    for robot in ('robot1', 'robot2'):
        params = (RESOURCE / f'nav2_{robot}_shared_map.yaml').read_text(
            encoding='utf-8')
        collision = params[params.index('collision_monitor:'):]
        assert 'use_sim_time: True' in collision


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


def test_wsl_cyclonedds_profile_avoids_low_fragment_bounds_for_ros_samples():
    """Keep the WSL loopback profile away from CycloneDDS low-limit overruns.

    The simulation publishes fragmented FrontierCandidateArray samples.  The
    old 1025/1472-byte limits reproduced an AddressSanitizer iovec over-read
    in CycloneDDS 0.10.x; the Wi-Fi profile is intentionally not covered here.
    """
    xml = (PACKAGE.parent.parent / 'config' / 'cyclonedds' /
           'wsl_loopback.xml').read_text(encoding='utf-8')
    # Keep CycloneDDS' transport fragment size at its supported default.  A
    # forced 4000-byte fragment size breaks transient-local robot descriptions
    # on the installed WSL loopback transport even when small samples work.
    assert '<FragmentSize>' not in xml
    assert '<MaxMessageSize>14720B</MaxMessageSize>' in xml
    # Leave MaxRexmitMessageSize at CycloneDDS' transport default.  Explicitly
    # setting it to 14720B suppresses loopback discovery with the installed
    # Jazzy CycloneDDS build even though ordinary data samples still publish.
    assert '<MaxRexmitMessageSize>' not in xml
    assert '<FragmentSize>1025B</FragmentSize>' not in xml
    assert '<NetworkInterface name="lo"/>' in xml
    assert '<AllowMulticast>false</AllowMulticast>' in xml
    assert '<AllowMulticast>spdp</AllowMulticast>' not in xml
    assert '<Peer Address="127.0.0.1"/>' in xml
    assert '<SharedMemory>' not in xml


def test_campaign_forwards_distinct_campaign_owned_frontend_diagnostics():
    runner = (PACKAGE / 'my_epuck_project' /
              'cooperative_regression.py').read_text(encoding='utf-8')
    assert 'frontend_diagnostic_output = ' in runner
    assert 'unknown_pose_diagnostic_output:=' in runner
    assert 'must be absolute' in runner
    assert 'escaped campaign directory' in runner


def test_scan_correction_stream_is_passive_and_required_when_enabled():
    evidence = (PACKAGE / 'my_epuck_project' /
                'forensic_evidence.py').read_text(encoding='utf-8')
    logger = (PACKAGE / 'my_epuck_project' /
              'cooperative_experiment_logger.py').read_text(encoding='utf-8')
    assert 'corrections.jsonl' in evidence
    assert 'local_map_to_odom_tf' in evidence
    assert 'T_map_base = T_map_odom * T_odom_base' in evidence
    assert 'scan_matching_enabled' in logger
    assert "'scan_matching'" in logger
    assert 'record_scan_correction_at_map_update' in logger
    assert 'delta_map_to_odom_between_local_map_updates' in evidence
    assert 'NO_PREVIOUS_MAP_TO_ODOM_BASELINE' in evidence


def test_cleanup_is_exact_campaign_path_owned():
    runner = (PACKAGE / 'my_epuck_project' /
              'cooperative_regression.py').read_text(encoding='utf-8')
    assert 'terminate_campaign_owned_processes' in runner
    assert 'campaign_text in command' in runner


def test_sim_clock_readiness_uses_collector_before_secondary_dds_probe():
    runner = (PACKAGE / 'my_epuck_project' /
              'cooperative_regression.py').read_text(encoding='utf-8')
    assert "if args.time_mode == 'sim':" in runner
    assert "source': 'collector_status.json'" in runner
    assert "else:\n                    clock_ok, clock_details = bounded_readiness_probe" in runner
    assert 'bounded_readiness_probe' in runner
    assert "kind, domain, unknown_initial_pose, result_queue" in runner
    assert 'process.terminate()' in runner


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
