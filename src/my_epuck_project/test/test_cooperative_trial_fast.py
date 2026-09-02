from pathlib import Path
import inspect
from types import SimpleNamespace

from my_epuck_project import cooperative_trial_fast as fast
from my_epuck_project.cooperative_trial_fast import (
    GRAPH_SUFFIXES,
    LOCAL_UNKNOWN_POSE_GRAPH_SUFFIXES,
    LAUNCH_FILE,
    LOCAL_NAV2_NODES,
    ReadyProbe,
    boolean,
    filtered_runtime_environment,
    is_campaign_webots_driver,
    launch_command,
    parser,
    requires_fixed_anchor_forensics,
    resolve_world,
    webots_driver_provenance,
)


def test_runtime_environment_removes_only_old_workspace_search_paths():
    source = {
        'AMENT_PREFIX_PATH': (
            '/home/arash/webots_ws/install/webots_ros2:/opt/ros/jazzy'),
        'PYTHONPATH': (
            '/home/arash/webots_ws/build/webots_ros2:/audit/build'),
        'PATH': '/usr/bin',
        'ROS_DOMAIN_ID': '221',
    }
    environment, removed = filtered_runtime_environment(source)
    assert environment['AMENT_PREFIX_PATH'] == '/opt/ros/jazzy'
    assert environment['PYTHONPATH'] == '/audit/build'
    assert environment['PATH'] == '/usr/bin'
    assert environment['ROS_DOMAIN_ID'] == '221'
    assert len(removed) == 2


def test_runtime_environment_keeps_audit_and_dependency_overlays():
    source = {
        'AMENT_PREFIX_PATH': (
            '/home/arash/webots_ws_clean_validation_20260823/install_audit_20260827'
            ':/home/arash/webots_ws/install'),
        'PYTHONPATH': '/home/arash/webots_ws_clean_validation_20260823/build_audit_20260827',
    }
    environment, removed = filtered_runtime_environment(source)
    assert 'install_audit_20260827' in environment['AMENT_PREFIX_PATH']
    assert 'build_audit_20260827' in environment['PYTHONPATH']
    assert environment['AMENT_PREFIX_PATH'].endswith('install_audit_20260827')
    assert len(removed) == 1


def test_runtime_environment_binds_explicit_webots_driver_before_stale_underlay():
    driver = '/home/arash/webots_ws_close_validation_2eb/install/webots_ros2_driver'
    source = {
        'MY_EPUCK_WEBOTS_DRIVER_PREFIX': driver,
        'AMENT_PREFIX_PATH': (
            '/home/arash/webots_ws/install/webots_ros2_driver:/opt/ros/jazzy'),
        'PYTHONPATH': '/home/arash/webots_ws/install/webots_ros2_driver/lib/python3.12/site-packages',
        'LD_LIBRARY_PATH': '/home/arash/webots_ws/install/webots_ros2_driver/lib',
    }
    environment, _ = filtered_runtime_environment(source)
    assert environment['AMENT_PREFIX_PATH'].split(':')[0] == driver
    assert driver not in environment['AMENT_PREFIX_PATH'].split(':')[1:]
    assert environment['PYTHONPATH'].split(':')[0] == (
        driver + '/lib/python3.12/site-packages')
    assert environment['LD_LIBRARY_PATH'].split(':')[0] == driver + '/lib'


def test_fast_parser_exposes_only_single_trial_options():
    args = parser().parse_args([])
    assert args.world_profile == 'large'
    assert args.sensor_profile == 'full'
    assert args.rendering is False
    assert args.rviz is False
    assert args.hold_open is False
    assert args.enable_observer is False
    assert args.enable_forensic_capture is False
    assert parser().parse_args([
        '--world-profile', 'large_unknown_pose_16m']).world_profile == (
            'large_unknown_pose_16m')
    assert parser().parse_args([
        '--world-profile', 'large_unknown_pose_close_start_20ms'
    ]).world_profile == 'large_unknown_pose_close_start_20ms'


def test_close_start_unknown_pose_runner_requires_passive_fixed_anchor_gt():
    assert requires_fixed_anchor_forensics(parser().parse_args([
        '--world-profile', 'large_unknown_pose_close_start_20ms_scan_matching',
    ])) is True
    assert requires_fixed_anchor_forensics(parser().parse_args([
        '--world-profile', 'large_unknown_pose_16m',
    ])) is False


def test_fast_runner_does_not_hard_code_original_dirty_workspace():
    source = (Path(__file__).resolve().parents[1] / 'my_epuck_project' /
              'cooperative_trial_fast.py').read_text()
    assert "MY_EPUCK_WORKSPACE" in source
    assert "Path('/home/arash/webots_ws')" not in source


def test_fast_runner_checks_runtime_contract_before_launch(monkeypatch):
    """A missing DDS contract fails before Webots can be spawned."""
    monkeypatch.delenv('RMW_IMPLEMENTATION', raising=False)
    monkeypatch.delenv('CYCLONEDDS_URI', raising=False)
    args = parser().parse_args([
        '--world-profile', 'large_unknown_pose_close_start_20ms_scan_matching',
    ])
    assert fast.run(args) == 1


def test_fast_runner_accepts_explicit_isolated_install_prefix(
        tmp_path, monkeypatch):
    isolated = tmp_path / 'install_audit' / 'my_epuck_project'
    monkeypatch.setenv('MY_EPUCK_INSTALL_PREFIX', str(isolated))
    monkeypatch.setattr(fast.shutil, 'which', lambda name: '/usr/bin/ros2')
    monkeypatch.setattr(
        fast.subprocess, 'run',
        lambda *args, **kwargs: SimpleNamespace(
            returncode=0, stdout=f'{isolated}\n', stderr=''))
    assert fast.package_prefix() == str(isolated)


def test_fast_runner_requires_and_validates_isolated_driver_prefix(
        tmp_path, monkeypatch):
    isolated = tmp_path / 'install_webots_driver' / 'webots_ros2_driver'
    executable = isolated / 'lib' / 'webots_ros2_driver' / 'driver'
    executable.parent.mkdir(parents=True)
    executable.touch()
    monkeypatch.setenv('MY_EPUCK_WEBOTS_DRIVER_PREFIX', str(isolated))
    monkeypatch.setattr(fast.shutil, 'which', lambda name: '/usr/bin/ros2')
    monkeypatch.setattr(
        fast.subprocess, 'run',
        lambda *args, **kwargs: SimpleNamespace(
            returncode=0, stdout=f'{isolated}\n', stderr=''))
    assert webots_driver_provenance() == (
        str(isolated.resolve()), str(executable.resolve()))


def test_fast_runner_rejects_stale_driver_prefix(tmp_path, monkeypatch):
    expected = tmp_path / 'audit' / 'webots_ros2_driver'
    actual = tmp_path / 'old' / 'webots_ros2_driver'
    executable = actual / 'lib' / 'webots_ros2_driver' / 'driver'
    executable.parent.mkdir(parents=True)
    executable.touch()
    monkeypatch.setenv('MY_EPUCK_WEBOTS_DRIVER_PREFIX', str(expected))
    monkeypatch.setattr(fast.shutil, 'which', lambda name: '/usr/bin/ros2')
    monkeypatch.setattr(
        fast.subprocess, 'run',
        lambda *args, **kwargs: SimpleNamespace(
            returncode=0, stdout=f'{actual}\n', stderr=''))
    try:
        webots_driver_provenance()
    except fast.FastTrialError as error:
        assert 'stale driver overlay refused' in str(error)
    else:
        raise AssertionError('stale driver prefix was accepted')


def test_single_robot_controller_fixtures_are_symmetric_and_namespaced():
    launch_dir = Path(__file__).resolve().parents[1] / 'launch'
    for robot in ('robot1', 'robot2'):
        source = (launch_dir /
                  f'{robot}_in_two_world_namespaced_launch.py').read_text()
        assert "executable='controller_startup_guard'" in source
        assert f"namespace='{robot}'" in source
        assert f"'controller_manager': '/{robot}/controller_manager'" in source
        assert (f"'controller_param_file': "
                f"'/tmp/my_epuck_project_{robot}_ros2_control.yml'") in source
        assert f"'/{robot}/controller_manager:\\n'" in source
        assert f"'\\n/{robot}/diffdrive_controller:\\n'" in source
        assert f"'\\n/{robot}/joint_state_broadcaster:\\n'" in source
        assert f"/{robot}/diffdrive_controller/odom:=/{robot}/odom" in source
        assert f"/{robot}/diffdrive_controller/cmd_vel:=/{robot}/cmd_vel" in source
        assert 'nodes_to_start=[diffdrive_controller_spawner]' in source
        assert 'RegisterEventHandler' in source
        assert 'OnProcessExit' in source


def test_single_robot_launch_declares_configurations_before_opaque_setup():
    """The deferred setup must not resolve an undeclared LaunchConfiguration."""
    launch_dir = Path(__file__).resolve().parents[1] / 'launch'
    for robot in ('robot1', 'robot2'):
        source = (launch_dir /
                  f'{robot}_in_two_world_namespaced_launch.py').read_text()
        declaration = source.index("DeclareLaunchArgument(\n            'world'")
        opaque = source.index('OpaqueFunction(function=_launch_setup)')
        assert declaration < opaque


def test_wait_process_continues_cleanup_after_sigint(monkeypatch):
    """A user stop cannot interrupt the runner before child cleanup."""
    class InterruptOnceProcess:
        def __init__(self):
            self.calls = 0
            self.returncode = None

        def poll(self):
            return self.returncode

        def wait(self, timeout=None):
            del timeout
            self.calls += 1
            if self.calls == 1:
                raise KeyboardInterrupt
            self.returncode = 0
            return 0

    monkeypatch.setattr(fast.time, 'monotonic', lambda: 0.0)
    process = InterruptOnceProcess()
    assert fast.wait_process(process, 1.0) is True
    assert process.calls == 2


def test_unknown_pose_readiness_accepts_local_graph_before_handoff():
    source = (Path(__file__).resolve().parents[1] / 'my_epuck_project' /
              'cooperative_trial_fast.py').read_text()
    assert "LOCAL_UNKNOWN_POSE_GRAPH_SUFFIXES" in source
    assert "all(item in names for item in LOCAL_UNKNOWN_POSE_GRAPH_SUFFIXES)" in source


def test_boolean_parser():
    assert boolean('true') is True
    assert boolean('false') is False


def test_launch_command_reuses_authoritative_campaign_launch(tmp_path):
    args = parser().parse_args([
        '--world-profile', 'small', '--sensor-profile', 'throughput',
        '--webots-mode', 'realtime', '--rendering', 'true',
        '--diagnostic-mode', 'true',
    ])
    command = launch_command(args, Path('/tmp/test_world.wbt'))
    assert command[:4] == [
        'ros2', 'launch', 'my_epuck_project', LAUNCH_FILE]
    assert 'enable_observer:=false' in command
    assert 'enable_forensic_capture:=false' in command
    assert 'nav2_autostart:=false' in command
    assert 'prehandoff_dispatch_delay_s:=20.0' in command
    assert 'unknown_initial_pose:=true' in command
    assert 'controller_variant:=rpp' in command
    assert 'sensor_profile:=throughput' in command
    assert not any(
        item.startswith('slam_tf_publish_probe_library:=')
        for item in command)
    assert not any(
        item.startswith('slam_tf_publish_probe_log:=')
        for item in command)
    assert not any(
        item.startswith('slam_tf_publication_mode:=')
        for item in command)
    assert 'webots_gui:=true' in command


def test_prehandoff_delay_is_explicitly_overridable():
    args = parser().parse_args(['--prehandoff-dispatch-delay-s', '120'])
    command = launch_command(args, Path('/tmp/test_world.wbt'))
    assert 'prehandoff_dispatch_delay_s:=120.0' in command


def test_motion_fixture_options_are_test_only_runner_overrides():
    args = parser().parse_args([
        '--enable-motion-fixture', 'true',
        '--motion-fixture-start-delay-s', '0',
        '--motion-fixture-cycles', '1',
    ])
    command = launch_command(args, Path('/tmp/test_world.wbt'))
    assert 'enable_motion_fixture:=true' in command
    assert 'motion_fixture_start_delay_s:=0.0' in command
    assert 'motion_fixture_cycles:=1' in command


def test_synchronized_motion_fixture_can_start_in_full_map_mode():
    source = (Path(__file__).resolve().parents[1] / 'my_epuck_project' /
              'unknown_pose_motion_fixture.py').read_text()
    assert 'synchronized_traffic_test' in source
    assert 'hold_prehandoff_motion' in source
    window = source[source.index('def _observation_can_start'):source.index(
        'def _finish_deferred')]
    assert 'return True' in window


def test_launch_command_can_enable_passive_evidence_in_attempt_directory(tmp_path):
    args = parser().parse_args([
        '--enable-observer', 'true',
        '--enable-forensic-capture', 'true',
    ])
    command = launch_command(
        args, Path('/tmp/test_world.wbt'),
        output_root=tmp_path / 'observer', run_id='attempt_01')
    assert 'enable_observer:=true' in command
    assert 'enable_forensic_capture:=true' in command
    assert f'output_root:={tmp_path / "observer"}' in command
    assert 'run_id:=attempt_01' in command
    assert f'unknown_pose_diagnostic_output:={tmp_path / "observer" / "attempt_01" / "frontend"}' in command


def test_headless_benchmark_does_not_enable_frontend_evidence_stream(tmp_path):
    args = parser().parse_args([])
    command = launch_command(
        args, Path('/tmp/test_world.wbt'),
        output_root=tmp_path / 'benchmark', run_id='attempt_01')
    assert 'enable_observer:=false' in command
    assert not any(item.startswith('unknown_pose_diagnostic_output:=')
                   for item in command)


def test_fast_runner_resolves_and_records_canonical_16m_profile(tmp_path, monkeypatch):
    worktree = Path(__file__).resolve().parents[3]
    monkeypatch.setattr(
        'my_epuck_project.cooperative_trial_fast.WORKSPACE', worktree)
    world = worktree / 'src' / 'my_epuck_project' / 'worlds' / (
        'epuck_d500_two_world_unknown_pose_16m_dynamic_low_slip_4ms_finite.wbt')
    args = parser().parse_args([
        '--world-profile', 'large_unknown_pose_16m',
        '--world-path', str(world),
    ])
    resolved = resolve_world(args)
    assert resolved == world.resolve()
    assert args.profile_metadata['world_profile'] == 'large_unknown_pose_16m'
    assert args.profile_metadata['world'] == world.name
    assert args.profile_metadata['initial_separation_m'] == 17.0


def test_fast_runner_rejects_explicit_generic_world_for_16m_profile(tmp_path, monkeypatch):
    worktree = Path(__file__).resolve().parents[3]
    monkeypatch.setattr(
        'my_epuck_project.cooperative_trial_fast.WORKSPACE', worktree)
    generic = worktree / 'src' / 'my_epuck_project' / 'worlds' / (
        'epuck_d500_two_world_large_dynamic_low_slip_4ms_finite.wbt')
    args = parser().parse_args([
        '--world-profile', 'large_unknown_pose_16m',
        '--world-path', str(generic),
    ])
    try:
        resolve_world(args)
    except ValueError as error:
        assert 'world profile/path mismatch' in str(error)
    else:
        raise AssertionError('fast runner accepted a conflicting world path')


def test_16m_profile_and_world_path_are_forwarded_together():
    args = parser().parse_args([
        '--world-profile', 'large_unknown_pose_16m',
    ])
    world = Path('/tmp/epuck_d500_two_world_unknown_pose_16m_dynamic_low_slip_4ms_finite.wbt')
    command = launch_command(args, world)
    assert 'world_profile:=large_unknown_pose_16m' in command
    assert f'world_path:={world}' in command


def test_full_launch_records_source_and_staged_profile_hashes():
    source = (Path(__file__).resolve().parents[1] / 'launch' /
              'two_robots_decentralized_exploration_launch.py').read_text()
    for marker in (
            'WORLD_PROFILE_SELECTED', 'staged_source_sha256',
            'staged_sha256', 'ForensicGroundTruthSupervisor'):
        assert marker in source


def test_readiness_graph_is_the_cooperative_graph():
    assert GRAPH_SUFFIXES == (
        '/robot1/distributed_frontier_assignment',
        '/robot2/distributed_frontier_assignment',
        '/robot1/map_fusion',
        '/robot2/map_fusion',
    )


def test_unknown_pose_readiness_uses_local_nav2_and_local_map_tf():
    assert LOCAL_NAV2_NODES
    assert all(name.startswith('local_') for name in LOCAL_NAV2_NODES)
    source = ReadyProbe.tf_ready.__code__.co_consts
    assert 'shared_map' not in source
    method_source = inspect.getsource(ReadyProbe.activate_and_check_nav2)
    assert 'LOCAL_NAV2_NODES' in method_source
    assert 'lifecycle_manager_navigation/manage_nodes' in method_source
    assert 'ManageLifecycleNodes.Request.STARTUP' in method_source


def test_fast_runner_accepts_already_autostarted_local_nav2():
    method_source = inspect.getsource(ReadyProbe.activate_and_check_nav2)
    assert 'active_now = True' in method_source
    assert 'startup_results[robot] = True' in method_source


def test_nav2_readiness_retries_a_failed_lifecycle_startup():
    source = inspect.getsource(ReadyProbe.activate_and_check_nav2)
    assert 'next_startup_attempt' in source
    assert 'if startup_results[robot]:' in source
    assert 'next_startup_attempt[robot]' in source


def test_mission_timeout_reserves_total_wall_clock_finalization_budget():
    source = inspect.getsource(fast.run)
    assert 'hard_deadline = started + max(0.0, total_wall_limit)' in source
    assert 'service_deadline = hard_deadline - FINALIZATION_BUDGET_S' in source
    assert 'mission_deadline = None if args.hold_open else service_deadline' in source


def test_campaign_driver_filter_is_namespaced_and_not_broad_kill():
    valid = [
        '/home/arash/webots_ws/install/webots_ros2_driver/lib/webots_ros2_driver/driver',
        '-r', '__ns:=/robot1',
        '--params-file', '/tmp/my_epuck_project_robot1_ros2_control.yml',
    ]
    jazzy_valid = [
        '/opt/ros/jazzy/lib/webots_ros2_driver/driver',
        '-r', '__ns:=/robot2',
        '--params-file', '/tmp/my_epuck_project_robot2_ros2_control.yml',
    ]
    unrelated = [
        '/opt/other_driver', '-r', '__ns:=/robot1',
        '--params-file', '/tmp/other.yml',
    ]
    assert is_campaign_webots_driver(valid)
    assert is_campaign_webots_driver(jazzy_valid)
    assert not is_campaign_webots_driver(unrelated)


def test_rviz_run_starts_overlay_and_uses_profile_camera(tmp_path):
    """The visual fast runner owns both the bridge and scaled RViz preset."""
    args = parser().parse_args([
        '--world-profile', 'large_unknown_pose_close_start_20ms_scan_matching',
        '--rviz', 'true',
    ])
    command = launch_command(args, tmp_path)
    assert 'launch_visualization_overlay:=true' in command
    config = Path(fast.rviz_command(args.world_profile)[2])
    text = config.read_text(encoding='utf-8')
    assert 'Distance: 45' in text
    assert 'X: 17.4' in text
