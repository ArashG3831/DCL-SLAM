import json
from pathlib import Path
import inspect
import threading
import time
from types import SimpleNamespace

from my_epuck_project import cooperative_trial_fast as fast
from my_epuck_project.cooperative_trial_fast import (
    GRAPH_SUFFIXES,
    LOCAL_UNKNOWN_POSE_GRAPH_SUFFIXES,
    LAUNCH_FILE,
    LOCAL_NAV2_NODES,
    ReadyProbe,
    SimulationHorizonMonitor,
    WallWatchdog,
    _logger_processes,
    _mission_processes,
    boolean,
    filtered_runtime_environment,
    is_campaign_webots_driver,
    launch_command,
    nav2_readiness_action,
    nav2_readiness_node_names,
    phase_aware_nav2_readiness,
    logger_artifact_finalization_status,
    observer_finalization_status,
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
    assert args.local_path_gate_mode is None
    assert args.hold_open is False
    assert args.enable_observer is False
    assert args.observer_architecture == 'legacy'
    assert args.enable_forensic_capture is False
    assert args.experiment_condition == 'C'
    assert args.webots_random_seed is None
    assert args.simulation_horizon_s is None
    assert args.wall_watchdog_s is None
    assert parser().parse_args([
        '--world-profile', 'large_unknown_pose_16m']).world_profile == (
            'large_unknown_pose_16m')
    assert parser().parse_args([
        '--world-profile', 'large_unknown_pose_close_start_20ms'
    ]).world_profile == 'large_unknown_pose_close_start_20ms'


def test_fast_runner_records_condition_and_seed_in_launch_command(tmp_path):
    args = parser().parse_args([
        '--experiment-condition', 'D', '--webots-random-seed', '1001',
    ])
    args.seed_provenance_json = '{"requested_seed":1001}'
    command = launch_command(args, tmp_path / 'world.wbt')
    assert 'experiment_condition:=D' in command
    assert 'seed_provenance_json:={"requested_seed":1001}' in command


def test_condition_c_command_enables_common_start_release_and_rosbag(tmp_path):
    args = parser().parse_args([
        '--experiment-condition', 'C',
        '--assignment-strategy', 'frontier_cost_only',
        '--local-path-gate-mode', 'MODE_B',
        '--enable-observer', 'true',
        '--enable-forensic-capture', 'true',
    ])
    command = launch_command(
        args, tmp_path / 'world.wbt', output_root=tmp_path,
        run_id='c-smoke')
    assert 'common_start_release_required:=true' in command
    assert 'enable_passive_rosbag:=true' in command


def test_thin_condition_c_disables_legacy_live_observer(tmp_path):
    args = parser().parse_args([
        '--experiment-condition', 'C',
        '--assignment-strategy', 'frontier_cost_only',
        '--local-path-gate-mode', 'MODE_B',
        '--enable-observer', 'true',
        '--enable-forensic-capture', 'true',
        '--observer-architecture', 'thin',
    ])
    command = launch_command(
        args, tmp_path / 'world.wbt', output_root=tmp_path,
        run_id='thin-c')
    assert 'observer_architecture:=thin' in command
    assert 'enable_observer:=false' in command
    # Thin mode may still stage the read-only Supervisor world node so the
    # external GT/contact recorder can connect; this does not enable the
    # legacy live forensic logger, which is gated separately by observer_architecture.
    assert 'enable_forensic_capture:=true' in command
    assert 'enable_passive_rosbag:=false' in command


def test_fast_runner_rejects_negative_webots_seed():
    import pytest
    with pytest.raises(SystemExit):
        parser().parse_args(['--webots-random-seed', '-1'])


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


def test_fast_runner_requires_explicit_gate_mode_before_environment(monkeypatch):
    args = parser().parse_args([])
    monkeypatch.setattr(
        fast, 'filtered_runtime_environment',
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError('environment lookup must not run')))
    assert fast.run(args) == 1


def test_fast_runner_requires_explicit_gate_mode_before_environment(monkeypatch):
    args = parser().parse_args([])
    monkeypatch.setattr(
        fast, 'filtered_runtime_environment',
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError('environment lookup must not run')))
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


def test_observer_shutdown_uses_dedicated_bounded_finalization_grace(monkeypatch):
    sent = []
    waits = []

    class Logger:
        pid = 7

        def cmdline(self):
            return ['/install/cooperative_experiment_logger']

        def send_signal(self, sig):
            sent.append(sig)

    class Root:
        def children(self, recursive=False):
            assert recursive is True
            return [Logger()]

    monkeypatch.setattr(fast.psutil, 'Process', lambda pid: Root())
    monkeypatch.setattr(
        fast.psutil, 'wait_procs',
        lambda processes, timeout: waits.append((processes, timeout)))

    selected = fast.shutdown_observer_before_launch(
        SimpleNamespace(pid=42))

    assert selected == [7]
    assert sent == [fast.signal.SIGINT]
    assert len(waits) == 1
    assert waits[0][1] == fast.OBSERVER_FINALIZATION_GRACE_S == 60.0


def test_launch_group_escalation_waits_for_observer_barrier(monkeypatch):
    events = []
    observer_started = threading.Event()
    release_observer = threading.Event()
    result = {}

    class Launch:
        pid = 99
        returncode = None

        def poll(self):
            return None

    def observer_barrier(launch, absolute_deadline=None):
        del launch, absolute_deadline
        events.append('observer_started')
        observer_started.set()
        assert release_observer.wait(1.0)
        events.append('observer_finished')
        return [7]

    def record_signal(process, sig):
        events.append(('launch_signal', process.pid, sig))

    def bounded_wait(process, timeout, absolute_deadline=None):
        del process, absolute_deadline
        events.append(('launch_wait', timeout))
        return False

    monkeypatch.setattr(fast, 'shutdown_mission_before_logger',
                        lambda *args, **kwargs: [])
    monkeypatch.setattr(fast, 'shutdown_observer_before_launch',
                        observer_barrier)
    monkeypatch.setattr(fast, 'stop_process', record_signal)
    monkeypatch.setattr(fast, 'wait_process', bounded_wait)

    thread = threading.Thread(
        target=lambda: result.setdefault(
            'cleanup', fast.shutdown_processes(Launch())))
    thread.start()
    assert observer_started.wait(1.0)
    assert not any(
        isinstance(event, tuple) and event[0] == 'launch_signal'
        for event in events)

    release_observer.set()
    thread.join(timeout=1.0)
    assert not thread.is_alive()
    assert events[:2] == ['observer_started', 'observer_finished']
    signals = [event[2] for event in events
               if isinstance(event, tuple) and event[0] == 'launch_signal']
    assert signals == [fast.signal.SIGINT, fast.signal.SIGTERM,
                       fast.signal.SIGKILL]
    assert result['cleanup']['graceful'] is False


def test_ready_probe_horizon_uses_live_clock_not_observer_files():
    probe = ReadyProbe.__new__(ReadyProbe)
    probe.clock_values = []
    probe.clock_start_s = None
    probe.latest_clock_s = None
    probe._on_clock(SimpleNamespace(clock=SimpleNamespace(sec=0, nanosec=0)))
    probe._on_clock(SimpleNamespace(clock=SimpleNamespace(sec=1499, nanosec=0)))
    assert probe.clock_start_s == 0
    assert probe.latest_clock_s == 1499
    assert not probe.simulation_horizon_reached(1500.0)
    probe._on_clock(SimpleNamespace(clock=SimpleNamespace(sec=1500, nanosec=0)))
    assert probe.simulation_horizon_reached(1500.0)


def test_horizon_monitor_records_horizon_without_interrupting_launch(
        monkeypatch):
    clock = [0.0]
    signals = []

    class Process:
        pid = 42

        def poll(self):
            return None

    monkeypatch.setattr(
        fast, 'stop_process',
        lambda process, sig: signals.append((process.pid, sig)))
    monitor = SimulationHorizonMonitor(180.0, lambda: clock[0])
    monitor.add_process(Process())
    monitor.start()
    clock[0] = 180.0
    assert monitor.reached.wait(1.0)
    monitor.stop()
    assert monitor.reached_sim_time_s == 180.0
    assert signals == []


def test_horizon_monitor_marks_horizon_overrun_and_contains_launch(
        monkeypatch):
    clock = [0.0]
    signals = []

    class Process:
        pid = 43

        def poll(self):
            return None

    monkeypatch.setattr(
        fast, 'stop_process',
        lambda process, sig: signals.append((process.pid, sig)))
    monitor = SimulationHorizonMonitor(
        180.0, lambda: clock[0], overrun_fence_s=240.0)
    monitor.add_process(Process())
    monitor.start()
    clock[0] = 180.0
    assert monitor.reached.wait(1.0)
    clock[0] = 240.0
    assert monitor.overrun.wait(1.0)
    monitor.stop()
    assert monitor.reached_sim_time_s == 180.0
    assert monitor.overrun_sim_time_s == 240.0
    assert signals == [(43, fast.signal.SIGINT)]


def test_horizon_cleanup_gives_logger_finalization_barrier_before_launch_stop():
    source = inspect.getsource(fast.shutdown_processes)
    logger_barrier = source.index('shutdown_observer_before_launch')
    launch_stop = source.index('stop_process(launch, signal.SIGINT)')
    logger_wait = inspect.getsource(fast.shutdown_observer_before_launch)
    assert logger_barrier < launch_stop
    assert 'psutil.wait_procs' in logger_wait
    assert 'processes' in logger_wait


def test_finalization_acknowledgement_requires_external_and_logger_contract(
        tmp_path):
    attempt = tmp_path / 'attempt'
    run = attempt / 'observer' / 'run-1'
    runtime = run / 'forensic' / 'runtime'
    runtime.mkdir(parents=True)
    (runtime / 'runtime_metrics.json').write_text(
        json.dumps({'finalized': True}), encoding='utf-8')
    (run / 'summary.json').write_text(json.dumps({
        'artifact_finalization': {'complete': True},
    }), encoding='utf-8')
    (run / 'artifact_finalization.json').write_text(json.dumps({
        'complete': True, 'status': 'COMPLETE', 'missing': [],
    }), encoding='utf-8')
    (run / 'run_manifest.json').write_text(json.dumps({
        'clean_shutdown': True,
        'shutdown_status': 'clean',
        'artifact_finalization': {
            'complete': True, 'status': 'COMPLETE', 'missing': [],
        },
    }), encoding='utf-8')
    assert logger_artifact_finalization_status(attempt) is True
    assert observer_finalization_status(attempt) is True

    (run / 'artifact_finalization.json').write_text(json.dumps({
        'complete': False, 'status': 'NOT_FINALIZED', 'missing': [],
    }), encoding='utf-8')
    assert logger_artifact_finalization_status(attempt) is False
    assert observer_finalization_status(attempt) is False


def test_horizon_monitor_does_not_teardown_processes_before_ordered_cleanup():
    source = inspect.getsource(fast.SimulationHorizonMonitor._run)
    assert 'self.reached.set()' in source
    assert 'if self.reached.is_set() and latest >= self.overrun_fence_s' in source


def test_horizon_monitor_is_started_before_all_condition_readiness_paths():
    source = inspect.getsource(fast.run)
    monitor_start = source.index('horizon_monitor.start()')
    readiness = source.index('readiness_deadline =')
    nav2_wait = source.index('activate_and_check_nav2(')
    assert monitor_start < readiness < nav2_wait
    assert 'horizon_monitor.reached.is_set()' in source


def test_horizon_completion_is_condition_independent_and_preserves_a_b_paths():
    source = inspect.getsource(fast.run)
    assert "termination_reason = 'SIM_TIME_COMPLETE'" in source
    assert 'SimulationHorizonReached' in source
    for condition in ('A', 'B', 'C', 'D'):
        assert condition in ('A', 'B', 'C', 'D')
    assert 'nav2_autostart=args.experiment_condition in (\'A\', \'B\')' in source


def test_wall_watchdog_has_independent_deadline_and_process_group_control():
    source = inspect.getsource(WallWatchdog)
    assert 'time.monotonic()' in source
    assert 'signal.SIGTERM' in source
    assert 'signal.SIGKILL' in source
    assert 'read_text' not in source
    assert 'open(' not in source


def test_cleanup_waits_are_bounded_by_absolute_wall_deadline():
    source = inspect.getsource(fast.shutdown_processes)
    assert 'absolute_deadline' in source
    assert 'absolute_deadline=absolute_deadline' in source


def test_observer_logger_is_finalized_before_ros_launch_shutdown():
    source = inspect.getsource(fast.shutdown_processes)
    assert source.index('shutdown_mission_before_logger') < source.index(
        'shutdown_observer_before_launch')
    assert source.index('shutdown_observer_before_launch') < source.index(
        'stop_process(launch, signal.SIGINT)')


def test_logger_process_filter_excludes_webots_and_controller(monkeypatch):
    class Process:
        def __init__(self, command):
            self.pid = 1
            self._command = command

        def cmdline(self):
            return self._command

    class Root:
        def children(self, recursive=False):
            assert recursive is True
            return [
                Process(['/mnt/c/Program Files/Webots/webots.exe', '--batch']),
                Process(['/install/my_epuck_project/cooperative_experiment_logger']),
                Process(['/path/webots-controller', '--robot-name=robot1']),
            ]

    monkeypatch.setattr(fast.psutil, 'Process', lambda pid: Root())
    selected = _logger_processes(SimpleNamespace(pid=42))
    assert len(selected) == 1
    assert 'cooperative_experiment_logger' in selected[0].cmdline()[0]


def test_mission_process_filter_keeps_webots_logger_and_supervisor_alive(
        monkeypatch):
    class Process:
        def __init__(self, pid, command):
            self.pid = pid
            self._command = command

        def cmdline(self):
            return self._command

    class Root:
        def children(self, recursive=False):
            assert recursive is True
            return [
                Process(1, ['/bin/webots.exe', '--batch']),
                Process(2, ['/install/cooperative_experiment_logger']),
                Process(3, ['/install/paced_ros2_supervisor']),
                Process(4, ['/install/unknown_pose_frontend']),
            ]

    monkeypatch.setattr(fast.psutil, 'Process', lambda pid: Root())
    selected = _mission_processes(SimpleNamespace(pid=42))
    assert [process.pid for process in selected] == [4]


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
        '--enable-contact-capture', 'true',
    ])
    command = launch_command(
        args, Path('/tmp/test_world.wbt'),
        output_root=tmp_path / 'observer', run_id='attempt_01')
    assert 'enable_observer:=true' in command
    assert 'enable_forensic_capture:=true' in command
    assert 'enable_contact_capture:=true' in command
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
    assert 'nav2_readiness_node_names' in method_source
    assert 'lifecycle_manager_navigation/manage_nodes' in method_source
    assert 'ManageLifecycleNodes.Request.STARTUP' in method_source


def test_fast_runner_accepts_already_autostarted_local_nav2():
    method_source = inspect.getsource(ReadyProbe.activate_and_check_nav2)
    assert 'active_now = True' in method_source
    assert 'startup_results[robot] = True' in method_source


def test_nav2_readiness_never_restarts_active_nodes():
    assert nav2_readiness_action(True, True) == 'READY_NO_STARTUP'
    assert nav2_readiness_action(True, False) == 'READY_NO_STARTUP'


def test_nav2_readiness_waits_for_launch_autostart_without_startup_request():
    assert nav2_readiness_action(False, True) == 'WAIT_FOR_LAUNCH_AUTOSTART'


def test_nav2_readiness_preserves_runner_startup_for_inactive_nodes():
    assert nav2_readiness_action(False, False) == 'SEND_STARTUP'


def test_nav2_readiness_uses_nonlocal_names_for_a_and_b():
    assert nav2_readiness_node_names('A') == fast.NAV2_NODES
    assert nav2_readiness_node_names('B') == fast.NAV2_NODES
    assert nav2_readiness_node_names('C') == LOCAL_NAV2_NODES
    assert nav2_readiness_node_names('D') == LOCAL_NAV2_NODES


def test_nav2_readiness_retries_a_failed_lifecycle_startup():
    source = inspect.getsource(ReadyProbe.activate_and_check_nav2)
    assert 'next_startup_attempt' in source
    assert 'if startup_results[robot]:' in source
    assert 'next_startup_attempt[robot]' in source


def test_cd_readiness_latches_local_before_handoff_and_shared_activation():
    ready, reason = phase_aware_nav2_readiness(
        {'robot1', 'robot2'}, True, {'robot1', 'robot2'})
    assert ready is True
    assert reason == 'shared_nav2_active_after_handoff'


def test_cd_readiness_does_not_hide_failed_local_activation():
    ready, reason = phase_aware_nav2_readiness(
        {'robot1'}, True, {'robot1', 'robot2'})
    assert ready is False
    assert reason == 'waiting_local_activation'


def test_cd_readiness_requires_shared_activation_after_handoff():
    ready, reason = phase_aware_nav2_readiness(
        {'robot1', 'robot2'}, True, {'robot1'})
    assert ready is False
    assert reason == 'waiting_shared_nav2_activation'


def test_cd_readiness_ignores_expected_local_teardown_after_latch():
    # The phase state contains only the latched evidence; disappearance of
    # local lifecycle services after handoff cannot revoke it.
    ready, reason = phase_aware_nav2_readiness(
        {'robot1', 'robot2'}, True, {'robot1', 'robot2'})
    assert ready is True
    assert reason == 'shared_nav2_active_after_handoff'


def test_ab_readiness_path_remains_separate_from_phase_aware_path():
    source = inspect.getsource(ReadyProbe.activate_and_check_nav2)
    assert "if self.experiment_condition in ('C', 'D')" in source
    assert 'nav2_readiness_action(' in source
    assert "nav2_readiness_node_names(\n                        self.experiment_condition)" in source


def test_runner_separates_simulation_horizon_and_wall_watchdog():
    source = inspect.getsource(fast.run)
    assert 'wall_deadline = started + max(0.0, float(configured_wall_limit))' in source
    assert 'simulation_horizon_target_s = configured_horizon' in source
    assert "termination_reason = 'SIM_TIME_COMPLETE'" in source
    assert "termination_reason = 'WALL_WATCHDOG'" in source


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
