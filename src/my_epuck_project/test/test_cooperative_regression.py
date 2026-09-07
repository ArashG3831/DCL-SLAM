import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from my_epuck_project.cooperative_regression import (
    apply_profile_defaults,
    abnormal_ros_exit_evidence,
    attempt_namespace,
    classify_attempt,
    detect_slam_filter_output_stall,
    execute_attempt,
    hold_open_artifacts_valid,
    handoff_observed,
    persisted_local_tf_readiness,
    internal_command,
    manual_rviz_command,
    mission_timeout_expired,
    parse_log_errors,
    parser,
    persist_post_completion_mission_result,
    perform_attempt,
    readiness_probe_due,
    required_graph_ready,
    run_parallel_stage,
    resolved_trial_resources,
    rviz_gui_environment,
    scoped_shutdown,
    update_progress,
    validate_cli_options,
    validate_existing_attempt,
    validate_resource_bounds,
    wait_for_attempt_supervisor,
    windows_port_pid,
)
from my_epuck_project.cooperative_regression_report import (
    _claim_value,
    _status_value,
    analyze_campaign,
)
from my_epuck_project.occupancy_map_comparison import (
    Geometry,
    OccupancyMap,
    save_map,
)
import numpy as np
import pytest


def options(tmp_path, trials=10):
    return argparse.Namespace(
        trials=trials,
        ros_domain_base=100,
        webots_port_base=23000,
        fast_mode=True,
        rendering=False,
        rviz=False,
        hold_open_after_completion=False,
        startup_timeout=90.0,
        mission_timeout=240.0,
        settling_period=4.0,
        graceful_shutdown_timeout=0.2,
        hard_shutdown_timeout=0.2,
    )


def test_filter_output_stall_requires_fresh_fixed_input(tmp_path):
    observer = tmp_path / 'observer'
    observer.mkdir()
    (observer / 'topic_health.csv').write_text(
        'robot_id,topic_name,event_sequence,ros_time_sec,stale,topic_age_s\n'
        'robot1,/robot1/scan_d500_fixed,1,10,False,0.1\n'
        'robot1,/robot1/scan_d500_slam,1,10,False,0.1\n'
        'robot1,/robot1/scan_d500_fixed,2,20,False,0.1\n'
        'robot1,/robot1/scan_d500_slam,2,20,True,9.0\n'
        'robot2,/robot2/scan_d500_fixed,1,10,True,9.0\n'
        'robot2,/robot2/scan_d500_slam,1,10,True,9.0\n',
        encoding='utf-8')
    result = detect_slam_filter_output_stall(observer)
    assert set(result) == {'robot1'}
    assert result['robot1']['first_observed_stale_ros_time_s'] == 20.0


def test_invalid_cyclonedds_endpoint_is_fatal_transport_evidence(tmp_path):
    launch_log = tmp_path / 'launch.log'
    launch_log.write_text(
        'ddsi_udp_conn_write to udp/127.0.0.1:65536 failed with retcode -3\n',
        encoding='utf-8')
    offset, matches = abnormal_ros_exit_evidence(launch_log)
    assert offset == launch_log.stat().st_size
    assert len(matches) == 1


def test_claim_report_handles_missing_claim_during_startup_failure():
    assert _claim_value({'robots': {'robot2': {'claim': None}}},
                        'robot2', 'state', 'UNKNOWN') == 'UNKNOWN'


def test_report_reads_distributed_state_when_legacy_status_is_null():
    """Interrupted final-launch artifacts remain reportable."""
    final = {'robots': {'robot1': {
        'status': None,
        'distributed_status': {'state': 0},
    }}}
    assert _status_value(final, 'robot1', 'state') == 'WAITING_FOR_INPUTS'


def test_trial_resources_are_unique_and_configurable(tmp_path):
    args = options(tmp_path)
    values = [
        attempt_namespace(args, number, 1, tmp_path)
        for number in range(1, 11)
    ]
    assert len({item.ros_domain_id for item in values}) == 10
    assert len({item.webots_port for item in values}) == 10
    assert len({item.attempt_dir for item in values}) == 10
    assert all(item.webots_mode == 'fast' for item in values)
    assert all(item.webots_gui is False for item in values)
    assert all(item.launch_rviz is False for item in values)


def test_headless_profile_selects_full_sensors_and_no_gui():
    args = parser().parse_args([])
    from my_epuck_project.cooperative_regression import apply_execution_profile
    apply_execution_profile(args)
    assert args.execution_profile == 'headless'
    assert args.rendering is False
    assert args.rviz is False
    assert args.sensor_profile == 'full'


def test_authoritative_launch_contract_never_embeds_rviz():
    """The authoritative launch hop keeps RViz strictly opt-in."""
    source = (
        Path(__file__).resolve().parents[1]
        / 'launch' / 'two_robots_decentralized_exploration_launch.py'
    ).read_text(encoding='utf-8')
    assert "DeclareLaunchArgument('launch_rviz', default_value='false'" in source
    assert "launch_rviz:=false" not in source


def test_rviz_profile_selects_full_sensors_without_webots_rendering():
    args = parser().parse_args(['--execution-profile', 'rviz'])
    from my_epuck_project.cooperative_regression import apply_execution_profile
    apply_execution_profile(args)
    assert args.execution_profile == 'rviz'
    assert args.rendering is False
    assert args.rviz is True
    assert args.sensor_profile == 'full'


def test_slam_diagnostic_overrides_default_false_and_cross_trial_boundary(
        tmp_path):
    """SLAM experiment switches are explicit and survive the supervisor hop."""
    defaults = parser().parse_args([])
    assert defaults.use_scan_matching is False
    assert defaults.do_loop_closing is False

    args = parser().parse_args([
        '--use-scan-matching', 'true', '--do-loop-closing', 'false'])
    assert args.use_scan_matching is True
    assert args.do_loop_closing is False

    args = options(tmp_path, trials=1)
    args.world_profile = 'large'
    args.source_world_path = str(tmp_path / 'large.wbt')
    args.use_scan_matching = True
    args.do_loop_closing = False
    namespace = attempt_namespace(args, 1, 1, tmp_path)
    command = internal_command(namespace)
    assert command[command.index('--use-scan-matching') + 1] == 'true'
    assert command[command.index('--do-loop-closing') + 1] == 'false'


def test_scan_input_reliability_crosses_trial_boundary(tmp_path):
    args = parser().parse_args(['--scan-input-reliability', 'best_effort'])
    assert args.scan_input_reliability == 'best_effort'
    args = options(tmp_path, trials=1)
    args.world_profile = 'large'
    args.source_world_path = str(tmp_path / 'large.wbt')
    args.scan_input_reliability = 'best_effort'
    namespace = attempt_namespace(args, 1, 1, tmp_path)
    command = internal_command(namespace)
    assert command[command.index('--scan-input-reliability') + 1] == 'best_effort'


def test_ideal_encoder_sensing_defaults_true_and_crosses_trial_boundary(tmp_path):
    args = parser().parse_args([])
    assert args.ideal_encoder_sensing is True

    args = options(tmp_path, trials=1)
    args.world_profile = 'large'
    args.source_world_path = str(tmp_path / 'large.wbt')
    args.ideal_encoder_sensing = True
    namespace = attempt_namespace(args, 1, 1, tmp_path)
    command = internal_command(namespace)
    assert command[command.index('--ideal-encoder-sensing') + 1] == 'true'


def test_throughput_profile_is_explicit_reduced_sensor_experiment():
    args = parser().parse_args(['--execution-profile', 'throughput'])
    from my_epuck_project.cooperative_regression import apply_execution_profile
    apply_execution_profile(args)
    assert args.execution_profile == 'throughput'
    assert args.rendering is False
    assert args.rviz is False
    assert args.sensor_profile == 'throughput'


@pytest.mark.parametrize('base', [0, 230])
def test_ros_domain_boundary_values_are_valid(base):
    args = parser().parse_args([
        '--trials', '1', '--ros-domain-base', str(base)])
    validate_resource_bounds(args)
    assert resolved_trial_resources(args, 1)[0] == base


@pytest.mark.parametrize('option,value', [
    ('--ros-domain-base', '-1'),
    ('--ros-domain-base', '231'),
])
def test_invalid_ros_domain_is_rejected_before_probe(option, value):
    args = parser().parse_args([option, value])
    with pytest.raises(SystemExit, match='ros-domain-base'):
        validate_resource_bounds(args)


def test_ros_domain_range_exhaustion_is_rejected():
    args = parser().parse_args([
        '--trials', '2', '--ros-domain-base', '230'])
    with pytest.raises(SystemExit, match='exceeds supported maximum'):
        validate_resource_bounds(args)


def test_retry_reuses_trial_resource_without_collision(tmp_path):
    args = options(tmp_path, trials=2)
    first = attempt_namespace(args, 1, 1, tmp_path)
    retry = attempt_namespace(args, 1, 2, tmp_path)
    second = attempt_namespace(args, 2, 1, tmp_path)
    assert (first.ros_domain_id, first.webots_port) == (
        retry.ros_domain_id, retry.webots_port)
    assert (retry.ros_domain_id, retry.webots_port) != (
        second.ros_domain_id, second.webots_port)


def test_supervisor_missing_classification_becomes_infrastructure_failure(
        monkeypatch, tmp_path):
    """A prematurely exited trial supervisor returns a stable classification."""
    attempt = tmp_path / 'attempt'
    attempt.mkdir()
    (attempt / 'runner_metadata.json').write_text(json.dumps({
        'attempt_id': 'trial_01_attempt_01',
    }))

    class Process:
        returncode = 1

        def wait(self, timeout=None):
            del timeout
            return self.returncode

    monkeypatch.setattr(subprocess, 'Popen', lambda *args, **kwargs: Process())
    namespace = argparse.Namespace(
        attempt_dir=str(attempt), attempt_id='trial_01_attempt_01')
    result = execute_attempt(namespace)
    assert result['classification'] == 'INFRASTRUCTURE_FAILURE'
    assert result['cleanup_complete'] is False


def test_windows_port_pid_timeout_is_nonfatal(monkeypatch):
    """Slow Windows PID telemetry cannot terminate a healthy trial."""

    def timeout(*args, **kwargs):
        del args, kwargs
        raise subprocess.TimeoutExpired('netstat.exe', 10)

    monkeypatch.setattr(
        'my_epuck_project.cooperative_regression.run', timeout)
    monkeypatch.setattr(Path, 'exists', lambda self: True)
    assert windows_port_pid(23100) is None


def test_manual_rviz_uses_installed_cooperative_config():
    """The supervisor launches RViz with the installed project preset."""
    command = manual_rviz_command()
    assert command[0:2] == ['rviz2', '-d']
    assert command[2].endswith(
        '/resource/cooperative_manual_exploration.rviz')
    assert Path(command[2]).is_file()
    assert command[-2:] == ['-p', 'use_sim_time:=false']
    assert manual_rviz_command('large', use_sim_time=True)[-2:] == [
        '-p', 'use_sim_time:=true']


def test_rviz_gui_environment_selects_wslg_xcb_without_software_gl():
    """The WSLg RViz process uses the working XCB/GL path."""
    environment = {
        'DISPLAY': ':0',
        'WAYLAND_DISPLAY': 'wayland-0',
        'LIBGL_ALWAYS_SOFTWARE': '1',
    }
    configured = rviz_gui_environment(environment)
    assert configured['QT_QPA_PLATFORM'] == 'xcb'
    assert configured['QT_X11_NO_MITSHM'] == '1'
    assert 'LIBGL_ALWAYS_SOFTWARE' not in configured
    assert environment['LIBGL_ALWAYS_SOFTWARE'] == '1'


def test_external_rviz_is_started_after_readiness_and_has_explicit_time():
    """The runner owns RViz startup after infrastructure readiness."""
    source = (
        Path(__file__).resolve().parents[1]
        / 'my_epuck_project' / 'cooperative_regression.py'
    ).read_text(encoding='utf-8')
    assert "'rviz_start_reason': 'clock_and_stack_readiness'" in source
    assert "use_sim_time=args.time_mode == 'sim'" in source
    assert 'if clock_ok and tf_ok and not ready:' in source
    assert (
        "if status.get('ready') and not metadata['mission_inputs_ready']"
        in source
    )


def test_graph_readiness_does_not_require_frontier_claims():
    """A live graph is infrastructure-ready before either claim exists."""
    nodes = [
        '/robot1/distributed_frontier_assignment',
        '/robot2/distributed_frontier_assignment',
        '/robot1/map_fusion',
        '/robot2/map_fusion',
    ]
    assert required_graph_ready(nodes)
    assert not required_graph_ready(nodes[:-1])


def test_unknown_pose_graph_readiness_uses_local_prehandoff_nodes():
    nodes = [
        '/robot1/local_distributed_frontier_assignment',
        '/robot2/local_distributed_frontier_assignment',
        '/robot1/unknown_pose_frontend',
        '/robot2/unknown_pose_frontend',
    ]
    assert required_graph_ready(nodes, unknown_initial_pose=True)
    assert not required_graph_ready(nodes[:-1], unknown_initial_pose=True)


def test_runner_profile_defaults_and_rviz_selection():
    """New runs default large while explicit small retains proven timeouts."""
    large = apply_profile_defaults(parser().parse_args([]))
    assert large.world_profile == 'large'
    assert large.startup_timeout == 300.0
    assert large.mission_timeout == 1800.0
    assert large.shift_window == 1
    small = apply_profile_defaults(
        parser().parse_args(['--world-profile', 'small']))
    assert small.startup_timeout == 120.0
    assert small.mission_timeout == 240.0
    assert small.shift_window == 3
    large_rviz = manual_rviz_command('large')
    assert large_rviz[:2] == ['rviz2', '-d']
    assert Path(large_rviz[2]).is_file()
    assert 'large.rviz' in large_rviz[2]


def test_world_profile_reaches_internal_trial_command(tmp_path):
    """The selected profile and source world cross the supervisor boundary."""
    args = options(tmp_path, trials=1)
    args.world_profile = 'large'
    args.source_world_path = str(tmp_path / 'large.wbt')
    namespace = attempt_namespace(args, 1, 1, tmp_path)
    command = internal_command(namespace)
    assert command[command.index('--world-profile') + 1] == 'large'
    assert command[command.index('--source-world-path') + 1] == (
        args.source_world_path)


def test_local_path_gate_mode_is_required_and_crosses_trial_boundary(tmp_path):
    missing = parser().parse_args([])
    with pytest.raises(SystemExit, match='local-path-gate-mode'):
        validate_cli_options(missing)
    args = options(tmp_path, trials=1)
    args.local_path_gate_mode = 'MODE_B'
    args.world_profile = 'large'
    args.source_world_path = str(tmp_path / 'large.wbt')
    namespace = attempt_namespace(args, 1, 1, tmp_path)
    command = internal_command(namespace)
    index = command.index('--local-path-gate-mode')
    assert command[index + 1] == 'MODE_B'


def test_hold_open_defaults_false_and_normal_launch_timeout_stays_enabled(
        tmp_path):
    """Runner trials defer the mission budget until readiness is complete."""
    assert parser().parse_args([]).hold_open_after_completion is False
    args = options(tmp_path, trials=1)
    namespace = attempt_namespace(args, 1, 1, tmp_path)
    command = internal_command(namespace)
    assert '--hold-open-after-completion' in command
    index = command.index('--hold-open-after-completion')
    assert command[index + 1] == 'false'
    source = (
        Path(__file__).resolve().parents[1]
        / 'my_epuck_project' / 'cooperative_regression.py'
    ).read_text(encoding='utf-8')
    assert "'enable_mission_timeout:=false'" in source


def test_campaign_launch_defers_local_nav2_until_readiness():
    """The gate starts local Nav2 after controller/TF readiness."""
    source = (
        Path(__file__).resolve().parents[1]
        / 'my_epuck_project' / 'cooperative_regression.py'
    ).read_text(encoding='utf-8')
    assert "'nav2_autostart:=false'" in source


@pytest.mark.parametrize('trials,concurrency', [(2, 1), (1, 2)])
def test_hold_open_requires_one_trial_and_concurrency_one(
        trials, concurrency):
    """Hold-open cannot be used by multi-trial or parallel campaigns."""
    args = parser().parse_args([
        '--hold-open-after-completion', 'true',
        '--rendering', 'true',
        '--trials', str(trials),
        '--maximum-concurrency', str(concurrency),
    ])
    with pytest.raises(SystemExit, match='requires --trials 1'):
        validate_cli_options(args)


def test_hold_open_requires_rendering():
    """A hidden Webots instance cannot enter manual inspection mode."""
    args = parser().parse_args([
        '--hold-open-after-completion', 'true',
        '--trials', '1', '--maximum-concurrency', '1',
        '--rendering', 'false',
    ])
    with pytest.raises(SystemExit, match='requires --rendering true'):
        validate_cli_options(args)


def test_mission_timeout_stops_after_verified_completion():
    """The wall deadline applies before completion, never during hold."""
    assert mission_timeout_expired(11.0, 10.0) is True
    assert mission_timeout_expired(
        11.0, 10.0, completion_verified=True) is False
    assert mission_timeout_expired(
        1e9, None, completion_verified=True) is False


def test_hold_active_does_not_signal_shutdown_immediately(
        monkeypatch, tmp_path, capsys):
    """Completion notification alone sends no signal to the trial group."""
    attempt = tmp_path / 'attempt'
    attempt.mkdir()
    (attempt / 'runner_metadata.json').write_text(
        json.dumps({'hold_open_active': True}))

    class Process:
        pid = 24680
        returncode = 0
        calls = 0

        def wait(self, timeout=None):
            del timeout
            self.calls += 1
            if self.calls == 1:
                raise subprocess.TimeoutExpired('supervisor', 0.2)
            return self.returncode

    namespace = argparse.Namespace(
        hold_open_after_completion=True, launch_rviz=False)
    monkeypatch.setattr(
        'my_epuck_project.cooperative_regression.os.killpg',
        lambda *unused: pytest.fail(f'unexpected signal: {unused}'))
    assert wait_for_attempt_supervisor(Process(), namespace, attempt) == 0
    assert 'Mission complete.' in capsys.readouterr().out


def test_hold_ctrl_c_forwards_one_scoped_sigint_and_finishes(
        monkeypatch, tmp_path):
    """One Ctrl+C reaches only the isolated supervisor process group."""
    attempt = tmp_path / 'attempt'

    class Process:
        pid = 13579
        returncode = 0
        calls = 0

        def wait(self, timeout=None):
            del timeout
            self.calls += 1
            if self.calls == 1:
                raise KeyboardInterrupt
            return self.returncode

    sent = []
    monkeypatch.setattr(
        'my_epuck_project.cooperative_regression.os.killpg',
        lambda pid, signum: sent.append((pid, signum)))
    monkeypatch.setattr(
        'my_epuck_project.cooperative_regression._terminate_attempt_descendants',
        lambda *unused: None)
    namespace = argparse.Namespace(
        hold_open_after_completion=True, launch_rviz=False)
    assert wait_for_attempt_supervisor(Process(), namespace, attempt) == 0
    assert sent == [(13579, signal.SIGINT)]


def test_watchdog_sigterm_enters_exact_cleanup(monkeypatch, tmp_path):
    """A watchdog SIGTERM must not orphan the attempt's child sessions."""
    attempt = tmp_path / 'attempt'
    attempt.mkdir()

    class Process:
        pid = 13579
        returncode = 0
        calls = 0

        def wait(self, timeout=None):
            del timeout
            self.calls += 1
            if self.calls == 1:
                threading.Timer(
                    0.01, lambda: os.kill(os.getpid(), signal.SIGTERM)
                ).start()
                time.sleep(0.05)
                raise subprocess.TimeoutExpired('supervisor', 0.2)
            return self.returncode

    sent = []
    cleaned = []
    monkeypatch.setattr(
        'my_epuck_project.cooperative_regression.os.killpg',
        lambda pid, signum: sent.append((pid, signum)))
    monkeypatch.setattr(
        'my_epuck_project.cooperative_regression._terminate_attempt_descendants',
        lambda path, pid: cleaned.append((path, pid)))
    namespace = argparse.Namespace(
        hold_open_after_completion=False, launch_rviz=False)
    assert wait_for_attempt_supervisor(Process(), namespace, attempt) == 0
    assert sent == [(13579, signal.SIGINT)]
    assert cleaned == [(attempt, 13579)]


def test_single_campaign_runs_in_main_thread(monkeypatch, tmp_path):
    """Concurrency one must preserve process-wide watchdog signal handling."""
    (tmp_path / 'attempts').mkdir()
    progress = {'attempts': [], 'valid_trials': {}, 'adaptive_reductions': []}
    calls = []

    def fake_attempt(args, campaign, state, number, retry=True):
        calls.append((number, retry))
        return {'classification': 'PASS', 'wall_time_s': 0.0}

    monkeypatch.setattr(
        'my_epuck_project.cooperative_regression.perform_attempt',
        fake_attempt)
    args = options(tmp_path, trials=1)
    args.no_infrastructure_retry = False
    assert run_parallel_stage(args, tmp_path, progress, [1], 1) == 1
    assert calls == [(1, True)]


def test_failure_before_completion_is_not_held(monkeypatch, tmp_path):
    """A failed supervisor returns immediately without entering a hold."""

    class Process:
        pid = 97531
        returncode = 7

        def wait(self, timeout=None):
            del timeout
            return self.returncode

    namespace = argparse.Namespace(
        hold_open_after_completion=True, launch_rviz=False)
    monkeypatch.setattr(
        'my_epuck_project.cooperative_regression.os.killpg',
        lambda *unused: pytest.fail(f'unexpected signal: {unused}'))
    assert wait_for_attempt_supervisor(Process(), namespace, tmp_path) == 7


def test_infrastructure_failure_retried_once(monkeypatch, tmp_path):
    campaign = tmp_path
    (campaign / 'attempts').mkdir()
    progress = {
        'attempts': [], 'valid_trials': {}, 'adaptive_reductions': []}
    outcomes = iter(['INFRASTRUCTURE_FAILURE', 'SYSTEM_FAILURE'])

    def fake_execute(namespace):
        Path(namespace.attempt_dir).mkdir()
        return {'classification': next(outcomes), 'wall_time_s': 1.0}

    monkeypatch.setattr(
        'my_epuck_project.cooperative_regression.execute_attempt',
        fake_execute)
    result = perform_attempt(
        options(tmp_path), campaign, progress, 1)
    assert result['classification'] == 'SYSTEM_FAILURE'
    assert len(progress['attempts']) == 2
    assert progress['infrastructure_retries'] == 1
    assert progress['valid_trials']['trial_01'].endswith('attempt_02')


def test_hold_open_infrastructure_failure_is_not_retried(
        monkeypatch, tmp_path):
    """A manual GUI attempt does not silently consume another two minutes."""
    (tmp_path / 'attempts').mkdir()
    progress = {
        'attempts': [], 'valid_trials': {}, 'adaptive_reductions': []}
    calls = []

    def fake_execute(namespace):
        calls.append(namespace.attempt_id)
        Path(namespace.attempt_dir).mkdir()
        return {
            'classification': 'INFRASTRUCTURE_FAILURE',
            'wall_time_s': 1.0,
        }

    args = options(tmp_path, trials=1)
    args.hold_open_after_completion = True
    monkeypatch.setattr(
        'my_epuck_project.cooperative_regression.execute_attempt',
        fake_execute)
    with pytest.raises(
            RuntimeError, match='not retried automatically'):
        perform_attempt(args, tmp_path, progress, 1)
    assert calls == ['trial_01_attempt_01']
    assert len(progress['attempts']) == 1
    assert progress.get('infrastructure_retries', 0) == 0


def test_rviz_start_is_announced_before_readiness(
        monkeypatch, tmp_path, capsys):
    """The parent reports the optional GUI without requiring stack readiness."""
    attempt = tmp_path / 'attempt'
    attempt.mkdir()
    (attempt / 'runner_metadata.json').write_text(
        json.dumps({'rviz_pid': 24680}))

    class Process:
        pid = 13579
        returncode = 0
        calls = 0

        def wait(self, timeout=None):
            del timeout
            self.calls += 1
            if self.calls == 1:
                raise subprocess.TimeoutExpired('supervisor', 0.2)
            return self.returncode

    namespace = argparse.Namespace(
        hold_open_after_completion=False, launch_rviz=True)
    monkeypatch.setattr(
        'my_epuck_project.cooperative_regression.os.killpg',
        lambda *unused: pytest.fail(f'unexpected signal: {unused}'))
    assert wait_for_attempt_supervisor(Process(), namespace, attempt) == 0
    assert 'RVIZ_STARTED pid=24680' in capsys.readouterr().out


def test_system_failure_is_not_retried(monkeypatch, tmp_path):
    (tmp_path / 'attempts').mkdir()
    progress = {
        'attempts': [], 'valid_trials': {}, 'adaptive_reductions': []}

    def fake_execute(namespace):
        Path(namespace.attempt_dir).mkdir()
        return {'classification': 'SYSTEM_FAILURE', 'wall_time_s': 1.0}

    monkeypatch.setattr(
        'my_epuck_project.cooperative_regression.execute_attempt',
        fake_execute)
    perform_attempt(options(tmp_path), tmp_path, progress, 1)
    assert len(progress['attempts']) == 1


def test_atomic_progress_writing(tmp_path):
    progress = {
        'attempts': [], 'valid_trials': {}, 'adaptive_reductions': []}
    update_progress(tmp_path, progress)
    assert json.loads(
        (tmp_path / 'campaign_progress.json').read_text())['attempts'] == []
    assert not list(tmp_path.glob('*.tmp'))


def test_scoped_graceful_shutdown_does_not_touch_unrelated_process():
    target = subprocess.Popen(
        [sys.executable, '-c', 'import time; time.sleep(30)'])
    unrelated = subprocess.Popen(
        [sys.executable, '-c', 'import time; time.sleep(30)'])
    try:
        result = scoped_shutdown([target], 1.0, 1.0)
        assert result['all_exited']
        assert unrelated.poll() is None
    finally:
        if unrelated.poll() is None:
            unrelated.terminate()
            unrelated.wait()


def test_hard_shutdown_fallback_is_bounded():
    target = subprocess.Popen([
        sys.executable, '-c',
        'import signal,time;'
        'signal.signal(signal.SIGINT,signal.SIG_IGN);'
        'signal.signal(signal.SIGTERM,signal.SIG_IGN);'
        'time.sleep(30)',
    ])
    time.sleep(0.1)
    result = scoped_shutdown([target], 0.2, 0.2)
    assert result['kill_required']
    assert result['all_exited']


def test_readiness_probe_stops_after_full_readiness():
    """A later transient TF probe cannot overwrite achieved readiness."""
    assert readiness_probe_due('sim', False, 6.0, 0.0)
    assert not readiness_probe_due('sim', True, 600.0, 0.0)
    assert not readiness_probe_due('wall', False, 6.0, 0.0)


def test_bounded_diagnostic_has_successful_supervisor_exit():
    """Bounded diagnostics are valid non-crash attempts for the supervisor."""
    source = (Path(__file__).parents[1] / 'my_epuck_project' /
              'cooperative_regression.py').read_text()
    assert 'NON_FAILURE_CLASSIFICATIONS' in source
    assert "'BOUNDED_DIAGNOSTIC'" in source


def test_collector_shutdown_isolated_and_precedes_launch_shutdown():
    """The collector can finish its buffered snapshot before ROS teardown."""
    source = (Path(__file__).parents[1] / 'my_epuck_project' /
              'cooperative_regression.py').read_text()
    assert 'preexec_fn=os.setpgrp' in source
    collector_signal = source.index(
        '_send_scope([collector], signal.SIGINT')
    launch_signal = source.index(
        'signal_process(launch, signal.SIGINT)')
    assert collector_signal < launch_signal
    assert 'process_group=collector.pid' in source


def test_project_nodes_handle_external_shutdown_without_invalid_publish():
    """Project nodes treat global ROS shutdown as normal process termination."""
    root = Path(__file__).parents[1] / 'my_epuck_project'
    for name in (
            'cooperative_experiment_logger.py',
            'cooperative_trial_collector.py', 'd500_scan_fix.py',
            'twist_stamper.py', 'teammate_scan_filter.py', 'map_exporter.py',
            'source_aware_map_fusion.py'):
        source = (root / name).read_text()
        assert 'ExternalShutdownException' in source
        assert 'if rclpy.ok()' in source or 'if node.context.ok()' in source


def test_campaign_report_does_not_count_bounded_diagnostic_as_failure(tmp_path):
    """A valid bounded diagnostic remains visible without failing campaign."""
    trial, attempt = create_attempt(
        tmp_path, 1, classification='BOUNDED_DIAGNOSTIC')
    (tmp_path / 'campaign_progress.json').write_text(json.dumps({
        'valid_trials': {trial: str(attempt.relative_to(tmp_path))},
    }))
    result = analyze_campaign(tmp_path)
    assert result['failure_count'] == 0
    assert result['diagnostic_count'] == 1


def test_campaign_report_does_not_require_shared_maps_without_handoff(tmp_path):
    """A bounded no-handoff run reports missing shared maps as not applicable."""
    trial, attempt = create_attempt(
        tmp_path, 1, classification='BOUNDED_DIAGNOSTIC')
    (attempt / 'robot1_final_shared_map.npz').unlink()
    (attempt / 'robot2_final_shared_map.npz').unlink()
    (tmp_path / 'campaign_progress.json').write_text(json.dumps({
        'valid_trials': {trial: str(attempt.relative_to(tmp_path))},
    }))
    (tmp_path / 'campaign_manifest.json').write_text(json.dumps({
        'campaign_id': 'no_handoff', 'git_commit': 'abc',
        'trial_count_requested': 1,
    }))
    result = analyze_campaign(tmp_path)
    assert result['shared_map_analysis'] == 'NOT_APPLICABLE_NO_HANDOFF'
    assert result['failure_count'] == 0
    assert (tmp_path / 'campaign_report.md').is_file()
    assert not (attempt / 'robot1_final_shared_map.npz').exists()


def test_runner_does_not_require_shared_maps_without_handoff(tmp_path):
    """Runner validation accepts a complete local-only diagnostic attempt."""
    trial, attempt = create_attempt(
        tmp_path, 1, classification='BOUNDED_DIAGNOSTIC')
    (attempt / 'robot1_final_shared_map.npz').unlink()
    (attempt / 'robot2_final_shared_map.npz').unlink()
    metadata = json.loads(
        (attempt / 'runner_metadata.json').read_text(encoding='utf-8'))
    assert handoff_observed(attempt, metadata['run_id']) is False
    assert validate_existing_attempt(attempt)


def test_runner_requires_shared_maps_after_handoff(tmp_path):
    """A recorded handoff retains the shared-map artifact contract."""
    trial, attempt = create_attempt(
        tmp_path, 1, classification='BOUNDED_DIAGNOSTIC')
    (attempt / 'robot1_final_shared_map.npz').unlink()
    (attempt / 'robot2_final_shared_map.npz').unlink()
    frontend = attempt / 'observer' / 'frontend'
    frontend.mkdir(parents=True, exist_ok=True)
    (frontend / 'robot1_unknown_pose_frontend.json').write_text(
        json.dumps({'counters': {'tf_handoffs': 1}}), encoding='utf-8')
    assert handoff_observed(attempt, attempt.name) is True
    assert not validate_existing_attempt(attempt)


def test_persisted_tf_readiness_requires_all_known_pose_chains(tmp_path):
    """Known-pose fallback requires the two odom and shared-map chains."""
    attempt = tmp_path / 'attempt'
    path = attempt / 'observer' / 'run' / 'forensic'
    path.mkdir(parents=True)
    (path / 'transforms.csv').write_text(
        'target_frame,source_frame,available\n'
        'robot1/base_footprint,robot1/odom,True\n'
        'robot2/base_footprint,robot2/odom,True\n', encoding='utf-8')
    ready, details = persisted_local_tf_readiness(attempt, 'run')
    assert not ready
    assert len(details['available_transforms']) == 2
    with (path / 'transforms.csv').open('a', encoding='utf-8') as stream:
        stream.write(
            'shared_map,robot1/base_footprint,True\n'
            'shared_map,robot2/base_footprint,True\n')
    ready, details = persisted_local_tf_readiness(attempt, 'run')
    assert ready
    assert details['reason'] == 'READY'


def test_persisted_tf_readiness_unknown_pose_requires_local_map_chains(tmp_path):
    """Unknown-pose fallback must not accept odometry-only readiness."""
    attempt = tmp_path / 'attempt'
    path = attempt / 'observer' / 'run' / 'forensic'
    path.mkdir(parents=True)
    (path / 'transforms.csv').write_text(
        'target_frame,source_frame,available\n'
        'robot1/base_footprint,robot1/odom,True\n'
        'robot2/base_footprint,robot2/odom,True\n', encoding='utf-8')
    ready, details = persisted_local_tf_readiness(
        attempt, 'run', unknown_initial_pose=True)
    assert not ready
    assert {item['role'] for item in details['available_transforms']} == {
        'odom_to_base'}
    with (path / 'transforms.csv').open('a', encoding='utf-8') as stream:
        stream.write(
            'robot1/map,robot1/base_footprint,True\n'
            'robot2/map,robot2/base_footprint,True\n')
    ready, details = persisted_local_tf_readiness(
        attempt, 'run', unknown_initial_pose=True)
    assert ready
    assert details['reason'] == 'READY'


def test_campaign_exit_requires_mission_completion_not_only_clean_cleanup():
    """A clean bounded diagnostic must not have success exit semantics."""
    source = (Path(__file__).parents[1] / 'my_epuck_project' /
              'cooperative_regression.py').read_text()
    return_block = source[source.index('return 0 if ('):source.index(
        '\n\n\ndef boolean', source.index('return 0 if ('))]
    assert "summary['mission_completion_rate'] == 1.0" in return_block


def create_attempt(campaign, number, classification='PASS', value=0):
    trial = f'trial_{number:02d}'
    name = f'{trial}_attempt_01'
    attempt = campaign / 'attempts' / name
    observer = attempt / 'observer' / name
    observer.mkdir(parents=True)
    geometry = Geometry(20, 15, 0.01, -0.1, -0.1)
    data = np.zeros((15, 20), dtype=np.int8)
    data[:, 10 + (value % 2)] = 100
    data[:value % 3, :2] = -1
    metadata = {
        'frame_id': 'shared_map',
        'validation_errors': [],
        'run_id': name,
    }
    item = OccupancyMap(data, geometry, metadata)
    for robot in ('robot1', 'robot2'):
        saved = save_map(
            attempt / f'{robot}_final_shared_map.npz', item)
        (attempt / f'{robot}_final_shared_map_metadata.json').write_text(
            json.dumps(saved))
    final = {
        'robots': {
            robot: {
                'status': {
                    'state': 'MISSION_COMPLETE', 'reason': 'consensus',
                    'age_at_write_s': 0.1,
                },
                'claim': {
                    'state': 'RELEASED', 'claim_id': number,
                    'reserving': False,
                },
                'navigation_active': False,
            } for robot in ('robot1', 'robot2')
        }
    }
    (attempt / 'final_state.json').write_text(json.dumps(final))
    summary = {
        'mission_completion_time_s': float(10 + number),
        'navigation': {
            'goals_sent': number, 'successes': number,
            'failures': 0, 'recoveries': 0,
        },
        'mapping': {'duplicated_known_fraction': 0.1},
        'motion': {
            'robot1': {'distance_travelled_m': 1.0},
            'robot2': {'distance_travelled_m': 1.0},
        },
        'system': {
            'internal_logger_error_count': 0, 'write_failures': 0,
        },
    }
    (observer / 'summary.json').write_text(json.dumps(summary))
    for filename in (
            'events.jsonl', 'warnings.jsonl', 'topic_health.csv',
            'coverage.csv', 'robot1_timeseries.csv', 'robot2_timeseries.csv'):
        (observer / filename).write_text('')
    runner = {
        'attempt_id': name, 'run_id': name,
        'classification': classification, 'wall_time_s': 20 + number,
        'observer_summary_path': f'observer/{name}/summary.json',
        'process_exit_codes': {'launch': 0, 'collector': 0},
        'cleanup_complete': True,
    }
    (attempt / 'runner_metadata.json').write_text(json.dumps(runner))
    return trial, attempt


def test_hold_open_requires_valid_final_completion_artifacts(tmp_path):
    """Only settled complete, idle, released robot states may be held."""
    _, attempt = create_attempt(tmp_path, 1)
    assert hold_open_artifacts_valid(attempt)
    final_path = attempt / 'final_state.json'
    final = json.loads(final_path.read_text())
    final['robots']['robot2']['claim']['reserving'] = True
    final_path.write_text(json.dumps(final))
    assert not hold_open_artifacts_valid(attempt)


def test_timeout_classification_is_valid_failure(tmp_path):
    trial, attempt = create_attempt(tmp_path, 1)
    classification, _ = classify_attempt(
        attempt, f'{trial}_attempt_01', True, True, False, False,
        {'all_exited': True})
    assert classification == 'BOUNDED_DIAGNOSTIC'


def test_requested_shutdown_traceback_is_not_a_runtime_crash(tmp_path):
    log = tmp_path / 'launch.log'
    log.write_text(
        '[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)\n'
        'Traceback (most recent call last):\n'
        'rclpy.executors.ExternalShutdownException\n')
    result = parse_log_errors(log)
    assert result['traceback'] is False
    assert result['shutdown_traceback_count'] == 1


def test_collection_trigger_age_precedes_slow_artifact_write(tmp_path):
    trial, attempt = create_attempt(tmp_path, 1)
    final_path = attempt / 'final_state.json'
    final = json.loads(final_path.read_text())
    for robot in ('robot1', 'robot2'):
        final['robots'][robot]['status']['age_at_write_s'] = 12.0
        final['robots'][robot]['status']['age_at_collection_s'] = 3.9
    final_path.write_text(json.dumps(final))
    classification, _ = classify_attempt(
        attempt, f'{trial}_attempt_01', True, False, False, False,
        {'all_exited': True})
    assert classification == 'MISSION_COMPLETE'


def test_settled_observation_proves_freshness_for_legacy_artifact(tmp_path):
    trial, attempt = create_attempt(tmp_path, 1)
    final_path = attempt / 'final_state.json'
    final = json.loads(final_path.read_text())
    for robot in ('robot1', 'robot2'):
        final['robots'][robot]['status']['age_at_write_s'] = 12.0
    final_path.write_text(json.dumps(final))
    classification, _ = classify_attempt(
        attempt, f'{trial}_attempt_01', True, False, False, False,
        {'all_exited': True}, settled_observed=True)
    assert classification == 'MISSION_COMPLETE'


def test_post_completion_collector_evidence_preserves_success_without_logger_artifacts(
        tmp_path):
    """Settled distributed completion is not downgraded by logger teardown."""
    trial, attempt = create_attempt(tmp_path, 1)
    observer = attempt / 'observer' / f'{trial}_attempt_01'
    (observer / 'summary.json').unlink()
    (observer / 'warnings.jsonl').unlink()
    final_path = attempt / 'final_state.json'
    final = json.loads(final_path.read_text())
    for robot in ('robot1', 'robot2'):
        final['robots'][robot].pop('status')
        final['robots'][robot]['distributed_status'] = {
            'state': 5, 'reason': 'MISSION_COMPLETE_NO_FRONTIERS',
        }
    final_path.write_text(json.dumps(final))
    (attempt / 'collector_status.json').write_text(json.dumps({
        'both_mission_complete': True, 'settled': True,
    }))
    classification, details = classify_attempt(
        attempt, f'{trial}_attempt_01', True, False, False, False,
        {'all_exited': True}, settled_observed=True)
    assert classification == 'MISSION_COMPLETE'
    assert details['post_completion_artifact_anomaly']['source'] == (
        'collector_status.json + final_state.json')


def test_post_completion_fallback_does_not_accept_timeout(tmp_path):
    """The completion fallback cannot turn a bounded timeout into success."""
    trial, attempt = create_attempt(tmp_path, 1)
    (attempt / 'collector_status.json').write_text(json.dumps({
        'both_mission_complete': True, 'settled': True,
    }))
    classification, _ = classify_attempt(
        attempt, f'{trial}_attempt_01', True, True, False, False,
        {'all_exited': True}, settled_observed=False)
    assert classification == 'BOUNDED_DIAGNOSTIC'


def test_completed_result_is_persisted_before_logger_teardown(tmp_path):
    """Collector completion is durable even if launch later kills logger."""
    _, attempt = create_attempt(tmp_path, 1)
    final_path = attempt / 'final_state.json'
    final = json.loads(final_path.read_text())
    for robot in ('robot1', 'robot2'):
        final['robots'][robot].pop('status')
        final['robots'][robot]['distributed_status'] = {
            'state': 5, 'reason': 'MISSION_COMPLETE_NO_FRONTIERS',
        }
    final_path.write_text(json.dumps(final))
    (attempt / 'collector_status.json').write_text(json.dumps({
        'both_mission_complete': True,
        'settled': True,
        'sim_time_seconds': 42.0,
        'wall_elapsed_s': 12.0,
    }))
    assert persist_post_completion_mission_result(attempt)
    result = json.loads((attempt / 'mission_result.json').read_text())
    assert result['mission_status'] == 'SUCCEEDED'
    assert result['terminal_agreement'] is True
    assert result['completion_committed_before_teardown'] is True
    assert result['recommended_exit_code'] == 0


def test_result_persistence_rejects_nonterminal_collector_state(tmp_path):
    """A collector that did not prove completion cannot create success."""
    _, attempt = create_attempt(tmp_path, 1)
    (attempt / 'collector_status.json').write_text(json.dumps({
        'both_mission_complete': False, 'settled': True,
    }))
    assert not persist_post_completion_mission_result(attempt)
    assert not (attempt / 'mission_result.json').exists()


def test_post_completion_fallback_does_not_hide_collector_crash(tmp_path):
    """A nonzero collector exit remains a post-completion failure."""
    trial, attempt = create_attempt(tmp_path, 1)
    (attempt / 'collector_status.json').write_text(json.dumps({
        'both_mission_complete': True, 'settled': True,
    }))
    classification, _ = classify_attempt(
        attempt, f'{trial}_attempt_01', True, False, False, False,
        {'all_exited': True}, settled_observed=True,
        process_exit_codes={'collector': 1, 'launch': 0})
    assert classification == 'SHUTDOWN_DEGRADED'


def test_resume_validation_rejects_partial_artifacts(tmp_path):
    _, attempt = create_attempt(tmp_path, 1)
    assert validate_existing_attempt(attempt)
    (attempt / 'robot2_final_shared_map.npz').unlink()
    assert not validate_existing_attempt(attempt)


def test_ten_trial_report_matrices_schema_and_outlier_preservation(tmp_path):
    campaign = tmp_path / 'regression_test'
    (campaign / 'attempts').mkdir(parents=True)
    valid = {}
    for number in range(1, 11):
        trial, attempt = create_attempt(
            campaign, number, value=number)
        valid[trial] = str(attempt.relative_to(campaign))
    (campaign / 'campaign_progress.json').write_text(json.dumps({
        'valid_trials': valid, 'attempts': [],
    }))
    (campaign / 'campaign_manifest.json').write_text(json.dumps({
        'campaign_id': campaign.name,
        'git_commit': 'abc',
        'trial_count_requested': 10,
        'chosen_concurrency': 2,
        'total_wall_time_s': 123.0,
        'simulation_time_measurement': 'unavailable',
        'limitations': ['simulation time unavailable'],
    }))
    summary = analyze_campaign(campaign)
    assert summary['valid_trial_count'] == 10
    assert summary['schema_version'] == '1.0.0'
    assert len((campaign / 'trials.csv').read_text().splitlines()) == 11
    assert len(
        (campaign / 'pairwise_map_metrics.csv').read_text().splitlines()
    ) == 46
    assert (campaign / 'matrices/known_iou.csv').is_file()
    report = (campaign / 'campaign_report.md').read_text()
    assert report.count('| trial_') == 10
    assert 'simulation time unavailable' in report
    assert (campaign / 'aggregate/consensus_map.npz').is_file()
    assert (campaign / 'aggregate/disagreement_frequency.npz').is_file()
