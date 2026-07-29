import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

from my_epuck_project.cooperative_regression import (
    attempt_namespace,
    classify_attempt,
    hold_open_artifacts_valid,
    internal_command,
    manual_rviz_command,
    mission_timeout_expired,
    parse_log_errors,
    parser,
    perform_attempt,
    scoped_shutdown,
    update_progress,
    validate_cli_options,
    validate_existing_attempt,
    wait_for_attempt_supervisor,
)
from my_epuck_project.cooperative_regression_report import analyze_campaign
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


def test_manual_rviz_uses_installed_cooperative_config():
    """The supervisor launches RViz with the installed project preset."""
    command = manual_rviz_command()
    assert command[0:2] == ['rviz2', '-d']
    assert command[2].endswith(
        '/resource/cooperative_manual_exploration.rviz')
    assert Path(command[2]).is_file()


def test_hold_open_defaults_false_and_normal_launch_timeout_stays_enabled(
        tmp_path):
    """Normal regression retains automatic completion shutdown."""
    assert parser().parse_args([]).hold_open_after_completion is False
    args = options(tmp_path, trials=1)
    namespace = attempt_namespace(args, 1, 1, tmp_path)
    command = internal_command(namespace)
    assert '--hold-open-after-completion' in command
    index = command.index('--hold-open-after-completion')
    assert command[index + 1] == 'false'


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

    namespace = argparse.Namespace(hold_open_after_completion=True)
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
    namespace = argparse.Namespace(hold_open_after_completion=True)
    assert wait_for_attempt_supervisor(Process(), namespace, attempt) == 0
    assert sent == [(13579, signal.SIGINT)]


def test_failure_before_completion_is_not_held(monkeypatch, tmp_path):
    """A failed supervisor returns immediately without entering a hold."""

    class Process:
        pid = 97531
        returncode = 7

        def wait(self, timeout=None):
            del timeout
            return self.returncode

    namespace = argparse.Namespace(hold_open_after_completion=True)
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
    assert classification == 'MISSION_TIMEOUT'


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
    assert classification == 'PASS'


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
    assert classification == 'PASS'


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
