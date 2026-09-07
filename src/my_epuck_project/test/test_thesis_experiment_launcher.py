"""Focused tests for the canonical launcher boundary."""

import importlib.util
import json
from pathlib import Path
import subprocess

import pytest


SOURCE = Path(__file__).parents[1] / 'tools/run_thesis_experiment.py'
SPEC = importlib.util.spec_from_file_location('thesis_launcher', SOURCE)
launcher = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(launcher)


def _env(root, **extra):
    values = {
        'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/lib',
        'WSL_INTEROP': '/run/WSL/test',
    }
    values.update(extra)
    return launcher.canonical_environment(
        values, root / 'results' / 'run', 230, 23420)


def test_nat_and_ros_vendor_environment_survive(tmp_path):
    env = _env(tmp_path)
    assert env['MY_EPUCK_WEBOTS_NETWORK_MODE'] == 'nat'
    assert '/opt/ros/jazzy/opt/sdformat_vendor/lib' in env['LD_LIBRARY_PATH']
    assert env['WSL_INTEROP'] == '/run/WSL/test'
    assert env['MY_EPUCK_WEBOTS_DRIVER_PREFIX'] == str(launcher.DRIVER_PREFIX)


def test_loopback_nat_is_rejected(tmp_path):
    env = _env(tmp_path)
    with pytest.raises(RuntimeError, match='loopback'):
        launcher.validate_launcher_values(
            env, '127.0.0.1', tmp_path / 'results' / 'run')


def test_result_root_must_be_below_results(tmp_path):
    env = _env(tmp_path)
    with pytest.raises(RuntimeError, match='result root'):
        launcher.validate_launcher_values(env, '172.18.32.1', tmp_path)


def test_stale_generated_interfaces_are_rejected(monkeypatch, tmp_path):
    monkeypatch.setattr(launcher, '_helper', lambda *args: '[]')
    with pytest.raises(RuntimeError, match='stale generated ROS interfaces'):
        launcher.validate_interface_schema(_env(tmp_path))


def test_main_passes_the_validated_environment_to_runner(monkeypatch, tmp_path):
    monkeypatch.setattr(launcher, 'WORKSPACE', tmp_path)
    monkeypatch.setattr(launcher, 'PROJECT_INSTALL', tmp_path / 'install')
    monkeypatch.setattr(launcher, 'DRIVER_PREFIX', tmp_path / 'driver')
    monkeypatch.setattr(launcher, 'source_clean_environment', lambda: {
        'LD_LIBRARY_PATH': launcher.VENDOR_LIBRARY, 'WSL_INTEROP': '/run/WSL/test'})
    monkeypatch.setattr(launcher, '_helper', lambda env, code, *args:
                        '172.18.32.1' if '_resolve_controller_host' in code
                        else json.dumps(list(('feasible_work_available', 'actionable_work_available', 'work_availability_reason', 'stationary_witness_scheme_version'))) if 'my_epuck_interfaces.msg' in code
                        else json.dumps(env))
    monkeypatch.setattr(launcher, '_ldd', lambda env: {'ldd': 'ok'})
    monkeypatch.setattr(launcher, 'existing_preflight', lambda *args: {
        'project_prefix': str(tmp_path / 'install/my_epuck_project'),
        'driver_prefix': str(tmp_path / 'driver'),
    })
    original = launcher.canonical_environment
    expected = {}

    def capture(*args):
        expected['env'] = original(*args)
        return expected['env']

    monkeypatch.setattr(launcher, 'canonical_environment', capture)
    monkeypatch.setattr(launcher.subprocess, 'check_output',
                        lambda *args, **kwargs: 'test-head')
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(launcher.subprocess, 'run', fake_run)
    assert launcher.main(['--condition', 'C', '--horizon', '1']) == 0
    child = next(item for item in calls if 'run_cooperative_trial_fast.py' in item[0][1])
    assert child[1]['env'] is expected['env']
    assert child[1]['env']['MY_EPUCK_WEBOTS_NETWORK_MODE'] == 'nat'


def test_dry_run_does_not_invoke_runner(monkeypatch, tmp_path):
    monkeypatch.setattr(launcher, 'WORKSPACE', tmp_path)
    monkeypatch.setattr(launcher, 'PROJECT_INSTALL', tmp_path / 'install')
    monkeypatch.setattr(launcher, 'DRIVER_PREFIX', tmp_path / 'driver')
    monkeypatch.setattr(launcher, 'source_clean_environment', lambda: {
        'LD_LIBRARY_PATH': launcher.VENDOR_LIBRARY, 'WSL_INTEROP': '/run/WSL/test'})
    monkeypatch.setattr(launcher, '_helper', lambda env, code, *args:
                        '172.18.32.1' if '_resolve_controller_host' in code
                        else json.dumps(list(('feasible_work_available', 'actionable_work_available', 'work_availability_reason', 'stationary_witness_scheme_version'))) if 'my_epuck_interfaces.msg' in code
                        else json.dumps(env))
    monkeypatch.setattr(launcher, '_ldd', lambda env: {'ldd': 'ok'})
    monkeypatch.setattr(launcher, 'existing_preflight', lambda *args: {
        'project_prefix': 'project', 'driver_prefix': str(tmp_path / 'driver')})
    monkeypatch.setattr(launcher.subprocess, 'check_output',
                        lambda *args, **kwargs: 'test-head')
    calls = []
    monkeypatch.setattr(launcher.subprocess, 'run',
                        lambda command, **kwargs: calls.append(command) or
                        subprocess.CompletedProcess(command, 0))
    assert launcher.main(['--condition', 'C', '--horizon', '1', '--dry-run']) == 0
    assert not any('run_cooperative_trial_fast.py' in command for command in calls)
