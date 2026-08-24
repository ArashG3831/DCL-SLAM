import json
from pathlib import Path
import inspect

import pytest

from my_epuck_project import ros_runtime_preflight as preflight
from my_epuck_project import nav2_frontier_diagnostic as diagnostic


def _fake_bound(monkeypatch, *, daemon_ok=True, webots_ok=True,
                node_return=0, topic_return=0, timed_out=False):
    def bind(port):
        if port == preflight.ROS_DAEMON_BASE_PORT + preflight.ROS_DOMAIN_MAX:
            return {'ok': daemon_ok, 'errno': 98 if not daemon_ok else None,
                    'error': 'address in use' if not daemon_ok else ''}
        if port == 23667:
            return {'ok': webots_ok, 'errno': 98 if not webots_ok else None,
                    'error': 'address in use' if not webots_ok else ''}
        return {'ok': True, 'errno': None, 'error': ''}

    def bounded(command, **kwargs):
        name = ' '.join(command)
        if 'node list' in name:
            return {'command': list(command), 'returncode': node_return,
                    'stdout': '', 'timed_out': timed_out,
                    'timeout_s': kwargs.get('timeout_s', 10.0),
                    'kill_after_s': kwargs.get('kill_after_s', 2.0)}
        if 'topic list' in name:
            return {'command': list(command), 'returncode': topic_return,
                    'stdout': '', 'timed_out': timed_out,
                    'timeout_s': kwargs.get('timeout_s', 10.0),
                    'kill_after_s': kwargs.get('kill_after_s', 2.0)}
        return {'command': list(command), 'returncode': 0, 'stdout': '',
                'timed_out': False, 'timeout_s': 10.0, 'kill_after_s': 2.0}

    monkeypatch.setattr(preflight, 'local_bind_probe', bind)
    monkeypatch.setattr(preflight, 'windows_tcp_bindings', lambda: [])
    monkeypatch.setattr(preflight, '_windows_process_context', lambda pids: [])
    monkeypatch.setattr(preflight, '_project_processes', lambda: [])
    monkeypatch.setattr(preflight, '_project_d_state_processes', lambda: [])
    monkeypatch.setattr(preflight.shutil, 'which', lambda name: '/usr/bin/ros2')
    monkeypatch.setattr(preflight, '_bounded_subprocess', bounded)


def test_cyclonedds_port_headroom_bound_is_derived_from_profile():
    assert preflight.ROS_DOMAIN_MAX == 230
    assert preflight.dds_max_unicast_port(230) <= preflight.DDS_PORT_MAX
    assert preflight.dds_max_unicast_port(231) > preflight.DDS_PORT_MAX


def test_daemon_port_conflict_with_no_daemon_graph_continues(tmp_path, monkeypatch):
    _fake_bound(monkeypatch, daemon_ok=False)
    report = preflight.run_preflight(
        ros_domain_id=preflight.ROS_DOMAIN_MAX, webots_port=23667,
        output_dir=tmp_path)
    assert report['status'] == 'SAFE_DAEMON_INDEPENDENT'
    assert report['daemon_port_bind']['ok'] is False
    assert json.loads((tmp_path / 'ros_preflight.json').read_text())['safe_daemon_independent']


@pytest.mark.parametrize('which', ['node', 'topic'])
def test_no_daemon_graph_timeout_aborts(tmp_path, monkeypatch, which):
    _fake_bound(monkeypatch, daemon_ok=False, timed_out=True)
    with pytest.raises(preflight.PreflightError):
        preflight.run_preflight(
            ros_domain_id=preflight.ROS_DOMAIN_MAX, webots_port=23667,
            output_dir=tmp_path)
    report = json.loads((tmp_path / 'ros_preflight.json').read_text())
    assert report['status'] == 'ROS_MIDDLEWARE_PREFLIGHT_FAILED'
    assert report[f'{which}_list_no_daemon']['timed_out'] is True


def test_occupied_webots_port_aborts(tmp_path, monkeypatch):
    _fake_bound(monkeypatch, webots_ok=False)
    with pytest.raises(preflight.PreflightError):
        preflight.run_preflight(
            ros_domain_id=preflight.ROS_DOMAIN_MAX, webots_port=23667,
            output_dir=tmp_path)


def test_project_d_state_aborts(tmp_path, monkeypatch):
    _fake_bound(monkeypatch)
    monkeypatch.setattr(
        preflight, '_project_d_state_processes',
        lambda: [{'pid': 99, 'status': 'disk-sleep', 'command': 'ros2 launch'}])
    with pytest.raises(preflight.PreflightError):
        preflight.run_preflight(
            ros_domain_id=preflight.ROS_DOMAIN_MAX, webots_port=23667,
            output_dir=tmp_path)


def test_domain_above_dds_port_safe_bound_is_rejected_and_reported(
        tmp_path, monkeypatch):
    _fake_bound(monkeypatch)
    with pytest.raises(preflight.PreflightError, match='between 0 and 230'):
        preflight.run_preflight(
            ros_domain_id=preflight.ROS_DOMAIN_MAX + 1,
            webots_port=23667, output_dir=tmp_path)
    assert json.loads((tmp_path / 'ros_preflight.json').read_text())[
        'ros_domain_id'] == preflight.ROS_DOMAIN_MAX + 1


def test_no_daemon_stop_command_is_not_called(tmp_path, monkeypatch):
    _fake_bound(monkeypatch, daemon_ok=False)
    commands = []
    original = preflight._bounded_subprocess

    def capture(command, **kwargs):
        commands.append(list(command))
        return original(command, **kwargs)

    # Use deterministic fake graph results, while still recording all calls.
    def fake(command, **kwargs):
        commands.append(list(command))
        name = ' '.join(command)
        if 'node list' in name or 'topic list' in name:
            return {'command': list(command), 'returncode': 0, 'stdout': '',
                    'timed_out': False, 'timeout_s': 10.0, 'kill_after_s': 2.0}
        return {'command': list(command), 'returncode': 0, 'stdout': '',
                'timed_out': False, 'timeout_s': 10.0, 'kill_after_s': 2.0}

    monkeypatch.setattr(preflight, '_bounded_subprocess', fake)
    preflight.run_preflight(ros_domain_id=preflight.ROS_DOMAIN_MAX,
                            webots_port=23667,
                            output_dir=tmp_path)
    assert not any('daemon' in item and 'stop' in item for cmd in commands for item in cmd)


def test_bounded_subprocess_records_deadlines(monkeypatch):
    calls = []

    class FakeProcess:
        pid = 12345
        returncode = 0

        def communicate(self, timeout=None):
            calls.append(timeout)
            return 'ok', None

    monkeypatch.setattr(preflight.subprocess, 'Popen', lambda *a, **k: FakeProcess())
    result = preflight._bounded_subprocess(['example'], timeout_s=1.25,
                                           kill_after_s=0.5)
    assert calls == [1.25]
    assert result['timeout_s'] == 1.25
    assert result['kill_after_s'] == 0.5


def test_standalone_runner_writes_json_and_does_not_launch(tmp_path, monkeypatch):
    args = diagnostic.runner_parser().parse_args([
        '--preflight-only', '--results-directory', str(tmp_path),
        '--ros-domain-id', str(preflight.ROS_DOMAIN_MAX),
        '--webots-port', '23667',
    ])
    world = tmp_path / 'world.wbt'
    world.write_text('WorldInfo {}\n')
    monkeypatch.setattr(diagnostic, 'package_prefix', lambda: tmp_path)
    monkeypatch.setattr(diagnostic, 'resolve_world', lambda _args: world)
    called = []

    def fake_preflight(**kwargs):
        called.append(kwargs)
        output = Path(kwargs['output_dir']) / 'ros_preflight.json'
        output.write_text(json.dumps({'status': 'SAFE_DAEMON_INDEPENDENT'}))
        return {'status': 'SAFE_DAEMON_INDEPENDENT'}

    monkeypatch.setattr(diagnostic, 'run_preflight', fake_preflight)
    monkeypatch.setattr(
        diagnostic.subprocess, 'Popen',
        lambda *a, **k: (_ for _ in ()).throw(AssertionError('launch spawned')))
    assert diagnostic.runner_run(args) == 0
    assert called
    assert list(tmp_path.glob('headless_*/ros_preflight.json'))


def test_standalone_runner_does_not_launch_when_preflight_fails(tmp_path, monkeypatch):
    args = diagnostic.runner_parser().parse_args([
        '--results-directory', str(tmp_path), '--ros-domain-id',
        str(preflight.ROS_DOMAIN_MAX),
        '--webots-port', '23667',
    ])
    world = tmp_path / 'world.wbt'
    world.write_text('WorldInfo {}\n')
    monkeypatch.setattr(diagnostic, 'package_prefix', lambda: tmp_path)
    monkeypatch.setattr(diagnostic, 'resolve_world', lambda _args: world)
    monkeypatch.setattr(
        diagnostic, 'run_preflight',
        lambda **kwargs: (_ for _ in ()).throw(
            preflight.PreflightError('failed')))
    monkeypatch.setattr(
        diagnostic.subprocess, 'Popen',
        lambda *a, **k: (_ for _ in ()).throw(AssertionError('launch spawned')))
    with pytest.raises(diagnostic.RunnerError, match='failed'):
        diagnostic.runner_run(args)
