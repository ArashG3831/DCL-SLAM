import json
from pathlib import Path

import pytest

from my_epuck_project import webots_port_negotiation as negotiation
from my_epuck_project.nav2_frontier_diagnostic import launch_command, runner_parser


def _fake_setup(tmp_path, monkeypatch, outputs):
    webots = tmp_path / 'webots.exe'
    webots.write_text('stub')
    world = tmp_path / 'world.wbt'
    world.write_text('WorldInfo {}\n')
    monkeypatch.setattr(negotiation, '_webots_executable', lambda env: webots)
    monkeypatch.setattr(
        negotiation.subprocess, 'check_output',
        lambda *args, **kwargs: 'C:\\world.wbt\n')
    monkeypatch.setattr(negotiation, 'windows_tcp_bindings', lambda: [])
    monkeypatch.setattr(negotiation, '_windows_process_context', lambda pids: [])
    sequence = iter(outputs)
    monkeypatch.setattr(
        negotiation, '_bounded_process', lambda command, timeout_s: (0, next(sequence)))
    return world


def test_requested_port_accepted_without_redirect(tmp_path, monkeypatch):
    world = _fake_setup(tmp_path, monkeypatch, [[
        "INFO: 'robot1' extern controller: Waiting for local or remote connection on port 25000 targeting robot named 'robot1'.",
    ]])
    report = negotiation.negotiate_webots_port(
        world=world, requested_port=25000, output_dir=tmp_path)
    assert report['actual_port'] == 25000
    assert report['redirect_history'] == []
    assert report['attempt_count'] == 1
    assert json.loads((tmp_path / 'webots_port_preflight.json').read_text())['final_confirmation']


def test_redirect_records_actual_and_multiple_redirects(tmp_path, monkeypatch):
    world = _fake_setup(tmp_path, monkeypatch, [[
        'WARNING: Could not listen to extern controllers on port 25000. Using port 25002 instead.',
        'WARNING: Could not listen to extern controllers on port 25001. Using port 25002 instead.',
        "INFO: 'robot1' extern controller: Waiting for local or remote connection on port 25002 targeting robot named 'robot1'.",
    ]])
    report = negotiation.negotiate_webots_port(
        world=world, requested_port=25000, output_dir=tmp_path)
    assert report['actual_port'] == 25002
    assert len(report['redirect_history']) == 2
    assert report['redirect_history'][0]['requested_port'] == 25000


def test_retry_limit_aborts_without_port_confirmation(tmp_path, monkeypatch):
    world = _fake_setup(tmp_path, monkeypatch, [[], [], [], [], []])
    with pytest.raises(negotiation.WebotsPortNegotiationError):
        negotiation.negotiate_webots_port(
            world=world, requested_port=25000, output_dir=tmp_path,
            max_attempts=5, observation_timeout_s=0.01)
    report = json.loads((tmp_path / 'webots_port_preflight.json').read_text())
    assert report['status'] == 'WEBOTS_PORT_NEGOTIATION_FAILED'
    assert report['attempt_count'] == 5


def test_partial_output_parser_handles_split_redirect():
    parser = negotiation.WebotsOutputParser()
    assert parser.feed('WARNING: Could not listen to extern controllers on po') == []
    events = parser.feed('rt 25000. Using port 25002 instead.\n')
    assert events == [('redirect', 25000, 25002)]


def test_launch_command_passes_confirmed_controller_port(tmp_path):
    args = runner_parser().parse_args([
        '--webots-port', '25000', '--ros-domain-id', '230',
    ])
    command = launch_command(args, Path('/tmp/world.wbt'), tmp_path,
                             controller_port=25002)
    assert 'webots_port:=25000' in command
    assert 'webots_controller_port:=25002' in command
    assert command.count('webots_port:=25000') == 1


def test_bind_probe_is_closed():
    import socket
    stream = socket.socket()
    stream.bind(('127.0.0.1', 0))
    port = stream.getsockname()[1]
    stream.close()
    with socket.socket() as check:
        check.bind(('127.0.0.1', port))
