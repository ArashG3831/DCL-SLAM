"""Bounded Webots actual-port negotiation for the standalone diagnostic.

Webots R2025a under this WSL mirrored-network setup may launch its
``webots-bin.exe`` child with the requested port in its command line while the
server actually listens on a redirected port.  This module treats the Webots
output/listener as authoritative and returns the requested/actual pair to the
 diagnostic launch, which passes the actual port to every controller.
"""
from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import queue
import re
import signal
import subprocess
import threading
import time

from .ros_runtime_preflight import windows_tcp_bindings, _windows_process_context

REDIRECT_RE = re.compile(
    r'could not listen.*?port\s+(\d+).*?using port\s+(\d+)', re.IGNORECASE)
WAITING_RE = re.compile(
    r'waiting for .* connection on port\s+(\d+)', re.IGNORECASE)


class WebotsPortNegotiationError(RuntimeError):
    """Webots did not provide a bounded common-port confirmation."""


def parse_webots_port_event(line: str):
    """Return ``(kind, requested, actual)`` for a Webots port line."""
    redirect = REDIRECT_RE.search(line)
    if redirect:
        return 'redirect', int(redirect.group(1)), int(redirect.group(2))
    waiting = WAITING_RE.search(line)
    if waiting:
        port = int(waiting.group(1))
        return 'listening', port, port
    return None


class WebotsOutputParser:
    """Preserve partial stdout chunks while detecting port events."""

    def __init__(self):
        self._buffer = ''

    def feed(self, chunk: str):
        self._buffer += chunk
        lines = self._buffer.split('\n')
        self._buffer = lines.pop()
        return [event for line in lines
                if (event := parse_webots_port_event(line)) is not None]

    def finish(self):
        if not self._buffer:
            return []
        event = parse_webots_port_event(self._buffer)
        self._buffer = ''
        return [event] if event else []


def _webots_executable(environment=None):
    environment = environment or os.environ
    home = environment.get('WEBOTS_HOME', '/mnt/c/Program Files/Webots')
    return Path(home) / 'msys64' / 'mingw64' / 'bin' / 'webots.exe'


def _bounded_process(command, timeout_s=3.0):
    process = subprocess.Popen(
        command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1, start_new_session=True)
    lines = queue.Queue()

    def reader():
        for line in iter(process.stdout.readline, ''):
            lines.put(line.rstrip('\n'))
        process.stdout.close()

    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    output = []
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            output.append(lines.get(timeout=0.1))
        except queue.Empty:
            if process.poll() is not None:
                break
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=3.0)
    thread.join(timeout=2.0)
    while True:
        try:
            output.append(lines.get_nowait())
        except queue.Empty:
            break
    return process.returncode, output


def negotiate_webots_port(*, world, requested_port, output_dir,
                          max_attempts=5, observation_timeout_s=20.0,
                          environment=None):
    """Probe Webots and return the requested/actual port pair.

    The probe is Webots-only: no ROS launch, controller, Nav2, or frontier
    process is started.  A redirected port is recorded and returned for the
    subsequent full launch, where the confirmed actual port is passed to
    both Webots and every controller.
    """
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    environment = dict(environment or os.environ)
    world = Path(world).resolve()
    webots = _webots_executable(environment)
    if not webots.is_file():
        raise WebotsPortNegotiationError(f'Webots executable not found: {webots}')
    try:
        world_windows = subprocess.check_output(
            ['wslpath', '-w', str(world)], text=True, timeout=5).strip()
    except (OSError, subprocess.SubprocessError) as error:
        raise WebotsPortNegotiationError(f'wslpath failed: {error}') from error

    report = {
        'schema_version': '1.0.0',
        'status': 'NEGOTIATING',
        'requested_port': int(requested_port),
        'actual_port': None,
        'attempt_count': 0,
        'redirect_history': [],
        'windows_owners_observed': [],
        'command_history': [],
        'output_events': [],
        'final_confirmation': False,
        'started_utc': datetime.now(timezone.utc).isoformat(),
    }
    current = int(requested_port)
    for attempt in range(1, int(max_attempts) + 1):
        report['attempt_count'] = attempt
        command = [
            str(webots), f'--port={current}', '--no-rendering', '--stdout',
            '--stderr', '--minimize', world_windows, '--batch',
            '--mode=realtime',
        ]
        report['command_history'].append(command)
        returncode, lines = _bounded_process(command, observation_timeout_s)
        candidate_actual = None
        for line in lines:
            report['output_events'].append({
                'attempt': attempt, 'line': line,
                'time_utc': datetime.now(timezone.utc).isoformat(),
            })
            event = parse_webots_port_event(line)
            if not event:
                continue
            kind, source, actual = event
            candidate_actual = actual
            owners = windows_tcp_bindings()
            owner_pids = [row['pid'] for row in owners
                          if row['local_port'] in (source, actual)
                          or row['remote_port'] in (source, actual)]
            report['windows_owners_observed'].append({
                'attempt': attempt, 'event': kind, 'source_port': source,
                'actual_port': actual, 'rows': owners,
                'processes': _windows_process_context(owner_pids),
            })
            if kind == 'redirect':
                report['redirect_history'].append({
                    'attempt': attempt, 'requested_port': source,
                    'actual_port': actual,
                })
        if candidate_actual is not None:
            report['requested_port'] = int(requested_port)
            report['actual_port'] = int(candidate_actual)
            report['final_confirmation'] = True
            report['status'] = 'WEBOTS_PORT_CONFIRMED'
            (output / 'webots_port_preflight.json').write_text(
                json.dumps(report, indent=2, sort_keys=True) + '\n')
            return report
        # No bounded confirmation: use a new candidate only for a bounded
        # number of attempts.  Never leave the probe process alive.
        current += 2
    report['status'] = 'WEBOTS_PORT_NEGOTIATION_FAILED'
    (output / 'webots_port_preflight.json').write_text(
        json.dumps(report, indent=2, sort_keys=True) + '\n')
    raise WebotsPortNegotiationError(
        f'WEBOTS_PORT_NEGOTIATION_FAILED; see {output / "webots_port_preflight.json"}')
