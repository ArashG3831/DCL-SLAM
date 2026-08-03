"""Bounded ROS/Windows runtime checks used before diagnostic launches."""

from __future__ import annotations

import json
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import subprocess

import psutil

ROS_DOMAIN_MIN = 0
ROS_DOMAIN_MAX = 232
ROS_DAEMON_BASE_PORT = 11511


class PreflightError(RuntimeError):
    """A required pre-launch runtime condition is not safe."""


def _bounded_subprocess(command, *, env=None, timeout_s=10.0,
                       kill_after_s=2.0, output_limit=4000):
    """Run a subprocess with normal and kill-after deadlines."""
    process = subprocess.Popen(
        command, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, start_new_session=True)
    timed_out = False
    try:
        output, _ = process.communicate(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        timed_out = True
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            output, _ = process.communicate(timeout=kill_after_s)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            output, _ = process.communicate(timeout=kill_after_s)
    return {
        'command': [str(item) for item in command],
        'returncode': process.returncode,
        'stdout': (output or '')[-int(output_limit):],
        'timed_out': timed_out,
        'timeout_s': float(timeout_s),
        'kill_after_s': float(kill_after_s),
    }


def windows_tcp_bindings():
    """Return bounded Windows TCP ownership rows visible from WSL."""
    executable = Path('/mnt/c/Windows/System32/netstat.exe')
    if not executable.exists():
        return []
    result = _bounded_subprocess(
        [str(executable), '-ano', '-p', 'tcp'], timeout_s=10.0,
        output_limit=2000000)
    if result['timed_out'] or result['returncode'] != 0:
        return []
    rows = []
    pattern = re.compile(
        r'^\s*TCP\s+\S+:(\d+)\s+\S+:(\d+)\s+'
        r'(\S+)\s+(\d+)\s*$', re.IGNORECASE)
    for line in result['stdout'].splitlines():
        match = pattern.match(line)
        if match:
            rows.append({
                'local_port': int(match.group(1)),
                'remote_port': int(match.group(2)),
                'state': match.group(3),
                'pid': int(match.group(4)),
            })
    return rows


def local_bind_probe(port):
    """Test whether mirrored-mode WSL can bind a loopback TCP port."""
    stream = socket.socket()
    try:
        stream.bind(('127.0.0.1', int(port)))
        return {'ok': True, 'errno': None, 'error': ''}
    except OSError as error:
        return {'ok': False, 'errno': error.errno, 'error': str(error)}
    finally:
        stream.close()


def _windows_process_context(pids):
    """Best-effort process/parent identity for Windows TCP owners."""
    contexts = []
    tasklist = Path('/mnt/c/Windows/System32/tasklist.exe')
    powershell = Path(
        '/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe')
    for pid in sorted(set(int(item) for item in pids)):
        item = {
            'pid': pid, 'process_name': None, 'process': None,
            'parent': None, 'signature': 'unknown',
        }
        if tasklist.exists():
            result = _bounded_subprocess(
                [str(tasklist), '/svc', '/FI', f'PID eq {pid}',
                 '/FO', 'CSV', '/NH'], timeout_s=5.0)
            for line in result['stdout'].splitlines():
                if line.startswith('"'):
                    fields = [part.strip('"') for part in line.split('","')]
                    if fields:
                        item['process_name'] = fields[0]
                    break
        if powershell.exists():
            script = (
                f'$p=Get-CimInstance Win32_Process -Filter "ProcessId={pid}"; '
                'if ($p) { $pp=Get-CimInstance Win32_Process -Filter '
                '("ProcessId=" + $p.ParentProcessId); '
                '$o=[pscustomobject]@{process=($p|Select ProcessId,ParentProcessId,Name,ExecutablePath,CommandLine);'
                'parent=($pp|Select ProcessId,ParentProcessId,Name,ExecutablePath,CommandLine)};'
                '$o|ConvertTo-Json -Compress }')
            result = _bounded_subprocess(
                [str(powershell), '-NoProfile', '-Command', script],
                timeout_s=8.0)
            try:
                decoded = json.loads(result['stdout'].strip())
                item['process'] = decoded.get('process')
                item['parent'] = decoded.get('parent')
            except (json.JSONDecodeError, AttributeError):
                pass
        contexts.append(item)
    return contexts


def _current_process_ancestry():
    """Return this process and its ancestors to avoid self-detection."""
    excluded = {os.getpid()}
    try:
        process = psutil.Process(os.getpid())
        while process:
            parent = process.parent()
            if parent is None:
                break
            excluded.add(parent.pid)
            process = parent
    except psutil.Error:
        pass
    return excluded


def _project_processes():
    # Do not use the broad package name: the shell command that starts this
    # runner contains the workspace path and would otherwise be a false hit.
    markers = (
        'webots', 'ros2 launch', 'nav2_frontier_diagnostic',
        'run_nav2_frontier_diagnostic', 'cooperative_regression.py',
        'webots-controller',
    )
    excluded = _current_process_ancestry()
    result = []
    for process in psutil.process_iter(['pid', 'status', 'cmdline']):
        pid = process.info.get('pid')
        if pid in excluded:
            continue
        command = ' '.join(process.info.get('cmdline') or [])
        if any(marker in command.lower() for marker in markers):
            result.append({
                'pid': pid,
                'status': process.info.get('status'),
                'command': command,
            })
    return result


def _project_d_state_processes():
    return [
        item for item in _project_processes()
        if item['status'] == psutil.STATUS_DISK_SLEEP
    ]


def run_preflight(*, ros_domain_id, webots_port, output_dir, environment=None):
    """Write preflight JSON and raise before launch if unsafe."""
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    environment = dict(environment or os.environ)
    domain = int(ros_domain_id)
    report = {
        'schema_version': '1.0.0',
        'status': 'PREFLIGHT_RUNNING',
        'ros_domain_id': domain,
        'ros_daemon_port': ROS_DAEMON_BASE_PORT + domain,
        'rmw_implementation': environment.get(
            'RMW_IMPLEMENTATION', 'default'),
        'webots_port': int(webots_port),
        'windows_port_rows': [],
        'windows_owner_context': [],
        'daemon_port_bind': None,
        'webots_port_bind': None,
        'project_processes': [],
        'project_d_state_processes': [],
        'node_list_no_daemon': None,
        'topic_list_no_daemon': None,
        'fallback_domains_0_101': [],
        'safe_daemon_independent': False,
    }
    if domain < ROS_DOMAIN_MIN or domain > ROS_DOMAIN_MAX:
        report['status'] = 'ROS_MIDDLEWARE_PREFLIGHT_FAILED'
        report['error'] = (
            f'ROS domain must be between {ROS_DOMAIN_MIN} and '
            f'{ROS_DOMAIN_MAX}')
        (output / 'ros_preflight.json').write_text(
            json.dumps(report, indent=2, sort_keys=True) + '\n')
        raise PreflightError(report['error'])

    rows = windows_tcp_bindings()
    daemon_port = report['ros_daemon_port']
    report['windows_port_rows'] = [
        row for row in rows
        if row['local_port'] in (daemon_port, int(webots_port))
        or row['remote_port'] in (daemon_port, int(webots_port))
    ]
    report['windows_owner_context'] = _windows_process_context(
        [row['pid'] for row in report['windows_port_rows']])
    report['daemon_port_bind'] = local_bind_probe(daemon_port)
    report['webots_port_bind'] = local_bind_probe(webots_port)
    report['project_processes'] = _project_processes()
    report['project_d_state_processes'] = _project_d_state_processes()
    windows_local_ports = {row['local_port'] for row in rows}
    for candidate in range(0, 102):
        candidate_port = ROS_DAEMON_BASE_PORT + candidate
        if candidate_port in windows_local_ports:
            continue
        if local_bind_probe(candidate_port)['ok']:
            report['fallback_domains_0_101'].append({
                'domain': candidate, 'daemon_port': candidate_port,
            })

    ros2 = shutil.which('ros2')
    if ros2 is None:
        report['error'] = 'ros2 not found in PATH'
    else:
        env = dict(environment)
        env['ROS_DOMAIN_ID'] = str(domain)
        report['node_list_no_daemon'] = _bounded_subprocess(
            [ros2, 'node', 'list', '--no-daemon'], env=env)
        report['topic_list_no_daemon'] = _bounded_subprocess(
            [ros2, 'topic', 'list', '--no-daemon'], env=env)

    daemon_conflict = not report['daemon_port_bind']['ok']
    node = report['node_list_no_daemon']
    topic = report['topic_list_no_daemon']
    if daemon_conflict and node and topic:
        if (node['returncode'] == 0 and topic['returncode'] == 0
                and not node['timed_out'] and not topic['timed_out']):
            print(
                'ROS2CLI_DAEMON_PORT_CONFLICT '
                f'domain={domain} port={daemon_port}; '
                'continuing_no_daemon=true', flush=True)

    node_ok = bool(node and node['returncode'] == 0
                   and not node['timed_out'])
    topic_ok = bool(topic and topic['returncode'] == 0
                    and not topic['timed_out'])
    webots_ok = report['webots_port_bind']['ok'] and not any(
        row['local_port'] == int(webots_port) for row in rows)
    report['safe_daemon_independent'] = bool(
        node_ok and topic_ok and webots_ok
        and not report['project_d_state_processes']
        and not report['project_processes'])
    report['status'] = (
        'SAFE_DAEMON_INDEPENDENT' if report['safe_daemon_independent']
        else 'ROS_MIDDLEWARE_PREFLIGHT_FAILED')
    (output / 'ros_preflight.json').write_text(
        json.dumps(report, indent=2, sort_keys=True) + '\n')
    if not report['safe_daemon_independent']:
        raise PreflightError(
            f"{report['status']}: see {output / 'ros_preflight.json'}")
    return report
