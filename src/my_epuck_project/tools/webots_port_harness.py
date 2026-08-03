#!/usr/bin/env python3
"""Bounded Webots-only port negotiation forensic harness.

This intentionally starts no ROS process. It launches the Windows Webots
executable against the saved world, records requested/fallback port evidence,
and terminates the one Webots process after a bounded interval.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import queue
import re
import shutil
import signal
import socket
import subprocess
import threading
import time

WORKSPACE = Path('/home/arash/webots_ws')
DEFAULT_WORLD = WORKSPACE / 'src/my_epuck_project/worlds/epuck_d500_two_world_large.wbt'
DEFAULT_WEBOTS = Path('/mnt/c/Program Files/Webots/msys64/mingw64/bin/webots.exe')


def run_bounded(command, timeout=3.0):
    try:
        completed = subprocess.run(
            command, capture_output=True, text=True, timeout=timeout,
            check=False)
        return {'command': [str(x) for x in command],
                'returncode': completed.returncode,
                'stdout': completed.stdout[-2000000:],
                'stderr': completed.stderr[-2000000:],
                'timed_out': False}
    except subprocess.TimeoutExpired as exc:
        return {'command': [str(x) for x in command], 'returncode': None,
                'stdout': (exc.stdout or '')[-200000:] if isinstance(exc.stdout, str) else '',
                'stderr': (exc.stderr or '')[-200000:] if isinstance(exc.stderr, str) else '',
                'timed_out': True}


def windows_snapshot(ports):
    values = {int(x) for x in ports}
    # Get-NetTCPConnection is attempted because it is the authoritative
    # PowerShell API, but it can stall on this WSL host.  Bounded netstat is
    # retained as a contemporaneous Windows-side fallback.
    port_text = ','.join(str(x) for x in sorted(values))
    script = (
        '$ports=@(' + port_text + '); '
        '$tcp=@(Get-NetTCPConnection -ErrorAction SilentlyContinue '
        '-LocalPort $ports; Get-NetTCPConnection -ErrorAction SilentlyContinue '
        '-RemotePort $ports) | Sort-Object LocalAddress,LocalPort,RemoteAddress,RemotePort,State -Unique | '
        'Select-Object LocalAddress,LocalPort,RemoteAddress,RemotePort,State,OwningProcess; '
        '$udp=Get-NetUDPEndpoint -ErrorAction SilentlyContinue -LocalPort $ports | '
        'Select-Object LocalAddress,LocalPort,OwningProcess; '
        '$o=[pscustomobject]@{tcp=@($tcp);udp=@($udp)}; '
        '$o | ConvertTo-Json -Compress -Depth 4')
    powershell_result = run_bounded([
        '/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe',
        '-NoProfile', '-Command', script], timeout=8.0)
    decoded = None
    try:
        decoded = (json.loads(powershell_result['stdout'].strip())
                   if powershell_result['stdout'].strip() else None)
    except json.JSONDecodeError:
        pass

    def netstat_rows(protocol):
        exe = '/mnt/c/Windows/System32/netstat.exe'
        result = run_bounded([exe, '-ano', '-p', protocol], timeout=5.0)
        rows = []
        pattern = re.compile(
            r'^\s*(?:TCP|UDP)\s+([^ ]+):(\d+)\s+'
            r'([^ ]+):(\d+)(?:\s+(\S+))?(?:\s+(\d+))?\s*$')
        for line in result['stdout'].splitlines():
            match = pattern.match(line)
            if not match:
                continue
            local_port, remote_port = int(match.group(2)), int(match.group(4))
            if local_port not in values and remote_port not in values:
                continue
            rows.append({
                'protocol': protocol, 'local_address': match.group(1),
                'local_port': local_port, 'remote_address': match.group(3),
                'remote_port': remote_port, 'state': match.group(5),
                'pid': int(match.group(6)) if match.group(6) else None,
            })
        return {'result': result, 'rows': rows}

    netstat_tcp = netstat_rows('tcp')
    netstat_udp = netstat_rows('udp')
    owners = set()
    for row in netstat_tcp['rows'] + netstat_udp['rows']:
        if row.get('pid'):
            owners.add(row['pid'])
    if decoded:
        for row in (decoded.get('tcp', []) or []) + (decoded.get('udp', []) or []):
            if row.get('OwningProcess'):
                owners.add(int(row['OwningProcess']))
    process_rows = []
    for pid in sorted(owners):
        command = (
            f'$p=Get-CimInstance Win32_Process -Filter "ProcessId={pid}"; '
            'if ($p) { $p | Select-Object ProcessId,ParentProcessId,Name,ExecutablePath,CommandLine | ConvertTo-Json -Compress }')
        row = run_bounded([
            '/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe',
            '-NoProfile', '-Command', command], timeout=5.0)
        try:
            process_rows.append(json.loads(row['stdout'].strip()))
        except json.JSONDecodeError:
            process_rows.append({'pid': pid, 'raw': row['stdout'][-4000:]})
    return {
        'tcp_udp': decoded,
        'owners': process_rows,
        'powershell': powershell_result,
        'netstat_tcp': netstat_tcp,
        'netstat_udp': netstat_udp,
    }


def wsl_snapshot(ports):
    terms = tuple(str(int(x)) for x in ports)
    def filtered(text):
        return '\n'.join(line for line in text.splitlines()
                          if any(term in line for term in terms))
    tcp = run_bounded(['ss', '-tanpo'], timeout=3.0)
    udp = run_bounded(['ss', '-lunp'], timeout=3.0)
    proc_tcp = Path('/proc/net/tcp').read_text(errors='replace')
    proc_tcp6 = Path('/proc/net/tcp6').read_text(errors='replace')
    return {
        'ss_tcp': filtered(tcp['stdout']),
        'ss_udp': filtered(udp['stdout']),
        'proc_tcp': proc_tcp,
        'proc_tcp6': proc_tcp6,
    }


def windows_active_bind_probe(port):
    script = (
        '$listener=$null; '
        'try { $listener=[System.Net.Sockets.TcpListener]::new([System.Net.IPAddress]::Any,'
        + str(int(port)) + '); $listener.Start(); "BIND_OK" } '
        'catch { "BIND_FAILED: $($_.Exception.Message)" } '
        'finally { if ($listener) { $listener.Stop() } }')
    return run_bounded([
        '/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe',
        '-NoProfile', '-Command', script], timeout=8.0)


def local_bind_probe(port):
    stream = socket.socket()
    try:
        stream.bind(('127.0.0.1', int(port)))
        return True
    except OSError:
        return False
    finally:
        stream.close()


def take_snapshot(label, ports):
    return {
        'label': label,
        'time_utc': datetime.now(timezone.utc).isoformat(),
        'monotonic_s': time.monotonic(),
        'ports': [int(x) for x in ports],
        'wsl': wsl_snapshot(ports),
        'windows': windows_snapshot(ports),
        'bind_probe': {str(int(p)): local_bind_probe(p) for p in ports},
    }


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', type=Path, default=DEFAULT_WORLD)
    parser.add_argument('--webots', type=Path, default=DEFAULT_WEBOTS)
    parser.add_argument('--port', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--runtime', type=float, default=15.0)
    parser.add_argument('--skip-wsl-bind-probe', action='store_true')
    parser.add_argument('--windows-bind-probe', action='store_true')
    return parser.parse_args()


def main():
    args = parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if not args.world.is_file():
        raise SystemExit(f'world not found: {args.world}')
    if not args.webots.is_file():
        raise SystemExit(f'Webots executable not found: {args.webots}')
    world_win = subprocess.check_output(['wslpath', '-w', str(args.world)], text=True).strip()
    fallback = args.port + 1
    ports = (args.port, fallback)
    env_keys = ('WEBOTS_HOME', 'WEBOTS_SERVER', 'WEBOTS_CONTROLLER_URL')
    report = {
        'requested_port': args.port,
        'fallback_port': fallback,
        'world': str(args.world),
        'world_windows_path': world_win,
        'webots_executable': str(args.webots),
        'environment': {key: os.environ.get(key) for key in env_keys},
        'wsl_bind_probe_enabled': not args.skip_wsl_bind_probe,
        'windows_active_bind_probe_enabled': args.windows_bind_probe,
        'windows_active_bind_probe': None,
        'command': [str(args.webots), f'--port={args.port}', '--no-rendering',
                    '--stdout', '--stderr', '--minimize', world_win,
                    '--batch', '--mode=realtime'],
        'snapshots': [], 'output_events': [], 'exit_status': None,
        'webots_pid': None, 'started_utc': datetime.now(timezone.utc).isoformat(),
    }
    if args.windows_bind_probe:
        report['windows_active_bind_probe'] = windows_active_bind_probe(args.port)
    if not args.skip_wsl_bind_probe:
        report['snapshots'].append(take_snapshot('before_launch_after_wsl_probe', ports))
    else:
        report['snapshots'].append(take_snapshot('before_launch_no_wsl_probe', ports))

    process = subprocess.Popen(report['command'], stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True,
                               bufsize=1, start_new_session=True)
    report['webots_pid'] = process.pid
    report['snapshots'].append(take_snapshot('after_process_creation', ports))
    lines = queue.Queue()
    def reader():
        for line in iter(process.stdout.readline, ''):
            lines.put((time.monotonic(), line.rstrip('\n')))
        process.stdout.close()
    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    started = time.monotonic()
    saw_failure = False
    saw_redirect = False
    while time.monotonic() - started < args.runtime:
        try:
            t, line = lines.get(timeout=0.2)
        except queue.Empty:
            if process.poll() is not None:
                break
            continue
        report['output_events'].append({'monotonic_s': t, 'line': line})
        lower = line.lower()
        if 'could not listen' in lower and not saw_failure:
            saw_failure = True
            report['snapshots'].append(take_snapshot('on_bind_failure_message', ports))
        if 'using port' in lower and not saw_redirect:
            saw_redirect = True
            match = re.search(r'using port\s+(\d+)', lower)
            if match:
                actual = int(match.group(1))
                report['actual_port_reported'] = actual
                snap_ports = tuple(dict.fromkeys((args.port, fallback, actual)))
                report['snapshots'].append(take_snapshot('on_redirect_message', snap_ports))
        if 'waiting for local or remote connection' in lower and 'port' in lower:
            report['snapshots'].append(take_snapshot('on_listen_success_message', ports))
        if process.poll() is not None:
            break
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=5.0)
    report['exit_status'] = process.returncode
    report['snapshots'].append(take_snapshot('after_cleanup', tuple(dict.fromkeys((args.port, fallback)))) )
    thread.join(timeout=2.0)
    (args.output / 'webots_port_harness.json').write_text(
        json.dumps(report, indent=2, sort_keys=True) + '\n')
    (args.output / 'webots_stdout.log').write_text(
        '\n'.join(item['line'] for item in report['output_events']) + '\n')
    print(json.dumps({
        'requested_port': args.port,
        'actual_port_reported': report.get('actual_port_reported'),
        'pid': report['webots_pid'],
        'exit_status': report['exit_status'],
        'snapshots': len(report['snapshots']),
        'output': str(args.output),
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
