"""Self-contained ten-trial cooperative simulation regression campaign."""

import argparse
from concurrent.futures import FIRST_COMPLETED, ThreadPoolExecutor, wait
import csv
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import platform
import re
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time

from ament_index_python.packages import get_package_share_directory
import psutil

from .cooperative_regression_report import analyze_campaign, atomic_json
from .cooperative_profiles import (
    PROFILE_SETTINGS,
    manual_rviz_path,
    profile,
    profile_summary,
)
from .occupancy_map_comparison import (
    canonical_map,
    common_geometry,
    compare_semantic,
    consensus_maps,
    load_map,
    resample_semantic,
)


CLASSIFICATIONS = (
    'PASS', 'SYSTEM_FAILURE', 'MISSION_TIMEOUT', 'PROCESS_CRASH',
    'INFRASTRUCTURE_FAILURE', 'INCOMPLETE_ARTIFACTS', 'USER_INTERRUPTED',
)
REQUIRED_OBSERVER_FILES = (
    'summary.json', 'events.jsonl', 'warnings.jsonl', 'topic_health.csv',
    'coverage.csv', 'robot1_timeseries.csv', 'robot2_timeseries.csv',
)
EXPECTED_NODE_SUFFIXES = (
    '/robot1/cooperative_frontier_coordinator',
    '/robot2/cooperative_frontier_coordinator',
    '/robot1/map_fusion',
    '/robot2/map_fusion',
)
PROGRESS_LOCK = threading.RLock()


def utc_now():
    return datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')


def safe_campaign_id():
    return 'regression_' + datetime.now(
        timezone.utc).strftime('%Y%m%dT%H%M%SZ')


def run(command, cwd=None, env=None, timeout=None, output=None):
    return subprocess.run(
        command, cwd=cwd, env=env, timeout=timeout, text=True,
        stdout=output or subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )


def git_value(workspace, arguments, default='unknown'):
    result = run(['git', *arguments], cwd=workspace, timeout=10)
    return result.stdout.strip() if result.returncode == 0 else default


def host_metadata():
    memory = psutil.virtual_memory()
    cpu_model = 'unknown'
    try:
        for line in Path('/proc/cpuinfo').read_text().splitlines():
            if line.startswith('model name'):
                cpu_model = line.split(':', 1)[1].strip()
                break
    except OSError:
        pass
    return {
        'hostname': socket.gethostname(),
        'os': platform.platform(),
        'kernel': platform.release(),
        'cpu_model': cpu_model,
        'logical_cpu_count': psutil.cpu_count(logical=True),
        'total_ram_bytes': memory.total,
        'ros_distribution': os.getenv('ROS_DISTRO'),
        'rmw_implementation': os.getenv('RMW_IMPLEMENTATION', 'default'),
    }


def webots_information():
    root = Path('/mnt/c/Program Files/Webots')
    executable = root / 'msys64/mingw64/bin/webots.exe'
    version_path = root / 'resources/version.txt'
    version = version_path.read_text().strip() if version_path.exists() \
        else 'unknown'
    help_result = run(
        ['timeout', '20s', str(executable), '--help'], timeout=25)
    supported = {
        flag: flag in help_result.stdout
        for flag in ('--mode=<mode>', '--no-rendering', '--batch', '--port')
    }
    return {
        'version': version,
        'executable': str(executable),
        'supported_options': supported,
        'help_verified': help_result.returncode == 0,
    }


def linux_port_used(port):
    try:
        with socket.socket() as stream:
            stream.settimeout(0.2)
            return stream.connect_ex(('127.0.0.1', int(port))) == 0
    except OSError:
        return True


def windows_port_pid(port):
    executable = Path('/mnt/c/Windows/System32/netstat.exe')
    if not executable.exists():
        return None
    result = run([str(executable), '-ano', '-p', 'tcp'], timeout=10)
    expression = re.compile(
        rf'^\s*TCP\s+\S*:{int(port)}\s+\S+\s+LISTENING\s+(\d+)\s*$',
        re.IGNORECASE | re.MULTILINE)
    match = expression.search(result.stdout)
    return int(match.group(1)) if match else None


def manual_rviz_command(world_profile='small'):
    """Return the installed passive RViz command used by manual trials."""
    package = Path(get_package_share_directory('my_epuck_project'))
    selected = profile(world_profile, package / 'worlds')
    config = manual_rviz_path(selected, package / 'resource')
    return ['rviz2', '-d', str(config)]


def resolve_runner_profile(args):
    """Resolve and verify source/installed copies of the selected world."""
    workspace = Path(args.workspace).resolve()
    source_worlds = workspace / 'src' / 'my_epuck_project' / 'worlds'
    installed_package = Path(get_package_share_directory('my_epuck_project'))
    source = profile(args.world_profile, source_worlds)
    installed = profile(args.world_profile, installed_package / 'worlds')
    source_hash = source['world_metadata']['sha256']
    installed_hash = installed['world_metadata']['sha256']
    if source_hash != installed_hash:
        raise RuntimeError(
            'selected source and installed worlds differ: '
            f'{source["world_path"]} != {installed["world_path"]}')
    args.source_world_path = source['world_path']
    args.installed_world_path = installed['world_path']
    args.profile_metadata = profile_summary(source)
    return source


def print_profile_selection(args, selected):
    """Print the exact reusable experiment configuration before launch."""
    metadata = selected['world_metadata']
    print(f'world_profile={selected["name"]}')
    print(f'source_world_path={args.source_world_path}')
    print(f'installed_world_path={args.installed_world_path}')
    print(
        'map_resolutions_m='
        f'slam:{selected["slam_resolution"]},'
        f'fusion:{selected["fusion_resolution"]},'
        f'global_costmap:{selected["global_costmap_resolution"]},'
        f'local_costmap:{selected["local_costmap_resolution"]}')
    for name in ('robot1', 'robot2'):
        robot = metadata['robots'][name]
        print(
            f'{name}_start=translation:{robot.translation},'
            f'rotation:{robot.rotation}')
    print(f'known_relative_transform={metadata["relative_transform"]}')
    print(f'mission_timeout_s={args.mission_timeout}')
    print(
        f'webots_port_range={args.webots_port_base}..'
        f'{args.webots_port_base + args.trials - 1}')
    print(
        f'ros_domain_range={args.ros_domain_base}..'
        f'{args.ros_domain_base + args.trials - 1}')


def hold_open_artifacts_valid(attempt):
    """Validate settled final robot state and lossless maps before hold."""
    try:
        final_state = json.loads(
            (attempt / 'final_state.json').read_text(encoding='utf-8'))
        robots = final_state['robots']
        for robot in ('robot1', 'robot2'):
            state = robots[robot]
            status = state['status']
            if status['state'] != 'MISSION_COMPLETE':
                return False
            if status.get(
                    'age_at_collection_s',
                    status.get('age_at_write_s', 1e9)) > 4.0:
                return False
            if state['claim'].get('reserving', True):
                return False
            if state.get('navigation_active') is not False:
                return False
            item = load_map(attempt / f'{robot}_final_shared_map.npz')
            if item.metadata.get('validation_errors'):
                return False
    except (OSError, ValueError, KeyError, TypeError):
        return False
    return True


def mission_timeout_expired(now, deadline, completion_verified=False):
    """Return whether the pre-completion mission wall timeout has elapsed."""
    return (
        not completion_verified
        and deadline is not None
        and now >= deadline
    )


def domain_nodes(domain):
    script = (
        'import json,rclpy,time;'
        'rclpy.init();n=rclpy.create_node("regression_domain_probe");'
        'end=time.monotonic()+1.0;'
        '[(rclpy.spin_once(n,timeout_sec=0.1)) '
        'for _ in range(10)];'
        'print(json.dumps(n.get_node_names_and_namespaces()));'
        'n.destroy_node();rclpy.shutdown()'
    )
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(domain)
    result = run([sys.executable, '-c', script], env=env, timeout=8)
    if result.returncode != 0:
        raise RuntimeError(
            f'ROS domain {domain} probe failed: {result.stdout[-500:]}')
    lines = [line for line in result.stdout.splitlines()
             if line.startswith('[')]
    return json.loads(lines[-1]) if lines else []


def validate_resource_range(args, completed_trials=None):
    completed_trials = completed_trials or set()
    failures = []
    for number in range(1, args.trials + 1):
        if number in completed_trials:
            continue
        domain = args.ros_domain_base + number - 1
        port = args.webots_port_base + number - 1
        if not 0 <= domain <= 232:
            failures.append(f'ROS_DOMAIN_ID {domain} is outside 0..232')
        if not 1024 <= port <= 65535:
            failures.append(f'Webots port {port} is outside 1024..65535')
        if linux_port_used(port) or windows_port_pid(port) is not None:
            failures.append(f'Webots port {port} is already in use')
        nodes = domain_nodes(domain)
        foreign = [item for item in nodes
                   if item[0] != 'regression_domain_probe']
        if foreign:
            failures.append(
                f'ROS domain {domain} is already in use: {foreign}')
    if failures:
        raise RuntimeError('; '.join(failures))


def observer_directory(attempt, run_id):
    direct = attempt / 'observer' / run_id
    if direct.exists():
        return direct
    matches = sorted((attempt / 'observer').glob(f'{run_id}*'))
    return matches[-1] if matches else direct


def parse_log_errors(path):
    try:
        text = path.read_text(encoding='utf-8', errors='replace')
    except OSError:
        return {
            'traceback': False, 'pre_shutdown_traceback': False,
            'shutdown_traceback_count': 0, 'error_line_count': 0,
            'error_lines': [],
        }
    shutdown_markers = (
        '[launch]: user interrupted with ctrl-c (SIGINT)',
        "sending signal 'SIGINT'",
    )
    marker_offsets = [
        text.find(marker) for marker in shutdown_markers
        if text.find(marker) >= 0
    ]
    shutdown_offset = min(marker_offsets) if marker_offsets else len(text)
    pre_shutdown_text = text[:shutdown_offset]
    traceback_count = text.count('Traceback (most recent call last)')
    pre_shutdown_traceback_count = pre_shutdown_text.count(
        'Traceback (most recent call last)')
    lines = [
        line.strip() for line in text.splitlines()
        if re.search(r'\b(ERROR|FATAL|Traceback)\b', line, re.IGNORECASE)
    ]
    return {
        # ``traceback`` retains its stable meaning as an unexpected traceback.
        # ROS nodes commonly emit ExternalShutdownException after the runner's
        # requested SIGINT; preserve those separately without calling them a
        # runtime process crash.
        'traceback': pre_shutdown_traceback_count > 0,
        'pre_shutdown_traceback': pre_shutdown_traceback_count > 0,
        'shutdown_traceback_count': (
            traceback_count - pre_shutdown_traceback_count),
        'error_line_count': len(lines),
        'error_lines': lines[:100],
        'first_error': lines[0] if lines else None,
        'last_error': lines[-1] if lines else None,
    }


def process_tree_metrics(processes):
    rss = 0
    cpu = 0.0
    count = 0
    seen = set()
    for process in processes:
        try:
            candidates = [process] + process.children(recursive=True)
        except (psutil.Error, OSError):
            candidates = [process]
        for item in candidates:
            if item.pid in seen:
                continue
            seen.add(item.pid)
            try:
                rss += item.memory_info().rss
                cpu += item.cpu_percent(None)
                count += 1
            except (psutil.Error, OSError):
                pass
    return rss, cpu, count, sorted(seen)


def signal_process(process, signum):
    if process.poll() is not None:
        return
    try:
        process.send_signal(signum)
    except ProcessLookupError:
        pass


def process_group_alive(process_group):
    if process_group is None:
        return False
    try:
        os.killpg(process_group, 0)
        return True
    except ProcessLookupError:
        return False


def wait_processes(processes, timeout, process_group=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if (all(process.poll() is not None for process in processes)
                and not process_group_alive(process_group)):
            return True
        time.sleep(0.1)
    return (
        all(process.poll() is not None for process in processes)
        and not process_group_alive(process_group)
    )


def _send_scope(processes, signum, process_group=None):
    if process_group is not None:
        try:
            os.killpg(process_group, signum)
            return
        except ProcessLookupError:
            pass
    for process in processes:
        signal_process(process, signum)


def scoped_shutdown(processes, graceful_timeout, hard_timeout,
                    process_group=None, send_initial_sigint=True):
    """Bounded shutdown of only the child processes supplied by this trial."""
    if send_initial_sigint:
        _send_scope(processes, signal.SIGINT, process_group)
    graceful = wait_processes(
        processes, graceful_timeout, process_group)
    if not graceful:
        _send_scope(processes, signal.SIGTERM, process_group)
        terminated = wait_processes(
            processes, hard_timeout, process_group)
    else:
        terminated = True
    killed = False
    if not terminated:
        killed = True
        _send_scope(processes, signal.SIGKILL, process_group)
        wait_processes(processes, hard_timeout, process_group)
    return {
        'graceful': graceful,
        'terminate_succeeded': terminated,
        'kill_required': killed,
        'all_exited': (
            all(process.poll() is not None for process in processes)
            and not process_group_alive(process_group)
        ),
    }


def required_observer_artifacts(directory):
    return {
        name: (directory / name).is_file()
        for name in REQUIRED_OBSERVER_FILES
    }


def classify_attempt(attempt, run_id, ready, timed_out, unexpected_exit,
                     interrupted, cleanup, settled_observed=False):
    final_state = None
    try:
        final_state = json.loads(
            (attempt / 'final_state.json').read_text(encoding='utf-8'))
    except (OSError, ValueError):
        pass
    maps_valid = True
    map_errors = []
    for robot in ('robot1', 'robot2'):
        path = attempt / f'{robot}_final_shared_map.npz'
        try:
            item = load_map(path)
            errors = item.metadata.get('validation_errors', [])
            if errors:
                maps_valid = False
                map_errors += [f'{robot}:{error}' for error in errors]
        except (OSError, ValueError, KeyError) as error:
            maps_valid = False
            map_errors.append(f'{robot}:{error}')
    observer = observer_directory(attempt, run_id)
    observer_files = required_observer_artifacts(observer)
    observer_summary = {}
    try:
        observer_summary = json.loads(
            (observer / 'summary.json').read_text(encoding='utf-8'))
    except (OSError, ValueError):
        pass
    log_review = parse_log_errors(attempt / 'launch.log')
    if interrupted:
        classification = 'USER_INTERRUPTED'
    elif not ready:
        classification = 'INFRASTRUCTURE_FAILURE'
    elif timed_out:
        classification = 'MISSION_TIMEOUT'
    elif unexpected_exit or log_review['traceback']:
        classification = 'PROCESS_CRASH'
    elif final_state is None or not maps_valid or not all(
            observer_files.values()):
        classification = 'INCOMPLETE_ARTIFACTS'
    else:
        robots = final_state.get('robots', {})
        statuses = [
            robots.get(robot, {}).get('status') for robot in (
                'robot1', 'robot2')
        ]
        claims = [
            robots.get(robot, {}).get('claim') for robot in (
                'robot1', 'robot2')
        ]
        navigation = [
            robots.get(robot, {}).get('navigation_active')
            for robot in ('robot1', 'robot2')
        ]
        system = observer_summary.get('system', {})
        passed = (
            all(status and status.get('state') == 'MISSION_COMPLETE'
                and (
                    status.get(
                        'age_at_collection_s',
                        status.get('age_at_write_s', 1e9)) <= 4.0
                    or settled_observed
                )
                for status in statuses)
            and all(claim and not claim.get('reserving', True)
                    for claim in claims)
            and all(active is False for active in navigation)
            and system.get('internal_logger_error_count', 0) == 0
            and system.get('write_failures', 0) == 0
            and cleanup.get('all_exited', False)
        )
        classification = 'PASS' if passed else 'SYSTEM_FAILURE'
    details = {
        'ready': ready,
        'timed_out': timed_out,
        'unexpected_exit': unexpected_exit,
        'settled_observed': settled_observed,
        'map_validation_errors': map_errors,
        'observer_artifacts': observer_files,
        'observer_summary_path': str(
            Path('observer') / observer.name / 'summary.json'),
        'log_review': log_review,
    }
    return classification, details


def internal_trial(args):
    """Run one launch and collector as children of one isolated supervisor."""
    attempt = Path(args.attempt_dir).resolve()
    attempt.mkdir(parents=True, exist_ok=False)
    (attempt / 'observer').mkdir()
    (attempt / 'ros_logs').mkdir()
    (attempt / 'tmp').mkdir()
    start = time.monotonic()
    metadata = {
        'schema_version': '1.0.0',
        'trial_id': args.trial_id,
        'attempt_id': args.attempt_id,
        'run_id': args.run_id,
        'ros_domain_id': args.ros_domain_id,
        'webots_port': args.webots_port,
        'output_directory': str(attempt),
        'ros_log_directory': str(attempt / 'ros_logs'),
        'temporary_directory': str(attempt / 'tmp'),
        'supervisor_pid': os.getpid(),
        'process_group_id': os.getpgrp(),
        'utc_start': utc_now(),
        'launch_arguments': {
            'world_profile': args.world_profile,
            'source_world_path': args.source_world_path,
            'run_id': args.run_id,
            'output_root': str(attempt / 'observer'),
            'mission_timeout_s': (
                args.mission_timeout + args.settling_period + 30.0),
            'webots_port': args.webots_port,
            'webots_mode': args.webots_mode,
            'webots_gui': str(args.webots_gui).lower(),
            'launch_rviz': 'false',
            'enable_mission_timeout':
                not args.hold_open_after_completion,
            'logger_console_status': False,
        },
        'rviz_requested': args.launch_rviz,
        'hold_open_after_completion': args.hold_open_after_completion,
    }
    atomic_json(attempt / 'runner_metadata.json', metadata)
    environment = os.environ.copy()
    environment.update({
        'ROS_DOMAIN_ID': str(args.ros_domain_id),
        'ROS_LOG_DIR': str(attempt / 'ros_logs'),
        'TMPDIR': str(attempt / 'tmp'),
        'TEMP': str(attempt / 'tmp'),
        'TMP': str(attempt / 'tmp'),
        'PYTHONUNBUFFERED': '1',
    })
    launch_command = [
        'ros2', 'launch', 'my_epuck_project',
        'two_robots_observed_continuous_exploration_launch.py',
        f'world_profile:={args.world_profile}',
        f'source_world_path:={args.source_world_path}',
        f'run_id:={args.run_id}',
        f'output_root:={attempt / "observer"}',
        f'mission_timeout_s:='
        f'{args.mission_timeout + args.settling_period + 30.0}',
        f'webots_port:={args.webots_port}',
        f'webots_mode:={args.webots_mode}',
        f'webots_gui:={str(args.webots_gui).lower()}',
        'launch_rviz:=false',
        f'enable_mission_timeout:='
        f'{str(not args.hold_open_after_completion).lower()}',
        'logger_console_status:=false',
    ]
    collector_command = [
        'ros2', 'run', 'my_epuck_project', 'cooperative_trial_collector',
        '--ros-args',
        '-p', f'output_dir:={attempt}',
        '-p', f'run_id:={args.run_id}',
        '-p', f'settling_period_s:={args.settling_period}',
    ]
    launch_log = (attempt / 'launch.log').open('w', encoding='utf-8')
    collector_log = (attempt / 'collector.log').open('w', encoding='utf-8')
    launch = subprocess.Popen(
        launch_command, env=environment, stdout=launch_log,
        stderr=subprocess.STDOUT, text=True, preexec_fn=os.setpgrp)
    collector = subprocess.Popen(
        collector_command, env=environment, stdout=collector_log,
        stderr=subprocess.STDOUT, text=True,
        preexec_fn=lambda: os.setpgid(0, launch.pid))
    processes = [launch, collector]
    ps_processes = [psutil.Process(process.pid) for process in processes]
    for process in ps_processes:
        try:
            process.cpu_percent(None)
        except psutil.Error:
            pass
    metadata.update({
        'launch_pid': launch.pid,
        'collector_pid': collector.pid,
        'launch_command': launch_command,
        'collector_command': collector_command,
    })
    atomic_json(attempt / 'runner_metadata.json', metadata)
    interrupted = False
    requested_shutdown = False

    holding_open = False
    collector_finalized_for_hold = False

    def stop(signum, frame):
        nonlocal interrupted
        del signum, frame
        interrupted = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    ready = False
    rviz = None
    rviz_log = None
    timed_out = False
    unexpected_exit = False
    peak_rss = 0
    peak_cpu = 0.0
    maximum_processes = 0
    metrics_file = (attempt / 'process_metrics.csv').open(
        'w', newline='', encoding='utf-8')
    writer = csv.DictWriter(metrics_file, fieldnames=(
        'elapsed_s', 'process_tree_rss_bytes', 'process_tree_cpu_percent',
        'process_count', 'host_cpu_percent', 'available_memory_bytes',
        'available_memory_fraction', 'swap_used_bytes', 'webots_windows_pid'))
    writer.writeheader()
    last_sample = 0.0
    readiness_deadline = start + args.startup_timeout
    mission_deadline = None
    status_path = attempt / 'collector_status.json'
    try:
        while not interrupted:
            now = time.monotonic()
            status = {}
            try:
                status = json.loads(status_path.read_text(encoding='utf-8'))
            except (OSError, ValueError):
                pass
            if status.get('ready') and not ready:
                nodes = status.get('nodes', [])
                ready = all(any(node.endswith(suffix) for node in nodes)
                            for suffix in EXPECTED_NODE_SUFFIXES)
                if ready:
                    mission_deadline = now + args.mission_timeout
                    metadata['readiness_elapsed_s'] = now - start
                    metadata['readiness_graph_nodes'] = nodes
                    if args.launch_rviz:
                        rviz_command = manual_rviz_command(args.world_profile)
                        rviz_environment = environment.copy()
                        rviz_environment['LIBGL_ALWAYS_SOFTWARE'] = 'true'
                        rviz_log = (attempt / 'rviz.log').open(
                            'w', encoding='utf-8')
                        rviz = subprocess.Popen(
                            rviz_command,
                            env=rviz_environment,
                            stdout=rviz_log,
                            stderr=subprocess.STDOUT,
                            text=True,
                            preexec_fn=lambda: os.setpgid(0, launch.pid),
                        )
                        processes.append(rviz)
                        rviz_process = psutil.Process(rviz.pid)
                        rviz_process.cpu_percent(None)
                        ps_processes.append(rviz_process)
                        metadata['rviz_pid'] = rviz.pid
                        metadata['rviz_command'] = rviz_command
                    atomic_json(attempt / 'runner_metadata.json', metadata)
            if (ready and status.get('settled')
                    and not holding_open):
                requested_shutdown = True
                if args.hold_open_after_completion:
                    # Finalize the passive collector first so normal settled
                    # status, claims, and lossless maps are safely on disk
                    # before the GUI inspection period begins.
                    signal_process(collector, signal.SIGINT)
                    wait_processes(
                        [collector], args.graceful_shutdown_timeout)
                    collector_finalized_for_hold = (
                        collector.poll() is not None)
                    if (not collector_finalized_for_hold
                            or not hold_open_artifacts_valid(attempt)):
                        metadata['hold_open_validation_failed'] = True
                        atomic_json(
                            attempt / 'runner_metadata.json', metadata)
                        break
                    holding_open = True
                    mission_deadline = None
                    metadata.update({
                        'hold_open_active': True,
                        'hold_open_started_utc': utc_now(),
                        'settled_observed': True,
                    })
                    atomic_json(attempt / 'runner_metadata.json', metadata)
                    print(
                        'Mission complete. Webots and RViz are being kept '
                        'open for inspection.\n'
                        'Press Ctrl+C to shut down and finalize the campaign.',
                        flush=True,
                    )
                else:
                    break
            if not ready and now >= readiness_deadline:
                break
            if mission_timeout_expired(
                    now, mission_deadline, holding_open):
                timed_out = True
                requested_shutdown = True
                break
            if launch.poll() is not None:
                unexpected_exit = not requested_shutdown
                break
            if collector.poll() is not None and not holding_open:
                unexpected_exit = True
                break
            if now - last_sample >= 1.0:
                rss, cpu, count, pids = process_tree_metrics(ps_processes)
                memory = psutil.virtual_memory()
                swap = psutil.swap_memory()
                row = {
                    'elapsed_s': now - start,
                    'process_tree_rss_bytes': rss,
                    'process_tree_cpu_percent': cpu,
                    'process_count': count,
                    'host_cpu_percent': psutil.cpu_percent(None),
                    'available_memory_bytes': memory.available,
                    'available_memory_fraction': (
                        memory.available / memory.total),
                    'swap_used_bytes': swap.used,
                    'webots_windows_pid':
                        windows_port_pid(args.webots_port),
                }
                writer.writerow(row)
                metrics_file.flush()
                peak_rss = max(peak_rss, rss)
                peak_cpu = max(peak_cpu, cpu)
                maximum_processes = max(maximum_processes, count)
                metadata['tracked_pids'] = pids
                last_sample = now
            time.sleep(0.2)
    finally:
        # Freeze the newest settled messages while their freshness is intact.
        if not collector_finalized_for_hold:
            signal_process(collector, signal.SIGINT)
            wait_processes([collector], args.graceful_shutdown_timeout)
        if rviz is not None:
            signal_process(rviz, signal.SIGINT)
        # Then let the launch's passive observer finalize its own outputs.
        signal_process(launch, signal.SIGINT)
        cleanup = scoped_shutdown(
            processes, args.graceful_shutdown_timeout,
            args.hard_shutdown_timeout, process_group=launch.pid,
            send_initial_sigint=False)
        metrics_file.close()
        launch_log.close()
        collector_log.close()
        if rviz_log is not None:
            rviz_log.close()
    port_clean = False
    for _ in range(30):
        if (not linux_port_used(args.webots_port)
                and windows_port_pid(args.webots_port) is None):
            port_clean = True
            break
        time.sleep(0.2)
    cleanup['port_released'] = port_clean
    remaining_pids = [
        pid for pid in metadata.get('tracked_pids', [])
        if psutil.pid_exists(pid)
    ]
    cleanup['remaining_tracked_pids'] = remaining_pids
    cleanup['all_exited'] = (
        cleanup['all_exited'] and port_clean and not remaining_pids)
    user_ended_hold = interrupted and holding_open
    classification, details = classify_attempt(
        attempt, args.run_id, ready, timed_out, unexpected_exit,
        interrupted and not user_ended_hold, cleanup,
        settled_observed=requested_shutdown and not timed_out)
    metadata.update({
        'utc_end': utc_now(),
        'wall_time_s': time.monotonic() - start,
        'classification': classification,
        'classification_details': details,
        'process_exit_codes': {
            'launch': launch.poll(),
            'collector': collector.poll(),
            'rviz': rviz.poll() if rviz is not None else None,
        },
        'cleanup': cleanup,
        'cleanup_complete': cleanup['all_exited'],
        'process_tree_peak_rss_bytes': peak_rss,
        'process_tree_peak_cpu_percent': peak_cpu,
        'maximum_process_count': maximum_processes,
        'webots_windows_pid_at_end': windows_port_pid(args.webots_port),
        'interrupted': interrupted and not user_ended_hold,
        'hold_open_requested': args.hold_open_after_completion,
        'hold_open_entered': holding_open,
        'hold_open_user_shutdown': user_ended_hold,
        'settled_observed': requested_shutdown and not timed_out,
    })
    atomic_json(attempt / 'runner_metadata.json', metadata)
    return CLASSIFICATIONS.index(classification)


def validate_existing_attempt(path):
    metadata = {}
    try:
        metadata = json.loads(
            (path / 'runner_metadata.json').read_text(encoding='utf-8'))
        if metadata.get('classification') not in CLASSIFICATIONS:
            return False
        if metadata.get('classification') == 'INFRASTRUCTURE_FAILURE':
            return False
        json.loads((path / 'final_state.json').read_text(encoding='utf-8'))
        load_map(path / 'robot1_final_shared_map.npz')
        load_map(path / 'robot2_final_shared_map.npz')
    except (OSError, ValueError, KeyError):
        return False
    return True


def refresh_selected_classifications(campaign, progress):
    """Revalidate selected attempts after infrastructure-only fixes."""
    changed = False
    for trial_id, relative in progress.get('valid_trials', {}).items():
        attempt = campaign / relative
        metadata_path = attempt / 'runner_metadata.json'
        try:
            metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            continue
        details = metadata.get('classification_details', {})
        ready = bool(details.get('ready', metadata.get('ready', False)))
        timed_out = bool(details.get('timed_out', False))
        unexpected_exit = bool(details.get('unexpected_exit', False))
        interrupted = bool(metadata.get('interrupted', False))
        settled = metadata.get('settled_observed')
        if settled is None:
            # Older runner versions reached normal loop exit only after
            # collector_status.json reported settled. All other exits set one
            # of the conditions excluded here.
            settled = (
                ready and not timed_out and not unexpected_exit
                and not interrupted
            )
        classification, new_details = classify_attempt(
            attempt, metadata.get('run_id', attempt.name), ready,
            timed_out, unexpected_exit, interrupted,
            metadata.get('cleanup', {
                'all_exited': metadata.get('cleanup_complete', False),
            }), settled_observed=settled)
        old = metadata.get('classification')
        if classification != old:
            metadata.setdefault('classification_history', []).append({
                'utc': utc_now(),
                'from': old,
                'to': classification,
                'reason': (
                    'artifact/log revalidation; controlled-shutdown '
                    'tracebacks excluded and settled freshness retained'),
            })
            metadata['classification'] = classification
            changed = True
        metadata['classification_details'] = new_details
        metadata['settled_observed'] = settled
        atomic_json(metadata_path, metadata)
        for row in progress.get('attempts', []):
            if row.get('path') == relative or (
                    row.get('trial_id') == trial_id
                    and row.get('attempt_id') == metadata.get('attempt_id')):
                row['classification'] = classification
    if changed:
        update_progress(campaign, progress)
    return changed


def load_progress(campaign):
    path = campaign / 'campaign_progress.json'
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return {
            'schema_version': '1.0.0',
            'valid_trials': {},
            'attempts': [],
            'adaptive_reductions': [],
            'interrupted': False,
        }


def next_attempt(campaign, trial_id):
    expression = f'{trial_id}_attempt_'
    numbers = []
    for path in (campaign / 'attempts').glob(f'{expression}*'):
        try:
            numbers.append(int(path.name.rsplit('_', 1)[1]))
        except ValueError:
            pass
    return max(numbers, default=0) + 1


def attempt_namespace(args, trial_number, attempt_number, campaign):
    trial_id = f'trial_{trial_number:02d}'
    attempt_id = f'{trial_id}_attempt_{attempt_number:02d}'
    return argparse.Namespace(
        internal_trial=True,
        attempt_dir=str(campaign / 'attempts' / attempt_id),
        trial_id=trial_id,
        attempt_id=attempt_id,
        run_id=attempt_id,
        world_profile=getattr(args, 'world_profile', 'small'),
        source_world_path=getattr(args, 'source_world_path', ''),
        ros_domain_id=args.ros_domain_base + trial_number - 1,
        webots_port=args.webots_port_base + trial_number - 1,
        webots_mode='fast' if args.fast_mode else 'realtime',
        webots_gui=args.rendering,
        launch_rviz=args.rviz,
        hold_open_after_completion=args.hold_open_after_completion,
        startup_timeout=args.startup_timeout,
        mission_timeout=args.mission_timeout,
        settling_period=args.settling_period,
        graceful_shutdown_timeout=args.graceful_shutdown_timeout,
        hard_shutdown_timeout=args.hard_shutdown_timeout,
    )


def internal_command(namespace):
    command = [
        sys.executable, '-m', 'my_epuck_project.cooperative_regression',
        '--internal-trial',
    ]
    for key, value in vars(namespace).items():
        if key == 'internal_trial':
            continue
        option = '--' + key.replace('_', '-')
        if isinstance(value, bool):
            command += [option, str(value).lower()]
        else:
            command += [option, str(value)]
    return command


def wait_for_attempt_supervisor(process, namespace, attempt):
    """Wait and forward Ctrl+C only to this attempt's process group."""
    announced = False
    while True:
        try:
            return process.wait(timeout=0.2)
        except subprocess.TimeoutExpired:
            if (getattr(
                    namespace, 'hold_open_after_completion', False)
                    and not announced):
                try:
                    metadata = json.loads(
                        (attempt / 'runner_metadata.json').read_text(
                            encoding='utf-8'))
                except (OSError, ValueError):
                    metadata = {}
                if metadata.get('hold_open_active'):
                    print(
                        'Mission complete. Webots and RViz are being kept '
                        'open for inspection.\n'
                        'Press Ctrl+C to shut down and finalize the campaign.',
                        flush=True,
                    )
                    announced = True
        except KeyboardInterrupt:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=30)
            if getattr(
                    namespace, 'hold_open_after_completion', False):
                return process.returncode
            raise


def execute_attempt(namespace):
    attempt = Path(namespace.attempt_dir)
    attempt.parent.mkdir(parents=True, exist_ok=True)
    supervisor_log = attempt.parent / f'.{namespace.attempt_id}.supervisor.log'
    with supervisor_log.open('w', encoding='utf-8') as output:
        process = subprocess.Popen(
            internal_command(namespace), stdout=output,
            stderr=subprocess.STDOUT, start_new_session=True, text=True)
        returncode = wait_for_attempt_supervisor(
            process, namespace, attempt)
    if attempt.exists():
        shutil.move(str(supervisor_log), str(attempt / 'supervisor.log'))
    metadata = json.loads(
        (attempt / 'runner_metadata.json').read_text(encoding='utf-8'))
    metadata['supervisor_return_code'] = returncode
    atomic_json(attempt / 'runner_metadata.json', metadata)
    return metadata


def update_progress(campaign, progress):
    with PROGRESS_LOCK:
        progress['updated_utc'] = utc_now()
        atomic_json(campaign / 'campaign_progress.json', progress)


def perform_attempt(args, campaign, progress, trial_number, retry=True):
    trial_id = f'trial_{trial_number:02d}'
    attempt_number = next_attempt(campaign, trial_id)
    namespace = attempt_namespace(
        args, trial_number, attempt_number, campaign)
    print(f'{namespace.attempt_id} START domain={namespace.ros_domain_id} '
          f'port={namespace.webots_port}', flush=True)
    metadata = execute_attempt(namespace)
    relative = str(Path(namespace.attempt_dir).relative_to(campaign))
    with PROGRESS_LOCK:
        progress['attempts'].append({
            'trial_id': trial_id,
            'attempt_id': namespace.attempt_id,
            'path': relative,
            'classification': metadata['classification'],
        })
    if metadata['classification'] == 'INFRASTRUCTURE_FAILURE' and retry:
        with PROGRESS_LOCK:
            progress['infrastructure_retries'] = (
                progress.get('infrastructure_retries', 0) + 1)
        update_progress(campaign, progress)
        print(f'{namespace.attempt_id} INFRASTRUCTURE_FAILURE retrying once',
              flush=True)
        return perform_attempt(
            args, campaign, progress, trial_number, retry=False)
    if metadata['classification'] == 'INFRASTRUCTURE_FAILURE':
        update_progress(campaign, progress)
        raise RuntimeError(
            f'{namespace.attempt_id} remained an infrastructure failure '
            'after its one permitted retry')
    with PROGRESS_LOCK:
        progress['valid_trials'][trial_id] = relative
    update_progress(campaign, progress)
    print(f'{namespace.attempt_id} {metadata["classification"]} '
          f'{metadata["wall_time_s"]:.1f}s', flush=True)
    return metadata


def print_progress(campaign_id, progress, running, requested):
    complete = len(progress['valid_trials'])
    passed = sum(
        item['classification'] == 'PASS'
        for item in progress['attempts']
        if item['trial_id'] in progress['valid_trials'])
    failed = complete - passed
    memory = psutil.virtual_memory()
    print(
        f'campaign={campaign_id} running={running} '
        f'complete={complete}/{requested} passed={passed} failed={failed} '
        f'cpu={psutil.cpu_percent(None):.0f}% '
        f'available_ram={memory.available / 2**30:.1f}GB',
        flush=True)


def run_parallel_stage(args, campaign, progress, trial_numbers,
                       concurrency):
    pending = list(trial_numbers)
    active = {}
    last_progress = 0.0
    overload_since = None
    with ThreadPoolExecutor(max_workers=concurrency) as executor:
        while pending or active:
            memory = psutil.virtual_memory()
            cpu = psutil.cpu_percent(None)
            overloaded = cpu > 90.0 or memory.available / memory.total < 0.20
            now = time.monotonic()
            if overloaded:
                overload_since = overload_since or now
            else:
                overload_since = None
            sustained = overload_since is not None and now-overload_since > 10
            while pending and len(active) < concurrency and not sustained:
                number = pending.pop(0)
                future = executor.submit(
                    perform_attempt, args, campaign, progress, number)
                active[future] = number
            if sustained and concurrency > 1:
                concurrency -= 1
                progress['adaptive_reductions'].append({
                    'utc': utc_now(), 'new_concurrency': concurrency,
                    'reason': 'sustained_host_overload',
                    'cpu_percent': cpu,
                    'available_memory_fraction':
                        memory.available / memory.total,
                })
                update_progress(campaign, progress)
                print(f'CONCURRENCY_REDUCTION new={concurrency}', flush=True)
                overload_since = None
            if active:
                done, _ = wait(active, timeout=1.0,
                               return_when=FIRST_COMPLETED)
                for future in done:
                    active.pop(future)
                    future.result()
            if now - last_progress >= 5.0:
                print_progress(
                    campaign.name, progress, len(active), args.trials)
                last_progress = now
    return concurrency


def choose_concurrency(args, calibration_metadata, pilot_ok):
    if args.maximum_concurrency <= 1 or not pilot_ok:
        return 1
    total = psutil.virtual_memory().total
    peak = calibration_metadata.get('process_tree_peak_rss_bytes', 0)
    memory_cap = max(1, int((total * 0.65) // peak)) if peak else 2
    return max(1, min(args.maximum_concurrency, 3, memory_cap))


def benchmark_map_analysis(attempt, free_threshold, occupied_threshold,
                           shift_window):
    """Benchmark actual calibration geometry using cached aligned arrays."""
    first = load_map(attempt / 'robot1_final_shared_map.npz')
    second = load_map(attempt / 'robot2_final_shared_map.npz')
    canonical, _, _ = canonical_map(
        first, second, None, free_threshold, occupied_threshold)
    maps = [canonical] * 10
    process = psutil.Process()
    rss_before = process.memory_info().rss
    start = time.perf_counter()
    geometry = common_geometry(maps)
    common_grid_time = time.perf_counter() - start
    start = time.perf_counter()
    aligned = [
        resample_semantic(
            item, geometry, free_threshold, occupied_threshold)
        for item in maps
    ]
    alignment_time = time.perf_counter() - start
    start = time.perf_counter()
    compare_semantic(
        aligned[0], aligned[1], geometry.resolution, shift_window)
    one_pair_time = time.perf_counter() - start
    start = time.perf_counter()
    for left in range(10):
        for right in range(left + 1, 10):
            compare_semantic(
                aligned[left], aligned[right], geometry.resolution,
                shift_window)
    all_pairs_time = time.perf_counter() - start
    start = time.perf_counter()
    consensus_maps(
        maps, geometry.resolution, free_threshold, occupied_threshold)
    consensus_time = time.perf_counter() - start
    return {
        'source': 'calibration final map geometry',
        'width': geometry.width, 'height': geometry.height,
        'resolution': geometry.resolution,
        'common_grid_time_s': common_grid_time,
        'cached_alignment_time_s': alignment_time,
        'one_pair_comparison_time_s': one_pair_time,
        'all_45_pair_comparisons_time_s': all_pairs_time,
        'consensus_generation_time_s': consensus_time,
        'rss_increase_bytes': max(
            0, process.memory_info().rss - rss_before),
    }


def build_and_test(workspace, skip_build, skip_tests):
    if not skip_build:
        result = run([
            'colcon', 'build', '--packages-select', 'my_epuck_project',
            '--symlink-install',
        ], cwd=workspace, timeout=900)
        if result.returncode:
            raise RuntimeError(f'build failed:\n{result.stdout}')
    if not skip_tests:
        result = run([
            sys.executable, '-m', 'pytest', '-q',
            'src/my_epuck_project/test/test_cooperative_regression.py',
            'src/my_epuck_project/test/test_cooperative_world_profiles.py',
            'src/my_epuck_project/test/test_cooperative_manual_rviz.py',
            'src/my_epuck_project/test/test_cooperative_trial_collector.py',
            'src/my_epuck_project/test/test_occupancy_map_comparison.py',
            'src/my_epuck_project/test/test_map_fusion_geometry.py',
            'src/my_epuck_project/test/test_experiment_logger_runtime.py',
            'src/my_epuck_project/test/test_experiment_metrics.py',
            'src/my_epuck_project/test/'
            'test_teammate_scan_filter_geometry.py',
        ], cwd=workspace, timeout=600)
        if result.returncode:
            raise RuntimeError(f'deterministic tests failed:\n{result.stdout}')
        result = run([
            'colcon', 'test', '--packages-select',
            'my_epuck_cooperative_exploration',
            'my_epuck_frontier_candidates',
            '--event-handlers', 'console_direct+',
        ], cwd=workspace, timeout=600)
        if result.returncode:
            raise RuntimeError(
                f'coordinator/frontier tests failed:\n{result.stdout}')
        for package in (
                'my_epuck_cooperative_exploration',
                'my_epuck_frontier_candidates'):
            result = run([
                'colcon', 'test-result', '--verbose',
                '--test-result-base', f'build/{package}',
            ], cwd=workspace, timeout=60)
            if result.returncode:
                raise RuntimeError(
                    f'{package} test results failed:\n{result.stdout}')


def final_cleanup_audit(campaign, manifest):
    """Prove no campaign-owned process, port, or open result file remains."""
    campaign_text = str(campaign.resolve())
    selected_ports = range(
        manifest['webots_port_range'][0],
        manifest['webots_port_range'][1] + 1)
    occupied_ports = [
        port for port in selected_ports
        if linux_port_used(port) or windows_port_pid(port) is not None
    ]
    associated_processes = []
    open_result_files = []
    for process in psutil.process_iter(['pid', 'cmdline']):
        try:
            command = ' '.join(process.info.get('cmdline') or [])
            if campaign_text in command and process.pid != os.getpid():
                associated_processes.append({
                    'pid': process.pid, 'command': command,
                })
            for item in process.open_files():
                if item.path.startswith(campaign_text + os.sep):
                    open_result_files.append({
                        'pid': process.pid, 'path': item.path,
                    })
        except (psutil.AccessDenied, psutil.NoSuchProcess, OSError):
            pass
    result = {
        'utc': utc_now(),
        'occupied_selected_ports': occupied_ports,
        'remaining_campaign_processes': associated_processes,
        'open_result_files': open_result_files,
        'clean': not (
            occupied_ports or associated_processes or open_result_files),
    }
    atomic_json(campaign / 'final_cleanup_audit.json', result)
    return result


def create_manifest(args, campaign, workspace):
    webots = webots_information()
    if not all(webots['supported_options'].values()):
        raise RuntimeError(
            f'installed Webots lacks required options: {webots}')
    commit = git_value(workspace, ['rev-parse', 'HEAD'])
    dirty = git_value(workspace, ['status', '--porcelain'], '')
    command = ' '.join([shlex_quote(item) for item in sys.argv])
    manifest = {
        'schema_version': '1.0.0',
        'campaign_id': campaign.name,
        'git_commit': commit,
        'dirty_worktree': bool(dirty),
        'dirty_worktree_status': dirty.splitlines(),
        'utc_start': utc_now(),
        **host_metadata(),
        'webots_version': webots['version'],
        'webots_executable': webots['executable'],
        'webots_mode': 'fast' if args.fast_mode else 'realtime',
        'rendering_mode': 'enabled' if args.rendering else 'disabled',
        'exact_webots_options': (
            f'--port=<trial-port> --batch '
            f'--mode={"fast" if args.fast_mode else "realtime"}'
            + ('' if args.rendering
               else ' --no-rendering --stdout --stderr --minimize')),
        'simulation_time_measurement': 'unavailable',
        'world_profile': args.world_profile,
        'world': args.profile_metadata['world'],
        'source_world_path': args.source_world_path,
        'installed_world_path': args.installed_world_path,
        'world_dimensions_m':
            args.profile_metadata['world_dimensions_m'],
        'world_sha256': args.profile_metadata['world_sha256'],
        'robot_start_poses': args.profile_metadata['robot_start_poses'],
        'known_initial_relative_transform':
            args.profile_metadata['known_relative_transform'],
        'slam_resolution': args.profile_metadata['slam_resolution'],
        'peer_export_resolution':
            args.profile_metadata['peer_export_resolution'],
        'fusion_resolution': args.profile_metadata['fusion_resolution'],
        'global_costmap_resolution':
            args.profile_metadata['global_costmap_resolution'],
        'local_costmap_resolution':
            args.profile_metadata['local_costmap_resolution'],
        'lidar_maximum_range':
            args.profile_metadata['lidar_maximum_range'],
        'initial_map_costmap_configuration': {
            'minimum_frontier_cells':
                args.profile_metadata['minimum_frontier_cells'],
            'minimum_known_cell_gain_for_activity':
                args.profile_metadata[
                    'minimum_known_cell_gain_for_activity'],
            'coverage_attribution_resolution':
                args.profile_metadata['coverage_attribution_resolution'],
            'map_comparison_shift_window':
                args.profile_metadata['map_comparison_shift_window'],
        },
        'launch_file':
            'two_robots_observed_continuous_exploration_launch.py',
        'launch_arguments': {
            'world_profile': args.world_profile,
            'use_sim_time': False,
            'coordinator_mode': 'continuous',
            'one_goal_only': False,
            'launch_rviz': args.rviz,
            'hold_open_after_completion':
                args.hold_open_after_completion,
        },
        'trial_count_requested': args.trials,
        'ros_domain_id_range': [
            args.ros_domain_base,
            args.ros_domain_base + args.trials - 1],
        'webots_port_range': [
            args.webots_port_base,
            args.webots_port_base + args.trials - 1],
        'map_thresholds': {
            'unknown': 'value < 0', 'free_max': args.free_threshold,
            'occupied_min': args.occupied_threshold,
            'uncertain': (
                f'{args.free_threshold} < value '
                f'< {args.occupied_threshold}'),
        },
        'timeouts': {
            'startup_s': args.startup_timeout,
            'mission_s': args.mission_timeout,
            'settling_s': args.settling_period,
            'graceful_shutdown_s': args.graceful_shutdown_timeout,
            'hard_shutdown_s': args.hard_shutdown_timeout,
        },
        'concurrency_policy': {
            'requested_maximum': args.maximum_concurrency,
            'normal_cap': 3, 'hard_default_cap': 4,
            'overload_cpu_percent': 90,
            'minimum_available_memory_fraction': 0.20,
        },
        'random_seeds': {
            'controlled': False,
            'reason': 'no project/Webots seed is exposed by committed launch',
        },
        'reproduction_command': command,
        'limitations': [
            'Webots simulation time is not published because use_sim_time=false '
            'is intentionally preserved; no real-time factor is invented.',
        ],
    }
    atomic_json(campaign / 'campaign_manifest.json', manifest)
    return manifest


def shlex_quote(value):
    import shlex
    return shlex.quote(value)


def campaign_main(args):
    workspace = Path(args.workspace).resolve()
    output_root = Path(args.output_root).resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    campaign_id = args.campaign_id or safe_campaign_id()
    campaign = output_root / campaign_id
    if args.analysis_only or args.regenerate_report:
        if not campaign.exists():
            raise RuntimeError(
                f'analysis campaign does not exist: {campaign}')
    elif args.resume:
        if not campaign.exists():
            raise RuntimeError(f'resume campaign does not exist: {campaign}')
    elif campaign.exists():
        raise RuntimeError(f'campaign directory already exists: {campaign}')
    else:
        campaign.mkdir()
        (campaign / 'attempts').mkdir()
    if args.world_profile is None:
        recorded_profile = None
        manifest_path = campaign / 'campaign_manifest.json'
        if manifest_path.exists():
            try:
                recorded = json.loads(
                    manifest_path.read_text(encoding='utf-8'))
                recorded_profile = recorded.get('world_profile')
                if recorded_profile is None and recorded.get('world') == (
                        'epuck_d500_two_world_teammate_visible.wbt'):
                    recorded_profile = 'small'
            except (OSError, ValueError):
                pass
        apply_profile_defaults(args, recorded_profile or 'large')
    else:
        apply_profile_defaults(args)
    if args.analysis_only or args.regenerate_report:
        progress = load_progress(campaign)
        refresh_selected_classifications(campaign, progress)
        manifest = json.loads(
            (campaign / 'campaign_manifest.json').read_text())
        calibration_attempt_id = manifest.get('calibration_attempt_id')
        if calibration_attempt_id:
            calibration_attempt = (
                campaign / 'attempts' / calibration_attempt_id)
            try:
                manifest['calibration_result'] = json.loads(
                    (calibration_attempt / 'runner_metadata.json')
                    .read_text(encoding='utf-8'))
                manifest['offline_map_benchmark'] = benchmark_map_analysis(
                    calibration_attempt, args.free_threshold,
                    args.occupied_threshold, args.shift_window)
            except (OSError, ValueError, KeyError):
                pass
        manifest['final_cleanup_audit'] = final_cleanup_audit(
            campaign, manifest)
        atomic_json(campaign / 'campaign_manifest.json', manifest)
        summary = analyze_campaign(
            campaign, args.free_threshold, args.occupied_threshold,
            args.shift_window)
        print(f'campaign_report={summary["artifact_paths"]["report"]}')
        print(f'campaign_summary={campaign / "campaign_summary.json"}')
        return 0
    build_and_test(workspace, args.skip_build, args.skip_tests)
    selected_profile = resolve_runner_profile(args)
    print_profile_selection(args, selected_profile)
    progress = load_progress(campaign)
    completed = set()
    for trial, relative in list(progress['valid_trials'].items()):
        if validate_existing_attempt(campaign / relative):
            completed.add(int(trial.rsplit('_', 1)[1]))
        else:
            del progress['valid_trials'][trial]
    if args.resume:
        manifest = json.loads(
            (campaign / 'campaign_manifest.json').read_text())
        commit = git_value(workspace, ['rev-parse', 'HEAD'])
        if manifest.get('git_commit') != commit:
            raise RuntimeError(
                'resume commit differs from campaign manifest')
        previous_startup = manifest.get(
            'timeouts', {}).get('startup_s')
        if previous_startup != args.startup_timeout:
            manifest.setdefault('configuration_adjustments', []).append({
                'utc': utc_now(),
                'field': 'startup_timeout_s',
                'from': previous_startup,
                'to': args.startup_timeout,
                'reason': (
                    'calibration Webots/controller readiness exceeded the '
                    'initial bound without host overload'),
            })
            manifest.setdefault('timeouts', {})['startup_s'] = (
                args.startup_timeout)
            atomic_json(campaign / 'campaign_manifest.json', manifest)
        previous_maximum = manifest.get(
            'concurrency_policy', {}).get('requested_maximum')
        if previous_maximum != args.maximum_concurrency:
            manifest.setdefault('configuration_adjustments', []).append({
                'utc': utc_now(),
                'field': 'maximum_concurrency',
                'from': previous_maximum,
                'to': args.maximum_concurrency,
                'reason': (
                    'two-instance isolation pilot exceeded the readiness '
                    'bound; sequential fallback selected'),
            })
            manifest.setdefault(
                'concurrency_policy', {})['requested_maximum'] = (
                    args.maximum_concurrency)
            atomic_json(campaign / 'campaign_manifest.json', manifest)
    else:
        manifest = create_manifest(args, campaign, workspace)
    validate_resource_range(args, completed)
    update_progress(campaign, progress)
    start = time.monotonic()
    calibration = None
    pilot_ok = False
    try:
        remaining = [
            number for number in range(1, args.trials + 1)
            if number not in completed]
        existing_calibration = manifest.get('calibration_result')
        if args.calibration and remaining and not existing_calibration:
            number = remaining.pop(0)
            calibration = perform_attempt(
                args, campaign, progress, number)
            completed.add(number)
            calibration_attempt = campaign / progress['valid_trials'][
                f'trial_{number:02d}']
            benchmark = benchmark_map_analysis(
                calibration_attempt, args.free_threshold,
                args.occupied_threshold, args.shift_window)
            atomic_json(campaign / 'offline_map_benchmark.json', benchmark)
        else:
            calibration = existing_calibration or {}
            benchmark = manifest.get('offline_map_benchmark')
        if (args.isolation_pilot and args.maximum_concurrency >= 2
                and len(remaining) >= 2):
            pilot_numbers = remaining[:2]
            remaining = remaining[2:]
            run_parallel_stage(
                args, campaign, progress, pilot_numbers, 2)
            pilot_metadata = [
                json.loads((campaign / progress['valid_trials'][
                    f'trial_{number:02d}'] / 'runner_metadata.json'
                ).read_text()) for number in pilot_numbers
            ]
            pilot_ok = all(
                item.get('ready')
                or item.get('classification_details', {}).get('ready')
                for item in pilot_metadata)
            pilot_ok = pilot_ok and all(
                item.get('cleanup_complete') for item in pilot_metadata)
            if not pilot_ok:
                manifest['limitations'].append(
                    'Two-instance isolation pilot failed; remaining trials '
                    'are sequential.')
        elif args.maximum_concurrency == 1 and any(
                item.get('trial_id') in ('trial_02', 'trial_03')
                and item.get('classification') == 'INFRASTRUCTURE_FAILURE'
                for item in progress.get('attempts', [])):
            limitation = (
                'Two-instance isolation used distinct domains, ports, '
                'process groups, and Windows Webots PIDs, but both pilot '
                'pairs exceeded startup readiness; the valid campaign '
                'continued sequentially.')
            if limitation not in manifest['limitations']:
                manifest['limitations'].append(limitation)
        chosen = choose_concurrency(args, calibration, pilot_ok)
        manifest['calibration_result'] = calibration
        manifest['offline_map_benchmark'] = benchmark
        manifest['isolation_pilot_passed'] = pilot_ok
        manifest['chosen_concurrency'] = chosen
        manifest['chosen_concurrency_reason'] = (
            'bounded by requested maximum, normal cap, calibration peak RSS, '
            'and isolation pilot outcome')
        atomic_json(campaign / 'campaign_manifest.json', manifest)
        if remaining:
            final_concurrency = run_parallel_stage(
                args, campaign, progress, remaining, chosen)
            manifest['final_concurrency'] = final_concurrency
    except KeyboardInterrupt:
        progress['interrupted'] = True
        update_progress(campaign, progress)
        manifest['clean_shutdown'] = False
        manifest['utc_end'] = utc_now()
        atomic_json(campaign / 'campaign_manifest.json', manifest)
        return 130
    manifest['active_execution_wall_time_s'] = time.monotonic() - start
    manifest['utc_end'] = utc_now()
    try:
        started = datetime.fromisoformat(
            manifest['utc_start'].replace('Z', '+00:00'))
        ended = datetime.fromisoformat(
            manifest['utc_end'].replace('Z', '+00:00'))
        manifest['total_wall_time_s'] = (
            ended - started).total_seconds()
    except (KeyError, TypeError, ValueError):
        manifest['total_wall_time_s'] = (
            manifest['active_execution_wall_time_s'])
    manifest['clean_shutdown'] = True
    manifest['adaptive_reductions'] = progress['adaptive_reductions']
    manifest['final_cleanup_audit'] = final_cleanup_audit(
        campaign, manifest)
    atomic_json(campaign / 'campaign_manifest.json', manifest)
    refresh_selected_classifications(campaign, progress)
    summary = analyze_campaign(
        campaign, args.free_threshold, args.occupied_threshold,
        args.shift_window)
    print(f'CAMPAIGN_COMPLETE id={campaign.name}')
    print(f'campaign_report={summary["artifact_paths"]["report"]}')
    print(f'campaign_summary={campaign / "campaign_summary.json"}')
    print(f'trials_csv={campaign / "trials.csv"}')
    print(f'pairwise_map_metrics={campaign / "pairwise_map_metrics.csv"}')
    return 0 if (
        summary['valid_trial_count'] == args.trials
        and summary['failure_count'] == 0
        and summary['all_processes_clean_boolean']
    ) else 2


def boolean(text):
    if isinstance(text, bool):
        return text
    lowered = text.lower()
    if lowered in ('true', '1', 'yes', 'on'):
        return True
    if lowered in ('false', '0', 'no', 'off'):
        return False
    raise argparse.ArgumentTypeError('expected true or false')


def apply_profile_defaults(args, profile_name=None):
    """Apply defaults only where the caller did not provide an override."""
    name = profile_name or args.world_profile or 'large'
    settings = PROFILE_SETTINGS[name]
    args.world_profile = name
    if args.startup_timeout is None:
        args.startup_timeout = settings['startup_timeout']
    if args.mission_timeout is None:
        args.mission_timeout = settings['mission_timeout']
    if args.shift_window is None:
        args.shift_window = settings['map_comparison_shift_window']
    return args


def parser():
    result = argparse.ArgumentParser(
        description='Automated isolated cooperative exploration regression')
    result.add_argument('--trials', type=int, default=10)
    result.add_argument('--maximum-concurrency', type=int, default=3)
    result.add_argument('--campaign-id')
    result.add_argument('--output-root', default='results')
    result.add_argument('--workspace', default='/home/arash/webots_ws')
    result.add_argument(
        '--world-profile',
        choices=['large', 'small'],
        default=None,
        help=(
            'World/configuration profile; defaults to large for new runs and '
            'to the recorded profile when resuming or analyzing'),
    )
    result.add_argument('--resume', action='store_true')
    result.add_argument('--ros-domain-base', type=int, default=100)
    result.add_argument('--webots-port-base', type=int, default=23000)
    result.add_argument('--startup-timeout', type=float)
    result.add_argument('--mission-timeout', type=float)
    result.add_argument('--settling-period', type=float, default=4.0)
    result.add_argument(
        '--fast-mode', type=boolean, default=True, metavar='BOOL')
    result.add_argument(
        '--rendering', type=boolean, default=False, metavar='BOOL')
    result.add_argument(
        '--hold-open-after-completion',
        type=boolean,
        default=False,
        metavar='BOOL',
        help=(
            'After a verified single rendered mission, keep Webots/RViz '
            'open until Ctrl+C'),
    )
    result.add_argument(
        '--rviz', type=boolean, default=False, metavar='BOOL',
        help='Open the passive RViz view (single-trial use only)')
    result.add_argument(
        '--calibration', type=boolean, default=True, metavar='BOOL')
    result.add_argument(
        '--isolation-pilot', type=boolean, default=True, metavar='BOOL')
    result.add_argument('--analysis-only', action='store_true')
    result.add_argument('--regenerate-report', action='store_true')
    result.add_argument('--skip-build', action='store_true')
    result.add_argument('--skip-tests', action='store_true')
    result.add_argument('--free-threshold', type=int, default=25)
    result.add_argument('--occupied-threshold', type=int, default=65)
    result.add_argument('--shift-window', type=int)
    result.add_argument('--graceful-shutdown-timeout', type=float, default=20.0)
    result.add_argument('--hard-shutdown-timeout', type=float, default=10.0)
    # Private per-attempt interface used only by campaign supervisors.
    result.add_argument('--internal-trial', action='store_true',
                        help=argparse.SUPPRESS)
    result.add_argument('--attempt-dir', help=argparse.SUPPRESS)
    result.add_argument('--trial-id', help=argparse.SUPPRESS)
    result.add_argument('--attempt-id', help=argparse.SUPPRESS)
    result.add_argument('--run-id', help=argparse.SUPPRESS)
    result.add_argument('--ros-domain-id', type=int, help=argparse.SUPPRESS)
    result.add_argument('--webots-port', type=int, help=argparse.SUPPRESS)
    result.add_argument('--source-world-path', help=argparse.SUPPRESS)
    result.add_argument('--webots-mode', help=argparse.SUPPRESS)
    result.add_argument('--webots-gui', type=boolean, help=argparse.SUPPRESS)
    result.add_argument('--launch-rviz', type=boolean, help=argparse.SUPPRESS)
    return result


def validate_cli_options(args):
    """Validate campaign-level manual GUI constraints."""
    if args.trials < 1:
        raise SystemExit('--trials must be positive')
    if not 1 <= args.maximum_concurrency <= 4:
        raise SystemExit('--maximum-concurrency must be between 1 and 4')
    if (not args.internal_trial and args.rviz
            and (args.trials != 1 or args.maximum_concurrency != 1)):
        raise SystemExit(
            '--rviz true requires --trials 1 --maximum-concurrency 1')
    if (not args.internal_trial and args.hold_open_after_completion
            and (args.trials != 1 or args.maximum_concurrency != 1)):
        raise SystemExit(
            '--hold-open-after-completion true requires '
            '--trials 1 --maximum-concurrency 1')
    if (not args.internal_trial and args.hold_open_after_completion
            and not args.rendering):
        raise SystemExit(
            '--hold-open-after-completion true requires --rendering true')


def main(argv=None):
    args = parser().parse_args(argv)
    validate_cli_options(args)
    try:
        if args.internal_trial:
            apply_profile_defaults(args, args.world_profile or 'small')
            if args.source_world_path is None:
                args.source_world_path = ''
            return internal_trial(args)
        return campaign_main(args)
    except (RuntimeError, ValueError, OSError) as error:
        print(f'INFRASTRUCTURE_ERROR: {error}', file=sys.stderr)
        return 3


if __name__ == '__main__':
    raise SystemExit(main())
