"""Bounded ROS/Windows runtime checks used before diagnostic launches."""

from __future__ import annotations

import json
import hashlib
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import subprocess
import sys
from urllib.parse import unquote, urlparse

import psutil

# CycloneDDS/RTPS default unicast-port arithmetic is:
#
#   PB + DG * domain + d3 + PG * participant_index
#
# with the Jazzy Cyclone defaults PB=7400, DG=250, d3=11, PG=2.  The
# simulation profile permits automatic participant indices through 200.  A
# domain of 231 or 232 can therefore produce an endpoint above the maximum
# UDP port (65535); the observed failure was repeated writes to 65536.  Keep
# the bound derived from the active profile rather than accepting the ROS 2
# domain-ID maximum blindly.
DDS_PORT_BASE = 7400
DDS_DOMAIN_GAIN = 250
DDS_UNICAST_OFFSET = 11
DDS_PARTICIPANT_GAIN = 2
DDS_MAX_AUTO_PARTICIPANT_INDEX = 200
DDS_PORT_MAX = 65535
ROS_DOMAIN_MIN = 0
ROS_DOMAIN_MAX = min(
    232,
    (DDS_PORT_MAX - DDS_PORT_BASE - DDS_UNICAST_OFFSET
     - DDS_PARTICIPANT_GAIN * DDS_MAX_AUTO_PARTICIPANT_INDEX)
    // DDS_DOMAIN_GAIN,
)
ROS_DAEMON_BASE_PORT = 11511


def dds_max_unicast_port(domain):
    """Return the highest CycloneDDS unicast port for a domain."""
    return (DDS_PORT_BASE + DDS_DOMAIN_GAIN * int(domain)
            + DDS_UNICAST_OFFSET
            + DDS_PARTICIPANT_GAIN * DDS_MAX_AUTO_PARTICIPANT_INDEX)


class PreflightError(RuntimeError):
    """A required pre-launch runtime condition is not safe."""


CRITICAL_RUNTIME_MODULES = (
    'my_epuck_project.cooperative_regression',
    'my_epuck_project.ros_runtime_preflight',
    'my_epuck_project.unknown_pose_frontend',
    'my_epuck_project.unknown_pose_frontend_core',
    'my_epuck_project.robust_relative_pose_selector',
    # These modules are the allocator/Nav2 ownership runtime closure.  They
    # must be checked in the install actually visible to the ROS child; a
    # matching package prefix is not sufficient provenance.
    'my_epuck_project.distributed_frontier_assignment',
    'my_epuck_project.distributed_assignment.local_nav2',
    'my_epuck_project.round_lifecycle',
    'my_epuck_project.distributed_assignment.protocol',
    'my_epuck_project.distributed_assignment.scoring',
    'my_epuck_project.distributed_assignment.canonical',
    'my_epuck_project.distributed_assignment.failures',
    'my_epuck_project.distributed_assignment.models',
    'my_epuck_project.distributed_assignment.ros_conversion',
    'my_epuck_project.distributed_assignment.traffic_scheduler',
    'my_epuck_project.mission_termination',
)


def _sha256_file(path):
    try:
        digest = hashlib.sha256()
        with Path(path).open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
        return digest.hexdigest()
    except (OSError, TypeError):
        return None


def _sha256_tree(root):
    """Hash a source tree deterministically without including build metadata."""
    root = Path(root)
    if not root.is_dir():
        return None
    digest = hashlib.sha256()
    try:
        for path in sorted(item for item in root.rglob('*') if item.is_file()):
            relative = path.relative_to(root).as_posix()
            if any(part in {'build', 'install', '.git', '__pycache__'}
                   for part in path.parts):
                continue
            digest.update(relative.encode('utf-8'))
            digest.update(b'\0')
            with path.open('rb') as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                    digest.update(chunk)
            digest.update(b'\0')
    except OSError:
        return None
    return digest.hexdigest()


def _ament_package_prefix(package_name, environment):
    """Resolve a package prefix using only the selected AMENT underlays."""
    for value in environment.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        if not value:
            continue
        prefix = Path(value).resolve()
        marker = (prefix / 'share/ament_index/resource_index/packages' /
                  package_name)
        if marker.is_file():
            return prefix
    return None


def _workspace_path(path, workspace):
    """Return whether a path is inside the selected validation checkout."""
    try:
        Path(path).resolve().relative_to(Path(workspace).resolve())
        return True
    except (OSError, ValueError, TypeError):
        return False


def _module_relative_path(module_name):
    """Return the source-relative Python path for a project module."""
    parts = module_name.split('.')
    if not parts or parts[0] != 'my_epuck_project' or len(parts) < 2:
        raise ValueError(f'unsupported runtime module: {module_name}')
    return Path(*parts[1:]).with_suffix('.py')


def _path_below(path, root):
    """Return whether *path* is contained by *root* after resolution."""
    try:
        Path(path).resolve().relative_to(Path(root).resolve())
        return True
    except (OSError, TypeError, ValueError):
        return False


def _resolve_runtime_module_paths(workspace, environment):
    """Resolve modules using a clean child with exactly *environment*.

    The launcher invokes the source-tree runner, whose wrapper intentionally
    inserts the source package into its own ``sys.path``.  Looking up modules
    in that process would therefore validate the wrapper rather than the ROS
    child.  Probe through a fresh interpreter so PYTHONPATH/overlay precedence
    is the same as the process that will import the installed package.
    """
    code = (
        'import importlib.util, json, sys; '
        'names = json.loads(sys.argv[1]); result = {}; '
        'for_name = None\n'
        'for for_name in names:\n'
        '    try:\n'
        '        spec = importlib.util.find_spec(for_name)\n'
        '        result[for_name] = spec.origin if spec is not None else None\n'
        '    except Exception as error:\n'
        '        result[for_name] = {"error": repr(error)}\n'
        'print(json.dumps(result, sort_keys=True))'
    )
    completed = subprocess.run(
        [sys.executable, '-c', code,
         json.dumps(list(CRITICAL_RUNTIME_MODULES))],
        cwd=workspace, env=dict(environment), capture_output=True,
        text=True, check=False, timeout=30)
    if completed.returncode:
        return {}, [
            'runtime module resolution probe failed: ' +
            (completed.stderr.strip() or completed.stdout.strip())]
    try:
        result = json.loads(completed.stdout.strip().splitlines()[-1])
    except (ValueError, IndexError):
        return {}, ['runtime module resolution probe returned invalid JSON']
    return result, []


def _collect_runtime_module_provenance(workspace, environment,
                                       resolved_paths=None):
    """Compare exact imported modules with source and selected build files.

    ``resolved_paths`` is injectable for pure tests.  Production callers leave
    it unset, causing resolution through the exact child environment.
    """
    workspace = Path(workspace).resolve()
    environment = dict(environment)
    build_base = Path(environment.get(
        'MY_EPUCK_BUILD_BASE', workspace / 'build')).resolve()
    install_prefix_text = environment.get('MY_EPUCK_INSTALL_PREFIX', '').strip()
    install_prefix = (Path(install_prefix_text).resolve()
                      if install_prefix_text else None)
    if resolved_paths is None:
        resolved_paths, resolution_issues = _resolve_runtime_module_paths(
            workspace, environment)
    else:
        resolution_issues = []
    records = {}
    issues = list(resolution_issues)
    source_root = workspace / 'src/my_epuck_project/my_epuck_project'
    build_root = build_base / 'my_epuck_project/my_epuck_project'
    for module_name in CRITICAL_RUNTIME_MODULES:
        relative = _module_relative_path(module_name)
        source_path = (source_root / relative).resolve()
        build_path = (build_root / relative).resolve()
        imported_value = resolved_paths.get(module_name)
        imported_path = None
        resolution_error = None
        if isinstance(imported_value, dict):
            resolution_error = imported_value.get('error')
        elif imported_value:
            imported_path = Path(imported_value).resolve()
        record = {
            'module': module_name,
            'resolved_path': str(imported_path) if imported_path else None,
            'imported_path': str(imported_path) if imported_path else None,
            'source_path': str(source_path),
            'build_path': str(build_path),
            'install_prefix': str(install_prefix) if install_prefix else None,
            'install_path': str(imported_path) if imported_path else None,
            'resolved_sha256': _sha256_file(imported_path),
            'imported_sha256': _sha256_file(imported_path),
            'source_sha256': _sha256_file(source_path),
            'build_sha256': _sha256_file(build_path),
            'install_sha256': _sha256_file(imported_path),
        }
        records[module_name] = record
        if resolution_error:
            issues.append(f'{module_name} resolution failed: {resolution_error}')
        if imported_path is None:
            issues.append(f'{module_name} is missing from the exact runtime environment')
            continue
        if install_prefix is not None and not _path_below(
                imported_path, install_prefix):
            issues.append(
                f'{module_name} resolved outside selected install prefix: '
                f'{imported_path}')
        if record['source_sha256'] is None:
            issues.append(f'{module_name} source file is missing: {source_path}')
        if record['build_sha256'] is None:
            issues.append(f'{module_name} build file is missing: {build_path}')
        if record['install_sha256'] is None:
            issues.append(f'{module_name} imported file is unreadable: {imported_path}')
        if (record['source_sha256'] != record['build_sha256'] or
                record['source_sha256'] != record['install_sha256']):
            issues.append(
                f'installed module hash differs from source/build: {module_name}')
    return records, issues


def runtime_provenance(workspace, environment=None, ros_domain_id=None):
    """Audit middleware, import paths, and source/build parity before launch.

    This check is intentionally independent of ROS graph discovery.  A shell
    that inherited the original dirty checkout can otherwise import an older
    runner and bypass the DDS domain guard before any preflight report exists.
    """
    workspace = Path(workspace).resolve()
    environment = dict(environment or os.environ)
    dirty_root = Path('/home/arash/webots_ws').resolve()
    allowed_external_prefixes = []
    for value in environment.get('MY_EPUCK_ALLOWED_EXTERNAL_PREFIXES', '').split(
            os.pathsep):
        if value:
            allowed_external_prefixes.append(Path(value).resolve())
    issues = []
    rmw = environment.get('RMW_IMPLEMENTATION', '').strip()
    cyclone_uri = environment.get('CYCLONEDDS_URI', '').strip()
    if rmw != 'rmw_cyclonedds_cpp':
        issues.append('RMW_IMPLEMENTATION must be rmw_cyclonedds_cpp')
    parsed_uri = urlparse(cyclone_uri)
    uri_path = Path(unquote(parsed_uri.path)).resolve() if (
        parsed_uri.scheme == 'file' and parsed_uri.path) else None
    expected_uri_path = (workspace / 'config/cyclonedds/wsl_loopback.xml').resolve()
    if uri_path != expected_uri_path:
        issues.append(
            'CYCLONEDDS_URI must point to the validation WSL loopback profile')
    if ros_domain_id is not None:
        try:
            inherited_domain = environment.get('ROS_DOMAIN_ID', '').strip()
            if inherited_domain and int(inherited_domain) != int(ros_domain_id):
                issues.append(
                    f'inherited ROS_DOMAIN_ID={inherited_domain} differs from '
                    f'requested domain {int(ros_domain_id)}')
        except ValueError:
            issues.append('inherited ROS_DOMAIN_ID is not an integer')
    contaminated = []
    for variable in ('PYTHONPATH', 'AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH'):
        for value in environment.get(variable, '').split(os.pathsep):
            if not value:
                continue
            try:
                resolved = Path(value).resolve()
            except OSError:
                continue
            if resolved == dirty_root or dirty_root in resolved.parents:
                if (not _workspace_path(resolved, workspace)
                        and not any(
                            resolved == prefix or prefix in resolved.parents
                            for prefix in allowed_external_prefixes)):
                    contaminated.append({
                        'variable': variable, 'path': str(resolved)})
    if contaminated:
        issues.append('environment references the original dirty checkout')

    runtime_module_provenance, module_issues = (
        _collect_runtime_module_provenance(workspace, environment))
    issues.extend(module_issues)
    module_paths = {
        name: record['resolved_path']
        for name, record in runtime_module_provenance.items()}
    module_hashes = {
        name: record['resolved_sha256']
        for name, record in runtime_module_provenance.items()}

    parity = {}
    frontier_prefix = _ament_package_prefix(
        'frontier_exploration_ros2', environment)
    frontier_source = workspace / 'src/frontier-exploration-ros2'
    expected_frontier_prefix = Path(
        environment.get('MY_EPUCK_FRONTIER_PREFIX', workspace / 'install'))
    frontier_report = {
        'package': 'frontier_exploration_ros2',
        'prefix': str(frontier_prefix) if frontier_prefix else None,
        'expected_prefix': str(expected_frontier_prefix.resolve()),
        'source': str(frontier_source),
        'source_tree_sha256': _sha256_tree(frontier_source),
        'pinned_commit': '476aaf4',
    }
    if frontier_prefix is None:
        issues.append('frontier_exploration_ros2 is not present in selected AMENT prefixes')
    else:
        if not _workspace_path(frontier_prefix, workspace):
            configured_prefix = expected_frontier_prefix.resolve()
            if frontier_prefix != configured_prefix:
                issues.append(
                    'frontier_exploration_ros2 resolved outside validation install: '
                    f'{frontier_prefix}')
        if dirty_root in frontier_prefix.parents and not _workspace_path(
                frontier_prefix, workspace):
            issues.append('frontier_exploration_ros2 resolved from original dirty checkout')
        config = (frontier_prefix / 'share/frontier_exploration_ros2/cmake/'
                  'frontier_exploration_ros2Config.cmake')
        if not config.is_file():
            issues.append(f'frontier_exploration_ros2 config missing: {config}')

    # The cooperative launch chain has a real package-closure requirement.
    # Resolve every project package from the selected overlay instead of
    # allowing an inherited /home/arash/webots_ws install to satisfy only the
    # first import or launch lookup.
    required_project_packages = (
        'my_epuck_project', 'my_epuck_interfaces',
        'my_epuck_frontier_candidates', 'my_epuck_cooperative_exploration',
        'reliable_slam_toolbox_wrapper')
    project_package_prefixes = {}
    for package_name in required_project_packages:
        prefix = _ament_package_prefix(package_name, environment)
        project_package_prefixes[package_name] = (
            str(prefix) if prefix is not None else None)
        if prefix is None:
            issues.append(
                f'{package_name} is not present in selected AMENT prefixes')
            continue
        if dirty_root in prefix.parents and not _workspace_path(
                prefix, workspace):
            issues.append(
                f'{package_name} resolved from stale project install: {prefix}')

    # A campaign may deliberately use an isolated colcon overlay (for
    # example, ``build_current_<commit>``) rather than the workspace's
    # default ``build`` directory.  Provenance must inspect the exact build
    # selected by the launch environment; silently falling back to
    # ``workspace/build`` can report a stale mismatch while the running
    # modules are correct.  Keep the default for backwards compatibility,
    # but allow the runner to bind this check to its explicit build base.
    build_base = Path(environment.get('MY_EPUCK_BUILD_BASE',
                                      workspace / 'build')).resolve()
    for relative in (
            'cooperative_regression.py', 'ros_runtime_preflight.py',
            'unknown_pose_frontend.py', 'unknown_pose_frontend_core.py',
            'robust_relative_pose_selector.py'):
        source = workspace / 'src/my_epuck_project/my_epuck_project' / relative
        build = build_base / 'my_epuck_project/my_epuck_project' / relative
        parity[relative] = {
            'source': str(source), 'build': str(build),
            'source_sha256': _sha256_file(source),
            'build_sha256': _sha256_file(build),
        }
        if parity[relative]['source_sha256'] != parity[relative]['build_sha256']:
            issues.append(f'source/build mismatch: {relative}')

    return {
        'workspace': str(workspace),
        'rmw_implementation': rmw,
        'cyclonedds_uri': cyclone_uri,
        'expected_cyclonedds_uri': f'file://{expected_uri_path}',
        'requested_ros_domain_id': ros_domain_id,
        'module_paths': module_paths,
        'module_sha256': module_hashes,
        'runtime_module_provenance': runtime_module_provenance,
        'source_build_parity': parity,
        'frontier_dependency': frontier_report,
        'project_package_prefixes': project_package_prefixes,
        'contaminated_environment_paths': contaminated,
        'allowed_external_prefixes': [str(path) for path in allowed_external_prefixes],
        'issues': issues,
        'passed': not issues,
    }


def require_runtime_provenance(workspace, environment=None, ros_domain_id=None):
    """Raise before launch when campaign provenance is not reproducible."""
    report = runtime_provenance(workspace, environment, ros_domain_id)
    if not report['passed']:
        raise PreflightError('; '.join(report['issues']))
    return report


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
        'webots-controller', 'webots_ros2_driver', 'slam_toolbox',
        'frontier_candidate_generator', 'source_aware_map_fusion',
        'map_exporter', 'teammate_scan_filter', 'robot_state_publisher',
        'controller_server', 'planner_server', 'smoother_server',
        'route_server', 'behavior_server', 'velocity_smoother',
        'collision_monitor', 'bt_navigator', 'waypoint_follower',
        'lifecycle_manager',
    )
    excluded = _current_process_ancestry()
    result = []
    for process in psutil.process_iter(['pid', 'status', 'cmdline']):
        pid = process.info.get('pid')
        if pid in excluded:
            continue
        command = ' '.join(process.info.get('cmdline') or [])
        lowered = command.lower()
        is_project_ros = (
            any(marker in lowered for marker in markers)
            and ('/robot1' in lowered or '/robot2' in lowered
                 or 'my_epuck_project' in lowered
                 or 'webots' in lowered
                 or 'slam_toolbox' in lowered))
        if is_project_ros:
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
        'dds_max_auto_participant_index': DDS_MAX_AUTO_PARTICIPANT_INDEX,
        'dds_max_unicast_port': dds_max_unicast_port(domain),
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
            f'{ROS_DOMAIN_MAX} for the active CycloneDDS port profile; '
            f'domain {domain} can reach UDP port '
            f'{dds_max_unicast_port(domain)}')
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
