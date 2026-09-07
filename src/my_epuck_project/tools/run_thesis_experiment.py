#!/usr/bin/env python3
"""Thin canonical bootstrap around the existing thesis runner."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys


WORKSPACE = Path(__file__).resolve().parents[3]
ROS_SETUP = Path('/opt/ros/jazzy/setup.bash')
PROJECT_INSTALL = WORKSPACE / 'install'
LOOPBACK_SETUP = WORKSPACE / 'scripts/ros2_wsl_cyclonedds_loopback.sh'
DRIVER_PREFIX = Path('/home/arash/webots_ws_close_validation_2eb/install/webots_ros2_driver')
WORLD_PROFILE = 'large_unknown_pose_close_start_20ms_scan_matching'
DEFAULT_DOMAIN = 230
DEFAULT_PORT = 23420
VENDOR_LIBRARY = '/opt/ros/jazzy/opt/sdformat_vendor/lib'
LDD_TARGETS = (Path('/opt/ros/jazzy/lib/libsdformat_urdf_plugin.so'),
               Path('/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher'))
RECORDED_ENV = ('MY_EPUCK_WORKSPACE', 'MY_EPUCK_INSTALL_PREFIX',
                'MY_EPUCK_FRONTIER_PREFIX', 'MY_EPUCK_BUILD_BASE',
                'MY_EPUCK_WEBOTS_DRIVER_PREFIX',
                'MY_EPUCK_WEBOTS_NETWORK_MODE', 'RUN_OUT', 'ROS_DOMAIN_ID',
                'RMW_IMPLEMENTATION', 'CYCLONEDDS_URI', 'AMENT_PREFIX_PATH',
                'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'PYTHONPATH',
                'LD_LIBRARY_PATH', 'WSL_INTEROP')


LauncherError = RuntimeError


def _positive(value: str) -> float:
    try:
        result = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('expected a positive number') from exc
    if result <= 0.0:
        raise argparse.ArgumentTypeError('expected a positive number')
    return result


def _parse_env0(data: bytes) -> dict[str, str]:
    return {item.split('=', 1)[0]: item.split('=', 1)[1]
            for item in data.decode().split('\0') if '=' in item}


def source_clean_environment() -> dict[str, str]:
    interop = os.environ.get('WSL_INTEROP', '').strip()
    if not interop:
        raise LauncherError('WSL_INTEROP is required in WSL NAT mode')
    seed = {'HOME': '/home/arash', 'PATH': '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin', 'WSL_INTEROP': interop}
    script = f'source {shlex.quote(str(ROS_SETUP))}; source {shlex.quote(str(PROJECT_INSTALL / "setup.bash"))}; source {shlex.quote(str(LOOPBACK_SETUP))}; env -0'
    completed = subprocess.run(
        ['bash', '--noprofile', '--norc', '-c', script],
        env=seed, capture_output=True, check=False, timeout=30)
    if completed.returncode:
        raise LauncherError(f'clean ROS environment failed: '
                             f'{completed.stderr.decode().strip()}')
    return _parse_env0(completed.stdout)


def canonical_environment(base: dict[str, str], result_root: Path,
                           domain: int, port: int) -> dict[str, str]:
    env = dict(base)
    env.update({
        'MY_EPUCK_WORKSPACE': str(WORKSPACE),
        'MY_EPUCK_INSTALL_PREFIX': str(PROJECT_INSTALL / 'my_epuck_project'),
        'MY_EPUCK_FRONTIER_PREFIX': str(PROJECT_INSTALL / 'frontier_exploration_ros2'),
        'MY_EPUCK_BUILD_BASE': str(WORKSPACE / 'build'),
        'MY_EPUCK_WEBOTS_DRIVER_PREFIX': str(DRIVER_PREFIX),
        'MY_EPUCK_WEBOTS_NETWORK_MODE': 'nat',
        'MY_EPUCK_DEFER_SYNC_MAP_FRAMES': '1',
        'RUN_OUT': str(result_root),
        'ROS_DOMAIN_ID': str(domain),
    })
    return env


def _helper(env: dict[str, str], code: str, *args: str) -> str:
    completed = subprocess.run(
        [sys.executable, '-c', code, *args], cwd=WORKSPACE, env=env,
        capture_output=True, text=True, check=False, timeout=60)
    if completed.returncode: raise LauncherError(completed.stderr.strip() or completed.stdout.strip())
    return completed.stdout.strip()


def apply_existing_filter(env: dict[str, str]) -> None:
    code = ('import json,os;from my_epuck_project.cooperative_trial_fast import '
            'filtered_runtime_environment;print(json.dumps('
            'filtered_runtime_environment(dict(os.environ))[0]))')
    filtered = json.loads(_helper(env, code))
    env.clear()
    env.update(filtered)


def _ldd(env: dict[str, str]) -> dict[str, str]:
    reports = {}
    for target in LDD_TARGETS:
        if not target.is_file(): raise LauncherError(f'missing loader target: {target}')
        result = subprocess.run(['ldd', str(target)], env=env, capture_output=True, text=True, check=False, timeout=30)
        output = result.stdout + result.stderr
        if result.returncode or 'not found' in output:
            raise LauncherError(f'ldd failed for {target}:\n{output}')
        reports[str(target)] = output
    return reports


def validate_launcher_values(env: dict[str, str], host: str,
                             result_root: Path) -> None:
    root = result_root.resolve()
    results = (WORKSPACE / 'results').resolve()
    if env.get('MY_EPUCK_WEBOTS_NETWORK_MODE') != 'nat':
        raise LauncherError('MY_EPUCK_WEBOTS_NETWORK_MODE must be nat')
    if host in {'127.0.0.1', 'localhost', '::1'}:
        raise LauncherError('WSL NAT endpoint must not be loopback')
    if (Path(env.get('MY_EPUCK_WEBOTS_DRIVER_PREFIX', '')).resolve() !=
            DRIVER_PREFIX.resolve()):
        raise LauncherError('approved Webots driver prefix is not selected')
    if root == WORKSPACE.resolve() or results not in root.parents: raise LauncherError(f'result root must be below {results}, not {root}')
    if env.get('RUN_OUT') != str(result_root): raise LauncherError('RUN_OUT does not equal the requested result root')
    if VENDOR_LIBRARY not in env.get('LD_LIBRARY_PATH', '').split(os.pathsep):
        raise LauncherError('ROS Jazzy vendor library path was lost')
    if not env.get('WSL_INTEROP'): raise LauncherError('WSL_INTEROP was not preserved')


def require_tracked_clean() -> None:
    if subprocess.run(['git', 'diff', '--quiet', 'HEAD', '--'],
                      cwd=WORKSPACE, check=False).returncode:
        raise LauncherError('tracked worktree is not clean')


def existing_preflight(env: dict[str, str], result_root: Path, domain: int,
                       port: int) -> dict[str, object]:
    code = '''import json, os, sys
from my_epuck_project.cooperative_trial_fast import package_prefix, webots_driver_provenance, port_is_free
from my_epuck_project.ros_runtime_preflight import require_runtime_provenance, run_preflight
root, out, domain, port = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
e = dict(os.environ)
if not port_is_free(port): raise SystemExit("Webots port is busy")
runtime = require_runtime_provenance(root, environment=e, ros_domain_id=domain)
prefix = package_prefix(e)
driver, executable = webots_driver_provenance(e)
ros = run_preflight(ros_domain_id=domain, webots_port=port, output_dir=out, environment=e)
print("CANONICAL_HELPER_RESULT=" + json.dumps({"runtime": runtime, "project_prefix": prefix, "driver_prefix": driver, "driver_executable": executable, "ros": ros}, sort_keys=True))'''
    output = _helper(env, code, str(WORKSPACE), str(result_root),
                     str(domain), str(port))
    marker = next((line for line in output.splitlines()
                   if line.startswith('CANONICAL_HELPER_RESULT=')), '')
    if not marker:
        raise LauncherError(f'existing preflight returned no result: {output}')
    return json.loads(marker.split('=', 1)[1])


def write_records(result_root: Path, env: dict[str, str], command: list[str],
                  config: dict[str, object], preflight: dict[str, object],
                  ldd: dict[str, str], host: str) -> None:
    result_root.mkdir(parents=True, exist_ok=True)
    (result_root / 'effective_environment.txt').write_text(
        ''.join(f'{key}={env.get(key, "")}\n' for key in RECORDED_ENV),
        encoding='utf-8')
    config = dict(config)
    config.update({'endpoint_host': host, 'preflight': preflight, 'ldd': ldd,
                   'downstream_command': command})
    (result_root / 'effective_configuration.json').write_text(
        json.dumps(config, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    (result_root / 'effective_command.txt').write_text(
        ' '.join(shlex.quote(item) for item in command) + '\n', encoding='utf-8')
    (result_root / 'preflight_environment.txt').write_text(
        json.dumps({'environment': {key: env.get(key, '') for key in RECORDED_ENV},
                    'network_mode': env['MY_EPUCK_WEBOTS_NETWORK_MODE'],
                    'endpoint_host': host, 'preflight': preflight},
                   indent=2, sort_keys=True) + '\n', encoding='utf-8')


def runner_command(args: argparse.Namespace, result_root: Path, port: int) -> list[str]:
    runner = WORKSPACE / 'src/my_epuck_project/tools/run_cooperative_trial_fast.py'
    return [sys.executable, str(runner), '--world-profile', WORLD_PROFILE,
            '--experiment-condition', args.condition, '--webots-random-seed', '1001',
            '--sensor-profile', 'full', '--ideal-encoder-sensing', 'true',
            '--webots-mode', 'fast', '--rendering', 'false', '--rviz', 'false',
            '--enable-observer', 'true', '--observer-architecture', 'legacy',
            '--enable-forensic-capture', 'true', '--enable-contact-capture', 'true',
            '--enable-scientific-raw-capture', 'true',
            '--assignment-strategy', 'frontier_cost_only',
            '--local-path-gate-mode', 'MODE_B', '--traffic-scheduler-enabled', 'true',
            '--prehandoff-dispatch-delay-s', '20.0', '--simulation-horizon-s', str(args.horizon),
            '--wall-watchdog-s', '1200', '--ros-domain-id', str(args.ros_domain_id),
            '--webots-port', str(port), '--results-directory', str(result_root)]


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--condition', choices=('A', 'B', 'C', 'D'), default='C')
    result.add_argument('--horizon', type=_positive, required=True)
    result.add_argument('--ros-domain-id', type=int, default=DEFAULT_DOMAIN)
    result.add_argument('--webots-port', type=int, default=DEFAULT_PORT)
    result.add_argument('--results-root', type=Path)
    result.add_argument('--dry-run', action='store_true')
    return result


def main(argv=None) -> int:
    args = parser().parse_args(argv)
    require_tracked_clean()
    if not 0 <= args.ros_domain_id <= 230: raise SystemExit('--ros-domain-id must be between 0 and 230')
    name = f'thesis_condition_{args.condition}_{args.horizon:g}s_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    result_root = (args.results_root or WORKSPACE / 'results' / name).resolve()
    env = canonical_environment(source_clean_environment(), result_root,
                                args.ros_domain_id, args.webots_port)
    apply_existing_filter(env)
    launch_file = WORKSPACE / 'src/my_epuck_project/launch/two_robots_namespaced_launch.py'
    host_code = ('import importlib.util, sys; s=importlib.util.spec_from_file_location('
                 '"canonical_launch", sys.argv[1]); m=importlib.util.module_from_spec(s); '
                 's.loader.exec_module(m); print(m._resolve_controller_host())')
    host = _helper(env, host_code, str(launch_file)).splitlines()[-1].strip()
    if not host: raise LauncherError('existing NAT resolver returned an empty host')
    validate_launcher_values(env, host, result_root)
    ldd = _ldd(env)
    preflight = existing_preflight(env, result_root, args.ros_domain_id, args.webots_port)
    command = runner_command(args, result_root, args.webots_port)
    config = {'head': subprocess.check_output(['git', 'rev-parse', 'HEAD'],
                                               cwd=WORKSPACE, text=True).strip(),
              'condition': args.condition,
              'horizon': args.horizon, 'world_profile': WORLD_PROFILE,
              'strategy': 'frontier_cost_only', 'mode': 'MODE_B', 'seed': 1001,
              'ros_domain_id': args.ros_domain_id, 'webots_port': args.webots_port,
              'project_prefix': preflight['project_prefix'], 'driver_prefix': preflight['driver_prefix'],
              'result_root': str(result_root), 'network_mode': 'nat'}
    write_records(result_root, env, command, config, preflight, ldd, host)
    print('CANONICAL_THESIS_PREFLIGHT_PASS')
    print(f'network_mode=nat endpoint_host={host}')
    print(f'driver_prefix={preflight["driver_prefix"]}')
    print(f'project_prefix={preflight["project_prefix"]}')
    print(f'result_root={result_root} ROS_domain={args.ros_domain_id}')
    if args.dry_run:
        print('dry_run=true; Webots not launched')
        return 0
    completed = subprocess.run(command, cwd=WORKSPACE, env=env, check=False)
    return completed.returncode


if __name__ == '__main__':
    raise SystemExit(main())
