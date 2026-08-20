"""Minimal single-trial runner for the cooperative Webots stack.

This module intentionally does not import the campaign runner.  It launches
the same authoritative exploration launch, waits for only the interfaces and
nodes needed for development, and writes three small attempt artifacts.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from functools import partial
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time

from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
import psutil
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from .cooperative_profiles import (
    PROFILE_SETTINGS, profile_for_world, profile_summary)


WORKSPACE = Path('/home/arash/webots_ws')
PACKAGE = 'my_epuck_project'
LAUNCH_FILE = 'two_robots_decentralized_exploration_launch.py'
NAV2_NODES = (
    'controller_server', 'smoother_server', 'planner_server',
    'route_server', 'behavior_server', 'velocity_smoother',
    'collision_monitor', 'bt_navigator', 'waypoint_follower',
)
LOCAL_NAV2_NODES = tuple(f'local_{name}' for name in NAV2_NODES)
GRAPH_SUFFIXES = (
    '/robot1/distributed_frontier_assignment',
    '/robot2/distributed_frontier_assignment',
    '/robot1/map_fusion',
    '/robot2/map_fusion',
)
SHUTDOWN_GRACE_S = 20.0
SHUTDOWN_TERM_S = 10.0


class FastTrialError(RuntimeError):
    """A bounded preflight, readiness, or cleanup failure."""


def boolean(value: str | bool) -> bool:
    if isinstance(value, bool):
        return value
    lowered = value.lower()
    if lowered in ('true', '1', 'yes', 'on'):
        return True
    if lowered in ('false', '0', 'no', 'off'):
        return False
    raise argparse.ArgumentTypeError('expected true or false')


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        description='Fast single-trial cooperative Webots runner')
    result.add_argument('--world-profile', choices=(
        'small', 'large', 'large_unknown_pose', 'large_unknown_pose_16m'),
                        default='large')
    result.add_argument('--world-path', default='')
    result.add_argument('--sensor-profile', choices=('full', 'throughput'),
                        default='full')
    result.add_argument('--ideal-encoder-sensing', type=boolean, default=True,
                        help='Use project-local noiseless, unlimited-resolution wheel sensors.')
    result.add_argument('--webots-mode', default='fast')
    result.add_argument('--rendering', type=boolean, default=False)
    result.add_argument('--rviz', type=boolean, default=False)
    result.add_argument('--diagnostic-mode', type=boolean, default=False)
    result.add_argument(
        '--enable-observer', type=boolean, default=False,
        help='Enable the passive cooperative evidence recorder.')
    result.add_argument(
        '--enable-forensic-capture', type=boolean, default=False,
        help='Enable passive Webots Supervisor/map forensic capture.')
    result.add_argument('--fusion-process-nice', type=int, default=0)
    result.add_argument('--slam-tf-publish-probe-library', default='')
    result.add_argument('--slam-tf-publish-probe-log', default='')
    result.add_argument('--slam-tf-publication-mode', default='')
    result.add_argument('--mission-timeout', type=float)
    result.add_argument('--startup-timeout', type=float)
    result.add_argument('--ros-domain-id', type=int, default=100)
    result.add_argument('--webots-port', type=int, default=23000)
    result.add_argument('--results-directory', default='results/fast_trials')
    result.add_argument('--hold-open', action='store_true')
    return result


def package_prefix() -> str:
    """Return the package prefix through the same ROS lookup users invoke."""
    ros2 = shutil.which('ros2')
    if ros2 is None:
        raise FastTrialError('ros2 is not available in PATH')
    result = subprocess.run(
        [ros2, 'pkg', 'prefix', PACKAGE],
        check=False, capture_output=True, text=True, timeout=10,
    )
    prefix = result.stdout.strip().splitlines()[-1] if result.stdout.strip() else ''
    print(prefix or result.stderr.strip(), flush=True)
    if result.returncode != 0 or not prefix:
        raise FastTrialError(
            f'ROS package lookup failed: {result.stderr.strip()}')
    expected = WORKSPACE / 'install' / PACKAGE
    if Path(prefix).resolve() != expected.resolve():
        raise FastTrialError(
            f'{PACKAGE} resolves to {prefix}, expected {expected}; '
            'build/source the intended webots_ws first')
    return prefix


def port_is_free(port: int) -> bool:
    if not 1024 <= port <= 65535:
        raise FastTrialError('--webots-port must be between 1024 and 65535')
    with socket.socket() as stream:
        stream.settimeout(0.25)
        if stream.connect_ex(('127.0.0.1', port)) == 0:
            return False
    netstat = Path('/mnt/c/Windows/System32/netstat.exe')
    if netstat.exists():
        result = subprocess.run(
            [str(netstat), '-ano', '-p', 'tcp'],
            check=False, capture_output=True, text=True, timeout=10,
        )
        needle = f':{port}'
        for line in result.stdout.splitlines():
            if needle in line and 'LISTENING' in line.upper():
                return False
    return True


def is_campaign_webots_driver(command: list[str]) -> bool:
    """Identify only this project's namespaced ros2_control driver processes."""
    text = ' '.join(command)
    return (
        'webots_ros2_driver/lib/webots_ros2_driver/driver' in text and
        any(
            f'__ns:=/{robot}' in text and
            f'/tmp/my_epuck_project_{robot}_ros2_control.yml' in text
            for robot in ('robot1', 'robot2')
        )
    )


def campaign_webots_drivers(created_after: float | None = None):
    drivers = []
    for process in psutil.process_iter(['pid', 'cmdline', 'create_time']):
        try:
            command = process.info.get('cmdline') or []
            if not is_campaign_webots_driver(command):
                continue
            if (created_after is not None and
                    process.info.get('create_time', 0.0) < created_after):
                continue
            drivers.append(process)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return drivers


def cleanup_campaign_webots_drivers(created_after: float) -> list[int]:
    drivers = campaign_webots_drivers(created_after)
    pids = [process.pid for process in drivers]
    for process in drivers:
        try:
            process.terminate()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    _, alive = psutil.wait_procs(drivers, timeout=5.0)
    for process in alive:
        try:
            process.kill()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    return pids


def resolve_world(args: argparse.Namespace) -> Path:
    source_worlds = WORKSPACE / 'src' / PACKAGE / 'worlds'
    selected = profile_for_world(
        args.world_profile, source_worlds,
        explicit_world_path=args.world_path,
        ideal_encoder_sensing=args.ideal_encoder_sensing)
    args.profile_metadata = profile_summary(selected)
    return Path(selected['world_path']).resolve()


def prepare_attempt(args: argparse.Namespace, prefix: str, world: Path):
    results = Path(args.results_directory).expanduser().resolve()
    results.mkdir(parents=True, exist_ok=True)
    if not results.is_dir():
        raise FastTrialError(f'results directory is not a directory: {results}')
    attempt = results / f'fast_trial_{datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")}'
    attempt.mkdir()
    return attempt


def launch_command(
        args: argparse.Namespace, world: Path, output_root: Path | None = None,
        run_id: str = '') -> list[str]:
    command = [
        'ros2', 'launch', PACKAGE, LAUNCH_FILE,
        f'world_profile:={args.world_profile}',
        f'world_path:={world}',
        f'webots_port:={args.webots_port}',
        f'webots_mode:={args.webots_mode}',
        f'webots_gui:={str(args.rendering).lower()}',
        f'sensor_profile:={args.sensor_profile}',
        f'ideal_encoder_sensing:={str(args.ideal_encoder_sensing).lower()}',
        f'diagnostic_mode:={str(args.diagnostic_mode).lower()}',
        f'fusion_process_nice:={args.fusion_process_nice}',
        'use_sim_time:=true',
        'nav2_autostart:=false',
        'dispatch_enabled:=true',
        # This runner is the unknown-pose full-exploration campaign entry
        # point; do not silently fall back to the known-relative launch mode.
        'unknown_initial_pose:=true',
        # The authoritative unknown-pose wrapper uses the frozen production
        # RPP controller.  Pass this explicitly through the nested launch
        # chain so an inherited/duplicate launch argument cannot select the
        # historical DWB diagnostic variant for the pre-handoff local stack.
        'controller_variant:=rpp',
        f'enable_observer:={str(args.enable_observer).lower()}',
        f'enable_forensic_capture:={str(args.enable_forensic_capture).lower()}',
        'launch_rviz:=false',
        'enable_mission_timeout:=false',
    ]
    if output_root is not None:
        command.extend([
            f'output_root:={output_root}',
            f'run_id:={run_id}',
            # The full unknown-pose launch consumes this supported parameter
            # directly. Keep frontend diagnostics beside this run's artifacts.
            f'unknown_pose_diagnostic_output:={output_root / run_id / "frontend"}',
        ])
    optional_launch_arguments = (
        ('slam_tf_publish_probe_library', args.slam_tf_publish_probe_library),
        ('slam_tf_publish_probe_log', args.slam_tf_publish_probe_log),
        ('slam_tf_publication_mode', args.slam_tf_publication_mode),
    )
    command.extend(
        f'{name}:={value}'
        for name, value in optional_launch_arguments
        if value
    )
    return command


class ReadyProbe(Node):
    """Small ROS graph probe used only by the fast runner."""

    def __init__(self):
        super().__init__('cooperative_trial_fast_probe')
        self.clock_values: list[float] = []
        self.scans: set[str] = set()
        self.odometry: set[str] = set()
        self.maps: set[str] = set()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=60.0))
        self.tf_listener = TransformListener(self.tf_buffer, self,
                                             spin_thread=False)
        self.executor = None
        map_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Clock, '/clock', self._on_clock, qos_profile_sensor_data)
        for robot in ('robot1', 'robot2'):
            self.create_subscription(
                LaserScan, f'/{robot}/scan_d500_fixed',
                partial(self._mark_scan, robot),
                qos_profile_sensor_data)
            self.create_subscription(
                Odometry, f'/{robot}/odom',
                partial(self._mark_odom, robot),
                qos_profile_sensor_data)
            self.create_subscription(
                OccupancyGrid, f'/{robot}/map',
                partial(self._mark_map, robot), map_qos)

    def _mark_scan(self, robot, message):
        del message
        self.scans.add(robot)

    def _mark_odom(self, robot, message):
        del message
        self.odometry.add(robot)

    def _mark_map(self, robot, message):
        del message
        self.maps.add(robot)

    def _on_clock(self, message: Clock):
        value = message.clock.sec + message.clock.nanosec * 1e-9
        if not self.clock_values or value != self.clock_values[-1]:
            self.clock_values.append(value)
            self.clock_values = self.clock_values[-4:]

    def clock_ready(self) -> bool:
        return len(self.clock_values) >= 2 and any(
            right > left for left, right in zip(
                self.clock_values, self.clock_values[1:]))

    def robot_interfaces_ready(self) -> bool:
        return self.scans == {'robot1', 'robot2'} and \
            self.odometry == {'robot1', 'robot2'}

    def maps_ready(self) -> bool:
        return self.maps == {'robot1', 'robot2'}

    def tf_ready(self) -> bool:
        for robot in ('robot1', 'robot2'):
            for target, source in (
                    (f'{robot}/map', f'{robot}/base_footprint'),
                    (f'{robot}/base_footprint', f'{robot}/odom')):
                if not self.tf_buffer.can_transform(
                        target, source, Time(),
                        timeout=Duration(seconds=0.0)):
                    return False
        return True

    def node_names(self) -> set[str]:
        return {
            f'{namespace.rstrip("/")}/{name}'
            for name, namespace in self.get_node_names_and_namespaces()
        }

    def cooperation_graph_ready(self) -> bool:
        names = self.node_names()
        return all(item in names for item in GRAPH_SUFFIXES)

    def spin_once(self, timeout_sec: float):
        if self.executor is not None:
            self.executor.spin_once(timeout_sec=timeout_sec)
        else:
            rclpy.spin_once(self, timeout_sec=timeout_sec)

    def _wait_future(self, future, deadline: float):
        while not future.done() and time.monotonic() < deadline:
            self.spin_once(0.1)
        return future.result() if future.done() else None

    def activate_and_check_nav2(self, deadline: float) -> dict:
        # Unknown-pose exploration starts in local-map mode.  Its local
        # lifecycle managers autostart; the shared managers must remain
        # waiting until the frontend handoff.  Poll activation to the same
        # bounded deadline because the second namespaced manager may still be
        # bringing up its nodes when the first state query completes.
        clients = {}
        for robot in ('robot1', 'robot2'):
            for node_name in LOCAL_NAV2_NODES:
                service_name = f'/{robot}/{node_name}/get_state'
                clients[(robot, node_name)] = self.create_client(
                    GetState, service_name)
        while time.monotonic() < deadline:
            all_active = True
            details = {}
            for robot in ('robot1', 'robot2'):
                active = {}
                for node_name in LOCAL_NAV2_NODES:
                    client = clients[(robot, node_name)]
                    if not client.service_is_ready():
                        client.wait_for_service(timeout_sec=0.0)
                    if not client.service_is_ready():
                        active[node_name] = False
                        all_active = False
                        continue
                    response = self._wait_future(
                        client.call_async(GetState.Request()), deadline)
                    state_id = response.current_state.id if response else -1
                    active[node_name] = state_id == State.PRIMARY_STATE_ACTIVE
                    all_active = all_active and active[node_name]
                details[robot] = active
            if all_active:
                return details
            self.spin_once(0.1)
        raise FastTrialError(
            'local Nav2 activation timed out: %s' % details)


def pump_output(stream, log_file, lock: threading.Lock):
    try:
        for line in iter(stream.readline, ''):
            with lock:
                log_file.write(line)
                log_file.flush()
                print(line, end='', flush=True)
    finally:
        stream.close()


def stop_process(process: subprocess.Popen, sig: int):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, sig)
    except ProcessLookupError:
        pass


def wait_process(process: subprocess.Popen, timeout: float) -> bool:
    try:
        process.wait(timeout=timeout)
        return True
    except subprocess.TimeoutExpired:
        return False


def shutdown_processes(launch, rviz=None) -> dict:
    stop_process(launch, signal.SIGINT)
    if rviz is not None:
        stop_process(rviz, signal.SIGINT)
    graceful = wait_process(launch, SHUTDOWN_GRACE_S)
    if not graceful:
        stop_process(launch, signal.SIGTERM)
        wait_process(launch, SHUTDOWN_TERM_S)
    if launch.poll() is None:
        stop_process(launch, signal.SIGKILL)
        wait_process(launch, SHUTDOWN_TERM_S)
    if rviz is not None and rviz.poll() is None:
        stop_process(rviz, signal.SIGTERM)
        if not wait_process(rviz, 5.0):
            stop_process(rviz, signal.SIGKILL)
            wait_process(rviz, 5.0)
    return {
        'graceful': graceful,
        'launch_return_code': launch.returncode,
        'rviz_return_code': rviz.returncode if rviz is not None else None,
    }


def rviz_command() -> list[str]:
    share = Path(get_package_share_directory(PACKAGE))
    config = share / 'resource' / 'cooperative_manual_exploration.rviz'
    return ['rviz2', '-d', str(config), '--ros-args',
            '-p', 'use_sim_time:=true']


def run(args: argparse.Namespace) -> int:
    started = time.monotonic()
    started_wall = time.time()
    start_time = utc_now()
    attempt = None
    launch = None
    rviz = None
    log_file = None
    output_threads = []
    lock = threading.Lock()
    ready_time = None
    phases = {}
    exit_reason = 'preflight_failure'
    launch_return_code = None
    prefix = ''
    world = None
    ready_wall_elapsed = None
    launch_spawn_elapsed = None
    peak_runner_rss = 0
    rclpy_started = False
    probe = None
    executor = None
    previous_domain = os.environ.get('ROS_DOMAIN_ID')

    def sample_runner_rss():
        nonlocal peak_runner_rss
        try:
            peak_runner_rss = max(
                peak_runner_rss,
                psutil.Process(os.getpid()).memory_info().rss,
            )
        except psutil.Error:
            pass

    try:
        prefix = package_prefix()
        world = resolve_world(args)
        stale_drivers = campaign_webots_drivers()
        if stale_drivers:
            raise FastTrialError(
                'stale campaign Webots drivers are still running: %s' %
                [process.pid for process in stale_drivers])
        if not port_is_free(args.webots_port):
            raise FastTrialError(f'Webots port is already in use: {args.webots_port}')
        attempt = prepare_attempt(args, prefix, world)
        command = launch_command(
            args, world, output_root=attempt / 'observer',
            run_id=attempt.name)
        effective = (
            f'ROS_DOMAIN_ID={args.ros_domain_id} '
            + ' '.join(shlex.quote(item) for item in command)
            + '\n')
        (attempt / 'effective_command.txt').write_text(effective, encoding='utf-8')
        print(f'launch_file={LAUNCH_FILE}', flush=True)
        print(f'world_path={world}', flush=True)
        print(f'results_directory={attempt}', flush=True)
        print(f'effective_command={effective.strip()}', flush=True)

        environment = os.environ.copy()
        environment.update({
            'ROS_DOMAIN_ID': str(args.ros_domain_id),
            'PYTHONUNBUFFERED': '1',
        })
        log_file = (attempt / 'launch.log').open('w', encoding='utf-8')
        launch = subprocess.Popen(
            command, env=environment, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, bufsize=1,
            start_new_session=True)
        launch_spawn_elapsed = time.monotonic() - started
        print(f'launch_spawn_elapsed_s={launch_spawn_elapsed:.3f}', flush=True)
        output_threads.append(threading.Thread(
            target=pump_output, args=(launch.stdout, log_file, lock),
            daemon=True))
        output_threads[-1].start()
        if args.rviz:
            rviz = subprocess.Popen(
                rviz_command(), env=environment, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, bufsize=1,
                start_new_session=True)
            output_threads.append(threading.Thread(
                target=pump_output, args=(rviz.stdout, log_file, lock),
                daemon=True))
            output_threads[-1].start()
        print(f'direct_subprocesses={1 + int(rviz is not None)}', flush=True)

        os.environ['ROS_DOMAIN_ID'] = str(args.ros_domain_id)
        rclpy.init()
        rclpy_started = True
        probe = ReadyProbe()
        executor = SingleThreadedExecutor()
        executor.add_node(probe)
        probe.executor = executor
        readiness_deadline = started + (args.startup_timeout or 300.0)
        checks = (
            ('clock_ready', probe.clock_ready),
            ('robot_interfaces_ready', probe.robot_interfaces_ready),
            ('maps_ready', probe.maps_ready),
            ('tf_ready', probe.tf_ready),
            ('cooperation_graph_ready', probe.cooperation_graph_ready),
        )
        for name, predicate in checks:
            phase_start = time.monotonic()
            print(f'{name}_waiting timeout_s={max(0.0, readiness_deadline - phase_start):.1f}', flush=True)
            while time.monotonic() < readiness_deadline:
                sample_runner_rss()
                if launch.poll() is not None:
                    raise FastTrialError(
                        f'launch exited during {name} with code {launch.returncode}')
                if predicate():
                    elapsed = time.monotonic() - phase_start
                    phases[name] = elapsed
                    print(f'{name}_s={elapsed:.3f}', flush=True)
                    break
                executor.spin_once(timeout_sec=0.1)
            else:
                raise FastTrialError(
                    f'{name} timed out after {args.startup_timeout or 300.0:.1f}s')

        phase_start = time.monotonic()
        # Nav2's lifecycle managers can be present before all controller and
        # costmap services have finished initializing.  Keep this final gate
        # inside the same bounded startup contract, rather than imposing a
        # shorter fixed deadline that can expire while the second namespaced
        # manager is still bringing up its nodes.
        nav2_deadline = readiness_deadline
        print(
            'nav2_ready_waiting timeout_s=%.1f' % max(
                0.0, nav2_deadline - phase_start), flush=True)
        nav2_details = probe.activate_and_check_nav2(nav2_deadline)
        phases['nav2_ready'] = time.monotonic() - phase_start
        print(f'nav2_ready_s={phases["nav2_ready"]:.3f}', flush=True)
        probe.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        ready_time = utc_now()
        ready_duration = time.monotonic() - started
        ready_wall_elapsed = ready_duration
        phases['total_ready'] = ready_duration
        print(f'total_ready_s={ready_duration:.3f}', flush=True)
        print('FAST_TRIAL_READY', flush=True)

        mission_deadline = None if args.hold_open else (
            time.monotonic() + (args.mission_timeout or 1800.0))
        while True:
            if launch.poll() is not None:
                launch_return_code = launch.returncode
                exit_reason = 'launch_process_exit'
                return_code = 1
                break
            if not args.hold_open and time.monotonic() >= mission_deadline:
                exit_reason = 'mission_timeout'
                break
            time.sleep(0.2)
            sample_runner_rss()
        return_code = 0
    except KeyboardInterrupt:
        exit_reason = 'interrupt'
        return_code = 130
    except (FastTrialError, OSError, subprocess.SubprocessError) as error:
        exit_reason = f'failure: {error}'
        print(f'FAST_TRIAL_FAILURE reason={error}', file=sys.stderr, flush=True)
        return_code = 1
    except Exception as error:
        exit_reason = f'unexpected_failure: {type(error).__name__}: {error}'
        print(f'FAST_TRIAL_FAILURE reason={exit_reason}', file=sys.stderr,
              flush=True)
        return_code = 1
    finally:
        if probe is not None:
            probe.destroy_node()
        if executor is not None:
            executor.shutdown()
        if rclpy_started and rclpy.ok():
            rclpy.shutdown()
        if previous_domain is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = previous_domain
        if launch is not None:
            cleanup = shutdown_processes(launch, rviz)
            launch_return_code = cleanup['launch_return_code']
        else:
            cleanup = {}
        cleanup['campaign_webots_driver_pids'] = cleanup_campaign_webots_drivers(
            started_wall)
        if log_file is not None:
            for thread in output_threads:
                thread.join(timeout=2.0)
            log_file.close()
        end_time = utc_now()
        summary = {
            'start_time': start_time,
            'ready_time': ready_time,
            'end_time': end_time,
            'wall_runtime_s': time.monotonic() - started,
            'ready_duration_s': ready_wall_elapsed,
            'launch_spawn_elapsed_s': launch_spawn_elapsed,
            'readiness_phase_durations': phases,
            'exit_reason': exit_reason,
            'launch_return_code': launch_return_code,
            'world_profile': args.world_profile,
            'world_path': str(world) if world else args.world_path,
            'profile_metadata': getattr(args, 'profile_metadata', {}),
            'sensor_profile': args.sensor_profile,
            'rendering': args.rendering,
            'rviz': args.rviz,
            'diagnostic_mode': args.diagnostic_mode,
            'ros_domain_id': args.ros_domain_id,
            'webots_port': args.webots_port,
            'launch_file': LAUNCH_FILE,
            'cleanup': cleanup,
            'direct_subprocesses': 1 + int(rviz is not None),
            'peak_runner_rss_bytes': peak_runner_rss,
            'prefix': prefix,
            'nav2': locals().get('nav2_details', {}),
        }
        if attempt is not None:
            (attempt / 'fast_trial_summary.json').write_text(
                json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    return return_code


def main(argv=None) -> int:
    args = parser().parse_args(argv)
    settings = PROFILE_SETTINGS[args.world_profile]
    if args.mission_timeout is None:
        args.mission_timeout = settings['mission_timeout']
    if args.startup_timeout is None:
        args.startup_timeout = settings['startup_timeout']
    if args.ros_domain_id < 0 or args.ros_domain_id > 232:
        raise SystemExit('--ros-domain-id must be between 0 and 232')
    return run(args)


if __name__ == '__main__':
    raise SystemExit(main())
