"""Minimal single-trial runner for the cooperative Webots stack.

This module intentionally does not import the campaign runner.  It launches
the same authoritative exploration launch, waits for only the interfaces and
nodes needed for development, and writes three small attempt artifacts.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from functools import partial
import importlib.util
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
from nav2_msgs.srv import ManageLifecycleNodes
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
    PROFILE_SETTINGS, manual_rviz_path, profile, profile_for_world,
    profile_summary)
from .ros_runtime_preflight import (
    ROS_DOMAIN_MIN, ROS_DOMAIN_MAX, require_runtime_provenance,
)


_SOURCE_WORKSPACE = Path(__file__).resolve().parents[3]
WORKSPACE = Path(os.environ.get('MY_EPUCK_WORKSPACE', _SOURCE_WORKSPACE))
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
LOCAL_UNKNOWN_POSE_GRAPH_SUFFIXES = (
    '/robot1/local_distributed_frontier_assignment',
    '/robot2/local_distributed_frontier_assignment',
    '/robot1/unknown_pose_frontend',
    '/robot2/unknown_pose_frontend',
)
SHUTDOWN_GRACE_S = 20.0
SHUTDOWN_TERM_S = 10.0
# The public mission timeout is a hard total wall-clock cap.  Reserve enough
# time for launch-group escalation, campaign-driver cleanup, and final log
# flushing so a timeout cannot turn into an over-limit run.
FINALIZATION_BUDGET_S = 55.0


def _stale_workspace_root() -> Path:
    """Resolve the explicitly excluded sibling workspace for this checkout.

    The old workspace is deployment-specific.  Keep it configurable so the
    runner does not embed a developer's absolute home path in the package,
    while retaining the repository-sibling default used by the local setup.
    """
    configured = os.environ.get('MY_EPUCK_STALE_WORKSPACE_ROOT')
    if configured:
        return Path(configured).expanduser().resolve()
    return (WORKSPACE.parent / 'webots_ws').resolve()


STALE_WORKSPACE_ROOT = _stale_workspace_root()
FILTERED_PATH_VARIABLES = (
    'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH',
    'LD_LIBRARY_PATH', 'PATH', 'PKG_CONFIG_PATH', 'PYTHONPATH',
    'ROS_PACKAGE_PATH',
)


class FastTrialError(RuntimeError):
    """A bounded preflight, readiness, or cleanup failure."""


def _is_stale_workspace_entry(entry: str) -> bool:
    """Return true only for entries owned by the unrelated old workspace."""
    if not entry:
        return False
    try:
        path = Path(entry).expanduser().resolve()
    except OSError:
        return False
    return path == STALE_WORKSPACE_ROOT or STALE_WORKSPACE_ROOT in path.parents


def filtered_runtime_environment(source=None) -> tuple[dict[str, str], list[str]]:
    """Remove old-workspace search paths from the campaign child environment.

    The caller may have sourced the old workspace before the audit overlays.
    Only path-list entries below the exact unrelated workspace are removed;
    all other inherited environment values are retained.
    """
    environment = dict(os.environ if source is None else source)
    removed = []
    for variable in FILTERED_PATH_VARIABLES:
        value = environment.get(variable)
        if not value:
            continue
        kept = []
        for entry in value.split(os.pathsep):
            if _is_stale_workspace_entry(entry):
                removed.append(f'{variable}={entry}')
            else:
                kept.append(entry)
        environment[variable] = os.pathsep.join(kept)
    # Isolated project installs can retain a stale Webots-driver underlay in
    # their generated setup chain.  The explicit driver prefix is the
    # deployment authority; put it first in every lookup path that matters
    # before provenance checks or launch subprocesses resolve the package.
    driver_prefix = environment.get('MY_EPUCK_WEBOTS_DRIVER_PREFIX', '').strip()
    if driver_prefix:
        driver = str(Path(driver_prefix).expanduser().resolve())
        for variable in ('AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH',
                         'COLCON_PREFIX_PATH'):
            value = environment.get(variable, '')
            entries = [entry for entry in value.split(os.pathsep)
                       if entry and Path(entry).resolve() != Path(driver)]
            environment[variable] = os.pathsep.join([driver, *entries])
        python_entry = str(Path(driver) / 'lib/python3.12/site-packages')
        library_entry = str(Path(driver) / 'lib')
        for variable, entry in (
                ('PYTHONPATH', python_entry),
                ('LD_LIBRARY_PATH', library_entry)):
            value = environment.get(variable, '')
            entries = [item for item in value.split(os.pathsep)
                       if item and item != entry]
            environment[variable] = os.pathsep.join([entry, *entries])
    return environment, removed


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
        'small', 'large', 'large_unknown_pose', 'large_unknown_pose_16m',
        'large_unknown_pose_close_start',
        'large_unknown_pose_close_start_20ms',
        'large_unknown_pose_close_start_20ms_scan_matching',
        'large_unknown_pose_far_start_20ms_scan_matching'),
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
        '--diagnostic-frontier-capture', type=boolean, default=False,
        help='Enable opt-in per-frontier planner/costmap forensic capture.')
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
    result.add_argument(
        '--prehandoff-dispatch-delay-s', type=float, default=20.0,
        help='Wall-time hold before local goals may move robots during evidence acquisition.')
    result.add_argument(
        '--traffic-scheduler-enabled', type=boolean, default=False,
        help='Enable the existing decentralized pre-dispatch traffic gate.')
    result.add_argument(
        '--assignment-strategy',
        choices=('frontier_cost_only', 'frontier_mrtsp'),
        default='frontier_mrtsp',
        help='Distributed pair scoring mode.')
    result.add_argument(
        '--synchronized-traffic-test', type=boolean, default=False,
        help='Enable the test-only simulated-time synchronized dispatch barrier.')
    result.add_argument(
        '--traffic-test-force-conflict-pair', type=boolean, default=False,
        help='Test-only: select two real bid paths that geometrically conflict.')
    result.add_argument(
        '--synchronized-traffic-hold-prehandoff-motion', type=boolean,
        default=True,
        help='In synchronized test mode, hold local goals until handoff.')
    result.add_argument(
        '--enable-motion-fixture', type=boolean, default=False,
        help='Test-only odometry-confirmed motion fixture for unknown-pose '
             'evidence acquisition.')
    result.add_argument('--motion-fixture-start-delay-s', type=float, default=20.0)
    result.add_argument('--motion-fixture-turn-duration-s', type=float, default=3.2)
    result.add_argument('--motion-fixture-drive-duration-s', type=float, default=12.0)
    result.add_argument('--motion-fixture-cycles', type=int, default=1)
    result.add_argument('--motion-fixture-mirror-turns', type=boolean, default=True)
    result.add_argument('--motion-fixture-robot2-static', type=boolean, default=False)
    result.add_argument(
        '--motion-fixture-robot2-static-after-first-cycle', type=boolean,
        default=False)
    result.add_argument('--motion-fixture-linear-speed', type=float, default=0.10)
    result.add_argument(
        '--motion-fixture-robot2-linear-scale', type=float, default=1.0)
    result.add_argument('--motion-fixture-angular-speed', type=float, default=0.45)
    result.add_argument('--ros-domain-id', type=int, default=100)
    result.add_argument('--webots-port', type=int, default=23000)
    result.add_argument('--results-directory', default='results/fast_trials')
    result.add_argument('--hold-open', action='store_true')
    return result


def package_prefix(environment=None) -> str:
    """Return the package prefix through the same ROS lookup users invoke."""
    ros2 = shutil.which('ros2')
    if ros2 is None:
        raise FastTrialError('ros2 is not available in PATH')
    result = subprocess.run(
        [ros2, 'pkg', 'prefix', PACKAGE],
        check=False, capture_output=True, text=True, timeout=10,
        env=environment if environment is not None else None,
    )
    prefix = result.stdout.strip().splitlines()[-1] if result.stdout.strip() else ''
    print(prefix or result.stderr.strip(), flush=True)
    if result.returncode != 0 or not prefix:
        raise FastTrialError(
            f'ROS package lookup failed: {result.stderr.strip()}')
    expected_prefix = os.environ.get('MY_EPUCK_INSTALL_PREFIX', '')
    expected = (Path(expected_prefix).expanduser().resolve()
                if expected_prefix else WORKSPACE / 'install' / PACKAGE)
    if Path(prefix).resolve() != expected.resolve():
        raise FastTrialError(
            f'{PACKAGE} resolves to {prefix}, expected {expected}; '
            'set MY_EPUCK_INSTALL_PREFIX to the intended isolated install')
    return prefix


def webots_driver_provenance(environment=None) -> tuple[str, str]:
    """Resolve and validate the Webots driver used by the campaign launch."""
    ros2 = shutil.which('ros2')
    if ros2 is None:
        raise FastTrialError('ros2 is not available in PATH')
    result = subprocess.run(
        [ros2, 'pkg', 'prefix', 'webots_ros2_driver'],
        check=False, capture_output=True, text=True, timeout=10,
        env=environment if environment is not None else None,
    )
    prefix = result.stdout.strip().splitlines()[-1] if result.stdout.strip() else ''
    if result.returncode != 0 or not prefix:
        raise FastTrialError(
            f'Webots driver package lookup failed: {result.stderr.strip()}')
    lookup_environment = environment if environment is not None else os.environ
    expected_prefix = lookup_environment.get(
        'MY_EPUCK_WEBOTS_DRIVER_PREFIX', '')
    if not expected_prefix:
        raise FastTrialError(
            'MY_EPUCK_WEBOTS_DRIVER_PREFIX must identify the intended '
            'isolated Webots driver install')
    expected = Path(expected_prefix).expanduser().resolve()
    resolved = Path(prefix).resolve()
    if resolved != expected:
        raise FastTrialError(
            f'webots_ros2_driver resolves to {prefix}, expected {expected}; '
            'stale driver overlay refused')
    executable = resolved / 'lib' / 'webots_ros2_driver' / 'driver'
    if not executable.is_file():
        raise FastTrialError(
            f'Webots driver executable is missing: {executable}')
    return str(resolved), str(executable.resolve())


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
        (
            'webots_ros2_driver/lib/webots_ros2_driver/driver' in text or
            '/lib/webots_ros2_driver/driver' in text
        ) and
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
        f'diagnostic_frontier_capture:={str(args.diagnostic_frontier_capture).lower()}',
        f'fusion_process_nice:={args.fusion_process_nice}',
        'use_sim_time:=true',
        # Webots' ros2_control simulation requires the controller lifecycle
        # to be started after interfaces, maps, and local TF are ready.
        # ReadyProbe owns that single gated startup request; the project
        # allocator remains the only goal dispatcher.
        # Local Nav2 is deliberately started by ReadyProbe only after both
        # robot interfaces, maps, and local TF are ready.  Leaving the
        # lifecycle managers on automatic startup races that explicit gate:
        # they can begin configuring before odom/TF exists and a second
        # startup request can then collide with the in-flight transition.
        'nav2_autostart:=false',
        'dispatch_enabled:=true',
        f'traffic_scheduler_enabled:={str(args.traffic_scheduler_enabled).lower()}',
        f'assignment_strategy:={args.assignment_strategy}',
        f'synchronized_traffic_test:={str(args.synchronized_traffic_test).lower()}',
        f'traffic_test_force_conflict_pair:='
        f'{str(args.traffic_test_force_conflict_pair).lower()}',
        'synchronized_traffic_hold_prehandoff_motion:='
        f'{str(args.synchronized_traffic_hold_prehandoff_motion).lower()}',
        # Preserve the validated close-start evidence window: local frontier
        # dispatch is held while both peers accumulate overlap evidence.  The
        # handoff gates are unchanged; this only prevents navigation from
        # moving the robots out of the shared observation region prematurely.
        f'prehandoff_dispatch_delay_s:={args.prehandoff_dispatch_delay_s}',
        f'enable_motion_fixture:={str(args.enable_motion_fixture).lower()}',
        f'motion_fixture_start_delay_s:={args.motion_fixture_start_delay_s}',
        f'motion_fixture_turn_duration_s:={args.motion_fixture_turn_duration_s}',
        f'motion_fixture_drive_duration_s:={args.motion_fixture_drive_duration_s}',
        f'motion_fixture_cycles:={args.motion_fixture_cycles}',
        f'motion_fixture_mirror_turns:={str(args.motion_fixture_mirror_turns).lower()}',
        f'motion_fixture_robot2_static:={str(args.motion_fixture_robot2_static).lower()}',
        'motion_fixture_robot2_static_after_first_cycle:='
        f'{str(args.motion_fixture_robot2_static_after_first_cycle).lower()}',
        f'motion_fixture_linear_speed:={args.motion_fixture_linear_speed}',
        f'motion_fixture_robot2_linear_scale:={args.motion_fixture_robot2_linear_scale}',
        f'motion_fixture_angular_speed:={args.motion_fixture_angular_speed}',
        # This runner is the unknown-pose full-exploration campaign entry
        # point; do not silently fall back to the known-relative launch mode.
        'unknown_initial_pose:=true',
        # The scan-matching close-start validation exercises the full-map
        # startup architecture.  Keep the runner explicit so a launch-file
        # default cannot silently route the experiment through historical
        # crop/keyframe registration.
        'full_map_registration:=true',
        # The authoritative unknown-pose wrapper uses the frozen production
        # RPP controller.  Pass this explicitly through the nested launch
        # chain so an inherited/duplicate launch argument cannot select the
        # historical DWB diagnostic variant for the pre-handoff local stack.
        'controller_variant:=rpp',
        f'enable_observer:={str(args.enable_observer).lower()}',
        f'enable_forensic_capture:={str(args.enable_forensic_capture).lower()}',
        # The runner owns the RViz process, while the launch graph owns the
        # passive map/path/handoff bridge. Start the bridge at simulation
        # launch so it cannot miss the volatile accepted-handoff message.
        'launch_rviz:=false',
        f'launch_visualization_overlay:={str(args.rviz).lower()}',
        'enable_mission_timeout:=false',
    ]
    if output_root is not None:
        command.extend([
            f'output_root:={output_root}',
            f'run_id:={run_id}',
        ])
        # Frontend diagnostics are an explicit observer/forensic workload.
        # Do not accidentally enable the 1,000,000-record evidence stream in
        # a headless performance run whose observer is disabled; preserve it
        # whenever evidence capture or diagnostic mode was requested.
        if (args.enable_observer or args.enable_forensic_capture or
                args.diagnostic_mode):
            command.append(
                f'unknown_pose_diagnostic_output:='
                f'{output_root / run_id / "frontend"}')
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


def lifecycle_startup_succeeded(response) -> bool:
    """Return whether a lifecycle STARTUP request was accepted."""
    return response is not None and bool(response.success)


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
        # Unknown-pose campaigns intentionally do not instantiate shared
        # fusion/assignment before the first canonical handoff.  Readiness
        # must therefore accept the complete local pre-handoff graph while
        # preserving the historical shared graph contract for known-pose
        # launches.
        return (
            all(item in names for item in GRAPH_SUFFIXES) or
            all(item in names for item in LOCAL_UNKNOWN_POSE_GRAPH_SUFFIXES)
        )

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
        manager_clients = {}
        for robot in ('robot1', 'robot2'):
            manager_clients[robot] = self.create_client(
                ManageLifecycleNodes,
                f'/{robot}/local_lifecycle_manager_navigation/manage_nodes')
            for node_name in LOCAL_NAV2_NODES:
                service_name = f'/{robot}/{node_name}/get_state'
                clients[(robot, node_name)] = self.create_client(
                    GetState, service_name)
        startup_sent = set()
        startup_results = {}
        next_startup_attempt = {robot: 0.0 for robot in ('robot1', 'robot2')}
        while time.monotonic() < deadline:
            all_active = True
            details = {}
            for robot in ('robot1', 'robot2'):
                manager = manager_clients[robot]
                if robot not in startup_sent:
                    # If the lifecycle manager has already activated every
                    # local node, treat that as successful startup and avoid
                    # issuing a redundant STARTUP command.
                    active_now = True
                    lifecycle_transitioning = False
                    for node_name in LOCAL_NAV2_NODES:
                        client = clients[(robot, node_name)]
                        if not client.service_is_ready():
                            client.wait_for_service(timeout_sec=0.0)
                        if not client.service_is_ready():
                            active_now = False
                            break
                        response = self._wait_future(
                            client.call_async(GetState.Request()), deadline)
                        if response is None:
                            active_now = False
                            break
                        state_id = response.current_state.id
                        if state_id != State.PRIMARY_STATE_ACTIVE:
                            active_now = False
                        # If the manager has already configured/activated any
                        # node, it owns an in-progress lifecycle transition.
                        # Do not inject a second STARTUP request into that
                        # transition; continue observing until all nodes are
                        # active.  The explicit STARTUP fallback remains for
                        # the all-inactive case and preserves the bounded
                        # readiness retry contract.
                        if state_id not in (
                                State.PRIMARY_STATE_UNCONFIGURED,
                                State.PRIMARY_STATE_INACTIVE):
                            lifecycle_transitioning = True
                    if active_now:
                        startup_results[robot] = True
                        startup_sent.add(robot)
                if (robot not in startup_sent and
                        not lifecycle_transitioning and
                        time.monotonic() >= next_startup_attempt[robot]):
                    if not manager.service_is_ready():
                        manager.wait_for_service(timeout_sec=0.0)
                    if manager.service_is_ready():
                        request = ManageLifecycleNodes.Request()
                        request.command = ManageLifecycleNodes.Request.STARTUP
                        future = manager.call_async(request)
                        response = self._wait_future(future, deadline)
                        startup_results[robot] = lifecycle_startup_succeeded(
                            response)
                        if startup_results[robot]:
                            startup_sent.add(robot)
                        else:
                            # A lifecycle manager can receive STARTUP while
                            # one late Nav2 node is still creating its
                            # change_state service.  Keep the bounded gate
                            # alive and retry that manager; marking a failed
                            # request as sent permanently strands the robot.
                            next_startup_attempt[robot] = (
                                time.monotonic() + 1.0)
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
            if all_active and len(startup_results) == 2 and all(
                    startup_results.values()):
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
    deadline = time.monotonic() + max(0.0, float(timeout))
    interrupted = False
    while process.poll() is None:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break
        try:
            process.wait(timeout=remaining)
            return True
        except subprocess.TimeoutExpired:
            break
        except KeyboardInterrupt:
            # SIGINT is also the runner's user-facing stop request.  Do not
            # abandon process-group cleanup while waiting for launch to exit;
            # continue to the bounded TERM/KILL escalation below.
            interrupted = True
            continue
    return process.poll() is not None


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


def rviz_command(world_profile='large') -> list[str]:
    """Return RViz with the selected profile's correctly scaled camera."""
    share = Path(get_package_share_directory(PACKAGE))
    selected = profile(world_profile, share / 'worlds')
    config = manual_rviz_path(selected, share / 'resource')
    return ['rviz2', '-d', str(config), '--ros-args',
            '-p', 'use_sim_time:=true']


def requires_fixed_anchor_forensics(args: argparse.Namespace) -> bool:
    """Return whether this runner profile must retain passive physical GT.

    The close-start unknown-pose profiles are the thesis validation/campaign
    harnesses.  Their fixed-anchor Supervisor evidence is evaluation-only,
    yet omitting it silently turns a normal run into an invalid-GT artifact.
    Keep the generic parser default lightweight for unrelated quick trials;
    normal close-start runs are upgraded here before their manifest/launch
    command is written.
    """
    return str(args.world_profile).startswith(
        'large_unknown_pose_close_start')


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
    webots_driver_prefix = ''
    webots_driver_executable = ''
    runtime_provenance = {}
    removed_stale_environment_entries = []
    world = None
    ready_wall_elapsed = None
    launch_spawn_elapsed = None
    peak_runner_rss = 0
    rclpy_started = False
    probe = None
    executor = None
    previous_domain = os.environ.get('ROS_DOMAIN_ID')

    if (requires_fixed_anchor_forensics(args) and
            not args.enable_forensic_capture):
        args.enable_forensic_capture = True
        print(
            'FIXED_ANCHOR_GT_FORENSICS auto_enabled=true '
            'reason=close_start_unknown_pose_validation',
            flush=True)

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
        inherited_environment = os.environ.copy()
        environment, removed_stale_environment_entries = (
            filtered_runtime_environment(inherited_environment))
        # Validate the complete middleware/install contract before Webots is
        # spawned.  Without this guard, a shell that silently inherited Fast
        # DDS or an old generated install could start Webots and then wait
        # forever for /clock, obscuring the real runtime failure.
        runtime_contract = require_runtime_provenance(
            WORKSPACE, environment, args.ros_domain_id)
        # Resolve both package authorities through the cleaned/bound child
        # environment.  Looking them up in the parent shell first can select
        # the stale driver underlay before filtering has any effect.
        prefix = package_prefix(environment)
        (webots_driver_prefix,
         webots_driver_executable) = webots_driver_provenance(environment)
        filtered_stale_entries = [
            f'{variable}={entry}'
            for variable in FILTERED_PATH_VARIABLES
            for entry in environment.get(variable, '').split(os.pathsep)
            if _is_stale_workspace_entry(entry)
        ]
        if filtered_stale_entries:
            raise FastTrialError(
                'filtered runtime environment still contains old workspace '
                f'entries: {filtered_stale_entries}')
        runtime_provenance = {
            'project_prefix': prefix,
            'project_module': str(importlib.util.find_spec(PACKAGE).origin),
            'webots_driver_prefix': webots_driver_prefix,
            'webots_driver_executable': webots_driver_executable,
            'webots_driver_module': str(
                importlib.util.find_spec('webots_ros2_driver').origin),
            'ros2_executable': str(Path(shutil.which('ros2')).resolve()),
            'python_executable': str(Path(sys.executable).resolve()),
            'ros_domain_id': str(args.ros_domain_id),
            'inherited_ament_prefix_path': inherited_environment.get(
                'AMENT_PREFIX_PATH', ''),
            'inherited_pythonpath': inherited_environment.get('PYTHONPATH', ''),
            'filtered_ament_prefix_path': environment.get(
                'AMENT_PREFIX_PATH', ''),
            'filtered_pythonpath': environment.get('PYTHONPATH', ''),
            'inherited_path_environment': {
                variable: inherited_environment.get(variable, '')
                for variable in FILTERED_PATH_VARIABLES
            },
            'filtered_path_environment': {
                variable: environment.get(variable, '')
                for variable in FILTERED_PATH_VARIABLES
            },
            'removed_stale_environment_entries':
                removed_stale_environment_entries,
            'runtime_contract': runtime_contract,
        }
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
            + '\n'
            + json.dumps(runtime_provenance, sort_keys=True) + '\n')
        (attempt / 'effective_command.txt').write_text(effective, encoding='utf-8')
        print(f'launch_file={LAUNCH_FILE}', flush=True)
        print(f'world_path={world}', flush=True)
        print(f'webots_driver_prefix={webots_driver_prefix}', flush=True)
        print(f'webots_driver_executable={webots_driver_executable}', flush=True)
        print(f'results_directory={attempt}', flush=True)
        print(f'effective_command={effective.strip()}', flush=True)

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
                rviz_command(args.world_profile), env=environment,
                stdout=subprocess.PIPE,
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
        total_wall_limit = args.mission_timeout or 1800.0
        hard_deadline = started + max(0.0, total_wall_limit)
        service_deadline = hard_deadline - FINALIZATION_BUDGET_S
        readiness_deadline = min(
            started + (args.startup_timeout or 300.0), service_deadline,
        )
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

        mission_deadline = None if args.hold_open else service_deadline
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
            'webots_driver_prefix': webots_driver_prefix,
            'webots_driver_executable': webots_driver_executable,
            'runtime_provenance': runtime_provenance,
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
    if not ROS_DOMAIN_MIN <= args.ros_domain_id <= ROS_DOMAIN_MAX:
        raise SystemExit(
            f'--ros-domain-id must be between {ROS_DOMAIN_MIN} and '
            f'{ROS_DOMAIN_MAX} for the active CycloneDDS port profile')
    return run(args)


if __name__ == '__main__':
    raise SystemExit(main())
