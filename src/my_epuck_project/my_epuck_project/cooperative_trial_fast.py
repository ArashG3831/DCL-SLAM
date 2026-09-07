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
import math
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
from my_epuck_interfaces.msg import RelativePoseHypothesis
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
from std_msgs.msg import Bool

from .cooperative_profiles import (
    PROFILE_SETTINGS, manual_rviz_path, profile, profile_for_world,
    profile_summary)
from .thesis_baseline_topology import (
    materialize_seeded_world, parse_world_random_seed, sha256_file)
from .ros_runtime_preflight import (
    ROS_DOMAIN_MIN, ROS_DOMAIN_MAX, require_runtime_provenance,
)
from .thin_experiment_recorder import ThinEvidenceSession


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
OBSERVER_FINALIZATION_GRACE_S = 60.0
SHUTDOWN_TERM_S = 10.0
WATCHDOG_TERM_GRACE_S = 3.0


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


class SimulationHorizonReached(FastTrialError):
    """The live ROS /clock reached the scientific stopping horizon."""


class SimulationHorizonOverrun(FastTrialError):
    """The live ROS /clock crossed the bounded post-horizon fence."""


class WallWatchdog:
    """Independent monotonic watchdog for the complete launch process group.

    The watchdog is armed before the launch subprocess is spawned.  It does
    not inspect observer files or ROS log output.  Once the absolute wall
    deadline is reached it records the latest live clock value, terminates all
    attached process groups, and escalates to SIGKILL after a short grace.
    """

    def __init__(self, deadline: float, clock_getter, started: float | None = None):
        self.deadline = float(deadline)
        self.started = time.monotonic() if started is None else float(started)
        self._clock_getter = clock_getter
        self._processes = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self.armed = False
        self.fired = False
        self.fired_wall_elapsed_s = None
        self.fired_sim_time_s = None

    def add_process(self, process):
        with self._lock:
            self._processes.append(process)
            fired = self.fired
        if fired:
            stop_process(process, signal.SIGTERM)
            stop_process(process, signal.SIGKILL)

    def _live_processes(self):
        with self._lock:
            return [process for process in self._processes
                    if process is not None and process.poll() is None]

    def _run(self):
        remaining = self.deadline - time.monotonic()
        if remaining > 0.0 and self._stop.wait(remaining):
            return
        if self._stop.is_set():
            return
        self.fired = True
        self.fired_wall_elapsed_s = time.monotonic() - self.started
        try:
            self.fired_sim_time_s = self._clock_getter()
        except Exception:
            self.fired_sim_time_s = None
        for process in self._live_processes():
            stop_process(process, signal.SIGTERM)
        grace_deadline = time.monotonic() + WATCHDOG_TERM_GRACE_S
        while time.monotonic() < grace_deadline:
            if not self._live_processes():
                return
            self._stop.wait(0.05)
        for process in self._live_processes():
            stop_process(process, signal.SIGKILL)

    def start(self):
        self.armed = True
        self._thread = threading.Thread(
            target=self._run, name='cooperative-wall-watchdog', daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.25)


class SimulationHorizonMonitor:
    """Stop the launch independently of readiness and mission state.

    The runner's main thread must continue spinning ROS so that ``/clock`` can
    be observed, but cooperative startup can spend an extended period inside
    a readiness/future loop.  This monitor therefore owns an independent
    wall-thread that watches the latest callback-delivered clock value and
    records the scientific horizon as soon as it is reached.  The main thread
    then records the normal SIM_TIME_COMPLETE result and performs the ordered
    cleanup, which gives the passive logger/observer a finalization barrier
    before the launch process group is stopped.
    """

    def __init__(self, horizon_s: float, clock_getter,
                 overrun_fence_s: float | None = None):
        self.horizon_s = float(horizon_s)
        self.overrun_fence_s = (
            self.horizon_s + 60.0 if overrun_fence_s is None
            else float(overrun_fence_s))
        if self.overrun_fence_s <= self.horizon_s:
            raise ValueError('horizon overrun fence must exceed horizon')
        self._clock_getter = clock_getter
        self._processes = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.reached = threading.Event()
        self.overrun = threading.Event()
        self._thread = None
        self.reached_sim_time_s = None
        self.overrun_sim_time_s = None

    def add_process(self, process):
        with self._lock:
            self._processes.append(process)
            reached = self.reached.is_set()
            overrun = self.overrun.is_set()
        if overrun:
            stop_process(process, signal.SIGINT)
        elif reached:
            stop_process(process, signal.SIGINT)

    def _live_processes(self):
        with self._lock:
            return [process for process in self._processes
                    if process is not None and process.poll() is None]

    def _run(self):
        while not self._stop.wait(0.01):
            try:
                latest = self._clock_getter()
            except Exception:
                latest = None
            if latest is None:
                continue
            if not self.reached.is_set() and latest >= self.horizon_s:
                self.reached_sim_time_s = float(latest)
                self.reached.set()
            if self.reached.is_set() and latest >= self.overrun_fence_s:
                self.overrun_sim_time_s = float(latest)
                self.overrun.set()
                # The normal runner consumes ``reached`` and performs the
                # ordered shutdown.  This branch is a bounded fail-closed
                # guard if that control path is not servicing the event.
                for process in self._live_processes():
                    stop_process(process, signal.SIGINT)
                return

    def start(self):
        self._thread = threading.Thread(
            target=self._run, name='cooperative-simulation-horizon',
            daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.25)


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


def nonnegative_int(value: str) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError('expected a nonnegative integer') from exc
    if parsed < 0:
        raise argparse.ArgumentTypeError('expected a nonnegative integer')
    return parsed


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
    result.add_argument('--experiment-condition', choices=('A', 'B', 'C', 'D'),
                        default='C', help='Thesis topology: A, B, C, or D.')
    result.add_argument(
        '--webots-random-seed', type=nonnegative_int, default=None,
        help='Explicit nonnegative Webots WorldInfo.randomSeed.')
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
        '--observer-architecture', choices=('legacy', 'thin'),
        default='legacy',
        help=('Observer/evidence path. Legacy remains the default reference '
              'path until thin-mode parity and runtime gates pass.'))
    result.add_argument(
        '--enable-forensic-capture', type=boolean, default=False,
        help='Enable passive Webots Supervisor/map forensic capture.')
    result.add_argument(
        '--enable-contact-capture', type=boolean, default=False,
        help='Enable passive contact capture when the selected launch supports it.')
    result.add_argument(
        '--enable-scientific-raw-capture', type=boolean, default=False,
        help='Enable native lossless scientific rosbag capture for legacy C.')
    result.add_argument('--forensic-ground-truth-sample-period-s',
                        type=float, default=0.02)
    result.add_argument('--contact-sampling-period-ms', type=int, default=20)
    result.add_argument('--fusion-process-nice', type=int, default=0)
    result.add_argument('--slam-tf-publish-probe-library', default='')
    result.add_argument('--slam-tf-publish-probe-log', default='')
    result.add_argument('--slam-tf-publication-mode', default='')
    result.add_argument(
        '--simulation-horizon-s', type=float, default=None,
        help='Scientific simulated-time horizon controlled by live /clock.')
    result.add_argument(
        '--wall-watchdog-s', type=float, default=None,
        help='Emergency monotonic wall-clock limit for the whole run.')
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
        '--local-path-gate-mode',
        choices=('MODE_A', 'MODE_B'), default=None,
        help='Required explicit final local-path dispatch gate experiment mode.')
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


def require_explicit_local_path_gate_mode(args: argparse.Namespace) -> str:
    """Fail closed when a campaign does not select its final gate policy."""
    mode = str(getattr(args, 'local_path_gate_mode', '') or '').strip().upper()
    if mode not in ('MODE_A', 'MODE_B'):
        raise FastTrialError(
            'local_path_gate_mode must be explicitly provided as '
            'MODE_A or MODE_B before simulation launch')
    args.local_path_gate_mode = mode
    return mode


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


def cleanup_campaign_webots_drivers(
        created_after: float, absolute_deadline: float | None = None) -> list[int]:
    drivers = campaign_webots_drivers(created_after)
    pids = [process.pid for process in drivers]
    for process in drivers:
        try:
            process.terminate()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    timeout = 5.0
    if absolute_deadline is not None:
        timeout = max(0.0, min(timeout, absolute_deadline - time.monotonic()))
    _, alive = psutil.wait_procs(drivers, timeout=timeout)
    for process in alive:
        try:
            process.kill()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    return pids


def logger_artifact_finalization_status(attempt: Path | None):
    """Return whether the ROS logger completed its persisted artifact contract."""
    if attempt is None:
        return False
    for summary_path in sorted(attempt.glob('**/summary.json')):
        root = summary_path.parent
        artifact_path = root / 'artifact_finalization.json'
        manifest_path = root / 'run_manifest.json'
        if not artifact_path.is_file() or not manifest_path.is_file():
            continue
        try:
            summary = json.loads(summary_path.read_text())
            artifact = json.loads(artifact_path.read_text())
            manifest = json.loads(manifest_path.read_text())
        except (OSError, ValueError, TypeError):
            continue
        summary_artifact = summary.get('artifact_finalization', {})
        manifest_artifact = manifest.get('artifact_finalization', {})
        if (artifact.get('complete') is True and
                artifact.get('status') == 'COMPLETE' and
                artifact.get('missing') == [] and
                summary_artifact.get('complete') is True and
                manifest.get('clean_shutdown') is True and
                manifest.get('shutdown_status') == 'clean' and
                manifest_artifact.get('complete') is True and
                manifest_artifact.get('status') == 'COMPLETE' and
                manifest_artifact.get('missing') == []):
            return True
    return False


def observer_finalization_status(attempt: Path | None):
    """Return true when the selected observer path finalized fail-closed."""
    if attempt is None:
        return False
    thin = sorted(attempt.glob('**/raw_evidence_finalization.json'))
    if thin:
        try:
            statuses = [json.loads(path.read_text()).get('complete') is True
                        for path in thin]
            return bool(statuses) and all(statuses)
        except (OSError, ValueError, TypeError):
            return False
    metrics = sorted(attempt.glob('**/runtime_metrics.json'))
    if not metrics:
        return False
    statuses = []
    for path in metrics:
        try:
            statuses.append(bool(json.loads(path.read_text()).get('finalized')))
        except (OSError, ValueError, TypeError):
            statuses.append(False)
    return bool(statuses) and all(statuses) and logger_artifact_finalization_status(attempt)


def resolve_world(args: argparse.Namespace) -> Path:
    source_worlds = WORKSPACE / 'src' / PACKAGE / 'worlds'
    selected = profile_for_world(
        args.world_profile, source_worlds,
        explicit_world_path=args.world_path,
        ideal_encoder_sensing=args.ideal_encoder_sensing)
    args.profile_metadata = profile_summary(selected)
    canonical = Path(selected['world_path']).resolve()
    requested = args.webots_random_seed
    effective = (parse_world_random_seed(canonical)
                 if requested is None else requested)
    if requested is None:
        derived = canonical
    else:
        import tempfile
        derived = materialize_seeded_world(
            canonical, tempfile.mkdtemp(prefix='my_epuck_seeded_'), requested)
    args.seed_provenance = {
        'requested_seed': requested,
        'effective_seed': effective,
        'canonical_base_world_path': str(canonical),
        'canonical_base_world_sha256': sha256_file(canonical),
        'derived_run_world_path': str(derived),
        'derived_run_world_sha256': sha256_file(derived),
    }
    args.seed_provenance_json = json.dumps(
        args.seed_provenance, sort_keys=True, separators=(',', ':'))
    return Path(derived).resolve()


def prepare_attempt(args: argparse.Namespace, prefix: str, world: Path):
    results = Path(args.results_directory).expanduser().resolve()
    results.mkdir(parents=True, exist_ok=True)
    if not results.is_dir():
        raise FastTrialError(f'results directory is not a directory: {results}')
    attempt = results / f'fast_trial_{datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")}'
    attempt.mkdir()
    return attempt


def launch_file_for_condition(args):
    condition = getattr(args, 'experiment_condition', 'C')
    return {
        'A': 'single_robot_thesis_baseline_launch.py',
        'B': 'two_robots_independent_exploration_launch.py',
        'C': LAUNCH_FILE,
        'D': LAUNCH_FILE,
    }[condition]


def launch_command(
        args: argparse.Namespace, world: Path, output_root: Path | None = None,
        run_id: str = '') -> list[str]:
    condition = getattr(args, 'experiment_condition', 'C')
    launch_file = launch_file_for_condition(args)
    observer_architecture = str(
        getattr(args, 'observer_architecture', 'legacy')).strip().lower()
    legacy_observer_enabled = bool(
        args.enable_observer and observer_architecture == 'legacy')
    legacy_forensic_enabled = bool(
        args.enable_forensic_capture and observer_architecture == 'legacy')
    # Thin mode still needs the launch-time temporary Supervisor Robot node
    # when GT/contact capture is requested. This stages the read-only world
    # node without re-enabling the legacy live logger.
    forensic_world_enabled = bool(
        args.enable_forensic_capture or
        (observer_architecture == 'thin' and args.enable_contact_capture))
    if condition in ('A', 'B'):
        return [item for item in [
            'ros2', 'launch', PACKAGE, launch_file,
            f'world_profile:={args.world_profile}', f'world_path:={world}',
            f'webots_port:={args.webots_port}',
            f'webots_mode:={args.webots_mode}',
            f'webots_gui:={str(args.rendering).lower()}',
            f'enable_observer:={str(legacy_observer_enabled).lower()}',
            f'enable_forensic_capture:={str(legacy_forensic_enabled).lower()}',
            f'enable_contact_capture:={str(args.enable_contact_capture).lower()}',
            f'experiment_condition:={condition}',
            f'seed_provenance_json:={getattr(args, "seed_provenance_json", "{}")}',
            f'output_root:={output_root}' if output_root is not None else '',
            f'run_id:={run_id}' if run_id else '',
        ] if item]
    # run() calls require_explicit_local_path_gate_mode before this helper.
    # Keep the construction helper usable by legacy non-launch unit tests
    # whose Namespace predates the required campaign option.
    gate_mode = getattr(args, 'local_path_gate_mode', None) or 'MODE_A'
    command = [
        'ros2', 'launch', PACKAGE, launch_file,
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
        f'local_path_gate_mode:={gate_mode}',
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
        # C's authoritative two-robot condition is released only after both
        # shared stacks and both replicated assignment peers report the same
        # pre-exploration readiness state.  A transient-local simulation-time
        # START_RELEASE then gates every exploration send.
        f'common_start_release_required:={'true' if condition == "C" else "false"}',
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
        f'enable_observer:={str(legacy_observer_enabled).lower()}',
        f'enable_forensic_capture:={str(forensic_world_enabled).lower()}',
        f'enable_contact_capture:={str(args.enable_contact_capture).lower()}',
        f'enable_passive_rosbag:={"true" if condition == "C" and observer_architecture == "legacy" else "false"}',
        f'enable_scientific_raw_capture:={str(getattr(args, "enable_scientific_raw_capture", False)).lower()}',
        f'observer_architecture:={observer_architecture}',
        f'experiment_condition:={condition}',
        f'seed_provenance_json:={getattr(args, "seed_provenance_json", "{}")}',
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
        if (legacy_observer_enabled or legacy_forensic_enabled or
                args.diagnostic_mode or
                (condition == 'C' and observer_architecture == 'thin')):
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


def nav2_readiness_action(all_active: bool, nav2_autostart: bool) -> str:
    """Select the single owner allowed to start Nav2 lifecycle nodes."""
    if all_active:
        return 'READY_NO_STARTUP'
    if nav2_autostart:
        return 'WAIT_FOR_LAUNCH_AUTOSTART'
    return 'SEND_STARTUP'


def nav2_readiness_node_names(experiment_condition: str) -> tuple[str, ...]:
    """Return the lifecycle names used by the selected launch topology."""
    return (NAV2_NODES if experiment_condition in ('A', 'B')
            else LOCAL_NAV2_NODES)


def phase_aware_nav2_readiness(
        local_activation_latched: set[str], handoff_detected: bool,
        shared_activation_reached: set[str],
        robots: tuple[str, ...] = ('robot1', 'robot2')) -> tuple[bool, str]:
    """Evaluate the C/D lifecycle phases without observing torn-down nodes.

    The local lifecycle manager owns the pre-handoff activation and publishes
    a successful STARTUP result before the phase manager can release it.  That
    success is latched; the later absence of ``local_*`` services is expected
    teardown, not a readiness failure.  Shared readiness is emitted only
    after the phase manager's shared lifecycle STARTUP succeeds.
    """
    required = set(robots)
    if not required.issubset(local_activation_latched):
        return False, 'waiting_local_activation'
    if not handoff_detected:
        return False, 'waiting_accepted_handoff'
    if not required.issubset(shared_activation_reached):
        return False, 'waiting_shared_nav2_activation'
    return True, 'shared_nav2_active_after_handoff'


class ReadyProbe(Node):
    """Small ROS graph probe used only by the fast runner."""

    def __init__(self, experiment_condition='C', nav2_autostart=None):
        super().__init__('cooperative_trial_fast_probe')
        self.experiment_condition = experiment_condition
        self.nav2_autostart = (
            experiment_condition in ('A', 'B')
            if nav2_autostart is None else bool(nav2_autostart))
        self.robots = ('robot1',) if experiment_condition == 'A' else (
            'robot1', 'robot2')
        self.clock_values: list[float] = []
        self.clock_start_s = None
        self.latest_clock_s = None
        self.scans: set[str] = set()
        self.odometry: set[str] = set()
        self.maps: set[str] = set()
        self.local_nav2_activation_latched: set[str] = set()
        self.handoff_detected = False
        self.handoff_detection_source = None
        self.shared_nav2_activation_reached: set[str] = set()
        self.readiness_telemetry = {
            'local_activation_latched': [],
            'handoff_detected': False,
            'handoff_detection_source': None,
            'shared_activation_reached': [],
            'local_activation_latched_at_sim_s': {},
            'handoff_detected_at_sim_s': None,
            'shared_activation_reached_at_sim_s': {},
            'final_readiness_reason': None,
        }
        self.tf_buffer = Buffer(cache_time=Duration(seconds=60.0))
        self.tf_listener = TransformListener(self.tf_buffer, self,
                                             spin_thread=False)
        self.executor = None
        map_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Clock, '/clock', self._on_clock, qos_profile_sensor_data)
        for robot in self.robots:
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
        if self.experiment_condition in ('C', 'D'):
            # The accepted hypothesis is the existing one-shot handoff
            # protocol signal.  The per-robot shared-readiness signal is
            # transient-local and is published only after that robot's shared
            # lifecycle-manager STARTUP succeeds.
            handoff_qos = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE)
            shared_qos = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(
                RelativePoseHypothesis, '/cslam/relative_pose/hypotheses',
                self._on_accepted_handoff, handoff_qos)
            for robot in self.robots:
                self.create_subscription(
                    Bool,
                    f'/cslam/unknown_pose/{robot}/shared_nav2_ready',
                    partial(self._on_shared_nav2_ready, robot), shared_qos)

    def _mark_scan(self, robot, message):
        del message
        self.scans.add(robot)

    def _mark_odom(self, robot, message):
        del message
        self.odometry.add(robot)

    def _mark_map(self, robot, message):
        del message
        self.maps.add(robot)

    def _readiness_sim_time(self):
        return self.latest_clock_s

    def _on_accepted_handoff(self, message: RelativePoseHypothesis):
        if not bool(message.accepted) or str(message.status) != 'ACCEPTED':
            return
        if self.handoff_detected:
            return
        self.handoff_detected = True
        self.handoff_detection_source = 'accepted_hypothesis'
        self.readiness_telemetry.update({
            'handoff_detected': True,
            'handoff_detection_source': self.handoff_detection_source,
            'handoff_detected_at_sim_s': self._readiness_sim_time(),
        })
        self.get_logger().info(
            'FAST_TRIAL_READINESS handoff_detected=true source=%s' %
            self.handoff_detection_source)

    def _on_shared_nav2_ready(self, robot, message: Bool):
        if not bool(message.data):
            return
        if not self.handoff_detected:
            # A late runner subscription can miss the volatile accepted
            # hypothesis.  This retained per-robot lifecycle signal is only
            # emitted after the phase manager has accepted the handoff and
            # completed shared STARTUP, so it is a safe handoff witness while
            # local activation remains independently mandatory below.
            self.handoff_detected = True
            self.handoff_detection_source = 'shared_nav2_ready'
            self.readiness_telemetry.update({
                'handoff_detected': True,
                'handoff_detection_source': self.handoff_detection_source,
                'handoff_detected_at_sim_s': self._readiness_sim_time(),
            })
            self.get_logger().info(
                'FAST_TRIAL_READINESS handoff_detected=true source=%s' %
                self.handoff_detection_source)
        if robot in self.shared_nav2_activation_reached:
            return
        self.shared_nav2_activation_reached.add(robot)
        sim_time = self._readiness_sim_time()
        self.readiness_telemetry['shared_activation_reached_at_sim_s'][robot] = sim_time
        self.readiness_telemetry['shared_activation_reached'] = sorted(
            self.shared_nav2_activation_reached)
        self.get_logger().info(
            'FAST_TRIAL_READINESS shared_activation_reached=true robot=%s' %
            robot)

    def _latch_local_activation(self, robot):
        if robot in self.local_nav2_activation_latched:
            return
        self.local_nav2_activation_latched.add(robot)
        sim_time = self._readiness_sim_time()
        self.readiness_telemetry['local_activation_latched_at_sim_s'][robot] = sim_time
        self.readiness_telemetry['local_activation_latched'] = sorted(
            self.local_nav2_activation_latched)
        self.get_logger().info(
            'FAST_TRIAL_READINESS local_activation_latched=true robot=%s' %
            robot)

    def readiness_telemetry_snapshot(self, final_reason=None):
        if final_reason is not None:
            self.readiness_telemetry['final_readiness_reason'] = final_reason
        snapshot = dict(self.readiness_telemetry)
        snapshot['local_activation_latched'] = sorted(
            self.local_nav2_activation_latched)
        snapshot['shared_activation_reached'] = sorted(
            self.shared_nav2_activation_reached)
        snapshot['handoff_detected'] = bool(self.handoff_detected)
        snapshot['handoff_detection_source'] = self.handoff_detection_source
        return snapshot

    def _on_clock(self, message: Clock):
        value = message.clock.sec + message.clock.nanosec * 1e-9
        if self.clock_start_s is None:
            self.clock_start_s = value
        self.latest_clock_s = value
        if not self.clock_values or value != self.clock_values[-1]:
            self.clock_values.append(value)
            self.clock_values = self.clock_values[-4:]

    def simulation_horizon_reached(self, horizon_s: float) -> bool:
        return (self.latest_clock_s is not None and
                self.latest_clock_s >= float(horizon_s))

    def clock_ready(self) -> bool:
        return len(self.clock_values) >= 2 and any(
            right > left for left, right in zip(
                self.clock_values, self.clock_values[1:]))

    def robot_interfaces_ready(self) -> bool:
        return self.scans == set(self.robots) and \
            self.odometry == set(self.robots)

    def maps_ready(self) -> bool:
        return self.maps == set(self.robots)

    def tf_ready(self) -> bool:
        for robot in self.robots:
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
        if self.experiment_condition in ('A', 'B'):
            return True
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

    def _wait_future(self, future, deadline: float,
                     horizon_reached=None):
        while not future.done() and time.monotonic() < deadline:
            if horizon_reached is not None and horizon_reached():
                raise SimulationHorizonReached(
                    'simulation horizon reached during Nav2 readiness')
            self.spin_once(0.1)
        if horizon_reached is not None and horizon_reached():
            raise SimulationHorizonReached(
                'simulation horizon reached during Nav2 readiness')
        return future.result() if future.done() else None

    def activate_and_check_nav2(self, deadline: float,
                                horizon_reached=None) -> dict:
        if self.experiment_condition in ('C', 'D'):
            return self._activate_phase_aware_cooperative_nav2(
                deadline, horizon_reached)
        # Unknown-pose exploration starts in local-map mode.  Its local
        # lifecycle managers autostart; the shared managers must remain
        # waiting until the frontend handoff.  Poll activation to the same
        # bounded deadline because the second namespaced manager may still be
        # bringing up its nodes when the first state query completes.
        # The cooperative path uses local_lifecycle_manager_navigation; the
        # A/B baseline path uses lifecycle_manager_navigation/manage_nodes.
        clients = {}
        manager_clients = {}
        nav2_prefix = '' if self.experiment_condition in ('A', 'B') else 'local_'
        manager_suffix = 'lifecycle_manager_navigation' if not nav2_prefix else 'local_lifecycle_manager_navigation'
        for robot in self.robots:
            manager_clients[robot] = self.create_client(
                ManageLifecycleNodes,
                f'/{robot}/{manager_suffix}/manage_nodes')
            for node_name in nav2_readiness_node_names(
                    self.experiment_condition):
                service_name = f'/{robot}/{nav2_prefix}{node_name}/get_state'
                clients[(robot, node_name)] = self.create_client(
                    GetState, service_name)
        startup_sent = set()
        startup_results = {}
        next_startup_attempt = {robot: 0.0 for robot in self.robots}
        while time.monotonic() < deadline:
            if horizon_reached is not None and horizon_reached():
                raise SimulationHorizonReached(
                    'simulation horizon reached during Nav2 readiness')
            all_active = True
            details = {}
            for robot in self.robots:
                manager = manager_clients[robot]
                if robot not in startup_sent:
                    # If the lifecycle manager has already activated every
                    # local node, treat that as successful startup and avoid
                    # issuing a redundant STARTUP command.
                    active_now = True
                    lifecycle_transitioning = False
                    for node_name in nav2_readiness_node_names(
                            self.experiment_condition):
                        client = clients[(robot, node_name)]
                        if not client.service_is_ready():
                            client.wait_for_service(timeout_sec=0.0)
                        if not client.service_is_ready():
                            active_now = False
                            break
                        response = self._wait_future(
                            client.call_async(GetState.Request()), deadline,
                            horizon_reached)
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
                    readiness_action = nav2_readiness_action(
                        active_now, self.nav2_autostart)
                    if readiness_action in (
                            'READY_NO_STARTUP',
                            'WAIT_FOR_LAUNCH_AUTOSTART'):
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
                        response = self._wait_future(
                            future, deadline, horizon_reached)
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
                for node_name in nav2_readiness_node_names(
                        self.experiment_condition):
                    client = clients[(robot, node_name)]
                    if not client.service_is_ready():
                        client.wait_for_service(timeout_sec=0.0)
                    if not client.service_is_ready():
                        active[node_name] = False
                        all_active = False
                        continue
                    response = self._wait_future(
                        client.call_async(GetState.Request()), deadline,
                        horizon_reached)
                    state_id = response.current_state.id if response else -1
                    active[node_name] = state_id == State.PRIMARY_STATE_ACTIVE
                    all_active = all_active and active[node_name]
                details[robot] = active
            if all_active and len(startup_results) == len(self.robots) and all(
                    startup_results.values()):
                return details
            self.spin_once(0.1)
        raise FastTrialError(
            'local Nav2 activation timed out: %s' % details)

    def _activate_phase_aware_cooperative_nav2(
            self, deadline: float, horizon_reached=None) -> dict:
        """Bring up local Nav2, then wait for the post-handoff shared stack.

        This is intentionally separate from the A/B path.  C/D local node
        services are expected to disappear after accepted handoff, so their
        state must be latched before that phase transition and never probed as
        a post-handoff readiness condition.
        """
        clients = {}
        manager_clients = {}
        for robot in self.robots:
            manager_clients[robot] = self.create_client(
                ManageLifecycleNodes,
                f'/{robot}/local_lifecycle_manager_navigation/manage_nodes')
            for node_name in LOCAL_NAV2_NODES:
                clients[(robot, node_name)] = self.create_client(
                    GetState, f'/{robot}/local_{node_name}/get_state')
        startup_sent = set()
        startup_results = {}
        next_startup_attempt = {robot: 0.0 for robot in self.robots}
        details = {}
        while time.monotonic() < deadline:
            if horizon_reached is not None and horizon_reached():
                self.readiness_telemetry_snapshot('horizon_during_readiness')
                raise SimulationHorizonReached(
                    'simulation horizon reached during Nav2 readiness')

            # Before handoff, retain the existing local lifecycle startup
            # semantics.  A successful manager STARTUP is the lifecycle
            # owner's proof that the complete managed local stack activated.
            if not self.handoff_detected:
                for robot in self.robots:
                    if robot not in startup_sent:
                        manager = manager_clients[robot]
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
                                client.call_async(GetState.Request()), deadline,
                                horizon_reached)
                            if response is None:
                                active_now = False
                                break
                            state_id = response.current_state.id
                            if state_id != State.PRIMARY_STATE_ACTIVE:
                                active_now = False
                            if state_id not in (
                                    State.PRIMARY_STATE_UNCONFIGURED,
                                    State.PRIMARY_STATE_INACTIVE):
                                lifecycle_transitioning = True
                        if active_now:
                            startup_results[robot] = True
                            startup_sent.add(robot)
                            self._latch_local_activation(robot)
                        elif (not lifecycle_transitioning and
                              time.monotonic() >= next_startup_attempt[robot]):
                            if not manager.service_is_ready():
                                manager.wait_for_service(timeout_sec=0.0)
                            if manager.service_is_ready():
                                request = ManageLifecycleNodes.Request()
                                request.command = (
                                    ManageLifecycleNodes.Request.STARTUP)
                                response = self._wait_future(
                                    manager.call_async(request), deadline,
                                    horizon_reached)
                                startup_results[robot] = (
                                    lifecycle_startup_succeeded(response))
                                if startup_results[robot]:
                                    startup_sent.add(robot)
                                    self._latch_local_activation(robot)
                                else:
                                    next_startup_attempt[robot] = (
                                        time.monotonic() + 1.0)

            ready, reason = phase_aware_nav2_readiness(
                self.local_nav2_activation_latched,
                self.handoff_detected,
                self.shared_nav2_activation_reached,
                self.robots)
            self.readiness_telemetry['final_readiness_reason'] = reason
            if ready and len(startup_results) == len(self.robots) and all(
                    startup_results.values()):
                details = {
                    robot: {'shared_nav2_active': True}
                    for robot in self.robots
                }
                self.get_logger().info(
                    'FAST_TRIAL_READINESS ready=true reason=%s' % reason)
                return {
                    'phase': 'shared_post_handoff',
                    'local_activation_latched': sorted(
                        self.local_nav2_activation_latched),
                    'handoff_detected': True,
                    'shared_activation_reached': sorted(
                        self.shared_nav2_activation_reached),
                    'details': details,
                    'readiness_telemetry': self.readiness_telemetry_snapshot(
                        reason),
                }
            self.spin_once(0.1)
        telemetry = self.readiness_telemetry_snapshot(
            self.readiness_telemetry.get('final_readiness_reason'))
        raise FastTrialError(
            'C/D Nav2 readiness timed out: %s telemetry=%s' %
            (details, telemetry))


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


def _logger_processes(launch):
    """Return passive experiment logger processes below this launch only."""
    try:
        root = psutil.Process(launch.pid)
        descendants = root.children(recursive=True)
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        return []
    result = []
    for process in descendants:
        try:
            command = process.cmdline()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
        if any('cooperative_experiment_logger' in part
               for part in command):
            result.append(process)
    return result


def _mission_processes(launch):
    """Return launch descendants that must stop before logger finalization.

    The Webots process and paced Supervisor stay alive while ROS mission
    children (including unknown-pose frontends) flush their terminal files.
    The experiment logger and external forensic observer are handled by their
    own ordered barriers below.
    """
    try:
        root = psutil.Process(launch.pid)
        descendants = root.children(recursive=True)
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        return []
    result = []
    for process in descendants:
        try:
            command = ' '.join(process.cmdline()).lower()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
        if any(token in command for token in (
                'cooperative_experiment_logger',
                'cooperative_ground_truth_observer',
                'webots.exe',
                'paced_ros2_supervisor')):
            continue
        result.append(process)
    return result


def shutdown_mission_before_logger(launch, absolute_deadline=None) -> list[int]:
    """Stop mission/front-end children before freezing logger artifacts."""
    processes = _mission_processes(launch)
    pids = [process.pid for process in processes]
    for process in processes:
        try:
            process.send_signal(signal.SIGINT)
        except (OSError, psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    if processes:
        timeout = SHUTDOWN_GRACE_S
        if absolute_deadline is not None:
            timeout = max(0.0, min(timeout, absolute_deadline - time.monotonic()))
        try:
            psutil.wait_procs(processes, timeout=timeout)
        except (OSError, psutil.Error):
            pass
    return pids


def shutdown_observer_before_launch(launch, absolute_deadline=None) -> list[int]:
    """Finalize the passive logger/observer before stopping Webots.

    The launch process group contains Webots and the ROS logger, while the
    forensic Supervisor observer is a separate session owned by the logger.
    Signal the logger directly, not the process group: it then signals the
    observer while Webots is still advancing, allowing the observer's next
    Supervisor.step() to return and its final metrics to be written.  Only
    after this barrier is it safe to stop Webots and the remaining launch.
    """
    processes = _logger_processes(launch)
    pids = [process.pid for process in processes]
    for process in processes:
        try:
            process.send_signal(signal.SIGINT)
        except (OSError, psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    if processes:
        # The legacy logger closes the 30-second rosbag recorder and then
        # materializes deferred artifacts before it exits.  Keep this
        # observer-only barrier separate from the launch teardown grace so a
        # still-finalizing logger is not killed with the launch process group.
        timeout = OBSERVER_FINALIZATION_GRACE_S
        if absolute_deadline is not None:
            timeout = max(0.0, min(timeout, absolute_deadline - time.monotonic()))
        try:
            psutil.wait_procs(processes, timeout=timeout)
        except (OSError, psutil.Error):
            pass
    return pids


def wait_process(process: subprocess.Popen, timeout: float,
                 absolute_deadline: float | None = None) -> bool:
    deadline = time.monotonic() + max(0.0, float(timeout))
    if absolute_deadline is not None:
        deadline = min(deadline, float(absolute_deadline))
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


def shutdown_processes(launch, rviz=None, absolute_deadline=None) -> dict:
    mission_pids = shutdown_mission_before_logger(
        launch, absolute_deadline=absolute_deadline)
    observer_logger_pids = shutdown_observer_before_launch(
        launch, absolute_deadline=absolute_deadline)
    stop_process(launch, signal.SIGINT)
    if rviz is not None:
        stop_process(rviz, signal.SIGINT)
    graceful = wait_process(
        launch, SHUTDOWN_GRACE_S, absolute_deadline=absolute_deadline)
    if not graceful:
        stop_process(launch, signal.SIGTERM)
        wait_process(
            launch, SHUTDOWN_TERM_S, absolute_deadline=absolute_deadline)
    if launch.poll() is None:
        stop_process(launch, signal.SIGKILL)
        wait_process(
            launch, SHUTDOWN_TERM_S, absolute_deadline=absolute_deadline)
    if rviz is not None and rviz.poll() is None:
        stop_process(rviz, signal.SIGTERM)
        if not wait_process(
                rviz, 5.0, absolute_deadline=absolute_deadline):
            stop_process(rviz, signal.SIGKILL)
            wait_process(
                rviz, 5.0, absolute_deadline=absolute_deadline)
    return {
        'graceful': graceful,
        'launch_return_code': launch.returncode,
        'rviz_return_code': rviz.returncode if rviz is not None else None,
        'mission_pre_shutdown_pids': mission_pids,
        'observer_logger_pre_shutdown_pids': observer_logger_pids,
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
    termination_reason = 'FAILURE'
    termination_detail = 'preflight_failure'
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
    wall_watchdog = None
    horizon_monitor = None
    thin_session = None
    thin_offline_evaluation = None
    simulation_start_s = None
    simulation_horizon_target_s = None
    simulation_stop_s = None
    mission_active_started = None
    previous_domain = os.environ.get('ROS_DOMAIN_ID')

    configured_wall_limit = getattr(args, 'wall_watchdog_s', None)
    if configured_wall_limit is None:
        configured_wall_limit = getattr(args, 'mission_timeout', None)
    if configured_wall_limit is None:
        configured_wall_limit = 1800.0
    wall_deadline = started + max(0.0, float(configured_wall_limit))
    configured_horizon = getattr(args, 'simulation_horizon_s', None)
    if configured_horizon is None:
        configured_horizon = 1500.0
    configured_horizon = float(configured_horizon)

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
        if (not math.isfinite(float(configured_wall_limit)) or
                float(configured_wall_limit) <= 0.0):
            raise FastTrialError(
                'wall watchdog limit must be finite and positive')
        if not math.isfinite(configured_horizon) or configured_horizon <= 0.0:
            raise FastTrialError(
                'simulation horizon must be finite and positive')
        if args.experiment_condition in ('C', 'D'):
            require_explicit_local_path_gate_mode(args)
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
            'my_epuck_interfaces_module': str(
                importlib.util.find_spec('my_epuck_interfaces.msg').origin),
            'my_epuck_interfaces_types': [
                'my_epuck_interfaces/msg/FrontierCandidateArray',
                'my_epuck_interfaces/msg/TaskSnapshot',
                'my_epuck_interfaces/msg/TaskBidArray',
                'my_epuck_interfaces/msg/PairDecision',
                'my_epuck_interfaces/msg/DistributedExplorationStatus',
                'my_epuck_interfaces/msg/DistributedExplorationEvent',
            ],
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
        print(f'launch_file={launch_file_for_condition(args)}', flush=True)
        print(f'world_path={world}', flush=True)
        print(f'webots_driver_prefix={webots_driver_prefix}', flush=True)
        print(f'webots_driver_executable={webots_driver_executable}', flush=True)
        print(f'results_directory={attempt}', flush=True)
        print(f'effective_command={effective.strip()}', flush=True)

        environment.update({
            'ROS_DOMAIN_ID': str(args.ros_domain_id),
            'PYTHONUNBUFFERED': '1',
            # Synchronized-map rows are derived forensic evidence.  Keep the
            # raw TF/odom/map streams and all runtime events unchanged, but
            # defer the exact existing row reconstruction until the launch
            # has stopped advancing simulation time.
            'MY_EPUCK_DEFER_SYNC_MAP_FRAMES': (
                '1' if args.enable_forensic_capture else '0'),
        })
        log_file = (attempt / 'launch.log').open('w', encoding='utf-8')
        if time.monotonic() >= wall_deadline:
            raise FastTrialError('wall watchdog budget expired before launch')
        # Arm the emergency watchdog before spawning the launch wrapper.  The
        # process list is attached immediately after each subprocess exists.
        wall_watchdog = WallWatchdog(
            wall_deadline,
            lambda: probe.latest_clock_s if probe is not None else None,
            started=started)
        wall_watchdog.start()
        if (args.experiment_condition == 'C' and
                str(getattr(args, 'observer_architecture', 'legacy')).lower()
                == 'thin'):
            thin_session = ThinEvidenceSession.start(
                attempt / 'observer', attempt.name,
                ('robot1', 'robot2'), environment, {
                    **runtime_provenance,
                    'observer_architecture': 'thin',
                    'world_profile': args.world_profile,
                    'world_path': str(world),
                    'experiment_condition': args.experiment_condition,
                    'assignment_strategy': args.assignment_strategy,
                    'local_path_gate_mode': args.local_path_gate_mode,
                    'seed_provenance': getattr(args, 'seed_provenance', {}),
                }, args.webots_port,
                args.forensic_ground_truth_sample_period_s,
                bool(args.enable_contact_capture),
                args.contact_sampling_period_ms,
                webots_driver_prefix, configured_horizon,
                condition=args.experiment_condition,
                unknown_initial_pose=(args.experiment_condition == 'C'),
                assignment_strategy=args.assignment_strategy)
            thin_session.add_to_watchdog(wall_watchdog)
        launch = subprocess.Popen(
            command, env=environment, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, bufsize=1,
            start_new_session=True)
        wall_watchdog.add_process(launch)
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
            wall_watchdog.add_process(rviz)
            output_threads.append(threading.Thread(
                target=pump_output, args=(rviz.stdout, log_file, lock),
                daemon=True))
            output_threads[-1].start()
        print(f'direct_subprocesses={1 + int(rviz is not None)}', flush=True)

        os.environ['ROS_DOMAIN_ID'] = str(args.ros_domain_id)
        rclpy.init()
        rclpy_started = True
        simulation_horizon_target_s = configured_horizon
        # A/B launch Nav2 with autostart enabled.  C/D explicitly disable
        # autostart and retain the runner-owned STARTUP path.
        probe = ReadyProbe(
            args.experiment_condition,
            nav2_autostart=args.experiment_condition in ('A', 'B'))
        executor = SingleThreadedExecutor()
        executor.add_node(probe)
        probe.executor = executor
        # This monitor starts before any condition-specific readiness or
        # cooperative handoff wait.  It is deliberately independent of the
        # readiness/mission state machine so C/D cannot bypass the scientific
        # horizon while waiting for their second-phase Nav2 graph.
        horizon_monitor = SimulationHorizonMonitor(
            configured_horizon,
            lambda: probe.latest_clock_s if probe is not None else None,
            overrun_fence_s=configured_horizon + 60.0)
        horizon_monitor.add_process(launch)
        horizon_monitor.start()
        readiness_deadline = min(
            started + (args.startup_timeout or 300.0), wall_deadline,
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
                if horizon_monitor.reached.is_set():
                    raise SimulationHorizonReached(
                        'simulation horizon reached during readiness')
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
        nav2_details = probe.activate_and_check_nav2(
            nav2_deadline, horizon_monitor.reached.is_set)
        phases['nav2_ready'] = time.monotonic() - phase_start
        print(f'nav2_ready_s={phases["nav2_ready"]:.3f}', flush=True)
        if probe.clock_start_s is None:
            raise FastTrialError('no live /clock sample was received')
        simulation_start_s = probe.clock_start_s
        simulation_horizon_target_s = configured_horizon
        ready_time = utc_now()
        ready_duration = time.monotonic() - started
        ready_wall_elapsed = ready_duration
        mission_active_started = time.monotonic()
        phases['total_ready'] = ready_duration
        print(f'total_ready_s={ready_duration:.3f}', flush=True)
        print(
            'SIMULATION_CLOCK_READY start_s=%.9f horizon_s=%.9f' % (
                simulation_start_s, simulation_horizon_target_s),
            flush=True)
        print('FAST_TRIAL_READY', flush=True)

        while True:
            if horizon_monitor.overrun.is_set():
                simulation_stop_s = (
                    horizon_monitor.overrun_sim_time_s or
                    probe.latest_clock_s)
                termination_reason = 'SIM_HORIZON_OVERRUN'
                termination_detail = 'simulation_horizon_overrun'
                return_code = 1
                break
            if horizon_monitor.reached.is_set():
                simulation_stop_s = (
                    horizon_monitor.reached_sim_time_s or
                    probe.latest_clock_s)
                termination_reason = 'SIM_TIME_COMPLETE'
                termination_detail = 'simulation_horizon_reached'
                return_code = 0
                break
            if wall_watchdog is not None and wall_watchdog.fired:
                termination_reason = 'WALL_WATCHDOG'
                termination_detail = 'wall_watchdog_expired'
                return_code = 1
                break
            if launch.poll() is not None:
                launch_return_code = launch.returncode
                termination_reason = 'FAILURE'
                termination_detail = 'launch_process_exit'
                return_code = 1
                break
            executor.spin_once(timeout_sec=0.1)
            sample_runner_rss()
            if (not args.hold_open and
                    probe.simulation_horizon_reached(
                        simulation_horizon_target_s)):
                simulation_stop_s = probe.latest_clock_s
                termination_reason = 'SIM_TIME_COMPLETE'
                termination_detail = 'simulation_horizon_reached'
                return_code = 0
                break
            if wall_watchdog is not None and wall_watchdog.fired:
                termination_reason = 'WALL_WATCHDOG'
                termination_detail = 'wall_watchdog_expired'
                return_code = 1
                break
    except SimulationHorizonReached:
        simulation_stop_s = (
            horizon_monitor.reached_sim_time_s
            if horizon_monitor is not None else None)
        termination_reason = 'SIM_TIME_COMPLETE'
        termination_detail = 'simulation_horizon_reached'
        return_code = 0
    except KeyboardInterrupt:
        termination_reason = 'MANUAL_ABORT'
        termination_detail = 'interrupt'
        return_code = 130
    except (FastTrialError, OSError, subprocess.SubprocessError) as error:
        termination_reason = 'FAILURE'
        termination_detail = f'failure: {error}'
        print(f'FAST_TRIAL_FAILURE reason={error}', file=sys.stderr, flush=True)
        return_code = 1
    except Exception as error:
        termination_reason = 'FAILURE'
        termination_detail = (
            f'unexpected_failure: {type(error).__name__}: {error}')
        print(f'FAST_TRIAL_FAILURE reason={termination_detail}', file=sys.stderr,
              flush=True)
        return_code = 1
    finally:
        # A wall watchdog firing during readiness or cleanup takes precedence
        # over a secondary launch/cleanup exception: the artifact must state
        # that the emergency real-time limit, not the scientific horizon,
        # ended the run.
        if wall_watchdog is not None and wall_watchdog.fired:
            termination_reason = 'WALL_WATCHDOG'
            termination_detail = 'wall_watchdog_expired'
        if simulation_start_s is None and probe is not None:
            simulation_start_s = probe.clock_start_s
        if horizon_monitor is not None:
            horizon_monitor.stop()
        if probe is not None:
            if simulation_stop_s is None:
                simulation_stop_s = probe.latest_clock_s
            probe.destroy_node()
        if executor is not None:
            executor.shutdown()
        if rclpy_started and rclpy.ok():
            rclpy.shutdown()
        if previous_domain is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = previous_domain
        if thin_session is not None:
            try:
                thin_session.stop(
                    timeout_s=max(0.0, min(8.0, wall_deadline - time.monotonic())),
                    recorder_timeout_s=max(
                        0.0, min(8.0, wall_deadline - time.monotonic())))
            except Exception as error:
                print(
                    f'THIN_EVIDENCE_FINALIZATION_FAILED {type(error).__name__}: {error}',
                    file=sys.stderr, flush=True)
        if launch is not None:
            cleanup = shutdown_processes(
                launch, rviz, absolute_deadline=wall_deadline)
            launch_return_code = cleanup['launch_return_code']
        else:
            cleanup = {}
        cleanup['campaign_webots_driver_pids'] = (
            cleanup_campaign_webots_drivers(
                started_wall, absolute_deadline=wall_deadline))
        if thin_session is not None:
            try:
                # Frontend handoff JSON is finalized during launch teardown;
                # refresh the immutable bag's semantic contract only after
                # those writers have exited.
                thin_session.refresh_post_shutdown_semantics()
            except Exception as error:
                print(
                    f'THIN_POST_SHUTDOWN_SEMANTICS_FAILED '
                    f'{type(error).__name__}: {error}',
                    file=sys.stderr, flush=True)
        if wall_watchdog is not None:
            wall_watchdog.stop()
        mission_end_monotonic = time.monotonic()
        mission_end_utc = utc_now()
        if thin_session is not None:
            active_wall_s = (mission_end_monotonic - mission_active_started
                             if mission_active_started is not None else None)
            sim_delta_s = (float(simulation_stop_s) -
                           float(simulation_start_s)
                           if simulation_stop_s is not None and
                           simulation_start_s is not None else None)
            thin_session.record_runtime_boundary({
                'simulation_start_time_s': simulation_start_s,
                'simulation_end_time_s': simulation_stop_s,
                'simulation_horizon_s': configured_horizon,
                'termination_reason': termination_reason,
                'termination_detail': termination_detail,
                'mission_wall_start_utc': ready_time,
                'mission_wall_end_utc': mission_end_utc,
                'mission_active_wall_duration_s': active_wall_s,
                'authoritative_rtf': (
                    sim_delta_s / active_wall_s
                    if sim_delta_s is not None and active_wall_s and
                    active_wall_s > 0.0 else None),
            })
        if thin_session is not None:
            try:
                thin_offline_evaluation = thin_session.evaluate_offline()
            except Exception as error:
                print(
                    f'THIN_OFFLINE_EVALUATION_FAILED {type(error).__name__}: {error}',
                    file=sys.stderr, flush=True)
        if log_file is not None:
            for thread in output_threads:
                remaining = max(0.0, wall_deadline - time.monotonic())
                thread.join(timeout=min(2.0, remaining))
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
            'exit_reason': termination_detail,
            'termination_reason': termination_reason,
            'termination_detail': termination_detail,
            'simulation_horizon_s': configured_horizon,
            'simulation_horizon_target_s': simulation_horizon_target_s,
            'simulation_horizon_fence_s': configured_horizon + 60.0,
            'simulation_start_time_s': simulation_start_s,
            'simulation_end_time_s': simulation_stop_s,
            'ros_clock_start_s': simulation_start_s,
            'ros_clock_end_s': simulation_stop_s,
            'simulation_horizon_reached': (
                termination_reason == 'SIM_TIME_COMPLETE'),
            'simulation_horizon_overrun': (
                termination_reason == 'SIM_HORIZON_OVERRUN'),
            'simulation_measurement_cutoff_s': (
                simulation_horizon_target_s
                if simulation_horizon_target_s is not None else None),
            'wall_watchdog_s': float(configured_wall_limit),
            'wall_watchdog_armed': bool(
                wall_watchdog is not None and wall_watchdog.armed),
            'wall_watchdog_termination': bool(
                wall_watchdog is not None and wall_watchdog.fired),
            'wall_watchdog_fired_wall_elapsed_s': (
                wall_watchdog.fired_wall_elapsed_s
                if wall_watchdog is not None else None),
            'wall_watchdog_fired_sim_time_s': (
                wall_watchdog.fired_sim_time_s
                if wall_watchdog is not None else None),
            'observer_finalization_complete': observer_finalization_status(
                attempt),
            'launch_return_code': launch_return_code,
            'world_profile': args.world_profile,
            'world_path': str(world) if world else args.world_path,
            'experiment_condition': args.experiment_condition,
                'observer_architecture': getattr(
                args, 'observer_architecture', 'legacy'),
            'seed_provenance': getattr(args, 'seed_provenance', {}),
            'profile_metadata': getattr(args, 'profile_metadata', {}),
            'sensor_profile': args.sensor_profile,
            'rendering': args.rendering,
            'rviz': args.rviz,
            'diagnostic_mode': args.diagnostic_mode,
            'local_path_gate_mode': args.local_path_gate_mode,
            'ros_domain_id': args.ros_domain_id,
            'webots_port': args.webots_port,
            'launch_file': launch_file_for_condition(args),
            'cleanup': cleanup,
            'direct_subprocesses': 1 + int(rviz is not None),
            'peak_runner_rss_bytes': peak_runner_rss,
            'prefix': prefix,
            'webots_driver_prefix': webots_driver_prefix,
            'webots_driver_executable': webots_driver_executable,
            'runtime_provenance': runtime_provenance,
            'nav2': locals().get('nav2_details', {}),
            'nav2_readiness_telemetry': (
                probe.readiness_telemetry_snapshot(
                    locals().get('nav2_readiness_final_reason'))
                if probe is not None else {}),
            'thin_evidence_finalization': (
                json.loads(thin_session.finalization_path.read_text())
                if thin_session is not None and
                thin_session.finalization_path.exists() else None),
            'thin_offline_evaluation': thin_offline_evaluation,
        }
        if attempt is not None:
            (attempt / 'fast_trial_summary.json').write_text(
                json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    return return_code


def main(argv=None) -> int:
    args = parser().parse_args(argv)
    settings = PROFILE_SETTINGS[args.world_profile]
    explicit_mission_timeout = args.mission_timeout
    if args.mission_timeout is None:
        args.mission_timeout = settings['mission_timeout']
    if args.wall_watchdog_s is None:
        args.wall_watchdog_s = (
            explicit_mission_timeout
            if explicit_mission_timeout is not None
            else settings['mission_timeout'])
    elif (explicit_mission_timeout is not None and
          float(explicit_mission_timeout) != float(args.wall_watchdog_s)):
        raise SystemExit(
            '--mission-timeout and --wall-watchdog-s must match when both '
            'are supplied; use --wall-watchdog-s for new campaigns')
    if args.simulation_horizon_s is None:
        args.simulation_horizon_s = settings.get(
            'simulation_horizon_s', 1500.0)
    if args.startup_timeout is None:
        args.startup_timeout = settings['startup_timeout']
    if not ROS_DOMAIN_MIN <= args.ros_domain_id <= ROS_DOMAIN_MAX:
        raise SystemExit(
            f'--ros-domain-id must be between {ROS_DOMAIN_MIN} and '
            f'{ROS_DOMAIN_MAX} for the active CycloneDDS port profile')
    return run(args)


if __name__ == '__main__':
    raise SystemExit(main())
