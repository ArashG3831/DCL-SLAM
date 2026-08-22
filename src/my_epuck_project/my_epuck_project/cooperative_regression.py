"""Self-contained ten-trial cooperative simulation regression campaign."""

import argparse
from concurrent.futures import FIRST_COMPLETED, ThreadPoolExecutor, wait
import csv
from datetime import datetime, timezone
import hashlib
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
import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer, TransformListener

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
    'MISSION_COMPLETE', 'BOUNDED_DIAGNOSTIC', 'SIMULATED_MISSION_TIMEOUT',
    'EMERGENCY_WALL_TIMEOUT', 'INFRASTRUCTURE_FAILURE',
    'RUNTIME_PROCESS_CRASH', 'MANUAL_STOP', 'CLEAN_SHUTDOWN',
    'SHUTDOWN_DEGRADED', 'SLAM_FILTER_OUTPUT_STALL',
    # Read compatibility for reports produced before the classification split.
    'PASS', 'SYSTEM_FAILURE', 'MISSION_TIMEOUT', 'PROCESS_CRASH',
    'INCOMPLETE_ARTIFACTS', 'USER_INTERRUPTED',
)
NON_FAILURE_CLASSIFICATIONS = {
    'MISSION_COMPLETE', 'BOUNDED_DIAGNOSTIC', 'CLEAN_SHUTDOWN', 'PASS',
}
REQUIRED_OBSERVER_FILES = (
    'summary.json', 'events.jsonl', 'warnings.jsonl', 'topic_health.csv',
    'coverage.csv', 'robot1_timeseries.csv', 'robot2_timeseries.csv',
)
EXPECTED_NODE_SUFFIXES = (
    '/robot1/distributed_frontier_assignment',
    '/robot2/distributed_frontier_assignment',
    '/robot1/map_fusion',
    '/robot2/map_fusion',
)
UNKNOWN_POSE_EXPECTED_NODE_SUFFIXES = (
    '/robot1/local_distributed_frontier_assignment',
    '/robot2/local_distributed_frontier_assignment',
    '/robot1/unknown_pose_frontend',
    '/robot2/unknown_pose_frontend',
)
NAV2_NODES = (
    'controller_server', 'smoother_server', 'planner_server',
    'route_server', 'behavior_server', 'velocity_smoother',
    'collision_monitor', 'bt_navigator', 'waypoint_follower',
)
ROS_DOMAIN_MIN = 0
ROS_DOMAIN_MAX = 232
WEBOTS_PORT_MIN = 1024
WEBOTS_PORT_MAX = 65535
PROGRESS_LOCK = threading.RLock()
# A fresh TF buffer may initially contain Slam Toolbox map->odom samples that
# are future-dated relative to the odom->base samples.  Keep this as a bounded
# startup readiness budget; it does not alter any TF publisher or Nav2
# transform tolerance.
TF_READINESS_TIMEOUT_S = 10.0


def utc_now():
    return datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')


def detect_slam_filter_output_stall(observer):
    """Detect fresh fixed scans followed by a permanently stale SLAM stream."""
    path = observer / 'topic_health.csv'
    if not path.is_file():
        return None
    rows = {}
    with path.open(newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            rows.setdefault((row['robot_id'], row['topic_name']), []).append(row)
    details = {}
    for robot in ('robot1', 'robot2'):
        fixed = rows.get((robot, f'/{robot}/scan_d500_fixed'), [])
        slam = rows.get((robot, f'/{robot}/scan_d500_slam'), [])
        fixed_fresh = {row['event_sequence'] for row in fixed
                       if row.get('stale') != 'True'}
        first = next((row for row in slam
                      if row.get('stale') == 'True'
                      and row['event_sequence'] in fixed_fresh), None)
        if first:
            details[robot] = {
                'first_observed_stale_ros_time_s': float(first['ros_time_sec']),
                'last_observed_age_s': float(slam[-1]['topic_age_s'])
                if slam and slam[-1].get('topic_age_s') else None,
            }
    return details or None


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
    try:
        result = run([str(executable), '-ano', '-p', 'tcp'], timeout=10)
    except (OSError, subprocess.TimeoutExpired):
        # This is supplemental Windows PID telemetry. The direct TCP probe
        # remains authoritative and a slow WSL interop call must not abort an
        # otherwise healthy simulation.
        return None
    expression = re.compile(
        rf'^\s*TCP\s+\S*:{int(port)}\s+\S+\s+LISTENING\s+(\d+)\s*$',
        re.IGNORECASE | re.MULTILINE)
    match = expression.search(result.stdout)
    return int(match.group(1)) if match else None



def manual_rviz_command(world_profile='small', use_sim_time=False):
    """Return the installed passive RViz command with explicit ROS time."""
    package = Path(get_package_share_directory('my_epuck_project'))
    selected = profile(world_profile, package / 'worlds')
    config = manual_rviz_path(selected, package / 'resource')
    return [
        'rviz2', '-d', str(config), '--ros-args',
        '-p', f'use_sim_time:={str(bool(use_sim_time)).lower()}',
    ]


def rviz_gui_environment(environment):
    """Prepare a WSLg-safe environment for the external RViz window.

    RViz's OGRE backend in the installed Jazzy build requires X11/XCB under
    WSLg.  Letting Qt choose Wayland can create a taskbar entry but fail to
    create the GLX render window.  The software-rasterizer override is also
    intentionally removed because it breaks RViz's indexed occupancy-map
    shader on this host.
    """
    result = dict(environment)
    result.pop('LIBGL_ALWAYS_SOFTWARE', None)
    if result.get('DISPLAY'):
        result['QT_QPA_PLATFORM'] = 'xcb'
        result['QT_X11_NO_MITSHM'] = '1'
    return result


def activate_rviz_window(pid=None, timeout_s=5.0):
    """Raise the RViz top-level X11 window when running under WSLg.

    ``Popen`` succeeding only proves that the process exists.  WSLg can leave
    a newly-created Qt window behind the taskbar or on an inactive surface.
    This best-effort helper uses only standard X11 utilities/library calls and
    silently does nothing on non-X11 hosts.
    """
    if not os.environ.get('DISPLAY'):
        return False
    try:
        import ctypes
        import ctypes.util

        lib = ctypes.CDLL(ctypes.util.find_library('X11') or 'libX11.so.6')
        lib.XOpenDisplay.restype = ctypes.c_void_p
        lib.XOpenDisplay.argtypes = [ctypes.c_char_p]
        display = lib.XOpenDisplay(os.environ['DISPLAY'].encode())
        if not display:
            return False
        lib.XMapRaised.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        lib.XRaiseWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        lib.XSetInputFocus.argtypes = [
            ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_ulong]
        lib.XFlush.argtypes = [ctypes.c_void_p]
        lib.XCloseDisplay.argtypes = [ctypes.c_void_p]
    except (AttributeError, OSError, TypeError):
        return False

    pattern = re.compile(r'^\s*(0x[0-9a-fA-F]+) "[^"]* - RViz"')
    deadline = time.monotonic() + max(0.0, timeout_s)
    try:
        while time.monotonic() < deadline:
            try:
                result = subprocess.run(
                    ['xwininfo', '-root', '-tree'],
                    check=False, capture_output=True, text=True, timeout=1.0)
            except (OSError, subprocess.TimeoutExpired):
                return False
            for line in result.stdout.splitlines():
                match = pattern.match(line)
                if not match:
                    continue
                window = int(match.group(1), 16)
                if pid is not None:
                    try:
                        properties = subprocess.run(
                            ['xprop', '-id', match.group(1), '_NET_WM_PID'],
                            check=False, capture_output=True, text=True,
                            timeout=1.0).stdout
                    except (OSError, subprocess.TimeoutExpired):
                        properties = ''
                    if not re.search(rf'\b{int(pid)}\b', properties):
                        continue
                lib.XMapRaised(display, window)
                lib.XRaiseWindow(display, window)
                # RevertToParent=2 and CurrentTime=0.
                lib.XSetInputFocus(display, window, 2, 0)
                lib.XFlush(display)
                return True
            time.sleep(0.1)
        return False
    finally:
        lib.XCloseDisplay(display)


def resolve_runner_profile(args):
    """Use the saved source world as authoritative runtime input.

    Launch code remains installed, but the selected world and its project
    companion are copied by WebotsLauncher from this exact source path.  This
    permits an intentional saved-world pose edit without a rebuild.
    """
    workspace = Path(args.workspace).resolve()
    source_worlds = workspace / 'src' / 'my_epuck_project' / 'worlds'
    installed_package = Path(get_package_share_directory('my_epuck_project'))
    source = profile(
        args.world_profile, source_worlds,
        ideal_encoder_sensing=args.ideal_encoder_sensing)
    installed = profile(
        args.world_profile, installed_package / 'worlds',
        ideal_encoder_sensing=args.ideal_encoder_sensing)
    source_hash = source['world_metadata']['sha256']
    installed_hash = installed['world_metadata']['sha256']
    args.source_world_path = source['world_path']
    args.installed_world_path = installed['world_path']
    args.profile_metadata = profile_summary(source)
    args.profile_metadata['installed_world_sha256'] = installed_hash
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
    print(f'initial_separation_m={metadata["initial_separation_m"]}')
    print('transform_source=WORLD_DERIVED')
    print(f'world_validation={metadata["validation"]}')
    print('transform_consumers=map_fusion,shared_map_alignment,peer_pose,teammate_filter,logger,metrics,report')
    print(f'mission_timeout_s={args.mission_timeout}')
    print(f'time_mode={args.time_mode}')
    print(f'use_sim_time={args.time_mode == "sim"}')
    print(f'use_scan_matching={args.use_scan_matching}')
    print(f'do_loop_closing={args.do_loop_closing}')
    print(f'ideal_encoder_sensing={args.ideal_encoder_sensing}')
    print(f'encoder_profile={selected["encoder_profile"]}')
    print(f'execution_profile={args.execution_profile}')
    print(f'rendering={args.rendering}')
    print(f'rviz={args.rviz}')
    print(f'sensor_profile={args.sensor_profile}')
    print(f'logging_mode=observer,console_status={args.logger_console_status}')
    print(f'fast_mode={args.fast_mode}')
    print(f'emergency_wall_runtime_s={args.emergency_wall_runtime}')
    print('expected_clock_publisher=webots_ros2_driver Ros2Supervisor -> /clock')
    print(
        f'webots_port_range={args.webots_port_base}..'
        f'{args.webots_port_base + args.trials - 1}')
    print(
        f'ros_domain_range={args.ros_domain_base}..'
        f'{args.ros_domain_base + args.trials - 1}')


def resolved_trial_resources(args, trial_number):
    """Return the isolated domain and Webots port for one trial."""
    if not 1 <= trial_number <= args.trials:
        raise ValueError(
            f'trial number {trial_number} is outside 1..{args.trials}')
    return (
        args.ros_domain_base + trial_number - 1,
        args.webots_port_base + trial_number - 1,
    )


def validate_resource_bounds(args):
    """Validate derived resources before any ROS graph probe is attempted."""
    if not ROS_DOMAIN_MIN <= args.ros_domain_base <= ROS_DOMAIN_MAX:
        raise SystemExit(
            f'--ros-domain-base must be between {ROS_DOMAIN_MIN} and '
            f'{ROS_DOMAIN_MAX}')
    last_domain = args.ros_domain_base + args.trials - 1
    if last_domain > ROS_DOMAIN_MAX:
        raise SystemExit(
            f'ROS domain range {args.ros_domain_base}..{last_domain} exceeds '
            f'supported maximum {ROS_DOMAIN_MAX}')
    if not WEBOTS_PORT_MIN <= args.webots_port_base <= WEBOTS_PORT_MAX:
        raise SystemExit(
            f'--webots-port-base must be between {WEBOTS_PORT_MIN} and '
            f'{WEBOTS_PORT_MAX}')
    last_port = args.webots_port_base + args.trials - 1
    if last_port > WEBOTS_PORT_MAX:
        raise SystemExit(
            f'Webots port range {args.webots_port_base}..{last_port} exceeds '
            f'supported maximum {WEBOTS_PORT_MAX}')


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


def clock_readiness(domain, timeout_s=4.0):
    """Probe one advancing clock in a fresh, isolated rclpy context.

    This intentionally does not use the ROS CLI daemon.  All discovery,
    publisher counting, and message receipt happen through the same node and
    context, and all deadlines use the process wall clock.
    """
    started = time.monotonic()
    previous_domain = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    context = Context()
    details = {
        'probe_pid': os.getpid(),
        'domain_id': int(domain),
        'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION', 'default'),
        'context_identity': id(context),
        'probe_start_wall_monotonic': started,
        'topic': '/clock',
        'topic_type': 'rosgraph_msgs/msg/Clock',
        'qos': {
            'history': 'KEEP_LAST', 'depth': 10,
            'reliability': 'BEST_EFFORT', 'durability': 'VOLATILE',
        },
        'publisher_count_samples': [],
        'samples': [],
        'sample_wall_times': [],
        'max_inter_sample_wall_gap_s': None,
    }
    values = []
    sample_wall_times = []
    node = None
    executor = None
    try:
        rclpy.init(args=None, context=context)
        node = rclpy.create_node('regression_clock_probe', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        def receive(message):
            values.append(message.clock.sec + message.clock.nanosec * 1e-9)
            sample_wall_times.append(time.monotonic())

        node.create_subscription(Clock, '/clock', receive, qos)
        loop_started = time.monotonic()
        details['setup_wall_elapsed_s'] = loop_started - started
        deadline = loop_started + timeout_s
        while time.monotonic() < deadline and len(values) < 2:
            count = node.count_publishers('/clock')
            details['publisher_count_samples'].append({
                'wall_elapsed_s': time.monotonic() - started,
                'count': count,
            })
            executor.spin_once(timeout_sec=0.1)
        count = node.count_publishers('/clock')
        details['publishers'] = count
        details['samples'] = list(values)
        details['sample_count'] = len(values)
        details['sample_wall_times'] = [t - started for t in sample_wall_times]
        if len(sample_wall_times) > 1:
            details['max_inter_sample_wall_gap_s'] = max(
                b - a for a, b in zip(sample_wall_times, sample_wall_times[1:]))
        if count == 0:
            details['reason'] = 'CLOCK_TOPIC_MISSING'
        elif count != 1:
            details['reason'] = 'CLOCK_PUBLISHER_COUNT_INVALID'
        elif len(values) < 2:
            details['reason'] = 'CLOCK_NOT_ADVANCING'
        elif values[-1] <= values[0]:
            details['reason'] = 'CLOCK_NOT_ADVANCING'
        else:
            details['reason'] = 'READY'
        return details['reason'] == 'READY', details
    except Exception as exc:  # probe diagnostics must identify internal faults
        details['reason'] = 'CLOCK_PROBE_INTERNAL_ERROR'
        details['error'] = f'{type(exc).__name__}: {exc}'
        return False, details
    finally:
        if node is not None:
            if executor is not None:
                executor.remove_node(node)
            node.destroy_node()
        if executor is not None:
            executor.shutdown()
        if context.ok():
            context.shutdown()
        if previous_domain is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = previous_domain


def tf_readiness_requirements(unknown_initial_pose=False):
    """Return the TF edges required before Nav2 startup.

    Unknown-pose runs intentionally have no shared frame before the canonical
    handoff.  Requiring that post-handoff edge here makes readiness impossible.
    """
    required = []
    for robot in ('robot1', 'robot2'):
        required.append(
            (f'{robot}/base_footprint', f'{robot}/odom', 'odom_to_base'))
        if not unknown_initial_pose:
            required.append(
                ('shared_map', f'{robot}/base_footprint', 'shared_to_base'))
    return required


def tf_readiness(
        domain, timeout_s=TF_READINESS_TIMEOUT_S, unknown_initial_pose=False):
    """Verify odometry and the global transforms required by costmaps."""
    started = time.monotonic()
    previous_domain = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    context = Context()
    details = {
        'probe_pid': os.getpid(),
        'domain_id': int(domain),
        'context_identity': id(context),
        'requested_transforms': [],
        'odom_received': {},
        'first_odom_wall_elapsed_s': {},
        'first_transform_wall_elapsed_s': {},
        'latest_odom_stamp': {},
    }
    odom_received = {}
    first_odom = {}
    first_transform = {}
    node = None
    executor = None
    try:
        rclpy.init(args=None, context=context)
        node = rclpy.create_node('regression_tf_probe', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        buffer = Buffer()
        TransformListener(buffer, node)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        for robot in ('robot1', 'robot2'):
            odom_received[robot] = False
        required = tf_readiness_requirements(unknown_initial_pose)
        details['requested_transforms'] = [
            {'target': target, 'source': source, 'role': role}
            for target, source, role in required]

        def receive(message, name):
            odom_received[name] = True
            first_odom.setdefault(name, time.monotonic() - started)
            stamp = message.header.stamp
            details['latest_odom_stamp'][name] = {
                'sec': stamp.sec, 'nanosec': stamp.nanosec,
            }

        for robot in ('robot1', 'robot2'):
            node.create_subscription(
                Odometry, f'/{robot}/odom',
                lambda message, name=robot: receive(message, name), qos)
        loop_started = time.monotonic()
        details['setup_wall_elapsed_s'] = loop_started - started
        deadline = loop_started + timeout_s
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
            for target, source, role in required:
                key = f'{role}:{source}->{target}'
                if key not in first_transform and buffer.can_transform(
                        target, source, Time(),
                        timeout=Duration(seconds=0.0)):
                    first_transform[key] = time.monotonic() - started
            if len(first_transform) == len(required):
                break
        details['odom_received'] = dict(odom_received)
        details['first_odom_wall_elapsed_s'] = dict(first_odom)
        details['first_transform_wall_elapsed_s'] = dict(first_transform)
        if len(first_transform) == len(required):
            details['reason'] = 'READY'
        else:
            details['missing_transforms'] = [
                {'target': target, 'source': source, 'role': role}
                for target, source, role in required
                if f'{role}:{source}->{target}' not in first_transform]
            details['reason'] = 'TF_READINESS_TIMEOUT'
        return details['reason'] == 'READY', details
    except Exception as exc:
        details['reason'] = 'TF_PROBE_INTERNAL_ERROR'
        details['error'] = f'{type(exc).__name__}: {exc}'
        return False, details
    finally:
        if node is not None:
            if executor is not None:
                executor.remove_node(node)
            node.destroy_node()
        if executor is not None:
            executor.shutdown()
        if context.ok():
            context.shutdown()
        if previous_domain is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = previous_domain


def lifecycle_startup_action(manager_active, prior_state):
    """Choose a lifecycle action without re-entering an active manager.

    Nav2 1.3.x implements ManageLifecycleNodes.STARTUP as an unconditional
    configure-then-activate sequence.  The campaign runner may be called again
    while the other robot's startup is still pending, so a previous per-robot
    startup state must prevent a second STARTUP request.
    """
    if manager_active:
        return 'ALREADY_ACTIVE'
    if prior_state in ('STARTING', 'ACTIVE', 'FAILED'):
        return 'WAIT_FOR_PRIOR_ATTEMPT'
    return 'SEND_STARTUP'


def wait_for_nav2_lifecycle_services(node, timeout_s):
    """Wait until every namespaced Nav2 lifecycle node exposes get_state."""
    missing = []
    deadline = time.monotonic() + timeout_s
    for robot in ('robot1', 'robot2'):
        for node_name in NAV2_NODES:
            service_name = f'/{robot}/{node_name}/get_state'
            client = node.create_client(GetState, service_name)
            remaining = max(0.0, deadline - time.monotonic())
            if not client.wait_for_service(timeout_sec=remaining):
                missing.append(service_name)
                return False, missing
    return True, missing


def get_nav2_lifecycle_states(node, executor, timeout_s):
    """Read every managed lifecycle node without issuing a transition."""
    states = {}
    deadline = time.monotonic() + timeout_s
    for robot in ('robot1', 'robot2'):
        states[robot] = {}
        for node_name in NAV2_NODES:
            client = node.create_client(
                GetState, f'/{robot}/{node_name}/get_state')
            remaining = max(0.0, deadline - time.monotonic())
            if not client.wait_for_service(timeout_sec=remaining):
                return False, states
            future = client.call_async(GetState.Request())
            while not future.done() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            if not future.done():
                return False, states
            response = future.result()
            states[robot][node_name] = {
                'id': int(response.current_state.id),
                'label': str(response.current_state.label),
            }
    return True, states


def activate_nav2(domain, timeout_s=10.0, startup_state=None):
    """Start both Nav2 managers after the required transforms are available.

    ``timeout_s`` is deliberately a *per-manager* budget.  Starting robot1
    can legitimately take most of a minute while Nav2 plugins and costmaps
    initialize; sharing that budget with robot2 can leave the latter only a
    few seconds to answer its lifecycle request.
    """
    previous_domain = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    context = Context()
    node = None
    executor = None
    details = {'domain_id': int(domain), 'services': {}}
    startup_state = startup_state if startup_state is not None else {}
    try:
        rclpy.init(args=None, context=context)
        node = rclpy.create_node(
            'regression_nav2_startup_gate', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        services_ready, missing = wait_for_nav2_lifecycle_services(
            node, timeout_s)
        if not services_ready:
            details['status'] = 'LIFECYCLE_SERVICES_NOT_READY'
            details['missing_services'] = missing
            return False, details
        state_services_ready, lifecycle_states = get_nav2_lifecycle_states(
            node, executor, timeout_s)
        if not state_services_ready:
            details['status'] = 'LIFECYCLE_STATE_QUERY_TIMEOUT'
            details['lifecycle_states'] = lifecycle_states
            return False, details
        details['lifecycle_states'] = lifecycle_states
        for robot in ('robot1', 'robot2'):
            robot_started = time.monotonic()
            manager_prefix = f'/{robot}/lifecycle_manager_navigation'
            robot_states = lifecycle_states[robot]
            active = all(
                state['label'] == 'active'
                for state in robot_states.values())
            action = lifecycle_startup_action(
                active, startup_state.get(robot))
            if action == 'ALREADY_ACTIVE':
                startup_state[robot] = 'ACTIVE'
                details['services'][robot] = 'ALREADY_ACTIVE'
                continue
            if action == 'WAIT_FOR_PRIOR_ATTEMPT':
                details['services'][robot] = 'STARTUP_IN_PROGRESS'
                return False, details

            if any(state['label'] != 'unconfigured'
                   for state in robot_states.values()):
                details['services'][robot] = 'PARTIAL_LIFECYCLE_STATE'
                return False, details

            service = f'{manager_prefix}/manage_nodes'
            client = node.create_client(ManageLifecycleNodes, service)
            remaining = max(0.0, timeout_s - (time.monotonic() - robot_started))
            if not client.wait_for_service(timeout_sec=remaining):
                details['services'][robot] = 'MANAGE_SERVICE_TIMEOUT'
                return False, details
            request = ManageLifecycleNodes.Request()
            request.command = ManageLifecycleNodes.Request.STARTUP
            startup_state[robot] = 'STARTING'
            future = client.call_async(request)
            deadline = time.monotonic() + max(
                0.0, timeout_s - (time.monotonic() - robot_started))
            while not future.done() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.1)
            if not future.done():
                details['services'][robot] = 'STARTUP_RESPONSE_TIMEOUT'
                return False, details
            response = future.result()
            details['services'][robot] = (
                'STARTED' if response.success else 'REJECTED')
            if not response.success:
                startup_state[robot] = 'FAILED'
                return False, details
            startup_state[robot] = 'ACTIVE'
        details['status'] = 'READY'
        return True, details
    except Exception as exc:
        details['status'] = 'INTERNAL_ERROR'
        details['error'] = f'{type(exc).__name__}: {exc}'
        return False, details
    finally:
        if node is not None:
            if executor is not None:
                executor.remove_node(node)
            node.destroy_node()
        if executor is not None:
            executor.shutdown()
        if context.ok():
            context.shutdown()
        if previous_domain is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = previous_domain


def validate_resource_range(args, completed_trials=None):
    validate_resource_bounds(args)
    completed_trials = completed_trials or set()
    failures = []
    for number in range(1, args.trials + 1):
        if number in completed_trials:
            continue
        domain, port = resolved_trial_resources(args, number)
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


def append_shutdown_event(path, event_type, **fields):
    """Append one durable, wall-timestamped cleanup event."""
    record = {
        'event_type': event_type,
        'wall_time_utc': utc_now(),
        'wall_monotonic_s': time.monotonic(),
        **fields,
    }
    with path.open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(record, sort_keys=True) + '\n')
        stream.flush()


def required_observer_artifacts(directory):
    return {
        name: (directory / name).is_file()
        for name in REQUIRED_OBSERVER_FILES
    }


def post_completion_evidence(
        attempt, final_state, settled_observed, cleanup, process_exit_codes):
    """Return evidence that completion preceded an artifact-only shutdown issue.

    The passive collector is the authoritative terminal-state witness.  The
    experiment logger is a diagnostic producer and can be killed by launch
    teardown after completion while it is flushing large bounded artifacts.
    This fallback is deliberately narrow: it requires coordinated settled
    shutdown, a clean process-tree cleanup, and two complete idle distributed
    states.  It is never used for an active or timed-out attempt.
    """
    if not settled_observed or not cleanup.get('all_exited', False):
        return None
    if any(code not in (None, 0)
           for code in (process_exit_codes or {}).values()):
        return None
    try:
        collector = json.loads(
            (attempt / 'collector_status.json').read_text(
                encoding='utf-8'))
    except (OSError, ValueError):
        return None
    if not (collector.get('both_mission_complete')
            and collector.get('settled')):
        return None
    robots = (final_state or {}).get('robots', {})
    evidence = {}
    for robot in ('robot1', 'robot2'):
        item = robots.get(robot, {})
        status = item.get('distributed_status')
        if not status or status.get('state') != 5:
            return None
        if item.get('navigation_active') is not False:
            return None
        evidence[robot] = {
            'state': status.get('state'),
            'reason': status.get('reason'),
            'navigation_active': item.get('navigation_active'),
        }
    return {
        'source': 'collector_status.json + final_state.json',
        'robots': evidence,
        'missing_or_incomplete_observer_artifacts': True,
    }


def persist_post_completion_mission_result(attempt):
    """Persist terminal success before launch teardown can kill the logger.

    The collector has already written ``final_state.json`` at this point, but
    the passive experiment logger is still inside the launch process group and
    may be terminated while flushing its larger diagnostic summary.  This
    small root-level result is the campaign-facing execution result; it is
    written only after both distributed terminal states are complete and idle.
    """
    try:
        final_state = json.loads(
            (attempt / 'final_state.json').read_text(encoding='utf-8'))
        collector = json.loads(
            (attempt / 'collector_status.json').read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return False
    if not (collector.get('both_mission_complete')
            and collector.get('settled')):
        return False
    robots = final_state.get('robots', {})
    statuses = []
    for robot in ('robot1', 'robot2'):
        item = robots.get(robot, {})
        status = item.get('distributed_status') or {}
        if (status.get('state') != 5
                or item.get('navigation_active') is not False):
            return False
        statuses.append(status)
    reasons = {str(item.get('reason', '')) for item in statuses}
    reason = next(iter(reasons), 'MISSION_COMPLETE')
    if len(reasons) != 1 or not reason.startswith('MISSION_COMPLETE_'):
        return False
    atomic_json(attempt / 'mission_result.json', {
        'schema_version': '1.0.0',
        'mission_status': 'SUCCEEDED',
        'terminal_reason': reason,
        'simulated_duration_s': collector.get('sim_time_seconds'),
        'wall_duration_s': collector.get('wall_elapsed_s'),
        'terminal_agreement': True,
        'robot1_final_state': {
            'state': statuses[0].get('state'),
            'terminal_reason': statuses[0].get('reason'),
            'navigation_active': False,
        },
        'robot2_final_state': {
            'state': statuses[1].get('state'),
            'terminal_reason': statuses[1].get('reason'),
            'navigation_active': False,
        },
        'completion_committed_before_teardown': True,
        'result_source': 'cooperative_trial_collector',
        'recommended_exit_code': 0,
    })
    return True


def classify_attempt(attempt, run_id, ready, timed_out, unexpected_exit,
                     interrupted, cleanup, settled_observed=False,
                     emergency_wall_timeout=False, process_exit_codes=None):
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
    completion_fallback = post_completion_evidence(
        attempt, final_state, settled_observed, cleanup, process_exit_codes)
    process_exit_codes = process_exit_codes or {}
    filter_stall = detect_slam_filter_output_stall(observer)
    shutdown_nonzero = {
        name: code for name, code in process_exit_codes.items()
        if code not in (None, 0)
    }
    if interrupted:
        classification = 'MANUAL_STOP'
    elif not ready:
        classification = 'INFRASTRUCTURE_FAILURE'
    elif emergency_wall_timeout:
        classification = 'EMERGENCY_WALL_TIMEOUT'
    elif unexpected_exit or log_review['traceback']:
        classification = 'RUNTIME_PROCESS_CRASH'
    elif filter_stall:
        classification = 'SLAM_FILTER_OUTPUT_STALL'
    elif timed_out:
        # A bounded diagnostic deliberately ends the attempt after a healthy
        # simulated interval; it is not a runtime failure.
        classification = 'BOUNDED_DIAGNOSTIC'
    elif final_state is None or not maps_valid or not all(
            observer_files.values()):
        classification = (
            'MISSION_COMPLETE' if completion_fallback
            else 'INCOMPLETE_ARTIFACTS')
    else:
        robots = final_state.get('robots', {})
        statuses = [
            robots.get(robot, {}).get('status') for robot in (
                'robot1', 'robot2')
        ]
        distributed_statuses = [
            robots.get(robot, {}).get('distributed_status') for robot in (
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
        distributed_protocol = all(item is not None
                                   for item in distributed_statuses)
        if distributed_protocol:
            status_complete = all(
                item.get('state') == 5
                and item.get('age_at_write_s', 1e9) <= 4.0
                for item in distributed_statuses
            )
            claims_terminal = all(claim is None or
                                  not claim.get('reserving', True)
                                  for claim in claims)
        else:
            status_complete = all(
                status and status.get('state') == 'MISSION_COMPLETE'
                and (
                    status.get(
                        'age_at_collection_s',
                        status.get('age_at_write_s', 1e9)) <= 4.0
                    or settled_observed
                )
                for status in statuses)
            claims_terminal = all(claim and not claim.get('reserving', True)
                                  for claim in claims)
        passed = (
            status_complete
            and claims_terminal
            and all(active is False for active in navigation)
            and system.get('internal_logger_error_count', 0) == 0
            and system.get('write_failures', 0) == 0
            and not shutdown_nonzero
            and cleanup.get('all_exited', False)
        )
        classification = 'MISSION_COMPLETE' if passed else 'SHUTDOWN_DEGRADED'
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
        'shutdown_time_nonzero_exits': shutdown_nonzero,
        'cleanup_quality': (
            'CLEAN' if cleanup.get('all_exited', False) else 'DEGRADED'),
        'primary_outcome': classification,
        'slam_filter_output_stall': filter_stall or {},
        'post_completion_artifact_anomaly': completion_fallback or {},
    }
    return classification, details


def readiness_probe_due(time_mode, ready, now, last_probe, interval=5.0):
    """Do not overwrite achieved readiness with later transient probes."""
    return (time_mode == 'sim' and not ready
            and now - last_probe >= interval)


def required_graph_ready(nodes, unknown_initial_pose=False):
    """Return whether the launch graph has reached infrastructure readiness.

    Frontier claims are mission-state messages, not startup infrastructure.
    The passive collector's ``ready`` field also requires one claim from each
    coordinator, which is intentionally not part of this gate: a coordinator
    may be healthy while it is still proposing or has no eligible claim.
    """
    expected = (UNKNOWN_POSE_EXPECTED_NODE_SUFFIXES
                if unknown_initial_pose else EXPECTED_NODE_SUFFIXES)
    return all(
        any(node.endswith(suffix) for node in nodes)
        for suffix in expected
    )


def mission_infrastructure_ready(
        clock_ok, tf_ok, nav2_started, nodes, unknown_initial_pose=False):
    """Require both Nav2 lifecycle managers before starting mission time."""
    return (clock_ok and tf_ok and nav2_started and
            required_graph_ready(nodes, unknown_initial_pose))


LIVE_PARAMETER_NODES = (
    'controller_server', 'velocity_smoother',
    'local_costmap/local_costmap', 'global_costmap/global_costmap',
    'planner_server',
)
LIVE_PARAMETER_FEATURE_VERSION = '2.0.0'


def navigation_preflight(workspace):
    """Verify the installed symlink/build contains current live-snapshot code."""
    workspace = Path(workspace)
    code_relative = (
        'my_epuck_project/navigation_live_parameters.py',
        'my_epuck_project/navigation_parameter_parity.py',
        'my_epuck_project/cooperative_regression.py',
        'my_epuck_project/cooperative_experiment_logger.py',
        'my_epuck_project/controller_pipeline_diagnostics.py',
        'my_epuck_project/teammate_scan_filter.py',
    )
    source_root = workspace / 'src/my_epuck_project'
    source_files = [source_root / item for item in code_relative]
    source_config = [source_root / 'resource/nav2_robot1_shared_map.yaml',
                     source_root / 'resource/nav2_robot2_shared_map.yaml']
    build_root = workspace / 'build/my_epuck_project'
    installed_files = [build_root / item for item in code_relative]
    installed_config = [
        workspace / 'install/my_epuck_project/share/my_epuck_project/resource'
        / path.name for path in source_config]

    def digest(paths):
        value = hashlib.sha256()
        for path in paths:
            try:
                value.update(path.read_bytes())
            except OSError:
                return None
        return value.hexdigest()

    source_hash = digest(source_files)
    installed_hash = digest(installed_files)
    source_config_hash = digest(source_config)
    installed_config_hash = digest(installed_config)
    marker = None
    marker_path = build_root / 'my_epuck_project/navigation_live_parameters.py'
    if marker_path.exists():
        text = marker_path.read_text(encoding='utf-8', errors='replace')
        if "SNAPSHOT_SCHEMA = '2.0.0'" in text:
            marker = LIVE_PARAMETER_FEATURE_VERSION
    return {
        'passed': source_hash == installed_hash
        and source_config_hash == installed_config_hash
        and marker == LIVE_PARAMETER_FEATURE_VERSION,
        'source_hash': source_hash,
        'installed_hash': installed_hash,
        'source_config_hash': source_config_hash,
        'installed_config_hash': installed_config_hash,
        'installed_module_path': str(marker_path),
        'installed_feature_version': marker,
        'expected_feature_version': LIVE_PARAMETER_FEATURE_VERSION,
    }


def capture_live_parameter_snapshots(attempt, environment, allocated_domain):
    """Capture bounded live Nav2 parameter dumps and normalized parity."""
    del environment
    from .navigation_live_parameters import (
        SNAPSHOT_SCHEMA, collect_snapshots)
    from .navigation_parameter_parity import live_snapshot_report, _normalize

    root = Path(attempt) / 'observer' / 'live_parameters'
    root.mkdir(parents=True, exist_ok=True)
    nodes = [f'/{robot}/{suffix}' for robot in ('robot1', 'robot2')
             for suffix in LIVE_PARAMETER_NODES]
    snapshots = collect_snapshots(nodes, allocated_domain)
    trees = {'robot1': {}, 'robot2': {}}
    statuses = []
    for snapshot in snapshots:
        node = snapshot['node']
        robot = node.split('/')[1]
        suffix = node.split('/', 2)[2]
        name = suffix.replace('/', '__')
        raw_path = root / f'{robot}__{name}.json'
        raw_path.write_text(json.dumps(snapshot, indent=2, sort_keys=True),
                            encoding='utf-8')
        item = {'robot': robot, 'node': node, 'status': snapshot['status'],
                'raw_file': raw_path.name,
                'returncode': 0 if snapshot['status'] == 'OK' else 1,
                'parsed': snapshot['status'] == 'OK'}
        if snapshot['status'] == 'OK':
            trees[robot][suffix] = snapshot['parameters']
        statuses.append(item)
    normalized = {robot: _normalize(tree) for robot, tree in trees.items()}
    (root / 'robot1_normalized.json').write_text(
        json.dumps(normalized['robot1'], indent=2, sort_keys=True),
        encoding='utf-8')
    (root / 'robot2_normalized.json').write_text(
        json.dumps(normalized['robot2'], indent=2, sort_keys=True),
        encoding='utf-8')
    report = live_snapshot_report(trees['robot1'], trees['robot2'], statuses)
    report.update({'schema_version': SNAPSHOT_SCHEMA,
                   'feature_version': LIVE_PARAMETER_FEATURE_VERSION,
                   'nodes': list(LIVE_PARAMETER_NODES),
                   'snapshots': statuses,
                   'bounded': True})
    (root / 'parity_report.json').write_text(
        json.dumps(report, indent=2, sort_keys=True), encoding='utf-8')
    return report


def internal_trial(args):
    """Run one launch and collector as children of one isolated supervisor."""
    attempt = Path(args.attempt_dir).resolve()
    attempt.mkdir(parents=True, exist_ok=False)
    (attempt / 'observer').mkdir()
    (attempt / 'ros_logs').mkdir()
    (attempt / 'tmp').mkdir()
    shutdown_events = attempt / 'shutdown_events.jsonl'
    start = time.monotonic()
    requested_rmw = os.environ.get('RMW_IMPLEMENTATION', '').strip()
    # Jazzy installations used by this workspace provide Fast DDS but may
    # not ship librmw_cyclonedds_cpp.so. Keep Fast DDS and disable its shared
    # memory transport, whose lock-file collision was observed in the failed
    # Robot 2 spawner.
    rmw_implementation = (
        requested_rmw if requested_rmw and requested_rmw != 'default'
        else 'rmw_fastrtps_cpp')
    fastdds_use_shm = os.environ.get('RMW_FASTRTPS_USE_SHM', '0')
    os.environ['RMW_IMPLEMENTATION'] = rmw_implementation
    os.environ['RMW_FASTRTPS_USE_SHM'] = fastdds_use_shm
    startup_timeline = {
        'runner_process_start': {'wall_elapsed_s': 0.0},
    }

    def mark_startup_stage(name, **details):
        startup_timeline[name] = {
            'wall_elapsed_s': time.monotonic() - start,
            **details,
        }

    mark_startup_stage('result_directory_created')
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
                args.mission_timeout + args.settling_period + 30.0
                if args.mission_timeout is not None else 600.0),
            'webots_port': args.webots_port,
            'webots_mode': args.webots_mode,
            'webots_gui': str(args.webots_gui).lower(),
            'sensor_profile': args.sensor_profile,
            'diagnostic_mode': args.diagnostic_mode,
            'launch_rviz': 'false',
            # The runner owns the mission budget.  The launch file's
            # TimerAction starts at launch time, before controller and
            # odometry readiness, so enabling it here can shut down a slow
            # startup before the mission has begun.
            'enable_mission_timeout': False,
            'use_sim_time': args.time_mode == 'sim',
            'use_scan_matching': args.use_scan_matching,
            'do_loop_closing': args.do_loop_closing,
            'unknown_initial_pose': args.unknown_initial_pose,
        'ideal_encoder_sensing': args.ideal_encoder_sensing,
        'encoder_profile': args.profile_metadata['encoder_profile'],
            'logger_console_status': False,
            'rmw_implementation': rmw_implementation,
            'rmw_fastdds_use_shm': fastdds_use_shm,
        },
        'rviz_requested': args.launch_rviz,
        'infrastructure_ready': False,
        'mission_inputs_ready': False,
        'hold_open_after_completion': args.hold_open_after_completion,
        'startup_timeline': startup_timeline,
    }
    mark_startup_stage('runtime_parameters_generated')
    preflight = navigation_preflight(args.workspace)
    atomic_json(attempt / 'runner_metadata.json', metadata)
    metadata['preflight'] = preflight
    atomic_json(attempt / 'runner_metadata.json', metadata)
    if not preflight['passed']:
        print('PREFLIGHT_FAILED ' + json.dumps(preflight, sort_keys=True),
              flush=True)
        return 1
    environment = os.environ.copy()
    # Fast DDS shared-memory port locks can collide between the many ROS 2
    # controller/spawner processes in an isolated WSL trial. Disable that
    # transport by default; an explicit user setting is preserved.
    environment.update({
        'ROS_DOMAIN_ID': str(args.ros_domain_id),
        'RMW_IMPLEMENTATION': rmw_implementation,
        'RMW_FASTRTPS_USE_SHM': fastdds_use_shm,
        'ROS_LOG_DIR': str(attempt / 'ros_logs'),
        'TMPDIR': str(attempt / 'tmp'),
        'TEMP': str(attempt / 'tmp'),
        'TMP': str(attempt / 'tmp'),
        'PYTHONUNBUFFERED': '1',
    })
    launch_command = [
        'ros2', 'launch', 'my_epuck_project',
        'two_robots_decentralized_exploration_launch.py',
        f'world_profile:={args.world_profile}',
        f'world_path:={args.source_world_path}',
        f'run_id:={args.run_id}',
        f'output_root:={attempt / "observer"}',
        f'mission_timeout_s:='
        f'{args.mission_timeout + args.settling_period + 30.0 if args.mission_timeout is not None else 600.0}',
        f'webots_port:={args.webots_port}',
        f'webots_mode:={args.webots_mode}',
        f'webots_gui:={str(args.webots_gui).lower()}',
        f'sensor_profile:={args.sensor_profile}',
        f'diagnostic_mode:={str(args.diagnostic_mode).lower()}',
        f'diagnostic_frontier_capture:={str(args.enable_forensic_capture).lower()}',
        'nav2_autostart:=false',
        'launch_rviz:=false',
        'enable_mission_timeout:=false',
        f'use_sim_time:={str(args.time_mode == "sim").lower()}',
        f'use_scan_matching:={str(args.use_scan_matching).lower()}',
        f'do_loop_closing:={str(args.do_loop_closing).lower()}',
        f'unknown_initial_pose:={str(args.unknown_initial_pose).lower()}',
        f'ideal_encoder_sensing:={str(args.ideal_encoder_sensing).lower()}',
        'logger_console_status:=false',
        f'enable_rosout_collection:={str(args.enable_rosout_collection).lower()}',
        f'enable_coverage_attribution:={str(args.enable_coverage_attribution).lower()}',
        f'enable_trajectory_overlap:={str(args.enable_trajectory_overlap).lower()}',
        f'enable_forensic_capture:={str(args.enable_forensic_capture).lower()}',
        f'forensic_snapshot_interval_s:={args.forensic_snapshot_interval_s}',
        f'enable_contact_capture:={str(args.enable_contact_capture).lower()}',
        f'contact_sampling_period_ms:={args.contact_sampling_period_ms}',
        f'controller_variant:={args.controller_variant}',
    ]
    collector_command = [
        'ros2', 'run', 'my_epuck_project', 'cooperative_trial_collector',
        '--ros-args',
        '-p', f'output_dir:={attempt}',
        '-p', f'run_id:={args.run_id}',
        '-p', f'settling_period_s:={args.settling_period}',
        '-p', f'use_sim_time:={str(args.time_mode == "sim").lower()}',
    ]
    launch_log = (attempt / 'launch.log').open('w', encoding='utf-8')
    collector_log = (attempt / 'collector.log').open('w', encoding='utf-8')
    launch = subprocess.Popen(
        launch_command, env=environment, stdout=launch_log,
        stderr=subprocess.STDOUT, text=True, preexec_fn=os.setpgrp)
    collector = subprocess.Popen(
        collector_command, env=environment, stdout=collector_log,
        stderr=subprocess.STDOUT, text=True, preexec_fn=os.setpgrp)
    processes = [launch, collector]
    diagnostic_processes = []
    diagnostic_logs = []
    mark_startup_stage('launch_process_started', pid=launch.pid)
    mark_startup_stage('collector_process_started', pid=collector.pid)
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
    rviz = None
    rviz_log = None
    rviz_attempted = False

    def start_high_rate_diagnostics():
        """Start passive command/wheel/odom capture after readiness only."""
        if (not args.enable_forensic_capture
                or not args.enable_high_rate_forensic_diagnostics
                or diagnostic_processes):
            return
        tool = Path(args.workspace) / 'src' / 'my_epuck_project' / 'tools' / (
            'turn_motion_diagnostics.py')
        output_dir = attempt / 'forensic' / 'high_rate'
        output_dir.mkdir(parents=True, exist_ok=True)
        for robot in ('robot1', 'robot2'):
            output = output_dir / f'{robot}_command_wheel_odom.csv'
            log_path = output_dir / f'{robot}_diagnostic.log'
            log = log_path.open('w', encoding='utf-8')
            command = [
                sys.executable, str(tool), '--robot', robot,
                '--output', str(output), '--duration-s',
                str(args.mission_timeout or 600.0), '--max-rows', '400000',
            ]
            process = subprocess.Popen(
                command, env=environment, stdout=log,
                stderr=subprocess.STDOUT, text=True, preexec_fn=os.setpgrp)
            diagnostic_processes.append(process)
            diagnostic_logs.append(log)
            processes.append(process)
            ps_processes.append(psutil.Process(process.pid))
            metadata.setdefault('diagnostic_processes', []).append({
                'robot': robot, 'pid': process.pid,
                'command': command, 'output': str(output),
                'start_stage': 'infrastructure_readiness_declared',
            })
        atomic_json(attempt / 'runner_metadata.json', metadata)

    def start_rviz():
        nonlocal rviz, rviz_log, rviz_attempted
        if not args.launch_rviz or rviz is not None or rviz_attempted:
            return
        rviz_attempted = True
        rviz_command = manual_rviz_command(
            args.world_profile, use_sim_time=args.time_mode == 'sim')
        rviz_environment = rviz_gui_environment(environment)
        rviz_log = (attempt / 'rviz.log').open('w', encoding='utf-8')
        metadata['rviz_start_attempted_utc'] = utc_now()
        metadata['rviz_command'] = rviz_command
        try:
            rviz = subprocess.Popen(
                rviz_command,
                env=rviz_environment,
                stdout=rviz_log,
                stderr=subprocess.STDOUT,
                text=True,
                preexec_fn=lambda: os.setpgid(0, launch.pid),
            )
        except OSError as error:
            metadata['rviz_start_error'] = (
                f'{type(error).__name__}: {error}')
            rviz_log.close()
            rviz_log = None
            atomic_json(attempt / 'runner_metadata.json', metadata)
            return
        processes.append(rviz)
        rviz_process = psutil.Process(rviz.pid)
        rviz_process.cpu_percent(None)
        ps_processes.append(rviz_process)
        # Popen only proves that the process exists.  WSLg may leave the Qt
        # surface behind the taskbar; raise it asynchronously without making
        # RViz startup part of the simulation readiness critical path.
        threading.Thread(
            target=activate_rviz_window, name='rviz-window-activation',
            args=(rviz.pid,), daemon=True).start()
        metadata.update({
            'rviz_pid': rviz.pid,
            'rviz_started_utc': utc_now(),
            'rviz_start_reason': 'clock_and_stack_readiness',
        })
    atomic_json(attempt / 'runner_metadata.json', metadata)
    interrupted = False
    requested_shutdown = False
    shutdown_reason = None

    holding_open = False
    collector_finalized_for_hold = False

    def stop(signum, frame):
        nonlocal interrupted
        del signum, frame
        interrupted = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    ready = False
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
    mission_sim_start = None
    clock_ok = args.time_mode == 'wall'
    tf_ok = args.time_mode == 'wall'
    nav2_started = args.time_mode == 'wall'
    nav2_startup_state = {}
    last_clock_probe = 0.0
    status_path = attempt / 'collector_status.json'
    try:
        while not interrupted:
            now = time.monotonic()
            status = {}
            try:
                status = json.loads(status_path.read_text(encoding='utf-8'))
            except (OSError, ValueError):
                pass
            if readiness_probe_due(
                    args.time_mode, ready, now, last_clock_probe):
                clock_ok, clock_details = clock_readiness(args.ros_domain_id)
                if 'first_clock_probe' not in startup_timeline:
                    mark_startup_stage('first_clock_probe')
                if clock_details.get('sample_wall_times'):
                    mark_startup_stage(
                        'first_clock_sample',
                        probe_wall_elapsed_s=clock_details[
                            'sample_wall_times'][0])
                if len(clock_details.get('samples', [])) >= 2:
                    mark_startup_stage(
                        'first_increasing_clock_pair',
                        samples=clock_details['samples'][:2])
                metadata['clock_readiness'] = clock_details
                metadata['clock_error'] = (
                    None if clock_ok else clock_details.get(
                        'reason', 'CLOCK_PROBE_INTERNAL_ERROR'))
                if clock_ok:
                    if mission_sim_start is None:
                        mission_sim_start = status.get('elapsed_s', 0.0)
                        metadata['clock_sim_start'] = mission_sim_start
                    tf_ok, tf_details = tf_readiness(
                        args.ros_domain_id,
                        unknown_initial_pose=args.unknown_initial_pose)
                    metadata['tf_readiness'] = tf_details
                    if tf_ok and not nav2_started:
                        startup_budget = max(
                            0.0, readiness_deadline - time.monotonic())
                        nav2_started, nav2_details = activate_nav2(
                            args.ros_domain_id, timeout_s=startup_budget,
                            startup_state=nav2_startup_state)
                        metadata['nav2_activation'] = nav2_details
                atomic_json(attempt / 'runner_metadata.json', metadata)
                # The probe uses a bounded wall-time wait and may finish
                # after the loop's initial timestamp. Readiness and mission
                # deadlines must use the actual observation time.
                now = time.monotonic()
                last_clock_probe = now
            # RViz is passive visualization. Start it as soon as the
            # simulation clock is alive, even if controller/spawner readiness
            # is still being diagnosed.
            if clock_ok and rviz is None:
                start_rviz()
            if status.get('ready') and not metadata['mission_inputs_ready']:
                metadata['mission_inputs_ready'] = True
                atomic_json(attempt / 'runner_metadata.json', metadata)
            if clock_ok and tf_ok and not ready:
                nodes = status.get('nodes', [])
                ready = mission_infrastructure_ready(
                    clock_ok, tf_ok, nav2_started, nodes,
                    args.unknown_initial_pose)
                if ready:
                    start_high_rate_diagnostics()
                    metadata['infrastructure_ready'] = True
                    mark_startup_stage(
                        'infrastructure_readiness_declared',
                        readiness_graph_nodes=nodes)
                    start_rviz()
                    mission_deadline = (now + args.mission_timeout
                                        if args.time_mode == 'wall'
                                        and args.mission_timeout is not None
                                        else None)
                    if mission_sim_start is None:
                        mission_sim_start = status.get('elapsed_s', 0.0)
                    metadata['readiness_elapsed_s'] = now - start
                    metadata['readiness_graph_nodes'] = nodes
                    mark_startup_stage(
                        'mission_timer_started',
                        mission_clock=(
                            'wall' if args.time_mode == 'wall' else 'sim'),
                        mission_sim_start=mission_sim_start)
                    metadata['startup_timeline'] = startup_timeline
                    atomic_json(attempt / 'runner_metadata.json', metadata)
                    try:
                        live_report = capture_live_parameter_snapshots(
                            attempt, environment, args.ros_domain_id)
                    except (OSError, subprocess.TimeoutExpired, ValueError) as error:
                        live_report = {
                            'schema_version': '1.0.0',
                            'conclusion': 'LIVE_PARAMETER_SNAPSHOT_FAILED',
                            'error': str(error),
                        }
                    metadata['live_parameter_snapshot'] = live_report
                    atomic_json(attempt / 'runner_metadata.json', metadata)
            if (ready and status.get('settled')
                    and not holding_open):
                requested_shutdown = True
                shutdown_reason = 'mission_complete'
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
                shutdown_reason = 'startup_timeout'
                break
            sim_expired = (
                args.time_mode == 'sim' and args.mission_timeout is not None
                and mission_sim_start is not None
                and status.get('elapsed_s', mission_sim_start)
                - mission_sim_start >= args.mission_timeout)
            if (mission_timeout_expired(now, mission_deadline, holding_open)
                    or (sim_expired and not holding_open)):
                timed_out = True
                requested_shutdown = True
                shutdown_reason = 'simulated_mission_timeout'
                break
            if launch.poll() is not None:
                unexpected_exit = not requested_shutdown
                shutdown_reason = 'launch_process_exit'
                break
            if collector.poll() is not None and not holding_open:
                unexpected_exit = True
                shutdown_reason = 'collector_process_exit'
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
            if (args.emergency_wall_runtime is not None
                    and now - start >= args.emergency_wall_runtime):
                timed_out = True
                requested_shutdown = True
                shutdown_reason = 'emergency_wall_timeout'
                break
            time.sleep(0.2)
    finally:
        append_shutdown_event(
            shutdown_events, 'shutdown_requested',
            shutdown_reason=shutdown_reason or (
                'manual_stop' if interrupted else 'cleanup'))
        # Freeze the newest settled messages while their freshness is intact.
        if not collector_finalized_for_hold:
            append_shutdown_event(
                shutdown_events, 'signal_sent', process='collector',
                signal='SIGINT')
            _send_scope([collector], signal.SIGINT,
                        process_group=collector.pid)
            wait_processes([collector], args.graceful_shutdown_timeout,
                            process_group=collector.pid)
            append_shutdown_event(
                shutdown_events, 'final_snapshot_complete', process='collector',
                complete=collector.poll() is not None)
        if shutdown_reason == 'mission_complete' and not timed_out:
            committed = persist_post_completion_mission_result(attempt)
            append_shutdown_event(
                shutdown_events, 'mission_result_persisted',
                committed=committed,
                phase='before_launch_teardown')
        if rviz is not None:
            append_shutdown_event(
                shutdown_events, 'signal_sent', process='rviz', signal='SIGINT')
            signal_process(rviz, signal.SIGINT)
        for process in diagnostic_processes:
            append_shutdown_event(
                shutdown_events, 'signal_sent', process=f'diagnostic_{process.pid}',
                signal='SIGINT')
            signal_process(process, signal.SIGINT)
        if diagnostic_processes:
            wait_processes(diagnostic_processes, args.graceful_shutdown_timeout)
            for process in diagnostic_processes:
                if process.poll() is None:
                    scoped_shutdown(
                        [process], args.graceful_shutdown_timeout,
                        args.hard_shutdown_timeout, process_group=process.pid,
                        send_initial_sigint=False)
        # Then let the launch's passive observer finalize its own outputs.
        append_shutdown_event(
            shutdown_events, 'signal_sent', process='launch', signal='SIGINT')
        signal_process(launch, signal.SIGINT)
        cleanup = scoped_shutdown(
            processes, args.graceful_shutdown_timeout,
            args.hard_shutdown_timeout, process_group=launch.pid,
            send_initial_sigint=False)
        append_shutdown_event(
            shutdown_events, 'escalation', graceful=cleanup['graceful'],
            terminate_succeeded=cleanup['terminate_succeeded'],
            kill_required=cleanup['kill_required'])
        for name, process in (
                ('collector', collector), ('launch', launch), ('rviz', rviz)):
            if process is not None:
                append_shutdown_event(
                    shutdown_events, 'process_exit', process=name,
                    exit_code=process.poll())
        append_shutdown_event(
            shutdown_events, 'cleanup_complete',
            complete=cleanup.get('all_exited', False))
        metrics_file.close()
        launch_log.close()
        collector_log.close()
        if rviz_log is not None:
            rviz_log.close()
        for log in diagnostic_logs:
            log.close()
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
        settled_observed=requested_shutdown and not timed_out,
        emergency_wall_timeout=shutdown_reason == 'emergency_wall_timeout',
        process_exit_codes={
            'launch': launch.poll(), 'collector': collector.poll(),
            'rviz': rviz.poll() if rviz is not None else None,
        })
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
        'shutdown_reason': shutdown_reason,
        'shutdown_events_path': 'shutdown_events.jsonl',
    })
    atomic_json(attempt / 'runner_metadata.json', metadata)
    return 0 if classification in NON_FAILURE_CLASSIFICATIONS else 1


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
    ros_domain_id, webots_port = resolved_trial_resources(args, trial_number)
    return argparse.Namespace(
        internal_trial=True,
        attempt_dir=str(campaign / 'attempts' / attempt_id),
        trial_id=trial_id,
        attempt_id=attempt_id,
        run_id=attempt_id,
        workspace=getattr(args, 'workspace', '/home/arash/webots_ws'),
        world_profile=getattr(args, 'world_profile', 'small'),
        source_world_path=getattr(args, 'source_world_path', ''),
        ros_domain_id=ros_domain_id,
        webots_port=webots_port,
        webots_mode='fast' if args.fast_mode else 'realtime',
        webots_gui=args.rendering,
        launch_rviz=args.rviz,
        sensor_profile=(
            getattr(args, 'sensor_profile', None) or
            ('throughput' if getattr(args, 'execution_profile', None) ==
             'throughput' else 'full')),
        use_scan_matching=getattr(args, 'use_scan_matching', False),
        do_loop_closing=getattr(args, 'do_loop_closing', False),
        unknown_initial_pose=getattr(args, 'unknown_initial_pose', False),
        ideal_encoder_sensing=getattr(args, 'ideal_encoder_sensing', True),
        diagnostic_mode=getattr(args, 'diagnostic_mode', False),
        enable_rosout_collection=getattr(args, 'enable_rosout_collection', True),
        enable_coverage_attribution=getattr(args, 'enable_coverage_attribution', True),
        enable_trajectory_overlap=getattr(args, 'enable_trajectory_overlap', True),
        enable_forensic_capture=getattr(args, 'enable_forensic_capture', False),
        enable_high_rate_forensic_diagnostics=getattr(
            args, 'enable_high_rate_forensic_diagnostics', True),
        enable_contact_capture=getattr(args, 'enable_contact_capture', False),
        contact_sampling_period_ms=getattr(
            args, 'contact_sampling_period_ms', 20),
        controller_variant=getattr(args, 'controller_variant', 'rpp'),
        forensic_snapshot_interval_s=getattr(
            args, 'forensic_snapshot_interval_s', 15.0),
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
    hold_announced = False
    rviz_announced = False
    while True:
        try:
            return process.wait(timeout=0.2)
        except subprocess.TimeoutExpired:
            wants_rviz_status = (
                getattr(namespace, 'launch_rviz', False)
                and not rviz_announced)
            wants_hold_status = (
                getattr(namespace, 'hold_open_after_completion', False)
                and not hold_announced)
            if wants_rviz_status or wants_hold_status:
                try:
                    metadata = json.loads(
                        (attempt / 'runner_metadata.json').read_text(
                            encoding='utf-8'))
                except (OSError, ValueError):
                    metadata = {}
                if wants_rviz_status and metadata.get('rviz_pid'):
                    print(
                        f'RVIZ_STARTED pid={metadata["rviz_pid"]}',
                        flush=True,
                    )
                    rviz_announced = True
                if wants_hold_status and metadata.get('hold_open_active'):
                    print(
                        'Mission complete. Webots and RViz are being kept '
                        'open for inspection.\n'
                        'Press Ctrl+C to shut down and finalize the campaign.',
                        flush=True,
                    )
                    hold_announced = True
        except KeyboardInterrupt:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                # Do not mask the user's Ctrl+C with a second traceback when
                # a Webots/ROS child takes longer than the graceful window.
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait(timeout=5)
            return process.returncode


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
    if 'classification' not in metadata:
        metadata.update({
            'classification': 'INFRASTRUCTURE_FAILURE',
            'classification_details': {
                'reason': (
                    'trial supervisor exited before final classification'),
                'supervisor_return_code': returncode,
                'supervisor_log': str(attempt / 'supervisor.log'),
            },
            'wall_time_s': metadata.get('wall_time_s', 0.0),
            'cleanup_complete': False,
        })
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
    retry_allowed = (
        retry and not args.hold_open_after_completion
        and not getattr(args, 'no_infrastructure_retry', False))
    if (metadata['classification'] == 'INFRASTRUCTURE_FAILURE'
            and retry_allowed):
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
        if args.hold_open_after_completion:
            raise RuntimeError(
                f'{namespace.attempt_id} failed before readiness; manual '
                'hold-open attempts are not retried automatically')
            raise RuntimeError(
                f'{namespace.attempt_id} infrastructure readiness failed '
                f'without retry: {metadata.get("clock_error") or "unspecified"}')
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
                    perform_attempt, args, campaign, progress, number,
                    not args.no_infrastructure_retry)
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
            'src/my_epuck_project/test/test_webots_robot_windows.py',
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
        'execution_profile': args.execution_profile,
        'rendering_mode': 'enabled' if args.rendering else 'disabled',
        'rviz': args.rviz,
        'sensor_profile': args.sensor_profile,
        'logging_mode': {
            'observer': True,
            'console_status': args.logger_console_status,
        },
        'fast_mode': args.fast_mode,
        'exact_webots_options': (
            f'--port=<trial-port> --batch '
            f'--mode={"fast" if args.fast_mode else "realtime"}'
            + ('' if args.rendering
               else ' --no-rendering --stdout --stderr --minimize')),
        'simulation_time_measurement': 'collector /clock telemetry',
        'time_mode': args.time_mode,
        'use_sim_time': args.time_mode == 'sim',
        'world_profile': args.world_profile,
        'world': args.profile_metadata['world'],
        'source_world_path': args.source_world_path,
        'installed_world_path': args.installed_world_path,
        'world_dimensions_m':
            args.profile_metadata['world_dimensions_m'],
        'world_sha256': args.profile_metadata['world_sha256'],
        'installed_world_sha256': args.profile_metadata.get('installed_world_sha256'),
        'robot_start_poses': args.profile_metadata['robot_start_poses'],
        'known_initial_relative_transform':
            args.profile_metadata['known_relative_transform'],
        'transform_source': 'WORLD_DERIVED',
        'initial_separation_m': args.profile_metadata['initial_separation_m'],
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
            'two_robots_decentralized_exploration_launch.py',
        'launch_arguments': {
            'world_profile': args.world_profile,
            'use_sim_time': args.time_mode == 'sim',
            'use_scan_matching': args.use_scan_matching,
            'do_loop_closing': args.do_loop_closing,
            'unknown_initial_pose': args.unknown_initial_pose,
            'ideal_encoder_sensing': args.ideal_encoder_sensing,
            'assignment_mode': 'replicated_two_robot_pair',
            'dispatch_enabled': True,
            'launch_rviz': args.rviz,
            'hold_open_after_completion':
                args.hold_open_after_completion,
            'time_mode': args.time_mode,
            'no_mission_timeout': args.no_mission_timeout,
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
            'mission_timeout_domain': args.time_mode,
            'emergency_wall_runtime_s': args.emergency_wall_runtime,
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
            'Process launch, startup, clock-stall, and emergency limits use wall time; '
            'mission timers use ROS simulation time when time_mode=sim.',
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
                args, campaign, progress, number,
                retry=not args.no_infrastructure_retry)
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
        and summary['mission_completion_rate'] == 1.0
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
    if args.mission_timeout is None and not args.no_mission_timeout:
        args.mission_timeout = settings['mission_timeout']
    if args.shift_window is None:
        args.shift_window = settings['map_comparison_shift_window']
    if settings.get('slam_runtime_parameters', {}).get('use_scan_matching'):
        args.use_scan_matching = True
        args.do_loop_closing = False
    if settings.get('unknown_initial_pose'):
        args.unknown_initial_pose = True
    return args


def apply_execution_profile(args):
    """Resolve rendering/RViz defaults while retaining explicit overrides."""
    name = args.execution_profile or 'headless'
    settings = {
        'visual': (True, True),
        'rendered': (True, False),
        'headless': (False, False),
        # Full-sensor diagnostic run with Webots rendering disabled but the
        # passive RViz2 view visible for human inspection.
        'rviz': (False, True),
        # Retained for controlled experiments only.  It is not the valid
        # benchmark recommendation because it changes the sensor profile.
        'throughput': (False, False),
    }
    profile_rendering, profile_rviz = settings[name]
    explicit_rendering = args.rendering is not None
    explicit_rviz = args.rviz is not None
    if explicit_rendering and args.rendering != profile_rendering:
        print(
            f'EXECUTION_PROFILE_CONFLICT profile={name} '
            f'rendering_override={args.rendering}', flush=True)
    if explicit_rviz and args.rviz != profile_rviz:
        print(
            f'EXECUTION_PROFILE_CONFLICT profile={name} '
            f'rviz_override={args.rviz}', flush=True)
    args.rendering = args.rendering if explicit_rendering else profile_rendering
    args.rviz = args.rviz if explicit_rviz else profile_rviz
    if getattr(args, 'sensor_profile', None) is None:
        args.sensor_profile = (
            'throughput' if name == 'throughput' else 'full')
    elif name == 'headless' and args.sensor_profile != 'full':
        print(
            'EXECUTION_PROFILE_CONFLICT profile=headless '
            f'sensor_profile_override={args.sensor_profile}; '
            'full sensors are the validated headless configuration',
            flush=True)
    args.execution_profile = name
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
        choices=['large', 'small', 'large_unknown_pose',
                 'large_unknown_pose_16m', 'large_unknown_pose_close_start',
                 'large_unknown_pose_close_start_20ms',
                 'large_unknown_pose_close_start_20ms_scan_matching'],
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
    result.add_argument('--no-mission-timeout', action='store_true',
                        help='Disable the simulated/wall mission timeout.')
    result.add_argument('--time-mode', choices=['sim', 'wall'], default='sim',
                        help='ROS mission clock: Webots simulation or wall time.')
    result.add_argument(
        '--use-scan-matching', type=boolean, default=False, metavar='BOOL',
        help=('Diagnostic Slam Toolbox override. Production YAML default is '
              'false; this does not alter the YAML.'))
    result.add_argument(
        '--do-loop-closing', type=boolean, default=False, metavar='BOOL',
        help=('Diagnostic Slam Toolbox override. Production YAML default is '
              'false; this does not alter the YAML.'))
    result.add_argument(
        '--unknown-initial-pose', type=boolean, default=False, metavar='BOOL',
        help='Run the decentralized unknown-relative-pose phase contract.')
    result.add_argument(
        '--ideal-encoder-sensing', type=boolean, default=True, metavar='BOOL',
        help=('Use the thesis simulation assumption of zero explicitly '
              'injected encoder/odometry noise. Webots wheel PositionSensor '
              'measurements still drive normal differential-drive odometry; '
              'Supervisor pose is never used as /odom.'))
    result.add_argument('--emergency-wall-runtime', type=float,
                        help='Optional wall-clock safety limit for unattended runs.')
    result.add_argument('--settling-period', type=float, default=4.0)
    result.add_argument(
        '--fast-mode', type=boolean, default=True, metavar='BOOL')
    result.add_argument(
        '--logger-console-status', type=boolean, default=False,
        metavar='BOOL', help='Enable periodic logger console status output.')
    result.add_argument(
        '--enable-rosout-collection', type=boolean, default=True,
        metavar='BOOL', help='Diagnostic observer: subscribe to /rosout.')
    result.add_argument(
        '--enable-coverage-attribution', type=boolean, default=True,
        metavar='BOOL', help='Diagnostic observer: retain transformed coverage cells.')
    result.add_argument(
        '--enable-trajectory-overlap', type=boolean, default=True,
        metavar='BOOL', help='Diagnostic observer: retain trajectory bins.')
    result.add_argument(
        '--rendering', type=boolean, default=None, metavar='BOOL')
    result.add_argument(
        '--execution-profile',
        choices=['visual', 'rendered', 'headless', 'rviz', 'throughput'], default=None,
        help=('Reusable execution configuration. headless is the validated '
              'full-sensor mode; rviz disables Webots rendering while opening '
              'passive RViz2; throughput is experimental/rejected.'))
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
        '--rviz', type=boolean, default=None, metavar='BOOL',
        help='Open the passive RViz view (single-trial use only)')
    result.add_argument(
        '--calibration', type=boolean, default=True, metavar='BOOL')
    result.add_argument(
        '--isolation-pilot', type=boolean, default=True, metavar='BOOL')
    result.add_argument('--analysis-only', action='store_true')
    result.add_argument('--regenerate-report', action='store_true')
    result.add_argument('--skip-build', action='store_true')
    result.add_argument('--skip-tests', action='store_true')
    result.add_argument(
        '--no-infrastructure-retry', action='store_true',
        help='Run exactly one attempt and preserve its first readiness failure.')
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
    result.add_argument(
        '--sensor-profile', choices=['full', 'throughput'], default=None,
        help='Simulated-device set; physical launches are unchanged.')
    result.add_argument(
        '--diagnostic-mode', type=boolean, default=False, metavar='BOOL',
        help='Enable bounded Nav2 command-pipeline diagnostics.')
    result.add_argument(
        '--enable-forensic-capture', type=boolean, default=False, metavar='BOOL',
        help='Enable passive Supervisor/map forensic capture in the trial.')
    result.add_argument(
        '--enable-high-rate-forensic-diagnostics', type=boolean, default=True,
        metavar='BOOL',
        help=('Enable the optional high-rate command/wheel/odom logger. '
              'Forensic Supervisor and map capture are independent.'))
    result.add_argument(
        '--enable-contact-capture', type=boolean, default=False, metavar='BOOL',
        help='Enable passive Webots contact-point capture in the trial.')
    result.add_argument(
        '--contact-sampling-period-ms', type=int, default=20,
        help='Sampling period for optional passive Webots contact capture.')
    result.add_argument(
        '--controller-variant',
        choices=['dwb', 'rotation_shim_dwb', 'rpp'], default='rpp',
        help='Optional controller variant; production default is frozen RPP. '
             'Use dwb or rotation_shim_dwb only for explicit diagnostics.')
    result.add_argument(
        '--forensic-snapshot-interval-s', type=float, default=15.0,
        help='Interval for bounded passive forensic map snapshots.')
    return result


def validate_cli_options(args):
    """Validate campaign-level manual GUI constraints."""
    if args.trials < 1:
        raise SystemExit('--trials must be positive')
    if not 1 <= args.maximum_concurrency <= 4:
        raise SystemExit('--maximum-concurrency must be between 1 and 4')
    if args.no_mission_timeout:
        args.mission_timeout = None
    if args.emergency_wall_runtime is not None and args.emergency_wall_runtime <= 0:
        raise SystemExit('--emergency-wall-runtime must be positive')
    validate_resource_bounds(args)
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
    apply_execution_profile(args)
    validate_cli_options(args)
    try:
        if args.internal_trial:
            apply_profile_defaults(args, args.world_profile or 'small')
            if args.source_world_path is None:
                args.source_world_path = ''
            # The campaign parent resolves the selected source/installed
            # worlds before launching a child.  Internal children are also
            # directly invokable, so resolve the same metadata here instead
            # of assuming the parent Namespace was serialized into them.
            resolve_runner_profile(args)
            return internal_trial(args)
        return campaign_main(args)
    except (RuntimeError, ValueError, OSError) as error:
        print(f'INFRASTRUCTURE_ERROR: {error}', file=sys.stderr)
        return 3


if __name__ == '__main__':
    raise SystemExit(main())
