"""External read-only Webots Supervisor trajectory recorder.

The process is deliberately outside ROS.  It reads only the world pose and
velocity of the two named e-pucks for offline forensic comparison.  It never
publishes, commands, edits the scene, or participates in estimation/control.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import sys
import time


def _effective_step_period_ms(
        basic_time_step_ms, sample_period_s, contact_sampling_period_ms=0):
    """Return a Webots step period aligned to the world's basic timestep.

    The Supervisor is a read-only forensic recorder.  It does not need to
    call ``step`` at every physics tick when its requested output rate is
    lower.  Batching those ticks is particularly important for 4 ms worlds:
    the external-controller IPC call itself is expensive even when the
    resulting pose is discarded.  Contact capture remains at its requested
    rate because contact events can be shorter-lived than pose samples.
    """
    basic = max(1, int(basic_time_step_ms))
    requested = max(basic, int(math.ceil(max(0.0, float(sample_period_s))
                                      * 1000.0)))
    contact_period = int(contact_sampling_period_ms)
    if contact_period > 0:
        requested = min(requested, max(basic, contact_period))
    return max(basic, int(math.ceil(requested / basic)) * basic)


def _write_runtime_metrics(path, metrics):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    temporary = f'{path}.tmp'
    with open(temporary, 'w', encoding='utf-8') as stream:
        json.dump(metrics, stream, indent=2, sort_keys=True)
        stream.write('\n')
    os.replace(temporary, path)


def _find_named_node(supervisor, name):
    requested = str(name).casefold()
    node = supervisor.getFromDef(name)
    if node is not None:
        return node
    root = supervisor.getRoot()
    children = root.getField('children')
    if children is None:
        return None
    pending = [children.getMFNode(index) for index in range(children.getCount())]
    while pending:
        node = pending.pop()
        if node is None:
            continue
        name_field = node.getField('name')
        if name_field is not None:
            try:
                # Webots may expose an externally-controlled robot's runtime
                # name with normalized capitalization even when the WBT
                # ``name`` field is lower-case.  Supervisor lookup is
                # diagnostic-only, so a case-insensitive name match is the
                # least invasive compatibility rule.
                if name_field.getSFString().casefold() == requested:
                    return node
            except RuntimeError:
                pass
        child_field = node.getField('children')
        if child_field is not None:
            pending.extend(
                child_field.getMFNode(index)
                for index in range(child_field.getCount()))
    return None


def _connect(Supervisor, attempts=120):
    last_error = None
    for _ in range(attempts):
        try:
            return Supervisor()
        except Exception as exc:  # Webots may not be listening yet.
            last_error = exc
            time.sleep(0.5)
    raise RuntimeError(f'Webots Supervisor connection failed: {last_error}')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--robot-def', action='append', required=True)
    parser.add_argument('--sample-period-s', type=float, default=0.10)
    parser.add_argument('--ready-file', default='')
    parser.add_argument('--connect-attempts', type=int, default=240)
    parser.add_argument('--contact-output', default='')
    parser.add_argument('--contact-sampling-period-ms', type=int, default=20)
    parser.add_argument(
        '--max-runtime-s', type=float, default=0.0,
        help='Optional wall-time bound for clean observer finalization.')
    parser.add_argument(
        '--controller-url', default='',
        help='Explicit Webots controller URL; avoids legacy /tmp discovery.')
    parser.add_argument(
        '--runtime-directory', default='',
        help='Fresh campaign-owned directory for observer runtime metadata.')
    args = parser.parse_args(argv)
    if args.runtime_directory:
        runtime_directory = os.path.abspath(args.runtime_directory)
        os.makedirs(runtime_directory, mode=0o700, exist_ok=True)
        if not os.access(runtime_directory, os.W_OK | os.X_OK):
            print(
                f'OBSERVER_RUNTIME_DIRECTORY_NOT_WRITABLE {runtime_directory}',
                file=sys.stderr)
            return 2
    if args.controller_url:
        os.environ['WEBOTS_CONTROLLER_URL'] = args.controller_url
    controller_url = os.environ.get('WEBOTS_CONTROLLER_URL', '')
    if '://' not in controller_url:
        print(
            'OBSERVER_CONTROLLER_URL_REQUIRED '
            'WEBOTS_CONTROLLER_URL must be an explicit tcp:// or ipc:// URL',
            file=sys.stderr)
        return 2
    try:
        from controller import Supervisor
    except Exception as exc:
        print(f'SUPERVISOR_IMPORT_FAILED {exc}', file=sys.stderr)
        return 2

    try:
        supervisor = _connect(Supervisor, attempts=max(1, args.connect_attempts))
    except RuntimeError as exc:
        print(f'SUPERVISOR_CONNECT_FAILED {exc}', file=sys.stderr)
        return 2

    robots = {}
    for name in args.robot_def:
        node = _find_named_node(supervisor, name)
        if node is None:
            print(f'SUPERVISOR_ROBOT_NOT_FOUND name={name}', file=sys.stderr)
            return 2
        robots[name] = node

    if args.ready_file:
        ready_path = os.path.abspath(args.ready_file)
        os.makedirs(os.path.dirname(ready_path), exist_ok=True)
        with open(ready_path, 'w', encoding='utf-8') as ready_stream:
            json.dump({
                'sim_time_s': supervisor.getTime(),
                'robot_defs': sorted(robots),
                'basic_time_step_ms': supervisor.getBasicTimeStep(),
            }, ready_stream, sort_keys=True)
        print(f'SUPERVISOR_READY path={ready_path}', flush=True)

    timestep = max(1, int(supervisor.getBasicTimeStep()))
    effective_step_period_ms = _effective_step_period_ms(
        timestep, args.sample_period_s,
        args.contact_sampling_period_ms if args.contact_output else 0)
    contact_stream = None
    contact_writer = None
    if args.contact_output:
        contact_path = os.path.abspath(args.contact_output)
        os.makedirs(os.path.dirname(contact_path), exist_ok=True)
        contact_stream = open(contact_path, 'w',
                              newline='', encoding='utf-8')
        contact_writer = csv.DictWriter(contact_stream, fieldnames=[
            'sim_time_s', 'robot_id', 'contact_count', 'point_x_m',
            'point_y_m', 'point_z_m', 'contacted_node_id',
            'contacted_node_def', 'contacted_node_name',
        ])
        contact_writer.writeheader()
        contact_stream.flush()
        period_ms = max(timestep, int(args.contact_sampling_period_ms))
        for node in robots.values():
            node.enableContactPointsTracking(period_ms, True)

    output = os.path.abspath(args.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    fields = [
        'sim_time_s', 'robot_id', 'world_x_m', 'world_y_m', 'world_z_m',
        'heading_x', 'heading_y', 'heading_z', 'planar_yaw_rad',
        'linear_velocity_x_mps', 'linear_velocity_y_mps',
        'linear_velocity_z_mps', 'ground_speed_mps',
        'angular_velocity_x_rps', 'angular_velocity_y_rps',
        'angular_velocity_z_rps',
    ]
    stop = {'value': False}

    def request_stop(_signum, _frame):
        stop['value'] = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    step_calls = 0
    sample_rows = 0
    rows_since_flush = 0
    observer_start_wall = time.monotonic()
    observer_start_sim = supervisor.getTime()
    last_sim_time = observer_start_sim
    exit_reason = 'completed'
    metrics_path = os.path.join(
        os.path.abspath(args.runtime_directory)
        if args.runtime_directory else os.path.dirname(output),
        'runtime_metrics.json')

    def save_metrics(reason, end_sim, end_wall, finalized=False):
        wall_elapsed = max(0.0, end_wall - observer_start_wall)
        sim_elapsed = max(0.0, end_sim - observer_start_sim)
        _write_runtime_metrics(metrics_path, {
            'observer': 'cooperative_ground_truth_observer',
            'basic_time_step_ms': timestep,
            'requested_sample_period_s': float(args.sample_period_s),
            'contact_sampling_period_ms': (
                int(args.contact_sampling_period_ms)
                if args.contact_output else None),
            'effective_step_period_ms': effective_step_period_ms,
            'step_calls': step_calls,
            'sample_rows': sample_rows,
            'sim_start_s': observer_start_sim,
            'sim_end_s': end_sim,
            'sim_elapsed_s': sim_elapsed,
            'monotonic_wall_start_s': observer_start_wall,
            'monotonic_wall_end_s': end_wall,
            'monotonic_wall_elapsed_s': wall_elapsed,
            'simulation_seconds_per_wall_second': (
                sim_elapsed / wall_elapsed if wall_elapsed > 0.0 else None),
            'exit_reason': reason,
            'finalized': finalized,
            'output': output,
        })

    with open(output, 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        stream.flush()
        while not stop['value']:
            if (args.max_runtime_s > 0.0
                    and time.monotonic() - observer_start_wall
                    >= args.max_runtime_s):
                exit_reason = 'max_runtime'
                break
            step_result = supervisor.step(effective_step_period_ms)
            step_calls += 1
            if step_result == -1:
                exit_reason = 'webots_step_end'
                break
            now = supervisor.getTime()
            last_sim_time = now
            rows = []
            for robot_id, node in robots.items():
                position = node.getPosition()
                orientation = node.getOrientation()
                velocity = node.getVelocity()
                heading_x, heading_y, heading_z = (
                    orientation[0], orientation[3], orientation[6])
                rows.append({
                    'sim_time_s': now, 'robot_id': robot_id,
                    'world_x_m': position[0], 'world_y_m': position[1],
                    'world_z_m': position[2], 'heading_x': heading_x,
                    'heading_y': heading_y, 'heading_z': heading_z,
                    'planar_yaw_rad': math.atan2(heading_y, heading_x),
                    'linear_velocity_x_mps': velocity[0],
                    'linear_velocity_y_mps': velocity[1],
                    'linear_velocity_z_mps': velocity[2],
                    'ground_speed_mps': math.hypot(velocity[0], velocity[1]),
                    'angular_velocity_x_rps': velocity[3],
                    'angular_velocity_y_rps': velocity[4],
                    'angular_velocity_z_rps': velocity[5],
                })
            writer.writerows(rows)
            sample_rows += 1
            rows_since_flush += 1
            if rows_since_flush >= 10:
                stream.flush()
                rows_since_flush = 0
                save_metrics('running', now, time.monotonic())
            if contact_writer is not None:
                for robot_id, node in robots.items():
                    try:
                        contacts = node.getContactPoints(True)
                    except Exception:
                        contacts = []
                    if not contacts:
                        contact_writer.writerow({
                            'sim_time_s': now, 'robot_id': robot_id,
                            'contact_count': 0,
                            'point_x_m': '', 'point_y_m': '',
                            'point_z_m': '', 'contacted_node_id': '',
                            'contacted_node_def': '',
                            'contacted_node_name': '',
                        })
                        continue
                    for contact in contacts:
                        contacted = None
                        try:
                            contacted = supervisor.getFromId(
                                contact.getNodeId())
                        except Exception:
                            pass
                        contacted_def = ''
                        contacted_name = ''
                        if contacted is not None:
                            try:
                                contacted_def = contacted.getDef() or ''
                            except Exception:
                                pass
                            try:
                                name_field = contacted.getField('name')
                                if name_field is not None:
                                    contacted_name = name_field.getSFString()
                            except Exception:
                                pass
                        point = contact.getPoint()
                        contact_writer.writerow({
                            'sim_time_s': now, 'robot_id': robot_id,
                            'contact_count': len(contacts),
                            'point_x_m': point[0], 'point_y_m': point[1],
                            'point_z_m': point[2],
                            'contacted_node_id': contact.getNodeId(),
                            'contacted_node_def': contacted_def,
                            'contacted_node_name': contacted_name,
                        })
                if rows_since_flush == 0:
                    contact_stream.flush()
        stream.flush()
    if contact_stream is not None:
        contact_stream.flush()
        contact_stream.close()
    observer_end_wall = time.monotonic()
    # Do not query a Supervisor connection after Webots has returned -1 from
    # step(): the controller binding may already have lost its socket.
    observer_end_sim = last_sim_time
    save_metrics(exit_reason, observer_end_sim, observer_end_wall, True)
    print(f'FORENSIC_RUNTIME_METRICS path={metrics_path}', flush=True)
    print(f'FORENSIC_SUPERVISOR_COMPLETE path={output}', flush=True)
    # The Webots Python controller binding has been observed to segfault in
    # its interpreter-exit destructor after a remote Webots shutdown, even
    # after all observer files are closed.  This process is a read-only
    # diagnostic child; bypass only that binding destructor after the durable
    # metrics and CSV finalization above, so teardown is reported as clean.
    os._exit(0)


if __name__ == '__main__':
    raise SystemExit(main())
