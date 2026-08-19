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
    sample_steps = max(1, int(round(
        max(args.sample_period_s, timestep / 1000.0) /
        (timestep / 1000.0))))
    row_number = 0
    observer_start_wall = time.monotonic()
    with open(output, 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        stream.flush()
        while (not stop['value']
               and (args.max_runtime_s <= 0.0
                    or time.monotonic() - observer_start_wall
                    < args.max_runtime_s)
               and supervisor.step(timestep) != -1):
            row_number += 1
            if row_number % sample_steps:
                continue
            now = supervisor.getTime()
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
            stream.flush()
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
                contact_stream.flush()
    if contact_stream is not None:
        contact_stream.flush()
        contact_stream.close()
    print(f'FORENSIC_SUPERVISOR_COMPLETE path={output}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
