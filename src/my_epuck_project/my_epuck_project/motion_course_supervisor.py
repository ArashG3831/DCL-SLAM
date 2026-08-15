"""External Webots Supervisor and distance-locked course gate.

The observer records world ground truth and writes a small file-based gate for
the ROS diagnostic driver.  It never publishes ROS data, commands the robot,
or modifies the Webots scene.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import sys
import tempfile

from .motion_course_spec import course_for_name


def _wrap(value):
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def _atomic_json(path, payload):
    directory = os.path.dirname(os.path.abspath(path))
    fd, temporary = tempfile.mkstemp(prefix='.course_gate_', dir=directory)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            json.dump(payload, stream, sort_keys=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _read_json(path):
    try:
        with open(path, encoding='utf-8') as stream:
            return json.load(stream)
    except (FileNotFoundError, OSError, ValueError, TypeError):
        return None


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--command-file', required=True)
    parser.add_argument('--gate-file', required=True)
    parser.add_argument('--robot-def', default='ROBOT1')
    parser.add_argument('--max-sim-time', type=float, default=140.0)
    parser.add_argument('--course-name', default='short')
    args = parser.parse_args(argv)
    course = course_for_name(args.course_name)

    try:
        from controller import Supervisor
    except Exception as exc:  # pragma: no cover - Webots-only process
        print(f'WEBOTS_SUPERVISOR_IMPORT_FAILED {exc}', file=sys.stderr)
        return 2

    supervisor = Supervisor()
    robot = supervisor.getFromDef(args.robot_def)
    if robot is None:
        print(f'SUPERVISOR_ROBOT_NOT_FOUND def={args.robot_def}', file=sys.stderr)
        return 2

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    stop = False

    def _stop(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    fields = [
        'sim_time_s', 'world_x_m', 'world_y_m', 'world_z_m',
        'heading_x', 'heading_y', 'heading_z', 'planar_yaw_rad',
        'linear_velocity_x_mps', 'linear_velocity_y_mps',
        'linear_velocity_z_mps', 'ground_speed_mps',
        'angular_velocity_x_rps', 'angular_velocity_y_rps',
        'angular_velocity_z_rps', 'requested_segment',
        'segment_progress', 'gate_completed_segment',
    ]
    timestep = max(1, int(supervisor.getBasicTimeStep()))
    active_index = None
    start_x = start_y = start_yaw = None
    gate_completed = -1
    done_time = None

    with open(args.output, 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        stream.flush()
        while not stop and supervisor.step(timestep) != -1:
            now = supervisor.getTime()
            position = robot.getPosition()
            orientation = robot.getOrientation()
            velocity = robot.getVelocity()
            heading_x, heading_y, heading_z = orientation[0], orientation[3], orientation[6]
            yaw = math.atan2(heading_y, heading_x)
            command = _read_json(args.command_file) or {}
            requested = command.get('segment_index')
            if requested != active_index:
                active_index = requested
                start_x, start_y, start_yaw = position[0], position[1], yaw

            segment = (course[active_index]
                       if isinstance(active_index, int) and
                       0 <= active_index < len(course) else None)
            if segment is None:
                progress = 0.0
            elif segment['kind'] == 'straight':
                progress = ((position[0] - start_x) * math.cos(start_yaw) +
                            (position[1] - start_y) * math.sin(start_yaw))
            else:
                signed = _wrap(yaw - start_yaw) * segment['direction']
                progress = signed

            if (segment is not None and active_index is not None and
                    active_index > gate_completed):
                target = (segment.get('distance_m') if segment['kind'] == 'straight'
                          else segment.get('angle_rad'))
                if progress >= target:
                    gate_completed = active_index
                    _atomic_json(args.gate_file, {
                        'completed_segment': gate_completed,
                        'sim_time_s': now,
                        'x_m': position[0], 'y_m': position[1],
                        'yaw_rad': yaw, 'progress': progress,
                    })

            if command.get('done') and done_time is None:
                done_time = now
            if done_time is not None and now - done_time >= 1.0:
                break

            writer.writerow({
                'sim_time_s': now,
                'world_x_m': position[0], 'world_y_m': position[1], 'world_z_m': position[2],
                'heading_x': heading_x, 'heading_y': heading_y, 'heading_z': heading_z,
                'planar_yaw_rad': yaw,
                'linear_velocity_x_mps': velocity[0], 'linear_velocity_y_mps': velocity[1],
                'linear_velocity_z_mps': velocity[2], 'ground_speed_mps': math.hypot(velocity[0], velocity[1]),
                'angular_velocity_x_rps': velocity[3], 'angular_velocity_y_rps': velocity[4],
                'angular_velocity_z_rps': velocity[5],
                'requested_segment': '' if requested is None else requested,
                'segment_progress': progress,
                'gate_completed_segment': gate_completed,
            })
            stream.flush()
            if now >= args.max_sim_time:
                break

    print(f'COURSE_SUPERVISOR_COMPLETE path={args.output} completed={gate_completed}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
