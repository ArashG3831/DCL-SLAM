"""Read-only Webots Supervisor ground truth for motion characterization.

This process is an external diagnostic observer. It never commands the robot,
changes simulation state, or publishes ROS data. The resulting CSV is joined
offline with ROS command, wheel, and odometry samples by simulation time.
"""

import argparse
import csv
import math
import os
import signal
import sys


def _controller_import():
    """Import the Webots API after the launch sets WEBOTS_HOME."""
    try:
        from controller import Supervisor
    except Exception as exc:  # pragma: no cover - exercised by launch smoke test
        raise RuntimeError(
            'Webots Supervisor API could not be imported; WEBOTS_HOME must point '
            'to the installed webots_ros2_driver prefix') from exc
    return Supervisor


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--robot-def', default='ROBOT1')
    parser.add_argument('--max-sim-time', type=float, default=25.0)
    args = parser.parse_args(argv)

    Supervisor = _controller_import()
    supervisor = Supervisor()
    robot = supervisor.getFromDef(args.robot_def)
    if robot is None:
        print(f'SUPERVISOR_ROBOT_NOT_FOUND def={args.robot_def}', file=sys.stderr)
        return 2

    output = os.path.abspath(args.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
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
        'angular_velocity_z_rps',
    ]
    with open(output, 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        stream.flush()
        timestep = max(1, int(supervisor.getBasicTimeStep()))
        while not stop and supervisor.step(timestep) != -1:
            position = robot.getPosition()
            orientation = robot.getOrientation()
            velocity = robot.getVelocity()
            # Webots returns a row-major rotation matrix. In this project's
            # world the arena is the x-y plane and z is vertical. The first
            # column is local +x in world coordinates.
            heading_x = orientation[0]
            heading_y = orientation[3]
            heading_z = orientation[6]
            writer.writerow({
                'sim_time_s': supervisor.getTime(),
                'world_x_m': position[0],
                'world_y_m': position[1],
                'world_z_m': position[2],
                'heading_x': heading_x,
                'heading_y': heading_y,
                'heading_z': heading_z,
                'planar_yaw_rad': math.atan2(heading_y, heading_x),
                'linear_velocity_x_mps': velocity[0],
                'linear_velocity_y_mps': velocity[1],
                'linear_velocity_z_mps': velocity[2],
                'ground_speed_mps': math.hypot(velocity[0], velocity[1]),
                'angular_velocity_x_rps': velocity[3],
                'angular_velocity_y_rps': velocity[4],
                'angular_velocity_z_rps': velocity[5],
            })
            stream.flush()
            if supervisor.getTime() >= args.max_sim_time:
                break
    print(f'SUPERVISOR_GROUND_TRUTH_COMPLETE path={output}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
