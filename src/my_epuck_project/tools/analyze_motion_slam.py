#!/usr/bin/env python3
"""Join diagnostic SLAM trajectory/map telemetry with Supervisor truth."""

import argparse
import csv
import math
import pathlib
import statistics


def f(row, key):
    try:
        value = row.get(key, '')
        return None if value in ('', None) else float(value)
    except (TypeError, ValueError):
        return None


def nearest(rows, stamp):
    return min(rows, key=lambda row: abs(f(row, 'sim_time_s') - stamp))


def wrap(value):
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def analyze(directory):
    with next(path for path in directory.glob('*.csv')
              if path.name not in ('supervisor_ground_truth.csv', 'slam_trajectory.csv',
                                   'slam_map_updates.csv') and
              path.name.startswith(('speed_', 'accel_'))).open(
                                       newline='', encoding='utf-8') as stream:
        motion = list(csv.DictReader(stream))
    profile_start = min(
        f(row, 'ros_time_s') - f(row, 'sim_elapsed_s') for row in motion
        if f(row, 'ros_time_s') is not None and f(row, 'sim_elapsed_s') is not None)
    with (directory / 'slam_trajectory.csv').open(newline='', encoding='utf-8') as stream:
        pose_rows = [row for row in csv.DictReader(stream) if f(row, 'lookup_ok') == 1]
    with (directory / 'supervisor_ground_truth.csv').open(newline='', encoding='utf-8') as stream:
        truth = list(csv.DictReader(stream))
    with (directory / 'slam_map_updates.csv').open(newline='', encoding='utf-8') as stream:
        maps = list(csv.DictReader(stream))
    if not pose_rows or not truth:
        return {'case': directory.name, 'valid_pose_samples': 0,
                'map_updates': len(maps)}
    first_pose = pose_rows[0]
    # The recorder normally uses sim time. Older captures made before the
    # parameter declaration used a system-clock epoch; normalize either form
    # to a relative motion timeline before joining Supervisor truth.
    pose_epoch = f(first_pose, 'sim_time_s')
    if pose_epoch > 1.0e6:
        pose_time = lambda row: profile_start + (f(row, 'sim_time_s') - pose_epoch)
    else:
        pose_time = lambda row: f(row, 'sim_time_s') - profile_start
    first_truth = nearest(truth, pose_time(first_pose))
    errors = []
    yaw_errors = []
    for pose in pose_rows:
        gt = nearest(truth, pose_time(pose))
        dx = (f(pose, 'x_m') - f(first_pose, 'x_m')) - (f(gt, 'world_x_m') - f(first_truth, 'world_x_m'))
        dy = (f(pose, 'y_m') - f(first_pose, 'y_m')) - (f(gt, 'world_y_m') - f(first_truth, 'world_y_m'))
        errors.append(math.hypot(dx, dy))
        gt_yaw = f(gt, 'planar_yaw_rad') - f(first_truth, 'planar_yaw_rad')
        yaw_errors.append(abs(wrap((f(pose, 'yaw_rad') - f(first_pose, 'yaw_rad')) - gt_yaw)))
    last = pose_rows[-1]
    last_gt = nearest(truth, pose_time(last))
    final_dx = (f(last, 'x_m') - f(first_pose, 'x_m')) - (f(last_gt, 'world_x_m') - f(first_truth, 'world_x_m'))
    final_dy = (f(last, 'y_m') - f(first_pose, 'y_m')) - (f(last_gt, 'world_y_m') - f(first_truth, 'world_y_m'))
    result = {
        'case': directory.name,
        'valid_pose_samples': len(pose_rows),
        'map_updates': len(maps),
        'slam_translation_rmse_m': math.sqrt(statistics.mean(x * x for x in errors)),
        'slam_translation_final_error_m': math.hypot(final_dx, final_dy),
        'slam_yaw_rmse_rad': math.sqrt(statistics.mean(x * x for x in yaw_errors)),
        'slam_yaw_final_error_rad': yaw_errors[-1],
    }
    if maps:
        last_map = maps[-1]
        for key in ('width', 'height', 'resolution_m', 'known_cells', 'free_cells',
                    'occupied_cells', 'unknown_cells'):
            result[f'final_map_{key}'] = last_map.get(key, '')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=pathlib.Path)
    args = parser.parse_args(argv)
    rows = [analyze(path) for path in sorted(args.root.iterdir())
            if path.is_dir() and (path / 'slam_trajectory.csv').exists()]
    if not rows:
        raise SystemExit('no SLAM cases found')
    fields = list(rows[0])
    with (args.root / 'slam_speed_comparison.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(f'cases={len(rows)} output={args.root / "slam_speed_comparison.csv"}')


if __name__ == '__main__':
    main()
