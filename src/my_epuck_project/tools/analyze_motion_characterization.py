#!/usr/bin/env python3
"""Create bounded motion-characterization CSV summaries.

The Supervisor CSV is simulation-only ground truth. It is never consumed by
the runtime controller; this tool joins it offline to ROS command, wheel and
odometry samples using the common simulation-time field.
"""

import argparse
import csv
import math
import pathlib
import statistics


def _f(row, key):
    try:
        value = row.get(key, '')
        return None if value in ('', None) else float(value)
    except (TypeError, ValueError):
        return None


def _nearest(rows, time_s, key='sim_time_s'):
    return min(rows, key=lambda row: abs(_f(row, key) - time_s))


def _quantile(values, q):
    if not values:
        return float('nan')
    if len(values) == 1:
        return values[0]
    return statistics.quantiles(values, n=100, method='inclusive')[max(0, min(99, int(q * 100) - 1))]


def _wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def analyze_case(directory):
    # A SLAM-enabled case also contains trajectory/map telemetry. Select the
    # motion collector by its schema instead of relying on directory order.
    ros_candidates = []
    for path in directory.glob('*.csv'):
        if path.name == 'supervisor_ground_truth.csv' or path.name.endswith('_sweep.csv') or path.name.endswith('_error.csv'):
            continue
        try:
            with path.open(newline='', encoding='utf-8') as stream:
                fields = next(csv.reader(stream), [])
            if 'sim_elapsed_s' in fields and 'wheel_predicted_vx' in fields:
                ros_candidates.append(path)
        except OSError:
            continue
    if not ros_candidates:
        raise ValueError(f'no motion collector CSV found in {directory}')
    ros_path = ros_candidates[0]
    truth_path = directory / 'supervisor_ground_truth.csv'
    with ros_path.open(newline='', encoding='utf-8') as stream:
        ros = list(csv.DictReader(stream))
    with truth_path.open(newline='', encoding='utf-8') as stream:
        truth = list(csv.DictReader(stream))

    ros = [row for row in ros if _f(row, 'sim_elapsed_s') is not None]
    truth = [row for row in truth if _f(row, 'sim_time_s') is not None]
    profile_start = min(
        _f(row, 'ros_time_s') - _f(row, 'sim_elapsed_s')
        for row in ros if _f(row, 'ros_time_s') is not None)
    relative_time = lambda row: _f(row, 'sim_time_s') - profile_start
    straight_ros = [row for row in ros if 1.0 <= _f(row, 'sim_elapsed_s') < 11.0]
    straight_truth = [row for row in truth if 1.0 <= relative_time(row) < 11.0]
    target = max((_f(row, 'cmd_vx') or 0.0) for row in straight_ros)
    steady_ros = [row for row in straight_ros if _f(row, 'sim_elapsed_s') >= 9.0]
    steady_truth = [row for row in straight_truth if _f(row, 'sim_time_s') >= 9.0]

    def mean_key(rows, key):
        values = [v for row in rows if (v := _f(row, key)) is not None]
        return statistics.mean(values) if values else float('nan')

    gt_velocity = []
    for row in straight_truth:
        vx = _f(row, 'linear_velocity_x_mps') or 0.0
        vy = _f(row, 'linear_velocity_y_mps') or 0.0
        hx = _f(row, 'heading_x') or 0.0
        hy = _f(row, 'heading_y') or 0.0
        gt_velocity.append((relative_time(row), vx * hx + vy * hy))
    accelerations = [
        (second - first) / 0.02
        for (_, first), (_, second) in zip(gt_velocity, gt_velocity[1:])
    ]
    positive_accel = [a for a in accelerations if a > 0.0]
    negative_accel = [a for a in accelerations if a < 0.0]
    window_accel = []
    for time_s, value in gt_velocity:
        if not 1.0 <= time_s < 10.9:
            continue
        future = min(gt_velocity, key=lambda sample: abs(sample[0] - (time_s + 0.1)))
        if future[0] > time_s:
            window_accel.append((future[1] - value) / (future[0] - time_s))

    rise_time = float('nan')
    if target > 0.0 and gt_velocity:
        threshold = 0.9 * target
        for time_s, value in gt_velocity:
            if value >= threshold:
                rise_time = time_s - 1.0
                break
    stop_time = float('nan')
    stop_distance = float('nan')
    if target > 0.0:
        stop_row = next((row for row in truth if 11.0 <= relative_time(row) < 15.0 and
                         (_f(row, 'linear_velocity_x_mps') or 0.0) * (_f(row, 'heading_x') or 0.0) +
                         (_f(row, 'linear_velocity_y_mps') or 0.0) * (_f(row, 'heading_y') or 0.0) <= 0.05 * target), None)
        at_brake = _nearest(truth, profile_start + 11.0)
        if stop_row:
            stop_time = relative_time(stop_row) - 11.0
            stop_distance = math.hypot(
                (_f(stop_row, 'world_x_m') or 0.0) - (_f(at_brake, 'world_x_m') or 0.0),
                (_f(stop_row, 'world_y_m') or 0.0) - (_f(at_brake, 'world_y_m') or 0.0))

    first_wheel = next((row for row in straight_ros if _f(row, 'left_position') is not None and
                        _f(row, 'right_position') is not None), None)
    last_wheel = next((row for row in reversed(straight_ros) if _f(row, 'left_position') is not None and
                       _f(row, 'right_position') is not None), None)
    measure_start = _f(first_wheel, 'sim_elapsed_s') if first_wheel else 1.0
    measure_end = _f(last_wheel, 'sim_elapsed_s') if last_wheel else 10.98
    start_truth = _nearest(truth, profile_start + measure_start)
    end_truth = _nearest(truth, profile_start + measure_end)
    dx = (_f(end_truth, 'world_x_m') or 0.0) - (_f(start_truth, 'world_x_m') or 0.0)
    dy = (_f(end_truth, 'world_y_m') or 0.0) - (_f(start_truth, 'world_y_m') or 0.0)
    hx = _f(start_truth, 'heading_x') or 1.0
    hy = _f(start_truth, 'heading_y') or 0.0
    ground_distance = math.hypot(dx, dy)
    lateral_drift = abs(-hy * dx + hx * dy)
    yaw_drift = abs(_wrap((_f(end_truth, 'planar_yaw_rad') or 0.0) -
                          (_f(start_truth, 'planar_yaw_rad') or 0.0)))

    wheel_radius = 0.02
    wheel_distance = float('nan')
    if first_wheel and last_wheel:
        wheel_distance = wheel_radius * 0.5 * (
            (_f(last_wheel, 'left_position') - _f(first_wheel, 'left_position')) +
            (_f(last_wheel, 'right_position') - _f(first_wheel, 'right_position')))

    slip = []
    for row in steady_ros:
        wheel = _f(row, 'wheel_predicted_vx')
        sim_time = _f(row, 'ros_time_s')
        if wheel is None or sim_time is None:
            continue
        gt = _nearest(truth, sim_time)
        vx = _f(gt, 'linear_velocity_x_mps') or 0.0
        vy = _f(gt, 'linear_velocity_y_mps') or 0.0
        fx = _f(gt, 'heading_x') or 0.0
        fy = _f(gt, 'heading_y') or 0.0
        forward = vx * fx + vy * fy
        slip.append((wheel - forward) / max(abs(wheel), 1.0e-3))

    odom_rows = [row for row in straight_ros if _f(row, 'odom_x') is not None and _f(row, 'odom_y') is not None]
    odom_rmse = float('nan')
    odom_final_error = float('nan')
    odom_yaw_error = float('nan')
    if odom_rows:
        first_odom = odom_rows[0]
        first_gt = _nearest(truth, _f(first_odom, 'ros_time_s'))
        errors = []
        for row in odom_rows:
            gt = _nearest(truth, _f(row, 'ros_time_s'))
            ox = (_f(row, 'odom_x') or 0.0) - (_f(first_odom, 'odom_x') or 0.0)
            oy = (_f(row, 'odom_y') or 0.0) - (_f(first_odom, 'odom_y') or 0.0)
            gx = (_f(gt, 'world_x_m') or 0.0) - (_f(first_gt, 'world_x_m') or 0.0)
            gy = (_f(gt, 'world_y_m') or 0.0) - (_f(first_gt, 'world_y_m') or 0.0)
            errors.append(math.hypot(ox - gx, oy - gy))
        odom_rmse = math.sqrt(statistics.mean(e * e for e in errors))
        last_odom = odom_rows[-1]
        last_gt = _nearest(truth, _f(last_odom, 'ros_time_s'))
        ox = (_f(last_odom, 'odom_x') or 0.0) - (_f(first_odom, 'odom_x') or 0.0)
        oy = (_f(last_odom, 'odom_y') or 0.0) - (_f(first_odom, 'odom_y') or 0.0)
        gx = (_f(last_gt, 'world_x_m') or 0.0) - (_f(first_gt, 'world_x_m') or 0.0)
        gy = (_f(last_gt, 'world_y_m') or 0.0) - (_f(first_gt, 'world_y_m') or 0.0)
        odom_final_error = math.hypot(ox - gx, oy - gy)
        odom_yaw_error = abs(_wrap((_f(last_odom, 'odom_yaw') or 0.0) -
                                   ((_f(last_gt, 'planar_yaw_rad') or 0.0) -
                                    (_f(first_gt, 'planar_yaw_rad') or 0.0))))

    curve = [row for row in truth if 15.0 <= relative_time(row) < 18.0]
    rotate = [row for row in truth if 18.0 <= relative_time(row) < 22.0]
    max_ang_curve = max((abs(_f(row, 'angular_velocity_z_rps') or 0.0) for row in curve), default=float('nan'))
    max_ang_rotate = max((abs(_f(row, 'angular_velocity_z_rps') or 0.0) for row in rotate), default=float('nan'))
    scan_end = max((_f(row, 'scan_count') or 0.0 for row in ros), default=0.0)
    scan_start = min((_f(row, 'scan_count') or 0.0 for row in ros), default=0.0)
    sim_start = _f(ros[0], 'ros_time_s') if ros else 0.0
    sim_end = _f(ros[-1], 'ros_time_s') if ros else 0.0

    return {
        'case': directory.name,
        'profile_start_sim_s': profile_start,
        'wheel_measurement_start_sim_elapsed_s': measure_start,
        'target_speed_mps': target,
        'command_steady_mps': mean_key(steady_ros, 'cmd_vx'),
        'final_command_steady_mps': mean_key(steady_ros, 'final_cmd_vx'),
        'ground_speed_steady_mps': mean_key(steady_truth, 'ground_speed_mps'),
        'ground_speed_peak_mps': max((_f(row, 'ground_speed_mps') or 0.0 for row in straight_truth), default=float('nan')),
        'wheel_speed_steady_mps': mean_key(steady_ros, 'wheel_predicted_vx'),
        'odom_speed_steady_mps': mean_key(steady_ros, 'odom_vx'),
        'ground_distance_straight_m': ground_distance,
        'wheel_distance_straight_m': wheel_distance,
        'wheel_ground_distance_error_m': wheel_distance - ground_distance if math.isfinite(wheel_distance) else float('nan'),
        'wheel_ground_distance_error_pct': 100.0 * (wheel_distance - ground_distance) / ground_distance if ground_distance and math.isfinite(wheel_distance) else float('nan'),
        'wheel_ground_slip_ratio_mean': statistics.mean(slip) if slip else float('nan'),
        'wheel_ground_slip_ratio_abs_p95': _quantile([abs(x) for x in slip], 0.95),
        'odom_position_rmse_m': odom_rmse,
        'odom_final_position_error_m': odom_final_error,
        'odom_final_yaw_error_rad': odom_yaw_error,
        'straight_lateral_drift_m': lateral_drift,
        'straight_ground_yaw_drift_rad': yaw_drift,
        'peak_ground_accel_mps2': max(positive_accel, default=float('nan')),
        'peak_ground_decel_mps2': min(negative_accel, default=float('nan')),
        'peak_ground_accel_100ms_mps2': max(window_accel, default=float('nan')),
        'peak_ground_decel_100ms_mps2': min(window_accel, default=float('nan')),
        'ground_accel_p95_mps2': _quantile(positive_accel, 0.95),
        'ground_decel_p05_mps2': _quantile(negative_accel, 0.05),
        'rise_time_to_90pct_s': rise_time,
        'stopping_time_s': stop_time,
        'stopping_distance_m': stop_distance,
        'max_curve_angular_speed_rps': max_ang_curve,
        'max_rotate_angular_speed_rps': max_ang_rotate,
        'scan_rate_hz': (scan_end - scan_start) / max(sim_end - sim_start, 1.0e-6),
        'ros_rows': len(ros),
        'supervisor_rows': len(truth),
    }


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=pathlib.Path)
    parser.add_argument('--case-prefix', default='speed')
    args = parser.parse_args(argv)
    rows = []
    for directory in sorted(args.root.glob(f'{args.case_prefix}_*')):
        if (directory / 'supervisor_ground_truth.csv').exists() and any(
                path.name != 'supervisor_ground_truth.csv' for path in directory.glob('*.csv')):
            rows.append(analyze_case(directory))
    if not rows:
        raise SystemExit('no complete speed cases found')
    fields = list(rows[0])
    output_name = 'speed_sweep.csv' if args.case_prefix == 'speed' else 'acceleration_sweep.csv'
    with (args.root / output_name).open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    error_fields = ['case', 'target_speed_mps', 'wheel_ground_distance_error_m',
                    'wheel_ground_distance_error_pct', 'wheel_ground_slip_ratio_mean',
                    'wheel_ground_slip_ratio_abs_p95', 'odom_position_rmse_m',
                    'odom_final_position_error_m', 'straight_lateral_drift_m',
                    'straight_ground_yaw_drift_rad']
    with (args.root / 'wheel_ground_error.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=error_fields)
        writer.writeheader()
        writer.writerows({key: row[key] for key in error_fields} for row in rows)
    print(f'cases={len(rows)} {output_name}={args.root / output_name}')


if __name__ == '__main__':
    main()
