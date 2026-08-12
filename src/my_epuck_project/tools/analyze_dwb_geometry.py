#!/usr/bin/env python3
"""Analyze and plot one bounded DWB geometry-capture directory."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def transform_point(point, transform):
    c, s = math.cos(transform[2]), math.sin(transform[2])
    return (c * point[0] - s * point[1] + transform[0],
            s * point[0] + c * point[1] + transform[1])


def inverse_transform_point(point, transform):
    dx, dy = point[0] - transform[0], point[1] - transform[1]
    c, s = math.cos(transform[2]), math.sin(transform[2])
    return (c * dx + s * dy, -s * dx + c * dy)


def read_jsonl(path):
    return [json.loads(line) for line in path.open()
            if line.strip()]


def plan_for(plans, time_s):
    available = [item for item in plans
                 if float(item.get('sim_time_s', 0.0)) <= time_s + 1e-9]
    return available[-1] if available else None


def tf_value(row, name):
    values = json.loads(row['tf_json'])
    item = values.get(name, {})
    if not item.get('translation'):
        return None
    t = item['translation']
    return float(t['x_m']), float(t['y_m']), float(item.get('yaw_rad', 0.0))


def path_metrics(row, plan):
    if plan is None or not plan.get('points'):
        return {
            'path_available': 0, 'closest_path_distance_m': '',
            'closest_path_ahead_m': '', 'closest_path_index': '',
            'first_point_distance_m': '', 'first_point_angle_rad': '',
            'path_tangent_error_rad': '', 'path_tangent_yaw_rad': '',
            'curvature_0p10_rad': '', 'curvature_0p25_rad': '',
            'curvature_0p50_rad': '', 'transformed_plan_length_m': '',
            'transformed_plan_point_count': '',
        }
    try:
        robot = (float(row['odom_x_m']), float(row['odom_y_m']),
                 float(row['odom_yaw_rad']))
    except (KeyError, ValueError):
        robot = tf_value(row, 'odom_to_base_footprint')
    if robot is None:
        return {'path_available': 0}
    points = [(float(p['x_m']), float(p['y_m'])) for p in plan['points']]
    distances = [math.hypot(p[0] - robot[0], p[1] - robot[1])
                 for p in points]
    closest = min(range(len(points)), key=distances.__getitem__)
    tangent_index = min(closest, len(points) - 2)
    if tangent_index < 0:
        tangent_index = 0
    tangent = math.atan2(points[tangent_index + 1][1] - points[tangent_index][1],
                         points[tangent_index + 1][0] - points[tangent_index][0])
    heading = robot[2]
    forward = ((points[closest][0] - robot[0]) * math.cos(heading) +
               (points[closest][1] - robot[1]) * math.sin(heading))
    first_angle = wrap(math.atan2(points[0][1] - robot[1],
                                  points[0][0] - robot[0]) - heading)
    def heading_at_distance(target):
        travelled = 0.0
        for i in range(len(points) - 1):
            segment = math.hypot(points[i + 1][0] - points[i][0],
                                points[i + 1][1] - points[i][1])
            travelled += segment
            if travelled >= target:
                return math.atan2(points[i + 1][1] - points[0][1],
                                  points[i + 1][0] - points[0][0])
        return None
    first_tangent = math.atan2(points[1][1] - points[0][1],
                               points[1][0] - points[0][0]) if len(points) > 1 else None
    def curvature(target):
        later = heading_at_distance(target)
        return '' if first_tangent is None or later is None else wrap(later - first_tangent)
    return {
        'path_available': 1,
        'closest_path_distance_m': distances[closest],
        'closest_path_ahead_m': forward,
        'closest_path_index': closest,
        'first_point_distance_m': distances[0],
        'first_point_angle_rad': first_angle,
        'path_tangent_error_rad': wrap(tangent - heading),
        'path_tangent_yaw_rad': tangent,
        'curvature_0p10_rad': curvature(0.10),
        'curvature_0p25_rad': curvature(0.25),
        'curvature_0p50_rad': curvature(0.50),
        'transformed_plan_length_m': plan.get('length_m', ''),
        'transformed_plan_point_count': plan.get('point_count', ''),
    }


def derive(run: Path):
    rows = list(csv.DictReader((run / 'dwb_geometry_timeseries.csv').open()))
    plans = read_jsonl(run / 'dwb_geometry_transformed_plans.jsonl')
    derived = []
    for row in rows:
        time_s = float(row['sim_time_s'])
        try:
            robot = (float(row['odom_x_m']), float(row['odom_y_m']),
                     float(row['odom_yaw_rad']))
        except (KeyError, ValueError):
            robot = tf_value(row, 'odom_to_base_footprint')
        shared_to_odom = tf_value(row, 'shared_map_to_odom')
        goal = (float(row['goal_x_m']), float(row['goal_y_m']))
        goal_odom = goal if shared_to_odom is None else inverse_transform_point(goal, shared_to_odom)
        try:
            current_yaw = float(row['odom_yaw_rad'])
            current_x = float(row['odom_x_m'])
            current_y = float(row['odom_y_m'])
            goal_bearing = wrap(math.atan2(
                goal_odom[1] - current_y, goal_odom[0] - current_x) - current_yaw)
        except (KeyError, ValueError):
            goal_bearing = '' if robot is None else wrap(
                math.atan2(goal_odom[1] - robot[1], goal_odom[0] - robot[0]) - robot[2])
        metrics = path_metrics(row, plan_for(plans, time_s))
        metrics.update({
            'sim_time_s': time_s,
            'goal_bearing_error_rad': goal_bearing,
            'robot_odom_x_m': '' if robot is None else robot[0],
            'robot_odom_y_m': '' if robot is None else robot[1],
            'robot_odom_yaw_rad': '' if robot is None else robot[2],
            'command_kind': row['command_kind'],
            'dwb_vx_mps': row['dwb_vx_mps'], 'dwb_wz_radps': row['dwb_wz_radps'],
            'selected_score': row['dwb_selected_score'],
            'best_forward_score': row['dwb_best_forward_score'],
            'score_gap_forward_minus_selected': row['dwb_score_gap_forward_minus_selected'],
            'pathalign_difference_forward_minus_selected': (
                '' if row['pathalign_forward'] == '' or row['pathalign_selected'] == '' else
                float(row['pathalign_forward']) - float(row['pathalign_selected'])),
            'pathdist_difference_forward_minus_selected': (
                '' if row['pathdist_forward'] == '' or row['pathdist_selected'] == '' else
                float(row['pathdist_forward']) - float(row['pathdist_selected'])),
            'goaldist_difference_forward_minus_selected': (
                '' if row['goaldist_forward'] == '' or row['goaldist_selected'] == '' else
                float(row['goaldist_forward']) - float(row['goaldist_selected'])),
        })
        derived.append(metrics)
    fields = list(derived[0]) if derived else []
    with (run / 'dwb_geometry_derived.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(derived)
    return derived, plans


def make_plots(run: Path, derived, plans):
    import matplotlib.pyplot as plt
    import numpy as np
    plot_dir = run / 'plots'; plot_dir.mkdir(exist_ok=True)
    times = np.array([r['sim_time_s'] for r in derived])
    def numeric(name):
        return np.array([float(r[name]) if r[name] != '' else np.nan for r in derived])
    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    axes[0].plot(times, numeric('robot_odom_yaw_rad'), label='robot yaw')
    axes[0].plot(times, numeric('goal_bearing_error_rad'), label='goal bearing error')
    axes[0].plot(times, numeric('path_tangent_error_rad'), label='local tangent error')
    axes[0].legend(); axes[0].set_ylabel('rad')
    axes[1].plot(times, numeric('dwb_vx_mps'), label='selected vx')
    axes[1].plot(times, numeric('dwb_wz_radps'), label='selected wz')
    axes[1].legend(); axes[1].set_ylabel('command')
    axes[2].plot(times, numeric('selected_score'), label='selected total')
    axes[2].plot(times, numeric('best_forward_score'), label='best forward total')
    axes[2].plot(times, numeric('score_gap_forward_minus_selected'), label='forward-selected')
    axes[2].legend(); axes[2].set_ylabel('score')
    axes[3].plot(times, numeric('pathalign_difference_forward_minus_selected'), label='PathAlign diff')
    axes[3].plot(times, numeric('pathdist_difference_forward_minus_selected'), label='PathDist diff')
    axes[3].plot(times, numeric('goaldist_difference_forward_minus_selected'), label='GoalDist diff')
    axes[3].legend(); axes[3].set_ylabel('forward - selected')
    axes[3].set_xlabel('simulation time (s)')
    fig.suptitle('Robot 1 first frontier geometry window; no angular-to-zero transition observed')
    fig.tight_layout(); fig.savefig(plot_dir / 'geometry_timeseries.png', dpi=150); plt.close(fig)

    sequences = read_jsonl(run / 'dwb_geometry_trajectory_sequences.jsonl')
    costmaps = read_jsonl(run / 'dwb_geometry_costmaps.jsonl')
    snapshots = [466.8, 468.7, 471.0, 472.6]
    for number, time_s in enumerate(snapshots, 1):
        row = min(derived, key=lambda item: abs(item['sim_time_s'] - time_s))
        actual = row['sim_time_s']
        plan = plan_for(plans, actual)
        fig, ax = plt.subplots(figsize=(8, 8))
        if number == 1 and costmaps:
            grid = costmaps[0]
            width, height = int(grid['width']), int(grid['height'])
            if width * height == len(grid['data']):
                values = np.array(grid['data'], dtype=float).reshape((height, width))
                origin = grid['origin']
                extent = (origin['x_m'], origin['x_m'] + width * grid['resolution_m'],
                          origin['y_m'], origin['y_m'] + height * grid['resolution_m'])
                ax.imshow(values, origin='lower', extent=extent, alpha=0.28,
                          cmap='gray_r', vmin=0, vmax=255,
                          interpolation='nearest', label='local costmap snapshot')
        if plan:
            pts = plan['points']; ax.plot([p['x_m'] for p in pts], [p['y_m'] for p in pts], 'k-', label='transformed DWB plan')
        robot = (float(row['robot_odom_x_m']), float(row['robot_odom_y_m']))
        yaw = float(row['robot_odom_yaw_rad'])
        ax.plot(*robot, 'bo', label='robot')
        ax.arrow(robot[0], robot[1], .2 * math.cos(yaw), .2 * math.sin(yaw), head_width=.03, color='b')
        # Trajectory poses are robot-local; use the nearest captured evaluation event.
        if sequences:
            event = min(sequences, key=lambda item: abs(float(item['sim_time_s']) - actual))
            for seq in event['sequences']:
                # DWB Trajectory2D poses are already expressed in the
                # evaluation/header frame (robot1/odom), not robot-local
                # coordinates.  Preserve them without a second transform.
                points = [(p['x_m'], p['y_m']) for p in seq['poses']]
                ax.plot([p[0] for p in points], [p[1] for p in points], label=seq['role'])
        ax.set_aspect('equal', adjustable='box'); ax.grid(True)
        ax.set_title(f'Robot 1 frontier geometry at {actual:.2f}s (global plan unavailable)')
        ax.legend(fontsize=8); fig.tight_layout(); fig.savefig(plot_dir / f'geometry_topdown_{number}_{actual:.2f}.png', dpi=150); plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    derived, plans = derive(args.run)
    make_plots(args.run, derived, plans)
    print(f'rows={len(derived)} plots={args.run / "plots"}')


if __name__ == '__main__':
    main()
