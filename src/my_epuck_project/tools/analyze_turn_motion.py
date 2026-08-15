#!/usr/bin/env python3
"""Offline analysis for the diagnostic-only autonomous turn capture."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def load_csv(path):
    with open(path, newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    fields = rows[0].keys() if rows else []
    out = {key: [] for key in fields}
    for row in rows:
        for key in fields:
            try:
                out[key].append(float(row[key]))
            except (TypeError, ValueError):
                out[key].append(np.nan)
    return {key: np.asarray(value, dtype=float) for key, value in out.items()}


def unwrap(a):
    return np.unwrap(a)


def interp(a, source_t, target_t):
    return np.interp(target_t, source_t, a)


def trapz_rate(rate, t):
    result = np.zeros(len(t), dtype=float)
    if len(t) > 1:
        result[1:] = np.cumsum((rate[1:] + rate[:-1]) * np.diff(t) * 0.5)
    return result


def contiguous(mask, t, max_gap=0.35):
    indexes = np.flatnonzero(mask)
    if not len(indexes):
        return []
    groups = [[indexes[0]]]
    for idx in indexes[1:]:
        if t[idx] - t[groups[-1][-1]] <= max_gap:
            groups[-1].append(idx)
        else:
            groups.append([idx])
    return [(g[0], g[-1]) for g in groups]


def write_csv(path, rows, fields):
    with open(path, 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--diagnostic', required=True)
    parser.add_argument('--ground-truth', required=True)
    parser.add_argument('--output-dir', required=True)
    args = parser.parse_args()
    outdir = Path(args.output_dir)
    outdir.mkdir(parents=True, exist_ok=True)
    d = load_csv(args.diagnostic)
    g = load_csv(args.ground_truth)
    t = d['sim_time_s']
    gt_t = g['sim_time_s']
    keep = np.isfinite(t)
    t = t[keep]
    for key in list(d):
        d[key] = d[key][keep]
    valid = (gt_t >= t[0]) & (gt_t <= t[-1])
    gt_t = gt_t[valid]
    for key in list(g):
        g[key] = g[key][valid]
    if len(t) < 10 or len(gt_t) < 10:
        raise RuntimeError('insufficient overlapping diagnostic and Supervisor data')

    # One initial SE(2) alignment: world GT is rotated into the initial robot
    # heading frame; odom is translated/yaw-offset at the same first sample.
    gx = interp(g['world_x_m'], gt_t, t)
    gy = interp(g['world_y_m'], gt_t, t)
    gyaw = unwrap(interp(unwrap(g['planar_yaw_rad']), gt_t, t))
    g0x, g0y, g0yaw = gx[0], gy[0], gyaw[0]
    c, s = math.cos(g0yaw), math.sin(g0yaw)
    gdx, gdy = gx - g0x, gy - g0y
    gt_x = c * gdx + s * gdy
    gt_y = -s * gdx + c * gdy
    gt_yaw = gyaw - g0yaw
    ox = d['odom_x_m']
    oy = d['odom_y_m']
    oyaw = unwrap(d['odom_yaw_rad'])
    finite_odom = np.isfinite(ox) & np.isfinite(oy) & np.isfinite(oyaw)
    first_odom = np.flatnonzero(finite_odom)[0]
    ox = ox - ox[first_odom]
    oy = oy - oy[first_odom]
    oyaw = oyaw - oyaw[first_odom]
    # Fill brief missing values only for derived plots; raw rows remain intact.
    ox = np.interp(t, t[finite_odom], ox[finite_odom])
    oy = np.interp(t, t[finite_odom], oy[finite_odom])
    oyaw = np.interp(t, t[finite_odom], oyaw[finite_odom])
    pos_err = np.hypot(ox - gt_x, oy - gt_y)
    yaw_err = oyaw - gt_yaw
    gt_ds = np.hypot(np.diff(gt_x, prepend=gt_x[0]),
                     np.diff(gt_y, prepend=gt_y[0]))
    odom_ds = np.hypot(np.diff(ox, prepend=ox[0]),
                       np.diff(oy, prepend=oy[0]))
    gt_distance = np.cumsum(gt_ds)
    odom_distance = np.cumsum(odom_ds)

    def arr(name):
        x = d.get(name, np.full(len(t), np.nan))
        return np.nan_to_num(x, nan=0.0)

    cmd_wz = arr('final_command_wz')
    cmd_vx = arr('final_command_vx')
    wheel_wz_ctrl = arr('wheel_predicted_wz_controller_radps')
    wheel_wz_phys = arr('wheel_predicted_wz_physical_radps')
    wheel_vx = arr('wheel_predicted_vx_mps')
    gt_speed = interp(g['ground_speed_mps'], gt_t, t)
    gt_omega = interp(g['angular_velocity_z_rps'], gt_t, t)
    cmd_yaw_int = trapz_rate(cmd_wz, t)
    wheel_yaw_ctrl_int = trapz_rate(wheel_wz_ctrl, t)
    wheel_yaw_phys_int = trapz_rate(wheel_wz_phys, t)
    gt_yaw_int = gt_yaw
    odom_yaw_int = oyaw
    # Turn candidates are based on actual body rotation, wheel rotation, or
    # final command.  This captures pure turns and moving arcs without relying
    # on a single noisy signal.
    turn_mask = (np.abs(gt_omega) >= 0.05) | (np.abs(wheel_wz_phys) >= 0.05) | (np.abs(cmd_wz) >= 0.05)
    intervals = []
    for a, b in contiguous(turn_mask, t):
        if abs(gt_yaw[b] - gt_yaw[a]) < 0.2 and abs(odom_yaw[b] - odom_yaw[a]) < 0.2:
            continue
        intervals.append((a, b))

    def delta(series, a, b):
        return float(series[b] - series[a])

    event_rows = []
    for number, (a, b) in enumerate(intervals, 1):
        prior = np.flatnonzero((t >= t[a] - 0.6) & (t < t[a]))
        after = np.flatnonzero((t > t[b]) & (t <= t[b] + 1.0))
        row = {
            'event': number, 'start_s': t[a], 'end_s': t[b],
            'duration_s': t[b] - t[a],
            'delta_yaw_cmd_rad': delta(cmd_yaw_int, a, b),
            'delta_yaw_wheels_controller_rad': delta(wheel_yaw_ctrl_int, a, b),
            'delta_yaw_wheels_physical_rad': delta(wheel_yaw_phys_int, a, b),
            'delta_yaw_odom_rad': delta(odom_yaw_int, a, b),
            'delta_yaw_gt_rad': delta(gt_yaw_int, a, b),
            'E_cmd_wheel_controller_rad': delta(cmd_yaw_int, a, b) - delta(wheel_yaw_ctrl_int, a, b),
            'E_cmd_wheel_physical_rad': delta(cmd_yaw_int, a, b) - delta(wheel_yaw_phys_int, a, b),
            'E_wheel_odom_controller_rad': delta(wheel_yaw_ctrl_int, a, b) - delta(odom_yaw_int, a, b),
            'E_wheel_odom_physical_rad': delta(wheel_yaw_phys_int, a, b) - delta(odom_yaw_int, a, b),
            'E_odom_gt_rad': delta(odom_yaw_int, a, b) - delta(gt_yaw_int, a, b),
            'gt_translation_m': float(gt_distance[b] - gt_distance[a]),
            'odom_translation_m': float(odom_distance[b] - odom_distance[a]),
            'peak_gt_speed_mps': float(np.max(np.abs(gt_speed[a:b + 1]))),
            'peak_cmd_wz_radps': float(np.max(np.abs(cmd_wz[a:b + 1]))),
            'peak_wheel_wz_radps': float(np.max(np.abs(wheel_wz_phys[a:b + 1]))),
            'start_from_rest': int(not len(prior) or np.max(np.abs(gt_speed[prior])) < 0.02),
            'turn_followed_by_forward': int(bool(len(after) and np.max(np.abs(cmd_vx[after])) > 0.05)),
            'recovery_associated': int(np.nanmax(arr('nav_recoveries')[a:b + 1]) > 0),
            'contact_observed': int(np.any(np.isfinite(d['collision_action_type'][a:b + 1]) & (d['collision_action_type'][a:b + 1] != 0))),
            'position_error_start_m': pos_err[a],
            'position_error_end_m': pos_err[b],
        }
        for distance in (1.0, 2.0, 5.0):
            future = np.flatnonzero(gt_distance >= gt_distance[b] + distance)
            row[f'position_error_growth_next_{distance:g}m'] = float(pos_err[future[0]] - pos_err[b]) if len(future) else np.nan
        row['category'] = 'PURE_ROTATION' if row['gt_translation_m'] < 0.05 else 'ARC_TURN'
        event_rows.append(row)

    event_fields = list(event_rows[0].keys()) if event_rows else [
        'event', 'start_s', 'end_s', 'duration_s']
    write_csv(outdir / 'turn_event_details.csv', event_rows, event_fields)
    abs_err = np.asarray([abs(x['E_odom_gt_rad']) for x in event_rows]) if event_rows else np.array([])
    summary_rows = []
    if len(abs_err):
        summary_rows.append({
            'category': 'ALL', 'count': len(abs_err),
            'median_abs_yaw_error_rad': np.median(abs_err),
            'mean_abs_yaw_error_rad': np.mean(abs_err),
            'p95_abs_yaw_error_rad': np.percentile(abs_err, 95),
            'max_abs_yaw_error_rad': np.max(abs_err),
        })
        cats = sorted(set(x['category'] for x in event_rows))
        for cat in cats:
            vals = np.asarray([abs(x['E_odom_gt_rad']) for x in event_rows if x['category'] == cat])
            summary_rows.append({'category': cat, 'count': len(vals),
                                 'median_abs_yaw_error_rad': np.median(vals),
                                 'mean_abs_yaw_error_rad': np.mean(vals),
                                 'p95_abs_yaw_error_rad': np.percentile(vals, 95),
                                 'max_abs_yaw_error_rad': np.max(vals)})
    write_csv(outdir / 'turn_error_summary.csv', summary_rows,
              ['category', 'count', 'median_abs_yaw_error_rad',
               'mean_abs_yaw_error_rad', 'p95_abs_yaw_error_rad',
               'max_abs_yaw_error_rad'])

    series_rows = []
    for i in range(len(t)):
        series_rows.append({
            'sim_time_s': t[i], 'gt_distance_m': gt_distance[i],
            'odom_distance_m': odom_distance[i], 'gt_x_m': gt_x[i],
            'gt_y_m': gt_y[i], 'odom_x_m': ox[i], 'odom_y_m': oy[i],
            'gt_yaw_rad': gt_yaw[i], 'odom_yaw_rad': oyaw[i],
            'position_error_m': pos_err[i], 'yaw_error_rad': yaw_err[i],
            'cmd_yaw_integral_rad': cmd_yaw_int[i],
            'wheel_yaw_integral_controller_rad': wheel_yaw_ctrl_int[i],
            'wheel_yaw_integral_physical_rad': wheel_yaw_phys_int[i],
            'gt_speed_mps': gt_speed[i], 'gt_omega_radps': gt_omega[i],
        })
    write_csv(outdir / 'command_wheel_odom_gt_timeseries.csv', series_rows,
              list(series_rows[0].keys()))

    # Strongest short-window error-growth intervals.
    growth_rows = []
    for window in (1.0, 2.0, 5.0):
        candidates = []
        for i in range(len(t)):
            j = int(np.searchsorted(t, t[i] + window))
            if j < len(t):
                candidates.append((float(pos_err[j] - pos_err[i]), i, j))
        for rank, (growth, i, j) in enumerate(sorted(candidates, reverse=True)[:10], 1):
            growth_rows.append({
                'window_s': window, 'rank': rank, 'start_s': t[i],
                'end_s': t[j], 'growth_m': growth,
                'start_error_m': pos_err[i], 'end_error_m': pos_err[j],
                'gt_distance_delta_m': gt_distance[j] - gt_distance[i],
                'yaw_error_start_rad': yaw_err[i],
                'yaw_error_end_rad': yaw_err[j],
            })
    write_csv(outdir / 'error_growth_windows.csv', growth_rows,
              list(growth_rows[0].keys()) if growth_rows else
              ['window_s', 'rank', 'start_s', 'end_s', 'growth_m',
               'start_error_m', 'end_error_m', 'gt_distance_delta_m',
               'yaw_error_start_rad', 'yaw_error_end_rad'])

    # Compact plots, intentionally diagnostic-only.
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    axes[0].plot(t, gt_yaw, label='Supervisor GT yaw')
    axes[0].plot(t, oyaw, label='odom yaw')
    axes[0].plot(t, cmd_yaw_int, label='command-integrated yaw', alpha=.8)
    axes[0].plot(t, wheel_yaw_ctrl_int, label='wheel yaw (controller L)', alpha=.8)
    axes[0].set_ylabel('relative yaw (rad)'); axes[0].legend(ncol=2)
    axes[1].plot(t, pos_err, label='GT→odom position error')
    axes[1].plot(t, np.abs(yaw_err), label='|GT→odom yaw error|')
    axes[1].set_xlabel('simulation time (s)'); axes[1].legend()
    fig.tight_layout(); fig.savefig(outdir / 'cumulative_error.png', dpi=150); plt.close(fig)

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, gt_yaw, label='GT')
    ax.plot(t, oyaw, label='odom')
    for row in event_rows:
        color = 'red' if abs(row['E_odom_gt_rad']) >= (np.median(abs_err) if len(abs_err) else 0.02) else 'green'
        ax.axvspan(row['start_s'], row['end_s'], color=color, alpha=.16)
    ax.set_xlabel('simulation time (s)'); ax.set_ylabel('relative yaw (rad)')
    ax.legend(); fig.tight_layout(); fig.savefig(outdir / 'good_bad_turns.png', dpi=150); plt.close(fig)

    if event_rows:
        strongest = max(event_rows, key=lambda x: abs(x['E_odom_gt_rad']))
        lo = max(t[0], strongest['start_s'] - 5.0)
        hi = min(t[-1], strongest['end_s'] + 20.0)
        mask = (t >= lo) & (t <= hi)
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        axes[0].plot(t[mask], cmd_wz[mask], label='final command wz')
        axes[0].plot(t[mask], wheel_wz_ctrl[mask], label='wheel wz (controller L)')
        axes[0].plot(t[mask], gt_omega[mask], label='Supervisor wz')
        axes[0].set_ylabel('angular rate (rad/s)'); axes[0].legend()
        axes[1].plot(t[mask], oyaw[mask], label='odom yaw')
        axes[1].plot(t[mask], gt_yaw[mask], label='GT yaw')
        axes[1].plot(t[mask], cmd_yaw_int[mask], label='command integral')
        axes[1].plot(t[mask], wheel_yaw_ctrl_int[mask], label='wheel integral')
        axes[1].set_ylabel('relative yaw (rad)'); axes[1].legend(ncol=2)
        axes[2].plot(t[mask], pos_err[mask], label='GT→odom position error')
        axes[2].set_xlabel('simulation time (s)'); axes[2].set_ylabel('error (m)'); axes[2].legend()
        for ax in axes:
            ax.axvspan(strongest['start_s'], strongest['end_s'], color='orange', alpha=.2)
        fig.tight_layout(); fig.savefig(outdir / 'strongest_turn_zoom.png', dpi=150); plt.close(fig)

        fig, ax = plt.subplots(figsize=(9, 5))
        ax.scatter([r['gt_translation_m'] for r in event_rows],
                   [abs(r['E_odom_gt_rad']) for r in event_rows])
        ax.set_xlabel('GT translation during turn (m)')
        ax.set_ylabel('|odom−GT turn yaw disagreement| (rad)')
        fig.tight_layout(); fig.savefig(outdir / 'turn_error_vs_motion.png', dpi=150); plt.close(fig)

    sens = {}
    for deg in (90, 118, 180):
        rad = math.radians(deg)
        sens[str(deg)] = {
            'physical_separation_rad': rad,
            'controller_separation_rad': rad / 1.095,
            'controller_minus_physical_rad': rad / 1.095 - rad,
            'controller_minus_physical_deg': math.degrees(rad / 1.095 - rad),
        }
    json.dump({
        'wheel_radius_m': .02, 'physical_separation_m': .052,
        'multiplier': 1.095, 'effective_separation_m': .052 * 1.095,
        'idealized_turn_sensitivity': sens,
        'note': 'Geometry sensitivity only; no production parameter changed.'},
        open(outdir / 'wheel_separation_sensitivity.json', 'w'), indent=2)
    report = {
        'alignment': {'first_common_sim_time_s': float(t[0]),
                      'method': 'single initial SE(2) offset; GT rotated by initial yaw; linear interpolation'},
        'sample_counts': {'diagnostic': int(len(t)), 'supervisor_overlap': int(len(gt_t))},
        'duration_s': float(t[-1] - t[0]),
        'turn_count': len(event_rows),
        'final_position_error_m': float(pos_err[-1]),
        'final_abs_yaw_error_rad': float(abs(yaw_err[-1])),
        'gt_distance_m': float(gt_distance[-1]), 'odom_distance_m': float(odom_distance[-1]),
        'max_position_error_m': float(np.max(pos_err)),
        'max_abs_yaw_error_rad': float(np.max(np.abs(yaw_err))),
        'contact_telemetry_available': bool(np.any(np.isfinite(d['collision_action_type']))),
        'event_count_by_category': {cat: sum(x['category'] == cat for x in event_rows) for cat in sorted(set(x['category'] for x in event_rows))},
    }
    json.dump(report, open(outdir / 'analysis_summary.json', 'w'), indent=2)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
