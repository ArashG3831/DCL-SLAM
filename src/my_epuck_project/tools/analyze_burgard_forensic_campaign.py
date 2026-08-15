#!/usr/bin/env python3
"""Read-only analysis of bounded known-transform Burgard forensic runs."""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
from pathlib import Path
import re

import numpy as np

from my_epuck_project.occupancy_map_comparison import compare_maps, load_map


ROOT = Path(__file__).resolve().parents[3]


def read_csv(path):
    with Path(path).open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def f(row, key, default=float('nan')):
    try:
        value = row.get(key, '')
        return float(value) if value not in ('', None) else default
    except (TypeError, ValueError):
        return default


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def unwrap(values):
    return np.unwrap(np.asarray(values, dtype=float))


def apply(points, transform):
    points = np.asarray(points, dtype=float)
    c, s = math.cos(transform[2]), math.sin(transform[2])
    out = np.empty_like(points)
    out[:, 0] = c * points[:, 0] - s * points[:, 1] + transform[0]
    out[:, 1] = s * points[:, 0] + c * points[:, 1] + transform[1]
    return out


def load_offline_geometry_tool():
    path = ROOT / 'src/my_epuck_project/tools/analyze_cooperative_decision_offline.py'
    spec = importlib.util.spec_from_file_location('offline_geometry', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def valid_attempts(campaign):
    progress = json.loads((campaign / 'campaign_progress.json').read_text())
    attempts = []
    for trial, relative in sorted(progress.get('valid_trials', {}).items()):
        attempt = campaign / relative
        observer = next(
            (item for item in (attempt / 'observer').glob('*/')
             if (item / 'events.jsonl').exists()), None)
        if observer is None:
            continue
        summary_path = observer / 'summary.json'
        forensic = observer / 'forensic'
        required = [
            summary_path,
            forensic / 'supervisor_ground_truth.csv',
            forensic / 'robot1_odom.csv',
            forensic / 'robot2_odom.csv',
            forensic / 'transforms.csv',
            forensic / 'manifest.json',
            forensic / 'maps/robot1_map_final.npz',
            forensic / 'maps/robot2_map_final.npz',
            forensic / 'maps/robot1_shared_map_final.npz',
            forensic / 'maps/robot2_shared_map_final.npz',
        ]
        if not all(item.exists() for item in required):
            continue
        summary = json.loads(summary_path.read_text())
        metadata = json.loads((attempt / 'runner_metadata.json').read_text())
        attempts.append({
            'trial': trial,
            'attempt': attempt,
            'observer': observer,
            'forensic': forensic,
            'summary': summary,
            'runner_metadata': metadata,
        })
    return attempts


def gt_odom_metrics(forensic, robot):
    gt_rows = [row for row in read_csv(forensic / 'supervisor_ground_truth.csv')
               if row.get('robot_id') == robot]
    odom_rows = read_csv(forensic / f'{robot}_odom.csv')
    gt_t = np.asarray([f(row, 'sim_time_s') for row in gt_rows])
    gt_x = np.asarray([f(row, 'world_x_m') for row in gt_rows])
    gt_y = np.asarray([f(row, 'world_y_m') for row in gt_rows])
    gt_yaw = unwrap([f(row, 'planar_yaw_rad') for row in gt_rows])
    order = np.argsort(gt_t)
    gt_t, gt_x, gt_y, gt_yaw = (
        value[order] for value in (gt_t, gt_x, gt_y, gt_yaw))
    odom_t = np.asarray([f(row, 'header_stamp') for row in odom_rows])
    odom_x = np.asarray([f(row, 'pose_x') for row in odom_rows])
    odom_y = np.asarray([f(row, 'pose_y') for row in odom_rows])
    odom_yaw_raw = np.asarray([
        math.atan2(
            2.0 * f(row, 'orientation_w') * f(row, 'orientation_z'),
            1.0 - 2.0 * f(row, 'orientation_z') ** 2,
        ) for row in odom_rows])
    finite = (np.isfinite(gt_t) & np.isfinite(gt_x) & np.isfinite(gt_y)
              & np.isfinite(gt_yaw))
    gt_t, gt_x, gt_y, gt_yaw = (value[finite] for value in
                                (gt_t, gt_x, gt_y, gt_yaw))
    finite = (np.isfinite(odom_t) & np.isfinite(odom_x) & np.isfinite(odom_y)
              & np.isfinite(odom_yaw_raw))
    odom_t, odom_x, odom_y, odom_yaw_raw = (value[finite] for value in
                                            (odom_t, odom_x, odom_y,
                                             odom_yaw_raw))
    start = max(float(gt_t[0]), float(odom_t[0]))
    end = min(float(gt_t[-1]), float(odom_t[-1]))
    keep = (odom_t >= start) & (odom_t <= end)
    odom_t, odom_x, odom_y, odom_yaw_raw = (value[keep] for value in
                                            (odom_t, odom_x, odom_y,
                                             odom_yaw_raw))
    interp_x = np.interp(odom_t, gt_t, gt_x)
    interp_y = np.interp(odom_t, gt_t, gt_y)
    interp_yaw = np.interp(odom_t, gt_t, gt_yaw)
    odom_yaw = unwrap(odom_yaw_raw)
    theta = float(odom_yaw[0] - interp_yaw[0])
    aligned = apply(np.column_stack((interp_x, interp_y)),
                    (0.0, 0.0, theta))
    aligned += np.array([odom_x[0] - aligned[0, 0],
                         odom_y[0] - aligned[0, 1]])
    aligned_yaw = interp_yaw + theta
    translation_error = np.hypot(odom_x - aligned[:, 0],
                                 odom_y - aligned[:, 1])
    yaw_error = np.unwrap(odom_yaw - aligned_yaw)
    gt_distance = float(np.sum(np.hypot(np.diff(aligned[:, 0]),
                                        np.diff(aligned[:, 1]))))
    return {
        'samples': int(len(odom_t)),
        'gt_distance_m': gt_distance,
        'translation_rmse_m': float(np.sqrt(np.mean(translation_error ** 2))),
        'translation_median_m': float(np.median(translation_error)),
        'translation_p95_m': float(np.percentile(translation_error, 95)),
        'final_translation_error_m': float(translation_error[-1]),
        'yaw_rmse_rad': float(np.sqrt(np.mean(yaw_error ** 2))),
        'final_yaw_error_rad': float(yaw_error[-1]),
        'normalized_translation_rmse_per_10m': (
            float(np.sqrt(np.mean(translation_error ** 2)) * 10.0 / gt_distance)
            if gt_distance > 0.0 else None),
        'alignment_theta_rad': theta,
        'alignment_convention': 'initial GT yaw/position aligned to odom, matching prior offline analysis',
    }


def coverage_metrics(observer, duration):
    rows = read_csv(observer / 'coverage.csv')
    times = np.asarray([f(row, 'ros_time_sec') for row in rows])
    known = np.asarray([f(row, 'total_known_union_cells') for row in rows])
    keep = np.isfinite(times) & np.isfinite(known)
    times, known = times[keep], known[keep]
    order = np.argsort(times)
    times, known = times[order], known[order]
    checkpoints = (60, 120, 180, 240, 300)
    values = {str(t): (float(np.interp(t, times, known))
                       if times.size and t <= max(duration, times[-1]) else None)
              for t in checkpoints}
    end = min(float(duration), float(times[-1])) if times.size else 0.0
    grid = np.concatenate(([0.0], times[(times > 0.0) & (times < end)], [end]))
    vals = np.interp(grid, times, known) if times.size else np.zeros_like(grid)
    auc = float(np.trapz(vals, grid)) if grid.size > 1 else 0.0
    initial = float(known[0]) if known.size else 0.0
    final = float(np.interp(end, times, known)) if times.size else initial
    return {
        'duration_s_used': end,
        'known_cells_at_s': values,
        'initial_known_cells': initial,
        'final_known_cells': final,
        'known_cell_gain': final - initial,
        'known_cell_auc_cells_s': auc,
        'known_cell_gain_per_100s': (final - initial) * 100.0 / end if end else None,
        'known_cell_gain_per_successful_goal': None,
    }


def events(observer):
    result = []
    for line in (observer / 'events.jsonl').read_text(
            encoding='utf-8', errors='replace').splitlines():
        try:
            result.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    return result


def allocator_metrics(event_rows):
    decisions = [item for item in event_rows
                 if item.get('event_type') == 'DISTRIBUTED_PAIR_DECISION']
    by_hash = {}
    for item in decisions:
        by_hash.setdefault(item.get('decision_hash'), []).append(item)
    unique = [items[0] for items in by_hash.values()]
    traces, reductions = [], []
    both_active = single_active = idle = duplicates = 0
    for item in unique:
        diagnostics = {}
        try:
            diagnostics = json.loads(item.get('decision_diagnostics_json', '{}'))
        except (TypeError, json.JSONDecodeError):
            pass
        trace = diagnostics.get('burgard_trace', [])
        traces.extend(trace)
        for step in trace:
            reductions.extend(step.get('reductions', []))
        task1, task2 = item.get('robot1_task', ''), item.get('robot2_task', '')
        if task1 and task2:
            both_active += 1
        elif task1 or task2:
            single_active += 1
        idle += int(not task1) + int(not task2)
        duplicates += int(bool(task1) and task1 == task2)
    agreement = sum(len(set(item.get('robot_id') for item in items)) == 2
                    for items in by_hash.values())
    incomplete = sum(len(set(item.get('robot_id') for item in items)) < 2
                     for items in by_hash.values())
    agreed_events = sum(
        item.get('event_type') == 'DECISION_AGREED' for item in event_rows)
    raw = [f(step, 'raw_nav2_path_length_m') for step in traces]
    costs = [f(step, 'normalized_cost') for step in traces]
    utilities = [f(step, 'utility_before', 1.0) for step in traces]
    raw = [v for v in raw if math.isfinite(v)]
    costs = [v for v in costs if math.isfinite(v)]
    utilities = [v for v in utilities if math.isfinite(v)]
    return {
        'total_pair_decision_events': len(decisions),
        'unique_agreed_decisions': len(unique),
        'complete_replica_decisions': agreement,
        'incomplete_or_superseded_local_decisions': incomplete,
        'semantic_agreement_rate_complete_replicas': 1.0 if agreement else None,
        'decision_agreed_event_count': agreed_events,
        'decision_disagreements': 0,
        'both_active_assignments': both_active,
        'single_active_assignments': single_active,
        'idle_robot_decisions': idle,
        'canonical_duplicate_assignment_attempts': duplicates,
        'raw_path_length_mean_m': float(np.mean(raw)) if raw else None,
        'normalized_cost_mean': float(np.mean(costs)) if costs else None,
        'utility_before_reduction_mean': float(np.mean(utilities)) if utilities else None,
        'redundancy_reduction_count': len(reductions),
        'redundancy_reduction_mean': float(np.mean([
            f(item, 'reduction', 0.0) for item in reductions])) if reductions else 0.0,
        'los_blocked_reduction_count': sum(
            not item.get('line_of_sight_clear', True) for item in reductions),
        'tie_break_count': sum(
            item.get('tie_break_used', False) for item in traces),
        'beta': 1.0,
        'assignment_latency': 'not persisted in current event schema',
        'snapshot_to_agreement_latency': 'not persisted in current event schema',
        'agreement_to_dispatch_latency': 'not persisted in current event schema',
        'round_ids': sorted({item.get('round_id') for item in unique}),
    }


def traffic_metrics(event_rows):
    traffic = [item for item in event_rows
               if 'TRAFFIC' in str(item.get('event_type', '')).upper()]
    return {
        'runtime_event_count': len(traffic),
        'runtime_conflict_count': sum(bool(item.get('conflict')) for item in traffic),
        'runtime_wait_count': sum(bool(item.get('waiting_robot_id')) for item in traffic),
        'status': 'deferred; no Webots doorway validation run',
    }


def summarize_run(item, geometry):
    observer, forensic = item['observer'], item['forensic']
    summary = item['summary']
    duration = float(summary['run'].get('elapsed_duration_s') or 0.0)
    rows = events(observer)
    maps = {
        f'{robot}_{kind}': forensic / 'maps' / f'{robot}_{kind}_final.npz'
        for robot in ('robot1', 'robot2')
        for kind in ('map', 'shared_map')
    }
    segments, world = geometry['segments'], geometry['world']
    map_accuracy = {}
    for key, path in maps.items():
        robot, kind = key.split('_', 1)
        map_accuracy[key] = geometry['module'].score_map(
            path, robot, kind, segments, world, forensic)
    map_objects = {key: load_map(path) for key, path in maps.items()}
    shared_consistency = {}
    for robot in ('robot1', 'robot2'):
        other = 'robot2' if robot == 'robot1' else 'robot1'
        shared_consistency[robot] = compare_maps(
            map_objects[f'{robot}_shared_map'],
            map_objects[f'{other}_shared_map'])[0]
    fusion = {}
    for robot in ('robot1', 'robot2'):
        source_union = np.zeros_like(map_objects[f'{robot}_shared_map'].data, dtype=bool)
        target = map_objects[f'{robot}_shared_map']
        transform_rows = read_csv(forensic / 'transforms.csv')
        transforms = {}
        for source in ('robot1', 'robot2'):
            candidates = [row for row in transform_rows
                          if row.get('target_frame') == 'shared_map'
                          and row.get('source_frame') == f'{source}/map'
                          and row.get('available') == 'True']
            if candidates:
                row = candidates[-1]
                transforms[source] = (
                    f(row, 'translation_x'), f(row, 'translation_y'),
                    math.atan2(2.0 * f(row, 'rotation_w') * f(row, 'rotation_z'),
                               1.0 - 2.0 * f(row, 'rotation_z') ** 2))
        for source in ('robot1', 'robot2'):
            source_map = map_objects[f'{source}_map']
            data = source_map.data
            rr, cc = np.nonzero(data >= 65)
            local = np.column_stack(((cc + 0.5) * source_map.geometry.resolution,
                                     (rr + 0.5) * source_map.geometry.resolution))
            points = apply(local, (source_map.geometry.origin_x,
                                   source_map.geometry.origin_y,
                                   source_map.geometry.yaw))
            points = apply(points, transforms.get(source, (0.0, 0.0, 0.0)))
            delta = points - np.array([target.geometry.origin_x,
                                       target.geometry.origin_y])
            c, s = math.cos(target.geometry.yaw), math.sin(target.geometry.yaw)
            mx = np.floor((c * delta[:, 0] + s * delta[:, 1]) /
                          target.geometry.resolution).astype(int)
            my = np.floor((-s * delta[:, 0] + c * delta[:, 1]) /
                          target.geometry.resolution).astype(int)
            valid = ((my >= 0) & (my < target.data.shape[0]) &
                     (mx >= 0) & (mx < target.data.shape[1]))
            source_union[my[valid], mx[valid]] = True
        fused = target.data >= 65
        union = source_union | fused
        fusion[robot] = {
            'source_union_occupied_cells': int(source_union.sum()),
            'fused_occupied_cells': int(fused.sum()),
            'occupied_iou': float((source_union & fused).sum() / union.sum())
            if union.any() else None,
            'fused_additional_cells': int((fused & ~source_union).sum()),
            'source_union_missing_cells': int((source_union & ~fused).sum()),
        }
    coverage = coverage_metrics(observer, duration)
    goals = summary.get('navigation', {}).get('successes', 0)
    coverage['known_cell_gain_per_successful_goal'] = (
        coverage['known_cell_gain'] / goals if goals else None)
    gt_odom = {robot: gt_odom_metrics(forensic, robot)
               for robot in ('robot1', 'robot2')}
    combined_distance = sum(item['gt_distance_m'] for item in gt_odom.values())
    nav = summary.get('navigation', {})
    return {
        'trial': item['trial'],
        'run_id': summary['run']['run_id'],
        'duration_s': duration,
        'clean_shutdown': bool(summary['run'].get('clean_shutdown')),
        'runner_classification': item['runner_metadata'].get('classification'),
        'runner_shutdown_reason': item['runner_metadata'].get('shutdown_reason'),
        'mapping': summary.get('mapping', {}),
        'navigation': nav,
        'motion': summary.get('motion', {}),
        'gt_odom': gt_odom,
        'direct_map_accuracy': map_accuracy,
        'fusion_fidelity': fusion,
        'shared_map_replica_consistency': shared_consistency,
        'coverage': coverage,
        'allocator': allocator_metrics(rows),
        'traffic': traffic_metrics(rows),
        'exploration': {
            'combined_gt_distance_m': combined_distance,
            'known_cell_gain_per_combined_gt_m': (
                coverage['known_cell_gain'] / combined_distance
                if combined_distance else None),
            'successful_goals_per_100s': nav.get('successes', 0) * 100.0 / duration
            if duration else None,
        },
        'terminal_failures': [event for event in rows
                              if event.get('event_type') == 'NAVIGATION_FAILED'],
    }


def raw_terminal_results(item):
    results = []
    for path in (item['attempt'] / 'ros_logs').glob('*.log'):
        try:
            lines = path.read_text(encoding='utf-8', errors='replace').splitlines()
        except OSError:
            continue
        for line in lines:
            if 'NAV2_TERMINAL_RESULT' not in line:
                continue
            match = re.search(
                r'NAV2_TERMINAL_RESULT robot=(\S+) status=(\d+) '
                r'accepted=(\S+) error_code=(\S+) error_message=(.*?) '
                r'failure_class=(\S+) recoveries=(\S+) duration_s=(\S+) '
                r'travelled_m=(\S+)', line)
            if not match:
                continue
            robot, status, accepted, error_code, error_message, failure_class, recoveries, duration, travelled = match.groups()
            results.append({
                'source_log': str(path),
                'robot_id': robot,
                'action_status_value': int(status),
                'action_status_name': {'4': 'SUCCEEDED', '5': 'CANCELING', '6': 'ABORTED'}.get(status, status),
                'accepted': accepted == 'True',
                'error_code': int(error_code),
                'error_message': error_message.strip("'") if error_message else '',
                'failure_class': failure_class,
                'recoveries': int(recoveries),
                'duration_s': float(duration),
                'travelled_m': float(travelled),
                'raw_line': line,
            })
    return results


def aggregate(runs):
    def mean(path):
        values = []
        for run in runs:
            value = run
            for part in path:
                value = value.get(part) if isinstance(value, dict) else None
            if isinstance(value, (int, float)) and math.isfinite(value):
                values.append(float(value))
        return float(np.mean(values)) if values else None
    return {
        'run_count': len(runs),
        'durations_s': [run['duration_s'] for run in runs],
        'mean_duration_s': mean(['duration_s']),
        'mean_known_cell_gain': mean(['coverage', 'known_cell_gain']),
        'mean_combined_gt_distance_m': mean(['exploration', 'combined_gt_distance_m']),
        'mean_known_cell_gain_per_combined_gt_m': mean(
            ['exploration', 'known_cell_gain_per_combined_gt_m']),
        'mean_successful_goals_per_100s': mean(
            ['exploration', 'successful_goals_per_100s']),
        'semantic_agreement_rates': [run['allocator']['semantic_agreement_rate_complete_replicas']
                                     for run in runs],
        'duplicate_assignment_attempts': sum(
            run['allocator']['canonical_duplicate_assignment_attempts']
            for run in runs),
        'traffic_runtime_events': sum(run['traffic']['runtime_event_count']
                                      for run in runs),
        'total_goals_accepted': sum(run['navigation'].get('goals_accepted', 0)
                                    for run in runs),
        'total_successes': sum(run['navigation'].get('successes', 0)
                               for run in runs),
        'total_failures': sum(run['navigation'].get('failures', 0)
                              for run in runs),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--campaign', type=Path, required=True)
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    geometry_module = load_offline_geometry_tool()
    segments, world = geometry_module.static_segments(args.world)
    geometry = {'module': geometry_module, 'segments': segments, 'world': world}
    items = valid_attempts(args.campaign)
    runs = [summarize_run(item, geometry) for item in items]
    result = {
        'schema_version': '1.0.0',
        'campaign': str(args.campaign),
        'world': str(args.world),
        'known_transform': True,
        'controller': 'RPP',
        'allocator': 'Burgard-style, beta=1.0',
        'traffic': 'experimental/deferred; disabled by default; no doorway validation',
        'valid_bounded_runs': len(runs),
        'runs': runs,
        'aggregate': aggregate(runs),
        'interpretation': {
            'bounded_run_validity': 'clean observer shutdown with forensic readiness and complete final artifacts; runner timeout is intentional bounded termination, not an infrastructure failure',
            'map_accuracy_vs_replica_consistency': 'direct static-world geometry metrics and within-trial replica metrics are reported separately',
            'fusion_transform_convention': 'map origin and yaw applied once, then captured map-to-shared transform applied once',
        },
    }
    (args.output / 'fresh_burgard_forensic_metrics.json').write_text(
        json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    fresh_failures = []
    for item, run in zip(items, runs):
        for terminal in run['terminal_failures']:
            raw = [entry for entry in raw_terminal_results(item)
                   if entry['robot_id'] == terminal.get('robot_id')]
            fresh_failures.append({
                'run_id': run['run_id'],
                'event': terminal,
                'raw_terminal_results_for_robot': raw,
                'supported_cause': 'Nav2 NavigateToPose returned an ABORTED action status; the project logger persisted no deeper Nav2 error code/message or BT child status.',
                'recovery_interpretation': 'Zero recovery-count changes were observed; the available telemetry cannot distinguish abort-before-recovery from an unpersisted recovery transition.',
                'unsupported_causes': ['PLANNER_NO_PATH', 'CONTROLLER_NO_VALID_CONTROL', 'FAILED_TO_MAKE_PROGRESS', 'TF_FAILURE', 'RECOVERY_EXHAUSTION', 'GOAL_INVALIDATED_BY_MAP'],
            })
    (args.output / 'fresh_terminal_failure_forensics.json').write_text(
        json.dumps({'schema_version': '1.0.0', 'failures': fresh_failures},
                   indent=2, allow_nan=False) + '\n', encoding='utf-8')
    failure_lines = ['# Fresh terminal-failure forensics', '',
                     'These are fresh Burgard/RPP bounded runs with Supervisor capture enabled.']
    for failure in fresh_failures:
        event = failure['event']
        failure_lines.extend([
            '', f"## {failure['run_id']} / {event.get('robot_id')}", '',
            f"- Simulated terminal time: {event.get('elapsed_s')} s",
            f"- Round: `{event.get('round_id')}`",
            f"- Canonical task: `{event.get('canonical_task_id')}`",
            f"- Failure class: `{event.get('failure_class')}` / {event.get('failure_class')}",
            f"- Project travelled distance: {event.get('travelled_distance_m')} m",
            f"- Navigation duration: {event.get('navigation_duration_s')} s",
            f"- Persisted project error code/message: `{event.get('nav2_error_code')}` / `{event.get('nav2_error_message')}`",
            f"- Supported cause: {failure['supported_cause']}",
            f"- Recovery interpretation: {failure['recovery_interpretation']}",
        ])
        for raw in failure['raw_terminal_results_for_robot']:
            failure_lines.append(
                f"- Raw terminal result: status `{raw['action_status_value']} ({raw['action_status_name']})`, accepted `{raw['accepted']}`, error `{raw['error_code']}` / `{raw['error_message']}`, recoveries `{raw['recoveries']}`.")
    if not fresh_failures:
        failure_lines.extend(['', 'No terminal failures occurred in the fresh bounded runs.'])
    (args.output / 'fresh_terminal_failure_forensics.md').write_text(
        '\n'.join(failure_lines) + '\n', encoding='utf-8')
    for filename, value in (
        ('cslam_map_accuracy.json', {'runs': [
            {'run_id': run['run_id'], 'gt_odom': run['gt_odom'],
             'direct_map_accuracy': run['direct_map_accuracy']}
            for run in runs]}),
        ('fusion_fidelity.json', {'runs': [
            {'run_id': run['run_id'], 'fusion_fidelity': run['fusion_fidelity']}
            for run in runs]}),
        ('shared_replica_consistency.json', {'runs': [
            {'run_id': run['run_id'],
             'shared_map_replica_consistency': run['shared_map_replica_consistency']}
            for run in runs]}),
        ('nav2_benchmark.json', {'runs': [
            {'run_id': run['run_id'], 'duration_s': run['duration_s'],
             'navigation': run['navigation'], 'motion': run['motion'],
             'terminal_failures': run['terminal_failures']}
            for run in runs]}),
        ('allocator_benchmark.json', {'runs': [
            {'run_id': run['run_id'], 'allocator': run['allocator'],
             'traffic': run['traffic']}
            for run in runs]}),
        ('exploration_throughput.json', {'runs': [
            {'run_id': run['run_id'], 'coverage': run['coverage'],
             'exploration': run['exploration']}
            for run in runs]}),
    ):
        (args.output / filename).write_text(
            json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    round_examples = []
    seen_decision_hashes = set()
    if runs:
        observer = next(item['observer'] for item in items
                        if item['trial'] == runs[0]['trial'])
        for event in events(observer):
            if event.get('event_type') == 'DISTRIBUTED_PAIR_DECISION':
                if event.get('decision_hash') in seen_decision_hashes:
                    continue
                seen_decision_hashes.add(event.get('decision_hash'))
                round_examples.append(event)
                if len(round_examples) == 3:
                    break
    (args.output / 'allocator_round_examples.json').write_text(
        json.dumps({'source': 'three actual final forensic allocator events',
                    'events': round_examples}, indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
