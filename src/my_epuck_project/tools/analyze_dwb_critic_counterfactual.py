#!/usr/bin/env python3
"""Offline, bounded DWB critic-scale counterfactual analysis.

This tool deliberately never simulates trajectories or contacts ROS.  It only
re-scores the selected and best-valid-forward trajectories retained in
``dwb_stall_events.jsonl``.  The resulting decision is therefore a bounded
two-trajectory counterfactual, not a replacement for DWB's full search.
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path
from typing import Any


FORWARD_THRESHOLD_MPS = 0.026
EPSILON = 1.0e-9
BASELINE = {'path_align': 8.0, 'path_dist': 24.0, 'goal_dist': 24.0}


def full_scale_candidates():
    """The bounded, requested joint path-critic matrix."""
    pairs = ((8, 24), (6, 18), (4, 12), (3, 10), (2, 8), (2, 6),
             (1, 8), (1, 6))
    return [(f'pa{align}_pd{path}_gd24', {
        'path_align': float(align), 'path_dist': float(path), 'goal_dist': 24.0,
    }) for align, path in pairs]


def _full_records(path: Path):
    return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()
            if line.strip()]


def _winner(trajectories, scales):
    """Return all tied minimum valid candidates; never invent a tie-break."""
    scored = [(item, rescored_total(item, scales)) for item in trajectories
              if item.get('valid')]
    if not scored:
        return [], None, None
    minimum = min(value for _, value in scored)
    winners = [item for item, value in scored if abs(value - minimum) <= EPSILON]
    second = min((value for _, value in scored if value > minimum + EPSILON),
                 default=None)
    return winners, minimum, None if second is None else second - minimum


def _winner_kind(item):
    velocity = item.get('velocity') or {}
    vx, wz = float(velocity.get('linear_x_mps', 0.0)), float(velocity.get('angular_z_radps', 0.0))
    if vx < -EPSILON:
        return 'reverse'
    if vx >= FORWARD_THRESHOLD_MPS - EPSILON:
        return 'forward_executable'
    if vx > EPSILON:
        return 'forward_below_threshold'
    return 'angular_only' if abs(wz) > EPSILON else 'zero'


def write_full_report(input_dir: Path, output_dir: Path) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    frames = _full_records(input_dir / 'dwb_full_candidate_frames.jsonl')
    configurations = full_scale_candidates()
    rows, baseline_selected_minimum, baseline_unique = [], [], []
    distinct_stalls = set()
    for frame in frames:
        evaluation = frame['evaluation']
        trajectories = evaluation.get('trajectories', [])
        selected_index = evaluation.get('selected_index')
        winners, _, _ = _winner(trajectories, BASELINE)
        baseline_selected_minimum.append(any(
            item.get('trajectory_index') == selected_index for item in winners))
        baseline_unique.append(len(winners) == 1 and winners[0].get(
            'trajectory_index') == selected_index)
        if not str(frame.get('capture_reason', '')).startswith('HEALTHY_'):
            distinct_stalls.add((frame.get('robot'), frame.get('goal', {}).get('label'),
                                 tuple((x.get('name'), round(float(x.get('raw_score', 0)), 4))
                                       for x in next((t for t in trajectories if t.get('selected')), {}).get('critics', []))))
    for name, scales in configurations:
        corrected, healthy_changed, reverse, ties = set(), 0, 0, 0
        for index, frame in enumerate(frames):
            trajectories = frame['evaluation'].get('trajectories', [])
            baseline_winners, baseline_total, _ = _winner(trajectories, BASELINE)
            winners, total, margin = _winner(trajectories, scales)
            winner = winners[0] if len(winners) == 1 else None
            reason = str(frame.get('capture_reason', ''))
            if len(winners) != 1:
                ties += 1
            kind = 'tied' if winner is None else _winner_kind(winner)
            selected = next((item for item in trajectories if item.get('selected')), {})
            if reason.startswith('HEALTHY_'):
                if kind != 'forward_executable':
                    healthy_changed += 1
            elif kind == 'forward_executable':
                corrected.add((frame.get('robot'), frame.get('goal', {}).get('label'), index))
            reverse += kind == 'reverse'
            rows.append({
                'configuration': name, 'robot': frame.get('robot'),
                'capture_reason': reason, 'sim_time_s': frame.get('simulation_timestamp_s'),
                'goal_label': frame.get('goal', {}).get('label'),
                'baseline_selected_index': selected.get('trajectory_index'),
                'counterfactual_selected_index': '' if winner is None else winner.get('trajectory_index'),
                'winner_kind': kind, 'winner_vx_mps': '' if winner is None else winner.get('velocity', {}).get('linear_x_mps'),
                'winner_wz_radps': '' if winner is None else winner.get('velocity', {}).get('angular_z_radps'),
                'winner_total_score': total, 'margin_over_second': margin,
                'trajectory_count': len(trajectories), 'valid_trajectory_count': sum(t.get('valid') for t in trajectories),
                'tied_winner_indices': '|'.join(str(t.get('trajectory_index')) for t in winners),
            })
        # one result row per frame is deliberately retained above; aggregate below.
        rows.append({'configuration': name, 'robot': 'ALL', 'capture_reason': 'AGGREGATE',
                     'sim_time_s': '', 'goal_label': '', 'baseline_selected_index': '',
                     'counterfactual_selected_index': '', 'winner_kind': '', 'winner_vx_mps': '',
                     'winner_wz_radps': '', 'winner_total_score': '', 'margin_over_second': '',
                     'trajectory_count': '', 'valid_trajectory_count': '', 'tied_winner_indices': '',
                     'distinct_stall_frames_corrected': len(corrected),
                     'healthy_nonforward_count': healthy_changed, 'reverse_count': reverse,
                     'tie_count': ties})
    fields = sorted({key for row in rows for key in row})
    with (output_dir / 'dwb_full_counterfactual_results.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(rows)
    aggregates = [row for row in rows if row.get('capture_reason') == 'AGGREGATE']
    summary = {
        'schema_version': 1, 'source': str(input_dir),
        'method': 'all retained valid LocalPlanEvaluation trajectories rescored; invalid candidates remain ineligible',
        'baseline_scales': BASELINE, 'frame_count': len(frames),
        'stall_frame_count': sum(not str(f.get('capture_reason', '')).startswith('HEALTHY_') for f in frames),
        'healthy_frame_count': sum(str(f.get('capture_reason', '')).startswith('HEALTHY_') for f in frames),
        'baseline_selected_among_minimum_count': sum(baseline_selected_minimum),
        'baseline_selected_not_minimum_count': (len(baseline_selected_minimum)
                                                - sum(baseline_selected_minimum)),
        'baseline_unique_selection_reproduced_count': sum(baseline_unique),
        'baseline_tied_minimum_count': len(baseline_unique) - sum(baseline_unique),
        'distinct_stall_context_count': len(distinct_stalls),
        'configurations': aggregates,
        'limitations': ['Tied minima are reported, not arbitrarily tie-broken.',
                        'Validity is retained from Jazzy DWB publication; no invalid trajectory becomes eligible.'],
    }
    (output_dir / 'dwb_full_counterfactual_summary.json').write_text(
        json.dumps(summary, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    return summary


def _contribution(record: dict[str, Any], critic: str) -> float:
    return float(record.get('critic_contributions', {}).get(critic, 0.0))


def rescored_total(record: dict[str, Any], scales: dict[str, float]) -> float:
    """Rescore one retained trajectory by scaling its recorded contributions."""
    baseline_total = float(record['total_score'])
    replacements = (
        ('PathAlign', 'path_align'),
        ('PathDist', 'path_dist'),
        ('GoalDist', 'goal_dist'),
    )
    result = baseline_total
    for critic, key in replacements:
        old = _contribution(record, critic)
        result += old * (scales[key] / BASELINE[key] - 1.0)
    return result


def choose_retained_trajectory(
    selected: dict[str, Any], forward: dict[str, Any], scales: dict[str, float],
) -> tuple[str, float, float]:
    """Choose between retained selected and retained valid-forward records.

    Ties preserve DWB's recorded selection because LocalPlanEvaluation does
    not publish DWB's internal tie-break ordering.
    """
    selected_total = rescored_total(selected, scales)
    forward_total = rescored_total(forward, scales)
    choice = 'forward' if forward_total < selected_total - EPSILON else 'selected'
    return choice, selected_total, forward_total


def _event_record(event: dict[str, Any]) -> dict[str, Any] | None:
    dwb = event.get('dwb') or {}
    selected = dwb.get('selected') or {}
    forward = dwb.get('best_valid_forward') or {}
    if not selected or not forward:
        return None
    velocity = forward.get('velocity') or {}
    if float(velocity.get('linear_x_mps', 0.0)) + EPSILON < FORWARD_THRESHOLD_MPS:
        return None
    return {
        'event': event,
        'selected': selected,
        'forward': forward,
    }


def load_stall_records(events_path: Path) -> list[dict[str, Any]]:
    records = []
    for line in events_path.read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        record = _event_record(json.loads(line))
        if record is not None:
            records.append(record)
    return records


def scale_candidates() -> list[tuple[str, dict[str, float]]]:
    """Individual changes plus a deliberately small, interpretable set."""
    candidates: list[tuple[str, dict[str, float]]] = [('baseline', dict(BASELINE))]
    for value in (8.0, 6.0, 4.0, 2.0, 1.0, 0.0):
        scales = dict(BASELINE)
        scales['path_align'] = value
        candidates.append((f'path_align_{value:g}', scales))
    for value in (24.0, 18.0, 12.0, 8.0, 6.0, 4.0):
        scales = dict(BASELINE)
        scales['path_dist'] = value
        candidates.append((f'path_dist_{value:g}', scales))
    for value in (24.0, 30.0, 36.0, 48.0, 60.0):
        scales = dict(BASELINE)
        scales['goal_dist'] = value
        candidates.append((f'goal_dist_{value:g}', scales))
    for align, path, goal in (
        (6.0, 18.0, 24.0), (4.0, 12.0, 24.0), (2.0, 8.0, 24.0),
        (1.0, 6.0, 24.0), (0.0, 4.0, 24.0), (6.0, 18.0, 30.0),
        (4.0, 12.0, 36.0), (2.0, 8.0, 48.0), (1.0, 6.0, 48.0),
        (0.0, 4.0, 60.0),
    ):
        candidates.append((
            f'combined_pa{align:g}_pd{path:g}_gd{goal:g}',
            {'path_align': align, 'path_dist': path, 'goal_dist': goal},
        ))
    # Preserve first occurrence only; the individual lists include baseline.
    unique: list[tuple[str, dict[str, float]]] = []
    seen = set()
    for name, scales in candidates:
        signature = tuple(sorted(scales.items()))
        if signature not in seen:
            seen.add(signature)
            unique.append((name, scales))
    return unique


def counterfactual_rows(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows = []
    for name, scales in scale_candidates():
        changed = 0
        near_total = near_changed = far_total = far_changed = 0
        for record in records:
            event = record['event']
            choice, selected_total, forward_total = choose_retained_trajectory(
                record['selected'], record['forward'], scales)
            changed += choice == 'forward'
            distance = event.get('goal', {}).get('distance_to_goal_m')
            if distance is not None:
                bucket = 'near' if float(distance) <= 1.0 else 'far'
                if bucket == 'near':
                    near_total += 1
                    near_changed += choice == 'forward'
                else:
                    far_total += 1
                    far_changed += choice == 'forward'
        delta = (abs(scales['path_align'] - BASELINE['path_align']) / BASELINE['path_align']
                 + abs(scales['path_dist'] - BASELINE['path_dist']) / BASELINE['path_dist']
                 + abs(scales['goal_dist'] - BASELINE['goal_dist']) / BASELINE['goal_dist'])
        rows.append({
            'configuration': name,
            **scales,
            'retained_stall_frames': len(records),
            'changed_to_forward_count': changed,
            'changed_to_forward_percent': 100.0 * changed / len(records) if records else 0.0,
            'near_goal_count': near_total,
            'near_goal_changed_count': near_changed,
            'far_goal_count': far_total,
            'far_goal_changed_count': far_changed,
            'parameter_change_l1_relative': delta,
            'healthy_forward_preservation': 'UNAVAILABLE_NO_ALTERNATIVES_RETAINED',
            'backward_selection': 'UNAVAILABLE_NO_BACKWARD_TRAJECTORIES_RETAINED',
            'excessive_angular_selection': 'UNAVAILABLE_NO_FULL_TRAJECTORY_SET_RETAINED',
            'obstacle_invalid_selected': 0,
            'obstacle_safety_scope': 'both compared trajectories were recorded valid only',
        })
    return rows


def score_fingerprint(record: dict[str, Any]) -> tuple[float, ...]:
    """Identify repeated retained score contexts without using goal identity."""
    selected, forward = record['selected'], record['forward']
    values = [selected.get('total_score'), forward.get('total_score')]
    for critic in ('PathAlign', 'PathDist', 'GoalDist'):
        values.extend((_contribution(selected, critic), _contribution(forward, critic)))
    return tuple(round(float(value), 9) for value in values)


def _record_row(record: dict[str, Any]) -> dict[str, Any]:
    event, selected, forward = record['event'], record['selected'], record['forward']
    local = event.get('local_costmap') or {}
    goal = event.get('goal') or {}
    peer = event.get('peer') or {}
    return {
        'record_type': 'stall_detailed',
        'sequence': event.get('sequence'),
        'robot': event.get('robot'),
        'sim_time_s': event.get('sim_time_s'),
        'trigger': event.get('trigger'),
        'goal_label': goal.get('label'),
        'goal_source': goal.get('source'),
        'distance_to_goal_m': goal.get('distance_to_goal_m'),
        'remaining_path_length_m': goal.get('remaining_path_length_m'),
        'selected_vx_mps': (selected.get('velocity') or {}).get('linear_x_mps'),
        'selected_wz_radps': (selected.get('velocity') or {}).get('angular_z_radps'),
        'selected_total_score': selected.get('total_score'),
        'forward_vx_mps': (forward.get('velocity') or {}).get('linear_x_mps'),
        'forward_wz_radps': (forward.get('velocity') or {}).get('angular_z_radps'),
        'forward_total_score': forward.get('total_score'),
        'fastest_valid_forward': 'UNAVAILABLE_ONLY_BEST_FORWARD_RETAINED',
        'valid_trajectory_count': (event.get('dwb') or {}).get('valid_count'),
        'forward_valid_count': (event.get('dwb') or {}).get('forward_valid_count'),
        'selected_obstacle_contribution': _contribution(selected, 'BaseObstacle'),
        'forward_obstacle_contribution': _contribution(forward, 'BaseObstacle'),
        'start_cell_cost': local.get('start_cell_cost'),
        'peer_distance_m': peer.get('distance_m'),
        'selected_path_align': _contribution(selected, 'PathAlign'),
        'forward_path_align': _contribution(forward, 'PathAlign'),
        'selected_path_dist': _contribution(selected, 'PathDist'),
        'forward_path_dist': _contribution(forward, 'PathDist'),
        'selected_goal_dist': _contribution(selected, 'GoalDist'),
        'forward_goal_dist': _contribution(forward, 'GoalDist'),
    }


def _healthy_command_rows(timeseries_path: Path, goals_path: Path) -> list[dict[str, Any]]:
    """Retain a balanced command-only healthy sample, with limitations explicit."""
    successful = set()
    with goals_path.open(encoding='utf-8', newline='') as stream:
        for row in csv.DictReader(stream):
            if row.get('result_status') == '4':
                successful.add((row.get('robot'), row.get('label')))
    groups: dict[tuple[str, str], list[dict[str, str]]] = {}
    with timeseries_path.open(encoding='utf-8', newline='') as stream:
        for row in csv.DictReader(stream):
            key = (row.get('robot'), row.get('goal_label'))
            if key not in successful or row.get('active_goal') != '1':
                continue
            if float(row.get('dwb_controller_linear_mps') or 0.0) + EPSILON < FORWARD_THRESHOLD_MPS:
                continue
            groups.setdefault(key, []).append(row)
    result = []
    for (robot, label), rows in sorted(groups.items()):
        # At most four evenly spaced examples per successful goal.
        indices = sorted({round(i * (len(rows) - 1) / min(3, len(rows) - 1))
                          for i in range(min(4, len(rows)))}) if len(rows) > 1 else [0]
        for index in indices:
            row = rows[index]
            result.append({
                'record_type': 'healthy_command_only', 'sequence': '', 'robot': robot,
                'sim_time_s': row.get('sim_time_s'), 'trigger': 'HEALTHY_FORWARD_SAMPLE',
                'goal_label': label, 'goal_source': row.get('goal_kind'),
                'distance_to_goal_m': row.get('goal_remaining_distance_m'),
                'remaining_path_length_m': row.get('goal_path_length_m'),
                'selected_vx_mps': row.get('dwb_controller_linear_mps'),
                'selected_wz_radps': row.get('dwb_controller_angular_radps'),
                'selected_total_score': 'UNAVAILABLE', 'forward_vx_mps': 'UNAVAILABLE',
                'forward_wz_radps': 'UNAVAILABLE', 'forward_total_score': 'UNAVAILABLE',
                'fastest_valid_forward': 'UNAVAILABLE', 'valid_trajectory_count': 'UNAVAILABLE',
                'forward_valid_count': 'UNAVAILABLE', 'selected_obstacle_contribution': 'UNAVAILABLE',
                'forward_obstacle_contribution': 'UNAVAILABLE', 'start_cell_cost': row.get('start_cost'),
                'peer_distance_m': row.get('peer_distance_m'), 'selected_path_align': 'UNAVAILABLE',
                'forward_path_align': 'UNAVAILABLE', 'selected_path_dist': 'UNAVAILABLE',
                'forward_path_dist': 'UNAVAILABLE', 'selected_goal_dist': 'UNAVAILABLE',
                'forward_goal_dist': 'UNAVAILABLE',
            })
    return result


def write_report(input_dir: Path, output_dir: Path) -> dict[str, Any]:
    if (input_dir / 'dwb_full_candidate_frames.jsonl').is_file():
        return write_full_report(input_dir, output_dir)
    records = load_stall_records(input_dir / 'dwb_stall_events.jsonl')
    output_dir.mkdir(parents=True, exist_ok=True)
    dataset = [_record_row(record) for record in records]
    dataset.extend(_healthy_command_rows(
        input_dir / 'controller_pipeline_timeseries.csv', input_dir / 'goal_timeline.csv'))
    columns = list(dataset[0]) if dataset else []
    with (output_dir / 'dwb_counterfactual_dataset.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(dataset)
    rows = counterfactual_rows(records)
    ranked = sorted(rows, key=lambda row: (-row['changed_to_forward_count'],
                                            row['parameter_change_l1_relative'], row['configuration']))
    fingerprints = {score_fingerprint(record) for record in records}
    for row in ranked:
        changed_fingerprints = set()
        scales = {key: row[key] for key in BASELINE}
        for record in records:
            choice, _, _ = choose_retained_trajectory(
                record['selected'], record['forward'], scales)
            if choice == 'forward':
                changed_fingerprints.add(score_fingerprint(record))
        row['unique_score_context_count'] = len(fingerprints)
        row['unique_score_contexts_changed'] = len(changed_fingerprints)
    with (output_dir / 'dwb_critic_counterfactual.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else [])
        if rows:
            writer.writeheader()
            writer.writerows(rows)
    summary = {
        'source': str(input_dir),
        'method': ('offline rescore of each retained DWB selected trajectory versus its '
                   'retained best-valid-forward trajectory; no trajectory simulation'),
        'baseline_scales': BASELINE,
        'forward_threshold_mps': FORWARD_THRESHOLD_MPS,
        'detailed_stall_evaluations': len(records),
        'healthy_forward_command_only_samples': sum(
            row['record_type'] == 'healthy_command_only' for row in dataset),
        'limitations': [
            'The capture did not retain every trajectory, only selected and best-valid-forward.',
            'Healthy frames have commands but not per-critic alternatives, so preservation cannot be rescored.',
            'Fastest forward, backward, and excessive-angular alternatives were not retained.',
            'Only already-valid selected/forward trajectories are compared; this is not a safety proof.',
            'Near-goal means remaining goal distance <= 1.0 m.',
        ],
        'ranked_configurations': ranked,
    }
    (output_dir / 'dwb_critic_counterfactual_summary.json').write_text(
        json.dumps(summary, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input_dir', type=Path, help='diagnostic result directory')
    parser.add_argument('--output-dir', type=Path, default=None)
    args = parser.parse_args()
    output_dir = args.output_dir or args.input_dir / 'dwb_critic_counterfactual'
    summary = write_report(args.input_dir, output_dir)
    print(json.dumps({
        'output_dir': str(output_dir),
        'detailed_stall_evaluations': summary.get('detailed_stall_evaluations',
                                                   summary.get('stall_frame_count', 0)),
        'healthy_forward_command_only_samples': summary.get(
            'healthy_forward_command_only_samples', summary.get('healthy_frame_count', 0)),
        'top_configuration': (summary.get('ranked_configurations') or
                              summary.get('configurations') or [None])[0],
    }, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
