#!/usr/bin/env python3
"""Export one row per unique replicated decision for an audit campaign.

Historical event files predate the solver diagnostics now emitted in
``diagnostics_json``.  Missing pre-solver rejection counters are therefore
reported as ``NOT_RECORDED`` rather than guessed.
"""

import argparse
import csv
import glob
import json
import math
import re
from pathlib import Path


FIELDS = [
    'round_id', 'union_hash', 'decision_hash',
    'robot1_snapshot_task_count', 'robot2_snapshot_task_count',
    'canonical_union_task_count', 'robot1_bid_count', 'robot2_bid_count',
    'robot1_valid_bid_count', 'robot2_valid_bid_count',
    'reachable_by_both_count', 'reachable_only_robot1_count',
    'reachable_only_robot2_count', 'rejected_by_task_equivalence',
    'rejected_by_freshness', 'rejected_by_failure_suppression',
    'rejected_by_active_task_reservation', 'rejected_by_gain_threshold',
    'rejected_by_local_score_threshold', 'rejected_by_path_backtracking',
    'best_non_idle_assignment', 'best_non_idle_utility',
    'best_non_idle_score_components', 'idle_idle_utility',
    'final_assignment', 'idle_reason', 'availability_reason',
]


def _events(campaign: Path):
    files = sorted(campaign.glob('attempts/*/observer/*/events.jsonl'))
    if not files:
        raise SystemExit(f'no observer events found below {campaign}')
    return files[0], [json.loads(line) for line in files[0].read_text().splitlines()]


def _union_counts(campaign: Path):
    counts = {}
    for filename in glob.glob(str(campaign / 'attempts/*/ros_logs/*.log')):
        for line in Path(filename).read_text(errors='ignore').splitlines():
            if 'CANONICAL_UNION' not in line:
                continue
            match = re.search(r'round=([0-9a-f]+).*?tasks=([^\s]*)', line)
            if match:
                counts[match.group(1)] = len(match.group(2).split(',')) if match.group(2) else 0
    return counts


def _best_single_score(tasks):
    best = None
    for task in tasks:
        gain = max(0.0, min(1.0, float(task.get('visible_reveal_gain', 0.0)) / 5.0))
        path = max(0.0, min(1.0, float(task.get('local_path_length_m', 0.0)) / 12.0))
        score = 3.0 * gain - path
        if best is None or score > best[0]:
            best = (score, gain, path, task.get('physical_signature', ''))
    return best


def export(campaign: Path, output: Path):
    _, events = _events(campaign)
    unions = _union_counts(campaign)
    snapshots = {}
    bids = {}
    decisions = {}
    for event in events:
        kind = event.get('event_type')
        if kind == 'DISTRIBUTED_TASK_SNAPSHOT':
            snapshots[(event.get('robot_id'), event.get('snapshot_epoch'))] = event
        elif kind == 'DISTRIBUTED_BID_ARRAY':
            bids.setdefault(event.get('round_id'), {})[event.get('robot_id')] = event
        elif kind == 'DISTRIBUTED_PAIR_DECISION':
            decisions.setdefault((event.get('round_id'), event.get('decision_hash')), event)

    rows = []
    for decision in sorted(decisions.values(), key=lambda item: item.get('elapsed_s', 0.0)):
        round_id = decision.get('round_id', '')
        first = snapshots.get(('robot1', decision.get('robot1_snapshot_epoch')), {})
        second = snapshots.get(('robot2', decision.get('robot2_snapshot_epoch')), {})
        first_tasks = first.get('tasks', [])
        second_tasks = second.get('tasks', [])
        round_bids = bids.get(round_id, {})
        first_bids = round_bids.get('robot1', {}).get('bids', [])
        second_bids = round_bids.get('robot2', {}).get('bids', [])
        first_valid = [item for item in first_bids if item.get('path_valid')]
        second_valid = [item for item in second_bids if item.get('path_valid')]
        first_ids = {item.get('canonical_task_id') for item in first_valid}
        second_ids = {item.get('canonical_task_id') for item in second_valid}
        union_count = unions.get(round_id, 'NOT_RECORDED')
        best = _best_single_score(first_tasks + second_tasks)
        selected_first = decision.get('robot1_task') or 'IDLE'
        selected_second = decision.get('robot2_task') or 'IDLE'
        idle = selected_first == 'IDLE' and selected_second == 'IDLE'
        if not idle:
            idle_reason = 'NOT_APPLICABLE'
        elif union_count == 0:
            idle_reason = 'NO_TASKS'
        elif not first_valid and not second_valid:
            idle_reason = 'NO_VALID_BIDS'
        elif best is not None and best[0] < 0.0:
            idle_reason = 'NON_IDLE_UTILITY_BELOW_IDLE'
        else:
            idle_reason = 'INSUFFICIENT_HISTORICAL_DIAGNOSTICS'
        if first_valid and second_valid:
            availability = 'BOTH_ROBOTS_REACHABLE'
        elif first_valid:
            availability = 'ONLY_ROBOT1_REACHABLE'
        elif second_valid:
            availability = 'ONLY_ROBOT2_REACHABLE'
        else:
            availability = 'NO_VALID_BIDS'
        rows.append({
            'round_id': round_id,
            'union_hash': decision.get('union_hash', ''),
            'decision_hash': decision.get('decision_hash', ''),
            'robot1_snapshot_task_count': len(first_tasks),
            'robot2_snapshot_task_count': len(second_tasks),
            'canonical_union_task_count': union_count,
            'robot1_bid_count': len(first_bids),
            'robot2_bid_count': len(second_bids),
            'robot1_valid_bid_count': len(first_valid),
            'robot2_valid_bid_count': len(second_valid),
            'reachable_by_both_count': len(first_ids & second_ids),
            'reachable_only_robot1_count': len(first_ids - second_ids),
            'reachable_only_robot2_count': len(second_ids - first_ids),
            'rejected_by_task_equivalence': 'NOT_RECORDED',
            'rejected_by_freshness': 'NOT_RECORDED',
            'rejected_by_failure_suppression': 'NOT_RECORDED',
            'rejected_by_active_task_reservation': 'NOT_RECORDED',
            'rejected_by_gain_threshold': 'NOT_RECORDED',
            'rejected_by_local_score_threshold': 'NOT_RECORDED',
            'rejected_by_path_backtracking': 'NOT_RECORDED',
            'best_non_idle_assignment': 'HISTORICAL_SINGLE_TASK_ESTIMATE',
            'best_non_idle_utility': 'NOT_RECORDED' if best is None else round(best[0], 6),
            'best_non_idle_score_components': 'NOT_RECORDED' if best is None else json.dumps({'gain': round(best[1], 6), 'path': round(best[2], 6), 'task_signature': best[3]}, sort_keys=True),
            'idle_idle_utility': 0.0,
            'final_assignment': f'{selected_first}|{selected_second}',
            'idle_reason': idle_reason,
            'availability_reason': availability,
        })
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    return len(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('campaign', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(f'wrote {export(args.campaign, args.output)} rows to {args.output}')


if __name__ == '__main__':
    main()
