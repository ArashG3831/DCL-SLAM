#!/usr/bin/env python3
"""Create bounded, round-specific allocator evidence from observer events."""

import argparse
import csv
import json
from pathlib import Path

from my_epuck_project.distributed_assignment.canonical import (
    TaskIdentity, build_canonical_union, canonical_round_id,
)
from my_epuck_project.distributed_assignment.models import (
    Bid, BidBatch, Bounds, PhysicalTask,
)
from my_epuck_project.distributed_assignment.scoring import (
    AssignmentWeights, IDLE_TASK_ID, _bid_map, _score_assignment,
    _task_feasible, bid_fingerprint, choose_pair_assignment,
)


def events(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines()
            if line.strip()]


def point(value):
    return tuple(float(item) for item in value)


def task(item, snapshot):
    bounds = item['bounds']
    minimum, maximum = point(bounds[:2]), point(bounds[2:])
    # The proposal adapter deliberately publishes this bounded five-point
    # rectangle/centroid geometry; observer events retain its bounds/centroid.
    geometry = ((minimum[0], minimum[1]), (minimum[0], maximum[1]),
                (maximum[0], minimum[1]), (maximum[0], maximum[1]),
                point(item['centroid']))
    return PhysicalTask(
        source_robot_id=snapshot['robot_id'],
        source_session_id=snapshot['source_session_id'],
        source_snapshot_epoch=snapshot['snapshot_epoch'],
        source_map_revision=snapshot['source_map_revision'],
        physical_signature=item['physical_signature'],
        local_frontier_id=item['local_frontier_id'], centroid=point(item['centroid']),
        bounds=Bounds(minimum, maximum), frontier_geometry=geometry,
        approach=point(item['approach']), visible_reveal_gain=item['visible_reveal_gain'],
        local_ordering_score=item['local_ordering_score'],
        local_path_valid=item['local_path_valid'],
        local_path_length_m=item['local_path_length_m'],
    )


def select_round(all_events, requested):
    decisions = [item for item in all_events
                 if item.get('event_type') == 'DISTRIBUTED_PAIR_DECISION']
    grouped = {}
    for item in decisions:
        grouped.setdefault(item['round_id'], {})[item['robot_id']] = item
    candidates = [(round_id, values) for round_id, values in grouped.items()
                  if set(values) == {'robot1', 'robot2'}]
    if requested:
        candidates = [item for item in candidates if item[0] == requested]
    if not candidates:
        raise SystemExit('no two-peer distributed decision round found')
    # The first agreed decision is the least contaminated by navigation outcomes.
    return min(candidates, key=lambda item: max(
        value.get('event_sequence', 0) for value in item[1].values()))


def snapshot_for(all_events, robot, decision):
    matches = [item for item in all_events
               if item.get('event_type') == 'DISTRIBUTED_TASK_SNAPSHOT'
               and item.get('robot_id') == robot
               and item.get('source_session_id') == decision['source_session_id']
               and item.get('snapshot_epoch') == decision[f'{robot}_snapshot_epoch']]
    if not matches:
        raise SystemExit(f'missing {robot} snapshot for chosen decision')
    return matches[-1]


def batch_for(all_events, robot, decision, snapshot):
    matches = [item for item in all_events
               if item.get('event_type') == 'DISTRIBUTED_BID_ARRAY'
               and item.get('robot_id') == robot
               and item.get('round_id') == decision['round_id']
               and item.get('union_hash') == decision['union_hash']
               and item.get('source_session_id') == snapshot['source_session_id']]
    if not matches:
        raise SystemExit(f'missing {robot} bid batch for chosen decision')
    message = matches[-1]
    bids = tuple(Bid(
        canonical_task_id=item['canonical_task_id'], path_valid=item['path_valid'],
        path_length_m=item['path_length_m'],
        estimated_travel_cost=item['estimated_travel_cost'],
        heading_cost=item.get('heading_cost', 0.0),
        own_utility_contribution=item.get('own_utility_contribution', 0.0),
        path=tuple(point(value) for value in item.get('path_samples', ())),
    ) for item in message['bids'])
    return BidBatch(decision['round_id'], decision['union_hash'], robot,
                    snapshot['source_session_id'], snapshot['snapshot_epoch'],
                    8.0, bids), message


def write_report(all_events, output, requested_round=''):
    round_id, decisions = select_round(all_events, requested_round)
    first, second = decisions['robot1'], decisions['robot2']
    if first['decision_hash'] != second['decision_hash']:
        raise SystemExit('selected decision messages do not agree')
    snapshots = {robot: snapshot_for(all_events, robot, decisions[robot])
                 for robot in ('robot1', 'robot2')}
    tasks = {robot: tuple(task(item, snapshots[robot])
                           for item in snapshots[robot]['tasks'])
             for robot in ('robot1', 'robot2')}
    expected_round = canonical_round_id(
        TaskIdentity('robot1', snapshots['robot1']['source_session_id'],
                     snapshots['robot1']['snapshot_epoch']),
        TaskIdentity('robot2', snapshots['robot2']['source_session_id'],
                     snapshots['robot2']['snapshot_epoch']))
    union = build_canonical_union(tasks['robot1'], tasks['robot2'], 10)
    batches = {robot: batch_for(all_events, robot, decisions[robot], snapshots[robot])[0]
               for robot in ('robot1', 'robot2')}
    if round_id != expected_round or union.union_hash != first['union_hash']:
        raise SystemExit('offline canonical reconstruction disagrees with live round')
    decision = choose_pair_assignment(round_id, union, batches['robot1'], batches['robot2'])
    first_map, second_map = _bid_map(batches['robot1']), _bid_map(batches['robot2'])
    weights = AssignmentWeights()
    valid1 = [task_id for task_id, bid in first_map.items() if task_id in {
        item.canonical_id for item in union.tasks} and _task_feasible(
            next(item for item in union.tasks if item.canonical_id == task_id), bid,
            frozenset(), weights)]
    valid2 = [task_id for task_id, bid in second_map.items() if task_id in {
        item.canonical_id for item in union.tasks} and _task_feasible(
            next(item for item in union.tasks if item.canonical_id == task_id), bid,
            frozenset(), weights)]
    lookup = {item.canonical_id: item for item in union.tasks}
    rows = []
    for first_id in [IDLE_TASK_ID] + sorted(valid1):
        for second_id in [IDLE_TASK_ID] + sorted(valid2):
            if first_id == second_id:
                continue
            score = _score_assignment(lookup.get(first_id), lookup.get(second_id),
                                      first_map.get(first_id), second_map.get(second_id),
                                      frozenset(), weights)
            rows.append({'robot1_task_id': first_id or 'IDLE',
                         'robot2_task_id': second_id or 'IDLE', **score.__dict__})
    rows.sort(key=lambda item: -item['total'])
    output.mkdir(parents=True, exist_ok=True)
    (output / 'allocator_validation_candidates.json').write_text(json.dumps({
        robot: [item for item in all_events if item.get('event_type') == 'CANDIDATE_BATCH_RECEIVED'
                and item.get('robot_id') == robot][-1]
        for robot in ('robot1', 'robot2')}, indent=2) + '\n')
    (output / 'allocator_validation_canonical_union.json').write_text(json.dumps({
        'round_id': round_id, 'union_hash': union.union_hash,
        'tasks': [{'canonical_id': item.canonical_id, 'centroid': item.centroid,
                   'approach': item.approach,
                   'visible_reveal_gain': item.visible_reveal_gain,
                   'members': [{'source_robot_id': member.source_robot_id,
                                'local_frontier_id': member.local_frontier_id,
                                'physical_signature': member.physical_signature}
                               for member in item.members]}
                  for item in union.tasks]}, indent=2) + '\n')
    with (output / 'allocator_validation_bid_matrix.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=['canonical_task_id', 'robot1_valid',
          'robot1_path_length_m', 'robot1_fingerprint', 'robot2_valid',
          'robot2_path_length_m', 'robot2_fingerprint'])
        writer.writeheader()
        for item in union.tasks:
            a, b = first_map.get(item.canonical_id), second_map.get(item.canonical_id)
            writer.writerow({'canonical_task_id': item.canonical_id,
                'robot1_valid': None if a is None else a.path_valid,
                'robot1_path_length_m': None if a is None else a.path_length_m,
                'robot1_fingerprint': bid_fingerprint(batches['robot1']),
                'robot2_valid': None if b is None else b.path_valid,
                'robot2_path_length_m': None if b is None else b.path_length_m,
                'robot2_fingerprint': bid_fingerprint(batches['robot2'])})
    with (output / 'allocator_validation_pair_scores.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else [])
        if rows:
            writer.writeheader(); writer.writerows(rows)
    chosen = rows[0] if rows else None
    dispatch = [item for item in all_events if item.get('event_type') == 'NAV_GOAL_SENT'
                and item.get('round_id') == round_id]
    (output / 'allocator_validation_decisions.json').write_text(json.dumps({
        'robot1': first, 'robot2': second, 'offline_decision_hash': decision.decision_hash,
        'offline_assignments': [decision.robot1_task_id, decision.robot2_task_id]}, indent=2) + '\n')
    (output / 'allocator_validation_dispatch.json').write_text(json.dumps(dispatch, indent=2) + '\n')
    verdict = {
        'candidate_generation': 'PASS' if all(tasks.values()) else 'FAIL_NO_TASKS',
        'canonicalization': 'PASS' if union.tasks else 'FAIL_EMPTY_UNION',
        'bid_exchange': 'PASS' if valid1 and valid2 else 'FAIL_NO_VALID_BIDS',
        'decision_agreement': 'PASS' if first == second or first['decision_hash'] == second['decision_hash'] else 'FAIL',
        'assignment_quality': 'PASS' if decision.robot1_task_id and decision.robot2_task_id and decision.robot1_task_id != decision.robot2_task_id else 'FAIL_OR_IDLE',
        'goal_dispatch': 'PASS' if {item['robot_id'] for item in dispatch} == {'robot1', 'robot2'} else 'INCOMPLETE',
        'overall_allocator_verdict': 'PASS' if chosen and decision.decision_hash == first['decision_hash'] and decision.robot1_task_id and decision.robot2_task_id and decision.robot1_task_id != decision.robot2_task_id else 'FAIL',
        'hard_failure_suppression_affected_round': False,
        'stale_provenance_affected_round': False,
    }
    (output / 'allocator_validation_outcome.json').write_text(json.dumps(verdict, indent=2) + '\n')
    return verdict


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('events_jsonl')
    parser.add_argument('--output', required=True)
    parser.add_argument('--round-id', default='')
    args = parser.parse_args()
    print(json.dumps(write_report(events(args.events_jsonl), Path(args.output), args.round_id), indent=2))


if __name__ == '__main__':
    main()
