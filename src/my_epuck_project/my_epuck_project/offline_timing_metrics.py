"""Authoritative offline idle and timestamp-join calculations.

The interval classifier is the former ``forensic_exact_avoidable_idle``
implementation, extracted so the command-line audit and the raw-bag evaluator
use one definition.  Inputs are already-parsed raw protocol records; this
module never changes runtime decisions and never treats absent evidence as a
zero.
"""

from __future__ import annotations

import math
import re


def _finite_path(candidate):
    state = candidate.get('reachability_state')
    length = candidate.get('path_length_m', candidate.get('local_path_length_m'))
    return state in (1, 'REACHABLE', 'reachable') and length is not None and \
        math.isfinite(float(length)) and float(length) >= 0.0


def _new_state():
    return {
        'batch_time': None, 'candidate_count': 0,
        'candidate_source_healthy': None, 'nav2_healthy': None,
        'tf_healthy': None, 'goal_active': False, 'recovery': 0,
        'traffic': False, 'safety': False,
    }


def _recovery_value(event):
    message = str(event.get('message', ''))
    match = re.search(r'->\s*(\d+)', message)
    return int(match.group(1)) if match else None


def _update(state, event):
    kind = str(event.get('event_type', ''))
    if kind == 'CANDIDATE_BATCH_RECEIVED':
        candidates = event.get('candidates') or []
        if 'candidate_count' in event:
            state['candidate_count'] = int(event.get('candidate_count') or 0)
        else:
            state['candidate_count'] = sum(
                1 for candidate in candidates if _finite_path(candidate))
        state['batch_time'] = float(event.get(
            'elapsed_s', event.get('sim_time_s', 0.0)) or 0.0)
        state['candidate_source_healthy'] = True
    elif kind == 'TOPIC_STALE' and str(event.get('topic_name', '')).endswith(
            'frontier_candidates'):
        state['candidate_source_healthy'] = False
    elif kind == 'DISTRIBUTED_STATUS':
        # A status heartbeat can report the coordinator's conservative source
        # flag while a newer candidate batch is already present.  Direct
        # candidate evidence wins for source freshness; TOPIC_STALE remains
        # the invalidation event.  Health/TF flags are still taken from the
        # status heartbeat because no equivalent per-candidate field exists.
        for name in ('nav2_healthy', 'tf_healthy'):
            if name in event and event[name] is not None:
                state[name] = bool(event[name])
        if 'local_nav_goal_active' in event:
            state['goal_active'] = bool(event['local_nav_goal_active'])
    elif kind == 'NAV_GOAL_SENT':
        state['goal_active'] = True
    elif kind in ('NAVIGATION_SUCCEEDED', 'NAVIGATION_FAILED',
                  'NAVIGATION_CANCELED', 'NAVIGATION_CANCELLED',
                  'NAVIGATION_TIMEOUT'):
        state['goal_active'] = False
    elif kind == 'RECOVERY_COUNT_CHANGED':
        value = _recovery_value(event)
        if value is not None:
            state['recovery'] = value
    elif kind == 'STUCK_STARTED':
        state['recovery'] = max(1, state['recovery'])
    elif kind == 'STUCK_CLEARED':
        state['recovery'] = 0


def _classification(state, now, ready_sim, stale_s):
    current_batch = (state['batch_time'] is not None and
                     now - state['batch_time'] <= stale_s and
                     state['candidate_count'] > 0)
    if state['traffic'] or state['safety'] or state['recovery'] > 0:
        return 'ALLOWED_BLOCKED'
    if now < ready_sim:
        return 'WORK_UNAVAILABLE'
    if (current_batch and state['candidate_source_healthy'] is True and
            state['nav2_healthy'] is True and state['tf_healthy'] is True):
        return 'FEASIBLE_WORK_AVAILABLE'
    return 'WORK_UNAVAILABLE'


def _percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(math.ceil(
        fraction * len(ordered))) - 1))
    return ordered[index]


def _merge_intervals(intervals):
    """Merge adjacent intervals using the established idle semantics."""
    merged = []
    for item in intervals:
        if (merged and merged[-1]['classification'] == item['classification']
                and abs(merged[-1]['end_s'] - item['start_s']) <= 1.0e-6):
            merged[-1]['end_s'] = item['end_s']
            merged[-1]['duration_s'] = (
                merged[-1]['end_s'] - merged[-1]['start_s'])
            merged[-1]['candidate_count'] = max(
                merged[-1]['candidate_count'], item['candidate_count'])
            merged[-1]['goal_active'] = item['goal_active']
        else:
            merged.append(dict(item))
    return merged


def analyze_event_records(events, ready_sim=0.0, end_sim=None,
                          robots=None, run_label=None):
    """Apply the exact avoidable-idle state machine to parsed event records."""
    events = list(events or [])
    if robots is None:
        robots = sorted({str(event.get('robot_id', event.get('robot', '')))
                         for event in events
                         if event.get('robot_id', event.get('robot')) in
                         ('robot1', 'robot2')})
    else:
        robots = [str(robot) for robot in robots]
    if end_sim is None:
        times = [float(event.get('elapsed_s', event.get('sim_time_s', 0.0)) or 0.0)
                 for event in events]
        end_sim = max(times, default=float(ready_sim or 0.0))
    ready_sim = float(ready_sim or 0.0)
    end_sim = float(end_sim or 0.0)
    by_robot = {
        robot: [event for event in events
                if str(event.get('robot_id', event.get('robot', ''))) == robot]
        for robot in robots
    }
    result = {
        'run': run_label,
        'ready_sim_s': ready_sim,
        'end_sim_s': end_sim,
        'robots': {},
        'method': {
            'candidate_source': 'CANDIDATE_BATCH_RECEIVED',
            'path_evidence': 'candidate reachability_state and finite path_length_m',
            'freshness_window_s': 5.0,
            'unknown_or_missing_evidence_is_not_feasible': True,
            'time_basis': 'simulation/header time',
        },
    }
    stale_s = 5.0
    for robot, robot_events in by_robot.items():
        timeline = sorted(
            robot_events,
            key=lambda event: (
                float(event.get('elapsed_s', event.get('sim_time_s', 0.0)) or 0.0),
                int(event.get('event_sequence', 0) or 0)))
        times = sorted({ready_sim, end_sim} | {
            float(event.get('elapsed_s', event.get('sim_time_s', 0.0)) or 0.0)
            for event in timeline
            if ready_sim <= float(event.get(
                'elapsed_s', event.get('sim_time_s', 0.0)) or 0.0) <= end_sim})
        state = _new_state()
        event_index = 0
        segments = []
        for start, stop in zip(times, times[1:]):
            while (event_index < len(timeline) and
                   float(timeline[event_index].get(
                       'elapsed_s', timeline[event_index].get(
                           'sim_time_s', 0.0)) or 0.0) <= start):
                _update(state, timeline[event_index])
                event_index += 1
            if stop <= start:
                continue
            classification = _classification(state, start, ready_sim, stale_s)
            segments.append({
                'start_s': start, 'end_s': stop,
                'duration_s': stop - start, 'classification': classification,
                'candidate_count': state['candidate_count'],
                'goal_active': state['goal_active'],
            })
        segments = _merge_intervals(segments)
        feasible = [item for item in segments
                    if item['classification'] == 'FEASIBLE_WORK_AVAILABLE']
        idle = [item for item in feasible if not item['goal_active']]
        feasible_s = sum(item['duration_s'] for item in feasible)
        idle_s = sum(item['duration_s'] for item in idle)
        productive_s = max(0.0, feasible_s - idle_s)
        threshold_counts = {
            f'greater_than_{threshold:g}_s': sum(
                1 for item in idle if item['duration_s'] > threshold)
            for threshold in (1.0, 2.0, 5.0, 10.0)}
        terminals = [event for event in timeline if event.get('event_type') in (
            'NAVIGATION_SUCCEEDED', 'NAVIGATION_FAILED')]
        dispatches = [event for event in timeline
                      if event.get('event_type') == 'NAV_GOAL_SENT']
        latencies = []
        for terminal in terminals:
            t0 = float(terminal.get(
                'elapsed_s', terminal.get('sim_time_s', 0.0)) or 0.0)
            following = [event for event in dispatches
                         if float(event.get(
                             'elapsed_s', event.get('sim_time_s', 0.0)) or 0.0) > t0]
            if not following:
                continue
            t1 = float(following[0].get(
                'elapsed_s', following[0].get('sim_time_s', 0.0)) or 0.0)
            covered = sum(item['duration_s'] for item in feasible
                          if item['start_s'] >= t0 and item['end_s'] <= t1)
            if abs(covered - (t1 - t0)) <= 0.11:
                latencies.append(t1 - t0)
        result['robots'][robot] = {
            'feasible_work_seconds': round(feasible_s, 6),
            'avoidable_idle_seconds': round(idle_s, 6),
            'avoidable_idle_fraction': (idle_s / feasible_s
                                        if feasible_s else None),
            'productive_engagement_seconds': round(productive_s, 6),
            'productive_engagement_fraction': (
                productive_s / feasible_s if feasible_s else None),
            'allowed_blocked_seconds': round(sum(
                item['duration_s'] for item in segments
                if item['classification'] == 'ALLOWED_BLOCKED'), 6),
            'work_unavailable_seconds': round(sum(
                item['duration_s'] for item in segments
                if item['classification'] == 'WORK_UNAVAILABLE'), 6),
            'longest_avoidable_idle_s': max(
                (item['duration_s'] for item in idle), default=0.0),
            **threshold_counts,
            'feasible_terminal_to_next_useful_dispatch_s': {
                'count': len(latencies),
                'median_s': (float(sorted(latencies)[len(latencies) // 2])
                             if latencies else None),
                'p95_s': _percentile(latencies, .95),
                'max_s': max(latencies, default=None),
            },
            'segments': segments,
        }
    return result


def _times(values):
    return [float(value.get('sim_time_s')) for value in values
            if value.get('sim_time_s') is not None]


def _latency_summary(values, reason=None):
    cleaned = []
    for value in values:
        try:
            value = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(value) and value >= 0.0:
            cleaned.append(value)
    values = sorted(cleaned)
    result = {'count': len(values), 'values_s': values}
    if values:
        result.update({
            'p50_s': values[len(values) // 2],
            'p95_s': _percentile(values, .95),
            'max_s': values[-1],
        })
    elif reason:
        result['unavailable_reason'] = reason
    return result


def _next_latencies(starts, ends, match=None):
    values = []
    for start in sorted(starts, key=lambda item: item[0]):
        candidates = [item for item in ends if item[0] > start[0] and
                     (match is None or match(start[1], item[1]))]
        if candidates:
            values.append(candidates[0][0] - start[0])
    return values


def _same_round(left, right):
    left_round = str(left.get('round_id', ''))
    right_round = str(right.get('round_id', ''))
    if left_round and right_round:
        return left_round == right_round
    return True


def protocol_snappiness(protocol, ready_sim=None, end_sim=None):
    """Compute deterministic joins from the parsed protocol streams."""
    events = list(protocol.get('cooperation_events', []))
    dispatches = list(protocol.get('dispatches', []))
    terminals = list(protocol.get('navigation_terminals', []))
    dispatch_pairs = [(float(item['sim_time_s']), item) for item in dispatches
                      if item.get('sim_time_s') is not None]
    terminal_pairs = [(float(item['sim_time_s']), item) for item in terminals
                      if item.get('sim_time_s') is not None]
    dispatch_to_terminal = []
    for start, dispatch in sorted(dispatch_pairs):
        matches = [end for end in terminal_pairs if end[0] > start and
                   end[1].get('robot') == dispatch.get('robot') and
                   end[1].get('task_id') == dispatch.get('task_id')]
        if matches:
            dispatch_to_terminal.append(matches[0][0] - start)
    terminal_to_dispatch = _next_latencies(
        terminal_pairs, dispatch_pairs,
        lambda left, right: left.get('robot') == right.get('robot'))

    agreement_types = {'DECISION_AGREED', 'CONTINUATION_DECISION_AGREED'}
    agreements = [(float(item['sim_time_s']), item) for item in events
                  if item.get('event_type') in agreement_types and
                  item.get('sim_time_s') is not None]
    agreement_to_goal = _next_latencies(
        agreements, dispatch_pairs,
        lambda left, right: left.get('robot') == right.get('robot') and
        _same_round(left, right))

    candidate = {}
    tasks = {}
    for item in protocol.get('generation_records', []):
        robot = item.get('robot')
        generation = int(item.get('candidate_generation_id', 0) or 0)
        if not robot or not generation:
            continue
        target = candidate if item.get('kind') == 'candidate' else tasks
        target[(robot, generation)] = item
    generation_values = [
        float(tasks[key]['sim_time_s']) - float(candidate[key]['sim_time_s'])
        for key in sorted(set(candidate) & set(tasks))
        if candidate[key].get('sim_time_s') is not None and
        tasks[key].get('sim_time_s') is not None and
        float(tasks[key]['sim_time_s']) >= float(candidate[key]['sim_time_s'])]

    snapshot_times = {
        (item.get('robot'), int(item.get('source_snapshot_epoch', 0) or 0)):
        item for item in protocol.get('generation_records', [])
        if item.get('kind') == 'task_snapshot' and
        item.get('source_snapshot_epoch') is not None and
        item.get('robot')
    }
    bid_values = []
    for bid in protocol.get('bid_records', []):
        key = (bid.get('robot'), int(bid.get('source_snapshot_epoch', 0) or 0))
        snapshot = snapshot_times.get(key)
        if snapshot and bid.get('sim_time_s') is not None:
            delta = float(bid['sim_time_s']) - float(snapshot['sim_time_s'])
            if delta >= 0.0:
                bid_values.append(delta)

    certificate_values = []
    certificate_records = protocol.get('certificate_records', [])
    for record in certificate_records:
        if record.get('sim_time_s') is None:
            continue
        starts = [(float(record['sim_time_s']), record)]
        matches = [item for item in dispatch_pairs if item[0] > starts[0][0]
                   and item[1].get('robot') == record.get('robot')
                   and _same_round(record, item[1])]
        if matches:
            certificate_values.append(matches[0][0] - starts[0][0])

    release_records = list(protocol.get('start_release_records', []))
    release_times = [float(item.get('release_sim_time_s'))
                     for item in release_records
                     if item.get('event') == 'START_RELEASE' and
                     item.get('accepted_handoff') and
                     item.get('release_sim_time_s') is not None]
    handoff_values = []
    if release_times:
        release = min(release_times)
        handoff_values = [item[0] - release for item in dispatch_pairs
                          if item[0] >= release]
    return {
        'time_basis': 'simulation/header time',
        'dispatch_to_terminal': _latency_summary(dispatch_to_terminal),
        'terminal_to_next_dispatch': _latency_summary(terminal_to_dispatch),
        'agreement_to_nav_goal_sent': _latency_summary(
            agreement_to_goal, 'no agreement and goal join'),
        'handoff_to_first_shared_goal': _latency_summary(
            [min(handoff_values)] if handoff_values else [],
            'no accepted handoff followed by a dispatch'),
        'candidate_generation': _latency_summary(
            generation_values, 'candidate/task generation IDs did not join'),
        'bid': _latency_summary(
            bid_values, 'no task snapshot/bid epoch join'),
        'certificate': _latency_summary(
            certificate_values, 'no certificate/dispatch join'),
        'join_counts': {
            'candidate_task_generation': len(generation_values),
            'task_bid': len(bid_values),
            'certificate_dispatch': len(certificate_values),
            'agreement_dispatch': len(agreement_to_goal),
        },
    }


def protocol_work_availability(protocol):
    """Return the raw status availability observations without inference."""
    result = {}
    for record in protocol.get('status_records', []):
        robot = record.get('robot')
        if not robot:
            continue
        result.setdefault(robot, []).append({
            key: record.get(key) for key in (
                'sim_time_s', 'feasible_work_available',
                'actionable_work_available', 'work_availability_reason',
                'detected_not_queried_count', 'actionable_reachable_count')})
    return {
        'available': bool(result),
        'robots': result,
        'missing_robot_status': [robot for robot in ('robot1', 'robot2')
                                 if robot not in result],
    }


def analyze_protocol(protocol, ready_sim=None, end_sim=None):
    """Return exact idle, work-availability, and snappiness outputs."""
    if end_sim is None:
        end_sim = max(_times(protocol.get('status_records', [])) or
                      _times(protocol.get('cooperation_events', [])) or [0.0])
    if ready_sim is None:
        releases = protocol.get('start_release_records', [])
        valid = [item.get('release_sim_time_s') for item in releases
                 if item.get('event') == 'START_RELEASE' and
                 item.get('accepted_handoff') and
                 item.get('release_sim_time_s') is not None]
        ready_sim = min(valid) if valid else 0.0
    events = []
    for item in protocol.get('generation_records', []):
        if item.get('kind') == 'candidate':
            events.append({
                'robot_id': item.get('robot'),
                'event_type': 'CANDIDATE_BATCH_RECEIVED',
                'sim_time_s': item.get('sim_time_s'),
                'elapsed_s': item.get('sim_time_s'),
                'candidate_count': item.get('candidate_count', 0),
                'event_sequence': item.get('event_sequence', 0),
            })
    for item in protocol.get('status_records', []):
        events.append({
            'robot_id': item.get('robot'),
            'event_type': 'DISTRIBUTED_STATUS',
            'sim_time_s': item.get('sim_time_s'),
            'elapsed_s': item.get('sim_time_s'),
            **{key: item.get(key) for key in (
                'local_nav_goal_active', 'nav2_healthy', 'tf_healthy',
                'candidate_source_healthy')},
        })
    for item in protocol.get('cooperation_events', []):
        events.append({
            'robot_id': item.get('robot'),
            'event_type': item.get('event_type'),
            'sim_time_s': item.get('sim_time_s'),
            'elapsed_s': item.get('sim_time_s'),
            'round_id': item.get('round_id'),
            'task_id': item.get('task_id'),
        })
    idle = analyze_event_records(events, ready_sim, end_sim,
                                 robots=('robot1', 'robot2'))
    return {
        'avoidable_idle': idle,
        'work_availability': protocol_work_availability(protocol),
        'snappiness': protocol_snappiness(protocol, ready_sim, end_sim),
    }
