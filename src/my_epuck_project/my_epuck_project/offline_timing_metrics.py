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
from bisect import bisect_left

from .distributed_assignment.models import (
    Bid,
    BidBatch,
    Bounds,
    CanonicalTask,
    CanonicalUnion,
)
from .minimal_frontier_selection import choose_assignment


def _simulation_time_for_bag_timestamp(bag_timestamp_ns, clock_samples):
    """Map a recorded bag timestamp to simulation time using ``/clock``."""
    try:
        bag_timestamp_ns = int(bag_timestamp_ns)
    except (TypeError, ValueError):
        return None
    samples = sorted((int(bag), float(sim)) for bag, sim in (clock_samples or ()))
    if not samples:
        return None
    position = bisect_left([bag for bag, _ in samples], bag_timestamp_ns)
    if position == 0:
        return samples[0][1]
    if position == len(samples):
        return samples[-1][1]
    before_bag, before_sim = samples[position - 1]
    after_bag, after_sim = samples[position]
    if after_bag == before_bag:
        return after_sim
    fraction = (bag_timestamp_ns - before_bag) / (after_bag - before_bag)
    return before_sim + fraction * (after_sim - before_sim)


def native_timing_events(navigation_action_replay, clock_samples):
    """Convert retained native Nav2 evidence into timing-state events.

    The replay is an offline summary of the recorded action status topics.  A
    successful recorded plan is direct planner/transform evidence; a native
    NavigateToPose status is direct execution evidence.  No ROS is imported.
    """
    replay = navigation_action_replay or {}
    if not replay.get('complete') or replay.get('missing_required_status_topics'):
        return []
    plan_times = {}
    for plan in replay.get('plan_records') or ():
        if not plan.get('valid'):
            continue
        stamp = _simulation_time_for_bag_timestamp(
            plan.get('bag_timestamp_ns'), clock_samples)
        robot = str(plan.get('robot_id', ''))
        if stamp is not None and robot in ('robot1', 'robot2'):
            plan_times.setdefault(robot, []).append(stamp)
    for values in plan_times.values():
        values.sort()
    terminal_statuses = {'SUCCEEDED', 'ABORTED', 'CANCELED', 'CANCELLED'}
    result = []
    for index, transition in enumerate(replay.get('status_transitions') or ()):
        action = str(transition.get('action', ''))
        status = str(transition.get('status_name', ''))
        if action not in ('COMPUTE_PATH_TO_POSE', 'NAVIGATE_TO_POSE'):
            continue
        if action == 'COMPUTE_PATH_TO_POSE' and status not in ('EXECUTING', 'SUCCEEDED'):
            continue
        if action == 'NAVIGATE_TO_POSE' and status not in ('EXECUTING', *terminal_statuses):
            continue
        stamp = _simulation_time_for_bag_timestamp(
            transition.get('bag_timestamp_ns'), clock_samples)
        robot = str(transition.get('robot_id', ''))
        if stamp is None or robot not in ('robot1', 'robot2'):
            continue
        has_prior_plan = any(value <= stamp + 1.0e-6
                             for value in plan_times.get(robot, ()))
        event = {
            'robot_id': robot,
            'elapsed_s': stamp,
            'event_sequence': 1_000_000 + index,
            'direct_nav2_healthy': True,
            'direct_tf_healthy': has_prior_plan,
        }
        if action == 'COMPUTE_PATH_TO_POSE':
            event['event_type'] = 'NATIVE_PLANNER_EVIDENCE'
        elif status == 'EXECUTING':
            event['event_type'] = 'NATIVE_NAVIGATION_EXECUTING'
            event['native_goal_active'] = True
        else:
            event['event_type'] = 'NATIVE_NAVIGATION_TERMINAL'
            event['native_goal_active'] = False
        result.append(event)
    return result


def merge_native_timing_events(events, navigation_action_replay=None,
                               clock_samples=None):
    """Add native action evidence without changing the legacy event records."""
    merged = list(events or [])
    merged.extend(native_timing_events(navigation_action_replay, clock_samples))
    return sorted(merged, key=lambda event: (
        float(event.get('elapsed_s', event.get('sim_time_s', 0.0)) or 0.0),
        int(event.get('event_sequence', 0) or 0)))


def _record_time(event):
    return float(event.get('elapsed_s', event.get('sim_time_s', 0.0)) or 0.0)


def _point(value, fallback=(0.0, 0.0)):
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        try:
            return float(value[0]), float(value[1])
        except (TypeError, ValueError):
            pass
    return fallback


def _task_model(record, task_id):
    approach = _point(record.get('approach'), _point(record.get('centroid')))
    centroid = _point(record.get('centroid'), approach)
    bounds = record.get('bounds')
    if isinstance(bounds, (list, tuple)) and len(bounds) >= 4:
        minimum = (float(bounds[0]), float(bounds[1]))
        maximum = (float(bounds[2]), float(bounds[3]))
    else:
        minimum = centroid
        maximum = centroid
    return CanonicalTask(
        canonical_id=str(task_id),
        members=(),
        centroid=centroid,
        bounds=Bounds(minimum, maximum),
        approach=approach,
        approach_yaw=0.0,
        frontier_geometry=(),
        visible_cells=(),
        visible_bounds=None,
        visible_reveal_gain=float(record.get('visible_reveal_gain', 0.0) or 0.0),
    )


def _bid_model(record):
    path = tuple(_point(point) for point in record.get('path_samples', ()))
    return Bid(
        canonical_task_id=str(record.get('canonical_task_id', '')),
        path_valid=bool(record.get('path_valid', False)),
        path_length_m=float(record.get('path_length_m', 0.0) or 0.0),
        estimated_travel_cost=float(record.get('estimated_travel_cost', 0.0) or 0.0),
        heading_cost=float(record.get('heading_cost', 0.0) or 0.0),
        own_utility_contribution=float(record.get('own_utility_contribution', 0.0) or 0.0),
        path=path,
    )


def _batch_model(event):
    union_hash = str(event.get('union_hash', ''))
    return BidBatch(
        round_id=str(event.get('round_id') or union_hash),
        union_hash=union_hash,
        source_robot_id=str(event.get('robot_id', '')),
        source_session_id=str(event.get('source_session_id', '')),
        source_snapshot_epoch=int(event.get('source_snapshot_epoch', 0) or 0),
        validity_s=0.0,
        bids=tuple(_bid_model(bid) for bid in event.get('bids', ())),
    )


def derive_minimal_assignment_events(events):
    """Reconstruct per-robot legal assignment state from recorded bid/status data.

    Minimal allocator status and bid arrays are the authoritative current-state
    evidence.  This helper deliberately emits no assignment when the two
    complete vectors or their union context do not match.
    """
    ordered = sorted(events or (), key=lambda event: (
        _record_time(event), int(event.get('event_sequence', 0) or 0)))
    batches = {}
    statuses = {}
    task_history = {}
    emitted = {'robot1': None, 'robot2': None}
    output = list(events or ())

    def current_assignment():
        first, second = batches.get('robot1'), batches.get('robot2')
        if first is None or second is None:
            return None, 'NO_CURRENT_COMPLETE_PAIR'
        if not first.union_hash or first.union_hash != second.union_hash:
            return None, 'NO_CURRENT_COMPLETE_PAIR'
        ids1 = {bid.canonical_task_id for bid in first.bids}
        ids2 = {bid.canonical_task_id for bid in second.bids}
        if not ids1 or ids1 != ids2:
            return None, 'INCOMPLETE_OR_MISMATCHED_VECTOR'
        union_hash = first.union_hash
        for status in statuses.values():
            status_hash = str(status.get('union_hash', ''))
            if status_hash and status_hash != union_hash:
                return None, 'STATUS_CONTEXT_MISMATCH'
        models = []
        for task_id in sorted(ids1):
            record = task_history.get(task_id)
            if record is None:
                return None, 'TASK_GEOMETRY_UNAVAILABLE'
            models.append(_task_model(record, task_id))
        union = CanonicalUnion(tuple(models), union_hash)
        active = {}
        for robot, status in statuses.items():
            if status.get('active'):
                active[robot] = str(status.get('active'))
        valid = {
            robot: {
                bid.canonical_task_id for bid in batches[robot].bids
                if bid.path_valid
            }
            for robot in ('robot1', 'robot2')
        }
        active1 = active.get('robot1', '') if active.get('robot1') in valid['robot1'] else ''
        active2 = active.get('robot2', '') if active.get('robot2') in valid['robot2'] else ''
        try:
            assignment = choose_assignment(
                union, first, second,
                active_robot1_id=active1,
                active_robot2_id=active2,
            )
        except (TypeError, ValueError):
            return None, 'SELECTOR_INPUT_UNAVAILABLE'
        if assignment is None:
            return None, 'NO_VALID_ASSIGNMENT'
        return (assignment, union_hash, active1, active2), 'COMPLETE_CURRENT_PAIR'

    def publish_context(event):
        assignment, reason = current_assignment()
        if assignment is None:
            choices = {'robot1': ('unknown', ''), 'robot2': ('unknown', '')}
            union_hash = ''
        else:
            selected, union_hash, active1, active2 = assignment
            choices = {}
            for robot, index, active_id in (
                    ('robot1', 0, active1), ('robot2', 1, active2)):
                selected_id = str(selected[index] or '')
                if active_id:
                    choices[robot] = ('local', active_id)
                elif selected_id:
                    choices[robot] = ('local', selected_id)
                elif selected[1 - index]:
                    choices[robot] = ('peer', str(selected[1 - index]))
                else:
                    choices[robot] = ('none', '')
        for offset, robot in enumerate(('robot1', 'robot2')):
            scope, task_id = choices[robot]
            scope_reason = {
                'local': 'LOCAL_ASSIGNED',
                'peer': 'PEER_ASSIGNED',
                'none': 'NO_LOCAL_ASSIGNMENT',
            }.get(scope, reason)
            value = (scope, task_id, union_hash, scope_reason)
            if value == emitted[robot]:
                continue
            emitted[robot] = value
            output.append({
                'robot_id': robot,
                'event_type': 'MINIMAL_ASSIGNMENT_CONTEXT',
                'elapsed_s': _record_time(event),
                'event_sequence': 2_000_000 + len(output) * 2 + offset,
                'assignment_scope': scope,
                'assigned_task_id': task_id,
                'union_hash': union_hash,
                'assignment_reason': scope_reason,
            })

    for event in ordered:
        kind = str(event.get('event_type', ''))
        if kind == 'CANDIDATE_BATCH_RECEIVED':
            for candidate in event.get('candidates') or ():
                task_id = candidate.get('frontier_id')
                if task_id is not None:
                    task_history[str(task_id)] = candidate
        elif kind == 'DISTRIBUTED_TASK_SNAPSHOT':
            for task in event.get('tasks') or ():
                task_id = task.get('local_frontier_id')
                if task_id is not None:
                    task_history[str(task_id)] = task
        elif kind == 'DISTRIBUTED_BID_ARRAY':
            robot = str(event.get('robot_id', ''))
            if robot in ('robot1', 'robot2'):
                batches[robot] = _batch_model(event)
                publish_context(event)
        elif kind == 'DISTRIBUTED_STATUS':
            robot = str(event.get('robot_id', ''))
            if robot in ('robot1', 'robot2'):
                statuses[robot] = {
                    'union_hash': str(event.get('union_hash', '')),
                    'active': (str(event.get('active_canonical_task_id', ''))
                               if event.get('local_nav_goal_active') else ''),
                }
                publish_context(event)
    return sorted(output, key=lambda event: (
        _record_time(event), int(event.get('event_sequence', 0) or 0)))


def _finite_path(candidate):
    state = candidate.get('reachability_state')
    length = candidate.get('path_length_m', candidate.get('local_path_length_m'))
    return state in (1, 'REACHABLE', 'reachable') and length is not None and \
        math.isfinite(float(length)) and float(length) >= 0.0


def _new_state():
    return {
        'batch_time': None, 'candidate_count': 0,
        'candidate_source_healthy': None, 'nav2_healthy': None,
        'tf_healthy': None, 'direct_nav2_healthy': None,
        'direct_tf_healthy': None, 'native_goal_active': None,
        'goal_active': False, 'recovery': 0,
        'traffic': False, 'safety': False,
        'blocked_reason': '',
        'assignment_mode': False, 'assignment_scope': 'unknown',
        'assignment_reason': 'NO_CURRENT_ASSIGNMENT',
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
        message = str(event.get('message', ''))
        try:
            status = int(event.get('state'))
        except (TypeError, ValueError):
            status = None
        state['traffic'] = status == 7 or message == 'WAITING_TRAFFIC'
        if state['traffic']:
            state['blocked_reason'] = 'TRAFFIC_WAIT'
        elif state['blocked_reason'] == 'TRAFFIC_WAIT':
            state['blocked_reason'] = ''
    elif kind == 'MINIMAL_ASSIGNMENT_CONTEXT':
        state['assignment_mode'] = True
        state['assignment_scope'] = str(event.get('assignment_scope', 'unknown'))
        state['assignment_reason'] = str(
            event.get('assignment_reason', 'NO_CURRENT_ASSIGNMENT'))
    elif kind == 'NATIVE_PLANNER_EVIDENCE':
        state['direct_nav2_healthy'] = True
        if event.get('direct_tf_healthy') is True:
            state['direct_tf_healthy'] = True
    elif kind == 'NATIVE_NAVIGATION_EXECUTING':
        state['direct_nav2_healthy'] = True
        if event.get('direct_tf_healthy') is True:
            state['direct_tf_healthy'] = True
        state['native_goal_active'] = True
    elif kind == 'NATIVE_NAVIGATION_TERMINAL':
        state['direct_nav2_healthy'] = True
        if event.get('direct_tf_healthy') is True:
            state['direct_tf_healthy'] = True
        state['native_goal_active'] = False
    elif kind == 'TRAFFIC_WAITING':
        state['traffic'] = True
        state['blocked_reason'] = 'TRAFFIC_WAIT'
    elif kind == 'TRAFFIC_CONFLICT_CLEARED':
        state['traffic'] = False
        if state['blocked_reason'] == 'TRAFFIC_WAIT':
            state['blocked_reason'] = ''
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
        state['blocked_reason'] = 'RECOVERY'
    elif kind == 'STUCK_CLEARED':
        state['recovery'] = 0
        if state['blocked_reason'] == 'RECOVERY':
            state['blocked_reason'] = ''


def _classification(state, now, ready_sim, stale_s):
    current_batch = (state['batch_time'] is not None and
                     now - state['batch_time'] <= stale_s and
                     state['candidate_count'] > 0)
    if state['traffic'] or state['safety'] or state['recovery'] > 0:
        return 'ALLOWED_BLOCKED'
    if now < ready_sim:
        return 'WORK_UNAVAILABLE'
    nav2_healthy = (state['nav2_healthy'] is True or
                    state['direct_nav2_healthy'] is True)
    tf_healthy = (state['tf_healthy'] is True or
                  state['direct_tf_healthy'] is True)
    if (state['assignment_mode'] and state['assignment_scope'] != 'local'):
        return 'WORK_UNAVAILABLE'
    if (current_batch and state['candidate_source_healthy'] is True and
            nav2_healthy and tf_healthy):
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
                and merged[-1]['goal_active'] == item['goal_active']
                and merged[-1].get('classification_reason') == item.get(
                    'classification_reason')
                and abs(merged[-1]['end_s'] - item['start_s']) <= 1.0e-6):
            merged[-1]['end_s'] = item['end_s']
            merged[-1]['duration_s'] = (
                merged[-1]['end_s'] - merged[-1]['start_s'])
            merged[-1]['candidate_count'] = max(
                merged[-1]['candidate_count'], item['candidate_count'])
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
            goal_active = (state['native_goal_active']
                           if state['native_goal_active'] is not None
                           else state['goal_active'])
            if classification == 'ALLOWED_BLOCKED':
                classification_reason = state['blocked_reason'] or 'BLOCKED'
            elif classification == 'FEASIBLE_WORK_AVAILABLE':
                classification_reason = 'LOCAL_ASSIGNMENT'
            elif state['assignment_mode']:
                classification_reason = state['assignment_reason']
            else:
                classification_reason = 'LEGACY_OR_UNAVAILABLE_EVIDENCE'
            segments.append({
                'start_s': start, 'end_s': stop,
                'duration_s': stop - start, 'classification': classification,
                'candidate_count': state['candidate_count'],
                'goal_active': goal_active,
                'classification_reason': classification_reason,
            })
        segments = _merge_intervals(segments)
        feasible = [item for item in segments
                    if item['classification'] == 'FEASIBLE_WORK_AVAILABLE']
        idle = [item for item in feasible if not item['goal_active']]
        blocked = [item for item in segments
                   if item['classification'] == 'ALLOWED_BLOCKED']
        feasible_s = sum(item['duration_s'] for item in feasible)
        idle_s = sum(item['duration_s'] for item in idle)
        productive_s = max(0.0, feasible_s - idle_s)
        unavailable_segments = [item for item in segments
                                if item['classification'] == 'WORK_UNAVAILABLE']
        traffic_s = sum(item['duration_s'] for item in blocked
                        if item.get('classification_reason') == 'TRAFFIC_WAIT')
        blocked_other_s = sum(item['duration_s'] for item in blocked) - traffic_s
        unavailable_s = sum(item['duration_s'] for item in blocked) + sum(
            item['duration_s'] for item in unavailable_segments)
        no_local_task_s = sum(
            item['duration_s'] for item in unavailable_segments
            if item.get('classification_reason') in (
                'PEER_ASSIGNED', 'NO_LOCAL_ASSIGNMENT'))
        other_unavailable_s = blocked_other_s + sum(
            item['duration_s'] for item in unavailable_segments
            if item.get('classification_reason') not in (
                'PEER_ASSIGNED', 'NO_LOCAL_ASSIGNMENT'))
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
                item['duration_s'] for item in blocked), 6),
            'traffic_wait_seconds': round(traffic_s, 6),
            'no_local_task_seconds': round(no_local_task_s, 6),
            'other_work_unavailable_seconds': round(other_unavailable_s, 6),
            'work_unavailable_seconds': round(unavailable_s, 6),
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


def _finite_number(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _odometry_time(row):
    for key in ('elapsed_s', 'sim_time_s', 'header_stamp',
                'received_ros_time_s', 'time_s'):
        value = _finite_number(row.get(key))
        if value is not None:
            return value
    return None


def _odometry_pose(row):
    x = _finite_number(row.get('pose_x', row.get('x')))
    y = _finite_number(row.get('pose_y', row.get('y')))
    if x is None or y is None:
        return None
    yaw = _finite_number(row.get('pose_yaw', row.get('yaw')))
    if yaw is None:
        qz = _finite_number(row.get('orientation_z'))
        qw = _finite_number(row.get('orientation_w'))
        if qz is not None and qw is not None:
            yaw = 2.0 * math.atan2(qz, qw)
    return x, y, yaw


def _odometry_twist(row):
    linear_x = _finite_number(row.get('twist_linear_x'))
    linear_y = _finite_number(row.get('twist_linear_y'))
    if linear_x is None:
        linear_x = _finite_number(row.get('linear_speed_mps'))
    if linear_y is None:
        linear_y = 0.0
    angular = _finite_number(row.get('twist_angular_z'))
    if angular is None:
        angular = _finite_number(row.get('angular_speed_radps'))
    if linear_x is None or angular is None:
        return None
    return math.hypot(linear_x, linear_y), abs(angular)


def _wrap_angle(value):
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def _physical_motion_intervals(odometry, translation_threshold_mps,
                               rotation_threshold_radps):
    samples = []
    for row in odometry or ():
        stamp = _odometry_time(row)
        pose = _odometry_pose(row)
        if stamp is None or pose is None:
            continue
        samples.append((stamp, pose, _odometry_twist(row)))
    samples.sort(key=lambda item: item[0])
    intervals = []
    for first, second in zip(samples, samples[1:]):
        start = first[0]
        end = second[0]
        if end <= start:
            continue
        dt = end - start
        first_pose, second_pose = first[1], second[1]
        translation_rate = math.hypot(
            second_pose[0] - first_pose[0],
            second_pose[1] - first_pose[1],
        ) / dt
        if first_pose[2] is None or second_pose[2] is None:
            rotation_rate = 0.0
            pose_rotation_available = False
        else:
            rotation_rate = abs(_wrap_angle(second_pose[2] - first_pose[2])) / dt
            pose_rotation_available = True
        pose_translation_active = translation_rate >= translation_threshold_mps
        pose_rotation_active = (pose_rotation_available and
                                rotation_rate >= rotation_threshold_radps)
        if pose_translation_active:
            category = 'translating'
        elif pose_rotation_active:
            category = 'rotating_only'
        else:
            category = 'stationary'
        twist = first[2]
        twist_active = (twist is not None and
                        (twist[0] >= translation_threshold_mps or
                         twist[1] >= rotation_threshold_radps))
        pose_active = pose_translation_active or pose_rotation_active
        intervals.append({
            'start_s': start,
            'end_s': end,
            'category': category,
            'pose_active': pose_active,
            'twist_active': twist_active,
            'pose_twist_disagreement': twist is not None and
            pose_active != twist_active,
        })
    return intervals


def analyze_physical_activity(timing, odometry_by_robot, ready_sim=0.0,
                              end_sim=None, robots=None,
                              translation_threshold_mps=0.01,
                              rotation_threshold_radps=0.02,
                              run_label=None):
    """Classify feasible time using measured odometry, not action status.

    ``timing`` is the existing event-state result.  An accepted native goal
    takes precedence over later assignment-context churn.  Traffic and
    no-local-task intervals remain outside the physical feasible denominator.
    Missing odometry is reported as infrastructure evidence unavailable rather
    than being silently counted as motion or productivity.
    """
    if robots is None:
        robots = ('robot1', 'robot2')
    if end_sim is None:
        end_sim = float(timing.get('end_sim_s', ready_sim) or ready_sim)
    ready_sim = float(ready_sim or 0.0)
    end_sim = float(end_sim or 0.0)
    result = {
        'run': run_label,
        'ready_sim_s': ready_sim,
        'end_sim_s': end_sim,
        'method': {
            'motion_source': 'forensic odometry pose delta and reported twist',
            'translation_threshold_mps': translation_threshold_mps,
            'rotation_threshold_radps': rotation_threshold_radps,
            'pose_delta_precedence': True,
            'missing_motion_evidence_is_unavailable': True,
        },
        'robots': {},
    }
    for robot in robots:
        base_segments = list(timing['robots'][robot].get('segments', ()))
        motion_segments = _physical_motion_intervals(
            (odometry_by_robot or {}).get(robot, ()),
            translation_threshold_mps,
            rotation_threshold_radps,
        )
        boundaries = {ready_sim, end_sim}
        for item in base_segments + motion_segments:
            boundaries.add(max(ready_sim, float(item['start_s'])))
            boundaries.add(min(end_sim, float(item['end_s'])))
        boundaries = sorted(value for value in boundaries
                            if ready_sim <= value <= end_sim)
        base_index = 0
        motion_index = 0
        totals = {
            'translating': 0.0,
            'rotating_only': 0.0,
            'stationary_active_goal': 0.0,
            'stationary_pre_dispatch': 0.0,
            'legitimate_traffic_wait': 0.0,
            'no_local_task_or_peer_assigned': 0.0,
            'true_infrastructure_unavailable': 0.0,
            'physical_motion_evidence_unavailable': 0.0,
            'navigation_action_active': 0.0,
            'pose_twist_disagreement': 0.0,
        }
        segments = []
        for left, right in zip(boundaries, boundaries[1:]):
            if right <= left:
                continue
            while (base_index < len(base_segments) and
                   base_segments[base_index]['end_s'] <= left + 1.0e-9):
                base_index += 1
            while (motion_index < len(motion_segments) and
                   motion_segments[motion_index]['end_s'] <= left + 1.0e-9):
                motion_index += 1
            base = (base_segments[base_index]
                    if base_index < len(base_segments) and
                    base_segments[base_index]['start_s'] <= left + 1.0e-9
                    else None)
            motion = (motion_segments[motion_index]
                      if motion_index < len(motion_segments) and
                      motion_segments[motion_index]['start_s'] <= left + 1.0e-9
                      and motion_segments[motion_index]['end_s'] >= right - 1.0e-9
                      else None)
            duration = right - left
            if base is None:
                classification = 'TRUE_INFRASTRUCTURE_UNAVAILABLE'
                reason = 'NO_TIMING_EVIDENCE'
                totals['true_infrastructure_unavailable'] += duration
            else:
                active_goal = bool(base.get('goal_active'))
                reason = str(base.get('classification_reason', ''))
                traffic_wait = (base.get('classification') == 'ALLOWED_BLOCKED'
                                and reason == 'TRAFFIC_WAIT')
                no_local = (reason in ('PEER_ASSIGNED', 'NO_LOCAL_ASSIGNMENT'))
                local_assignment = (base.get('classification') ==
                                    'FEASIBLE_WORK_AVAILABLE')
                if traffic_wait:
                    classification = 'LEGITIMATE_TRAFFIC_WAIT'
                    totals['legitimate_traffic_wait'] += duration
                elif not active_goal and no_local:
                    classification = 'NO_LOCAL_TASK_OR_PEER_ASSIGNED'
                    totals['no_local_task_or_peer_assigned'] += duration
                elif active_goal or local_assignment:
                    if motion is None:
                        classification = 'PHYSICAL_MOTION_EVIDENCE_UNAVAILABLE'
                        totals['physical_motion_evidence_unavailable'] += duration
                        totals['true_infrastructure_unavailable'] += duration
                    else:
                        category = motion['category']
                        classification = category.upper()
                        totals[category if category != 'stationary' else
                               ('stationary_active_goal' if active_goal else
                                'stationary_pre_dispatch')] += duration
                        if motion['pose_twist_disagreement']:
                            totals['pose_twist_disagreement'] += duration
                else:
                    classification = 'TRUE_INFRASTRUCTURE_UNAVAILABLE'
                    totals['true_infrastructure_unavailable'] += duration
                if active_goal:
                    totals['navigation_action_active'] += duration
            segments.append({
                'start_s': left,
                'end_s': right,
                'duration_s': duration,
                'classification': classification,
                'base_classification': (base.get('classification')
                                        if base else None),
                'base_reason': reason,
                'goal_active': bool(base and base.get('goal_active')),
                'motion_evidence': motion['category'] if motion else None,
            })
        physical_productive = (totals['translating'] +
                               totals['rotating_only'])
        physical_idle = (totals['stationary_active_goal'] +
                         totals['stationary_pre_dispatch'])
        physical_feasible = physical_productive + physical_idle
        excluded = (totals['legitimate_traffic_wait'] +
                    totals['no_local_task_or_peer_assigned'] +
                    totals['true_infrastructure_unavailable'])
        post_ready = max(0.0, end_sim - ready_sim)
        result['robots'][robot] = {
            'physical_feasible_work_seconds': round(physical_feasible, 6),
            'physically_productive_seconds': round(physical_productive, 6),
            'translating_seconds': round(totals['translating'], 6),
            'rotating_only_seconds': round(totals['rotating_only'], 6),
            'stationary_feasible_seconds': round(physical_idle, 6),
            'stationary_active_goal_seconds': round(
                totals['stationary_active_goal'], 6),
            'stationary_pre_dispatch_seconds': round(
                totals['stationary_pre_dispatch'], 6),
            'avoidable_physical_idle_seconds': round(physical_idle, 6),
            'legitimate_traffic_wait_seconds': round(
                totals['legitimate_traffic_wait'], 6),
            'no_local_task_or_peer_assigned_seconds': round(
                totals['no_local_task_or_peer_assigned'], 6),
            'true_infrastructure_unavailable_seconds': round(
                totals['true_infrastructure_unavailable'], 6),
            'physical_motion_evidence_unavailable_seconds': round(
                totals['physical_motion_evidence_unavailable'], 6),
            'navigation_action_active_seconds': round(
                totals['navigation_action_active'], 6),
            'pose_twist_disagreement_seconds': round(
                totals['pose_twist_disagreement'], 6),
            'physical_productivity_percent': (
                physical_productive / physical_feasible * 100.0
                if physical_feasible else None),
            'post_readiness_seconds': round(post_ready, 6),
            'segments': segments,
            'validation': {
                'physical_feasible_equals_productive_plus_idle': math.isclose(
                    physical_feasible, physical_productive + physical_idle,
                    abs_tol=0.12),
                'post_readiness_equals_feasible_plus_excluded': math.isclose(
                    post_ready, physical_feasible + excluded, abs_tol=0.12),
            },
        }
    result['combined'] = {
        key: round(sum(result['robots'][robot][key] for robot in robots), 6)
        for key in (
            'physical_feasible_work_seconds',
            'physically_productive_seconds',
            'translating_seconds',
            'rotating_only_seconds',
            'stationary_feasible_seconds',
            'stationary_active_goal_seconds',
            'stationary_pre_dispatch_seconds',
            'avoidable_physical_idle_seconds',
            'legitimate_traffic_wait_seconds',
            'no_local_task_or_peer_assigned_seconds',
            'true_infrastructure_unavailable_seconds',
            'physical_motion_evidence_unavailable_seconds',
            'navigation_action_active_seconds',
            'pose_twist_disagreement_seconds',
        )
    }
    feasible = result['combined']['physical_feasible_work_seconds']
    result['combined']['physical_productivity_percent'] = (
        result['combined']['physically_productive_seconds'] / feasible * 100.0
        if feasible else None)
    result['combined']['post_readiness_seconds'] = round(
        sum(result['robots'][robot]['post_readiness_seconds']
            for robot in robots), 6)
    result['combined']['validation'] = {
        'physical_feasible_equals_productive_plus_idle': math.isclose(
            result['combined']['physical_feasible_work_seconds'],
            result['combined']['physically_productive_seconds'] +
            result['combined']['avoidable_physical_idle_seconds'],
            abs_tol=0.12),
        'post_readiness_equals_feasible_plus_excluded': math.isclose(
            result['combined']['post_readiness_seconds'],
            result['combined']['physical_feasible_work_seconds'] +
            result['combined']['legitimate_traffic_wait_seconds'] +
            result['combined']['no_local_task_or_peer_assigned_seconds'] +
            result['combined']['true_infrastructure_unavailable_seconds'],
            abs_tol=0.12),
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
    for start, dispatch in sorted(dispatch_pairs, key=lambda item: item[0]):
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
