"""Deterministic task-level adaptation of Burgard et al. Algorithm 1.

This module intentionally keeps the original weighted pair score out of the
production path.  Canonical frontier tasks replace individual frontier cells,
and local Nav2 path lengths replace the paper's grid value iteration costs.
"""

from dataclasses import replace
import hashlib
import json
import math
from typing import Mapping, Optional

from .models import (
    AssignmentDiagnostics,
    AssignmentScore,
    Bid,
    BidBatch,
    CanonicalTask,
    CanonicalUnion,
    PairDecision,
    Point,
)
from .scoring import bid_fingerprint


IDLE_TASK_ID = ''


def _hash(payload: object) -> str:
    """Hash only deterministic, JSON-compatible decision data."""
    return hashlib.sha256(json.dumps(
        payload, sort_keys=True, separators=(',', ':'), allow_nan=False,
    ).encode('utf-8')).hexdigest()


def _bounded(value: float) -> float:
    """Clamp the project-normalized Nav2 cost to its declared range."""
    return max(0.0, min(1.0, value))


def _grid_yaw(grid) -> float:
    orientation = grid.info.origin.orientation
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
    )


def occupancy_value(grid, point: Point) -> Optional[int]:
    """Read a world point from an occupancy grid with arbitrary origin yaw."""
    if grid is None or grid.info.resolution <= 0.0:
        return None
    resolution = grid.info.resolution
    origin = grid.info.origin.position
    dx, dy = point[0] - origin.x, point[1] - origin.y
    cosine, sine = math.cos(_grid_yaw(grid)), math.sin(_grid_yaw(grid))
    cell_x = math.floor((cosine * dx + sine * dy) / resolution)
    cell_y = math.floor((-sine * dx + cosine * dy) / resolution)
    if (cell_x < 0 or cell_y < 0 or cell_x >= grid.info.width or
            cell_y >= grid.info.height):
        return None
    index = cell_y * grid.info.width + cell_x
    if index < 0 or index >= len(grid.data):
        return None
    return int(grid.data[index])


def line_of_sight_clear(
        grid, first: Point, second: Point, occupied_threshold: int) -> bool:
    """Ray cast through the shared grid; unknown cells do not block visibility.

    The sampling interval is at most half a grid cell.  A map-boundary exit is
    conservative (not clear), while values above the project's existing
    frontier occupied threshold block visibility.  Unknown occupancy values
    are negative and are deliberately not treated as occupied obstacles.
    """
    if grid is None or grid.info.resolution <= 0.0:
        return False
    distance = math.dist(first, second)
    count = max(1, math.ceil(distance / max(grid.info.resolution * 0.5, 1e-9)))
    for index in range(count + 1):
        fraction = index / count
        value = occupancy_value(
            grid,
            (first[0] + fraction * (second[0] - first[0]),
             first[1] + fraction * (second[1] - first[1])),
        )
        if value is None or value > occupied_threshold:
            return False
    return True


def _bid_map(batch: BidBatch) -> Mapping[str, Bid]:
    result: dict[str, Bid] = {}
    for bid in batch.bids:
        if bid.canonical_task_id in result:
            raise ValueError('duplicate canonical task in bid batch')
        if (math.isfinite(bid.path_length_m) and bid.path_length_m >= 0.0 and
                math.isfinite(bid.estimated_travel_cost) and
                bid.estimated_travel_cost >= 0.0):
            result[bid.canonical_task_id] = bid
    return result


def _task_feasible(
        task: CanonicalTask, bid: Optional[Bid], hard_failed_tasks: frozenset[str],
        minimum_visible_gain_m: float, maximum_path_length_m: float) -> bool:
    """Keep existing candidate quality, Nav2, and hard-failure gates."""
    return bool(
        bid is not None and bid.path_valid and
        task.canonical_id not in hard_failed_tasks and
        math.isfinite(task.visible_reveal_gain) and
        task.visible_reveal_gain >= minimum_visible_gain_m and
        math.isfinite(bid.path_length_m) and
        0.0 <= bid.path_length_m <= maximum_path_length_m and
        math.isfinite(bid.estimated_travel_cost) and
        0.0 <= bid.estimated_travel_cost <= maximum_path_length_m
    )


def _robot_number(robot_id: str) -> int:
    return 1 if robot_id == 'robot1' else 2


def _reduction(
        assigned: CanonicalTask, remaining: CanonicalTask, grid,
        sensor_max_range_m: float, occupied_threshold: int) -> tuple[float, bool, float]:
    """Return Burgard's range/LOS utility reduction for task centroids."""
    distance = math.dist(assigned.centroid, remaining.centroid)
    if distance >= sensor_max_range_m:
        return distance, False, 0.0
    clear = line_of_sight_clear(
        grid, assigned.centroid, remaining.centroid, occupied_threshold,
    )
    if not clear:
        return distance, False, 0.0
    return distance, True, 1.0 - distance / sensor_max_range_m


def choose_burgard_assignment(
        round_id: str, union: CanonicalUnion,
        robot1_bids: BidBatch, robot2_bids: BidBatch,
        *, beta: float = 1.0, maximum_path_length_m: float = 18.0,
        minimum_visible_gain_m: float = 0.05, sensor_max_range_m: float = 11.98,
        occupied_threshold: int = 50, shared_map=None,
        hard_failed_tasks: frozenset[str] = frozenset()) -> PairDecision:
    """Run the bounded two-robot Burgard-inspired sequential assignment.

    ``U_t`` starts at one for every eligible canonical task.  ``C_i,t`` is the
    robot's existing Nav2 path length normalized by the already authoritative
    feasible-path ceiling.  This is a project adaptation, not a claim that
    Burgard et al. used Nav2 or this normalization.
    """
    if beta < 0.0 or not math.isfinite(beta):
        raise ValueError('beta must be finite and non-negative')
    if maximum_path_length_m <= 0.0 or sensor_max_range_m <= 0.0:
        raise ValueError('path limit and sensor range must be positive')
    for batch, robot_id in ((robot1_bids, 'robot1'), (robot2_bids, 'robot2')):
        if batch.round_id != round_id or batch.union_hash != union.union_hash:
            raise ValueError('bid batch does not reference the canonical round')
        if batch.source_robot_id != robot_id:
            raise ValueError('unexpected bidder identity')

    tasks = {task.canonical_id: task for task in union.tasks}
    bids = {'robot1': _bid_map(robot1_bids), 'robot2': _bid_map(robot2_bids)}
    feasible = {
        robot_id: sorted(task_id for task_id, task in tasks.items()
                         if _task_feasible(
                             task, bids[robot_id].get(task_id), hard_failed_tasks,
                             minimum_visible_gain_m, maximum_path_length_m,
                         ))
        for robot_id in ('robot1', 'robot2')
    }
    eligible_tasks = sorted(set(feasible['robot1']) | set(feasible['robot2']))
    utilities = {task_id: 1.0 for task_id in eligible_tasks}
    remaining_robots = ['robot1', 'robot2']
    remaining_tasks = set(eligible_tasks)
    assigned: dict[str, str] = {'robot1': IDLE_TASK_ID, 'robot2': IDLE_TASK_ID}
    trace: list[dict[str, object]] = []
    selected_scores: list[float] = []

    while remaining_robots:
        candidates = []
        for robot_id in remaining_robots:
            for task_id in feasible[robot_id]:
                if task_id not in remaining_tasks:
                    continue
                bid = bids[robot_id][task_id]
                normalized_cost = _bounded(bid.path_length_m / maximum_path_length_m)
                score = utilities[task_id] - beta * normalized_cost
                # Primary score, then lower normalized cost, robot ID, task ID.
                rank = (-round(score, 12), round(normalized_cost, 12),
                        _robot_number(robot_id), task_id)
                candidates.append((rank, robot_id, task_id, bid, normalized_cost, score))
        if not candidates:
            for robot_id in remaining_robots:
                trace.append({
                    'step': len(trace) + 1,
                    'robot_id': robot_id,
                    'task_id': IDLE_TASK_ID,
                    'reason': 'NO_FEASIBLE_REMAINING_TASK',
                })
            break
        _, robot_id, task_id, bid, normalized_cost, score = min(
            candidates, key=lambda item: item[0],
        )
        task = tasks[task_id]
        assigned[robot_id] = task_id
        selected_scores.append(score)
        reductions = []
        for other_id in sorted(remaining_tasks - {task_id}):
            distance, clear, reduction = _reduction(
                task, tasks[other_id], shared_map, sensor_max_range_m,
                occupied_threshold,
            )
            before = utilities[other_id]
            utilities[other_id] = before - reduction
            reductions.append({
                'task_id': other_id,
                'distance_m': round(distance, 12),
                'line_of_sight_clear': clear,
                'reduction': round(reduction, 12),
                'utility_before': round(before, 12),
                'utility_after': round(utilities[other_id], 12),
            })
        trace.append({
            'step': len(trace) + 1,
            'robot_id': robot_id,
            'task_id': task_id,
            'task_centroid': [round(task.centroid[0], 6), round(task.centroid[1], 6)],
            'initial_utility': 1.0,
            'utility_before': round(utilities[task_id], 12),
            'raw_nav2_path_length_m': round(bid.path_length_m, 12),
            'normalized_cost': round(normalized_cost, 12),
            'beta': beta,
            'score': round(score, 12),
            'tie_break_order': 'score_then_lower_cost_then_robot_id_then_task_id',
            'reductions': reductions,
        })
        remaining_robots.remove(robot_id)
        remaining_tasks.remove(task_id)

    first_fingerprint = bid_fingerprint(robot1_bids)
    second_fingerprint = bid_fingerprint(robot2_bids)
    combined = sum(
        bids[robot_id][task_id].path_length_m
        for robot_id, task_id in assigned.items() if task_id
    )
    maximum = max((
        bids[robot_id][task_id].path_length_m
        for robot_id, task_id in assigned.items() if task_id
    ), default=0.0)
    score = AssignmentScore(total=sum(selected_scores))
    decision_payload = {
        'round_id': round_id,
        'union_hash': union.union_hash,
        'robot1_bid_fingerprint': first_fingerprint,
        'robot2_bid_fingerprint': second_fingerprint,
        'robot1_task': assigned['robot1'],
        'robot2_task': assigned['robot2'],
        'burgard_total_millionths': round(score.total * 1_000_000),
    }
    diagnostics = AssignmentDiagnostics(
        idle_reason='NO_TASKS' if not union.tasks else 'OTHER',
        availability_reason=(
            'BOTH_ROBOTS_REACHABLE' if feasible['robot1'] and feasible['robot2']
            else 'ONLY_ROBOT1_REACHABLE' if feasible['robot1']
            else 'ONLY_ROBOT2_REACHABLE' if feasible['robot2'] else 'NO_VALID_BIDS'
        ),
        union_task_count=len(union.tasks),
        robot1_bid_count=len(robot1_bids.bids),
        robot2_bid_count=len(robot2_bids.bids),
        robot1_valid_bid_count=len(feasible['robot1']),
        robot2_valid_bid_count=len(feasible['robot2']),
        reachable_by_both_count=len(set(feasible['robot1']) & set(feasible['robot2'])),
        reachable_only_robot1_count=len(set(feasible['robot1']) - set(feasible['robot2'])),
        reachable_only_robot2_count=len(set(feasible['robot2']) - set(feasible['robot1'])),
        rejected_equivalence_count=sum(max(0, len(task.members) - 1)
                                       for task in union.tasks),
        rejected_failure_suppression_count=sum(
            task.canonical_id in hard_failed_tasks for task in union.tasks),
        rejected_gain_threshold_count=sum(
            task.visible_reveal_gain < minimum_visible_gain_m for task in union.tasks),
        rejected_path_threshold_count=sum(
            bid.path_length_m > maximum_path_length_m or
            bid.estimated_travel_cost > maximum_path_length_m
            for batch in (robot1_bids, robot2_bids) for bid in batch.bids),
        feasible_useful_robot1_count=len(feasible['robot1']),
        feasible_useful_robot2_count=len(feasible['robot2']),
        valid_one_active_assignment_count=int(bool(assigned['robot1'])) +
        int(bool(assigned['robot2'])),
        valid_two_active_pair_count=int(bool(assigned['robot1'] and assigned['robot2'])),
        idle_idle_permitted=not bool(assigned['robot1'] or assigned['robot2']),
        robot1_idle_reason='NOT_IDLE' if assigned['robot1'] else 'NO_FEASIBLE_REMAINING_TASK',
        robot2_idle_reason='NOT_IDLE' if assigned['robot2'] else 'NO_FEASIBLE_REMAINING_TASK',
        best_non_idle_robot1_task_id=assigned['robot1'],
        best_non_idle_robot2_task_id=assigned['robot2'],
        best_non_idle_score=score,
        strategy='burgard', beta=beta,
        feasible_path_limit_m=maximum_path_length_m,
        sensor_max_range_m=sensor_max_range_m,
        burgard_trace=tuple(trace),
    )
    return PairDecision(
        round_id=round_id, union_hash=union.union_hash,
        robot1_task_id=assigned['robot1'], robot2_task_id=assigned['robot2'],
        robot1_bid_fingerprint=first_fingerprint,
        robot2_bid_fingerprint=second_fingerprint,
        score=score, decision_hash=_hash(decision_payload),
        combined_path_length_m=combined, maximum_path_length_m=maximum,
        diagnostics=diagnostics,
    )
