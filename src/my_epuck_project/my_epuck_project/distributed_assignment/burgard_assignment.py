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
from ..frontier_actionability import gain_meets_minimum


IDLE_TASK_ID = ''


def _hash(payload: object) -> str:
    """Hash only deterministic, JSON-compatible decision data."""
    return hashlib.sha256(json.dumps(
        payload, sort_keys=True, separators=(',', ':'), allow_nan=False,
    ).encode('utf-8')).hexdigest()


def _bounded(value: float) -> float:
    """Clamp a geometric reduction term to its declared range."""
    return max(0.0, min(1.0, value))


def _path_cost(value: float, scale_m: float) -> float:
    """Convert path length to a soft cost without imposing a distance cap."""
    return max(0.0, value / scale_m)


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
            if bid.path_valid and (
                    not bid.path or any(
                        not (math.isfinite(float(x)) and math.isfinite(float(y)))
                        for x, y in bid.path
                    )):
                continue
            result[bid.canonical_task_id] = bid
    return result


def _task_feasible(
        task: CanonicalTask, bid: Optional[Bid], hard_failed_tasks: frozenset[str],
        minimum_visible_gain_m: float) -> bool:
    """Keep existing candidate quality, Nav2, and hard-failure gates."""
    return bool(
        bid is not None and bid.path_valid and
        task.canonical_id not in hard_failed_tasks and
        gain_meets_minimum(task.visible_reveal_gain, minimum_visible_gain_m) and
        math.isfinite(bid.path_length_m) and
        bid.path_length_m >= 0.0 and
        math.isfinite(bid.estimated_travel_cost) and
        bid.estimated_travel_cost >= 0.0
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


def _pair_candidate(
        first_robot: str, first_task_id: str,
        second_robot: str, second_task_id: str,
        tasks: Mapping[str, CanonicalTask], bids: Mapping[str, Mapping[str, Bid]],
        shared_map, sensor_max_range_m: float, occupied_threshold: int,
        beta: float, path_cost_scale_m: float):
    """Score one distinct two-robot pair in one Burgard assignment order.

    Burgard's utility reduction is order-dependent.  The production solver
    therefore evaluates both robot/task orders for a complete pair, rather
    than committing the first greedy choice before the second robot is known.
    This keeps the paper-derived utility and the existing Nav2 cost term
    unchanged while allowing the bounded two-robot problem to be solved
    exactly.
    """
    first_task = tasks[first_task_id]
    second_task = tasks[second_task_id]
    first_bid = bids[first_robot][first_task_id]
    second_bid = bids[second_robot][second_task_id]
    first_cost = _path_cost(first_bid.path_length_m, path_cost_scale_m)
    first_score = 1.0 - beta * first_cost
    distance, clear, reduction = _reduction(
        first_task, second_task, shared_map, sensor_max_range_m,
        occupied_threshold,
    )
    second_utility = 1.0 - reduction
    second_cost = _path_cost(second_bid.path_length_m, path_cost_scale_m)
    second_score = second_utility - beta * second_cost
    trace = (
        {
            'step': 1, 'robot_id': first_robot,
            'task_id': first_task_id,
            'task_centroid': [round(first_task.centroid[0], 6),
                              round(first_task.centroid[1], 6)],
            'initial_utility': 1.0, 'utility_before': 1.0,
            'raw_nav2_path_length_m': round(first_bid.path_length_m, 12),
            'normalized_cost': round(first_cost, 12), 'beta': beta,
            'score': round(first_score, 12),
            'tie_break_order': 'pair_score_then_combined_cost_then_task_ids',
            'reductions': [{
                'task_id': second_task_id,
                'distance_m': round(distance, 12),
                'line_of_sight_clear': clear,
                'reduction': round(reduction, 12),
                'utility_before': 1.0,
                'utility_after': round(second_utility, 12),
            }],
        },
        {
            'step': 2, 'robot_id': second_robot,
            'task_id': second_task_id,
            'task_centroid': [round(second_task.centroid[0], 6),
                              round(second_task.centroid[1], 6)],
            'initial_utility': 1.0,
            'utility_before': 1.0,
            'utility_after': round(second_utility, 12),
            'raw_nav2_path_length_m': round(second_bid.path_length_m, 12),
            'normalized_cost': round(second_cost, 12), 'beta': beta,
            'score': round(second_score, 12),
            'tie_break_order': 'pair_score_then_combined_cost_then_task_ids',
            'reductions': [],
        },
    )
    return first_score + second_score, trace


def choose_burgard_assignment(
        round_id: str, union: CanonicalUnion,
        robot1_bids: BidBatch, robot2_bids: BidBatch,
        *, beta: float = 1.0, path_cost_scale_m: float = 12.0,
        minimum_visible_gain_m: float = 0.05, sensor_max_range_m: float = 11.98,
        occupied_threshold: int = 50, shared_map=None,
        hard_failed_tasks: frozenset[str] = frozenset(),
        fixed_robot1_task_id: str = '',
        fixed_robot2_task_id: str = '') -> PairDecision:
    """Run the bounded exact two-robot Burgard-inspired assignment.

    ``U_t`` starts at one for every eligible canonical task.  ``C_i,t`` is the
    robot's existing Nav2 path length converted to a soft cost using the
    configured scale.  The scale is not a feasibility ceiling: valid paths
    longer than it remain eligible and incur proportionally larger cost. This
    is a project adaptation, not a claim that Burgard et al. used Nav2 or this
    normalization.
    """
    if beta < 0.0 or not math.isfinite(beta):
        raise ValueError('beta must be finite and non-negative')
    if path_cost_scale_m <= 0.0 or sensor_max_range_m <= 0.0:
        raise ValueError('path cost scale and sensor range must be positive')
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
                             minimum_visible_gain_m,
                         ))
        for robot_id in ('robot1', 'robot2')
    }
    for fixed_id, robot_id in (
            (fixed_robot1_task_id, 'robot1'),
            (fixed_robot2_task_id, 'robot2')):
        if fixed_id and fixed_id not in feasible[robot_id]:
            raise ValueError(
                '%s fixed task is not a valid bid in this round: %s' %
                (robot_id, fixed_id))
    eligible_tasks = sorted(set(feasible['robot1']) | set(feasible['robot2']))
    pair_candidates = []
    robot1_choices = ([fixed_robot1_task_id] if fixed_robot1_task_id else
                      ([IDLE_TASK_ID] if fixed_robot2_task_id and
                       not feasible['robot1'] else feasible['robot1']
                       if fixed_robot2_task_id else
                       [IDLE_TASK_ID] + feasible['robot1']))
    robot2_choices = ([fixed_robot2_task_id] if fixed_robot2_task_id else
                      ([IDLE_TASK_ID] if fixed_robot1_task_id and
                       not feasible['robot2'] else feasible['robot2']
                       if fixed_robot1_task_id else
                       [IDLE_TASK_ID] + feasible['robot2']))
    for robot1_task in robot1_choices:
        for robot2_task in robot2_choices:
            if robot1_task and robot2_task and robot1_task == robot2_task:
                continue
            if robot1_task and robot2_task:
                orders = (
                    ('robot1', robot1_task, 'robot2', robot2_task),
                    ('robot2', robot2_task, 'robot1', robot1_task),
                )
                scored_orders = [
                    _pair_candidate(
                        *order, tasks, bids, shared_map,
                        sensor_max_range_m, occupied_threshold, beta,
                        path_cost_scale_m,
                    )
                    for order in orders
                ]
                # If order scores tie, keep robot 1 as the deterministic
                # first reduction source, matching the old tie convention.
                selected_score, trace = max(
                    scored_orders,
                    key=lambda item: (round(item[0], 12),
                                      1 if item is scored_orders[0] else 0),
                )
            elif robot1_task:
                bid = bids['robot1'][robot1_task]
                selected_score = 1.0 - beta * _path_cost(
                    bid.path_length_m, path_cost_scale_m)
                trace = ({
                    'step': 1, 'robot_id': 'robot1', 'task_id': robot1_task,
                    'task_centroid': [round(tasks[robot1_task].centroid[0], 6),
                                      round(tasks[robot1_task].centroid[1], 6)],
                    'initial_utility': 1.0, 'utility_before': 1.0,
                    'raw_nav2_path_length_m': round(bid.path_length_m, 12),
                    'normalized_cost': round(_path_cost(
                        bid.path_length_m, path_cost_scale_m), 12),
                    'beta': beta, 'score': round(selected_score, 12),
                    'reductions': [],
                },)
            elif robot2_task:
                bid = bids['robot2'][robot2_task]
                selected_score = 1.0 - beta * _path_cost(
                    bid.path_length_m, path_cost_scale_m)
                trace = ({
                    'step': 1, 'robot_id': 'robot2', 'task_id': robot2_task,
                    'task_centroid': [round(tasks[robot2_task].centroid[0], 6),
                                      round(tasks[robot2_task].centroid[1], 6)],
                    'initial_utility': 1.0, 'utility_before': 1.0,
                    'raw_nav2_path_length_m': round(bid.path_length_m, 12),
                    'normalized_cost': round(_path_cost(
                        bid.path_length_m, path_cost_scale_m), 12),
                    'beta': beta, 'score': round(selected_score, 12),
                    'reductions': [],
                },)
            else:
                selected_score, trace = 0.0, ()
            combined_cost = sum(
                bids[robot_id][task_id].path_length_m
                for robot_id, task_id in (
                    ('robot1', robot1_task), ('robot2', robot2_task))
                if task_id
            )
            # Maximize Burgard pair utility, then prefer less travel and
            # deterministic canonical task IDs.  This is the only new
            # cooperation mechanism; no fairness or route term is added.
            active_count = int(bool(robot1_task)) + int(bool(robot2_task))
            rank = (-active_count, -round(selected_score, 12),
                    round(combined_cost, 12), robot1_task, robot2_task)
            pair_candidates.append((rank, robot1_task, robot2_task,
                                    selected_score, trace))
    _, robot1_task, robot2_task, pair_total, trace = min(
        pair_candidates, key=lambda item: item[0],
    )
    assigned = {'robot1': robot1_task, 'robot2': robot2_task}

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
    score = AssignmentScore(total=pair_total)
    decision_payload = {
        'round_id': round_id,
        'union_hash': union.union_hash,
        'robot1_bid_fingerprint': first_fingerprint,
        'robot2_bid_fingerprint': second_fingerprint,
        'robot1_task': assigned['robot1'],
        'robot2_task': assigned['robot2'],
        'burgard_total_millionths': round(score.total * 1_000_000),
        'pair_solver': 'exact_two_robot_burgard',
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
        # Kept as a compatibility diagnostic field; it no longer represents
        # a path-length threshold and is therefore always zero.
        rejected_path_threshold_count=0,
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
        feasible_path_limit_m=0.0,
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
