"""Pure exhaustive pair scoring for exactly Robot 1 and Robot 2."""

from dataclasses import dataclass
import hashlib
import json
import math
from typing import Iterable, Mapping, Optional, Sequence

from .canonical import bounds_iou
from .models import (
    AssignmentScore,
    AssignmentDiagnostics,
    Bid,
    BidBatch,
    CanonicalTask,
    CanonicalUnion,
    PairDecision,
    Point,
)


IDLE_TASK_ID = ''


@dataclass(frozen=True)
class AssignmentWeights:
    """Explicit normalized pair-utility weights and geometry scales."""

    gain: float = 3.0
    path: float = 1.0
    nearby_goal: float = 1.5
    route_overlap: float = 3.0
    hard_failure: float = 5.0
    sensing_overlap: float = 2.0
    workload_imbalance: float = 0.35
    visible_gain_scale: float = 5.0
    path_cost_scale_m: float = 12.0
    nearby_goal_distance_m: float = 0.60
    route_corridor_radius_m: float = 0.16
    sensing_approach_scale_m: float = 1.5
    minimum_useful_score: float = 1e-6
    minimum_visible_gain_m: float = 0.05
    maximum_path_length_m: float = 18.0


def _bounded(value: float) -> float:
    return max(0.0, min(1.0, value))


def _hash(payload: object) -> str:
    data = json.dumps(
        payload, sort_keys=True, separators=(',', ':'), allow_nan=False,
    ).encode('utf-8')
    return hashlib.sha256(data).hexdigest()


def bid_fingerprint(batch: BidBatch) -> str:
    """Fingerprint semantic bid data independent of message arrival order."""
    payload = []
    for bid in sorted(batch.bids, key=lambda item: item.canonical_task_id):
        payload.append({
            'id': bid.canonical_task_id,
            'valid': bid.path_valid,
            'length_mm': round(bid.path_length_m * 1000.0),
            'travel_milli': round(bid.estimated_travel_cost * 1000.0),
            'heading_milli': round(bid.heading_cost * 1000.0),
            'path_cm': [(round(x * 100.0), round(y * 100.0))
                        for x, y in bid.path],
        })
    return _hash({
        'round': batch.round_id,
        'union': batch.union_hash,
        'robot': batch.source_robot_id,
        'session': batch.source_session_id,
        'epoch': batch.source_snapshot_epoch,
        'bids': payload,
    })


def _path_overlap_one_way(
        first: Sequence[Point], second: Sequence[Point], radius: float) -> float:
    if not first or not second:
        return 0.0
    return sum(
        any(math.dist(point, other) <= 2.0 * radius for other in second)
        for point in first
    ) / len(first)


def route_overlap(
        first: Sequence[Point], second: Sequence[Point], corridor_radius_m: float) -> float:
    """Return bounded geometric corridor overlap, without traffic timing claims."""
    if not first or not second:
        return 0.0
    return _bounded(0.5 * (
        _path_overlap_one_way(first, second, corridor_radius_m) +
        _path_overlap_one_way(second, first, corridor_radius_m)
    ))


def nearby_goal_penalty(
        first: Point, second: Point, distance_scale_m: float) -> float:
    """Penalize distinct goals that occupy the same local work region."""
    if distance_scale_m <= 0.0:
        return 0.0
    return _bounded(1.0 - math.dist(first, second) / distance_scale_m)


def _visible_cell_overlap(first: Iterable[Point], second: Iterable[Point]) -> Optional[float]:
    quantum = 0.05
    first_set = {(round(x / quantum), round(y / quantum)) for x, y in first}
    second_set = {(round(x / quantum), round(y / quantum)) for x, y in second}
    if not first_set or not second_set:
        return None
    union = first_set | second_set
    return len(first_set & second_set) / len(union)


def sensing_overlap_estimate(
        first: CanonicalTask, second: CanonicalTask,
        approach_scale_m: float) -> float:
    """Use visible cells when present, otherwise a bounded geometry approximation."""
    cell_overlap = _visible_cell_overlap(first.visible_cells, second.visible_cells)
    if cell_overlap is not None:
        return _bounded(cell_overlap)
    first_bounds = first.visible_bounds or first.bounds
    second_bounds = second.visible_bounds or second.bounds
    viewpoint_overlap = _bounded(
        1.0 - math.dist(first.approach, second.approach) /
        max(approach_scale_m, 1e-9),
    )
    return _bounded(0.65 * bounds_iou(first_bounds, second_bounds) +
                    0.35 * viewpoint_overlap)


def _score_assignment(
        first_task: Optional[CanonicalTask], second_task: Optional[CanonicalTask],
        first_bid: Optional[Bid], second_bid: Optional[Bid],
        hard_failed_tasks: frozenset[str],
        weights: AssignmentWeights) -> AssignmentScore:
    tasks = tuple(task for task in (first_task, second_task) if task is not None)
    bids = tuple(bid for bid in (first_bid, second_bid) if bid is not None)
    gain = sum(_bounded(task.visible_reveal_gain / weights.visible_gain_scale)
               for task in tasks)
    path = sum(_bounded(bid.estimated_travel_cost / weights.path_cost_scale_m)
               for bid in bids)
    proximity = overlap = sensing = imbalance = 0.0
    if first_task is not None and second_task is not None:
        proximity = nearby_goal_penalty(
            first_task.approach, second_task.approach,
            weights.nearby_goal_distance_m,
        )
        overlap = route_overlap(
            first_bid.path, second_bid.path, weights.route_corridor_radius_m,
        )
        sensing = sensing_overlap_estimate(
            first_task, second_task, weights.sensing_approach_scale_m,
        )
        imbalance = _bounded(
            abs(first_bid.path_length_m - second_bid.path_length_m) /
            weights.path_cost_scale_m,
        )
    failures = sum(task.canonical_id in hard_failed_tasks for task in tasks)
    failure_penalty = _bounded(float(failures))
    total = (
        weights.gain * gain - weights.path * path -
        weights.nearby_goal * proximity -
        weights.route_overlap * overlap -
        weights.hard_failure * failure_penalty -
        weights.sensing_overlap * sensing -
        weights.workload_imbalance * imbalance
    )
    return AssignmentScore(
        team_visible_gain=gain,
        combined_path_cost=path,
        nearby_goal_penalty=proximity,
        route_overlap_penalty=overlap,
        hard_failure_penalty=failure_penalty,
        sensing_overlap_penalty=sensing,
        workload_imbalance_penalty=imbalance,
        total=total,
    )


def _bid_map(batch: BidBatch) -> Mapping[str, Bid]:
    result = {}
    for bid in batch.bids:
        if bid.canonical_task_id in result:
            raise ValueError('duplicate canonical task in bid batch')
        if not math.isfinite(bid.path_length_m) or bid.path_length_m < 0.0:
            continue
        if not math.isfinite(bid.estimated_travel_cost) or bid.estimated_travel_cost < 0.0:
            continue
        result[bid.canonical_task_id] = bid
    return result


def _task_feasible(
        task: CanonicalTask, bid: Optional[Bid],
        hard_failed_tasks: frozenset[str],
        weights: AssignmentWeights) -> bool:
    """Apply explicit assignment safety/utility feasibility gates.

    Soft pair utility is deliberately not part of this predicate.  A useful
    frontier may have negative absolute score after travel and overlap terms,
    but it must still be assignable when it is the best feasible work.
    """
    if bid is None or not bid.path_valid:
        return False
    if task.canonical_id in hard_failed_tasks:
        return False
    if not math.isfinite(task.visible_reveal_gain):
        return False
    if task.visible_reveal_gain < weights.minimum_visible_gain_m:
        return False
    if not math.isfinite(bid.path_length_m) or bid.path_length_m < 0.0:
        return False
    if bid.path_length_m > weights.maximum_path_length_m:
        return False
    if (not math.isfinite(bid.estimated_travel_cost) or
            bid.estimated_travel_cost < 0.0 or
            bid.estimated_travel_cost > weights.maximum_path_length_m):
        return False
    return True


def _assignment_rank(item: tuple) -> tuple:
    """Keep the deterministic pair ranking independent of selection policy."""
    return (
        -round(item[2].total, 12),
        round(item[3], 12),
        round(item[4], 12),
        item[0], item[1],
    )


def _is_conflict_free_two_active_assignment(item: tuple) -> bool:
    """Return whether a feasible pair has no cooperative-conflict signal.

    Feasibility is established before this predicate is evaluated.  Exact zero
    is intentional: the geometry helpers clamp genuinely separated work to
    0.0, while any nonzero value represents an existing soft conflict signal.
    """
    first_id, second_id, score, _, _ = item
    return bool(first_id and second_id) and (
        score.nearby_goal_penalty == 0.0 and
        score.route_overlap_penalty == 0.0 and
        score.sensing_overlap_penalty == 0.0 and
        score.hard_failure_penalty == 0.0
    )


def choose_pair_assignment(
        round_id: str, union: CanonicalUnion,
        robot1_bids: BidBatch, robot2_bids: BidBatch,
        hard_failed_tasks: frozenset[str] = frozenset(),
        weights: AssignmentWeights = AssignmentWeights()) -> PairDecision:
    """Exhaustively evaluate bounded ordered task pairs and idle cases."""
    for batch, robot_id in ((robot1_bids, 'robot1'), (robot2_bids, 'robot2')):
        if batch.round_id != round_id or batch.union_hash != union.union_hash:
            raise ValueError('bid batch does not reference the canonical round')
        if batch.source_robot_id != robot_id:
            raise ValueError('unexpected bidder identity')
    tasks = {task.canonical_id: task for task in union.tasks}
    first_map, second_map = _bid_map(robot1_bids), _bid_map(robot2_bids)
    valid_first = {
        task_id for task_id, bid in first_map.items()
        if task_id in tasks and _task_feasible(
            tasks[task_id], bid, hard_failed_tasks, weights,
        )
    }
    valid_second = {
        task_id for task_id, bid in second_map.items()
        if task_id in tasks and _task_feasible(
            tasks[task_id], bid, hard_failed_tasks, weights,
        )
    }
    choices1 = [IDLE_TASK_ID] + sorted(
        valid_first
    )
    choices2 = [IDLE_TASK_ID] + sorted(
        valid_second
    )
    candidates = []
    for first_id in choices1:
        for second_id in choices2:
            if first_id == second_id:
                continue
            first_task = tasks.get(first_id)
            second_task = tasks.get(second_id)
            first_bid = first_map.get(first_id)
            second_bid = second_map.get(second_id)
            score = _score_assignment(
                first_task, second_task, first_bid, second_bid,
                hard_failed_tasks, weights,
            )
            combined = sum(
                bid.path_length_m for bid in (first_bid, second_bid)
                if bid is not None
            )
            maximum = max(
                (bid.path_length_m for bid in (first_bid, second_bid)
                 if bid is not None), default=0.0,
            )
            candidates.append((first_id, second_id, score, combined, maximum))
    non_idle = [item for item in candidates
                if item[0] or item[1]]
    conflict_free_two_active = [
        item for item in non_idle
        if _is_conflict_free_two_active_assignment(item)
    ]
    if conflict_free_two_active:
        # Independent useful work should use both robots.  Existing soft
        # utility and deterministic ties still select which independent pair.
        selected = min(conflict_free_two_active, key=_assignment_rank)
    elif non_idle:
        selected = min(non_idle, key=_assignment_rank)
    else:
        selected = (IDLE_TASK_ID, IDLE_TASK_ID, AssignmentScore(), 0.0, 0.0)
    first_id, second_id, score, combined, maximum = selected
    best_non_idle = selected if non_idle else None
    bid_task_ids = set(first_map) | set(second_map)
    hard_rejected = bid_task_ids & hard_failed_tasks
    gain_rejected = {
        task_id for task_id in bid_task_ids if task_id in tasks and
        tasks[task_id].visible_reveal_gain < weights.minimum_visible_gain_m
    }
    path_rejected = {
        task_id for task_id, bid in {**first_map, **second_map}.items()
        if task_id in tasks and (
            bid.path_length_m > weights.maximum_path_length_m or
            bid.estimated_travel_cost > weights.maximum_path_length_m
        )
    }
    if not union.tasks:
        idle_reason = 'NO_TASKS'
    elif not bid_task_ids:
        idle_reason = 'NO_VALID_BIDS'
    elif not valid_first and not valid_second:
        if hard_rejected and hard_rejected >= bid_task_ids:
            idle_reason = 'FAILURE_SUPPRESSED'
        elif gain_rejected and gain_rejected >= bid_task_ids:
            idle_reason = 'BELOW_GAIN_THRESHOLD'
        elif path_rejected and path_rejected >= bid_task_ids:
            idle_reason = 'EXCESSIVE_BACKTRACK'
        else:
            idle_reason = 'NO_FEASIBLE_TASK'
    else:
        idle_reason = 'OTHER'
    if valid_first and valid_second:
        availability_reason = 'BOTH_ROBOTS_REACHABLE'
    elif valid_first:
        availability_reason = 'ONLY_ROBOT1_REACHABLE'
    elif valid_second:
        availability_reason = 'ONLY_ROBOT2_REACHABLE'
    else:
        availability_reason = 'NO_VALID_BIDS'
    rejected_equivalence = sum(
        max(0, len(task.members) - 1) for task in union.tasks
    )
    best_score = best_non_idle[2] if best_non_idle else AssignmentScore()
    diagnostics = AssignmentDiagnostics(
        idle_reason=idle_reason,
        availability_reason=availability_reason,
        union_task_count=len(union.tasks),
        robot1_bid_count=len(robot1_bids.bids),
        robot2_bid_count=len(robot2_bids.bids),
        robot1_valid_bid_count=len(valid_first),
        robot2_valid_bid_count=len(valid_second),
        reachable_by_both_count=len(valid_first & valid_second),
        reachable_only_robot1_count=len(valid_first - valid_second),
        reachable_only_robot2_count=len(valid_second - valid_first),
        rejected_equivalence_count=rejected_equivalence,
        rejected_failure_suppression_count=sum(
            task.canonical_id in hard_failed_tasks for task in union.tasks
        ),
        rejected_gain_threshold_count=len(gain_rejected),
        rejected_path_threshold_count=len(path_rejected),
        feasible_useful_robot1_count=len(valid_first),
        feasible_useful_robot2_count=len(valid_second),
        valid_one_active_assignment_count=sum(
            1 for item in non_idle if bool(item[0]) ^ bool(item[1])
        ),
        valid_two_active_pair_count=sum(
            1 for item in non_idle if item[0] and item[1]
        ),
        idle_idle_permitted=not bool(non_idle),
        robot1_idle_reason=(
            'NOT_IDLE' if first_id else (
                idle_reason if not second_id else
                'BEST_FEASIBLE_ASSIGNMENT_TO_ROBOT2'
            )
        ),
        robot2_idle_reason=(
            'NOT_IDLE' if second_id else (
                idle_reason if not first_id else
                'BEST_FEASIBLE_ASSIGNMENT_TO_ROBOT1'
            )
        ),
        best_non_idle_robot1_task_id=best_non_idle[0] if best_non_idle else '',
        best_non_idle_robot2_task_id=best_non_idle[1] if best_non_idle else '',
        best_non_idle_score=best_score,
    )
    first_fingerprint = bid_fingerprint(robot1_bids)
    second_fingerprint = bid_fingerprint(robot2_bids)
    decision_payload = {
        'round_id': round_id,
        'union_hash': union.union_hash,
        'robot1_bid_fingerprint': first_fingerprint,
        'robot2_bid_fingerprint': second_fingerprint,
        'robot1_task': first_id,
        'robot2_task': second_id,
        'score_millionths': {
            name: round(getattr(score, name) * 1_000_000)
            for name in score.__dataclass_fields__
        },
    }
    return PairDecision(
        round_id=round_id,
        union_hash=union.union_hash,
        robot1_task_id=first_id,
        robot2_task_id=second_id,
        robot1_bid_fingerprint=first_fingerprint,
        robot2_bid_fingerprint=second_fingerprint,
        score=score,
        decision_hash=_hash(decision_payload),
        combined_path_length_m=combined,
        maximum_path_length_m=maximum,
        diagnostics=diagnostics,
    )


def decisions_match(first: PairDecision, second: PairDecision) -> bool:
    """Require positive agreement on every decision-binding fingerprint."""
    return (
        first.round_id == second.round_id and
        first.union_hash == second.union_hash and
        first.robot1_bid_fingerprint == second.robot1_bid_fingerprint and
        first.robot2_bid_fingerprint == second.robot2_bid_fingerprint and
        first.robot1_task_id == second.robot1_task_id and
        first.robot2_task_id == second.robot2_task_id and
        first.decision_hash == second.decision_hash
    )
