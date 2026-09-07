"""Offline workload contribution and fairness summaries."""

from __future__ import annotations


def _jain(values):
    values = [float(value) for value in values]
    total = sum(values)
    denominator = len(values) * sum(value * value for value in values)
    return (total * total / denominator) if denominator else None


def _shares(values):
    total = sum(float(value) for value in values)
    return ([float(value) / total for value in values]
            if total else [None for value in values])


def _component_shares(components, robots, name):
    values = [components[robot].get(name) for robot in robots]
    if any(value is None for value in values):
        return {robot: None for robot in robots}
    return dict(zip(robots, _shares(values)))


def summarize_fairness(evaluation, robots=("robot1", "robot2")):
    """Summarize required per-robot workload without inventing missing data.

    The thesis-facing scalar is Jain's index over first-seen known-cell
    contribution.  Distance, goals, feasible work, avoidable idle and
    productive engagement are emitted as component series, not folded into a
    second opaque score.
    """
    robots = tuple(str(robot) for robot in robots)
    coverage = evaluation.get("coverage", {})
    ownership = coverage.get("ownership", {})
    if not ownership.get("available"):
        return {"available": False,
                "reason": ownership.get("reason", "ownership unavailable"),
                "robots": {robot: {} for robot in robots}}
    unique = ownership.get("unique_first_seen_cells", {})
    duplicate = ownership.get("later_duplicated_cells", {})
    motion = evaluation.get("motion", {})
    cooperation = evaluation.get("cooperation_summary", {})
    assignments = cooperation.get("assignments", {}).get("by_robot", {})
    terminals = cooperation.get("terminals", {})
    idle = evaluation.get("avoidable_idle", {}).get("robots", {})
    components = {}
    for robot in robots:
        terminal = terminals.get(robot, {})
        idle_row = idle.get(robot, {})
        components[robot] = {
            "first_seen_cells": int(unique.get(robot, 0)),
            "later_duplicate_cells": int(duplicate.get(robot, 0)),
            "distance_travelled_m": motion.get(robot, {}).get(
                "travel_distance_m"),
            "dispatched_goals": int(assignments.get(robot, 0)),
            "successful_goals": int(terminal.get("succeeded", 0)),
            "feasible_work_seconds": idle_row.get("feasible_work_seconds"),
            "avoidable_idle_seconds": idle_row.get("avoidable_idle_seconds"),
            "productive_engagement_seconds": idle_row.get(
                "productive_engagement_seconds"),
        }
    first_seen = [components[robot]["first_seen_cells"] for robot in robots]
    return {
        "available": True,
        "definition": {
            "workload_component": "first_seen_cells",
            "scalar": "Jain fairness index",
            "formula": "(sum(x)^2)/(n*sum(x^2))",
            "zero_total": "valid zero workload; index is null",
        },
        "robots": components,
        "first_seen_cell_shares": dict(zip(robots, _shares(first_seen))),
        "jain_first_seen_cell_fairness": _jain(first_seen),
        "component_shares": {
            name: _component_shares(components, robots, name)
            for name in ("first_seen_cells", "later_duplicate_cells",
                         "distance_travelled_m", "dispatched_goals",
                         "successful_goals")
        },
    }
