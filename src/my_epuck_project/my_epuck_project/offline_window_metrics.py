"""Simulation-time window summaries for already reconstructed evidence."""

from __future__ import annotations

DEFAULT_WINDOWS = ((0.0, 180.0), (180.0, 360.0), (360.0, 600.0),
                   (600.0, 900.0), (900.0, 1200.0))


def _time(item):
    value = item.get("sim_time_s", item.get("elapsed_s"))
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _in_window(item, start, end):
    value = _time(item)
    return value is not None and start <= value < end


def _count(items, start, end):
    return sum(_in_window(item, start, end) for item in items)


def _slice_segments(segments, start, end):
    output = []
    for item in segments:
        left = max(start, float(item.get("start_s", start)))
        right = min(end, float(item.get("end_s", end)))
        if right > left:
            row = dict(item)
            row["start_s"] = left
            row["end_s"] = right
            row["duration_s"] = right - left
            output.append(row)
    return output


def _coverage_window(series, start, end):
    points = [item for item in series if _in_window(item, start, end)]
    if not points:
        return {"available": False, "reason": "no coverage samples in window"}
    return {
        "available": True,
        "sample_count": len(points),
        "first_known_area_m2": points[0].get("area_m2"),
        "last_known_area_m2": points[-1].get("area_m2"),
        "known_area_gain_m2": (
            float(points[-1].get("area_m2", 0.0)) -
            float(points[0].get("area_m2", 0.0))),
        # A segment is assigned to a window only when both source samples are
        # in it.  This avoids inventing a boundary interpolation rule.
        "known_area_auc_m2_s": sum(
            max(0.0, float(second["sim_time_s"]) -
                float(first["sim_time_s"])) *
            (float(first.get("area_m2", 0.0)) +
             float(second.get("area_m2", 0.0))) / 2.0
            for first, second in zip(points, points[1:])),
    }


def _event_windows(evaluation, start, end):
    protocol = evaluation.get("cooperation", {})
    sources = {
        "candidate_batches": protocol.get("generation_records", []),
        "bids": protocol.get("bid_records", []),
        "pair_decisions": protocol.get("pair_decisions", []),
        "cooperation_events": protocol.get("cooperation_events", []),
        "dispatches": protocol.get("dispatches", []),
        "navigation_terminals": protocol.get("navigation_terminals", []),
        "certificate_observations": protocol.get("certificate_records", []),
    }
    return {name: _count(items, start, end)
            for name, items in sources.items()}


def _idle_windows(evaluation, start, end):
    robots = evaluation.get("avoidable_idle", {}).get("robots", {})
    result = {}
    for robot, row in robots.items():
        segments = _slice_segments(row.get("segments", []), start, end)
        feasible = [item for item in segments
                    if item.get("classification") ==
                    "FEASIBLE_WORK_AVAILABLE"]
        idle = [item for item in feasible if not item.get("goal_active")]
        feasible_s = sum(float(item["duration_s"]) for item in feasible)
        idle_s = sum(float(item["duration_s"]) for item in idle)
        result[robot] = {
            "feasible_work_seconds": feasible_s,
            "avoidable_idle_seconds": idle_s,
            "productive_engagement_seconds": max(0.0, feasible_s - idle_s),
            "longest_avoidable_idle_s": max(
                (float(item["duration_s"]) for item in idle), default=0.0),
            "segment_count": len(segments),
        }
    return result


def analyze_windows(evaluation, windows=DEFAULT_WINDOWS):
    """Return fixed simulation-time windows without zero-filling absent data."""
    clock_end = evaluation.get("clock", {}).get("end_s")
    try:
        clock_end = float(clock_end)
    except (TypeError, ValueError):
        clock_end = None
    coverage_series = evaluation.get("coverage", {}).get("series", [])
    anomaly_events = []
    for row in evaluation.get("motion_anomalies", {}).get("robots", {}).values():
        anomaly_events.extend(row.get("events", []))
    output = []
    for start, end in windows:
        row = {"start_sim_time_s": start, "end_sim_time_s": end,
               "boundary": "[start,end)"}
        if clock_end is None or clock_end < end:
            row.update({"status": "NOT_AVAILABLE",
                        "reason": "artifact horizon does not reach window end"})
            output.append(row)
            continue
        row.update({
            "status": "AVAILABLE",
            "coverage": _coverage_window(coverage_series, start, end),
            "events": _event_windows(evaluation, start, end),
            "idle": _idle_windows(evaluation, start, end),
            "motion_anomaly_events": _count(anomaly_events, start, end),
        })
        output.append(row)
    return {
        "available": any(item["status"] == "AVAILABLE" for item in output),
        "time_basis": "simulation time",
        "boundary_rule": "half-open [start,end); no post-boundary sample",
        "windows": output,
    }
