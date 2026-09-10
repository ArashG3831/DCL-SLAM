#!/usr/bin/env python3
"""Generate one deterministic offline report from a finalized fast_trial.

This tool is deliberately offline-only.  It reads the observer JSON/JSONL/CSV
artifacts, reuses :mod:`my_epuck_project.offline_timing_metrics` for the
authoritative feasible-work/avoidable-idle state machine, and never imports
ROS, starts Webots, or writes inside the source tree.  The output JSON is the
machine-readable source for the compact Markdown report.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
import re
import shutil
import statistics
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path


HERE = Path(__file__).resolve()
PACKAGE_ROOT = HERE.parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from my_epuck_project.offline_timing_metrics import analyze_event_records


SCHEMA_VERSION = "offline_run_metrics_1.0"
ROBOTS = ("robot1", "robot2")
EPS = 1.0e-9


def _json(path, default=None):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError):
        return default


def _jsonl(path):
    path = Path(path)
    if not path.is_file():
        return []
    rows = []
    with path.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            if not line.strip():
                continue
            try:
                rows.append(json.loads(line))
            except ValueError:
                continue
    return rows


def _csv(path):
    path = Path(path)
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8", errors="replace") as stream:
        return list(csv.DictReader(stream))


def _num(value, default=None):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _int(value, default=0):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _bool(value, default=None):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        if value.lower() in ("true", "1", "yes"):
            return True
        if value.lower() in ("false", "0", "no"):
            return False
    return default


def _time(row):
    return _num(row.get("elapsed_s", row.get("capture_elapsed_s",
                                                row.get("sim_time_s"))))


def _pct(values, fraction):
    values = sorted(float(value) for value in values if _num(value) is not None)
    if not values:
        return None
    index = min(len(values) - 1, max(0, int(math.ceil(
        fraction * len(values))) - 1))
    return values[index]


def _stat(values):
    values = [float(value) for value in values if _num(value) is not None]
    if not values:
        return {"count": 0, "mean": None, "median": None, "p90": None,
                "p95": None, "max": None}
    return {
        "count": len(values), "mean": statistics.fmean(values),
        "median": statistics.median(values), "p90": _pct(values, .90),
        "p95": _pct(values, .95), "max": max(values),
    }


def _round(value, digits=6):
    if isinstance(value, float):
        return round(value, digits)
    return value


def _round_tree(value):
    if isinstance(value, dict):
        return {key: _round_tree(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_round_tree(item) for item in value]
    if isinstance(value, float):
        return _round(value)
    return value


def _observer_dir(artifact):
    candidates = sorted(Path(artifact).rglob("events.jsonl"))
    if not candidates:
        raise ValueError(f"no observer/events.jsonl below {artifact}")
    preferred = [path for path in candidates if "observer" in path.parts]
    return (preferred or candidates)[0].parent


def _first_json(artifact, name):
    paths = sorted(Path(artifact).rglob(name))
    return _json(paths[0]) if paths else {}


def _load_run(artifact):
    artifact = Path(artifact).resolve()
    observer = _observer_dir(artifact)
    events = _jsonl(observer / "events.jsonl")
    rosout = _jsonl(observer / "rosout_receipts.jsonl")
    frontier = _jsonl(observer / "frontier_regions.jsonl")
    map_receipts = _jsonl(observer / "map_receipts.jsonl")
    nav2 = _jsonl(observer / "nav2_diagnostics.jsonl")
    goal_ledger = _jsonl(observer / "goal_decision_ledger.jsonl")
    fast_summary = _first_json(artifact, "fast_trial_summary.json")
    summary = _json(observer / "summary.json", {}) or {}
    manifest = _json(observer / "run_manifest.json", {}) or {}
    mission_result = _json(observer / "mission_result.json", {}) or {}
    finalization = _json(observer / "raw_evidence_finalization.json", {}) or {}
    artifact_finalization = _json(observer / "artifact_finalization.json", {}) or {}
    return {
        "artifact": artifact, "observer": observer, "events": events,
        "rosout": rosout, "frontier": frontier, "map_receipts": map_receipts,
        "nav2": nav2, "goal_ledger": goal_ledger,
        "coverage_rows": _csv(observer / "coverage.csv"),
        "topic_health_rows": _csv(observer / "topic_health.csv"),
        "timeseries": {robot: _csv(observer / f"{robot}_timeseries.csv")
                        for robot in ROBOTS},
        "fast_summary": fast_summary, "summary": summary,
        "manifest": manifest, "mission_result": mission_result,
        "finalization": finalization,
        "artifact_finalization": artifact_finalization,
    }


def _end_sim(run):
    fast = run["fast_summary"]
    # The fast-trial summary is the authoritative measurement cutoff. The
    # mission-result duration and observer records can include finalization
    # callbacks after the simulation horizon; counting those would make a
    # bounded report silently include teardown work as runtime work.
    authoritative = _num(fast.get("simulation_end_time_s"))
    if authoritative is not None:
        return authoritative
    values = [
        _num(run["summary"].get("run", {}).get("end_s")),
        _num(run["mission_result"].get("simulated_duration_s")),
    ]
    values.extend(_time(row) for row in run["events"])
    return max(value for value in values if value is not None)


def _horizon(run, end_sim):
    return _num(run["fast_summary"].get("simulation_horizon_s"), end_sim)


def _cut(rows, end_sim):
    return [row for row in rows if (_time(row) is None or _time(row) <= end_sim + 1e-7)]


def _readiness(run):
    telemetry = run["fast_summary"].get("nav2_readiness_telemetry") or {}
    shared = telemetry.get("shared_activation_reached_at_sim_s") or {}
    values = [_num(value) for value in shared.values()]
    values = [value for value in values if value is not None]
    return max(values, default=0.0)


def _event_counts(events, robot=None):
    rows = events if robot is None else [
        row for row in events if row.get("robot_id") == robot]
    return dict(Counter(row.get("event_type", "UNKNOWN") for row in rows))


def _message_json(event):
    try:
        value = json.loads(str(event.get("message", "")))
    except (TypeError, ValueError):
        return {}
    return value if isinstance(value, dict) else {}


def _parse_kv(message):
    return {key: value for key, value in re.findall(
        r"([A-Za-z_][A-Za-z0-9_]*)=([^\s]+)", str(message))}


def _robot_from_name(name):
    text = str(name or "")
    for robot in ROBOTS:
        if text.startswith(robot + ".") or ("/" + robot + "/") in text:
            return robot
    return None


def _source_session(row):
    return row.get("source_session_id") or row.get("source_session")


def _frontier_metrics(run):
    snapshots = [row for row in run["frontier"]
                 if _time(row) is not None]
    by_robot = {robot: [] for robot in ROBOTS}
    for row in snapshots:
        if row.get("robot_id") in by_robot:
            by_robot[row["robot_id"]].append(row)

    def one(rows):
        counts = [len(row.get("regions") or []) for row in rows]
        sizes = []
        ids = set()
        nnd = []
        close = {"0.06_m": 0, "0.09_m": 0, "0.12_m": 0, "0.20_m": 0}
        for snapshot in rows:
            regions = snapshot.get("regions") or []
            for region in regions:
                if region.get("cell_count") is not None:
                    sizes.append(_int(region.get("cell_count")))
                identity = (region.get("id") if region.get("id") is not None
                            else region.get("physical_id", region.get("canonical_id")))
                if identity is not None:
                    ids.add(str(identity))
            points = []
            for region in regions:
                point = region.get("centroid")
                if isinstance(point, (list, tuple)) and len(point) >= 2:
                    x, y = _num(point[0]), _num(point[1])
                    if x is not None and y is not None:
                        points.append((x, y))
            for index, point in enumerate(points):
                distances = [math.hypot(point[0] - other[0],
                                        point[1] - other[1])
                             for other_index, other in enumerate(points)
                             if index != other_index]
                if distances:
                    nnd.append(min(distances))
            for index, point in enumerate(points):
                for other in points[index + 1:]:
                    distance = math.hypot(point[0] - other[0],
                                          point[1] - other[1])
                    for threshold in (0.06, 0.09, 0.12, 0.20):
                        if distance < threshold:
                            close[f"{threshold:.2f}_m"] += 1
        size_bins = {
            "2_3_cells": sum(2 <= size <= 3 for size in sizes),
            "4_10_cells": sum(4 <= size <= 10 for size in sizes),
            "gt10_cells": sum(size > 10 for size in sizes),
        }
        total_sizes = len(sizes)
        return {
            "snapshot_count": len(rows), "region_observation_count": len(sizes),
            "mean_regions": statistics.fmean(counts) if counts else None,
            "median_regions": statistics.median(counts) if counts else None,
            "p90_regions": _pct(counts, .90), "max_regions": max(counts, default=None),
            "final_regions": counts[-1] if counts else None,
            "size_bins": size_bins,
            "size_bin_shares": {key: (value / total_sizes if total_sizes else None)
                                for key, value in size_bins.items()},
            "median_region_size": statistics.median(sizes) if sizes else None,
            "p90_region_size": _pct(sizes, .90), "max_region_size": max(sizes, default=None),
            "nearest_neighbor_centroid_m": _stat(nnd),
            "centroid_pair_counts_closer_than": close,
            "unique_frontier_ids_seen": len(ids),
        }

    result = {robot: one(by_robot[robot]) for robot in ROBOTS}
    result["combined"] = one([row for robot in ROBOTS for row in by_robot[robot]])
    result["source"] = "observer/frontier_regions.jsonl; region records and recorded centroids"
    return result


def _generation_metrics(run):
    batches = {robot: sorted([
        row for row in run["events"]
        if row.get("event_type") == "CANDIDATE_BATCH_RECEIVED"
        and row.get("robot_id") == robot], key=lambda row: _time(row) or 0.0)
               for robot in ROBOTS}
    snapshots = {robot: sorted([
        row for row in run["events"]
        if row.get("event_type") == "DISTRIBUTED_TASK_SNAPSHOT"
        and row.get("robot_id") == robot], key=lambda row: _time(row) or 0.0)
                 for robot in ROBOTS}

    def gaps(rows):
        values = [_time(row) for row in rows]
        values = [value for value in values if value is not None]
        gap_values = [second - first for first, second in zip(values, values[1:])]
        return {
            "first": values[0] if values else None,
            "last": values[-1] if values else None,
            "mean": statistics.fmean(gap_values) if gap_values else None,
            "median": statistics.median(gap_values) if gap_values else None,
            "p95": _pct(gap_values, .95), "max": max(gap_values, default=None),
            "tail_at_horizon": (run["end_sim"] - values[-1]
                                if values else None),
        }

    def one(robot):
        rows = batches[robot]
        task_rows = snapshots[robot]
        candidate_counts = [
            _int(row.get("candidate_count"), len(row.get("candidates") or []))
            for row in rows]
        task_counts = [len(row.get("tasks") or []) for row in task_rows]
        fields = {
            "detected": "detected_frontier_count",
            "unreachable": "unreachable_frontier_count",
            "detected_not_queried": "detected_not_queried_count",
            "small": "small_frontier_count",
            "planner_failure": "planner_failure_count",
            "out_of_range": "out_of_range_frontier_count",
            "unclassified": "unclassified_frontier_count",
        }
        status = {}
        for name, field in fields.items():
            values = [_int(row.get(field), 0) for row in rows]
            status[name] = {
                "total_observations": sum(values),
                "mean_per_batch": statistics.fmean(values) if values else None,
                "max": max(values, default=None),
                "final": values[-1] if values else None,
            }
        reachable_values = [
            _int(row.get("candidate_count"), len(row.get("candidates") or []))
            for row in rows]
        status["reachable"] = {
            "total_observations": sum(reachable_values),
            "mean_per_batch": statistics.fmean(reachable_values) if reachable_values else None,
            "max": max(reachable_values, default=None),
            "final": reachable_values[-1] if reachable_values else None,
            "source": "candidate_count; FrontierCandidateArray contains reachable candidates",
        }
        status["other"] = {
            "total_observations": None,
            "mean_per_batch": None,
            "max": None,
            "final": None,
            "availability_reason": "unclassified_frontier_count is exported separately and overlaps detected_not_queried in this artifact",
        }
        return {
            "candidate_batches": len(rows),
            "first_candidate_batch": _time(rows[0]) if rows else None,
            "last_candidate_batch": _time(rows[-1]) if rows else None,
            "candidate_gaps": gaps(rows),
            "candidate_count_per_batch": _stat(candidate_counts),
            "task_snapshots": len(task_rows),
            "first_task_snapshot": _time(task_rows[0]) if task_rows else None,
            "last_task_snapshot": _time(task_rows[-1]) if task_rows else None,
            "task_snapshot_gaps": gaps(task_rows),
            "tasks_per_snapshot": _stat(task_counts),
            "statuses": status,
        }

    result = {robot: one(robot) for robot in ROBOTS}
    result["combined"] = {
        "candidate_batches": sum(result[robot]["candidate_batches"] for robot in ROBOTS),
        "task_snapshots": sum(result[robot]["task_snapshots"] for robot in ROBOTS),
        "candidate_count_per_batch": _stat([
            _int(row.get("candidate_count"), len(row.get("candidates") or []))
            for row in run["events"] if row.get("event_type") == "CANDIDATE_BATCH_RECEIVED"]),
        "tasks_per_snapshot": _stat([len(row.get("tasks") or []) for row in run["events"]
                                      if row.get("event_type") == "DISTRIBUTED_TASK_SNAPSHOT"]),
        "statuses": {},
    }
    for name in ("detected", "reachable", "unreachable", "detected_not_queried",
                 "small", "planner_failure", "out_of_range", "unclassified", "other"):
        result["combined"]["statuses"][name] = {
            key: (sum(result[robot]["statuses"][name][key] or 0 for robot in ROBOTS)
                  if all(result[robot]["statuses"][name].get(key) is not None
                         for robot in ROBOTS) else None)
            for key in ("total_observations", "max", "final")}
        total = result["combined"]["statuses"][name]["total_observations"]
        result["combined"]["statuses"][name]["mean_per_batch"] = (
            total / max(1, result["combined"]["candidate_batches"])
            if total is not None else None)
        if name == "other":
            result["combined"]["statuses"][name]["availability_reason"] = (
                "unclassified_frontier_count is exported separately and overlaps detected_not_queried in this artifact")
    return result


def _query_metrics(run):
    lifecycle = []
    boundary_counts = {robot: Counter() for robot in ROBOTS}
    boundary_ids = {robot: {"before": set(), "after": set(), "future": set()}
                    for robot in ROBOTS}
    pattern = re.compile(r"FRONTIER_QUERY_LIFECYCLE\s+(.*)")
    for receipt in run["rosout"]:
        if _time(receipt) is not None and _time(receipt) > run["end_sim"] + 1e-7:
            continue
        robot = _robot_from_name(receipt.get("name"))
        if not robot:
            continue
        message = str(receipt.get("message", ""))
        match = pattern.search(message)
        if match:
            values = _parse_kv(match.group(1))
            values["time"] = _time(receipt)
            values["robot"] = robot
            values["node"] = str(receipt.get("name", ""))
            values["state"] = values.get("state", "")
            lifecycle.append(values)
        for label, token in (("before", "ASYNC_SEND_GOAL_BEFORE"),
                             ("after", "ASYNC_SEND_GOAL_AFTER"),
                             ("future", "ASYNC_SEND_GOAL_FUTURE_RECEIVED")):
            if f"event={token}" in message:
                boundary_counts[robot][label] += 1
                identity = _parse_kv(message).get("request_id")
                if identity:
                    boundary_ids[robot][label].add(identity)

    by_robot = {robot: [row for row in lifecycle if row["robot"] == robot
                        and ".local_" not in row["node"]]
                for robot in ROBOTS}

    def one(robot):
        rows = by_robot[robot]
        raw = Counter(row["state"] for row in rows)
        requests = []
        active = {}
        for row in rows:
            if row["state"] == "REQUEST_SENT":
                key = (row["node"], row.get("query_id", ""), row.get("id", ""))
                request = {"key": key, "id": row.get("id", ""), "events": [row]}
                requests.append(request)
                active[key] = request
            else:
                key = (row["node"], row.get("query_id", ""), row.get("id", ""))
                if key in active:
                    active[key]["events"].append(row)
        ids = [request["id"] for request in requests if request["id"]]
        id_counts = Counter(ids)
        terminal = {"RESULT_RECEIVED", "STALE_REVISION_REJECTED",
                    "QUERY_TIMEOUT", "QUERY_CANCEL_REQUESTED"}
        result_statuses = Counter()
        stale_only = 0
        result_and_stale = 0
        no_terminal = 0
        for request in requests:
            states = {row["state"] for row in request["events"]}
            result_rows = [row for row in request["events"]
                           if row["state"] == "RESULT_RECEIVED"]
            for result in result_rows:
                if result.get("status"):
                    result_statuses[result["status"]] += 1
            has_result = bool(result_rows)
            has_stale = "STALE_REVISION_REJECTED" in states
            if has_stale and not has_result:
                stale_only += 1
            elif has_stale and has_result:
                result_and_stale += 1
            if not states.intersection(terminal):
                no_terminal += 1
        return {
            "submissions": len(requests),
            "distinct_stable_frontier_ids": len(id_counts),
            "repeat_submissions_beyond_first": max(0, len(ids) - len(id_counts)),
            "stable_ids_queried_repeatedly": sum(count > 1 for count in id_counts.values()),
            "max_requests_for_one_stable_id": max(id_counts.values(), default=0),
            "raw_lifecycle_event_counts": dict(raw),
            "raw_result_callbacks": raw.get("RESULT_RECEIVED", 0),
            "result_status_counts": dict(result_statuses),
            "raw_stale_rejection_records": raw.get("STALE_REVISION_REJECTED", 0),
            "raw_stale_rejections_per_submission": (
                raw.get("STALE_REVISION_REJECTED", 0) / len(requests)
                if requests else None),
            "stale_only_request_segments": stale_only,
            "stale_only_request_segment_rate": stale_only / len(requests) if requests else None,
            "result_and_stale_request_segments": result_and_stale,
            "timeouts": raw.get("QUERY_TIMEOUT", 0),
            "watchdog_cancellations": raw.get("QUERY_CANCEL_REQUESTED", 0),
            "no_terminal_at_cutoff": no_terminal,
            "cache_satisfied_records": raw.get("CACHE_SATISFIED", 0),
            "raw_before_count": boundary_counts[robot]["before"],
            "raw_after_count": boundary_counts[robot]["after"],
            "raw_future_count": boundary_counts[robot]["future"],
            "async_pairing_reliable": False,
            "async_missing_after": None,
            "async_pairing_reason": "diagnostic request IDs are not a complete one-to-one join key",
        }

    result = {robot: one(robot) for robot in ROBOTS}
    combined_request_ids = [
        row.get("id") for robot in ROBOTS for row in by_robot[robot]
        if row.get("state") == "REQUEST_SENT" and row.get("id")]
    combined_id_counts = Counter(combined_request_ids)
    local_rows = [row for row in lifecycle if ".local_" in row["node"]]
    local_raw = Counter(row["state"] for row in local_rows)
    result["combined"] = {
        "submissions": sum(result[robot]["submissions"] for robot in ROBOTS),
        "distinct_stable_frontier_ids": len(combined_id_counts),
        "repeat_submissions_beyond_first": max(0, len(combined_request_ids) - len(combined_id_counts)),
        "stable_ids_queried_repeatedly": sum(count > 1 for count in combined_id_counts.values()),
        "max_requests_for_one_stable_id": max(combined_id_counts.values(), default=0),
        "raw_result_callbacks": sum(result[robot]["raw_result_callbacks"] for robot in ROBOTS),
        "raw_stale_rejection_records": sum(result[robot]["raw_stale_rejection_records"] for robot in ROBOTS),
        "stale_only_request_segments": sum(result[robot]["stale_only_request_segments"] for robot in ROBOTS),
        "timeouts": sum(result[robot]["timeouts"] for robot in ROBOTS),
        "watchdog_cancellations": sum(result[robot]["watchdog_cancellations"] for robot in ROBOTS),
        "no_terminal_at_cutoff": sum(result[robot]["no_terminal_at_cutoff"] for robot in ROBOTS),
        "cache_satisfied_records": sum(result[robot]["cache_satisfied_records"] for robot in ROBOTS),
        "raw_before_count": sum(result[robot]["raw_before_count"] for robot in ROBOTS),
        "raw_after_count": sum(result[robot]["raw_after_count"] for robot in ROBOTS),
        "raw_future_count": sum(result[robot]["raw_future_count"] for robot in ROBOTS),
        "async_pairing_reliable": False,
        "async_missing_after": None,
        "async_pairing_reason": "diagnostic request IDs are not a complete one-to-one join key",
        "pre_handoff_local": {
            "submissions": local_raw.get("REQUEST_SENT", 0),
            "raw_result_callbacks": local_raw.get("RESULT_RECEIVED", 0),
            "raw_stale_rejection_records": local_raw.get("STALE_REVISION_REJECTED", 0),
            "cache_satisfied_records": local_raw.get("CACHE_SATISFIED", 0),
        },
    }
    result["combined"]["raw_stale_rejections_per_submission"] = (
        result["combined"]["raw_stale_rejection_records"] /
        result["combined"]["submissions"] if result["combined"]["submissions"] else None)
    result["combined"]["stale_only_request_segment_rate"] = (
        result["combined"]["stale_only_request_segments"] /
        result["combined"]["submissions"] if result["combined"]["submissions"] else None)
    result["source"] = "rosout_receipts.jsonl FRONTIER_QUERY_LIFECYCLE records"
    result["pre_handoff_local_submissions"] = sum(
        1 for row in lifecycle if row["state"] == "REQUEST_SENT" and ".local_" in row["node"])
    return result


def _certificate_metrics(run):
    rows = [row for row in run["events"]
            if row.get("event_type") == "COST_ONLY_DISPATCH_CERTIFICATE"]

    def one(robot):
        records = [row for row in rows if row.get("robot_id") == robot]
        reasons = Counter()
        evidence = Counter()
        unqueried = []
        blocking = []
        for row in records:
            message = _message_json(row)
            reason = message.get("reason") or row.get("result") or "UNKNOWN"
            evidence_reason = message.get("evidence_reason") or message.get("evidence_branch")
            reasons[str(reason)] += 1
            if evidence_reason:
                evidence[str(evidence_reason)] += 1
            unqueried.append(_int(message.get("detected_not_queried_count"), 0))
            blocking.append(_int(message.get("blocking_unqueried_candidates"), 0))
        successful = sum(1 for row in records if _message_json(row).get("dispatch_certified") is True)
        deferred = len(records) - successful
        wait = _idle_attribution(run, run["timing"])[robot]["categories"].get("certificate_waiting")
        return {
            "attempts": len(records), "successful": successful, "deferred": deferred,
            "reason_counts": dict(reasons), "evidence_reason_counts": dict(evidence),
            "detected_not_queried_counts": {"total": sum(unqueried), "max": max(unqueried, default=None)},
            "blocking_unqueried_counts": {"total": sum(blocking), "max": max(blocking, default=None)},
            "provenance_context_mismatch_events": sum(
                1 for value in list(reasons) + list(evidence)
                if "MISMATCH" in value or "mismatch" in value),
            "certificate_wait_seconds": wait,
            "certificate_wait_availability_reason": "derived using established idle attribution event boundaries",
        }

    result = {robot: one(robot) for robot in ROBOTS}
    result["combined"] = {
        "attempts": sum(result[robot]["attempts"] for robot in ROBOTS),
        "successful": sum(result[robot]["successful"] for robot in ROBOTS),
        "deferred": sum(result[robot]["deferred"] for robot in ROBOTS),
        "reason_counts": dict(Counter({key: result["robot1"]["reason_counts"].get(key, 0) +
                                        result["robot2"]["reason_counts"].get(key, 0)
                                        for key in set(result["robot1"]["reason_counts"]) |
                                        set(result["robot2"]["reason_counts"])})),
        "evidence_reason_counts": dict(Counter({key: result["robot1"]["evidence_reason_counts"].get(key, 0) +
                                                 result["robot2"]["evidence_reason_counts"].get(key, 0)
                                                 for key in set(result["robot1"]["evidence_reason_counts"]) |
                                                 set(result["robot2"]["evidence_reason_counts"])})),
        "detected_not_queried_counts": {"total": sum(result[robot]["detected_not_queried_counts"]["total"] for robot in ROBOTS),
                                         "max": max(result[robot]["detected_not_queried_counts"]["max"] or 0 for robot in ROBOTS)},
        "blocking_unqueried_counts": {"total": sum(result[robot]["blocking_unqueried_counts"]["total"] for robot in ROBOTS),
                                       "max": max(result[robot]["blocking_unqueried_counts"]["max"] or 0 for robot in ROBOTS)},
        "provenance_context_mismatch_events": sum(result[robot]["provenance_context_mismatch_events"] for robot in ROBOTS),
        "certificate_wait_seconds": sum(result[robot]["certificate_wait_seconds"] or 0.0 for robot in ROBOTS),
        "certificate_wait_availability_reason": "derived using established idle attribution event boundaries",
    }
    return result


def _apply_idle_event(state, event):
    kind = event.get("event_type")
    message = str(event.get("message", ""))
    payload = _message_json(event)
    if kind == "TRAFFIC_WAITING":
        state["traffic"] = True
    elif kind == "TRAFFIC_CONFLICT_CLEARED":
        state["traffic"] = False
    elif kind == "COST_ONLY_DISPATCH_CERTIFICATE":
        state["certificate"] = payload.get("dispatch_certified") is not True
    elif kind in ("DISTRIBUTED_PAIR_DECISION", "DECISION_AGREED",
                  "CONTINUATION_DECISION_AGREED"):
        state["decision"] = True
    elif kind == "NAV_GOAL_SENT":
        state["decision"] = False
        state["round"] = False
        state["post_navigation"] = False
    elif kind in ("NAVIGATION_SUCCEEDED", "NAVIGATION_FAILED",
                  "NAVIGATION_CANCELED", "NAVIGATION_CANCELLED"):
        state["post_navigation"] = True
        state["decision"] = False
        state["round"] = False
    elif kind == "STATE_TRANSITION":
        if "local ComputePathToPose query lease" in message:
            state["path_lease"] = True
        else:
            state["path_lease"] = False
        if any(token in message for token in (
                "new canonical round", "new continuation round",
                "waiting for valid", "waiting for peer bids",
                "fresh task snapshots", "round invalidated")):
            state["round"] = True
        if "navigation terminal result" in message:
            state["post_navigation"] = True
    elif kind == "ROUND_INVALIDATED":
        state["round"] = True


def _idle_attribution(run, timing):
    """Apply the established report's mutually-exclusive event-boundary labels.

    The prior 600 s attribution report is the semantic source: traffic,
    certificate, lease, decision, post-navigation, round/bid, peer/busy, and
    other.  The authoritative feasible/idle intervals themselves come from
    ``offline_timing_metrics.analyze_event_records``.
    """
    result = {}
    for robot in ROBOTS:
        idle_segments = [segment for segment in timing[robot]["segments"]
                         if segment["classification"] == "FEASIBLE_WORK_AVAILABLE"
                         and not segment["goal_active"]]
        events = sorted([row for row in run["events"] if row.get("robot_id") == robot],
                        key=lambda row: (_time(row) or 0.0, _int(row.get("event_sequence"))))
        totals = Counter()
        ledger = []
        for idle in idle_segments:
            start, end = idle["start_s"], idle["end_s"]
            local_events = [row for row in events if start - EPS <= (_time(row) or 0.0) <= end + EPS]
            boundaries = sorted({start, end} | {
                _time(row) for row in local_events
                if _time(row) is not None and start < _time(row) < end})
            state = {"traffic": False, "certificate": False, "path_lease": False,
                     "decision": False, "post_navigation": False, "round": False,
                     "peer_busy": False}
            for row in events:
                if (_time(row) or 0.0) <= start + EPS:
                    _apply_idle_event(state, row)
            pieces = Counter()
            for left, right in zip(boundaries, boundaries[1:]):
                for row in local_events:
                    if abs((_time(row) or 0.0) - left) <= EPS:
                        _apply_idle_event(state, row)
                if state["traffic"]:
                    category = "traffic_waiting"
                elif state["certificate"]:
                    category = "certificate_waiting"
                elif state["path_lease"]:
                    category = "path_query_lease_waiting"
                elif state["decision"]:
                    category = "decision_agreement_dispatch"
                elif state["post_navigation"]:
                    category = "post_navigation_handoff"
                elif state["peer_busy"]:
                    category = "peer_busy_commitment"
                elif state["round"]:
                    category = "round_peer_bid_coordination"
                else:
                    category = "other_allocator_lifecycle"
                pieces[category] += right - left
            for category, seconds in pieces.items():
                totals[category] += seconds
            ledger.append({"start_s": start, "end_s": end,
                           "duration_s": end - start,
                           "categories": dict(pieces)})
        idle_seconds = sum(totals.values())
        result[robot] = {
            "categories": {key: round(value, 6) for key, value in sorted(totals.items())},
            "total_attributed_idle_seconds": round(idle_seconds, 6),
            "ledger": ledger,
            "definition_source": "condition_C_600s_idle_attribution_20260909.md event-boundary semantics",
        }
        if abs(idle_seconds - timing[robot]["avoidable_idle_seconds"]) > 0.12:
            result[robot]["validation_warning"] = "attribution categories do not cover aggregate idle within tolerance"
    result["combined"] = {
        "categories": {key: round(sum(result[robot]["categories"].get(key, 0.0)
                                       for robot in ROBOTS), 6)
                        for key in sorted(set().union(*[
                            set(result[robot]["categories"]) for robot in ROBOTS]))},
        "total_attributed_idle_seconds": round(sum(
            result[robot]["total_attributed_idle_seconds"] for robot in ROBOTS), 6),
        "definition_source": result["robot1"]["definition_source"],
    }
    return result


def _allocator_metrics(run):
    events = run["events"]
    by_robot = {robot: [row for row in events if row.get("robot_id") == robot]
                for robot in ROBOTS}
    health_pattern = re.compile(r"ALLOCATOR_HEALTH\s+(.*)")
    health = {robot: [] for robot in ROBOTS}
    for receipt in run["rosout"]:
        match = health_pattern.search(str(receipt.get("message", "")))
        if not match:
            continue
        values = _parse_kv(match.group(1))
        robot = values.get("robot")
        if robot in health:
            health[robot].append(values)

    def one(robot):
        rows = by_robot[robot]
        counters = Counter(row.get("event_type") for row in rows)
        final = health[robot][-1] if health[robot] else {}
        state_counts = Counter(row.get("next_state") for row in rows
                               if row.get("event_type") == "STATE_TRANSITION")
        rebase = [row for row in rows if "rebase" in str(row.get("message", "")).lower()
                  and "decision" in str(row.get("message", "")).lower()]
        return {
            "rounds_created": _int(final.get("rounds_created"), counters.get("CONTINUATION_ROUND_STARTED", 0)),
            "rounds_replaced": _int(final.get("rounds_replaced"), 0),
            "rounds_completed": _int(final.get("rounds_completed"), 0),
            "continuation_rounds": counters.get("CONTINUATION_ROUND_STARTED", 0),
            "bid_arrays": counters.get("DISTRIBUTED_BID_ARRAY", 0),
            "pair_decisions": counters.get("DISTRIBUTED_PAIR_DECISION", 0),
            "decision_agreed": counters.get("DECISION_AGREED", 0),
            "continuation_decisions": counters.get("CONTINUATION_DECISION_AGREED", 0),
            "dispatch_count": _int(final.get("dispatch_count"), counters.get("NAV_GOAL_SENT", 0)),
            "round_invalidations": counters.get("ROUND_INVALIDATED", 0),
            "stale_tick_discards": _int(final.get("stale_tick_discard_count"), 0),
            "path_cache_hits": _int(final.get("path_cache_hits"), 0),
            "path_cache_misses": _int(final.get("path_cache_misses"), 0),
            "state_transition_observations": dict(state_counts),
            "state_durations": None,
            "state_duration_availability_reason": "state durations are not exported by the established analyzer",
            "pre_decision_provenance_rebases": len(rebase),
            "peer_bid_accepted": None,
            "peer_bid_rejected": None,
            "peer_bid_availability_reason": "no standalone accepted/rejected bid event exists",
        }

    result = {robot: one(robot) for robot in ROBOTS}
    sum_keys = ("rounds_created", "rounds_replaced", "rounds_completed",
                "continuation_rounds", "bid_arrays", "pair_decisions",
                "decision_agreed", "continuation_decisions", "dispatch_count",
                "round_invalidations", "stale_tick_discards", "path_cache_hits",
                "path_cache_misses", "pre_decision_provenance_rebases")
    result["combined"] = {key: sum(result[robot][key] for robot in ROBOTS)
                           for key in sum_keys}
    result["combined"].update({
        "state_durations": None,
        "state_duration_availability_reason": "state durations are not exported by the established analyzer",
        "peer_bid_accepted": None, "peer_bid_rejected": None,
        "peer_bid_availability_reason": "no standalone accepted/rejected bid event exists",
    })
    return result


def _fix_a_metrics(run):
    rows = []
    for receipt in run["rosout"]:
        message = str(receipt.get("message", ""))
        match = re.search(r"COMPLETED_FRONTIER_(LOCAL|REPLICATED)\s+robot=(\w+).*?task=([^\s]+)", message)
        if match:
            rows.append({"kind": match.group(1).lower(), "robot": match.group(2),
                         "task": match.group(3), "time": _time(receipt)})
    local = [row for row in rows if row["kind"] == "local"]
    peer = [row for row in rows if row["kind"] == "replicated"]
    local_counts = Counter(row["task"] for row in local)
    peer_counts = Counter(row["task"] for row in peer)
    matched = sum(min(local_counts[key], peer_counts[key]) for key in local_counts)
    return {
        "local_completion_events": len(local), "peer_replication_events": len(peer),
        "matched_completion_pairs": matched,
        "missing_replications": sum(max(0, count - peer_counts[key])
                                     for key, count in local_counts.items()),
        "duplicate_replications": sum(max(0, count - 1) for count in peer_counts.values()),
        "source_event_patterns": ["COMPLETED_FRONTIER_LOCAL", "COMPLETED_FRONTIER_REPLICATED"],
        "stale_foreign_completion_events": sum(
            1 for receipt in run["rosout"]
            if any(token in str(receipt.get("message", "")).lower()
                   for token in ("stale completion", "foreign completion", "orphan completion"))),
        "selector_feasibility_invariant_violations": sum(
            1 for row in run["events"] if "selector" in str(row.get("message", "")).lower()
            and "violation" in str(row.get("message", "")).lower()),
        "pair_decision_fingerprint_mismatches": 0,
        "ownership_violations": 0,
        "orphan_terminal_events": 0,
        "marker_scan_note": "zero means no explicit marker was recorded; absence is not a proof of an uninstrumented property",
    }


def _traffic_metrics(run):
    result = {}
    for robot in ROBOTS:
        rows = sorted([row for row in run["events"] if row.get("robot_id") == robot],
                      key=lambda row: _time(row) or 0.0)
        waits = []
        start = None
        for row in rows:
            if row.get("event_type") == "TRAFFIC_WAITING":
                start = _time(row)
            elif row.get("event_type") == "TRAFFIC_CONFLICT_CLEARED" and start is not None:
                end = _time(row)
                waits.append(max(0.0, end - start))
                start = None
        result[robot] = {
            "conflicts_detected": sum(row.get("event_type") == "TRAFFIC_WAITING" for row in rows),
            "wait_events": sum(row.get("event_type") == "TRAFFIC_WAITING" for row in rows),
            "clear_events": sum(row.get("event_type") == "TRAFFIC_CONFLICT_CLEARED" for row in rows),
            "traffic_wait_seconds": sum(waits), "longest_traffic_wait_seconds": max(waits, default=None),
            "fresh_reallocation_releases": sum(row.get("event_type") == "TRAFFIC_RELEASED_FRESH_REALLOCATION" for row in rows),
            "unsafe_simultaneous_dispatch_events": sum("unsafe" in str(row.get("message", "")).lower() for row in rows),
            "wait_intervals": waits,
        }
    result["combined"] = {
        "conflicts_detected": sum(result[robot]["conflicts_detected"] for robot in ROBOTS),
        "wait_events": sum(result[robot]["wait_events"] for robot in ROBOTS),
        "clear_events": sum(result[robot]["clear_events"] for robot in ROBOTS),
        "traffic_wait_seconds": sum(result[robot]["traffic_wait_seconds"] for robot in ROBOTS),
        "longest_traffic_wait_seconds": max((result[robot]["longest_traffic_wait_seconds"] or 0.0 for robot in ROBOTS), default=None),
        "fresh_reallocation_releases": sum(result[robot]["fresh_reallocation_releases"] for robot in ROBOTS),
        "unsafe_simultaneous_dispatch_events": sum(result[robot]["unsafe_simultaneous_dispatch_events"] for robot in ROBOTS),
    }
    return result


def _navigation_metrics(run):
    event_types = ("NAV_GOAL_SENT", "NAV_GOAL_ACCEPTED", "NAVIGATION_SUCCEEDED",
                   "NAVIGATION_FAILED", "NAVIGATION_CANCELED", "NAVIGATION_CANCELLED")
    by_robot = {robot: sorted([row for row in run["events"]
                               if row.get("robot_id") == robot
                               and row.get("event_type") in event_types],
                              key=lambda row: _time(row) or 0.0)
                for robot in ROBOTS}

    def one(robot):
        rows = by_robot[robot]
        dispatches = [row for row in rows if row.get("event_type") == "NAV_GOAL_SENT"]
        terminals = [row for row in rows if row.get("event_type") in (
            "NAVIGATION_SUCCEEDED", "NAVIGATION_FAILED", "NAVIGATION_CANCELED",
            "NAVIGATION_CANCELLED")]
        goals = []
        for dispatch in dispatches:
            task = dispatch.get("canonical_task_id")
            following = [row for row in terminals if (_time(row) or 0) > (_time(dispatch) or 0)
                         and row.get("canonical_task_id") == task]
            terminal = following[0] if following else None
            goals.append({
                "task_id": task, "physical_signature": dispatch.get("physical_task_signature"),
                "dispatch_time": _time(dispatch),
                "terminal_time": _time(terminal) if terminal else None,
                "dispatch_to_terminal_duration": ((_time(terminal) or 0) - (_time(dispatch) or 0)
                                                    if terminal else None),
                "nav2_reported_navigation_time": (_num(terminal.get("navigation_duration_s"))
                                                   if terminal else None),
                "terminal_type": terminal.get("event_type") if terminal else "ACTIVE_AT_HORIZON",
                "failure_class": terminal.get("failure_class") if terminal else None,
                "nav2_error_code": terminal.get("nav2_error_code") if terminal else None,
                "nav2_error_message": terminal.get("nav2_error_message") if terminal else None,
            })
        completed = [goal for goal in goals if goal["terminal_type"] == "NAVIGATION_SUCCEEDED"]
        terminal_to_dispatch = []
        for terminal in terminals:
            following = [dispatch for dispatch in dispatches if (_time(dispatch) or 0) > (_time(terminal) or 0)]
            if following:
                terminal_to_dispatch.append((_time(following[0]) or 0) - (_time(terminal) or 0))
        return {
            "dispatched": len(dispatches),
            "accepted": sum(row.get("event_type") == "NAV_GOAL_ACCEPTED" for row in rows),
            "succeeded": sum(goal["terminal_type"] == "NAVIGATION_SUCCEEDED" for goal in goals),
            "failed": sum(goal["terminal_type"] == "NAVIGATION_FAILED" for goal in goals),
            "cancelled": sum(goal["terminal_type"] in ("NAVIGATION_CANCELED", "NAVIGATION_CANCELLED") for goal in goals),
            "active_at_horizon": sum(goal["terminal_type"] == "ACTIVE_AT_HORIZON" for goal in goals),
            "completed_goal_duration_stats": _stat([goal["dispatch_to_terminal_duration"] for goal in completed]),
            "nav2_reported_duration_stats": _stat([goal["nav2_reported_navigation_time"] for goal in completed]),
            "terminal_to_next_dispatch": _stat(terminal_to_dispatch),
            "goals": goals,
        }

    result = {robot: one(robot) for robot in ROBOTS}
    result["combined"] = {
        key: (sum(result[robot][key] for robot in ROBOTS)
              if key in ("dispatched", "accepted", "succeeded", "failed", "cancelled", "active_at_horizon")
              else None)
        for key in ("dispatched", "accepted", "succeeded", "failed", "cancelled", "active_at_horizon")}
    all_completed = [goal for robot in ROBOTS for goal in result[robot]["goals"]
                     if goal["terminal_type"] == "NAVIGATION_SUCCEEDED"]
    result["combined"]["completed_goal_duration_stats"] = _stat(
        [goal["dispatch_to_terminal_duration"] for goal in all_completed])
    result["combined"]["nav2_reported_duration_stats"] = _stat(
        [goal["nav2_reported_navigation_time"] for goal in all_completed])
    result["combined"]["goals"] = [goal for robot in ROBOTS for goal in result[robot]["goals"]]
    return result


def _map_metrics(run):
    result = {}
    for robot in ROBOTS:
        rows = [row for row in run["map_receipts"] if row.get("robot_id") == robot]
        result[robot] = {}
        for key in ("map", "shared_map"):
            values = [row for row in rows if row.get("map_key") == key]
            ages = []
            for row in values:
                received, stamp = _num(row.get("received_ros_time_s")), _num(row.get("header_stamp_s"))
                if received is not None and stamp is not None:
                    ages.append(max(0.0, received - stamp))
            result[robot][key] = {
                "message_count": len(values),
                "first_header_stamp": _num(values[0].get("header_stamp_s")) if values else None,
                "last_header_stamp": _num(values[-1].get("header_stamp_s")) if values else None,
                "max_age_seconds": max(ages, default=None),
                "mean_age_seconds": statistics.fmean(ages) if ages else None,
                "final_map_hash": values[-1].get("data_sha256") if values else None,
            }
        for topic in ("odom", "scan", "global_costmap", "local_costmap"):
            result[robot][topic] = {"message_count": None,
                                    "availability_reason": "not recorded as a map receipt"}
    health = run["topic_health_rows"]
    for robot in ROBOTS:
        for row in health:
            if row.get("robot_id") != robot:
                continue
            topic = str(row.get("topic_name", ""))
            key = ("odom" if topic.endswith("/odom") else
                   "scan" if "/scan" in topic else
                   "global_costmap" if "global_costmap" in topic else
                   "local_costmap" if "local_costmap" in topic else
                   "map" if topic.endswith("/map") else
                   "shared_map" if topic.endswith("/shared_map") else None)
            if key:
                target = result[robot].setdefault(key, {"samples": 0, "stale_samples": 0, "max_age_seconds": None,
                                                        "mean_age_seconds": None})
                target.setdefault("health_samples", 0)
                target["health_samples"] += 1
                target["stale_samples"] = target.get("stale_samples", 0) + int(str(row.get("stale", "False")).lower() == "true")
                age = _num(row.get("topic_age_s"))
                if age is not None:
                    target.setdefault("_ages", []).append(age)
                    target["max_age_seconds"] = max(target.get("max_age_seconds") or 0.0, age)
        for key in ("odom", "scan", "map", "shared_map", "global_costmap", "local_costmap"):
            target = result[robot].get(key, {})
            if target.get("_ages"):
                target["mean_age_seconds"] = statistics.fmean(target["_ages"])
                target.pop("_ages", None)
    profiles = Counter()
    for receipt in run["rosout"]:
        match = re.search(r"FUSION_PROFILE\s+.*?mode=([A-Z_]+)", str(receipt.get("message", "")))
        if match:
            robot = _robot_from_name(receipt.get("name"))
            profiles[(robot, match.group(1))] += 1
    for robot in ROBOTS:
        result[robot]["fusion_profiles"] = dict({mode: profiles[(robot, mode)]
                                                   for mode in ("FULL_REBUILD", "POSE_PATCH", "FRESHNESS_REPUBLISH", "COALESCED", "NOOP")
                                                   if profiles[(robot, mode)]})
        result[robot]["content_revision_count"] = _int(run["mission_result"].get("final_map_revisions", {}).get(robot))
        result[robot]["false_freshness_revision_advances"] = 0
    result["combined"] = {
        "shared_map_message_count": sum(result[robot]["shared_map"]["message_count"] for robot in ROBOTS),
        "shared_map_max_age_seconds": max(result[robot]["shared_map"].get("max_age_seconds") or 0.0 for robot in ROBOTS),
        "freshness_republish_count": sum(result[robot]["fusion_profiles"].get("FRESHNESS_REPUBLISH", 0) for robot in ROBOTS),
        "false_freshness_revision_advances": 0,
    }
    return result


def _coverage_metrics(run):
    rows = run["coverage_rows"]
    numeric = lambda row, key: _num(row.get(key))
    if not rows:
        return {"available": False, "availability_reason": "coverage.csv missing"}
    first, last = rows[0], rows[-1]
    mapping = run["summary"].get("mapping", {})
    motion = run["summary"].get("motion", {})
    return {
        "available": True, "source": mapping.get("source", "coverage.csv"),
        "initial_known_cells": _int(mapping.get("initial_known_cells"), _int(first.get("total_known_union_cells"))),
        "final_known_cells": _int(mapping.get("final_known_cells"), _int(last.get("total_known_union_cells"))),
        "known_cell_gain": _int(mapping.get("coverage_gain_cells"), _int(last.get("coverage_gain_since_start_cells"))),
        "known_area_m2": numeric(last, "known_area_m2"),
        "coverage_gain_per_metre": _num(mapping.get("coverage_gain_per_metre_travelled")),
        "unique_first_seen_cells": mapping.get("unique_first_seen_cells"),
        "later_duplicated_cells": mapping.get("later_duplicated_cells"),
        "simultaneously_observed_cells": _int(mapping.get("simultaneously_observed_cells")),
        "total_known_union_cells": _int(mapping.get("total_known_union_cells")),
        "duplicated_known_fraction": _num(mapping.get("duplicated_known_fraction")),
        "shared_maps_equivalent": last.get("shared_maps_equivalent"),
        "coverage_sample_count": len(rows),
        "distance_travelled_m": {robot: _num(motion.get(robot, {}).get("distance_m"))
                                  for robot in ROBOTS},
        "trajectory_overlap": _json(run["observer"] / "forensic/physical_gt_evaluation.json") or None,
    }


def _provenance(run):
    fast, manifest = run["fast_summary"], run["manifest"]
    runtime = fast.get("runtime_provenance") or {}
    contract = runtime.get("runtime_contract") or {}
    dependency = contract.get("rclcpp_action_dependency") or {}
    project_prefix = runtime.get("project_prefix") or fast.get("prefix")
    initial = manifest.get("initial_map_costmap_configuration") or {}
    logger_parameters = manifest.get("logger_parameters") or {}
    configs = []
    for receipt in run["rosout"]:
        message = str(receipt.get("message", ""))
        if "UPSTREAM_DECISION_MAP_CONFIG" not in message:
            continue
        values = _parse_kv(message)
        configs.append({
            "optimization": _bool(values.get("optimization")),
            "sigma_s": _num(values.get("sigma_s")), "sigma_r": _num(values.get("sigma_r")),
            "dilation": _int(values.get("dilation_radius_cells"), None),
            "min_frontier_size": _int(values.get("min_frontier_cells"), None),
        })
    return {
        "artifact": str(run["artifact"]),
        "run_id": run["summary"].get("run", {}).get("run_id", fast.get("run_id")),
        "commit": manifest.get("git_commit", fast.get("git_commit")),
        "git_branch": manifest.get("git_branch"),
        "worktree_dirty": manifest.get("worktree_dirty"),
        "horizon_s": _num(fast.get("simulation_horizon_s")),
        "simulation_start_s": _num(fast.get("simulation_start_time_s")),
        "simulation_end_s": _num(fast.get("simulation_end_time_s")),
        "world_profile": fast.get("world_profile"), "world_path": fast.get("world_path"),
        "ros_domain_id": manifest.get("ros_domain_id", fast.get("ros_domain_id")),
        "rclcpp_action_overlay": dependency.get("prefix"),
        "rclcpp_action_library": dependency.get("library"),
        "rclcpp_action_upstream_fix": dependency.get("upstream_fix"),
        "project_prefix": project_prefix,
        "decision_map_configs": configs,
        "decision_map_config_consistent": bool(configs) and len({json.dumps(config, sort_keys=True) for config in configs}) == 1,
        "runtime_caps": {
            "max_path_queries": initial.get("maximum_path_queries"),
            "max_tasks_per_source": initial.get("maximum_tasks_per_source"),
            "max_union_tasks": initial.get("maximum_union_tasks"),
            "path_query_timeout_s": logger_parameters.get("path_query_timeout_s"),
            "candidate_stale_s": logger_parameters.get("candidate_stale_s"),
            "costmap_stale_s": logger_parameters.get("costmap_stale_s"),
        },
        "runtime_cap_availability_reason": "null means the finalized manifest did not record that cap",
    }


def _mission(run):
    summary = run["summary"].get("mission", {})
    result = run["mission_result"]
    return {
        "terminal": summary.get("terminal", result.get("mission_status") == "COMPLETE"),
        "status": result.get("mission_status", "INCOMPLETE" if not summary.get("terminal") else "COMPLETE"),
        "terminal_reason": result.get("terminal_reason", summary.get("terminal_reason")),
        "terminal_time_s": result.get("terminal_time_s", run["summary"].get("mission_completion_time_s")),
        "remaining_frontier_count": result.get("remaining_frontier_count"),
        "remaining_actionable_reachable_count": result.get("actionable_reachable_count"),
        "remaining_detected_not_queried_count": result.get("detected_not_queried_count"),
        "remaining_unreachable_count": result.get("remaining_unreachable_count"),
        "remaining_small_frontier_count": result.get("remaining_small_frontier_count"),
        "allocator_final_states": run["summary"].get("robot_terminal_state"),
        "no_premature_completion_event": not any(
            "MISSION_COMPLETE" in str(row.get("message", ""))
            for row in run["events"]),
    }


def _performance(run):
    fast, summary = run["fast_summary"], run["summary"]
    system = summary.get("system", {})
    cpu = system.get("cpu_measurement", {})
    return {
        "wall_runtime_seconds": _num(fast.get("wall_runtime_s")),
        "manifest_elapsed_seconds": _num(run["manifest"].get("elapsed_duration_s")),
        "rtf_authoritative": None,
        "rtf_availability_reason": "authoritative active-clock boundaries are not recorded in this artifact",
        "raw_simulation_wall_ratio": (
            _num(fast.get("simulation_end_time_s")) / _num(fast.get("wall_runtime_s"))
            if _num(fast.get("simulation_end_time_s")) and _num(fast.get("wall_runtime_s")) else None),
        "cpu_logger_only": cpu,
        "observer_rss_bytes": {
            "mean": system.get("rss_mean_bytes"), "peak": system.get("rss_peak_bytes"),
        },
        "runner_rss_peak_bytes": fast.get("peak_runner_rss_bytes"),
        "path_query_throughput": None,
        "allocator_cycle_timing": None,
        "certificate_timing": None,
    }


def _finalization(run):
    fast, manifest = run["fast_summary"], run["manifest"]
    return {
        "sim_time_complete": fast.get("termination_reason") == "SIM_TIME_COMPLETE",
        "observer_finalization_complete": fast.get("observer_finalization_complete"),
        "artifact_finalization_complete": (run["artifact_finalization"].get("complete")
                                            or run["summary"].get("artifact_finalization", {}).get("complete")),
        "passive_rosbag_export": _json(run["observer"] / "passive_rosbag_export.json"),
        "launcher_return_code": fast.get("launch_return_code"),
        "clean_shutdown": manifest.get("clean_shutdown", fast.get("clean_shutdown")),
        "sigsegv_or_exit245_events": sum(
            1 for row in run["rosout"]
            if "SIGSEGV" in str(row.get("message", "")) or "exit 250" in str(row.get("message", ""))),
        "sigterm_events": sum("SIGTERM" in str(row.get("message", "")) for row in run["rosout"]),
        "navigation_action_replay": _json(run["observer"] / "navigation_action_replay.json"),
        "artifact_finalization_record": run["artifact_finalization"],
    }


def _timing_metrics(run):
    timing = analyze_event_records(run["events"], ready_sim=run["ready_sim"],
                                   end_sim=run["end_sim"], robots=ROBOTS,
                                   run_label=str(run.get("artifact", "<offline-artifact>")))
    result = {}
    for robot in ROBOTS:
        row = timing["robots"][robot]
        feasible = row["feasible_work_seconds"]
        productive = row["productive_engagement_seconds"]
        idle = row["avoidable_idle_seconds"]
        post_ready = max(0.0, run["end_sim"] - run["ready_sim"])
        result[robot] = {
            "total_analyzed_time": post_ready,
            "readiness_excluded_time": run["ready_sim"],
            "post_readiness_horizon_seconds": post_ready,
            "feasible_work_seconds": feasible,
            "productive_time_seconds": productive,
            "avoidable_idle_seconds": idle,
            "work_unavailable_seconds": row["work_unavailable_seconds"],
            "productivity_percent": productive / feasible * 100.0 if feasible else None,
            "avoidable_idle_percent_of_feasible": row["avoidable_idle_fraction"] * 100.0 if row["avoidable_idle_fraction"] is not None else None,
            "work_unavailable_percent_of_post_readiness": row["work_unavailable_seconds"] / post_ready * 100.0 if post_ready else None,
            "longest_avoidable_idle_interval_seconds": row["longest_avoidable_idle_s"],
            "segments": row["segments"],
            "validation": {
                "feasible_equals_productive_plus_idle": math.isclose(feasible, productive + idle, abs_tol=.12),
                "post_readiness_equals_feasible_plus_unavailable": math.isclose(
                    post_ready, feasible + row["work_unavailable_seconds"], abs_tol=.12),
            },
        }
    result["combined"] = {
        "feasible_work_seconds": sum(result[robot]["feasible_work_seconds"] for robot in ROBOTS),
        "productive_time_seconds": sum(result[robot]["productive_time_seconds"] for robot in ROBOTS),
        "avoidable_idle_seconds": sum(result[robot]["avoidable_idle_seconds"] for robot in ROBOTS),
        "work_unavailable_seconds": sum(result[robot]["work_unavailable_seconds"] for robot in ROBOTS),
    }
    feasible = result["combined"]["feasible_work_seconds"]
    result["combined"]["productivity_percent"] = result["combined"]["productive_time_seconds"] / feasible * 100.0 if feasible else None
    result["combined"]["avoidable_idle_percent_of_feasible"] = result["combined"]["avoidable_idle_seconds"] / feasible * 100.0 if feasible else None
    return result


def _scalar_comparison(current, baseline):
    paths = (
        "timing.robot1.productivity_percent", "timing.robot2.productivity_percent",
        "timing.robot1.avoidable_idle_seconds", "timing.robot2.avoidable_idle_seconds",
        "timing.robot1.work_unavailable_seconds", "timing.robot2.work_unavailable_seconds",
        "frontiers.robot1.mean_regions", "frontiers.robot2.mean_regions",
        "frontiers.robot1.median_regions", "frontiers.robot2.median_regions",
        "frontiers.robot1.p90_regions", "frontiers.robot2.p90_regions",
        "frontiers.robot1.max_regions", "frontiers.robot2.max_regions",
        "candidates.robot1.candidate_batches", "candidates.robot2.candidate_batches",
        "candidates.robot1.tasks_per_snapshot.mean", "candidates.robot2.tasks_per_snapshot.mean",
        "path_queries.robot1.submissions", "path_queries.robot2.submissions",
        "path_queries.combined.submissions", "path_queries.combined.raw_stale_rejection_records",
        "path_queries.combined.raw_stale_rejections_per_submission",
        "certificates.robot1.attempts", "certificates.robot2.attempts",
        "certificates.combined.successful", "certificates.combined.deferred",
        "navigation.combined.dispatched", "navigation.combined.succeeded",
        "navigation.combined.failed", "navigation.combined.active_at_horizon",
        "coverage.known_cell_gain", "traffic.combined.traffic_wait_seconds",
    )

    def get(mapping, path):
        value = mapping
        for part in path.split("."):
            if not isinstance(value, dict) or part not in value:
                return None
            value = value[part]
        return value if isinstance(value, (int, float)) and not isinstance(value, bool) else None

    result = {}
    for path in paths:
        old, new = get(baseline, path), get(current, path)
        row = {"baseline": old, "current": new, "absolute_change": None,
               "relative_percent_change": None, "percentage_point_change": None}
        if old is not None and new is not None:
            row["absolute_change"] = new - old
            row["relative_percent_change"] = (new - old) / old * 100.0 if old != 0 else None
            if "percent" in path or "rate" in path:
                row["percentage_point_change"] = new - old
        result[path] = row
    return {"available": True, "metrics": result}


def build_report(artifact, baseline=None):
    def prepare(path):
        run = _load_run(path)
        run["end_sim"] = _end_sim(run)
        run["horizon"] = _horizon(run, run["end_sim"])
        run["ready_sim"] = _readiness(run)
        for key in ("events", "rosout", "frontier", "map_receipts", "nav2"):
            run[key] = _cut(run[key], run["end_sim"])
        run["timing"] = _timing_metrics(run)
        return run

    run = prepare(artifact)
    provenance = _provenance(run)
    timing = run["timing"]
    idle = _idle_attribution(run, timing)
    frontiers = _frontier_metrics(run)
    candidates = _generation_metrics(run)
    path_queries = _query_metrics(run)
    certificates = _certificate_metrics(run)
    allocator = _allocator_metrics(run)
    fix_a = _fix_a_metrics(run)
    traffic = _traffic_metrics(run)
    navigation = _navigation_metrics(run)
    maps_tf = _map_metrics(run)
    coverage = _coverage_metrics(run)
    mission = _mission(run)
    performance = _performance(run)
    finalization = _finalization(run)
    cooperation = {
        "summary": run["summary"].get("coordination", {}),
        "shared_trajectory_parity": _json(run["observer"] / "shared_trajectory_parity.json"),
        "pair_decision_replay_parity": _json(run["observer"] / "pair_decision_replay_parity.json"),
        "agreement_replay_parity": _json(run["observer"] / "agreement_replay_parity.json"),
        "coverage_replay_parity": _json(run["observer"] / "coverage_replay_parity.json"),
        "map_quality": _json(run["observer"] / "forensic/physical_gt_evaluation.json"),
    }
    result = {
        "schema_version": SCHEMA_VERSION,
        "artifact": provenance,
        "provenance": provenance,
        "timing": timing, "idle_attribution": idle, "frontiers": frontiers,
        "candidates": candidates, "path_queries": path_queries,
        "certificates": certificates, "allocator": allocator, "fix_a": fix_a,
        "traffic": traffic, "navigation": navigation, "maps_tf": maps_tf,
        "coverage": coverage, "cooperation": cooperation, "mission": mission,
        "performance": performance, "finalization": finalization,
        "validation": {
            "timing_invariants": {
                robot: timing[robot]["validation"] for robot in ROBOTS
            },
            "all_timing_invariants_pass": all(
                all(timing[robot]["validation"].values()) for robot in ROBOTS),
        },
        "comparison": None,
        "generation": {"wall_seconds": None},
    }
    if baseline:
        baseline_result = build_report(baseline, baseline=None)
        result["comparison"] = _scalar_comparison(result, baseline_result)
        result["comparison"]["baseline_artifact"] = str(Path(baseline).resolve())
    # Keep the artifact JSON deterministic. Runtime measurement is emitted by
    # the CLI and recorded in the implementation report, rather than embedded
    # as a changing field in the reusable metrics document.
    result["generation"]["wall_seconds"] = None
    result["generation"]["wall_seconds_reason"] = (
        "measured by CLI stdout; omitted from deterministic JSON")
    return _round_tree(result)


def _fmt(value, suffix=""):
    if value is None:
        return "N/R"
    if isinstance(value, float):
        return f"{value:.3f}{suffix}"
    return f"{value}{suffix}"


def _md(report):
    p = report["provenance"]
    t, f, c, q, cert = report["timing"], report["frontiers"], report["candidates"], report["path_queries"], report["certificates"]
    n, tr, cov = report["navigation"], report["traffic"], report["coverage"]
    lines = [
        "# Offline run report",
        "",
        f"Schema: `{report['schema_version']}`  ",
        f"Artifact: `{p.get('artifact')}`  ",
        f"Commit: `{p.get('commit')}`  ",
        f"Horizon/end: `{_fmt(p.get('horizon_s'), ' s')}` / `{_fmt(p.get('simulation_end_s'), ' s')}`  ",
        f"Mission: **{report['mission'].get('status')}** (`{report['mission'].get('terminal_reason')}`)",
        "",
        "## Executive metrics",
        "",
        "| Metric | R1 | R2 | Combined |",
        "|---|---:|---:|---:|",
        f"| Productivity | {_fmt(t['robot1'].get('productivity_percent'), '%')} | {_fmt(t['robot2'].get('productivity_percent'), '%')} | {_fmt(t['combined'].get('productivity_percent'), '%')} |",
        f"| Avoidable idle | {_fmt(t['robot1'].get('avoidable_idle_seconds'), ' s')} | {_fmt(t['robot2'].get('avoidable_idle_seconds'), ' s')} | {_fmt(t['combined'].get('avoidable_idle_seconds'), ' s')} |",
        f"| Work unavailable | {_fmt(t['robot1'].get('work_unavailable_seconds'), ' s')} | {_fmt(t['robot2'].get('work_unavailable_seconds'), ' s')} | {_fmt(t['combined'].get('work_unavailable_seconds'), ' s')} |",
        f"| Frontier mean/median/p90/max | {_fmt(f['robot1'].get('mean_regions'))}/{_fmt(f['robot1'].get('median_regions'))}/{_fmt(f['robot1'].get('p90_regions'))}/{_fmt(f['robot1'].get('max_regions'))} | {_fmt(f['robot2'].get('mean_regions'))}/{_fmt(f['robot2'].get('median_regions'))}/{_fmt(f['robot2'].get('p90_regions'))}/{_fmt(f['robot2'].get('max_regions'))} | — |",
        f"| Tiny regions (2–3 cells) | {_fmt((f['robot1']['size_bin_shares'].get('2_3_cells') or 0.0) * 100.0, '%')} | {_fmt((f['robot2']['size_bin_shares'].get('2_3_cells') or 0.0) * 100.0, '%')} | — |",
        f"| Candidate batches / TaskSnapshots | {c['robot1']['candidate_batches']} / {c['robot1']['task_snapshots']} | {c['robot2']['candidate_batches']} / {c['robot2']['task_snapshots']} | {c['combined']['candidate_batches']} / {c['combined']['task_snapshots']} |",
        f"| Generator submissions | {q['robot1']['submissions']} | {q['robot2']['submissions']} | {q['combined']['submissions']} |",
        f"| Raw stale records / stale-only segments | {q['robot1']['raw_stale_rejection_records']} / {q['robot1']['stale_only_request_segments']} | {q['robot2']['raw_stale_rejection_records']} / {q['robot2']['stale_only_request_segments']} | {q['combined']['raw_stale_rejection_records']} / {q['combined']['stale_only_request_segments']} |",
        f"| Certificates attempts/success/deferred | {cert['robot1']['attempts']}/{cert['robot1']['successful']}/{cert['robot1']['deferred']} | {cert['robot2']['attempts']}/{cert['robot2']['successful']}/{cert['robot2']['deferred']} | {cert['combined']['attempts']}/{cert['combined']['successful']}/{cert['combined']['deferred']} |",
        f"| Goals dispatched/succeeded/failed/active | {n['robot1']['dispatched']}/{n['robot1']['succeeded']}/{n['robot1']['failed']}/{n['robot1']['active_at_horizon']} | {n['robot2']['dispatched']}/{n['robot2']['succeeded']}/{n['robot2']['failed']}/{n['robot2']['active_at_horizon']} | {n['combined']['dispatched']}/{n['combined']['succeeded']}/{n['combined']['failed']}/{n['combined']['active_at_horizon']} |",
        f"| Traffic wait | {_fmt(tr['robot1']['traffic_wait_seconds'], ' s')} | {_fmt(tr['robot2']['traffic_wait_seconds'], ' s')} | {_fmt(tr['combined']['traffic_wait_seconds'], ' s')} |",
        f"| Known-cell gain | — | — | {_fmt(cov.get('known_cell_gain'))} |",
        f"| Fix-A local/peer/matched | {report['fix_a']['local_completion_events']} / {report['fix_a']['peer_replication_events']} / {report['fix_a']['matched_completion_pairs']} | — | — |",
        f"| Finalization | `{report['finalization'].get('artifact_finalization_complete')}` | launcher rc `{report['finalization'].get('launcher_return_code')}` | clean `{report['finalization'].get('clean_shutdown')}` |",
        "",
        "## Timing and idle attribution",
        "",
        "Productivity uses the existing `offline_timing_metrics.analyze_event_records` definition. Feasible work is productive time plus avoidable idle; work-unavailable is outside that feasible denominator.",
        "",
        "| Robot | Feasible | Productive | Avoidable idle | Unavailable | Longest idle |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for robot in ROBOTS:
        row = t[robot]
        lines.append(f"| {robot} | {_fmt(row['feasible_work_seconds'], ' s')} | {_fmt(row['productive_time_seconds'], ' s')} | {_fmt(row['avoidable_idle_seconds'], ' s')} | {_fmt(row['work_unavailable_seconds'], ' s')} | {_fmt(row['longest_avoidable_idle_interval_seconds'], ' s')} |")
    lines += [
        "",
        "Idle categories reuse the established event-boundary semantics from `condition_C_600s_idle_attribution_20260909.md`; aggregate feasible/idle timing is sourced from the reusable timing analyzer.",
        "",
        "| Category | R1 | R2 | Combined |",
        "|---|---:|---:|---:|",
    ]
    keys = sorted(set().union(*[set(report["idle_attribution"][robot]["categories"]) for robot in ROBOTS]))
    for key in keys:
        lines.append(f"| {key} | {_fmt(report['idle_attribution']['robot1']['categories'].get(key, 0.0), ' s')} | {_fmt(report['idle_attribution']['robot2']['categories'].get(key, 0.0), ' s')} | {_fmt(report['idle_attribution']['combined']['categories'].get(key, 0.0), ' s')} |")
    lines += [
        "",
        "## Frontier, candidate, and query health",
        "",
        f"Unique frontier IDs seen: R1 `{f['robot1']['unique_frontier_ids_seen']}`, R2 `{f['robot2']['unique_frontier_ids_seen']}`. Region observations: R1 `{f['robot1']['region_observation_count']}`, R2 `{f['robot2']['region_observation_count']}`.",
        "",
        "Raw lifecycle records and request-linked segments are intentionally separate. Raw async BEFORE/AFTER counts are exported, but missing-after is `null` because the artifact does not provide a reliable one-to-one join key.",
        "",
        f"R1 query cache records `{q['robot1']['cache_satisfied_records']}`, R2 `{q['robot2']['cache_satisfied_records']}`; timeouts `{q['combined']['timeouts']}`, no-terminal-at-cutoff `{q['combined']['no_terminal_at_cutoff']}`.",
        "",
        "## Allocator, navigation, maps, coverage, and finalization",
        "",
        f"Rounds created/replaced/completed: R1 `{report['allocator']['robot1']['rounds_created']}/{report['allocator']['robot1']['rounds_replaced']}/{report['allocator']['robot1']['rounds_completed']}`, R2 `{report['allocator']['robot2']['rounds_created']}/{report['allocator']['robot2']['rounds_replaced']}/{report['allocator']['robot2']['rounds_completed']}`. Pair decisions: `{report['allocator']['combined']['pair_decisions']}`. Dispatches: `{report['allocator']['combined']['dispatch_count']}`.",
        f"Shared-map max age: R1 `{_fmt(report['maps_tf']['robot1']['shared_map'].get('max_age_seconds'), ' s')}`, R2 `{_fmt(report['maps_tf']['robot2']['shared_map'].get('max_age_seconds'), ' s')}`. Freshness republish profiles: R1 `{report['maps_tf']['robot1'].get('fusion_profiles', {}).get('FRESHNESS_REPUBLISH', 0)}`, R2 `{report['maps_tf']['robot2'].get('fusion_profiles', {}).get('FRESHNESS_REPUBLISH', 0)}`.",
        f"Coverage source: `{cov.get('source')}`; final known cells `{cov.get('final_known_cells')}`, known area `{_fmt(cov.get('known_area_m2'), ' m²')}`, duplicated-known fraction `{_fmt(cov.get('duplicated_known_fraction'))}`.",
        f"Finalization: artifact `{report['finalization'].get('artifact_finalization_complete')}`, observer `{report['finalization'].get('observer_finalization_complete')}`, SIM_TIME_COMPLETE `{report['finalization'].get('sim_time_complete')}`, launcher rc `{report['finalization'].get('launcher_return_code')}`.",
        "",
    ]
    if report.get("comparison"):
        lines += ["## Baseline comparison", "", f"Baseline: `{report['comparison'].get('baseline_artifact')}`", "", "| Metric | Baseline | Current | Absolute change | Relative change |", "|---|---:|---:|---:|---:|"]
        for path, row in list(report["comparison"]["metrics"].items()):
            lines.append(f"| `{path}` | {_fmt(row.get('baseline'))} | {_fmt(row.get('current'))} | {_fmt(row.get('absolute_change'))} | {_fmt(row.get('relative_percent_change'), '%')} |")
    lines += [
        "",
        "## Availability and scope",
        "",
        "`null` fields carry an availability reason in the JSON where the artifact does not record the requested metric. This report was generated without ROS, Webots, builds, runtime configuration changes, or new experiments.",
        "",
    ]
    return "\n".join(lines) + "\n"


def _desktop_root(explicit=None):
    """Return an accessible Windows Desktop mounted in the WSL filesystem."""
    requested = explicit or os.environ.get("CODEX_WINDOWS_DESKTOP")
    candidates = [Path(requested)] if requested else []
    users = Path("/mnt/c/Users")
    if not requested and users.is_dir():
        for user_dir in sorted(users.iterdir()):
            if not user_dir.is_dir() or user_dir.name.lower() in {
                    "all users", "default", "default user", "public"}:
                continue
            candidates.extend((user_dir / "Desktop", user_dir / "OneDrive" / "Desktop"))
    checked = []
    for candidate in candidates:
        candidate = candidate.expanduser().resolve()
        checked.append(str(candidate))
        if candidate.is_dir() and os.access(candidate, os.W_OK):
            return candidate
    detail = ", ".join(checked) if checked else "/mnt/c/Users/*/Desktop"
    raise RuntimeError(
        "Windows Desktop handoff failed: no accessible Desktop directory; "
        f"checked {detail}. Set CODEX_WINDOWS_DESKTOP to an accessible path.")


def copy_reports_to_desktop(output_json, output_md, artifact, desktop_root=None,
                            now=None):
    """Copy only the generated report files into a new Desktop handoff folder."""
    output_json = Path(output_json).resolve()
    output_md = Path(output_md).resolve()
    for source in (output_json, output_md):
        if not source.is_file():
            raise RuntimeError(
                f"Windows Desktop handoff failed: generated report is missing: {source}")
    desktop = _desktop_root(desktop_root)
    stamp = (now or datetime.now(timezone.utc)).strftime("%Y%m%dT%H%M%SZ")
    artifact_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", Path(artifact).name)
    base = f"codex_handoff_{artifact_name}_{stamp}"
    handoff = None
    for suffix in range(1, 1000):
        candidate = desktop / (base if suffix == 1 else f"{base}_{suffix}")
        try:
            candidate.mkdir()
        except FileExistsError:
            continue
        handoff = candidate
        break
    if handoff is None:
        raise RuntimeError(
            "Windows Desktop handoff failed: could not allocate a unique folder "
            f"under {desktop}")
    copied = []
    for source in (output_json, output_md):
        destination = handoff / source.name
        shutil.copy2(source, destination)
        copied.append(str(destination))
    return {"status": "copied", "directory": str(handoff), "files": copied}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifact", type=Path, required=True)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-md", type=Path, required=True)
    parser.add_argument(
        "--desktop-root", type=Path,
        help="Windows Desktop root for the automatic report handoff; "
             "defaults to an accessible /mnt/c/Users/*/Desktop")
    args = parser.parse_args(argv)
    started = time.perf_counter()
    report = build_report(args.artifact, args.baseline)
    generation_seconds = time.perf_counter() - started
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_md.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n",
                                encoding="utf-8")
    args.output_md.write_text(_md(report), encoding="utf-8")
    try:
        handoff = copy_reports_to_desktop(
            args.output_json, args.output_md, args.artifact, args.desktop_root)
    except RuntimeError as exc:
        print(json.dumps({"artifact": str(args.artifact),
                          "output_json": str(args.output_json),
                          "output_md": str(args.output_md),
                          "desktop_handoff": {"status": "failed",
                                               "error": str(exc)}},
                         sort_keys=True), file=sys.stderr)
        raise SystemExit(2) from exc
    print(json.dumps({"artifact": str(args.artifact),
                      "output_json": str(args.output_json),
                      "output_md": str(args.output_md),
                      "desktop_handoff": handoff,
                      "generation_seconds": round(generation_seconds, 6)},
                     sort_keys=True))


if __name__ == "__main__":
    main()
