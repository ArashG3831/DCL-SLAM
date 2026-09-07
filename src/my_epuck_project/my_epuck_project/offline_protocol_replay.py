"""Payload-level replay of the thin cooperative raw evidence streams.

The bag is the authoritative raw representation.  This module keeps only
compact parsed records needed by thesis metrics and contract validation; it
does not create a second copy of complete ROS messages.
"""

from __future__ import annotations

import json
from pathlib import Path


def _stamp(message):
    header = getattr(message, "header", None)
    if header is None:
        return None
    stamp = header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _time_value(value):
    if value is None:
        return None
    try:
        return float(value.sec) + float(value.nanosec) * 1.0e-9
    except (AttributeError, TypeError, ValueError):
        return None


def _uuid_text(value):
    try:
        return bytes(value.uuid).hex()
    except (AttributeError, TypeError, ValueError):
        return ""


def _increment(mapping, key, amount=1):
    key = str(key)
    mapping[key] = mapping.get(key, 0) + amount


def _float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _state_name(value):
    return {
        0: "WAITING_FOR_INPUTS", 1: "BIDDING",
        2: "WAITING_FOR_MATCHING_DECISION", 3: "NAVIGATING",
        4: "DEGRADED_SOLO", 5: "COMPLETE", 6: "BLOCKED",
        7: "WAITING_FOR_TRAFFIC",
    }.get(int(value), "UNKNOWN")


def _bound_entry_count(message):
    raw = str(getattr(message, "terminal_frontier_regions_json", "") or
              getattr(message, "diagnostic_regions_json", "") or "")
    if not raw:
        return 0
    try:
        payload = json.loads(raw)
    except (TypeError, ValueError):
        return 0
    if isinstance(payload, dict):
        payload = payload.get("regions", [])
    return len(payload) if isinstance(payload, list) else 0


def _failure_name(value):
    return {
        0: "HARD_UNREACHABLE", 1: "PLANNER_FAILURE",
        2: "CONTROLLER_NO_PROGRESS", 3: "DYNAMIC_BLOCKAGE",
        4: "TF_OR_LIFECYCLE", 5: "ACTION_REJECTION",
        6: "TIMEOUT", 7: "EXPLICIT_CANCELLATION", 8: "UNKNOWN",
    }.get(int(value), f"UNKNOWN_{int(value)}")


def _empty_robot_state():
    return {
        "frontier_batches": 0, "task_snapshots": 0, "bid_batches": 0,
        "pair_decisions": 0, "status_messages": 0, "cooperation_events": 0,
        "failure_messages": 0, "candidate_count": 0, "task_count": 0,
        "bid_count": 0, "valid_bid_count": 0, "states": {},
        "event_types": {}, "failure_classes": {},
    }


def _certificate_status(certificate_records, bid_records, pair_decisions,
                        cooperation_events):
    """Describe certificate observability without synthesizing values.

    A distributed-event topic can be non-empty even when a cost-only
    certificate was never entered (for example, a run that stayed in a
    degraded/local path).  Keep that distinction explicit for the offline
    contract.  Only an actual ``COST_ONLY_DISPATCH_CERTIFICATE`` payload is
    an observed certificate; absence is never converted into a fake record.
    """
    required_fields = (
        "evaluated_candidate_count", "detected_not_queried_count",
        "blocking_unqueried_candidates",
        "current_evaluated_assignment_score",
        "best_optimistic_unqueried_score", "dispatch_certified", "reason")
    if certificate_records:
        missing = sorted({field for field in required_fields
                          if any(field not in record
                                 for record in certificate_records)})
        return {
            "state": "OBSERVED" if not missing else "OBSERVED_INCOMPLETE",
            "payload_complete": not missing,
            "observation_count": len(certificate_records),
            "reason": "certificate payloads replayed",
            "required_fields": list(required_fields),
            "missing_fields": missing,
        }
    event_types = sorted({str(item.get("event_type", ""))
                          for item in cooperation_events})
    non_cooperative = {
        "DEGRADED_SOLO_COMMITMENT", "ROUND_INVALIDATED",
        "INITIAL_LOCAL_EXPLORATION_SKIPPED_DUE_TO_HANDOFF",
    }
    # A local/degraded run can publish a bid array for its own fallback work
    # without ever entering the pair certificate path.  The authoritative
    # event stream proves that distinction; do not turn that valid absence
    # into a missing certificate observation.
    if (bid_records and not pair_decisions and
            set(event_types) & non_cooperative):
        return {
            "state": "NOT_INVOKED",
            "payload_complete": False,
            "observation_count": 0,
            "reason": "authoritative local/degraded events show the pair "
                       "certificate path was not entered",
            "event_types": event_types,
        }
    if bid_records or pair_decisions:
        return {
            "state": "MISSING_OBSERVATIONS",
            "payload_complete": False,
            "observation_count": 0,
            "reason": "bids/decision payloads exist but no certificate payload",
        }
    if cooperation_events and set(event_types) & non_cooperative:
        return {
            "state": "NOT_INVOKED",
            "payload_complete": False,
            "observation_count": 0,
            "reason": "no bid/pair stream and authoritative local/degraded "
                      "events show the pair certificate path was not entered",
            "event_types": event_types,
        }
    return {
        "state": "NO_EVIDENCE",
        "payload_complete": False,
        "observation_count": 0,
        "reason": "no certificate, bid, pair, or explanatory protocol event",
        "event_types": event_types,
    }


def _per_robot_records(records, robots):
    return {
        robot: [item for item in records if item.get("robot") == robot]
        for robot in robots
    }


def protocol_summary(protocol, robots=("robot1", "robot2")):
    """Reduce parsed raw protocol records into thesis-facing summaries.

    The complete payload records remain available in ``protocol``.  This
    function stores only counts, explicit zero states, and compact time-series
    rows needed by the final report.
    """
    robots = tuple(str(robot) for robot in robots)
    events = list(protocol.get("cooperation_events", []))
    event_types = {}
    for event in events:
        name = str(event.get("event_type", ""))
        _increment(event_types, name)

    def event_subset(names):
        return [event for event in events
                if event.get("event_type") in set(names)]

    agreement_events = event_subset(("DECISION_AGREED",))
    continuation_events = [event for event in events
                           if "CONTINUATION" in str(
                               event.get("event_type", "")).upper()]
    claims = [event for event in events
              if "CLAIM" in str(event.get("event_type", "")).upper()]
    failures = list(protocol.get("failure_records", []))
    terminals = list(protocol.get("navigation_terminals", []))
    dispatches = list(protocol.get("dispatches", []))
    dnu_series = []
    for item in protocol.get("generation_records", []):
        if item.get("kind") == "candidate":
            dnu_series.append({
                "robot": item.get("robot"),
                "sim_time_s": item.get("sim_time_s"),
                "source": "candidate",
                "detected_not_queried_count": item.get(
                    "detected_not_queried_count", 0),
                "candidate_generation_id": item.get(
                    "candidate_generation_id", 0),
            })
    for item in protocol.get("status_records", []):
        dnu_series.append({
            "robot": item.get("robot"),
            "sim_time_s": item.get("sim_time_s"),
            "source": "distributed_status",
            "detected_not_queried_count": item.get(
                "detected_not_queried_count", 0),
            "actionable_reachable_count": item.get(
                "actionable_reachable_count", 0),
        })
    dnu_series.sort(key=lambda item: (
        float(item.get("sim_time_s") or 0.0), str(item.get("robot", ""))))

    terminal_by_robot = _per_robot_records(terminals, robots)
    dispatch_by_robot = _per_robot_records(dispatches, robots)
    terminal_summary = {}
    for robot in robots:
        values = terminal_by_robot[robot]
        terminal_summary[robot] = {
            "count": len(values),
            "succeeded": sum(event.get("event_type") ==
                              "NAVIGATION_SUCCEEDED" for event in values),
            "failed": sum(event.get("event_type") ==
                           "NAVIGATION_FAILED" for event in values),
            "cancelled": sum(event.get("event_type") in
                              ("NAVIGATION_CANCELED", "NAVIGATION_CANCELLED")
                              for event in values),
        }

    round_ids = sorted({str(item.get("round_id", "")) for item in events
                        if item.get("round_id")})
    rounds = []
    for round_id in round_ids:
        round_events = [item for item in events
                        if str(item.get("round_id", "")) == round_id]
        rounds.append({
            "round_id": round_id,
            "event_count": len(round_events),
            "event_types": sorted({str(item.get("event_type", ""))
                                    for item in round_events}),
            "bid_count": sum(item.get("round_id") == round_id
                              for item in protocol.get("bid_records", [])),
            "pair_decision_count": sum(item.get("round_id") == round_id
                                        for item in protocol.get(
                                            "pair_decisions", [])),
            "dispatch_count": sum(item.get("round_id") == round_id
                                   for item in dispatches),
        })

    certificate_status = protocol.get("certificate_status", {
        "state": "NO_EVIDENCE", "payload_complete": False,
    })
    failure_by_class = {}
    for robot in robots:
        for key, value in protocol.get("robots", {}).get(
                robot, {}).get("failure_classes", {}).items():
            failure_by_class[key] = failure_by_class.get(key, 0) + int(value)
    return {
        "available": bool(protocol.get("available")),
        "event_types": event_types,
        "candidate_batches": {
            "total": sum(item.get("kind") == "candidate" for item in
                          protocol.get("generation_records", [])),
            "by_robot": {
                robot: sum(item.get("kind") == "candidate" and
                           item.get("robot") == robot for item in
                           protocol.get("generation_records", []))
                for robot in robots},
        },
        "task_snapshots": {
            "total": sum(item.get("kind") == "task_snapshot" for item in
                          protocol.get("generation_records", [])),
        },
        "bids": {
            "batches": sum(int(protocol.get("robots", {}).get(
                robot, {}).get("bid_batches", 0)) for robot in robots),
            "records": len(protocol.get("bid_records", [])),
            "valid_records": sum(bool(item.get("path_valid")) for item in
                                  protocol.get("bid_records", [])),
        },
        "pair_decisions": {
            "records": len(protocol.get("pair_decisions", [])),
            "unique_decision_hashes": len({item.get("decision_hash") for item
                                           in protocol.get("pair_decisions", [])
                                           if item.get("decision_hash")}),
        },
        "agreements": {
            "publications": len(agreement_events),
            "unique_rounds": len({item.get("round_id") for item in
                                  agreement_events if item.get("round_id")}),
            "unique_decisions": len({item.get("decision_hash") for item in
                                     agreement_events if item.get(
                                         "decision_hash")}),
        },
        "continuations": {"records": len(continuation_events)},
        "claims": {"records": len(claims)},
        "assignments": {"dispatches": len(dispatches),
                         "by_robot": {robot: len(dispatch_by_robot[robot])
                                       for robot in robots}},
        "terminals": terminal_summary,
        "failures": {
            "records": len(failures),
            "by_class": failure_by_class,
            "by_robot": {
                robot: protocol.get("robots", {}).get(robot, {}).get(
                    "failure_classes", {}) for robot in robots},
        },
        "certificate": {
            "status": certificate_status,
            "observations": len(protocol.get("certificate_records", [])),
        },
        "dnu": {
            "series": dnu_series,
            "observations": len(dnu_series),
            "by_robot": {
                robot: [item for item in dnu_series
                        if item.get("robot") == robot]
                for robot in robots},
        },
        "rounds": rounds,
        "zero_event_semantics": {
            "pair_decisions": "observed_zero_only_if_topic_was_read",
            "agreements": "observed_zero_only_if_event_stream_was_read",
            "certificates": "NOT_INVOKED or MISSING_EVIDENCE, never implicit zero",
        },
    }


def replay_protocol(run_directory: Path, robots=("robot1", "robot2")) -> dict:
    """Replay cooperative payloads from ``passive_rosbag``."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag = Path(run_directory) / "passive_rosbag"
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"),
    )
    type_map = {item.name: item.type
                for item in reader.get_all_topics_and_types()}
    selected = {
        f"/{robot}/{suffix}"
        for robot in robots
        for suffix in (
            "frontier_candidates", "task_snapshot", "task_bids",
            "pair_decision", "distributed_status", "distributed_event",
            "exploration_failure")
    }
    selected.add("/cslam/unknown_pose/start_release")
    topic_counts = {topic: 0 for topic in sorted(selected)}
    parse_errors = {topic: 0 for topic in sorted(selected)}
    robots_out = {robot: _empty_robot_state() for robot in robots}
    status_records = []
    cooperation_events = []
    bid_records = []
    pair_decisions = []
    dispatches = []
    navigation_terminals = []
    failure_records = []
    generation_records = []
    certificate_records = []
    start_release_records = []

    while reader.has_next():
        topic, data, _bag_timestamp = reader.read_next()
        if topic not in selected:
            continue
        topic_counts[topic] += 1
        robot = topic.split("/")[1] if topic.startswith("/robot") else ""
        try:
            message = deserialize_message(data, get_message(type_map[topic]))
        except Exception as exc:
            parse_errors[topic] += 1
            continue
        time_s = _stamp(message)
        suffix = topic.rsplit("/", 1)[-1]
        if topic == "/cslam/unknown_pose/start_release":
            try:
                payload = json.loads(str(getattr(message, "data", "")))
            except (TypeError, ValueError):
                payload = None
            if isinstance(payload, dict):
                start_release_records.append({
                    "sim_time_s": time_s,
                    "event": str(payload.get("event", "")),
                    "release_sim_time_s": _float(
                        payload.get("release_sim_time_s"), time_s),
                    "accepted_handoff": bool(
                        payload.get("accepted_handoff", False)),
                    "ready_robots": list(payload.get("ready_robots", [])),
                    "shared_nav2_ready": bool(
                        payload.get("shared_nav2_ready", False)),
                })
            else:
                parse_errors[topic] += 1
            continue
        state = robots_out[robot]

        if suffix == "frontier_candidates":
            state["frontier_batches"] += 1
            candidates = list(getattr(message, "candidates", []))
            state["candidate_count"] += len(candidates)
            generation_records.append({
                "robot": robot, "kind": "candidate",
                "sim_time_s": time_s,
                "candidate_count": len(candidates),
                "candidate_generation_id": int(
                    getattr(message, "candidate_generation_id", 0) or 0),
                "map_revision": int(getattr(message, "map_revision", 0)),
                "costmap_revision": int(
                    getattr(message, "costmap_revision", 0)),
                "lower_bound_context_fingerprint": str(
                    getattr(message, "lower_bound_context_fingerprint", "")),
                "detected_not_queried_count": int(getattr(
                    message, "detected_not_queried_count", 0)),
                "bound_entry_count": _bound_entry_count(message),
            })
        elif suffix == "task_snapshot":
            state["task_snapshots"] += 1
            state["task_count"] += len(getattr(message, "tasks", []))
            generation_records.append({
                "robot": robot, "kind": "task_snapshot",
                "sim_time_s": time_s,
                "candidate_generation_id": int(
                    getattr(message, "candidate_generation_id", 0) or 0),
                "map_revision": int(getattr(message, "source_map_revision", 0)),
                "costmap_revision": int(getattr(
                    message, "source_costmap_revision", 0)),
                "task_generation_stamp_s": _time_value(
                    getattr(message, "task_generation_stamp", None)),
                "task_count": len(getattr(message, "tasks", [])),
                "lower_bound_context_fingerprint": str(getattr(
                    message, "lower_bound_context_fingerprint", "")),
                "source_session_id": _uuid_text(
                    getattr(message, "source_session_id", None)),
                "source_snapshot_epoch": int(getattr(
                    message, "source_snapshot_epoch", 0)),
            })
        elif suffix == "task_bids":
            state["bid_batches"] += 1
            bids = list(getattr(message, "bids", []))
            state["bid_count"] += len(bids)
            state["valid_bid_count"] += sum(
                bool(getattr(bid, "path_valid", False)) for bid in bids)
            for bid in bids:
                bid_records.append({
                    "robot": robot, "sim_time_s": time_s,
                    "round_id": str(getattr(message, "round_id", "")),
                    "union_hash": str(getattr(message, "union_hash", "")),
                    "source_snapshot_epoch": int(getattr(
                        message, "source_snapshot_epoch", 0) or 0),
                    "task_id": str(getattr(bid, "canonical_task_id", "")),
                    "path_valid": bool(getattr(bid, "path_valid", False)),
                    "path_length_m": _float(getattr(bid, "path_length_m", 0.0)),
                    "estimated_travel_cost": _float(getattr(
                        bid, "estimated_travel_cost", 0.0)),
                    "heading_cost": _float(getattr(bid, "heading_cost", 0.0)),
                    "utility": _float(getattr(
                        bid, "own_utility_contribution", 0.0)),
                })
        elif suffix == "pair_decision":
            state["pair_decisions"] += 1
            pair_decisions.append({
                "robot": robot, "sim_time_s": time_s,
                "round_id": str(getattr(message, "round_id", "")),
                "union_hash": str(getattr(message, "union_hash", "")),
                "decision_hash": str(getattr(message, "decision_hash", "")),
                "robot1_task_id": str(getattr(
                    message, "robot1_canonical_task_id", "")),
                "robot2_task_id": str(getattr(
                    message, "robot2_canonical_task_id", "")),
                "team_score": _float(getattr(message, "total_team_score", 0.0)),
                "combined_path_cost": _float(getattr(
                    message, "combined_path_cost", 0.0)),
                "nearby_goal_penalty": _float(getattr(
                    message, "nearby_goal_penalty", 0.0)),
                "route_overlap_penalty": _float(getattr(
                    message, "route_overlap_penalty", 0.0)),
                "sensing_overlap_penalty": _float(getattr(
                    message, "sensing_overlap_penalty", 0.0)),
                "workload_imbalance_penalty": _float(getattr(
                    message, "workload_imbalance_penalty", 0.0)),
                "coordinator_state": str(getattr(
                    message, "coordinator_state", "")),
            })
        elif suffix == "distributed_status":
            state["status_messages"] += 1
            state_name = _state_name(getattr(message, "state", -1))
            _increment(state["states"], state_name)
            record = {
                "robot": robot, "sim_time_s": time_s, "state": state_name,
                "round_id": str(getattr(message, "round_id", "")),
                "active_task_id": str(getattr(
                    message, "active_canonical_task_id", "")),
                "actionable_reachable_count": int(getattr(
                    message, "actionable_reachable_count", 0)),
                "detected_not_queried_count": int(getattr(
                    message, "detected_not_queried_count", 0)),
                "feasible_work_available": getattr(
                    message, "feasible_work_available", None),
                "actionable_work_available": getattr(
                    message, "actionable_work_available", None),
                "work_availability_reason": str(getattr(
                    message, "work_availability_reason", "")),
                "nav2_healthy": bool(getattr(message, "nav2_healthy", False)),
                "tf_healthy": bool(getattr(message, "tf_healthy", False)),
                "candidate_source_healthy": getattr(
                    message, "candidate_source_healthy", None),
                "local_nav_goal_active": getattr(
                    message, "local_nav_goal_active", None),
                "reason": str(getattr(message, "reason", "")),
            }
            status_records.append(record)
        elif suffix == "distributed_event":
            state["cooperation_events"] += 1
            event_type = str(getattr(message, "event_type", ""))
            _increment(state["event_types"], event_type)
            event = {
                "robot": robot, "sim_time_s": time_s,
                "event_type": event_type,
                "round_id": str(getattr(message, "round_id", "")),
                "task_id": str(getattr(message, "canonical_task_id", "")),
                "physical_task_signature": str(getattr(
                    message, "physical_task_signature", "")),
                "source_session_id": _uuid_text(
                    getattr(message, "source_session_id", None)),
                "previous_state": str(getattr(message, "previous_state", "")),
                "next_state": str(getattr(message, "next_state", "")),
                "decision_hash": str(getattr(message, "decision_hash", "")),
                "result": str(getattr(message, "result", "")),
                "reason": str(getattr(message, "reason", "")),
                "path_length_m": _float(getattr(message, "path_length_m", 0.0)),
                "navigation_duration_s": _float(getattr(
                    message, "navigation_duration_s", 0.0)),
                "newly_discovered_cells": _float(getattr(
                    message, "newly_discovered_cells", 0.0)),
                "peer_first_discovered_cells": _float(getattr(
                    message, "peer_first_discovered_cells", 0.0)),
                "duplicated_cells": _float(getattr(
                    message, "duplicated_cells", 0.0)),
                "route_overlap_score": _float(getattr(
                    message, "route_overlap_score", 0.0)),
                "sensing_overlap_estimate": _float(getattr(
                    message, "sensing_overlap_estimate", 0.0)),
                "failure_class": _failure_name(getattr(
                    message, "failure_class", 8)),
                "recoveries": int(getattr(message, "recoveries", 0)),
                "nav2_error_code": int(getattr(message, "nav2_error_code", 0)),
                "nav2_error_message": str(getattr(
                    message, "nav2_error_message", "")),
                "nav2_error_name": str(getattr(
                    message, "nav2_error_name", "")),
            }
            cooperation_events.append(event)
            if event_type == "NAV_GOAL_SENT":
                dispatches.append(event)
            if event_type in ("NAVIGATION_SUCCEEDED", "NAVIGATION_FAILED"):
                navigation_terminals.append(event)
            if event_type == "COST_ONLY_DISPATCH_CERTIFICATE":
                try:
                    payload = json.loads(event["reason"])
                except (TypeError, ValueError):
                    payload = None
                if isinstance(payload, dict):
                    certificate_records.append({
                        "robot": robot, "sim_time_s": time_s,
                        "event_type": event_type, **payload,
                    })
        elif suffix == "exploration_failure":
            state["failure_messages"] += 1
            failure = {
                "robot": robot, "sim_time_s": time_s,
                "round_id": str(getattr(message, "round_id", "")),
                "task_id": str(getattr(message, "canonical_task_id", "")),
                "failure_class": _failure_name(getattr(
                    message, "failure_class", 8)),
                "path_length_m": _float(getattr(message, "path_length_m", 0.0)),
                "retry_count": int(getattr(message, "retry_count", 0)),
                "nav2_error_code": int(getattr(message, "nav2_error_code", 0)),
                "nav2_error_message": str(getattr(
                    message, "nav2_error_message", "")),
            }
            failure_records.append(failure)
            _increment(state["failure_classes"], failure["failure_class"])

    certificate_status = _certificate_status(
        certificate_records, bid_records, pair_decisions, cooperation_events)
    return {
        "available": not any(parse_errors.values()),
        "topic_counts": topic_counts,
        "payload_parse_errors": {
            topic: count for topic, count in parse_errors.items() if count
        },
        "robots": robots_out,
        "status_records": status_records,
        "cooperation_events": cooperation_events,
        "bid_records": bid_records,
        "pair_decisions": pair_decisions,
        "dispatches": dispatches,
        "navigation_terminals": navigation_terminals,
        "failure_records": failure_records,
        "certificate_records": certificate_records,
        "certificate_status": certificate_status,
        "start_release_records": start_release_records,
        "generation_records": generation_records,
        "dispatch_to_terminal_latency_s": _dispatch_latencies(
            dispatches, navigation_terminals),
    }


def _dispatch_latencies(dispatches, terminals):
    pending = {}
    values = []
    events = [(item, "dispatch") for item in dispatches]
    events += [(item, "terminal") for item in terminals]
    for event, kind in sorted(events,
                              key=lambda item: item[0].get("sim_time_s") or 0.0):
        key = (event.get("robot"), event.get("task_id", ""))
        if kind == "dispatch":
            pending[key] = event.get("sim_time_s")
        elif key in pending:
            start = pending.pop(key)
            if start is not None and event.get("sim_time_s") is not None:
                values.append(float(event["sim_time_s"]) - float(start))
    values.sort()
    result = {"count": len(values), "values_s": values}
    if values:
        result.update({
            "minimum_s": values[0],
            "median_s": values[len(values) // 2],
            "maximum_s": values[-1],
        })
    return result
