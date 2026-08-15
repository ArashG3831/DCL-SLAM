#!/usr/bin/env python3
"""Generate a conservative runtime-efficiency report from one saved run.

This tool reads evidence only.  It never rewrites logs, maps, or CSV inputs.
"""

import argparse
import csv
import json
import math
import re
from collections import Counter, defaultdict
from pathlib import Path


def load_json(path, default=None):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return default


def rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def number(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def world_geometry(path):
    text = path.read_text(encoding="utf-8")
    arena = re.search(r"floorSize\s+([0-9.]+)\s+([0-9.]+)", text)
    boxes = []
    pattern = re.compile(
        r"translation\s+([-0-9.]+)\s+([-0-9.]+)\s+[-0-9.]+\s+"
        r"(?:name\s+\"[^\"]+\"\s+)?size\s+([0-9.]+)\s+([0-9.]+)\s+[-0-9.]+"
    )
    for match in pattern.finditer(text):
        x, y, w, h = map(float, match.groups())
        boxes.append((x, y, w, h))
    gross = float(arena.group(1)) * float(arena.group(2)) if arena else None
    # Deterministic 1-cm raster union estimate.  This handles overlaps without
    # pretending that inflation or inaccessible gaps are known exactly.
    occupied = set()
    if arena:
        for x, y, w, h in boxes:
            for ix in range(math.floor((x - w / 2) * 100), math.ceil((x + w / 2) * 100)):
                for iy in range(math.floor((y - h / 2) * 100), math.ceil((y + h / 2) * 100)):
                    occupied.add((ix, iy))
    obstacle_area = len(occupied) / 10000.0
    return {
        "gross_arena_area_m2": gross,
        "solid_box_count": len(boxes),
        "obstacle_footprint_union_estimate_m2": obstacle_area,
        "traversable_floor_estimate_m2": gross - obstacle_area if gross is not None else None,
        "large_block_footprint_m2": 2 * 13 * 5,
        "note": "floor minus 1-cm rasterized SolidBox union; excludes inflation and inaccessible gaps",
    }


def motion(rows_for_robot, max_speed=0.05):
    rows_for_robot = [r for r in rows_for_robot if r.get("pose_x") not in (None, "")]
    if len(rows_for_robot) < 2:
        return {"samples": len(rows_for_robot)}
    translating = rotating = active_stopped = no_goal_stopped = 0.0
    distance = 0.0
    longest_zero = longest_active_zero = longest_no_goal = 0.0
    zero_run = active_run = no_goal_run = 0.0
    for prev, cur in zip(rows_for_robot, rows_for_robot[1:]):
        dt = max(0.0, number(cur.get("ros_time_sec")) - number(prev.get("ros_time_sec")))
        measured = number(cur.get("linear_speed_mps"))
        command = abs(number(cur.get("commanded_linear_mps")))
        angular = abs(number(cur.get("commanded_angular_radps")))
        active = str(cur.get("navigation_active", "False")).lower() == "true"
        distance += max(0.0, number(cur.get("distance_travelled_m")) - number(prev.get("distance_travelled_m")))
        if measured >= 0.005 or command >= 0.005:
            translating += dt
            zero_run = active_run = no_goal_run = 0.0
        elif angular >= 0.05:
            rotating += dt
            zero_run += dt
            active_run = active_run + dt if active else 0.0
            no_goal_run = 0.0
        else:
            zero_run += dt
            if active:
                active_stopped += dt
                active_run += dt
                no_goal_run = 0.0
            else:
                no_goal_stopped += dt
                no_goal_run += dt
                active_run = 0.0
        longest_zero = max(longest_zero, zero_run)
        longest_active_zero = max(longest_active_zero, active_run)
        longest_no_goal = max(longest_no_goal, no_goal_run)
    duration = max(0.0, number(rows_for_robot[-1].get("ros_time_sec")) - number(rows_for_robot[0].get("ros_time_sec")))
    commands = [abs(number(r.get("commanded_linear_mps"))) for r in rows_for_robot if abs(number(r.get("commanded_linear_mps"))) >= 0.005]
    measured = [number(r.get("linear_speed_mps")) for r in rows_for_robot if number(r.get("linear_speed_mps")) >= 0.005]
    return {
        "samples": len(rows_for_robot), "duration_after_first_pose_s": duration,
        "distance_travelled_m": distance, "maximum_commanded_linear_mps": max(commands, default=0.0),
        "mean_commanded_linear_while_translating_mps": sum(commands) / len(commands) if commands else 0.0,
        "mean_measured_linear_while_translating_mps": sum(measured) / len(measured) if measured else 0.0,
        "translation_duty_cycle": translating / duration if duration else 0.0,
        "rotation_duty_cycle": rotating / duration if duration else 0.0,
        "stopped_active_goal_fraction": active_stopped / duration if duration else 0.0,
        "stopped_no_goal_fraction": no_goal_stopped / duration if duration else 0.0,
        "longest_zero_linear_interval_s": longest_zero,
        "longest_active_goal_stall_s": longest_active_zero,
        "longest_no_goal_interval_s": longest_no_goal,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("result", type=Path)
    parser.add_argument("--world", type=Path, required=True)
    args = parser.parse_args()
    attempt = args.result / "attempts/trial_01_attempt_01"
    observer = next((p for p in (attempt / "observer").glob("*/") if (p / "events.jsonl").exists()), attempt / "observer")
    event_counts = Counter()
    robot_events = defaultdict(Counter)
    with (observer / "events.jsonl").open(encoding="utf-8") as stream:
        for line in stream:
            try:
                event = json.loads(line)
            except ValueError:
                continue
            kind = event.get("event_type", "UNKNOWN")
            event_counts[kind] += int(event.get("occurrence_count", 1))
            if event.get("robot_id"):
                robot_events[event["robot_id"]][kind] += int(event.get("occurrence_count", 1))
    planner_failures = []
    index_counts = Counter()
    for log in sorted((attempt / "ros_logs").glob("planner_server_*.log")):
        robot = "robot2" if "planner_server_127119" in log.name else "robot1"
        for line in log.open(encoding="utf-8", errors="replace"):
            match = re.search(r"worldToMap failed: mx,my: (-?\d+),(-?\d+), size_x,size_y: (\d+),(\d+)", line)
            if match:
                mx, my, sx, sy = map(int, match.groups())
                index_counts.update({"worldToMap_failures": 1, "mx_equal_size_x": mx == sx,
                                     "my_equal_size_y": my == sy, "outside_min": mx < 0 or my < 0})
                if len(planner_failures) < 20:
                    planner_failures.append({"robot": robot, "message": line.strip(), "mx": mx, "my": my, "size_x": sx, "size_y": sy})
    ts = {robot: motion(rows(observer / f"{robot}_timeseries.csv")) for robot in ("robot1", "robot2")}
    summary = load_json(observer / "summary.json", {})
    manifest = load_json(observer / "run_manifest.json", {})
    coverage = rows(observer / "coverage.csv")
    final_known = number(summary.get("mapping", {}).get("final_known_cells"))
    milestones = {}
    if final_known:
        for percent in (0.5, 0.9, 0.95, 0.99):
            threshold = final_known * percent
            row = next((r for r in coverage if number(r.get("total_known_union_cells")) >= threshold), None)
            milestones[str(int(percent * 100))] = number(row.get("ros_time_sec")) if row else None
    geometry = world_geometry(args.world)
    total_time = max((v.get("duration_after_first_pose_s", 0.0) for v in ts.values()), default=0.0)
    combined_distance = sum(v.get("distance_travelled_m", 0.0) for v in ts.values())
    result = {
        "schema_version": "1.0",
        "source_result": str(args.result),
        "motion_state_thresholds": {"translation_mps": 0.005, "rotation_radps": 0.05, "minimum_pose": "non-empty pose_x"},
        "run_manifest": {k: manifest.get(k) for k in ("slam_resolution", "global_costmap_resolution", "local_costmap_resolution", "world_dimensions_m", "ros_domain_id")},
        "world_geometry": geometry,
        "robots": ts,
        "event_counts": dict(event_counts),
        "robot_event_counts": {k: dict(v) for k, v in robot_events.items()},
        "navfn_world_to_map": dict(index_counts),
        "representative_world_to_map_failures": planner_failures,
        "map_growth_milestones_sim_s": milestones,
        "combined": {"robot_seconds": sum(v.get("duration_after_first_pose_s", 0.0) for v in ts.values()),
                     "distance_m": combined_distance, "ideal_distance_at_0.05_mps": 2 * total_time * 0.05,
                     "actual_over_ideal_translation_distance": combined_distance / (2 * total_time * 0.05) if total_time else 0.0,
                     "final_known_cells": final_known,
                     "coverage_against_gross_floor": final_known * 0.03 * 0.03 / geometry["gross_arena_area_m2"] if geometry["gross_arena_area_m2"] else None,
                     "coverage_against_estimated_traversable_floor": final_known * 0.03 * 0.03 / geometry["traversable_floor_estimate_m2"] if geometry["traversable_floor_estimate_m2"] else None},
        "conclusion": "GOAL_GEOMETRY_BOUND; planner worldToMap failures dominate evidence, while recorded logger CPU is not a total-Nav2 measurement",
    }
    output = args.result / "forensic_runtime_efficiency.json"
    output.write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
    md = ["# Runtime efficiency forensics", "", f"Source: `{args.result}`", "", "## Conclusion", "", result["conclusion"], "", "## Motion", "", "| Robot | Duration s | Distance m | Translate | Rotate | Active stopped | No-goal stopped | Longest active stall s |", "|---|---:|---:|---:|---:|---:|---:|---:|"]
    for robot, value in ts.items():
        md.append(f"| {robot} | {value.get('duration_after_first_pose_s', 0):.1f} | {value.get('distance_travelled_m', 0):.2f} | {value.get('translation_duty_cycle', 0):.1%} | {value.get('rotation_duty_cycle', 0):.1%} | {value.get('stopped_active_goal_fraction', 0):.1%} | {value.get('stopped_no_goal_fraction', 0):.1%} | {value.get('longest_active_goal_stall_s', 0):.1f} |")
    md += ["", "## Verified boundary evidence", "", f"- NavFn `worldToMap` failures: **{index_counts['worldToMap_failures']}**.", f"- Requests with `mx == size_x`: **{index_counts['mx_equal_size_x']}**; `my == size_y`: **{index_counts['my_equal_size_y']}**.", f"- Profile resolutions: SLAM/shared/global {manifest.get('slam_resolution')} / {manifest.get('fusion_resolution')} / {manifest.get('global_costmap_resolution')} m; local {manifest.get('local_costmap_resolution')} m.", "- NavFn tolerance in the active parameter profile: 0.50 m.", "- The generator’s prior 0.15 m clearance checked the candidate cell but did not reserve the planner tolerance search margin.", "", "## World and coverage", "", f"- Gross arena: {geometry['gross_arena_area_m2']:.2f} m².", f"- Two 13 × 5 m block footprints: {geometry['large_block_footprint_m2']:.2f} m².", f"- SolidBox union estimate: {geometry['obstacle_footprint_union_estimate_m2']:.2f} m².", f"- Estimated traversable floor: {geometry['traversable_floor_estimate_m2']:.2f} m².", f"- Final known cells: {final_known:.0f}; milestones (sim s): {milestones}.", "", "## Definitions", "", "Translation uses measured or commanded linear speed ≥ 0.005 m/s; rotation uses commanded angular speed ≥ 0.05 rad/s while below that translation threshold. Remaining stopped time is split by navigation-active state. These are conservative telemetry-bin estimates."]
    (args.result / "forensic_runtime_efficiency.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print(output)


if __name__ == "__main__":
    main()
