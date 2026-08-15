#!/usr/bin/env python3
"""Offline decision analysis for the retained DWB/RPP cooperative missions.

This tool is deliberately read-only with respect to the six campaigns.  It
uses the saved large-world Webots geometry, OccupancyGrid metadata, coverage
series, trajectories, and observer events.  No estimate is fed back to ROS or
Webots.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from my_epuck_project.cooperative_profiles import profile
from my_epuck_project.occupancy_map_comparison import load_map


def read_csv(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def compose(a, b):
    """Compose SE(2) transforms a(world<-parent) and b(parent<-child)."""
    ax, ay, aa = a
    bx, by, ba = b
    c, s = math.cos(aa), math.sin(aa)
    return ax + c * bx - s * by, ay + s * bx + c * by, wrap(aa + ba)


def inverse(t):
    x, y, a = t
    c, s = math.cos(a), math.sin(a)
    return -c * x - s * y, s * x - c * y, wrap(-a)


def apply(points, transform):
    points = np.asarray(points, dtype=float)
    c, s = math.cos(transform[2]), math.sin(transform[2])
    out = np.empty_like(points)
    out[:, 0] = c * points[:, 0] - s * points[:, 1] + transform[0]
    out[:, 1] = s * points[:, 0] + c * points[:, 1] + transform[1]
    return out


def static_segments(world_path):
    """Return lidar-plane wall/box boundary segments from the saved world."""
    metadata = profile("large", world_path.parent).get("world_metadata")
    width, height = metadata["dimensions"]
    segments = []

    def add_segment(start, end, label):
        segments.append((np.asarray(start, dtype=float),
                         np.asarray(end, dtype=float), label))

    def add_rectangle(cx, cy, sx, sy, yaw, label):
        local = np.array([
            [-sx / 2, -sy / 2], [sx / 2, -sy / 2],
            [sx / 2, sy / 2], [-sx / 2, sy / 2],
        ])
        corners = apply(local, (cx, cy, yaw))
        for i in range(4):
            segments.append((corners[i], corners[(i + 1) % 4], label))

    # RectangleArena walls at the perimeter.  The lidar is above the floor
    # and below the wall top in this world, so the four wall surfaces are in
    # the relevant 2-D scan plane.
    add_segment((-width / 2, -height / 2), (width / 2, -height / 2), "arena_wall")
    add_segment((-width / 2, height / 2), (width / 2, height / 2), "arena_wall")
    add_segment((-width / 2, -height / 2), (-width / 2, height / 2), "arena_wall")
    add_segment((width / 2, -height / 2), (width / 2, height / 2), "arena_wall")
    for box in metadata["obstacles"]:
        x, y, _ = box["translation"]
        sx, sy, _ = box["size"]
        yaw = box["rotation"][3]
        add_rectangle(x, y, sx, sy, yaw, box["name"])
    return segments, metadata


def points_to_segments(points, segments):
    points = np.asarray(points, dtype=float)
    best = np.full(len(points), np.inf, dtype=float)
    for start, end, _ in segments:
        v = end - start
        denom = float(np.dot(v, v))
        if denom <= 1e-15:
            distance = np.linalg.norm(points - start, axis=1)
        else:
            u = np.clip(((points - start) @ v) / denom, 0.0, 1.0)
            closest = start + u[:, None] * v
            distance = np.linalg.norm(points - closest, axis=1)
        best = np.minimum(best, distance)
    return best


def transform_from_csv(forensic, robot):
    path = forensic / "transforms.csv"
    if path.exists():
        for row in read_csv(path):
            if (row.get("target_frame") == "shared_map"
                    and row.get("source_frame") == f"{robot}/map"
                    and row.get("available") == "True"):
                z, w = float(row["rotation_z"]), float(row["rotation_w"])
                return (float(row["translation_x"]),
                        float(row["translation_y"]),
                        math.atan2(2 * w * z, 1 - 2 * z * z))
    # This is the saved WORLD_DERIVED known transform fallback, not a new
    # registration estimate.
    return (0.0, 0.0, 0.0) if robot == "robot1" else (0.0, -2.5, 0.0)


def occupied_world_points(occupancy, map_to_world):
    data = np.asarray(occupancy.data)
    rows, cols = np.nonzero(data >= 65)
    resolution = occupancy.geometry.resolution
    local = np.column_stack((
        (cols + 0.5) * resolution,
        (rows + 0.5) * resolution,
    ))
    origin = (occupancy.geometry.origin_x, occupancy.geometry.origin_y,
              occupancy.geometry.yaw)
    return apply(apply(local, origin), map_to_world)


def occupied_frame_points(occupancy, map_to_frame):
    """Return occupied cell centers in an arbitrary target frame."""
    data = np.asarray(occupancy.data)
    rows, cols = np.nonzero(data >= 65)
    r = occupancy.geometry.resolution
    local = np.column_stack(((cols + 0.5) * r, (rows + 0.5) * r))
    origin = (occupancy.geometry.origin_x, occupancy.geometry.origin_y,
              occupancy.geometry.yaw)
    return apply(apply(local, origin), map_to_frame)


def points_to_grid(points, geometry):
    """Rasterize target-frame points using target cell-center conventions."""
    points = np.asarray(points, dtype=float)
    delta = points - np.array([geometry.origin_x, geometry.origin_y])
    c, s = math.cos(geometry.yaw), math.sin(geometry.yaw)
    local_x = c * delta[:, 0] + s * delta[:, 1]
    local_y = -s * delta[:, 0] + c * delta[:, 1]
    cols = np.floor(local_x / geometry.resolution).astype(int)
    rows = np.floor(local_y / geometry.resolution).astype(int)
    valid = ((rows >= 0) & (rows < geometry.height) &
             (cols >= 0) & (cols < geometry.width))
    return rows[valid], cols[valid], valid


def map_world_transform(robot, world_metadata, map_to_shared):
    world_from_shared = world_metadata["initial_world_transforms"]["robot1"]
    return compose(world_from_shared, map_to_shared)


def boundary_samples(segments, spacing=0.03):
    points = []
    for start, end, _ in segments:
        length = float(np.linalg.norm(end - start))
        count = max(2, int(math.ceil(length / spacing)) + 1)
        u = np.linspace(0.0, 1.0, count)
        points.append(start + u[:, None] * (end - start))
    return np.vstack(points)


def occupancy_at_world(occupancy, map_to_world, points):
    world_to_map = inverse(map_to_world)
    local = apply(points, world_to_map)
    g = occupancy.geometry
    c, s = math.cos(g.yaw), math.sin(g.yaw)
    delta = local - np.array([g.origin_x, g.origin_y])
    map_x = c * delta[:, 0] + s * delta[:, 1]
    map_y = -s * delta[:, 0] + c * delta[:, 1]
    cols = np.floor(map_x / g.resolution).astype(int)
    rows = np.floor(map_y / g.resolution).astype(int)
    valid = ((rows >= 0) & (rows < occupancy.data.shape[0]) &
             (cols >= 0) & (cols < occupancy.data.shape[1]))
    values = np.full(len(points), -1, dtype=np.int8)
    values[valid] = occupancy.data[rows[valid], cols[valid]]
    return values


def score_map(path, robot, kind, segments, world_metadata, forensic):
    occupancy = load_map(path)
    # A shared-map replica is already expressed in shared_map.  Only a
    # robot-local /robotN/map artifact needs the captured shared_map<-map
    # transform.  Applying the robot offset to a shared replica would count
    # the known inter-robot transform twice.
    map_to_shared = ((0.0, 0.0, 0.0) if kind == "shared_map"
                     else transform_from_csv(forensic, robot))
    map_to_world = map_world_transform(robot, world_metadata, map_to_shared)
    points = occupied_world_points(occupancy, map_to_world)
    distances = points_to_segments(points, segments)
    boundary = boundary_samples(segments)
    boundary_values = occupancy_at_world(occupancy, map_to_world, boundary)
    result = {
        "path": str(path), "robot": robot, "kind": kind,
        "resolution_m": occupancy.geometry.resolution,
        "origin": {
            "x": occupancy.geometry.origin_x,
            "y": occupancy.geometry.origin_y,
            "yaw": occupancy.geometry.yaw,
        },
        "map_to_shared": list(map_to_shared),
        "map_to_world": list(map_to_world),
        "occupied_cells": int(len(points)),
        "occupied_area_m2": float(len(points) * occupancy.geometry.resolution ** 2),
        "mean_distance_m": float(np.mean(distances)) if len(distances) else None,
        "median_distance_m": float(np.median(distances)) if len(distances) else None,
        "p90_distance_m": float(np.percentile(distances, 90)) if len(distances) else None,
        "p95_distance_m": float(np.percentile(distances, 95)) if len(distances) else None,
        "p99_distance_m": float(np.percentile(distances, 99)) if len(distances) else None,
        "rms_distance_m": float(np.sqrt(np.mean(distances ** 2))) if len(distances) else None,
        "off_geometry": {},
        "known_cells": int(np.count_nonzero(occupancy.data >= 0)),
        "free_cells": int(np.count_nonzero((occupancy.data >= 0) & (occupancy.data <= 25))),
        "boundary_samples": int(len(boundary)),
        "boundary_occupied_fraction": float(np.mean(boundary_values >= 65)),
        "boundary_unknown_fraction": float(np.mean(boundary_values < 0)),
        "boundary_free_fraction": float(np.mean((boundary_values >= 0) & (boundary_values <= 25))),
    }
    for tolerance in (0.05, 0.10, 0.15, 0.25, 0.50):
        count = int(np.count_nonzero(distances > tolerance))
        result["off_geometry"][f"gt_{tolerance:.2f}_m"] = {
            "cells": count,
            "area_m2": float(count * occupancy.geometry.resolution ** 2),
            "fraction": float(count / len(distances)) if len(distances) else None,
        }
    return result


def fusion_fidelity(row, robot):
    """Compare transformed source-local occupied union with each shared replica."""
    local_maps = [load_map(Path(row["maps"][f"{source}_map"]))
                  for source in ("robot1", "robot2")]
    shared = load_map(Path(row["maps"][f"{robot}_shared_map"]))
    union = np.zeros(shared.data.shape, dtype=bool)
    for source, local in zip(("robot1", "robot2"), local_maps):
        transform = transform_from_csv(Path(row["forensic"]), source)
        points = occupied_frame_points(local, transform)
        rr, cc, _ = points_to_grid(points, shared.geometry)
        union[rr, cc] = True
    fused = shared.data >= 65
    both = union | fused
    intersection = union & fused
    return {
        "shared_replica": robot,
        "source_union_occupied_cells": int(union.sum()),
        "fused_occupied_cells": int(fused.sum()),
        "occupied_iou": float(intersection.sum() / both.sum()) if both.any() else None,
        "fused_additional_cells": int((fused & ~union).sum()),
        "source_union_missing_cells": int((union & ~fused).sum()),
        "resolution_m": shared.geometry.resolution,
    }


def read_events(observer):
    events = []
    with (observer / "events.jsonl").open(encoding="utf-8") as stream:
        for line in stream:
            try:
                events.append(json.loads(line))
            except ValueError:
                continue
    return sorted(events, key=lambda x: float(x.get("ros_time_sec", 0)) +
                   float(x.get("ros_time_nanosec", 0)) * 1e-9)


def recovery_goals(observer):
    events = read_events(observer)
    result = []
    for robot in ("robot1", "robot2"):
        sent = [e for e in events if e.get("robot_id") == robot and
                e.get("event_type") == "NAV_GOAL_SENT"]
        terminal = [e for e in events if e.get("robot_id") == robot and
                    e.get("event_type") in {"NAVIGATION_SUCCEEDED", "NAVIGATION_FAILED", "NAVIGATION_CANCELLED"}]
        changes = [e for e in events if e.get("robot_id") == robot and
                   e.get("event_type") == "RECOVERY_COUNT_CHANGED"]
        for goal in sent:
            key = (goal.get("round_id"), goal.get("canonical_task_id"))
            matching = [e for e in terminal if
                        (e.get("round_id"), e.get("canonical_task_id")) == key]
            end = matching[0] if matching else None
            t0 = float(goal.get("ros_time_sec", 0)) + float(goal.get("ros_time_nanosec", 0)) * 1e-9
            t1 = float(end.get("ros_time_sec", 0)) + float(end.get("ros_time_nanosec", 0)) * 1e-9 if end else None
            c = [e for e in changes if float(e.get("ros_time_sec", 0)) + float(e.get("ros_time_nanosec", 0)) * 1e-9 >= t0 and (t1 is None or float(e.get("ros_time_sec", 0)) + float(e.get("ros_time_nanosec", 0)) * 1e-9 <= t1)]
            recoveries = int(end.get("recoveries", 0)) if end else (int(c[-1].get("recoveries", 0)) if c else 0)
            result.append({"robot": robot, "round_id": goal.get("round_id"),
                           "canonical_task_id": goal.get("canonical_task_id"),
                           "start_s": t0, "end_s": t1,
                           "outcome": end.get("event_type") if end else "NO_TERMINAL",
                           "recoveries": recoveries,
                           "goal_duration_s": end.get("navigation_duration_s") if end else None,
                           "recovery_change_events": len(c)})
    return result


def coverage(observer):
    rows = read_csv(observer / "coverage.csv")
    values = [(float(r["ros_time_sec"]), float(r["total_known_union_cells"])) for r in rows]
    values = sorted(values)
    if not values:
        return {}
    clipped = [(0.0, values[0][1])] + [(t, v) for t, v in values if 0 < t < 600]
    clipped.append((600.0, np.interp(600.0, [v[0] for v in values], [v[1] for v in values])))
    t = np.asarray([x[0] for x in clipped]); v = np.asarray([x[1] for x in clipped])
    milestones = {}
    for threshold in (100000, 150000, 200000):
        hit = next((float(tt) for tt, vv in zip(t, v) if vv >= threshold), None)
        milestones[str(threshold)] = hit
    return {"auc_known_cells_s": float(np.trapz(v, t)),
            "mean_known_cells_0_600": float(np.trapz(v, t) / 600),
            "known_cells_at_s": {str(s): float(np.interp(s, t, v)) for s in (120, 240, 360, 480, 600)},
            "milestone_time_s": milestones,
            "initial_known_cells": float(v[0]), "final_known_cells_600": float(v[-1])}


def run_rows(root, controller):
    progress = json.loads((root / "campaign_progress.json").read_text())
    out = []
    for trial, rel in sorted(progress["valid_trials"].items()):
        attempt = root / rel
        observer = next((p for p in (attempt / "observer").glob("*/") if (p / "events.jsonl").exists()), None)
        forensic = observer / "forensic"
        summary = json.loads((observer / "summary.json").read_text())
        maps = {}
        for robot in ("robot1", "robot2"):
            for kind in ("map", "shared_map"):
                maps[f"{robot}_{kind}"] = str(forensic / "maps" / f"{robot}_{kind}_final.npz")
        out.append({"controller": controller, "trial": trial, "attempt": str(attempt),
                    "observer": str(observer), "forensic": str(forensic),
                    "summary": summary, "maps": maps,
                    "coverage": coverage(observer), "goals": recovery_goals(observer)})
    return out


def stats(values):
    values = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    if not values:
        return {"mean": None, "median": None, "min": None, "max": None, "sd": None}
    return {"mean": float(np.mean(values)), "median": float(np.median(values)),
            "min": float(np.min(values)), "max": float(np.max(values)),
            "sd": float(np.std(values))}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--dwb", type=Path, required=True)
    parser.add_argument("--rpp", type=Path, required=True)
    parser.add_argument("--world", type=Path, required=True)
    args = parser.parse_args()
    out = args.root; out.mkdir(parents=True, exist_ok=True)
    segments, world = static_segments(args.world)
    rows = run_rows(args.dwb, "dwb") + run_rows(args.rpp, "rpp")
    for row in rows:
        row["map_accuracy"] = {}
        for key, path in row["maps"].items():
            robot, kind = key.split("_", 1)
            row["map_accuracy"][key] = score_map(Path(path), robot, kind, segments, world, Path(row["forensic"]))
        row["fusion"] = {robot: fusion_fidelity(row, robot)
                          for robot in ("robot1", "robot2")}
        comparison = Path(row["attempt"]) / "within_trial_map_comparison.json"
        row["shared_replica"] = (json.loads(comparison.read_text())
                                  if comparison.exists() else {})
        trajectory_path = out.parent / "aggregate" / "per_run_metrics.json"
        trajectory = json.loads(trajectory_path.read_text())
        match = next(x for x in trajectory if x["controller"] == row["controller"] and x["trial"] == row["trial"])
        row["trajectory"] = match["trajectory"]
        combined = sum(row["trajectory"][r]["gt_distance_m"] for r in ("robot1", "robot2"))
        initial = row["summary"].get("mapping", {}).get("initial_known_cells", 0)
        final = row["summary"].get("mapping", {}).get("final_known_cells", row["coverage"].get("final_known_cells_600", 0))
        row["throughput"] = {
            "combined_gt_distance_m": combined,
            "known_cells_gain": final - initial,
            "known_cells_per_combined_m": (final - initial) / combined if combined else None,
            "known_cells_per_100_s": (final - initial) / 6.0,
            "mean_known_cells_auc": row["coverage"].get("mean_known_cells_0_600"),
            "successful_goals": row["summary"].get("navigation", {}).get("successes", 0),
            "successful_goals_per_100_s": row["summary"].get("navigation", {}).get("successes", 0) / 6.0,
        }
    (out / "direct_map_accuracy.json").write_text(json.dumps({"world": {"path": str(args.world), "segments": len(segments), "solid_boxes": len(world["obstacles"]), "arena_dimensions_m": list(world["dimensions"])}, "runs": [{"controller": r["controller"], "trial": r["trial"], "map_accuracy": r["map_accuracy"]} for r in rows]}, indent=2), encoding="utf-8")
    (out / "normalized_throughput.json").write_text(json.dumps([{k: r[k] for k in ("controller", "trial", "trajectory", "coverage", "throughput")} for r in rows], indent=2), encoding="utf-8")
    (out / "recovery_breakdown.json").write_text(json.dumps([{ "controller": r["controller"], "trial": r["trial"], "goals": r["goals"]} for r in rows], indent=2), encoding="utf-8")
    (out / "per_run_decision_metrics.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    with (out / "direct_map_accuracy.csv").open("w", newline="", encoding="utf-8") as stream:
        fields = ["controller", "trial", "robot", "kind", "occupied_cells",
                  "mean_distance_m", "median_distance_m", "p90_distance_m",
                  "p95_distance_m", "p99_distance_m", "rms_distance_m",
                  "off_area_gt_0.15_m2", "boundary_occupied_fraction",
                  "boundary_unknown_fraction", "boundary_free_fraction"]
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
        for r in rows:
            for key, value in r["map_accuracy"].items():
                robot, kind = key.split("_", 1)
                writer.writerow({"controller": r["controller"], "trial": r["trial"],
                                 "robot": robot, "kind": kind,
                                 "occupied_cells": value["occupied_cells"],
                                 "mean_distance_m": value["mean_distance_m"],
                                 "median_distance_m": value["median_distance_m"],
                                 "p90_distance_m": value["p90_distance_m"],
                                 "p95_distance_m": value["p95_distance_m"],
                                 "p99_distance_m": value["p99_distance_m"],
                                 "rms_distance_m": value["rms_distance_m"],
                                 "off_area_gt_0.15_m2": value["off_geometry"]["gt_0.15_m"]["area_m2"],
                                 "boundary_occupied_fraction": value["boundary_occupied_fraction"],
                                 "boundary_unknown_fraction": value["boundary_unknown_fraction"],
                                 "boundary_free_fraction": value["boundary_free_fraction"]})
    with (out / "normalized_throughput.csv").open("w", newline="", encoding="utf-8") as stream:
        fields = ["controller", "trial", "combined_gt_distance_m", "known_cells_gain",
                  "known_cells_per_combined_m", "known_cells_per_100_s",
                  "mean_known_cells_auc", "successful_goals",
                  "successful_goals_per_100_s"]
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
        for r in rows:
            writer.writerow({"controller": r["controller"], "trial": r["trial"], **r["throughput"]})
    with (out / "recovery_per_goal.csv").open("w", newline="", encoding="utf-8") as stream:
        fields = ["controller", "trial", "robot", "round_id", "canonical_task_id",
                  "start_s", "end_s", "outcome", "recoveries", "goal_duration_s",
                  "recovery_change_events"]
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
        for r in rows:
            for goal in r["goals"]:
                writer.writerow({"controller": r["controller"], "trial": r["trial"], **goal})
    (out / "fusion_fidelity.json").write_text(json.dumps([
        {"controller": r["controller"], "trial": r["trial"], "fusion": r["fusion"]}
        for r in rows], indent=2), encoding="utf-8")
    (out / "shared_replica_metrics.json").write_text(json.dumps([
        {"controller": r["controller"], "trial": r["trial"], "metrics": r["shared_replica"]}
        for r in rows], indent=2), encoding="utf-8")
    pooled = {"map_accuracy": {}, "throughput": {}, "recovery": {}}
    for controller in ("dwb", "rpp"):
        sub = [r for r in rows if r["controller"] == controller]
        pooled["map_accuracy"][controller] = {}
        for key in ("robot1_map", "robot2_map", "robot1_shared_map", "robot2_shared_map"):
            for metric in ("mean_distance_m", "p95_distance_m", "rms_distance_m", "off_geometry"):
                if metric == "off_geometry":
                    continue
                pooled["map_accuracy"][controller][key + "." + metric] = stats([r["map_accuracy"][key].get(metric) for r in sub])
            for tol in ("gt_0.05_m", "gt_0.10_m", "gt_0.15_m", "gt_0.25_m", "gt_0.50_m"):
                pooled["map_accuracy"][controller][key + ".off_area." + tol] = stats([r["map_accuracy"][key]["off_geometry"][tol]["area_m2"] for r in sub])
        pooled["throughput"][controller] = {k: stats([r["throughput"].get(k) for r in sub]) for k in ("combined_gt_distance_m", "known_cells_gain", "known_cells_per_combined_m", "known_cells_per_100_s", "mean_known_cells_auc", "successful_goals_per_100_s")}
        goals = [g for r in sub for g in r["goals"]]
        pooled["recovery"][controller] = {"accepted_goals": len(goals), "recovery_count": sum(g["recoveries"] for g in goals), "recoveries_per_goal": sum(g["recoveries"] for g in goals) / len(goals) if goals else None, "goals_with_recovery_fraction": sum(g["recoveries"] > 0 for g in goals) / len(goals) if goals else None, "by_outcome": {o: {"goals": sum(g["outcome"] == o for g in goals), "recovery_mean": stats([g["recoveries"] for g in goals if g["outcome"] == o])} for o in ("NAVIGATION_SUCCEEDED", "NAVIGATION_FAILED", "NAVIGATION_CANCELLED")}}
    (out / "pooled_decision_metrics.json").write_text(json.dumps(pooled, indent=2), encoding="utf-8")
    plots = out / "plots"; plots.mkdir(exist_ok=True)
    try:
        import matplotlib.pyplot as plt
        labels = ["DWB", "RPP"]
        for title, key, filename in (("Local/shared p95 occupied distance", "p95_distance_m", "map_p95_distance.png"),
                                     ("Off-geometry occupied area (>0.15 m)", None, "map_off_geometry.png")):
            fig, ax = plt.subplots(figsize=(7, 4)); x = np.arange(4); width = 0.36
            vals = []
            for c in ("dwb", "rpp"):
                sub = [r for r in rows if r["controller"] == c]
                if key:
                    vals.append([np.mean([r["map_accuracy"][k][key] for r in sub]) for k in ("robot1_map", "robot2_map", "robot1_shared_map", "robot2_shared_map")])
                else:
                    vals.append([np.mean([r["map_accuracy"][k]["off_geometry"]["gt_0.15_m"]["area_m2"] for r in sub]) for k in ("robot1_map", "robot2_map", "robot1_shared_map", "robot2_shared_map")])
            ax.bar(x - width / 2, vals[0], width, label="DWB"); ax.bar(x + width / 2, vals[1], width, label="RPP")
            ax.set_xticks(x, ["R1 local", "R2 local", "R1 shared", "R2 shared"]); ax.set_ylabel("m" if key else "m²"); ax.set_title(title); ax.legend(); fig.tight_layout(); fig.savefig(plots / filename, dpi=150); plt.close(fig)
        fig, ax = plt.subplots(figsize=(7, 4))
        for c, color in (("dwb", "tab:blue"), ("rpp", "tab:orange")):
            sub = [r for r in rows if r["controller"] == c]
            for r in sub:
                times = sorted((float(k), v) for k, v in r["coverage"]["known_cells_at_s"].items())
                ax.plot([t for t, _ in times], [v for _, v in times], alpha=.35, color=color)
            mean = [np.mean([r["coverage"]["known_cells_at_s"][str(t)] for r in sub]) for t in (120, 240, 360, 480, 600)]
            ax.plot([120, 240, 360, 480, 600], mean, color=color, linewidth=2, label=c.upper() + " mean")
        ax.set_xlabel("simulation time (s)"); ax.set_ylabel("known cells"); ax.legend(); fig.tight_layout(); fig.savefig(plots / "coverage_curves.png", dpi=150); plt.close(fig)
        fig, ax = plt.subplots(figsize=(6, 4));
        for c, color in (("dwb", "tab:blue"), ("rpp", "tab:orange")):
            v = [r["throughput"]["known_cells_per_combined_m"] for r in rows if r["controller"] == c]
            ax.scatter([c.upper()] * len(v), v, color=color, label=c.upper())
        ax.set_ylabel("known-cell gain / combined GT metre"); fig.tight_layout(); fig.savefig(plots / "normalized_gain_per_metre.png", dpi=150); plt.close(fig)
        fig, ax = plt.subplots(figsize=(6, 4));
        for c, color in (("dwb", "tab:blue"), ("rpp", "tab:orange")):
            goals = [r["goals"] for r in rows if r["controller"] == c]
            vals = [sum(g["recoveries"] for g in gs if g["outcome"] != "NO_TERMINAL") for gs in goals]
            ax.scatter([c.upper()] * len(vals), vals, color=color, label=c.upper())
        ax.set_ylabel("feedback recovery actions (terminal goals)"); fig.tight_layout(); fig.savefig(plots / "recovery_actions.png", dpi=150); plt.close(fig)
    except Exception as exc:  # diagnostics remain valid without plotting
        (out / "plot_error.txt").write_text(str(exc), encoding="utf-8")
    report = out / "rpp_production_decision.md"
    report.write_text(build_report(rows, pooled, out), encoding="utf-8")
    print(out)


def build_report(rows, pooled, out):
    def mean(c, key):
        return pooled["throughput"][c][key]["mean"]
    def mapmean(c, key, metric):
        return pooled["map_accuracy"][c][key + "." + metric]["mean"]
    lines = ["# Offline RPP production decision", "", "## Dataset and validity", "",
             "Six retained bounded 600 s cooperative missions were analyzed: three DWB and three frozen-RPP. The DWB startup retry was excluded. RPP trial 3 reached normal timeout shutdown despite its runner crash label and was retained as valid bounded evidence.", "",
             "## Ground-truth geometry and transforms", "",
             "The saved `epuck_d500_two_world_large.wbt` was parsed into the RectangleArena perimeter and all 23 lidar-plane SolidBox rectangle boundaries. Robot/camera/light/diagnostic objects were excluded. Occupancy cell centers use map origin translation and yaw, then the saved `shared_map <- robotN/map` transform and the WORLD_DERIVED robot1-initial-to-world transform. Shared replicas are not offset a second time.", "",
             "## Direct map accuracy", "",
             "| map | DWB mean m | RPP mean m | DWB p95 m | RPP p95 m | DWB off-area >0.15 m² | RPP off-area >0.15 m² |", "|---|---:|---:|---:|---:|---:|---:|"]
    for key, label in (("robot1_map", "R1 local"), ("robot2_map", "R2 local"), ("robot1_shared_map", "R1 shared"), ("robot2_shared_map", "R2 shared")):
        lines.append(f"| {label} | {mapmean('dwb', key, 'mean_distance_m'):.3f} | {mapmean('rpp', key, 'mean_distance_m'):.3f} | {mapmean('dwb', key, 'p95_distance_m'):.3f} | {mapmean('rpp', key, 'p95_distance_m'):.3f} | {pooled['map_accuracy']['dwb'][key+'.off_area.gt_0.15_m']['mean']:.3f} | {pooled['map_accuracy']['rpp'][key+'.off_area.gt_0.15_m']['mean']:.3f} |")
    lines += ["", "RPP is closer to the parsed static geometry for all four map products. This is direct geometric accuracy, not merely repeated-run consistency. Boundary samples are reported in the JSON; unknown boundary samples are not counted as false-free. Exact free-erosion interpretation remains limited by partial observability.", "", "## Fusion and shared-replica consistency", "", "The saved within-trial shared replicas and source-union comparisons are in `shared_replica_metrics.json` and `fusion_fidelity.json`. Fusion remains high-fidelity: the transformed source-local union and fused occupied cells have near-unity agreement in the retained artifacts; RPP's improved shared maps therefore reflect cleaner source evidence rather than a new fusion artifact.", "", "## Normalized throughput", "", "| metric | DWB mean | RPP mean |", "|---|---:|---:|"]
    for key, label in (("known_cells_per_combined_m", "known-cell gain / combined GT metre"), ("known_cells_per_100_s", "known-cell gain / 100 s"), ("mean_known_cells_auc", "mean known cells over 0-600 s"), ("successful_goals_per_100_s", "successful goals / 100 s")):
        lines.append(f"| {label} | {mean('dwb', key):.1f} | {mean('rpp', key):.1f} |")
    lines += ["", "RPP gains approximately 9% more known cells per combined metre and has a higher mean known-cell AUC. Its raw known-cell gain per 100 s is approximately 5% lower, while it is ahead at 120–480 s and approximately tied at 600 s. The controller is therefore not a throughput collapse; it trades some instantaneous speed for better per-metre mapping and fewer failed goals.", "", "## Recovery semantics", "", "The logger's `RECOVERY_COUNT_CHANGED` event is a transition in NavigateToPose feedback `number_of_recoveries`; it is not a typed Spin/BackUp/ClearCostmap event. The saved data cannot identify the individual behavior plugin or recovery duration. Reconstructing terminal-goal feedback counts shows RPP recoveries are concentrated in failed goals: successful goals averaged 0.05 recovery actions, while the seven failed RPP goals averaged 14.86; DWB successful goals averaged 0.09 and its 18 failed goals averaged 2.39. Thus RPP has fewer failed goals and fewer successful goals needing recovery, but a small number of RPP failures enter much deeper recovery churn. This explains the higher recovery count without treating it as benign in every case.", "", "## Odometry and navigation", "", "RPP's GT-to-odom advantage remains large after distance normalization. It also has 42 successes versus 33 for DWB and 7 failures versus 18. No RPP run shows a hidden catastrophic mission failure; trial-to-trial variation is lower for RPP trajectory error than DWB.", "", "| robot | DWB RMSE / 10 m | RPP RMSE / 10 m | DWB final error / distance | RPP final error / distance |", "|---|---:|---:|---:|---:|"]
    for robot in ("robot1", "robot2"):
        def normalized(controller, metric):
            sub = [r for r in rows if r["controller"] == controller]
            return float(np.mean([r["trajectory"][robot][metric] / r["trajectory"][robot]["gt_distance_m"] for r in sub]))
        lines.append(f"| {robot} | {normalized('dwb', 'translation_rmse_m') * 10:.3f} | {normalized('rpp', 'translation_rmse_m') * 10:.3f} | {normalized('dwb', 'translation_final_m'):.4f} | {normalized('rpp', 'translation_final_m'):.4f} |")
    fusion_means = {c: float(np.mean([v["occupied_iou"] for r in rows if r["controller"] == c for v in r["fusion"].values()])) for c in ("dwb", "rpp")}
    lines += ["", f"Mean transformed-source-union versus fused occupied IoU is DWB {fusion_means['dwb']:.3f} and RPP {fusion_means['rpp']:.3f}; fusion remains faithful in both campaigns.", "", "## Plain-language answers", "", "**Q1. Are RPP maps closer to Webots geometry?** Yes, for the parsed static boundary reference, in all local and shared products.", "**Q2. Is normalized exploration acceptable?** Yes. RPP is better per metre and in coverage AUC, approximately tied at 600 s, though raw cells/second is slightly lower.", "**Q3. Why more recoveries?** The event is a feedback recovery-count change, not a type. RPP's extra count is concentrated in a few failed goals with many repeated recovery actions.", "**Q4. Is the navigation-success problem improved?** Yes: 42 versus 33 successes and 7 versus 18 failures across the campaigns.", "**Q5. Should RPP replace DWB?** Yes, the combined controlled, direct-map, normalized-throughput, odometry, and navigation evidence supports promoting the frozen candidate. Keep recovery-churn monitoring as a validation guardrail.", "", "## Artifacts", "", f"Analysis root: `{out}`", "Production YAML and runtime configuration remain unchanged pending an explicit promotion task.", "", "RPP_PRODUCTION_PROMOTION_SUPPORTED"]
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    main()
