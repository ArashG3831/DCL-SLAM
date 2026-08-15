#!/usr/bin/env python3
"""Frame-correct, time-aligned analysis for motion characterization runs.

This tool is diagnostic-only.  It aligns odometry and SLAM poses to Webots
Supervisor truth exactly once at the first timestamp for which all three
trajectories are available.  It never feeds Supervisor data back into ROS.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import statistics


def _f(row, key):
    try:
        value = row.get(key, "")
        return None if value in ("", None) else float(value)
    except (TypeError, ValueError):
        return None


def _wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _unwrap(values):
    if not values:
        return []
    result = [values[0]]
    for value in values[1:]:
        result.append(result[-1] + _wrap(value - result[-1]))
    return result


def _pose(row, prefix):
    x = _f(row, f"{prefix}x_m")
    y = _f(row, f"{prefix}y_m")
    yaw = _f(row, f"{prefix}yaw_rad")
    return None if None in (x, y, yaw) else (x, y, yaw)


def _compose(a, b):
    """SE(2) composition a*b."""
    ax, ay, ath = a
    bx, by, bth = b
    c = math.cos(ath)
    s = math.sin(ath)
    return (ax + c * bx - s * by,
            ay + s * bx + c * by,
            ath + bth)


def _inverse(pose):
    x, y, yaw = pose
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (-c * x - s * y, s * x - c * y, -yaw)


def _interpolate(rows, times, t):
    if not rows or t < times[0] or t > times[-1]:
        return None
    if len(rows) == 1:
        return _pose(rows[0], "")
    lo = 0
    hi = len(times) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if times[mid] <= t:
            lo = mid
        else:
            hi = mid
    r0, r1 = rows[lo], rows[hi]
    p0, p1 = _pose(r0, ""), _pose(r1, "")
    if p0 is None or p1 is None:
        return None
    dt = times[hi] - times[lo]
    alpha = 0.0 if dt <= 0.0 else (t - times[lo]) / dt
    return (p0[0] + alpha * (p1[0] - p0[0]),
            p0[1] + alpha * (p1[1] - p0[1]),
            p0[2] + alpha * (p1[2] - p0[2]))


def _read(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _series(rows, time_key, prefix):
    selected = []
    for row in rows:
        t = _f(row, time_key)
        p = _pose(row, prefix)
        if t is not None and p is not None:
            selected.append((t, {"x_m": p[0], "y_m": p[1], "yaw_rad": p[2]}))
    selected.sort(key=lambda item: item[0])
    if not selected:
        return [], []
    values = [item[1] for item in selected]
    unwrapped = _unwrap([v["yaw_rad"] for v in values])
    for value, yaw in zip(values, unwrapped):
        value["yaw_rad"] = yaw
    return [item[0] for item in selected], values


def _as_pose_rows(times, values):
    return [
        {"x_m": value["x_m"], "y_m": value["y_m"], "yaw_rad": value["yaw_rad"]}
        for value in values
    ]


def _interp_series(times, values, t):
    if not times or t < times[0] or t > times[-1]:
        return None
    if len(times) == 1:
        return (values[0]["x_m"], values[0]["y_m"], values[0]["yaw_rad"])
    lo, hi = 0, len(times) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if times[mid] <= t:
            lo = mid
        else:
            hi = mid
    dt = times[hi] - times[lo]
    alpha = 0.0 if dt <= 0.0 else (t - times[lo]) / dt
    a, b = values[lo], values[hi]
    return tuple(a[k] + alpha * (b[k] - a[k]) for k in ("x_m", "y_m", "yaw_rad"))


def _align_to_gt(gt0, pose0):
    return _compose(gt0, _inverse(pose0))


def _error(a, b):
    dx, dy = a[0] - b[0], a[1] - b[1]
    return math.hypot(dx, dy), abs(_wrap(a[2] - b[2]))


def analyze(directory: Path):
    truth_rows = _read(directory / "supervisor_ground_truth.csv")
    motion_paths = []
    for path in directory.glob("*.csv"):
        if path.name in {"supervisor_ground_truth.csv", "slam_trajectory.csv", "scan_timestamps.csv"}:
            continue
        with path.open(newline="", encoding="utf-8") as stream:
            fields = next(csv.reader(stream), [])
        if "ros_time_s" in fields and ("odom_x" in fields or "odom_x_m" in fields):
            motion_paths.append(path)
    if not motion_paths:
        raise ValueError(f"no motion collector CSV found in {directory}")
    motion_rows = _read(sorted(motion_paths)[0])
    slam_rows = _read(directory / "slam_trajectory.csv") if (directory / "slam_trajectory.csv").exists() else []

    # The motion collector stores both simulation elapsed time and the ROS
    # clock.  This offset maps its ROS timestamps to Supervisor sim time.
    offsets = []
    for row in motion_rows:
        ros_t, elapsed = _f(row, "ros_time_s"), _f(row, "sim_elapsed_s")
        if ros_t is not None and elapsed is not None:
            offsets.append(ros_t - elapsed)
    if not offsets:
        raise ValueError(f"no usable ROS/simulation timestamps in {directory}")
    motion_offset = statistics.median(offsets)

    gt_times, gt_values = _series(truth_rows, "sim_time_s", "world_")
    # _series expects x_m/y_m suffixes; normalize the truth fields first.
    gt_norm = [{"sim_time_s": row.get("sim_time_s", ""),
                "x_m": row.get("world_x_m", ""),
                "y_m": row.get("world_y_m", ""),
                "yaw_rad": row.get("planar_yaw_rad", "")} for row in truth_rows]
    gt_times, gt_values = _series(gt_norm, "sim_time_s", "")
    odom_norm = []
    for row in motion_rows:
        if _f(row, "ros_time_s") is None:
            continue
        odom_norm.append({"t": _f(row, "ros_time_s"), "x_m": row.get("odom_x", ""),
                          "y_m": row.get("odom_y", ""), "yaw_rad": row.get("odom_yaw", "")})
    odom_times, odom_values = _series(odom_norm, "t", "")
    slam_times, slam_values = _series(slam_rows, "sim_time_s", "")
    if not gt_times or not odom_times or not slam_times:
        return {"directory": str(directory), "valid": False, "reason": "missing trajectory"}
    # In the corrected recorder ros_time_s is already the /clock domain.  The
    # elapsed-time offset is only needed for older captures that wrote a wall
    # epoch; subtract it once in that case, never from normal sim timestamps.
    odom_to_sim_offset = motion_offset if min(odom_times) > 1.0e6 else 0.0
    odom_sim_times = [value - odom_to_sim_offset for value in odom_times]
    if max(gt_times) < 1.0e6 and min(slam_times) > 1.0e6:
        go_start = max(gt_times[0], odom_sim_times[0])
        go_end = min(gt_times[-1], odom_sim_times[-1])
        go_rows = []
        for t in gt_times:
            if not go_start <= t <= go_end:
                continue
            gt = _interp_series(gt_times, gt_values, t)
            od = _interp_series(odom_sim_times, odom_values, t)
            if gt is not None and od is not None:
                go_rows.append((t, gt, od))
        partial = {}
        if go_rows:
            gt0, od0 = go_rows[0][1], go_rows[0][2]
            align = _align_to_gt(gt0, od0)
            errors = [_error(_compose(align, od), gt) for _, gt, od in go_rows]
            trans = [item[0] for item in errors]
            yaw = [item[1] for item in errors]
            partial = {
                "aligned_samples": len(go_rows),
                "initial_transform_odom_to_gt": {
                    "x_m": align[0], "y_m": align[1], "yaw_rad": align[2]},
                "gt_vs_odom": {
                    "translation_rmse_m": math.sqrt(statistics.mean(v * v for v in trans)),
                    "yaw_rmse_rad": math.sqrt(statistics.mean(v * v for v in yaw)),
                    "final_translation_m": trans[-1],
                    "final_yaw_rad": yaw[-1],
                },
            }
        return {
            "directory": str(directory),
            "valid": False,
            "reason": "SLAM_TIMESTAMP_DOMAIN_MISMATCH",
            "supervisor_time_range_s": [gt_times[0], gt_times[-1]],
            "slam_time_range_s": [slam_times[0], slam_times[-1]],
            "note": "SLAM recorder used a wall/system epoch; no rigid pose alignment can repair a time-domain mismatch.",
            **partial,
        }

    start = max(gt_times[0], odom_sim_times[0], slam_times[0])
    end = min(gt_times[-1], odom_sim_times[-1], slam_times[-1])
    common = [t for t in gt_times if start <= t <= end]
    aligned = []
    for t in common:
        gt = _interp_series(gt_times, gt_values, t)
        od = _interp_series(odom_sim_times, odom_values, t)
        sl = _interp_series(slam_times, slam_values, t)
        if gt is not None and od is not None and sl is not None:
            aligned.append((t, gt, od, sl))
    if not aligned:
        return {"directory": str(directory), "valid": False, "reason": "no common time"}

    t0, gt0, od0, sl0 = aligned[0]
    od_align = _align_to_gt(gt0, od0)
    sl_align = _align_to_gt(gt0, sl0)
    od_errors, sl_errors, os_errors = [], [], []
    for _, gt, od, sl in aligned:
        od_a, sl_a = _compose(od_align, od), _compose(sl_align, sl)
        od_errors.append(_error(od_a, gt))
        sl_errors.append(_error(sl_a, gt))
        os_errors.append(_error(od_a, sl_a))

    def stats(values):
        trans = [v[0] for v in values]
        yaw = [v[1] for v in values]
        return {
            "translation_rmse_m": math.sqrt(statistics.mean(v * v for v in trans)),
            "yaw_rmse_rad": math.sqrt(statistics.mean(v * v for v in yaw)),
            "final_translation_m": trans[-1],
            "final_yaw_rad": yaw[-1],
        }

    result = {
        "directory": str(directory),
        "valid": True,
        "odom_to_supervisor_time_offset_s": odom_to_sim_offset,
        "common_start_sim_s": t0,
        "common_end_sim_s": aligned[-1][0],
        "aligned_samples": len(aligned),
        "initial_pose_gt": {"x_m": gt0[0], "y_m": gt0[1], "yaw_rad": gt0[2]},
        "initial_transform_odom_to_gt": {"x_m": od_align[0], "y_m": od_align[1], "yaw_rad": od_align[2]},
        "initial_transform_slam_to_gt": {"x_m": sl_align[0], "y_m": sl_align[1], "yaw_rad": sl_align[2]},
        "gt_vs_odom": stats(od_errors),
        "gt_vs_slam": stats(sl_errors),
        "odom_vs_slam": stats(os_errors),
    }

    scan_path = directory / "scan_timestamps.csv"
    if scan_path.exists():
        scan_rows = _read(scan_path)
        scan_stats = {}
        for topic in sorted({row.get("topic", "") for row in scan_rows}):
            ts = sorted(t for row in scan_rows if row.get("topic") == topic
                        for t in [_f(row, "sim_time_s")] if t is not None)
            dts = [b - a for a, b in zip(ts, ts[1:]) if b > a]
            record = {
                "count": len(ts),
                "rate_hz": (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 1 and ts[-1] > ts[0] else None,
                "median_period_s": statistics.median(dts) if dts else None,
            }
            # Ground-truth interpolation gives the measured chassis distance
            # between the observations, rather than assuming commanded speed.
            positions = []
            for stamp in ts:
                if gt_times[0] <= stamp <= gt_times[-1]:
                    pose = _interp_series(gt_times, gt_values, stamp)
                    if pose is not None:
                        positions.append((pose[0], pose[1]))
            spacings = [math.hypot(b[0] - a[0], b[1] - a[1])
                        for a, b in zip(positions, positions[1:])]
            if spacings:
                moving_spacings = [value for value in spacings if value > 1.0e-4]
                record.update({
                    "measured_spacing_median": statistics.median(spacings),
                    "measured_spacing_p95": (
                        statistics.quantiles(spacings, n=20, method='inclusive')[18]
                        if len(spacings) >= 2 else spacings[0]),
                    "measured_spacing_samples": len(spacings),
                    "moving_spacing_median": (statistics.median(moving_spacings)
                                               if moving_spacings else None),
                    "moving_spacing_p95": (
                        statistics.quantiles(moving_spacings, n=20, method='inclusive')[18]
                        if len(moving_spacings) >= 2 else (moving_spacings[0]
                                                           if moving_spacings else None)),
                    "moving_spacing_samples": len(moving_spacings),
                })
            if topic == "slam" and len(ts) > 1:
                # This is the expected Karto input after throttle_scans=3,
                # before its independent minimum-time and travel gates.
                throttled = ts[::3]
                throttled_positions = []
                for stamp in throttled:
                    if gt_times[0] <= stamp <= gt_times[-1]:
                        pose = _interp_series(gt_times, gt_values, stamp)
                        if pose is not None:
                            throttled_positions.append((pose[0], pose[1]))
                throttled_spacings = [math.hypot(b[0] - a[0], b[1] - a[1])
                                     for a, b in zip(throttled_positions, throttled_positions[1:])]
                moving_throttled_spacings = [value for value in throttled_spacings
                                             if value > 1.0e-4]
                gate_distance = math.sqrt(0.8) * 0.02
                record.update({
                    "throttle_scans": 3,
                    "expected_throttled_rate_hz": (len(throttled) - 1) / (throttled[-1] - throttled[0])
                    if len(throttled) > 1 and throttled[-1] > throttled[0] else None,
                    "expected_throttled_spacing_median": statistics.median(throttled_spacings) if throttled_spacings else None,
                    "expected_throttled_spacing_p95": (
                        statistics.quantiles(throttled_spacings, n=20, method='inclusive')[18]
                        if len(throttled_spacings) >= 2 else (throttled_spacings[0] if throttled_spacings else None)),
                    "expected_throttled_moving_spacing_median": (
                        statistics.median(moving_throttled_spacings)
                        if moving_throttled_spacings else None),
                    "expected_throttled_moving_spacing_p95": (
                        statistics.quantiles(moving_throttled_spacings, n=20, method='inclusive')[18]
                        if len(moving_throttled_spacings) >= 2 else (
                            moving_throttled_spacings[0] if moving_throttled_spacings else None)),
                    "movement_gate_distance_m": gate_distance,
                    "expected_throttled_samples_over_movement_gate": sum(
                        spacing >= gate_distance for spacing in throttled_spacings),
                    "expected_throttled_samples": len(throttled_spacings),
                })
            scan_stats[topic] = record
        result["scan_topics"] = scan_stats
    map_odom_path = directory / "map_odom.csv"
    if map_odom_path.exists():
        map_odom_rows = _read(map_odom_path)
        map_odom = []
        for row in map_odom_rows:
            t = _f(row, "sim_time_s")
            pose = _pose(row, "")
            if t is not None and pose is not None and _f(row, "lookup_ok") != 0:
                map_odom.append((t, pose))
        map_odom.sort(key=lambda item: item[0])
        translation_jumps = []
        yaw_jumps = []
        for (_, a), (_, b) in zip(map_odom, map_odom[1:]):
            translation_jumps.append(math.hypot(b[0] - a[0], b[1] - a[1]))
            yaw_jumps.append(abs(_wrap(b[2] - a[2])))
        result["map_to_odom_jumps"] = {
            "valid_samples": len(map_odom),
            "max_instantaneous_translation_change_m": max(translation_jumps)
            if translation_jumps else None,
            "max_instantaneous_yaw_change_rad": max(yaw_jumps)
            if yaw_jumps else None,
            "p95_translation_change_m": (
                statistics.quantiles(translation_jumps, n=20, method="inclusive")[18]
                if len(translation_jumps) >= 2 else (translation_jumps[0]
                                                     if translation_jumps else None)),
            "p95_yaw_change_rad": (
                statistics.quantiles(yaw_jumps, n=20, method="inclusive")[18]
                if len(yaw_jumps) >= 2 else (yaw_jumps[0] if yaw_jumps else None)),
        }
    return result


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    directories = [args.root] if (args.root / "supervisor_ground_truth.csv").exists() else sorted(
        path for path in args.root.iterdir() if path.is_dir() and (path / "supervisor_ground_truth.csv").exists())
    results = [analyze(path) for path in directories]
    output = args.output or (args.root / "pose_error_comparison.json")
    output.write_text(json.dumps(results, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"cases={len(results)} output={output}")


if __name__ == "__main__":
    main()
