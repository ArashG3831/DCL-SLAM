#!/usr/bin/env python3
"""Diagnostic map-vs-Webots-geometry metrics for the motion course.

The characterization world contains only a 6 m x 6 m rectangular arena.  This
tool treats the four perimeter walls as reference geometry and never feeds the
reference into ROS, SLAM, or control.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path

from PIL import Image


def wall_distance(x: float, y: float, half_extent: float = 3.0) -> float:
    if -half_extent <= x <= half_extent and -half_extent <= y <= half_extent:
        return min(x + half_extent, half_extent - x,
                   y + half_extent, half_extent - y)
    qx = min(half_extent, max(-half_extent, x))
    qy = min(half_extent, max(-half_extent, y))
    return math.hypot(x - qx, y - qy)


def _p95(values):
    if not values:
        return None
    if len(values) < 2:
        return values[0]
    return statistics.quantiles(values, n=20, method="inclusive")[18]


def analyze(directory: Path, tolerance_m: float = 0.12):
    metadata_path = directory / "slam_map_updates.csv"
    image_path = directory / "slam_final_map.pgm"
    if not metadata_path.exists() or not image_path.exists():
        return {"directory": str(directory), "valid": False,
                "reason": "missing final map or metadata"}
    with metadata_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        return {"directory": str(directory), "valid": False,
                "reason": "empty map metadata"}
    last = rows[-1]
    resolution = float(last["resolution_m"])
    origin_x = float(last["origin_x_m"])
    origin_y = float(last["origin_y_m"])
    image = Image.open(image_path).convert("L")
    occupied = []
    pixels = image.load()
    # MotionSlamRecorder reverses rows when writing PGM, so undo that here.
    for row in range(image.height):
        grid_y = image.height - 1 - row
        for column in range(image.width):
            if pixels[column, row] <= 10:
                occupied.append((
                    origin_x + (column + 0.5) * resolution,
                    origin_y + (grid_y + 0.5) * resolution))
    distances = [wall_distance(x, y) for x, y in occupied]
    near = [distance for distance in distances if distance <= tolerance_m]
    return {
        "directory": str(directory),
        "valid": bool(occupied),
        "reference": "6m x 6m RectangleArena perimeter walls",
        "map_resolution_m": resolution,
        "occupied_cells": len(occupied),
        "occupied_cells_within_tolerance": len(near),
        "occupied_within_tolerance_fraction": len(near) / len(occupied)
        if occupied else None,
        "occupied_wall_distance_mean_m": statistics.mean(distances)
        if distances else None,
        "occupied_wall_distance_median_m": statistics.median(distances)
        if distances else None,
        "occupied_wall_distance_p95_m": _p95(distances),
        "ghost_occupied_cells_beyond_tolerance": len(occupied) - len(near),
        "tolerance_m": tolerance_m,
        "observability_note": (
            "This is geometric accuracy for the known arena perimeter; it is "
            "not a complete map-entropy or unseen-obstacle metric."),
    }


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    directories = [args.root] if (args.root / "slam_final_map.pgm").exists() else sorted(
        path for path in args.root.iterdir() if path.is_dir())
    results = [analyze(path) for path in directories]
    output = args.output or args.root / "map_quality.json"
    output.write_text(json.dumps(results, indent=2, sort_keys=True) + "\n",
                      encoding="utf-8")
    print(f"cases={len(results)} output={output}")


if __name__ == "__main__":
    main()
