"""Offline replay of the legacy passive motion anomaly detector.

The detector implementation in :mod:`experiment_metrics` is the authority.
This module only supplies the legacy telemetry rows after the run and records
the same edge-triggered events; it does not define a second set of thresholds.
"""

from __future__ import annotations

import csv
from collections import Counter
from pathlib import Path

from .experiment_metrics import MotionDetector, MotionSample


def _float(row, name, default=None):
    value = row.get(name, "")
    if value in (None, ""):
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def replay_timeseries(path: Path, robot: str, **detector_kwargs):
    """Replay one legacy ``robot*_timeseries.csv`` without changing semantics.

    The live logger calls ``MotionDetector.update`` once per telemetry row with
    the same fields used below.  Missing required pose/time/command columns
    fail closed; an empty, valid file is reported as a valid zero-event stream.
    """
    path = Path(path)
    if not path.is_file():
        return {"available": False, "reason": f"{path.name} absent"}
    detector = MotionDetector(**detector_kwargs)
    events = []
    sample_count = 0
    required = {
        "elapsed_s", "pose_x", "pose_y", "commanded_linear_mps",
        "commanded_angular_radps", "navigation_active",
    }
    try:
        with path.open(newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            missing = sorted(required - set(reader.fieldnames or ()))
            if missing:
                return {"available": False,
                        "reason": f"missing columns: {missing}"}
            for row_number, row in enumerate(reader, start=2):
                values = {
                    "time_s": _float(row, "elapsed_s"),
                    "x": _float(row, "pose_x"),
                    "y": _float(row, "pose_y"),
                    "command_linear": _float(row, "commanded_linear_mps", 0.0),
                    "command_angular": _float(
                        row, "commanded_angular_radps", 0.0),
                }
                if any(value is None for value in
                       (values["time_s"], values["x"], values["y"])):
                    # The logger itself waits for a pose before invoking the
                    # detector.  Such rows are therefore not detector input.
                    continue
                remaining = _float(row, "distance_remaining_m")
                active = str(row.get("navigation_active", ""))
                active = active.lower() in ("1", "true", "yes")
                sample = MotionSample(
                    values["time_s"], values["x"], values["y"], remaining,
                    values["command_linear"], values["command_angular"])
                near_goal = remaining is not None and remaining < 0.08
                for kind in detector.update(
                        sample, active, near_goal=near_goal):
                    events.append({"robot": str(robot),
                                   "sim_time_s": values["time_s"],
                                   "event_type": kind,
                                   "source": path.name,
                                   "row": row_number})
                sample_count += 1
    except (OSError, csv.Error) as exc:
        return {"available": False, "reason": f"read failure: {exc}"}

    counts = Counter(event["event_type"] for event in events)
    episodes = []
    starts = {}
    for event in events:
        kind = event["event_type"]
        if kind.endswith("_STARTED"):
            base = kind[:-8]
            starts[base] = event["sim_time_s"]
        elif kind.endswith("_CLEARED"):
            base = kind[:-8]
            if base in starts:
                episodes.append({
                    "type": base,
                    "start_sim_time_s": starts.pop(base),
                    "end_sim_time_s": event["sim_time_s"],
                })
    for base, start in starts.items():
        episodes.append({"type": base, "start_sim_time_s": start,
                         "end_sim_time_s": None})
    episodes.sort(key=lambda item: (item["start_sim_time_s"], item["type"]))
    return {
        "available": True,
        "source": path.name,
        "sample_count": sample_count,
        "event_count": len(events),
        "event_counts": dict(sorted(counts.items())),
        "events": events,
        "episodes": episodes,
        "detector": detector_kwargs,
    }


def replay_run(run_directory: Path, robots=("robot1", "robot2"), **kwargs):
    """Replay all available legacy telemetry streams with explicit zero/missing."""
    results = {}
    for robot in robots:
        result = replay_timeseries(
            Path(run_directory) / f"{robot}_timeseries.csv", robot, **kwargs)
        results[str(robot)] = result
    present = [item for item in results.values() if item.get("available")]
    return {
        "available": bool(present),
        "robots": results,
        "stream_count": len(present),
        "zero_event_semantics": (
            "available stream with event_count=0 is a valid zero; absent or "
            "malformed stream is unavailable"),
    }
