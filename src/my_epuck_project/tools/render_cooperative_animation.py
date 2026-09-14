#!/usr/bin/env python3
"""Offline thesis-quality renderer for saved cooperative-exploration runs.

The renderer never connects to ROS or Webots.  It uses only saved observer
artifacts.  World geometry is drawn in a fixed, equal-scale metric viewport;
the unused horizontal space is reserved for a restrained information panel.
"""

from __future__ import annotations

import argparse
import bisect
import csv
from dataclasses import dataclass, field
import json
import math
import multiprocessing
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
from functools import lru_cache
from typing import Any, Optional

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


PROJECT_SRC = Path(__file__).resolve().parents[1]
if str(PROJECT_SRC) not in sys.path:
    sys.path.insert(0, str(PROJECT_SRC))
from my_epuck_project.offline_timing_metrics import (  # noqa: E402
    analyze_event_records,
    analyze_physical_activity,
    derive_minimal_assignment_events,
    merge_native_timing_events,
    _simulation_time_for_bag_timestamp,
)


SCHEMA_VERSION = "publication_animation_3.1"
MAP_MARGIN_M = 0.45
MAX_TIME_GAP_S = 5.0
MAX_CONTINUOUS_STEP_M = 1.0
NATIVE_GOAL_STATUS_JOIN_TOLERANCE_S = 1.0
ARROW_LENGTH_M = 0.80

# OpenCV uses BGR.  These are intentionally distinct in every layer.
R1_COLOR = (48, 125, 232)       # warm orange/red, RGB #e87d30
R2_COLOR = (210, 104, 42)       # clear blue, RGB #2a68d2
R1_TRAJECTORY = (112, 103, 177)
R2_TRAJECTORY = (171, 111, 78)
FRONTIER_COLOR = (38, 158, 194) # cyan/teal, unlike either robot
GOAL_WHITE = (250, 250, 250)
TEXT_DARK = (35, 43, 53)
TEXT_MUTED = (93, 103, 115)


@dataclass(frozen=True)
class MapSnapshot:
    stamp_s: float
    path: Path
    occupancy: np.ndarray
    origin_x: float
    origin_y: float
    resolution: float

    @property
    def height(self) -> int:
        return int(self.occupancy.shape[0])

    @property
    def width(self) -> int:
        return int(self.occupancy.shape[1])

    @property
    def extent(self) -> tuple[float, float, float, float]:
        return (self.origin_x, self.origin_x + self.width * self.resolution,
                self.origin_y, self.origin_y + self.height * self.resolution)


@dataclass(frozen=True)
class PoseSample:
    stamp_s: float
    x: float
    y: float
    yaw: float
    row: dict[str, str]


@dataclass(frozen=True)
class TransformSample:
    stamp_s: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Candidate:
    stamp_s: float
    robot: str
    frontier_id: str
    physical_signature: str
    centroid: tuple[float, float]
    bounds: Optional[tuple[float, float, float, float]]
    approach: Optional[tuple[float, float]]
    score: Optional[float]
    gain: Optional[float]
    path_length_m: Optional[float]
    heading_rad: Optional[float]
    geometry: tuple[tuple[float, float], ...]
    frontier_cells: tuple[tuple[int, int], ...]
    grid_origin: tuple[float, float, float]
    grid_resolution: Optional[float]


@dataclass(frozen=True)
class FrontierRegion:
    stamp_s: float
    robot: str
    frontier_id: str
    centroid: tuple[float, float]
    cells: tuple[tuple[int, int], ...]
    bounds: Optional[tuple[float, float, float, float]] = None
    grid_origin: tuple[float, float, float] = (0.0, 0.0, 0.0)
    grid_resolution: Optional[float] = None


@dataclass(frozen=True)
class FrontierRegionSnapshot:
    stamp_s: float
    robot: str
    regions: tuple[FrontierRegion, ...]
    resolution: float
    origin: tuple[float, float, float]


@dataclass(frozen=True)
class FrontierLayer:
    """Cached pixel layer for one current frontier snapshot pair."""

    colors: np.ndarray
    coverage: np.ndarray


@dataclass(frozen=True)
class TaskRecord:
    stamp_s: float
    robot: str
    physical_signature: str
    local_frontier_id: str
    centroid: tuple[float, float]
    bounds: Optional[tuple[float, float, float, float]]
    approach: Optional[tuple[float, float]]


@dataclass(frozen=True)
class GoalEvent:
    stamp_s: float
    robot: str
    action: str
    canonical_task_id: str
    physical_signature: str
    goal_xy: Optional[tuple[float, float]]


@dataclass(frozen=True)
class BidPathBatch:
    stamp_s: float
    robot: str
    paths: dict[str, tuple[tuple[float, float], ...]]


@dataclass(frozen=True)
class OverlayData:
    candidate_batches: dict[str, tuple[tuple[float, tuple[Candidate, ...]], ...]]
    task_records: dict[str, tuple[TaskRecord, ...]]
    goal_events: dict[str, tuple[GoalEvent, ...]]
    bid_batches: dict[str, tuple[BidPathBatch, ...]]
    raw_frontier_geometry_records: int
    event_records: tuple[dict[str, Any], ...] = ()
    handoff_error_m: Optional[float] = None
    handoff_error_yaw_deg: Optional[float] = None
    distance_travelled_m: dict[str, Optional[float]] = field(default_factory=dict)
    productive_percent: dict[str, Optional[float]] = field(default_factory=dict)
    productivity_segments: dict[str, tuple[dict[str, Any], ...]] = field(default_factory=dict)
    frontier_regions: dict[str, tuple[FrontierRegionSnapshot, ...]] = field(default_factory=dict)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--campaign', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--width', type=int, default=1920)
    parser.add_argument('--height', type=int, default=1080)
    parser.add_argument('--fps', type=int, default=60)
    parser.add_argument('--start-s', type=float, default=0.0,
                        help='absolute source simulation time at video start')
    parser.add_argument('--duration-s', type=float, default=0.0,
                        help='source simulation seconds to render (0 = through artifact end)')
    parser.add_argument('--speedup', type=float, default=1.0,
                        help='source simulation seconds per output second')
    parser.add_argument('--workers', type=int, default=1,
                        help='parallel render/encode workers (1 = serial; requires external ffmpeg/libx264)')
    parser.add_argument('--title', default='DCL-SLAM')
    parser.add_argument('--policy', default='Cooperative Cost-Only')
    parser.add_argument('--style', default='thesis', choices=('thesis',))
    parser.add_argument('--stills-dir', default='')
    parser.add_argument('--still-times', default='')
    parser.add_argument('--stills-only', action='store_true')
    parser.add_argument('--show-frontiers', action='store_true', default=True)
    parser.add_argument('--hide-frontiers', action='store_false', dest='show_frontiers')
    parser.add_argument('--show-goals', action='store_true', default=True)
    parser.add_argument('--hide-goals', action='store_false', dest='show_goals')
    parser.add_argument('--show-candidates', action='store_true', default=True)
    parser.add_argument('--hide-candidates', action='store_false', dest='show_candidates')
    parser.add_argument('--show-planned-path', action='store_true', default=True)
    parser.add_argument('--hide-planned-path', action='store_false', dest='show_planned_path')
    parser.add_argument('--provenance-output', default='')
    return parser.parse_args(argv)


def find_observer_root(campaign: Path) -> Path:
    roots = []
    for path in campaign.rglob('robot1_timeseries.csv'):
        if (path.parent / 'robot2_timeseries.csv').is_file():
            roots.append(path.parent)
    if not roots:
        raise RuntimeError(f'no observer robot time-series artifacts under {campaign}')
    return sorted(roots, key=lambda p: str(p))[-1]


def load_csv(path: Path):
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def numeric(row: dict[str, str], key: str) -> Optional[float]:
    try:
        raw = row.get(key, '')
        if raw in ('', None):
            return None
        value = float(raw)
        return value if math.isfinite(value) else None
    except (TypeError, ValueError):
        return None


def load_pose_records(path: Path) -> list[Optional[PoseSample]]:
    records: list[Optional[PoseSample]] = []
    for row in load_csv(path):
        values = [numeric(row, key)
                  for key in ('elapsed_s', 'pose_x', 'pose_y', 'pose_yaw')]
        records.append(None if any(value is None for value in values) else
                       PoseSample(values[0], values[1], values[2], values[3], row))
    return records


def _yaw_from_quaternion(z: float, w: float) -> float:
    return 2.0 * math.atan2(z, w)


def load_shared_odom_transforms(path: Path, robot: str) -> list[TransformSample]:
    if not path.is_file():
        return []
    samples = []
    with path.open(newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            if row.get('target_frame') != 'shared_map':
                continue
            if row.get('source_frame') != f'{robot}/odom':
                continue
            if row.get('available') != 'True':
                continue
            values = [numeric(row, key) for key in
                      ('query_ros_time_s', 'translation_x', 'translation_y',
                       'rotation_z', 'rotation_w')]
            if any(value is None for value in values):
                continue
            samples.append(TransformSample(
                values[0], values[1], values[2],
                _yaw_from_quaternion(values[3], values[4])))
    unique = {}
    for sample in sorted(samples, key=lambda item: item.stamp_s):
        unique[round(sample.stamp_s, 6)] = sample
    return list(sorted(unique.values(), key=lambda item: item.stamp_s))


def _angle_difference(a: float, b: float) -> float:
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def transform_at(samples: list[TransformSample], stamp_s: float,
                 max_hold_s: float = 30.0) -> Optional[TransformSample]:
    if not samples or stamp_s < samples[0].stamp_s:
        return None
    if stamp_s >= samples[-1].stamp_s:
        return samples[-1] if stamp_s - samples[-1].stamp_s <= max_hold_s else None
    index = bisect.bisect_left([item.stamp_s for item in samples], stamp_s)
    if index == 0:
        return samples[0]
    left, right = samples[index - 1], samples[index]
    span = right.stamp_s - left.stamp_s
    if span <= 0.0 or span > max_hold_s:
        return None
    alpha = (stamp_s - left.stamp_s) / span
    return TransformSample(
        stamp_s,
        left.x + alpha * (right.x - left.x),
        left.y + alpha * (right.y - left.y),
        left.yaw + alpha * _angle_difference(right.yaw, left.yaw))


def transform_pose(pose: PoseSample, transform: TransformSample) -> PoseSample:
    c, s = math.cos(transform.yaw), math.sin(transform.yaw)
    return PoseSample(
        pose.stamp_s,
        transform.x + c * pose.x - s * pose.y,
        transform.y + s * pose.x + c * pose.y,
        (transform.yaw + pose.yaw + math.pi) % (2.0 * math.pi) - math.pi,
        pose.row)


def infer_handoff_time(observer: Path, fallback: float) -> float:
    accepted = []
    for path in sorted((observer.parent / 'frontend').glob('*_unknown_pose_frontend.json')):
        try:
            data = json.loads(path.read_text(encoding='utf-8'))
            value = float(data.get('accepted_ros_time_s'))
            if data.get('accepted') and math.isfinite(value):
                accepted.append(value)
        except (OSError, TypeError, ValueError, json.JSONDecodeError):
            pass
    if accepted:
        return min(accepted)
    return fallback


def load_map(path: Path) -> MapSnapshot:
    with np.load(path, allow_pickle=True) as data:
        occupancy = np.asarray(data['occupancy'], dtype=np.int8)
        metadata = json.loads(str(data['metadata_json'].item()))
    origin = metadata['origin']['position']
    stamp = metadata.get('header_stamp_s', metadata.get('received_ros_time_s', 0.0))
    return MapSnapshot(float(stamp), path, occupancy, float(origin['x']),
                       float(origin['y']), float(metadata['resolution']))


def map_snapshots(observer: Path) -> list[MapSnapshot]:
    paths = list(observer.glob('forensic/maps/*_robot1_shared_map.npz'))
    snapshots = [load_map(path) for path in paths
                 if not path.name.startswith('robot1_shared_map_final')]
    snapshots.sort(key=lambda item: item.stamp_s)
    unique, seen = [], set()
    for snapshot in snapshots:
        stamp = round(snapshot.stamp_s, 6)
        if stamp not in seen:
            unique.append(snapshot)
            seen.add(stamp)
    if not unique:
        raise RuntimeError('no time-indexed shared-map snapshots found')
    return unique


def final_map_extent(observer: Path, snapshots: list[MapSnapshot]):
    path = observer / 'forensic/maps/robot1_shared_map_final.npz'
    return load_map(path).extent if path.is_file() else snapshots[-1].extent


def compute_fixed_viewport(snapshots: list[MapSnapshot], final_extent,
                           margin_m: float = MAP_MARGIN_M):
    extents = [item.extent for item in snapshots] + [final_extent]
    return (min(item[0] for item in extents) - margin_m,
            max(item[1] for item in extents) + margin_m,
            min(item[2] for item in extents) - margin_m,
            max(item[3] for item in extents) + margin_m)


def compute_map_rect(width: int, height: int, viewport,
                     left=64, top=126, right=1480, bottom=1018):
    """Return a letterboxed map ROI and one shared pixels-per-metre scale."""
    xmin, xmax, ymin, ymax = viewport
    world_width, world_height = xmax - xmin, ymax - ymin
    scale = min((right - left) / world_width, (bottom - top) / world_height)
    map_width = max(1, int(round(world_width * scale)))
    map_height = max(1, int(round(world_height * scale)))
    map_left = left + ((right - left) - map_width) // 2
    map_top = top + ((bottom - top) - map_height) // 2
    return map_left, map_top, map_width, map_height, scale


def timing_metadata(source_start: float, source_end: float, speedup: float,
                    fps: int):
    if speedup <= 0.0 or fps <= 0:
        raise ValueError('speedup and fps must be positive')
    source_duration = max(0.0, source_end - source_start)
    output_duration = source_duration / speedup
    frames = max(1, int(math.ceil(output_duration * fps)))
    return {'source_start_s': source_start, 'source_end_s': source_end,
            'source_duration_s': source_duration, 'speedup': speedup,
            'fps': fps, 'expected_output_duration_s': output_duration,
            'expected_frame_count': frames}


def resolve_source_end(source_start: float, artifact_data_end: float,
                       duration_s: float) -> float:
    """Resolve a requested source duration relative to the requested start."""
    return source_start + duration_s if duration_s > 0.0 else artifact_data_end


def map_to_pixel(x: float, y: float, viewport, map_rect):
    xmin, _, ymin, _ = viewport
    left, top, _, height, scale = map_rect
    return (left + int(round((x - xmin) * scale)),
            top + height - int(round((y - ymin) * scale)))


def _map_colors(occupancy: np.ndarray) -> np.ndarray:
    pixels = np.full((*occupancy.shape, 3), (232, 235, 238), dtype=np.uint8)
    pixels[(occupancy >= 0) & (occupancy <= 25)] = (255, 255, 255)
    pixels[(occupancy > 25) & (occupancy < 65)] = (186, 192, 198)
    pixels[occupancy >= 65] = (42, 46, 52)
    return pixels


def render_snapshot(snapshot: MapSnapshot, viewport, map_rect):
    left, top, width, height, scale = map_rect
    image = np.full((height, width, 3), (232, 235, 238), dtype=np.uint8)
    source = cv2.resize(
        np.flipud(_map_colors(snapshot.occupancy)),
        (max(1, int(round(snapshot.width * snapshot.resolution * scale))),
         max(1, int(round(snapshot.height * snapshot.resolution * scale)))),
        interpolation=cv2.INTER_NEAREST)
    xmin, _, ymin, _ = viewport
    x0 = int(round((snapshot.origin_x - xmin) * scale))
    y0 = height - int(round((snapshot.origin_y +
                             snapshot.height * snapshot.resolution - ymin) * scale))
    sx0, sy0 = max(0, -x0), max(0, -y0)
    sx1 = min(source.shape[1], width - x0)
    sy1 = min(source.shape[0], height - y0)
    if sx0 < sx1 and sy0 < sy1:
        image[max(0, y0):max(0, y0) + sy1 - sy0,
              max(0, x0):max(0, x0) + sx1 - sx0] = source[sy0:sy1, sx0:sx1]
    return image


def split_shared_pose_segments(records: list[Optional[PoseSample]],
                               transforms: list[TransformSample], handoff_s: float):
    """Transform into shared_map and break at invalid/gapped/teleport joins."""
    segments, current = [], []
    previous = None
    for record in records:
        if record is None or record.stamp_s < handoff_s:
            if current:
                segments.append(current)
            current, previous = [], None
            continue
        transform = transform_at(transforms, record.stamp_s)
        if transform is None:
            if current:
                segments.append(current)
            current, previous = [], None
            continue
        mapped = transform_pose(record, transform)
        if previous is not None:
            dt = mapped.stamp_s - previous.stamp_s
            step = math.hypot(mapped.x - previous.x, mapped.y - previous.y)
            if dt <= 0.0 or dt > MAX_TIME_GAP_S or step > MAX_CONTINUOUS_STEP_M:
                if current:
                    segments.append(current)
                current = []
        current.append(mapped)
        previous = mapped
    if current:
        segments.append(current)
    return segments


def pose_at_segments(segments: list[list[PoseSample]], stamp_s: float):
    latest = None
    for segment in segments:
        if segment and segment[0].stamp_s <= stamp_s:
            item = segment[min(len(segment) - 1,
                              bisect.bisect_right([s.stamp_s for s in segment], stamp_s) - 1)]
            if latest is None or item.stamp_s > latest.stamp_s:
                latest = item
    return latest


def draw_segments(canvas, segments, stamp_s, viewport, map_rect, color):
    left, top, width, height, _ = map_rect
    view = canvas[top:top + height, left:left + width]
    for segment in segments:
        visible = [item for item in segment if item.stamp_s <= stamp_s]
        if len(visible) < 2:
            continue
        points = np.asarray([(map_to_pixel(item.x, item.y, viewport, map_rect)[0] - left,
                             map_to_pixel(item.x, item.y, viewport, map_rect)[1] - top)
                            for item in visible], dtype=np.int32)
        cv2.polylines(view, [points], False, color, 2, cv2.LINE_AA)


def _local_points(points, viewport, map_rect):
    left, top = map_rect[0], map_rect[1]
    return np.asarray([(map_to_pixel(x, y, viewport, map_rect)[0] - left,
                        map_to_pixel(x, y, viewport, map_rect)[1] - top)
                       for x, y in points], dtype=np.int32)


def draw_dashed(view, points, color, width=2, dash=10, gap=7):
    """Draw one continuous dash pattern across a polyline.

    The phase is deliberately carried from one segment to the next.  Resetting
    it at every path vertex makes short, densely sampled Nav2 paths appear
    solid even though they are requested as dashed paths.
    """
    period = float(dash + gap)
    if dash <= 0 or gap < 0 or period <= 0:
        return
    phase = 0.0
    for first, second in zip(points, points[1:]):
        dx, dy = second[0] - first[0], second[1] - first[1]
        length = math.hypot(dx, dy)
        if length <= 0:
            continue
        ux, uy = dx / length, dy / length
        cursor = 0.0
        while cursor < length:
            cycle_position = (phase + cursor) % period
            if cycle_position < dash:
                run = min(length - cursor, dash - cycle_position)
                end = cursor + run
                p1 = (int(round(first[0] + ux * cursor)),
                      int(round(first[1] + uy * cursor)))
                p2 = (int(round(first[0] + ux * end)),
                      int(round(first[1] + uy * end)))
                cv2.line(view, p1, p2, color, width, cv2.LINE_AA)
            else:
                run = min(length - cursor, period - cycle_position)
            # Both branches advance by a strictly positive amount for valid
            # dash/gap values, preventing a zero-length floating-point loop.
            cursor += max(run, 1e-9)
        phase = (phase + length) % period


def _candidate_batch_at(data: OverlayData, robot: str, stamp_s: float):
    batches = data.candidate_batches.get(robot, ())
    index = bisect.bisect_right([item[0] for item in batches], stamp_s) - 1
    return batches[index] if index >= 0 else None


def _candidate_at(data: OverlayData, robot: str, stamp_s: float):
    batch = _candidate_batch_at(data, robot, stamp_s)
    return batch[1] if batch is not None else ()


def _candidate_status(data: OverlayData, robot: str, stamp_s: float):
    batch = _candidate_batch_at(data, robot, stamp_s)
    if batch is None:
        return (), None
    source_stamp, candidates = batch
    return candidates, max(0.0, stamp_s - source_stamp)


def _pose_row_metric(pose: Optional[PoseSample], key: str):
    if pose is None:
        return None
    try:
        value = float(pose.row.get(key, ''))
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _display_speed(pose: Optional[PoseSample], key: str):
    value = _pose_row_metric(pose, key)
    if value is None:
        return None
    return 0.0 if abs(value) < 0.0005 else value


def _productivity_at(data: OverlayData, robot: str, stamp_s: float):
    """Return cumulative physical productivity through the current frame."""
    feasible = 0.0
    productive = 0.0
    for item in data.productivity_segments.get(robot, ()):
        start = float(item.get('start_s', 0.0))
        end = min(float(item.get('end_s', 0.0)), stamp_s)
        if end <= start or item.get('classification') not in (
                'TRANSLATING', 'ROTATING_ONLY', 'STATIONARY'):
            continue
        duration = end - start
        feasible += duration
        if item.get('classification') in ('TRANSLATING', 'ROTATING_ONLY'):
            productive += duration
    if feasible <= 0.0:
        return None
    return 100.0 * productive / feasible


def _traffic_labels(data: OverlayData, stamp_s: float):
    """Return compact per-robot traffic state labels at a rendered frame."""
    states = {'robot1': 'clear', 'robot2': 'clear'}
    for event in data.event_records:
        event_stamp = _event_time(event)
        if event_stamp is None or event_stamp > stamp_s:
            break
        kind = str(event.get('event_type', ''))
        robot = event.get('robot_id')
        if kind in {'TRAFFIC_WAITING', 'WAITING_FOR_TRAFFIC'} and robot in states:
            states[robot] = 'waiting'
        elif kind == 'TRAFFIC_PRIORITY_GRANTED' and robot in states:
            states[robot] = 'priority'
        elif kind in {'TRAFFIC_CONFLICT_CLEARED',
                      'TRAFFIC_RELEASED_FRESH_REALLOCATION'}:
            states = {'robot1': 'clear', 'robot2': 'clear'}

    labels = {}
    for robot, state in states.items():
        other = 'robot2' if robot == 'robot1' else 'robot1'
        other_label = 'R2' if other == 'robot2' else 'R1'
        if state == 'waiting':
            labels[robot] = f'traffic waiting | {other_label} moving'
        elif state == 'priority' and states[other] == 'waiting':
            labels[robot] = f'traffic moving | {other_label} waiting'
        elif state == 'priority':
            labels[robot] = 'traffic priority'
        else:
            labels[robot] = 'traffic clear'
    return labels


def _task_for_goal(data: OverlayData, event: GoalEvent):
    if event.goal_xy is not None:
        return event.goal_xy
    # Canonical physical tasks may be represented in the peer's task
    # snapshot after union construction.  Search both recorded task streams
    # so an active commitment never loses its target merely because the
    # dispatching robot did not originate that snapshot.
    choices = [item for robot in ('robot1', 'robot2')
               for item in data.task_records.get(robot, ())
               if ((event.canonical_task_id and
                    item.local_frontier_id == event.canonical_task_id) or
                   (event.physical_signature and
                    item.physical_signature == event.physical_signature))]
    if not choices:
        return None
    choices.sort(key=lambda item: abs(item.stamp_s - event.stamp_s))
    return choices[0].approach or choices[0].centroid


def _native_navigation_goal_events(observer: Path, events, task_records):
    """Reconstruct minimal-coordinator goal lifecycle from retained evidence.

    The action replay proves that a NavigateToPose goal existed; the nearby
    minimal ``DISTRIBUTED_STATUS`` row supplies its canonical task ID.  Task
    snapshots then supply coordinates when available.  No start is inferred
    from a completion event alone.
    """
    replay_path = observer / 'navigation_action_replay.json'
    if not replay_path.is_file():
        return []
    try:
        replay = json.loads(replay_path.read_text(encoding='utf-8'))
    except (OSError, TypeError, ValueError, json.JSONDecodeError):
        return []
    if (not replay.get('complete') or
            replay.get('missing_required_status_topics')):
        return []
    clock_samples = _load_clock_samples(observer)
    if not clock_samples:
        return []
    status_by_robot = {'robot1': [], 'robot2': []}
    for event in events or ():
        if (event.get('event_type') == 'DISTRIBUTED_STATUS' and
                event.get('robot_id') in status_by_robot and
                event.get('local_nav_goal_active') and
                event.get('active_canonical_task_id')):
            status_by_robot[event['robot_id']].append(event)
    for robot in status_by_robot:
        status_by_robot[robot].sort(key=lambda item: _event_time(item) or 0.0)

    def task_at(robot, canonical_id, stamp):
        choices = [item for owner in ('robot1', 'robot2')
                   for item in task_records.get(owner, ())
                   if item.local_frontier_id == canonical_id]
        if not choices:
            return None
        return min(choices, key=lambda item: abs(item.stamp_s - stamp))

    def status_at(robot, stamp):
        choices = [item for item in status_by_robot[robot]
                   if abs((_event_time(item) or 0.0) - stamp) <=
                   NATIVE_GOAL_STATUS_JOIN_TOLERANCE_S]
        if not choices:
            return None
        return min(choices, key=lambda item: abs(
            (_event_time(item) or 0.0) - stamp))

    native = []
    starts = {}
    terminal_statuses = {'SUCCEEDED', 'ABORTED', 'CANCELED', 'CANCELLED'}
    for transition in replay.get('status_transitions') or ():
        if (transition.get('action') != 'NAVIGATE_TO_POSE' or
                transition.get('status_name') not in
                {'EXECUTING', *terminal_statuses}):
            continue
        robot = transition.get('robot_id')
        if robot not in status_by_robot:
            continue
        stamp = _simulation_time_for_bag_timestamp(
            transition.get('bag_timestamp_ns'), clock_samples)
        if stamp is None:
            continue
        uuid = str(transition.get('goal_uuid', ''))
        if transition.get('status_name') == 'EXECUTING':
            status = status_at(robot, stamp)
            canonical_id = (str(status.get('active_canonical_task_id'))
                            if status else '')
            if not canonical_id:
                continue
            task = task_at(robot, canonical_id, stamp)
            event = GoalEvent(
                stamp, robot, 'start', canonical_id,
                task.physical_signature if task else '',
                ((task.approach or task.centroid) if task else None))
            starts[uuid] = event
            native.append(event)
        elif uuid in starts:
            start = starts[uuid]
            native.append(GoalEvent(
                stamp, robot, 'stop', start.canonical_task_id,
                start.physical_signature, None))
    return native


def active_goals_at(data: OverlayData, stamp_s: float):
    result = {}
    for robot in ('robot1', 'robot2'):
        for event in data.goal_events.get(robot, ()):
            if event.stamp_s > stamp_s:
                break
            if event.action == 'start':
                result[robot] = event
            elif event.action == 'stop':
                current = result.get(robot)
                if current is None:
                    continue
                signature_matches = (not event.physical_signature or
                                      current.physical_signature ==
                                      event.physical_signature)
                task_matches = (not event.canonical_task_id or
                                current.canonical_task_id ==
                                event.canonical_task_id)
                # Legacy terminal records may have neither identifier.  A
                # populated identifier must match the active commitment so a
                # delayed completion cannot clear a newer native goal.
                if signature_matches and task_matches:
                    result.pop(robot, None)
    return result


def planned_path_at(data: OverlayData, robot: str, canonical_id: str, stamp_s: float):
    batches = data.bid_batches.get(robot, ())
    selected = None
    for batch in batches:
        if batch.stamp_s > stamp_s:
            break
        if canonical_id in batch.paths:
            selected = batch.paths[canonical_id]
    return selected


def _safe_pair(value: Any) -> Optional[tuple[float, float]]:
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        try:
            pair = (float(value[0]), float(value[1]))
            return pair if all(math.isfinite(x) for x in pair) else None
        except (TypeError, ValueError):
            return None
    return None


def _safe_bounds(value: Any):
    if isinstance(value, (list, tuple)) and len(value) >= 4:
        try:
            values = tuple(float(value[index]) for index in range(4))
            return values if all(math.isfinite(x) for x in values) else None
        except (TypeError, ValueError):
            return None
    return None


def _safe_geometry(value: Any):
    if not isinstance(value, list):
        return ()
    points = tuple(pair for item in value if (pair := _safe_pair(item)) is not None)
    return points if len(points) >= 2 else ()


def _safe_cells(value: Any):
    if not isinstance(value, list):
        return ()
    cells = []
    for item in value:
        if not isinstance(item, (list, tuple)) or len(item) < 2:
            continue
        try:
            x, y = int(item[0]), int(item[1])
        except (TypeError, ValueError):
            continue
        cells.append((x, y))
    return tuple(cells)


def _safe_origin(value: Any):
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        return (0.0, 0.0, 0.0)
    try:
        origin = tuple(float(value[index]) for index in range(3))
        return origin if all(math.isfinite(item) for item in origin) else (0.0, 0.0, 0.0)
    except (TypeError, ValueError):
        return (0.0, 0.0, 0.0)


def _centroid_from_cells(cells, resolution, origin):
    if not cells or resolution <= 0.0:
        return None
    ox, oy, oyaw = origin
    cos_yaw, sin_yaw = math.cos(oyaw), math.sin(oyaw)
    gx = sum((cell_x + 0.5) * resolution for cell_x, _ in cells) / len(cells)
    gy = sum((cell_y + 0.5) * resolution for _, cell_y in cells) / len(cells)
    return (ox + cos_yaw * gx - sin_yaw * gy,
            oy + sin_yaw * gx + cos_yaw * gy)


def load_frontier_region_records(observer: Path):
    path = observer / 'frontier_regions.jsonl'
    records = {'robot1': [], 'robot2': []}
    if not path.is_file():
        return records
    with path.open(encoding='utf-8', errors='replace') as stream:
        for line in stream:
            try:
                payload = json.loads(line)
                robot = payload.get('capture_robot')
                stamp = float(payload.get('capture_elapsed_s'))
                resolution = float(payload.get('resolution'))
                if robot not in records or not math.isfinite(stamp) or resolution <= 0.0:
                    continue
                regions = {}
                origin = _safe_origin(payload.get('origin'))
                for region in payload.get('regions') or []:
                    if not isinstance(region, dict):
                        continue
                    key = region.get('physical_id', region.get('id'))
                    cells = _safe_cells(region.get('cells'))
                    centroid = _safe_pair(region.get('centroid'))
                    if centroid is None:
                        centroid = _centroid_from_cells(cells, resolution, origin)
                    if key is not None and centroid is not None:
                        regions[str(key)] = FrontierRegion(
                            stamp, robot, str(key), centroid, cells,
                            _safe_bounds(region.get('bbox')), origin, resolution)
                records[robot].append(FrontierRegionSnapshot(
                    stamp, robot, tuple(regions.values()), resolution, origin))
            except (TypeError, ValueError, json.JSONDecodeError):
                continue
    for robot in records:
        records[robot].sort(key=lambda item: item.stamp_s)
    return records


def frontier_region_at(records, robot: str, stamp: float, frontier_id: str):
    items = records.get(robot, ())
    index = bisect.bisect_right([item.stamp_s for item in items], stamp + 1e-6) - 1
    if index < 0:
        return (), (0.0, 0.0, 0.0), None
    snapshot = items[index]
    region = next((item for item in snapshot.regions
                   if item.frontier_id == str(frontier_id)), None)
    return (region.cells if region else ()), snapshot.origin, snapshot.resolution


def frontier_snapshot_at(records, robot: str, stamp: float):
    items = records.get(robot, ())
    index = bisect.bisect_right([item.stamp_s for item in items], stamp + 1e-6) - 1
    return items[index] if index >= 0 else None


def frontier_ids_seen_at(records, robot: str, stamp: float):
    cutoff = stamp + 1e-6
    return frozenset(region.frontier_id
                     for item in records.get(robot, ())
                     if item.stamp_s <= cutoff
                     for region in item.regions)


def frontier_counts_at(records, robot: str, stamp: float):
    snapshot = frontier_snapshot_at(records, robot, stamp)
    return (len(snapshot.regions) if snapshot is not None else 0,
            len(frontier_ids_seen_at(records, robot, stamp)))


def _event_time(event: dict[str, Any]) -> Optional[float]:
    value = event.get('elapsed_s')
    try:
        value = float(value)
        return value if math.isfinite(value) else None
    except (TypeError, ValueError):
        return None


def load_event_records(observer: Path) -> list[dict[str, Any]]:
    path = observer / 'events.jsonl'
    if not path.is_file():
        path = observer / 'goal_decision_ledger.jsonl'
    records = []
    if not path.is_file():
        return records
    with path.open(encoding='utf-8', errors='replace') as stream:
        for line in stream:
            try:
                event = json.loads(line)
                stamp = _event_time(event)
                if stamp is not None:
                    records.append(event)
            except json.JSONDecodeError:
                continue
    return sorted(records, key=lambda item: (_event_time(item) or 0.0,
                                             int(item.get('event_sequence', 0) or 0)))


def infer_cooperative_start_time(events: list[dict[str, Any]], fallback: float) -> float:
    """Return the simulation time at which the run enters cooperative mode.

    The shared-pose/frontend handoff can happen earlier than the experiment's
    cooperative release.  The overlay phase is about the latter, so use the
    common START_RELEASE event when it is recorded.  A ready-state event is a
    conservative fallback for older artifacts without START_RELEASE.
    """
    release_times = [stamp for event in events
                     if event.get('event_type') == 'START_RELEASE'
                     for stamp in [_event_time(event)] if stamp is not None]
    if release_times:
        return min(release_times)
    ready_times = [stamp for event in events
                   if event.get('event_type') == 'COOPERATIVE_START_STATE_READY'
                   for stamp in [_event_time(event)] if stamp is not None]
    return max(ready_times) if ready_times else fallback


def _load_distance_travelled(observer: Path):
    result = {}
    for robot in ('robot1', 'robot2'):
        path = observer / f'{robot}_timeseries.csv'
        latest = None
        if path.is_file():
            for row in load_csv(path):
                value = numeric(row, 'distance_travelled_m')
                if value is not None:
                    latest = value
        result[robot] = latest
    return result


def _load_clock_samples(observer: Path):
    path = observer / 'passive_rosbag_export.jsonl'
    if not path.is_file():
        return []
    samples = []
    with path.open(encoding='utf-8', errors='replace') as stream:
        for line in stream:
            try:
                row = json.loads(line)
                if row.get('topic') != '/clock':
                    continue
                samples.append((int(row['bag_time_ns']), float(row['clock_s'])))
            except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                continue
    return samples


def _load_productivity_data(observer: Path, events: list[dict[str, Any]]):
    result = {'robot1': None, 'robot2': None}
    segments = {'robot1': (), 'robot2': ()}
    canonical_path = observer.parent.parent / 'offline_metrics.json'
    if canonical_path.is_file():
        try:
            canonical = json.loads(canonical_path.read_text(encoding='utf-8'))
            timing = canonical.get('timing') or {}
            found = False
            for robot in ('robot1', 'robot2'):
                values = timing.get(robot) or {}
                fraction = values.get('physical_productivity_percent')
                if fraction is not None:
                    fraction = float(fraction)
                    if math.isfinite(fraction):
                        result[robot] = fraction
                        found = True
                raw_segments = values.get('physical_segments')
                if raw_segments is None:
                    raw_segments = values.get('segments')
                if isinstance(raw_segments, list):
                    segments[robot] = tuple(
                        item for item in raw_segments if isinstance(item, dict))
                    found = found or bool(segments[robot])
            if found:
                return result, segments
        except (OSError, TypeError, ValueError, json.JSONDecodeError):
            pass
    summary_path = observer.parent.parent / 'fast_trial_summary.json'
    if not summary_path.is_file():
        return result, segments
    try:
        summary = json.loads(summary_path.read_text(encoding='utf-8'))
        readiness = summary.get('nav2_readiness_telemetry') or {}
        shared = readiness.get('shared_activation_reached_at_sim_s') or {}
        ready_s = max((float(value) for value in shared.values()), default=0.0)
        end_s = float(summary.get('simulation_horizon_s') or
                      summary.get('simulation_end_time_s') or 0.0)
        replay_path = observer / 'navigation_action_replay.json'
        replay = json.loads(replay_path.read_text(encoding='utf-8')) \
            if replay_path.is_file() else {}
        timing_events = merge_native_timing_events(
            derive_minimal_assignment_events(events), replay,
            _load_clock_samples(observer))
        timing = analyze_event_records(
            timing_events, ready_sim=ready_s, end_sim=end_s,
            robots=('robot1', 'robot2'))
        odometry = {}
        for robot in ('robot1', 'robot2'):
            path = observer / 'forensic' / f'{robot}_odom.csv'
            odometry[robot] = load_csv(path) if path.is_file() else []
        metrics = analyze_physical_activity(
            timing, odometry, ready_sim=ready_s, end_sim=end_s,
            robots=('robot1', 'robot2'))
        for robot, values in metrics.get('robots', {}).items():
            fraction = values.get('physical_productivity_percent')
            if fraction is not None and math.isfinite(float(fraction)):
                result[robot] = float(fraction)
            segments[robot] = tuple(values.get('segments') or ())
    except (OSError, TypeError, ValueError, json.JSONDecodeError):
        pass
    return result, segments


def _load_handoff_error(observer: Path):
    path = observer / 'forensic/physical_gt_evaluation.json'
    if not path.is_file():
        return None, None
    try:
        error = json.loads(path.read_text(encoding='utf-8')).get('error_transform') or {}
        tx, ty = float(error['tx']), float(error['ty'])
        yaw_deg = float(error['yaw_deg'])
        if not all(math.isfinite(value) for value in (tx, ty, yaw_deg)):
            return None, None
        return math.hypot(tx, ty), yaw_deg
    except (OSError, TypeError, ValueError, KeyError, json.JSONDecodeError):
        return None, None


def issue_counts_at(data: OverlayData, stamp_s: float):
    errors = warnings = 0
    for event in data.event_records:
        event_stamp = _event_time(event)
        if event_stamp is None or event_stamp > stamp_s:
            break
        severity = str(event.get('severity', '')).upper()
        if severity in {'ERROR', 'FATAL'}:
            errors += 1
        elif severity in {'WARN', 'WARNING'}:
            warnings += 1
    return errors, warnings


def load_overlay_data(observer: Path) -> OverlayData:
    events = load_event_records(observer)
    frontier_regions = load_frontier_region_records(observer)
    candidates = {'robot1': [], 'robot2': []}
    tasks = {'robot1': [], 'robot2': []}
    goals = {'robot1': [], 'robot2': []}
    bids = {'robot1': [], 'robot2': []}
    raw_geometry_records = 0
    for event in events:
        stamp = _event_time(event)
        robot = event.get('robot_id')
        if robot not in candidates or stamp is None:
            continue
        kind = event.get('event_type', '')
        if kind == 'CANDIDATE_BATCH_RECEIVED':
            batch = []
            for item in event.get('candidates') or []:
                centroid = _safe_pair(item.get('centroid'))
                if centroid is None:
                    continue
                geometry = _safe_geometry(item.get('geometry') or
                                          item.get('frontier_geometry') or
                                          item.get('boundary_points'))
                frontier_id = str(item.get('frontier_id', ''))
                cells, origin, resolution = frontier_region_at(
                    frontier_regions, robot, stamp, frontier_id)
                raw_geometry_records += int(bool(geometry or cells))
                batch.append(Candidate(
                    stamp, robot, frontier_id,
                    str(item.get('physical_signature', '')),
                    centroid, _safe_bounds(item.get('bounds')),
                    _safe_pair(item.get('approach')),
                    numeric(item, 'score'), numeric(item, 'visible_reveal_gain'),
                    numeric(item, 'path_length_m') or numeric(item, 'local_path_length_m'),
                    numeric(item, 'path_heading_cost_rad'), geometry, cells,
                    origin, resolution))
            candidates[robot].append((stamp, tuple(batch)))
        elif kind == 'DISTRIBUTED_TASK_SNAPSHOT':
            for item in event.get('tasks') or []:
                centroid = _safe_pair(item.get('centroid'))
                if centroid is None:
                    continue
                tasks[robot].append(TaskRecord(
                    stamp, robot, str(item.get('physical_signature', '')),
                    str(item.get('local_frontier_id', '')),
                    centroid, _safe_bounds(item.get('bounds')),
                    _safe_pair(item.get('approach'))))
        elif kind == 'DISTRIBUTED_BID_ARRAY':
            path_map = {}
            for item in event.get('bids') or []:
                points = tuple(pair for raw in item.get('path_samples') or []
                               if (pair := _safe_pair(raw)) is not None)
                if len(points) >= 2 and item.get('canonical_task_id'):
                    path_map[str(item['canonical_task_id'])] = points
            bids[robot].append(BidPathBatch(stamp, robot, path_map))
        elif kind == 'NAV_GOAL_SENT':
            goal = _safe_pair(event.get('approach') or event.get('goal_xy'))
            goals[robot].append(GoalEvent(
                stamp, robot, 'start', str(event.get('canonical_task_id', '')),
                str(event.get('physical_task_signature', '')), goal))
        elif kind in {'NAVIGATION_SUCCEEDED', 'NAVIGATION_FAILED',
                      'NAVIGATION_CANCELED', 'NAVIGATION_CANCELLED',
                      'NAVIGATION_TIMEOUT', 'NAV_GOAL_CANCELLED'}:
            goals[robot].append(GoalEvent(
                stamp, robot, 'stop', str(event.get('canonical_task_id', '')),
                str(event.get('physical_task_signature', '')), None))
    for event in _native_navigation_goal_events(observer, events, tasks):
        robot = event.robot
        # Keep the legacy event path authoritative where it already recorded
        # the same dispatch; native evidence fills the minimal-allocator gap.
        duplicate = any(
            existing.action == event.action and
            existing.canonical_task_id == event.canonical_task_id and
            abs(existing.stamp_s - event.stamp_s) <= 1.0e-3
            for existing in goals[robot])
        if not duplicate:
            goals[robot].append(event)
    for robot in candidates:
        candidates[robot].sort(key=lambda item: item[0])
        tasks[robot].sort(key=lambda item: item.stamp_s)
        goals[robot].sort(key=lambda item: (
            item.stamp_s, 0 if item.action == 'start' else 1))
        bids[robot].sort(key=lambda item: item.stamp_s)
    handoff_error_m, handoff_error_yaw_deg = _load_handoff_error(observer)
    productive_percent, productivity_segments = _load_productivity_data(observer, events)
    return OverlayData(
        {robot: tuple(value) for robot, value in candidates.items()},
        {robot: tuple(value) for robot, value in tasks.items()},
        {robot: tuple(value) for robot, value in goals.items()},
        {robot: tuple(value) for robot, value in bids.items()},
        raw_geometry_records,
        tuple(events), handoff_error_m, handoff_error_yaw_deg,
        _load_distance_travelled(observer), productive_percent,
        productivity_segments,
        {robot: tuple(value) for robot, value in frontier_regions.items()})


@lru_cache(maxsize=None)
def find_font(bold=False, size=20):
    names = ([
        '/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf',
        '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
    ] if bold else [
        '/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf',
        '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
    ])
    for name in names:
        if Path(name).is_file():
            return ImageFont.truetype(name, size=size)
    return ImageFont.load_default()


def _right_aligned_item(value, y, font, fill, right_x):
    value = str(value)
    bbox = font.getbbox(value)
    width = bbox[2] - bbox[0]
    return value, (right_x - width, y), font, fill


def add_text(canvas, items):
    image = Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(image)
    for text, xy, font, fill in items:
        draw.text(xy, text, font=font, fill=fill)
    return cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)


def _status_value(row, key, default='-'):
    if row is None:
        return default
    value = row.row.get(key, '')
    return value if value not in ('', None) else default


def _frontier_region_polygons(region: FrontierRegion, viewport, map_rect):
    """Return one pixel polygon per recorded cell, or a bbox fallback."""
    if region.cells and region.grid_resolution:
        ox, oy, oyaw = region.grid_origin
        cos_yaw, sin_yaw = math.cos(oyaw), math.sin(oyaw)
        polygons = []
        for cell_x, cell_y in region.cells:
            world = []
            for dx, dy in ((0.0, 0.0), (1.0, 0.0),
                           (1.0, 1.0), (0.0, 1.0)):
                gx = (cell_x + dx) * region.grid_resolution
                gy = (cell_y + dy) * region.grid_resolution
                world.append((ox + cos_yaw * gx - sin_yaw * gy,
                              oy + sin_yaw * gx + cos_yaw * gy))
            polygons.append(_local_points(world, viewport, map_rect))
        return polygons
    if region.bounds:
        xmin, ymin, xmax, ymax = region.bounds
        xmin, xmax = min(xmin, xmax), max(xmin, xmax)
        ymin, ymax = min(ymin, ymax), max(ymin, ymax)
        return [_local_points(((xmin, ymin), (xmax, ymin),
                               (xmax, ymax), (xmin, ymax)),
                              viewport, map_rect)]
    return []


def _build_frontier_layer(frontier_regions, viewport, map_rect):
    """Rasterize one frontier snapshot into a reusable translucent layer."""
    width, height = map_rect[2], map_rect[3]
    colors = np.zeros((height, width, 3), dtype=np.uint8)
    coverage = np.zeros((height, width), dtype=np.uint8)
    for region in frontier_regions:
        polygons = _frontier_region_polygons(region, viewport, map_rect)
        if not polygons:
            continue
        region_mask = np.zeros(coverage.shape, dtype=np.uint8)
        cv2.fillPoly(region_mask, polygons, 255)
        color = R1_COLOR if region.robot == 'robot1' else R2_COLOR
        colors[region_mask > 0] = color
        coverage[region_mask > 0] = 255
        contours, _ = cv2.findContours(
            region_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(colors, contours, -1, color, 1, cv2.LINE_AA)
        cv2.drawContours(coverage, contours, -1, 255, 1, cv2.LINE_AA)
    return FrontierLayer(colors, coverage)


def _cached_frontier_layer(frontier_regions, cache_key, viewport, map_rect, cache):
    """Keep only the current frontier layer so cache memory stays bounded."""
    if cache is None:
        return _build_frontier_layer(frontier_regions, viewport, map_rect)
    layer = cache.get(cache_key)
    if layer is None:
        cache.clear()
        layer = _build_frontier_layer(frontier_regions, viewport, map_rect)
        cache[cache_key] = layer
    return layer


def _blend_frontier_layer(view, layer: FrontierLayer):
    if layer is None:
        return
    blended = cv2.addWeighted(layer.colors, 0.20, view, 0.80, 0.0)
    view[layer.coverage > 0] = blended[layer.coverage > 0]


def draw_frontier_overlays(canvas, candidates, stamp_s, viewport, map_rect,
                           show_frontiers=True, show_candidates=True,
                           selected_signatures=frozenset(),
                           selected_frontier_ids=frozenset(),
                           frontier_regions=(), frontier_layer=None):
    if not candidates or (not show_frontiers and not show_candidates):
        if not frontier_regions or not show_frontiers:
            return 0
    left, top, width, height, _ = map_rect
    view = canvas[top:top + height, left:left + width]
    marker_count = 0
    if show_frontiers:
        if frontier_regions:
            frontier_layer = frontier_layer or _build_frontier_layer(
                frontier_regions, viewport, map_rect)
            _blend_frontier_layer(view, frontier_layer)
        # Put one centroid marker per region on top of the translucent layer.
        for region in frontier_regions:
            px, py = map_to_pixel(*region.centroid, viewport, map_rect)
            point = (px - left, py - top)
            color = R1_COLOR if region.robot == 'robot1' else R2_COLOR
            selected = (region.robot, region.frontier_id) in selected_frontier_ids
            cv2.drawMarker(view, point, color, cv2.MARKER_DIAMOND,
                           9 if selected else 7, 2 if selected else 1,
                           cv2.LINE_AA)
            marker_count += 1
    if show_candidates:
        for item in candidates:
            selected = (
                (item.robot, item.physical_signature) in selected_signatures or
                (item.robot, item.frontier_id) in selected_frontier_ids
            )
            color = R1_COLOR if item.robot == 'robot1' else R2_COLOR
            px, py = map_to_pixel(*(item.approach or item.centroid), viewport, map_rect)
            px, py = px - left, py - top
            cv2.circle(view, (px, py), 5 if selected else 4, (248, 248, 248),
                       -1, cv2.LINE_AA)
            cv2.circle(view, (px, py), 4 if selected else 3, color, -1, cv2.LINE_AA)
    return marker_count


def draw_goal(canvas, goal_xy, robot, viewport, map_rect):
    if goal_xy is None:
        return
    left, top = map_rect[0], map_rect[1]
    view = canvas[top:top + map_rect[3], left:left + map_rect[2]]
    px, py = map_to_pixel(*goal_xy, viewport, map_rect)
    point = (px - left, py - top)
    color = R1_COLOR if robot == 'robot1' else R2_COLOR
    cv2.circle(view, point, 11, GOAL_WHITE, 2, cv2.LINE_AA)
    cv2.circle(view, point, 7, color, 2, cv2.LINE_AA)
    cv2.line(view, (point[0] - 12, point[1]), (point[0] + 12, point[1]), color, 1,
              cv2.LINE_AA)
    cv2.line(view, (point[0], point[1] - 12), (point[0], point[1] + 12), color, 1,
              cv2.LINE_AA)


def draw_robot(canvas, pose: Optional[PoseSample], label, color, viewport, map_rect):
    if pose is None:
        return
    left, top = map_rect[0], map_rect[1]
    view = canvas[top:top + map_rect[3], left:left + map_rect[2]]
    px, py = map_to_pixel(pose.x, pose.y, viewport, map_rect)
    point = (px - left, py - top)
    arrow = max(15, int(round(ARROW_LENGTH_M * map_rect[4])))
    end = (point[0] + int(round(math.cos(pose.yaw) * arrow)),
           point[1] - int(round(math.sin(pose.yaw) * arrow)))
    cv2.arrowedLine(view, point, end, (255, 255, 255), 5, cv2.LINE_AA, tipLength=.28)
    cv2.arrowedLine(view, point, end, color, 3, cv2.LINE_AA, tipLength=.28)
    cv2.circle(view, point, 9, (255, 255, 255), -1, cv2.LINE_AA)
    cv2.circle(view, point, 7, color, -1, cv2.LINE_AA)


def _short(value: str):
    return value[:8] if value else '-'


def _ascii_display(value: str):
    return str(value).replace('\u2014', '-').replace('\u2013', '-').replace('\u00b7', '|')


def _bounded_display(value: str, maximum: int):
    value = _ascii_display(value)
    return value if len(value) <= maximum else value[:maximum - 3] + '...'


def render_frame(stamp_s, snapshots, snapshot_cache, segments, viewport, map_rect,
                 width, height, title, handoff_s, policy, overlay_data=None,
                 show_frontiers=True, show_goals=True, show_candidates=True,
                 show_planned_path=True, frontier_layer_cache=None):
    canvas = np.full((height, width, 3), (248, 249, 251), dtype=np.uint8)
    map_index = bisect.bisect_right([item.stamp_s for item in snapshots], stamp_s) - 1
    active_snapshot = snapshots[map_index] if map_index >= 0 else None
    left, top, map_width, map_height, _ = map_rect
    if active_snapshot is not None:
        if active_snapshot.path not in snapshot_cache:
            snapshot_cache[active_snapshot.path] = render_snapshot(
                active_snapshot, viewport, map_rect)
        canvas[top:top + map_height, left:left + map_width] = snapshot_cache[active_snapshot.path]
    cv2.rectangle(canvas, (left, top), (left + map_width, top + map_height),
                  (102, 108, 116), 1, cv2.LINE_AA)

    data = overlay_data or OverlayData({}, {}, {}, {}, 0)
    all_candidates = []
    all_frontier_regions = []
    frontier_snapshot_keys = []
    current_frontier_counts = {}
    unique_frontier_counts = {}
    for robot in ('robot1', 'robot2'):
        all_candidates.extend(_candidate_at(data, robot, stamp_s))
        snapshot = frontier_snapshot_at(data.frontier_regions, robot, stamp_s)
        current_frontier_counts[robot], unique_frontier_counts[robot] = \
            frontier_counts_at(data.frontier_regions, robot, stamp_s)
        frontier_snapshot_keys.append(
            None if snapshot is None else (robot, id(snapshot)))
        if snapshot is not None:
            all_frontier_regions.extend(snapshot.regions)
    goals = active_goals_at(data, stamp_s)
    selected_signatures = frozenset(
        (robot, event.physical_signature)
        for robot, event in goals.items()
        if event.physical_signature
    )
    # The task snapshot preserves the generator's stable local frontier ID,
    # which is the only selected-candidate join key present in the saved
    # candidate event schema.
    selected_frontier_ids = frozenset(
        (robot, item.local_frontier_id)
        for robot, event in goals.items()
        if event.physical_signature
        for item in sorted(
            (item for item in data.task_records.get(robot, ())
             if item.physical_signature == event.physical_signature),
            key=lambda item: abs(item.stamp_s - event.stamp_s),
        )[:1]
    )
    frontier_layer = None
    if show_frontiers and all_frontier_regions:
        frontier_layer = _cached_frontier_layer(
            tuple(all_frontier_regions), tuple(frontier_snapshot_keys),
            viewport, map_rect, frontier_layer_cache)
    draw_frontier_overlays(
        canvas, all_candidates, stamp_s, viewport, map_rect,
        show_frontiers, show_candidates, selected_signatures,
        selected_frontier_ids, all_frontier_regions, frontier_layer)

    poses = {robot: pose_at_segments(segments[robot], stamp_s)
             for robot in ('robot1', 'robot2')}
    if show_planned_path:
        for robot in ('robot1', 'robot2'):
            event = goals.get(robot)
            if event:
                path = planned_path_at(data, robot, event.canonical_task_id, stamp_s)
                if path:
                    points = _local_points(path, viewport, map_rect)
                    view = canvas[top:top + map_height, left:left + map_width]
                    draw_dashed(view, points, R1_COLOR if robot == 'robot1' else R2_COLOR,
                                width=2, dash=12, gap=8)
    for robot in ('robot1', 'robot2'):
        draw_segments(canvas, segments[robot], stamp_s, viewport, map_rect,
                      R1_TRAJECTORY if robot == 'robot1' else R2_TRAJECTORY)
    if show_goals:
        for robot, event in goals.items():
            draw_goal(canvas, _task_for_goal(data, event), robot, viewport, map_rect)
    draw_robot(canvas, poses['robot1'], 'R1', R1_COLOR, viewport, map_rect)
    draw_robot(canvas, poses['robot2'], 'R2', R2_COLOR, viewport, map_rect)

    sidebar_x = 1550
    cv2.line(canvas, (1518, 126), (1518, 1055), (220, 224, 229), 1)
    fonts = {'title': find_font(True, 32), 'subtitle': find_font(False, 18),
             'section': find_font(True, 16), 'body': find_font(False, 19),
             'small': find_font(False, 15), 'maplabel': find_font(True, 17)}
    errors, warnings = issue_counts_at(data, stamp_s)
    traffic = _traffic_labels(data, stamp_s)
    phase = 'COOPERATIVE' if stamp_s >= handoff_s else 'PRE-HANDOFF'
    phase_detail = (f'since {handoff_s:,.2f} s' if phase == 'COOPERATIVE'
                    else f'starts at {handoff_s:,.2f} s')
    handoff_error = ('handoff error '
                     f'{data.handoff_error_m * 100.0:.2f} cm | '
                     f'{data.handoff_error_yaw_deg:.3f} deg'
                     if data.handoff_error_m is not None and
                     data.handoff_error_yaw_deg is not None else
                     'handoff err -')
    sidebar_right = width - 40
    text = [(_ascii_display(title), (70, 42), fonts['title'], TEXT_DARK),
            ('Two-robot cooperative exploration', (72, 82),
             fonts['subtitle'], TEXT_MUTED),
            ('SIM TIME', (sidebar_x, 140), fonts['section'], TEXT_MUTED),
            _right_aligned_item(f'{stamp_s:,.2f} s', 162, fonts['body'],
                                TEXT_DARK, sidebar_right),
            ('POLICY', (sidebar_x, 198), fonts['section'], TEXT_MUTED),
            _right_aligned_item(_bounded_display(policy, 23), 220,
                                fonts['body'], TEXT_DARK, sidebar_right),
            ('MAP SNAPSHOT', (sidebar_x, 258), fonts['section'], TEXT_MUTED),
            _right_aligned_item(
                f'{active_snapshot.stamp_s:,.2f} s' if active_snapshot else '-',
                280, fonts['body'], TEXT_DARK, sidebar_right),
            ('PHASE', (sidebar_x, 318), fonts['section'], TEXT_MUTED),
            _right_aligned_item(phase, 342, fonts['body'], TEXT_DARK,
                                sidebar_right),
            _right_aligned_item(phase_detail, 374, fonts['small'], TEXT_MUTED,
                                sidebar_right),
            ('handoff error', (sidebar_x, 396), fonts['small'], TEXT_MUTED),
            _right_aligned_item(handoff_error.removeprefix('handoff error '),
                                396, fonts['small'], TEXT_MUTED, sidebar_right)]
    goals = active_goals_at(data, stamp_s)
    for index, robot in enumerate(('robot1', 'robot2')):
        y = 420 + index * 285
        pose = poses[robot]
        event = goals.get(robot)
        color_rgb = (232, 125, 48) if robot == 'robot1' else (42, 104, 210)
        candidates, candidate_age = _candidate_status(data, robot, stamp_s)
        goal_xy = _task_for_goal(data, event) if event else None
        candidate_age_text = '-' if candidate_age is None else f'{candidate_age:,.2f} s'
        if candidate_age is not None and candidate_age > MAX_TIME_GAP_S:
            candidate_age_text += ' STALE'
        distance = _pose_row_metric(pose, 'distance_travelled_m')
        linear_speed = _display_speed(pose, 'linear_speed_mps')
        angular_speed = _display_speed(pose, 'angular_speed_radps')
        productive = _productivity_at(data, robot, stamp_s)
        productivity_text = (f'{productive:.1f}%'
                             if productive is not None else 'N/R')
        text.extend([
            ('R1' if robot == 'robot1' else 'R2', (sidebar_x, y), fonts['section'], color_rgb),
            ('position', (sidebar_x, y + 28), fonts['small'], TEXT_DARK),
            _right_aligned_item(
                f'{pose.x:.2f}, {pose.y:.2f}' if pose else '-',
                y + 28, fonts['small'], TEXT_DARK, sidebar_right),
            ('heading', (sidebar_x, y + 45), fonts['small'], TEXT_DARK),
            _right_aligned_item(
                f'{math.degrees(pose.yaw):.1f}°' if pose else '-',
                y + 45, fonts['small'], TEXT_DARK, sidebar_right),
            ('goal active', (sidebar_x, y + 66), fonts['small'], TEXT_DARK),
            _right_aligned_item(
                _short(event.canonical_task_id) if event else 'idle',
                y + 66, fonts['small'], TEXT_DARK, sidebar_right),
            ('goal', (sidebar_x, y + 83), fonts['small'], TEXT_DARK),
            _right_aligned_item(
                f'{goal_xy[0]:.2f}, {goal_xy[1]:.2f}' if goal_xy else '-',
                y + 83, fonts['small'], TEXT_DARK, sidebar_right),
            ('linear speed', (sidebar_x, y + 103), fonts['small'], TEXT_DARK),
            _right_aligned_item(
                f'{linear_speed * 100.0:.1f} cm/s' if linear_speed is not None else '-',
                y + 103, fonts['small'], TEXT_DARK, sidebar_right),
            ('turn rate', (sidebar_x, y + 123), fonts['small'], TEXT_DARK),
            _right_aligned_item(
                f'{math.degrees(angular_speed or 0.0):.1f} deg/s'
                if angular_speed is not None else '-',
                y + 123, fonts['small'], TEXT_DARK, sidebar_right),
            ('traveled', (sidebar_x, y + 143), fonts['small'], TEXT_DARK),
            _right_aligned_item(
                f'{distance:.2f} m' if distance is not None else '-',
                y + 143, fonts['small'], TEXT_DARK, sidebar_right),
            ('physical productivity', (sidebar_x, y + 163), fonts['small'], TEXT_DARK),
            _right_aligned_item(productivity_text, y + 163, fonts['small'],
                                TEXT_DARK, sidebar_right),
            ('frontier regions', (sidebar_x, y + 183), fonts['small'], TEXT_DARK),
            _right_aligned_item(current_frontier_counts[robot], y + 183,
                                fonts['small'], TEXT_DARK, sidebar_right),
            ('unique IDs seen', (sidebar_x, y + 203), fonts['small'], TEXT_DARK),
            _right_aligned_item(unique_frontier_counts[robot], y + 203,
                                fonts['small'], TEXT_DARK, sidebar_right),
            ('traffic', (sidebar_x, y + 223), fonts['small'], TEXT_DARK),
            _right_aligned_item(traffic[robot], y + 223, fonts['small'],
                                TEXT_DARK, sidebar_right),
            ('reachable candidates', (sidebar_x, y + 243), fonts['small'], TEXT_MUTED),
            _right_aligned_item(len(candidates), y + 243, fonts['small'],
                                TEXT_MUTED, sidebar_right),
            ('batch age', (sidebar_x, y + 263), fonts['small'], TEXT_MUTED),
            _right_aligned_item(candidate_age_text, y + 263, fonts['small'],
                                TEXT_MUTED, sidebar_right),
        ])
    text.extend([
        ('ISSUES', (sidebar_x, 1000), fonts['section'], TEXT_MUTED),
        ('errors', (sidebar_x, 1024), fonts['small'], TEXT_DARK),
        _right_aligned_item(errors, 1024, fonts['small'], TEXT_DARK,
                            sidebar_right),
        ('warnings', (sidebar_x, 1047), fonts['small'], TEXT_MUTED),
        _right_aligned_item(warnings, 1047, fonts['small'], TEXT_MUTED,
                            sidebar_right),
    ])
    for robot in ('robot1', 'robot2'):
        pose = poses[robot]
        if pose is None:
            continue
        px, py = map_to_pixel(pose.x, pose.y, viewport, map_rect)
        label_x = min(max(px + 13, left + 4), left + map_width - 33)
        label_y = min(max(py - 24, top + 2), top + map_height - 22)
        text.append(('R1' if robot == 'robot1' else 'R2', (label_x, label_y),
                     fonts['maplabel'], (232, 125, 48) if robot == 'robot1' else (42, 104, 210)))
    return add_text(canvas, text)


def _source_end(poses, snapshots):
    return max(max((sample.stamp_s for rows in poses.values() for sample in rows
                    if sample is not None), default=0.0), snapshots[-1].stamp_s)


def geometry_validation(viewport, map_rect, segments, snapshots, duration_s, handoff_s,
                        overlay_data):
    xmin, xmax, ymin, ymax = viewport
    samples = [sample for robot in segments.values() for segment in robot for sample in segment]
    outside = [sample for sample in samples
               if not (xmin <= sample.x <= xmax and ymin <= sample.y <= ymax)]
    steps = [math.hypot(b.x - a.x, b.y - a.y)
             for robot in segments.values() for segment in robot
             for a, b in zip(segment, segment[1:])]
    return {
        'schema_version': SCHEMA_VERSION,
        'viewport_m': {'xmin': xmin, 'xmax': xmax, 'ymin': ymin, 'ymax': ymax},
        'map_rect_px': {'left': map_rect[0], 'top': map_rect[1],
                        'width': map_rect[2], 'height': map_rect[3]},
        'pixels_per_m_x': map_rect[4], 'pixels_per_m_y': map_rect[4],
        'relative_metric_scale_error': 0.0,
        'pose_samples_rendered': len(samples),
        'trajectory_segment_count': {robot: len(segments[robot]) for robot in ('robot1', 'robot2')},
        'discontinuity_split_count': sum(max(0, len(segments[robot]) - 1)
                                         for robot in segments),
        'max_continuous_step_m': max(steps, default=0.0),
        'poses_outside_viewport': len(outside),
        'handoff_s': handoff_s,
        'coordinate_source': 'observer odometry transformed by recorded shared_map<-robot/odom TF',
        'transform_convention': 'shared_map_T_robot_odom applied to robot/odom pose',
        'fixed_for_all_frames': True,
        'duration_s': duration_s,
        'overlay_capability': {
            'frontier_representation': 'current_upstream_frontier_region_centroids',
            'frontier_region_source': 'frontier_regions.jsonl.region.centroid (cell centroid fallback only when absent)',
            'raw_frontier_geometry_records': overlay_data.raw_frontier_geometry_records,
            'selected_goal_source': 'legacy NAV_GOAL_SENT or native NavigateToPose EXECUTING joined to DISTRIBUTED_STATUS.active_canonical_task_id and task snapshot approach',
            'planned_path_source': 'DISTRIBUTED_BID_ARRAY.path_samples when available',
            'candidate_source': 'CANDIDATE_BATCH_RECEIVED.candidates',
            'frame_id_note': 'candidate/task overlay records do not serialize frame_id; rendered in their recorded artifact coordinate frame',
        },
    }


def _ffmpeg_libx264():
    executable = shutil.which('ffmpeg')
    if not executable:
        return None
    try:
        result = subprocess.run([executable, '-encoders'], capture_output=True,
                                text=True, timeout=5, check=False)
        if 'libx264' in result.stdout:
            return executable
    except (OSError, subprocess.SubprocessError):
        pass
    return None


def _chunk_ranges(frame_count: int, worker_count: int):
    """Return contiguous, non-empty frame ranges in output order."""
    if frame_count <= 0 or worker_count <= 0:
        return ()
    count = min(frame_count, worker_count)
    base, remainder = divmod(frame_count, count)
    ranges = []
    start = 0
    for index in range(count):
        end = start + base + (1 if index < remainder else 0)
        ranges.append((start, end))
        start = end
    return tuple(ranges)


_PARALLEL_RENDER_CONTEXT = None


def _render_encode_chunk(task):
    """Render one contiguous frame chunk in a forked worker."""
    index, start, end, chunk_path = task
    context = _PARALLEL_RENDER_CONTEXT
    if context is None:
        raise RuntimeError('parallel renderer context was not initialized')
    snapshot_cache = {}
    frontier_layer_cache = {}
    sink = FrameSink(chunk_path, context['width'], context['height'], context['fps'])
    try:
        for frame_index in range(start, end):
            stamp = min(
                context['source_end'],
                context['source_start'] + frame_index * context['speedup'] /
                float(context['fps']))
            sink.write(render_frame(
                stamp, context['snapshots'], snapshot_cache, context['segments'],
                context['viewport'], context['map_rect'], context['width'],
                context['height'], context['title'], context['cooperative_start_s'],
                context['policy'], context['overlay_data'], context['show_frontiers'],
                context['show_goals'], context['show_candidates'],
                context['show_planned_path'], frontier_layer_cache))
    finally:
        sink.close()
    return index, str(chunk_path), end - start


def _concat_encoded_chunks(ffmpeg: str, chunks, output: Path):
    """Concatenate same-format MP4 chunks without decoding/re-encoding."""
    manifest = output.parent / f'.{output.name}.concat.txt'
    lines = []
    for chunk in chunks:
        # TemporaryDirectory paths normally contain no quotes; retain valid
        # concat-demuxer quoting for unusual caller-selected output parents.
        escaped = str(chunk).replace("'", "'\\''")
        lines.append(f"file '{escaped}'")
    manifest.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    try:
        result = subprocess.run(
            [ffmpeg, '-y', '-f', 'concat', '-safe', '0', '-i', str(manifest),
             '-c', 'copy', '-movflags', '+faststart', str(output)],
            capture_output=True, text=True, check=False)
        if result.returncode != 0:
            raise RuntimeError(
                f'ffmpeg chunk concatenation failed ({result.returncode}): '
                f'{result.stderr[-1000:]}')
    finally:
        manifest.unlink(missing_ok=True)


def _render_parallel_chunks(output: Path, timing, context, worker_count: int):
    """Render and encode ordered chunks, then stream-copy them together."""
    ffmpeg = _ffmpeg_libx264()
    if not ffmpeg:
        raise RuntimeError(
            'parallel chunk encoding requires an external ffmpeg with libx264; '
            'use --workers 1 with the available OpenCV writer')
    ranges = _chunk_ranges(timing['expected_frame_count'], worker_count)
    if len(ranges) <= 1:
        raise RuntimeError('parallel chunk encoding needs at least two frame chunks')
    global _PARALLEL_RENDER_CONTEXT
    _PARALLEL_RENDER_CONTEXT = context
    try:
        with tempfile.TemporaryDirectory(prefix='cooperative_render_chunks_') as temporary:
            temporary_path = Path(temporary)
            tasks = [
                (index, start, end, temporary_path / f'chunk_{index:04d}.mp4')
                for index, (start, end) in enumerate(ranges)
            ]
            process_context = multiprocessing.get_context('fork')
            with process_context.Pool(processes=len(tasks)) as pool:
                results = pool.map(_render_encode_chunk, tasks)
            results.sort(key=lambda item: item[0])
            _concat_encoded_chunks(ffmpeg, [Path(item[1]) for item in results], output)
    finally:
        _PARALLEL_RENDER_CONTEXT = None
    return 'ffmpeg_libx264_chunked'


class FrameSink:
    def __init__(self, path: Path, width: int, height: int, fps: int):
        self.path, self.width, self.height, self.fps = path, width, height, fps
        self.mode = 'opencv_avc1'
        self.process = None
        self.writer = None
        ffmpeg = _ffmpeg_libx264()
        if ffmpeg:
            self.mode = 'ffmpeg_libx264'
            self.process = subprocess.Popen(
                [ffmpeg, '-y', '-f', 'rawvideo', '-pix_fmt', 'bgr24',
                 '-s', f'{width}x{height}', '-r', str(fps), '-i', '-',
                 '-an', '-c:v', 'libx264', '-preset', 'medium', '-crf', '18',
                 '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(path)],
                stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        else:
            self.writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'avc1'),
                                          fps, (width, height))
            if not self.writer.isOpened():
                self.writer.release()
                self.writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'mp4v'),
                                              fps, (width, height))
                self.mode = 'opencv_mp4v_fallback'
            if not self.writer.isOpened():
                raise RuntimeError('no usable video encoder; ffmpeg/libx264 unavailable and OpenCV writers failed')

    def write(self, frame):
        if self.process is not None:
            assert self.process.stdin is not None
            self.process.stdin.write(np.ascontiguousarray(frame).tobytes())
        else:
            self.writer.write(frame)

    def close(self):
        if self.process is not None:
            assert self.process.stdin is not None
            self.process.stdin.close()
            stderr = self.process.stderr.read().decode('utf-8', errors='replace') if self.process.stderr else ''
            code = self.process.wait()
            if code != 0:
                raise RuntimeError(f'ffmpeg failed ({code}): {stderr[-1000:]}')
        else:
            self.writer.release()


def probe_video(path: Path, expected_width, expected_height, expected_fps, expected_frames):
    ffprobe = shutil.which('ffprobe')
    if ffprobe:
        try:
            result = subprocess.run(
                [ffprobe, '-v', 'error', '-select_streams', 'v:0', '-show_entries',
                 'stream=codec_name,width,height,avg_frame_rate,nb_frames,duration',
                 '-of', 'json', str(path)], capture_output=True, text=True,
                timeout=15, check=True)
            stream = json.loads(result.stdout)['streams'][0]
            num, den = (stream.get('avg_frame_rate', '0/1').split('/') + ['1'])[:2]
            fps = float(num) / float(den)
            return {'tool': 'ffprobe', 'codec_name': stream.get('codec_name'),
                    'width': int(stream.get('width', 0)), 'height': int(stream.get('height', 0)),
                    'fps': fps, 'frames': int(stream.get('nb_frames') or 0),
                    'duration_s': float(stream.get('duration') or 0.0),
                    'h264_verified': stream.get('codec_name') == 'h264'}
        except (OSError, subprocess.SubprocessError, KeyError, ValueError, json.JSONDecodeError):
            pass
    capture = cv2.VideoCapture(str(path))
    opened = capture.isOpened()
    width = int(round(capture.get(cv2.CAP_PROP_FRAME_WIDTH))) if opened else 0
    height = int(round(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))) if opened else 0
    fps = capture.get(cv2.CAP_PROP_FPS) if opened else 0.0
    frames = int(round(capture.get(cv2.CAP_PROP_FRAME_COUNT))) if opened else 0
    fourcc_value = int(round(capture.get(cv2.CAP_PROP_FOURCC))) if opened else 0
    capture.release()
    fourcc = ''.join(chr((fourcc_value >> (8 * index)) & 0xff) for index in range(4)) if fourcc_value else ''
    return {'tool': 'opencv', 'codec_name': fourcc, 'width': width, 'height': height,
            'fps': fps, 'frames': frames, 'duration_s': frames / fps if fps else 0.0,
            'h264_verified': fourcc == 'avc1',
            'resolution_ok': width == expected_width and height == expected_height,
            'fps_ok': abs(fps - expected_fps) < .01, 'frame_count_ok': frames == expected_frames}


def parse_still_times(value):
    return [float(item.strip()) for item in value.split(',') if item.strip()] if value.strip() else []


def render(args):
    if args.width <= 0 or args.height <= 0 or args.fps <= 0:
        raise RuntimeError('width, height, and fps must be positive')
    if args.workers <= 0:
        raise RuntimeError('workers must be positive')
    if not math.isfinite(args.start_s) or args.start_s < 0.0:
        raise RuntimeError('start-s must be a finite non-negative number')
    if not math.isfinite(args.duration_s) or args.duration_s < 0.0:
        raise RuntimeError('duration-s must be finite and non-negative')
    if not math.isfinite(args.speedup) or args.speedup <= 0.0:
        raise RuntimeError('speedup must be a finite positive number')
    campaign = Path(args.campaign).expanduser().resolve()
    observer = find_observer_root(campaign)
    snapshots = map_snapshots(observer)
    poses = {robot: load_pose_records(observer / f'{robot}_timeseries.csv')
             for robot in ('robot1', 'robot2')}
    artifact_data_end = _source_end(poses, snapshots)
    source_start = args.start_s
    # The requested window is authoritative for the video timeline.  If
    # observer sampling ended a little before the requested boundary, render
    # the last valid saved state through the boundary rather than silently
    # shortening the deliverable.  No unsaved state is invented; the
    # provenance records the held tail explicitly.
    source_end = resolve_source_end(
        source_start, artifact_data_end, args.duration_s)
    if source_end <= source_start:
        raise RuntimeError('source end must be after source start')
        raise RuntimeError('artifacts contain no positive simulation time')
    viewport = compute_fixed_viewport(snapshots, final_map_extent(observer, snapshots))
    map_rect = compute_map_rect(args.width, args.height, viewport)
    events = load_event_records(observer)
    handoff_s = infer_handoff_time(observer, snapshots[0].stamp_s)
    cooperative_start_s = infer_cooperative_start_time(events, handoff_s)
    transforms = {robot: load_shared_odom_transforms(observer / 'forensic/transforms.csv', robot)
                  for robot in ('robot1', 'robot2')}
    segments = {robot: split_shared_pose_segments(poses[robot], transforms[robot], handoff_s)
                for robot in ('robot1', 'robot2')}
    overlay_data = load_overlay_data(observer)
    title = _ascii_display(args.title)
    policy = _ascii_display(args.policy)
    timing = timing_metadata(source_start, source_end, args.speedup, args.fps)
    geometry = geometry_validation(viewport, map_rect, segments, snapshots,
                                   source_end - source_start, handoff_s, overlay_data)
    snapshot_cache = {}
    frontier_layer_cache = {}
    stills = parse_still_times(args.still_times)
    still_paths = []
    if args.stills_dir and stills:
        still_dir = Path(args.stills_dir).expanduser().resolve()
        still_dir.mkdir(parents=True, exist_ok=True)
        canonical = {0.0: 'frame_t0000.png', 300.0: 'frame_t0300.png',
                     750.0: 'frame_t0750.png', 1200.0: 'frame_t1200.png',
                     1500.0: 'frame_t1500.png'}
        for stamp in stills:
            bounded = min(max(source_start, stamp), source_end)
            frame = render_frame(bounded, snapshots, snapshot_cache, segments, viewport,
                                 map_rect, args.width, args.height, title, cooperative_start_s, policy,
                                 overlay_data, args.show_frontiers, args.show_goals,
                                 args.show_candidates, args.show_planned_path,
                                 frontier_layer_cache)
            name = next((value for key, value in canonical.items() if abs(stamp - key) < 1e-6),
                        'frame_post_cooperative.png' if abs(stamp - cooperative_start_s) < 2.0 else
                        f"frame_t{stamp:07.2f}".replace('.', '_') + '.png')
            output_path = still_dir / name
            cv2.imwrite(str(output_path), frame)
            still_paths.append(str(output_path))

    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    video_probe = None
    encoder_mode = None
    if not args.stills_only:
        if args.workers == 1:
            sink = FrameSink(output, args.width, args.height, args.fps)
            encoder_mode = sink.mode
            try:
                for frame_index in range(timing['expected_frame_count']):
                    stamp = min(source_end, source_start +
                                frame_index * args.speedup / float(args.fps))
                    sink.write(render_frame(
                        stamp, snapshots, snapshot_cache, segments, viewport, map_rect,
                        args.width, args.height, title, cooperative_start_s, policy, overlay_data,
                        args.show_frontiers, args.show_goals, args.show_candidates,
                        args.show_planned_path, frontier_layer_cache))
            finally:
                sink.close()
        else:
            parallel_context = {
                'source_start': source_start,
                'source_end': source_end,
                'speedup': args.speedup,
                'fps': args.fps,
                'width': args.width,
                'height': args.height,
                'snapshots': snapshots,
                'segments': segments,
                'viewport': viewport,
                'map_rect': map_rect,
                'title': title,
                'cooperative_start_s': cooperative_start_s,
                'policy': policy,
                'overlay_data': overlay_data,
                'show_frontiers': args.show_frontiers,
                'show_goals': args.show_goals,
                'show_candidates': args.show_candidates,
                'show_planned_path': args.show_planned_path,
            }
            encoder_mode = _render_parallel_chunks(
                output, timing, parallel_context, args.workers)
        if not output.is_file() or output.stat().st_size == 0:
            raise RuntimeError('renderer produced no output')
        video_probe = probe_video(output, args.width, args.height, args.fps,
                                  timing['expected_frame_count'])
    result = {'schema_version': SCHEMA_VERSION, 'output': str(output),
              'observer_root': str(observer), 'source_campaign': str(campaign),
              'width': args.width, 'height': args.height, **timing,
              'artifact_data_end_s': artifact_data_end,
              'held_final_frame_duration_s': max(0.0, source_end - artifact_data_end),
              'fixed_viewport_m': {'xmin': viewport[0], 'xmax': viewport[1],
                                   'ymin': viewport[2], 'ymax': viewport[3]},
              'map_rect_px': dict(zip(('left', 'top', 'width', 'height', 'pixels_per_m'), map_rect)),
              'handoff_s': handoff_s,
              'cooperative_start_s': cooperative_start_s,
              'render_workers': args.workers,
              'map_snapshots': len(snapshots),
              'trajectory_segments': {robot: len(segments[robot]) for robot in ('robot1', 'robot2')},
              'font_family': 'Liberation Sans' if Path('/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf').is_file() else 'DejaVu Sans',
              'encoder_mode': encoder_mode, 'video_probe': video_probe,
              'geometry_validation': geometry,
              'overlay_counts': {
                  'frontier_region_snapshots_robot1': len(overlay_data.frontier_regions['robot1']),
                  'frontier_region_snapshots_robot2': len(overlay_data.frontier_regions['robot2']),
                  'candidate_batches_robot1': len(overlay_data.candidate_batches['robot1']),
                  'candidate_batches_robot2': len(overlay_data.candidate_batches['robot2']),
                  'task_snapshots_robot1': len(overlay_data.task_records['robot1']),
                  'task_snapshots_robot2': len(overlay_data.task_records['robot2']),
                  'goal_events_robot1': len(overlay_data.goal_events['robot1']),
                  'goal_events_robot2': len(overlay_data.goal_events['robot2']),
                  'bid_path_batches_robot1': len(overlay_data.bid_batches['robot1']),
                  'bid_path_batches_robot2': len(overlay_data.bid_batches['robot2']),
                  'raw_frontier_geometry_records': overlay_data.raw_frontier_geometry_records,
              },
              'overlay_options': {'frontiers': args.show_frontiers, 'goals': args.show_goals,
                                  'candidates': args.show_candidates,
                                  'planned_path': args.show_planned_path},
              'still_paths': still_paths}
    if args.provenance_output:
        provenance = Path(args.provenance_output).expanduser().resolve()
        provenance.parent.mkdir(parents=True, exist_ok=True)
        provenance.write_text(json.dumps(result, indent=2, sort_keys=True), encoding='utf-8')
    print(json.dumps(result, indent=2, sort_keys=True))
    return result


def main(argv=None):
    try:
        render(parse_args(argv))
    except (OSError, RuntimeError, ValueError) as error:
        print(f'ANIMATION_RENDER_FAILURE: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
