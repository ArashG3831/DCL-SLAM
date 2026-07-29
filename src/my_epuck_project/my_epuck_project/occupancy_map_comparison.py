"""World-aligned, vectorized occupancy-grid comparison and consensus tools."""

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path

import numpy as np


UNKNOWN = 0
FREE = 1
OCCUPIED = 2
UNCERTAIN = 3
CLASS_NAMES = ('unknown', 'free', 'occupied', 'uncertain')


@dataclass(frozen=True)
class Geometry:
    """Axis-aligned target geometry or a source OccupancyGrid geometry."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    yaw: float = 0.0


@dataclass
class OccupancyMap:
    """Lossless occupancy data plus its world geometry."""

    data: np.ndarray
    geometry: Geometry
    metadata: dict
    conflict: np.ndarray = None


def _yaw(quaternion):
    x = quaternion['x']
    y = quaternion['y']
    z = quaternion['z']
    w = quaternion['w']
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def load_map(path):
    """Load a collector or canonical NPZ artifact."""
    with np.load(path, allow_pickle=False) as archive:
        if 'occupancy' in archive:
            data = archive['occupancy'].astype(np.int8, copy=False)
        else:
            data = archive['data'].astype(np.int8, copy=False)
        raw = archive['metadata_json'].item()
        metadata = json.loads(str(raw))
        conflict = archive['conflict'].astype(bool) \
            if 'conflict' in archive else None
    origin = metadata['origin']
    orientation = origin['orientation']
    geometry = Geometry(
        width=int(metadata['width']),
        height=int(metadata['height']),
        resolution=float(metadata['resolution']),
        origin_x=float(origin['position']['x']),
        origin_y=float(origin['position']['y']),
        yaw=_yaw(orientation),
    )
    if data.shape != (geometry.height, geometry.width):
        raise ValueError(f'{path}: occupancy shape does not match metadata')
    return OccupancyMap(data, geometry, metadata, conflict)


def save_map(path, occupancy_map, extra_metadata=None):
    """Atomically save an OccupancyMap using the collector-compatible schema."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    geometry = occupancy_map.geometry
    metadata = dict(occupancy_map.metadata)
    metadata.update({
        'width': geometry.width,
        'height': geometry.height,
        'resolution': geometry.resolution,
        'origin': {
            'position': {
                'x': geometry.origin_x, 'y': geometry.origin_y, 'z': 0.0,
            },
            'orientation': {
                'x': 0.0, 'y': 0.0,
                'z': math.sin(geometry.yaw / 2.0),
                'w': math.cos(geometry.yaw / 2.0),
            },
        },
    })
    if extra_metadata:
        metadata.update(extra_metadata)
    temporary = path.with_name(f'.{path.name}.{os.getpid()}.tmp')
    arrays = {
        'occupancy': np.asarray(occupancy_map.data, dtype=np.int8),
        'metadata_json': np.asarray(
            json.dumps(metadata, sort_keys=True, allow_nan=False)),
    }
    if occupancy_map.conflict is not None:
        arrays['conflict'] = np.asarray(occupancy_map.conflict, dtype=bool)
    with temporary.open('wb') as stream:
        np.savez_compressed(stream, **arrays)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    return metadata


def classify(values, free_threshold=25, occupied_threshold=65):
    """Classify exact occupancy values without folding uncertain into free."""
    values = np.asarray(values)
    result = np.full(values.shape, UNCERTAIN, dtype=np.uint8)
    result[values < 0] = UNKNOWN
    result[(values >= 0) & (values <= free_threshold)] = FREE
    result[values >= occupied_threshold] = OCCUPIED
    return result


def world_corners(geometry):
    local = np.array([
        [0.0, 0.0],
        [geometry.width * geometry.resolution, 0.0],
        [0.0, geometry.height * geometry.resolution],
        [geometry.width * geometry.resolution,
         geometry.height * geometry.resolution],
    ])
    cosine, sine = math.cos(geometry.yaw), math.sin(geometry.yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    return local @ rotation.T + np.array(
        [geometry.origin_x, geometry.origin_y])


def common_geometry(maps, resolution=None):
    """Build an axis-aligned, world-coordinate union grid."""
    if not maps:
        raise ValueError('at least one map is required')
    resolutions = [item.geometry.resolution for item in maps]
    resolution = float(resolution or resolutions[0])
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError('comparison resolution must be finite and positive')
    corners = np.concatenate([world_corners(item.geometry) for item in maps])
    minimum = np.floor(corners.min(axis=0) / resolution) * resolution
    maximum = np.ceil(corners.max(axis=0) / resolution) * resolution
    size = np.maximum(1, np.rint((maximum - minimum) / resolution).astype(int))
    return Geometry(
        width=int(size[0]), height=int(size[1]), resolution=resolution,
        origin_x=float(minimum[0]), origin_y=float(minimum[1]), yaw=0.0)


def _source_indices(occupancy_map, target):
    source = occupancy_map.geometry
    columns = target.origin_x + (
        np.arange(target.width, dtype=np.float64) + 0.5
    ) * target.resolution
    rows = target.origin_y + (
        np.arange(target.height, dtype=np.float64) + 0.5
    ) * target.resolution
    world_x, world_y = np.meshgrid(columns, rows)
    delta_x = world_x - source.origin_x
    delta_y = world_y - source.origin_y
    cosine, sine = math.cos(source.yaw), math.sin(source.yaw)
    local_x = cosine * delta_x + sine * delta_y
    local_y = -sine * delta_x + cosine * delta_y
    source_x = np.floor(local_x / source.resolution).astype(np.int64)
    source_y = np.floor(local_y / source.resolution).astype(np.int64)
    valid = (
        (source_x >= 0) & (source_x < source.width)
        & (source_y >= 0) & (source_y < source.height)
    )
    return source_x, source_y, valid


def resample_values(occupancy_map, target):
    """Nearest-cell lossless values; cells outside the source are unknown."""
    source_x, source_y, valid = _source_indices(occupancy_map, target)
    output = np.full((target.height, target.width), -1, dtype=np.int8)
    output[valid] = occupancy_map.data[source_y[valid], source_x[valid]]
    return output


def resample_semantic(occupancy_map, target, free_threshold=25,
                      occupied_threshold=65):
    """Nearest-cell resampling by target cell centers in world coordinates."""
    source_x, source_y, valid = _source_indices(occupancy_map, target)
    output = np.full((target.height, target.width), UNKNOWN, dtype=np.uint8)
    semantic = classify(
        occupancy_map.data, free_threshold, occupied_threshold)
    output[valid] = semantic[source_y[valid], source_x[valid]]
    return output


def semantic_to_occupancy(semantic):
    """Stable canonical representative values for each semantic class."""
    result = np.full(np.asarray(semantic).shape, -1, dtype=np.int8)
    result[semantic == FREE] = 0
    result[semantic == OCCUPIED] = 100
    result[semantic == UNCERTAIN] = 50
    return result


def safe_ratio(numerator, denominator):
    return float(numerator / denominator) if denominator else None


def _correlation(a_values, b_values):
    if a_values.size < 2 or b_values.size < 2:
        return None
    if np.ptp(a_values) == 0 or np.ptp(b_values) == 0:
        return None
    value = float(np.corrcoef(a_values, b_values)[0, 1])
    return value if math.isfinite(value) else None


def _shift_overlap(a, b, dx, dy):
    height, width = a.shape
    ax0, ax1 = max(0, dx), min(width, width + dx)
    ay0, ay1 = max(0, dy), min(height, height + dy)
    bx0, bx1 = max(0, -dx), min(width, width - dx)
    by0, by1 = max(0, -dy), min(height, height - dy)
    return a[ay0:ay1, ax0:ax1], b[by0:by1, bx0:bx1]


def shift_diagnostic(a, b, resolution, window=3):
    """Search small translations without altering authoritative metrics."""
    scores = {}
    for dy in range(-window, window + 1):
        for dx in range(-window, window + 1):
            left, right = _shift_overlap(a, b, dx, dy)
            if np.issubdtype(left.dtype, np.signedinteger):
                known = (left >= 0) & (right >= 0)
            else:
                known = (left != UNKNOWN) & (right != UNKNOWN)
            scores[(dx, dy)] = _correlation(
                left[known].astype(float), right[known].astype(float))
    available = {key: value for key, value in scores.items()
                 if value is not None}
    best = max(available, key=available.get) if available else (0, 0)
    zero = scores.get((0, 0))
    best_value = scores.get(best)
    return {
        'zero_shift_correlation': zero,
        'best_shift_correlation': best_value,
        'best_shift_dx_cells': best[0],
        'best_shift_dy_cells': best[1],
        'best_shift_dx_m': best[0] * resolution,
        'best_shift_dy_m': best[1] * resolution,
        'correlation_improvement': (
            best_value - zero
            if best_value is not None and zero is not None else None
        ),
    }


def tolerant_occupied_agreement(a, b, tolerance=1):
    """Supplementary symmetric occupied agreement within a cell radius."""
    occupied_a = a == OCCUPIED
    occupied_b = b == OCCUPIED

    def dilate(mask):
        padded = np.pad(mask, tolerance)
        views = [
            padded[
                tolerance + dy:tolerance + dy + mask.shape[0],
                tolerance + dx:tolerance + dx + mask.shape[1],
            ]
            for dy in range(-tolerance, tolerance + 1)
            for dx in range(-tolerance, tolerance + 1)
            if dx * dx + dy * dy <= tolerance * tolerance
        ]
        return np.logical_or.reduce(views)

    count = int(occupied_a.sum() + occupied_b.sum())
    if count == 0:
        return None
    matches = int((occupied_a & dilate(occupied_b)).sum())
    matches += int((occupied_b & dilate(occupied_a)).sum())
    return matches / count


def compare_semantic(a, b, resolution, shift_window=3,
                     raw_a=None, raw_b=None):
    """Compute authoritative semantic metrics on one common grid."""
    known_a, known_b = a != UNKNOWN, b != UNKNOWN
    known_union = known_a | known_b
    jointly_known = known_a & known_b
    free_a, free_b = a == FREE, b == FREE
    occupied_a, occupied_b = a == OCCUPIED, b == OCCUPIED
    cell_area = resolution * resolution

    def iou(left, right):
        return safe_ratio(int((left & right).sum()), int((left | right).sum()))

    conflicts = ((a == FREE) & (b == OCCUPIED)) | (
        (a == OCCUPIED) & (b == FREE))
    confusion = {}
    for left_index, left_name in enumerate(CLASS_NAMES):
        for right_index, right_name in enumerate(CLASS_NAMES):
            confusion[f'{left_name}/{right_name}'] = int(
                ((a == left_index) & (b == right_index)).sum())
    area_a = int(known_a.sum()) * cell_area
    area_b = int(known_b.sum()) * cell_area
    occupied_area_a = int(occupied_a.sum()) * cell_area
    occupied_area_b = int(occupied_b.sum()) * cell_area
    free_area_a = int(free_a.sum()) * cell_area
    free_area_b = int(free_b.sum()) * cell_area
    correlation_a = a if raw_a is None else raw_a
    correlation_b = b if raw_b is None else raw_b
    result = {
        'known_iou': iou(known_a, known_b),
        'free_iou': iou(free_a, free_b),
        'occupied_iou': iou(occupied_a, occupied_b),
        'known_cell_agreement': safe_ratio(
            int((a[jointly_known] == b[jointly_known]).sum()),
            int(jointly_known.sum())),
        'occupied_free_conflict_rate': safe_ratio(
            int(conflicts.sum()), int(jointly_known.sum())),
        'unknown_mismatch_rate': safe_ratio(
            int((known_a ^ known_b).sum()), int(known_union.sum())),
        'coverage_area_a_m2': area_a,
        'coverage_area_b_m2': area_b,
        'coverage_area_difference_m2': abs(area_a - area_b),
        'coverage_area_relative_difference': safe_ratio(
            abs(area_a - area_b), max(area_a, area_b)),
        'occupied_area_difference_m2': abs(
            occupied_area_a - occupied_area_b),
        'occupied_area_relative_difference': safe_ratio(
            abs(occupied_area_a - occupied_area_b),
            max(occupied_area_a, occupied_area_b)),
        'free_area_difference_m2': abs(free_area_a - free_area_b),
        'free_area_relative_difference': safe_ratio(
            abs(free_area_a - free_area_b),
            max(free_area_a, free_area_b)),
        'raw_occupancy_correlation': _correlation(
            correlation_a[jointly_known].astype(float),
            correlation_b[jointly_known].astype(float)),
        'jointly_known_cell_count': int(jointly_known.sum()),
        'known_union_count': int(known_union.sum()),
        'semantic_confusion_matrix': confusion,
        'boundary_tolerant_occupied_agreement':
            tolerant_occupied_agreement(a, b),
    }
    result.update(shift_diagnostic(
        correlation_a, correlation_b, resolution, shift_window))
    return result


def compare_maps(map_a, map_b, resolution=None, free_threshold=25,
                 occupied_threshold=65, shift_window=3):
    target = common_geometry([map_a, map_b], resolution)
    raw_a = resample_values(map_a, target)
    raw_b = resample_values(map_b, target)
    a = classify(raw_a, free_threshold, occupied_threshold)
    b = classify(raw_b, free_threshold, occupied_threshold)
    result = compare_semantic(
        a, b, target.resolution, shift_window, raw_a, raw_b)
    ga, gb = map_a.geometry, map_b.geometry
    result.update({
        'geometry_equal': ga == gb,
        'resolution_equal': math.isclose(
            ga.resolution, gb.resolution, rel_tol=0.0, abs_tol=1e-12),
        'origin_position_difference_m': math.hypot(
            ga.origin_x - gb.origin_x, ga.origin_y - gb.origin_y),
        'origin_yaw_difference_rad': abs(ga.yaw - gb.yaw),
        'dimensions_a': [ga.width, ga.height],
        'dimensions_b': [gb.width, gb.height],
        'comparison_resolution': target.resolution,
        'common_grid': {
            'width': target.width, 'height': target.height,
            'resolution': target.resolution,
            'origin_x': target.origin_x, 'origin_y': target.origin_y,
        },
    })
    return result, target, a, b


def canonical_map(robot1, robot2, resolution=None, free_threshold=25,
                  occupied_threshold=65):
    """Conservative union retaining a separate known-known conflict mask."""
    target = common_geometry([robot1, robot2], resolution)
    first = resample_semantic(
        robot1, target, free_threshold, occupied_threshold)
    second = resample_semantic(
        robot2, target, free_threshold, occupied_threshold)
    known_first, known_second = first != UNKNOWN, second != UNKNOWN
    only_first = known_first & ~known_second
    only_second = known_second & ~known_first
    agree = known_first & known_second & (first == second)
    conflict = known_first & known_second & (first != second)
    semantic = np.full(first.shape, UNKNOWN, dtype=np.uint8)
    semantic[only_first] = first[only_first]
    semantic[only_second] = second[only_second]
    semantic[agree] = first[agree]
    # Conservative occupied preference is encoded only for visualization/data;
    # the authoritative conflict array below preserves every disagreement.
    semantic[conflict] = np.where(
        (first[conflict] == OCCUPIED)
        | (second[conflict] == OCCUPIED), OCCUPIED, UNCERTAIN)
    counts = {
        'robot1_only_known_cells': int(only_first.sum()),
        'robot2_only_known_cells': int(only_second.sum()),
        'jointly_known_agreeing_cells': int(agree.sum()),
        'jointly_known_conflicting_cells': int(conflict.sum()),
        'conflict_fraction_of_jointly_known': safe_ratio(
            int(conflict.sum()), int((known_first & known_second).sum())),
    }
    metadata = {
        'frame_id': 'shared_map',
        'canonical_policy': 'known_union_with_explicit_conflict_mask',
        'canonical_counts': counts,
    }
    return (
        OccupancyMap(
            semantic_to_occupancy(semantic), target, metadata, conflict),
        counts,
        semantic,
    )


def consensus_maps(maps, resolution=None, free_threshold=25,
                   occupied_threshold=65):
    """Create lossless class frequencies, majority, and disagreement arrays."""
    target = common_geometry(maps, resolution)
    stack = np.stack([
        resample_semantic(item, target, free_threshold, occupied_threshold)
        for item in maps
    ])
    frequencies = np.stack(
        [(stack == value).mean(axis=0) for value in range(4)])
    majority = np.argmax(frequencies, axis=0).astype(np.uint8)
    known_frequency = 1.0 - frequencies[UNKNOWN]
    semantic_agreement = frequencies.max(axis=0)
    disagreement = 1.0 - semantic_agreement
    return {
        'geometry': target,
        'stack': stack,
        'known_frequency': known_frequency,
        'free_frequency': frequencies[FREE],
        'occupied_frequency': frequencies[OCCUPIED],
        'uncertain_frequency': frequencies[UNCERTAIN],
        'semantic_agreement_fraction': semantic_agreement,
        'disagreement_frequency': disagreement,
        'majority_semantic': majority,
        'consensus_occupancy': semantic_to_occupancy(majority),
    }
