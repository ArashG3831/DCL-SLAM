"""Export cooperative occupancy-map artifacts as portable PNG files."""

import argparse
import bisect
import csv
import json
import math
import os
import struct
import zlib
from pathlib import Path

import numpy as np

from .occupancy_map_comparison import canonical_map, load_map


PNG_SIGNATURE = b'\x89PNG\r\n\x1a\n'
UNKNOWN_RGB = (205, 205, 205)
FREE_RGB = (255, 255, 255)
OCCUPIED_RGB = (0, 0, 0)
UNCERTAIN_RGB = (255, 193, 7)
DIFFERENCE_RGB = (220, 0, 0)
AGREEMENT_RGB = (238, 238, 238)
POSE_COLORS = {
    'robot1': (220, 30, 30),
    'robot2': (25, 95, 220),
}
PATH_OUTLINE_RGB = (70, 70, 70)
POSE_OUTLINE_RGB = (20, 20, 20)


_FONT = {
    '0': ('01110', '10001', '10011', '10101', '11001', '10001', '01110'),
    '1': ('00100', '01100', '00100', '00100', '00100', '00100', '01110'),
    '2': ('01110', '10001', '00001', '00010', '00100', '01000', '11111'),
    'R': ('11110', '10001', '10001', '11110', '10100', '10010', '10001'),
}


def _png_chunk(kind, payload):
    checksum = zlib.crc32(kind)
    checksum = zlib.crc32(payload, checksum)
    return (
        struct.pack('>I', len(payload)) + kind + payload
        + struct.pack('>I', checksum & 0xffffffff)
    )


def write_rgb_png(path, rgb):
    """Write an RGB uint8 array as a dependency-free, atomic PNG."""
    path = Path(path)
    array = np.asarray(rgb, dtype=np.uint8)
    if array.ndim != 3 or array.shape[2] != 3:
        raise ValueError('PNG input must have shape (height, width, 3)')
    height, width, _ = array.shape
    scanlines = b''.join(
        b'\x00' + array[row].tobytes() for row in range(height)
    )
    header = struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)
    content = (
        PNG_SIGNATURE
        + _png_chunk(b'IHDR', header)
        + _png_chunk(b'IDAT', zlib.compress(scanlines, level=9))
        + _png_chunk(b'IEND', b'')
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f'.{path.name}.{os.getpid()}.tmp')
    with temporary.open('wb') as stream:
        stream.write(content)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def occupancy_rgb(data, free_threshold=25, occupied_threshold=65, scale=4):
    """Color a ROS occupancy array and orient world +y toward image top."""
    values = np.asarray(data)
    rgb = np.empty(values.shape + (3,), dtype=np.uint8)
    rgb[:] = UNCERTAIN_RGB
    rgb[values < 0] = UNKNOWN_RGB
    rgb[(values >= 0) & (values <= free_threshold)] = FREE_RGB
    rgb[values >= occupied_threshold] = OCCUPIED_RGB
    rgb = np.flipud(rgb)
    if scale < 1:
        raise ValueError('scale must be at least one')
    if scale > 1:
        rgb = np.repeat(np.repeat(rgb, scale, axis=0), scale, axis=1)
    return rgb


def _yaw_from_quaternion(row):
    """Return planar yaw from a transform CSV row."""
    z = float(row['rotation_z'])
    w = float(row['rotation_w'])
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def _transform_row(rows, target, source):
    """Return the newest available captured 2-D transform."""
    candidates = []
    for row in rows:
        if (row.get('target_frame') != target
                or row.get('source_frame') != source
                or row.get('available') != 'True'):
            continue
        try:
            candidates.append((float(row['query_ros_time_s']), row))
        except (KeyError, TypeError, ValueError):
            continue
    if not candidates:
        return None
    return max(candidates, key=lambda item: item[0])[1]


def _compose_transform(first, second):
    """Compose target<-middle with middle<-source in the plane."""
    first_yaw = _yaw_from_quaternion(first)
    second_yaw = _yaw_from_quaternion(second)
    cosine, sine = math.cos(first_yaw), math.sin(first_yaw)
    second_x, second_y = float(second['translation_x']), float(
        second['translation_y'])
    return {
        'x_m': float(first['translation_x']) + cosine * second_x - sine * second_y,
        'y_m': float(first['translation_y']) + sine * second_x + cosine * second_y,
        'yaw_rad': first_yaw + second_yaw,
        'query_ros_time_s': max(
            float(first['query_ros_time_s']),
            float(second['query_ros_time_s'])),
        'source': 'forensic transforms.csv: shared_map<-odom<-base_footprint',
    }


def load_final_shared_poses(attempt):
    """Load final poses in shared_map coordinates without ROS dependencies.

    The forensic TF capture is authoritative.  The low-rate observer
    timeseries is a compatibility fallback for older campaigns.
    """
    attempt = Path(attempt)
    poses = {}
    transform_path = next(
        (path for path in (
            attempt / 'forensic' / 'transforms.csv',
            *sorted((attempt / 'observer').glob('*/forensic/transforms.csv')),
        ) if path.is_file()), None)
    rows = []
    if transform_path is not None:
        try:
            with transform_path.open(newline='', encoding='utf-8') as stream:
                rows = list(csv.DictReader(stream))
        except OSError:
            rows = []
    for robot in ('robot1', 'robot2'):
        shared_to_odom = _transform_row(
            rows, 'shared_map', f'{robot}/odom')
        odom_to_base = _transform_row(
            rows, f'{robot}/odom', f'{robot}/base_footprint')
        if shared_to_odom and odom_to_base:
            poses[robot] = _compose_transform(shared_to_odom, odom_to_base)

    if len(poses) == 2:
        return poses

    # The logger stores the same shared_map pose used by the cross-robot
    # metrics.  This fallback keeps old campaigns exportable if TF capture
    # was not enabled.
    for robot in ('robot1', 'robot2'):
        if robot in poses:
            continue
        paths = sorted((attempt / 'observer').glob(
            f'*/{robot}_timeseries.csv'))
        if not paths:
            paths = sorted(attempt.glob(f'**/{robot}_timeseries.csv'))
        for path in reversed(paths):
            try:
                with path.open(newline='', encoding='utf-8') as stream:
                    rows = list(csv.DictReader(stream))
            except OSError:
                continue
            for row in reversed(rows):
                try:
                    if row.get('pose_x', '') == '' or row.get('pose_y', '') == '':
                        continue
                    poses[robot] = {
                        'x_m': float(row['pose_x']),
                        'y_m': float(row['pose_y']),
                        'yaw_rad': float(row.get('pose_yaw') or 0.0),
                        'query_ros_time_s': float(row.get('ros_time_sec') or 0.0),
                        'source': 'observer robot_timeseries.csv',
                    }
                    break
                except (TypeError, ValueError):
                    continue
            if robot in poses:
                break
    return poses


def load_shared_map_paths(attempt, max_gap_s=5.0, max_jump_m=1.0):
    """Load bounded robot trajectories in the final ``shared_map`` frame.

    The observer ``robot*_timeseries.csv`` pose columns are native odometry
    poses in ``robotN/odom``; they are not shared-map coordinates.  Rendering
    those values directly makes paths spill away from the OccupancyGrid when
    the accepted map/odom relation is non-identity.  The forensic transform
    capture contains the time-varying ``shared_map<-robotN/odom`` relation,
    so it is applied to every recorded odometry pose here.

    Large time gaps and implausible jumps are split into separate segments so
    a dropped capture interval or a discrete scan-matching correction cannot
    draw a misleading line across the map.
    """
    attempt = Path(attempt)
    transforms = _forensic_transforms(attempt)
    result = {}
    for robot in ('robot1', 'robot2'):
        relation = sorted(
            (
                float(row['query_ros_time_s']),
                (
                    float(row['translation_x']),
                    float(row['translation_y']),
                    _yaw_from_quaternion(row),
                ),
            )
            for row in transforms
            if row.get('target_frame') == 'shared_map'
            and row.get('source_frame') == f'{robot}/odom'
            and row.get('available') == 'True'
            and row.get('query_ros_time_s') not in (None, '')
        )
        candidates = sorted((attempt / 'forensic').glob(
            f'{robot}_odom.csv'))
        if not candidates:
            candidates = sorted(attempt.glob(f'**/forensic/{robot}_odom.csv'))
        if not candidates or not relation:
            continue
        path = candidates[-1]
        try:
            with path.open(newline='', encoding='utf-8') as stream:
                rows = list(csv.DictReader(stream))
        except OSError:
            continue

        relation_times = [item[0] for item in relation]
        segments = []
        current = []
        previous_time = None
        previous_point = None
        for row in rows:
            try:
                timestamp = float(row['header_stamp'])
                odom_point = (float(row['pose_x']), float(row['pose_y']))
                if not all(math.isfinite(value) for value in odom_point):
                    raise ValueError
            except (KeyError, TypeError, ValueError):
                continue

            relation_index = bisect.bisect_right(
                relation_times, timestamp) - 1
            # The accepted shared frame is created after the first local
            # odometry samples, but its locked relation is valid for the
            # complete recorded trajectory.  Use the earliest captured
            # relation for those initial samples.
            relation_index = max(0, relation_index)
            transform = relation[relation_index][1]
            cosine, sine = math.cos(transform[2]), math.sin(transform[2])
            point = (
                transform[0] + cosine * odom_point[0]
                - sine * odom_point[1],
                transform[1] + sine * odom_point[0]
                + cosine * odom_point[1],
            )
            if not all(math.isfinite(value) for value in point):
                continue

            gap = (timestamp - previous_time) if previous_time is not None else 0.0
            jump = (
                math.hypot(point[0] - previous_point[0],
                           point[1] - previous_point[1])
                if previous_point is not None else 0.0)
            if current and (gap > max_gap_s or jump > max_jump_m):
                if len(current) >= 2:
                    segments.append(current)
                current = []
            current.append(point)
            previous_time = timestamp
            previous_point = point
        if len(current) >= 2:
            segments.append(current)
        if segments:
            result[robot] = {
                'source': (
                    f'{path} + forensic transforms.csv '
                    f'(shared_map<-{robot}/odom)'),
                'segments': segments,
                'point_count': sum(len(segment) for segment in segments),
            }
    return result


def _forensic_transforms(attempt):
    """Load captured transforms without consulting simulator/world state."""
    path = next((candidate for candidate in (
        Path(attempt) / 'forensic' / 'transforms.csv',
        *sorted((Path(attempt) / 'observer').glob(
            '*/forensic/transforms.csv')),
    ) if candidate.is_file()), None)
    if path is None:
        return []
    try:
        with path.open(newline='', encoding='utf-8') as stream:
            return list(csv.DictReader(stream))
    except OSError:
        return []


def _scan_matching_map_to_odom(attempt, robot):
    """Load recorded time-varying local ``map<-odom`` corrections.

    Some observer runs captured scan-matching corrections but did not include
    ``robotN/map<-robotN/odom`` in ``transforms.csv``.  Those corrections are
    still authoritative for projecting the odometry path; using raw odometry
    directly on a map silently draws a geometrically wrong path.
    """
    candidates = [Path(attempt) / 'forensic' / 'scan_matching' /
                  f'{robot}_corrections.jsonl']
    candidates.extend(sorted(Path(attempt).glob(
        f'**/forensic/scan_matching/{robot}_corrections.jsonl')))
    path = next((candidate for candidate in candidates if candidate.is_file()),
                None)
    if path is None:
        return []
    result = []
    try:
        with path.open(encoding='utf-8') as stream:
            for line in stream:
                try:
                    value = json.loads(line)
                    if (not value.get('available')
                            or value.get('map_to_odom_x') is None):
                        continue
                    result.append((
                        float(value.get('map_to_odom_stamp')
                              or value['query_ros_time_s']),
                        float(value['map_to_odom_x']),
                        float(value['map_to_odom_y']),
                        float(value['map_to_odom_yaw'])))
                except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                    continue
    except OSError:
        return []
    return sorted(result, key=lambda item: item[0])


def _map_to_odom_at(corrections, timestamp):
    """Return the latest recorded map<-odom correction at ``timestamp``."""
    selected = None
    for correction in corrections:
        if correction[0] > timestamp:
            break
        selected = correction
    return selected or (0.0, 0.0, 0.0, 0.0)


def _project_odom_row(row, correction):
    """Project one odometry row through a planar map<-odom correction."""
    timestamp, translation_x, translation_y, correction_yaw = correction
    odom_x, odom_y = float(row['pose_x']), float(row['pose_y'])
    cosine, sine = math.cos(correction_yaw), math.sin(correction_yaw)
    return (
        translation_x + cosine * odom_x - sine * odom_y,
        translation_y + sine * odom_x + cosine * odom_y,
        correction_yaw + math.atan2(
            2.0 * float(row['orientation_w']) * float(row['orientation_z']),
            1.0 - 2.0 * float(row['orientation_z']) ** 2),
        timestamp,
    )


def _apply_transform(row, point):
    """Apply a captured planar target<-source transform to an odom point."""
    yaw_value = _yaw_from_quaternion(row)
    cosine, sine = math.cos(yaw_value), math.sin(yaw_value)
    x, y = point
    return (float(row['translation_x']) + cosine * x - sine * y,
            float(row['translation_y']) + sine * x + cosine * y)


def load_local_map_poses(attempt):
    """Load local-map poses from odometry and captured map<-odom TF.

    This is deliberately separate from :func:`load_final_shared_poses`: a
    no-handoff run has no valid shared frame, so only each robot's own map
    frame may be used.
    """
    attempt = Path(attempt)
    transforms = _forensic_transforms(attempt)
    result = {}
    for robot in ('robot1', 'robot2'):
        tf_row = _transform_row(transforms, f'{robot}/map', f'{robot}/odom')
        corrections = ([] if tf_row is not None else
                       _scan_matching_map_to_odom(attempt, robot))
        paths = sorted((attempt / 'forensic').glob(f'{robot}_odom.csv'))
        if not paths:
            paths = sorted(attempt.glob(f'**/{robot}_odom.csv'))
        if not paths or (tf_row is None and not corrections):
            continue
        try:
            with paths[-1].open(newline='', encoding='utf-8') as stream:
                rows = list(csv.DictReader(stream))
        except OSError:
            continue
        for row in reversed(rows):
            try:
                if tf_row is not None:
                    point = _apply_transform(tf_row, (
                        float(row['pose_x']), float(row['pose_y'])))
                    yaw = float(row.get('pose_yaw') or 0.0)
                    source = (f'{paths[-1]} + captured '
                              f'{robot}/map<-{robot}/odom')
                else:
                    timestamp = float(row['header_stamp'])
                    x, y, yaw, _ = _project_odom_row(
                        row, _map_to_odom_at(corrections, timestamp))
                    point = (x, y)
                    source = (f'{paths[-1]} + recorded scan-matching '
                              f'{robot}/map<-{robot}/odom')
                result[robot] = {
                    'x_m': point[0], 'y_m': point[1],
                    'yaw_rad': yaw,
                    'query_ros_time_s': float(
                        row.get('received_ros_time_s') or 0.0),
                    'source': source,
                }
                break
            except (KeyError, TypeError, ValueError):
                continue
    return result


def load_local_map_paths(attempt, max_gap_s=5.0, max_jump_m=1.0):
    """Project captured odom trajectories into each robot's local map frame."""
    attempt = Path(attempt)
    transforms = _forensic_transforms(attempt)
    result = {}
    for robot in ('robot1', 'robot2'):
        tf_row = _transform_row(transforms, f'{robot}/map', f'{robot}/odom')
        corrections = ([] if tf_row is not None else
                       _scan_matching_map_to_odom(attempt, robot))
        paths = sorted((attempt / 'forensic').glob(f'{robot}_odom.csv'))
        if not paths:
            paths = sorted(attempt.glob(f'**/{robot}_odom.csv'))
        if not paths or (tf_row is None and not corrections):
            continue
        try:
            with paths[-1].open(newline='', encoding='utf-8') as stream:
                rows = list(csv.DictReader(stream))
        except OSError:
            continue
        segments, current = [], []
        previous_time, previous_point = None, None
        for row in rows:
            try:
                if tf_row is not None:
                    point = _apply_transform(tf_row, (
                        float(row['pose_x']), float(row['pose_y'])))
                    timestamp = float(row.get('received_ros_time_s') or 0.0)
                else:
                    timestamp = float(row['header_stamp'])
                    x, y, _, _ = _project_odom_row(
                        row, _map_to_odom_at(corrections, timestamp))
                    point = (x, y)
            except (KeyError, TypeError, ValueError):
                continue
            gap = timestamp - previous_time if previous_time is not None else 0.0
            jump = (math.hypot(point[0] - previous_point[0],
                               point[1] - previous_point[1])
                    if previous_point is not None else 0.0)
            if current and (gap > max_gap_s or jump > max_jump_m):
                if len(current) >= 2:
                    segments.append(current)
                current = []
            current.append(point)
            previous_time, previous_point = timestamp, point
        if len(current) >= 2:
            segments.append(current)
        if segments:
            result[robot] = {
                'source': (
                    f'{paths[-1]} projected with captured '
                    f'{robot}/map<-{robot}/odom'
                    if tf_row is not None else
                    f'{paths[-1]} projected with recorded scan-matching '
                    f'{robot}/map<-{robot}/odom'),
                'segments': segments,
                'point_count': sum(len(segment) for segment in segments),
            }
    return result


def _draw_line(rgb, start, end, color, width=1):
    """Draw a bounded anti-aliased-free line into an RGB array."""
    height, width_pixels, _ = rgb.shape
    x0, y0 = start
    x1, y1 = end
    steps = max(abs(x1 - x0), abs(y1 - y0), 1)
    radius = max(0, int(width) // 2)
    for index in range(steps + 1):
        fraction = index / steps
        x = int(round(x0 + (x1 - x0) * fraction))
        y = int(round(y0 + (y1 - y0) * fraction))
        if x < -radius or y < -radius or x >= width_pixels + radius or y >= height + radius:
            continue
        x_min, x_max = max(0, x - radius), min(width_pixels, x + radius + 1)
        y_min, y_max = max(0, y - radius), min(height, y + radius + 1)
        rgb[y_min:y_max, x_min:x_max] = color


def _draw_disk(rgb, center, radius, color):
    height, width, _ = rgb.shape
    cx, cy = center
    x_min, x_max = max(0, cx - radius), min(width, cx + radius + 1)
    y_min, y_max = max(0, cy - radius), min(height, cy + radius + 1)
    if x_min >= x_max or y_min >= y_max:
        return
    yy, xx = np.ogrid[y_min:y_max, x_min:x_max]
    mask = (xx - cx) ** 2 + (yy - cy) ** 2 <= radius ** 2
    view = rgb[y_min:y_max, x_min:x_max]
    view[mask] = color


def _draw_text(rgb, origin, text, color, scale):
    """Draw the tiny fixed-font labels R1/R2 without Pillow."""
    x, y = origin
    glyph_width = 5 * scale
    for character in text:
        glyph = _FONT.get(character)
        if glyph is not None:
            for row, bits in enumerate(glyph):
                for column, bit in enumerate(bits):
                    if bit == '1':
                        x0 = x + column * scale
                        y0 = y + row * scale
                        rgb[y0:y0 + scale, x0:x0 + scale] = color
        x += glyph_width + scale


def render_map_with_poses(data, geometry, poses, free_threshold=25,
                          occupied_threshold=65, scale=4,
                          margin_cells=12):
    """Render an occupancy map with a non-clipped shared-map pose overlay."""
    if scale < 1 or margin_cells < 0:
        raise ValueError('scale must be at least one and margin non-negative')
    base = occupancy_rgb(
        data, free_threshold, occupied_threshold, scale=scale)
    margin = int(margin_cells) * scale
    canvas = np.full(
        (base.shape[0] + 2 * margin, base.shape[1] + 2 * margin, 3),
        UNKNOWN_RGB, dtype=np.uint8)
    canvas[margin:margin + base.shape[0], margin:margin + base.shape[1]] = base

    def image_point(pose):
        delta_x = float(pose['x_m']) - geometry.origin_x
        delta_y = float(pose['y_m']) - geometry.origin_y
        cosine, sine = math.cos(geometry.yaw), math.sin(geometry.yaw)
        local_x = cosine * delta_x + sine * delta_y
        local_y = -sine * delta_x + cosine * delta_y
        return (
            int(round(margin + local_x / geometry.resolution * scale)),
            int(round(margin + (geometry.height - local_y / geometry.resolution)
                      * scale)),
        )

    rendered = {}
    for robot in ('robot1', 'robot2'):
        pose = poses.get(robot)
        if pose is None:
            continue
        point = image_point(pose)
        color = POSE_COLORS[robot]
        arrow_length = max(
            18 * scale,
            int(round(0.65 / geometry.resolution * scale)))
        image_yaw = -float(pose.get('yaw_rad', 0.0))
        tip = (
            int(round(point[0] + arrow_length * math.cos(image_yaw))),
            int(round(point[1] + arrow_length * math.sin(image_yaw))),
        )
        _draw_line(canvas, point, tip, POSE_OUTLINE_RGB, width=max(5, scale * 3))
        _draw_line(canvas, point, tip, color, width=max(2, scale))
        _draw_disk(canvas, tip, max(3, scale * 2), POSE_OUTLINE_RGB)
        _draw_disk(canvas, tip, max(2, scale), color)
        _draw_disk(canvas, point, max(6, scale * 4), POSE_OUTLINE_RGB)
        _draw_disk(canvas, point, max(4, scale * 3), color)
        label_x = point[0] + max(8, scale * 5)
        label_y = point[1] - max(10, scale * 8)
        label_width = 2 * (5 * scale + scale)
        label_x = max(2, min(canvas.shape[1] - label_width - 2, label_x))
        label_y = max(2, min(canvas.shape[0] - 7 * scale - 2, label_y))
        _draw_text(canvas, (label_x, label_y),
                   'R1' if robot == 'robot1' else 'R2', color, scale)
        rendered[robot] = {
            **pose,
            'image_x_px': point[0],
            'image_y_px': point[1],
            'heading_tip_x_px': tip[0],
            'heading_tip_y_px': tip[1],
            'in_map_bounds': (
                margin <= point[0] <= canvas.shape[1] - margin
                and margin <= point[1] <= canvas.shape[0] - margin),
            'label': 'R1' if robot == 'robot1' else 'R2',
        }
    return canvas, rendered


def render_map_with_paths(data, geometry, poses, paths, free_threshold=25,
                          occupied_threshold=65, scale=4,
                          margin_cells=12):
    """Render a map, full shared-map trajectories, and final pose overlays.

    Unlike the historical fixed-margin renderer, this variant grows the
    canvas on whichever side is needed to contain every recorded trajectory
    point.  The occupancy data retain their original origin and orientation;
    only presentation padding is added around them.
    """
    if scale < 1 or margin_cells < 0:
        raise ValueError('scale must be at least one and margin non-negative')
    base = occupancy_rgb(
        data, free_threshold, occupied_threshold, scale=scale)
    height, width = np.asarray(data).shape
    cosine, sine = math.cos(geometry.yaw), math.sin(geometry.yaw)

    def local_cells(point):
        delta_x = float(point[0]) - geometry.origin_x
        delta_y = float(point[1]) - geometry.origin_y
        return (
            (cosine * delta_x + sine * delta_y) / geometry.resolution,
            (-sine * delta_x + cosine * delta_y) / geometry.resolution,
        )

    overlay_points = []
    for pose in poses.values():
        overlay_points.append(local_cells((pose['x_m'], pose['y_m'])))
    for value in paths.values():
        segments = value.get('segments', []) if isinstance(value, dict) else value
        for segment in segments:
            overlay_points.extend(local_cells(point) for point in segment)
    if overlay_points:
        minimum_x = min(point[0] for point in overlay_points)
        maximum_x = max(point[0] for point in overlay_points)
        minimum_y = min(point[1] for point in overlay_points)
        maximum_y = max(point[1] for point in overlay_points)
    else:
        minimum_x = maximum_x = minimum_y = maximum_y = 0.0

    left_cells = int(margin_cells) + max(0, math.ceil(-minimum_x))
    right_cells = int(margin_cells) + max(0, math.ceil(maximum_x - width))
    top_cells = int(margin_cells) + max(0, math.ceil(maximum_y - height))
    bottom_cells = int(margin_cells) + max(0, math.ceil(-minimum_y))
    top = top_cells * scale
    left = left_cells * scale
    canvas = np.full(
        (base.shape[0] + (top_cells + bottom_cells) * scale,
         base.shape[1] + (left_cells + right_cells) * scale, 3),
        UNKNOWN_RGB, dtype=np.uint8)
    canvas[top:top + base.shape[0], left:left + base.shape[1]] = base

    def image_point(point):
        local_x, local_y = local_cells(point)
        return (
            int(round(left + local_x * scale)),
            int(round(top + (height - local_y) * scale)),
        )

    path_records = {}
    for robot in ('robot1', 'robot2'):
        value = paths.get(robot)
        if value is None:
            continue
        segments = value.get('segments', []) if isinstance(value, dict) else value
        rendered_segments = 0
        for segment in segments:
            if len(segment) < 2:
                continue
            for start, end in zip(segment, segment[1:]):
                start_point = image_point(start)
                end_point = image_point(end)
                _draw_line(
                    canvas, start_point, end_point, PATH_OUTLINE_RGB,
                    width=max(4, scale * 3))
                _draw_line(
                    canvas, start_point, end_point, POSE_COLORS[robot],
                    width=max(2, scale))
            rendered_segments += 1
        path_records[robot] = {
            'source': value.get('source') if isinstance(value, dict) else None,
            'point_count': int(value.get('point_count', 0))
            if isinstance(value, dict) else sum(len(s) for s in segments),
            'segment_count': len(segments),
            'rendered_segment_count': rendered_segments,
        }

    rendered_poses = {}
    for robot in ('robot1', 'robot2'):
        pose = poses.get(robot)
        if pose is None:
            continue
        point = image_point((pose['x_m'], pose['y_m']))
        color = POSE_COLORS[robot]
        arrow_length = max(
            18 * scale,
            int(round(0.65 / geometry.resolution * scale)))
        image_yaw = -float(pose.get('yaw_rad', 0.0))
        tip = (
            int(round(point[0] + arrow_length * math.cos(image_yaw))),
            int(round(point[1] + arrow_length * math.sin(image_yaw))),
        )
        _draw_line(canvas, point, tip, POSE_OUTLINE_RGB, width=max(5, scale * 3))
        _draw_line(canvas, point, tip, color, width=max(2, scale))
        _draw_disk(canvas, tip, max(3, scale * 2), POSE_OUTLINE_RGB)
        _draw_disk(canvas, tip, max(2, scale), color)
        _draw_disk(canvas, point, max(6, scale * 4), POSE_OUTLINE_RGB)
        _draw_disk(canvas, point, max(4, scale * 3), color)
        label_x = point[0] + max(8, scale * 5)
        label_y = point[1] - max(10, scale * 8)
        label_width = 2 * (5 * scale + scale)
        label_x = max(2, min(canvas.shape[1] - label_width - 2, label_x))
        label_y = max(2, min(canvas.shape[0] - 7 * scale - 2, label_y))
        _draw_text(canvas, (label_x, label_y),
                   'R1' if robot == 'robot1' else 'R2', color, scale)
        local_x, local_y = local_cells((pose['x_m'], pose['y_m']))
        rendered_poses[robot] = {
            **pose,
            'image_x_px': point[0],
            'image_y_px': point[1],
            'heading_tip_x_px': tip[0],
            'heading_tip_y_px': tip[1],
            'in_map_bounds': 0.0 <= local_x <= width
            and 0.0 <= local_y <= height,
            'label': 'R1' if robot == 'robot1' else 'R2',
        }
    return canvas, rendered_poses, path_records


def difference_rgb(first, second, scale=4):
    """Render equal cells light gray and every exact difference red."""
    if first.shape != second.shape:
        raise ValueError('difference image requires equal array shapes')
    rgb = np.empty(first.shape + (3,), dtype=np.uint8)
    rgb[:] = AGREEMENT_RGB
    rgb[first != second] = DIFFERENCE_RGB
    rgb = np.flipud(rgb)
    if scale > 1:
        rgb = np.repeat(np.repeat(rgb, scale, axis=0), scale, axis=1)
    return rgb


def latest_campaign(results_root):
    """Return the newest completed or active regression campaign directory."""
    candidates = sorted(
        path for path in Path(results_root).glob('regression_*')
        if (path / 'campaign_progress.json').is_file()
    )
    candidates.extend(
        path for path in Path(results_root).iterdir()
        if path.is_dir()
        and not path.name.startswith('regression_')
        and any(_fast_trial_map_dir(trial) is not None
                for trial in path.glob('fast_trial_*'))
    )
    if not candidates:
        raise ValueError(f'no regression campaign found in {results_root}')
    return candidates[-1]


def _fast_trial_map_dir(trial):
    """Return the forensic map directory for a flat fast-trial result."""
    trial = Path(trial)
    candidates = sorted(trial.glob('observer/*/forensic/maps'))
    candidates.extend(sorted(trial.glob('**/forensic/maps')))
    for candidate in candidates:
        if ((candidate / 'robot1_shared_map_final.npz').is_file() and
                (candidate / 'robot2_shared_map_final.npz').is_file()) or (
                (candidate / 'robot1_map_final.npz').is_file() and
                (candidate / 'robot2_map_final.npz').is_file()):
            return candidate
    return None


def _fast_trial_attempts(campaign):
    """Return flat fast-trial directories that have final shared maps."""
    return sorted(
        trial for trial in Path(campaign).glob('fast_trial_*')
        if trial.is_dir() and _fast_trial_map_dir(trial) is not None
    )


def _final_map_path(attempt, robot, shared=True):
    """Resolve final shared or local map artifacts."""
    direct = (Path(attempt) / f'{robot}_final_shared_map.npz'
              if shared else Path(attempt) / f'{robot}_map_final.npz')
    if direct.is_file():
        return direct
    stem = 'shared_map_final' if shared else 'map_final'
    candidates = sorted(Path(attempt).glob(f'**/forensic/maps/{robot}_{stem}.npz'))
    if candidates:
        return candidates[-1]
    raise ValueError(
        f'{attempt} has no final {"shared" if shared else "local"} map for {robot}')


def selected_attempt(campaign, trial_id=None, allow_incomplete=False):
    """Resolve a selected trial, optionally including an interrupted attempt."""
    campaign = Path(campaign)
    # A fast trial can be passed directly (for example by ``webotsreport``)
    # rather than through a campaign-progress file.  Treat its own forensic
    # directory as the attempt and preserve the local-only no-handoff mode.
    if (campaign / 'forensic' / 'maps').is_dir():
        local_pair = all((campaign / 'forensic' / 'maps' /
                          f'{robot}_map_final.npz').is_file()
                         for robot in ('robot1', 'robot2'))
        shared_pair = all((campaign / 'forensic' / 'maps' /
                           f'{robot}_shared_map_final.npz').is_file()
                          for robot in ('robot1', 'robot2'))
        if local_pair or shared_pair:
            return campaign, trial_id or campaign.name
    progress_path = campaign / 'campaign_progress.json'
    if not progress_path.is_file():
        fast_trials = _fast_trial_attempts(campaign)
        if trial_id:
            fast_trials = [path for path in fast_trials
                           if path.name == trial_id]
        if fast_trials:
            return fast_trials[-1], fast_trials[-1].name
        raise ValueError(f'{campaign} has no campaign progress')
    progress = json.loads(progress_path.read_text(encoding='utf-8'))
    selected = progress.get('valid_trials', {})
    if selected:
        key = trial_id or sorted(selected)[-1]
        if key in selected:
            return campaign / selected[key], key
        if not allow_incomplete:
            raise ValueError(f'{key} is not a selected valid trial')
    elif not allow_incomplete:
        raise ValueError(f'{campaign} has no valid selected trial')

    if not allow_incomplete:
        raise ValueError(f'{campaign} has no valid selected trial')
    if not trial_id:
        raise ValueError(
            '--allow-incomplete requires --trial-id to identify the attempt')
    attempts = sorted(
        path for path in (campaign / 'attempts').glob(
            f'{trial_id}_attempt_*') if path.is_dir())
    if not attempts:
        raise ValueError(f'{trial_id} has no attempt directory')
    attempt = attempts[-1]
    _final_map_path(attempt, 'robot1')
    _final_map_path(attempt, 'robot2')
    return attempt, trial_id


def export_maps(campaign, output_dir, trial_id=None, scale=4,
                include_robot_maps=True, allow_incomplete=False,
                draw_poses=True, margin_cells=12, draw_paths=True):
    """Export map PNGs, pose overlays, trajectory overlays, and a manifest."""
    campaign = Path(campaign).resolve()
    output = Path(output_dir).resolve()
    attempt, selected_trial = selected_attempt(
        campaign, trial_id, allow_incomplete=allow_incomplete)
    def has_map(robot, shared):
        stem = 'shared_map_final' if shared else 'map_final'
        return ((Path(attempt) / f'{robot}_{stem}.npz').is_file() or
                any(Path(attempt).glob(f'**/forensic/maps/{robot}_{stem}.npz')))

    shared_available = all(has_map(robot, True) for robot in ('robot1', 'robot2'))
    # Resolve the map pair without treating a missing shared map as an error:
    # a no-handoff run is expected to contain only local final maps.
    if shared_available:
        robot1 = load_map(_final_map_path(attempt, 'robot1', shared=True))
        robot2 = load_map(_final_map_path(attempt, 'robot2', shared=True))
        canonical_path = attempt / 'canonical_final_map.npz'
        if canonical_path.is_file():
            canonical = load_map(canonical_path)
        else:
            canonical, _, _ = canonical_map(robot1, robot2)
        poses = load_final_shared_poses(attempt) if draw_poses else {}
        paths = load_shared_map_paths(attempt) if draw_paths else {}
        path_frame = 'shared_map'
        status_label = 'HANDOFF — SHARED MAP AVAILABLE'
    else:
        robot1 = load_map(_final_map_path(attempt, 'robot1', shared=False))
        robot2 = load_map(_final_map_path(attempt, 'robot2', shared=False))
        # Local maps are independent coordinate frames before handoff and
        # must never be merged or compared as though they shared a frame.
        canonical = robot1
        poses = load_local_map_poses(attempt) if draw_poses else {}
        paths = load_local_map_paths(attempt) if draw_paths else {}
        path_frame = 'robotN/map (projected from robotN/odom)'
        status_label = 'NO_HANDOFF — SHARED MAP UNAVAILABLE'
    same_geometry = (shared_available and robot1.geometry == robot2.geometry)
    exact_equal = (same_geometry and np.array_equal(robot1.data, robot2.data))
    output.mkdir(parents=True, exist_ok=False)
    written = []
    pose_records = {}

    def save(name, image, records=None):
        path = output / name
        write_rgb_png(path, image)
        written.append(str(path))
        if records:
            pose_records[name] = records

    # Before handoff the two local maps are independent coordinate frames;
    # never draw robot2's local pose/path on robot1's map.  The canonical
    # image is robot1's local map in this mode, while each robot's own image
    # below receives its own trajectory.
    canonical_poses = (poses if shared_available else
                       {'robot1': poses['robot1']}
                       if 'robot1' in poses else {})
    canonical_paths = (paths if shared_available else
                       {'robot1': paths['robot1']}
                       if 'robot1' in paths else {})
    image, records = render_map_with_poses(
        canonical.data, canonical.geometry, canonical_poses, scale=scale,
        margin_cells=margin_cells)
    save('final_merged_map.png', image, records)
    if draw_paths:
        image, records, path_records = render_map_with_paths(
            canonical.data, canonical.geometry, canonical_poses,
            canonical_paths, scale=scale,
            margin_cells=margin_cells)
        save('final_merged_map_with_paths.png', image, records)
    else:
        path_records = {}
    if include_robot_maps:
        if not shared_available:
            for robot, local_map in (('robot1', robot1), ('robot2', robot2)):
                local_poses = {robot: poses[robot]} if robot in poses else {}
                local_paths = {robot: paths[robot]} if robot in paths else {}
                image, records = render_map_with_poses(
                    local_map.data, local_map.geometry, local_poses,
                    scale=scale, margin_cells=margin_cells)
                save(f'{robot}_local_map.png', image, records)
                if draw_paths:
                    image, records, local_path_records = render_map_with_paths(
                        local_map.data, local_map.geometry, local_poses,
                        local_paths, scale=scale, margin_cells=margin_cells)
                    save(f'{robot}_local_map_with_paths.png', image, records)
                    path_records[robot] = local_path_records.get(robot, {})
            label_path = output / 'NO_HANDOFF_SHARED_MAP_UNAVAILABLE.txt'
            label_path.write_text(status_label + '\n', encoding='utf-8')
            written.append(str(label_path))
        else:
            image, records = render_map_with_poses(
                robot1.data, robot1.geometry, poses, scale=scale,
                margin_cells=margin_cells)
            save('robot1_final_shared_map.png', image, records)
            image, records = render_map_with_poses(
                robot2.data, robot2.geometry, poses, scale=scale,
                margin_cells=margin_cells)
            save('robot2_final_shared_map.png', image, records)
            if draw_paths:
                image, records, robot_path_records = render_map_with_paths(
                    robot1.data, robot1.geometry, poses, paths, scale=scale,
                    margin_cells=margin_cells)
                save('robot1_final_shared_map_with_paths.png', image, records)
                image, records, _ = render_map_with_paths(
                    robot2.data, robot2.geometry, poses, paths, scale=scale,
                    margin_cells=margin_cells)
                save('robot2_final_shared_map_with_paths.png', image, records)
                # All robot map replicas use the same shared-map coordinate frame;
                # retain one compact path manifest rather than duplicating it.
                path_records = robot_path_records
            if same_geometry:
                save('robot1_robot2_exact_difference.png',
                     difference_rgb(robot1.data, robot2.data, scale=scale))
    manifest = {
        'schema_version': '1.0.0',
        'campaign_id': campaign.name,
        'trial_id': selected_trial,
        'attempt_id': attempt.name,
        'source_attempt': str(attempt),
        'output_directory': str(output),
        'scale': scale,
        'margin_cells': int(margin_cells),
        'pose_overlay_enabled': bool(draw_poses),
        'path_overlay_enabled': bool(draw_paths),
        'status_label': status_label,
        'handoff_occurred': shared_available,
        'path_frame': path_frame,
        'path_projection': (
            'shared_map coordinates projected through the OccupancyGrid '
            'origin translation and inverse origin yaw exactly once'),
        'path_source': {
            robot: value.get('source') for robot, value in paths.items()
        },
        'path_overlays': path_records,
        'pose_source': {
            robot: pose.get('source') for robot, pose in poses.items()
        },
        'final_shared_map_poses': pose_records.get('final_merged_map.png', {}),
        'legend': {
            'unknown': list(UNKNOWN_RGB),
            'free': list(FREE_RGB),
            'occupied': list(OCCUPIED_RGB),
            'uncertain': list(UNCERTAIN_RGB),
            'exact_difference': list(DIFFERENCE_RGB),
        },
        'robot_maps_same_geometry': same_geometry,
        'robot_maps_exactly_identical': exact_equal,
        'different_cell_count': (
            int(np.count_nonzero(robot1.data != robot2.data))
            if same_geometry else None
        ),
        'map_geometry': {
            'width': canonical.geometry.width,
            'height': canonical.geometry.height,
            'resolution_m': canonical.geometry.resolution,
            'origin_x_m': canonical.geometry.origin_x,
            'origin_y_m': canonical.geometry.origin_y,
            'origin_yaw_rad': canonical.geometry.yaw,
        },
        'files': written,
    }
    manifest_path = output / 'export_manifest.json'
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    return manifest


def parser():
    """Build the command-line parser."""
    result = argparse.ArgumentParser(
        description='Export cooperative regression maps to PNG')
    result.add_argument(
        '--campaign',
        help='Campaign directory; defaults to newest under --results-root')
    result.add_argument('--results-root', default='results')
    result.add_argument('--trial-id', help='Defaults to last selected trial')
    result.add_argument(
        '--allow-incomplete', action='store_true',
        help='Export an explicitly selected interrupted attempt when map '
             'artifacts are present.')
    result.add_argument('--output-dir', required=True)
    result.add_argument('--scale', type=int, default=4)
    result.add_argument('--margin-cells', type=int, default=12)
    result.add_argument(
        '--draw-poses', action=argparse.BooleanOptionalAction, default=True,
        help='Draw final robot poses and heading arrows from forensic TF data.')
    result.add_argument(
        '--draw-paths', action=argparse.BooleanOptionalAction, default=True,
        help='Draw the complete recorded shared-map trajectories.')
    result.add_argument(
        '--include-robot-maps', action=argparse.BooleanOptionalAction,
        default=True)
    return result


def main(argv=None):
    """Run the command-line exporter."""
    args = parser().parse_args(argv)
    campaign = (
        Path(args.campaign) if args.campaign
        else latest_campaign(args.results_root)
    )
    try:
        manifest = export_maps(
            campaign, args.output_dir, args.trial_id, args.scale,
            args.include_robot_maps, args.allow_incomplete,
            args.draw_poses, args.margin_cells, args.draw_paths)
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as error:
        print(f'EXPORT_ERROR: {error}')
        return 2
    print(f'output_directory={manifest["output_directory"]}')
    print(
        'robot_maps_exactly_identical='
        f'{manifest["robot_maps_exactly_identical"]}')
    for path in manifest['files']:
        print(f'png={path}')
    return 0
