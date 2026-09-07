"""Deterministic offline reconstruction of the thin raw-evidence contract.

This module intentionally owns derived metrics only after the mission has
stopped.  In particular, map streams are kept source-aware: ``map`` is never
silently combined with ``shared_map``.  The established definitions are:

* A: robot-local map;
* B: transformed union of robot-local maps;
* C: shared map (robot1's replicated stream is the canonical curve, with the
  peer stream retained for equivalence diagnostics).
"""

from __future__ import annotations

import bisect
import csv
import json
import math
import re
from pathlib import Path


def _stamp(message):
    header = getattr(message, 'header', None)
    if header is None:
        return None
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1.0e-9


def _clock_for_bag(clock_rows, bag_time_ns):
    if not clock_rows:
        return None
    index = bisect.bisect_right(
        clock_rows, (int(bag_time_ns), float('inf'))) - 1
    return float(clock_rows[index][1]) if index >= 0 else None


def _read_ground_truth(path):
    if not Path(path).is_file():
        return []
    with Path(path).open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def _ground_truth_metrics(rows):
    by_robot = {}
    for row in rows:
        robot = str(row.get('robot_id', ''))
        try:
            item = (float(row['sim_time_s']), float(row['world_x_m']),
                    float(row['world_y_m']),
                    float(row.get('ground_speed_mps', 0.0)))
        except (KeyError, TypeError, ValueError):
            continue
        by_robot.setdefault(robot, []).append(item)
    distances = {}
    active = {}
    for robot, values in by_robot.items():
        values.sort()
        distances[robot] = sum(
            math.hypot(second[1] - first[1], second[2] - first[2])
            for first, second in zip(values, values[1:]))
        active[robot] = sum(
            max(0.0, second[0] - first[0])
            for first, second in zip(values, values[1:])
            if first[3] >= 0.02)
    return {
        'sample_rows': len(rows),
        'robots': {
            robot: {'samples': len(values),
                    'travel_distance_m': distances.get(robot, 0.0),
                    'active_time_s': active.get(robot, 0.0),
                    'idle_time_s': max(
                        0.0, (values[-1][0] - values[0][0]) -
                        active.get(robot, 0.0)) if values else 0.0}
            for robot, values in by_robot.items()
        },
    }


def _map_record(message, sim_time, topic, retain_data=True):
    """Make a compact-but-replayable occupancy snapshot."""
    import numpy as np

    info = message.info
    data = np.asarray(list(message.data), dtype=np.int16)
    known = int(np.count_nonzero(data >= 0))
    occupied = int(np.count_nonzero(data >= 50))
    q = info.origin.orientation
    origin_yaw = math.atan2(
        2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y)),
        1.0 - 2.0 * (float(q.y) ** 2 + float(q.z) ** 2))
    return {
        'sim_time_s': float(sim_time),
        'topic': topic,
        'known_cells': known,
        'occupied_cells': occupied,
        'unknown_cells': int(np.count_nonzero(data < 0)),
        'resolution': float(info.resolution),
        'width': int(info.width),
        'height': int(info.height),
        'origin_x': float(info.origin.position.x),
        'origin_y': float(info.origin.position.y),
        'origin_yaw': origin_yaw,
        # C/A coverage needs only counts.  Keeping every historical grid in
        # memory makes a long bag needlessly expensive; B's local-map union
        # explicitly opts into cell retention.
        'data': data if retain_data else None,
    }


def _read_bag(run_directory: Path, robots, retain_map_data=False):
    """Replay the raw bag without creating a second raw artifact."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag = Path(run_directory) / 'passive_rosbag'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions(
                    input_serialization_format='cdr',
                    output_serialization_format='cdr'))
    type_map = {item.name: item.type
                for item in reader.get_all_topics_and_types()}
    clock_rows = []
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic != '/clock':
            continue
        message = deserialize_message(data, get_message(type_map[topic]))
        value = message.clock.sec + message.clock.nanosec * 1.0e-9
        clock_rows.append((int(timestamp), float(value)))
    clock_rows.sort()

    wanted = {'/clock'}
    for robot in robots:
        wanted.update({
            f'/{robot}/odom', f'/{robot}/map', f'/{robot}/shared_map',
            f'/{robot}/navigate_to_pose/_action/status',
        })
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions(
                    input_serialization_format='cdr',
                    output_serialization_format='cdr'))
    odom = {robot: [] for robot in robots}
    map_series = {
        robot: {'local': [], 'shared': []} for robot in robots
    }
    goal_status = {robot: {'messages': 0, 'terminal_by_goal': {}}
                   for robot in robots}
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic not in wanted or topic == '/clock':
            continue
        type_name = type_map.get(topic)
        if not type_name:
            continue
        message = deserialize_message(data, get_message(type_name))
        sim_time = _clock_for_bag(clock_rows, timestamp)
        if sim_time is None:
            sim_time = _stamp(message)
        if sim_time is None:
            continue
        for robot in robots:
            if topic == f'/{robot}/odom':
                pose = message.pose.pose.position
                values = odom[robot]
                distance = (math.hypot(float(pose.x) - values[-1][1],
                                       float(pose.y) - values[-1][2])
                            if values else 0.0)
                values.append((float(sim_time), float(pose.x),
                                float(pose.y), distance))
            elif topic in (f'/{robot}/map', f'/{robot}/shared_map'):
                kind = 'local' if topic.endswith('/map') else 'shared'
                map_series[robot][kind].append(
                    _map_record(message, sim_time, topic,
                                retain_data=retain_map_data))
            elif topic == f'/{robot}/navigate_to_pose/_action/status':
                state = goal_status[robot]
                state['messages'] += 1
                for item in message.status_list:
                    goal_id = bytes(item.goal_info.goal_id.uuid).hex()
                    status = int(item.status)
                    previous = state['terminal_by_goal'].get(goal_id)
                    if status in (4, 5, 6) and previous not in (4, 5, 6):
                        state['terminal_by_goal'][goal_id] = status

    motion = {}
    for robot, values in odom.items():
        motion[robot] = {
            'samples': len(values),
            'travel_distance_m': sum(item[3] for item in values),
            'start_sim_s': values[0][0] if values else None,
            'end_sim_s': values[-1][0] if values else None,
        }
    navigation = {}
    for robot, state in goal_status.items():
        terminals = list(state['terminal_by_goal'].values())
        navigation[robot] = {
            'status_messages': state['messages'],
            'terminal_goals': len(terminals),
            'succeeded': terminals.count(4),
            'canceled': terminals.count(5),
            'aborted': terminals.count(6),
        }
    return {
        'run_directory': str(Path(run_directory)),
        'clock': {
            'message_count': len(clock_rows),
            'start_s': clock_rows[0][1] if clock_rows else None,
            'end_s': clock_rows[-1][1] if clock_rows else None,
        },
        'motion': motion,
        'map_series': map_series,
        'navigation': navigation,
        'topic_types': type_map,
    }


def _protocol_replay(run_directory, robots):
    """Compatibility wrapper for the modular protocol replay."""
    from .offline_protocol_replay import replay_protocol

    return replay_protocol(run_directory, robots)
def _load_forensic_maps(run_directory, bag, robots):
    """Load preserved legacy forensic NPZ maps when the old bag omitted maps."""
    import numpy as np

    directory = Path(run_directory) / 'forensic' / 'maps'
    pattern = re.compile(
        r'^sim_(\d+)_(\d+)_robot([^_]+)_(map|shared_map)\.npz$')
    for path in sorted(directory.glob('sim_*_robot*_*.npz')):
        match = pattern.match(path.name)
        if not match:
            continue
        robot, kind = 'robot' + match.group(3), match.group(4)
        if robot not in robots:
            continue
        try:
            archive = np.load(path, allow_pickle=False)
            data = np.asarray(archive['occupancy'], dtype=np.int16)
            metadata = json.loads(str(archive['metadata_json'].item()))
            record = {
                'sim_time_s': float(metadata.get(
                    'header_stamp_s', f'{match.group(1)}.{match.group(2)}')),
                'topic': metadata.get('topic', f'/{robot}/{kind}'),
                'known_cells': int(np.count_nonzero(data >= 0)),
                'occupied_cells': int(np.count_nonzero(data >= 50)),
                'unknown_cells': int(np.count_nonzero(data < 0)),
                'resolution': float(metadata['resolution']),
                'width': int(metadata['width']),
                'height': int(metadata['height']),
                'origin_x': float(metadata['origin']['position']['x']),
                'origin_y': float(metadata['origin']['position']['y']),
                'origin_yaw': 0.0,
                'data': data,
            }
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
            continue
        bag['map_series'][robot]['local' if kind == 'map' else 'shared'].append(record)
    # Final snapshots have no ``sim_<stamp>`` prefix.  They are still raw map
    # evidence and must be included when the periodic stream stopped before
    # the recorder finalized.
    final_pattern = re.compile(r'^([^_]+)_(map|shared_map)_final\.npz$')
    for path in sorted(directory.glob('robot*_*_final.npz')):
        match = final_pattern.match(path.name)
        if not match or match.group(1) not in robots:
            continue
        try:
            archive = np.load(path, allow_pickle=False)
            data = np.asarray(archive['occupancy'], dtype=np.int16)
            metadata = json.loads(str(archive['metadata_json'].item()))
            kind = match.group(2)
            bag['map_series'][match.group(1)][
                'local' if kind == 'map' else 'shared'].append({
                    'sim_time_s': float(metadata['header_stamp_s']),
                    'topic': metadata.get('topic',
                                          f'/{match.group(1)}/{kind}'),
                    'known_cells': int(np.count_nonzero(data >= 0)),
                    'occupied_cells': int(np.count_nonzero(data >= 50)),
                    'unknown_cells': int(np.count_nonzero(data < 0)),
                    'resolution': float(metadata['resolution']),
                    'width': int(metadata['width']),
                    'height': int(metadata['height']),
                    'origin_x': float(metadata['origin']['position']['x']),
                    'origin_y': float(metadata['origin']['position']['y']),
                    'origin_yaw': 0.0, 'data': data,
                })
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
            continue
    return bag


def _dedupe_series(series):
    values = sorted(series, key=lambda item: item['sim_time_s'])
    output = []
    for value in values:
        comparable = (value['sim_time_s'], value['known_cells'],
                      value['occupied_cells'], value['unknown_cells'])
        if output and output[-1]['_key'] == comparable:
            continue
        output.append({
            '_key': comparable,
            'sim_time_s': value['sim_time_s'],
            'known_cells': value['known_cells'],
            'occupied_cells': value['occupied_cells'],
            'unknown_cells': value['unknown_cells'],
            'area_m2': value['known_cells'] * value['resolution'] ** 2,
            'topic': value['topic'],
        })
    for value in output:
        value.pop('_key', None)
    return output


def _auc(series):
    return sum(
        max(0.0, second['sim_time_s'] - first['sim_time_s']) *
        (first['area_m2'] + second['area_m2']) / 2.0
        for first, second in zip(series, series[1:]))


def _ground_truth_overlap(rows, bin_size=0.05):
    bins = {}
    for row in rows:
        try:
            robot = str(row['robot_id'])
            cell = (math.floor(float(row['world_x_m']) / bin_size),
                    math.floor(float(row['world_y_m']) / bin_size))
        except (KeyError, TypeError, ValueError):
            continue
        bins.setdefault(robot, set()).add(cell)
    if len(bins) < 2:
        return {'available': False, 'reason': 'fewer than two GT trajectories'}
    values = list(bins.values())
    shared = set.intersection(*values)
    union = set.union(*values)
    return {
        'available': True, 'bin_size_m': bin_size,
        'intersection_bins': len(shared), 'union_bins': len(union),
        'iou': len(shared) / len(union) if union else 0.0,
    }


def _contact_metrics(run_directory):
    path = Path(run_directory) / 'forensic' / 'contact_points.csv'
    if not path.exists():
        return {'available': False, 'reason': 'contact_points.csv absent'}
    by_robot = {}
    try:
        with path.open(newline='', encoding='utf-8') as stream:
            for row in csv.DictReader(stream):
                robot = str(row.get('robot_id', ''))
                try:
                    time_s = float(row['sim_time_s'])
                except (KeyError, TypeError, ValueError):
                    continue
                by_robot.setdefault(robot, set()).add(time_s)
    except OSError as exc:
        return {'available': False, 'reason': str(exc)}
    episodes = {}
    for robot, times in by_robot.items():
        ordered = sorted(times)
        count = 0
        previous = None
        for value in ordered:
            if previous is None or value - previous > 0.041:
                count += 1
            previous = value
        episodes[robot] = {'contact_samples': len(ordered),
                           'contact_episodes': count}
    return {'available': True, 'robots': episodes,
            'total_contact_samples': sum(
                item['contact_samples'] for item in episodes.values())}


def _warning_metrics(run_directory):
    run_directory = Path(run_directory)
    path = run_directory / 'warnings.jsonl'
    if not path.exists():
        receipt_path = run_directory / 'rosout_receipts.jsonl'
        if not receipt_path.is_file():
            return {
                'available': False,
                'reason': 'warnings.jsonl and rosout_receipts.jsonl absent',
            }
        try:
            from .deferred_protocol import replay_warning_records_from_receipts
            replay = replay_warning_records_from_receipts(receipt_path)
            temporary = path.with_suffix('.jsonl.replay.tmp')
            with temporary.open('w', encoding='utf-8') as stream:
                for record in replay['records']:
                    stream.write(json.dumps(record, sort_keys=True) + '\n')
            temporary.replace(path)
        except (OSError, RuntimeError, ValueError, TypeError,
                json.JSONDecodeError) as exc:
            return {
                'available': False,
                'reason': f'rosout receipt replay failure: {exc}',
            }
        counts = {}
        for record in replay['records']:
            category = str(record.get('category', 'PROCESS_WARNING'))
            counts[category] = counts.get(category, 0) + int(
                record.get('occurrence_count', 1))
        return {
            'available': True,
            'counts': counts,
            'source': 'rosout_receipts.jsonl',
            'receipt_count': replay['receipt_count'],
            'warning_receipt_count': replay['warning_receipt_count'],
            'record_count': len(replay['records']),
        }
    counts = {}
    try:
        for line in path.read_text(encoding='utf-8').splitlines():
            if not line.strip():
                continue
            row = json.loads(line)
            category = str(row.get('category', 'PROCESS_WARNING'))
            counts[category] = counts.get(category, 0) + int(
                row.get('occurrence_count', 1))
    except (OSError, ValueError, TypeError):
        return {'available': False, 'reason': 'warning parse failure'}
    return {
        'available': True,
        'counts': counts,
        'source': 'warnings.jsonl',
        'record_count': sum(1 for line in path.read_text(
            encoding='utf-8').splitlines() if line.strip()),
    }


def _known_cells(record, transform=(0.0, 0.0, 0.0)):
    """Return integer world cells for B local-map union reconstruction."""
    import numpy as np

    data = record['data']
    if data is None:
        return set()
    indices = np.flatnonzero(data >= 0)
    if not len(indices):
        return set()
    cols = indices % record['width']
    rows = indices // record['width']
    dx = (cols.astype(float) + 0.5) * record['resolution']
    dy = (rows.astype(float) + 0.5) * record['resolution']
    yaw = record['origin_yaw']
    ox = record['origin_x'] + math.cos(yaw) * dx - math.sin(yaw) * dy
    oy = record['origin_y'] + math.sin(yaw) * dx + math.cos(yaw) * dy
    tx, ty, tyaw = transform
    xs = np.floor((math.cos(tyaw) * ox - math.sin(tyaw) * oy + tx) /
                  record['resolution']).astype(np.int64)
    ys = np.floor((math.sin(tyaw) * ox + math.cos(tyaw) * oy + ty) /
                  record['resolution']).astype(np.int64)
    return set(zip(xs.tolist(), ys.tolist()))


def _manifest(run_directory):
    candidates = [Path(run_directory) / 'run_manifest.json',
                  Path(run_directory) / 'raw_evidence_manifest.json']
    for path in candidates:
        if path.exists():
            try:
                return json.loads(path.read_text(encoding='utf-8'))
            except (OSError, ValueError):
                pass
    return {}


def _condition(run_directory, metadata):
    manifest = _manifest(run_directory)
    return str(metadata.get('condition') or manifest.get('experiment_condition')
               or 'C').upper(), manifest


def _coverage(bag, robots, condition, manifest):
    condition = str(condition).upper()
    source_kind = {'A': 'local', 'B': 'local', 'C': 'shared'}.get(
        condition, 'shared')
    source_topics = {robot: f'/{robot}/{source_kind}_map'
                     for robot in robots}
    if condition == 'B':
        transforms = {robot: (0.0, 0.0, 0.0) for robot in robots}
        transform = manifest.get('known_initial_relative_transform')
        if len(robots) > 1 and isinstance(transform, list) and len(transform) >= 3:
            transforms[robots[1]] = tuple(float(value) for value in transform[:3])
        events = []
        latest_cells = {}
        first_seen = {}
        unique = {robot: 0 for robot in robots}
        duplicate = {robot: 0 for robot in robots}
        for robot in robots:
            for record in bag['map_series'][robot]['local']:
                events.append((record['sim_time_s'], robot, record))
        for sim_time, robot, record in sorted(events):
            cells = _known_cells(record, transforms[robot])
            prior = latest_cells.get(robot, set())
            for cell in cells - prior:
                if cell not in first_seen:
                    first_seen[cell] = (robot, sim_time)
                    unique[robot] += 1
                elif first_seen[cell][0] != robot:
                    duplicate[robot] += 1
            latest_cells[robot] = cells
        # Rebuild a deterministic curve from event-time latest snapshots.
        curve = []
        latest = {}
        for sim_time, robot, record in sorted(events):
            latest[robot] = _known_cells(record, transforms[robot])
            union = set().union(*latest.values()) if latest else set()
            curve.append({
                'sim_time_s': sim_time,
                'known_cells': len(union),
                'occupied_cells': None,
                'unknown_cells': None,
                'area_m2': len(union) * float(record['resolution']) ** 2,
                'topic': f'/{robot}/map',
            })
        curve = _dedupe_series(curve)
        result = {
            'source': 'local_map_union', 'source_topics': source_topics,
            'series': curve, 'snapshot_count': len(curve),
            'final_known_cells': curve[-1]['known_cells'] if curve else 0,
            'final_known_area_m2': curve[-1]['area_m2'] if curve else 0.0,
            'known_area_auc_m2_s': _auc(curve),
            'unique_first_seen_cells': unique,
            'later_duplicated_cells': duplicate,
            'total_known_union_cells': len(first_seen),
            'duplicated_known_fraction': (
                sum(duplicate.values()) / len(first_seen) if first_seen else 0.0),
        }
        result['ownership'] = _local_ownership(
            bag['map_series'], robots, manifest)
        return _coverage_milestones(result, manifest)

    # A is explicitly robot1-local; C is explicitly robot1-shared.  Keep the
    # peer curve as a diagnostic, never silently merge it into the main curve.
    canonical_robot = robots[0] if robots else 'robot1'
    selected = None
    if condition == 'C':
        selected = _load_finalized_coverage_series(
            Path(bag.get('run_directory', '')), canonical_robot)
    if selected is None:
        selected = _dedupe_series(
            bag['map_series'].get(canonical_robot, {}).get(source_kind, []))
    peer = {
        robot: _dedupe_series(bag['map_series'][robot][source_kind])
        for robot in robots if robot != canonical_robot
    }
    result = {
        'source': 'local_map' if condition == 'A' else 'shared_map',
        'source_topics': source_topics,
        'canonical_robot': canonical_robot,
        'series': selected,
        'snapshot_count': len(selected),
        'final_known_cells': selected[-1]['known_cells'] if selected else 0,
        'final_known_area_m2': selected[-1]['area_m2'] if selected else 0.0,
        'known_area_auc_m2_s': _auc(selected),
        'peer_series': peer,
        'ownership': _local_ownership(bag['map_series'], robots, manifest),
    }
    if condition == 'C' and peer:
        peer_series = next(iter(peer.values()))
        result['shared_stream_equivalence'] = {
            'peer_snapshot_count': len(peer_series),
            'peer_final_known_cells': (peer_series[-1]['known_cells']
                                       if peer_series else 0),
            'final_known_cell_difference': (
                result['final_known_cells'] -
                (peer_series[-1]['known_cells'] if peer_series else 0)),
        }
    return _coverage_milestones(result, manifest)


def _load_finalized_coverage_series(run_directory, canonical_robot):
    """Load the observer's authoritative request-time coverage artifact.

    Condition-C fusion publishes a new ``OccupancyGrid`` only when its payload
    changes.  Once exploration has reached a fixed map state, the native
    shared-map topic is therefore intentionally sparse even though the map
    state remains valid.  The legacy finalizer already materializes the exact
    request-time coverage rows from that payload plus the causal map receipts.
    Prefer that artifact for the offline curve so a change-only ROS stream is
    not mistaken for the end of the scientific time series.

    A present-but-empty or malformed artifact is an error, not a signal to
    fall back to the sparse map stream.  This keeps finalization fail-closed.
    """
    if not run_directory:
        return None
    path = Path(run_directory) / 'coverage.csv'
    if not path.is_file():
        return None
    required = {
        'ros_time_sec', 'ros_time_nanosec', 'known_area_m2',
        f'{canonical_robot}_shared_known', 'shared_occupied_cells',
        'shared_unknown_cells',
    }
    series = []
    with path.open(newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        fieldnames = set(reader.fieldnames or ())
        missing = sorted(required - fieldnames)
        if missing:
            raise ValueError(
                f'coverage.csv missing authoritative fields: {missing}')
        for row_number, row in enumerate(reader, start=2):
            try:
                sim_time = (float(row['ros_time_sec']) +
                            float(row['ros_time_nanosec']) * 1.0e-9)
                known_cells = int(float(row[
                    f'{canonical_robot}_shared_known']))
                occupied_cells = int(float(row['shared_occupied_cells']))
                unknown_cells = int(float(row['shared_unknown_cells']))
                area_m2 = float(row['known_area_m2'])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(
                    f'invalid authoritative coverage row {row_number}') from exc
            if not all(math.isfinite(value) for value in (
                    sim_time, area_m2)) or known_cells < 0:
                raise ValueError(
                    f'invalid authoritative coverage values row {row_number}')
            series.append({
                'sim_time_s': sim_time,
                'known_cells': known_cells,
                'occupied_cells': occupied_cells,
                'unknown_cells': unknown_cells,
                'area_m2': area_m2,
                'topic': f'/{canonical_robot}/shared_map',
            })
    if not series:
        raise ValueError('coverage.csv is present but contains no rows')
    ordered = sorted(series, key=lambda item: item['sim_time_s'])
    for previous, current in zip(ordered, ordered[1:]):
        if current['sim_time_s'] <= previous['sim_time_s']:
            raise ValueError('coverage.csv simulation times are not increasing')
    return ordered


def _coverage_milestones(result, manifest):
    dimensions = manifest.get('world_dimensions_m')
    extent_area = (float(dimensions[0]) * float(dimensions[1])
                   if isinstance(dimensions, list) and len(dimensions) >= 2
                   else None)
    milestones = {}
    for fraction in (0.25, 0.50, 0.75, 0.90):
        target = extent_area * fraction if extent_area is not None else None
        hit = next((item['sim_time_s'] for item in result['series']
                    if target is not None and item['area_m2'] >= target), None)
        milestones[f'{int(fraction * 100)}%'] = hit
    result['world_extent_area_m2'] = extent_area
    result['time_to_world_extent_thresholds_s'] = milestones
    result['known_cell_auc_cells_s'] = (
        result['known_area_auc_m2_s'] /
        (result['series'][-1]['area_m2'] / result['series'][-1]['known_cells'])
        if result['series'] and result['series'][-1]['known_cells']
        else None)
    return result


def _local_ownership(map_series, robots, manifest):
    """Reconstruct first-seen/duplicate ownership from raw local maps."""
    if (len(robots) < 2 or
            not all(map_series.get(robot, {}).get('local')
                    for robot in robots)):
        return {'available': False, 'reason': 'local maps incomplete'}
    if any(record.get('data') is None
           for robot in robots for record in map_series[robot]['local']):
        return {'available': False,
                'reason': 'raw local map cells not retained in this replay'}
    transforms = {robot: (0.0, 0.0, 0.0) for robot in robots}
    value = manifest.get('known_initial_relative_transform')
    if isinstance(value, list) and len(value) >= 3:
        transforms[robots[1]] = tuple(float(item) for item in value[:3])
    first = {}
    unique = {robot: 0 for robot in robots}
    later = {robot: 0 for robot in robots}
    for robot in robots:
        records = sorted(map_series[robot]['local'],
                         key=lambda item: item['sim_time_s'])
        seen = set()
        for record in records:
            cells = _known_cells(record, transforms[robot])
            for cell in cells - seen:
                if cell not in first:
                    first[cell] = (robot, record['sim_time_s'])
                    unique[robot] += 1
                elif first[cell][0] != robot:
                    later[robot] += 1
            seen.update(cells)
    return {
        'available': True,
        'unique_first_seen_cells': unique,
        'later_duplicated_cells': later,
        'total_known_union_cells': len(first),
        'duplicated_known_fraction': (
            sum(later.values()) / len(first) if first else 0.0),
        'transform_source': 'known_initial_relative_transform',
    }


def _stream_local_ownership(run_directory, robots, manifest):
    """One-pass ownership replay that does not retain all historical grids."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag = Path(run_directory) / 'passive_rosbag'
    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions(
                        input_serialization_format='cdr',
                        output_serialization_format='cdr'))
    except Exception as exc:
        return {'available': False, 'reason': f'bag open failed: {exc}'}
    type_map = {item.name: item.type
                for item in reader.get_all_topics_and_types()}
    topics = {f'/{robot}/map': robot for robot in robots}
    transforms = {robot: (0.0, 0.0, 0.0) for robot in robots}
    value = manifest.get('known_initial_relative_transform')
    if len(robots) > 1 and isinstance(value, list) and len(value) >= 3:
        transforms[robots[1]] = tuple(float(item) for item in value[:3])
    seen = {robot: set() for robot in robots}
    first = {}
    unique = {robot: 0 for robot in robots}
    later = {robot: 0 for robot in robots}
    snapshot_count = 0
    while reader.has_next():
        topic, data, _timestamp = reader.read_next()
        robot = topics.get(topic)
        if robot is None or topic not in type_map:
            continue
        message = deserialize_message(data, get_message(type_map[topic]))
        record = _map_record(message, _stamp(message) or 0.0, topic,
                             retain_data=True)
        cells = _known_cells(record, transforms[robot])
        snapshot_count += 1
        for cell in cells - seen[robot]:
            if cell not in first:
                first[cell] = (robot, record['sim_time_s'])
                unique[robot] += 1
            elif first[cell][0] != robot:
                later[robot] += 1
        seen[robot].update(cells)
    if not snapshot_count:
        return {'available': False, 'reason': 'no local map messages'}
    return {
        'available': True, 'snapshot_count': snapshot_count,
        'unique_first_seen_cells': unique,
        'later_duplicated_cells': later,
        'total_known_union_cells': len(first),
        'duplicated_known_fraction': (
            sum(later.values()) / len(first) if first else 0.0),
        'transform_source': 'known_initial_relative_transform',
    }


def _handoff_metrics(run_directory, manifest, gt_rows):
    root = Path(run_directory)
    candidates = list(root.glob('frontend/*_unknown_pose_frontend.json'))
    candidates += list(root.glob('forensic/frontend/*_unknown_pose_frontend.json'))
    # The canonical runner stores the frontend evidence beside the per-run
    # observer directory (trial/frontend), while older fixtures may keep it
    # inside the run directory.  Both are the same structured source.
    candidates += list(root.parent.glob('frontend/*_unknown_pose_frontend.json'))
    candidates += list(root.parent.parent.glob(
        'frontend/*_unknown_pose_frontend.json'))
    if root.name.endswith('-01'):
        trial_root = root.parent / root.name[:-3]
        candidates += list(trial_root.glob(
            'frontend/*_unknown_pose_frontend.json'))
    records = []
    expected = manifest.get('known_initial_relative_transform')
    for path in sorted(set(candidates)):
        try:
            payload = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            continue
        hypothesis = payload.get('accepted_hypothesis') or {}
        transform = hypothesis.get('transform_se2')
        accepted = payload.get('accepted') is True
        error = None
        if accepted and isinstance(transform, list) and len(transform) >= 3:
            source = hypothesis.get('source_robot_id')
            target = hypothesis.get('target_robot_id')
            # The manifest stores robot2 in robot1's initial frame.  The
            # frontend stores source->target frame coordinates, hence the
            # inverse convention for the common robot1->robot2 handoff.
            if (isinstance(expected, list) and len(expected) >= 3 and
                    source == 'robot1' and target == 'robot2'):
                target_transform = [-float(expected[0]), -float(expected[1]),
                                    -float(expected[2])]
                error = {
                    'translation_m': math.hypot(
                        float(transform[0]) - target_transform[0],
                        float(transform[1]) - target_transform[1]),
                    'yaw_rad': abs(float(transform[2]) - target_transform[2]),
                    'expected_source_to_target': target_transform,
                    'source': 'world-derived initial transform in run manifest',
                }
        records.append({
            'path': str(path), 'robot_id': payload.get('robot_id'),
            'accepted': accepted,
            'accepted_ros_time_s': payload.get('accepted_ros_time_s'),
            'evidence_set_hash': hypothesis.get('evidence_set_hash'),
            'transform_se2': transform,
            'quality': {key: hypothesis.get(key) for key in (
                'final_confidence', 'geometric_inlier_ratio',
                'median_registration_residual_m', 'p95_registration_residual_m',
                'occupied_free_agreement', 'overlap_fraction',
                'translation_uncertainty_m', 'yaw_uncertainty_rad')},
            'error_against_physical_gt': error,
        })
    return {
        'structured_files': records,
        'complete': bool(records) and all(item['accepted'] for item in records),
        'physical_gt_source': ('run_manifest.known_initial_relative_transform'
                               if expected else None),
    }


def evaluate_run(run_directory: Path, robots=('robot1', 'robot2'),
                 reference_summary: Path | None = None,
                 output_path: Path | None = None,
                 condition: str | None = None,
                 unknown_initial_pose: bool | None = None) -> dict:
    """Replay one finalized run and write a versioned metric artifact.

    ``output_path`` prevents parity analysis from mutating preserved fixture
    directories.  The legacy in-place default remains for the live thin
    finalizer until its caller adopts an explicit output path.
    """
    run_directory = Path(run_directory)
    validation_metadata_path = run_directory / 'raw_bag_validation.json'
    finalization_path = run_directory / 'raw_evidence_finalization.json'
    if validation_metadata_path.exists():
        metadata = json.loads(validation_metadata_path.read_text(encoding='utf-8'))
    else:
        export_metadata_path = run_directory / 'passive_rosbag_export.json'
        metadata = json.loads(export_metadata_path.read_text(encoding='utf-8'))
    resolved_condition, manifest = _condition(run_directory, metadata)
    condition = str(condition or resolved_condition).upper()
    assignment_strategy = str(
        metadata.get('assignment_strategy') or
        manifest.get('assignment_strategy') or '').strip().lower()
    if unknown_initial_pose is None:
        unknown_initial_pose = bool(metadata.get('unknown_initial_pose',
                                                condition == 'C'))
    gt_rows = _read_ground_truth(
        run_directory / 'forensic' / 'supervisor_ground_truth.csv')
    gt = _ground_truth_metrics(gt_rows)
    gt['trajectory_overlap'] = _ground_truth_overlap(gt_rows)
    bag = _read_bag(run_directory, tuple(robots), retain_map_data=(condition == 'B'))
    bag = _load_forensic_maps(run_directory, bag, tuple(robots))
    coverage = _coverage(bag, tuple(robots), condition, manifest)
    if not coverage.get('ownership', {}).get('available', False):
        coverage['ownership'] = _stream_local_ownership(
            run_directory, tuple(robots), manifest)
    finalization = (json.loads(finalization_path.read_text(encoding='utf-8'))
                    if finalization_path.exists() else {})
    handoff = _handoff_metrics(run_directory, manifest, gt_rows)
    from .offline_handoff import reciprocal_verification
    handoff['reciprocal_verification'] = reciprocal_verification(
        handoff.get('structured_files', []))
    contacts = _contact_metrics(run_directory)
    warnings = _warning_metrics(run_directory)
    semantic = (finalization.get('semantic_contract') or
                metadata.get('semantic_contract'))
    if not semantic:
        from .raw_evidence_contract import semantic_contract_status
        semantic = semantic_contract_status(
            metadata.get('message_counts', {}), condition, tuple(robots),
            unknown_initial_pose=bool(unknown_initial_pose),
            semantic_artifacts={'handoff_structured': handoff['complete']},
            assignment_strategy=assignment_strategy)
    cooperation = _protocol_replay(run_directory, tuple(robots))
    status_records = cooperation.get('status_records', [])
    work_availability_complete = bool(status_records) and all(
        record.get('feasible_work_available') is not None and
        record.get('actionable_work_available') is not None
        for record in status_records)
    certificate_status = cooperation.get('certificate_status') or {
        'state': 'NO_EVIDENCE', 'payload_complete': False,
        'observation_count': 0,
        'reason': 'certificate status absent from protocol replay',
    }
    certificate_complete = bool(
        (certificate_status.get('payload_complete') and
         cooperation.get('certificate_records')) or
        certificate_status.get('state') == 'NOT_INVOKED')
    releases = cooperation.get('start_release_records', [])
    valid_releases = [item for item in releases
                      if item.get('event') == 'START_RELEASE' and
                      item.get('accepted_handoff') and
                      set(item.get('ready_robots', [])) >= set(robots)]
    release_time = (valid_releases[0].get('release_sim_time_s')
                    if len(valid_releases) == 1 else None)
    pre_release_dispatches = [item for item in cooperation.get('dispatches', [])
                              if release_time is not None and
                              item.get('sim_time_s') is not None and
                              item['sim_time_s'] < release_time]
    from .offline_timing_metrics import analyze_protocol
    timing = analyze_protocol(
        cooperation, ready_sim=release_time, end_sim=bag['clock'].get('end_s'))
    from .offline_motion_metrics import replay_run
    motion_anomalies = replay_run(run_directory, tuple(robots))
    cooperation_summary = cooperation.get('summary')
    if cooperation_summary is None:
        from .offline_protocol_replay import protocol_summary
        cooperation_summary = protocol_summary(cooperation, tuple(robots))
    from .offline_fairness import summarize_fairness
    fairness = summarize_fairness({
        'coverage': coverage,
        'motion': bag['motion'],
        'cooperation_summary': cooperation_summary,
        'avoidable_idle': timing['avoidable_idle'],
    }, tuple(robots))
    from .offline_window_metrics import analyze_windows
    window_evaluation = {
        'clock': bag['clock'], 'coverage': coverage,
        'cooperation': cooperation,
        'avoidable_idle': timing['avoidable_idle'],
        'motion_anomalies': motion_anomalies,
    }
    scaling_windows = analyze_windows(window_evaluation)
    from .offline_map_quality import map_quality_report
    map_quality_report_value = map_quality_report(run_directory, manifest)
    protocol_contract = {
        'payload_parse_complete': bool(cooperation.get('available')),
        'work_availability_complete': work_availability_complete,
        'certificate_payload_complete': (
            certificate_complete if assignment_strategy == 'frontier_cost_only'
            else True),
        'certificate_status': certificate_status,
        'certificate_observation_count': len(
            cooperation.get('certificate_records', [])),
        'common_start_release_complete': len(valid_releases) == 1,
        'start_release_count': len(releases),
        'start_release_sim_time_s': release_time,
        'pre_release_dispatch_count': len(pre_release_dispatches),
    }
    protocol_contract['complete'] = bool(
        protocol_contract['payload_parse_complete'] and
        (condition != 'C' or protocol_contract['work_availability_complete']) and
        protocol_contract['certificate_payload_complete'] and
        (condition != 'C' or not unknown_initial_pose or
         protocol_contract['common_start_release_complete']) and
        (condition != 'C' or protocol_contract['pre_release_dispatch_count'] == 0))
    evaluation = {
        'schema_version': 'thin_offline_evaluation_2.0',
        'complete': bool(metadata.get('complete') and gt['sample_rows'] > 0
                         and semantic.get('complete', False)
                         and protocol_contract['complete']),
        'condition': condition,
        'coverage_contract': {
            'A': 'robot1 /map local map',
            'B': 'transformed union of per-robot /map local maps',
            'C': 'robot1 /shared_map canonical shared map stream',
        },
        'raw_finalization': finalization,
        'raw_message_counts': metadata.get('message_counts', {}),
        'clock': bag['clock'],
        'ground_truth': gt,
        'motion': bag['motion'],
        'mapping': coverage,
        # Keep the previous key for consumers, but make its source explicit.
        'coverage': coverage,
        'efficiency': _efficiency_metrics(coverage, bag['motion']),
        'navigation': bag['navigation'],
        'cooperation': cooperation,
        'cooperation_summary': cooperation_summary,
        'avoidable_idle': timing['avoidable_idle'],
        'work_availability': timing['work_availability'],
        'snappiness': timing['snappiness'],
        'motion_anomalies': motion_anomalies,
        'fairness': fairness,
        'scaling_windows': scaling_windows,
        'map_quality_report': map_quality_report_value,
        'protocol_contract': protocol_contract,
        'semantic_contract': semantic,
        'handoff': handoff if unknown_initial_pose else {
            'complete': True, 'not_required': True},
        'contacts': contacts,
        'warnings': warnings,
        'map_quality': _load_optional_json(
            run_directory / 'forensic' / 'physical_gt_evaluation.json'),
        'topic_types': bag['topic_types'],
        'reference_summary': str(reference_summary)
        if reference_summary is not None else None,
    }
    destination = Path(output_path) if output_path is not None \
        else run_directory / 'thin_metrics.json'
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(evaluation, indent=2, sort_keys=True) + '\n',
                           encoding='utf-8')
    # These sidecars make the two required item-1 products directly consumable
    # without asking downstream thesis tooling to understand the full bundle.
    (destination.parent / 'avoidable_idle.json').write_text(
        json.dumps(timing['avoidable_idle'], indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    (destination.parent / 'snappiness.json').write_text(
        json.dumps({
            'snappiness': timing['snappiness'],
            'work_availability': timing['work_availability'],
        }, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    (destination.parent / 'cooperation_summary.json').write_text(
        json.dumps(cooperation_summary, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    (destination.parent / 'motion_anomalies.json').write_text(
        json.dumps(motion_anomalies, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    (destination.parent / 'fairness.json').write_text(
        json.dumps(fairness, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    (destination.parent / 'scaling_windows.json').write_text(
        json.dumps(scaling_windows, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    (destination.parent / 'map_quality_report.json').write_text(
        json.dumps(map_quality_report_value, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    (destination.parent / 'handoff_reciprocal_verification.json').write_text(
        json.dumps(handoff['reciprocal_verification'], indent=2,
                   sort_keys=True) + '\n', encoding='utf-8')
    return evaluation


def _efficiency_metrics(coverage, motion):
    total_distance = sum(float(item.get('travel_distance_m', 0.0))
                         for item in motion.values())
    return {
        'total_odom_distance_m': total_distance,
        'known_area_per_odom_m': (
            coverage['final_known_area_m2'] / total_distance
            if total_distance > 0 else None),
        'known_cells_per_odom_m': (
            coverage['final_known_cells'] / total_distance
            if total_distance > 0 else None),
    }


def _load_optional_json(path):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return {'available': False, 'reason': f'{path.name} absent or invalid'}
