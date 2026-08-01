#!/usr/bin/env python3
"""Extract conservative Robot 2 reachability evidence from a trial artifact.

The current generator log predates per-frontier diagnostic fields.  This tool
therefore preserves aggregate candidate batches and planner failures while
marking unavailable map/costmap/candidate fields as ``NOT_RECORDED`` rather
than inferring a cause from a generic GridBased warning.
"""

import argparse
import csv
import re
from pathlib import Path


METRICS = re.compile(
    r"\[(?P<stamp>\d+\.\d+)\].*CANDIDATE_METRICS "
    r"revision=(?P<revision>\d+) regions=(?P<regions>\d+) "
    r"coarse=(?P<coarse>\d+) queries=(?P<queries>\d+) "
    r"reachable=(?P<reachable>\d+) suppressed=(?P<suppressed>\d+) "
    r"extraction_ms=(?P<extraction>[-+\d.]+) "
    r"stale_results=(?P<stale>\d+)"
)
PLANNER = re.compile(
    r"\[(?P<stamp>\d+\.\d+)\].*GridBased plugin failed to plan from "
    r"\((?P<start_x>[-+\d.]+), (?P<start_y>[-+\d.]+)\) to "
    r"\((?P<goal_x>[-+\d.]+), (?P<goal_y>[-+\d.]+)\): "
    r"(?P<reason>.*)"
)

FIELDS = [
    'record_type', 'robot_id', 'ros_time_s', 'steady_receipt_time_s',
    'shared_map_revision', 'shared_map_dimensions', 'shared_map_origin',
    'shared_map_orientation', 'shared_map_resolution',
    'global_costmap_dimensions', 'global_costmap_origin',
    'global_costmap_orientation', 'global_costmap_resolution',
    'shared_map_timestamp', 'global_costmap_timestamp', 'pose',
    'transform_chain_age', 'frontier_region_count', 'generated_approach_count',
    'local_path_reuse_count', 'new_path_query_count', 'lease_wait_count',
    'query_result', 'planner_result_code', 'candidate_age_s',
    'map_costmap_skew_s', 'physical_signature', 'frontier_centroid',
    'frontier_bounds', 'approach_pose', 'alternative_accessible_cells',
    'occupancy_value', 'global_costmap_value', 'local_costmap_value',
    'obstacle_clearance_m', 'euclidean_distance_m',
    'connected_component', 'classification', 'detail',
]


def empty(record_type: str) -> dict[str, str]:
    return {field: 'NOT_RECORDED' for field in FIELDS} | {
        'record_type': record_type,
        'robot_id': 'robot2',
    }


def parse(artifact: Path) -> list[dict[str, str]]:
    logs = artifact / 'ros_logs'
    generator = next(logs.glob('frontier_candidate_generator*443936*'), None)
    if generator is None:
        matches = sorted(logs.glob('frontier_candidate_generator*'))
        generator = matches[-1] if matches else None
    planner_matches = sorted(logs.glob('planner_server*'))
    planner = next((path for path in planner_matches if 'robot2' in path.read_text(errors='replace')[:20000]), None)
    if planner is None and planner_matches:
        planner = planner_matches[-1]

    rows: list[dict[str, str]] = []
    metrics: list[tuple[float, dict[str, str]]] = []
    if generator is not None:
        for line in generator.read_text(errors='replace').splitlines():
            match = METRICS.search(line)
            if not match:
                continue
            data = match.groupdict()
            row = empty('candidate_batch')
            row.update({
                'ros_time_s': data['stamp'],
                'shared_map_revision': data['revision'],
                'frontier_region_count': data['regions'],
                'generated_approach_count': data['regions'],
                'new_path_query_count': data['queries'],
                'query_result': 'reachable=%s; suppressed=%s; stale_results=%s' % (
                    data['reachable'], data['suppressed'], data['stale'],
                ),
                'detail': 'aggregate generator metrics; per-frontier fields were not logged',
            })
            row['_stamp'] = data['stamp']
            metrics.append((float(data['stamp']), row))
            rows.append(row)

    if planner is not None:
        for line in planner.read_text(errors='replace').splitlines():
            match = PLANNER.search(line)
            if not match:
                continue
            data = match.groupdict()
            nearest = min(metrics, key=lambda item: abs(item[0] - float(data['stamp']))) if metrics else None
            row = empty('planner_failure')
            row.update({
                'ros_time_s': data['stamp'],
                'pose': '(%s,%s)' % (data['start_x'], data['start_y']),
                'approach_pose': '(%s,%s)' % (data['goal_x'], data['goal_y']),
                'query_result': 'GridBased failure',
                'planner_result_code': 'NOT_RECORDED',
                'classification': 'PATH_ACTION_FAILURE',
                'detail': data['reason'],
            })
            if nearest is not None:
                row['shared_map_revision'] = nearest[1]['shared_map_revision']
                row['frontier_region_count'] = nearest[1]['frontier_region_count']
                row['new_path_query_count'] = nearest[1]['new_path_query_count']
            rows.append(row)
    for row in rows:
        row.pop('_stamp', None)
    return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('artifact', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rows = parse(args.artifact)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    print('wrote %d conservative Robot 2 records to %s' % (len(rows), args.output))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
