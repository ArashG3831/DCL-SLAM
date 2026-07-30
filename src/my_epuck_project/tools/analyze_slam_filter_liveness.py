#!/usr/bin/env python3
"""Bounded offline analysis of teammate-filtered SLAM input liveness."""

import argparse
import csv
import json
import re
from pathlib import Path


FILTER_LINE = re.compile(
    r"\[(robot[12])\.teammate_scan_filter\]: FILTER_METRICS (.*)$")


def values(text):
    result = {}
    for token in text.split():
        if '=' not in token:
            continue
        key, value = token.split('=', 1)
        if value.startswith('{') or value.startswith('('):
            continue
        try:
            result[key] = float(value) if '.' in value else int(value)
        except ValueError:
            result[key] = value
    return result


def topic_rows(observer, robot, topic):
    path = observer / 'topic_health.csv'
    rows = []
    with path.open(newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            if row['robot_id'] == robot and row['topic_name'] == topic:
                rows.append(row)
    return rows


def metric_rows(launch_log, robot):
    rows = []
    for line in launch_log.read_text(encoding='utf-8', errors='replace').splitlines():
        match = FILTER_LINE.search(line)
        if match and match.group(1) == robot:
            rows.append(values(match.group(2)))
    return rows


def summarize(attempt):
    observers = [path for path in attempt.glob('observer/*')
                 if (path / 'topic_health.csv').is_file()]
    if not observers:
        raise ValueError(f'no observer directory under {attempt}')
    observer = observers[0]
    summary = {'attempt': str(attempt), 'robots': {}}
    for robot in ('robot1', 'robot2'):
        fixed = topic_rows(observer, robot, f'/{robot}/scan_d500_fixed')
        slam = topic_rows(observer, robot, f'/{robot}/scan_d500_slam')
        metrics = metric_rows(attempt / 'launch.log', robot)
        stale = [row for row in slam if row.get('stale') == 'True']
        first_stale = None
        was_fresh = False
        for row in slam:
            if row.get('stale') != 'True':
                was_fresh = True
            elif was_fresh:
                first_stale = float(row['ros_time_sec'])
                break
        summary['robots'][robot] = {
            'fixed_samples': len(fixed),
            'slam_samples': len(slam),
            'first_slam_stale_ros_s': first_stale,
            'last_fixed_ros_s': float(fixed[-1]['ros_time_sec']) if fixed else None,
            'last_slam_ros_s': float(slam[-1]['ros_time_sec']) if slam else None,
            'last_slam_age_s': float(slam[-1]['topic_age_s']) if slam and slam[-1]['topic_age_s'] else None,
            'slam_stale_samples': len(stale),
            'filter_metric_samples': len(metrics),
            'last_filter_metrics': metrics[-1] if metrics else {},
        }
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('attempt', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = summarize(args.attempt)
    encoded = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        args.output.write_text(encoded + '\n', encoding='utf-8')
    else:
        print(encoded)


if __name__ == '__main__':
    main()
