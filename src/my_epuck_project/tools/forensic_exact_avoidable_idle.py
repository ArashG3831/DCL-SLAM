#!/usr/bin/env python3
"""Classify directly evidenced feasible-work and avoidable-idle intervals.

This is an offline audit tool.  It never changes runtime decisions.  A segment
is counted as feasible only when a current candidate batch, readiness, TF, and
health evidence are all present and no goal/recovery/traffic block is active.
Missing evidence is excluded from the feasible denominator rather than guessed
as useful work.
"""

import argparse
import json
from pathlib import Path

from my_epuck_project.offline_timing_metrics import analyze_event_records


def _load_run(run):
    event_files = sorted(run.rglob('events.jsonl'))
    if not event_files:
        raise ValueError(f'no events.jsonl below {run}')
    events = [json.loads(line) for line in event_files[0].read_text(
        encoding='utf-8').splitlines() if line.strip()]
    summary = json.loads(next(run.glob('fast_trial_summary.json')).read_text(
        encoding='utf-8'))
    return events, summary


def analyze(run):
    events, summary = _load_run(run)
    readiness = summary.get('nav2_readiness_telemetry') or {}
    shared = readiness.get('shared_activation_reached_at_sim_s') or {}
    ready_sim = max((float(value) for value in shared.values()), default=0.0)
    end_sim = float(summary.get('simulation_end_time_s') or 0.0)
    return analyze_event_records(events, ready_sim=ready_sim,
                                 end_sim=end_sim, run_label=str(run))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = analyze(args.run)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True),
                           encoding='utf-8')
    print(json.dumps({robot: {
        key: value for key, value in metrics.items() if key != 'segments'}
        for robot, metrics in result['robots'].items()}, indent=2,
        sort_keys=True))


if __name__ == '__main__':
    main()
