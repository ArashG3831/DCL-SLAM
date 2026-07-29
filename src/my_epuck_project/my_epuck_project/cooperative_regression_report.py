"""Aggregate campaign analysis and deterministic report generation."""

from collections import Counter
import csv
from datetime import datetime
import json
import math
import os
from pathlib import Path
import statistics

import numpy as np

from .occupancy_map_comparison import (
    canonical_map,
    common_geometry,
    compare_maps,
    compare_semantic,
    consensus_maps,
    load_map,
    resample_semantic,
    save_map,
)


PRINCIPAL_MATRICES = {
    'known_iou': 'known_iou',
    'free_iou': 'free_iou',
    'occupied_iou': 'occupied_iou',
    'known_agreement': 'known_cell_agreement',
    'conflict_rate': 'occupied_free_conflict_rate',
    'unknown_mismatch': 'unknown_mismatch_rate',
    'occupancy_correlation': 'raw_occupancy_correlation',
    'coverage_area_difference': 'coverage_area_difference_m2',
    'best_shift_correlation': 'best_shift_correlation',
    'best_shift_dx': 'best_shift_dx_cells',
    'best_shift_dy': 'best_shift_dy_cells',
}


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f'.{path.name}.{os.getpid()}.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def read_json(path, default=None):
    try:
        with Path(path).open(encoding='utf-8') as stream:
            return json.load(stream)
    except (OSError, ValueError):
        return default


def finite(value):
    if isinstance(value, dict):
        return {key: finite(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite(item) for item in value]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        return float(value) if math.isfinite(float(value)) else None
    return value


def write_csv(path, rows, fieldnames=None):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = list(rows)
    fields = fieldnames or sorted({
        key for row in rows for key in row
    })
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({
                key: json.dumps(value, separators=(',', ':'))
                if isinstance(value, (dict, list)) else value
                for key, value in finite(row).items()
            })


def stats(values):
    numbers = [float(value) for value in values
               if value is not None and math.isfinite(float(value))]
    if not numbers:
        return {
            key: None for key in (
                'mean', 'median', 'minimum', 'maximum',
                'standard_deviation', 'p10', 'p90')
        }
    array = np.asarray(numbers)
    return {
        'mean': float(array.mean()),
        'median': float(np.median(array)),
        'minimum': float(array.min()),
        'maximum': float(array.max()),
        'standard_deviation': float(array.std()),
        'p10': float(np.percentile(array, 10)),
        'p90': float(np.percentile(array, 90)),
    }


def _attempts(campaign):
    progress = read_json(campaign / 'campaign_progress.json', {})
    selected = progress.get('valid_trials', {})
    result = []
    for trial_id, relative in sorted(selected.items()):
        attempt = campaign / relative
        metadata = read_json(attempt / 'runner_metadata.json', {})
        result.append((trial_id, attempt, metadata))
    return result


def _status_value(final_state, robot, field, default=None):
    return (
        final_state.get('robots', {}).get(robot, {})
        .get('status', {}).get(field, default)
    )


def _claim_value(final_state, robot, field, default=None):
    return (
        final_state.get('robots', {}).get(robot, {})
        .get('claim', {}).get(field, default)
    )


def _observer_summary(attempt, metadata):
    path = metadata.get('observer_summary_path')
    if path:
        return read_json(attempt / path, {})
    observer = attempt / 'observer'
    candidates = sorted(observer.glob('*/summary.json')) + [
        observer / 'summary.json']
    return next((read_json(path, {}) for path in candidates
                 if path.exists()), {})


def _warning_counts(attempt):
    counts = Counter()
    paths = list((attempt / 'observer').glob('*/warnings.jsonl'))
    direct = attempt / 'observer' / 'warnings.jsonl'
    if direct.exists():
        paths.append(direct)
    for path in paths:
        try:
            for line in path.read_text(encoding='utf-8').splitlines():
                row = json.loads(line)
                counts[row.get('category', 'PROCESS_WARNING')] += int(
                    row.get('occurrence_count', 1))
        except (OSError, ValueError):
            counts['warning_parse_errors'] += 1
    return dict(counts)


def _observer_events(attempt):
    paths = list((attempt / 'observer').glob('*/events.jsonl'))
    paths += [attempt / 'observer' / 'events.jsonl']
    rows = []
    for path in paths:
        try:
            rows.extend(
                json.loads(line)
                for line in path.read_text(encoding='utf-8').splitlines()
                if line.strip()
            )
        except (OSError, ValueError):
            pass
    return rows


def _event_elapsed(events, event_type, robot=None, last=False):
    values = [
        row.get('elapsed_s') for row in events
        if row.get('event_type') == event_type
        and (robot is None or row.get('robot_id') == robot)
        and row.get('elapsed_s') is not None
    ]
    if not values:
        return None
    return float(max(values) if last else min(values))


def _utc_timestamp(value):
    try:
        return datetime.fromisoformat(value.replace('Z', '+00:00')).timestamp()
    except (AttributeError, TypeError, ValueError):
        return None


def _normalized_warnings(attempt, trial_id, events):
    paths = list((attempt / 'observer').glob('*/warnings.jsonl'))
    direct = attempt / 'observer' / 'warnings.jsonl'
    if direct.exists():
        paths.append(direct)
    rows = []
    for path in paths:
        try:
            for line in path.read_text(encoding='utf-8').splitlines():
                row = json.loads(line)
                row['trial_id'] = trial_id
                row['artifact_path'] = str(path)
                last = _utc_timestamp(row.get('last_occurrence'))
                completion_times = [
                    _utc_timestamp(event.get('wall_time_utc'))
                    for event in events
                    if event.get('event_type') == 'MISSION_COMPLETE'
                ]
                completion_times = [
                    value for value in completion_times
                    if value is not None
                ]
                recovery_times = [
                    _utc_timestamp(event.get('wall_time_utc'))
                    for event in events
                    if 'RECOVER' in event.get('event_type', '')
                ]
                recovery_times = [
                    value for value in recovery_times if value is not None
                ]
                row['recovery_event_followed'] = (
                    last is not None
                    and any(value > last for value in recovery_times)
                )
                row['observed_at_or_after_mission_end'] = (
                    last is not None and bool(completion_times)
                    and last >= min(completion_times)
                )
                rows.append(row)
        except (OSError, ValueError):
            pass
    return rows


def _merge_process_metrics(campaign, attempts):
    rows = []
    for trial_id, attempt, metadata in attempts:
        path = attempt / 'process_metrics.csv'
        try:
            with path.open(newline='', encoding='utf-8') as stream:
                for row in csv.DictReader(stream):
                    rows.append({
                        'trial_id': trial_id,
                        'attempt_id': metadata.get('attempt_id'),
                        **row,
                    })
        except OSError:
            pass
    write_csv(campaign / 'process_metrics.csv', rows)


def _directory_size(path):
    total = 0
    for item in path.rglob('*'):
        try:
            if item.is_file():
                total += item.stat().st_size
        except OSError:
            pass
    return total


def classify_repeatability(metric_stats, coverage_cv):
    known = metric_stats['known_iou']['median']
    occupied = metric_stats['occupied_iou']['median']
    agreement = metric_stats['known_cell_agreement']['median']
    conflict = metric_stats['occupied_free_conflict_rate']['median']
    if None in (known, occupied, agreement, conflict, coverage_cv):
        return 'UNAVAILABLE'
    if (known >= 0.97 and occupied >= 0.90 and agreement >= 0.97
            and conflict <= 0.01 and coverage_cv <= 0.03):
        return 'STRONG'
    if (known >= 0.93 and occupied >= 0.80 and agreement >= 0.94
            and conflict <= 0.03):
        return 'MODERATE'
    return 'CONCERN'


def robust_outliers(rows):
    """Transparent median/MAD flags without removing any trial."""
    features = (
        'wall_time_s', 'known_area_m2', 'occupied_area_m2',
        'free_area_m2', 'mean_known_iou', 'mean_occupied_iou',
        'mean_conflict_rate', 'goals', 'failures', 'recoveries',
        'distance_travelled_m', 'warning_count',
    )
    flags = {row['trial_id']: [] for row in rows}
    for feature in features:
        values = [(row['trial_id'], row.get(feature)) for row in rows]
        values = [(key, float(value)) for key, value in values
                  if value is not None and math.isfinite(float(value))]
        if len(values) < 3:
            continue
        array = np.asarray([value for _, value in values])
        median = float(np.median(array))
        mad = float(np.median(np.abs(array - median)))
        if mad == 0.0:
            continue
        for trial_id, value in values:
            robust_z = 0.6745 * abs(value - median) / mad
            if robust_z > 3.5:
                flags[trial_id].append({
                    'feature': feature, 'value': value,
                    'median': median, 'mad': mad, 'robust_z': robust_z,
                })
    return {key: value for key, value in flags.items() if value}


def _save_array(path, key, array, geometry):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f'.{path.name}.{os.getpid()}.tmp')
    metadata = {
        'width': geometry.width, 'height': geometry.height,
        'resolution': geometry.resolution,
        'origin_x': geometry.origin_x, 'origin_y': geometry.origin_y,
        'frame_id': 'shared_map',
    }
    with temporary.open('wb') as stream:
        np.savez_compressed(
            stream, **{key: array},
            metadata_json=np.asarray(json.dumps(metadata, sort_keys=True)))
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def analyze_campaign(campaign_dir, free_threshold=25,
                     occupied_threshold=65, shift_window=3):
    """Validate selected attempts, compare maps, and regenerate all reports."""
    campaign = Path(campaign_dir).resolve()
    manifest = read_json(campaign / 'campaign_manifest.json', {})
    attempts = _attempts(campaign)
    if not attempts:
        raise ValueError('campaign has no selected valid trials')
    thresholds = {
        'free_max': free_threshold, 'occupied_min': occupied_threshold,
    }
    trial_rows = []
    warning_rows = []
    canonical = {}
    within = {}
    for trial_id, attempt, metadata in attempts:
        final_state = read_json(attempt / 'final_state.json', {})
        first_path = attempt / 'robot1_final_shared_map.npz'
        second_path = attempt / 'robot2_final_shared_map.npz'
        first, second = load_map(first_path), load_map(second_path)
        within_metrics, _, _, _ = compare_maps(
            first, second, None, free_threshold, occupied_threshold,
            shift_window)
        within[trial_id] = finite(within_metrics)
        atomic_json(attempt / 'within_trial_map_comparison.json',
                    within[trial_id])
        item, counts, semantic = canonical_map(
            first, second, None, free_threshold, occupied_threshold)
        item.metadata.update({
            'run_id': metadata.get('run_id', trial_id),
            'trial_id': trial_id,
            'classification_thresholds': thresholds,
        })
        saved_metadata = save_map(
            attempt / 'canonical_final_map.npz', item)
        atomic_json(
            attempt / 'canonical_final_map_metadata.json', saved_metadata)
        _save_array(
            attempt / 'canonical_conflict_mask.npz', 'conflict_mask',
            item.conflict, item.geometry)
        canonical[trial_id] = load_map(
            attempt / 'canonical_final_map.npz')
        observer = _observer_summary(attempt, metadata)
        navigation = observer.get('navigation', {})
        system = observer.get('system', {})
        motion = observer.get('motion', {})
        mapping = observer.get('mapping', {})
        warnings = _warning_counts(attempt)
        events = _observer_events(attempt)
        warning_rows.extend(
            _normalized_warnings(attempt, trial_id, events))
        log_review = metadata.get(
            'classification_details', {}).get('log_review', {})
        event_counts = observer.get('events', {})
        continuous = observer.get('continuous_exploration', {})
        anomalies = observer.get('anomalies', {})
        distance_by_robot = motion.get('distance_travelled_m', {})
        if not isinstance(distance_by_robot, dict):
            distance_by_robot = {}
        terminal_goal_durations = [
            float(event['duration_s']) for event in events
            if event.get('event_type') in (
                'NAVIGATION_SUCCEEDED', 'NAVIGATION_FAILED',
                'NAVIGATION_CANCELED', 'NAVIGATION_TIMED_OUT')
            and event.get('duration_s') is not None
        ]
        known = semantic != 0
        occupied = semantic == 2
        free = semantic == 1
        cell_area = item.geometry.resolution ** 2
        row = {
            'trial_id': trial_id,
            'attempt_id': metadata.get('attempt_id'),
            'classification': metadata.get('classification', 'UNKNOWN'),
            'robot1_status': _status_value(final_state, 'robot1', 'state'),
            'robot2_status': _status_value(final_state, 'robot2', 'state'),
            'robot1_status_reason': _status_value(
                final_state, 'robot1', 'reason'),
            'robot2_status_reason': _status_value(
                final_state, 'robot2', 'reason'),
            'robot1_claim': _claim_value(final_state, 'robot1', 'state'),
            'robot2_claim': _claim_value(final_state, 'robot2', 'state'),
            'robot1_claim_id': _claim_value(
                final_state, 'robot1', 'claim_id'),
            'robot2_claim_id': _claim_value(
                final_state, 'robot2', 'claim_id'),
            'mission_completion_time_s':
                observer.get('mission_completion_time_s'),
            'wall_time_s': metadata.get('wall_time_s'),
            'known_area_m2': int(known.sum()) * cell_area,
            'occupied_area_m2': int(occupied.sum()) * cell_area,
            'free_area_m2': int(free.sum()) * cell_area,
            'goals': navigation.get('goals_sent', 0),
            'goals_accepted': navigation.get('goals_accepted', 0),
            'successes': navigation.get('successes', 0),
            'failures': navigation.get('failures', 0),
            'rejections': navigation.get('rejections', 0),
            'cancellations': navigation.get('cancellations', 0),
            'timeouts': navigation.get('timeouts', 0),
            'recoveries': navigation.get('recoveries', 0),
            'unknown_nav2_failures':
                navigation.get('unknown_nav2_failures', 0),
            'average_goal_duration_s': (
                statistics.fmean(terminal_goal_durations)
                if terminal_goal_durations else None),
            'maximum_goal_duration_s': (
                max(terminal_goal_durations)
                if terminal_goal_durations else None),
            'robot1_cycles':
                continuous.get('robot1', {}).get('exploration_cycles'),
            'robot2_cycles':
                continuous.get('robot2', {}).get('exploration_cycles'),
            'robot1_local_exhaustion_time_s':
                _event_elapsed(events, 'LOCALLY_EXHAUSTED', 'robot1'),
            'robot2_local_exhaustion_time_s':
                _event_elapsed(events, 'LOCALLY_EXHAUSTED', 'robot2'),
            'consensus_start_time_s':
                _event_elapsed(events, 'COMPLETION_CONSENSUS_STARTED'),
            'mission_complete_event_time_s':
                _event_elapsed(events, 'MISSION_COMPLETE', last=True),
            'claims': event_counts.get('CLAIM_PROPOSED', 0),
            'arbitration_conflicts':
                event_counts.get('ARBITRATION_CONFLICT', 0),
            'arbitration_wins': event_counts.get('ARBITRATION_WON', 0),
            'arbitration_losses': event_counts.get('ARBITRATION_LOST', 0),
            'dual_accepted_events':
                event_counts.get('DUAL_ACCEPTED_CONFLICT', 0),
            'arbitration_cancellations':
                event_counts.get('ARBITRATION_CANCELED', 0),
            'success_cooldowns':
                event_counts.get('SUCCESS_COOLDOWN_CREATED', 0),
            'failure_suppressions':
                event_counts.get('FAILURE_SUPPRESSION_CREATED', 0),
            'peer_session_changes':
                event_counts.get('PEER_SESSION_CHANGED', 0),
            'stale_peer_events':
                event_counts.get('PEER_STATUS_STALE', 0),
            'distance_travelled_m': sum(
                float(value) for value in distance_by_robot.values()),
            'robot1_distance_travelled_m':
                distance_by_robot.get('robot1'),
            'robot2_distance_travelled_m':
                distance_by_robot.get('robot2'),
            'trajectory_overlap_fraction':
                motion.get('cross_robot_overlap_fraction'),
            'no_progress_episodes':
                anomalies.get('no_progress_episodes', 0),
            'stuck_episodes': anomalies.get('stuck_episodes', 0),
            'oscillation_episodes':
                anomalies.get('oscillation_episodes', 0),
            'duplicated_coverage_fraction':
                mapping.get('duplicated_known_fraction'),
            'initial_known_cells': mapping.get('initial_known_cells'),
            'final_known_cells': mapping.get('final_known_cells'),
            'coverage_gain_cells': mapping.get('coverage_gain_cells'),
            'warning_count': sum(warnings.values()),
            'logger_internal_errors':
                system.get('internal_logger_error_count'),
            'logger_write_failures': system.get('write_failures'),
            'launch_error_line_count': log_review.get(
                'error_line_count',
                len(log_review.get('error_lines', []))),
            'pre_shutdown_traceback':
                log_review.get('pre_shutdown_traceback',
                               log_review.get('traceback', False)),
            'shutdown_traceback_count':
                log_review.get('shutdown_traceback_count', 0),
            'process_exit_codes': metadata.get('process_exit_codes', {}),
            'cleanup_complete': metadata.get('cleanup_complete', False),
            'process_tree_peak_rss_bytes':
                metadata.get('process_tree_peak_rss_bytes'),
            'process_tree_peak_cpu_percent':
                metadata.get('process_tree_peak_cpu_percent'),
            'logger_mean_cpu_percent':
                system.get('cpu_measurement', {}).get('mean_percent'),
            'logger_peak_cpu_percent':
                system.get('cpu_measurement', {}).get('peak_percent'),
            'logger_peak_rss_bytes': system.get('rss_peak_bytes'),
            'first_relevant_error': log_review.get('first_error'),
            'last_relevant_error': log_review.get('last_error'),
            'artifact_path': str(attempt),
            'output_size_bytes': _directory_size(attempt),
            **{f'warning_{key}': value for key, value in warnings.items()},
            **{f'canonical_{key}': value for key, value in counts.items()},
        }
        trial_rows.append(finite(row))
    trial_ids = sorted(canonical)
    cross_run_geometry = common_geometry(
        [canonical[key] for key in trial_ids])
    aligned_canonical = {
        key: resample_semantic(
            canonical[key], cross_run_geometry,
            free_threshold, occupied_threshold)
        for key in trial_ids
    }
    pair_rows = []
    per_trial_pairs = {trial: [] for trial in trial_ids}
    for index, trial_a in enumerate(trial_ids):
        for trial_b in trial_ids[index + 1:]:
            geometry = cross_run_geometry
            metrics = compare_semantic(
                aligned_canonical[trial_a], aligned_canonical[trial_b],
                geometry.resolution, shift_window)
            row = {
                'trial_a': trial_a,
                'trial_b': trial_b,
                **finite(metrics),
                'classification_thresholds': thresholds,
                'comparison_validity': True,
                'warnings': [],
                'common_grid_geometry': {
                    'width': geometry.width, 'height': geometry.height,
                    'resolution': geometry.resolution,
                    'origin_x': geometry.origin_x,
                    'origin_y': geometry.origin_y,
                },
            }
            pair_rows.append(row)
            per_trial_pairs[trial_a].append(row)
            per_trial_pairs[trial_b].append(row)
    write_csv(campaign / 'pairwise_map_metrics.csv', pair_rows)
    matrix_dir = campaign / 'matrices'
    matrix_dir.mkdir(exist_ok=True)
    for filename, metric in PRINCIPAL_MATRICES.items():
        lookup = {}
        for row in pair_rows:
            lookup[(row['trial_a'], row['trial_b'])] = row.get(metric)
            lookup[(row['trial_b'], row['trial_a'])] = row.get(metric)
        rows = []
        for left in trial_ids:
            row = {'trial_id': left}
            for right in trial_ids:
                row[right] = 0.0 if left == right and (
                    'difference' in filename or 'shift_d' in filename
                ) else (1.0 if left == right else lookup.get((left, right)))
            rows.append(row)
        write_csv(
            matrix_dir / f'{filename}.csv', rows,
            ['trial_id'] + trial_ids)
    for row in trial_rows:
        pairs = per_trial_pairs[row['trial_id']]
        row['mean_known_iou'] = stats(
            [item['known_iou'] for item in pairs])['mean']
        row['mean_occupied_iou'] = stats(
            [item['occupied_iou'] for item in pairs])['mean']
        row['mean_conflict_rate'] = stats(
            [item['occupied_free_conflict_rate']
             for item in pairs])['mean']
    write_csv(campaign / 'trials.csv', trial_rows)
    write_csv(campaign / 'warnings.csv', warning_rows)
    _merge_process_metrics(campaign, attempts)
    aggregate_dir = campaign / 'aggregate'
    aggregate_dir.mkdir(exist_ok=True)
    consensus = consensus_maps(
        [canonical[key] for key in trial_ids], None,
        free_threshold, occupied_threshold)
    geometry = consensus.pop('geometry')
    consensus.pop('stack')
    for name, array in consensus.items():
        filename = 'consensus_map.npz' if name == 'consensus_occupancy' \
            else f'{name}.npz'
        _save_array(aggregate_dir / filename, name, array, geometry)
    consensus_summary = {
        'mean_known_frequency':
            float(consensus['known_frequency'].mean()),
        'cells_known_in_all_trials': int(
            (consensus['known_frequency'] == 1.0).sum()),
        'mean_disagreement_frequency':
            float(consensus['disagreement_frequency'].mean()),
        'maximum_disagreement_frequency':
            float(consensus['disagreement_frequency'].max()),
        'cells_with_any_disagreement': int(
            (consensus['disagreement_frequency'] > 0.0).sum()),
        'cells_with_disagreement_at_least_0_2': int(
            (consensus['disagreement_frequency'] >= 0.2).sum()),
        'majority_semantic_cell_counts': {
            name: int((consensus['majority_semantic'] == value).sum())
            for value, name in enumerate(
                ('unknown', 'free', 'occupied', 'uncertain'))
            },
    }
    consensus_metadata = {
        'trial_ids': trial_ids,
        'width': geometry.width, 'height': geometry.height,
        'resolution': geometry.resolution,
        'origin_x': geometry.origin_x, 'origin_y': geometry.origin_y,
        'classification_thresholds': thresholds,
        'policy': 'per-cell semantic majority; ties use class order',
    }
    atomic_json(
        aggregate_dir / 'consensus_metadata.json', consensus_metadata)
    metric_stats = {
        metric: stats([row.get(metric) for row in pair_rows])
        for metric in (
            'known_iou', 'free_iou', 'occupied_iou',
            'known_cell_agreement', 'occupied_free_conflict_rate',
            'unknown_mismatch_rate', 'raw_occupancy_correlation',
            'best_shift_correlation')
    }
    known_areas = [row['known_area_m2'] for row in trial_rows]
    coverage_cv = (
        float(np.std(known_areas) / np.mean(known_areas))
        if known_areas and np.mean(known_areas) else None
    )
    repeatability = classify_repeatability(metric_stats, coverage_cv)
    shift_patterns = Counter(
        (int(row['best_shift_dx_cells']),
         int(row['best_shift_dy_cells']))
        for row in pair_rows
        if row.get('best_shift_dx_cells') or row.get('best_shift_dy_cells')
    )
    maximum_shift_improvement = max(
        (row.get('correlation_improvement') or 0.0 for row in pair_rows),
        default=0.0)
    shift_diagnostics = {
        'nonzero_best_shift_pair_count': sum(shift_patterns.values()),
        'material_improvement_pair_count': sum(
            (row.get('correlation_improvement') or 0.0) >= 0.01
            for row in pair_rows),
        'maximum_correlation_improvement': maximum_shift_improvement,
        'repeated_nonzero_shift_patterns': [
            {'dx_cells': dx, 'dy_cells': dy, 'pair_count': count}
            for (dx, dy), count in sorted(shift_patterns.items())
            if count >= 2
        ],
        'possible_systematic_alignment_problem': (
            maximum_shift_improvement >= 0.01
            and any(count >= 2 for count in shift_patterns.values())
        ),
    }

    def pair_score(row):
        values = [
            row.get('known_iou'), row.get('occupied_iou'),
            row.get('known_cell_agreement'),
        ]
        return statistics.fmean(
            [value for value in values if value is not None])
    worst = min(pair_rows, key=pair_score) if pair_rows else None
    best = max(pair_rows, key=pair_score) if pair_rows else None
    outliers = robust_outliers(trial_rows)
    non_outlier_pairs = [
        row for row in pair_rows
        if row['trial_a'] not in outliers and row['trial_b'] not in outliers
    ]
    sensitivity_stats = {
        metric: stats([row.get(metric) for row in non_outlier_pairs])
        for metric in metric_stats
    }
    manifest_attempts = list((campaign / 'attempts').glob(
        'trial_*_attempt_*'))
    all_attempt_metadata = [
        read_json(path / 'runner_metadata.json', {}) for path in manifest_attempts
    ]
    counts = Counter(
        item.get('classification', 'UNKNOWN') for item in all_attempt_metadata)
    pass_count = sum(
        row['classification'] == 'PASS' for row in trial_rows)
    summary = {
        'schema_version': '1.0.0',
        'campaign_id': manifest.get('campaign_id', campaign.name),
        'code_commit': manifest.get('git_commit'),
        'trial_count_requested': manifest.get(
            'trial_count_requested', len(trial_rows)),
        'valid_trial_count': len(trial_rows),
        'pass_count': pass_count,
        'failure_count': len(trial_rows) - pass_count,
        'timeout_count': sum(
            row['classification'] == 'MISSION_TIMEOUT'
            for row in trial_rows),
        'crash_count': sum(
            row['classification'] == 'PROCESS_CRASH'
            for row in trial_rows),
        'infrastructure_retry_count': max(
            0, len(all_attempt_metadata) - len(trial_rows)),
        'mission_completion_rate': (
            pass_count / len(trial_rows) if trial_rows else None),
        'all_trials_complete_boolean':
            len(trial_rows) == manifest.get(
                'trial_count_requested', len(trial_rows)),
        'all_processes_clean_boolean': bool(
            manifest.get(
                'final_cleanup_audit', {}).get('clean', False)),
        'attempt_cleanup_failure_count': sum(
            not row.get('cleanup_complete', False)
            for row in all_attempt_metadata),
        'chosen_concurrency': manifest.get('chosen_concurrency'),
        'total_wall_time_s': manifest.get('total_wall_time_s'),
        'output_size_bytes': _directory_size(campaign),
        'aggregate_robot_metrics': {
            'goals': sum(row.get('goals', 0) for row in trial_rows),
            'goals_accepted': sum(
                row.get('goals_accepted', 0) for row in trial_rows),
            'successes': sum(row.get('successes', 0) for row in trial_rows),
            'failures': sum(row.get('failures', 0) for row in trial_rows),
            'rejections': sum(
                row.get('rejections', 0) for row in trial_rows),
            'cancellations': sum(
                row.get('cancellations', 0) for row in trial_rows),
            'timeouts': sum(
                row.get('timeouts', 0) for row in trial_rows),
            'recoveries': sum(row.get('recoveries', 0) for row in trial_rows),
            'distance_travelled_m': sum(
                row.get('distance_travelled_m', 0.0)
                for row in trial_rows),
        },
        'aggregate_warning_metrics': dict(sum(
            (Counter({
                key.removeprefix('warning_'): value
                for key, value in row.items()
                if key.startswith('warning_') and key != 'warning_count'
                and isinstance(value, int)
            }) for row in trial_rows), Counter())),
        'aggregate_launch_log_error_lines': sum(
            row.get('launch_error_line_count', 0)
            for row in trial_rows),
        'attempt_classification_counts': dict(counts),
        'map_metric_aggregates': metric_stats,
        'map_metric_sensitivity_excluding_flagged_outliers': {
            'excluded_trials': sorted(outliers),
            'remaining_pair_count': len(non_outlier_pairs),
            'metrics': sensitivity_stats,
        },
        'coverage_area_coefficient_of_variation': coverage_cv,
        'repeatability_classification': repeatability,
        'cross_correlation_diagnostics': shift_diagnostics,
        'consensus_map_summary': consensus_summary,
        'worst_pair': (
            {key: worst.get(key) for key in (
                'trial_a', 'trial_b', 'known_iou', 'occupied_iou',
                'known_cell_agreement', 'occupied_free_conflict_rate')}
            if worst else None),
        'best_pair': (
            {key: best.get(key) for key in (
                'trial_a', 'trial_b', 'known_iou', 'occupied_iou',
                'known_cell_agreement', 'occupied_free_conflict_rate')}
            if best else None),
        'outlier_trials': outliers,
        'artifact_paths': {
            'report': str(campaign / 'campaign_report.md'),
            'trials_csv': str(campaign / 'trials.csv'),
            'pairwise_csv': str(campaign / 'pairwise_map_metrics.csv'),
            'consensus_map': str(aggregate_dir / 'consensus_map.npz'),
            'disagreement_map':
                str(aggregate_dir / 'disagreement_frequency.npz'),
        },
        'limitations': manifest.get('limitations', []),
    }
    atomic_json(campaign / 'campaign_summary.json', finite(summary))
    render_report(
        campaign, manifest, summary, trial_rows, within, metric_stats,
        pair_rows)
    return finite(summary)


def render_report(campaign, manifest, summary, trials, within, metric_stats,
                  pair_rows):
    """Write a compact failure-forward Markdown report."""
    worst = summary['worst_pair'] or {}
    consensus_text = json.dumps(
        summary['consensus_map_summary'], indent=2, sort_keys=True)
    report_manifest = dict(manifest)
    calibration = manifest.get('calibration_result', {})
    if calibration:
        report_manifest['calibration_result'] = {
            key: calibration.get(key) for key in (
                'attempt_id', 'classification', 'wall_time_s',
                'readiness_elapsed_s', 'process_tree_peak_rss_bytes',
                'process_tree_peak_cpu_percent', 'maximum_process_count',
                'cleanup_complete', 'process_exit_codes')
        }
    lines = [
        '# Cooperative exploration regression campaign',
        '',
        '## Verdict',
        '',
        '- Trials completed: {}/{}'.format(
            summary['valid_trial_count'], summary['trial_count_requested']),
        '- Trials passed: {}'.format(summary['pass_count']),
        '- Mission-completion rate: {:.1%}'.format(
            summary['mission_completion_rate'])
        if summary['mission_completion_rate'] is not None
        else '- Mission-completion rate: unavailable',
        '- Crashes: {}'.format(summary['crash_count']),
        '- Timeouts: {}'.format(summary['timeout_count']),
        '- Final-map repeatability: {}'.format(
            summary['repeatability_classification']),
        '- Worst map pair: {} / {}'.format(
            worst.get('trial_a', 'unavailable'),
            worst.get('trial_b', 'unavailable')),
        '- Major warning counts: {}'.format(
            json.dumps(
                summary['aggregate_warning_metrics'], sort_keys=True)),
        '- Selected concurrency: {}'.format(summary['chosen_concurrency']),
        '- Total campaign wall time: {} s'.format(
            summary['total_wall_time_s']),
        '',
        '## 1. Configuration',
        '',
        '```json',
        json.dumps(finite(report_manifest), indent=2, sort_keys=True),
        '```',
        '',
        '## 2. Parallel-execution methodology',
        '',
        'Each complete two-robot stack used a distinct ROS domain, Webots '
        'port, process group, ROS log directory, temporary directory, and '
        'attempt directory. Calibration and the two-instance pilot gate '
        'adaptive bounded scheduling.',
        '',
        '## 3. Trial outcomes',
        '',
        '| Trial | Result | Robot 1 | Robot 2 | Claims (r1/r2) | Time (s) |',
        '|---|---|---|---|---|---:|',
    ]
    for row in trials:
        lines.append(
            '| {} | {} | {} | {} | {}/{} | {} |'.format(
                row['trial_id'], row['classification'],
                row['robot1_status'], row['robot2_status'],
                row['robot1_claim'], row['robot2_claim'],
                row.get('mission_completion_time_s')))
    section_text = [
        ('4. Final robot statuses',
         'The table above records the exact final status and claim snapshot.'),
        ('5. Errors and warnings',
         json.dumps(summary['aggregate_warning_metrics'], sort_keys=True)),
        ('6. Navigation reliability',
         json.dumps(summary['aggregate_robot_metrics'], sort_keys=True)),
        ('7. Mission-completion reliability',
         f"{summary['pass_count']} of {summary['valid_trial_count']} valid "
         'trials met every pass condition.'),
        ('8. Within-trial robot map agreement',
         json.dumps(within, indent=2, sort_keys=True)),
        ('9. Across-trial map repeatability',
         json.dumps(metric_stats, indent=2, sort_keys=True)),
        ('10. Cross-correlation diagnostics',
         '{} unique pairs retain zero-shift authoritative metrics; best '
         'small shifts are supplementary only.\n\n```json\n{}\n```'.format(
             len(pair_rows),
             json.dumps(
                 summary['cross_correlation_diagnostics'],
                 indent=2, sort_keys=True))),
        ('11. Consensus and disagreement maps',
         f"Artifacts: {summary['artifact_paths']['consensus_map']} and "
         f"{summary['artifact_paths']['disagreement_map']}.\n\n```json\n"
         + consensus_text + '\n```'),
        ('12. Performance and resource usage',
         f"Campaign wall time: {summary['total_wall_time_s']} s."),
        ('13. Outliers',
         'All trials remain in the primary aggregates. Flagged trials and '
         'the explicitly labeled sensitivity analysis follow.\n\n```json\n'
         '{}\n```\n\n```json\n{}\n```'.format(
             json.dumps(
                 summary['outlier_trials'], indent=2, sort_keys=True),
             json.dumps(
                 summary[
                     'map_metric_sensitivity_excluding_flagged_outliers'],
                 indent=2, sort_keys=True))),
        ('14. Limitations',
         json.dumps(summary['limitations'], indent=2, sort_keys=True)),
        ('15. Exact reproduction command',
         f"`{manifest.get('reproduction_command', 'unavailable')}`"),
    ]
    for heading, body in section_text:
        lines += ['', f'## {heading}', '', body]
    failed = [row for row in trials if row['classification'] != 'PASS']
    if failed:
        lines += ['', '## Failed trial details', '']
        for row in failed:
            lines += [
                f"### {row['trial_id']} — {row['classification']}",
                '',
                f"- Robot statuses: {row['robot1_status']} / "
                f"{row['robot2_status']}",
                f"- Last claims: {row['robot1_claim']} / "
                f"{row['robot2_claim']}",
                f"- Process exit codes: {row['process_exit_codes']}",
                '- First relevant error: {}'.format(
                    row.get('first_relevant_error')),
                '- Last relevant error: {}'.format(
                    row.get('last_relevant_error')),
                f"- Artifact path: `{row['artifact_path']}`",
            ]
    (campaign / 'campaign_report.md').write_text(
        '\n'.join(lines) + '\n', encoding='utf-8')
