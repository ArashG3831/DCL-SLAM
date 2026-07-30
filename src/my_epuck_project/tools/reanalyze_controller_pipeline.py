#!/usr/bin/env python3
"""Reclassify saved bounded controller incidents without changing originals."""

import argparse
import json
from pathlib import Path

from my_epuck_project.controller_pipeline_diagnostics import Sample, command_reason


def reclassify(path):
    records = []
    for line in path.read_text(encoding='utf-8').splitlines():
        incident = json.loads(line)
        samples = [Sample(
            item['sim_s'], item['wall_s'], item.get('pose_x', 0.0),
            item.get('pose_y', 0.0), item.get('pose_yaw', 0.0),
            item.get('odom_vx', 0.0), item.get('odom_wz', 0.0),
            item.get('commands', {}), item.get('command_meta', {}),
            item.get('distance_remaining'), item.get('active_goal', False),
            item.get('collision_state', {}), item.get('dwb', {}))
            for item in incident.get('samples', [])]
        # Recreate the trigger-time decision.  Samples after the trigger may
        # contain a later command or a stale retained value and must not
        # retroactively change the original causal classification.
        evidence = [item for item in samples
                    if item.sim_s <= incident['trigger_sim_s']]
        dwb_records = [item.get('dwb', {}) for item in evidence
                       if item.get('dwb')]
        incident['new_dwb_evidence_available'] = any(
            item.get('selected') is not None and
            item.get('best_valid_forward') is not None
            for item in dwb_records)
        incident['new_dwb_evidence_unavailable_fields'] = [
            'selected trajectory critic comparison',
            'best valid forward trajectory',
            'raw/scale/weighted critic contributions',
            'path heading and costmap crop',
        ] if not incident['new_dwb_evidence_available'] else []
        classification, contributing, confidence, missing = command_reason(evidence)
        incident['corrected_classification'] = classification
        incident['corrected_contributing'] = contributing
        incident['corrected_confidence'] = confidence
        incident['corrected_missing_evidence'] = missing
        records.append(incident)
    return records


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--diagnostic-root', type=Path, required=True)
    parser.add_argument('--output-root', type=Path, required=True)
    args = parser.parse_args()
    args.output_root.mkdir(parents=True, exist_ok=True)
    all_records = {}
    for robot in ('robot1', 'robot2'):
        all_records[robot] = reclassify(
            args.diagnostic_root / f'{robot}_controller_stalls.jsonl')
    summary = {'source': str(args.diagnostic_root), 'robots': {}}
    for robot, records in all_records.items():
        counts = {}
        original_counts = {}
        false_rotation = 0
        stale_or_missing = 0
        for incident in records:
            original = incident.get('classification', 'UNKNOWN')
            corrected = incident['corrected_classification']
            original_counts[original] = original_counts.get(original, 0) + 1
            counts[corrected] = counts.get(corrected, 0) + 1
            if original == 'BASE_NOT_RESPONDING' and corrected not in (
                    'BASE_NOT_RESPONDING', 'BASE_LINEAR_NOT_RESPONDING',
                    'BASE_ANGULAR_NOT_RESPONDING'):
                false_rotation += 1
            if corrected in ('COMMAND_STAGE_STALE', 'UNKNOWN_COMMAND_STALL'):
                stale_or_missing += 1
        summary['robots'][robot] = {
            'original_incidents_by_class': original_counts,
            'corrected_incidents_by_class': counts,
            'false_rotation_base_labels': false_rotation,
            'stale_or_insufficient_evidence': stale_or_missing,
            'incidents': records,
        }
    summary['historical_compatibility'] = {
        'new_bounded_dwb_fields_present': any(
            incident.get('new_dwb_evidence_available', False)
            for value in summary['robots'].values()
            for incident in value['incidents']),
        'note': 'The historical run predates selected-versus-forward DWB capture; unavailable fields are listed per incident and no historical files were rewritten.',
    }
    (args.output_root / 'corrected_controller_pipeline_analysis.json').write_text(
        json.dumps(summary, indent=2), encoding='utf-8')
    lines = ['# Corrected controller-pipeline offline analysis', '',
             f'Source: `{args.diagnostic_root}`', '']
    lines += [f"Historical compatibility: {summary['historical_compatibility']['note']}", '']
    for robot, value in summary['robots'].items():
        lines += [f'## {robot}', '',
                  f"- Original: `{value['original_incidents_by_class']}`",
                  f"- Corrected: `{value['corrected_incidents_by_class']}`",
                  f"- False rotation base labels: {value['false_rotation_base_labels']}",
                  f"- Stale/insufficient evidence: {value['stale_or_insufficient_evidence']}", '']
    (args.output_root / 'corrected_controller_pipeline_analysis.md').write_text(
        '\n'.join(lines), encoding='utf-8')


if __name__ == '__main__':
    main()
