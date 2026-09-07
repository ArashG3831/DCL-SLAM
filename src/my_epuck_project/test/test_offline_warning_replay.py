"""Offline warning output remains complete and fail-closed."""

import json

from my_epuck_project.offline_evidence_replay import _warning_metrics


def test_warning_metrics_replays_receipts_and_materializes_legacy_output(
        tmp_path):
    path = tmp_path / 'rosout_receipts.jsonl'
    path.write_text(
        ''.join(json.dumps({
            'level': 30,
            'name': 'costmap',
            'message': f'update {index}.0',
            'wall_time_utc': f't{index}',
        }) + '\n' for index in range(11000)),
        encoding='utf-8')

    result = _warning_metrics(tmp_path)

    assert result['available'] is True
    assert result['source'] == 'rosout_receipts.jsonl'
    assert result['receipt_count'] == 11000
    assert result['warning_receipt_count'] == 11000
    assert result['record_count'] == 1
    records = [json.loads(line) for line in
               (tmp_path / 'warnings.jsonl').read_text(
                   encoding='utf-8').splitlines()]
    assert records[0]['occurrence_count'] == 11000


def test_warning_metrics_accepts_valid_zero_warning_receipts(tmp_path):
    (tmp_path / 'rosout_receipts.jsonl').write_text(
        json.dumps({'level': 20, 'name': 'node', 'message': 'info',
                    'wall_time_utc': 't'}) + '\n', encoding='utf-8')

    result = _warning_metrics(tmp_path)

    assert result['available'] is True
    assert result['counts'] == {}
    assert result['warning_receipt_count'] == 0
    assert (tmp_path / 'warnings.jsonl').read_text(encoding='utf-8') == ''


def test_warning_metrics_missing_or_corrupt_raw_evidence_fails_closed(tmp_path):
    missing = _warning_metrics(tmp_path)
    assert missing['available'] is False

    (tmp_path / 'rosout_receipts.jsonl').write_text(
        '{not-json}\n', encoding='utf-8')
    corrupt = _warning_metrics(tmp_path)
    assert corrupt['available'] is False
    assert not (tmp_path / 'warnings.jsonl').exists()
