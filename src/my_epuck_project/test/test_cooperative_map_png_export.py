"""Tests for the dependency-free cooperative map PNG exporter."""

import json
import struct
from pathlib import Path

from my_epuck_project.cooperative_map_png_export import (
    DIFFERENCE_RGB,
    FREE_RGB,
    OCCUPIED_RGB,
    UNCERTAIN_RGB,
    UNKNOWN_RGB,
    difference_rgb,
    occupancy_rgb,
    selected_attempt,
    write_rgb_png,
)

import numpy as np


def test_occupancy_colors_orientation_and_scale():
    """Occupancy colors, ROS orientation, and integer scaling are preserved."""
    data = np.asarray([
        [-1, 0],
        [100, 50],
    ], dtype=np.int8)
    result = occupancy_rgb(data, scale=2)
    assert result.shape == (4, 4, 3)
    assert tuple(result[0, 0]) == OCCUPIED_RGB
    assert tuple(result[0, 2]) == UNCERTAIN_RGB
    assert tuple(result[2, 0]) == UNKNOWN_RGB
    assert tuple(result[2, 2]) == FREE_RGB


def test_difference_marks_only_changed_cells():
    """Only cells whose raw occupancy differs are highlighted."""
    first = np.asarray([[0, -1], [100, 0]], dtype=np.int8)
    second = np.asarray([[0, -1], [0, 0]], dtype=np.int8)
    result = difference_rgb(first, second, scale=1)
    assert tuple(result[0, 0]) == DIFFERENCE_RGB
    assert np.count_nonzero(np.all(result == DIFFERENCE_RGB, axis=2)) == 1


def test_dependency_free_png_has_expected_dimensions(tmp_path):
    """The writer emits a valid PNG header with the requested dimensions."""
    image = np.zeros((7, 11, 3), dtype=np.uint8)
    path = tmp_path / 'map.png'
    write_rgb_png(path, image)
    content = path.read_bytes()
    assert content.startswith(b'\x89PNG\r\n\x1a\n')
    width, height = struct.unpack('>II', content[16:24])
    assert (width, height) == (11, 7)
    assert not list(tmp_path.glob('*.tmp'))


def test_manifest_schema_example_is_strict_json(tmp_path):
    """Boolean and count manifest values remain strict JSON."""
    path = Path(tmp_path) / 'manifest.json'
    value = {'identical': True, 'different_cell_count': 0}
    path.write_text(json.dumps(value, allow_nan=False))
    assert json.loads(path.read_text()) == value


def test_interrupted_attempt_requires_explicit_opt_in(tmp_path):
    """An interrupted attempt is exportable only with explicit selection."""
    campaign = tmp_path / 'regression_interrupted'
    attempt = campaign / 'attempts' / 'trial_01_attempt_01'
    attempt.mkdir(parents=True)
    (campaign / 'campaign_progress.json').write_text(
        json.dumps({'valid_trials': {}}), encoding='utf-8')
    for name in ('robot1_final_shared_map.npz', 'robot2_final_shared_map.npz'):
        (attempt / name).write_bytes(b'placeholder')

    try:
        selected_attempt(campaign, 'trial_01')
    except ValueError as error:
        assert 'no valid selected trial' in str(error)
    else:
        raise AssertionError('interrupted attempt was selected implicitly')

    selected, trial = selected_attempt(
        campaign, 'trial_01', allow_incomplete=True)
    assert selected == attempt
    assert trial == 'trial_01'
