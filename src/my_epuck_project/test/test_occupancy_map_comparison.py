import json
import math
import time

from my_epuck_project.occupancy_map_comparison import (
    canonical_map,
    classify,
    compare_maps,
    compare_semantic,
    consensus_maps,
    FREE,
    Geometry,
    load_map,
    OccupancyMap,
    OCCUPIED,
    save_map,
    UNCERTAIN,
    UNKNOWN,
)
import numpy as np
import pytest


def grid(data, resolution=1.0, x=0.0, y=0.0, yaw=0.0):
    data = np.asarray(data, dtype=np.int8)
    geometry = Geometry(
        data.shape[1], data.shape[0], resolution, x, y, yaw)
    return OccupancyMap(data, geometry, {'frame_id': 'shared_map'})


def test_identical_maps_have_perfect_available_metrics():
    item = grid([[-1, 0, 50, 100], [0, 0, 100, 100]])
    metrics, _, _, _ = compare_maps(item, item)
    assert metrics['known_iou'] == 1.0
    assert metrics['free_iou'] == 1.0
    assert metrics['occupied_iou'] == 1.0
    assert metrics['known_cell_agreement'] == 1.0
    assert metrics['occupied_free_conflict_rate'] == 0.0
    assert metrics['unknown_mismatch_rate'] == 0.0


def test_unknown_mismatch_and_occupied_free_conflict():
    left = grid([[-1, 0], [100, 0]])
    right = grid([[0, 100], [0, 0]])
    metrics, _, _, _ = compare_maps(left, right)
    assert metrics['unknown_mismatch_rate'] == pytest.approx(0.25)
    assert metrics['occupied_free_conflict_rate'] == pytest.approx(2 / 3)


def test_uncertain_cells_remain_explicit():
    classes = classify(np.array([-1, 0, 25, 26, 64, 65, 100]))
    assert classes.tolist() == [
        UNKNOWN, FREE, FREE, UNCERTAIN, UNCERTAIN, OCCUPIED, OCCUPIED]


def test_translated_and_negative_origins_align_in_world_coordinates():
    left = grid([[0, 100]], x=-1.0)
    right = grid([[100]], x=0.0)
    metrics, geometry, a, b = compare_maps(left, right)
    assert geometry.origin_x == -1.0
    assert metrics['occupied_iou'] == 1.0
    assert metrics['unknown_mismatch_rate'] == pytest.approx(0.5)
    assert a.shape == b.shape == (1, 2)


def test_different_extents_and_compatible_resolutions():
    coarse = grid([[0, 100]], resolution=1.0)
    fine = grid(
        [[0, 0, 100, 100], [0, 0, 100, 100]], resolution=0.5)
    metrics, target, _, _ = compare_maps(coarse, fine, resolution=0.5)
    assert target.width == 4
    assert metrics['known_iou'] == 1.0
    assert metrics['free_iou'] == 1.0
    assert metrics['occupied_iou'] == 1.0


def test_rotated_origin_is_resampled_by_inverse_transform():
    rotated = grid([[100]], x=1.0, y=0.0, yaw=math.pi / 2)
    axis = grid([[100]], x=0.0, y=0.0)
    metrics, _, _, _ = compare_maps(rotated, axis)
    assert metrics['occupied_iou'] == 1.0


def test_zero_variance_correlation_is_unavailable():
    metrics = compare_semantic(
        np.full((5, 5), FREE, dtype=np.uint8),
        np.full((5, 5), FREE, dtype=np.uint8), 1.0)
    assert metrics['raw_occupancy_correlation'] is None


def test_small_shift_diagnostic_does_not_change_zero_shift_iou():
    left = np.zeros((10, 10), dtype=np.uint8)
    right = np.zeros((10, 10), dtype=np.uint8)
    left[:, :5] = FREE
    left[:, 5:] = OCCUPIED
    right[:, 1:6] = FREE
    right[:, 6:] = OCCUPIED
    metrics = compare_semantic(left, right, 0.1, shift_window=3)
    assert metrics['best_shift_dx_cells'] != 0
    assert metrics['known_iou'] < 1.0


def test_single_cell_boundary_difference_exact_and_tolerant():
    left = np.full((7, 7), FREE, dtype=np.uint8)
    right = left.copy()
    left[3, 3] = OCCUPIED
    right[3, 4] = OCCUPIED
    metrics = compare_semantic(left, right, 1.0)
    assert metrics['occupied_iou'] == 0.0
    assert metrics['boundary_tolerant_occupied_agreement'] == 1.0


def test_canonical_union_preserves_conflict_mask():
    first = grid([[-1, 0, 100]])
    second = grid([[0, -1, 0]])
    canonical, counts, semantic = canonical_map(first, second)
    assert counts['robot1_only_known_cells'] == 1
    assert counts['robot2_only_known_cells'] == 1
    assert counts['jointly_known_conflicting_cells'] == 1
    assert canonical.conflict.tolist() == [[False, False, True]]
    assert semantic[0, 2] == OCCUPIED


def test_consensus_and_disagreement_frequency():
    maps = [
        grid([[0, 100, -1]]),
        grid([[0, 0, -1]]),
        grid([[0, 100, 0]]),
    ]
    result = consensus_maps(maps)
    assert result['known_frequency'][0, 0] == 1.0
    assert result['occupied_frequency'][0, 1] == pytest.approx(2 / 3)
    assert result['disagreement_frequency'][0, 1] == pytest.approx(1 / 3)
    assert result['majority_semantic'][0, 2] == UNKNOWN


def test_lossless_round_trip_and_no_nonfinite_json(tmp_path):
    item = grid([[-1, 0, 50, 100]], x=-2.0)
    path = tmp_path / 'map.npz'
    metadata = save_map(path, item)
    loaded = load_map(path)
    assert np.array_equal(loaded.data, item.data)
    assert loaded.data.dtype == np.int8
    assert json.dumps(metadata, allow_nan=False)


def test_vectorized_representative_map_performance():
    rng = np.random.default_rng(42)
    data = rng.choice([-1, 0, 50, 100], size=(600, 800)).astype(np.int8)
    left = grid(data, resolution=0.01, x=-4.0, y=-3.0)
    right = grid(data.copy(), resolution=0.01, x=-4.0, y=-3.0)
    start = time.perf_counter()
    metrics, _, _, _ = compare_maps(left, right, shift_window=1)
    assert metrics['known_iou'] == 1.0
    assert time.perf_counter() - start < 5.0
