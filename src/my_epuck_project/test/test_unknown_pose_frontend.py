"""Deterministic tests for descriptor-first unknown-pose discovery."""

import math

import numpy as np

from my_epuck_project.unknown_pose_frontend_core import (
    GridCrop,
    compare_descriptors,
    crop_grid,
    descriptor_checksum,
    hypothesis_is_acceptable,
    polar_descriptor,
    register_crops,
    rigidify_affine,
    temporal_consistency,
)


def scene(size=160):
    values = np.full((size, size), -1, dtype=np.int16)
    values[20:-20, 20:-20] = 0
    values[35:40, 30:130] = 100
    values[35:125, 30:35] = 100
    values[115:120, 30:130] = 100
    values[75:115, 90:95] = 100
    values[75:80, 90:130] = 100
    values[48:65, 112:118] = 100
    values[48:54, 112:135] = 100
    return values


def transform_grid(values, yaw, tx_cells, ty_cells):
    result = np.full_like(values, -1)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    occupied = np.argwhere(values >= 50)
    center = (np.asarray(values.shape, dtype=np.float64) - 1.0) / 2.0
    for row, col in occupied:
        x, y = float(col) - center[1], float(row) - center[0]
        target_x = cosine * x - sine * y + center[1] + tx_cells
        target_y = sine * x + cosine * y + center[0] + ty_cells
        ix, iy = int(round(target_x)), int(round(target_y))
        if 0 <= iy < len(values) and 0 <= ix < values.shape[1]:
            result[iy, ix] = 100
    known = values != -1
    result[known & (result == -1)] = 0
    return result


def test_descriptor_checksum_and_unknown_handling_are_deterministic():
    crop = GridCrop(scene(), 0.05, -4.0, -4.0)
    descriptor = polar_descriptor(crop)
    assert len(descriptor) == 12 * 24 * 2
    assert descriptor_checksum(descriptor) == descriptor_checksum(descriptor)
    assert compare_descriptors(descriptor, descriptor).similarity > 0.99


def test_translation_and_yaw_offset_have_a_cheap_descriptor_match():
    first = GridCrop(scene(), 0.05, 0.0, 0.0)
    second = GridCrop(transform_grid(scene(), 0.20, 8, -5), 0.05, 0.0, 0.0)
    match = compare_descriptors(polar_descriptor(first), polar_descriptor(second))
    assert match.similarity >= 0.72
    assert match.margin >= 0.005


def test_no_overlap_and_repetitive_weak_scene_do_not_pass_geometry():
    first = GridCrop(scene(), 0.05, 0.0, 0.0)
    empty = np.full_like(first.values, -1)
    empty[10:20, 10:20] = 100
    result = register_crops(first, GridCrop(empty, 0.05, 30.0, 30.0))
    assert not result.accepted
    assert result.reason in {'NO_COARSE_ALIGNMENT', 'GEOMETRIC_VERIFICATION_REJECTED'}


def test_partial_overlap_is_verified_by_rigid_registration():
    values = scene()
    shifted = transform_grid(values, 0.10, 5, 3)
    first = GridCrop(values, 0.05, 0.0, 0.0)
    second = GridCrop(shifted, 0.05, 0.0, 0.0)
    result = register_crops(first, second)
    assert result.inlier_ratio > 0.35
    assert math.isfinite(result.residual_m)
    assert result.transform[2] == result.transform[2]


def test_rejected_affine_scale_or_shear_never_becomes_an_se2_hypothesis():
    assert rigidify_affine(np.array([[1.2, 0.0, 1.0], [0.0, 1.2, 2.0]])) is None
    assert rigidify_affine(np.array([[1.0, 0.2, 1.0], [0.0, 1.0, 2.0]])) is None
    assert rigidify_affine(np.array([[0.0, -1.0, 1.0], [1.0, 0.0, 2.0]])) is not None


def test_confidence_and_handoff_gate_require_mutual_acceptance():
    assert not hypothesis_is_acceptable('PROPOSED', True, 0.99)
    assert not hypothesis_is_acceptable('ACCEPTED', False, 0.99)
    assert not hypothesis_is_acceptable('ACCEPTED', True, 0.64)
    assert hypothesis_is_acceptable('ACCEPTED', True, 0.65)


def test_temporal_consistency_rejects_single_weak_or_distant_evidence():
    assert temporal_consistency([1]) == 0.5
    assert temporal_consistency([1, 2_000_000_000]) > 0.5
    assert temporal_consistency([1, 10_000_000_001]) == 0.0


def test_crop_is_bounded_and_preserves_unknown_cells():
    values = np.full((400, 400), -1, dtype=np.int16)
    values[180:220, 180:220] = 100
    crop = crop_grid(values, 0.05, -10.0, -10.0, size_m=8.0)
    assert crop.values.shape == (160, 160)
    assert np.count_nonzero(crop.values == -1) > 0


def test_only_mutually_accepted_hypothesis_reaches_existing_merger_boundary():
    class ExistingMergerBoundary:
        def __init__(self):
            self.handoffs = []

        def handoff(self, status, accepted, confidence):
            if hypothesis_is_acceptable(status, accepted, confidence):
                self.handoffs.append((status, confidence))

    merger = ExistingMergerBoundary()
    merger.handoff('PROPOSED', True, 0.99)
    merger.handoff('REJECTED', False, 0.99)
    merger.handoff('ACCEPTED', True, 0.80)
    assert merger.handoffs == [('ACCEPTED', 0.80)]
