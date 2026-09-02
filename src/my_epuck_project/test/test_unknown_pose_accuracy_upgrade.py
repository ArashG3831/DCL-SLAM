"""Deterministic accuracy and long-baseline gates for unknown-pose matching."""

import math
from types import SimpleNamespace

import numpy as np

import my_epuck_project.unknown_pose_frontend_core as frontend_core
from my_epuck_project.unknown_pose_frontend import UnknownPoseFrontend
from my_epuck_project.unknown_pose_frontend_core import (
    GridCrop,
    RegistrationResult,
    bounded_candidate_verification_order,
    compose_se2,
    compare_descriptors,
    compare_descriptor_pairs,
    invert_se2,
    polar_descriptor,
    projected_registration_error,
    physical_candidate_geometry_identity,
    register_crop_set,
    register_crops,
    _field_distances,
    _translation_grid_field_distances,
    _distance_field,
    wrap_angle,
)
from my_epuck_project.robust_relative_pose_selector import (
    ACCEPTED_HYPOTHESIS,
    AMBIGUOUS_HYPOTHESES,
    INSUFFICIENT_EVIDENCE,
    IncrementalHypothesisAccumulator,
    PoseConstraint,
    select_robust_hypothesis,
)


def _scalar_polar_descriptor_reference(crop, rings=12, sectors=24):
    """Reference implementation for the optimized descriptor reduction."""
    values = np.asarray(crop.values)
    height, width = values.shape
    yy, xx = np.indices((height, width), dtype=np.float64)
    cx, cy = (width - 1) / 2.0, (height - 1) / 2.0
    dx = (xx - cx) * crop.resolution
    dy = (yy - cy) * crop.resolution
    radius = np.hypot(dx, dy)
    max_radius = max(crop.resolution, float(radius.max()))
    ring_index = np.minimum(
        rings - 1, (radius / max_radius * rings).astype(np.int32))
    angle = (np.arctan2(dy, dx) + 2.0 * math.pi) % (2.0 * math.pi)
    sector_index = np.minimum(
        sectors - 1,
        (angle / (2.0 * math.pi) * sectors).astype(np.int32))
    known = values != -1
    occupied = known & (values >= 100)
    total = np.zeros((rings, sectors), dtype=np.float64)
    known_count = np.zeros_like(total)
    occupied_count = np.zeros_like(total)
    for ring in range(rings):
        for sector in range(sectors):
            mask = (ring_index == ring) & (sector_index == sector)
            total[ring, sector] = float(np.count_nonzero(mask))
            known_count[ring, sector] = float(np.count_nonzero(mask & known))
            occupied_count[ring, sector] = float(
                np.count_nonzero(mask & occupied))
    known_fraction = np.divide(
        known_count, total, out=np.zeros_like(total), where=total > 0.0)
    occupied_fraction = np.divide(
        occupied_count, known_count, out=np.zeros_like(total),
        where=known_count > 0.0)
    encoded = np.empty((rings, sectors, 2), dtype=np.uint8)
    encoded[:, :, 0] = np.rint(occupied_fraction * 255.0).astype(np.uint8)
    encoded[:, :, 1] = np.rint(known_fraction * 255.0).astype(np.uint8)
    return encoded.tobytes()


def structured_scene(size=180):
    values = np.full((size, size), -1, dtype=np.int16)
    values[12:-12, 12:-12] = 0
    values[25:29, 25:150] = 100
    values[25:145, 25:29] = 100
    values[140:144, 25:150] = 100
    values[70:140, 83:87] = 100
    values[70:74, 83:130] = 100
    values[40:60, 120:124] = 100
    values[40:44, 120:145] = 100
    values[102:106, 42:75] = 100
    return values


def translated_crop(values, resolution, origin_x, origin_y, tx, ty):
    """Same local scene with a known translation between map frames."""
    return GridCrop(values.copy(), resolution, origin_x + tx, origin_y + ty)


def test_se2_composition_and_inverse_are_explicit_and_closed():
    first = (1.2, -0.3, 0.2)
    second = (-0.4, 0.8, -0.1)
    identity = compose_se2(first, invert_se2(first))
    assert np.allclose(identity[:2], (0.0, 0.0), atol=1e-9)
    assert abs(identity[2]) < 1e-9
    assert compose_se2(first, second) == compose_se2(
        first, second)
    assert abs(wrap_angle(compose_se2(first, second)[2] - 0.1)) < 1e-9


def test_vectorized_polar_descriptor_matches_scalar_reference():
    rng = np.random.default_rng(42)
    values = rng.choice(np.asarray([-1, 0, 100], dtype=np.int16),
                        size=(37, 43), p=[0.25, 0.60, 0.15])
    crop = GridCrop(values, 0.05, -1.2, 0.7, math.radians(13.0))
    expected = _scalar_polar_descriptor_reference(crop)
    actual = polar_descriptor(crop)
    assert actual == expected
    assert compare_descriptors(actual, expected).similarity == 1.0


def test_descriptor_pair_batch_matches_individual_scores():
    """Batching removes setup work without changing any pair result."""
    rng = np.random.default_rng(17)
    first = [rng.bytes(12 * 24 * 2) for _ in range(4)]
    second = [rng.bytes(12 * 24 * 2) for _ in range(4)]
    batched = compare_descriptor_pairs(first, second)
    individual = tuple(compare_descriptors(a, b) for a, b in zip(first, second))
    assert batched == individual


def test_translation_grid_sampling_matches_world_sampling():
    values = np.zeros((60, 70), dtype=np.int16)
    values[::5, 8:55] = 100
    crop = GridCrop(values, 0.05, -1.3, 2.7, math.radians(17.0))
    field = _distance_field(crop)
    offsets = np.stack(np.meshgrid(
        np.arange(-0.4, 0.401, 0.05),
        np.arange(-0.4, 0.401, 0.05), indexing='ij'), axis=-1).reshape(-1, 2)
    rotated = np.asarray([[0.3, 1.1], [1.7, 2.4], [2.2, 0.8]])
    base = np.asarray([-0.2, 0.4])
    world_points = (
        rotated[None, :, :] + base[None, None, :] + offsets[:, None, :]
    ).reshape(-1, 2)
    reference = _field_distances(
        world_points, crop, field).reshape(len(offsets), len(rotated))
    optimized = _translation_grid_field_distances(
        rotated, base, crop, field, offsets)
    assert np.array_equal(optimized, reference)


def test_map_origin_translation_is_not_double_applied():
    values = structured_scene()
    source = GridCrop(values, 0.05, -3.0, 2.0)
    target = translated_crop(values, 0.05, -3.0, 2.0, 0.7, -0.2)
    result = register_crops(source, target)
    assert result.accepted
    assert np.linalg.norm(np.asarray(result.transform[:2]) - (0.7, -0.2)) < 0.08
    assert abs(result.transform[2]) < math.radians(0.25)


def test_rotated_map_origin_is_applied_once():
    values = structured_scene()
    yaw = math.radians(17.0)
    source = GridCrop(values, 0.05, -3.0, 2.0, yaw)
    target = GridCrop(values, 0.05, -2.3, 1.8, yaw)
    result = register_crops(source, target)
    assert result.accepted
    assert np.linalg.norm(np.asarray(result.transform[:2]) - (0.7, -0.2)) < 0.08
    assert abs(result.transform[2]) < math.radians(0.25)


def test_registration_quality_is_direction_invariant_for_asymmetric_maps():
    """Peer verification must score one physical pair identically either way."""
    values = structured_scene()
    target_values = values.copy()
    # A small map-only addition makes the two views intentionally asymmetric;
    # it must not make the quality metric depend on the verifier direction.
    target_values[90:110, 130:140] = 100
    source = GridCrop(values, 0.05, 0.0, 0.0)
    target = GridCrop(target_values, 0.05, 0.7, -0.2)
    forward = register_crops(source, target)
    reverse = register_crops(target, source)
    assert forward.accepted == reverse.accepted
    assert np.allclose(
        np.asarray(reverse.transform[:2]), -np.asarray(forward.transform[:2]),
        atol=0.08)
    assert abs(forward.occupied_free_agreement -
               reverse.occupied_free_agreement) < 1e-9
    assert abs(forward.overlap_fraction - reverse.overlap_fraction) < 1e-9


def test_long_baseline_projected_error_rejects_one_degree_uncertainty():
    assert projected_registration_error((0.02, 0.01, math.radians(1.0)), 40.0) > 0.6
    assert projected_registration_error((0.03, 0.02, math.radians(0.2)), 40.0) < 0.20


def test_multi_keyframe_consensus_rejects_one_wrong_constraint():
    values = structured_scene()
    pairs = [
        (GridCrop(values, 0.05, 0.0 + i, 0.0),
         translated_crop(values, 0.05, 0.0 + i, 0.0, 0.7, -0.2))
        for i in (0.0, 1.0, 2.0)
    ]
    pairs.append((
        GridCrop(values, 0.05, 3.0, 0.0),
        translated_crop(values, 0.05, 3.0, 0.0, 2.2, 1.1)))
    result = register_crop_set(
        pairs, min_consistent_constraints=3,
        min_spatial_baseline_m=0.75,
        max_projected_registration_error_m=0.20)
    assert result.accepted
    assert result.constraint_count == 4
    assert result.consistent_constraint_count >= 3
    assert np.linalg.norm(np.asarray(result.transform[:2]) - (0.7, -0.2)) < 0.10
    assert result.projected_error_m < 0.20


def _selector_constraint(transform, index, quality=0.95, source_center=None,
                         target_center=None, source_viewpoint=None,
                         target_viewpoint=None, source_viewpoint_required=False,
                         target_viewpoint_required=False):
    return PoseConstraint(
        transform=tuple(transform),
        covariance=(0.03 ** 2, 0.0, 0.0, 0.0, 0.03 ** 2,
                    0.0, 0.0, 0.0, math.radians(0.35) ** 2),
        quality=quality,
        evidence_id=f'evidence-{index}',
        source_center=(float(index), 0.0) if source_center is None else source_center,
        target_center=((float(index) + 0.7, -0.2)
                      if target_center is None else target_center),
        source_viewpoint=source_viewpoint,
        target_viewpoint=target_viewpoint,
        source_viewpoint_required=source_viewpoint_required,
        target_viewpoint_required=target_viewpoint_required,
        source_timestamp_ns=index * 1_000_000_000,
        target_timestamp_ns=(index + 1) * 1_000_000_000)


def test_robust_selector_rejects_current_internally_inconsistent_three_set():
    """The persisted 1.313-degree pair must not reach handoff."""
    selection = select_robust_hypothesis([
        _selector_constraint((-2.7218818, 0.0085150, -0.0130691), 0),
        _selector_constraint((-2.6999520, 0.0129301, -0.0239829), 1),
        _selector_constraint((-2.7715699, 0.0130774, -0.0010675), 2),
    ], min_inliers=3)
    assert selection.status != ACCEPTED_HYPOTHESIS
    # The diagnostics may retain the high-probability set for forensic
    # reporting even when the structural compatibility gate rejects it.
    assert selection.runner_up_margin >= 0.0


def test_incremental_accumulator_preserves_old_1313_degree_rejection():
    """Accumulation must not turn the known inconsistent set into a handoff."""
    accumulator = IncrementalHypothesisAccumulator()
    selection = accumulator.update([
        _selector_constraint((-2.7218818, 0.0085150, -0.0130691), 0),
        _selector_constraint((-2.6999520, 0.0129301, -0.0239829), 1),
        _selector_constraint((-2.7715699, 0.0130774, -0.0010675), 2),
    ])
    assert selection.status != ACCEPTED_HYPOTHESIS
    assert len(selection.selected_indices) < 3


def test_spatial_baseline_is_invariant_to_reverse_verification_direction():
    """A diverse target-frame baseline must survive peer-side inversion."""
    constraints = [
        _selector_constraint((0.7, -0.2, 0.03), index,
                             source_center=(0.0, 0.0),
                             target_center=(float(index), 0.0),
                             target_viewpoint=(float(index), 0.0))
        for index in range(3)]
    selection = select_robust_hypothesis(constraints, min_inliers=3)
    assert selection.status == ACCEPTED_HYPOTHESIS
    assert selection.diagnostics[0]['spatial_baseline_m'] >= 2.0


def test_selector_uses_physical_viewpoints_when_crop_centers_are_close():
    """Three physical views must not fail on a small crop-center baseline."""
    constraints = [
        PoseConstraint(
            transform=(-2.7, 0.0, 0.0),
            covariance=(0.03 ** 2, 0.0, 0.0, 0.0, 0.03 ** 2,
                        0.0, 0.0, 0.0, math.radians(0.35) ** 2),
            quality=0.95, evidence_id=f'physical-{index}',
            source_center=(0.0, 0.0), target_center=(0.1, 0.1),
            source_viewpoint=(float(index), 0.0),
            source_timestamp_ns=index * 2_000_000_000,
            target_timestamp_ns=(index + 1) * 2_000_000_000)
        for index in range(3)]
    selection = select_robust_hypothesis(constraints, min_inliers=3)
    assert selection.status == ACCEPTED_HYPOTHESIS
    assert abs(selection.diagnostics[0]['spatial_baseline_m'] - 2.0) < 1e-9


def test_missing_physical_viewpoints_cannot_use_crop_centers_for_baseline():
    constraints = [
        _selector_constraint((0.7, -0.2, 0.03), index,
                             source_center=(float(index), 0.0),
                             target_center=(float(index), 0.0),
                             source_viewpoint_required=True,
                             target_viewpoint_required=True)
        for index in range(3)]
    selection = select_robust_hypothesis(constraints, min_inliers=3)
    assert selection.status != ACCEPTED_HYPOTHESIS
    assert all(diagnostic.get('spatial_baseline_m', 0.0) == 0.0
               for diagnostic in selection.diagnostics
               if diagnostic.get('kind') == 'robust_hypothesis')


def test_selector_allows_quality_below_former_scalar_floor():
    """Explicit geometry and consensus, not scalar quality, decide inliers."""
    constraints = [
        PoseConstraint(
            transform=(-2.7, 0.0, 0.0),
            covariance=(0.03 ** 2, 0.0, 0.0, 0.0, 0.03 ** 2,
                        0.0, 0.0, 0.0, math.radians(0.35) ** 2),
            quality=0.50, evidence_id=f'low-quality-{index}',
            source_viewpoint=(float(index), 0.0),
            source_timestamp_ns=index * 2_000_000_000,
            target_timestamp_ns=(index + 1) * 2_000_000_000)
        for index in range(3)]
    selection = select_robust_hypothesis(constraints, min_inliers=3)
    assert selection.status == ACCEPTED_HYPOTHESIS
    assert set(selection.selected_indices) == {0, 1, 2}


def test_accumulator_does_not_count_duplicate_physical_evidence_twice():
    """Repeated evidence IDs cannot manufacture the three-inlier minimum."""
    accumulator = IncrementalHypothesisAccumulator()
    repeated = _selector_constraint((-2.7, 0.0, 0.0), 0)
    result = accumulator.update([
        repeated,
        repeated,
        _selector_constraint((-2.7, 0.0, 0.0), 1),
    ])
    assert result.status != ACCEPTED_HYPOTHESIS
    assert len(accumulator._constraints) == 2


def test_selector_rejects_inconsistent_pi_and_quarter_turn_families():
    """Inconsistent transform families remain below the three-inlier gate."""
    constraints = [
        _selector_constraint((-2.7, 0.0, yaw), index,
                             source_center=(float(index), 0.0),
                             target_center=(0.1, 0.1))
        for index, yaw in enumerate((0.0, math.pi / 2.0, math.pi))]
    selection = select_robust_hypothesis(constraints, min_inliers=3)
    assert selection.status != ACCEPTED_HYPOTHESIS
    assert len(selection.selected_indices) < 3


def test_incremental_accumulator_promotes_consistent_evidence_over_batches():
    accumulator = IncrementalHypothesisAccumulator()
    first = [_selector_constraint((0.7, -0.2, 0.03), i) for i in range(2)]
    assert accumulator.update(first).status != ACCEPTED_HYPOTHESIS
    result = accumulator.update([
        _selector_constraint((0.698, -0.201, 0.029), 2),
    ])
    assert result.status == ACCEPTED_HYPOTHESIS
    assert len(result.selected_indices) >= 3
    assert result.diagnostics[-1]['kind'] == 'incremental_hypothesis_accumulator'


def test_unknown_pose_minimum_inlier_floor_cannot_be_configured_below_three():
    """Every estimator entry point retains the three-constraint safety gate."""
    accumulator = IncrementalHypothesisAccumulator(min_inliers=2)
    assert accumulator.min_inliers == 3
    two = [
        _selector_constraint((0.7, -0.2, 0.03), index)
        for index in range(2)
    ]
    assert select_robust_hypothesis(two, min_inliers=2).status != ACCEPTED_HYPOTHESIS
    assert accumulator.update(two).status != ACCEPTED_HYPOTHESIS


def test_incremental_accumulator_retains_ambiguous_three_inlier_cluster():
    """A valid cluster must survive a batch containing a competing outlier."""
    accumulator = IncrementalHypothesisAccumulator()
    result = accumulator.update([
        _selector_constraint((0.7, -0.2, 0.03), 0),
        _selector_constraint((0.702, -0.198, 0.031), 1),
        _selector_constraint((0.698, -0.201, 0.029), 2),
        _selector_constraint((1.7, 1.0, 0.5), 3, quality=0.60),
    ])
    assert len(result.selected_indices) >= 3
    assert result.status == ACCEPTED_HYPOTHESIS


def test_robust_selector_accepts_clear_cluster_and_rejects_outlier():
    selection = select_robust_hypothesis([
        _selector_constraint((0.7, -0.2, 0.03), 0),
        _selector_constraint((0.702, -0.198, 0.031), 1),
        _selector_constraint((0.698, -0.201, 0.029), 2),
        _selector_constraint((1.7, 1.0, 0.5), 3, quality=0.60),
    ], min_inliers=3)
    assert selection.status == ACCEPTED_HYPOTHESIS
    assert set(selection.selected_indices) == {0, 1, 2}


def test_robust_selector_rejects_nearby_yaw_outlier_from_six_candidates():
    """A close-but-inconsistent registration must not poison the cluster."""
    values = [
        (-2.7962477372, -0.0142963933, 0.0226977369),
        (-2.7726778499, 0.0002668353, 0.0),
        (-2.7726778499, 0.0002668353, 0.0),
        (-2.7726778499, -0.0297331641, 0.0),
        (-2.7297047653, 0.0148109169, -0.0102339061),
        (-2.7663514397, -0.0554152723, 0.0059169707),
    ]
    constraints = []
    for index, transform in enumerate(values):
        constraints.append(PoseConstraint(
            transform=transform,
            covariance=(0.0006 ** 2, 0.0, 0.0,
                        0.0, 0.0006 ** 2, 0.0,
                        0.0, 0.0, math.radians(0.0004) ** 2),
            quality=0.95,
            evidence_id=f'near-outlier-{index}',
            source_center=(float(index), 0.0),
            target_center=(float(index) + 0.7, -0.2),
            source_timestamp_ns=index * 1_000_000_000,
            target_timestamp_ns=(index + 1) * 1_000_000_000))

    selection = select_robust_hypothesis(constraints, min_inliers=3)
    assert selection.status == ACCEPTED_HYPOTHESIS
    assert set(selection.selected_indices) == {1, 2, 3, 4, 5}


def test_robust_selector_refuses_competing_hypotheses():
    first = [
        (1.0, 0.0, 0.02), (1.002, 0.001, 0.021),
        (0.998, -0.001, 0.019),
    ]
    second = [
        (3.0, 1.0, 0.40), (3.002, 1.001, 0.401),
        (2.998, 0.999, 0.399),
    ]
    selection = select_robust_hypothesis([
            _selector_constraint(value, index, source_center=(float(index), 0.0))
        for index, value in enumerate(first + second)
    ], min_inliers=3)
    assert selection.status in (AMBIGUOUS_HYPOTHESES, INSUFFICIENT_EVIDENCE)


def test_robust_selector_null_refuses_all_outliers():
    selection = select_robust_hypothesis([
        _selector_constraint((0.0, 0.0, 0.0), 0),
        _selector_constraint((4.0, 2.0, 1.0), 1),
        _selector_constraint((-4.0, -2.0, -1.0), 2),
    ], min_inliers=3)
    assert selection.status != ACCEPTED_HYPOTHESIS


def test_robust_selector_handles_yaw_wraparound():
    selection = select_robust_hypothesis([
        _selector_constraint((1.0, 0.0, math.pi - 0.004), 0),
        _selector_constraint((1.001, 0.001, -math.pi + 0.004), 1),
        _selector_constraint((0.999, -0.001, math.pi - 0.002), 2),
    ], min_inliers=3)
    assert selection.status == ACCEPTED_HYPOTHESIS


def _synthetic_consensus_result(transform, accepted=True):
    return RegistrationResult(
        accepted=accepted,
        transform=transform,
        covariance=tuple([0.0] * 36),
        inlier_ratio=0.90,
        residual_m=0.02,
        occupied_free_agreement=0.90,
        overlap_fraction=0.80,
        reason='ACCEPTED' if accepted else 'GEOMETRIC_VERIFICATION_REJECTED',
        median_residual_m=0.02,
        p95_residual_m=0.03,
        translation_uncertainty_m=0.005,
        yaw_uncertainty_rad=0.001,
        condition_number=10.0,
        projected_error_m=0.045,
        final_confidence=0.9)


def _synthetic_pairs(count):
    values = structured_scene(size=80)
    return [
        (GridCrop(values, 0.05, float(index), 0.0),
         GridCrop(values, 0.05, float(index) + 0.7, -0.2))
        for index in range(count)
    ]


def test_consensus_diagnostics_accepts_three_mutually_consistent_constraints(
        monkeypatch):
    transforms = [(0.7, -0.2, 0.03)] * 3
    monkeypatch.setattr(
        frontend_core, 'register_crops',
        lambda source, target: _synthetic_consensus_result(transforms.pop(0)))
    result = register_crop_set(_synthetic_pairs(3), min_consistent_constraints=3)
    subsets = [item for item in result.consensus_diagnostics
               if item['kind'] == 'subset_comparison']
    assert result.accepted
    assert result.consistent_constraint_count == 3
    assert len(subsets) == 1
    assert subsets[0]['consistent_subset']
    assert subsets[0]['rejection_reasons'] == []


def test_consensus_diagnostics_identifies_three_consistent_constraints_plus_outlier(
        monkeypatch):
    transforms = [(0.7, -0.2, 0.03)] * 3 + [(1.4, 0.8, 0.5)]
    monkeypatch.setattr(
        frontend_core, 'register_crops',
        lambda source, target: _synthetic_consensus_result(transforms.pop(0)))
    result = register_crop_set(_synthetic_pairs(4), min_consistent_constraints=3)
    subsets = [item for item in result.consensus_diagnostics
               if item['kind'] == 'subset_comparison']
    assert result.accepted
    assert result.consistent_constraint_count >= 3
    assert any(item['consistent_subset'] for item in subsets)
    assert any(3 in item['indices'] for item in subsets
               if 'PAIRWISE_TRANSFORM_INCONSISTENT' in item['rejection_reasons'])


def test_late_fourth_valid_constraint_rescues_early_outlier(monkeypatch):
    transforms = [(1.4, 0.8, 0.5), (0.7, -0.2, 0.03),
                  (0.7, -0.2, 0.03), (0.7, -0.2, 0.03)]
    monkeypatch.setattr(
        frontend_core, 'register_crops',
        lambda source, target: _synthetic_consensus_result(transforms.pop(0)))

    result = register_crop_set(_synthetic_pairs(4), min_consistent_constraints=3)

    assert result.accepted
    assert result.constraint_count == 4
    assert result.consistent_constraint_count >= 3
    assert any(item['consistent_subset'] for item in result.consensus_diagnostics
               if item['kind'] == 'subset_comparison')


def test_consensus_diagnostics_rejects_four_mutually_inconsistent_constraints(
        monkeypatch):
    transforms = [(0.0, 0.0, 0.0), (0.4, 0.0, 0.0),
                  (0.0, 0.4, 0.0), (0.4, 0.4, 0.0)]
    monkeypatch.setattr(
        frontend_core, 'register_crops',
        lambda source, target: _synthetic_consensus_result(transforms.pop(0)))
    result = register_crop_set(_synthetic_pairs(4), min_consistent_constraints=3)
    subsets = [item for item in result.consensus_diagnostics
               if item['kind'] == 'subset_comparison']
    assert not result.accepted
    assert result.reason == 'INSUFFICIENT_CONSISTENT_CONSTRAINTS'
    assert subsets
    assert not any(item['consistent_subset'] for item in subsets)
    assert all('PAIRWISE_TRANSFORM_INCONSISTENT' in item['rejection_reasons']
               for item in subsets)


def test_runtime_five_constraint_pattern_rejects_three_false_matches(
        monkeypatch):
    """Reproduce the persisted run: two good transforms, three outliers."""
    transforms = [
        (0.00674275, 2.50899375, 0.00666587),
        (0.02278460, 2.50248557, 0.00220515),
        (-0.07202596, 3.50310318, 0.03776201),
        (-0.06512076, 4.11538009, 0.02126475),
        (0.38732146, -0.00901558, -3.10136296),
    ]
    monkeypatch.setattr(
        frontend_core, 'register_crops',
        lambda source, target: _synthetic_consensus_result(
            transforms.pop(0)))

    result = register_crop_set(
        _synthetic_pairs(5), min_consistent_constraints=3,
        min_spatial_baseline_m=0.75)

    assert not result.accepted
    assert result.reason == 'INSUFFICIENT_CONSISTENT_CONSTRAINTS'
    assert result.constraint_count == 5
    assert result.consistent_constraint_count == 2
    subsets = [item for item in result.consensus_diagnostics
               if item['kind'] == 'subset_comparison']
    assert subsets
    assert not any(item['consistent_subset'] for item in subsets)


def test_bounded_verification_accepts_later_correct_candidates_after_outliers(
        monkeypatch):
    """The measured false-minimum pattern is recoverable by 3-of-N."""
    transforms = [
        (-0.07202596, 3.50310318, 0.03776201),
        (-0.06512076, 4.11538009, 0.02126475),
        (0.38732146, -0.00901558, -3.10136296),
        (0.00674275, 2.50899375, 0.00666587),
        (0.02278460, 2.50248557, 0.00220515),
        (0.01000000, 2.50100000, 0.00400000),
    ]
    monkeypatch.setattr(
        frontend_core, 'register_crops',
        lambda source, target: _synthetic_consensus_result(transforms.pop(0)))
    result = register_crop_set(
        _synthetic_pairs(6), min_consistent_constraints=3,
        min_spatial_baseline_m=0.75)
    assert result.accepted
    assert result.consistent_constraint_count >= 3
    assert result.projected_error_m < 0.20


def test_all_false_candidates_remain_rejected_by_bounded_consensus(monkeypatch):
    transforms = [
        (0.0, 0.0, 0.0), (0.4, 0.0, 0.0), (0.0, 0.4, 0.0),
        (0.4, 0.4, 0.0), (1.0, 1.0, 0.8),
    ]
    monkeypatch.setattr(
        frontend_core, 'register_crops',
        lambda source, target: _synthetic_consensus_result(transforms.pop(0)))
    result = register_crop_set(
        _synthetic_pairs(5), min_consistent_constraints=3,
        min_spatial_baseline_m=0.75)
    assert not result.accepted
    assert result.reason == 'INSUFFICIENT_CONSISTENT_CONSTRAINTS'


def test_candidate_verification_budget_is_deterministic_and_bounded():
    class UnhashableDescriptor:
        __hash__ = None

    candidates = [
        (-0.95, 'peer-2', 'own-2', UnhashableDescriptor(),
         UnhashableDescriptor()),
        (-0.99, 'peer-1', 'own-1', UnhashableDescriptor(),
         UnhashableDescriptor()),
        (-0.98, 'peer-1-duplicate', 'own-1', UnhashableDescriptor(),
         UnhashableDescriptor()),
        (-0.90, 'peer-3', 'own-3', UnhashableDescriptor(),
         UnhashableDescriptor()),
    ]
    ordered = bounded_candidate_verification_order(
        candidates, {('own-2', 'peer-2')}, budget=2)
    assert [(item[2], item[1]) for item in ordered] == [
        ('own-1', 'peer-1'), ('own-1', 'peer-1-duplicate')]
    assert len(ordered) == 2


def test_consensus_requires_temporal_span_when_runtime_timestamps_exist(
        monkeypatch):
    transforms = [(0.7, -0.2, 0.03)] * 3
    monkeypatch.setattr(
        frontend_core, 'register_crops',
        lambda source, target: _synthetic_consensus_result(transforms.pop(0)))
    result = register_crop_set(
        _synthetic_pairs(3), min_consistent_constraints=3,
        evidence_timestamps=[(10_000_000_000, 10_000_000_000)] * 3)
    assert not result.accepted


def test_rejected_geometry_is_stable_across_map_revisions():
    """A changed checksum must not retry the same physical crop footprint."""
    values = np.zeros((20, 20), dtype=np.int16)
    own_crop = GridCrop(values, 0.05, 1.0, 2.0)
    own = SimpleNamespace(map_epoch=1, checksum=11)
    peer_a = SimpleNamespace(
        map_epoch=4, checksum=101, resolution=0.05,
        crop_width=20, crop_height=20, crop_origin_x=3.0,
        crop_origin_y=4.0, crop_origin_yaw=0.0)
    peer_b = SimpleNamespace(**{**peer_a.__dict__, 'map_epoch': 5,
                                'checksum': 202})
    crops = {'own-a': own_crop, 'own-b': own_crop}
    first = (0.9, 'peer-a', 'own-a', peer_a, own)
    revision = (0.8, 'peer-b', 'own-b', peer_b, own)
    assert physical_candidate_geometry_identity(first, crops) == \
        physical_candidate_geometry_identity(revision, crops)


def test_accepted_geometry_revisions_are_not_independent_constraints():
    values = np.zeros((20, 20), dtype=np.int16)
    own_crop = GridCrop(values, 0.05, 1.0, 2.0)
    own = SimpleNamespace(map_epoch=1, checksum=11)
    peer_a = SimpleNamespace(
        map_epoch=4, checksum=101, resolution=0.05,
        crop_width=20, crop_height=20, crop_origin_x=3.0,
        crop_origin_y=4.0, crop_origin_yaw=0.0)
    peer_revision = SimpleNamespace(
        **{**peer_a.__dict__, 'map_epoch': 5, 'checksum': 202})
    crops = {'own-a': own_crop}
    accepted = (0.9, 'peer-a', 'own-a', peer_a, own)
    revision = (0.8, 'peer-revision', 'own-a', peer_revision, own)
    accepted_geometry = {physical_candidate_geometry_identity(accepted, crops)}

    assert physical_candidate_geometry_identity(revision, crops) in (
        accepted_geometry)


def test_active_evidence_batch_retries_when_new_candidates_arrive():
    """A batch opened before a candidate arrives remains requestable."""
    frontend = object.__new__(UnknownPoseFrontend)
    frontend.evidence_acquisition_started = True
    frontend.batch_proposal_published = False
    calls = []
    frontend._request_next_candidate_verification = (
        lambda: calls.append('request') or True)

    frontend._schedule_active_evidence_request()

    assert calls == ['request']


def test_completed_evidence_batch_does_not_retry():
    frontend = object.__new__(UnknownPoseFrontend)
    frontend.evidence_acquisition_started = False
    frontend.batch_proposal_published = False
    calls = []
    frontend._request_next_candidate_verification = (
        lambda: calls.append('request') or True)

    frontend._schedule_active_evidence_request()

    assert calls == []


def test_next_verification_candidate_prefers_spatially_displaced_view():
    frontend = object.__new__(UnknownPoseFrontend)
    frontend.evidence_pairs = {('own-0', 'peer-0'): (None, None)}
    frontend.keyframes = {}
    frontend.peer_descriptors = {}
    frontend.attempted_physical_view_reuse_counts = {}

    def crop(center):
        return GridCrop(np.zeros((2, 2), dtype=np.int16), 1.0,
                        center[0] - 1.0, center[1] - 1.0)

    def descriptor(center):
        return SimpleNamespace(
            crop_origin_x=center[0] - 1.0, crop_origin_y=center[1] - 1.0,
            crop_width=2, crop_height=2, resolution=1.0)

    frontend.keyframes['own-0'] = (SimpleNamespace(map_epoch=1, checksum=1),
                                   crop((0.0, 0.0)))
    frontend.keyframes['own-near'] = (
        SimpleNamespace(map_epoch=2, checksum=2), crop((0.1, 0.0)))
    frontend.keyframes['own-far'] = (
        SimpleNamespace(map_epoch=3, checksum=3), crop((1.2, 0.0)))
    frontend.peer_descriptors['peer-0'] = descriptor((0.0, 0.0))
    frontend.peer_descriptors['peer-near'] = descriptor((0.1, 0.0))
    frontend.peer_descriptors['peer-far'] = descriptor((1.2, 0.0))
    frontend._crop_geometry = lambda value, *args: {
        'center': [value.origin_x + 1.0, value.origin_y + 1.0]}
    frontend._descriptor_geometry = lambda value: {
        'center': [value.crop_origin_x + 1.0, value.crop_origin_y + 1.0]}

    near = (-0.99, 'peer-near', 'own-near',
            frontend.peer_descriptors['peer-near'], None)
    far = (-0.80, 'peer-far', 'own-far',
           frontend.peer_descriptors['peer-far'], None)

    assert frontend._candidate_spatial_novelty_key(far) < (
        frontend._candidate_spatial_novelty_key(near))


def test_next_verification_candidate_balances_both_crop_displacements():
    """A far local crop cannot outrank a genuinely displaced pair."""
    frontend = object.__new__(UnknownPoseFrontend)
    frontend.evidence_pairs = {('own-0', 'peer-0'): (None, None)}
    frontend.keyframes = {}
    frontend.peer_descriptors = {}
    frontend.attempted_physical_view_reuse_counts = {}

    def crop(center):
        return GridCrop(np.zeros((2, 2), dtype=np.int16), 1.0,
                        center[0] - 1.0, center[1] - 1.0)

    def descriptor(center):
        return SimpleNamespace(
            crop_origin_x=center[0] - 1.0, crop_origin_y=center[1] - 1.0,
            crop_width=2, crop_height=2, resolution=1.0)

    frontend.keyframes['own-0'] = (SimpleNamespace(map_epoch=1, checksum=1),
                                   crop((0.0, 0.0)))
    frontend.keyframes['own-far-peer-near'] = (
        SimpleNamespace(map_epoch=2, checksum=2), crop((2.0, 0.0)))
    frontend.keyframes['own-balanced'] = (
        SimpleNamespace(map_epoch=3, checksum=3), crop((1.0, 0.0)))
    frontend.peer_descriptors['peer-0'] = descriptor((0.0, 0.0))
    frontend.peer_descriptors['peer-near'] = descriptor((0.2, 0.0))
    frontend.peer_descriptors['peer-balanced'] = descriptor((1.0, 0.0))
    frontend._crop_geometry = lambda value, *args: {
        'center': [value.origin_x + 1.0, value.origin_y + 1.0]}
    frontend._descriptor_geometry = lambda value: {
        'center': [value.crop_origin_x + 1.0, value.crop_origin_y + 1.0]}

    far_local = (-0.90, 'peer-near', 'own-far-peer-near',
                 frontend.peer_descriptors['peer-near'], None)
    balanced = (-0.80, 'peer-balanced', 'own-balanced',
                frontend.peer_descriptors['peer-balanced'], None)

    assert frontend._candidate_spatial_novelty_key(balanced) < (
        frontend._candidate_spatial_novelty_key(far_local))


def test_one_pair_and_repetitive_or_empty_geometry_do_not_meet_production_gate():
    values = structured_scene()
    pair = (GridCrop(values, 0.05, 0.0, 0.0),
            translated_crop(values, 0.05, 0.0, 0.0, 0.7, -0.2))
    result = register_crop_set(
        [pair], min_consistent_constraints=3,
        max_projected_registration_error_m=0.20)
    assert not result.accepted
    assert result.reason == 'INSUFFICIENT_CONSISTENT_CONSTRAINTS'

    unknown = np.full_like(values, -1)
    unknown[80:90, 80:90] = 100
    no_overlap = register_crop_set([
        (GridCrop(values, 0.05, 0.0, 0.0),
         GridCrop(unknown, 0.05, 30.0, 30.0))] * 3,
        min_consistent_constraints=3, min_spatial_baseline_m=0.75)
    assert not no_overlap.accepted


def test_descriptor_keeps_unknown_separate_from_occupied():
    values = structured_scene()
    descriptor = polar_descriptor(GridCrop(values, 0.05, 0.0, 0.0))
    unknown_as_occupied = values.copy()
    unknown_as_occupied[unknown_as_occupied == -1] = 100
    altered = polar_descriptor(GridCrop(unknown_as_occupied, 0.05, 0.0, 0.0))
    assert descriptor != altered
