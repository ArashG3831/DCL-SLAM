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
    invert_se2,
    polar_descriptor,
    projected_registration_error,
    physical_candidate_geometry_identity,
    register_crop_set,
    register_crops,
    wrap_angle,
)


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
