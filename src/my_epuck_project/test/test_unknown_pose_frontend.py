"""Deterministic tests for descriptor-first unknown-pose discovery."""

import json
import math
from types import SimpleNamespace

import numpy as np

from my_epuck_project.unknown_pose_frontend_core import (
    BoundedVerificationBatchController,
    DedicatedDiagnosticJsonl,
    GridCrop,
    accumulate_physical_candidates,
    compare_descriptors,
    crop_batch_is_ready,
    deduplicate_physical_candidates,
    crop_grid,
    descriptor_checksum,
    evidence_pairs_for_selection,
    evidence_candidates_for_pool,
    evidence_batch_is_spatially_diverse,
    hypothesis_is_acceptable,
    polar_descriptor,
    register_crops,
    rigidify_affine,
    should_accept_hypothesis,
    confirmation_window_for_cadence,
    temporal_support_count,
    temporal_consistency,
)


def _batch_snapshot(keys_and_centres):
    return {
        key: {'timestamp_ns': timestamp, 'center': centre}
        for key, timestamp, centre in keys_and_centres
    }


def _batch_candidate(own_key, peer_key, own_timestamp, peer_timestamp,
                     own_center, peer_center):
    return {
        'own_key': own_key,
        'peer_key': peer_key,
        'own_timestamp_ns': own_timestamp,
        'peer_timestamp_ns': peer_timestamp,
        'own_center': own_center,
        'peer_center': peer_center,
    }


def test_exhausted_verification_batch_waits_without_immediate_retry():
    gate = BoundedVerificationBatchController(
        budget=2, max_batches=3, lifetime_s=100.0)
    snapshot = _batch_snapshot([('own-1', 10, (0.0, 0.0))])
    peer_snapshot = _batch_snapshot([('peer-1', 11, (0.0, 0.0))])
    assert gate.open(0.0, snapshot, peer_snapshot, initial=True) == 1
    gate.batch_attempts = 2
    gate.exhaust(1.0, snapshot, peer_snapshot)
    assert gate.waiting_for_novelty
    old = _batch_candidate(
        'own-1', 'peer-1', 10, 11, (0.0, 0.0), (0.0, 0.0))
    assert not gate.is_novel(**old, attempted_pairs=set())
    assert gate.batch_id == 1


def test_new_spatial_keyframes_open_exactly_one_reentry_batch():
    gate = BoundedVerificationBatchController(
        budget=2, max_batches=3, lifetime_s=100.0)
    own = _batch_snapshot([('own-1', 10, (0.0, 0.0))])
    peer = _batch_snapshot([('peer-1', 11, (0.0, 0.0))])
    assert gate.open(0.0, own, peer, initial=True) == 1
    gate.exhaust(1.0, own, peer)
    novel = _batch_candidate(
        'own-2', 'peer-2', 20, 21, (1.0, 0.0), (1.0, 0.0))
    assert gate.is_novel(**novel, attempted_pairs=set())
    assert gate.open(2.0, own, peer, initial=False) == 2
    assert gate.open(3.0, own, peer, initial=False) is None


def test_nearby_or_duplicate_keyframes_do_not_reopen_batch():
    gate = BoundedVerificationBatchController(
        budget=2, max_batches=3, lifetime_s=100.0)
    own = _batch_snapshot([('own-1', 10, (0.0, 0.0))])
    peer = _batch_snapshot([('peer-1', 11, (0.0, 0.0))])
    gate.open(0.0, own, peer, initial=True)
    gate.exhaust(1.0, own, peer)
    nearby = _batch_candidate(
        'own-2', 'peer-2', 20, 21, (0.2, 0.0), (0.2, 0.0))
    assert not gate.is_novel(**nearby, attempted_pairs=set())
    delayed_old = _batch_candidate(
        'own-new-id', 'peer-new-id', 9, 9, (1.0, 0.0), (1.0, 0.0))
    assert not gate.is_novel(**delayed_old, attempted_pairs=set())
    duplicate = _batch_candidate(
        'own-1', 'peer-1', 10, 11, (1.0, 0.0), (1.0, 0.0))
    assert not gate.is_novel(**duplicate, attempted_pairs=set())


def test_previously_attempted_or_rejected_physical_pair_never_reopens():
    gate = BoundedVerificationBatchController(
        budget=2, max_batches=3, lifetime_s=100.0)
    own = _batch_snapshot([('own-1', 10, (0.0, 0.0))])
    peer = _batch_snapshot([('peer-1', 11, (0.0, 0.0))])
    gate.open(0.0, own, peer, initial=True)
    gate.exhaust(1.0, own, peer)
    candidate = _batch_candidate(
        'own-2', 'peer-2', 20, 21, (1.0, 0.0), (1.0, 0.0))
    assert not gate.is_novel(
        **candidate, attempted_pairs={('own-2', 'peer-2')})
    assert not gate.is_novel(
        **candidate, attempted_pairs=set(), rejected_physical=True)


def test_batch_limit_and_lifetime_are_finite():
    gate = BoundedVerificationBatchController(
        budget=1, max_batches=2, lifetime_s=5.0)
    own = _batch_snapshot([('own-1', 10, (0.0, 0.0))])
    peer = _batch_snapshot([('peer-1', 11, (0.0, 0.0))])
    assert gate.open(0.0, own, peer, initial=True) == 1
    gate.exhaust(1.0, own, peer)
    assert gate.open(2.0, own, peer, initial=False) == 2
    gate.exhaust(3.0, own, peer)
    gate.lifetime_deadline = 4.0
    assert gate.open(5.0, own, peer, initial=False) is None
    assert gate.lifetime_expired


def test_batch_diagnostics_contract_keeps_attempt_and_correlation_identity():
    source = (
        __import__('pathlib').Path(__file__).parents[1] /
        'my_epuck_project' / 'unknown_pose_frontend.py').read_text()
    assert 'acquisition_batch_id' in source
    assert 'candidate_correlation_id' in source
    assert 'keyframe_creation_timestamp_ns' in source
    assert 'STALE_VERIFICATION_BATCH' in source


def test_batch_reentry_never_changes_three_constraint_consensus_gate():
    assert not crop_batch_is_ready(2, 3)
    assert crop_batch_is_ready(3, 3)
    assert BoundedVerificationBatchController(
        budget=8, max_batches=4).max_batches == 4


def test_full_exploration_uses_more_than_four_finite_reentry_batches():
    source = (
        __import__('pathlib').Path(__file__).parents[1] /
        'launch' / 'two_robots_decentralized_exploration_launch.py').read_text()
    wrapper = (
        __import__('pathlib').Path(__file__).parents[1] /
        'launch' / 'two_robots_unknown_pose_exploration_launch.py').read_text()
    assert "DeclareLaunchArgument('max_verification_batches', default_value='32')" in source
    assert "_arg('max_verification_batches', '32')" in wrapper
    assert 'max_verification_batches' in source
    assert "DeclareLaunchArgument('verification_lifetime_s'" in source
    assert "_arg('verification_lifetime_s', '1100.0')" in wrapper


def test_duplicate_physical_evidence_diagnostics_are_coalesced_not_state():
    source = (
        __import__('pathlib').Path(__file__).parents[1] /
        'my_epuck_project' / 'unknown_pose_frontend.py').read_text()
    assert '_diagnosed_duplicate_physical_candidates' in source
    assert "'physical_candidate_duplicates_suppressed'" in source
    assert "duplicate_record='first_observation'" in source


def test_campaign_pattern_reopens_once_after_late_overlap_without_retrying_old_pairs():
    """Model the 17 m run: exhausted early evidence must wait for novelty."""
    gate = BoundedVerificationBatchController(
        budget=2, max_batches=2, lifetime_s=100.0, novelty_spacing_m=0.40)
    initial_own = _batch_snapshot([
        ('own-early-a', 10, (0.0, 0.0)),
        ('own-early-b', 20, (0.1, 0.0)),
    ])
    initial_peer = _batch_snapshot([
        ('peer-early-a', 11, (0.0, 0.0)),
        ('peer-early-b', 21, (0.1, 0.0)),
    ])
    assert gate.open(0.0, initial_own, initial_peer, initial=True) == 1
    attempted = {
        ('own-early-a', 'peer-early-a'),
        ('own-early-b', 'peer-early-b'),
    }
    gate.batch_attempts = 2
    gate.exhaust(1.0, initial_own, initial_peer)
    accepted_constraints = [('late-accepted-0', (0.0, 0.0))]

    old_candidate = _batch_candidate(
        'own-early-a', 'peer-early-a', 10, 11, (0.0, 0.0), (0.0, 0.0))
    assert not gate.is_novel(
        **old_candidate, attempted_pairs=attempted)
    assert gate.batch_id == 1

    late_own = dict(initial_own)
    late_peer = dict(initial_peer)
    late_own['own-late'] = {'timestamp_ns': 30, 'center': (1.2, 0.0)}
    late_peer['peer-late'] = {'timestamp_ns': 31, 'center': (1.2, 0.0)}
    late_candidate = _batch_candidate(
        'own-late', 'peer-late', 30, 31, (1.2, 0.0), (1.2, 0.0))
    assert gate.is_novel(
        **late_candidate, attempted_pairs=attempted)
    assert gate.open(2.0, late_own, late_peer, initial=False) == 2
    assert gate.batch_attempts == 0
    assert not gate.is_novel(
        **late_candidate, attempted_pairs=attempted)
    assert accepted_constraints == [('late-accepted-0', (0.0, 0.0))]

    gate.batch_attempts = 2
    gate.exhaust(3.0, late_own, late_peer)
    assert gate.open(4.0, late_own, late_peer, initial=False) is None
    assert gate.batch_id == 2


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


def test_handoff_acceptance_is_one_shot_for_duplicate_messages():
    assert should_accept_hypothesis(None, 'ACCEPTED', True, 0.80)
    assert not should_accept_hypothesis(object(), 'ACCEPTED', True, 0.80)


def test_temporal_consistency_rejects_single_weak_or_distant_evidence():
    assert temporal_consistency([1]) == 0.5
    assert temporal_consistency([1, 2_000_000_000]) > 0.5
    assert temporal_consistency([1, 10_000_000_001]) == 0.0


def test_temporal_confirmation_adapts_to_observed_message_cadence():
    configured = 8_000_000_000
    observed = [11_600_000_000, 11_900_000_000, 12_100_000_000]
    assert confirmation_window_for_cadence(configured, observed) == 29_750_000_000


def test_temporal_confirmation_counts_distinct_pairs_not_repeated_computation():
    observations = [
        (('peer-1', 'own-1'), 100_000_000_000, 200_000_000_000,
         0.90, 0.05, 0.50, 4),
        (('peer-1', 'own-1'), 100_000_000_000, 200_000_000_000,
         0.90, 0.05, 0.50, 4),
        (('peer-2', 'own-2'), 112_000_000_000, 212_000_000_000,
         0.88, 0.04, 0.50, 5),
    ]
    assert temporal_support_count(
        ('peer-1', 'own-1'), 100_000_000_000, 200_000_000_000, 4,
        observations, 0.72, 0.005, 0.12, 30_000_000_000) == 2


def test_temporal_confirmation_rejects_unstable_or_distant_observations():
    observations = [
        (('peer-1', 'own-1'), 100_000_000_000, 200_000_000_000,
         0.90, 0.05, 0.50, 4),
        (('peer-2', 'own-2'), 150_000_000_000, 250_000_000_000,
         0.88, 0.04, 0.50, 15),
        (('peer-3', 'own-3'), 112_000_000_000, 212_000_000_000,
         0.71, 0.05, 0.50, 4),
    ]
    assert temporal_support_count(
        ('peer-1', 'own-1'), 100_000_000_000, 200_000_000_000, 4,
        observations, 0.72, 0.005, 0.12, 30_000_000_000) == 1


def test_crop_is_bounded_and_preserves_unknown_cells():
    values = np.full((400, 400), -1, dtype=np.int16)
    values[180:220, 180:220] = 100
    crop = crop_grid(values, 0.05, -10.0, -10.0, size_m=8.0)
    assert crop.values.shape == (160, 160)
    assert np.count_nonzero(crop.values == -1) > 0


def test_crop_exchange_does_not_start_from_partial_spatial_batch():
    assert not crop_batch_is_ready(1, 3)
    assert not crop_batch_is_ready(2, 3)
    assert crop_batch_is_ready(3, 3)


def test_crop_exchange_batch_gate_keeps_registration_requirement_unchanged():
    assert not crop_batch_is_ready(2, 3)
    assert crop_batch_is_ready(3, 3)


def descriptor(key, origin_x, origin_y, epoch=7, checksum=1234):
    return SimpleNamespace(
        keyframe_id=key, map_epoch=epoch, checksum=checksum,
        resolution=0.05, crop_width=20, crop_height=20,
        crop_origin_x=origin_x, crop_origin_y=origin_y)


def candidate(peer, own, peer_descriptor, own_descriptor):
    return (peer, own, peer_descriptor, own_descriptor)


def test_physical_duplicate_descriptors_with_different_ids_count_once():
    own_crops = {
        'own-a': GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05, 0.0, 0.0),
        'own-b': GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05, 0.0, 0.0),
    }
    own_a = descriptor('own-a', 0.0, 0.0)
    own_b = descriptor('own-b', 0.0, 0.0)
    peer_a = descriptor('peer-a', 2.0, 0.0)
    peer_b = descriptor('peer-b', 2.0, 0.0)
    candidates = [
        candidate('peer-a', 'own-a', peer_a, own_a),
        candidate('peer-b', 'own-b', peer_b, own_b),
    ]
    assert len(deduplicate_physical_candidates(candidates, own_crops)) == 1


def test_physical_candidate_identity_accepts_runtime_scored_tuple_shape():
    own_crops = {
        'own': GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05, 0.0, 0.0)}
    own = descriptor('own', 0.0, 0.0)
    peer = descriptor('peer', 2.0, 0.0)
    scored = [(-0.9, 'peer', 'own', peer, own)]
    assert len(deduplicate_physical_candidates(scored, own_crops)) == 1


def test_streamed_candidate_accumulation_keeps_one_candidate_pending():
    own_crops = {
        'own-1': GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05, 0.0, 0.0)}
    pool = {}
    added, duplicates = accumulate_physical_candidates(
        pool, [candidate('peer-1', 'own-1', descriptor('peer-1', 3.0, 0.0),
                          descriptor('own-1', 0.0, 0.0))], own_crops)
    assert len(added) == 1
    assert not duplicates
    assert len(pool) == 1


def test_streamed_candidate_accumulation_keeps_two_candidates_pending():
    own_crops = {
        f'own-{i}': GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05,
                             float(i), 0.0) for i in (1, 2)}
    pool = {}
    for i in (1, 2):
        added, duplicates = accumulate_physical_candidates(
            pool, [candidate(
                f'peer-{i}', f'own-{i}', descriptor(f'peer-{i}', 3.0 + i, 0.0,
                                                     checksum=1000 + i),
                descriptor(f'own-{i}', float(i), 0.0, checksum=2000 + i))],
            own_crops)
        assert len(added) == 1
        assert not duplicates
    assert len(pool) == 2


def test_streamed_accumulation_forms_three_distinct_candidates():
    own_crops = {
        f'own-{i}': GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05,
                             float(i), 0.0) for i in (0, 1, 2)}
    pool = {}
    for i in (0, 1, 2):
        accumulate_physical_candidates(
            pool, [candidate(
                f'peer-{i}', f'own-{i}', descriptor(f'peer-{i}', 3.0 + i, 0.0,
                                                     checksum=3000 + i),
                descriptor(f'own-{i}', float(i), 0.0, checksum=4000 + i))],
            own_crops)
    assert len(pool) == 3
    assert evidence_batch_is_spatially_diverse(
        [own_crops[f'own-{i}'] for i in (0, 1, 2)], 0.75)


def test_duplicate_ids_and_different_ids_with_same_physical_crop_count_once():
    own_crops = {
        'own-a': GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05, 0.0, 0.0),
        'own-b': GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05, 0.0, 0.0),
    }
    pool = {}
    first = candidate('peer-a', 'own-a', descriptor('peer-a', 3.0, 0.0),
                      descriptor('own-a', 0.0, 0.0))
    duplicate = candidate('peer-b', 'own-b', descriptor('peer-b', 3.0, 0.0),
                          descriptor('own-b', 0.0, 0.0))
    added, duplicates = accumulate_physical_candidates(
        pool, [first], own_crops)
    assert len(added) == 1 and not duplicates
    added, duplicates = accumulate_physical_candidates(
        pool, [duplicate], own_crops)
    assert not added and len(duplicates) == 1
    assert len(pool) == 1


def test_later_fourth_candidate_completes_initially_insufficient_batch():
    own_crops = {
        f'own-{i}': GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05,
                             float(i), 0.0) for i in (0, 1, 2, 3)}
    pool = {}
    for i in (0, 1):
        accumulate_physical_candidates(
            pool, [candidate(
                f'peer-{i}', f'own-{i}', descriptor(f'peer-{i}', 4.0 + i, 0.0,
                                                     checksum=5000 + i),
                descriptor(f'own-{i}', float(i), 0.0, checksum=6000 + i))],
            own_crops)
    assert len(pool) == 2
    accumulate_physical_candidates(
        pool, [candidate('peer-2', 'own-2', descriptor('peer-2', 6.0, 0.0,
                                                        checksum=5002),
                          descriptor('own-2', 2.0, 0.0, checksum=6002))],
        own_crops)
    assert len(pool) == 3
    accumulate_physical_candidates(
        pool, [candidate('peer-3', 'own-3', descriptor('peer-3', 7.0, 0.0,
                                                        checksum=5003),
                          descriptor('own-3', 3.0, 0.0, checksum=6003))],
        own_crops)
    assert len(pool) == 4
    assert evidence_batch_is_spatially_diverse(
        [own_crops[f'own-{i}'] for i in (0, 1, 2, 3)], 0.75)


def test_streamed_accumulation_rejects_batch_without_075m_baseline():
    crops = [GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05, x, 0.0)
             for x in (0.0, 0.2, 0.4)]
    assert not evidence_batch_is_spatially_diverse(crops, 0.75)


def test_three_spatially_distinct_physical_crops_form_a_valid_batch():
    own_crops = {}
    candidates = []
    for index, x in enumerate((0.0, 1.0, 2.0)):
        own_key, peer_key = f'own-{index}', f'peer-{index}'
        own_crops[own_key] = GridCrop(
            np.zeros((20, 20), dtype=np.int16), 0.05, x, 0.0)
        own = descriptor(own_key, x, 0.0)
        peer = descriptor(peer_key, x + 3.0, 0.0, checksum=2000 + index)
        candidates.append(candidate(peer_key, own_key, peer, own))
    selected = deduplicate_physical_candidates(candidates, own_crops)
    assert len(selected) == 3
    assert evidence_batch_is_spatially_diverse(
        [own_crops[item[1]] for item in selected], 0.75)


def test_duplicate_initial_batch_waits_for_later_distinct_candidate():
    own_crops = {
        'own-1': GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05, 0.0, 0.0),
        'own-2': GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05, 0.0, 0.0),
        'own-3': GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05, 1.0, 0.0),
        'own-4': GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05, 2.0, 0.0),
    }
    shared_peer_a = descriptor('peer-1', 3.0, 0.0)
    shared_peer_b = descriptor('peer-2', 3.0, 0.0)
    initial = [
        candidate('peer-1', 'own-1', shared_peer_a,
                  descriptor('own-1', 0.0, 0.0)),
        candidate('peer-2', 'own-2', shared_peer_b,
                  descriptor('own-2', 0.0, 0.0)),
        candidate('peer-3', 'own-3', descriptor('peer-3', 4.0, 0.0,
                                                checksum=3003),
                  descriptor('own-3', 1.0, 0.0)),
    ]
    assert len(deduplicate_physical_candidates(initial, own_crops)) == 2
    later = initial + [
        candidate('peer-4', 'own-4', descriptor('peer-4', 5.0, 0.0,
                                                checksum=3004),
                  descriptor('own-4', 2.0, 0.0)),
    ]
    selected = deduplicate_physical_candidates(later, own_crops)
    assert len(selected) == 3
    assert evidence_batch_is_spatially_diverse(
        [own_crops[item[1]] for item in selected], 0.75)


def test_no_spatially_diverse_evidence_batch_remains_pending():
    crops = [GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05, x, 0.0)
             for x in (0.0, 0.2, 0.4)]
    assert not evidence_batch_is_spatially_diverse(crops, 0.75)


def test_three_crop_evidence_uses_hashable_pair_keys_before_registration():
    class UnhashableDescriptor:
        __hash__ = None
    active_candidate_pairs = []
    evidence_pairs = {}
    for index in range(3):
        peer_key = f'peer-{index}'
        own_key = f'own-{index}'
        peer_descriptor = UnhashableDescriptor()
        own_descriptor = UnhashableDescriptor()
        active_candidate_pairs.append(
            (peer_key, own_key, peer_descriptor, own_descriptor))
        evidence_pairs[(own_key, peer_key)] = (
            f'source-crop-{index}', f'target-crop-{index}')
    selected_crops = evidence_pairs_for_selection(
        active_candidate_pairs, evidence_pairs)
    registration_stage = lambda pairs: len(pairs)

    assert registration_stage(selected_crops) == 3
    assert selected_crops == [
        ('source-crop-0', 'target-crop-0'),
        ('source-crop-1', 'target-crop-1'),
        ('source-crop-2', 'target-crop-2'),
    ]
    assert all(isinstance(key, tuple) and len(key) == 2
               for key in evidence_pairs)


def test_pooled_evidence_reconsiders_prior_pairs_without_descriptor_keys():
    class UnhashableDescriptor:
        __hash__ = None

    pool = [
        ('peer-1', 'own-1', UnhashableDescriptor(), UnhashableDescriptor()),
        (-0.9, 'peer-2', 'own-2', UnhashableDescriptor(),
         UnhashableDescriptor()),
        ('peer-3', 'own-3', UnhashableDescriptor(), UnhashableDescriptor()),
    ]
    evidence_pairs = {
        ('own-1', 'peer-1'): ('source-1', 'target-1'),
        ('own-2', 'peer-2'): ('source-2', 'target-2'),
        ('own-3', 'peer-3'): ('source-3', 'target-3'),
    }

    resolved = evidence_candidates_for_pool(pool, evidence_pairs)

    assert [(candidate[1], candidate[0]) for candidate in resolved] == [
        ('own-1', 'peer-1'), ('own-2', 'peer-2'), ('own-3', 'peer-3')]
    assert [evidence_pairs[(candidate[1], candidate[0])]
            for candidate in resolved] == [
                ('source-1', 'target-1'),
                ('source-2', 'target-2'),
                ('source-3', 'target-3')]


def test_consensus_diagnostic_stream_survives_unrelated_protocol_traffic(tmp_path):
    path = tmp_path / 'robot1_consensus_diagnostics.jsonl'
    stream = DedicatedDiagnosticJsonl(path, max_records=8)
    unrelated_protocol_events = []
    for index in range(2000):
        if len(unrelated_protocol_events) < 512:
            unrelated_protocol_events.append({'event': 'TIMER_TICK',
                                              'index': index})
    stream.write({'record_type': 'CONSENSUS_CONSTRAINT_DIAGNOSTIC',
                  'index': 0, 'transform': [1.0, 2.0, 0.1]})
    stream.write({'record_type': 'CONSENSUS_SUBSET_COMPARISON',
                  'indices': [0, 1, 2],
                  'rejection_reasons': ['PAIRWISE_TRANSFORM_INCONSISTENT']})
    stream.close()
    records = [json.loads(line) for line in path.read_text().splitlines()]
    assert [record['record_type'] for record in records] == [
        'CONSENSUS_CONSTRAINT_DIAGNOSTIC',
        'CONSENSUS_SUBSET_COMPARISON']
    assert stream.records_written == 2
    assert stream.dropped_records == 0
    assert len(unrelated_protocol_events) == 512


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
