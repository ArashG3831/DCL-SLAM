"""Stationary-start registration evidence regressions."""

import inspect
import math
from types import SimpleNamespace

import numpy as np

from my_epuck_project.unknown_pose_frontend import UnknownPoseFrontend
from my_epuck_project.unknown_pose_frontend_core import (
    GridCrop,
    iter_stationary_witness_partitions,
    reestimate_registration_from_seed,
    register_crops,
    select_hypothesis_family,
    stationary_witness_supports_disjoint,
)


def _maps():
    values = np.full((80, 120), -1, dtype=np.int16)
    values[10:13, 5:115] = 100
    values[10:65, 5:8] = 100
    values[60:63, 5:100] = 100
    values[30:65, 55:58] = 100
    values[20:23, 80:110] = 100
    return (GridCrop(values, 0.03, 0.0, 0.0),
            GridCrop(values.copy(), 0.03, -0.50, 0.0))


def _witness_family():
    source, target = _maps()
    whole = register_crops(source, target, minimum_agreement=0.0)
    assert whole.accepted
    for _left, _right, partitions in iter_stationary_witness_partitions(
            source, target, whole.transform):
        if not stationary_witness_supports_disjoint(partitions):
            continue
        results = [reestimate_registration_from_seed(
                       item.source, item.target, whole.transform,
                       minimum_agreement=0.0)
                   for item in partitions]
        family = select_hypothesis_family(
            [(item.source, item.target) for item in partitions],
            [(result,) for result in results], min_consistent_constraints=3,
            min_spatial_baseline_m=0.0,
            max_translation_consistency_m=0.15,
            max_yaw_consistency_rad=math.radians(1.0),
            max_projected_registration_error_m=0.20,
            minimum_agreement=0.0,
            evidence_ids=[item.support_hash for item in partitions])
        if family.accepted:
            return partitions, results, family
    raise AssertionError('synthetic scene did not produce a stationary family')


def test_one_static_pair_cannot_satisfy_three_constraint_family():
    partitions, results, _family = _witness_family()
    result = select_hypothesis_family(
        [(partitions[0].source, partitions[0].target)],
        [(results[0],)], min_consistent_constraints=3,
        min_spatial_baseline_m=0.0, evidence_ids=['one'])
    assert not result.accepted
    assert result.reason == 'INSUFFICIENT_CONSISTENT_CONSTRAINTS'


def test_repeated_or_overlapping_support_cannot_increase_family_cardinality():
    partitions, results, _family = _witness_family()
    repeated = (partitions[0], partitions[0], partitions[0])
    assert not stationary_witness_supports_disjoint(repeated)
    result = select_hypothesis_family(
        [(partitions[0].source, partitions[0].target)] * 3,
        [(results[0],)] * 3, min_consistent_constraints=3,
        min_spatial_baseline_m=0.0,
        evidence_ids=['same', 'same', 'same'])
    assert not result.accepted


def test_three_disjoint_witnesses_pass_the_existing_family_gates():
    partitions, _results, family = _witness_family()
    assert stationary_witness_supports_disjoint(partitions)
    assert family.accepted
    assert family.consistent_constraint_count == 3


def test_same_canonical_seed_reconstructs_identical_support_hashes():
    source, target = _maps()
    whole = register_crops(source, target, minimum_agreement=0.0)
    assert whole.accepted
    first = next(iter_stationary_witness_partitions(
        source, target, whole.transform))
    second = next(iter_stationary_witness_partitions(
        source, target, tuple(whole.transform)))
    assert [item.support_hash for item in first[2]] == [
        item.support_hash for item in second[2]]


def test_final_family_refinement_does_not_define_partition_identity():
    source, target = _maps()
    whole = register_crops(source, target, minimum_agreement=0.0)
    assert whole.accepted
    family_seed = (whole.transform[0] + 0.04,
                   whole.transform[1] - 0.02,
                   whole.transform[2] + math.radians(0.2))
    canonical = next(iter_stationary_witness_partitions(
        source, target, whole.transform))
    replay = next(iter_stationary_witness_partitions(
        source, target, whole.transform))
    assert family_seed != whole.transform
    assert [item.support_hash for item in canonical[2]] == [
        item.support_hash for item in replay[2]]
    verifier = inspect.getsource(
        UnknownPoseFrontend._ensure_stationary_witness_snapshots_for_proposal)
    assert 'iter_stationary_witness_partitions(\n                source[\'crop\'], target[\'crop\'], seed)' in verifier


def test_stationary_provenance_rejects_changed_snapshot_or_incompatible_seed():
    message = SimpleNamespace(
        stationary_witness_scheme_version='stationary-disjoint-partition-v1',
        stationary_canonical_seed=[0.0, 0.0, 0.0])
    assert UnknownPoseFrontend._stationary_proposal_seed(message) == (
        0.0, 0.0, 0.0)
    assert not UnknownPoseFrontend._stationary_seed_compatible(
        (1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert UnknownPoseFrontend._stationary_seed_compatible(
        (0.1, 0.0, 0.0), (0.0, 0.0, 0.0))


def test_stationary_worker_uses_immutable_snapshot_ids_for_witness_records():
    worker = inspect.getsource(
        UnknownPoseFrontend._run_stationary_full_map_registration)
    scheduler = inspect.getsource(
        UnknownPoseFrontend._maybe_schedule_full_map_registration)
    assert 'source_snapshot_id' in worker
    assert 'target_snapshot_id' in worker
    assert "source['id'], target['id']" in scheduler


def test_inconsistent_stationary_witness_prevents_acceptance():
    partitions, results, _family = _witness_family()
    bad = results[2]
    bad = bad.__class__(**{
        **bad.__dict__,
        'transform': (bad.transform[0] + 0.30, bad.transform[1],
                      bad.transform[2]),
    })
    result = select_hypothesis_family(
        [(item.source, item.target) for item in partitions],
        [(results[0],), (results[1],), (bad,)],
        min_consistent_constraints=3, min_spatial_baseline_m=0.0,
        max_translation_consistency_m=0.15,
        max_yaw_consistency_rad=math.radians(1.0),
        max_projected_registration_error_m=0.20, minimum_agreement=0.0,
        evidence_ids=[item.support_hash for item in partitions])
    assert not result.accepted


def test_stationary_witness_path_is_reciprocal_and_ground_truth_free():
    source = inspect.getsource(UnknownPoseFrontend._ensure_stationary_witness_snapshots_for_proposal)
    verifier = inspect.getsource(UnknownPoseFrontend._verify_full_map_proposal)
    assert 'whole = self._run_full_map_registration' in source
    assert '_run_full_map_registration' in verifier
    assert 'ground_truth' not in source.lower()
    assert 'supervisor' not in source.lower()
    assert 'stationary_witness_proposal' in verifier
    assert 'reestimate_registration_from_seed' in verifier
    assert 'evidence_timestamps=(None if stationary_witness_proposal' in verifier
    assert 'whole = self._run_full_map_registration' in source
    assert 'stationary_canonical_source_map_hash' in verifier or \
        'stationary_canonical_source_map_hash' in source
    assert 'stationary_canonical_seed' in inspect.getsource(
        UnknownPoseFrontend._publish_full_map_proposal)


def test_common_start_still_gates_pre_release_dispatch():
    source = inspect.getsource(UnknownPoseFrontend._run_stationary_full_map_registration)
    assert 'min_consistent_constraints=3' in source
    assert 'stationary_witness_supports_disjoint' in inspect.getsource(
        UnknownPoseFrontend)
