"""Deterministic tests for descriptor-first unknown-pose discovery."""

import json
import math
from collections import Counter, deque
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from my_epuck_project import unknown_pose_frontend_core as frontend_core
from my_epuck_project.unknown_pose_frontend import UnknownPoseFrontend
from my_epuck_project.unknown_pose_frontend_core import (
    BoundedVerificationBatchController,
    candidate_reuses_accepted_physical_view,
    candidate_views_are_spatially_separated,
    DedicatedDiagnosticJsonl,
    GridCrop,
    accumulate_physical_candidates,
    compare_descriptors,
    crop_batch_is_ready,
    deduplicate_physical_candidates,
    crop_grid,
    descriptor_checksum,
    descriptor_match_is_ambiguous,
    DescriptorMatch,
    evidence_pairs_for_selection,
    evidence_candidates_for_pool,
    evidence_batch_is_spatially_diverse,
    hypothesis_is_acceptable,
    polar_descriptor,
    prioritize_unambiguous_candidates,
    physical_candidate_geometry_identity,
    register_crops,
    register_crop_set,
    consensus_admission_quality,
    consensus_crop_maturity,
    RegistrationResult,
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


def test_geometry_rejection_is_scoped_to_verification_batch():
    frontend = object.__new__(UnknownPoseFrontend)
    geometry_key = ('source-geometry', 'target-geometry')
    frontend.rejected_physical_geometry_keys = {geometry_key}
    frontend.rejected_physical_geometry_batches = {geometry_key: 3}
    frontend.verification_batches = SimpleNamespace(batch_id=3)
    assert frontend._geometry_rejected_in_active_batch(geometry_key)
    frontend.verification_batches.batch_id = 4
    assert not frontend._geometry_rejected_in_active_batch(geometry_key)


def test_consensus_maturity_requires_known_and_occupied_support():
    sparse = np.full((40, 40), -1, dtype=np.int16)
    sparse[:10, :10] = 0
    sparse[:10, :10][0, :100] = 100
    mature = np.full((40, 40), 0, dtype=np.int16)
    mature[:20, :20] = 100
    assert not consensus_crop_maturity(GridCrop(sparse, 0.03, 0.0, 0.0))[0]
    assert consensus_crop_maturity(GridCrop(mature, 0.03, 0.0, 0.0))[0]


def test_mature_consensus_admission_does_not_hard_gate_exact_agreement():
    result = RegistrationResult(
        accepted=True, transform=(1.0, 2.0, 0.1), covariance=(0.0,) * 36,
        inlier_ratio=0.60, residual_m=0.02,
        occupied_free_agreement=0.48, overlap_fraction=0.40,
        reason='ACCEPTED', reverse_inlier_ratio=0.55,
        condition_number=10.0)
    admitted, reason = consensus_admission_quality(result, True)
    assert admitted
    assert reason == 'INDIVIDUAL_GEOMETRY_ADMITTED_TO_CONSENSUS'


def test_geometry_attempted_in_active_batch_blocks_inflight_duplicate_only():
    frontend = object.__new__(UnknownPoseFrontend)
    geometry_key = ('source-geometry', 'target-geometry')
    frontend.attempted_physical_geometry_batches = {geometry_key: 3}
    frontend.verification_batches = SimpleNamespace(batch_id=3)
    assert frontend._geometry_attempted_in_active_batch(geometry_key)
    frontend.verification_batches.batch_id = 4
    assert not frontend._geometry_attempted_in_active_batch(geometry_key)


def test_ambiguous_descriptor_candidates_are_deferred_not_deleted():
    ambiguous = [('score-a', 'peer-a', 'own-a', object(), object())]
    clear = [('score-b', 'peer-b', 'own-b', object(), object())]
    selected = prioritize_unambiguous_candidates(
        ambiguous + clear, {('peer-a', 'own-a')})
    assert selected == clear
    assert prioritize_unambiguous_candidates(
        ambiguous, {('peer-a', 'own-a')}) == ambiguous


@pytest.mark.parametrize('robot_id', ('robot1', 'robot2'))
def test_identical_crop_content_requires_viewpoint_novelty(robot_id):
    frontend = object.__new__(UnknownPoseFrontend)
    frontend.verification_novelty_spacing_m = 0.40
    frontend.counters = Counter()
    frontend.keyframe_content_history = {}
    crop = GridCrop(np.zeros((8, 8), dtype=np.int16), 0.03, 1.0, 2.0)
    identity = frontend._crop_content_identity(crop)
    assert frontend._should_publish_keyframe(crop, (0.0, 0.0, 0.0))
    frontend.keyframe_content_history[identity] = (0.0, 0.0, 0.0)
    assert not frontend._should_publish_keyframe(crop, (0.10, 0.0, 0.0))
    assert frontend._should_publish_keyframe(crop, (0.40, 0.0, 0.0))
    assert frontend.counters['keyframe_motion_novelty_admitted'] == 1


def _evidence_admission_fixture(robot_id='robot1'):
    frontend = object.__new__(UnknownPoseFrontend)
    frontend.robot_id = robot_id
    frontend.evidence_keyframe_translation_threshold_m = 0.80
    frontend.keyframes = {}
    frontend.keyframe_content_history = {}
    frontend._last_evidence_viewpoint = None
    frontend.counters = Counter()
    return frontend


def test_first_evidence_keyframe_is_retained_without_motion_baseline():
    frontend = _evidence_admission_fixture()
    crop = GridCrop(np.zeros((8, 8), dtype=np.int16), 0.03, 1.0, 2.0)
    assert frontend._should_publish_keyframe(crop, None)


def test_rotation_without_translation_is_not_independent_evidence():
    frontend = _evidence_admission_fixture()
    first = GridCrop(np.zeros((8, 8), dtype=np.int16), 0.03, 1.0, 2.0)
    changed = GridCrop(np.ones((8, 8), dtype=np.int16), 0.03, 1.0, 2.0)
    frontend.keyframes['first'] = (object(), first)
    frontend.keyframe_content_history[frontend._crop_content_identity(first)] = (
        0.0, 0.0, 0.0)
    frontend._last_evidence_viewpoint = (0.0, 0.0, 0.0)
    assert not frontend._should_publish_keyframe(changed, (0.0, 0.0, 2.0))


@pytest.mark.parametrize('distance', (0.10, 0.79))
def test_subthreshold_translation_does_not_create_evidence_view(distance):
    frontend = _evidence_admission_fixture()
    first = GridCrop(np.zeros((8, 8), dtype=np.int16), 0.03, 1.0, 2.0)
    changed = GridCrop(np.ones((8, 8), dtype=np.int16), 0.03, 1.0, 2.0)
    frontend.keyframes['first'] = (object(), first)
    frontend.keyframe_content_history[frontend._crop_content_identity(first)] = (
        0.0, 0.0, 0.0)
    frontend._last_evidence_viewpoint = (0.0, 0.0, 0.0)
    assert not frontend._should_publish_keyframe(
        changed, (distance, 0.0, math.pi))


def test_translation_above_threshold_creates_new_evidence_view():
    frontend = _evidence_admission_fixture()
    first = GridCrop(np.zeros((8, 8), dtype=np.int16), 0.03, 1.0, 2.0)
    changed = GridCrop(np.ones((8, 8), dtype=np.int16), 0.03, 1.0, 2.0)
    frontend.keyframes['first'] = (object(), first)
    frontend.keyframe_content_history[frontend._crop_content_identity(first)] = (
        0.0, 0.0, 0.0)
    frontend._last_evidence_viewpoint = (0.0, 0.0, 0.0)
    assert frontend._should_publish_keyframe(changed, (0.81, 0.0, math.pi))


def test_pose_less_startup_does_not_poison_later_physical_baseline():
    frontend = _evidence_admission_fixture()
    first = GridCrop(np.zeros((8, 8), dtype=np.int16), 0.03, 1.0, 2.0)
    later = GridCrop(np.ones((8, 8), dtype=np.int16), 0.03, 1.0, 2.0)
    assert frontend._should_publish_keyframe(first, None)
    frontend.keyframes['startup'] = (object(), first)
    frontend.keyframe_content_history[
        frontend._crop_content_identity(first)] = None
    # The first valid local pose establishes the physical baseline even
    # though the immediately preceding startup crop had no TF pose.
    assert frontend._should_publish_keyframe(later, (0.0, 0.0, 0.0))
    frontend._last_evidence_viewpoint = (0.0, 0.0, 0.0)
    assert not frontend._should_publish_keyframe(
        GridCrop(np.full((8, 8), 2, dtype=np.int16), 0.03, 1.0, 2.0),
        (0.79, 0.0, 0.0))
    assert frontend._should_publish_keyframe(
        GridCrop(np.full((8, 8), 3, dtype=np.int16), 0.03, 1.0, 2.0),
        (0.80, 0.0, 0.0))


def test_duplicate_content_needs_translation_not_map_revision_or_new_id():
    frontend = _evidence_admission_fixture()
    crop = GridCrop(np.zeros((8, 8), dtype=np.int16), 0.03, 1.0, 2.0)
    identity = frontend._crop_content_identity(crop)
    frontend.keyframes['first'] = (object(), crop)
    frontend.keyframe_content_history[identity] = (0.0, 0.0, 0.0)
    frontend._last_evidence_viewpoint = (0.0, 0.0, 0.0)
    assert not frontend._should_publish_keyframe(crop, (0.80 - 1e-6, 0.0, 1.0))
    assert frontend._should_publish_keyframe(crop, (0.80, 0.0, 1.0))


def _canonical_pool_fixture():
    frontend = object.__new__(UnknownPoseFrontend)
    frontend.robot_id = 'robot1'
    frontend.peer_robot_id = 'robot2'
    frontend.canonical_constraint_pool = {}
    frontend.evidence_keyframe_translation_threshold_m = 0.80
    frontend.max_evidence_constraints = 5
    frontend.min_consistent_constraints = 3
    frontend.keyframes = {}
    frontend.keyframe_viewpoints = {}
    frontend.counters = Counter()
    frontend._record_diagnostic_event = lambda *args, **kwargs: None
    frontend._write_physical_evidence_diagnostic = (
        lambda *args, **kwargs: None)
    return frontend


def _test_descriptor(seconds):
    stamp = SimpleNamespace(sec=seconds, nanosec=0)
    return SimpleNamespace(header=SimpleNamespace(stamp=stamp))


def _test_constraint_result(transform=(-2.77, 0.0, 0.0)):
    return RegistrationResult(
        accepted=True, transform=transform, covariance=(0.0,) * 36,
        inlier_ratio=0.80, reverse_inlier_ratio=0.80,
        residual_m=0.02, occupied_free_agreement=0.40,
        overlap_fraction=0.80, reason='ACCEPTED', condition_number=1.0)


def test_canonical_union_combines_constraints_from_both_directions():
    frontend = _canonical_pool_fixture()
    r1_crops = [GridCrop(np.full((8, 8), value, dtype=np.int16), 0.03,
                         0.0, 0.0) for value in (1, 2, 3)]
    r2_crops = [GridCrop(np.full((8, 8), value + 10, dtype=np.int16), 0.03,
                         0.0, 0.0) for value in (1, 2, 3)]
    r1_desc = [_test_descriptor(index + 1) for index in range(3)]
    r2_desc = [_test_descriptor(index + 11) for index in range(3)]
    for index in (0, 1):
        key = f'robot1-{index + 1:08d}'
        frontend.keyframes[key] = (r1_desc[index], r1_crops[index])
        frontend.keyframe_viewpoints[key] = (float(index), 0.0)
        candidate = (f'robot2-{index + 1:08d}', key,
                     r2_desc[index], r1_desc[index])
        assert frontend._add_canonical_constraint(
            candidate, _test_constraint_result(), r1_crops[index],
            r2_crops[index], key, f'robot2-{index + 1:08d}')
    frontend.robot_id = 'robot2'
    key = 'robot2-00000003'
    frontend.keyframes[key] = (r2_desc[2], r2_crops[2])
    frontend.keyframe_viewpoints[key] = (2.0, 0.0)
    candidate = ('robot1-00000003', key, r1_desc[2], r2_desc[2])
    assert frontend._add_canonical_constraint(
        candidate, _test_constraint_result((2.77, 0.0, 0.0)), r2_crops[2],
        r1_crops[2], key, 'robot1-00000003')
    assert set(frontend.canonical_constraint_pool) == {
        'robot1:robot1-00000001|robot2:robot2-00000001',
        'robot1:robot1-00000002|robot2:robot2-00000002',
        'robot1:robot1-00000003|robot2:robot2-00000003'}
    assert all(record['result'].transform[0] < 0.0
               for record in frontend.canonical_constraint_pool.values())


def test_canonical_union_counts_reciprocal_physical_pair_once():
    frontend = _canonical_pool_fixture()
    source = GridCrop(np.ones((8, 8), dtype=np.int16), 0.03, 0.0, 0.0)
    target = GridCrop(np.full((8, 8), 2, dtype=np.int16), 0.03, 1.0, 0.0)
    r1 = _test_descriptor(1)
    r2 = _test_descriptor(2)
    frontend.keyframes['robot1-00000001'] = (r1, source)
    frontend.keyframe_viewpoints['robot1-00000001'] = (0.0, 0.0)
    candidate = ('robot2-00000001', 'robot1-00000001', r2, r1)
    assert frontend._add_canonical_constraint(
        candidate, _test_constraint_result(), source, target,
        'robot1-00000001', 'robot2-00000001')
    frontend.robot_id = 'robot2'
    frontend.keyframes['robot2-00000001'] = (r2, target)
    frontend.keyframe_viewpoints['robot2-00000001'] = (0.0, 0.0)
    reciprocal = ('robot1-00000001', 'robot2-00000001', r1, r2)
    assert not frontend._add_canonical_constraint(
        reciprocal, _test_constraint_result((2.77, 0.0, 0.0)), target,
        source, 'robot2-00000001', 'robot1-00000001')
    assert len(frontend.canonical_constraint_pool) == 1


@pytest.mark.parametrize('robot_id', ('robot1', 'robot2'))
def test_three_separated_evidence_viewpoints_remain_representable(robot_id):
    frontend = _evidence_admission_fixture(robot_id)
    for index, x in enumerate((0.0, 0.81, 1.62)):
        crop = GridCrop(np.full((8, 8), index, dtype=np.int16), 0.03, 1.0, 2.0)
        pose = (x, 0.0, 0.4 * index)
        assert frontend._should_publish_keyframe(crop, pose)
        frontend.keyframes[f'kf-{index}'] = (object(), crop)
        frontend.keyframe_content_history[
            frontend._crop_content_identity(crop)] = pose
        frontend._last_evidence_viewpoint = pose

def test_content_duplicate_evidence_rejects_reciprocal_pair():
    frontend = object.__new__(UnknownPoseFrontend)
    frontend.evidence_content_pairs = set()
    frontend.evidence_source_content = set()
    frontend.evidence_peer_content = set()
    source = GridCrop(np.zeros((8, 8), dtype=np.int16), 0.03, 0.0, 0.0)
    target = GridCrop(np.ones((8, 8), dtype=np.int16), 0.03, 1.0, 0.0)
    source_id = frontend._crop_content_identity(source)
    target_id = frontend._crop_content_identity(target)
    frontend.evidence_content_pairs.add((source_id, target_id))
    assert frontend._content_pair_reuses_evidence(target, source)


def test_late_registration_result_uses_request_batch_for_geometry_suppression():
    frontend = object.__new__(UnknownPoseFrontend)
    frontend.verification_batches = SimpleNamespace(batch_id=9)
    assert frontend._request_batch_id({'acquisition_batch_id': 3}) == 3
    assert frontend._request_batch_id({}) == 9


def test_verification_budget_waits_for_inflight_registration_work():
    frontend = object.__new__(UnknownPoseFrontend)
    frontend._registration_future = object()
    frontend._registration_pending_contexts = deque()
    assert frontend._verification_worker_busy()
    frontend._registration_future = None
    frontend._registration_pending_contexts.append(object())
    assert frontend._verification_worker_busy()
    frontend._registration_pending_contexts.clear()
    assert not frontend._verification_worker_busy()


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


def test_hypothesis_callback_persists_directional_protocol_diagnostics():
    source = (
        __import__('pathlib').Path(__file__).parents[1] /
        'my_epuck_project' / 'unknown_pose_frontend.py').read_text()
    assert "'HYPOTHESIS_RECEIVED'" in source
    assert "'HYPOTHESIS_IGNORED_SCOPE'" in source
    assert "'HYPOTHESIS_IGNORED_PENDING_TARGET'" in source
    assert "'HYPOTHESIS_PROPOSAL_ACCEPTED_FOR_CONFIRMATION'" in source
    assert "'HYPOTHESIS_ACK_IGNORED_NO_PENDING_PROPOSAL'" in source


def test_hypothesis_ack_uses_protocol_hash_not_registration_result_field():
    source = (
        __import__('pathlib').Path(__file__).parents[1] /
        'my_epuck_project' / 'unknown_pose_frontend.py').read_text()
    assert 'pending_proposal_evidence_hashes' in source
    assert 'proposal_evidence_set_hash' in source
    assert 'bool(proposal_evidence_set_hash)' in source


def test_accepted_alignment_is_published_as_persistent_static_tf():
    frontend = object.__new__(UnknownPoseFrontend)
    frontend.accepted = SimpleNamespace(
        source_to_target=SimpleNamespace(
            translation=SimpleNamespace(x=1.25, y=-0.5),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    )
    frontend.robot_id = 'robot1'
    frontend.peer_robot_id = 'robot2'
    frontend.shared_frame = 'shared_map'
    frontend.tf_broadcaster = SimpleNamespace(sent=[])
    frontend.tf_broadcaster.sendTransform = (
        lambda message: frontend.tf_broadcaster.sent.append(message))
    frontend.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: SimpleNamespace()))
    frontend.counters = {'tf_handoffs': 0}

    frontend.publish_accepted_tf()

    assert len(frontend.tf_broadcaster.sent) == 2
    assert frontend.tf_broadcaster.sent[0].header.frame_id == 'shared_map'
    assert frontend.tf_broadcaster.sent[0].child_frame_id == 'robot1/local_world'
    assert frontend.tf_broadcaster.sent[1].child_frame_id == 'robot2/local_world'
    # Registration's source_to_target message maps source points into the
    # target frame.  The persistent shared_map -> target/local_world TF is the
    # target pose in the source frame, so its translation is the inverse.
    assert frontend.tf_broadcaster.sent[1].transform.translation.x == pytest.approx(-1.25)
    assert frontend.tf_broadcaster.sent[1].transform.translation.y == pytest.approx(0.5)
    assert frontend.tf_broadcaster.sent[1].transform.rotation.z == pytest.approx(0.0)
    assert frontend.tf_broadcaster.sent[1].transform.rotation.w == pytest.approx(1.0)
    assert frontend.counters['tf_handoffs'] == 1
    source = (
        __import__('pathlib').Path(__file__).parents[1] /
        'my_epuck_project' / 'unknown_pose_frontend.py').read_text()
    assert 'StaticTransformBroadcaster' in source


def test_descriptor_comparisons_are_queued_and_bounded_per_timer_tick():
    source = (
        __import__('pathlib').Path(__file__).parents[1] /
        'my_epuck_project' / 'unknown_pose_frontend.py').read_text()
    assert 'pending_descriptor_pair_keys' in source
    assert 'pending_descriptor_pair_key_set' in source
    assert 'descriptor_pair_budget_per_tick = 16' in source
    assert 'len(uncomputed_pairs) < self.descriptor_pair_budget_per_tick' in source
    assert "if self.pending_descriptor_pair_keys:" in source


def test_busy_registration_responses_are_retained_in_bounded_fifo():
    """A busy worker must queue valid responses instead of losing evidence."""
    frontend = object.__new__(UnknownPoseFrontend)
    frontend._registration_pending_contexts = deque(maxlen=3)
    frontend._registration_pending_keys = set()
    frontend._registration_queue_drops = 0
    frontend._registration_queue_enqueues = 0
    frontend._registration_queue_max_depth = 0
    frontend._record_diagnostic_event = lambda *args, **kwargs: None
    frontend._write_physical_evidence_diagnostic = (
        lambda *args, **kwargs: None)

    for index in range(3):
        context = ((f'peer-{index}', index),) + (None,) * 11
        assert frontend._queue_registration_context(context)
    assert len(frontend._registration_pending_contexts) == 3
    assert frontend._registration_queue_enqueues == 3
    assert frontend._registration_queue_max_depth == 3
    assert frontend._registration_queue_drops == 0
    # A fourth item is explicitly bounded and diagnosed, never silently
    # replacing an earlier response.
    overflow = (('peer-overflow', 99),) + (None,) * 11
    assert not frontend._queue_registration_context(overflow)
    assert frontend._registration_queue_drops == 1
    assert frontend._registration_pending_contexts[0][0] == ('peer-0', 0)


def test_keyframe_identity_and_timestamp_are_not_map_revision_identity():
    frontend = object.__new__(UnknownPoseFrontend)
    frontend.robot_id = 'robot1'
    frontend.keyframe_sequence = 0
    first = frontend._allocate_keyframe_id()
    second = frontend._allocate_keyframe_id()
    assert first == 'robot1-00000001'
    assert second == 'robot1-00000002'
    assert first != second
    source = (
        __import__('pathlib').Path(__file__).parents[1] /
        'my_epuck_project' / 'unknown_pose_frontend.py').read_text()
    assert 'message.header.stamp = self.get_clock().now().to_msg()' in source
    assert 'message.map_epoch = self.map_revision' in source


def test_rejected_proposal_releases_target_confirmation_latch():
    frontend = object.__new__(UnknownPoseFrontend)
    frontend.robot_id = 'robot2'
    frontend.peer_robot_id = 'robot1'
    frontend.accepted = None
    frontend.pending_target_proposal = False
    frontend.pending_target_proposal = True
    frontend.negotiation_started = True
    frontend.peer_proposals = {'robot1-00000055': object()}
    frontend._record_diagnostic_event = lambda *args, **kwargs: None
    message = SimpleNamespace(
        source_robot_id='robot1', target_robot_id='robot2',
        source_keyframe_id='robot1-00000055',
        target_keyframe_id='robot2-00000035', status='REJECTED',
        accepted=False, final_confidence=0.0,
        rejection_reason='INSUFFICIENT_CONSISTENT_CONSTRAINTS')

    frontend.hypothesis_callback(message)

    assert frontend.pending_target_proposal is False
    assert frontend.peer_proposals == {}
    assert frontend.negotiation_started is True


def test_rejected_proposal_reopens_initiator_for_novel_evidence():
    frontend = object.__new__(UnknownPoseFrontend)
    frontend.robot_id = 'robot1'
    frontend.peer_robot_id = 'robot2'
    frontend.accepted = None
    frontend.pending_target_proposal = False
    frontend.batch_proposal_published = True
    frontend.negotiation_started = True
    frontend.pending_proposals = {('robot1-00000055', 'robot2-00000035'):
                                  object()}
    frontend.verification_batches = BoundedVerificationBatchController(
        budget=2, max_batches=3, lifetime_s=100.0)
    frontend.verification_batches.mark_completed()
    frontend._record_diagnostic_event = lambda *args, **kwargs: None
    message = SimpleNamespace(
        source_robot_id='robot1', target_robot_id='robot2',
        source_keyframe_id='robot1-00000055',
        target_keyframe_id='robot2-00000035', status='REJECTED',
        accepted=False, final_confidence=0.0,
        rejection_reason='INSUFFICIENT_CONSISTENT_CONSTRAINTS')

    frontend.hypothesis_callback(message)

    assert frontend.batch_proposal_published is False
    assert frontend.negotiation_started is False
    assert frontend.pending_proposals == {}
    assert frontend.verification_batches.completed is False
    assert frontend.verification_batches.waiting_for_novelty is True


def test_consensus_proposal_pool_retains_evicted_evidence_metadata():
    frontend = object.__new__(UnknownPoseFrontend)
    frontend.evidence_pairs = {('own-1', 'peer-1'): ('own-crop', 'peer-crop')}
    retained = (
        'peer-1', 'own-1',
        SimpleNamespace(map_epoch=4, checksum=11),
        SimpleNamespace(map_epoch=7, checksum=22))
    frontend.evidence_candidates = {('own-1', 'peer-1'): retained}
    frontend.keyframes = {}
    frontend.peer_descriptors = {}

    pool = frontend._proposal_candidate_pool(result=object())

    assert pool == [retained]


def test_incremental_consensus_uses_retained_descriptor_after_cache_eviction():
    """Evidence metadata must outlive the bounded live descriptor cache."""
    source = (
        __import__('pathlib').Path(__file__).parents[1] /
        'my_epuck_project' / 'unknown_pose_frontend.py').read_text()
    assert 'peer_descriptor = None if candidate is None else candidate[2]' in source
    assert 'peer_descriptor = selected_pairs[0][2]' in source
    assert 'self._stamp_ns(self.peer_descriptors[peer_key])' not in source


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
    assert "DeclareLaunchArgument('max_verification_batches', default_value='64')" in source
    assert "_arg('max_verification_batches', '64')" in wrapper
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


def test_descriptor_callbacks_defer_heavy_comparison_to_bounded_timer_work():
    source = (
        __import__('pathlib').Path(__file__).parents[1] /
        'my_epuck_project' / 'unknown_pose_frontend.py').read_text()
    assert 'pending_peer_descriptor_keys' in source
    assert 'pending_own_descriptor_keys' in source
    assert 'at most one descriptor key per timer tick' in source
    assert 'self._compare_peer_descriptors()' not in source


def test_expired_descriptor_pair_caches_are_pruned_with_bounded_history():
    """Historical Cartesian matches must not grow beyond live keyframes."""
    frontend = object.__new__(UnknownPoseFrontend)
    frontend.keyframes = {'own-live': object()}
    frontend.peer_descriptors = {'peer-live': object()}
    live = ('peer-live', 'own-live')
    expired = ('peer-expired', 'own-expired')
    frontend.matches = {live: 'live', expired: 'expired'}
    frontend.compared_pairs = {live, expired}
    frontend.descriptor_gate_status = {live: None, expired: 'old'}
    frontend.temporal_support_cache = {live: 2, expired: 1}
    frontend.temporal_gate_rejected_pairs = {live, expired}
    frontend.confirmations = {live: {live}, expired: {expired}}
    frontend.descriptor_ambiguous_pairs = {live, expired}
    frontend.descriptor_gate_survivors = {live, expired}
    frontend.temporal_gate_survivors = {live, expired}

    frontend._prune_expired_descriptor_state()

    assert set(frontend.matches) == {live}
    assert frontend.compared_pairs == {live}
    assert set(frontend.descriptor_gate_status) == {live}
    assert set(frontend.temporal_support_cache) == {live}
    assert frontend.temporal_gate_rejected_pairs == {live}
    assert set(frontend.confirmations) == {live}
    assert frontend.descriptor_gate_survivors == {live}
    assert frontend.temporal_gate_survivors == {live}


def test_temporal_support_cache_adds_only_new_valid_observations():
    """A new pair updates cached anchors without replaying old pairs."""
    def stamped(seconds):
        return SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=seconds, nanosec=0)))

    frontend = object.__new__(UnknownPoseFrontend)
    frontend.keyframes = {
        'own-old': (stamped(0), None),
        'own-new': (stamped(2), None),
    }
    frontend.peer_descriptors = {
        'peer-old': stamped(0),
        'peer-new': stamped(2),
    }
    old = ('peer-old', 'own-old')
    new = ('peer-new', 'own-new')
    match = SimpleNamespace(
        similarity=0.9, margin=0.1, known_fraction=0.5, sector_shift=0)
    frontend.matches = {old: match, new: match}
    frontend.descriptor_gate_status = {old: None, new: None}
    frontend.temporal_support_cache = {old: 1}
    frontend.similarity_gate = 0.72
    frontend.margin_gate = 0.005
    frontend.effective_confirmation_window_ns = 8_000_000_000

    frontend._update_temporal_support_cache({new})

    assert frontend.temporal_support_cache[old] == 2
    assert frontend.temporal_support_cache[new] == 2


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


def test_incremental_consensus_reuses_completed_individual_registrations(
        monkeypatch):
    first = GridCrop(scene(), 0.05, 0.0, 0.0)
    second = GridCrop(transform_grid(scene(), 0.10, 5, 3), 0.05, 0.0, 0.0)
    cached = frontend_core.register_crops(first, second)

    def unexpected_registration(*args, **kwargs):
        raise AssertionError('incremental consensus recomputed a crop')

    monkeypatch.setattr(frontend_core, 'register_crops',
                        unexpected_registration)
    result = register_crop_set(
        [(first, second), (first, second), (first, second)],
        min_consistent_constraints=3,
        min_spatial_baseline_m=0.0,
        individual_results=[cached, cached, cached])
    assert result.constraint_count == 3
    assert result.consistent_constraint_count == 3


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


def descriptor(key, origin_x, origin_y, epoch=7, checksum=1234,
               known_fraction=None, occupied_cells=None):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=0, nanosec=0), frame_id='map'),
        keyframe_id=key, map_epoch=epoch, checksum=checksum,
        resolution=0.05, crop_width=20, crop_height=20,
        crop_origin_x=origin_x, crop_origin_y=origin_y,
        crop_known_fraction=known_fraction,
        crop_occupied_cells=occupied_cells)


def candidate(peer, own, peer_descriptor, own_descriptor):
    return (peer, own, peer_descriptor, own_descriptor)


def test_low_margin_repetitive_alias_is_ambiguous_but_isolated_match_survives():
    weak = DescriptorMatch(0.90, 0.006, 12, 0.50)
    near_twin = DescriptorMatch(0.895, 0.007, 13, 0.50)
    assert descriptor_match_is_ambiguous(weak, [near_twin], 0.005)
    assert not descriptor_match_is_ambiguous(weak, [], 0.005)


@pytest.mark.parametrize(('robot_id', 'peer_id'), [
    ('robot1', 'robot2'), ('robot2', 'robot1')])
def test_rejected_view_does_not_block_later_view_symmetrically(
        robot_id, peer_id):
    frontend = object.__new__(UnknownPoseFrontend)
    frontend.evidence_pairs = {('own-anchor', 'peer-anchor'): (None, None)}
    frontend.verification_novelty_spacing_m = 0.40
    frontend.candidate_verification_attempted = {('own-old', 'peer-old')}
    frontend.request_candidate_by_request_key = {}
    frontend.keyframes = {
        'own-anchor': (descriptor('own-anchor', 0.0, 0.0),
                       GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05,
                                0.0, 0.0)),
        'own-old': (descriptor('own-old', 0.6, 0.0),
                    GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05,
                             0.6, 0.0)),
        'own-near-old': (descriptor('own-near-old', 0.8, 0.0),
                     GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05,
                              0.8, 0.0)),
        'own-far': (descriptor('own-far', 1.0, 0.0),
                    GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05,
                             1.0, 0.0)),
    }
    frontend.peer_descriptors = {
        'peer-anchor': descriptor('peer-anchor', 2.0, 0.0),
        'peer-old': descriptor('peer-old', 2.6, 0.0),
        'peer-near-old': descriptor('peer-near-old', 2.8, 0.0),
        'peer-far': descriptor('peer-far', 3.0, 0.0),
    }
    frontend._descriptor_geometry = lambda value: {
        'center': [float(value.crop_origin_x),
                   float(value.crop_origin_y)]}
    near_old = candidate('peer-near-old', 'own-near-old',
                         frontend.peer_descriptors['peer-near-old'],
                         frontend.keyframes['own-near-old'][0])
    far = candidate('peer-far', 'own-far',
                    frontend.peer_descriptors['peer-far'],
                    frontend.keyframes['own-far'][0])
    assert frontend._candidate_is_distinct_from_evidence(near_old)
    assert frontend._candidate_is_distinct_from_evidence(far)




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


def test_accepted_evidence_cannot_reuse_one_physical_view():
    own_crops = {
        'own-a': GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05, 0.0, 0.0),
        'own-b': GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05, 1.0, 0.0),
        'own-c': GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05, 2.0, 0.0),
    }
    first = candidate(
        'peer-a', 'own-a', descriptor('peer-a', 3.0, 0.0),
        descriptor('own-a', 0.0, 0.0))
    same_peer = candidate(
        'peer-b', 'own-b', descriptor('peer-b', 3.0, 0.0, checksum=2),
        descriptor('own-b', 1.0, 0.0, checksum=2))
    new_view = candidate(
        'peer-c', 'own-c', descriptor('peer-c', 5.0, 0.0, checksum=3),
        descriptor('own-c', 2.0, 0.0, checksum=3))
    accepted = {physical_candidate_geometry_identity(first, own_crops)}
    assert candidate_reuses_accepted_physical_view(
        same_peer, accepted, own_crops)
    same_own = candidate(
        'peer-c', 'own-a', descriptor('peer-c', 5.0, 0.0, checksum=3),
        descriptor('own-a', 0.0, 0.0, checksum=3))
    assert candidate_reuses_accepted_physical_view(
        same_own, accepted, own_crops)
    assert not candidate_reuses_accepted_physical_view(
        new_view, accepted, own_crops)


def test_pending_selection_requires_displacement_on_both_sides():
    own_prior = [np.asarray([0.0, 0.0])]
    peer_prior = [np.asarray([3.0, 0.0])]
    assert not candidate_views_are_spatially_separated(
        [1.0, 0.0], [3.1, 0.0], own_prior, peer_prior)
    assert not candidate_views_are_spatially_separated(
        [0.1, 0.0], [4.0, 0.0], own_prior, peer_prior)
    assert candidate_views_are_spatially_separated(
        [1.0, 0.0], [4.0, 0.0], own_prior, peer_prior)


def _maturity_filter_fixture(own_maturity, peer_maturity):
    frontend = object.__new__(UnknownPoseFrontend)
    frontend.consensus_min_known_fraction = 0.25
    frontend.consensus_min_occupied_cells = 400
    frontend.counters = Counter()
    frontend.matches = {}
    own_values = np.zeros((40, 40), dtype=np.int16)
    if own_maturity:
        own_values[:25, :20] = 100
    else:
        own_values[:10, :10] = 100
        own_values[10:, :] = -1
    frontend.keyframes = {
        'own': (descriptor('own', 0.0, 0.0),
                GridCrop(own_values, 0.05, 0.0, 0.0))}
    frontend._write_physical_evidence_diagnostic = lambda *args, **kwargs: None
    peer = descriptor(
        'peer', 2.0, 0.0,
        known_fraction=(0.50 if peer_maturity else 0.10),
        occupied_cells=(800 if peer_maturity else 100))
    return frontend, candidate('peer', 'own', peer,
                               frontend.keyframes['own'][0])


def test_immature_endpoint_is_not_schedulable_before_crop_request():
    frontend, item = _maturity_filter_fixture(False, True)
    assert frontend._candidate_intrinsically_immature(item)
    assert frontend.counters['immature_candidates_not_scheduled'] == 1


def test_mature_endpoint_pair_remains_schedulable():
    frontend, item = _maturity_filter_fixture(True, True)
    assert not frontend._candidate_intrinsically_immature(item)
    assert frontend.counters['immature_candidates_not_scheduled'] == 0


def test_early_descriptor_metadata_does_not_delete_or_blacklist_keyframe():
    frontend, item = _maturity_filter_fixture(True, False)
    assert frontend._candidate_intrinsically_immature(item)
    assert 'own' in frontend.keyframes
    assert frontend.keyframes['own'][0].keyframe_id == 'own'


def test_later_mature_descriptor_update_can_be_scheduled():
    frontend, item = _maturity_filter_fixture(True, False)
    assert frontend._candidate_intrinsically_immature(item)
    item[2].crop_known_fraction = 0.50
    item[2].crop_occupied_cells = 800
    assert not frontend._candidate_intrinsically_immature(item)


def test_scheduler_checks_maturity_before_request_budget():
    source = Path(UnknownPoseFrontend.__module__.replace('.', '/') + '.py')
    text = source.read_text() if source.exists() else Path(
        'src/my_epuck_project/my_epuck_project/unknown_pose_frontend.py').read_text()
    assert 'if self._candidate_intrinsically_immature(candidate):' in text
    assert "stage='PRE_CROP_REQUEST'" in text


def test_candidate_order_prefers_unattempted_physical_views_over_reused_family():
    """Rejected pair variants must not starve a genuinely new view."""
    frontend = object.__new__(UnknownPoseFrontend)
    frontend.evidence_pairs = {}
    frontend.keyframes = {
        'own-reused': (descriptor('own-reused', 0.0, 0.0),
                       GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05,
                                0.0, 0.0)),
        'own-new': (descriptor('own-new', 2.0, 0.0, epoch=8, checksum=8),
                    GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05,
                             2.0, 0.0)),
    }
    frontend.peer_descriptors = {
        'peer-reused': descriptor('peer-reused', 3.0, 0.0),
        'peer-new': descriptor('peer-new', 5.0, 0.0, epoch=8, checksum=8),
    }
    frontend._crop_geometry = lambda crop, *_args: {
        'center': [crop.origin_x + 0.5 * crop.values.shape[1] * crop.resolution,
                   crop.origin_y + 0.5 * crop.values.shape[0] * crop.resolution]}
    frontend._descriptor_geometry = lambda value: {
        'center': [float(value.crop_origin_x +
                         0.5 * value.crop_width * value.resolution),
                   float(value.crop_origin_y +
                         0.5 * value.crop_height * value.resolution)]}
    frontend.attempted_physical_view_reuse_counts = {}
    reused = candidate(
        'peer-reused', 'own-reused', frontend.peer_descriptors['peer-reused'],
        frontend.keyframes['own-reused'][0])
    fresh = candidate(
        'peer-new', 'own-new', frontend.peer_descriptors['peer-new'],
        frontend.keyframes['own-new'][0])
    reused_geometry = frontend._candidate_physical_geometry_key(reused)
    frontend.attempted_physical_view_reuse_counts[reused_geometry[0]] = 2
    frontend.attempted_physical_view_reuse_counts[reused_geometry[1]] = 2

    assert frontend._candidate_spatial_novelty_key(fresh)[0] == 0
    assert frontend._candidate_spatial_novelty_key(reused)[0] == 4
    assert frontend._candidate_spatial_novelty_key(fresh) < \
        frontend._candidate_spatial_novelty_key(reused)


@pytest.mark.parametrize('robot_id', ('robot1', 'robot2'))
def test_reentry_without_requestable_candidate_does_not_hold_navigation(robot_id):
    """A novelty advisory cannot create a lease when no crop can be sent."""
    class Batch:
        batch_id = 1
        waiting_for_novelty = True
        lifetime_expired = False

        def open(self, *_args, **_kwargs):
            self.batch_id += 1
            return self.batch_id

        def exhaust(self, *_args, **_kwargs):
            self.waiting_for_novelty = True

    node = object.__new__(UnknownPoseFrontend)
    node.robot_id = robot_id
    node.evidence_acquisition_started = False
    node.batch_proposal_published = False
    node.pending_candidate_pairs = {'novel-but-unrequestable': object()}
    node.verification_batches = Batch()
    node.evidence_physical_keys = {}
    node.min_consistent_constraints = 3
    node.counters = Counter()
    node.evidence_acquisition_window_s = 8.0
    node.evidence_acquisition_deadline_wall = None
    node._evidence_opportunity_deadline_wall = None
    statuses = []
    ended = []
    node._batch_snapshot = lambda: ({}, {})
    node._candidate_is_novel_for_reentry = lambda _candidate: True
    node._request_next_candidate_verification = lambda: False
    node._verification_worker_busy = lambda: False
    node._publish_evidence_status = lambda active: statuses.append(active)
    node._record_diagnostic_event = lambda *_args, **_kwargs: None
    node._write_physical_evidence_diagnostic = lambda *_args, **_kwargs: None

    original_end = UnknownPoseFrontend._end_evidence_acquisition
    node._end_evidence_acquisition = lambda reason: (
        ended.append(reason), original_end(node, reason))[1]

    UnknownPoseFrontend._begin_evidence_acquisition(node)

    assert ended == ['NO_REQUESTABLE_NOVEL_EVIDENCE']
    assert node.evidence_acquisition_started is False
    assert statuses == [False]


@pytest.mark.parametrize('robot_id', ('robot1', 'robot2'))
def test_rejected_candidate_without_next_request_releases_evidence_lease(robot_id):
    """A rejected pair cannot keep an empty verification lease open."""
    node = object.__new__(UnknownPoseFrontend)
    node.robot_id = robot_id
    node.batch_proposal_published = False
    node.evidence_acquisition_started = True
    node.pending_requests = set()
    node._registration_pending_contexts = deque()
    node._registration_backpressure_depth = 8
    node._registration_future = None
    node._registration_shutdown = False
    node.candidate_verification_batch_attempts = 1
    node.candidate_verification_attempts = 1
    node.candidate_verification_budget = 8
    node.evidence_pairs = {}
    node._rank_next_verification_candidate = lambda: None
    node._write_physical_evidence_diagnostic = lambda *_args, **_kwargs: None
    node._verification_worker_busy = lambda: False
    ended = []
    node._end_evidence_acquisition = lambda reason: (
        ended.append(reason), setattr(node, 'evidence_acquisition_started', False))[1]

    assert UnknownPoseFrontend._request_next_candidate_verification(node) is False
    assert ended == ['NO_REQUESTABLE_NOVEL_EVIDENCE']
    assert node.evidence_acquisition_started is False


def test_non_actionable_descriptor_survivor_does_not_renew_navigation_lease():
    """Temporal-ineligible descriptor survivors cannot hold both robots."""
    # Resolve from the checked-out test tree rather than an installed copy.
    source = Path(__file__).parents[1] / 'my_epuck_project' / (
        'unknown_pose_frontend.py')
    text = source.read_text(encoding='utf-8')
    branch = text.split('if not eligible:', 1)[1].split(
        'eligible.sort', 1)[0]
    assert 'EVIDENCE_OPPORTUNITY_NOT_ACTIONABLE' in branch
    assert '_evidence_opportunity_deadline_wall' not in branch
    assert '_publish_evidence_status(True)' not in branch


def test_next_verification_rejects_local_view_near_any_accepted_source():
    """A peer-keyframe change cannot increase the source spatial baseline."""
    frontend = object.__new__(UnknownPoseFrontend)
    frontend.evidence_pairs = {
        ('own-a', 'peer-a'): (None, None),
        ('own-b', 'peer-b'): (None, None),
    }
    frontend.keyframes = {
        'own-a': (descriptor('own-a', 0.0, 0.0),
                  GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05, 0.0, 0.0)),
        'own-b': (descriptor('own-b', 1.0, 0.0),
                  GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05, 1.0, 0.0)),
        'own-near-a': (descriptor('own-near-a', 0.1, 0.0),
                       GridCrop(np.zeros((20, 20), dtype=np.int16), 0.05,
                                0.1, 0.0)),
    }
    frontend.peer_descriptors = {
        'peer-a': descriptor('peer-a', 0.0, 0.0),
        'peer-b': descriptor('peer-b', 1.0, 0.0),
        'peer-new': descriptor('peer-new', 2.0, 0.0),
    }
    frontend._descriptor_geometry = lambda value: {
        'center': [float(value.crop_origin_x),
                   float(value.crop_origin_y)]}
    candidate = ('peer-new', 'own-near-a',
                 frontend.peer_descriptors['peer-new'],
                 frontend.keyframes['own-near-a'][0])
    assert not frontend._candidate_is_distinct_from_evidence(candidate)


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


def test_completed_consensus_rebuilds_proposal_pool_from_evidence_pairs():
    """Verified pairs remain publishable after leaving the pending pool."""
    frontend = object.__new__(UnknownPoseFrontend)
    frontend.evidence_pairs = {
        ('own-1', 'peer-1'): ('own-crop-1', 'peer-crop-1'),
        ('own-2', 'peer-2'): ('own-crop-2', 'peer-crop-2'),
        ('own-3', 'peer-3'): ('own-crop-3', 'peer-crop-3'),
    }
    frontend.pending_candidate_pairs = {}
    frontend.active_candidate_pairs = []
    frontend.keyframes = {
        f'own-{i}': (SimpleNamespace(), f'own-crop-{i}')
        for i in (1, 2, 3)}
    frontend.peer_descriptors = {
        f'peer-{i}': SimpleNamespace()
        for i in (1, 2, 3)}

    pool = frontend._proposal_candidate_pool(
        SimpleNamespace(accepted=True))

    assert [(item[1], item[0]) for item in pool] == [
        ('own-1', 'peer-1'), ('own-2', 'peer-2'), ('own-3', 'peer-3')]
    assert not frontend.pending_candidate_pairs


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


def test_evidence_hash_is_direction_independent():
    forward = UnknownPoseFrontend._canonical_evidence_hash(
        'robot1', 'robot2', ['r1-k0', 'r1-k1'], ['r2-k0', 'r2-k1'])
    reverse = UnknownPoseFrontend._canonical_evidence_hash(
        'robot2', 'robot1', ['r2-k0', 'r2-k1'], ['r1-k0', 'r1-k1'])
    assert forward == reverse


def test_physical_diagnostic_writer_has_byte_bound(tmp_path):
    stream = DedicatedDiagnosticJsonl(
        tmp_path / 'bounded.jsonl', max_records=1000, max_bytes=1024)
    for index in range(100):
        stream.write({'record_type': 'REPETITIVE', 'payload': 'x' * 100,
                       'index': index})
    stream.close()
    assert stream.bytes_written <= 1024
    assert stream.dropped_records > 0


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
