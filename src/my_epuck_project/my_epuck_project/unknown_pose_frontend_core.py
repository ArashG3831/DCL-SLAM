"""Bounded, map-native front-end primitives for unknown relative pose discovery.

The runtime node deliberately keeps this module free of ROS entities.  That
makes descriptor rejection and registration deterministic and allows the
same code to be exercised without Webots or a ground-truth transform.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from itertools import combinations
import json
import math
from dataclasses import replace
from pathlib import Path
from typing import Iterable

import numpy as np

from .robust_relative_pose_selector import (
    ACCEPTED_HYPOTHESIS,
    IncrementalHypothesisAccumulator,
    PoseConstraint,
    select_robust_hypothesis,
)

try:
    import cv2
except ImportError:  # pragma: no cover - exercised only on minimal systems.
    cv2 = None

try:
    from scipy.spatial import cKDTree
except ImportError:  # pragma: no cover - project runtime has scipy.
    cKDTree = None


OCCUPIED_THRESHOLD = 50
UNKNOWN_VALUE = -1
MINIMUM_ACCEPTED_CONFIDENCE = 0.65


class DedicatedDiagnosticJsonl:
    """Bounded streaming store independent from the protocol-event buffer."""

    def __init__(self, path: str | Path, max_records: int = 8192,
                 max_bytes: int = 32 * 1024 * 1024):
        self.path = Path(path)
        self.max_records = max(1, int(max_records))
        self.max_bytes = max(1024, int(max_bytes))
        self.bytes_written = 0
        self.records_written = 0
        self.dropped_records = 0
        self.write_failures = 0
        self._stream = None
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._stream = self.path.open('a', encoding='utf-8', buffering=1)
        except OSError:
            self.write_failures += 1

    def write(self, record: dict) -> None:
        if (self.records_written + self.dropped_records >= self.max_records or
                self.bytes_written >= self.max_bytes):
            self.dropped_records += 1
            return
        if self._stream is None:
            self.write_failures += 1
            return
        try:
            payload = json.dumps(record, sort_keys=True) + '\n'
            encoded_size = len(payload.encode('utf-8'))
            if self.bytes_written + encoded_size > self.max_bytes:
                self.dropped_records += 1
                return
            self._stream.write(payload)
            self._stream.flush()
            self.records_written += 1
            self.bytes_written += encoded_size
        except (OSError, TypeError, ValueError):
            self.write_failures += 1

    def close(self) -> None:
        if self._stream is not None:
            try:
                self._stream.flush()
                self._stream.close()
            except OSError:
                self.write_failures += 1
            finally:
                self._stream = None


class BoundedVerificationBatchController:
    """Novelty-gated lifecycle for bounded candidate-verification batches.

    This helper owns scheduling state only.  It never changes descriptor,
    registration, consensus, or handoff acceptance rules.  Pair identities
    and rejected physical identities remain owned by the frontend so a new
    batch cannot retry old evidence merely because its attempt counter reset.
    """

    def __init__(self, budget=8, max_batches=4, lifetime_s=600.0,
                 novelty_spacing_m=0.40):
        self.budget = max(1, int(budget))
        self.max_batches = max(1, int(max_batches))
        self.lifetime_s = max(1.0, float(lifetime_s))
        self.novelty_spacing_m = max(0.01, float(novelty_spacing_m))
        self.batch_id = 0
        self.batch_attempts = 0
        self.batch_opened_at = None
        self.lifetime_deadline = None
        self.active = False
        self.waiting_for_novelty = False
        self.completed = False
        self.lifetime_expired = False
        self.reference_own = {}
        self.reference_peer = {}

    @staticmethod
    def _copy_snapshot(snapshot):
        return {
            str(key): {
                'timestamp_ns': int(value['timestamp_ns']),
                'center': (float(value['center'][0]),
                           float(value['center'][1])),
            }
            for key, value in snapshot.items()
        }

    def open(self, now, own_snapshot, peer_snapshot, initial=False):
        """Open one batch, returning its ID, or ``None`` when bounded out."""
        if self.completed or self.active:
            return None
        now = float(now)
        if self.lifetime_deadline is not None and now > self.lifetime_deadline:
            self.lifetime_expired = True
            self.waiting_for_novelty = False
            return None
        if self.batch_id >= self.max_batches and not (initial and not self.batch_id):
            self.waiting_for_novelty = False
            return None
        self.batch_id += 1
        self.batch_attempts = 0
        self.batch_opened_at = now
        if self.lifetime_deadline is None:
            self.lifetime_deadline = now + self.lifetime_s
        self.active = True
        self.waiting_for_novelty = False
        self.reference_own = self._copy_snapshot(own_snapshot)
        self.reference_peer = self._copy_snapshot(peer_snapshot)
        return self.batch_id

    def exhaust(self, now, own_snapshot, peer_snapshot):
        """Close a batch and wait for novel evidence before reopening."""
        self.active = False
        self.waiting_for_novelty = True
        self.reference_own = self._copy_snapshot(own_snapshot)
        self.reference_peer = self._copy_snapshot(peer_snapshot)
        if self.lifetime_deadline is not None and float(now) >= self.lifetime_deadline:
            self.lifetime_expired = True
            self.waiting_for_novelty = False

    def mark_completed(self):
        self.active = False
        self.waiting_for_novelty = False
        self.completed = True

    def reopen_after_rejected_proposal(self):
        """Allow novel evidence after a peer rejects a proposal.

        A locally acceptable evidence batch is marked complete while the
        responder verifies it.  If that remote verification rejects the
        proposal, the batch was not a handoff and the initiator must be able
        to acquire a later, physically novel batch.  Reopening here does not
        retry old pairs: the frontend's attempted/rejected physical evidence
        sets still provide that safety boundary.
        """
        if self.lifetime_expired:
            return
        self.active = False
        self.completed = False
        self.waiting_for_novelty = True

    def is_novel(self, own_key, peer_key, own_timestamp_ns, peer_timestamp_ns,
                 own_center, peer_center, attempted_pairs=(),
                 rejected_physical=False):
        """Return true only for an unseen, displaced post-batch candidate."""
        if not self.waiting_for_novelty or self.completed or self.lifetime_expired:
            return False
        if (str(own_key), str(peer_key)) in {
                (str(pair[0]), str(pair[1])) for pair in attempted_pairs}:
            return False
        if rejected_physical:
            return False
        own_key = str(own_key)
        peer_key = str(peer_key)
        def created_after_snapshot(key, timestamp_ns, references):
            if key in references:
                return False
            if not references:
                return True
            latest_timestamp = max(
                value['timestamp_ns'] for value in references.values())
            return int(timestamp_ns) > int(latest_timestamp)

        own_new = created_after_snapshot(
            own_key, own_timestamp_ns, self.reference_own)
        peer_new = created_after_snapshot(
            peer_key, peer_timestamp_ns, self.reference_peer)
        if not own_new and not peer_new:
            return False
        def displaced(center, references):
            if not references:
                return True
            return any(
                math.hypot(float(center[0]) - value['center'][0],
                           float(center[1]) - value['center'][1]) >=
                self.novelty_spacing_m
                for value in references.values())
        return ((own_new and displaced(own_center, self.reference_own)) or
                (peer_new and displaced(peer_center, self.reference_peer)))


def crop_batch_is_ready(selected_count: int, minimum_constraints: int) -> bool:
    """Return whether a selected batch can start crop exchange.

    Selection may briefly produce fewer spatially distinct pairs than the
    registration contract requires.  Such a partial selection must remain
    pending; starting one-shot negotiation at that point would permanently
    prevent the remaining evidence from being requested.
    """
    return int(selected_count) >= max(1, int(minimum_constraints))


def evidence_pairs_for_selection(active_candidate_pairs, evidence_pairs):
    """Resolve selected candidates through their stable pair identities."""
    return [
        evidence_pairs[(pair[1], pair[0])]
        for pair in active_candidate_pairs
        if (pair[1], pair[0]) in evidence_pairs
    ]


def evidence_candidates_for_pool(candidate_pool, evidence_pairs):
    """Return pooled candidates whose evidence has actually arrived.

    Runtime candidate records may carry a leading similarity sort key while
    older selection records do not.  Normalize both shapes to the stable
    ``(peer_key, own_key, peer_descriptor, own_descriptor)`` form and resolve
    evidence only through the hashable ``(own_key, peer_key)`` identity.
    Descriptor objects are deliberately never used as dictionary keys.
    """
    resolved = []
    for candidate in candidate_pool:
        if len(candidate) == 5:
            _, peer_key, own_key, peer_descriptor, own_descriptor = candidate
        else:
            peer_key, own_key, peer_descriptor, own_descriptor = candidate
        if (own_key, peer_key) in evidence_pairs:
            resolved.append((peer_key, own_key, peer_descriptor,
                             own_descriptor))
    return resolved


def bounded_candidate_verification_order(candidate_pool, attempted_pairs,
                                         budget):
    """Return a deterministic, bounded ranking of unattempted pair IDs.

    Candidate objects may contain descriptor instances that are deliberately
    not hashable.  Only the stable ``(own_key, peer_key)`` identity is used for
    bookkeeping; descriptors never become dictionary/set keys.
    """
    attempted = set(attempted_pairs)
    ranked = []
    seen = set()
    for candidate in candidate_pool:
        if len(candidate) == 5:
            score, peer_key, own_key = candidate[:3]
        else:
            score, peer_key, own_key = 0.0, candidate[0], candidate[1]
        pair = (own_key, peer_key)
        if pair in attempted or pair in seen:
            continue
        seen.add(pair)
        ranked.append((float(score), str(peer_key), str(own_key), candidate))
    ranked.sort(key=lambda item: item[:3])
    return [item[3] for item in ranked[:max(0, int(budget))]]


def _identity_float(value: float) -> float:
    """Quantize geometry only to suppress serialization-level jitter."""
    return round(float(value), 6)


def physical_crop_identity(crop: GridCrop, map_epoch: int = 0,
                           checksum: int = 0) -> tuple:
    """Return a hashable identity for one physical crop.

    Keyframe IDs are deliberately absent.  The map revision/checksum and the
    complete crop geometry identify the evidence that registration can
    actually observe; keyframe IDs only identify the advertisement carrying
    it.  The tuple is intentionally bounded and contains no cell-data copy.
    """
    height, width = crop.values.shape[:2]
    centre_x, centre_y = _crop_center(crop)
    return (
        int(map_epoch), int(checksum), _identity_float(crop.resolution),
        int(width), int(height), _identity_float(crop.origin_x),
        _identity_float(crop.origin_y), _identity_float(crop.origin_yaw),
        _identity_float(centre_x), _identity_float(centre_y))


def physical_crop_geometry_identity(crop: GridCrop) -> tuple:
    """Identify the physical view independently of revision metadata.

    Epochs and checksums remain part of the exact evidence identity and are
    still required for request/response freshness.  This second, geometry-only
    identity is used for negative scheduling evidence: a map revision that
    advertises the same crop footprint must not consume another verification
    attempt after that footprint already failed geometric verification.
    """
    height, width = crop.values.shape[:2]
    centre_x, centre_y = _crop_center(crop)
    return (
        _identity_float(crop.resolution), int(width), int(height),
        _identity_float(crop.origin_x), _identity_float(crop.origin_y),
        _identity_float(crop.origin_yaw), _identity_float(centre_x),
        _identity_float(centre_y))


def physical_candidate_geometry_identity(candidate, own_crops: dict) -> tuple:
    """Return revision-independent geometry for a candidate crop pair."""
    if len(candidate) == 5:
        _, peer_key, own_key, peer_descriptor, _ = candidate
    else:
        peer_key, own_key, peer_descriptor, _ = candidate
    own_crop = own_crops[own_key]
    peer_crop = GridCrop(
        values=np.empty((int(peer_descriptor.crop_height),
                         int(peer_descriptor.crop_width)), dtype=np.int16),
        resolution=float(peer_descriptor.resolution),
        origin_x=float(peer_descriptor.crop_origin_x),
        origin_y=float(peer_descriptor.crop_origin_y),
        origin_yaw=float(getattr(peer_descriptor, 'crop_origin_yaw', 0.0)))
    return (physical_crop_geometry_identity(own_crop),
            physical_crop_geometry_identity(peer_crop))


def candidate_reuses_accepted_physical_view(
        candidate, accepted_geometry_keys: Iterable, own_crops: dict) -> bool:
    """Reject a pair that reuses either crop footprint already accepted.

    A pair can be geometrically distinct while still reusing the same peer
    crop (or the same local crop).  Such a pair is not an independent
    cross-robot view and can admit a locally plausible but globally
    inconsistent registration.  This is an acquisition rule only; the
    registration and consensus thresholds remain unchanged.
    """
    own_geometry, peer_geometry = physical_candidate_geometry_identity(
        candidate, own_crops)
    return any(
        own_geometry == accepted[0] or peer_geometry == accepted[1]
        for accepted in accepted_geometry_keys)


def candidate_views_are_spatially_separated(
        own_center, peer_center, prior_own_centers, prior_peer_centers,
        minimum_spacing_m: float = 0.40) -> bool:
    """Require both sides of a pending pair to add a displaced view.

    This is an acquisition-order rule for the bounded request batch.  It does
    not alter the production spatial-baseline or consensus thresholds.
    """
    spacing = float(minimum_spacing_m)
    if any(float(np.linalg.norm(np.asarray(own_center) - prior)) < spacing
           for prior in prior_own_centers):
        return False
    if any(float(np.linalg.norm(np.asarray(peer_center) - prior)) < spacing
           for prior in prior_peer_centers):
        return False
    return True


def physical_descriptor_identity(descriptor) -> tuple:
    """Return the corresponding physical identity for a descriptor message."""
    resolution = float(getattr(descriptor, 'resolution', 0.0))
    width = int(getattr(descriptor, 'crop_width', 0))
    height = int(getattr(descriptor, 'crop_height', 0))
    origin_x = float(getattr(descriptor, 'crop_origin_x', 0.0))
    origin_y = float(getattr(descriptor, 'crop_origin_y', 0.0))
    centre_x = origin_x + 0.5 * width * resolution
    centre_y = origin_y + 0.5 * height * resolution
    return (
        int(getattr(descriptor, 'map_epoch', 0)),
        int(getattr(descriptor, 'checksum', 0)),
        _identity_float(resolution), width, height,
        _identity_float(origin_x), _identity_float(origin_y), 0.0,
        _identity_float(centre_x), _identity_float(centre_y))


def physical_candidate_identity(candidate, own_crops: dict) -> tuple:
    """Identify a candidate by the advertised physical crop pair."""
    # Runtime eligible tuples carry the descending similarity sort key as a
    # leading field; pure formation fixtures may use the four-field identity
    # tuple.  The sort key is not physical evidence and is ignored here.
    if len(candidate) == 5:
        _, peer_key, own_key, peer_descriptor, own_descriptor = candidate
    else:
        peer_key, own_key, peer_descriptor, own_descriptor = candidate
    own_crop = own_crops[own_key]
    own_identity = physical_crop_identity(
        own_crop, getattr(own_descriptor, 'map_epoch', 0),
        getattr(own_descriptor, 'checksum', 0))
    return own_identity, physical_descriptor_identity(peer_descriptor)


def deduplicate_physical_candidates(candidates: Iterable, own_crops: dict) -> list:
    """Keep one deterministic candidate for each physical crop pair."""
    selected = []
    seen = set()
    for candidate in candidates:
        identity = physical_candidate_identity(candidate, own_crops)
        if identity in seen:
            continue
        seen.add(identity)
        selected.append(candidate)
    return selected


def accumulate_physical_candidates(candidate_pool: dict, candidates: Iterable,
                                   own_crops: dict) -> tuple[tuple, tuple]:
    """Add newly observed physical candidates to a bounded caller-owned pool.

    The returned tuples are ``(added, duplicates)``.  Advertisement/keyframe
    IDs remain available in each candidate for request routing, but the pool
    identity is physical evidence, so repeated observations cannot consume
    additional evidence slots.
    """
    added = []
    duplicates = []
    for candidate in candidates:
        identity = physical_candidate_identity(candidate, own_crops)
        if identity in candidate_pool:
            duplicates.append((identity, candidate))
            continue
        candidate_pool[identity] = candidate
        added.append((identity, candidate))
    return tuple(added), tuple(duplicates)


def evidence_batch_is_spatially_diverse(crops: Iterable[GridCrop],
                                        min_spatial_baseline_m: float = 0.75) -> bool:
    """Apply the existing physical spatial-diversity requirement."""
    crop_list = list(crops)
    if len(crop_list) < 2:
        return False
    centres = np.asarray([_crop_center(crop) for crop in crop_list])
    baseline = float(np.max(np.linalg.norm(
        centres[:, None, :] - centres[None, :, :], axis=2)))
    return baseline >= float(min_spatial_baseline_m)


@dataclass(frozen=True)
class GridCrop:
    """A bounded occupancy crop expressed in its source map frame."""

    values: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0


@dataclass(frozen=True)
class DescriptorMatch:
    """Cheap descriptor comparison result."""

    similarity: float
    margin: float
    sector_shift: int
    known_fraction: float


@dataclass(frozen=True)
class RegistrationResult:
    """Rigid source-to-target registration and quality metrics."""

    accepted: bool
    transform: tuple[float, float, float]
    covariance: tuple[float, ...]
    inlier_ratio: float
    residual_m: float
    occupied_free_agreement: float
    overlap_fraction: float
    reason: str
    constraint_count: int = 1
    consistent_constraint_count: int = 1
    spatial_baseline_m: float = 0.0
    angular_spread_rad: float = 0.0
    median_residual_m: float = math.inf
    p95_residual_m: float = math.inf
    translation_uncertainty_m: float = math.inf
    yaw_uncertainty_rad: float = math.inf
    condition_number: float = math.inf
    projected_error_m: float = math.inf
    final_confidence: float = 0.0
    # Bounded, JSON-friendly forensic data.  This is diagnostic only and does
    # not participate in registration or acceptance decisions.
    consensus_diagnostics: tuple = ()
    # Robust multi-hypothesis selector evidence.  These values are carried in
    # the replicated proposal/ack message so each peer can verify the same
    # winner without a central estimator.
    selector_status: str = 'INSUFFICIENT_EVIDENCE'
    selector_score: float = 0.0
    selector_null_score: float = 0.0
    selector_runner_up_score: float = -math.inf
    selector_runner_up_margin: float = 0.0
    selector_inlier_probabilities: tuple = ()


def hypothesis_is_acceptable(status: str, accepted: bool,
                             final_confidence: float) -> bool:
    """Gate merger/TF handoff on a mutually accepted strong hypothesis."""
    return bool(
        accepted and str(status) == 'ACCEPTED' and
        math.isfinite(float(final_confidence)) and
        float(final_confidence) >= MINIMUM_ACCEPTED_CONFIDENCE)


def crop_grid(
        values: np.ndarray,
        resolution: float,
        origin_x: float,
        origin_y: float,
        center_x: float | None = None,
        center_y: float | None = None,
        size_m: float = 8.0, origin_yaw: float = 0.0) -> GridCrop:
    """Return a bounded square crop without converting unknown to occupied."""
    array = np.asarray(values, dtype=np.int16)
    if array.ndim != 2 or resolution <= 0.0:
        raise ValueError('values must be a 2-D grid and resolution positive')
    side = max(4, int(round(size_m / resolution)))
    side = min(side, min(array.shape))
    center_x = (array.shape[1] * resolution / 2.0
                if center_x is None else center_x)
    center_y = (array.shape[0] * resolution / 2.0
                if center_y is None else center_y)
    cosine, sine = math.cos(origin_yaw), math.sin(origin_yaw)
    delta_x, delta_y = center_x - origin_x, center_y - origin_y
    local_center_x = cosine * delta_x + sine * delta_y
    local_center_y = -sine * delta_x + cosine * delta_y
    center_col = int(round(local_center_x / resolution))
    center_row = int(round(local_center_y / resolution))
    half = side // 2
    col0 = max(0, min(array.shape[1] - side, center_col - half))
    row0 = max(0, min(array.shape[0] - side, center_row - half))
    cropped = array[row0:row0 + side, col0:col0 + side].copy()
    local_origin = np.array([col0 * resolution, row0 * resolution])
    world_origin = np.array([origin_x, origin_y]) + np.array([
        cosine * local_origin[0] - sine * local_origin[1],
        sine * local_origin[0] + cosine * local_origin[1]])
    return GridCrop(
        values=cropped, resolution=float(resolution),
        origin_x=float(world_origin[0]), origin_y=float(world_origin[1]),
        origin_yaw=float(origin_yaw))


def should_accept_hypothesis(current, status: str, accepted: bool,
                             final_confidence: float) -> bool:
    """Allow only the first mutually accepted hypothesis to trigger handoff."""
    return current is None and hypothesis_is_acceptable(
        status, accepted, final_confidence)


def polar_descriptor(
        crop: GridCrop,
        rings: int = 12,
        sectors: int = 24) -> bytes:
    """Encode occupied/free structure with explicit unknown coverage.

    Each bin stores two bytes: occupied fraction and known fraction.  Unknown
    cells contribute to the denominator of known fraction but never to the
    occupied count.  Sector cyclic shifts make yaw comparison inexpensive;
    the radial aggregation makes the cheap gate tolerant to modest translation
    before geometric verification.
    """
    if rings <= 0 or sectors <= 0:
        raise ValueError('rings and sectors must be positive')
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
    known = values != UNKNOWN_VALUE
    occupied = known & (values >= OCCUPIED_THRESHOLD)
    # The bin populations are reductions over a fixed integer index.  Using
    # bincount is exactly equivalent to the previous ring/sector mask loop,
    # while avoiding 2 * rings * sectors full-grid temporary masks for every
    # descriptor period.
    bin_index = (ring_index * sectors + sector_index).ravel()
    total = np.bincount(
        bin_index, minlength=rings * sectors).astype(np.float64).reshape(
            rings, sectors)
    known_count = np.bincount(
        bin_index, weights=known.ravel().astype(np.float64),
        minlength=rings * sectors).reshape(rings, sectors)
    occupied_count = np.bincount(
        bin_index, weights=occupied.ravel().astype(np.float64),
        minlength=rings * sectors).reshape(rings, sectors)
    known_fraction = np.divide(
        known_count, total, out=np.zeros_like(total), where=total > 0.0)
    occupied_fraction = np.divide(
        occupied_count, known_count, out=np.zeros_like(total), where=known_count > 0.0)
    encoded = np.empty((rings, sectors, 2), dtype=np.uint8)
    encoded[:, :, 0] = np.rint(occupied_fraction * 255.0).astype(np.uint8)
    encoded[:, :, 1] = np.rint(known_fraction * 255.0).astype(np.uint8)
    return encoded.tobytes()


@lru_cache(maxsize=256)
def _prepared_descriptor(data: bytes, rings: int, sectors: int):
    """Decode one bounded descriptor and materialize its cyclic shifts once.

    Descriptor matching is repeated across the Cartesian product of the
    bounded local/peer keyframe histories.  The descriptor bytes are stable,
    so caching this immutable-by-convention numeric representation removes
    repeated conversion and ``roll`` allocations without changing the score.
    The cache is bounded to keep the frontend memory bounded.
    """
    array = np.frombuffer(data, dtype=np.uint8).reshape(rings, sectors, 2)
    array_float = array.astype(np.float32)
    shifted = np.stack(
        [np.roll(array, shift, axis=1) for shift in range(sectors)], axis=0)
    return array_float, shifted


def compare_descriptors(
        first: bytes,
        second: bytes,
        rings: int = 12,
        sectors: int = 24,
        minimum_known_fraction: float = 0.12) -> DescriptorMatch:
    """Compare descriptors over all cyclic yaw shifts."""
    expected = rings * sectors * 2
    if len(first) != expected or len(second) != expected:
        raise ValueError('descriptor size/version mismatch')
    a_float, _ = _prepared_descriptor(first, rings, sectors)
    _, shifted = _prepared_descriptor(second, rings, sectors)
    # Compare all cyclic sector shifts together.  This preserves the same
    # weighted occupied/known objective while avoiding repeated conversion,
    # roll, and reduction work for every candidate pair.
    known = (a_float[None, :, :, 1] +
             shifted[:, :, :, 1].astype(np.float32)) / 510.0
    weight = np.where(known >= minimum_known_fraction, known, 0.0)
    denominator = weight.sum(axis=(1, 2))
    diff = np.abs(a_float[None, :, :, :] -
                  shifted.astype(np.float32)) / 255.0
    weighted_error = (diff * weight[:, :, :, None]).sum(axis=(1, 2, 3))
    errors = np.divide(
        weighted_error, denominator * 2.0,
        out=np.ones_like(denominator), where=denominator > 1e-6)
    scores_array = np.clip(1.0 - errors, 0.0, 1.0)
    known_scores_array = weight.mean(axis=(1, 2))
    scores = scores_array.tolist()
    known_scores = known_scores_array.tolist()
    order = np.argsort(scores_array)[::-1]
    best = int(order[0])
    second = float(scores[order[1]]) if len(order) > 1 else 0.0
    return DescriptorMatch(
        similarity=float(scores[best]),
        margin=float(scores[best] - second),
        sector_shift=best,
        known_fraction=float(known_scores[best]),
    )


def compare_descriptor_batch(
        first: bytes, seconds: Iterable[bytes], rings: int = 12,
        sectors: int = 24, minimum_known_fraction: float = 0.12
        ) -> tuple[DescriptorMatch, ...]:
    """Compare one descriptor against a bounded batch without pair loops.

    This is algebraically the same objective as ``compare_descriptors``.  The
    batch dimension only removes repeated NumPy setup for the common anchor;
    each returned item is still evaluated by the unchanged descriptor,
    temporal, geometric, and consensus gates.
    """
    values = tuple(seconds)
    return compare_descriptor_pairs(
        (first,) * len(values), values, rings, sectors,
        minimum_known_fraction)


def compare_descriptor_pairs(
        firsts: Iterable[bytes], seconds: Iterable[bytes], rings: int = 12,
        sectors: int = 24, minimum_known_fraction: float = 0.12
        ) -> tuple[DescriptorMatch, ...]:
    """Compare a bounded set of independent descriptor pairs in one batch."""
    first_values = tuple(firsts)
    values = tuple(seconds)
    if len(first_values) != len(values):
        raise ValueError('descriptor batch lengths differ')
    expected = rings * sectors * 2
    if any(len(value) != expected for value in first_values + values):
        raise ValueError('descriptor size/version mismatch')
    if not values:
        return ()
    first_arrays = np.stack([
        np.frombuffer(value, dtype=np.uint8).reshape(rings, sectors, 2)
        for value in first_values
    ], axis=0)
    arrays = np.stack([
        np.frombuffer(value, dtype=np.uint8).reshape(rings, sectors, 2)
        for value in values
    ], axis=0)
    shifted = np.stack(
        [np.roll(arrays, shift, axis=2) for shift in range(sectors)], axis=1)
    known = (first_arrays[:, None, :, :, 1].astype(np.float32) +
             shifted[:, :, :, :, 1].astype(np.float32)) / 510.0
    weight = np.where(known >= minimum_known_fraction, known, 0.0)
    denominator = weight.sum(axis=(2, 3))
    diff = np.abs(first_arrays[:, None, :, :, :].astype(np.float32) -
                  shifted.astype(np.float32)) / 255.0
    weighted_error = (diff * weight[:, :, :, :, None]).sum(axis=(2, 3, 4))
    errors = np.divide(
        weighted_error, denominator * 2.0,
        out=np.ones_like(denominator), where=denominator > 1e-6)
    score_table = np.clip(1.0 - errors, 0.0, 1.0)
    known_table = weight.mean(axis=(2, 3))
    result = []
    for index in range(len(values)):
        order = np.argsort(score_table[index])[::-1]
        best = int(order[0])
        second = float(score_table[index, order[1]]) if sectors > 1 else 0.0
        result.append(DescriptorMatch(
            similarity=float(score_table[index, best]),
            margin=float(score_table[index, best] - second),
            sector_shift=best,
            known_fraction=float(known_table[index, best]),
        ))
    return tuple(result)


def rigidify_affine(matrix: np.ndarray, scale_tolerance: float = 0.05,
                    shear_tolerance: float = 0.05) -> tuple[float, float, float] | None:
    """Accept only a proper SE(2) component from a 2-D affine estimate."""
    affine = np.asarray(matrix, dtype=np.float64)
    if affine.shape != (2, 3) or not np.isfinite(affine).all():
        return None
    linear = affine[:, :2]
    determinant = float(np.linalg.det(linear))
    if determinant <= 0.0:
        return None
    singular = np.linalg.svd(linear, compute_uv=False)
    if abs(float(singular[0] - singular[1])) > shear_tolerance:
        return None
    scale = float(singular.mean())
    if abs(scale - 1.0) > scale_tolerance:
        return None
    rotation, _, rotation_t = np.linalg.svd(linear)
    orthogonal = rotation @ rotation_t
    if np.linalg.det(orthogonal) < 0.0:
        rotation[:, -1] *= -1.0
        orthogonal = rotation @ rotation_t
    yaw = math.atan2(float(orthogonal[1, 0]), float(orthogonal[0, 0]))
    return float(affine[0, 2]), float(affine[1, 2]), yaw


def _points(crop: GridCrop, occupied: bool) -> np.ndarray:
    values = np.asarray(crop.values)
    mask = values >= OCCUPIED_THRESHOLD if occupied else (
        (values != UNKNOWN_VALUE) & (values < OCCUPIED_THRESHOLD))
    rows, cols = np.nonzero(mask)
    if len(rows) == 0:
        return np.empty((0, 2), dtype=np.float64)
    local = np.column_stack(((cols + 0.5) * crop.resolution,
                             (rows + 0.5) * crop.resolution)).astype(np.float64)
    cosine, sine = math.cos(crop.origin_yaw), math.sin(crop.origin_yaw)
    rotation = np.asarray([[cosine, -sine], [sine, cosine]])
    return local @ rotation.T + np.asarray([crop.origin_x, crop.origin_y])


def _apply(points: np.ndarray, transform: tuple[float, float, float]) -> np.ndarray:
    tx, ty, yaw = transform
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    return points @ rotation.T + np.array([tx, ty])


def _nearest(points: np.ndarray, target: np.ndarray, tree=None):
    if cKDTree is not None:
        if tree is None:
            tree = cKDTree(target)
        return tree.query(points, k=1)
    distances = np.linalg.norm(points[:, None, :] - target[None, :, :], axis=2)
    indices = distances.argmin(axis=1)
    return distances[np.arange(len(points)), indices], indices


def _rigid_fit(source: np.ndarray, target: np.ndarray) -> tuple[float, float, float]:
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    centered_source = source - source_mean
    centered_target = target - target_mean
    u, _, vt = np.linalg.svd(centered_source.T @ centered_target)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = target_mean - rotation @ source_mean
    return (
        float(translation[0]), float(translation[1]),
        float(math.atan2(rotation[1, 0], rotation[0, 0])),
    )


def _empty_registration(reason: str) -> RegistrationResult:
    return RegistrationResult(
        False, (0.0, 0.0, 0.0), (0.0,) * 36, 0.0, math.inf, 0.0, 0.0,
        reason, constraint_count=0, consistent_constraint_count=0)


def _distance_field(crop: GridCrop):
    """Return an occupied distance field in metres, when OpenCV is present."""
    if cv2 is None:
        return None
    occupied = (np.asarray(crop.values) >= OCCUPIED_THRESHOLD).astype(np.uint8)
    # Distance to zero pixels; occupied cells are zero in the input.
    return cv2.distanceTransform(1 - occupied, cv2.DIST_L2, 3) * crop.resolution


def _field_distances(points: np.ndarray, crop: GridCrop, field) -> np.ndarray:
    """Sample an occupancy distance field at world-frame point coordinates."""
    if field is None or len(points) == 0:
        return np.full(len(points), math.inf, dtype=np.float64)
    cosine, sine = math.cos(crop.origin_yaw), math.sin(crop.origin_yaw)
    delta_x = points[:, 0] - crop.origin_x
    delta_y = points[:, 1] - crop.origin_y
    local_x = cosine * delta_x + sine * delta_y
    local_y = -sine * delta_x + cosine * delta_y
    columns = np.rint(local_x / crop.resolution - 0.5).astype(np.int64)
    rows = np.rint(local_y / crop.resolution - 0.5).astype(np.int64)
    valid = (
        (columns >= 0) & (columns < crop.values.shape[1]) &
        (rows >= 0) & (rows < crop.values.shape[0]))
    distances = np.full(len(points), max(crop.values.shape) * crop.resolution,
                        dtype=np.float64)
    distances[valid] = field[rows[valid], columns[valid]]
    return distances


def _translation_grid_field_distances(
        rotated_points: np.ndarray, base: np.ndarray, crop: GridCrop,
        field, translation_offsets: np.ndarray) -> np.ndarray:
    """Sample one yaw's translation grid without rebuilding world points.

    This is algebraically the same transform used by ``_field_distances``:
    the target-origin rotation is distributed over the rotated source points
    and the translation offsets before cell rounding.  Keeping the grid
    dimension explicit avoids a large flattened world-coordinate temporary
    for every yaw seed.
    """
    cosine, sine = math.cos(crop.origin_yaw), math.sin(crop.origin_yaw)
    delta = rotated_points + base[None, :] - np.asarray(
        [crop.origin_x, crop.origin_y], dtype=np.float64)
    local_base_x = cosine * delta[:, 0] + sine * delta[:, 1]
    local_base_y = -sine * delta[:, 0] + cosine * delta[:, 1]
    offset_x = (cosine * translation_offsets[:, 0] +
                sine * translation_offsets[:, 1])[:, None]
    offset_y = (-sine * translation_offsets[:, 0] +
                cosine * translation_offsets[:, 1])[:, None]
    columns = np.rint(
        (local_base_x[None, :] + offset_x) / crop.resolution - 0.5,
    ).astype(np.int64)
    rows = np.rint(
        (local_base_y[None, :] + offset_y) / crop.resolution - 0.5,
    ).astype(np.int64)
    valid = (
        (columns >= 0) & (columns < crop.values.shape[1]) &
        (rows >= 0) & (rows < crop.values.shape[0]))
    distances = np.full(
        columns.shape, max(crop.values.shape) * crop.resolution,
        dtype=np.float64)
    distances[valid] = field[rows[valid], columns[valid]]
    return distances


def _world_extent(crop: GridCrop):
    cosine, sine = math.cos(crop.origin_yaw), math.sin(crop.origin_yaw)
    corners = np.asarray([
        (0.0, 0.0),
        (crop.values.shape[1] * crop.resolution, 0.0),
        (0.0, crop.values.shape[0] * crop.resolution),
        (crop.values.shape[1] * crop.resolution,
         crop.values.shape[0] * crop.resolution),
    ])
    rotation = np.asarray([[cosine, -sine], [sine, cosine]])
    world = corners @ rotation.T + np.asarray([crop.origin_x, crop.origin_y])
    return np.array([
        world[:, 0].min(), world[:, 1].min(),
        world[:, 0].max(), world[:, 1].max()], dtype=np.float64)


def _coarse_registration_seeds(
        source_points: np.ndarray, target: GridCrop, target_points: np.ndarray,
        max_yaw_steps: int = 72, translation_step_m: float = 0.05,
        translation_radius_m: float = 0.40, keep: int = 8):
    """Find globally distinct rigid seeds using an occupied distance field.

    This is deliberately bounded.  It is a distance-field correlation rather
    than ICP, so large initial translation/yaw errors do not depend on the
    nearest-neighbour basin selected by the first iteration.
    """
    field = _distance_field(target)
    if field is None:
        return []
    source = source_points
    if len(source) > 1000:
        source = source[::max(1, len(source) // 1000)]
    target_center = target_points.mean(axis=0)
    source_center = source.mean(axis=0)
    candidates = []
    offsets = np.arange(
        -translation_radius_m, translation_radius_m + 0.5 * translation_step_m,
        translation_step_m)
    # Keep the original deterministic dx-major/dy-minor ordering, but score
    # every translation offset for one yaw in one NumPy operation.  The old
    # nested Python loop called _field_distances and np.median once per offset
    # (up to 20,808 calls per registration); batching removes only that
    # interpreter/temporary-array overhead and preserves the exact objective.
    translation_offsets = np.stack(
        np.meshgrid(offsets, offsets, indexing='ij'), axis=-1).reshape(-1, 2)
    for yaw in np.linspace(-math.pi, math.pi, max_yaw_steps, endpoint=False):
        cosine, sine = math.cos(float(yaw)), math.sin(float(yaw))
        rotation = np.array([[cosine, -sine], [sine, cosine]])
        base = target_center - rotation @ source_center
        rotated = source @ rotation.T
        distances = _translation_grid_field_distances(
            rotated, base, target, field, translation_offsets)
        clipped = np.minimum(distances, 0.50)
        scores = np.median(clipped, axis=1) + 0.25 * np.mean(clipped, axis=1)
        transforms = np.column_stack((
            base[0] + translation_offsets[:, 0],
            base[1] + translation_offsets[:, 1],
            np.full(len(translation_offsets), float(yaw))))
        candidates.extend(
            (float(score), (float(transform[0]), float(transform[1]),
                            float(transform[2])))
            for score, transform in zip(scores, transforms))
    candidates.sort(key=lambda value: (value[0], value[1]))
    distinct = []
    for score, transform in candidates:
        if all(
                abs(transform[0] - other[1][0]) > 0.20 or
                abs(transform[1] - other[1][1]) > 0.20 or
                abs(wrap_angle(transform[2] - other[1][2])) > math.radians(8.0)
                for other in distinct):
            distinct.append((score, transform))
        if len(distinct) >= keep:
            break
    return [transform for _, transform in distinct]


def _ecc_registration_seed(source: GridCrop, target: GridCrop):
    """Return a rigid image-correlation seed when crop rasters are compatible.

    OpenCV's ECC result is a pixel-coordinate warp from the source image into
    the target image.  The half-cell and map-origin terms below convert that
    result exactly into the same world-frame SE(2) convention used by the
    point registration code.  ECC is only a seed; geometric verification and
    multi-keyframe consensus remain authoritative.
    """
    if (cv2 is None or source.values.shape != target.values.shape or
            abs(source.resolution - target.resolution) > 1e-9):
        return None
    source_image = (np.asarray(source.values) >= OCCUPIED_THRESHOLD).astype(np.float32)
    target_image = (np.asarray(target.values) >= OCCUPIED_THRESHOLD).astype(np.float32)
    if np.count_nonzero(source_image) < 12 or np.count_nonzero(target_image) < 12:
        return None
    warp = np.eye(2, 3, dtype=np.float32)
    try:
        _, warp = cv2.findTransformECC(
            source_image, target_image, warp, cv2.MOTION_EUCLIDEAN,
            (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 250, 1e-6),
            None, 1)
    except cv2.error:
        return None
    linear = np.asarray(warp[:, :2], dtype=np.float64)
    if np.linalg.det(linear) <= 0.0:
        return None
    # Re-orthogonalize tiny raster/ECC scale errors instead of allowing affine
    # distortion into the runtime hypothesis.
    u, _, vt = np.linalg.svd(linear)
    pixel_rotation = u @ vt
    if np.linalg.det(pixel_rotation) < 0.0:
        u[:, -1] *= -1.0
        pixel_rotation = u @ vt
    half = np.array([0.5, 0.5], dtype=np.float64)
    source_origin = np.array([source.origin_x, source.origin_y])
    target_origin = np.array([target.origin_x, target.origin_y])
    source_cos, source_sin = math.cos(source.origin_yaw), math.sin(source.origin_yaw)
    target_cos, target_sin = math.cos(target.origin_yaw), math.sin(target.origin_yaw)
    source_rotation = np.asarray([[source_cos, -source_sin],
                                   [source_sin, source_cos]])
    target_rotation = np.asarray([[target_cos, -target_sin],
                                   [target_sin, target_cos]])
    rotation = target_rotation @ pixel_rotation @ source_rotation.T
    translation = (target_origin + target_rotation @ (
        target.resolution * (np.asarray(warp[:, 2], dtype=np.float64) + half))
        - rotation @ (source_origin + source_rotation @ (
            source.resolution * half)))
    return (float(translation[0]), float(translation[1]),
            float(math.atan2(rotation[1, 0], rotation[0, 0])))


def _refine_registration(source_points, target_points, target, seed,
                         max_iterations=35, max_correspondence_m=0.30,
                         target_tree=None):
    """Robust trimmed point-to-point refinement from one global seed."""
    transform = seed
    for _ in range(max_iterations):
        transformed = _apply(source_points, transform)
        distances, indices = _nearest(transformed, target_points, target_tree)
        threshold = min(
            max_correspondence_m,
            max(3.0 * source_points.dtype.type(target.resolution),
                float(np.percentile(distances, 60)) * 2.0))
        mask = distances <= threshold
        if np.count_nonzero(mask) < 8:
            break
        # Trim the worst tail before fitting so map ghosts do not rotate the
        # solution.  This keeps the fit rigid and deterministic.
        selected = np.flatnonzero(mask)
        if len(selected) > 600:
            selected = selected[np.argsort(distances[selected])[:600]]
        refined = _rigid_fit(source_points[selected], target_points[indices[selected]])
        delta_t = np.linalg.norm(
            np.asarray(refined[:2]) - np.asarray(transform[:2]))
        delta_yaw = abs(wrap_angle(refined[2] - transform[2]))
        transform = refined
        if delta_t < 1e-5 and delta_yaw < 1e-5:
            break
    return transform


def _occupancy_consistency(
        points: np.ndarray, crop: GridCrop) -> tuple[float, float, float]:
    """Score transformed occupied points in one map frame.

    The geometric verifier is run independently in both directions by the
    two robots.  A target-only occupancy score therefore made an otherwise
    identical crop pair depend on which peer happened to be the verifier.
    Return occupied agreement, known-free consistency, and in-bounds overlap
    in the supplied crop frame so the caller can combine both directions
    symmetrically.  Map-origin yaw is handled explicitly here; the old
    target-only calculation assumed an axis-aligned crop.
    """
    points = np.asarray(points, dtype=np.float64)
    if not len(points):
        return 0.0, 0.0, 0.0
    cosine, sine = math.cos(crop.origin_yaw), math.sin(crop.origin_yaw)
    rotation_t = np.asarray([[cosine, sine], [-sine, cosine]])
    local = (points - np.asarray([crop.origin_x, crop.origin_y])) @ rotation_t.T
    columns = np.rint(local[:, 0] / crop.resolution - 0.5).astype(int)
    rows = np.rint(local[:, 1] / crop.resolution - 0.5).astype(int)
    values = np.asarray(crop.values)
    valid = (
        (columns >= 0) & (columns < values.shape[1]) &
        (rows >= 0) & (rows < values.shape[0]))
    occupied_match = np.zeros(len(points), dtype=bool)
    occupied_match[valid] = (
        values[rows[valid], columns[valid]] >= OCCUPIED_THRESHOLD)
    known_free = np.zeros(len(points), dtype=bool)
    known_free[valid] = values[rows[valid], columns[valid]] == 0
    occupied_fraction = float(np.count_nonzero(occupied_match)) / float(
        len(points))
    free_consistency = 1.0 - float(np.count_nonzero(known_free)) / float(
        len(points))
    overlap = float(np.count_nonzero(valid)) / float(len(points))
    return occupied_fraction, free_consistency, overlap


def _registration_quality(source: GridCrop, target: GridCrop,
                          source_points: np.ndarray,
                          target_points: np.ndarray, transform,
                          max_correspondence_m=0.30,
                          source_tree=None, target_tree=None) -> RegistrationResult:
    transformed = _apply(source_points, transform)
    distances, indices = _nearest(transformed, target_points, target_tree)
    threshold = min(
        max_correspondence_m,
        max(3.0 * source.resolution, float(np.percentile(distances, 60)) * 2.5))
    mask = distances <= threshold
    inlier_count = int(np.count_nonzero(mask))
    inlier_ratio = float(inlier_count) / float(max(1, len(source_points)))
    residuals = distances[mask] if inlier_count else np.empty(0)
    residual = (float(np.sqrt(np.mean(np.square(residuals))))
                if inlier_count else math.inf)
    median_residual = float(np.median(residuals)) if inlier_count else math.inf
    p95_residual = float(np.percentile(residuals, 95)) if inlier_count else math.inf

    # Verify in both directions.  A one-way nearest-neighbour fit can match a
    # small repeated fragment in a large unrelated crop.
    inverse = invert_se2(transform)
    reverse = _apply(target_points, inverse)
    reverse_distances, _ = _nearest(reverse, source_points, source_tree)
    reverse_threshold = max(3.0 * target.resolution, threshold)
    reverse_ratio = float(np.count_nonzero(reverse_distances <= reverse_threshold)) / \
        float(max(1, len(target_points)))

    forward_occupied, forward_free, forward_overlap = _occupancy_consistency(
        transformed, target)
    reverse_occupied, reverse_free, reverse_overlap = _occupancy_consistency(
        reverse, source)
    # Both peers must score the same physical evidence the same way.  Use the
    # mean map consistency from both frames (unknown cells remain neutral) and
    # the stricter overlap of the two frames.  This preserves the existing
    # 0.55 acceptance threshold instead of lowering it for one direction.
    occupied_fraction = 0.5 * (forward_occupied + reverse_occupied)
    free_consistency = 0.5 * (forward_free + reverse_free)
    occupied_agreement = max(
        0.0, min(1.0, 0.55 * min(inlier_ratio, reverse_ratio)
                 + 0.25 * occupied_fraction + 0.20 * free_consistency))
    overlap = min(forward_overlap, reverse_overlap)

    centered = source_points - source_points.mean(axis=0)
    eigenvalues = np.linalg.eigvalsh(centered.T @ centered)
    condition = (float(eigenvalues[-1] / max(eigenvalues[0], 1e-9))
                 if len(eigenvalues) == 2 else math.inf)
    variance = residual * residual / max(1, inlier_count)
    covariance = np.zeros((6, 6), dtype=np.float64)
    covariance[0, 0] = covariance[1, 1] = variance
    yaw_variance = variance / max(1e-6, float(eigenvalues.sum()))
    covariance[5, 5] = yaw_variance
    accepted = (
        inlier_ratio >= 0.35 and reverse_ratio >= 0.30 and
        residual <= max(0.12, 3.0 * source.resolution) and
        occupied_agreement >= 0.55 and overlap >= 0.15)
    reason = 'ACCEPTED' if accepted else 'GEOMETRIC_VERIFICATION_REJECTED'
    return RegistrationResult(
        accepted=accepted, transform=transform,
        covariance=tuple(float(value) for value in covariance.ravel()),
        inlier_ratio=inlier_ratio, residual_m=residual,
        occupied_free_agreement=occupied_agreement, overlap_fraction=overlap,
        reason=reason, median_residual_m=median_residual,
        p95_residual_m=p95_residual,
        translation_uncertainty_m=math.sqrt(max(0.0, variance)),
        yaw_uncertainty_rad=math.sqrt(max(0.0, yaw_variance)),
        condition_number=condition,
        projected_error_m=math.inf)


def register_crops(
        source: GridCrop,
        target: GridCrop,
        max_iterations: int = 25,
        max_correspondence_m: float = 0.30) -> RegistrationResult:
    """Register one crop with bounded global hypotheses then robust refinement.

    The function remains the backwards-compatible single-pair diagnostic API.
    Production acceptance should use :func:`register_crop_set`, which requires
    independent evidence and a physical projected-error gate.
    """
    source_points = _points(source, occupied=True)
    target_points = _points(target, occupied=True)
    if len(source_points) < 12 or len(target_points) < 12:
        return _empty_registration('INSUFFICIENT_OCCUPIED_GEOMETRY')
    if len(source_points) > 1200:
        source_points = source_points[::max(1, len(source_points) // 1200)]
    if len(target_points) > 1200:
        target_points = target_points[::max(1, len(target_points) // 1200)]
    seeds = _coarse_registration_seeds(
        source_points, target, target_points, max_yaw_steps=72,
        translation_step_m=max(source.resolution * 2.0, 0.05),
        translation_radius_m=max(0.40, max_correspondence_m * 1.5), keep=6)
    ecc_seed = _ecc_registration_seed(source, target)
    if ecc_seed is not None:
        seeds.insert(0, ecc_seed)
    if not seeds:
        return _empty_registration('NO_COARSE_ALIGNMENT')
    results = []
    target_field = _distance_field(target)
    source_tree = cKDTree(source_points) if cKDTree is not None else None
    target_tree = cKDTree(target_points) if cKDTree is not None else None
    for seed in seeds:
        seed_result = _registration_quality(
            source, target, source_points, target_points, seed,
            max_correspondence_m, source_tree, target_tree)
        transform = _refine_registration(
            source_points, target_points, target, seed,
            max_iterations=max_iterations,
            max_correspondence_m=max_correspondence_m,
            target_tree=target_tree)
        refined_result = _registration_quality(
            source, target, source_points, target_points, transform,
            max_correspondence_m, source_tree, target_tree)
        results.extend((seed_result, refined_result))
    def ranking(result):
        if target_field is None:
            global_score = result.residual_m
        else:
            distances = _field_distances(
                _apply(source_points, result.transform), target, target_field)
            global_score = float(np.mean(np.minimum(distances, 0.50)))
        return (-int(result.accepted), global_score, result.residual_m,
                -result.inlier_ratio, abs(result.transform[2]))
    return min(results, key=ranking)


def wrap_angle(angle: float) -> float:
    """Return an angle in [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def compose_se2(first: tuple[float, float, float],
                second: tuple[float, float, float]) -> tuple[float, float, float]:
    """Compose transforms using ``T_A_B * T_B_C = T_A_C``."""
    tx, ty, yaw = first
    sx, sy, syaw = second
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return (tx + cosine * sx - sine * sy,
            ty + sine * sx + cosine * sy,
            wrap_angle(yaw + syaw))


def invert_se2(transform: tuple[float, float, float]) -> tuple[float, float, float]:
    """Invert an SE(2) transform under the explicit point convention."""
    tx, ty, yaw = transform
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return (-cosine * tx - sine * ty,
            sine * tx - cosine * ty,
            wrap_angle(-yaw))


def projected_registration_error(transform_error: tuple[float, float, float],
                                 target_map_radius_m: float = 40.0) -> float:
    """Project SE(2) error to the configured physical map radius."""
    return (math.hypot(transform_error[0], transform_error[1]) +
            abs(wrap_angle(transform_error[2])) * float(target_map_radius_m))


def _transform_distance(first, second):
    delta = compose_se2(invert_se2(second), first)
    return math.hypot(delta[0], delta[1]), abs(wrap_angle(delta[2]))


def _crop_center(crop: GridCrop) -> tuple[float, float]:
    cosine, sine = math.cos(crop.origin_yaw), math.sin(crop.origin_yaw)
    local = np.asarray([
        0.5 * crop.values.shape[1] * crop.resolution,
        0.5 * crop.values.shape[0] * crop.resolution])
    return tuple((np.asarray([crop.origin_x, crop.origin_y]) +
                  np.asarray([[cosine, -sine], [sine, cosine]]) @ local)
                 .tolist())


def _pair_spatial_baseline(pairs, indices):
    """Measure baseline in each frame and retain the stronger one.

    Source and target crop centers live in different map frames.  Compute
    each frame's baseline independently and take the maximum; mixing the two
    coordinate systems would make the gate order-dependent under inversion.
    """
    baselines = []
    for side in (0, 1):
        centers = [_crop_center(pairs[index][side]) for index in indices]
        if len(centers) < 2:
            continue
        points = np.asarray(centers, dtype=np.float64)
        baselines.append(float(np.max(np.linalg.norm(
            points[:, None, :] - points[None, :, :], axis=2))))
    return max(baselines, default=0.0)


def consensus_subset_diagnostics(
        pairs: list[tuple[GridCrop, GridCrop]],
        results: list[RegistrationResult],
        target_map_radius_m: float = 40.0,
        min_consistent_constraints: int = 3,
        min_spatial_baseline_m: float = 0.75,
        max_translation_consistency_m: float = 0.15,
        max_yaw_consistency_rad: float = math.radians(1.0),
        max_projected_registration_error_m: float = 0.20,
        min_inlier_ratio: float = 0.55,
        max_robust_residual_m: float = 0.08) -> tuple[dict, ...]:
    """Return bounded per-constraint and subset forensic diagnostics.

    This function intentionally mirrors the existing consensus thresholds but
    does not choose a cluster or alter the production result.  It makes the
    distinction between an invalid individual registration, an inconsistent
    transform subset, and a subset that reaches a later physical gate.
    """
    diagnostics = []
    for index, (pair, result) in enumerate(zip(pairs, results)):
        diagnostics.append({
            'kind': 'constraint',
            'index': index,
            'source_origin': [float(pair[0].origin_x), float(pair[0].origin_y),
                              float(pair[0].origin_yaw)],
            'target_origin': [float(pair[1].origin_x), float(pair[1].origin_y),
                              float(pair[1].origin_yaw)],
            'source_center': list(_crop_center(pair[0])),
            'target_center': list(_crop_center(pair[1])),
            'accepted_geometric': bool(result.accepted),
            'transform': [float(value) for value in result.transform],
            'residual_m': float(result.residual_m),
            'median_residual_m': float(result.median_residual_m),
            'p95_residual_m': float(result.p95_residual_m),
            'inlier_ratio': float(result.inlier_ratio),
            'occupied_free_agreement': float(result.occupied_free_agreement),
            'overlap_fraction': float(result.overlap_fraction),
            'translation_uncertainty_m': float(result.translation_uncertainty_m),
            'yaw_uncertainty_rad': float(result.yaw_uncertainty_rad),
            'condition_number': float(result.condition_number),
            'projected_error_m': float(result.projected_error_m),
            'registration_reason': str(result.reason),
        })

    accepted_indices = [index for index, result in enumerate(results)
                        if result.accepted]
    for first, second in combinations(accepted_indices, 2):
        translation, yaw = _transform_distance(
            results[first].transform, results[second].transform)
        diagnostics.append({
            'kind': 'pairwise_comparison',
            'first_index': first,
            'second_index': second,
            'translation_disagreement_m': float(translation),
            'yaw_disagreement_rad': float(yaw),
            'max_translation_consistency_m': float(
                max_translation_consistency_m),
            'max_yaw_consistency_rad': float(max_yaw_consistency_rad),
            'consistent': bool(
                translation <= max_translation_consistency_m and
                yaw <= max_yaw_consistency_rad),
        })

    subset_size = max(1, int(min_consistent_constraints))
    for indices in combinations(range(len(results)), subset_size):
        subset_results = [results[index] for index in indices]
        subset_pairs = [pairs[index] for index in indices]
        reasons = []
        if not all(result.accepted for result in subset_results):
            reasons.append('GEOMETRIC_MEMBER_REJECTED')
        disagreements = [
            _transform_distance(left.transform, right.transform)
            for left, right in combinations(subset_results, 2)]
        max_translation = max((value[0] for value in disagreements),
                              default=0.0)
        max_yaw = max((value[1] for value in disagreements), default=0.0)
        if (max_translation > max_translation_consistency_m or
                max_yaw > max_yaw_consistency_rad):
            reasons.append('PAIRWISE_TRANSFORM_INCONSISTENT')
        spatial_baseline = _pair_spatial_baseline(
            pairs, indices)
        if spatial_baseline < min_spatial_baseline_m:
            reasons.append('INSUFFICIENT_SPATIAL_BASELINE')
        inlier_ratio = float(np.mean(
            [result.inlier_ratio for result in subset_results]))
        if inlier_ratio < min_inlier_ratio:
            reasons.append('INLIER_RATIO_BELOW_THRESHOLD')
        robust_residual = float(np.median(
            [result.residual_m for result in subset_results]))
        if robust_residual > max_robust_residual_m:
            reasons.append('ROBUST_RESIDUAL_ABOVE_THRESHOLD')
        translation_uncertainty = math.sqrt(float(np.mean([
            result.translation_uncertainty_m ** 2
            for result in subset_results])))
        yaw_uncertainty = math.sqrt(float(np.mean([
            result.yaw_uncertainty_rad ** 2
            for result in subset_results])))
        projected = (translation_uncertainty +
                     float(target_map_radius_m) * yaw_uncertainty)
        if projected > max_projected_registration_error_m:
            reasons.append('PROJECTED_ERROR_ABOVE_THRESHOLD')
        condition = max(result.condition_number for result in subset_results)
        if condition >= 1e4:
            reasons.append('CONDITIONING_ABOVE_THRESHOLD')
        diagnostics.append({
            'kind': 'subset_comparison',
            'indices': list(indices),
            'subset_size': subset_size,
            'all_geometric_members_accepted': not any(
                reason == 'GEOMETRIC_MEMBER_REJECTED' for reason in reasons),
            'max_translation_disagreement_m': max_translation,
            'max_yaw_disagreement_rad': max_yaw,
            'spatial_baseline_m': spatial_baseline,
            'mean_inlier_ratio': inlier_ratio,
            'median_residual_m': robust_residual,
            'translation_uncertainty_m': translation_uncertainty,
            'yaw_uncertainty_rad': yaw_uncertainty,
            'projected_error_m_at_40m': projected,
            'target_map_radius_m': float(target_map_radius_m),
            'condition_number': condition,
            'thresholds': {
                'min_consistent_constraints': subset_size,
                'min_spatial_baseline_m': float(min_spatial_baseline_m),
                'max_translation_consistency_m': float(
                    max_translation_consistency_m),
                'max_yaw_consistency_rad': float(max_yaw_consistency_rad),
                'max_projected_registration_error_m': float(
                    max_projected_registration_error_m),
                'min_inlier_ratio': float(min_inlier_ratio),
                'max_robust_residual_m': float(max_robust_residual_m),
            },
            'rejection_reasons': reasons,
            'consistent_subset': not reasons,
        })
    return tuple(diagnostics)


def register_crop_set(
        pairs: Iterable[tuple[GridCrop, GridCrop]],
        target_map_radius_m: float = 40.0,
        min_consistent_constraints: int = 3,
        min_spatial_baseline_m: float = 0.75,
        max_translation_consistency_m: float = 0.15,
        max_yaw_consistency_rad: float = math.radians(1.0),
        max_projected_registration_error_m: float = 0.20,
        min_inlier_ratio: float = 0.55,
        max_robust_residual_m: float = 0.08,
        min_candidate_margin: float = 0.02,
        individual_results: Iterable[RegistrationResult] | None = None,
        evidence_timestamps: Iterable[tuple[int, int]] | None = None,
        evidence_ids: Iterable[str] | None = None,
        hypothesis_accumulator: IncrementalHypothesisAccumulator | None = None
        ) -> RegistrationResult:
    """Estimate one transform from an independently verified crop set.

    Each pair is registered independently, then transforms are clustered in
    SE(2).  The surviving cluster is refined by weighted averaging of its
    translations and circular yaw values.  This is a bounded PCM-like
    consistency gate: one wrong crop cannot determine the handoff.
    """
    pair_list = list(pairs)
    if not pair_list:
        return _empty_registration('NO_CONSTRAINTS')
    if individual_results is None:
        results = [register_crops(source, target)
                   for source, target in pair_list]
    else:
        cached = list(individual_results)
        if len(cached) != len(pair_list):
            raise ValueError(
                'individual_results must align one-for-one with pairs')
        # Candidate verification has already run the exact same bounded
        # single-crop registration for each accepted pair.  Reuse those
        # immutable results during incremental consensus; this removes only
        # duplicate work and leaves all clustering, spatial-diversity,
        # consistency, uncertainty, and projected-error gates unchanged.
        results = [
            result if result is not None else register_crops(source, target)
            for (source, target), result in zip(pair_list, cached)]
    forensic = consensus_subset_diagnostics(
        pair_list, results,
        target_map_radius_m=target_map_radius_m,
        min_consistent_constraints=min_consistent_constraints,
        min_spatial_baseline_m=min_spatial_baseline_m,
        max_translation_consistency_m=max_translation_consistency_m,
        max_yaw_consistency_rad=max_yaw_consistency_rad,
        max_projected_registration_error_m=max_projected_registration_error_m,
        min_inlier_ratio=min_inlier_ratio,
        max_robust_residual_m=max_robust_residual_m)
    accepted = [result for result in results if result.accepted]
    if not accepted:
        return _empty_registration('NO_GEOMETRIC_CONSTRAINT')
    # The old implementation grew one evolving mean in arrival order.  That
    # could absorb a third constraint even when the final set contained a
    # pairwise-inconsistent transform.  Use a bounded multi-hypothesis,
    # covariance-weighted selector instead; the single-crop registration above
    # remains unchanged and still supplies the observations.
    accepted_indices = [index for index, result in enumerate(results)
                        if result.accepted]
    timestamp_list = list(evidence_timestamps or ())
    robust_candidates = []
    evidence_id_list = list(evidence_ids or ())
    for index in accepted_indices:
        pair = pair_list[index]
        item = results[index]
        timestamp_pair = (timestamp_list[index]
                          if index < len(timestamp_list) else (0, 0))
        robust_candidates.append(PoseConstraint(
            transform=tuple(float(value) for value in item.transform),
            covariance=tuple(float(value) for value in item.covariance),
            quality=max(0.05, min(1.0, float(
                0.45 * item.inlier_ratio +
                0.30 * item.occupied_free_agreement +
                0.25 * item.overlap_fraction))),
            evidence_id=(str(evidence_id_list[index])
                         if index < len(evidence_id_list) else str(index)),
            source_center=_crop_center(pair[0]),
            target_center=_crop_center(pair[1]),
            source_timestamp_ns=int(timestamp_pair[0]),
            target_timestamp_ns=int(timestamp_pair[1])))
    if hypothesis_accumulator is None:
        robust = select_robust_hypothesis(
            robust_candidates,
            min_inliers=min_consistent_constraints,
            min_spatial_baseline_m=min_spatial_baseline_m,
            max_translation_disagreement_m=max_translation_consistency_m,
            max_yaw_disagreement_rad=max_yaw_consistency_rad)
    else:
        robust = hypothesis_accumulator.update(robust_candidates)
        # The accumulator keeps a lexicographically ordered historical set,
        # while this registration call retains insertion order.  Convert the
        # stable winner IDs back to the current result indices before the
        # legacy RegistrationResult assembly below consumes them.
        winner_ids = ()
        for diagnostic in reversed(robust.diagnostics):
            if diagnostic.get('kind') == 'incremental_hypothesis_accumulator':
                winner_ids = tuple(str(value) for value in diagnostic.get(
                    'winner_evidence_ids', ()))
                break
        if winner_ids:
            id_to_index = {
                str(candidate.evidence_id): index
                for index, candidate in enumerate(robust_candidates)}
            selected = tuple(id_to_index[value] for value in winner_ids
                             if value in id_to_index)
            selected_set = set(selected)
            probabilities = tuple(
                1.0 if index in selected_set else 0.0
                for index in range(len(robust_candidates)))
            robust = replace(robust, selected_indices=selected,
                             inlier_probabilities=probabilities)
    forensic = tuple(forensic) + tuple(robust.diagnostics)
    if robust.status != ACCEPTED_HYPOTHESIS:
        return RegistrationResult(
            accepted=False, transform=robust.transform,
            covariance=robust.covariance, inlier_ratio=0.0,
            residual_m=math.inf, occupied_free_agreement=0.0,
            overlap_fraction=0.0,
            reason='INSUFFICIENT_CONSISTENT_CONSTRAINTS',
            constraint_count=len(results),
            consistent_constraint_count=len(robust.selected_indices),
            projected_error_m=math.inf,
            final_confidence=0.0,
            consensus_diagnostics=forensic,
            selector_status=robust.status,
            selector_score=robust.score,
            selector_null_score=robust.null_score,
            selector_runner_up_score=robust.runner_up_score,
            selector_runner_up_margin=robust.runner_up_margin,
            selector_inlier_probabilities=robust.inlier_probabilities)
    selected_result_indices = [accepted_indices[index]
                               for index in robust.selected_indices]
    items = [results[index] for index in selected_result_indices]
    transform = robust.transform
    residuals = np.asarray([item.residual_m for item in items])
    inlier_ratio = float(np.average(
        [item.inlier_ratio for item in items],
        weights=np.maximum(1e-3, 1.0 / np.maximum(residuals, 1e-3))))
    robust_residual = float(np.median(residuals))
    p95_residual = float(np.percentile(residuals, 95))
    # Constraint diversity is measured from the observed crop locations, not
    # from the estimated transforms (a correct rigid transform is expected to
    # be nearly identical for every crop).
    selected_result_ids = {id(selected) for selected in items}
    selected_pair_indices = [index for index, item in enumerate(results)
                             if id(item) in selected_result_ids]
    spatial_baseline = _pair_spatial_baseline(
        pair_list, selected_pair_indices)
    yaws = np.asarray([item.transform[2] for item in items])
    angular_spread = (float(np.max([abs(wrap_angle(yaw - transform[2]))
                                    for yaw in yaws]))
                      if len(yaws) > 1 else 0.0)
    condition = max(item.condition_number for item in items)
    translation_uncertainty = math.sqrt(max(
        0.0, float(np.mean([item.translation_uncertainty_m ** 2
                            for item in items]))))
    yaw_uncertainty = math.sqrt(max(
        0.0, float(np.mean([item.yaw_uncertainty_rad ** 2
                            for item in items]))))
    confidence = max(0.0, min(1.0,
        0.25 * min(1.0, len(items) / max(1, min_consistent_constraints)) +
        0.20 * inlier_ratio +
        0.15 * max(0.0, 1.0 - robust_residual / max(max_robust_residual_m, 1e-3)) +
        0.15 * min(1.0, spatial_baseline / max(min_spatial_baseline_m, 1e-3)) +
        0.15 * min(1.0, sum(item.occupied_free_agreement for item in items) /
                   max(1, len(items))) +
        0.10 * min(1.0, sum(item.overlap_fraction for item in items) /
                   max(1, len(items)))))
    result = items[0]
    projected = translation_uncertainty + target_map_radius_m * yaw_uncertainty
    accepted_final = (
        len(items) >= min_consistent_constraints and
        spatial_baseline >= min_spatial_baseline_m and
        inlier_ratio >= min_inlier_ratio and
        robust_residual <= max_robust_residual_m and
        projected <= max_projected_registration_error_m and
        condition < 1e4 and confidence >= min_candidate_margin and
        robust.status == ACCEPTED_HYPOTHESIS)
    reason = 'ACCEPTED_MULTI_CONSTRAINT' if accepted_final else (
        'INSUFFICIENT_CONSISTENT_CONSTRAINTS' if len(items) < min_consistent_constraints
        else 'PHYSICAL_ACCURACY_GATE_REJECTED')
    covariance = list(result.covariance)
    covariance[0] = covariance[7] = translation_uncertainty ** 2
    covariance[35] = yaw_uncertainty ** 2
    return RegistrationResult(
        accepted=accepted_final, transform=transform,
        covariance=tuple(covariance), inlier_ratio=inlier_ratio,
        residual_m=robust_residual,
        occupied_free_agreement=float(np.mean([
            item.occupied_free_agreement for item in items])),
        overlap_fraction=float(np.mean([
            item.overlap_fraction for item in items])), reason=reason,
        constraint_count=len(results), consistent_constraint_count=len(items),
        spatial_baseline_m=spatial_baseline,
        angular_spread_rad=angular_spread,
        median_residual_m=robust_residual,
        p95_residual_m=p95_residual,
        translation_uncertainty_m=translation_uncertainty,
        yaw_uncertainty_rad=yaw_uncertainty, condition_number=condition,
        projected_error_m=projected, final_confidence=confidence,
        consensus_diagnostics=forensic,
        selector_status=robust.status,
        selector_score=robust.score,
        selector_null_score=robust.null_score,
        selector_runner_up_score=robust.runner_up_score,
        selector_runner_up_margin=robust.runner_up_margin,
        selector_inlier_probabilities=robust.inlier_probabilities)


def descriptor_checksum(descriptor: bytes) -> int:
    """Return a deterministic bounded checksum for duplicate suppression."""
    checksum = 2166136261
    for byte in descriptor:
        checksum ^= int(byte)
        checksum = (checksum * 16777619) & 0xFFFFFFFF
    return checksum


def temporal_consistency(stamps_ns: Iterable[int], window_ns: int = 5_000_000_000) -> float:
    """Score repeated nearby keyframes without requiring synchronized clocks."""
    values = sorted(int(value) for value in stamps_ns)
    if not values:
        return 0.0
    if len(values) == 1:
        return 0.5
    span = values[-1] - values[0]
    return max(0.0, min(1.0, 1.0 - float(span) / float(window_ns)))


def confirmation_window_for_cadence(
        configured_window_ns: int,
        observed_intervals_ns: Iterable[int],
        cadence_factor: float = 2.5) -> int:
    """Keep temporal confirmation possible at the observed message cadence.

    The configured window remains the lower bound.  When map/keyframe message
    stamps arrive more slowly than that bound, allow two distinct observations
    plus normal timing jitter by scaling the robust median interval.  This is
    deliberately based on observed message-clock cadence, not on a reduced
    confirmation count or a relaxed descriptor/geometry gate.
    """
    configured = max(1, int(configured_window_ns))
    intervals = sorted(
        int(value) for value in observed_intervals_ns if int(value) > 0)
    if not intervals:
        return configured
    median = intervals[len(intervals) // 2]
    return max(configured, int(math.ceil(float(median) * float(cadence_factor))))


def temporal_support_count(
        anchor_pair: tuple[str, str], anchor_own_stamp_ns: int,
        anchor_peer_stamp_ns: int, anchor_sector_shift: int, observations,
        similarity_gate: float, margin_gate: float,
        known_fraction_gate: float, window_ns: int, sector_count: int = 24) -> int:
    """Count independent keyframe-pair observations for one cheap candidate.

    ``observations`` contains tuples of ``(pair, own_stamp_ns,
    peer_stamp_ns, similarity, margin, known_fraction, sector_shift)``.  A
    repeated evaluation of the same pair contributes once; only a distinct
    own/peer keyframe pair inside the message-clock window and with a nearby
    angular shift contributes additional evidence.
    """
    support = set()
    anchor_pair = tuple(anchor_pair)
    for (pair, own_stamp_ns, peer_stamp_ns, similarity, margin,
         known_fraction, sector_shift) in observations:
        pair = tuple(pair)
        if pair == anchor_pair:
            support.add(pair)
            continue
        if (similarity < similarity_gate or margin < margin_gate or
                known_fraction < known_fraction_gate):
            continue
        if (abs(int(own_stamp_ns) - int(anchor_own_stamp_ns)) > window_ns or
                abs(int(peer_stamp_ns) - int(anchor_peer_stamp_ns)) > window_ns):
            continue
        shift_delta = abs(int(sector_shift) - int(anchor_sector_shift))
        shift_delta = min(shift_delta, sector_count - shift_delta)
        if shift_delta <= 2:
            support.add(pair)
    return len(support)
