"""Bounded, map-native front-end primitives for unknown relative pose discovery.

The runtime node deliberately keeps this module free of ROS entities.  That
makes descriptor rejection and registration deterministic and allows the
same code to be exercised without Webots or a ground-truth transform.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np

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


@dataclass(frozen=True)
class GridCrop:
    """A bounded occupancy crop expressed in its source map frame."""

    values: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float


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
        size_m: float = 8.0) -> GridCrop:
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
    center_col = int(round((center_x - origin_x) / resolution))
    center_row = int(round((center_y - origin_y) / resolution))
    half = side // 2
    col0 = max(0, min(array.shape[1] - side, center_col - half))
    row0 = max(0, min(array.shape[0] - side, center_row - half))
    cropped = array[row0:row0 + side, col0:col0 + side].copy()
    return GridCrop(
        values=cropped,
        resolution=float(resolution),
        origin_x=float(origin_x + col0 * resolution),
        origin_y=float(origin_y + row0 * resolution),
    )


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
        occupied_count, known_count, out=np.zeros_like(total), where=known_count > 0.0)
    encoded = np.empty((rings, sectors, 2), dtype=np.uint8)
    encoded[:, :, 0] = np.rint(occupied_fraction * 255.0).astype(np.uint8)
    encoded[:, :, 1] = np.rint(known_fraction * 255.0).astype(np.uint8)
    return encoded.tobytes()


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
    a = np.frombuffer(first, dtype=np.uint8).reshape(rings, sectors, 2)
    b = np.frombuffer(second, dtype=np.uint8).reshape(rings, sectors, 2)
    scores = []
    known_scores = []
    for shift in range(sectors):
        shifted = np.roll(b, shift, axis=1)
        known = (a[:, :, 1].astype(np.float32) / 255.0 +
                 shifted[:, :, 1].astype(np.float32) / 255.0) / 2.0
        weight = np.where(known >= minimum_known_fraction, known, 0.0)
        denominator = float(weight.sum())
        if denominator <= 1e-6:
            scores.append(0.0)
            known_scores.append(0.0)
            continue
        diff = np.abs(a.astype(np.float32) - shifted.astype(np.float32)) / 255.0
        score = 1.0 - float((diff * weight[:, :, None]).sum() /
                             (denominator * 2.0))
        scores.append(max(0.0, min(1.0, score)))
        known_scores.append(float(weight.mean()))
    order = np.argsort(scores)[::-1]
    best = int(order[0])
    second = float(scores[order[1]]) if len(order) > 1 else 0.0
    return DescriptorMatch(
        similarity=float(scores[best]),
        margin=float(scores[best] - second),
        sector_shift=best,
        known_fraction=float(known_scores[best]),
    )


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
    return np.column_stack((
        crop.origin_x + (cols + 0.5) * crop.resolution,
        crop.origin_y + (rows + 0.5) * crop.resolution,
    )).astype(np.float64)


def _apply(points: np.ndarray, transform: tuple[float, float, float]) -> np.ndarray:
    tx, ty, yaw = transform
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    return points @ rotation.T + np.array([tx, ty])


def _nearest(points: np.ndarray, target: np.ndarray):
    if cKDTree is not None:
        return cKDTree(target).query(points, k=1)
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


def register_crops(
        source: GridCrop,
        target: GridCrop,
        max_iterations: int = 25,
        max_correspondence_m: float = 0.30) -> RegistrationResult:
    """Register occupied geometry with coarse yaw search and robust ICP."""
    source_points = _points(source, occupied=True)
    target_points = _points(target, occupied=True)
    if len(source_points) < 12 or len(target_points) < 12:
        return RegistrationResult(
            False, (0.0, 0.0, 0.0), (0.0,) * 36, 0.0, math.inf, 0.0, 0.0,
            'INSUFFICIENT_OCCUPIED_GEOMETRY')
    if len(source_points) > 1200:
        source_points = source_points[::max(1, len(source_points) // 1200)]
    if len(target_points) > 1200:
        target_points = target_points[::max(1, len(target_points) // 1200)]
    source_center = source_points.mean(axis=0)
    target_center = target_points.mean(axis=0)
    best = None
    for yaw in np.linspace(-math.pi, math.pi, 24, endpoint=False):
        rotated = _apply(source_points, (0.0, 0.0, float(yaw)))
        initial = (
            float(target_center[0] - rotated.mean(axis=0)[0]),
            float(target_center[1] - rotated.mean(axis=0)[1]),
            float(yaw),
        )
        transformed = _apply(source_points, initial)
        distances, _ = _nearest(transformed, target_points)
        inliers = distances[distances <= max_correspondence_m]
        if len(inliers) < 4:
            continue
        score = float(np.median(inliers))
        if best is None or score < best[0]:
            best = (score, initial)
    if best is None:
        return RegistrationResult(
            False, (0.0, 0.0, 0.0), (0.0,) * 36, 0.0, math.inf, 0.0, 0.0,
            'NO_COARSE_ALIGNMENT')
    transform = best[1]
    for _ in range(max_iterations):
        transformed = _apply(source_points, transform)
        distances, indices = _nearest(transformed, target_points)
        threshold = min(max_correspondence_m,
                        max(3.0 * source.resolution,
                            float(np.median(distances)) * 2.5))
        mask = distances <= threshold
        if np.count_nonzero(mask) < 8:
            break
        refined = _rigid_fit(source_points[mask], target_points[indices[mask]])
        if (np.linalg.norm(np.array(refined[:2]) - np.array(transform[:2])) < 1e-4
                and abs(math.atan2(math.sin(refined[2] - transform[2]),
                                   math.cos(refined[2] - transform[2]))) < 1e-4):
            transform = refined
            break
        transform = refined
    transformed = _apply(source_points, transform)
    distances, indices = _nearest(transformed, target_points)
    threshold = min(max_correspondence_m,
                    max(3.0 * source.resolution, float(np.median(distances)) * 2.5))
    mask = distances <= threshold
    inlier_ratio = float(np.count_nonzero(mask)) / float(len(source_points))
    residual = float(np.sqrt(np.mean(np.square(distances[mask])))) if np.any(mask) else math.inf
    target_free = _points(target, occupied=False)
    if len(target_free) and len(transformed):
        free_distances, _ = _nearest(transformed, target_free)
        free_consistency = float(np.mean(free_distances > threshold))
    else:
        free_consistency = 0.0
    occupied_agreement = max(0.0, min(1.0, 0.75 * inlier_ratio + 0.25 * free_consistency))
    target_extent = np.array([
        target.origin_x, target.origin_y,
        target.origin_x + target.values.shape[1] * target.resolution,
        target.origin_y + target.values.shape[0] * target.resolution,
    ])
    in_target = (
        (transformed[:, 0] >= target_extent[0]) &
        (transformed[:, 0] <= target_extent[2]) &
        (transformed[:, 1] >= target_extent[1]) &
        (transformed[:, 1] <= target_extent[3]))
    overlap = float(np.count_nonzero(in_target)) / float(len(transformed))
    variance = residual * residual / max(1, int(np.count_nonzero(mask)))
    covariance = np.zeros((6, 6), dtype=np.float64)
    covariance[0, 0] = covariance[1, 1] = variance
    covariance[5, 5] = variance / max(1e-6, float(np.mean(np.square(
        source_points[mask] - source_points[mask].mean(axis=0)))))
    accepted = (
        inlier_ratio >= 0.35 and residual <= max(0.12, 3.0 * source.resolution)
        and occupied_agreement >= 0.55 and overlap >= 0.15)
    return RegistrationResult(
        accepted=accepted,
        transform=transform,
        covariance=tuple(float(value) for value in covariance.ravel()),
        inlier_ratio=inlier_ratio,
        residual_m=residual,
        occupied_free_agreement=occupied_agreement,
        overlap_fraction=overlap,
        reason='ACCEPTED' if accepted else 'GEOMETRIC_VERIFICATION_REJECTED',
    )


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
