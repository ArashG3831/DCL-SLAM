"""Bounded robust SE(2) hypothesis selection for unknown-pose handoff.

This module is deliberately independent of ROS.  It implements the practical
part of the Indelman et al. formulation used by this project: every candidate
correspondence proposes a transform, candidate transforms are evaluated from
multiple deterministic seeds, inlier/outlier probabilities are refined by a
bounded EM-style loop, and a null/ambiguous result prevents premature frame
handoff.  It is not a full pose-graph EM implementation.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


ACCEPTED_HYPOTHESIS = 'ACCEPTED_HYPOTHESIS'
AMBIGUOUS_HYPOTHESES = 'AMBIGUOUS_HYPOTHESES'
INSUFFICIENT_EVIDENCE = 'INSUFFICIENT_EVIDENCE'
NULL_NO_TRUSTWORTHY_ALIGNMENT = 'NULL_NO_TRUSTWORTHY_ALIGNMENT'


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _inverse(transform: tuple[float, float, float]):
    tx, ty, yaw = (float(value) for value in transform)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return (-cosine * tx - sine * ty,
            sine * tx - cosine * ty,
            wrap_angle(-yaw))


def _compose(first, second):
    tx, ty, yaw = first
    sx, sy, syaw = second
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return (float(tx + cosine * sx - sine * sy),
            float(ty + sine * sx + cosine * sy),
            wrap_angle(yaw + syaw))


def se2_residual(transform, observation):
    """Return Log(T^-1 * observation) in the project planar convention."""
    return np.asarray(_compose(_inverse(transform), observation),
                      dtype=np.float64)


@dataclass(frozen=True)
class PoseConstraint:
    """One candidate transform and the evidence used to evaluate it."""

    transform: tuple[float, float, float]
    covariance: tuple[float, ...] = ()
    quality: float = 1.0
    evidence_id: str = ''
    source_center: tuple[float, float] | None = None
    target_center: tuple[float, float] | None = None
    source_timestamp_ns: int = 0
    target_timestamp_ns: int = 0


@dataclass(frozen=True)
class RobustPoseSelection:
    """Deterministic selector output, including bounded forensic evidence."""

    status: str
    transform: tuple[float, float, float]
    covariance: tuple[float, ...]
    selected_indices: tuple[int, ...]
    inlier_probabilities: tuple[float, ...]
    score: float
    null_score: float
    runner_up_score: float
    runner_up_margin: float
    diagnostics: tuple[dict, ...] = ()


def _safe_quality(value: float) -> float:
    return max(0.05, min(1.0, float(value)))


def _covariance_3x3(candidate: PoseConstraint) -> np.ndarray:
    """Extract x/y/yaw covariance with conservative uncertainty floors."""
    raw = np.asarray(candidate.covariance, dtype=np.float64)
    matrix = np.zeros((3, 3), dtype=np.float64)
    if raw.size >= 36:
        full = raw.reshape(6, 6)
        indices = (0, 1, 5)
        matrix = full[np.ix_(indices, indices)].copy()
    elif raw.size == 9:
        matrix = raw.reshape(3, 3).copy()
    matrix = 0.5 * (matrix + matrix.T)
    # Registration covariance can be numerically optimistic on a raster map.
    # Floors keep the likelihood well-conditioned while remaining much tighter
    # than the broad outlier model.
    floors = np.asarray([
        0.02 ** 2,
        0.02 ** 2,
        math.radians(0.25) ** 2,
    ])
    diagonal = np.diag(matrix).copy()
    diagonal[~np.isfinite(diagonal)] = 0.0
    diagonal = np.maximum(diagonal, floors)
    matrix[np.diag_indices(3)] = diagonal
    # Correlations are useful only when the supplied matrix is valid.  A small
    # PSD projection avoids a bad registration covariance poisoning selection.
    try:
        eigenvalues, eigenvectors = np.linalg.eigh(matrix)
        eigenvalues = np.maximum(eigenvalues, floors.min())
        matrix = eigenvectors @ np.diag(eigenvalues) @ eigenvectors.T
    except np.linalg.LinAlgError:
        matrix = np.diag(diagonal)
    return matrix


def _log_gaussian(residual: np.ndarray, covariance: np.ndarray) -> float:
    try:
        sign, logdet = np.linalg.slogdet(covariance)
        if sign <= 0.0 or not math.isfinite(float(logdet)):
            return -math.inf
        solved = np.linalg.solve(covariance, residual)
        return float(-0.5 * (residual @ solved + logdet +
                             3.0 * math.log(2.0 * math.pi)))
    except (np.linalg.LinAlgError, ValueError, FloatingPointError):
        return -math.inf


def _weighted_mean(constraints: list[PoseConstraint], weights: np.ndarray):
    values = np.asarray([item.transform for item in constraints],
                        dtype=np.float64)
    weights = np.maximum(np.asarray(weights, dtype=np.float64), 1e-9)
    weights /= float(weights.sum())
    yaw = math.atan2(float(np.sum(weights * np.sin(values[:, 2]))),
                     float(np.sum(weights * np.cos(values[:, 2]))))
    return (float(np.sum(weights * values[:, 0])),
            float(np.sum(weights * values[:, 1])),
            wrap_angle(yaw))


def _distance(first, second):
    delta = se2_residual(second, first)
    return float(np.linalg.norm(delta[:2])), abs(float(delta[2]))


def _spatial_baseline(constraints: list[PoseConstraint], indices):
    centers = [constraints[index].source_center for index in indices
               if constraints[index].source_center is not None]
    if len(centers) < 2:
        return 0.0
    points = np.asarray(centers, dtype=np.float64)
    return float(np.max(np.linalg.norm(points[:, None, :] - points[None, :, :],
                                       axis=2)))


def _timestamp_span(constraints: list[PoseConstraint], indices):
    stamps = []
    for index in indices:
        item = constraints[index]
        stamps.extend(value for value in (
            item.source_timestamp_ns, item.target_timestamp_ns) if value)
    if len(stamps) < 2:
        return 0.0
    return float(max(stamps) - min(stamps)) / 1.0e9


def _covariance_tuple(covariance: np.ndarray) -> tuple[float, ...]:
    result = np.zeros((6, 6), dtype=np.float64)
    indices = (0, 1, 5)
    result[np.ix_(indices, indices)] = covariance
    return tuple(float(value) for value in result.ravel())


def select_robust_hypothesis(
        candidates: Iterable[PoseConstraint],
        min_inliers: int = 3,
        min_spatial_baseline_m: float = 0.75,
        max_translation_disagreement_m: float = 0.15,
        max_yaw_disagreement_rad: float = math.radians(1.0),
        min_timestamp_span_s: float = 1.0,
        max_iterations: int = 20,
        min_runner_up_margin: float = 0.10,
        min_inlier_probability: float = 0.60,
        min_quality: float = 0.55) -> RobustPoseSelection:
    """Select a transform using bounded multi-hypothesis EM-style inference.

    The null model is explicit: if no hypothesis has enough compatible,
    high-probability inliers and a clear score margin, the function refuses to
    establish a common frame.  All ordering and tie-breaking is deterministic.
    """
    items = tuple(candidates)
    minimum = max(1, int(min_inliers))
    if not items:
        return RobustPoseSelection(
            NULL_NO_TRUSTWORTHY_ALIGNMENT, (0.0, 0.0, 0.0), (0.0,) * 36,
            (), (), 0.0, 0.0, -math.inf, 0.0, ())

    covariances = [_covariance_3x3(item) for item in items]
    # The outlier model is deliberately broad.  Its finite likelihood is the
    # null alternative for perceptually aliased or unrelated map crops.
    outlier_covariance = np.diag([
        2.5 ** 2, 2.5 ** 2, math.radians(45.0) ** 2])

    seeds = [tuple(float(value) for value in item.transform) for item in items]
    # Pair means provide deterministic seeds for small, separated clusters.
    for first in range(len(items)):
        for second in range(first + 1, len(items)):
            distance, yaw = _distance(items[first].transform,
                                      items[second].transform)
            if distance <= max_translation_disagreement_m * 3.0 and \
                    yaw <= max_yaw_disagreement_rad * 3.0:
                seeds.append(_weighted_mean(
                    [items[first], items[second]], [
                        _safe_quality(items[first].quality),
                        _safe_quality(items[second].quality)]))

    hypotheses = []
    for seed_index, seed in enumerate(seeds):
        transform = seed
        probabilities = np.full(len(items), 0.5, dtype=np.float64)
        for _ in range(max(1, int(max_iterations))):
            log_inlier = []
            log_outlier = []
            residuals = []
            for item, covariance in zip(items, covariances):
                residual = se2_residual(transform, item.transform)
                residuals.append(residual)
                log_inlier.append(_log_gaussian(residual, covariance) +
                                  math.log(0.55 * _safe_quality(item.quality)))
                log_outlier.append(_log_gaussian(residual, outlier_covariance) +
                                   math.log(0.45))
            log_inlier = np.asarray(log_inlier)
            log_outlier = np.asarray(log_outlier)
            logits = np.clip(log_inlier - log_outlier, -60.0, 60.0)
            updated_probabilities = 1.0 / (1.0 + np.exp(-logits))
            weights = updated_probabilities * np.asarray(
                [_safe_quality(item.quality) for item in items])
            if float(weights.sum()) <= 1e-9:
                break
            updated_transform = _weighted_mean(list(items), weights)
            delta = se2_residual(transform, updated_transform)
            transform = updated_transform
            probabilities = updated_probabilities
            if float(np.linalg.norm(delta[:2])) < 1e-5 and \
                    abs(float(delta[2])) < 1e-5:
                break

        high = tuple(index for index, value in enumerate(probabilities)
                     if float(value) >= float(min_inlier_probability) and
                     _safe_quality(items[index].quality) >= float(min_quality))
        pairwise = []
        for first, second in __import__('itertools').combinations(high, 2):
            pairwise.append(_distance(items[first].transform,
                                      items[second].transform))
        max_translation = max((value[0] for value in pairwise), default=0.0)
        max_yaw = max((value[1] for value in pairwise), default=0.0)
        weighted_mass = float(np.sum(probabilities))
        normalized_residual = 0.0
        for index, residual in enumerate(
                [se2_residual(transform, item.transform) for item in items]):
            try:
                normalized_residual += float(
                    probabilities[index] * residual @
                    np.linalg.solve(covariances[index], residual))
            except np.linalg.LinAlgError:
                normalized_residual += 1.0e6
        # Model score rewards inlier mass and penalizes normalized residual;
        # it is not compared to an unbounded raw registration cost.
        score = weighted_mass - 0.25 * normalized_residual
        hypotheses.append({
            'seed_index': seed_index,
            'transform': transform,
            'probabilities': probabilities,
            'high': high,
            'score': float(score),
            'max_translation_disagreement_m': float(max_translation),
            'max_yaw_disagreement_rad': float(max_yaw),
            'spatial_baseline_m': _spatial_baseline(list(items), high),
            'timestamp_span_s': _timestamp_span(list(items), high),
            'normalized_residual': float(normalized_residual),
        })

    # Merge nearby seed solutions so a single physical cluster cannot appear
    # as multiple competing models solely because it had several seeds.
    merged = []
    for hypothesis in sorted(hypotheses, key=lambda item: (
            -item['score'], item['transform'])):
        duplicate = None
        for existing in merged:
            distance, yaw = _distance(hypothesis['transform'],
                                      existing['transform'])
            if distance <= max_translation_disagreement_m and \
                    yaw <= max_yaw_disagreement_rad:
                duplicate = existing
                break
        if duplicate is None:
            merged.append(hypothesis)
        elif (hypothesis['score'], hypothesis['high']) > \
                (duplicate['score'], duplicate['high']):
            merged[merged.index(duplicate)] = hypothesis

    merged.sort(key=lambda item: (-item['score'], item['transform']))
    null_score = 0.0
    if not merged:
        return RobustPoseSelection(
            NULL_NO_TRUSTWORTHY_ALIGNMENT, (0.0, 0.0, 0.0), (0.0,) * 36,
            (), (), -math.inf, null_score, -math.inf, 0.0, ())

    winner = merged[0]
    runner_up_score = merged[1]['score'] if len(merged) > 1 else -math.inf
    margin = float(winner['score'] - max(null_score, runner_up_score))
    diagnostics = []
    for rank, hypothesis in enumerate(merged):
        diagnostics.append({
            'kind': 'robust_hypothesis',
            'rank': rank,
            'seed_index': int(hypothesis['seed_index']),
            'transform': [float(value) for value in hypothesis['transform']],
            'score': float(hypothesis['score']),
            'null_score': float(null_score),
            'runner_up_margin': float(margin if rank == 0 else 0.0),
            'selected_indices': list(hypothesis['high']),
            'inlier_probabilities': [float(value) for value in
                                     hypothesis['probabilities']],
            'max_translation_disagreement_m': float(
                hypothesis['max_translation_disagreement_m']),
            'max_yaw_disagreement_rad': float(
                hypothesis['max_yaw_disagreement_rad']),
            'spatial_baseline_m': float(hypothesis['spatial_baseline_m']),
            'timestamp_span_s': float(hypothesis['timestamp_span_s']),
            'normalized_residual': float(hypothesis['normalized_residual']),
        })

    valid_structure = (
        len(winner['high']) >= minimum and
        winner['max_translation_disagreement_m'] <=
        float(max_translation_disagreement_m) and
        winner['max_yaw_disagreement_rad'] <= float(max_yaw_disagreement_rad) and
        winner['spatial_baseline_m'] >= float(min_spatial_baseline_m) and
        (winner['timestamp_span_s'] <= 0.0 or
         winner['timestamp_span_s'] >= float(min_timestamp_span_s)) and
        margin >= float(min_runner_up_margin))
    if len(winner['high']) < minimum:
        status = INSUFFICIENT_EVIDENCE
    elif not valid_structure:
        status = AMBIGUOUS_HYPOTHESES if len(merged) > 1 else INSUFFICIENT_EVIDENCE
    elif winner['score'] <= null_score:
        status = NULL_NO_TRUSTWORTHY_ALIGNMENT
    else:
        status = ACCEPTED_HYPOTHESIS

    selected = tuple(int(index) for index in winner['high'])
    selected_covariances = [covariances[index] for index in selected]
    covariance = (np.mean(selected_covariances, axis=0)
                  if selected_covariances else np.zeros((3, 3)))
    return RobustPoseSelection(
        status=status,
        transform=tuple(float(value) for value in winner['transform']),
        covariance=_covariance_tuple(covariance),
        selected_indices=selected,
        inlier_probabilities=tuple(float(value) for value in
                                   winner['probabilities']),
        score=float(winner['score']), null_score=float(null_score),
        runner_up_score=float(runner_up_score),
        runner_up_margin=float(margin),
        diagnostics=tuple(diagnostics))
