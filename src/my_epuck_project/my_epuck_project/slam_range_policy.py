"""Simulation-only LaserScan completion and installed Karto semantics."""

import math
from dataclasses import dataclass


SCAN_RANGE_MAX = 12.0
KARTO_TOLERANCE = 1e-6
MAP_RESOLUTION = 0.01
FREE_SPACE_CAP = SCAN_RANGE_MAX - max(
    2.0 * KARTO_TOLERANCE, 2.0 * MAP_RESOLUTION, 0.01
)


def validate_free_space_cap(cap, range_min, range_max=SCAN_RANGE_MAX):
    """Validate a cap that is safe for Karto free-only threshold rays."""
    cap = float(cap)
    if not math.isfinite(cap):
        raise ValueError('free_space_cap must be finite')
    if cap >= float(range_max):
        raise ValueError('free_space_cap must be below LaserScan.range_max')
    if cap <= float(range_min) + 2.0 * MAP_RESOLUTION:
        raise ValueError('free_space_cap is too close to LaserScan.range_min')
    return cap


@dataclass(frozen=True)
class ScanCompletionStats:
    """Bounded counts emitted for one completed scan."""

    raw_positive_infinity: int = 0
    converted_free_cap: int = 0
    nan: int = 0
    negative_infinity: int = 0
    invalid: int = 0
    finite_obstacle: int = 0
    exact_range_max: int = 0


def complete_natural_no_returns(ranges, cap, range_min, range_max,
                                enabled=True):
    """Convert only natural simulation no-returns, preserving bad readings."""
    cap = validate_free_space_cap(cap, range_min, range_max)
    output = list(ranges)
    stats = ScanCompletionStats()
    for index, value in enumerate(output):
        if math.isinf(value):
            if value > 0.0:
                stats = stats.__class__(
                    **{**stats.__dict__, 'raw_positive_infinity':
                       stats.raw_positive_infinity + 1})
                if enabled:
                    output[index] = cap
                    stats = stats.__class__(
                        **{**stats.__dict__, 'converted_free_cap':
                           stats.converted_free_cap + 1})
            else:
                stats = stats.__class__(
                    **{**stats.__dict__, 'negative_infinity':
                       stats.negative_infinity + 1})
        elif math.isnan(value):
            stats = stats.__class__(
                **{**stats.__dict__, 'nan': stats.nan + 1})
        elif value <= range_min or value < 0.0:
            stats = stats.__class__(
                **{**stats.__dict__, 'invalid': stats.invalid + 1})
        elif value == range_max:
            stats = stats.__class__(
                **{**stats.__dict__, 'exact_range_max':
                   stats.exact_range_max + 1})
        elif value < cap:
            stats = stats.__class__(
                **{**stats.__dict__, 'finite_obstacle':
                   stats.finite_obstacle + 1})
    return output, stats


@dataclass(frozen=True)
class KartoSemantics:
    """Retained Karto AddScan effects for one deterministic reading."""

    ignored: bool
    ray_traced: bool
    occupied_endpoint: bool
    bounds_expanded: bool


def installed_karto_semantics(value, range_min, range_max, threshold):
    """Deterministic model of the shipped Karto AddScan path."""
    if (not math.isfinite(value) or value <= range_min or
            value >= range_max):
        return KartoSemantics(True, False, False, False)
    return KartoSemantics(
        False, True, value < threshold - KARTO_TOLERANCE, True)
