"""Deterministic tests for the simulation SLAM range policy."""

import math

import pytest
from sensor_msgs.msg import LaserScan

from my_epuck_project.slam_range_policy import (
    FREE_SPACE_CAP,
    SCAN_RANGE_MAX,
    complete_natural_no_returns,
    installed_karto_semantics,
    validate_free_space_cap,
)
from my_epuck_project.teammate_scan_filter import filtered_scan


def test_derived_cap_has_explicit_margin():
    """The derived cap leaves the documented geometric safety margin."""
    assert FREE_SPACE_CAP == pytest.approx(11.98)
    assert FREE_SPACE_CAP < SCAN_RANGE_MAX
    assert SCAN_RANGE_MAX - FREE_SPACE_CAP > 0.019999


@pytest.mark.parametrize('value', [math.nan, -math.inf, 0.0, -1.0])
def test_malformed_no_returns_remain_ignored(value):
    """Malformed values are not converted into free space."""
    output, stats = complete_natural_no_returns(
        [value], FREE_SPACE_CAP, 0.05, SCAN_RANGE_MAX)
    assert math.isnan(output[0]) if math.isnan(value) else output[0] == value
    assert stats.converted_free_cap == 0


def test_positive_infinity_is_completed_only_for_slam():
    """Only the SLAM branch converts natural positive infinity."""
    slam, slam_stats = complete_natural_no_returns(
        [math.inf], FREE_SPACE_CAP, 0.05, SCAN_RANGE_MAX, True)
    raw, raw_stats = complete_natural_no_returns(
        [math.inf], FREE_SPACE_CAP, 0.05, SCAN_RANGE_MAX, False)
    assert slam == [FREE_SPACE_CAP]
    assert raw == [math.inf]
    assert slam_stats.converted_free_cap == 1
    assert raw_stats.raw_positive_infinity == 1


@pytest.mark.parametrize('value', [1.0, 8.0, FREE_SPACE_CAP - 1e-5])
def test_finite_returns_below_cap_are_occupied(value):
    """Finite obstacle returns below the threshold retain endpoints."""
    result = installed_karto_semantics(value, 0.05, 12.0, FREE_SPACE_CAP)
    assert result == result.__class__(False, True, True, True)


@pytest.mark.parametrize('value', [math.inf, -math.inf, math.nan, 12.0])
def test_absolute_invalid_readings_are_ignored(value):
    """Karto ignores nonfinite and absolute-range readings."""
    assert installed_karto_semantics(value, 0.05, 12.0, FREE_SPACE_CAP).ignored


@pytest.mark.parametrize('value', [FREE_SPACE_CAP, FREE_SPACE_CAP + 0.01,
                                   12.0 - 1e-9])
def test_threshold_and_late_finite_readings_are_free_only(value):
    """Threshold and late finite rays expand bounds without endpoints."""
    result = installed_karto_semantics(value, 0.05, 12.0, FREE_SPACE_CAP)
    assert result.ray_traced and result.bounds_expanded
    assert not result.occupied_endpoint


def test_cap_validation_rejects_unsafe_values():
    """Unsafe caps are rejected before a node can publish."""
    with pytest.raises(ValueError):
        validate_free_space_cap(12.0, 0.05, 12.0)
    with pytest.raises(ValueError):
        validate_free_space_cap(0.06, 0.05, 12.0)


def test_completed_natural_no_return_still_masks_teammate_before_cap():
    """A teammate before the cap masks a converted natural no-return."""
    scan = LaserScan()
    scan.angle_min = 0.0
    scan.angle_increment = 0.0
    scan.range_min = 0.05
    scan.range_max = 12.0
    scan.ranges = [FREE_SPACE_CAP]
    output, indices, _ = filtered_scan(
        scan, 1.0, 0.0, 0.1, 0.012, [(1.0, 0.0)], {0})
    assert indices == [0]
    assert math.isnan(output.ranges[0])
