"""Focused evidence-based tests for teammate exclusion and start gating."""

import math

from my_epuck_project.live_map_sanitizer import sanitize_shared_map
from my_epuck_project.nav2_frontier_diagnostic import classify_start_cell
from my_epuck_project.teammate_scan_filter import (
    mask_teammate_returns,
    prepare_slam_scan,
    VERIFIED_SILHOUETTE_RADIUS_M,
)
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan


def scan_with(value):
    scan = LaserScan()
    scan.angle_min = 0.0
    scan.angle_increment = 0.1
    scan.range_min = 0.03
    scan.range_max = 12.0
    scan.ranges = [value]
    return scan


def grid():
    result = OccupancyGrid()
    result.info.resolution = 0.01
    result.info.width = 30
    result.info.height = 30
    result.info.origin.position.x = -0.15
    result.info.origin.position.y = -0.15
    result.info.origin.orientation.w = 1.0
    result.data = [100] * (result.info.width * result.info.height)
    return result


def footprint(x=0.0, y=0.0, age=0.0, radius=0.037):
    return {'robot_frame': 'robot1/base_footprint', 'x': x, 'y': y,
            'radius_m': radius, 'pose_age_s': age}


def test_actual_verified_robot_silhouette_is_fully_masked():
    scan = scan_with(1.0 - VERIFIED_SILHOUETTE_RADIUS_M)
    _, masked = mask_teammate_returns(
        scan, 1.0, 0.0, 0.035, 0.005,
        geometry_radius_m=VERIFIED_SILHOUETTE_RADIUS_M)
    assert masked == [0]


def test_wheel_or_protrusion_return_inside_verified_silhouette_is_masked():
    scan = scan_with(1.0 - 0.0255)
    _, masked = mask_teammate_returns(
        scan, 1.0, 0.0, 0.035, 0.005,
        geometry_radius_m=VERIFIED_SILHOUETTE_RADIUS_M)
    assert masked == [0]


def test_wall_behind_teammate_is_preserved():
    scan = scan_with(1.0 - VERIFIED_SILHOUETTE_RADIUS_M + 0.2)
    output, masked = mask_teammate_returns(
        scan, 1.0, 0.0, 0.035, 0.005,
        geometry_radius_m=VERIFIED_SILHOUETTE_RADIUS_M)
    assert masked == [] and math.isclose(output.ranges[0], scan.ranges[0])


def test_wall_beside_teammate_is_preserved():
    scan = LaserScan()
    scan.angle_min = 0.3
    scan.angle_increment = 0.1
    scan.range_min = 0.03
    scan.range_max = 12.0
    scan.ranges = [1.0]
    output, masked = mask_teammate_returns(
        scan, 1.0, 0.0, 0.035, 0.005,
        geometry_radius_m=VERIFIED_SILHOUETTE_RADIUS_M)
    assert masked == [] and output.ranges[0] == 1.0


def test_natural_positive_infinity_is_completion_not_masking():
    output, masked, stats = prepare_slam_scan(
        scan_with(math.inf), 1.0, 0.0, 0.035, 0.005, 11.98,
        geometry_radius_m=VERIFIED_SILHOUETTE_RADIUS_M)
    assert masked == [] and math.isclose(output.ranges[0], 11.98, abs_tol=1e-5)
    assert stats.raw_positive_infinity == 1 and stats.converted_free_cap == 1


def test_measured_residual_within_uncertainty_is_masked():
    output, masked = mask_teammate_returns(
        scan_with(1.0 - VERIFIED_SILHOUETTE_RADIUS_M + 0.004),
        1.0, 0.0, 0.035, 0.005,
        geometry_radius_m=VERIFIED_SILHOUETTE_RADIUS_M)
    assert masked == [0] and math.isnan(output.ranges[0])


def test_return_outside_justified_range_is_preserved():
    output, masked = mask_teammate_returns(
        scan_with(1.0 - VERIFIED_SILHOUETTE_RADIUS_M + 0.006),
        1.0, 0.0, 0.035, 0.005,
        geometry_radius_m=VERIFIED_SILHOUETTE_RADIUS_M)
    assert masked == [] and math.isclose(output.ranges[0], 0.980, abs_tol=1e-5)


def test_small_odometry_timestamp_error_is_within_bound():
    output, masked = mask_teammate_returns(
        scan_with(1.0 - VERIFIED_SILHOUETTE_RADIUS_M),
        1.0 - 0.002, 0.0, 0.035, 0.005,
        geometry_radius_m=VERIFIED_SILHOUETTE_RADIUS_M)
    assert masked == [0] and math.isnan(output.ranges[0])


def test_live_robot_footprint_is_removed_from_sanitized_map():
    sanitized, details = sanitize_shared_map(
        grid(), [footprint()], uncertainty_cells=0)
    assert details['cleared_cell_count'] > 0
    assert sanitized.data[15 * 30 + 15] == 0


def test_nearby_wall_outside_footprint_remains_occupied():
    source = grid()
    source.data[15 * 30 + 25] = 100
    sanitized, _ = sanitize_shared_map(
        source, [footprint()], uncertainty_cells=1)
    assert sanitized.data[15 * 30 + 25] == 100


def test_stale_robot_pose_causes_no_map_clearing():
    source = grid()
    sanitized, details = sanitize_shared_map(
        source, [footprint(age=0.51)], uncertainty_cells=1)
    assert sanitized.data == source.data
    assert details['stale_pose_count'] == 1


def test_two_live_footprints_are_both_removed():
    sanitized, _ = sanitize_shared_map(
        grid(), [footprint(-0.05), footprint(0.05)], uncertainty_cells=0)
    assert sanitized.data[15 * 30 + 10] == 0
    assert sanitized.data[15 * 30 + 20] == 0


def test_start_gate_classifies_free_and_inflated():
    source = grid()
    source.data[15 * 30 + 15] = 0
    assert classify_start_cell(source, 0.0, 0.0)[0] == 'FREE'
    source.data[15 * 30 + 15] = 50
    assert classify_start_cell(source, 0.0, 0.0)[0] == 'INFLATED'


def test_occupied_start_blocks_path_gate_categories():
    source = grid()
    source.data[15 * 30 + 15] = 99
    assert classify_start_cell(source, 0.0, 0.0)[0] == 'INSCRIBED'
    source.data[15 * 30 + 15] = 100
    assert classify_start_cell(source, 0.0, 0.0)[0] == 'LETHAL'


def test_start_becoming_free_resumes_planning_category():
    source = grid()
    source.data[15 * 30 + 15] = 100
    assert classify_start_cell(source, 0.0, 0.0)[0] == 'LETHAL'
    source.data[15 * 30 + 15] = 0
    assert classify_start_cell(source, 0.0, 0.0)[0] == 'FREE'
