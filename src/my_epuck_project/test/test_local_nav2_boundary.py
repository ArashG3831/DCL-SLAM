"""Tests for local-only Nav2 geometry and bounded path telemetry."""

import math

from my_epuck_project.distributed_assignment.local_nav2 import (
    LocalNav2,
    downsample_path,
    occupancy_value,
    path_length,
)

from nav_msgs.msg import OccupancyGrid


def grid_with_rotation(yaw):
    """Create a small occupancy grid with one distinctive cell."""
    grid = OccupancyGrid()
    grid.info.resolution = 1.0
    grid.info.width = 3
    grid.info.height = 3
    grid.info.origin.position.x = 1.0
    grid.info.origin.position.y = 2.0
    grid.info.origin.orientation.z = math.sin(yaw / 2.0)
    grid.info.origin.orientation.w = math.cos(yaw / 2.0)
    grid.data = [0] * 9
    grid.data[3] = 42
    return grid


def test_rotated_costmap_lookup_uses_origin_orientation():
    """Read cell (0, 1) after a 90-degree map rotation."""
    grid = grid_with_rotation(math.pi / 2.0)
    assert occupancy_value(grid, (-0.5, 2.5)) == 42


def test_outside_grid_is_not_confused_with_unknown_cell():
    """Return None for outside geometry and -1 for an in-grid unknown cell."""
    grid = grid_with_rotation(0.0)
    grid.data[0] = -1
    assert occupancy_value(grid, (1.5, 2.5)) == -1
    assert occupancy_value(grid, (-10.0, -10.0)) is None


def test_path_length_and_samples_are_measured_and_bounded():
    """Keep path endpoints while bounding the transmitted corridor."""
    points = tuple((float(index), 0.0) for index in range(100))
    samples = downsample_path(points, 8)
    assert len(samples) == 8
    assert samples[0] == points[0]
    assert samples[-1] == points[-1]
    assert path_length(points) == 99.0


def test_pending_navigation_goal_counts_as_active_before_handle_arrives():
    """Prevent a second dispatch during the NavigateToPose handle race."""
    nav = LocalNav2.__new__(LocalNav2)
    nav._navigation_goal_handle = None
    nav._navigation_send_pending = True
    assert nav.local_goal_active
