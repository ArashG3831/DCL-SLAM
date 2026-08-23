"""Tests for local-only Nav2 geometry and bounded path telemetry."""

import math

from my_epuck_project.distributed_assignment.local_nav2 import (
    DispatchPreconditions,
    LocalNav2,
    PathEvaluation,
    classify_dispatch_precondition_failure,
    classify_follow_path_controller_error,
    follow_path_controller_error_name,
    downsample_path,
    local_path_clear,
    occupancy_value,
    upstream_point_validation,
    path_length,
)
from my_epuck_project.distributed_assignment.models import FailureClass

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


def test_upstream_point_validation_preserves_threshold_and_unknown_semantics():
    """Mirror v1.6.0 point helper: strict >50, unknown separate."""
    grid = grid_with_rotation(0.0)
    grid.data[0] = 50
    grid.data[1] = 51
    grid.data[2] = -1
    assert upstream_point_validation(grid, (1.5, 2.5))['status'] == 'FREE_OR_ACCEPTED'
    assert upstream_point_validation(grid, (2.5, 2.5))['status'] == 'BLOCKED'
    assert upstream_point_validation(grid, (3.5, 2.5))['status'] == 'UNKNOWN'
    assert upstream_point_validation(grid, (-10.0, -10.0))['status'] == 'OUT_OF_BOUNDS'


def test_upstream_point_validation_missing_grid_is_not_blocked():
    """Absent dispatch-time evidence is explicit rather than a rejection."""
    assert upstream_point_validation(None, (0.0, 0.0))['status'] == 'NO_DATA'


def test_local_path_clear_rejects_inflated_or_unknown_cells():
    """A globally valid path must not enter the rolling obstacle corridor."""
    grid = OccupancyGrid()
    grid.info.resolution = 1.0
    grid.info.width = 3
    grid.info.height = 3
    grid.data = [0] * 9
    grid.data[4] = 80
    assert not local_path_clear(
        grid, ((0.5, 0.5), (1.5, 1.5)), lambda point: point,
    )
    grid.data[4] = -1
    assert not local_path_clear(
        grid, ((0.5, 0.5), (1.5, 1.5)), lambda point: point,
    )


def test_local_path_clear_ignores_samples_outside_rolling_window():
    """Only the locally observable path segment is a dispatch safety gate."""
    grid = OccupancyGrid()
    grid.info.resolution = 1.0
    grid.info.width = 2
    grid.info.height = 2
    grid.data = [0, 0, 0, 0]
    assert local_path_clear(
        grid, ((0.5, 0.5), (10.0, 10.0)), lambda point: point,
    )


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


def test_bid_path_reuse_requires_unchanged_map_and_costmap_stamps():
    """A cached bid is rejected after either planning input advances."""
    nav = LocalNav2.__new__(LocalNav2)
    nav._map = OccupancyGrid()
    nav._costmap = OccupancyGrid()
    nav._map.header.stamp.sec = 10
    nav._costmap.header.stamp.sec = 20
    evaluation = PathEvaluation(
        True, 1.0, ((0.0, 0.0), (1.0, 0.0)), 0, 0, '',
        failure_class=FailureClass.UNKNOWN,
        map_stamp_ns=10_000_000_000,
        costmap_stamp_ns=20_000_000_000,
    )
    assert nav.path_context_matches(evaluation)
    nav._costmap.header.stamp.sec = 21
    assert not nav.path_context_matches(evaluation)


def test_propagated_follow_path_controller_abort_is_hard_failure_evidence():
    """Nav2 child-controller code 104 must suppress repeated task retries."""
    assert classify_follow_path_controller_error(104) == FailureClass.CONTROLLER_NO_PROGRESS
    assert classify_follow_path_controller_error(999) == FailureClass.UNKNOWN
    assert follow_path_controller_error_name(104) == 'PATIENCE_EXCEEDED'
    assert follow_path_controller_error_name(107) == 'CONTROLLER_TIMED_OUT'
    assert follow_path_controller_error_name(999) == ''


def test_propagated_follow_path_tf_abort_is_infrastructure_evidence():
    """Keep FollowPath TF_ERROR separate from structural task failure."""
    assert classify_follow_path_controller_error(102) == FailureClass.TF_OR_LIFECYCLE
    assert follow_path_controller_error_name(102) == 'TF_ERROR'


def _dispatch_checks(**overrides):
    values = dict(
        action_server_ready=True,
        lifecycle_active=False,
        transform_available=True,
        transform_age_s=0.1,
        goal_inside_map=True,
        goal_inside_costmap=True,
        goal_map_value=0,
        goal_costmap_value=0,
        local_path_clear=True,
        no_local_goal_active=True,
        final_path_valid=True,
        reason='',
    )
    values.update(overrides)
    return DispatchPreconditions(**values)


def test_unknown_or_lethal_goal_is_hard_failure_even_before_lifecycle_query():
    """Geometry must not be misclassified by the default lifecycle flag."""
    checks = _dispatch_checks(
        goal_map_value=-1,
        goal_costmap_value=-1,
        reason='goal costmap cell is unknown or lethal',
    )
    assert classify_dispatch_precondition_failure(checks) == FailureClass.HARD_UNREACHABLE


def test_local_path_obstacle_is_hard_failure_before_lifecycle_query():
    """Do not dispatch a path whose local corridor is already blocked."""
    checks = _dispatch_checks(
        local_path_clear=False,
        reason='final path enters unknown or inflated local costmap cell',
    )
    assert classify_dispatch_precondition_failure(checks) == FailureClass.HARD_UNREACHABLE


def test_lifecycle_or_action_unavailability_remains_infrastructure_failure():
    """A geometrically valid task still reports infrastructure evidence."""
    checks = _dispatch_checks(
        reason='lifecycle services unavailable: local_controller_server',
    )
    assert classify_dispatch_precondition_failure(checks) == FailureClass.TF_OR_LIFECYCLE


def test_active_goal_race_is_action_rejection_after_valid_geometry():
    """Do not suppress a task merely because a local goal is still active."""
    checks = _dispatch_checks(
        lifecycle_active=True,
        no_local_goal_active=False,
        reason='another local navigation goal is active',
    )
    assert classify_dispatch_precondition_failure(checks) == FailureClass.ACTION_REJECTION


def test_bounded_failure_crop_and_geometry_signature_are_deterministic():
    """Failure snapshots carry bounded local geometry, not an unbounded map."""
    grid = OccupancyGrid()
    grid.info.resolution = 0.1
    grid.info.width = 100
    grid.info.height = 100
    grid.data = [0] * (100 * 100)
    from my_epuck_project.distributed_assignment.local_nav2 import (
        bounded_grid_crop, execution_geometry_signature,
    )
    crop = bounded_grid_crop(grid, (5.0, 5.0), radius_cells=30)
    assert crop['width'] == 41
    assert len(crop['values']) == 41 * 41
    assert len(execution_geometry_signature((5.0, 5.0), crop)) == 20
