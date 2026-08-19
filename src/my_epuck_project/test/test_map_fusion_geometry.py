"""Deterministic geometry checks for source-aware occupancy-grid fusion."""

import math
from types import SimpleNamespace

from my_epuck_project.source_aware_map_fusion import SourceAwareMapFusion
from my_epuck_project.source_aware_map_fusion import (
    _changed_update_bounds,
    _same_grid_content,
    _scalar_fused_data,
    _vectorized_fused_data,
)


def quaternion(yaw):
    return SimpleNamespace(
        x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def grid(width, height, resolution, x, y, yaw):
    return SimpleNamespace(info=SimpleNamespace(
        width=width,
        height=height,
        resolution=resolution,
        origin=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=0.0),
            orientation=quaternion(yaw),
        ),
    ))


def test_rotated_negative_origin_and_dynamic_bounds_are_preserved():
    fusion = object.__new__(SourceAwareMapFusion)
    message = grid(4, 2, 0.03, -1.2, -0.7, math.pi / 2.0)
    transform = (0.4, -2.0, -math.pi / 2.0)
    corners = fusion.corners(message, transform)
    assert len(corners) == 4
    assert all(math.isfinite(value) for point in corners for value in point)
    expected = [
        fusion.point_in_output(message, transform, x, y)
        for x, y in ((0.0, 0.0), (0.12, 0.0), (0.0, 0.06), (0.12, 0.06))
    ]
    assert corners == expected
    assert min(point[0] for point in corners) < 0.0
    assert min(point[1] for point in corners) < 0.0


def test_profile_resolution_produces_metadata_sized_dynamic_grid():
    resolution = 0.03
    corners = [(-0.04, -0.07), (0.11, -0.07), (-0.04, 0.08), (0.11, 0.08)]
    minimum_x = math.floor(min(x for x, _ in corners) / resolution) * resolution
    minimum_y = math.floor(min(y for _, y in corners) / resolution) * resolution
    maximum_x = math.ceil(max(x for x, _ in corners) / resolution) * resolution
    maximum_y = math.ceil(max(y for _, y in corners) / resolution) * resolution
    width = int(round((maximum_x - minimum_x) / resolution))
    height = int(round((maximum_y - minimum_y) / resolution))
    assert (minimum_x, minimum_y) == (-0.06, -0.09)
    assert (width, height) == (6, 6)
    assert width * height == 36


def occupancy_grid(width, height, resolution, x, y, yaw, data):
    message = grid(width, height, resolution, x, y, yaw)
    message.header = SimpleNamespace(frame_id='robot/map')
    message.data = list(data)
    message.header.stamp = SimpleNamespace(sec=1, nanosec=0)
    return message


def test_vectorized_fusion_matches_scalar_reference():
    messages = [
        occupancy_grid(
            4, 3, 0.05, -0.2, 0.1, 0.1,
            [-1, 0, 100, -1, 50, 0, -1, 80, 0, -1, -1, 20]),
        occupancy_grid(
            3, 4, 0.05, 0.15, -0.1, -0.2,
            [0, 70, -1, 100, -1, 0, 30, -1, 90, 0, -1, -1]),
    ]
    transforms = [(0.2, -0.3, 0.4), (-0.1, 0.2, -0.5)]
    scalar = _scalar_fused_data(
        messages, transforms, -0.5, -0.5, 30, 30, 0.05)
    vector = _vectorized_fused_data(
        messages, transforms, -0.5, -0.5, 30, 30, 0.05, {})
    assert vector.tolist() == scalar.tolist()


def test_vectorized_fusion_matches_scalar_for_randomized_geometries():
    """Compare vectorized fusion with scalar fusion across varied transforms."""
    import random

    randomizer = random.Random(17)
    for _ in range(8):
        messages = []
        transforms = []
        for _ in range(2):
            width = randomizer.randint(2, 7)
            height = randomizer.randint(2, 7)
            data = [randomizer.choice((-1, 0, 35, 70, 100))
                    for _ in range(width * height)]
            messages.append(occupancy_grid(
                width, height, randomizer.choice((0.03, 0.05, 0.1)),
                randomizer.uniform(-0.4, 0.4),
                randomizer.uniform(-0.4, 0.4),
                randomizer.uniform(-math.pi, math.pi), data))
            transforms.append((
                randomizer.uniform(-0.4, 0.4),
                randomizer.uniform(-0.4, 0.4),
                randomizer.uniform(-math.pi, math.pi),
            ))
        scalar = _scalar_fused_data(
            messages, transforms, -1.0, -1.0, 40, 40, 0.05)
        vector = _vectorized_fused_data(
            messages, transforms, -1.0, -1.0, 40, 40, 0.05, {})
        assert vector.tolist() == scalar.tolist()


def test_identical_map_content_ignores_timestamp_changes():
    first = occupancy_grid(2, 2, 0.1, 0.0, 0.0, 0.0, [0, 100, -1, 50])
    second = occupancy_grid(2, 2, 0.1, 0.0, 0.0, 0.0, [0, 100, -1, 50])
    second.header.stamp = SimpleNamespace(sec=99, nanosec=2)
    assert _same_grid_content(first, second)


def test_changed_map_content_is_detected_without_list_comparison():
    first = occupancy_grid(2, 2, 0.1, 0.0, 0.0, 0.0, [0, 100, -1, 50])
    second = occupancy_grid(2, 2, 0.1, 0.0, 0.0, 0.0, [0, 80, -1, 50])
    assert not _same_grid_content(first, second)


def test_visualization_update_bounds_are_minimal_and_empty_changes_are_skipped():
    previous = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
    current = [[0, 0, 0, 0], [0, 0, 40, 0], [0, 0, 0, 80]]
    assert _changed_update_bounds(current, current) is None
    assert _changed_update_bounds(current, previous) == (2, 1, 2, 2)


def test_visualization_update_bounds_reject_geometry_shape_change():
    assert _changed_update_bounds([[0, 0]], [[0, 0, 0]]) is None
