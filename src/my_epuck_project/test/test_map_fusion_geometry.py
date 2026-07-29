"""Deterministic geometry checks for source-aware occupancy-grid fusion."""

import math
from types import SimpleNamespace

from my_epuck_project.source_aware_map_fusion import SourceAwareMapFusion


def quaternion(yaw):
    return SimpleNamespace(
        x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def grid(width, height, resolution, x, y, yaw):
    return SimpleNamespace(info=SimpleNamespace(
        width=width,
        height=height,
        resolution=resolution,
        origin=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y),
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
