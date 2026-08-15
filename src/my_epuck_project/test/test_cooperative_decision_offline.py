"""Focused tests for the offline cooperative decision geometry helpers."""

import importlib.util
from pathlib import Path

import numpy as np


TOOL = Path(__file__).parents[1] / "tools" / "analyze_cooperative_decision_offline.py"
SPEC = importlib.util.spec_from_file_location("decision_offline", TOOL)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_se2_compose_and_inverse_round_trip():
    transform = (1.2, -0.7, 0.4)
    assert np.allclose(MODULE.compose(transform, MODULE.inverse(transform)), (0, 0, 0), atol=1e-12)


def test_cell_center_respects_rotated_origin():
    point = MODULE.apply(np.array([[0.5, 0.5]]), (2.0, 3.0, np.pi / 2.0))
    assert np.allclose(point[0], (1.5, 3.5), atol=1e-12)


def test_large_world_reference_contains_arena_and_boxes():
    segments, metadata = MODULE.static_segments(
        Path(__file__).parents[1] / "worlds" / "epuck_d500_two_world_large.wbt")
    assert metadata["dimensions"] == (40.0, 10.0)
    assert len(metadata["obstacles"]) == 23
    assert len(segments) == 4 + 4 * 23


def test_boundary_sampling_is_on_reference_segments():
    segments, _ = MODULE.static_segments(
        Path(__file__).parents[1] / "worlds" / "epuck_d500_two_world_large.wbt")
    points = MODULE.boundary_samples(segments, spacing=0.5)
    assert len(points) > 100
    assert np.all(np.isfinite(points))
