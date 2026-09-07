"""Focused regression tests for the offline publication renderer."""

from pathlib import Path
import sys

import numpy as np


TOOLS = Path(__file__).parents[1] / 'tools'
sys.path.insert(0, str(TOOLS))

from render_cooperative_animation import (  # noqa: E402
    Candidate,
    MapSnapshot,
    OverlayData,
    PoseSample,
    TransformSample,
    compute_fixed_viewport,
    compute_map_rect,
    draw_frontier_overlays,
    draw_goal,
    draw_dashed,
    map_to_pixel,
    split_shared_pose_segments,
    timing_metadata,
)


def snapshot(stamp, width, height, origin=(0.0, 0.0)):
    return MapSnapshot(
        stamp, Path(f'/tmp/map_{stamp}.npz'),
        np.full((height, width), -1, dtype=np.int8),
        origin[0], origin[1], 1.0,
    )


def identity_transform(stamp):
    return TransformSample(stamp, 0.0, 0.0, 0.0)


def test_metric_aspect_and_no_stretch():
    rect = compute_map_rect(1920, 1080, (0.0, 10.0, 0.0, 20.0))
    assert rect[2] / 10.0 == rect[3] / 20.0
    assert rect[4] == rect[2] / 10.0


def test_fixed_viewport_contains_growing_snapshots():
    first = [snapshot(0.0, 10, 20, (0.0, 0.0))]
    growing = first + [snapshot(10.0, 30, 40, (-5.0, -2.0))]
    extent = growing[-1].extent
    assert compute_fixed_viewport(first, extent) == compute_fixed_viewport(
        growing, extent)


def test_timing_1500_seconds_at_10x_60fps():
    timing = timing_metadata(0.0, 1500.0, 10.0, 60)
    assert timing['expected_output_duration_s'] == 150.0
    assert timing['expected_frame_count'] == 9000


def test_metric_scale_is_equal_in_both_axes():
    rect = compute_map_rect(1920, 1080, (-2.0, 8.0, -3.0, 17.0))
    assert abs(rect[4] - rect[2] / 10.0) < 1e-12
    assert abs(rect[4] - rect[3] / 20.0) < 1e-12


def test_candidate_bounds_and_goal_stay_inside_map_roi():
    viewport = (0.0, 10.0, 0.0, 20.0)
    rect = compute_map_rect(1920, 1080, viewport)
    canvas = np.zeros((1080, 1920, 3), dtype=np.uint8)
    candidate = Candidate(1.0, 'robot1', 'f', '', (5.0, 10.0),
                         (4.9, 9.9, 5.1, 10.1), (5.0, 10.0),
                         -1.0, 0.1, 2.0, 0.2, ())
    draw_frontier_overlays(canvas, (candidate,), 1.0, viewport, rect, True, True)
    draw_goal(canvas, (5.0, 10.0), 'robot1', viewport, rect)
    left, top, width, height, _ = rect
    assert np.any(canvas[top:top + height, left:left + width] != 0)
    assert not np.any(canvas[:top, 0:1920] != 0)
    assert not np.any(canvas[:, 1518:1920] != 0)


def test_overlay_fallback_is_recorded_as_candidate_geometry_only():
    data = OverlayData({'robot1': ((1.0, (),),), 'robot2': ()},
                       {'robot1': (), 'robot2': ()},
                       {'robot1': (), 'robot2': ()},
                       {'robot1': (), 'robot2': ()}, 0)
    assert data.raw_frontier_geometry_records == 0


def test_compact_labels_are_not_legacy_robot_names():
    source = (Path(__file__).parents[1] / 'tools' /
              'render_cooperative_animation.py').read_text()
    assert "'ROBOT1'" not in source
    assert "'ROBOT2'" not in source


def test_teleport_splits_trajectory():
    records = [
        PoseSample(0.0, 0.0, 0.0, 0.0, {}),
        PoseSample(1.0, 0.1, 0.0, 0.0, {}),
        PoseSample(2.0, 9.0, 9.0, 0.0, {}),
    ]
    segments = split_shared_pose_segments(
        records, [identity_transform(0.0), identity_transform(2.0)], 0.0)
    assert [len(segment) for segment in segments] == [2, 1]


def test_nonfinite_row_breaks_trajectory():
    records = [
        PoseSample(0.0, 0.0, 0.0, 0.0, {}),
        None,
        PoseSample(2.0, 0.2, 0.0, 0.0, {}),
    ]
    segments = split_shared_pose_segments(
        records, [identity_transform(0.0), identity_transform(2.0)], 0.0)
    assert [len(segment) for segment in segments] == [1, 1]


def test_handoff_does_not_join_pre_and_post_frames():
    records = [
        PoseSample(0.0, 100.0, 100.0, 0.0, {}),
        PoseSample(2.0, 0.2, 0.0, 0.0, {}),
    ]
    segments = split_shared_pose_segments(
        records, [identity_transform(2.0)], handoff_s=1.0)
    assert len(segments) == 1
    assert segments[0][0].x == 0.2


def test_dashed_path_carries_phase_across_vertices(monkeypatch):
    calls = []
    monkeypatch.setattr(
        'render_cooperative_animation.cv2.line',
        lambda _view, p1, p2, *_args: calls.append((p1, p2)))
    draw_dashed(np.zeros((20, 20, 3), dtype=np.uint8),
                [(0, 0), (5, 0), (10, 0), (15, 0)],
                (255, 255, 255), dash=8, gap=4)
    assert calls == [((0, 0), (5, 0)), ((5, 0), (8, 0)),
                     ((12, 0), (15, 0))]
