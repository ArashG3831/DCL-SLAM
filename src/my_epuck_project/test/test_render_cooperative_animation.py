"""Focused regression tests for the offline publication renderer."""

from pathlib import Path
import sys

import numpy as np


TOOLS = Path(__file__).parents[1] / 'tools'
sys.path.insert(0, str(TOOLS))

import render_cooperative_animation  # noqa: E402
from render_cooperative_animation import (  # noqa: E402
    Candidate,
    FrontierRegion,
    FrontierRegionSnapshot,
    MapSnapshot,
    OverlayData,
    PoseSample,
    TransformSample,
    active_goals_at,
    compute_fixed_viewport,
    compute_map_rect,
    draw_frontier_overlays,
    draw_goal,
    draw_dashed,
    frontier_counts_at,
    frontier_snapshot_at,
    infer_cooperative_start_time,
    load_overlay_data,
    map_to_pixel,
    split_shared_pose_segments,
    timing_metadata,
    _traffic_labels,
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
                         -1.0, 0.1, 2.0, 0.2, (), (), (0.0, 0.0, 0.0), None)
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


def _frontier_region(stamp, robot, frontier_id, x=1.0, y=1.0):
    return FrontierRegion(stamp, robot, frontier_id, (x, y), ((0, 0),))


def test_frontier_overlay_draws_one_marker_per_current_region(monkeypatch):
    calls = []
    monkeypatch.setattr(
        'render_cooperative_animation.cv2.drawMarker',
        lambda *args, **kwargs: calls.append((args, kwargs)))
    viewport = (0.0, 10.0, 0.0, 10.0)
    rect = compute_map_rect(1920, 1080, viewport)
    canvas = np.zeros((1080, 1920, 3), dtype=np.uint8)
    regions = tuple(_frontier_region(1.0, 'robot2', str(index),
                                     1.0 + index, 2.0)
                    for index in range(73))
    marker_count = draw_frontier_overlays(
        canvas, (), 1.0, viewport, rect, True, False,
        frontier_regions=regions)
    assert marker_count == 73
    assert len(calls) == 73


def test_current_and_cumulative_frontier_counts_follow_snapshots():
    snapshots = {
        'robot2': (
            FrontierRegionSnapshot(
                1.0, 'robot2',
                (_frontier_region(1.0, 'robot2', 'a'),
                 _frontier_region(1.0, 'robot2', 'b')), 0.03,
                (0.0, 0.0, 0.0)),
            FrontierRegionSnapshot(
                2.0, 'robot2',
                (_frontier_region(2.0, 'robot2', 'a'),
                 _frontier_region(2.0, 'robot2', 'c')), 0.03,
                (0.0, 0.0, 0.0)),
        )
    }
    assert len(frontier_snapshot_at(snapshots, 'robot2', 1.5).regions) == 2
    latest = frontier_snapshot_at(snapshots, 'robot2', 2.0)
    assert {region.frontier_id for region in latest.regions} == {'a', 'c'}
    assert len({region.frontier_id for item in snapshots['robot2']
                if item.stamp_s <= 2.0 for region in item.regions}) == 3
    assert frontier_counts_at(snapshots, 'robot2', 2.0) == (2, 3)


def test_frontier_snapshot_timestamp_tolerance_handles_recorded_float_rounding():
    snapshots = {
        'robot2': (FrontierRegionSnapshot(
            224.98000000000002, 'robot2',
            tuple(_frontier_region(224.98, 'robot2', str(index))
                  for index in range(58)), 0.03, (0.0, 0.0, 0.0)),)
    }
    assert frontier_counts_at(snapshots, 'robot2', 224.98)[0] == 58


def test_frontier_loader_uses_recorded_centroid_and_cell_fallback(tmp_path):
    import json
    payload = {
        'capture_robot': 'robot1', 'capture_elapsed_s': 1.0,
        'resolution': 0.5, 'origin': [10.0, 20.0, 0.0],
        'regions': [
            {'physical_id': 'recorded', 'centroid': [11.0, 22.0],
             'cells': [[0, 0]]},
            {'physical_id': 'fallback', 'cells': [[2, 4]]},
        ],
    }
    (tmp_path / 'frontier_regions.jsonl').write_text(
        json.dumps(payload) + '\n', encoding='utf-8')
    data = load_overlay_data(tmp_path)
    regions = data.frontier_regions['robot1'][0].regions
    assert {region.frontier_id for region in regions} == {'recorded', 'fallback'}
    centroids = {region.frontier_id: region.centroid for region in regions}
    assert centroids['recorded'] == (11.0, 22.0)
    assert centroids['fallback'] == (11.25, 22.25)


def test_frontier_region_geometry_and_centroid_are_drawn(monkeypatch):
    fill_calls = []
    marker_calls = []
    blend_calls = []
    original_fill_poly = render_cooperative_animation.cv2.fillPoly
    original_draw_marker = render_cooperative_animation.cv2.drawMarker
    original_add_weighted = render_cooperative_animation.cv2.addWeighted

    def fill_poly(*args, **kwargs):
        fill_calls.append((args, kwargs))
        return original_fill_poly(*args, **kwargs)

    def draw_marker(*args, **kwargs):
        marker_calls.append((args, kwargs))
        return original_draw_marker(*args, **kwargs)

    def add_weighted(*args, **kwargs):
        blend_calls.append((args, kwargs))
        return original_add_weighted(*args, **kwargs)

    monkeypatch.setattr('render_cooperative_animation.cv2.fillPoly', fill_poly)
    monkeypatch.setattr('render_cooperative_animation.cv2.drawMarker', draw_marker)
    monkeypatch.setattr('render_cooperative_animation.cv2.addWeighted', add_weighted)
    viewport = (0.0, 10.0, 0.0, 10.0)
    rect = compute_map_rect(1920, 1080, viewport)
    canvas = np.zeros((1080, 1920, 3), dtype=np.uint8)
    region = FrontierRegion(
        1.0, 'robot1', 'f', (1.5, 1.5), ((1, 1), (2, 1)),
        (1.0, 1.0, 3.0, 1.5), (0.0, 0.0, 0.0), 1.0)
    marker_count = draw_frontier_overlays(
        canvas, (), 1.0, viewport, rect, True, False,
        frontier_regions=(region,))
    assert marker_count == 1
    assert len(fill_calls) == 1
    assert len(fill_calls[0][0][1]) == 2
    assert len(marker_calls) == 1
    assert len(blend_calls) == 1
    assert blend_calls[0][0][1] == 0.20
    assert blend_calls[0][0][3] == 0.80


def test_traffic_labels_show_waiting_robot_and_moving_peer():
    data = OverlayData(
        {'robot1': (), 'robot2': ()}, {'robot1': (), 'robot2': ()},
        {'robot1': (), 'robot2': ()}, {'robot1': (), 'robot2': ()}, 0,
        event_records=(
            {'elapsed_s': 10.0, 'event_type': 'TRAFFIC_PRIORITY_GRANTED',
             'robot_id': 'robot1'},
            {'elapsed_s': 10.1, 'event_type': 'TRAFFIC_WAITING',
             'robot_id': 'robot2'},
        ))
    labels = _traffic_labels(data, 11.0)
    assert labels['robot1'] == 'traffic moving | R2 waiting'
    assert labels['robot2'] == 'traffic waiting | R1 moving'


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


def test_allocator_warning_does_not_end_active_goal(tmp_path):
    events = [
        {'elapsed_s': 1.0, 'event_sequence': 1, 'robot_id': 'robot1',
         'event_type': 'NAV_GOAL_SENT', 'canonical_task_id': 'task',
         'physical_task_signature': 'sig'},
        {'elapsed_s': 2.0, 'event_sequence': 2, 'robot_id': 'robot1',
         'event_type': 'DISTRIBUTED_TASK_FAILURE', 'canonical_task_id': 'task',
         'physical_task_signature': 'sig'},
    ]
    (tmp_path / 'events.jsonl').write_text(
        ''.join(__import__('json').dumps(item) + '\n' for item in events),
        encoding='utf-8')
    data = load_overlay_data(tmp_path)
    assert 'robot1' in active_goals_at(data, 3.0)


def test_cooperative_phase_uses_release_time():
    events = [
        {'elapsed_s': 17.04, 'event_type': 'SHARED_TF_READY'},
        {'elapsed_s': 36.42, 'event_type': 'START_RELEASE'},
    ]
    assert infer_cooperative_start_time(events, 17.04) == 36.42
