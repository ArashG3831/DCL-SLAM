"""Tests for the dependency-free cooperative map PNG exporter."""

import json
import struct
from pathlib import Path

from my_epuck_project.cooperative_map_png_export import (
    DIFFERENCE_RGB,
    FREE_RGB,
    OCCUPIED_RGB,
    UNCERTAIN_RGB,
    UNKNOWN_RGB,
    render_map_with_paths,
    render_map_with_poses,
    difference_rgb,
    occupancy_rgb,
    selected_attempt,
    export_maps,
    write_rgb_png,
)
from my_epuck_project.occupancy_map_comparison import Geometry, OccupancyMap, save_map

import numpy as np


def test_occupancy_colors_orientation_and_scale():
    """Occupancy colors, ROS orientation, and integer scaling are preserved."""
    data = np.asarray([
        [-1, 0],
        [100, 50],
    ], dtype=np.int8)
    result = occupancy_rgb(data, scale=2)
    assert result.shape == (4, 4, 3)
    assert tuple(result[0, 0]) == OCCUPIED_RGB
    assert tuple(result[0, 2]) == UNCERTAIN_RGB
    assert tuple(result[2, 0]) == UNKNOWN_RGB
    assert tuple(result[2, 2]) == FREE_RGB


def test_difference_marks_only_changed_cells():
    """Only cells whose raw occupancy differs are highlighted."""
    first = np.asarray([[0, -1], [100, 0]], dtype=np.int8)
    second = np.asarray([[0, -1], [0, 0]], dtype=np.int8)
    result = difference_rgb(first, second, scale=1)
    assert tuple(result[0, 0]) == DIFFERENCE_RGB
    assert np.count_nonzero(np.all(result == DIFFERENCE_RGB, axis=2)) == 1


def test_dependency_free_png_has_expected_dimensions(tmp_path):
    """The writer emits a valid PNG header with the requested dimensions."""
    image = np.zeros((7, 11, 3), dtype=np.uint8)
    path = tmp_path / 'map.png'
    write_rgb_png(path, image)
    content = path.read_bytes()
    assert content.startswith(b'\x89PNG\r\n\x1a\n')
    width, height = struct.unpack('>II', content[16:24])
    assert (width, height) == (11, 7)
    assert not list(tmp_path.glob('*.tmp'))


def test_pose_overlay_adds_margin_and_draws_shared_map_pose():
    """Final pose markers stay inside a padded image instead of being clipped."""
    geometry = Geometry(
        width=10, height=8, resolution=0.1,
        origin_x=-0.5, origin_y=-0.4, yaw=0.0)
    image, records = render_map_with_poses(
        np.zeros((8, 10), dtype=np.int8), geometry, {
            'robot1': {'x_m': -0.5, 'y_m': -0.4, 'yaw_rad': 0.0},
            'robot2': {'x_m': 0.4, 'y_m': 0.3, 'yaw_rad': 1.57},
        }, scale=2, margin_cells=4)
    assert image.shape == (8 * 2 + 8 * 2, 10 * 2 + 8 * 2, 3)
    assert records['robot1']['in_map_bounds']
    assert records['robot2']['in_map_bounds']
    assert np.any(np.all(image == (220, 30, 30), axis=2))
    assert np.any(np.all(image == (25, 95, 220), axis=2))


def test_pose_overlay_respects_rotated_occupancy_origin():
    """Pose placement uses the OccupancyGrid origin yaw, not array indices."""
    geometry = Geometry(
        width=4, height=4, resolution=1.0,
        origin_x=10.0, origin_y=20.0, yaw=1.5707963267948966)
    image, records = render_map_with_poses(
        np.zeros((4, 4), dtype=np.int8), geometry, {
            'robot1': {'x_m': 10.0, 'y_m': 20.0, 'yaw_rad': 0.0},
        }, scale=1, margin_cells=2)
    assert image.shape == (8, 8, 3)
    assert records['robot1']['image_x_px'] == 2
    assert records['robot1']['image_y_px'] == 6


def test_path_overlay_preserves_rotated_map_transform_and_expands_canvas():
    """Trajectory overlays are not clipped when a recorded path exceeds map bounds."""
    geometry = Geometry(
        width=4, height=4, resolution=1.0,
        origin_x=10.0, origin_y=20.0, yaw=1.5707963267948966)
    image, poses, paths = render_map_with_paths(
        np.zeros((4, 4), dtype=np.int8), geometry, {
            'robot1': {'x_m': 10.0, 'y_m': 20.0, 'yaw_rad': 0.0},
        }, {
            'robot1': {
                'source': 'test shared-map timeseries',
                'segments': [[(10.0, 20.0), (10.0, 25.0), (8.0, 25.0)]],
                'point_count': 3,
            },
        }, scale=1, margin_cells=2)
    assert image.shape[0] > 8 or image.shape[1] > 8
    assert poses['robot1']['image_x_px'] == 2
    assert paths['robot1']['point_count'] == 3
    assert np.any(np.all(image == (220, 30, 30), axis=2))


def test_manifest_schema_example_is_strict_json(tmp_path):
    """Boolean and count manifest values remain strict JSON."""
    path = Path(tmp_path) / 'manifest.json'
    value = {'identical': True, 'different_cell_count': 0}
    path.write_text(json.dumps(value, allow_nan=False))
    assert json.loads(path.read_text()) == value


def test_interrupted_attempt_requires_explicit_opt_in(tmp_path):
    """An interrupted attempt is exportable only with explicit selection."""
    campaign = tmp_path / 'regression_interrupted'
    attempt = campaign / 'attempts' / 'trial_01_attempt_01'
    attempt.mkdir(parents=True)
    (campaign / 'campaign_progress.json').write_text(
        json.dumps({'valid_trials': {}}), encoding='utf-8')
    for name in ('robot1_final_shared_map.npz', 'robot2_final_shared_map.npz'):
        (attempt / name).write_bytes(b'placeholder')

    try:
        selected_attempt(campaign, 'trial_01')
    except ValueError as error:
        assert 'no valid selected trial' in str(error)
    else:
        raise AssertionError('interrupted attempt was selected implicitly')

    selected, trial = selected_attempt(
        campaign, 'trial_01', allow_incomplete=True)
    assert selected == attempt
    assert trial == 'trial_01'


def test_flat_fast_trial_layout_is_selectable_without_campaign_progress(tmp_path):
    """The report exporter supports observer-enabled fast-trial campaigns."""
    campaign = tmp_path / 'ideal_encoder_cooperative'
    maps = (campaign / 'fast_trial_20260818T173122Z' / 'observer'
            / 'fast_trial_20260818T173122Z' / 'forensic' / 'maps')
    maps.mkdir(parents=True)
    for name in ('robot1_shared_map_final.npz', 'robot2_shared_map_final.npz'):
        (maps / name).write_bytes(b'placeholder')

    selected, trial = selected_attempt(campaign)
    assert selected.name == 'fast_trial_20260818T173122Z'
    assert trial == 'fast_trial_20260818T173122Z'


def test_no_handoff_local_maps_export_with_explicit_status(tmp_path):
    """No-handoff runs export local maps and never invent a shared map."""
    campaign = tmp_path / 'unknown_pose_no_handoff'
    trial = campaign / 'fast_trial_20260827T000000Z' / 'forensic' / 'maps'
    trial.mkdir(parents=True)
    (campaign / 'fast_trial_20260827T000000Z' / 'forensic' /
     'transforms.csv').write_text('', encoding='utf-8')
    geometry = Geometry(4, 3, 0.1, 0.0, 0.0, 0.0)
    for robot, value in (('robot1', 0), ('robot2', 100)):
        save_map(trial / f'{robot}_map_final.npz', OccupancyMap(
            np.full((3, 4), value, dtype=np.int8), geometry,
            {'topic': f'/{robot}/map', 'robot_id': robot,
             'frame_id': f'{robot}/map', 'header_stamp': '1.000000000',
             'header_stamp_s': 1.0, 'map_load_time': '1.000000000',
             'received_ros_time_s': 1.0, 'received_wall_elapsed_s': 1.0,
             'dtype': 'int8', 'data_length': 12}))
    output = tmp_path / 'png'
    manifest = export_maps(campaign, output, scale=1, draw_poses=False,
                            draw_paths=False)
    assert manifest['handoff_occurred'] is False
    assert manifest['status_label'] == 'NO_HANDOFF — SHARED MAP UNAVAILABLE'
    assert (output / 'robot1_local_map.png').is_file()
    assert (output / 'robot2_local_map.png').is_file()
    assert (output / 'NO_HANDOFF_SHARED_MAP_UNAVAILABLE.txt').read_text(
        encoding='utf-8').strip() == manifest['status_label']
    assert not (output / 'robot1_robot2_exact_difference.png').exists()
