"""Focused full-map unknown-pose path checks."""

import math
from types import SimpleNamespace

import numpy as np

from my_epuck_project.unknown_pose_frontend import UnknownPoseFrontend
from my_epuck_project.unknown_pose_frontend_core import (
    GridCrop, refine_registration_locally)


def _occupancy(values, resolution=0.03, origin=(1.2, -0.7), yaw=0.35):
    q = SimpleNamespace(x=0.0, y=0.0, z=np.sin(yaw / 2.0),
                        w=np.cos(yaw / 2.0))
    info = SimpleNamespace(
        width=len(values[0]), height=len(values), resolution=resolution,
        origin=SimpleNamespace(
            position=SimpleNamespace(x=origin[0], y=origin[1], z=0.0),
            orientation=q))
    return SimpleNamespace(header=SimpleNamespace(frame_id='robot1/map'),
                           info=info, data=list(np.asarray(values).ravel()))


def test_full_map_conversion_preserves_native_geometry_and_unknowns():
    values = np.asarray([[-1, 0, 100], [25, 50, -1]], dtype=np.int16)
    crop = UnknownPoseFrontend._full_map_crop(_occupancy(values))
    assert isinstance(crop, GridCrop)
    assert crop.values.shape == (2, 3)
    np.testing.assert_array_equal(crop.values, values)
    assert crop.resolution == 0.03
    assert crop.origin_x == 1.2
    assert crop.origin_y == -0.7
    assert abs(crop.origin_yaw - 0.35) < 1e-12
    assert not crop.values.flags.writeable


def test_full_map_input_is_not_a_local_window():
    values = np.arange(30, dtype=np.int16).reshape(5, 6)
    crop = UnknownPoseFrontend._full_map_crop(_occupancy(values))
    assert crop.values.shape == values.shape
    np.testing.assert_array_equal(crop.values, values)


def test_full_map_tick_bypasses_historical_pair_scheduler():
    source = open(
        'src/my_epuck_project/my_epuck_project/unknown_pose_frontend.py',
        encoding='utf-8').read()
    full_branch = source.index('if self.full_map_registration:',
                               source.index('def tick'))
    branch_end = source.index('self._maybe_schedule_full_map_registration()',
                              full_branch)
    branch = source[full_branch:branch_end]
    assert '_compare_peer_descriptors' not in branch
    assert '_queue_crop_request' not in branch
    assert 'return' in source[branch_end:source.index(
        'def _drain_candidate_registration', branch_end)]


def test_full_map_registration_has_one_canonical_order_and_seeded_peer_path():
    source = open(
        'src/my_epuck_project/my_epuck_project/unknown_pose_frontend.py',
        encoding='utf-8').read()
    scheduler = source[source.index('def _maybe_schedule_full_map_registration'):
                       source.index('def _full_map_descriptor')]
    verifier = source[source.index('def _verify_full_map_proposal'):
                      source.index('def _release_full_map_proposal_pins')]
    assert "canonical_source_robot='robot1'" in scheduler
    assert "canonical_target_robot='robot2'" in scheduler
    assert '_run_stationary_full_map_registration' in scheduler
    assert 'initial_transform=proposal_seed' in verifier
    assert 'register_crops(' not in verifier


def test_full_map_family_proposal_is_symmetric():
    source = open(
        'src/my_epuck_project/my_epuck_project/unknown_pose_frontend.py',
        encoding='utf-8').read()
    drain = source[source.index('def _drain_full_map_registration'):source.index(
        'def _maybe_schedule_full_map_registration')]
    assert 'self._publish_full_map_proposal(family)' in drain
    assert "self.robot_id == 'robot1'" not in drain


def test_full_map_operability_does_not_require_old_maturity_threshold():
    frontend = UnknownPoseFrontend.__new__(UnknownPoseFrontend)
    values = np.zeros((8, 8), dtype=np.int16)
    values[:2, :8] = 100
    source = {
        'robot_id': 'robot1', 'crop': GridCrop(values, 0.03, 0.0, 0.0),
        'known_cells': 64, 'occupied_cells': 16,
    }
    target = {
        'robot_id': 'robot2', 'crop': GridCrop(values, 0.03, 0.5, 0.0),
        'known_cells': 64, 'occupied_cells': 16,
    }
    operable, details = frontend._full_map_pair_operable(source, target)
    assert operable
    assert details['source']['occupied_cells'] == 16


def test_local_refinement_is_deterministic_and_subcell():
    values = np.zeros((100, 120), dtype=np.int16)
    values[15:18, 12:105] = 100
    values[15:80, 12:15] = 100
    values[70:73, 12:92] = 100
    values[35:72, 60:63] = 100
    source = GridCrop(values, 0.03, -2.0, -1.0)
    expected = (0.713, -0.187, 0.0)
    target = GridCrop(values, 0.03, -2.0 + expected[0],
                      -1.0 + expected[1])
    first = refine_registration_locally(
        source, target, (0.710, -0.190, 0.0))
    second = refine_registration_locally(
        source, target, (0.710, -0.190, 0.0))
    assert first.transform == second.transform
    assert np.linalg.norm(np.asarray(first.transform[:2]) - expected[:2]) < 0.006
    assert abs(first.transform[2] - expected[2]) < math.radians(0.08)
    assert abs(first.transform[0] / 0.001 - round(first.transform[0] / 0.001)) < 1e-9


def test_exact_full_map_snapshot_response_preserves_original_cells():
    frontend = UnknownPoseFrontend.__new__(UnknownPoseFrontend)
    frontend.robot_id = 'robot1'
    frontend.local_full_map_snapshots = {}
    frontend._full_map_proposal_pins = set()
    values = np.asarray([[-1, 0, 100], [25, 50, -1]], dtype=np.int16)
    values.setflags(write=False)
    frontend.local_full_map_snapshots['robot1-full-map-00000007-x'] = {
        'id': 'robot1-full-map-00000007-x', 'timestamp_ns': 123,
        'revision': 7, 'frame_id': 'robot1/map', 'crop': GridCrop(
            values=values, resolution=0.03, origin_x=1.2, origin_y=-0.7,
            origin_yaw=0.35), 'width': 3, 'height': 2, 'resolution': 0.03,
        'origin_x': 1.2, 'origin_y': -0.7, 'origin_yaw': 0.35,
        'known_cells': 4, 'occupied_cells': 2, 'free_cells': 2,
    }
    response, cell_count = frontend._snapshot_response(
        'robot1-full-map-00000007-x')
    assert response.available
    assert response.snapshot_id == 'robot1-full-map-00000007-x'
    assert list(response.occupancy_data) == [-1, 0, 100, 25, 50, -1]
    assert response.origin_x == 1.2
    assert cell_count == 6


def test_full_map_snapshot_cache_keeps_pinned_evidence():
    frontend = UnknownPoseFrontend.__new__(UnknownPoseFrontend)
    frontend.full_map_max_snapshots = 2
    frontend._full_map_proposal_pins = {'pinned'}
    cache = {'pinned': 1, 'old': 2, 'new': 3}
    frontend._trim_full_map_cache(cache)
    assert set(cache) == {'pinned', 'new'}


def test_missing_peer_snapshot_is_requested_by_exact_id():
    frontend = UnknownPoseFrontend.__new__(UnknownPoseFrontend)
    frontend.robot_id = 'robot1'
    frontend.peer_robot_id = 'robot2'
    frontend._requested_full_map_snapshot_ids = set()
    frontend.counters = {'full_map_evidence_requests': 0}
    frontend._record_diagnostic_event = lambda *args, **kwargs: None
    published = []

    class Publisher:
        def publish(self, message):
            published.append(message)

    frontend.full_map_snapshot_request_pub = Publisher()
    assert frontend._request_full_map_snapshots([
        'robot2-full-map-00000042-exact']) is True
    assert len(published) == 1
    request = published[0]
    assert request.requester_robot_id == 'robot1'
    assert request.owner_robot_id == 'robot2'
    assert list(request.snapshot_ids) == ['robot2-full-map-00000042-exact']
