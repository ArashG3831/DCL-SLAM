"""Focused tests for the bounded MRPT registration bridge."""

from unittest.mock import patch

import numpy as np

from my_epuck_project import mrpt_registration_backend
from my_epuck_project.unknown_pose_frontend_core import (
    GridCrop,
    RegistrationResult,
    register_crop_hypotheses,
    select_hypothesis_family,
)


def _crop(offset=0.0):
    values = np.zeros((16, 16), dtype=np.int16)
    values[2:14, 3] = 100
    values[2:14, 11] = 100
    values[4, 3:12] = 100
    values[11, 3:12] = 100
    return GridCrop(values, 0.03, float(offset), 0.0, 0.0)


def _result(transform, support=1):
    return RegistrationResult(
        accepted=True, transform=tuple(transform), covariance=(0.0,) * 36,
        inlier_ratio=1.0, reverse_inlier_ratio=1.0, residual_m=0.0,
        occupied_free_agreement=0.40, overlap_fraction=1.0, reason='ACCEPTED',
        condition_number=1.0, translation_uncertainty_m=0.0,
        yaw_uncertainty_rad=0.0, backend='mrpt', mode_support=support)


def test_occupancy_encoding_and_metadata_are_forwarded_losslessly():
    observed = {}

    class FakeNative:
        @staticmethod
        def align_maps(*args):
            observed['source'] = np.frombuffer(args[0], dtype=np.int16).copy()
            observed['target'] = np.frombuffer(args[1], dtype=np.int16).copy()
            observed['args'] = args[2:]
            return []

    source = _crop(1.25)
    source.values[0, 0] = -1
    target = _crop(-2.5)
    with patch.object(mrpt_registration_backend, '_native_module',
                      return_value=FakeNative()):
        assert mrpt_registration_backend.align_crops(source, target) == ()
    assert observed['source'][0] == -1
    assert observed['source'][1] == 0
    assert observed['source'][2] == 0
    assert observed['args'][0:8] == (
        16, 16, 16, 16, 0.03, 1.25, 0.0, 0.0)
    assert observed['args'][8:11] == (-2.5, 0.0, 0.0)


def test_different_native_crop_extents_are_forwarded_without_resampling():
    observed = {}

    class FakeNative:
        @staticmethod
        def align_maps(*args):
            observed['args'] = args[2:]
            return []

    source = GridCrop(np.zeros((8, 10), dtype=np.int16), 0.03, 1.0, 2.0)
    target = GridCrop(np.zeros((12, 14), dtype=np.int16), 0.03, -1.0, -2.0)
    with patch.object(mrpt_registration_backend, '_native_module',
                      return_value=FakeNative()):
        assert mrpt_registration_backend.align_crops(source, target) == ()
    assert observed['args'][0:8] == (
        10, 8, 14, 12, 0.03, 1.0, 2.0, 0.0)
    assert observed['args'][8:11] == (-1.0, -2.0, 0.0)


def test_repeated_modes_from_one_physical_pair_are_deduplicated():
    calls = [
        ((0.0, 0.0, 0.0, -1.0, (0.0,) * 9, 0),
         (1.0, 0.0, 0.0, -2.0, (0.0,) * 9, 1)),
        ((0.01, 0.0, 0.001, -0.5, (0.0,) * 9, 0),
         (1.0, 0.0, 0.0, -3.0, (0.0,) * 9, 1)),
    ]

    def fake_align(*args, **kwargs):
        return tuple(mrpt_registration_backend.MrptMode(
            transform=tuple(item[:3]), covariance=tuple(item[4]),
            log_weight=float(item[3]), mode_index=int(item[5]))
            for item in calls.pop(0))

    with patch('my_epuck_project.mrpt_registration_backend.align_crops',
               side_effect=fake_align):
        modes = register_crop_hypotheses(
            _crop(), _crop(), mrpt_repetitions=2, max_distinct_modes=10)
    assert len(modes) == 2
    assert modes[0].mode_support == 2
    assert modes[0].backend == 'mrpt'


def test_family_uses_at_most_one_mode_per_physical_id():
    pairs = [(_crop(0.0), _crop(0.0)), (_crop(1.0), _crop(1.0)),
             (_crop(2.0), _crop(2.0))]
    hypotheses = [
        (_result((0.0, 0.0, 0.0), 3), _result((2.0, 0.0, 0.0), 9)),
        (_result((0.01, 0.0, 0.002), 3), _result((2.0, 0.0, 0.0), 9)),
        (_result((-0.01, 0.0, -0.002), 3), _result((2.0, 0.0, 0.0), 9)),
    ]
    result = select_hypothesis_family(
        pairs, hypotheses, evidence_ids=('r1|r2a', 'r1b|r2b', 'r1c|r2c'),
        evidence_timestamps=((1, 1), (2_000_000_000, 2_000_000_000),
                             (4_000_000_000, 4_000_000_000)),
        source_viewpoints=((0.0, 0.0), (1.0, 0.0), (2.0, 0.0)),
        target_viewpoints=((0.0, 0.0), (1.0, 0.0), (2.0, 0.0)))
    assert result.accepted
    selected = [item for item in result.consensus_diagnostics
                if item.get('kind') == 'mrpt_selected_family']
    assert selected
    assert len(selected[-1]['physical_evidence_ids']) == 3


def test_empty_mode_set_fails_safely_without_a_family():
    result = select_hypothesis_family(
        [(_crop(), _crop()), (_crop(1.0), _crop(1.0)),
         (_crop(2.0), _crop(2.0))],
        [(_result((0.0, 0.0, 0.0)),), (),
         (_result((0.0, 0.0, 0.0)),)],
        evidence_ids=('a', 'b', 'c'))
    assert not result.accepted
    assert result.reason == 'INSUFFICIENT_CONSISTENT_CONSTRAINTS'
