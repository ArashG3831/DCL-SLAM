"""Thin Python boundary for MRPT's published grid-map aligner.

This module does not implement registration.  The C++ extension delegates to
``mrpt::slam::CGridMapAligner`` with ``amModifiedRANSAC`` and returns its
bounded CPosePDFSOG modes in the project's canonical R1-map-to-R2-map
convention.  Keeping this boundary small makes the estimator backend explicit
and leaves the existing consensus/peer-safety code in Python.
"""

from __future__ import annotations

from dataclasses import dataclass
import importlib
import math

import numpy as np


@dataclass(frozen=True)
class MrptMode:
    """One MRPT SOG mode for one physical map pair."""

    transform: tuple[float, float, float]
    covariance: tuple[float, ...]
    log_weight: float
    mode_index: int


def _native_module():
    try:
        return importlib.import_module('my_epuck_project._mrpt_registration')
    except (ImportError, OSError) as error:
        raise RuntimeError(
            'MRPT backend unavailable; build the package with MRPT and set '
            'LD_LIBRARY_PATH for its libraries') from error


def implementation_path() -> str:
    """Return the loaded extension path for runtime provenance."""
    return str(_native_module().__file__)


def align_crops(source, target, *, max_kld: float = 0.05,
                max_modes: int = 64) -> tuple[MrptMode, ...]:
    """Align two immutable ``GridCrop`` values and preserve SOG modes."""
    source_values = np.ascontiguousarray(
        np.asarray(source.values, dtype=np.int16))
    target_values = np.ascontiguousarray(
        np.asarray(target.values, dtype=np.int16))
    if source_values.ndim != 2 or target_values.ndim != 2:
        raise ValueError('MRPT registration requires 2-D occupancy crops')
    if not math.isfinite(float(source.resolution)) or not math.isfinite(
            float(target.resolution)) or abs(float(source.resolution) -
                                             float(target.resolution)) > 1e-6:
        raise ValueError('MRPT registration requires equal finite resolutions')
    native_modes = _native_module().align_maps(
        source_values.tobytes(order='C'), target_values.tobytes(order='C'),
        int(source_values.shape[1]), int(source_values.shape[0]),
        int(target_values.shape[1]), int(target_values.shape[0]),
        float(source.resolution), float(source.origin_x),
        float(source.origin_y), float(source.origin_yaw),
        float(target.origin_x), float(target.origin_y),
        float(target.origin_yaw), float(max_kld), int(max_modes))
    modes = []
    for item in native_modes:
        if len(item) != 6:
            raise RuntimeError('invalid MRPT mode payload')
        transform = tuple(float(value) for value in item[:3])
        covariance = tuple(float(value) for value in item[4])
        if len(covariance) != 9 or not np.isfinite(transform).all() or not np.isfinite(
                covariance).all():
            continue
        modes.append(MrptMode(
            transform=transform,
            covariance=covariance,
            log_weight=float(item[3]),
            mode_index=int(item[5])))
    return tuple(modes)
