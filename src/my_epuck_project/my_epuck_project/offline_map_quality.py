"""Expose established map-quality artifacts without inventing quality scores."""

from __future__ import annotations

import json
import importlib.util
from pathlib import Path


def _load(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def _world_path(root, manifest):
    manifest = manifest or {}
    seed = manifest.get("seed_provenance") or {}
    candidates = [
        seed.get("canonical_base_world_path"),
        manifest.get("source_world_path"),
        manifest.get("world_path"),
        manifest.get("world_resource"),
    ]
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            return Path(candidate)
    return None


def _existing_direct_geometry_report(root, manifest):
    """Reuse the project's established world-geometry map analyzer."""
    world = _world_path(root, manifest)
    map_root = root / "forensic" / "maps"
    map_paths = {
        f"{robot}_{kind}": map_root / f"{robot}_{kind}_final.npz"
        for robot in ("robot1", "robot2")
        for kind in ("map", "shared_map")
    }
    if world is None or not all(path.is_file() for path in map_paths.values()):
        return None
    tool_path = Path(__file__).resolve().parents[1] / "tools" / \
        "analyze_cooperative_decision_offline.py"
    if not tool_path.is_file():
        return None
    try:
        spec = importlib.util.spec_from_file_location(
            "existing_map_quality_tool", tool_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        segments, world_metadata = module.static_segments(world)
        maps = {}
        for key, path in map_paths.items():
            robot, kind = key.split("_", 1)
            maps[key] = module.score_map(
                path, robot, kind, segments, world_metadata,
                root / "forensic")
        return {
            "available": True,
            "source": str(tool_path),
            "quality_reference": "saved world geometry and final occupancy maps",
            "metrics": {
                "reference": {
                    "world_path": str(world),
                    "segments": len(segments),
                    "solid_boxes": len(world_metadata["obstacles"]),
                    "dimensions_m": list(world_metadata["dimensions"]),
                },
                "maps": maps,
            },
            "missing": [],
        }
    except (ImportError, OSError, ValueError, KeyError, TypeError):
        # A missing or malformed optional reference remains unavailable; do
        # not turn it into a fabricated quality score.
        return None


def map_quality_report(run_directory: Path, manifest=None):
    """Load existing quality analysis and identify unobservable fields.

    ``analyze_slam_map_quality.py`` and physical-GT finalization are existing
    authorities.  This adapter never turns coverage counts or a missing
    reference map into a quality score.
    """
    root = Path(run_directory)
    map_quality_candidates = (
        root / "map_quality.json",
        root / "forensic" / "map_quality.json",
    )
    for path in map_quality_candidates:
        payload = _load(path)
        if payload is not None:
            source = str(path.relative_to(root))
            return {
                "available": True,
                "source": source,
                "metrics": payload,
                "quality_reference": (
                    "existing project map-quality/physical-GT artifact"),
                "missing": [],
            }
    direct = _existing_direct_geometry_report(root, manifest)
    if direct is not None:
        return direct
    # Preserve the historical fallback for fixtures that only contain the
    # physical-GT finalization artifact, but do not mistake it for map
    # geometry when a real map-quality source is available.
    physical_gt = root / "forensic" / "physical_gt_evaluation.json"
    payload = _load(physical_gt)
    if payload is not None:
        return {
            "available": True,
            "source": str(physical_gt.relative_to(root)),
            "metrics": payload,
            "quality_reference": "existing physical-GT finalization artifact",
            "missing": [],
        }
    return {
        "available": False,
        "source": None,
        "metrics": {},
        "missing": ["map-quality artifact or reference map"],
        "reason": "no established map-quality result is present",
    }
