"""Expose established map-quality artifacts without inventing quality scores."""

from __future__ import annotations

import json
from pathlib import Path


def _load(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def map_quality_report(run_directory: Path):
    """Load existing quality analysis and identify unobservable fields.

    ``analyze_slam_map_quality.py`` and physical-GT finalization are existing
    authorities.  This adapter never turns coverage counts or a missing
    reference map into a quality score.
    """
    root = Path(run_directory)
    candidates = (
        root / "map_quality.json",
        root / "forensic" / "map_quality.json",
        root / "forensic" / "physical_gt_evaluation.json",
    )
    for path in candidates:
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
    return {
        "available": False,
        "source": None,
        "metrics": {},
        "missing": ["map-quality artifact or reference map"],
        "reason": "no established map-quality result is present",
    }
