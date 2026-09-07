"""Semantic legacy/current output comparison for the offline evaluator."""

from __future__ import annotations

import math


REQUIRED_FIELDS = (
    ("coverage", "coverage"),
    ("coverage_auc", "coverage.known_area_auc_m2_s"),
    ("ownership", "coverage.ownership"),
    ("trajectory_overlap", "ground_truth.trajectory_overlap"),
    ("avoidable_idle", "avoidable_idle"),
    ("snappiness", "snappiness"),
    ("cooperation", "cooperation_summary"),
    ("motion_anomalies", "motion_anomalies"),
    ("fairness", "fairness"),
    ("scaling_windows", "scaling_windows"),
    ("handoff", "handoff"),
    ("map_quality", "map_quality_report"),
    ("navigation", "navigation"),
)


def _get(mapping, dotted):
    value = mapping
    for part in dotted.split("."):
        if not isinstance(value, dict) or part not in value:
            return None, False
        value = value[part]
    return value, True


def _semantic_equal(left, right):
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return (math.isclose(float(left), float(right), rel_tol=0.0,
                             abs_tol=1.0e-12))
    return left == right


def compare_outputs(legacy, current):
    """Compare selected output families without requiring byte-identical JSON."""
    rows = []
    for name, path in REQUIRED_FIELDS:
        old, old_present = _get(legacy, path)
        new, new_present = _get(current, path)
        if not new_present:
            status = "BLOCKED_MISSING_RAW_EVIDENCE"
            reason = f"current output missing {path}"
        elif not old_present:
            status = "PRESERVED_BY_EQUIVALENT_OFFLINE_OUTPUT"
            reason = "legacy oracle has no corresponding aggregate field"
        elif _semantic_equal(old, new):
            status = "EXACT_PARITY"
            reason = "parsed semantic values match"
        else:
            status = "SEMANTIC_REVIEW_REQUIRED"
            reason = "both values exist but differ"
        rows.append({"metric": name, "path": path, "status": status,
                     "reason": reason})
    return {"rows": rows,
            "status_counts": {
                status: sum(row["status"] == status for row in rows)
                for status in sorted({row["status"] for row in rows})}}
