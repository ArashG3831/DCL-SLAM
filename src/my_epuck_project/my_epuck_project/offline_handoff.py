"""Offline reciprocal checks for structured accepted handoff hypotheses."""

from __future__ import annotations

import math


def _transform(record):
    value = record.get("transform_se2")
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        return None
    try:
        result = tuple(float(item) for item in value[:3])
    except (TypeError, ValueError):
        return None
    return result if all(math.isfinite(item) for item in result) else None


def _compose(first, second):
    ax, ay, ayaw = first
    bx, by, byaw = second
    c, s = math.cos(ayaw), math.sin(ayaw)
    return (ax + c * bx - s * by, ay + s * bx + c * by,
            ayaw + byaw)


def reciprocal_verification(records):
    """Verify inverse consistency only when both directions are observed."""
    by_direction = {}
    for record in records:
        if not record.get("accepted"):
            continue
        source = record.get("source_robot_id")
        target = record.get("target_robot_id")
        transform = _transform(record)
        if source and target and transform is not None:
            by_direction.setdefault((str(source), str(target)), []).append(
                (record, transform))
    directions = sorted(by_direction)
    verified = []
    missing = []
    seen = set()
    for source, target in directions:
        if (source, target) in seen:
            continue
        reverse = (target, source)
        seen.update(((source, target), reverse))
        forward = by_direction[(source, target)]
        backward = by_direction.get(reverse, [])
        if not backward:
            missing.append({"source_robot_id": source,
                            "target_robot_id": target,
                            "reason": "reciprocal direction absent"})
            continue
        left = forward[0][1]
        right = backward[0][1]
        composed = _compose(left, right)
        verified.append({
            "source_robot_id": source,
            "target_robot_id": target,
            "inverse_translation_error_m": math.hypot(
                composed[0], composed[1]),
            "inverse_yaw_error_rad": abs(math.atan2(
                math.sin(composed[2]), math.cos(composed[2]))),
            "evidence_hashes_match": (
                forward[0][0].get("evidence_set_hash") ==
                backward[0][0].get("evidence_set_hash")),
            "forward_acceptance_time_s": forward[0][0].get(
                "accepted_ros_time_s"),
            "reverse_acceptance_time_s": backward[0][0].get(
                "accepted_ros_time_s"),
        })
    return {
        "status": "VERIFIED" if verified else (
            "MISSING_RECIPROCAL_EVIDENCE" if missing else "NO_ACCEPTED_HANDOFF"),
        "verified_pairs": verified,
        "missing_pairs": missing,
        "record_count": len(records),
    }
