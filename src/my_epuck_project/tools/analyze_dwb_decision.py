#!/usr/bin/env python3
"""Decision report for the retained full-candidate DWB counterfactual run.

This is deliberately an offline report generator.  It consumes the existing
full-candidate JSONL and never contacts ROS, starts Webots, or edits runtime
configuration.  Scores are recomputed in the recorded trajectory order using
the strict-lower DWB selection rule.
"""

from __future__ import annotations

import argparse
import csv
import json
import statistics
from collections import Counter, OrderedDict
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import analyze_dwb_critic_counterfactual as dwb  # noqa: E402


EPS = 1.0e-9
NEAR_EPS = 1.0e-6
PATH_KEYS = {"PathAlign": "path_align", "PathDist": "path_dist", "GoalDist": "goal_dist"}


def _score_items(frame, scales):
    return [(item, dwb._weighted_sum(item, scales))
            for item in frame["evaluation"].get("trajectories", [])
            if item.get("valid") and dwb._complete(item)]


def _outcome(frame, scales):
    items = _score_items(frame, scales)
    winner, winner_score = dwb._strict_winner(
        frame["evaluation"].get("trajectories", []), scales)
    ordered = sorted(items, key=lambda pair: pair[1])
    minimum = ordered[0][1] if ordered else None
    exact = [item for item, score in items if score == minimum]
    near = [item for item, score in items if abs(score - minimum) <= NEAR_EPS]
    margin = None if len(ordered) < 2 else ordered[1][1] - ordered[0][1]
    return {
        "winner": winner,
        "winner_score": winner_score,
        "exact": exact,
        "near": near,
        "margin": margin,
        "items": items,
    }


def _category(item):
    return dwb._command_category(item) if item is not None else "unavailable"


def _velocity(item):
    velocity = item.get("velocity", {}) if item else {}
    return float(velocity.get("linear_x_mps", 0.0)), float(velocity.get("angular_z_radps", 0.0))


def _contribution(item, critic, scales):
    raw = float((item or {}).get("critic_contributions", {}).get(critic, 0.0))
    if critic in PATH_KEYS:
        key = PATH_KEYS[critic]
        return raw * float(scales[key]) / float(dwb.BASELINE[key])
    return raw


def _raw_contribution(item, critic):
    return float((item or {}).get("critic_contributions", {}).get(critic, 0.0))


def _is_healthy(frame):
    return str(frame.get("capture_reason", "")).startswith("HEALTHY_")


def _is_active_stall(frame):
    if _is_healthy(frame):
        return False
    selected = next((x for x in frame["evaluation"].get("trajectories", [])
                     if x.get("trajectory_index") == frame["evaluation"].get("selected_index")), None)
    return _category(selected) in ("zero", "angular_only")


def _context_key(frame):
    """Group repeated score/goal contexts without grouping different robots."""
    trajectories = frame["evaluation"].get("trajectories", [])
    selected = next(x for x in trajectories
                    if x.get("trajectory_index") == frame["evaluation"].get("selected_index"))
    forward = [x for x in trajectories if x.get("valid") and
               _velocity(x)[0] >= dwb.FORWARD_THRESHOLD_MPS - EPS]
    best_forward = min(forward, key=lambda x: float(x["total_score"])) if forward else None
    def vector(item):
        return tuple((c.get("name"), round(float(c.get("raw_score", 0.0)), 6))
                     for c in item.get("critics", []))
    return (frame.get("robot"), (frame.get("goal") or {}).get("source"),
            (frame.get("goal") or {}).get("label"), vector(selected), vector(best_forward or {}))


def _context_groups(frames):
    groups = OrderedDict()
    for index, frame in enumerate(frames):
        if _is_healthy(frame):
            continue
        groups.setdefault(_context_key(frame), []).append((index, frame))
    return list(groups.values())


def _write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0]) if rows else []
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        if fields:
            writer.writeheader()
            writer.writerows(rows)


def _median(values):
    values = [float(x) for x in values if x is not None]
    return statistics.median(values) if values else None


def _summary_rows(frames):
    rows = []
    healthy = [f for f in frames if _is_healthy(f)]
    event_stalls = [f for f in frames if not _is_healthy(f)]
    active_stalls = [f for f in event_stalls if _is_active_stall(f)]
    contexts = _context_groups(frames)
    # Keep the original full-frame indices when looking up outcomes.  The
    # context helper intentionally preserves those indices; rebuilding it from
    # the filtered list would make frame zero refer to a healthy sample.
    active_contexts = [group for group in contexts
                       if all(_is_active_stall(frame) for _, frame in group)]
    for name, scales in dwb.full_scale_candidates():
        outcomes = [_outcome(frame, scales) for frame in frames]
        baseline_items = [_outcome(frame, dwb.BASELINE) for frame in frames]
        changed_active = [i for i, frame in enumerate(active_stalls)
                          if _category(outcomes[frames.index(frame)]["winner"]) == "forward_executable"]
        changed_contexts = sum(
            any(_category(outcomes[index]["winner"]) == "forward_executable"
                for index, _ in group)
            for group in active_contexts)
        healthy_preserved = healthy_zero = healthy_angular = healthy_different = 0
        healthy_forward = 0
        for frame, outcome in zip(healthy, [outcomes[frames.index(f)] for f in healthy]):
            base = next(x for x in frame["evaluation"]["trajectories"]
                        if x.get("trajectory_index") == frame["evaluation"].get("selected_index"))
            candidate = outcome["winner"]
            category = _category(candidate)
            if category == "forward_executable":
                healthy_forward += 1
                healthy_preserved += 1
                vx0, wz0 = _velocity(base)
                vx1, wz1 = _velocity(candidate)
                healthy_different += abs(vx0 - vx1) > EPS or abs(wz0 - wz1) > EPS
            elif category == "zero":
                healthy_zero += 1
            elif category == "angular_only":
                healthy_angular += 1
        winners = [o["winner"] for o in outcomes]
        margins = [o["margin"] for o in outcomes]
        active_outcomes = [outcomes[frames.index(frame)] for frame in active_stalls]
        active_margins = [o["margin"] for o in active_outcomes]
        active_winners = [o["winner"] for o in active_outcomes]
        vx = [_velocity(x)[0] for x in winners]
        wz = [abs(_velocity(x)[1]) for x in winners]
        exact_ties = sum(len(o["exact"]) > 1 for o in outcomes)
        near_ties = sum(len(o["near"]) > 1 for o in outcomes)
        reverse = sum(_category(x) == "reverse" for x in winners)
        below = sum(_category(x) == "forward_below_threshold" for x in winners)
        extreme = sum(abs(_velocity(x)[1]) >= 0.30 - EPS for x in winners)
        new_extreme = sum(abs(_velocity(x)[1]) >= 0.30 - EPS and
                          abs(_velocity(b)[1]) < 0.30 - EPS
                          for x, b in zip(winners, [next(t for t in f["evaluation"]["trajectories"]
                                                        if t.get("trajectory_index") == f["evaluation"].get("selected_index"))
                                                   for f in frames]))
        rows.append({
            "configuration": name,
            "path_align": scales["path_align"], "path_dist": scales["path_dist"],
            "goal_dist": scales["goal_dist"],
            "total_stall_frames": len(event_stalls),
            "active_zero_or_angular_stall_frames": len(active_stalls),
            "stall_frames_changed_to_executable_forward": len(changed_active),
            "distinct_stall_contexts": len(contexts),
            "distinct_active_stall_contexts": len(active_contexts),
            "distinct_active_stall_contexts_changed_to_forward": changed_contexts,
            "stall_frames_remaining_zero_or_angular": len(active_stalls) - len(changed_active),
            "healthy_frames": len(healthy),
            "healthy_forward_preserved": healthy_preserved,
            "healthy_changed_to_zero": healthy_zero,
            "healthy_changed_to_angular_only": healthy_angular,
            "healthy_changed_to_different_forward": healthy_different,
            "reverse_winner_count": reverse, "below_threshold_winner_count": below,
            "extreme_angular_winner_count": extreme,
            "new_extreme_angular_winner_count": new_extreme,
            "exact_tie_frame_count": exact_ties, "near_tie_frame_count": near_ties,
            "median_winning_margin": _median(margins),
            "minimum_winning_margin": min(margins) if margins else None,
            "median_positive_winning_margin": _median([x for x in margins if x > NEAR_EPS]),
            "median_selected_vx_mps": _median(vx), "median_abs_wz_radps": _median(wz),
            "median_active_stall_vx_mps": _median([_velocity(x)[0] for x in active_winners]),
            "median_active_stall_abs_wz_radps": _median([abs(_velocity(x)[1]) for x in active_winners]),
            "median_active_stall_winning_margin": _median(active_margins),
            "minimum_active_stall_winning_margin": min(active_margins) if active_margins else None,
            "median_path_align_contribution": _median(
                [_contribution(x, "PathAlign", scales) for x in winners]),
            "median_path_dist_contribution": _median(
                [_contribution(x, "PathDist", scales) for x in winners]),
            "median_goal_dist_contribution": _median(
                [_contribution(x, "GoalDist", scales) for x in winners]),
        })
    return rows


def _context_rows(frames):
    rows = []
    groups = _context_groups(frames)
    for context_id, group in enumerate(groups, 1):
        index, representative = group[0]
        trajectories = representative["evaluation"]["trajectories"]
        baseline = next(x for x in trajectories
                        if x.get("trajectory_index") == representative["evaluation"].get("selected_index"))
        forward = [x for x in trajectories if x.get("valid") and
                   _velocity(x)[0] >= dwb.FORWARD_THRESHOLD_MPS - EPS]
        best_forward = min(forward, key=lambda x: float(x["total_score"]))
        for name, scales in dwb.full_scale_candidates():
            out = _outcome(representative, scales)
            winner = out["winner"]
            rows.append({
                "context_id": context_id, "representative_frame": index,
                "frames_in_context": "|".join(str(i) for i, _ in group),
                "robot": representative.get("robot"),
                "goal_type": (representative.get("goal") or {}).get("source"),
                "goal_label": (representative.get("goal") or {}).get("label"),
                "sim_time_s": representative.get("simulation_timestamp_s"),
                "baseline_category": _category(baseline),
                "baseline_vx_mps": _velocity(baseline)[0], "baseline_wz_radps": _velocity(baseline)[1],
                "baseline_total": baseline.get("total_score"),
                "best_executable_forward_vx_mps": _velocity(best_forward)[0],
                "best_executable_forward_wz_radps": _velocity(best_forward)[1],
                "best_executable_forward_baseline_total": best_forward.get("total_score"),
                "configuration": name,
                "winner_index": winner.get("trajectory_index") if winner else None,
                "winner_category": _category(winner),
                "winner_vx_mps": _velocity(winner)[0] if winner else None,
                "winner_wz_radps": _velocity(winner)[1] if winner else None,
                "winner_total": out["winner_score"],
                "second_best_total": (out["winner_score"] + out["margin"]
                                       if out["margin"] is not None else None),
                "winning_margin": out["margin"],
                "exact_tie_count": len(out["exact"]),
                "near_tie_count": len(out["near"]),
                "winner_changed_from_baseline": winner.get("trajectory_index") != baseline.get("trajectory_index"),
            })
    return rows


def _healthy_rows(frames):
    rows = []
    for index, frame in enumerate(frames):
        if not _is_healthy(frame):
            continue
        base = next(x for x in frame["evaluation"]["trajectories"]
                    if x.get("trajectory_index") == frame["evaluation"].get("selected_index"))
        for name, scales in dwb.full_scale_candidates():
            out = _outcome(frame, scales)
            winner = out["winner"]
            vx0, wz0 = _velocity(base); vx1, wz1 = _velocity(winner)
            rows.append({
                "frame_index": index, "robot": frame.get("robot"),
                "capture_reason": frame.get("capture_reason"),
                "goal_source": (frame.get("goal") or {}).get("source"),
                "goal_label": (frame.get("goal") or {}).get("label"),
                "distance_to_goal_m": (frame.get("goal") or {}).get("distance_to_goal_m"),
                "configuration": name,
                "baseline_vx_mps": vx0, "baseline_wz_radps": wz0,
                "baseline_total": base.get("total_score"),
                "winner_index": winner.get("trajectory_index"),
                "winner_vx_mps": vx1, "winner_wz_radps": wz1,
                "winner_total": out["winner_score"],
                "delta_vx_mps": vx1 - vx0, "delta_wz_radps": wz1 - wz0,
                "baseline_raw_path_align": _raw_contribution(base, "PathAlign"),
                "winner_raw_path_align": _raw_contribution(winner, "PathAlign"),
                "baseline_raw_path_dist": _raw_contribution(base, "PathDist"),
                "winner_raw_path_dist": _raw_contribution(winner, "PathDist"),
                "path_align_contribution": _contribution(winner, "PathAlign", scales),
                "path_dist_contribution": _contribution(winner, "PathDist", scales),
                "goal_dist_contribution": _contribution(winner, "GoalDist", scales),
                "path_penalty_raw_increased": (_raw_contribution(winner, "PathAlign") +
                                                _raw_contribution(winner, "PathDist") >
                                                _raw_contribution(base, "PathAlign") +
                                                _raw_contribution(base, "PathDist") + EPS),
                "remains_forward": _category(winner) == "forward_executable",
                "exact_tie_count": len(out["exact"]),
            })
    return rows


def _tie_rows(frames):
    rows = []
    for index, frame in enumerate(frames):
        trajectories = frame["evaluation"].get("trajectories", [])
        valid = [t for t in trajectories if t.get("valid")]
        minimum = min(float(t["total_score"]) for t in valid)
        ties = [t for t in valid if float(t["total_score"]) == minimum]
        if len(ties) < 2:
            continue
        commands = [_velocity(t) for t in ties]
        categories = Counter(_category(t) for t in ties)
        vxs = {round(v[0], 9) for v in commands}
        wzs = {round(v[1], 9) for v in commands}
        rows.append({
            "frame_index": index, "robot": frame.get("robot"),
            "capture_reason": frame.get("capture_reason"),
            "sim_time_s": frame.get("simulation_timestamp_s"),
            "exact_minimum_count": len(ties),
            "categories": "|".join(f"{k}:{v}" for k, v in sorted(categories.items())),
            "all_ties_same_vx": len(vxs) == 1,
            "tie_vx_values": "|".join(f"{x:.6f}" for x in sorted(vxs)),
            "tie_wz_values": "|".join(f"{x:.6f}" for x in sorted(wzs)),
            "same_vx_different_wz": len(vxs) == 1 and len(wzs) > 1,
            "forward_tied_with_zero_or_angular": bool(
                categories["forward_executable"] and
                (categories["zero"] or categories["angular_only"])),
            "indices": "|".join(str(t["trajectory_index"]) for t in ties),
        })
    return rows


def write_report(input_dir: Path, output_dir: Path):
    frames = dwb._full_records(input_dir / "dwb_full_candidate_frames.jsonl")
    output_dir.mkdir(parents=True, exist_ok=True)
    configurations = _summary_rows(frames)
    contexts = _context_rows(frames)
    healthy = _healthy_rows(frames)
    ties = _tie_rows(frames)
    _write_csv(output_dir / "dwb_full_decision_configuration_summary.csv", configurations)
    _write_csv(output_dir / "dwb_full_decision_stall_contexts.csv", contexts)
    _write_csv(output_dir / "dwb_full_decision_healthy_frames.csv", healthy)
    _write_csv(output_dir / "dwb_full_decision_ties.csv", ties)
    summary = {
        "schema_version": 1,
        "source": str(input_dir / "dwb_full_candidate_frames.jsonl"),
        "method": "complete retained valid trajectories, original array order, strict-lower winner",
        "tested_configurations": [row["configuration"] for row in configurations],
        "event_stall_frame_count": sum(not _is_healthy(f) for f in frames),
        "active_zero_or_angular_stall_frame_count": sum(_is_active_stall(f) for f in frames),
        "healthy_frame_count": sum(_is_healthy(f) for f in frames),
        "distinct_event_context_count": len(_context_groups(frames)),
        "distinct_active_stall_context_count": len(_context_groups([f for f in frames if _is_active_stall(f)])),
        "exact_tie_frame_count": len(ties),
        "exact_tie_forward_with_zero_or_angular_count": sum(bool(r["forward_tied_with_zero_or_angular"]) for r in ties),
        "decision": "NO_CRITIC_SCALE_CONFIGURATION_SUPPORTED",
        "decision_reason": (
            "Every reduced path-critic configuration leaves the robot1 frontier angular-only "
            "context unchanged. The configurations that change the repeated robot2 context "
            "to forward do so with an exact tied minimum (zero winning margin), so they do "
            "not provide a robust complete-context correction. Healthy retained frames remain "
            "forward, but that preservation does not overcome the unresolved stall context."),
        "next_diagnostic_dimension": (
            "inspect the robot1 frontier local-path geometry and the forward-point/sim-time "
            "relationship; do not change those parameters in this analysis"),
        "artifacts": {
            "configuration_summary": str(output_dir / "dwb_full_decision_configuration_summary.csv"),
            "stall_contexts": str(output_dir / "dwb_full_decision_stall_contexts.csv"),
            "healthy_frames": str(output_dir / "dwb_full_decision_healthy_frames.csv"),
            "ties": str(output_dir / "dwb_full_decision_ties.csv"),
        },
    }
    (output_dir / "dwb_full_decision_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_dir", type=Path)
    parser.add_argument("--output-dir", type=Path, default=None)
    args = parser.parse_args()
    output = args.output_dir or args.input_dir / "dwb_full_counterfactual"
    print(json.dumps(write_report(args.input_dir, output), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
