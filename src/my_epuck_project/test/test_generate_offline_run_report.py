import importlib.util
from pathlib import Path

import pytest


MODULE_PATH = (Path(__file__).parents[1] / "tools" /
               "generate_offline_run_report.py")
SPEC = importlib.util.spec_from_file_location("offline_report_generator", MODULE_PATH)
REPORT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REPORT)


def _event(robot, time_s, kind, **fields):
    return {
        "robot_id": robot,
        "event_type": kind,
        "elapsed_s": time_s,
        **fields,
    }


def test_generation_statuses_keep_reachable_and_unclassified_distinct():
    run = {
        "events": [
            _event("robot1", 1.0, "CANDIDATE_BATCH_RECEIVED",
                   candidate_count=3, detected_frontier_count=5,
                   unreachable_frontier_count=1,
                   detected_not_queried_count=1,
                   unclassified_frontier_count=1),
            _event("robot1", 1.1, "DISTRIBUTED_TASK_SNAPSHOT", tasks=[1, 2]),
            _event("robot2", 2.0, "CANDIDATE_BATCH_RECEIVED",
                   candidate_count=2, detected_frontier_count=2),
        ],
        "end_sim": 3.0,
    }
    result = REPORT._generation_metrics(run)
    assert result["robot1"]["statuses"]["reachable"]["total_observations"] == 3
    assert result["robot1"]["statuses"]["unclassified"]["total_observations"] == 1
    assert result["robot1"]["statuses"]["other"]["total_observations"] is None
    assert result["combined"]["statuses"]["reachable"]["total_observations"] == 5


def test_query_metrics_do_not_claim_invalid_async_missing_after_pairs():
    run = {
        "end_sim": 5.0,
        "rosout": [
            {"elapsed_s": 1.0, "name": "robot1.frontier_candidate_generator",
             "message": "FRONTIER_QUERY_LIFECYCLE state=REQUEST_SENT query_id=1 id=abc"},
            {"elapsed_s": 1.1, "name": "robot1.frontier_candidate_generator",
             "message": "FRONTIER_QUERY_LIFECYCLE state=STALE_REVISION_REJECTED query_id=1 id=abc"},
            {"elapsed_s": 1.2, "name": "robot1.frontier_candidate_generator",
             "message": "event=ASYNC_SEND_GOAL_BEFORE request_id=1"},
        ],
    }
    result = REPORT._query_metrics(run)
    assert result["robot1"]["submissions"] == 1
    assert result["robot1"]["raw_stale_rejection_records"] == 1
    assert result["robot1"]["stale_only_request_segments"] == 1
    assert result["robot1"]["async_missing_after"] is None
    assert result["robot1"]["async_pairing_reliable"] is False


def test_scalar_comparison_handles_nulls_and_percent_changes():
    current = {"timing": {"robot1": {"productivity_percent": 75.0,
                                       "avoidable_idle_seconds": 2.0,
                                       "work_unavailable_seconds": 1.0},
                            "robot2": {"productivity_percent": None,
                                       "avoidable_idle_seconds": None,
                                       "work_unavailable_seconds": None}},
               "frontiers": {"robot1": {}, "robot2": {}},
               "candidates": {"robot1": {}, "robot2": {}},
               "path_queries": {"robot1": {}, "robot2": {}, "combined": {}},
               "certificates": {"robot1": {}, "robot2": {}, "combined": {}},
               "navigation": {"combined": {}}, "coverage": {},
               "traffic": {"combined": {}}}
    baseline = {"timing": {"robot1": {"productivity_percent": 50.0,
                                        "avoidable_idle_seconds": 4.0,
                                        "work_unavailable_seconds": 1.0},
                             "robot2": {"productivity_percent": 60.0,
                                        "avoidable_idle_seconds": 1.0,
                                        "work_unavailable_seconds": 1.0}},
                "frontiers": {"robot1": {}, "robot2": {}},
                "candidates": {"robot1": {}, "robot2": {}},
                "path_queries": {"robot1": {}, "robot2": {}, "combined": {}},
                "certificates": {"robot1": {}, "robot2": {}, "combined": {}},
                "navigation": {"combined": {}}, "coverage": {},
                "traffic": {"combined": {}}}
    result = REPORT._scalar_comparison(current, baseline)
    assert result["metrics"]["timing.robot1.productivity_percent"]["absolute_change"] == 25.0
    assert result["metrics"]["timing.robot2.productivity_percent"]["absolute_change"] is None


def test_timing_invariant_validation_uses_authoritative_state_machine():
    events = [
        _event("robot1", 0.0, "CANDIDATE_BATCH_RECEIVED", candidate_count=1),
        _event("robot1", 0.0, "DISTRIBUTED_STATUS", nav2_healthy=True,
               tf_healthy=True, local_nav_goal_active=False),
        _event("robot1", 2.0, "NAV_GOAL_SENT"),
        _event("robot1", 4.0, "NAVIGATION_SUCCEEDED"),
    ]
    run = {"events": events, "ready_sim": 0.0, "end_sim": 5.0}
    result = REPORT._timing_metrics(run)
    row = result["robot1"]
    assert row["total_analyzed_time"] == 5.0
    assert row["validation"]["feasible_equals_productive_plus_idle"]
    assert row["validation"]["post_readiness_equals_feasible_plus_unavailable"]


def test_d2_artifact_report_is_deterministic_when_available():
    artifact = (Path(__file__).parents[3] / "results" /
                "thesis_condition_C_180s_20260909T232109Z" /
                "fast_trial_20260909T232114Z")
    if not artifact.is_dir():
        pytest.skip("validation artifact is not present in this checkout")
    first = REPORT.build_report(artifact)
    second = REPORT.build_report(artifact)
    assert first == second
    assert first["provenance"]["simulation_end_s"] == 180.08
    assert first["path_queries"]["combined"]["submissions"] == 199
    assert first["validation"]["all_timing_invariants_pass"]
