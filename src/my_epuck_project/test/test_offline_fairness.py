from my_epuck_project.offline_fairness import summarize_fairness


def test_fairness_emits_components_and_jain_index():
    result = summarize_fairness({
        "coverage": {"ownership": {
            "available": True,
            "unique_first_seen_cells": {"robot1": 3, "robot2": 1},
            "later_duplicated_cells": {"robot1": 2, "robot2": 0},
        }},
        "motion": {
            "robot1": {"travel_distance_m": 4.0},
            "robot2": {"travel_distance_m": 2.0},
        },
        "cooperation_summary": {
            "assignments": {"by_robot": {"robot1": 2, "robot2": 1}},
            "terminals": {
                "robot1": {"succeeded": 2},
                "robot2": {"succeeded": 1},
            },
        },
        "avoidable_idle": {"robots": {
            "robot1": {"feasible_work_seconds": 10.0,
                        "avoidable_idle_seconds": 1.0,
                        "productive_engagement_seconds": 9.0},
            "robot2": {"feasible_work_seconds": 8.0,
                        "avoidable_idle_seconds": 0.0,
                        "productive_engagement_seconds": 8.0},
        }},
    })
    assert result["available"]
    assert result["robots"]["robot1"]["first_seen_cells"] == 3
    assert result["first_seen_cell_shares"]["robot1"] == 0.75
    assert result["jain_first_seen_cell_fairness"] == 0.8


def test_fairness_fails_closed_when_ownership_is_unavailable():
    result = summarize_fairness({"coverage": {"ownership": {
        "available": False, "reason": "local maps incomplete"}}})
    assert not result["available"]
    assert result["reason"] == "local maps incomplete"
