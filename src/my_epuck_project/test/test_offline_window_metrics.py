from my_epuck_project.offline_window_metrics import analyze_windows


def test_windows_are_half_open_and_do_not_zero_fill_beyond_horizon():
    evaluation = {
        "clock": {"end_s": 180.0},
        "coverage": {"series": [
            {"sim_time_s": 0.0, "area_m2": 1.0},
            {"sim_time_s": 179.0, "area_m2": 2.0},
            {"sim_time_s": 180.0, "area_m2": 3.0},
        ]},
        "cooperation": {"dispatches": [
            {"sim_time_s": 179.9}, {"sim_time_s": 180.0},
        ]},
        "avoidable_idle": {"robots": {}},
        "motion_anomalies": {"robots": {}},
    }
    result = analyze_windows(evaluation)
    first, second = result["windows"][:2]
    assert first["status"] == "AVAILABLE"
    assert first["events"]["dispatches"] == 1
    assert second["status"] == "NOT_AVAILABLE"


def test_available_window_reports_idle_segments():
    evaluation = {
        "clock": {"end_s": 10.0},
        "coverage": {"series": []}, "cooperation": {},
        "motion_anomalies": {"robots": {}},
        "avoidable_idle": {"robots": {"robot1": {"segments": [
            {"start_s": 0.0, "end_s": 10.0,
             "classification": "FEASIBLE_WORK_AVAILABLE",
             "goal_active": False},
        ]}}},
    }
    result = analyze_windows(evaluation, ((0.0, 10.0),))
    assert result["windows"][0]["idle"]["robot1"][
        "avoidable_idle_seconds"] == 10.0
