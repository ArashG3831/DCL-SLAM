import csv

from my_epuck_project.offline_motion_metrics import replay_run, replay_timeseries


FIELDS = (
    "elapsed_s", "pose_x", "pose_y", "commanded_linear_mps",
    "commanded_angular_radps", "navigation_active", "distance_remaining_m",
)


def write_rows(path, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def row(t, active=True, remaining="1.0", angular="0.0"):
    return {"elapsed_s": t, "pose_x": 0.0, "pose_y": 0.0,
            "commanded_linear_mps": 0.0,
            "commanded_angular_radps": angular,
            "navigation_active": str(active),
            "distance_remaining_m": remaining}


def test_replay_uses_the_authoritative_motion_detector(tmp_path):
    path = tmp_path / "robot1_timeseries.csv"
    write_rows(path, [row(0), row(1), row(2), row(3, active=False)])
    result = replay_timeseries(
        path, "robot1", progress_window=2.0, stuck_window=99.0)
    assert result["available"]
    assert result["event_counts"]["NO_PROGRESS_STARTED"] == 1
    assert result["event_counts"]["NO_PROGRESS_CLEARED"] == 1
    assert result["episodes"] == [{
        "type": "NO_PROGRESS",
        "start_sim_time_s": 2.0,
        "end_sim_time_s": 3.0,
    }]


def test_empty_stream_is_zero_but_absent_stream_is_unavailable(tmp_path):
    path = tmp_path / "robot1_timeseries.csv"
    write_rows(path, [])
    result = replay_run(tmp_path, ("robot1",))
    assert result["available"]
    assert result["robots"]["robot1"]["event_count"] == 0
    missing = replay_timeseries(tmp_path / "missing.csv", "robot1")
    assert not missing["available"]
