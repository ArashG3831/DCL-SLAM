import pytest

from my_epuck_project.offline_evidence_replay import _coverage


def _bag(run_directory):
    return {
        "run_directory": str(run_directory),
        "map_series": {
            "robot1": {"local": [], "shared": []},
            "robot2": {"local": [], "shared": []},
        },
    }


def _write_coverage(path, rows):
    path.write_text(
        "ros_time_sec,ros_time_nanosec,known_area_m2,"
        "robot1_shared_known,shared_occupied_cells,shared_unknown_cells\n"
        + "".join(
            f"{time},0,{area},{known},1,3\n"
            for time, area, known in rows),
        encoding="utf-8",
    )


def test_condition_c_uses_request_time_coverage_after_change_only_map_stream(
        tmp_path):
    _write_coverage(
        tmp_path / "coverage.csv",
        ((20, 0.18, 2), (260, 9.0, 100), (400, 9.0, 100)),
    )
    result = _coverage(
        _bag(tmp_path), ("robot1", "robot2"), "C",
        {"world_dimensions_m": [10.0, 10.0]},
    )

    assert result["snapshot_count"] == 3
    assert result["series"][-1]["sim_time_s"] == 400.0
    assert result["series"][-1]["known_cells"] == 100
    assert result["series"][-1]["area_m2"] == 9.0


def test_present_empty_authoritative_coverage_fails_closed(tmp_path):
    (tmp_path / "coverage.csv").write_text(
        "ros_time_sec,ros_time_nanosec,known_area_m2,"
        "robot1_shared_known,shared_occupied_cells,shared_unknown_cells\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="contains no rows"):
        _coverage(
            _bag(tmp_path), ("robot1", "robot2"), "C",
            {"world_dimensions_m": [10.0, 10.0]},
        )
