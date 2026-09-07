from my_epuck_project.offline_handoff import reciprocal_verification


def test_reciprocal_handoff_checks_inverse_consistency_and_hash():
    result = reciprocal_verification([
        {"accepted": True, "source_robot_id": "robot1",
         "target_robot_id": "robot2", "transform_se2": [1.0, 0.0, 0.0],
         "evidence_set_hash": "h", "accepted_ros_time_s": 3.0},
        {"accepted": True, "source_robot_id": "robot2",
         "target_robot_id": "robot1", "transform_se2": [-1.0, 0.0, 0.0],
         "evidence_set_hash": "h", "accepted_ros_time_s": 4.0},
    ])
    assert result["status"] == "VERIFIED"
    pair = result["verified_pairs"][0]
    assert pair["inverse_translation_error_m"] == 0.0
    assert pair["inverse_yaw_error_rad"] == 0.0
    assert pair["evidence_hashes_match"]


def test_missing_reciprocal_direction_is_not_zero_error():
    result = reciprocal_verification([{
        "accepted": True, "source_robot_id": "robot1",
        "target_robot_id": "robot2", "transform_se2": [1.0, 2.0, 0.1],
    }])
    assert result["status"] == "MISSING_RECIPROCAL_EVIDENCE"
    assert result["missing_pairs"]
