from my_epuck_project.offline_final_comparison import compare_outputs


def test_comparison_is_semantic_and_fail_closed_for_missing_current_fields():
    result = compare_outputs(
        {"coverage": {"known_area_auc_m2_s": 2.0},
         "coverage.known_area_auc_m2_s": 2.0},
        {"coverage": {"known_area_auc_m2_s": 2.0},
         "coverage.known_area_auc_m2_s": 2.0})
    rows = {row["metric"]: row for row in result["rows"]}
    assert rows["coverage"]["status"] == "EXACT_PARITY"
    assert rows["coverage_auc"]["status"] == "EXACT_PARITY"
    assert rows["fairness"]["status"] == "BLOCKED_MISSING_RAW_EVIDENCE"


def test_new_output_without_legacy_oracle_is_explicitly_equivalent():
    result = compare_outputs({"coverage": {}}, {
        "coverage": {"known_area_auc_m2_s": 3.0}})
    row = next(item for item in result["rows"] if item["metric"] ==
               "coverage_auc")
    assert row["status"] == "PRESERVED_BY_EQUIVALENT_OFFLINE_OUTPUT"
