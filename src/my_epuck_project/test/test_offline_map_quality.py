import json

from my_epuck_project.offline_map_quality import map_quality_report


def test_map_quality_uses_existing_artifact(tmp_path):
    payload = {"valid": True, "occupied_iou": 0.9}
    (tmp_path / "map_quality.json").write_text(
        json.dumps(payload), encoding="utf-8")
    result = map_quality_report(tmp_path)
    assert result["available"]
    assert result["source"] == "map_quality.json"
    assert result["metrics"] == payload


def test_map_quality_does_not_invent_missing_reference(tmp_path):
    result = map_quality_report(tmp_path)
    assert not result["available"]
    assert "map-quality artifact or reference map" in result["missing"]


def test_map_quality_preserves_missing_reference_when_world_maps_are_absent(
        tmp_path):
    result = map_quality_report(
        tmp_path, {"source_world_path": str(tmp_path / "missing.wbt")})
    assert not result["available"]
