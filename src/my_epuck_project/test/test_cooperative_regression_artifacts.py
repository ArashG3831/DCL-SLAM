from my_epuck_project.cooperative_regression import shared_map_exports_available


def test_no_handoff_probe_does_not_require_shared_map_exports(tmp_path):
    assert not shared_map_exports_available(tmp_path)


def test_shared_map_exports_require_both_robot_files(tmp_path):
    (tmp_path / 'robot1_final_shared_map.npz').write_bytes(b'placeholder')
    assert not shared_map_exports_available(tmp_path)
    (tmp_path / 'robot2_final_shared_map.npz').write_bytes(b'placeholder')
    assert shared_map_exports_available(tmp_path)
