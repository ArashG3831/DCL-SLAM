"""Regression coverage for the unknown-pose observer world override."""

from pathlib import Path


LAUNCH = Path(__file__).resolve().parents[1] / 'launch'


def test_unknown_pose_launch_passes_optional_world_path_to_namespaced_stack():
    source = (LAUNCH / 'two_robots_unknown_pose_mapping_launch.py').read_text(
        encoding='utf-8')
    assert "DeclareLaunchArgument('world_path', default_value='')" in source
    assert "'world_path': launch_world_path" in source


def test_unknown_pose_forensic_launch_stages_supervisor_and_map_export():
    source = (LAUNCH / 'two_robots_unknown_pose_mapping_launch.py').read_text(
        encoding='utf-8')
    assert 'stage_forensic_world' in source
    assert 'ForensicGroundTruthSupervisor' in source
    assert "DeclareLaunchArgument('enable_forensic_capture'" in source
    assert "DeclareLaunchArgument('staging_directory'" in source
    assert 'required forensic/maps export path is unavailable' in source


def test_unknown_pose_frontend_keeps_bounded_requestable_keyframe_history():
    source = (LAUNCH.parent / 'my_epuck_project' /
              'unknown_pose_frontend.py').read_text(
        encoding='utf-8')
    assert "declare_parameter('max_keyframes', 64)" in source
    assert 'does not make the history unbounded' in source


def test_ground_truth_observer_has_bounded_clean_finalization_mode():
    source = (LAUNCH.parent / 'my_epuck_project' /
              'cooperative_ground_truth_observer.py').read_text(
                  encoding='utf-8')
    assert "'--max-runtime-s'" in source
    assert 'FORENSIC_SUPERVISOR_COMPLETE' in source


def test_ground_truth_observer_requires_explicit_controller_url_and_runtime_dir():
    source = (LAUNCH.parent / 'my_epuck_project' /
              'cooperative_ground_truth_observer.py').read_text(
                  encoding='utf-8')
    logger = (LAUNCH.parent / 'my_epuck_project' /
              'cooperative_experiment_logger.py').read_text(
                  encoding='utf-8')
    assert "'--controller-url'" in source
    assert 'OBSERVER_CONTROLLER_URL_REQUIRED' in source
    assert "'--runtime-directory'" in source
    assert "'--controller-url', environment['WEBOTS_CONTROLLER_URL']" in logger
    assert "'--runtime-directory', str(forensic_dir / 'runtime')" in logger


def test_unknown_pose_frontend_persists_bounded_shutdown_diagnostics():
    source = (LAUNCH.parent / 'my_epuck_project' /
              'unknown_pose_frontend.py').read_text(encoding='utf-8')
    assert "'DESCRIPTOR_GATE_SURVIVED'" in source
    assert "'CANDIDATE_SELECTION'" in source
    assert "'CROP_REQUEST_PUBLISHED'" in source
    assert "'descriptor_gate_rejection_reason_counts'" in source
    assert "'callback_timing'" in source
    assert "'cpu_samples'" in source
    assert 'node.finalize()' in source


def test_unknown_pose_frontend_records_each_descriptor_gate_reason():
    source = (LAUNCH.parent / 'my_epuck_project' /
              'unknown_pose_frontend.py').read_text(encoding='utf-8')
    for reason in (
            'SIMILARITY_BELOW_GATE', 'MARGIN_BELOW_GATE',
            'KNOWN_FRACTION_BELOW_GATE', 'INSUFFICIENT_CONFIRMATIONS'):
        assert reason in source


def test_unknown_pose_frontend_rechecks_cached_temporal_support_from_timer():
    source = (LAUNCH.parent / 'my_epuck_project' /
              'unknown_pose_frontend.py').read_text(encoding='utf-8')
    assert 'self._compare_peer_descriptors()' in source
    assert 'new_peer_key=key' in source
    assert 'confirmation_window_for_cadence' in source
