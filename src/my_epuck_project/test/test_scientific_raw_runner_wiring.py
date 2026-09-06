from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_runner_exposes_opt_in_scientific_raw_capture():
    source = (ROOT / 'my_epuck_project' / 'cooperative_trial_fast.py').read_text()
    assert "--enable-scientific-raw-capture" in source
    assert "enable_scientific_raw_capture:=" in source
    assert "getattr(args, \"enable_scientific_raw_capture\", False)" in source


def test_launch_passes_scientific_raw_capture_to_legacy_logger():
    source = (ROOT / 'launch' /
              'two_robots_decentralized_exploration_launch.py').read_text()
    assert "'enable_scientific_raw_capture'" in source
    assert "DeclareLaunchArgument(\n            'enable_scientific_raw_capture'" in source
