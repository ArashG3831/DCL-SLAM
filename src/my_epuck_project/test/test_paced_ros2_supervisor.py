from pathlib import Path


SOURCE = Path(__file__).parents[1] / 'my_epuck_project' / 'paced_ros2_supervisor.py'
LAUNCH = Path(__file__).parents[1] / 'launch' / 'two_robots_namespaced_launch.py'


def test_fast_supervisor_clock_is_best_effort_depth_one():
    text = SOURCE.read_text(encoding='utf-8')
    assert 'ReliabilityPolicy.BEST_EFFORT' in text
    assert 'DurabilityPolicy.VOLATILE' in text
    assert 'HistoryPolicy.KEEP_LAST' in text
    assert 'depth=1' in text
    assert 'destroy_publisher(stock_publisher)' in text
    assert "create_publisher(Clock, 'clock', clock_qos)" in text


def test_fast_launch_uses_the_paced_supervisor_without_respawn():
    text = LAUNCH.read_text(encoding='utf-8')
    assert "executable='paced_ros2_supervisor'" in text
    assert 'respawn=False' in text
