"""Deterministic namespace, world-pose, and RViz identity checks."""

from pathlib import Path


ROOT = Path(__file__).parents[1]
WORLD = ROOT / 'worlds' / 'epuck_d500_two_world_large.wbt'


def test_saved_world_assigns_expected_robot_poses():
    """The saved Webots world is the authoritative robot identity source."""
    text = WORLD.read_text()
    assert 'translation 17 0 0.001' in text
    assert 'name "robot1"' in text
    assert 'translation -18.94 0 0.001' in text
    assert 'name "robot2"' in text
    assert text.index('translation 17 0 0.001') < text.index('name "robot1"')
    assert text.index('translation -18.94 0 0.001') < text.index('name "robot2"')


def test_launch_controller_and_frames_preserve_robot_identity():
    """Controller names, namespaces, and frame prefixes remain paired."""
    launch = (ROOT / 'launch' / 'two_robots_namespaced_launch.py').read_text()
    assert "for robot_name in ('robot1', 'robot2')" in launch
    assert "robot_name=robot_name" in launch
    assert "namespace=robot_name" in launch
    assert '<frameName>{robot_name}/d500_lidar</frameName>' in launch
    assert "f'{robot_name}/'" in launch


def test_slam_nav_and_rviz_topics_are_not_crossed():
    """Each displayed and consumed topic has the matching robot namespace."""
    rviz = (ROOT / 'resource' / 'cooperative_manual_exploration.rviz').read_text()
    for robot in ('robot1', 'robot2'):
        assert f'/{robot}/shared_map' in rviz
        assert f'/{robot}/global_costmap/costmap' in rviz
        assert f'/{robot}/local_costmap/costmap' in rviz
        assert f'/{robot}/scan_d500_fixed' in rviz
    assert 'Robot1 Global Costmap' in rviz
    assert 'Robot2 Global Costmap' in rviz


def test_robot_specific_slam_parameters_use_own_scan_and_frames():
    """SLAM instances cannot silently consume the peer scan or TF tree."""
    for robot in ('robot1', 'robot2'):
        text = (ROOT / 'resource' /
                f'slam_toolbox_{robot}_teammate_filtered.yaml').read_text()
        assert f'odom_frame: {robot}/odom' in text
        assert f'map_frame: {robot}/map' in text
        assert f'base_frame: {robot}/base_footprint' in text
        assert f'scan_topic: /{robot}/scan_d500_slam' in text
        peer = 'robot2' if robot == 'robot1' else 'robot1'
        assert f'/{peer}/scan_d500_slam' not in text
