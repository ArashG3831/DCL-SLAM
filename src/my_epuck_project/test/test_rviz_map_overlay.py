"""Deterministic tests for the visualization-only map overlay."""

from pathlib import Path
import math

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid

from my_epuck_project.rviz_map_overlay import (
    VIZ_WORLD,
    alias_map,
    alias_transform,
)


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / 'launch' / 'two_robots_decentralized_exploration_launch.py'


def sample_map(frame):
    message = OccupancyGrid()
    message.header.frame_id = frame
    message.header.stamp.sec = 17
    message.header.stamp.nanosec = 29
    message.info.resolution = 0.03
    message.info.width = 2
    message.info.height = 2
    message.info.origin = Pose()
    message.info.origin.position.x = -1.25
    message.info.origin.position.y = 2.5
    message.data = [-1, 0, 100, 50]
    return message


def test_both_local_maps_are_relayed_with_aliased_frames():
    robot1 = alias_map(sample_map('robot1/map'), 'viz/robot1_map')
    robot2 = alias_map(sample_map('robot2/map'), 'viz/robot2_map')
    assert robot1.header.frame_id == 'viz/robot1_map'
    assert robot2.header.frame_id == 'viz/robot2_map'
    assert robot1.header.stamp == sample_map('robot1/map').header.stamp
    assert robot1.info == sample_map('robot1/map').info
    assert list(robot1.data) == [-1, 0, 100, 50]
    assert list(robot2.data) == [-1, 0, 100, 50]


def test_offsets_are_applied_only_to_visualization_alias_transforms():
    transform = alias_transform('viz/robot2_map', 25.0, -3.0, 0.0,
                                math.pi / 2.0)
    assert transform.header.frame_id == VIZ_WORLD
    assert transform.child_frame_id == 'viz/robot2_map'
    assert transform.transform.translation.x == 25.0
    assert transform.transform.translation.y == -3.0
    assert math.isclose(transform.transform.rotation.z, math.sqrt(0.5))
    assert math.isclose(transform.transform.rotation.w, math.sqrt(0.5))
    assert 'robot1/map' not in transform.header.frame_id
    assert 'robot2/map' not in transform.header.frame_id
    assert 'robot1/map' not in transform.child_frame_id
    assert 'robot2/map' not in transform.child_frame_id


def test_launch_contains_optional_overlay_and_no_real_frame_aliases():
    source = LAUNCH.read_text(encoding='utf-8')
    assert "executable='rviz_map_overlay'" in source
    assert "'viz_world_frame': 'viz/world'" in source
    assert "'robot1_map_topic': '/robot1/map'" in source
    assert "'robot2_map_topic': '/robot2/map'" in source
    assert "'robot1_output_topic': '/viz/robot1_map'" in source
    assert "'robot2_output_topic': '/viz/robot2_map'" in source
    assert 'launch_visualization_overlay' in source
    assert 'viz/world -> robot1/map' not in source
    assert 'viz/world -> robot2/map' not in source
    assert 'robot1/map -> robot2/map' not in source
    assert 'robot2/map -> robot1/map' not in source
