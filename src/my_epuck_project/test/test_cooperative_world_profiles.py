"""Focused validation for cooperative world/profile selection."""

from dataclasses import replace
import math
from pathlib import Path
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import ExecuteProcess
from webots_ros2_driver.webots_launcher import WebotsLauncher
import yaml

from my_epuck_project.cooperative_profiles import (
    PROFILE_SETTINGS,
    manual_rviz_path,
    parse_world,
    profile,
    relative_transform,
)


PACKAGE = Path(__file__).resolve().parents[1]
WORLDS = PACKAGE / 'worlds'
LAUNCH = PACKAGE / 'launch'


def selected(name):
    return profile(name, WORLDS)


def point_to_box_distance(point, obstacle):
    """Distance from an X-Y point to one rotated SolidBox footprint."""
    px, py = point
    cx, cy, _ = obstacle['translation']
    width, height, _ = obstacle['size']
    angle = obstacle['rotation'][3]
    cosine, sine = math.cos(angle), math.sin(angle)
    dx, dy = px - cx, py - cy
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    outside_x = max(abs(local_x) - width / 2.0, 0.0)
    outside_y = max(abs(local_y) - height / 2.0, 0.0)
    return math.hypot(outside_x, outside_y)


def test_large_world_exact_saved_geometry_and_devices():
    value = selected('large')
    metadata = value['world_metadata']
    assert metadata['dimensions'] == (40.0, 10.0)
    assert metadata['wall_height'] == 0.3
    assert metadata['solid_box_count'] == 21
    assert {item['type'] for item in metadata['obstacles']} == {'SolidBox'}
    assert metadata['robot_order'] == ('robot1', 'robot2')
    expected = {
        'robot1': ((17.0, 0.0, 0.001), (0.0, 0.0, 1.0, math.pi / 2)),
        'robot2': ((-18.94, 0.0, 0.001), (0.0, 0.0, 1.0, math.pi / 2)),
    }
    content = Path(value['world_path']).read_text(encoding='utf-8')
    assert content.count('Pi-puck {') == 2
    assert content.count('boundingObject Cylinder {') == 2
    assert 'EXTERNPROTO "webots://projects/objects/solids/protos/SolidBox.proto"' in content
    assert 'EXTERNPROTO "http' not in content
    for name, robot in metadata['robots'].items():
        assert (robot.translation, robot.rotation) == expected[name]
        assert robot.controller == '<extern>'
        assert robot.window == '<none>'
        assert robot.lidar_name == 'd500_lidar'
        assert robot.lidar_resolution == 720
        assert math.isclose(robot.lidar_field_of_view, 2.0 * math.pi)
        assert robot.lidar_minimum_range == 0.03
        assert robot.lidar_maximum_range == 12.0
        assert (robot.camera_width, robot.camera_height) == (640, 480)


def test_large_world_start_clearance_contact_and_nonintersection():
    metadata = selected('large')['world_metadata']
    robots = metadata['robots']
    separation = math.dist(
        robots['robot1'].translation[:2],
        robots['robot2'].translation[:2],
    )
    assert math.isclose(separation, 35.94, abs_tol=1e-12)
    assert separation > 2.0 * metadata['robots']['robot1'].lidar_maximum_range
    for robot in robots.values():
        assert robot.translation[2] == 0.001
        nearest = min(
            point_to_box_distance(robot.translation[:2], obstacle)
            for obstacle in metadata['obstacles']
        )
        assert nearest > 0.11
        assert 20.0 - abs(robot.translation[0]) > 0.11
        assert 5.0 - abs(robot.translation[1]) > 0.11


def test_project_camera_overlays_hidden_and_viewpoint_preserved():
    metadata = selected('large')['world_metadata']
    assert all(math.isfinite(v) for v in metadata['viewpoint']['orientation'])
    assert all(math.isfinite(v) for v in metadata['viewpoint']['position'])
    assert len(metadata['viewpoint']['orientation']) == 4
    assert len(metadata['viewpoint']['position']) == 3
    project = (WORLDS / '.epuck_d500_two_world_large.wbproj').read_text()
    for robot in ('robot1', 'robot2'):
        match = re.search(
            rf'renderingDevicePerspectives: {robot}:camera;([^;\n]+);[^\n]+',
            project)
        assert match and match.group(1).strip() == '0'
    assert 'centralWidgetVisible: 1' in project


def test_webots_temporary_copy_keeps_large_world_project_settings(
        monkeypatch, tmp_path):
    """WebotsLauncher's selected-world copy retains the hidden overlays."""
    monkeypatch.setenv('TMPDIR', str(tmp_path))
    monkeypatch.setattr(tempfile, 'tempdir', str(tmp_path))
    monkeypatch.setattr(ExecuteProcess, 'execute', lambda self, context: None)
    launcher = WebotsLauncher(
        world=str(WORLDS / 'epuck_d500_two_world_large.wbt'),
        gui=True,
        mode='fast',
        port='23101',
    )
    launcher.execute(LaunchContext())
    generated = Path(launcher._WebotsLauncher__world_copy.name)
    generated_project = generated.with_name(f'.{generated.stem}.wbproj')
    try:
        assert generated.parent == tmp_path
        metadata = parse_world(generated)
        assert all(
            robot.window == '<none>'
            for robot in metadata['robots'].values()
        )
        project = generated_project.read_text(encoding='utf-8')
        for robot in ('robot1', 'robot2'):
            match = re.search(
                rf'renderingDevicePerspectives: {robot}:camera;([^;\n]+);[^\n]+',
                project)
            assert match and match.group(1).strip() == '0'
    finally:
        launcher._WebotsLauncher__world_copy.close()
        generated.unlink(missing_ok=True)
        generated_project.unlink(missing_ok=True)


def test_world_derived_relative_transform_uses_full_planar_geometry():
    small = selected('small')['world_metadata']
    large = selected('large')['world_metadata']
    assert large['initial_separation_m'] > 2.0 * 12.0
    assert math.isclose(large['planar_yaws']['robot1'],
                        large['planar_yaws']['robot2'], abs_tol=1e-12)
    robot1 = large['robots']['robot1']
    robot2 = large['robots']['robot2']
    shift = (4.0, -2.0, 0.0)
    shifted1 = replace(robot1, translation=tuple(
        a + b for a, b in zip(robot1.translation, shift)))
    shifted2 = replace(robot2, translation=tuple(
        a + b for a, b in zip(robot2.translation, shift)))
    assert relative_transform(shifted1, shifted2) == relative_transform(
        robot1, robot2)
    moved2 = replace(
        robot2,
        translation=(
            robot2.translation[0] + 0.02,
            robot2.translation[1],
            robot2.translation[2],
        ),
    )
    assert not all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip(
        relative_transform(robot1, moved2),
        relative_transform(robot1, robot2),
    ))


def test_large_world_relative_transform_composes_back_to_robot2():
    metadata = selected('large')['world_metadata']
    robot1 = metadata['initial_world_transforms']['robot1']
    robot2 = metadata['initial_world_transforms']['robot2']
    recovered = __import__('my_epuck_project.cooperative_profiles', fromlist=['compose_transform']).compose_transform(
        robot1, metadata['relative_transform'])
    assert all(math.isclose(a, b, abs_tol=1e-9)
               for a, b in zip(recovered, robot2))


def test_profile_resolutions_thresholds_and_launch_defaults():
    small = PROFILE_SETTINGS['small']
    large = PROFILE_SETTINGS['large']
    assert (
        large['slam_resolution'],
        large['fusion_resolution'],
        large['global_costmap_resolution'],
        large['local_costmap_resolution'],
    ) == (0.03, 0.03, 0.03, 0.02)
    assert (
        small['slam_resolution'],
        small['fusion_resolution'],
        small['global_costmap_resolution'],
        small['local_costmap_resolution'],
    ) == (0.01, 0.01, 0.005, 0.005)
    assert (small['minimum_frontier_cells'], 0.01) == (5, 0.01)
    assert (large['minimum_frontier_cells'], 0.03) == (2, 0.03)
    assert small['minimum_known_cell_gain_for_activity'] == 5
    assert large['minimum_known_cell_gain_for_activity'] == 1
    observed = (
        LAUNCH / 'two_robots_observed_continuous_exploration_launch.py'
    ).read_text()
    single = (
        LAUNCH / 'two_robots_cooperative_single_goal_launch.py'
    ).read_text()
    baseline = (LAUNCH / 'robot_d500_launch_port23000.py').read_text()
    assert "'world_profile',\n            default_value='large'" in observed
    assert "'world_profile',\n            default_value='small'" in single
    assert "default_value='epuck_d500_world.wbt'" in baseline


def test_source_and_installed_worlds_agree_after_build():
    installed = (
        Path(get_package_share_directory('my_epuck_project')) / 'worlds')
    for name in ('small', 'large'):
        source = selected(name)
        installed_value = profile(name, installed)
        assert source['world_metadata']['sha256'] == (
            installed_value['world_metadata']['sha256'])


def test_large_rviz_keeps_displays_and_selects_useful_orbit_camera():
    resource = PACKAGE / 'resource'
    small_path = Path(manual_rviz_path(selected('small'), resource))
    large_path = Path(manual_rviz_path(selected('large'), resource))
    small = yaml.safe_load(small_path.read_text())
    large = yaml.safe_load(large_path.read_text())
    small_manager = small['Visualization Manager']
    large_manager = large['Visualization Manager']
    assert small_manager['Displays'] == large_manager['Displays']
    view = large_manager['Views']['Current']
    assert view['Class'] == 'rviz_default_plugins/Orbit'
    assert view['Target Frame'] == 'shared_map'
    assert view['Distance'] == 45
    assert view['Focal Point']['X'] == 18
    assert 'TopDownOrtho' not in large_path.read_text()
