"""Focused validation for cooperative world/profile selection."""

from dataclasses import replace
import math
import hashlib
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
    profile_for_world,
    relative_transform,
)


PACKAGE = Path(__file__).resolve().parents[1]
WORLDS = PACKAGE / 'worlds'
LAUNCH = PACKAGE / 'launch'


def selected(name):
    return profile(name, WORLDS)


def test_large_unknown_pose_16m_is_the_canonical_far_start_profile():
    value = selected('large_unknown_pose_16m')
    metadata = value['world_metadata']
    assert value['world'] == (
        'epuck_d500_two_world_unknown_pose_16m_dynamic_low_slip_4ms_finite.wbt')
    assert metadata['robot_order'] == ('robot1', 'robot2')
    assert metadata['robots']['robot1'].translation[:2] == (16.0, 0.0)
    assert metadata['robots']['robot2'].translation[:2] == (-1.0, 0.0)
    assert math.isclose(metadata['initial_separation_m'], 17.0, abs_tol=1e-6)
    assert metadata['robots']['robot1'].lidar_maximum_range == 12.0
    assert value['physics_profile'] == 'dynamic_low_slip_4ms_finite'


def test_far_start_20ms_scan_matching_profile_is_explicit_and_17m():
    value = selected('large_unknown_pose_far_start_20ms_scan_matching')
    metadata = value['world_metadata']
    assert value['world'] == (
        'epuck_d500_two_world_unknown_pose_far_start_dynamic_low_slip_20ms_finite.wbt')
    assert value['physics_profile'] == 'dynamic_low_slip_20ms_finite'
    assert value['unknown_initial_pose'] is True
    assert metadata['robots']['robot1'].translation[:2] == (16.0, 0.0)
    assert metadata['robots']['robot2'].translation[:2] == (-1.0, 0.0)
    assert math.isclose(metadata['initial_separation_m'], 17.0, abs_tol=1e-6)
    assert metadata['robots']['robot1'].planar_yaw == (
        metadata['robots']['robot2'].planar_yaw)
    content = Path(value['world_path']).read_text(encoding='utf-8')
    assert 'basicTimeStep 20' in content
    assert 'basicTimeStep 4' not in content
    assert content.count('noise 0') >= 2
    assert content.count('resolution -1') >= 2
    assert value['slam_runtime_parameters']['use_scan_matching'] is True
    assert value['slam_runtime_parameters']['do_loop_closing'] is False
    assert value['slam_runtime_parameters']['use_response_expansion'] is False


def test_close_start_profile_matches_manual_saved_world_poses():
    value = selected('large_unknown_pose_close_start')
    metadata = value['world_metadata']
    assert value['world'] == (
        'epuck_d500_two_world_unknown_pose_close_start_dynamic_low_slip_4ms_finite.wbt')
    assert metadata['robot_order'] == ('robot1', 'robot2')
    assert metadata['robots']['robot1'].translation == (16.0, 0.0, 0.001)
    assert metadata['robots']['robot2'].translation == (
        18.77, -1.6718e-17, 0.001)
    assert metadata['robots']['robot1'].rotation == (
        0.0, 0.0, 1.0, 1.5707963267948966)
    assert metadata['robots']['robot2'].rotation == (
        0.0, 0.0, 1.0, 1.5707963267948966)
    assert math.isclose(metadata['initial_separation_m'], 2.77, abs_tol=1e-9)
    content = Path(value['world_path']).read_text(encoding='utf-8')
    assert 'basicTimeStep 4' in content
    assert content.count('noise 0') >= 2
    assert content.count('resolution -1') >= 2
    assert content.count('E-puck {') == 2


def test_close_start_20ms_profile_preserves_close_start_geometry():
    value = selected('large_unknown_pose_close_start_20ms')
    reference = selected('large_unknown_pose_close_start')
    assert value['world'] == (
        'epuck_d500_two_world_unknown_pose_close_start_dynamic_low_slip_20ms_finite.wbt')
    assert value['physics_profile'] == 'dynamic_low_slip_20ms_finite'
    # The 20 ms fixture intentionally changes Robot 2's pose to keep the
    # robots 2.77 m apart; the 4 ms saved world places Robot 2 at x=18.77.
    assert value['world_metadata']['robots']['robot1'] == (
        reference['world_metadata']['robots']['robot1'])
    assert value['world_metadata']['robots']['robot2'].translation == (
        16.0, 2.77, 0.001)
    assert value['world_metadata']['robots']['robot2'].rotation == (
        reference['world_metadata']['robots']['robot2'].rotation)
    assert math.isclose(
        value['world_metadata']['initial_separation_m'], 2.77, abs_tol=1e-9)
    reference_text = Path(reference['world_path']).read_text(encoding='utf-8')
    actual_text = Path(value['world_path']).read_text(encoding='utf-8')
    expected_text = reference_text.replace(
        'basicTimeStep 4', 'basicTimeStep 20', 1).replace(
            'translation 18.77 -1.6718e-17 0.001',
            'translation 16 2.77 0.001', 1)
    assert actual_text == expected_text
    assert 'CFM 0.00001' in actual_text
    assert 'ERP 0.2' in actual_text
    assert 'randomSeed 20260818' in actual_text
    assert actual_text.count('noise 0') >= 2
    assert actual_text.count('resolution -1') >= 2


def test_profile_for_world_requires_matching_path_and_hash(tmp_path):
    source = WORLDS / (
        'epuck_d500_two_world_unknown_pose_16m_dynamic_low_slip_4ms_finite.wbt')
    staged_worlds = tmp_path / 'worlds'
    staged_worlds.mkdir()
    staged = staged_worlds / source.name
    staged.write_bytes(source.read_bytes())
    value = profile_for_world(
        'large_unknown_pose_16m', staged_worlds, explicit_world_path=staged)
    assert Path(value['world_path']) == staged.resolve()
    assert value['world_metadata']['sha256'] == hashlib.sha256(
        staged.read_bytes()).hexdigest()
    assert value['world'] != 'epuck_d500_two_world_large_dynamic_low_slip_4ms_finite.wbt'


def test_profile_for_world_rejects_generic_or_conflicting_world_path(tmp_path):
    staged_worlds = tmp_path / 'worlds'
    staged_worlds.mkdir()
    expected = staged_worlds / (
        'epuck_d500_two_world_unknown_pose_16m_dynamic_low_slip_4ms_finite.wbt')
    expected.write_bytes(
        (WORLDS / expected.name).read_bytes())
    generic = staged_worlds / 'epuck_d500_two_world_large_dynamic_low_slip_4ms_finite.wbt'
    generic.write_bytes(
        (WORLDS / 'epuck_d500_two_world_large_dynamic_low_slip_4ms_finite.wbt')
        .read_bytes())
    try:
        profile_for_world(
            'large_unknown_pose_16m', staged_worlds,
            explicit_world_path=generic)
    except ValueError as error:
        assert 'world profile/path mismatch' in str(error)
    else:
        raise AssertionError('conflicting generic world path was accepted')


def test_full_launch_chain_uses_one_profile_path_resolver():
    for name in (
            'two_robots_decentralized_exploration_launch.py',
            'two_robots_frontier_candidates_launch.py',
            'two_robots_teammate_filtered_stack_launch.py',
            'two_robots_teammate_filtered_dual_slam_launch.py'):
        source = (LAUNCH / name).read_text(encoding='utf-8')
        assert 'profile_for_world' in source
        assert 'explicit_world_path=world_path' in source


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


def test_large_world_saved_geometry_and_devices():
    value = selected('large')
    metadata = value['world_metadata']
    assert metadata['dimensions'] == (40.0, 10.0)
    assert metadata['wall_height'] == 0.3
    assert metadata['robot_order'] == ('robot1', 'robot2')
    content = Path(value['world_path']).read_text(encoding='utf-8')
    assert content.count('Pi-puck {') == 2
    assert content.count('boundingObject Cylinder {') == 2
    assert 'EXTERNPROTO "webots://projects/objects/solids/protos/SolidBox.proto"' in content
    assert 'EXTERNPROTO "http' not in content
    for name, robot in metadata['robots'].items():
        assert len(robot.translation) == 3
        assert all(math.isfinite(item) for item in robot.translation)
        assert len(robot.rotation) == 4
        assert all(math.isfinite(item) for item in robot.rotation)
        assert robot.controller == '<extern>'
        assert robot.window == '<none>'
        assert robot.lidar_name == 'd500_lidar'
        assert robot.lidar_resolution == 720
        assert math.isclose(robot.lidar_field_of_view, 2.0 * math.pi)
        assert robot.lidar_minimum_range == 0.03
        assert robot.lidar_maximum_range == 12.0
        assert (robot.camera_width, robot.camera_height) == (640, 480)


def test_large_world_selects_project_local_ideal_encoder_model():
    ideal = selected('large')
    content = Path(ideal['world_path']).read_text(encoding='utf-8')
    proto = (PACKAGE / 'protos' / 'e-puck' / 'E-puck.proto').read_text(
        encoding='utf-8')
    assert ideal['encoder_profile'] == 'webots_ideal_wheel_encoders'
    assert '../protos/e-puck/E-puck.proto' in content
    assert content.count('E-puck {') == 2
    assert proto.count('PositionSensor {') == 2
    assert proto.count('noise 0') == 2
    assert proto.count('resolution -1') == 2
    assert 'kinematic                    FALSE' in proto
    assert 'supervisor                   FALSE' in proto


def test_large_default_selects_reversible_finite_low_slip_world():
    value = selected('large')
    assert value['physics_profile'] == 'dynamic_low_slip_4ms_finite'
    assert value['baseline_world'] == 'epuck_d500_two_world_large.wbt'
    content = Path(value['world_path']).read_text(encoding='utf-8')
    assert 'ThesisRectangleArena.proto' in content
    assert 'basicTimeStep 4' in content
    assert 'CFM 0.00001' in content
    assert 'ERP 0.2' in content
    assert 'optimalThreadCount 1' in content
    assert 'randomSeed 20260818' in content
    assert content.count('wheel_contactMaterial "epuck_wheel"') == 2
    assert content.count('floorContactMaterial "epuck_floor"') == 1
    assert content.count('wallContactMaterial "default"') == 1
    assert 'coulombFriction 10' in content
    assert 'forceDependentSlip 0' in content
    assert 'rollingFriction 0 0 0' in content
    assert 'bounce 0' in content
    assert 'softCFM 0.00001' in content


def test_large_baseline_encoder_model_remains_reproducible():
    baseline = profile('large', WORLDS, ideal_encoder_sensing=False)
    content = Path(baseline['world_path']).read_text(encoding='utf-8')
    assert baseline['encoder_profile'] == (
        'webots_upstream_quantized_wheel_encoders')
    assert baseline['world'].endswith('_baseline.wbt')
    assert 'webots://projects/robots/gctronic/e-puck/protos/E-puck.proto' in content
    assert '../protos/e-puck/E-puck.proto' not in content


def test_ideal_encoder_profile_does_not_add_ground_truth_odom_path():
    ideal_proto = (PACKAGE / 'protos' / 'e-puck' / 'E-puck.proto').read_text(
        encoding='utf-8')
    world = Path(selected('large')['world_path']).read_text(encoding='utf-8')
    launch = (LAUNCH / 'two_robots_namespaced_launch.py').read_text(
        encoding='utf-8')
    assert 'supervisor TRUE' not in ideal_proto
    assert 'supervisor TRUE' not in world
    assert 'supervisor TRUE' not in launch
    assert 'ground_truth_pose' not in launch
    assert 'Supervisor pose' not in launch
    assert 'enable_odom_tf: true' in launch


def test_motion_comparison_worlds_use_same_start_and_explicit_encoder_variants():
    ideal = (WORLDS / 'epuck_motion_characterization.wbt').read_text(
        encoding='utf-8')
    baseline = (WORLDS / 'epuck_motion_characterization_baseline.wbt').read_text(
        encoding='utf-8')
    assert '../protos/e-puck/E-puck.proto' in ideal
    assert 'webots://projects/robots/gctronic/e-puck/protos/E-puck.proto' in baseline
    for content in (ideal, baseline):
        assert 'DEF ROBOT1 E-puck {' in content
        assert 'translation ' in content
        assert 'rotation ' in content


def test_dynamic_low_slip_profile_is_explicit_and_wheel_floor_only():
    """Dynamic profile changes contact determinism without changing the robot stack."""
    world = (WORLDS / 'epuck_motion_characterization_dynamic_low_slip.wbt')
    content = world.read_text(encoding='utf-8')
    assert 'ThesisRectangleArena.proto' in content
    assert 'basicTimeStep 4' in content
    assert 'optimalThreadCount 1' in content
    assert 'randomSeed 20260818' in content
    assert 'floorContactMaterial "epuck_floor"' in content
    assert 'wallContactMaterial "default"' in content
    assert 'material1 "epuck_wheel"' in content
    assert 'material2 "epuck_floor"' in content
    assert 'coulombFriction -1' in content
    assert 'forceDependentSlip 0' in content
    assert 'rollingFriction 0 0 0' in content
    assert 'softCFM 0.00001' in content
    assert 'wheel_contactMaterial "epuck_wheel"' in content
    assert 'kinematic TRUE' not in content


def test_kinematic_ideal_profile_is_separate_and_ground_truth_free():
    """Kinematic fallback selects the PROTO kinematic branch only."""
    world = (WORLDS / 'epuck_motion_characterization_kinematic_ideal.wbt')
    content = world.read_text(encoding='utf-8')
    assert 'kinematic TRUE' in content
    assert 'MotionGroundTruthSupervisor' in content
    assert 'ground_truth_pose' not in content
    assert 'supervisor TRUE' in content
    assert 'wheel_contactMaterial' not in content
    proto = (PACKAGE / 'protos' / 'e-puck' / 'E-puck.proto').read_text(
        encoding='utf-8')
    assert '%< if (!kinematic) { >%' in proto
    assert proto.count('wheel_contactMaterial') >= 3


def test_physics_profile_timestep_variants_are_reproducible():
    """The 4/2/1 ms dynamic sweep is explicit and fixed-seed."""
    for name, timestep in (
            ('epuck_motion_characterization_dynamic_low_slip.wbt', '4'),
            ('epuck_motion_characterization_dynamic_low_slip_finite_4ms.wbt', '4'),
            ('epuck_motion_characterization_dynamic_low_slip_2ms.wbt', '2'),
            ('epuck_motion_characterization_dynamic_low_slip_1ms.wbt', '1')):
        content = (WORLDS / name).read_text(encoding='utf-8')
        assert f'basicTimeStep {timestep}' in content
        assert 'randomSeed 20260818' in content
        assert 'optimalThreadCount 1' in content


def test_large_world_start_clearance_contact_and_nonintersection():
    metadata = selected('large')['world_metadata']
    robots = metadata['robots']
    separation = math.dist(
        robots['robot1'].translation[:2],
        robots['robot2'].translation[:2],
    )
    assert separation >= 0.22
    for robot in robots.values():
        assert robot.translation[2] == 0.001
        if metadata['obstacles']:
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
    large = selected('large')['world_metadata']
    assert math.isfinite(large['initial_separation_m'])
    assert large['initial_separation_m'] >= 0.22
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
