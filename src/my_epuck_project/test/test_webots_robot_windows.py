"""Verify two-robot Webots worlds permanently disable Robot Windows."""

import re
import tempfile
from pathlib import Path

from launch import LaunchContext
from launch.actions import ExecuteProcess
from webots_ros2_driver.webots_launcher import WebotsLauncher


PACKAGE = Path(__file__).resolve().parents[1]
WORLDS = PACKAGE / 'worlds'
TWO_ROBOT_WORLDS = (
    'epuck_d500_two_world.wbt',
    'epuck_d500_two_world_both_active.wbt',
    'epuck_d500_two_world_robot1_active.wbt',
    'epuck_d500_two_world_robot2_active.wbt',
    'epuck_d500_two_world_teammate_visible.wbt',
    'epuck_d500_two_world_large.wbt',
)


def robot_block(content, robot):
    """Return the brace-balanced E-puck block containing a robot name."""
    for match in re.finditer(r'(?m)^E-puck\s*\{', content):
        depth = 0
        for index in range(match.end() - 1, len(content)):
            if content[index] == '{':
                depth += 1
            elif content[index] == '}':
                depth -= 1
                if depth == 0:
                    block = content[match.start():index + 1]
                    if f'name "{robot}"' in block:
                        return block
                    break
    raise AssertionError(f'missing E-puck instance {robot}')


def assert_robot_windows_disabled(content):
    """Require an explicit disabled window field on both robot instances."""
    for robot in ('robot1', 'robot2'):
        block = robot_block(content, robot)
        assert len(re.findall(r'(?m)^\s*window\s+"<none>"\s*$', block)) == 1


def assert_camera_overlays_hidden(content):
    """Require both Webots camera rendering-device overlays to be hidden."""
    for robot in ('robot1', 'robot2'):
        matches = re.findall(
            rf'renderingDevicePerspectives: {robot}:camera;([^;\n]+);[^\n]+',
            content)
        assert len(matches) == 1
        assert matches[0].strip() == '0'


def test_every_persistent_two_robot_world_disables_robot_windows():
    """Every committed two-e-puck world disables both GUI overlays."""
    for name in TWO_ROBOT_WORLDS:
        assert_robot_windows_disabled(
            (WORLDS / name).read_text(encoding='utf-8'))
        project = WORLDS / f'.{Path(name).stem}.wbproj'
        assert_camera_overlays_hidden(project.read_text(encoding='utf-8'))


def test_webots_generated_temporary_world_disables_both_windows(
        monkeypatch, tmp_path):
    """Inspect WebotsLauncher's real temporary copy before process start."""
    monkeypatch.setenv('TMPDIR', str(tmp_path))
    monkeypatch.setattr(tempfile, 'tempdir', str(tmp_path))
    monkeypatch.setattr(ExecuteProcess, 'execute', lambda self, context: None)
    launcher = WebotsLauncher(
        world=str(WORLDS / 'epuck_d500_two_world_teammate_visible.wbt'),
        gui=True,
        mode='fast',
        port='23100',
    )
    launcher.execute(LaunchContext())
    generated = Path(launcher._WebotsLauncher__world_copy.name)
    generated_project = generated.with_name(f'.{generated.stem}.wbproj')
    try:
        assert generated.parent == tmp_path
        assert_robot_windows_disabled(
            generated.read_text(encoding='utf-8'))
        assert_camera_overlays_hidden(
            generated_project.read_text(encoding='utf-8'))
    finally:
        launcher._WebotsLauncher__world_copy.close()
        generated.unlink(missing_ok=True)
        generated_project.unlink(missing_ok=True)
