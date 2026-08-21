"""Authoritative cooperative-world profiles derived from saved Webots worlds."""

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import re
import tempfile


CONSERVATIVE_SCAN_MATCHING_PARAMETERS = {
    'use_scan_matching': True,
    'do_loop_closing': False,
    'correlation_search_space_dimension': 0.12,
    'correlation_search_space_resolution': 0.01,
    'correlation_search_space_smear_deviation': 0.015,
    'distance_variance_penalty': 0.05,
    'angle_variance_penalty': 0.05235987755982989,
    'minimum_distance_penalty': 0.15,
    'minimum_angle_penalty': 0.70,
    'coarse_search_angle_offset': 0.0523596583,
    'coarse_angle_resolution': 0.0174532925,
    'fine_search_angle_offset': 0.0034906585,
    'use_response_expansion': False,
    'map_update_interval': 1.0,
}


PROFILE_SETTINGS = {
    'small': {
        'world': 'epuck_d500_two_world_teammate_visible.wbt',
        'slam_resolution': 0.01,
        'fusion_resolution': 0.01,
        'global_costmap_resolution': 0.005,
        'local_costmap_resolution': 0.005,
        'minimum_frontier_cells': 5,
        'minimum_known_cell_gain_for_activity': 5,
        'coverage_attribution_resolution': 0.01,
        'map_comparison_shift_window': 3,
        'rviz': 'cooperative_manual_exploration.rviz',
        'startup_timeout': 120.0,
        'mission_timeout': 240.0,
    },
    'large': {
        'world': 'epuck_d500_two_world_large_dynamic_low_slip_4ms_finite.wbt',
        'baseline_world': 'epuck_d500_two_world_large.wbt',
        'physics_profile': 'dynamic_low_slip_4ms_finite',
        'slam_resolution': 0.03,
        'fusion_resolution': 0.03,
        'global_costmap_resolution': 0.03,
        'local_costmap_resolution': 0.02,
        'minimum_frontier_cells': 2,
        'minimum_known_cell_gain_for_activity': 1,
        'coverage_attribution_resolution': 0.03,
        'map_comparison_shift_window': 1,
        'rviz': 'cooperative_manual_exploration.rviz',
        'rviz_view': {
            'distance': 45,
            'focal_x': 18,
            'focal_y': 0,
            'focal_z': 0,
        },
        # Windows Webots over WSL can take about three minutes to publish
        # the first stable clock and finish both controller spawners.
        'startup_timeout': 300.0,
        'mission_timeout': 1800.0,
    },
    'large_unknown_pose': {
        'world': 'epuck_d500_two_world_large_unknown_pose_dynamic_low_slip_4ms_finite.wbt',
        'baseline_world': 'epuck_d500_two_world_large.wbt',
        'physics_profile': 'dynamic_low_slip_4ms_finite',
        'slam_resolution': 0.03,
        'fusion_resolution': 0.03,
        'global_costmap_resolution': 0.03,
        'local_costmap_resolution': 0.02,
        'minimum_frontier_cells': 2,
        'minimum_known_cell_gain_for_activity': 1,
        'coverage_attribution_resolution': 0.03,
        'map_comparison_shift_window': 1,
        'rviz': 'cooperative_manual_exploration.rviz',
        'rviz_view': {
            'distance': 45,
            'focal_x': 0,
            'focal_y': 0,
            'focal_z': 0,
        },
        'startup_timeout': 300.0,
        'mission_timeout': 1800.0,
    },
    'large_unknown_pose_16m': {
        'world': 'epuck_d500_two_world_unknown_pose_16m_dynamic_low_slip_4ms_finite.wbt',
        'baseline_world': 'epuck_d500_two_world_large.wbt',
        'physics_profile': 'dynamic_low_slip_4ms_finite',
        'slam_resolution': 0.03,
        'fusion_resolution': 0.03,
        'global_costmap_resolution': 0.03,
        'local_costmap_resolution': 0.02,
        'minimum_frontier_cells': 2,
        'minimum_known_cell_gain_for_activity': 1,
        'coverage_attribution_resolution': 0.03,
        'map_comparison_shift_window': 1,
        'rviz': 'cooperative_manual_exploration.rviz',
        'rviz_view': {
            'distance': 45,
            'focal_x': 7.5,
            'focal_y': 0,
            'focal_z': 0,
        },
        'startup_timeout': 300.0,
        'mission_timeout': 1800.0,
    },
    'large_unknown_pose_close_start': {
        'world': 'epuck_d500_two_world_unknown_pose_close_start_dynamic_low_slip_4ms_finite.wbt',
        'baseline_world': 'epuck_d500_two_world_large.wbt',
        'physics_profile': 'dynamic_low_slip_4ms_finite',
        'slam_resolution': 0.03,
        'fusion_resolution': 0.03,
        'global_costmap_resolution': 0.03,
        'local_costmap_resolution': 0.02,
        'minimum_frontier_cells': 2,
        'minimum_known_cell_gain_for_activity': 1,
        'coverage_attribution_resolution': 0.03,
        'map_comparison_shift_window': 1,
        'rviz': 'cooperative_manual_exploration.rviz',
        'rviz_view': {
            'distance': 45,
            'focal_x': 17.4,
            'focal_y': 0,
            'focal_z': 0,
        },
        'startup_timeout': 300.0,
        'mission_timeout': 1800.0,
    },
    'large_unknown_pose_close_start_20ms': {
        'world': 'epuck_d500_two_world_unknown_pose_close_start_dynamic_low_slip_20ms_finite.wbt',
        'baseline_world': 'epuck_d500_two_world_large.wbt',
        'physics_profile': 'dynamic_low_slip_20ms_finite',
        'slam_resolution': 0.03,
        'fusion_resolution': 0.03,
        'global_costmap_resolution': 0.03,
        'local_costmap_resolution': 0.02,
        'minimum_frontier_cells': 2,
        'minimum_known_cell_gain_for_activity': 1,
        'coverage_attribution_resolution': 0.03,
        'map_comparison_shift_window': 1,
        'rviz': 'cooperative_manual_exploration.rviz',
        'rviz_view': {
            'distance': 45,
            'focal_x': 17.4,
            'focal_y': 0,
            'focal_z': 0,
        },
        'startup_timeout': 300.0,
        'mission_timeout': 1800.0,
    },
    'large_unknown_pose_close_start_20ms_scan_matching': {
        'world': 'epuck_d500_two_world_unknown_pose_close_start_dynamic_low_slip_20ms_finite.wbt',
        'baseline_world': 'epuck_d500_two_world_large.wbt',
        'physics_profile': 'dynamic_low_slip_20ms_finite',
        'slam_resolution': 0.03,
        'fusion_resolution': 0.03,
        'global_costmap_resolution': 0.03,
        'local_costmap_resolution': 0.02,
        'minimum_frontier_cells': 2,
        'minimum_known_cell_gain_for_activity': 1,
        'coverage_attribution_resolution': 0.03,
        'map_comparison_shift_window': 1,
        'rviz': 'cooperative_manual_exploration.rviz',
        'rviz_view': {
            'distance': 45,
            'focal_x': 17.4,
            'focal_y': 0,
            'focal_z': 0,
        },
        'startup_timeout': 300.0,
        'mission_timeout': 300.0,
        'unknown_initial_pose': True,
        'slam_runtime_parameters': dict(
            CONSERVATIVE_SCAN_MATCHING_PARAMETERS),
    },
}


@dataclass(frozen=True)
class RobotStart:
    """One robot pose and retained device metadata from a saved world."""

    name: str
    translation: tuple
    rotation: tuple
    controller: str
    window: str
    lidar_name: str
    lidar_resolution: int
    lidar_field_of_view: float
    lidar_minimum_range: float
    lidar_maximum_range: float
    camera_width: int
    camera_height: int

    @property
    def planar_yaw(self):
        return axis_angle_yaw(self.rotation)


def _blocks(content, node_type):
    expression = re.compile(
        rf'(?m)^[ \t]*{re.escape(node_type)}\s*\{{')
    for match in expression.finditer(content):
        depth = 0
        for index in range(match.end() - 1, len(content)):
            if content[index] == '{':
                depth += 1
            elif content[index] == '}':
                depth -= 1
                if depth == 0:
                    yield content[match.start():index + 1]
                    break


def _field(block, name, default=None):
    match = re.search(
        rf'(?m)^\s*{re.escape(name)}\s+([^\r\n#]+?)\s*$',
        block,
    )
    if match:
        return match.group(1).strip()
    if default is not None:
        return default
    raise ValueError(f'missing {name} field')


def _numbers(value):
    return tuple(float(item) for item in value.split())


def _quoted(value):
    if len(value) >= 2 and value[0] == value[-1] == '"':
        return value[1:-1]
    raise ValueError(f'expected quoted Webots field, got {value!r}')


def _robot(block):
    lidar_blocks = list(_blocks(block, 'Lidar'))
    if len(lidar_blocks) != 1:
        raise ValueError('each robot must contain exactly one Lidar')
    lidar = lidar_blocks[0]
    return RobotStart(
        name=_quoted(_field(block, 'name')),
        translation=_numbers(_field(block, 'translation')),
        rotation=_numbers(_field(block, 'rotation', '0 0 1 0')),
        controller=_quoted(_field(block, 'controller')),
        # Older proven worlds rely on Webots' default Robot Window setting.
        # Large-world validation still requires its explicit "<none>" field.
        window=_quoted(_field(block, 'window', '"<default>"')),
        lidar_name=_quoted(_field(lidar, 'name')),
        lidar_resolution=int(_field(lidar, 'horizontalResolution')),
        lidar_field_of_view=float(_field(lidar, 'fieldOfView')),
        lidar_minimum_range=float(_field(lidar, 'minRange')),
        lidar_maximum_range=float(_field(lidar, 'maxRange')),
        camera_width=int(_field(block, 'camera_width')),
        camera_height=int(_field(block, 'camera_height')),
    )


def normalize_angle(value):
    """Normalize an angle to [-pi, pi)."""
    if not math.isfinite(value):
        raise ValueError(f'angle must be finite, got {value!r}')
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def axis_angle_yaw(rotation, tolerance=1e-9):
    """Convert a Webots axis-angle rotation to a planar yaw.

    Webots stores rotations as axis-angle.  A ground robot may only have a
    rotation about the Z axis; accepting a tilted axis would make the planar
    relative transform ambiguous, so reject it precisely.
    """
    if len(rotation) != 4 or not all(math.isfinite(v) for v in rotation):
        raise ValueError(f'rotation must contain four finite values: {rotation!r}')
    axis_x, axis_y, axis_z, angle = rotation
    axis_norm = math.sqrt(axis_x**2 + axis_y**2 + axis_z**2)
    if axis_norm <= tolerance:
        if abs(angle) <= tolerance:
            return 0.0
        raise ValueError(f'rotation axis is zero for nonzero angle: {rotation!r}')
    if math.hypot(axis_x, axis_y) > tolerance * axis_norm:
        raise ValueError(f'robot rotation is not planar Z-axis rotation: {rotation!r}')
    return normalize_angle(angle * axis_z / axis_norm)


def relative_transform(source, target):
    """Return target in source coordinates on the world's X-Y ground plane."""
    source_yaw = source.planar_yaw
    target_yaw = target.planar_yaw
    delta_x = target.translation[0] - source.translation[0]
    delta_y = target.translation[1] - source.translation[1]
    cosine = math.cos(source_yaw)
    sine = math.sin(source_yaw)
    return (
        cosine * delta_x + sine * delta_y,
        -sine * delta_x + cosine * delta_y,
        normalize_angle(target_yaw - source_yaw),
    )


def compose_transform(source, relative):
    """Compose a source world pose with a target-in-source SE(2) pose."""
    sx, sy, syaw = source
    tx, ty, tyaw = relative
    cosine, sine = math.cos(syaw), math.sin(syaw)
    return (
        sx + cosine * tx - sine * ty,
        sy + sine * tx + cosine * ty,
        normalize_angle(syaw + tyaw),
    )


def invert_transform(transform):
    """Return the inverse of an SE(2) transform."""
    x, y, yaw = transform
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return (
        -cosine * x - sine * y,
        sine * x - cosine * y,
        normalize_angle(-yaw),
    )


def parse_world(path):
    """Parse the profile fields that must stay consistent across the stack."""
    path = Path(path).resolve()
    content = path.read_text(encoding='utf-8')
    arenas = list(_blocks(content, 'RectangleArena'))
    if not arenas:
        arenas = list(_blocks(content, 'ThesisRectangleArena'))
    viewpoints = list(_blocks(content, 'Viewpoint'))
    robots = [_robot(block) for block in _blocks(content, 'E-puck')]
    if len(arenas) != 1 or len(viewpoints) != 1:
        raise ValueError('world must contain one RectangleArena and Viewpoint')
    names = [robot.name for robot in robots]
    if len(names) != len(set(names)):
        raise ValueError(f'world contains duplicate robot names: {names!r}')
    if set(names) != {'robot1', 'robot2'} or len(names) != 2:
        raise ValueError('world must contain exactly robot1 and robot2')
    arena = arenas[0]
    viewpoint = viewpoints[0]
    by_name = {robot.name: robot for robot in robots}
    obstacles = []
    for index, block in enumerate(_blocks(content, 'SolidBox'), start=1):
        obstacles.append({
            'type': 'SolidBox',
            'name': _quoted(_field(block, 'name', f'"box({index})"')),
            'translation': _numbers(_field(block, 'translation')),
            'rotation': _numbers(_field(block, 'rotation', '0 0 1 0')),
            'size': _numbers(_field(block, 'size')),
        })
    dimensions = _numbers(_field(arena, 'floorSize'))
    if len(dimensions) != 2 or not all(math.isfinite(v) and v > 0 for v in dimensions):
        raise ValueError(f'arena floorSize must be two positive finite values: {dimensions!r}')
    for robot in robots:
        if len(robot.translation) != 3 or not all(math.isfinite(v) for v in robot.translation):
            raise ValueError(f'non-finite pose for {robot.name}')
        if abs(robot.translation[0]) + 0.11 > dimensions[0] / 2 or \
                abs(robot.translation[1]) + 0.11 > dimensions[1] / 2:
            raise ValueError(f'{robot.name} is outside the arena')
        axis_angle_yaw(robot.rotation)
    robot1, robot2 = by_name['robot1'], by_name['robot2']
    separation = math.dist(robot1.translation[:2], robot2.translation[:2])
    if not math.isfinite(separation) or separation < 0.22:
        raise ValueError(f'robots overlap or are too close: separation={separation}')
    for robot in robots:
        for obstacle in obstacles:
            cx, cy, _ = obstacle['translation']
            width, height, _ = obstacle['size']
            angle = obstacle['rotation'][3]
            cosine, sine = math.cos(angle), math.sin(angle)
            dx, dy = robot.translation[0] - cx, robot.translation[1] - cy
            local_x = cosine * dx + sine * dy
            local_y = -sine * dx + cosine * dy
            if (max(abs(local_x) - width / 2, 0.0) ** 2 +
                    max(abs(local_y) - height / 2, 0.0) ** 2) < 0.11 ** 2:
                raise ValueError(f'{robot.name} intersects obstacle {obstacle["name"]}')
    relative = relative_transform(robot1, robot2)
    return {
        'path': str(path),
        'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'dimensions': dimensions,
        'wall_height': float(_field(arena, 'wallHeight')),
        'viewpoint': {
            'orientation': _numbers(_field(viewpoint, 'orientation')),
            'position': _numbers(_field(viewpoint, 'position')),
        },
        'robots': by_name,
        'robot_order': tuple(robot.name for robot in robots),
        'initial_world_transforms': {
            name: (robot.translation[0], robot.translation[1], robot.planar_yaw)
            for name, robot in by_name.items()
        },
        'planar_yaws': {name: robot.planar_yaw for name, robot in by_name.items()},
        'relative_transform': relative,
        'reverse_relative_transform': relative_transform(
            by_name['robot2'], by_name['robot1']),
        'obstacles': obstacles,
        'solid_box_count': len(obstacles),
        'solid_count': len(list(_blocks(content, 'Solid'))),
        'initial_separation_m': separation,
        'validation': {
            'robots_inside_arena': True,
            'robots_nonoverlapping': True,
            'robots_clear_of_obstacles': True,
            'finite_viewpoint': all(math.isfinite(v) for v in (*_numbers(_field(viewpoint, 'orientation')), *_numbers(_field(viewpoint, 'position')))),
        },
        'uses_remote_proto': bool(re.search(
            r'(?m)^EXTERNPROTO\s+"https?://', content)),
    }


def profile(name, worlds_directory, ideal_encoder_sensing=True):
    """Resolve one named profile and parse its selected installed/source world."""
    if name not in PROFILE_SETTINGS:
        raise ValueError(
            f'world profile must be one of {sorted(PROFILE_SETTINGS)}')
    result = dict(PROFILE_SETTINGS[name])
    if name == 'large' and not ideal_encoder_sensing:
        result['world'] = 'epuck_d500_two_world_large_baseline.wbt'
    result['encoder_profile'] = (
        'webots_ideal_wheel_encoders'
        if ideal_encoder_sensing else 'webots_upstream_quantized_wheel_encoders')
    result.setdefault('slam_runtime_parameters', {})
    result.setdefault('unknown_initial_pose', False)
    result['name'] = name
    result['world_path'] = str(
        Path(worlds_directory).resolve() / result['world'])
    result['world_metadata'] = parse_world(result['world_path'])
    return result


def profile_for_world(name, worlds_directory, explicit_world_path='',
                      ideal_encoder_sensing=True):
    """Resolve a profile and reject conflicting explicit world paths."""
    selected = profile(
        name, worlds_directory,
        ideal_encoder_sensing=ideal_encoder_sensing)
    if explicit_world_path:
        explicit = Path(explicit_world_path).expanduser().resolve()
        expected = Path(selected['world_path']).resolve()
        if explicit != expected:
            raise ValueError(
                'world profile/path mismatch: '
                f'profile={name!r} expects {expected}, '
                f'explicit path is {explicit}')
    return selected


def profile_summary(value):
    """Return strict manifest-friendly metadata without dataclass instances."""
    metadata = value['world_metadata']
    robots = {}
    for name, robot in metadata['robots'].items():
        robots[name] = {
            'translation': list(robot.translation),
            'rotation': list(robot.rotation),
            'planar_yaw': robot.planar_yaw,
            'controller': robot.controller,
            'window': robot.window,
        }
    return {
        'world_profile': value['name'],
        'world': value['world'],
        'physics_profile': value.get('physics_profile', 'default_contacts'),
        'baseline_world': value.get('baseline_world', value['world']),
        'encoder_profile': value['encoder_profile'],
        'wheel_position_sensor': {
            'noise': 0.0 if value['encoder_profile'] ==
            'webots_ideal_wheel_encoders' else 'WEBOTS_DEFAULT',
            'resolution': -1 if value['encoder_profile'] ==
            'webots_ideal_wheel_encoders' else 0.00628,
        },
        'world_path': value['world_path'],
        'world_sha256': metadata['sha256'],
        'world_dimensions_m': list(metadata['dimensions']),
        'robot_start_poses': robots,
        'known_relative_transform': list(metadata['relative_transform']),
        'reverse_relative_transform': list(metadata['reverse_relative_transform']),
        'initial_separation_m': metadata['initial_separation_m'],
        'transform_source': 'WORLD_DERIVED',
        'world_validation': metadata['validation'],
        'slam_resolution': value['slam_resolution'],
        'peer_export_resolution': value['slam_resolution'],
        'fusion_resolution': value['fusion_resolution'],
        'global_costmap_resolution': value['global_costmap_resolution'],
        'local_costmap_resolution': value['local_costmap_resolution'],
        'lidar_maximum_range': metadata[
            'robots']['robot1'].lidar_maximum_range,
        'minimum_frontier_cells': value['minimum_frontier_cells'],
        'minimum_known_cell_gain_for_activity':
            value['minimum_known_cell_gain_for_activity'],
        'coverage_attribution_resolution':
            value['coverage_attribution_resolution'],
        'map_comparison_shift_window':
            value['map_comparison_shift_window'],
        'unknown_initial_pose': value.get('unknown_initial_pose', False),
        'slam_runtime_parameters': dict(
            value.get('slam_runtime_parameters', {})),
    }


def manual_rviz_path(value, resource_directory):
    """Return the small preset or a generated large-world Orbit-camera preset."""
    source = Path(resource_directory) / value['rviz']
    if value['name'] == 'small':
        return str(source)
    view = value['rviz_view']
    content = source.read_text(encoding='utf-8')
    replacements = {
        '      Distance: 3\n': (
            f'      Distance: {view["distance"]}\n'),
        '        X: -0.15\n': (
            f'        X: {view["focal_x"]}\n'),
        '        Y: 0\n': (
            f'        Y: {view["focal_y"]}\n'),
        '        Z: 0\n': (
            f'        Z: {view["focal_z"]}\n'),
    }
    for old, new in replacements.items():
        if old not in content:
            raise ValueError(f'RViz Orbit template is missing {old.strip()!r}')
        content = content.replace(old, new, 1)
    destination = (
        Path(tempfile.gettempdir())
        / 'my_epuck_cooperative_manual_exploration_large.rviz')
    destination.write_text(content, encoding='utf-8')
    return str(destination)
