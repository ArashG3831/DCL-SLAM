"""Export cooperative occupancy-map artifacts as portable PNG files."""

import argparse
import json
import os
import struct
import zlib
from pathlib import Path

import numpy as np

from .occupancy_map_comparison import canonical_map, load_map


PNG_SIGNATURE = b'\x89PNG\r\n\x1a\n'
UNKNOWN_RGB = (205, 205, 205)
FREE_RGB = (255, 255, 255)
OCCUPIED_RGB = (0, 0, 0)
UNCERTAIN_RGB = (255, 193, 7)
DIFFERENCE_RGB = (220, 0, 0)
AGREEMENT_RGB = (238, 238, 238)


def _png_chunk(kind, payload):
    checksum = zlib.crc32(kind)
    checksum = zlib.crc32(payload, checksum)
    return (
        struct.pack('>I', len(payload)) + kind + payload
        + struct.pack('>I', checksum & 0xffffffff)
    )


def write_rgb_png(path, rgb):
    """Write an RGB uint8 array as a dependency-free, atomic PNG."""
    path = Path(path)
    array = np.asarray(rgb, dtype=np.uint8)
    if array.ndim != 3 or array.shape[2] != 3:
        raise ValueError('PNG input must have shape (height, width, 3)')
    height, width, _ = array.shape
    scanlines = b''.join(
        b'\x00' + array[row].tobytes() for row in range(height)
    )
    header = struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)
    content = (
        PNG_SIGNATURE
        + _png_chunk(b'IHDR', header)
        + _png_chunk(b'IDAT', zlib.compress(scanlines, level=9))
        + _png_chunk(b'IEND', b'')
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f'.{path.name}.{os.getpid()}.tmp')
    with temporary.open('wb') as stream:
        stream.write(content)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def occupancy_rgb(data, free_threshold=25, occupied_threshold=65, scale=4):
    """Color a ROS occupancy array and orient world +y toward image top."""
    values = np.asarray(data)
    rgb = np.empty(values.shape + (3,), dtype=np.uint8)
    rgb[:] = UNCERTAIN_RGB
    rgb[values < 0] = UNKNOWN_RGB
    rgb[(values >= 0) & (values <= free_threshold)] = FREE_RGB
    rgb[values >= occupied_threshold] = OCCUPIED_RGB
    rgb = np.flipud(rgb)
    if scale < 1:
        raise ValueError('scale must be at least one')
    if scale > 1:
        rgb = np.repeat(np.repeat(rgb, scale, axis=0), scale, axis=1)
    return rgb


def difference_rgb(first, second, scale=4):
    """Render equal cells light gray and every exact difference red."""
    if first.shape != second.shape:
        raise ValueError('difference image requires equal array shapes')
    rgb = np.empty(first.shape + (3,), dtype=np.uint8)
    rgb[:] = AGREEMENT_RGB
    rgb[first != second] = DIFFERENCE_RGB
    rgb = np.flipud(rgb)
    if scale > 1:
        rgb = np.repeat(np.repeat(rgb, scale, axis=0), scale, axis=1)
    return rgb


def latest_campaign(results_root):
    """Return the newest completed or active regression campaign directory."""
    candidates = sorted(
        path for path in Path(results_root).glob('regression_*')
        if (path / 'campaign_progress.json').is_file()
    )
    if not candidates:
        raise ValueError(f'no regression campaign found in {results_root}')
    return candidates[-1]


def selected_attempt(campaign, trial_id=None, allow_incomplete=False):
    """Resolve a selected trial, optionally including an interrupted attempt."""
    progress = json.loads(
        (campaign / 'campaign_progress.json').read_text(encoding='utf-8'))
    selected = progress.get('valid_trials', {})
    if selected:
        key = trial_id or sorted(selected)[-1]
        if key in selected:
            return campaign / selected[key], key
        if not allow_incomplete:
            raise ValueError(f'{key} is not a selected valid trial')
    elif not allow_incomplete:
        raise ValueError(f'{campaign} has no valid selected trial')

    if not allow_incomplete:
        raise ValueError(f'{campaign} has no valid selected trial')
    if not trial_id:
        raise ValueError(
            '--allow-incomplete requires --trial-id to identify the attempt')
    attempts = sorted(
        path for path in (campaign / 'attempts').glob(
            f'{trial_id}_attempt_*') if path.is_dir())
    if not attempts:
        raise ValueError(f'{trial_id} has no attempt directory')
    attempt = attempts[-1]
    required = (
        attempt / 'robot1_final_shared_map.npz',
        attempt / 'robot2_final_shared_map.npz',
    )
    missing = [str(path.name) for path in required if not path.is_file()]
    if missing:
        raise ValueError(
            f'{attempt} is incomplete; missing map artifacts: '
            + ', '.join(missing))
    return attempt, trial_id


def export_maps(campaign, output_dir, trial_id=None, scale=4,
                include_robot_maps=True, allow_incomplete=False):
    """Export canonical, robot, and exact-difference PNGs plus a manifest."""
    campaign = Path(campaign).resolve()
    output = Path(output_dir).resolve()
    attempt, selected_trial = selected_attempt(
        campaign, trial_id, allow_incomplete=allow_incomplete)
    robot1 = load_map(attempt / 'robot1_final_shared_map.npz')
    robot2 = load_map(attempt / 'robot2_final_shared_map.npz')
    same_geometry = robot1.geometry == robot2.geometry
    exact_equal = (
        same_geometry and np.array_equal(robot1.data, robot2.data)
    )
    canonical_path = attempt / 'canonical_final_map.npz'
    if canonical_path.is_file():
        canonical = load_map(canonical_path)
    else:
        canonical, _, _ = canonical_map(robot1, robot2)
    output.mkdir(parents=True, exist_ok=False)
    written = []

    def save(name, image):
        path = output / name
        write_rgb_png(path, image)
        written.append(str(path))

    save('final_merged_map.png', occupancy_rgb(canonical.data, scale=scale))
    if include_robot_maps:
        save(
            'robot1_final_shared_map.png',
            occupancy_rgb(robot1.data, scale=scale))
        save(
            'robot2_final_shared_map.png',
            occupancy_rgb(robot2.data, scale=scale))
        if same_geometry:
            save(
                'robot1_robot2_exact_difference.png',
                difference_rgb(robot1.data, robot2.data, scale=scale))
    manifest = {
        'schema_version': '1.0.0',
        'campaign_id': campaign.name,
        'trial_id': selected_trial,
        'attempt_id': attempt.name,
        'source_attempt': str(attempt),
        'output_directory': str(output),
        'scale': scale,
        'legend': {
            'unknown': list(UNKNOWN_RGB),
            'free': list(FREE_RGB),
            'occupied': list(OCCUPIED_RGB),
            'uncertain': list(UNCERTAIN_RGB),
            'exact_difference': list(DIFFERENCE_RGB),
        },
        'robot_maps_same_geometry': same_geometry,
        'robot_maps_exactly_identical': exact_equal,
        'different_cell_count': (
            int(np.count_nonzero(robot1.data != robot2.data))
            if same_geometry else None
        ),
        'map_geometry': {
            'width': canonical.geometry.width,
            'height': canonical.geometry.height,
            'resolution_m': canonical.geometry.resolution,
            'origin_x_m': canonical.geometry.origin_x,
            'origin_y_m': canonical.geometry.origin_y,
            'origin_yaw_rad': canonical.geometry.yaw,
        },
        'files': written,
    }
    manifest_path = output / 'export_manifest.json'
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    return manifest


def parser():
    """Build the command-line parser."""
    result = argparse.ArgumentParser(
        description='Export cooperative regression maps to PNG')
    result.add_argument(
        '--campaign',
        help='Campaign directory; defaults to newest under --results-root')
    result.add_argument('--results-root', default='results')
    result.add_argument('--trial-id', help='Defaults to last selected trial')
    result.add_argument(
        '--allow-incomplete', action='store_true',
        help='Export an explicitly selected interrupted attempt when map '
             'artifacts are present.')
    result.add_argument('--output-dir', required=True)
    result.add_argument('--scale', type=int, default=4)
    result.add_argument(
        '--include-robot-maps', action=argparse.BooleanOptionalAction,
        default=True)
    return result


def main(argv=None):
    """Run the command-line exporter."""
    args = parser().parse_args(argv)
    campaign = (
        Path(args.campaign) if args.campaign
        else latest_campaign(args.results_root)
    )
    try:
        manifest = export_maps(
            campaign, args.output_dir, args.trial_id, args.scale,
            args.include_robot_maps, args.allow_incomplete)
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as error:
        print(f'EXPORT_ERROR: {error}')
        return 2
    print(f'output_directory={manifest["output_directory"]}')
    print(
        'robot_maps_exactly_identical='
        f'{manifest["robot_maps_exactly_identical"]}')
    for path in manifest['files']:
        print(f'png={path}')
    return 0
