"""Small, explicit topology helpers for the thesis A/B baselines."""

from __future__ import annotations

import hashlib
import re
from pathlib import Path

_TOP_LEVEL_NODE = re.compile(r"^[A-Za-z][A-Za-z0-9_-]* \{$")
_ACTIVE_ROBOTS = frozenset(('robot1', 'robot2'))
_RANDOM_SEED_RE = re.compile(r'(?m)^(\s*randomSeed\s+)(\d+)(\s*)$')


def parse_active_robots(value: str) -> tuple[str, ...]:
    """Parse the comma-separated active robot launch argument."""
    robots = tuple(item.strip() for item in str(value).split(',') if item.strip())
    if (not robots or len(set(robots)) != len(robots)
            or any(robot not in _ACTIVE_ROBOTS for robot in robots)):
        raise ValueError('active_robots must contain robot1 and/or robot2')
    return robots


def _top_level_blocks(text: str):
    lines = text.splitlines(keepends=True)
    depth = 0
    start = None
    node_type = None
    for index, line in enumerate(lines):
        stripped = line.rstrip("\r\n")
        if depth == 0 and _TOP_LEVEL_NODE.match(stripped):
            start = index
            node_type = stripped.split()[0]
        depth += line.count("{") - line.count("}")
        if start is not None and depth == 0:
            yield start, index + 1, node_type, "".join(lines[start:index + 1])
            start = None
            node_type = None


def remove_robot_nodes(text: str, robot_name: str = "robot2") -> str:
    """Remove only the named top-level E-puck, preserving other world nodes."""
    lines = text.splitlines(keepends=True)
    removals = [
        (start, end) for start, end, node_type, block in _top_level_blocks(text)
        if node_type == "E-puck" and f'name "{robot_name}"' in block
    ]
    for start, end in reversed(removals):
        del lines[start:end]
    if len(removals) != 1:
        raise ValueError(f"expected one E-puck named {robot_name}, found {len(removals)}")
    return "".join(lines)


def remove_forensic_supervisor(text: str) -> str:
    """Exclude the passive temporary Supervisor from geometry provenance."""
    lines = text.splitlines(keepends=True)
    removals = [
        (start, end) for start, end, node_type, block in _top_level_blocks(text)
        if node_type == 'Robot' and
        'name "ForensicGroundTruthSupervisor"' in block
    ]
    for start, end in reversed(removals):
        del lines[start:end]
    return ''.join(lines)


def environment_geometry_signature(text: str) -> str:
    # WorldInfo.randomSeed changes stochastic behavior, not measured geometry.
    # Exclude it so seeded materializations can be compared structurally.
    geometry = _RANDOM_SEED_RE.sub(
        lambda match: match.group(1) + '0' + match.group(3), text)
    geometry = remove_robot_nodes(geometry, "robot1")
    if 'name "robot2"' in geometry:
        geometry = remove_robot_nodes(geometry, "robot2")
    geometry = remove_forensic_supervisor(geometry)
    return hashlib.sha256(geometry.encode("utf-8")).hexdigest()


def parse_world_random_seed(source: str | Path) -> int:
    text = Path(source).read_text(encoding='utf-8')
    matches = list(_RANDOM_SEED_RE.finditer(text))
    if len(matches) != 1:
        raise ValueError('world must contain exactly one integer WorldInfo.randomSeed')
    return int(matches[0].group(2))


def replace_world_random_seed(text: str, random_seed: int) -> str:
    if isinstance(random_seed, bool) or not isinstance(random_seed, int) or random_seed < 0:
        raise ValueError('random_seed must be a nonnegative integer')
    matches = list(_RANDOM_SEED_RE.finditer(text))
    if len(matches) != 1:
        raise ValueError('world must contain exactly one integer WorldInfo.randomSeed')
    match = matches[0]
    return text[:match.start(2)] + str(random_seed) + text[match.end(2):]


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def materialize_single_robot_world(
        source: str | Path, output_dir: str | Path,
        random_seed: int | None = None) -> Path:
    source_path = Path(source).resolve()
    project_root = Path(output_dir).resolve()
    worlds_dir = project_root / 'worlds'
    worlds_dir.mkdir(parents=True, exist_ok=True)
    canonical_protos = source_path.parent.parent / 'protos'
    if not canonical_protos.is_dir():
        raise FileNotFoundError(f'canonical Webots protos directory missing: {canonical_protos}')
    protos_link = project_root / 'protos'
    if protos_link.exists() or protos_link.is_symlink():
        raise FileExistsError(f'temporary Webots protos path already exists: {protos_link}')
    protos_link.symlink_to(canonical_protos, target_is_directory=True)
    # Keep the canonical basename under worlds/.  The project profile resolver
    # uses that project layout when validating an explicit world path.
    destination = worlds_dir / source_path.name
    text = remove_robot_nodes(source_path.read_text(encoding="utf-8"), "robot2")
    if random_seed is not None:
        text = replace_world_random_seed(text, random_seed)
    destination.write_text(text, encoding="utf-8")
    return destination


def materialize_seeded_world(
        source: str | Path, output_dir: str | Path, random_seed: int) -> Path:
    """Materialize a seeded Webots project without changing source assets."""
    source_path = Path(source).resolve()
    project_root = Path(output_dir).resolve()
    worlds_dir = project_root / 'worlds'
    worlds_dir.mkdir(parents=True, exist_ok=True)
    canonical_protos = source_path.parent.parent / 'protos'
    if not canonical_protos.is_dir():
        raise FileNotFoundError(f'canonical Webots protos directory missing: {canonical_protos}')
    protos_link = project_root / 'protos'
    if protos_link.exists() or protos_link.is_symlink():
        raise FileExistsError(f'temporary Webots protos path already exists: {protos_link}')
    protos_link.symlink_to(canonical_protos, target_is_directory=True)
    destination = worlds_dir / source_path.name
    destination.write_text(
        replace_world_random_seed(
            source_path.read_text(encoding='utf-8'), random_seed),
        encoding='utf-8')
    return destination


def add_forensic_supervisor(world: str | Path) -> Path:
    """Add only the passive Supervisor controller required by A forensics."""
    world_path = Path(world)
    text = world_path.read_text(encoding='utf-8')
    if 'name "ForensicGroundTruthSupervisor"' not in text:
        world_path.write_text(
            text.rstrip() +
            '\nRobot {\n'
            '  name "ForensicGroundTruthSupervisor"\n'
            '  controller "<extern>"\n'
            '  supervisor TRUE\n'
            '  window "<none>"\n'
            '}\n',
            encoding='utf-8')
    return world_path
