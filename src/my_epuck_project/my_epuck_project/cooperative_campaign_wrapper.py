"""Safe explicit-argv wrapper for the authoritative unknown-pose campaign.

This wrapper deliberately does not use a shell.  It validates the selected
profile/world contract, then delegates to the existing single-trial runner.
The wrapper is orchestration-only; estimator and navigation behavior remain in
``cooperative_trial_fast`` and the launch graph.
"""

from __future__ import annotations

import argparse
import shlex
import sys
from pathlib import Path

from . import cooperative_trial_fast as trial
from .cooperative_profiles import profile_for_world


PROFILE = 'large_unknown_pose_16m'
OBSERVER_SUFFIX = 'ForensicGroundTruthSupervisor'


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--workspace', type=Path, required=True)
    result.add_argument('--world-path', type=Path, required=True)
    result.add_argument('--results-directory', type=Path, required=True)
    result.add_argument('--observer-url', default='')
    result.add_argument('--webots-port', type=int, default=23000)
    result.add_argument('--ros-domain-id', type=int, default=110)
    result.add_argument('--mission-timeout', type=float, default=850.0)
    result.add_argument('--startup-timeout', type=float, default=280.0)
    result.add_argument('--webots-mode', default='fast')
    result.add_argument('--dry-run', action='store_true')
    return result


def runner_arguments(config: argparse.Namespace) -> list[str]:
    """Build the runner argv as tokens; no shell syntax is ever included."""
    return [
        '--world-profile', PROFILE,
        '--world-path', str(config.world_path.resolve()),
        '--sensor-profile', 'full',
        '--ideal-encoder-sensing', 'true',
        '--webots-mode', config.webots_mode,
        '--rendering', 'false',
        '--rviz', 'false',
        '--diagnostic-mode', 'false',
        '--enable-observer', 'true',
        '--enable-forensic-capture', 'true',
        '--fusion-process-nice', '0',
        '--mission-timeout', str(config.mission_timeout),
        '--startup-timeout', str(config.startup_timeout),
        '--ros-domain-id', str(config.ros_domain_id),
        '--webots-port', str(config.webots_port),
        '--results-directory', str(config.results_directory.resolve()),
    ]


def validate(config: argparse.Namespace) -> dict:
    workspace = config.workspace.resolve()
    world = config.world_path.resolve()
    if not world.is_file():
        raise ValueError(f'selected staged world does not exist: {world}')
    if config.webots_port < 1024 or config.webots_port > 65535:
        raise ValueError('webots port must be between 1024 and 65535')
    expected_url = (
        f'tcp://127.0.0.1:{config.webots_port}/{OBSERVER_SUFFIX}')
    if config.observer_url and config.observer_url != expected_url:
        raise ValueError(
            f'observer URL mismatch: expected {expected_url}, '
            f'got {config.observer_url}')
    profile = profile_for_world(
        PROFILE, str(world.parent), explicit_world_path=str(world),
        ideal_encoder_sensing=True)
    launch = workspace / 'src' / 'my_epuck_project' / 'launch'
    for name in (
            'two_robots_decentralized_exploration_launch.py',
            'two_robots_frontier_candidates_launch.py',
            'two_robots_teammate_filtered_stack_launch.py',
            'two_robots_teammate_filtered_dual_slam_launch.py'):
        source = (launch / name).read_text(encoding='utf-8')
        if 'profile_for_world' not in source or 'explicit_world_path' not in source:
            raise ValueError(f'launch layer lacks canonical profile resolver: {name}')
    argv = runner_arguments(config)
    forbidden = [token for token in argv if token in ('\\', '\\\\') or '\\' in token]
    if forbidden:
        raise ValueError(f'runner argv contains shell continuation token(s): {forbidden}')
    if '--world-profile' not in argv or PROFILE not in argv:
        raise ValueError('runner argv does not contain the canonical profile')
    if str(world) not in argv:
        raise ValueError('runner argv does not contain the selected world path')
    if 'large_dynamic_low_slip_4ms_finite.wbt' in str(world):
        raise ValueError('generic large world selected for unknown-pose campaign')
    return {
        'workspace': str(workspace),
        'profile': PROFILE,
        'world_path': str(world),
        'world_sha256': profile['world_metadata']['sha256'],
        'physics_profile': profile['physics_profile'],
        'observer_url': expected_url,
        'frontend_diagnostic_output': str(
            config.results_directory.resolve() / 'frontend'),
        'runner_argv': argv,
        'shell_command': shlex.join([sys.executable, '-m',
                                     'my_epuck_project.cooperative_campaign_wrapper',
                                     *argv]),
    }


def main(argv=None) -> int:
    config = parser().parse_args(argv)
    metadata = validate(config)
    print('CAMPAIGN_PREFLIGHT ' + shlex.join(metadata['runner_argv']), flush=True)
    print('CAMPAIGN_EXECUTABLE ' + metadata['shell_command'], flush=True)
    if config.dry_run:
        print('CAMPAIGN_DRY_RUN_OK', flush=True)
        return 0
    trial.WORKSPACE = config.workspace.resolve()
    return trial.main(metadata['runner_argv'])


if __name__ == '__main__':
    raise SystemExit(main())
