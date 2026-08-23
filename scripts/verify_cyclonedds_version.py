#!/usr/bin/env python3
"""Verify the pinned ROS Jazzy CycloneDDS packages for WSL validation."""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys


PACKAGES = {
    'ros-jazzy-cyclonedds': 'ROS_JAZZY_CYCLONEDDS_VERSION',
    'ros-jazzy-rmw-cyclonedds-cpp': 'ROS_JAZZY_RMW_CYCLONEDDS_CPP_VERSION',
}


def read_lock(path: Path) -> dict[str, str]:
    values = {}
    for line in path.read_text(encoding='utf-8').splitlines():
        line = line.strip()
        if not line or line.startswith('#') or '=' not in line:
            continue
        key, value = line.split('=', 1)
        values[key.strip()] = value.strip()
    return values


def installed_version(package: str) -> str | None:
    result = subprocess.run(
        ['dpkg-query', '-W', '-f=${Version}', package],
        check=False, capture_output=True, text=True)
    if result.returncode != 0:
        return None
    return result.stdout.strip() or None


def main() -> int:
    workspace = Path(__file__).resolve().parents[1]
    lock = read_lock(workspace / 'config/cyclonedds/ros_jazzy_versions.env')
    packages = {
        package: {
            'expected': lock.get(variable),
            'installed': installed_version(package),
        }
        for package, variable in PACKAGES.items()
    }
    report = {
        'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION'),
        'cyclonedds_uri': os.environ.get('CYCLONEDDS_URI'),
        'packages': packages,
        'passed': all(
            item['expected'] and item['installed'] == item['expected']
            for item in packages.values()
        ),
    }
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
