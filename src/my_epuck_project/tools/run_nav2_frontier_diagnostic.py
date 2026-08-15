#!/usr/bin/env python3
"""Source-tree entry point for the lean Nav2/frontier diagnostic runner."""

from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from my_epuck_project.nav2_frontier_diagnostic import runner_main  # noqa: E402


if __name__ == '__main__':
    raise SystemExit(runner_main())
