#!/usr/bin/env python3
"""Source-tree entry point for the cooperative regression campaign."""

from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from my_epuck_project.cooperative_regression import main  # noqa: E402


if __name__ == '__main__':
    raise SystemExit(main())
