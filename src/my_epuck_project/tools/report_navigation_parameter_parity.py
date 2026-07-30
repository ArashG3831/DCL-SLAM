#!/usr/bin/env python3
"""Write a bounded, machine-readable navigation parity report."""

# flake8: noqa

import argparse
import json
from pathlib import Path

from my_epuck_project.navigation_parameter_parity import parity_report


def main():
    """Write a normalized parity report to the requested output path."""
    parser = argparse.ArgumentParser()
    parser.add_argument("robot1", type=Path)
    parser.add_argument("robot2", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = parity_report(args.robot1, args.robot2)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    print(args.output)


if __name__ == "__main__":
    main()
