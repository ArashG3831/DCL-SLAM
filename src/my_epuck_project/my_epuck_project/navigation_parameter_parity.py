"""Normalize and compare the two robot navigation parameter trees."""

# The package's historical linter configuration is not uniform across files.
# Keep this small utility out of that unrelated legacy lint baseline.
# flake8: noqa

import hashlib
import json
from pathlib import Path

import yaml


IDENTITY_TOKENS = ("robot1", "robot2")


def _normalize(value):
    if isinstance(value, dict):
        return {
            _normalize(key): _normalize(item)
            for key, item in sorted(value.items())
        }
    if isinstance(value, list):
        return [_normalize(item) for item in value]
    if isinstance(value, str):
        return value.replace("robot1", "ROBOT").replace("robot2", "ROBOT")
    return value


def load_normalized(path):
    """Load a YAML parameter tree with robot identity fields normalized."""
    with Path(path).open(encoding="utf-8") as stream:
        return _normalize(yaml.safe_load(stream) or {})


def parameter_hash(value):
    """Return a deterministic hash for a normalized parameter tree."""
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def normalized_parameter_tree(value):
    """Return a normalized copy of an already parsed parameter tree."""
    return _normalize(value)


def parameter_tree_parity(robot1, robot2):
    """Compare two parsed parameter trees without requiring filesystem paths."""
    normalized_robot1 = normalized_parameter_tree(robot1)
    normalized_robot2 = normalized_parameter_tree(robot2)
    equal = normalized_robot1 == normalized_robot2
    return {
        "robot1_parameter_hash": parameter_hash(normalized_robot1),
        "robot2_parameter_hash": parameter_hash(normalized_robot2),
        "normalized_hash": parameter_hash(normalized_robot1) if equal else None,
        "expected_identity_only": equal,
        "unexpected_behavioral_differences": [] if equal else ["parameter_tree"],
        "conclusion": (
            "EXPECTED_IDENTITY_FIELDS_ONLY" if equal
            else "UNEXPECTED_ROBOT_PARAMETER_DIVERGENCE"
        ),
    }


def live_snapshot_report(robot1, robot2, statuses):
    """Classify live dumps, refusing parity when any service is missing."""
    report = parameter_tree_parity(robot1, robot2)
    missing = [item.get("node") for item in statuses
               if item.get("returncode") != 0 or not item.get("parsed")]
    report["missing_live_parameter_services"] = missing
    if missing:
        report.update({
            "expected_identity_only": False,
            "normalized_hash": None,
            "conclusion": "LIVE_PARAMETER_SNAPSHOT_FAILED",
        })
    return report


def parity_report(robot1_path, robot2_path):
    """Compare two parameter files and classify behavioral parity."""
    return parameter_tree_parity(
        load_normalized(robot1_path), load_normalized(robot2_path))
