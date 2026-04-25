#!/usr/bin/env python3
"""Mirror left-side PathPlanner autos and paths to right-side assets.

Usage:
    python scripts/mirror_pathplanner_autos.py
    python scripts/mirror_pathplanner_autos.py --check

The left-side PathPlanner files are treated as the canonical authored source.
This script generates right-side files by mirroring geometry across the field's
Y-axis centerline, preserving X and transforming Y with FIELD_WIDTH - y.
"""

from __future__ import annotations

import argparse
import copy
import json
import re
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parent.parent
PATHPLANNER_DIR = REPO_ROOT / "src" / "main" / "deploy" / "pathplanner"
AUTOS_DIR = PATHPLANNER_DIR / "autos"
PATHS_DIR = PATHPLANNER_DIR / "paths"
NAVGRID_PATH = PATHPLANNER_DIR / "navgrid.json"

LEFT_NAME_PATTERNS = (
    re.compile(r"\bLeft\b"),
    re.compile(r"\bLTrench\b"),
    re.compile(r"\bLT\b"),
    re.compile(r"\bS1\b"),
)
RIGHT_NAME_PATTERNS = (
    re.compile(r"\bRight\b"),
    re.compile(r"\bRTrench\b"),
    re.compile(r"\bRT\b"),
    re.compile(r"\bS7\b"),
)
NAME_REPLACEMENTS = (
    (re.compile(r"\bLeft\b"), "Right"),
    (re.compile(r"\bleft\b"), "right"),
    (re.compile(r"\bLTrench\b"), "RTrench"),
    (re.compile(r"\bLT\b"), "RT"),
    (re.compile(r"\bS1\b"), "S7"),
)
COORDINATE_KEYS = {"anchor", "prevControl", "nextControl", "targetPosition", "position"}
ROTATION_KEYS = {"rotation", "rotationDegrees", "rotationOffset", "angleOffset"}
EPSILON = 1e-9


def is_left_name(name: str) -> bool:
    return any(pattern.search(name) for pattern in LEFT_NAME_PATTERNS) and not any(
        pattern.search(name) for pattern in RIGHT_NAME_PATTERNS
    )


def is_right_name(name: str) -> bool:
    return any(pattern.search(name) for pattern in RIGHT_NAME_PATTERNS)


def mirror_name(name: str) -> str:
    mirrored = name
    for pattern, replacement in NAME_REPLACEMENTS:
        mirrored = pattern.sub(replacement, mirrored)
    if mirrored == name:
        raise ValueError(f"Cannot derive right-side name from '{name}'")
    return mirrored


def normalize_angle(angle_degrees: float) -> float:
    mirrored = -float(angle_degrees)
    while mirrored <= -180.0:
        mirrored += 360.0
    while mirrored > 180.0:
        mirrored -= 360.0
    if abs(mirrored) < EPSILON:
        return 0.0
    return round(mirrored, 12)


def mirror_y(y_value: float, field_width: float) -> float:
    mirrored = field_width - float(y_value)
    if abs(mirrored) < EPSILON:
        return 0.0
    return round(mirrored, 12)


def load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def dump_json(data: Any) -> str:
    return json.dumps(data, indent=2, ensure_ascii=False) + "\n"


def read_field_width() -> float:
    navgrid = load_json(NAVGRID_PATH)
    return float(navgrid["field_size"]["y"])


def mirror_coordinate_object(node: dict[str, Any], field_width: float) -> None:
    if "y" in node:
        node["y"] = mirror_y(node["y"], field_width)


def mirror_path_payload(node: Any, field_width: float) -> None:
    if isinstance(node, dict):
        for key, value in node.items():
            if key in COORDINATE_KEYS and isinstance(value, dict):
                mirror_coordinate_object(value, field_width)
            elif key == "linkedName" and isinstance(value, str) and is_left_name(value):
                node[key] = mirror_name(value)
            elif key in ROTATION_KEYS and isinstance(value, (int, float)):
                node[key] = normalize_angle(value)
            else:
                mirror_path_payload(value, field_width)
    elif isinstance(node, list):
        for item in node:
            mirror_path_payload(item, field_width)


def mirror_auto_payload(node: Any) -> None:
    if isinstance(node, dict):
        for key, value in node.items():
            if key == "pathName" and isinstance(value, str):
                node[key] = mirror_name(value)
            else:
                mirror_auto_payload(value)
    elif isinstance(node, list):
        for item in node:
            mirror_auto_payload(item)


def iter_left_files(directory: Path, suffix: str) -> list[Path]:
    return sorted(path for path in directory.glob(f"*{suffix}") if is_left_name(path.stem))


def generate_expected_assets(field_width: float) -> tuple[dict[Path, str], list[str]]:
    expected: dict[Path, str] = {}
    issues: list[str] = []

    for left_path in iter_left_files(PATHS_DIR, ".path"):
        mirrored_payload = copy.deepcopy(load_json(left_path))
        mirror_path_payload(mirrored_payload, field_width)
        try:
            right_name = mirror_name(left_path.stem)
        except ValueError as exc:
            issues.append(str(exc))
            continue
        expected[PATHS_DIR / f"{right_name}.path"] = dump_json(mirrored_payload)

    for left_auto in iter_left_files(AUTOS_DIR, ".auto"):
        mirrored_payload = copy.deepcopy(load_json(left_auto))
        mirror_auto_payload(mirrored_payload)
        try:
            right_name = mirror_name(left_auto.stem)
        except ValueError as exc:
            issues.append(str(exc))
            continue
        expected[AUTOS_DIR / f"{right_name}.auto"] = dump_json(mirrored_payload)

    return expected, issues


def find_unexpected_right_files(expected_paths: set[Path]) -> list[Path]:
    candidates = []
    for directory, suffix in ((PATHS_DIR, ".path"), (AUTOS_DIR, ".auto")):
        for path in sorted(directory.glob(f"*{suffix}")):
            if is_right_name(path.stem) and path not in expected_paths:
                candidates.append(path)
    return candidates


def write_generated_assets(expected: dict[Path, str], stale_paths: list[Path]) -> tuple[list[Path], list[Path]]:
    changed: list[Path] = []
    for path, rendered in sorted(expected.items()):
        current = path.read_text(encoding="utf-8") if path.exists() else None
        if current != rendered:
            path.write_text(rendered, encoding="utf-8")
            changed.append(path)
    deleted: list[Path] = []
    for path in stale_paths:
        path.unlink(missing_ok=True)
        deleted.append(path)
    return changed, deleted


def validate_generated_assets(expected: dict[Path, str]) -> list[str]:
    issues: list[str] = []
    for path, rendered in sorted(expected.items()):
        if not path.exists():
            issues.append(f"Missing generated file: {path.relative_to(REPO_ROOT)}")
            continue
        try:
            actual_normalized = dump_json(load_json(path))
        except json.JSONDecodeError as exc:
            issues.append(f"Invalid JSON in {path.relative_to(REPO_ROOT)}: {exc}")
            continue
        if actual_normalized != rendered:
            issues.append(f"Out-of-date mirrored file: {path.relative_to(REPO_ROOT)}")
    return issues


def main() -> int:
    parser = argparse.ArgumentParser(description="Mirror left-side PathPlanner autos and paths to the right side")
    parser.add_argument(
        "--check",
        action="store_true",
        help="Validate that generated right-side assets are current without writing files",
    )
    args = parser.parse_args()

    field_width = read_field_width()
    expected, issues = generate_expected_assets(field_width)
    unexpected_right_files = find_unexpected_right_files(set(expected))

    if args.check:
        issues.extend(validate_generated_assets(expected))
        if unexpected_right_files:
            issues.extend(
                f"Unexpected right-side file without left source: {path.relative_to(REPO_ROOT)}"
                for path in unexpected_right_files
            )
        if issues:
            for issue in issues:
                print(issue, file=sys.stderr)
            return 1
        print(f"Mirrored autos are up to date ({len(expected)} generated files checked).")
        return 0

    changed, deleted = write_generated_assets(expected, unexpected_right_files)
    for issue in issues:
        print(f"WARNING: {issue}", file=sys.stderr)
    print(f"Mirrored {len(expected)} right-side assets; updated {len(changed)} files.")
    for path in changed:
        print(f"  {path.relative_to(REPO_ROOT)}")
    if deleted:
        print(f"Deleted {len(deleted)} stale right-side assets.")
        for path in deleted:
            print(f"  {path.relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())