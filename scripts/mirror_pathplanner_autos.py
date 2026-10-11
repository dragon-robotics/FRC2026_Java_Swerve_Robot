#!/usr/bin/env python3
"""Mirror PathPlanner autos and paths between field sides.

Usage:
    python scripts/mirror_pathplanner_autos.py --direction left-to-right
    python scripts/mirror_pathplanner_autos.py --direction right-to-left
    python scripts/mirror_pathplanner_autos.py --check --direction left-to-right

The selected source side is mirrored across the field's Y-axis centerline,
preserving X and transforming Y with FIELD_WIDTH - y.
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
REVERSE_NAME_REPLACEMENTS = (
    (re.compile(r"\bRight\b"), "Left"),
    (re.compile(r"\bright\b"), "left"),
    (re.compile(r"\bRTrench\b"), "LTrench"),
    (re.compile(r"\bRT\b"), "LT"),
    (re.compile(r"\bS7\b"), "S1"),
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


def is_source_name(name: str, direction: str) -> bool:
    return is_left_name(name) if direction == "left-to-right" else is_right_name(name)


def is_target_name(name: str, direction: str) -> bool:
    return is_right_name(name) if direction == "left-to-right" else is_left_name(name)


def mirror_name(name: str, direction: str) -> str:
    mirrored = name
    replacements = NAME_REPLACEMENTS if direction == "left-to-right" else REVERSE_NAME_REPLACEMENTS
    for pattern, replacement in replacements:
        mirrored = pattern.sub(replacement, mirrored)
    if mirrored == name:
        raise ValueError(f"Cannot derive mirrored name from '{name}'")
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


def mirror_path_payload(node: Any, field_width: float, direction: str) -> None:
    if isinstance(node, dict):
        for key, value in node.items():
            if key in COORDINATE_KEYS and isinstance(value, dict):
                mirror_coordinate_object(value, field_width)
            elif key == "linkedName" and isinstance(value, str) and is_source_name(value, direction):
                node[key] = mirror_name(value, direction)
            elif key in ROTATION_KEYS and isinstance(value, (int, float)):
                node[key] = normalize_angle(value)
            else:
                mirror_path_payload(value, field_width, direction)
    elif isinstance(node, list):
        for item in node:
            mirror_path_payload(item, field_width, direction)


def mirror_auto_payload(node: Any, direction: str) -> None:
    if isinstance(node, dict):
        for key, value in node.items():
            if key == "pathName" and isinstance(value, str) and is_source_name(value, direction):
                node[key] = mirror_name(value, direction)
            else:
                mirror_auto_payload(value, direction)
    elif isinstance(node, list):
        for item in node:
            mirror_auto_payload(item, direction)


def iter_source_files(directory: Path, suffix: str, direction: str) -> list[Path]:
    return sorted(path for path in directory.glob(f"*{suffix}") if is_source_name(path.stem, direction))


def generate_expected_assets(field_width: float, direction: str) -> tuple[dict[Path, str], list[str]]:
    expected: dict[Path, str] = {}
    issues: list[str] = []

    for source_path in iter_source_files(PATHS_DIR, ".path", direction):
        mirrored_payload = copy.deepcopy(load_json(source_path))
        mirror_path_payload(mirrored_payload, field_width, direction)
        try:
            target_name = mirror_name(source_path.stem, direction)
        except ValueError as exc:
            issues.append(str(exc))
            continue
        expected[PATHS_DIR / f"{target_name}.path"] = dump_json(mirrored_payload)

    for source_auto in iter_source_files(AUTOS_DIR, ".auto", direction):
        mirrored_payload = copy.deepcopy(load_json(source_auto))
        mirror_auto_payload(mirrored_payload, direction)
        try:
            target_name = mirror_name(source_auto.stem, direction)
        except ValueError as exc:
            issues.append(str(exc))
            continue
        expected[AUTOS_DIR / f"{target_name}.auto"] = dump_json(mirrored_payload)

    return expected, issues


def find_unexpected_target_files(expected_paths: set[Path], direction: str) -> list[Path]:
    candidates = []
    for directory, suffix in ((PATHS_DIR, ".path"), (AUTOS_DIR, ".auto")):
        for path in sorted(directory.glob(f"*{suffix}")):
            if is_target_name(path.stem, direction) and path not in expected_paths:
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
    parser = argparse.ArgumentParser(description="Mirror PathPlanner autos and paths between field sides")
    parser.add_argument(
        "--check",
        action="store_true",
        help="Validate that mirrored target-side assets are current without writing files",
    )
    parser.add_argument(
        "--direction",
        choices=("left-to-right", "right-to-left"),
        default="left-to-right",
        help="Direction to mirror (default: left-to-right)",
    )
    args = parser.parse_args()

    field_width = read_field_width()
    expected, issues = generate_expected_assets(field_width, args.direction)
    unexpected_target_files = find_unexpected_target_files(set(expected), args.direction)

    if args.check:
        issues.extend(validate_generated_assets(expected))
        if unexpected_target_files:
            issues.extend(
                f"Unexpected target-side file without source: {path.relative_to(REPO_ROOT)}"
                for path in unexpected_target_files
            )
        if issues:
            for issue in issues:
                print(issue, file=sys.stderr)
            return 1
        print(f"Mirrored autos are up to date ({len(expected)} generated files checked).")
        return 0

    # Preserve left-only authored assets when mirroring right-to-left.
    stale_paths = unexpected_target_files if args.direction == "left-to-right" else []
    changed, deleted = write_generated_assets(expected, stale_paths)
    for issue in issues:
        print(f"WARNING: {issue}", file=sys.stderr)
    print(f"Mirrored {len(expected)} assets {args.direction}; updated {len(changed)} files.")
    for path in changed:
        print(f"  {path.relative_to(REPO_ROOT)}")
    if deleted:
        print(f"Deleted {len(deleted)} stale right-side assets.")
        for path in deleted:
            print(f"  {path.relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
