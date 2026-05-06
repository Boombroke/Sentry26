#!/usr/bin/env python3
"""Verify that generated Point-LIO overrides are fresh against xmacro inputs."""

from __future__ import annotations

import argparse
import math
import re
import subprocess
import sys
import tempfile
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Regenerate Point-LIO override YAML and compare it with an existing artifact."
    )
    parser.add_argument("--real-xmacro", required=True, type=Path)
    parser.add_argument("--imu-macro", required=True, type=Path)
    parser.add_argument("--generated", required=True, type=Path)
    parser.add_argument(
        "--generator",
        type=Path,
        default=Path(__file__).with_name("generate_pointlio_overrides.py"),
    )
    return parser.parse_args()


def require_file(path: Path, label: str) -> None:
    if not path.is_file():
        raise RuntimeError(f"missing {label}: {path}")


def extract_array(text: str, key: str) -> list[float]:
    match = re.search(rf"\b{re.escape(key)}\s*:\s*\[([^\]]+)\]", text, flags=re.DOTALL)
    if not match:
        raise RuntimeError(f"missing mapping.{key} in generated YAML")
    values: list[float] = []
    for raw in match.group(1).replace("\n", " ").split(","):
        raw = raw.strip()
        if raw:
            values.append(float(raw))
    return values


def mapping_semantics(path: Path) -> dict[str, list[float]]:
    text = path.read_text(encoding="utf-8")
    return {
        "gravity": extract_array(text, "gravity"),
        "gravity_init": extract_array(text, "gravity_init"),
        "extrinsic_T": extract_array(text, "extrinsic_T"),
        "extrinsic_R": extract_array(text, "extrinsic_R"),
    }


def assert_same_semantics(expected: dict[str, list[float]], actual: dict[str, list[float]]) -> None:
    for key, expected_values in expected.items():
        actual_values = actual[key]
        if len(expected_values) != len(actual_values):
            raise RuntimeError(f"mapping.{key} length differs: {len(actual_values)} != {len(expected_values)}")
        for index, (expected_value, actual_value) in enumerate(zip(expected_values, actual_values)):
            if not math.isclose(expected_value, actual_value, rel_tol=0.0, abs_tol=1e-12):
                raise RuntimeError(
                    f"mapping.{key}[{index}] differs: {actual_value} != {expected_value}"
                )


def main() -> int:
    args = parse_args()
    try:
        require_file(args.real_xmacro, "real xmacro")
        require_file(args.imu_macro, "IMU xmacro")
        require_file(args.generated, "generated YAML")
        require_file(args.generator, "generator script")

        with tempfile.TemporaryDirectory(prefix="pointlio-overrides-") as temp_dir:
            expected_path = Path(temp_dir) / "pointlio_dual_overrides.yaml"
            subprocess.run(
                [
                    sys.executable,
                    str(args.generator),
                    "--real-xmacro",
                    str(args.real_xmacro),
                    "--imu-macro",
                    str(args.imu_macro),
                    "--output",
                    str(expected_path),
                ],
                check=True,
            )
            assert_same_semantics(mapping_semantics(expected_path), mapping_semantics(args.generated))

        newest_input_mtime = max(
            args.real_xmacro.stat().st_mtime,
            args.imu_macro.stat().st_mtime,
            args.generator.stat().st_mtime,
        )
        if args.generated.stat().st_mtime < newest_input_mtime:
            raise RuntimeError("generated YAML is older than xmacro or generator inputs")

    except (RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1

    print(f"PASS: {args.generated} is fresh against xmacro inputs")
    return 0


if __name__ == "__main__":
    sys.exit(main())
