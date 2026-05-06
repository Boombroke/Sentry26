#!/usr/bin/env python3
"""Deterministic profiling helper for the dual Mid360 pointcloud merger.

The synthetic mode intentionally mirrors the hot operations in
src/merger_node.cpp without importing ROS 2 or Livox Python packages:

* front/back native-frame min-distance filtering (isBeyondMinDistance)
* back_mid360 -> front_mid360 affine transform
* per-message offset_time rebasing and uint32 overflow drop checks
* stable sort by offset_time before publishing

The live mode only inspects a supplied PID via /proc.  It never starts, stops,
or modifies ROS processes.
"""

from __future__ import annotations

import argparse
import math
import os
import random
import statistics
import sys
import time
from pathlib import Path
from typing import NamedTuple


UINT32_MAX = (1 << 32) - 1
NSEC_PER_MSEC = 1_000_000
Point = tuple[float, float, float, int, int, int, int]


class Transform(NamedTuple):
    r00: float
    r01: float
    r02: float
    tx: float
    r10: float
    r11: float
    r12: float
    ty: float
    r20: float
    r21: float
    r22: float
    tz: float


class MergeStats(NamedTuple):
    output_count: int
    drop_front_min_dist: int
    drop_back_min_dist: int
    drop_front_offset_overflow: int
    drop_back_offset_overflow: int


class LatencySummary(NamedTuple):
    samples: int
    median_ms: float
    p95_ms: float
    p99_ms: float
    max_ms: float
    min_ms: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Profile dual Mid360 merger hot operations with synthetic CustomMsg-like data."
    )
    parser.add_argument("--front-points", type=int, default=2000, help="Synthetic front CustomMsg point count.")
    parser.add_argument("--back-points", type=int, default=2000, help="Synthetic back CustomMsg point count.")
    parser.add_argument("--iterations", type=int, default=160, help="Measured synthetic iterations.")
    parser.add_argument("--warmup-iterations", type=int, default=10, help="Untimed warmup iterations.")
    parser.add_argument("--seed", type=int, default=20260506, help="Fixed synthetic data seed.")
    parser.add_argument("--min-dist-front-m", type=float, default=0.4, help="Front native min-distance filter.")
    parser.add_argument("--min-dist-back-m", type=float, default=0.4, help="Back native min-distance filter.")
    parser.add_argument(
        "--close-point-ratio",
        type=float,
        default=0.06,
        help="Fraction of points generated inside each native min-distance radius.",
    )
    parser.add_argument(
        "--stamp-diff-ms",
        type=float,
        default=0.4,
        help="Synthetic back stamp delay relative to front; used for offset rebasing.",
    )
    parser.add_argument(
        "--back-translation",
        type=float,
        nargs=3,
        default=(-0.10, 0.0, 0.0),
        metavar=("X", "Y", "Z"),
        help="Synthetic back->front translation in meters.",
    )
    parser.add_argument("--back-yaw-rad", type=float, default=math.pi, help="Synthetic back->front yaw.")
    parser.add_argument("--max-median-ms", type=float, default=2.0, help="Median latency threshold.")
    parser.add_argument("--max-p99-ms", type=float, default=5.0, help="P99 latency threshold.")
    parser.add_argument("--live-pid", type=int, help="Inspect a live merger_node PID via /proc instead of synthetic mode.")
    parser.add_argument("--live-samples", type=int, default=10, help="Live CPU/RSS sample count.")
    parser.add_argument("--live-interval", type=float, default=1.0, help="Seconds between live samples.")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.front_points < 0 or args.back_points < 0:
        raise ValueError("point counts must be non-negative")
    if args.front_points + args.back_points <= 0:
        raise ValueError("at least one synthetic point is required")
    if args.iterations <= 0:
        raise ValueError("--iterations must be positive")
    if args.warmup_iterations < 0:
        raise ValueError("--warmup-iterations must be non-negative")
    if args.min_dist_front_m < 0.0 or args.min_dist_back_m < 0.0:
        raise ValueError("min-distance thresholds must be non-negative")
    if not 0.0 <= args.close_point_ratio <= 1.0:
        raise ValueError("--close-point-ratio must be in [0, 1]")
    if args.stamp_diff_ms < 0.0:
        raise ValueError("--stamp-diff-ms must be non-negative")
    if args.max_median_ms < 0.0 or args.max_p99_ms < 0.0:
        raise ValueError("latency thresholds must be non-negative")
    if args.live_samples <= 0:
        raise ValueError("--live-samples must be positive")
    if args.live_interval <= 0.0:
        raise ValueError("--live-interval must be positive")


def make_yaw_transform(yaw: float, translation: list[float]) -> Transform:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return Transform(c, -s, 0.0, translation[0], s, c, 0.0, translation[1], 0.0, 0.0, 1.0, translation[2])


def generate_points(count: int, rng: random.Random, min_distance_m: float, close_ratio: float) -> list[Point]:
    points: list[Point] = []
    safe_min = max(min_distance_m, 0.01)
    offset_step = max(1, count // 160)
    for index in range(count):
        if rng.random() < close_ratio:
            radius = rng.uniform(0.02, safe_min * 0.92)
        else:
            radius = rng.uniform(safe_min * 1.05, 12.0)
        yaw = rng.uniform(-math.pi, math.pi)
        pitch = rng.uniform(-0.38, 0.38)
        horizontal = radius * math.cos(pitch)
        points.append(
            (
                horizontal * math.cos(yaw),
                horizontal * math.sin(yaw),
                radius * math.sin(pitch),
                (index // offset_step) * 1000,
                rng.randrange(0, 256),
                rng.randrange(0, 4),
                rng.randrange(0, 4),
            )
        )
    return points


def build_input_pool(args: argparse.Namespace) -> list[tuple[list[Point], list[Point]]]:
    rng = random.Random(args.seed)
    return [
        (
            generate_points(args.front_points, rng, args.min_dist_front_m, args.close_point_ratio),
            generate_points(args.back_points, rng, args.min_dist_back_m, args.close_point_ratio),
        )
        for _ in range(args.iterations)
    ]


def append_with_rebased_offset(
    merged: list[tuple[int, int, float, float, float, int, int, int]],
    point: Point,
    rebase_offset_ns: int,
    stable_index: int,
) -> bool:
    rebased_offset = point[3] + rebase_offset_ns
    if rebased_offset > UINT32_MAX:
        return False
    merged.append((rebased_offset, stable_index, point[0], point[1], point[2], point[4], point[5], point[6]))
    return True


def merge_pair(
    front_points: list[Point],
    back_points: list[Point],
    transform: Transform,
    min_dist_front_sq: float,
    min_dist_back_sq: float,
    front_offset_rebase_ns: int,
    back_offset_rebase_ns: int,
) -> tuple[list[tuple[int, int, float, float, float, int, int, int]], MergeStats]:
    merged: list[tuple[int, int, float, float, float, int, int, int]] = []
    drop_front_min_dist = 0
    drop_back_min_dist = 0
    drop_front_offset_overflow = 0
    drop_back_offset_overflow = 0
    stable_index = 0

    for point in front_points:
        x, y, z = point[0], point[1], point[2]
        if x * x + y * y + z * z < min_dist_front_sq:
            drop_front_min_dist += 1
            continue
        if not append_with_rebased_offset(merged, point, front_offset_rebase_ns, stable_index):
            drop_front_offset_overflow += 1
        stable_index += 1

    for point in back_points:
        x, y, z = point[0], point[1], point[2]
        if x * x + y * y + z * z < min_dist_back_sq:
            drop_back_min_dist += 1
            continue
        tx = transform.r00 * x + transform.r01 * y + transform.r02 * z + transform.tx
        ty = transform.r10 * x + transform.r11 * y + transform.r12 * z + transform.ty
        tz = transform.r20 * x + transform.r21 * y + transform.r22 * z + transform.tz
        transformed_point = (tx, ty, tz, point[3], point[4], point[5], point[6])
        if not append_with_rebased_offset(merged, transformed_point, back_offset_rebase_ns, stable_index):
            drop_back_offset_overflow += 1
        stable_index += 1

    merged.sort(key=lambda point: point[0])
    return merged, MergeStats(
        len(merged),
        drop_front_min_dist,
        drop_back_min_dist,
        drop_front_offset_overflow,
        drop_back_offset_overflow,
    )


def verify_stable_offset_order(points: list[tuple[int, int, float, float, float, int, int, int]]) -> None:
    previous_offset = -1
    previous_stable_index = -1
    for point in points:
        offset_time = point[0]
        stable_index = point[1]
        if offset_time < previous_offset:
            raise RuntimeError("offset_time order is not monotonic after stable sort")
        if offset_time == previous_offset and stable_index < previous_stable_index:
            raise RuntimeError("stable sort order was not preserved for duplicate offset_time")
        previous_offset = offset_time
        previous_stable_index = stable_index


def percentile(sorted_values: list[float], quantile: float) -> float:
    index = max(0, min(len(sorted_values) - 1, math.ceil(len(sorted_values) * quantile) - 1))
    return sorted_values[index]


def summarize(latency_ms: list[float]) -> LatencySummary:
    ordered = sorted(latency_ms)
    return LatencySummary(
        len(ordered),
        statistics.median(ordered),
        percentile(ordered, 0.95),
        percentile(ordered, 0.99),
        max(ordered),
        min(ordered),
    )


def run_synthetic(args: argparse.Namespace) -> int:
    pool = build_input_pool(args)
    transform = make_yaw_transform(args.back_yaw_rad, args.back_translation)
    min_dist_front_sq = args.min_dist_front_m * args.min_dist_front_m
    min_dist_back_sq = args.min_dist_back_m * args.min_dist_back_m
    front_offset_rebase_ns = 0
    back_offset_rebase_ns = int(round(args.stamp_diff_ms * NSEC_PER_MSEC))

    for index in range(args.warmup_iterations):
        merged, _ = merge_pair(
            pool[index % len(pool)][0],
            pool[index % len(pool)][1],
            transform,
            min_dist_front_sq,
            min_dist_back_sq,
            front_offset_rebase_ns,
            back_offset_rebase_ns,
        )
        verify_stable_offset_order(merged)

    latencies_ms: list[float] = []
    output_counts: list[int] = []
    total_drop_front_min = 0
    total_drop_back_min = 0
    total_drop_front_overflow = 0
    total_drop_back_overflow = 0

    for index, (front_points, back_points) in enumerate(pool):
        start_ns = time.perf_counter_ns()
        merged, stats = merge_pair(
            front_points,
            back_points,
            transform,
            min_dist_front_sq,
            min_dist_back_sq,
            front_offset_rebase_ns,
            back_offset_rebase_ns,
        )
        end_ns = time.perf_counter_ns()
        if index == 0 or index == len(pool) - 1:
            verify_stable_offset_order(merged)
        latencies_ms.append((end_ns - start_ns) / 1_000_000.0)
        output_counts.append(stats.output_count)
        total_drop_front_min += stats.drop_front_min_dist
        total_drop_back_min += stats.drop_back_min_dist
        total_drop_front_overflow += stats.drop_front_offset_overflow
        total_drop_back_overflow += stats.drop_back_offset_overflow

    summary = summarize(latencies_ms)
    passed = summary.median_ms <= args.max_median_ms and summary.p99_ms <= args.max_p99_ms

    print("mode: synthetic CustomMsg-like merger hot-path")
    print(f"python: {sys.version.split()[0]}")
    print(f"platform: {os.uname().sysname} {os.uname().release} {os.uname().machine}")
    print(f"seed: {args.seed}")
    print(f"front_points: {args.front_points}")
    print(f"back_points: {args.back_points}")
    print(f"min_dist_front_m: {args.min_dist_front_m:.6f}")
    print(f"min_dist_back_m: {args.min_dist_back_m:.6f}")
    print(f"stamp_diff_ms_for_rebase: {args.stamp_diff_ms:.6f}")
    print("operations: native min-distance filters, affine back transform, offset rebasing, uint32 overflow drops, stable sort by offset_time")
    print(f"samples: {summary.samples}")
    print(f"median_ms: {summary.median_ms:.6f}")
    print(f"p95_ms: {summary.p95_ms:.6f}")
    print(f"p99_ms: {summary.p99_ms:.6f}")
    print(f"max_ms: {summary.max_ms:.6f}")
    print(f"min_ms: {summary.min_ms:.6f}")
    print(f"median_output_points: {int(statistics.median(output_counts))}")
    print(f"drop_front_min_dist_total: {total_drop_front_min}")
    print(f"drop_back_min_dist_total: {total_drop_back_min}")
    print(f"drop_front_offset_overflow_total: {total_drop_front_overflow}")
    print(f"drop_back_offset_overflow_total: {total_drop_back_overflow}")
    print(f"threshold_max_median_ms: {args.max_median_ms:.6f}")
    print(f"threshold_max_p99_ms: {args.max_p99_ms:.6f}")
    print(f"result: {'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


def read_proc_jiffies(pid: int) -> int:
    text = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
    fields = text.rsplit(") ", 1)[1].split()
    return int(fields[11]) + int(fields[12])


def read_total_jiffies() -> int:
    first_line = Path("/proc/stat").read_text(encoding="utf-8").splitlines()[0]
    return sum(int(value) for value in first_line.split()[1:])


def read_rss_kb(pid: int) -> int:
    for line in Path(f"/proc/{pid}/status").read_text(encoding="utf-8").splitlines():
        if line.startswith("VmRSS:"):
            return int(line.split()[1])
    raise RuntimeError(f"VmRSS not found for PID {pid}")


def run_live(args: argparse.Namespace) -> int:
    pid = args.live_pid
    if pid is None:
        raise ValueError("--live-pid is required for live mode")
    if not Path(f"/proc/{pid}").exists():
        print(f"mode: live merger_node CPU/RSS")
        print(f"pid: {pid}")
        print("result: BLOCKED")
        print("reason: PID does not exist")
        return 2

    cpu_count = os.cpu_count() or 1
    previous_proc = read_proc_jiffies(pid)
    previous_total = read_total_jiffies()
    rss_samples = [read_rss_kb(pid)]
    cpu_samples: list[float] = []

    for _ in range(args.live_samples):
        time.sleep(args.live_interval)
        current_proc = read_proc_jiffies(pid)
        current_total = read_total_jiffies()
        rss_samples.append(read_rss_kb(pid))
        proc_delta = max(0, current_proc - previous_proc)
        total_delta = max(1, current_total - previous_total)
        cpu_samples.append((proc_delta / total_delta) * cpu_count * 100.0)
        previous_proc = current_proc
        previous_total = current_total

    print("mode: live merger_node CPU/RSS")
    print(f"pid: {pid}")
    print(f"samples: {len(cpu_samples)}")
    print(f"interval_s: {args.live_interval:.3f}")
    print(f"avg_cpu_percent: {sum(cpu_samples) / len(cpu_samples):.3f}")
    print(f"max_cpu_percent: {max(cpu_samples):.3f}")
    print(f"min_rss_kb: {min(rss_samples)}")
    print(f"max_rss_kb: {max(rss_samples)}")
    print(f"rss_delta_kb: {rss_samples[-1] - rss_samples[0]}")
    print("result: PASS")
    return 0


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
        if args.live_pid is not None:
            return run_live(args)
        return run_synthetic(args)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"result: FAIL", file=sys.stderr)
        print(f"reason: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
