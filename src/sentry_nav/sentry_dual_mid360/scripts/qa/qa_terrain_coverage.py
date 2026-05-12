#!/usr/bin/env python3
"""qa_terrain_coverage.py

Verify that terrain_analysis and terrain_analysis_ext publish adequate terrain
coverage, including rear/back-half coverage, by subscribing to their output
PointCloud2 topics.

terrain_analysis publishes ``terrain_map`` (sensor_msgs/PointCloud2).
terrain_analysis_ext publishes ``terrain_map_ext`` (sensor_msgs/PointCloud2).

Both topics are published in the odom/map-like local terrain frame.  Rear
coverage is classified as points with x < 0 in the message frame (i.e. behind
the robot's forward axis as seen by the terrain node).  This is an approximation
because the terrain frame origin tracks the vehicle position; the classification
is valid when the vehicle's forward axis aligns with the frame's +x axis, which
is the standard convention for terrain_analysis.  If a more precise
classification is needed, a TF lookup from the message frame to ``base_footprint``
would be required; this script documents that limitation in the report rather
than silently misclassifying.

Coverage area is estimated by counting occupied XY grid bins of size
``--coverage-voxel-size`` metres.  No PCL or ROS geometry library is required.

Exit codes:
  0  PASS    - both topics publish and rear coverage >= --min-rear-coverage
  1  FAIL    - live data received but rear coverage below threshold
  2  BLOCKED - prerequisite missing (no ROS runtime, topics absent, import error)
  3  usage / argument error (argparse)
"""

from __future__ import annotations

import argparse
import math
import os
import statistics
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Set, Tuple

SCRIPT_VERSION = "0.1.0"

VERDICT_PASS = "PASS"
VERDICT_FAIL = "FAIL"
VERDICT_BLOCKED = "BLOCKED"


@dataclass
class TopicStats:
    topic: str
    frames_received: int = 0
    mean_point_count: float = 0.0
    median_point_count: float = 0.0
    min_point_count: int = 0
    max_point_count: int = 0
    coverage_area_m2: float = 0.0
    rear_coverage_area_m2: float = 0.0
    rear_point_ratio: float = 0.0
    x_min: float = 0.0
    x_max: float = 0.0
    y_min: float = 0.0
    y_max: float = 0.0
    verdict: str = VERDICT_BLOCKED
    reason: str = "not evaluated"
    notes: List[str] = field(default_factory=list)


@dataclass
class CoverageReport:
    verdict: str
    reason: str
    duration_s: float
    terrain_topic: str
    terrain_ext_topic: str
    voxel_size_m: float
    min_rear_coverage_m2: float
    sample_stride: int = 1
    terrain: TopicStats = field(default_factory=lambda: TopicStats(topic=""))
    terrain_ext: TopicStats = field(default_factory=lambda: TopicStats(topic=""))
    baseline_terrain_area_m2: Optional[float] = None
    baseline_ext_area_m2: Optional[float] = None
    notes: List[str] = field(default_factory=list)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="qa_terrain_coverage.py",
        description=(
            "Verify terrain_analysis and terrain_analysis_ext coverage by subscribing "
            "to their PointCloud2 output topics and estimating XY coverage area and "
            "rear (x<0 in message frame) coverage."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--terrain-topic",
        default="/terrain_map",
        help="terrain_analysis output topic (sensor_msgs/PointCloud2)",
    )
    parser.add_argument(
        "--terrain-ext-topic",
        default="/terrain_map_ext",
        help="terrain_analysis_ext output topic (sensor_msgs/PointCloud2)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Sampling window in seconds",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Path to write the Markdown report (also printed to stdout)",
    )
    parser.add_argument(
        "--coverage-voxel-size",
        type=float,
        default=0.2,
        help="XY grid bin size in metres for coverage area estimation",
    )
    parser.add_argument(
        "--min-rear-coverage",
        type=float,
        default=0.01,
        help=(
            "Minimum rear (x<0 in message frame) coverage area in m^2 for PASS. "
            "Default 0.01 m^2 is intentionally small; any rear terrain data passes."
        ),
    )
    parser.add_argument(
        "--sample-stride",
        type=int,
        default=1,
        help=(
            "Process every Nth point within each scan for performance. "
            "Coverage area is computed from the sampled subset; reported as-is. "
            "Default 1 = process all points."
        ),
    )
    parser.add_argument(
        "--baseline-terrain-area",
        type=float,
        default=None,
        metavar="M2",
        help=(
            "Optional baseline total coverage area in m^2 for terrain_map. "
            "If provided, the report notes the delta but does not gate PASS/FAIL on it."
        ),
    )
    parser.add_argument(
        "--baseline-ext-area",
        type=float,
        default=None,
        metavar="M2",
        help=(
            "Optional baseline total coverage area in m^2 for terrain_map_ext. "
            "If provided, the report notes the delta but does not gate PASS/FAIL on it."
        ),
    )
    parser.add_argument(
        "--version",
        action="version",
        version=f"%(prog)s {SCRIPT_VERSION}",
    )
    return parser


def _validate_args(args: argparse.Namespace, parser: argparse.ArgumentParser) -> None:
    if args.duration <= 0.0:
        parser.error("--duration must be positive")
    if args.coverage_voxel_size <= 0.0:
        parser.error("--coverage-voxel-size must be positive")
    if args.min_rear_coverage < 0.0:
        parser.error("--min-rear-coverage must be non-negative")
    if args.sample_stride < 1:
        parser.error("--sample-stride must be >= 1")
    if args.baseline_terrain_area is not None and args.baseline_terrain_area < 0.0:
        parser.error("--baseline-terrain-area must be non-negative")
    if args.baseline_ext_area is not None and args.baseline_ext_area < 0.0:
        parser.error("--baseline-ext-area must be non-negative")


def _xy_bin(x: float, y: float, voxel_size: float) -> Tuple[int, int]:
    return (int(math.floor(x / voxel_size)), int(math.floor(y / voxel_size)))


def _coverage_area_from_bins(occupied_bins: Set[Tuple[int, int]], voxel_size: float) -> float:
    return len(occupied_bins) * voxel_size * voxel_size


def _compute_topic_stats(
    topic: str,
    point_lists: List[List[Tuple[float, float, float]]],
    voxel_size: float,
    min_rear_coverage: float,
    sample_stride: int,
) -> TopicStats:
    stats = TopicStats(topic=topic)
    if not point_lists:
        stats.verdict = VERDICT_BLOCKED
        stats.reason = f"no messages received on {topic}"
        return stats

    stats.frames_received = len(point_lists)
    counts = [len(pts) for pts in point_lists]
    stats.mean_point_count = statistics.mean(counts)
    stats.median_point_count = float(statistics.median(counts))
    stats.min_point_count = min(counts)
    stats.max_point_count = max(counts)

    all_bins: Set[Tuple[int, int]] = set()
    rear_bins: Set[Tuple[int, int]] = set()
    total_points = 0
    rear_points = 0
    x_vals: List[float] = []
    y_vals: List[float] = []

    stride = max(1, sample_stride)
    for pts in point_lists:
        for idx, (x, y, _z) in enumerate(pts):
            if idx % stride != 0:
                continue
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            b = _xy_bin(x, y, voxel_size)
            all_bins.add(b)
            x_vals.append(x)
            y_vals.append(y)
            total_points += 1
            if x < 0.0:
                rear_bins.add(b)
                rear_points += 1

    stats.coverage_area_m2 = _coverage_area_from_bins(all_bins, voxel_size)
    stats.rear_coverage_area_m2 = _coverage_area_from_bins(rear_bins, voxel_size)
    stats.rear_point_ratio = rear_points / total_points if total_points > 0 else 0.0

    if x_vals:
        stats.x_min = min(x_vals)
        stats.x_max = max(x_vals)
    if y_vals:
        stats.y_min = min(y_vals)
        stats.y_max = max(y_vals)

    if stats.rear_coverage_area_m2 >= min_rear_coverage:
        stats.verdict = VERDICT_PASS
        stats.reason = (
            f"rear coverage {stats.rear_coverage_area_m2:.4f} m^2 >= "
            f"threshold {min_rear_coverage:.4f} m^2 over {stats.frames_received} frames"
        )
    else:
        stats.verdict = VERDICT_FAIL
        stats.reason = (
            f"rear coverage {stats.rear_coverage_area_m2:.4f} m^2 < "
            f"threshold {min_rear_coverage:.4f} m^2 over {stats.frames_received} frames"
        )

    if stride > 1:
        stats.notes.append(
            f"sample_stride={stride}: every {stride}th point processed; "
            "coverage area computed from sampled subset"
        )

    return stats


def _format_topic_section(
    label: str,
    stats: TopicStats,
    baseline_area: Optional[float],
) -> List[str]:
    lines = [
        f"## {label}: `{stats.topic}`",
        "",
        f"- verdict: **{stats.verdict}**",
        f"- reason: {stats.reason}",
        f"- frames_received: {stats.frames_received}",
        "",
        "### point count per frame",
        "",
        "| mean | median | min | max |",
        "|------|--------|-----|-----|",
        (
            f"| {stats.mean_point_count:.1f} | {stats.median_point_count:.1f} | "
            f"{stats.min_point_count} | {stats.max_point_count} |"
        ),
        "",
        "### coverage (XY grid bins, message frame)",
        "",
        f"- total_coverage_area_m2: {stats.coverage_area_m2:.4f}",
        f"- rear_coverage_area_m2 (x<0): {stats.rear_coverage_area_m2:.4f}",
        f"- rear_point_ratio: {stats.rear_point_ratio:.4f}",
        f"- x_extent: [{stats.x_min:.2f}, {stats.x_max:.2f}] m",
        f"- y_extent: [{stats.y_min:.2f}, {stats.y_max:.2f}] m",
    ]
    if baseline_area is not None:
        delta = stats.coverage_area_m2 - baseline_area
        lines.append(
            f"- baseline_total_area_m2: {baseline_area:.4f} "
            f"(delta: {delta:+.4f} m^2; informational only, not gating PASS/FAIL)"
        )
    if stats.notes:
        lines.append("")
        lines.append("### notes")
        lines.append("")
        for note in stats.notes:
            lines.append(f"- {note}")
    return lines


def _format_report(report: CoverageReport) -> str:
    lines = [
        "# qa_terrain_coverage.py report",
        "",
        f"- verdict: **{report.verdict}**",
        f"- reason: {report.reason}",
        f"- duration_s: {report.duration_s:.1f}",
        f"- terrain_topic: `{report.terrain_topic}`",
        f"- terrain_ext_topic: `{report.terrain_ext_topic}`",
        f"- coverage_voxel_size_m: {report.voxel_size_m:.4f}",
        f"- min_rear_coverage_m2: {report.min_rear_coverage_m2:.4f}",
        f"- sample_stride: {report.sample_stride}",
        "",
        "## frame classification note",
        "",
        "- Rear coverage is classified as points with x < 0 in the message frame.",
        "- terrain_analysis publishes in the odom/map-like local terrain frame where",
        "  the origin tracks the vehicle position and +x aligns with the vehicle forward axis.",
        "- This classification is valid when the vehicle forward axis aligns with frame +x.",
        "- For precise classification, a TF lookup from the message frame to base_footprint",
        "  would be required; this script does not perform that lookup.",
        "",
    ]

    lines.extend(
        _format_topic_section("terrain_map", report.terrain, report.baseline_terrain_area_m2)
    )
    lines.append("")
    lines.extend(
        _format_topic_section("terrain_map_ext", report.terrain_ext, report.baseline_ext_area_m2)
    )

    if report.notes:
        lines.append("")
        lines.append("## global notes")
        lines.append("")
        for note in report.notes:
            lines.append(f"- {note}")

    return "\n".join(lines) + "\n"


def _write_output(path: str, text: str) -> None:
    try:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(text)
    except OSError as exc:
        sys.stderr.write(f"WARNING: could not write --output {path}: {exc}\n")


def _verdict_exit_code(verdict: str) -> int:
    if verdict == VERDICT_PASS:
        return 0
    if verdict == VERDICT_FAIL:
        return 1
    return 2


def _blocked_report(
    reason: str,
    duration_s: float,
    terrain_topic: str,
    terrain_ext_topic: str,
    voxel_size: float,
    min_rear: float,
    sample_stride: int,
    notes: Optional[List[str]] = None,
    baseline_terrain: Optional[float] = None,
    baseline_ext: Optional[float] = None,
) -> CoverageReport:
    blocked_stats_terrain = TopicStats(
        topic=terrain_topic,
        verdict=VERDICT_BLOCKED,
        reason=reason,
    )
    blocked_stats_ext = TopicStats(
        topic=terrain_ext_topic,
        verdict=VERDICT_BLOCKED,
        reason=reason,
    )
    return CoverageReport(
        verdict=VERDICT_BLOCKED,
        reason=reason,
        duration_s=duration_s,
        terrain_topic=terrain_topic,
        terrain_ext_topic=terrain_ext_topic,
        voxel_size_m=voxel_size,
        min_rear_coverage_m2=min_rear,
        sample_stride=sample_stride,
        terrain=blocked_stats_terrain,
        terrain_ext=blocked_stats_ext,
        baseline_terrain_area_m2=baseline_terrain,
        baseline_ext_area_m2=baseline_ext,
        notes=list(notes or []),
    )


def run_live(
    terrain_topic: str,
    terrain_ext_topic: str,
    duration_s: float,
    voxel_size: float,
    min_rear_coverage: float,
    sample_stride: int,
    baseline_terrain_area: Optional[float],
    baseline_ext_area: Optional[float],
) -> CoverageReport:
    try:
        import rclpy  # noqa: WPS433
        from rclpy.node import Node  # noqa: WPS433
    except ImportError as exc:
        return _blocked_report(
            reason="rclpy is not importable; source a ROS2 Jazzy workspace first",
            duration_s=duration_s,
            terrain_topic=terrain_topic,
            terrain_ext_topic=terrain_ext_topic,
            voxel_size=voxel_size,
            min_rear=min_rear_coverage,
            sample_stride=sample_stride,
            notes=[
                f"ImportError: {exc}",
                "Run: source /opt/ros/jazzy/setup.bash",
                "Then re-run this script.",
            ],
            baseline_terrain=baseline_terrain_area,
            baseline_ext=baseline_ext_area,
        )

    try:
        from sensor_msgs.msg import PointCloud2  # noqa: WPS433
        from sensor_msgs_py import point_cloud2 as pc2  # noqa: WPS433
    except ImportError as exc:
        return _blocked_report(
            reason="sensor_msgs / sensor_msgs_py is not importable",
            duration_s=duration_s,
            terrain_topic=terrain_topic,
            terrain_ext_topic=terrain_ext_topic,
            voxel_size=voxel_size,
            min_rear=min_rear_coverage,
            sample_stride=sample_stride,
            notes=[
                f"ImportError: {exc}",
                "Run: source /opt/ros/jazzy/setup.bash",
            ],
            baseline_terrain=baseline_terrain_area,
            baseline_ext=baseline_ext_area,
        )

    rclpy.init(args=None)
    node = Node("qa_terrain_coverage")

    terrain_frames: List[List[Tuple[float, float, float]]] = []
    terrain_ext_frames: List[List[Tuple[float, float, float]]] = []
    parse_errors: List[str] = []

    def _make_callback(
        frame_list: List[List[Tuple[float, float, float]]],
    ):
        def callback(msg: PointCloud2) -> None:
            pts: List[Tuple[float, float, float]] = []
            try:
                for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                    pts.append((float(point[0]), float(point[1]), float(point[2])))
            except Exception as exc:  # noqa: BLE001
                if len(parse_errors) < 3:
                    parse_errors.append(f"point parse error: {exc}")
                return
            frame_list.append(pts)

        return callback

    sub_terrain = node.create_subscription(
        PointCloud2, terrain_topic, _make_callback(terrain_frames), 10
    )
    sub_ext = node.create_subscription(
        PointCloud2, terrain_ext_topic, _make_callback(terrain_ext_frames), 10
    )

    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()

    terrain_received = len(terrain_frames)
    ext_received = len(terrain_ext_frames)

    if terrain_received == 0 and ext_received == 0:
        return _blocked_report(
            reason=(
                f"no messages received on either {terrain_topic} or {terrain_ext_topic} "
                f"within {duration_s:.1f}s; are terrain_analysis nodes running?"
            ),
            duration_s=duration_s,
            terrain_topic=terrain_topic,
            terrain_ext_topic=terrain_ext_topic,
            voxel_size=voxel_size,
            min_rear=min_rear_coverage,
            sample_stride=sample_stride,
            notes=[
                "Start the full navigation stack before running this QA:",
                "  Terminal 1: ros2 launch rmu_gazebo_simulator bringup_sim.launch.py",
                "  Terminal 2: ros2 launch sentry_nav_bringup rm_navigation_simulation_launch.py "
                "world:=rmuc_2026 slam:=True",
                "terrain_analysis subscribes to /registered_scan and /lidar_odometry "
                "(published by odom_bridge).",
                "terrain_analysis_ext subscribes to /terrain_map, /registered_scan, "
                "and /lidar_odometry.",
            ],
            baseline_terrain=baseline_terrain_area,
            baseline_ext=baseline_ext_area,
        )

    global_notes: List[str] = list(parse_errors)

    if terrain_received == 0:
        terrain_stats = TopicStats(
            topic=terrain_topic,
            verdict=VERDICT_BLOCKED,
            reason=f"no messages received on {terrain_topic} within {duration_s:.1f}s",
            notes=[
                "terrain_analysis must be running and receiving /registered_scan + /lidar_odometry",
            ],
        )
    else:
        terrain_stats = _compute_topic_stats(
            terrain_topic, terrain_frames, voxel_size, min_rear_coverage, sample_stride
        )

    if ext_received == 0:
        ext_stats = TopicStats(
            topic=terrain_ext_topic,
            verdict=VERDICT_BLOCKED,
            reason=f"no messages received on {terrain_ext_topic} within {duration_s:.1f}s",
            notes=[
                "terrain_analysis_ext must be running and receiving /terrain_map + /registered_scan",
            ],
        )
    else:
        ext_stats = _compute_topic_stats(
            terrain_ext_topic, terrain_ext_frames, voxel_size, min_rear_coverage, sample_stride
        )

    verdicts = {terrain_stats.verdict, ext_stats.verdict}
    if VERDICT_BLOCKED in verdicts:
        overall_verdict = VERDICT_BLOCKED
        overall_reason = "one or more topics are BLOCKED; see per-topic sections"
    elif VERDICT_FAIL in verdicts:
        overall_verdict = VERDICT_FAIL
        overall_reason = "one or more topics failed rear coverage threshold; see per-topic sections"
    else:
        overall_verdict = VERDICT_PASS
        overall_reason = (
            f"both topics pass rear coverage >= {min_rear_coverage:.4f} m^2"
        )

    return CoverageReport(
        verdict=overall_verdict,
        reason=overall_reason,
        duration_s=duration_s,
        terrain_topic=terrain_topic,
        terrain_ext_topic=terrain_ext_topic,
        voxel_size_m=voxel_size,
        min_rear_coverage_m2=min_rear_coverage,
        sample_stride=sample_stride,
        terrain=terrain_stats,
        terrain_ext=ext_stats,
        baseline_terrain_area_m2=baseline_terrain_area,
        baseline_ext_area_m2=baseline_ext_area,
        notes=global_notes,
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    _validate_args(args, parser)

    report = run_live(
        terrain_topic=args.terrain_topic,
        terrain_ext_topic=args.terrain_ext_topic,
        duration_s=args.duration,
        voxel_size=args.coverage_voxel_size,
        min_rear_coverage=args.min_rear_coverage,
        sample_stride=args.sample_stride,
        baseline_terrain_area=args.baseline_terrain_area,
        baseline_ext_area=args.baseline_ext_area,
    )

    text = _format_report(report)
    sys.stdout.write(text)
    sys.stdout.flush()

    if args.output:
        _write_output(args.output, text)

    return _verdict_exit_code(report.verdict)


if __name__ == "__main__":
    sys.exit(main())
