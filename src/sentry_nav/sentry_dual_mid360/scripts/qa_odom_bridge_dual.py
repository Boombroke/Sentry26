#!/usr/bin/env python3
"""qa_odom_bridge_dual.py

Verify that odom_bridge publishes a /registered_scan with adequate coverage
of both the front and back hemispheres when dual Mid360 is active.

/registered_scan is published in the ``odom`` frame by odom_bridge.  Hemisphere
classification (x > 0 = front, x < 0 = back) is defined in the ``front_mid360``
frame.  Therefore a TF lookup from the scan's frame to ``front_mid360`` is
required before classifying points.  If the TF cannot be obtained, the script
emits an honest BLOCKED report rather than classifying in the wrong frame.

Exit codes:
  0  PASS   - back hemisphere mean >= --min-back-points over the sampling window
  1  FAIL   - live data received but back hemisphere mean below threshold
  2  BLOCKED - prerequisite missing (no ROS runtime, topic absent, TF unavailable)
  3  usage / argument error (argparse)
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

SCRIPT_VERSION = "0.1.0"

VERDICT_PASS = "PASS"
VERDICT_FAIL = "FAIL"
VERDICT_BLOCKED = "BLOCKED"


@dataclass
class HemisphereReport:
    verdict: str
    reason: str
    cloud_topic: str
    target_frame: str
    duration_s: float
    frames_received: int = 0
    frames_used: int = 0
    mean_total: float = 0.0
    median_total: float = 0.0
    min_total: int = 0
    max_total: int = 0
    mean_front: float = 0.0
    median_front: float = 0.0
    min_front: int = 0
    max_front: int = 0
    mean_back: float = 0.0
    median_back: float = 0.0
    min_back: int = 0
    max_back: int = 0
    back_ratio_mean: float = 0.0
    threshold_min_back: int = 0
    sample_stride: int = 1
    notes: List[str] = field(default_factory=list)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="qa_odom_bridge_dual.py",
        description=(
            "Verify /registered_scan hemisphere coverage for dual Mid360 odom_bridge QA. "
            "Subscribes to a PointCloud2 topic, transforms each scan into the target frame, "
            "and counts front (x>0) vs back (x<0) hemisphere points."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--cloud-topic",
        default="/registered_scan",
        help="PointCloud2 topic to subscribe to",
    )
    parser.add_argument(
        "--target-frame",
        default="front_mid360",
        help="Frame in which hemisphere classification is performed (x>0=front, x<0=back)",
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
        "--min-back-points",
        type=int,
        default=3000,
        help="Minimum mean back-hemisphere point count per scan for PASS",
    )
    parser.add_argument(
        "--tf-timeout",
        type=float,
        default=2.0,
        help="TF lookup timeout in seconds",
    )
    parser.add_argument(
        "--sample-stride",
        type=int,
        default=1,
        help=(
            "Process every Nth point within each scan for performance. "
            "Counts are scaled back up by stride so reported numbers reflect "
            "the full scan. Default 1 = process all points."
        ),
    )
    parser.add_argument(
        "--version",
        action="version",
        version=f"%(prog)s {SCRIPT_VERSION}",
    )
    return parser


def _blocked_report(
    cloud_topic: str,
    target_frame: str,
    duration_s: float,
    threshold: int,
    reason: str,
    notes: Optional[List[str]] = None,
) -> HemisphereReport:
    return HemisphereReport(
        verdict=VERDICT_BLOCKED,
        reason=reason,
        cloud_topic=cloud_topic,
        target_frame=target_frame,
        duration_s=duration_s,
        threshold_min_back=threshold,
        notes=list(notes or []),
    )


def _format_report(report: HemisphereReport) -> str:
    lines = [
        "# qa_odom_bridge_dual.py report",
        "",
        f"- verdict: **{report.verdict}**",
        f"- reason: {report.reason}",
        f"- cloud_topic: `{report.cloud_topic}`",
        f"- target_frame: `{report.target_frame}`",
        f"- duration_s: {report.duration_s:.1f}",
        f"- frames_received: {report.frames_received}",
        f"- frames_used: {report.frames_used}",
        f"- sample_stride: {report.sample_stride}",
        "",
        "## point counts (per scan, after stride scaling)",
        "",
        f"| metric | total | front (x>0) | back (x<0) |",
        f"|--------|-------|-------------|------------|",
        f"| mean   | {report.mean_total:.1f} | {report.mean_front:.1f} | {report.mean_back:.1f} |",
        f"| median | {report.median_total:.1f} | {report.median_front:.1f} | {report.median_back:.1f} |",
        f"| min    | {report.min_total} | {report.min_front} | {report.min_back} |",
        f"| max    | {report.max_total} | {report.max_front} | {report.max_back} |",
        "",
        f"- back_ratio_mean: {report.back_ratio_mean:.3f}",
        f"- threshold_min_back_mean: {report.threshold_min_back}",
        f"- note: total counts front (x>0) + back (x<0); points at x==0 are in total only",
    ]
    if report.notes:
        lines.append("")
        lines.append("## notes")
        lines.append("")
        for note in report.notes:
            lines.append(f"- {note}")
    return "\n".join(lines) + "\n"


def _write_output(path: str, text: str) -> None:
    try:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
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


def run_live(
    cloud_topic: str,
    target_frame: str,
    duration_s: float,
    min_back_points: int,
    tf_timeout: float,
    sample_stride: int,
) -> HemisphereReport:
    try:
        import rclpy  # noqa: WPS433
        from rclpy.node import Node  # noqa: WPS433
        import rclpy.time  # noqa: WPS433
        from rclpy.duration import Duration  # noqa: WPS433
    except ImportError as exc:
        return _blocked_report(
            cloud_topic,
            target_frame,
            duration_s,
            min_back_points,
            "rclpy is not importable; source a ROS2 Jazzy workspace first",
            notes=[f"ImportError: {exc}"],
        )

    try:
        import tf2_ros  # noqa: WPS433
        import tf2_py  # noqa: WPS433
    except ImportError as exc:
        return _blocked_report(
            cloud_topic,
            target_frame,
            duration_s,
            min_back_points,
            "tf2_ros / tf2_py is not importable; source a ROS2 Jazzy workspace first",
            notes=[f"ImportError: {exc}"],
        )

    try:
        from sensor_msgs.msg import PointCloud2  # noqa: WPS433
        from sensor_msgs_py import point_cloud2 as pc2  # noqa: WPS433
    except ImportError as exc:
        return _blocked_report(
            cloud_topic,
            target_frame,
            duration_s,
            min_back_points,
            "sensor_msgs / sensor_msgs_py is not importable",
            notes=[f"ImportError: {exc}"],
        )

    try:
        import tf2_sensor_msgs  # noqa: WPS433
        from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud  # noqa: WPS433
    except ImportError as exc:
        return _blocked_report(
            cloud_topic,
            target_frame,
            duration_s,
            min_back_points,
            "tf2_sensor_msgs is not importable; install ros-jazzy-tf2-sensor-msgs",
            notes=[f"ImportError: {exc}"],
        )

    rclpy.init(args=None)
    node = Node("qa_odom_bridge_dual")
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)

    frames_received = 0
    total_counts: List[int] = []
    front_counts: List[int] = []
    back_counts: List[int] = []
    notes: List[str] = []
    tf_available = False
    tf_source_frame: Optional[str] = None

    def cloud_callback(msg: PointCloud2) -> None:
        nonlocal frames_received, tf_available, tf_source_frame

        frames_received += 1
        source_frame = msg.header.frame_id

        if source_frame == target_frame:
            transformed_msg = msg
            tf_available = True
            tf_source_frame = source_frame
        else:
            try:
                transform = tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=tf_timeout),
                )
                transformed_msg = do_transform_cloud(msg, transform)
                tf_available = True
                tf_source_frame = source_frame
            except Exception as exc:  # noqa: BLE001
                if len(notes) < 3:
                    notes.append(
                        f"TF lookup {source_frame} -> {target_frame} failed: {exc}"
                    )
                return

        stride = max(1, sample_stride)
        total = 0
        front = 0
        back = 0
        try:
            for i, point in enumerate(
                pc2.read_points(transformed_msg, field_names=("x", "y", "z"), skip_nans=True)
            ):
                if i % stride != 0:
                    continue
                total += 1
                if point[0] > 0.0:
                    front += 1
                elif point[0] < 0.0:
                    back += 1
        except Exception as exc:  # noqa: BLE001
            if len(notes) < 3:
                notes.append(f"point parsing error: {exc}")
            return

        total_counts.append(total * stride)
        front_counts.append(front * stride)
        back_counts.append(back * stride)

    sub = node.create_subscription(PointCloud2, cloud_topic, cloud_callback, 10)

    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()

    frames_used = len(total_counts)

    if frames_received == 0:
        return _blocked_report(
            cloud_topic,
            target_frame,
            duration_s,
            min_back_points,
            f"no messages received on {cloud_topic} within {duration_s:.1f}s; "
            "is odom_bridge running?",
            notes=notes + [
                "start the navigation stack (odom_bridge + Point-LIO) before running this QA",
            ],
        )

    if not tf_available or frames_used == 0:
        return _blocked_report(
            cloud_topic,
            target_frame,
            duration_s,
            min_back_points,
            f"TF from scan frame to {target_frame} was never available; "
            "hemisphere classification requires this transform",
            notes=notes + [
                f"frames received: {frames_received}, frames classified: {frames_used}",
                f"ensure robot_state_publisher is running and {target_frame} is in the TF tree",
            ],
        )

    mean_total = statistics.mean(total_counts)
    median_total = statistics.median(total_counts)
    mean_front = statistics.mean(front_counts)
    median_front = statistics.median(front_counts)
    mean_back = statistics.mean(back_counts)
    median_back = statistics.median(back_counts)
    back_ratio = mean_back / mean_total if mean_total > 0 else 0.0

    if mean_back >= min_back_points:
        verdict = VERDICT_PASS
        reason = (
            f"mean back-hemisphere points {mean_back:.1f} >= threshold {min_back_points} "
            f"over {frames_used} frames"
        )
    else:
        verdict = VERDICT_FAIL
        reason = (
            f"mean back-hemisphere points {mean_back:.1f} < threshold {min_back_points} "
            f"over {frames_used} frames"
        )

    if tf_source_frame and tf_source_frame != target_frame:
        notes.append(
            f"scan frame was '{tf_source_frame}'; transformed to '{target_frame}' for classification"
        )
    if sample_stride > 1:
        notes.append(
            f"sample_stride={sample_stride}: every {sample_stride}th point processed; "
            "counts scaled back up by stride"
        )

    return HemisphereReport(
        verdict=verdict,
        reason=reason,
        cloud_topic=cloud_topic,
        target_frame=target_frame,
        duration_s=duration_s,
        frames_received=frames_received,
        frames_used=frames_used,
        mean_total=mean_total,
        median_total=float(median_total),
        min_total=min(total_counts),
        max_total=max(total_counts),
        mean_front=mean_front,
        median_front=float(median_front),
        min_front=min(front_counts),
        max_front=max(front_counts),
        mean_back=mean_back,
        median_back=float(median_back),
        min_back=min(back_counts),
        max_back=max(back_counts),
        back_ratio_mean=back_ratio,
        threshold_min_back=min_back_points,
        sample_stride=sample_stride,
        notes=notes,
    )


def check_tf(
    source_frame: str,
    target_frame: str,
    tf_timeout: float,
) -> str:
    try:
        import rclpy  # noqa: WPS433
        from rclpy.node import Node  # noqa: WPS433
        import rclpy.time  # noqa: WPS433
        from rclpy.duration import Duration  # noqa: WPS433
        import tf2_ros  # noqa: WPS433
    except ImportError as exc:
        return (
            f"BLOCKED: rclpy / tf2_ros not importable — source a ROS2 Jazzy workspace.\n"
            f"ImportError: {exc}\n"
        )

    rclpy.init(args=None)
    node = Node("qa_odom_bridge_dual_tf_check")
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)

    deadline = time.monotonic() + tf_timeout + 1.0
    transform_result = None
    error_msg = None

    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            transform_result = tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=tf_timeout),
            )
            break
        except Exception as exc:  # noqa: BLE001
            error_msg = str(exc)

    node.destroy_node()
    rclpy.shutdown()

    if transform_result is not None:
        t = transform_result.transform.translation
        r = transform_result.transform.rotation
        return (
            f"PASS: TF {source_frame} -> {target_frame} available\n"
            f"  translation: x={t.x:.6f} y={t.y:.6f} z={t.z:.6f}\n"
            f"  rotation:    x={r.x:.6f} y={r.y:.6f} z={r.z:.6f} w={r.w:.6f}\n"
        )
    return (
        f"BLOCKED: TF {source_frame} -> {target_frame} not available within {tf_timeout:.1f}s\n"
        f"  last error: {error_msg}\n"
        f"  action: start robot_state_publisher and ensure {target_frame} is in the TF tree\n"
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    if args.duration <= 0.0:
        parser.error("--duration must be positive")
    if args.min_back_points < 0:
        parser.error("--min-back-points must be non-negative")
    if args.sample_stride < 1:
        parser.error("--sample-stride must be >= 1")
    if args.tf_timeout <= 0.0:
        parser.error("--tf-timeout must be positive")

    report = run_live(
        cloud_topic=args.cloud_topic,
        target_frame=args.target_frame,
        duration_s=args.duration,
        min_back_points=args.min_back_points,
        tf_timeout=args.tf_timeout,
        sample_stride=args.sample_stride,
    )

    text = _format_report(report)
    sys.stdout.write(text)
    sys.stdout.flush()

    if args.output:
        _write_output(args.output, text)

    return _verdict_exit_code(report.verdict)


if __name__ == "__main__":
    sys.exit(main())
