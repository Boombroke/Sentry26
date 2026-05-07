#!/usr/bin/env python3
"""analyze_map_odom_stability.py

Sample the ``map -> odom`` TF over a configurable window and report
translation jitter in centimetres.  Intended for verifying that
small_gicp_relocalization produces a stable correction when the robot is
stationary with a dual Mid360 prior PCD loaded.

The script also attempts to read one message from ``/relocalization_diagnostics``
(if the topic exists) and appends the raw content to the report.  The current
small_gicp_relocalization node does NOT publish this topic; GICP quality
metrics are only available in node logs.

Exit codes:
  0  PASS    - TF sampled successfully and observed jitter <= --max-jitter-cm
  1  FAIL    - TF sampled successfully but jitter exceeds threshold
  2  BLOCKED - prerequisite missing (no ROS runtime, TF unavailable, import
               error, or no prior PCD / dual Mid360 runtime)
  3  usage / argument error (argparse)
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

SCRIPT_VERSION = "0.1.0"

VERDICT_PASS = "PASS"
VERDICT_FAIL = "FAIL"
VERDICT_BLOCKED = "BLOCKED"


@dataclass
class TfSample:
    timestamp_s: float
    x: float
    y: float
    z: float


@dataclass
class StabilityReport:
    verdict: str
    reason: str
    duration_s: float
    fixed_frame: str
    moving_frame: str
    sample_count: int
    max_jitter_cm: float
    observed_jitter_cm: float
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float
    diagnostics_snippet: Optional[str]
    notes: List[str] = field(default_factory=list)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="analyze_map_odom_stability.py",
        description=(
            "Sample the map->odom TF over --duration seconds and report "
            "translation jitter in centimetres.  Exits 0 (PASS), 1 (FAIL), "
            "or 2 (BLOCKED) when prerequisites are missing."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=300.0,
        help="Sampling window in seconds (5 min for live validation)",
    )
    parser.add_argument(
        "--max-jitter-cm",
        type=float,
        default=5.0,
        help="Maximum allowed translation jitter in centimetres for PASS",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Path to write the Markdown report (also printed to stdout)",
    )
    parser.add_argument(
        "--fixed-frame",
        default="map",
        help="Fixed TF frame (parent)",
    )
    parser.add_argument(
        "--moving-frame",
        default="odom",
        help="Moving TF frame (child)",
    )
    parser.add_argument(
        "--sample-period",
        type=float,
        default=1.0,
        help="Seconds between TF lookups",
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
    if args.max_jitter_cm < 0.0:
        parser.error("--max-jitter-cm must be non-negative")
    if args.sample_period <= 0.0:
        parser.error("--sample-period must be positive")


def _compute_jitter(samples: List[TfSample]) -> Tuple[float, float, float, float, float, float, float]:
    """Return (jitter_cm, x_min, x_max, y_min, y_max, z_min, z_max).

    Jitter is the L2 norm of per-axis range (max - min), converted to cm.
    This is a conservative upper bound: it captures the worst-case excursion
    across the entire sampling window rather than a rolling window statistic.
    """
    xs = [s.x for s in samples]
    ys = [s.y for s in samples]
    zs = [s.z for s in samples]
    x_range = max(xs) - min(xs)
    y_range = max(ys) - min(ys)
    z_range = max(zs) - min(zs)
    jitter_m = math.sqrt(x_range ** 2 + y_range ** 2 + z_range ** 2)
    return (
        jitter_m * 100.0,
        min(xs), max(xs),
        min(ys), max(ys),
        min(zs), max(zs),
    )


def _format_report(report: StabilityReport) -> str:
    lines = [
        "# analyze_map_odom_stability.py report",
        "",
        f"- verdict: **{report.verdict}**",
        f"- reason: {report.reason}",
        f"- duration_s: {report.duration_s:.1f}",
        f"- fixed_frame: `{report.fixed_frame}`",
        f"- moving_frame: `{report.moving_frame}`",
        f"- sample_count: {report.sample_count}",
        f"- max_jitter_cm (threshold): {report.max_jitter_cm:.2f}",
        f"- observed_jitter_cm: {report.observed_jitter_cm:.4f}",
        "",
        "## translation extent (metres)",
        "",
        "| axis | min | max | range |",
        "|------|-----|-----|-------|",
        (
            f"| x | {report.x_min:.6f} | {report.x_max:.6f} "
            f"| {report.x_max - report.x_min:.6f} |"
        ),
        (
            f"| y | {report.y_min:.6f} | {report.y_max:.6f} "
            f"| {report.y_max - report.y_min:.6f} |"
        ),
        (
            f"| z | {report.z_min:.6f} | {report.z_max:.6f} "
            f"| {report.z_max - report.z_min:.6f} |"
        ),
        "",
    ]

    if report.diagnostics_snippet is not None:
        lines += [
            "## /relocalization_diagnostics (one message)",
            "",
            "```",
            report.diagnostics_snippet.strip(),
            "```",
            "",
        ]
    else:
        lines += [
            "## /relocalization_diagnostics",
            "",
            "- topic not available or no message received within sampling window",
            "- the current small_gicp_relocalization node does NOT publish this topic",
            "- GICP quality metrics (inlier_ratio, fitness_error) are only in node logs",
            "- to capture on target hardware:",
            "  `ros2 topic list | grep reloc`",
            "  `ros2 node info /small_gicp_relocalization`",
            "",
        ]

    if report.notes:
        lines += ["## notes", ""]
        for note in report.notes:
            lines.append(f"- {note}")
        lines.append("")

    lines += [
        "## rerun instructions (live validation)",
        "",
        "Prerequisites:",
        "  1. Dual Mid360 hardware connected (IPs 192.168.1.144 and 192.168.1.145 reachable)",
        "  2. Prior PCD built from dual Mid360 scan (T21 output)",
        "     NOTE: single-Mid360 PCD is NOT compatible with dual Mid360 registered_scan",
        "  3. Full navigation stack running in localization mode:",
        "     ```",
        "     source install/setup.bash",
        "     ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py \\",
        "       slam:=False use_dual_mid360:=True",
        "     ```",
        "  4. small_gicp_relocalization running with `enable_periodic_relocalization: true`",
        "  5. Robot stationary for the full sampling window",
        "",
        "Rerun command (5-minute window, 5 cm threshold):",
        "  ```",
        "  source install/setup.bash",
        "  python3 src/sentry_nav/sentry_dual_mid360/scripts/analyze_map_odom_stability.py \\",
        "    --duration 300 --max-jitter-cm 5.0 \\",
        "    --output .sisyphus/evidence/task-19-stability.md",
        "  ```",
        "",
        "GICP diagnostics (node logs, not a topic):",
        "  ```",
        "  ros2 topic list | grep reloc",
        "  ros2 node info /small_gicp_relocalization",
        "  # look for lines containing inlier_ratio and fitness_error in node output",
        "  # acceptance: inlier_ratio > 0.5, fitness_error < 0.1",
        "  ```",
    ]

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
    fixed_frame: str,
    moving_frame: str,
    max_jitter_cm: float,
    notes: Optional[List[str]] = None,
) -> StabilityReport:
    return StabilityReport(
        verdict=VERDICT_BLOCKED,
        reason=reason,
        duration_s=duration_s,
        fixed_frame=fixed_frame,
        moving_frame=moving_frame,
        sample_count=0,
        max_jitter_cm=max_jitter_cm,
        observed_jitter_cm=0.0,
        x_min=0.0,
        x_max=0.0,
        y_min=0.0,
        y_max=0.0,
        z_min=0.0,
        z_max=0.0,
        diagnostics_snippet=None,
        notes=list(notes or []),
    )


def _try_read_diagnostics_once(node) -> Optional[str]:  # type: ignore[no-untyped-def]
    """Attempt one message from /relocalization_diagnostics (3 s timeout).

    Returns the string representation of the message, or None if the topic
    is absent or no message arrives.  The current node does not publish this
    topic; this function will always return None in practice.
    """
    try:
        from diagnostic_msgs.msg import DiagnosticArray  # noqa: WPS433
    except ImportError:
        return None

    received: List[str] = []

    def _cb(msg: DiagnosticArray) -> None:  # type: ignore[no-untyped-def]
        if not received:
            received.append(str(msg))

    sub = node.create_subscription(DiagnosticArray, "/relocalization_diagnostics", _cb, 1)
    deadline = time.monotonic() + 3.0
    try:
        import rclpy  # noqa: WPS433
        while time.monotonic() < deadline and not received:
            rclpy.spin_once(node, timeout_sec=0.1)
    except Exception:  # noqa: BLE001
        pass
    finally:
        node.destroy_subscription(sub)

    return received[0] if received else None


def run_live(
    fixed_frame: str,
    moving_frame: str,
    duration_s: float,
    sample_period_s: float,
    max_jitter_cm: float,
) -> StabilityReport:
    try:
        import rclpy  # noqa: WPS433
        import rclpy.time  # noqa: WPS433
        from rclpy.duration import Duration  # noqa: WPS433
        from rclpy.node import Node  # noqa: WPS433
    except ImportError as exc:
        return _blocked_report(
            reason="rclpy is not importable; source a ROS2 Jazzy workspace first",
            duration_s=duration_s,
            fixed_frame=fixed_frame,
            moving_frame=moving_frame,
            max_jitter_cm=max_jitter_cm,
            notes=[
                f"ImportError: {exc}",
                "Run: source /opt/ros/jazzy/setup.bash",
                "Then re-run this script.",
            ],
        )

    try:
        import tf2_ros  # noqa: WPS433
        from geometry_msgs.msg import TransformStamped  # noqa: WPS433
    except ImportError as exc:
        return _blocked_report(
            reason="tf2_ros is not importable; source a ROS2 Jazzy workspace first",
            duration_s=duration_s,
            fixed_frame=fixed_frame,
            moving_frame=moving_frame,
            max_jitter_cm=max_jitter_cm,
            notes=[
                f"ImportError: {exc}",
                "Run: source /opt/ros/jazzy/setup.bash",
            ],
        )

    rclpy.init(args=None)
    node = Node("analyze_map_odom_stability")
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)

    samples: List[TfSample] = []
    lookup_errors: List[str] = []

    deadline = time.monotonic() + duration_s
    next_sample = time.monotonic() + sample_period_s

    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.05, sample_period_s * 0.1))
        now = time.monotonic()
        if now >= next_sample:
            next_sample = now + sample_period_s
            try:
                t: TransformStamped = tf_buffer.lookup_transform(
                    fixed_frame,
                    moving_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.5),
                )
                samples.append(
                    TfSample(
                        timestamp_s=now,
                        x=t.transform.translation.x,
                        y=t.transform.translation.y,
                        z=t.transform.translation.z,
                    )
                )
            except Exception as exc:  # noqa: BLE001
                msg = str(exc)
                if len(lookup_errors) < 5:
                    lookup_errors.append(msg)

    diagnostics_snippet = _try_read_diagnostics_once(node)

    node.destroy_node()
    rclpy.shutdown()

    if not samples:
        return _blocked_report(
            reason=(
                f"no TF lookup succeeded for {fixed_frame} -> {moving_frame} "
                f"within {duration_s:.1f}s; is the navigation stack running?"
            ),
            duration_s=duration_s,
            fixed_frame=fixed_frame,
            moving_frame=moving_frame,
            max_jitter_cm=max_jitter_cm,
            notes=lookup_errors + [
                "Ensure the full navigation stack is running in localization mode:",
                "  ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py "
                "slam:=False use_dual_mid360:=True",
                "small_gicp_relocalization must have published at least one "
                "map->odom correction before this TF becomes available.",
                "Prior PCD must be a dual Mid360 map (T21 output); "
                "single-Mid360 PCD is incompatible.",
            ],
        )

    jitter_cm, x_min, x_max, y_min, y_max, z_min, z_max = _compute_jitter(samples)

    notes: List[str] = []
    if lookup_errors:
        notes.append(f"{len(lookup_errors)} TF lookup error(s) during sampling (first 5 shown):")
        notes.extend(lookup_errors)

    if jitter_cm <= max_jitter_cm:
        verdict = VERDICT_PASS
        reason = (
            f"observed jitter {jitter_cm:.4f} cm <= threshold {max_jitter_cm:.2f} cm "
            f"over {len(samples)} samples in {duration_s:.1f}s"
        )
    else:
        verdict = VERDICT_FAIL
        reason = (
            f"observed jitter {jitter_cm:.4f} cm > threshold {max_jitter_cm:.2f} cm "
            f"over {len(samples)} samples in {duration_s:.1f}s"
        )

    return StabilityReport(
        verdict=verdict,
        reason=reason,
        duration_s=duration_s,
        fixed_frame=fixed_frame,
        moving_frame=moving_frame,
        sample_count=len(samples),
        max_jitter_cm=max_jitter_cm,
        observed_jitter_cm=jitter_cm,
        x_min=x_min,
        x_max=x_max,
        y_min=y_min,
        y_max=y_max,
        z_min=z_min,
        z_max=z_max,
        diagnostics_snippet=diagnostics_snippet,
        notes=notes,
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    _validate_args(args, parser)

    report = run_live(
        fixed_frame=args.fixed_frame,
        moving_frame=args.moving_frame,
        duration_s=args.duration,
        sample_period_s=args.sample_period,
        max_jitter_cm=args.max_jitter_cm,
    )

    text = _format_report(report)
    sys.stdout.write(text)
    sys.stdout.flush()

    if args.output:
        _write_output(args.output, text)

    return _verdict_exit_code(report.verdict)


if __name__ == "__main__":
    sys.exit(main())
