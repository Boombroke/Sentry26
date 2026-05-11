#!/usr/bin/env python3
"""analyze_static_drift.py.

Compute planar static drift from a rosbag2 recording of /odometry for the
dual Mid360 static bench test (Task 16).

Given a rosbag2 directory that contains odometry messages captured while the
robot was completely stationary, this tool computes:

  * bag duration in seconds (first-message to last-message header stamp)
  * planar displacement between the first and last odometry pose:
        dx   = x_end - x_start
        dy   = y_end - y_start
        d_cm = sqrt(dx^2 + dy^2) * 100
  * drift rate in cm / minute:
        drift_cm_per_min = d_cm / (duration_s / 60.0)

The drift rate is compared against ``--max-drift-cm-per-min`` to emit one of
three honest verdicts:

  * PASS     - drift below threshold
  * FAIL     - drift at or above threshold (or bag malformed/empty)
  * BLOCKED  - prerequisites missing (e.g. ``rosbag2_py`` not importable,
               bag path missing, odometry topic absent). A BLOCKED verdict
               is NOT a quality failure; it says the measurement could not
               be taken. The operator must resolve the blocker and re-run.

Exit codes:
  0  PASS
  1  FAIL
  2  BLOCKED
  3  usage / argument error

The module MUST remain import-safe without a ROS environment: ``rosbag2_py``
and ``rosidl_runtime_py`` / ``nav_msgs`` are imported lazily inside the
analysis path so that ``--help`` works on any Python 3 interpreter.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

SCRIPT_VERSION = "0.1.0"

VERDICT_PASS = "PASS"
VERDICT_FAIL = "FAIL"
VERDICT_BLOCKED = "BLOCKED"


@dataclass
class DriftReport:
    verdict: str
    reason: str
    bag_path: str
    topic: str
    sample_count: int = 0
    duration_s: float = 0.0
    start_x: float = 0.0
    start_y: float = 0.0
    end_x: float = 0.0
    end_y: float = 0.0
    displacement_cm: float = 0.0
    drift_cm_per_min: float = 0.0
    threshold_cm_per_min: float = 0.0
    notes: List[str] = field(default_factory=list)

    def as_dict(self) -> dict:
        return {
            "verdict": self.verdict,
            "reason": self.reason,
            "bag_path": self.bag_path,
            "topic": self.topic,
            "sample_count": self.sample_count,
            "duration_s": self.duration_s,
            "start_x": self.start_x,
            "start_y": self.start_y,
            "end_x": self.end_x,
            "end_y": self.end_y,
            "displacement_cm": self.displacement_cm,
            "drift_cm_per_min": self.drift_cm_per_min,
            "threshold_cm_per_min": self.threshold_cm_per_min,
            "notes": list(self.notes),
        }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="analyze_static_drift.py",
        description=(
            "Compute planar static drift (cm/min) from a rosbag2 recording "
            "of /odometry captured while the robot was stationary."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--bag", required=False, default=None,
                        help="Path to a rosbag2 directory (contains "
                             "metadata.yaml + .db3/.mcap). Required unless "
                             "--help is supplied.")
    parser.add_argument("--odometry-topic", default="/odometry",
                        help="Odometry topic name to analyze")
    parser.add_argument("--max-drift-cm-per-min", type=float, default=5.0,
                        help="PASS/FAIL threshold for drift rate "
                             "(cm per minute)")
    parser.add_argument("--output", default=None,
                        help="Optional path to write the Markdown report; "
                             "report is also printed to stdout")
    parser.add_argument("--json-output", default=None,
                        help="Optional path to write a machine-readable JSON "
                             "summary alongside the Markdown report")
    parser.add_argument("--min-samples", type=int, default=2,
                        help="Minimum odometry samples required for a "
                             "meaningful verdict (otherwise BLOCKED)")
    parser.add_argument("--version", action="version",
                        version=f"%(prog)s {SCRIPT_VERSION}")
    return parser


def _blocked(bag: str, topic: str, threshold: float, reason: str,
             notes: Optional[List[str]] = None) -> DriftReport:
    return DriftReport(
        verdict=VERDICT_BLOCKED,
        reason=reason,
        bag_path=bag,
        topic=topic,
        threshold_cm_per_min=threshold,
        notes=list(notes or []),
    )


def _fail(bag: str, topic: str, threshold: float, reason: str,
          notes: Optional[List[str]] = None, **kwargs) -> DriftReport:
    report = DriftReport(
        verdict=VERDICT_FAIL,
        reason=reason,
        bag_path=bag,
        topic=topic,
        threshold_cm_per_min=threshold,
        notes=list(notes or []),
    )
    for key, value in kwargs.items():
        setattr(report, key, value)
    return report


def analyze_bag(bag_path: str, odometry_topic: str,
                threshold_cm_per_min: float,
                min_samples: int = 2) -> DriftReport:
    """Open a rosbag2 directory and compute the static drift report.

    The ROS-specific imports are deferred so that ``--help`` and unit tests
    can import this module on machines without ROS installed.
    """
    notes: List[str] = []

    if not bag_path:
        return _blocked(bag_path or "(none)", odometry_topic,
                        threshold_cm_per_min,
                        "no bag path supplied")

    if not os.path.isdir(bag_path):
        return _blocked(bag_path, odometry_topic, threshold_cm_per_min,
                        f"bag directory does not exist: {bag_path}",
                        notes=["expected a rosbag2 directory containing "
                               "metadata.yaml + storage file"])

    metadata_yaml = os.path.join(bag_path, "metadata.yaml")
    if not os.path.isfile(metadata_yaml):
        notes.append(
            f"metadata.yaml not found under {bag_path}; rosbag2 reader "
            "may still work if the directory is storage-only, but this is "
            "unusual"
        )

    # Lazy imports: these raise ImportError outside a ROS environment.
    try:
        import rosbag2_py  # noqa: WPS433 - intentional lazy import
        from rclpy.serialization import deserialize_message  # noqa: WPS433
        from rosidl_runtime_py.utilities import get_message  # noqa: WPS433
    except ImportError as exc:
        return _blocked(
            bag_path, odometry_topic, threshold_cm_per_min,
            "required ROS2 Python packages are not importable "
            "(rosbag2_py / rclpy / rosidl_runtime_py)",
            notes=[
                f"underlying ImportError: {exc}",
                "source a ROS2 Jazzy workspace before running the "
                "analyzer: source /opt/ros/jazzy/setup.bash",
            ],
        )

    # Detect storage format from files on disk; rosbag2_py needs an explicit
    # storage_id for some versions.
    storage_id = _detect_storage_id(bag_path)
    if storage_id is None:
        return _blocked(
            bag_path, odometry_topic, threshold_cm_per_min,
            "unable to detect rosbag2 storage format "
            "(no .db3 / .mcap files found)",
            notes=[
                "supply a rosbag2 directory written by `ros2 bag record`",
            ],
        )

    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(storage_options, converter_options)
    except Exception as exc:  # noqa: BLE001
        return _blocked(
            bag_path, odometry_topic, threshold_cm_per_min,
            f"rosbag2_py failed to open bag: {exc}",
            notes=[
                "ensure the bag was written by a compatible rosbag2 "
                "version (Jazzy writes sqlite3 by default)",
            ],
        )

    topic_type_map = {}
    try:
        for topic_meta in reader.get_all_topics_and_types():
            topic_type_map[topic_meta.name] = topic_meta.type
    except Exception as exc:  # noqa: BLE001
        notes.append(f"get_all_topics_and_types failed: {exc}")

    if odometry_topic not in topic_type_map:
        available = sorted(topic_type_map.keys())
        return _blocked(
            bag_path, odometry_topic, threshold_cm_per_min,
            f"odometry topic not found in bag: {odometry_topic}",
            notes=[
                f"topics present in bag: {available}",
                "use --odometry-topic to point at the correct topic name",
            ],
        )

    odometry_type_name = topic_type_map[odometry_topic]
    try:
        odometry_type = get_message(odometry_type_name)
    except Exception as exc:  # noqa: BLE001
        return _blocked(
            bag_path, odometry_topic, threshold_cm_per_min,
            f"cannot resolve message type {odometry_type_name}: {exc}",
            notes=["source a workspace where this message type is built"],
        )

    first_stamp_ns: Optional[int] = None
    last_stamp_ns: Optional[int] = None
    first_x: Optional[float] = None
    first_y: Optional[float] = None
    last_x: Optional[float] = None
    last_y: Optional[float] = None
    count = 0

    try:
        while reader.has_next():
            topic, raw_msg, bag_ns = reader.read_next()
            if topic != odometry_topic:
                continue
            try:
                msg = deserialize_message(raw_msg, odometry_type)
            except Exception as exc:  # noqa: BLE001
                notes.append(f"deserialize_message failed (skipping): {exc}")
                continue

            # nav_msgs/msg/Odometry.pose.pose.position.{x,y}
            pose = getattr(msg, "pose", None)
            inner_pose = getattr(pose, "pose", None) if pose is not None else None
            position = getattr(inner_pose, "position", None) if inner_pose is not None else None
            if position is None:
                notes.append("message missing pose.pose.position; skipping")
                continue

            x = float(getattr(position, "x", 0.0))
            y = float(getattr(position, "y", 0.0))

            header = getattr(msg, "header", None)
            stamp = getattr(header, "stamp", None) if header is not None else None
            if stamp is not None:
                stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            else:
                stamp_ns = int(bag_ns)

            if first_stamp_ns is None:
                first_stamp_ns = stamp_ns
                first_x = x
                first_y = y
            last_stamp_ns = stamp_ns
            last_x = x
            last_y = y
            count += 1
    except Exception as exc:  # noqa: BLE001
        return _blocked(
            bag_path, odometry_topic, threshold_cm_per_min,
            f"error while reading bag messages: {exc}",
            notes=notes,
        )

    if count < max(2, min_samples):
        return _blocked(
            bag_path, odometry_topic, threshold_cm_per_min,
            f"not enough odometry samples to measure drift "
            f"({count} < {max(2, min_samples)})",
            notes=notes + [
                "increase --duration when recording the bag, or verify "
                "that /odometry was actually publishing during capture",
            ],
        )

    assert first_stamp_ns is not None and last_stamp_ns is not None
    assert first_x is not None and first_y is not None
    assert last_x is not None and last_y is not None

    duration_s = (last_stamp_ns - first_stamp_ns) / 1e9
    if duration_s <= 0.0:
        return _blocked(
            bag_path, odometry_topic, threshold_cm_per_min,
            f"non-positive bag duration: {duration_s:.6f}s",
            notes=notes + [
                "bag timestamps are not monotonic; re-record with a clean "
                "ros2 bag record invocation",
            ],
        )

    dx = last_x - first_x
    dy = last_y - first_y
    displacement_cm = math.sqrt(dx * dx + dy * dy) * 100.0
    drift_cm_per_min = displacement_cm / (duration_s / 60.0)

    if drift_cm_per_min < threshold_cm_per_min:
        verdict = VERDICT_PASS
        reason = (f"drift {drift_cm_per_min:.3f} cm/min < threshold "
                  f"{threshold_cm_per_min:.3f} cm/min over {duration_s:.2f}s")
    else:
        verdict = VERDICT_FAIL
        reason = (f"drift {drift_cm_per_min:.3f} cm/min >= threshold "
                  f"{threshold_cm_per_min:.3f} cm/min over {duration_s:.2f}s")

    return DriftReport(
        verdict=verdict,
        reason=reason,
        bag_path=bag_path,
        topic=odometry_topic,
        sample_count=count,
        duration_s=duration_s,
        start_x=first_x,
        start_y=first_y,
        end_x=last_x,
        end_y=last_y,
        displacement_cm=displacement_cm,
        drift_cm_per_min=drift_cm_per_min,
        threshold_cm_per_min=threshold_cm_per_min,
        notes=notes,
    )


def _detect_storage_id(bag_path: str) -> Optional[str]:
    try:
        entries = os.listdir(bag_path)
    except OSError:
        return None
    for entry in entries:
        lower = entry.lower()
        if lower.endswith(".db3"):
            return "sqlite3"
        if lower.endswith(".mcap"):
            return "mcap"
    # Fall back to sqlite3 (rosbag2 Jazzy default) if we cannot detect.
    return "sqlite3"


def format_report(report: DriftReport) -> str:
    lines = [
        "# analyze_static_drift.py report",
        "",
        f"- verdict: **{report.verdict}**",
        f"- reason: {report.reason}",
        f"- bag: `{report.bag_path}`",
        f"- topic: `{report.topic}`",
        f"- sample_count: {report.sample_count}",
        f"- duration_s: {report.duration_s:.3f}",
        f"- start_xy_m: ({report.start_x:.6f}, {report.start_y:.6f})",
        f"- end_xy_m:   ({report.end_x:.6f}, {report.end_y:.6f})",
        f"- displacement_cm: {report.displacement_cm:.3f}",
        f"- drift_cm_per_min: {report.drift_cm_per_min:.3f}",
        f"- threshold_cm_per_min: {report.threshold_cm_per_min:.3f}",
    ]
    if report.notes:
        lines.append("")
        lines.append("## notes")
        lines.append("")
        for note in report.notes:
            lines.append(f"- {note}")
    return "\n".join(lines) + "\n"


def _verdict_exit_code(verdict: str) -> int:
    if verdict == VERDICT_PASS:
        return 0
    if verdict == VERDICT_FAIL:
        return 1
    return 2


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    if args.bag is None:
        parser.error("--bag is required unless --help/--version is used")

    report = analyze_bag(
        bag_path=args.bag,
        odometry_topic=args.odometry_topic,
        threshold_cm_per_min=args.max_drift_cm_per_min,
        min_samples=args.min_samples,
    )

    text = format_report(report)
    sys.stdout.write(text)
    sys.stdout.flush()

    if args.output:
        try:
            os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
            with open(args.output, "w") as fh:
                fh.write(text)
        except OSError as exc:
            sys.stderr.write(f"WARNING: could not write --output: {exc}\n")

    if args.json_output:
        try:
            os.makedirs(os.path.dirname(args.json_output) or ".",
                        exist_ok=True)
            with open(args.json_output, "w") as fh:
                json.dump(report.as_dict(), fh, indent=2, sort_keys=True)
                fh.write("\n")
        except OSError as exc:
            sys.stderr.write(f"WARNING: could not write --json-output: {exc}\n")

    return _verdict_exit_code(report.verdict)


if __name__ == "__main__":
    sys.exit(main())
