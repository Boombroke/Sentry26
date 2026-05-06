#!/usr/bin/env python3
"""verify_dual_mid360_sync.py.

Verify whether two Livox Mid360 LiDARs share the same hardware time source.

Modes:
  * real mode (default): subscribe to two `livox_ros_driver2/msg/CustomMsg`
    topics, collect N paired samples, compute statistics, emit a verdict.
  * `--mock-data` mode: generate deterministic paired samples in pure Python
    without importing any ROS packages. Intended for CI and local sanity
    checks when the LiDAR is not available.

Verdict thresholds match `docs/SYNC_VERIFICATION.md` §5.

Exit codes:
  0  verdict matches --expected-result (or matches HARDWARE_SYNC if not given)
  1  verdict differs from --expected-result
  2  runtime error (missing samples, ROS import failure in real mode, etc.)

SAFETY INVARIANT: Module MUST stay import-safe without a ROS environment.
rclpy / livox_ros_driver2 imports are deferred into the real-mode entrypoint;
do NOT move them to module level, doing so breaks CI on hosts without ROS.
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

SCRIPT_VERSION = "0.1.0"

VERDICT_HARDWARE_SYNC = "HARDWARE_SYNC"
VERDICT_DRIFTING = "DRIFTING"
VERDICT_NOT_SYNCED = "NOT_SYNCED"
VERDICT_CHOICES = (
    VERDICT_HARDWARE_SYNC,
    VERDICT_DRIFTING,
    VERDICT_NOT_SYNCED,
)

DEFAULT_HARDWARE_MEDIAN_MS = 1.0
DEFAULT_HARDWARE_MAX_MS = 3.0
DEFAULT_HARDWARE_STDDEV_MS = 0.5
DEFAULT_HARDWARE_DRIFT_SLOPE_MS_PER_MIN = 0.5
DEFAULT_HARDWARE_DRIFT_RANGE_MS = 0.5
DEFAULT_NOT_SYNCED_MEDIAN_MS = 10.0


# =============================================================================
# Data classes
# =============================================================================

@dataclass
class PairedSample:
    """One pair of aligned front/back stamps (in nanoseconds).

    The monotonic time field is used for drift slope regression. It is the
    wall-clock time (or simulated wall-clock) when the sample was captured,
    expressed in seconds.
    """

    front_ns: int
    back_ns: int
    capture_time_s: float

    @property
    def diff_ns(self) -> int:
        return self.back_ns - self.front_ns

    @property
    def diff_ms(self) -> float:
        return self.diff_ns / 1e6


@dataclass
class Thresholds:
    hardware_median_ms: float = DEFAULT_HARDWARE_MEDIAN_MS
    hardware_max_ms: float = DEFAULT_HARDWARE_MAX_MS
    hardware_stddev_ms: float = DEFAULT_HARDWARE_STDDEV_MS
    hardware_drift_slope_ms_per_min: float = DEFAULT_HARDWARE_DRIFT_SLOPE_MS_PER_MIN
    hardware_drift_range_ms: float = DEFAULT_HARDWARE_DRIFT_RANGE_MS
    not_synced_median_ms: float = DEFAULT_NOT_SYNCED_MEDIAN_MS


@dataclass
class Statistics:
    sample_count: int = 0
    median_diff_ms: float = 0.0
    mean_diff_ms: float = 0.0
    max_abs_diff_ms: float = 0.0
    min_diff_ms: float = 0.0
    max_diff_ms: float = 0.0
    stddev_ms: float = 0.0
    drift_slope_ms_per_min: float = 0.0
    drift_range_ms: float = 0.0
    per_sample_ms: List[float] = field(default_factory=list)


@dataclass
class VerdictReport:
    verdict: str
    reasons: List[str]
    stats: Statistics
    thresholds: Thresholds
    sample_count: int
    tolerance_ms: float
    source_mode: str


# =============================================================================
# Core analysis (no ROS / no third-party dependency)
# =============================================================================

def _linear_regression_slope(xs: Sequence[float], ys: Sequence[float]) -> float:
    """Least-squares slope of y over x. Returns 0.0 when degenerate."""
    n = len(xs)
    if n < 2:
        return 0.0
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    numerator = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    denominator = sum((x - mean_x) ** 2 for x in xs)
    if denominator == 0.0:
        return 0.0
    return numerator / denominator


def compute_statistics(samples: Sequence[PairedSample]) -> Statistics:
    """Compute summary statistics from paired samples."""
    if not samples:
        return Statistics()

    diffs_ms = [s.diff_ms for s in samples]
    capture_s = [s.capture_time_s for s in samples]

    # Normalize capture time so regression is stable regardless of epoch.
    t0 = capture_s[0]
    rel_s = [t - t0 for t in capture_s]

    # Slope of diff_ms over rel seconds, convert to per-minute.
    slope_ms_per_sec = _linear_regression_slope(rel_s, diffs_ms)
    slope_ms_per_min = slope_ms_per_sec * 60.0

    if len(diffs_ms) >= 2:
        stddev = statistics.stdev(diffs_ms)
    else:
        stddev = 0.0

    return Statistics(
        sample_count=len(diffs_ms),
        median_diff_ms=statistics.median(diffs_ms),
        mean_diff_ms=statistics.fmean(diffs_ms),
        max_abs_diff_ms=max(abs(d) for d in diffs_ms),
        min_diff_ms=min(diffs_ms),
        max_diff_ms=max(diffs_ms),
        stddev_ms=stddev,
        drift_slope_ms_per_min=slope_ms_per_min,
        drift_range_ms=max(diffs_ms) - min(diffs_ms),
        per_sample_ms=list(diffs_ms),
    )


def classify(stats: Statistics, thresholds: Thresholds) -> Tuple[str, List[str]]:
    """Map statistics to a verdict. Returns (verdict, list of human reasons)."""
    reasons: List[str] = []

    if stats.sample_count == 0:
        return VERDICT_NOT_SYNCED, ["no samples collected"]

    abs_median = abs(stats.median_diff_ms)
    abs_slope = abs(stats.drift_slope_ms_per_min)

    # NOT_SYNCED takes precedence over DRIFTING (large absolute offset).
    if abs_median >= thresholds.not_synced_median_ms:
        reasons.append(
            f"median |diff|={abs_median:.3f}ms >= {thresholds.not_synced_median_ms}ms"
        )
        return VERDICT_NOT_SYNCED, reasons

    if stats.max_abs_diff_ms >= thresholds.not_synced_median_ms * 2.0:
        reasons.append(
            f"max |diff|={stats.max_abs_diff_ms:.3f}ms "
            f">= {thresholds.not_synced_median_ms * 2.0}ms"
        )
        return VERDICT_NOT_SYNCED, reasons

    # HARDWARE_SYNC requires ALL conditions satisfied.
    hw_ok = True
    if abs_median >= thresholds.hardware_median_ms:
        reasons.append(
            f"median |diff|={abs_median:.3f}ms >= "
            f"{thresholds.hardware_median_ms}ms"
        )
        hw_ok = False
    if stats.max_abs_diff_ms >= thresholds.hardware_max_ms:
        reasons.append(
            f"max |diff|={stats.max_abs_diff_ms:.3f}ms >= "
            f"{thresholds.hardware_max_ms}ms"
        )
        hw_ok = False
    if stats.stddev_ms >= thresholds.hardware_stddev_ms:
        reasons.append(
            f"stddev={stats.stddev_ms:.3f}ms >= "
            f"{thresholds.hardware_stddev_ms}ms"
        )
        hw_ok = False
    if abs_slope >= thresholds.hardware_drift_slope_ms_per_min:
        reasons.append(
            f"drift slope |{abs_slope:.3f}ms/min| >= "
            f"{thresholds.hardware_drift_slope_ms_per_min}ms/min"
        )
        hw_ok = False
    if stats.drift_range_ms >= thresholds.hardware_drift_range_ms:
        reasons.append(
            f"drift range={stats.drift_range_ms:.3f}ms >= "
            f"{thresholds.hardware_drift_range_ms}ms"
        )
        hw_ok = False

    if hw_ok:
        return VERDICT_HARDWARE_SYNC, ["all HARDWARE_SYNC thresholds satisfied"]

    return VERDICT_DRIFTING, reasons


# =============================================================================
# Mock data generator (deterministic)
# =============================================================================

def _mock_samples(expected: str, count: int) -> List[PairedSample]:
    """Generate deterministic mock samples matching the expected verdict.

    The sequences are crafted so that `compute_statistics` produces values
    clearly inside each verdict's region, with no randomness involved. Stable
    output is required for evidence files.
    """
    if count < 2:
        count = 2

    samples: List[PairedSample] = []

    base_front_ns = 1_700_000_000_000_000_000  # fixed epoch for determinism
    period_ns = 100_000_000  # 10 Hz
    capture_dt_s = 0.1

    if expected == VERDICT_HARDWARE_SYNC:
        # Diff oscillates around ~0.2ms, within HARDWARE_SYNC envelope.
        diff_ns_sequence = [
            100_000 if i % 2 == 0 else 300_000  # 0.1 / 0.3 ms
            for i in range(count)
        ]
    elif expected == VERDICT_DRIFTING:
        # Linear drift from 0.5ms to 0.5+slope*duration ms, ~3ms/min slope.
        # 60s duration at 10Hz -> slope ms/min ≈ slope_ms_per_sec * 60.
        # Aim for ~3ms/min: slope_ms_per_sec = 0.05 ms/s, delta = 0.05*0.1*i.
        diff_ns_sequence = [
            int(500_000 + 0.05e6 * i * 0.1)  # baseline 0.5ms + drift
            for i in range(count)
        ]
    elif expected == VERDICT_NOT_SYNCED:
        # Large baseline offset around 25ms with moderate jitter.
        diff_ns_sequence = [
            25_000_000 + (200_000 if i % 3 == 0 else -150_000)
            for i in range(count)
        ]
    else:  # pragma: no cover
        raise ValueError(f"unknown expected verdict: {expected}")

    for i, diff_ns in enumerate(diff_ns_sequence):
        front_ns = base_front_ns + i * period_ns
        back_ns = front_ns + diff_ns
        capture_s = i * capture_dt_s
        samples.append(PairedSample(front_ns=front_ns,
                                    back_ns=back_ns,
                                    capture_time_s=capture_s))

    return samples


# =============================================================================
# Real ROS subscriber (import guarded)
# =============================================================================

def _collect_ros_samples(
    front_topic: str,
    back_topic: str,
    sample_count: int,
    sync_tolerance_ms: float,
    timeout_s: float,
) -> List[PairedSample]:
    """Collect N paired samples by subscribing to two CustomMsg topics.

    Imports rclpy and livox messages lazily. On ImportError the caller must
    report a runtime error (exit code 2), not crash at module import time.
    """
    # Deferred imports: keep the file safe to import without a ROS environment.
    import rclpy  # noqa: WPS433 - intentional lazy import
    from rclpy.node import Node
    from message_filters import ApproximateTimeSynchronizer, Subscriber
    from livox_ros_driver2.msg import CustomMsg  # type: ignore

    collected: List[PairedSample] = []

    class _SyncProbe(Node):
        def __init__(self) -> None:
            super().__init__("verify_dual_mid360_sync_probe")
            self._start_wall_s: Optional[float] = None
            self._sub_front = Subscriber(self, CustomMsg, front_topic)
            self._sub_back = Subscriber(self, CustomMsg, back_topic)
            self._sync = ApproximateTimeSynchronizer(
                [self._sub_front, self._sub_back],
                queue_size=20,
                slop=sync_tolerance_ms / 1000.0,
            )
            self._sync.registerCallback(self._on_pair)

        def _on_pair(self, front_msg, back_msg) -> None:  # type: ignore[no-untyped-def]
            now_wall_s = self.get_clock().now().nanoseconds / 1e9
            if self._start_wall_s is None:
                self._start_wall_s = now_wall_s
            front_ns = (front_msg.header.stamp.sec * 1_000_000_000
                        + front_msg.header.stamp.nanosec)
            back_ns = (back_msg.header.stamp.sec * 1_000_000_000
                       + back_msg.header.stamp.nanosec)
            collected.append(PairedSample(
                front_ns=front_ns,
                back_ns=back_ns,
                capture_time_s=now_wall_s - self._start_wall_s,
            ))

    rclpy.init()
    probe = _SyncProbe()
    try:
        end_wall_s = probe.get_clock().now().nanoseconds / 1e9 + timeout_s
        while (len(collected) < sample_count
               and probe.get_clock().now().nanoseconds / 1e9 < end_wall_s):
            rclpy.spin_once(probe, timeout_sec=0.1)
    finally:
        probe.destroy_node()
        rclpy.shutdown()

    return collected[:sample_count]


# =============================================================================
# Reporting
# =============================================================================

def _format_report(report: VerdictReport) -> str:
    s = report.stats
    lines = [
        "==================================================",
        " verify_dual_mid360_sync.py report",
        "==================================================",
        f" source mode     : {report.source_mode}",
        f" sample count    : {s.sample_count} "
        f"(requested {report.sample_count})",
        f" tolerance (slop): {report.tolerance_ms:.3f} ms",
        "--------------------------------------------------",
        f" median diff     : {s.median_diff_ms:+.3f} ms",
        f" mean diff       : {s.mean_diff_ms:+.3f} ms",
        f" min diff        : {s.min_diff_ms:+.3f} ms",
        f" max diff        : {s.max_diff_ms:+.3f} ms",
        f" max |diff|      : {s.max_abs_diff_ms:.3f} ms",
        f" stddev          : {s.stddev_ms:.3f} ms",
        f" drift slope     : {s.drift_slope_ms_per_min:+.3f} ms/min",
        f" drift range     : {s.drift_range_ms:.3f} ms",
        "--------------------------------------------------",
        f" thresholds (HARDWARE_SYNC requires ALL):",
        f"   |median|      < {report.thresholds.hardware_median_ms} ms",
        f"   max |diff|    < {report.thresholds.hardware_max_ms} ms",
        f"   stddev        < {report.thresholds.hardware_stddev_ms} ms",
        f"   |slope|       < "
        f"{report.thresholds.hardware_drift_slope_ms_per_min} ms/min",
        f"   drift range   < "
        f"{report.thresholds.hardware_drift_range_ms} ms",
        f" NOT_SYNCED when |median| >= "
        f"{report.thresholds.not_synced_median_ms} ms",
        "--------------------------------------------------",
        f" verdict         : {report.verdict}",
    ]
    if report.reasons:
        lines.append(" reasons:")
        for reason in report.reasons:
            lines.append(f"   - {reason}")
    lines.append("==================================================")
    return "\n".join(lines)


# =============================================================================
# CLI
# =============================================================================

def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="verify_dual_mid360_sync.py",
        description=(
            "Verify whether two Livox Mid360 LiDARs share a hardware time "
            "source. See src/sentry_nav/sentry_dual_mid360/docs/"
            "SYNC_VERIFICATION.md for the end-to-end workflow."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument("--front-topic", default="/livox/lidar_front",
                        help="front Mid360 CustomMsg topic")
    parser.add_argument("--back-topic", default="/livox/lidar_back",
                        help="back Mid360 CustomMsg topic")
    parser.add_argument("--sample-count", type=int, default=100,
                        help="number of paired samples to collect")
    parser.add_argument("--sync-tolerance-ms", type=float, default=10.0,
                        help="ApproximateTimeSynchronizer slop (ms)")
    parser.add_argument("--timeout-s", type=float, default=60.0,
                        help="real mode max wait time for samples (s)")
    parser.add_argument("--mock-data", action="store_true",
                        help="skip ROS and use deterministic mock data "
                             "driven by --expected-result")
    parser.add_argument("--expected-result", choices=VERDICT_CHOICES,
                        default=None,
                        help="expected verdict; mandatory in --mock-data "
                             "mode; in real mode it only affects exit code")
    parser.add_argument("--version", action="version",
                        version=f"%(prog)s {SCRIPT_VERSION}")

    # Threshold overrides (optional, mostly for advanced tuning).
    parser.add_argument("--hardware-median-ms", type=float,
                        default=DEFAULT_HARDWARE_MEDIAN_MS)
    parser.add_argument("--hardware-max-ms", type=float,
                        default=DEFAULT_HARDWARE_MAX_MS)
    parser.add_argument("--hardware-stddev-ms", type=float,
                        default=DEFAULT_HARDWARE_STDDEV_MS)
    parser.add_argument("--hardware-drift-slope-ms-per-min", type=float,
                        default=DEFAULT_HARDWARE_DRIFT_SLOPE_MS_PER_MIN)
    parser.add_argument("--hardware-drift-range-ms", type=float,
                        default=DEFAULT_HARDWARE_DRIFT_RANGE_MS)
    parser.add_argument("--not-synced-median-ms", type=float,
                        default=DEFAULT_NOT_SYNCED_MEDIAN_MS)

    return parser


def _run(args: argparse.Namespace) -> int:
    thresholds = Thresholds(
        hardware_median_ms=args.hardware_median_ms,
        hardware_max_ms=args.hardware_max_ms,
        hardware_stddev_ms=args.hardware_stddev_ms,
        hardware_drift_slope_ms_per_min=args.hardware_drift_slope_ms_per_min,
        hardware_drift_range_ms=args.hardware_drift_range_ms,
        not_synced_median_ms=args.not_synced_median_ms,
    )

    if args.mock_data:
        if args.expected_result is None:
            sys.stderr.write(
                "ERROR: --expected-result is required with --mock-data\n"
            )
            return 2
        samples = _mock_samples(args.expected_result, args.sample_count)
        source_mode = "mock"
    else:
        try:
            samples = _collect_ros_samples(
                front_topic=args.front_topic,
                back_topic=args.back_topic,
                sample_count=args.sample_count,
                sync_tolerance_ms=args.sync_tolerance_ms,
                timeout_s=args.timeout_s,
            )
        except ImportError as exc:
            sys.stderr.write(
                "ERROR: real mode requires rclpy and livox_ros_driver2 "
                "Python bindings to be on PYTHONPATH. Either source a ROS "
                "workspace with sentry_dual_mid360 built, or run with "
                f"--mock-data. Underlying error: {exc}\n"
            )
            return 2
        except Exception as exc:  # noqa: BLE001
            sys.stderr.write(f"ERROR: real mode failed: {exc}\n")
            return 2
        source_mode = "ros"

    stats = compute_statistics(samples)
    verdict, reasons = classify(stats, thresholds)

    report = VerdictReport(
        verdict=verdict,
        reasons=reasons,
        stats=stats,
        thresholds=thresholds,
        sample_count=args.sample_count,
        tolerance_ms=args.sync_tolerance_ms,
        source_mode=source_mode,
    )

    print(_format_report(report))

    if stats.sample_count == 0:
        return 2

    expected = args.expected_result or VERDICT_HARDWARE_SYNC
    return 0 if verdict == expected else 1


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    return _run(args)


if __name__ == "__main__":
    sys.exit(main())
