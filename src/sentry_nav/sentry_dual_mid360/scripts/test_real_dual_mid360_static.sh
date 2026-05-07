#!/bin/bash
# test_real_dual_mid360_static.sh
# Dual Mid360 + Point-LIO static bench test orchestrator (Task 16).
#
# Purpose:
#   Verify that, with the real dual Mid360 hardware running but the chassis /
#   gimbal COMPLETELY STATIONARY, the end-to-end stack (livox_ros_driver2 ->
#   pointcloud_merger -> Point-LIO -> odom_bridge -> /odometry) stays stable:
#     * merger publishes /livox/lidar without `lidar loop back` errors
#     * Point-LIO does not diverge
#     * /odometry planar drift stays below a small threshold (default 5cm/min)
#
# This script NEVER sends a Nav Goal and NEVER publishes motion commands. It
# is a read-only observer: it launches the navigation stack, records a bag
# of /odometry, tears the stack down, then hands the bag to
# analyze_static_drift.py for PASS/FAIL/BLOCKED classification.
#
# Modes (mutually exclusive; at most one may be supplied):
#   (default)        Real-hardware path. Pings both Mid360 IPs, invokes T7
#                    sync verification, launches rm_navigation_reality_launch
#                    with SLAM True + dual enabled, records /odometry, runs
#                    the analyzer. Emits BLOCKED (exit 2) if the hardware or
#                    ROS stack is not reachable -- never a fake PASS.
#   --preflight      Run preflight checks only. Do not launch anything.
#   --skip-hardware  Bag-only mode: no ping, no launch, no recording; just
#                    analyze a user-supplied bag at --bag PATH. Useful for
#                    off-robot replay of an earlier run.
#   --fake-bag       Analyze /tmp/static_fake_bag (or --fake-bag-path). If
#                    that directory does not exist, emits BLOCKED with the
#                    exact instructions to produce one -- no fake data is
#                    ever fabricated.
#
# Exit codes:
#   0  PASS     analyzer verdict was PASS
#   1  FAIL     analyzer verdict was FAIL (stack ran, drift exceeded threshold)
#   2  BLOCKED  prerequisites missing (no hardware, no bag, etc.)
#   3  usage / argument error
#
# Evidence files (under <output-dir>/, default .sisyphus/evidence/):
#   task-16-help.txt             captured by the user when running --help
#   task-16-preflight.md         preflight matrix (ros2, pings, scripts, deps)
#   task-16-blocker.md           only emitted when the overall verdict = BLOCKED
#   task-16-summary.md           run summary (mode, phases, exit code)
#   task-16-sync.log             T7 verify_dual_mid360_sync.py output
#   task-16-analyze.md           analyzer Markdown report (copy of stdout)
#   task-16-analyze.json         analyzer JSON report (machine-readable)
#   task-16-pointlio.log         excerpt of point_lio log OR BLOCKED marker
#   task-16-merger.log           excerpt of pointcloud_merger log OR BLOCKED
#   task-16-merger-diag.txt      merger diagnostics topic echo OR BLOCKED
#   task-16-launch.log           raw stdout/stderr of the navigation launch
#   task-16-rosbag.log           raw stdout/stderr of the ros2 bag recorder
#
# Reference: .sisyphus/plans/dual-mid360-fusion.md Task 16.

set -euo pipefail

# =============================================================================
# Constants
# =============================================================================

SCRIPT_VERSION="0.1.0"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# scripts/ -> pkg/ -> sentry_nav/ -> src/ -> ws/
WS_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

NAV_LAUNCH_PKG="sentry_nav_bringup"
NAV_LAUNCH_FILE="rm_navigation_reality_launch.py"

REQUIRED_PACKAGES=(
    "sentry_nav_bringup"
    "sentry_dual_mid360"
    "livox_ros_driver2"
    "point_lio"
)

DEFAULT_FRONT_IP="192.168.1.144"
DEFAULT_BACK_IP="192.168.1.145"
DEFAULT_FAKE_BAG_PATH="/tmp/static_fake_bag"

SYNC_VERIFIER_NAME="verify_dual_mid360_sync.py"
DRIFT_ANALYZER_NAME="analyze_static_drift.py"

# =============================================================================
# Defaults (overridable via flags)
# =============================================================================

MODE_REAL=true
MODE_PREFLIGHT=false
MODE_SKIP_HARDWARE=false
MODE_FAKE_BAG=false

FRONT_IP="${DEFAULT_FRONT_IP}"
BACK_IP="${DEFAULT_BACK_IP}"
DURATION=60
BAG_PATH=""
FAKE_BAG_PATH="${DEFAULT_FAKE_BAG_PATH}"
OUTPUT_DIR=""
NAMESPACE=""
WORLD=""
SLAM="True"
ODOMETRY_TOPIC="/odometry"
MAX_DRIFT_CM_PER_MIN="5.0"
SYNC_SAMPLE_COUNT=100
SYNC_TIMEOUT_S=60
SYNC_TOLERANCE_MS=10
MERGER_DIAG_TOPIC="/pointcloud_merger/diagnostics"
NAV_WARMUP=30
DRY_RUN=false
VERBOSE=false

# =============================================================================
# Logging helpers
# =============================================================================

log_info()  { echo "[INFO]  $*"; }
log_warn()  { echo "[WARN]  $*" >&2; }
log_error() { echo "[ERROR] $*" >&2; }
log_ok()    { echo "[OK]    $*"; }
log_fail()  { echo "[FAIL]  $*" >&2; }

print_usage() {
    cat <<EOF
test_real_dual_mid360_static.sh v${SCRIPT_VERSION} -- dual Mid360 static bench test

USAGE:
    bash test_real_dual_mid360_static.sh [OPTIONS]

MODES (mutually exclusive; at most one):
    (default)           Real hardware path: ping both Mid360 IPs, run sync
                        verification, launch reality nav stack, record
                        /odometry for --duration seconds (bounded by GNU
                        timeout --preserve-status, so the recorder always
                        terminates cleanly), analyze drift. If the T7 sync
                        verifier returns non-zero, the run is aborted as
                        BLOCKED before any bag is recorded -- sync failures
                        are never swallowed as success.
    --preflight         Preflight only. Do not launch anything or record bags.
                        Cannot be combined with --bag; use --skip-hardware
                        --bag PATH instead for bag-only analysis.
    --skip-hardware     Bag-only mode. Requires --bag PATH. No ping, no launch.
    --fake-bag          Analyze \$(--fake-bag-path | default ${DEFAULT_FAKE_BAG_PATH}).
                        If the path does not exist, emits BLOCKED -- this script
                        never fabricates bag data.

OPTIONS:
    --help, -h                      Show this help and exit 0.
    --version                       Print script version and exit 0.
    --duration SECS                 Bag record / stationary dwell seconds.
                                    Default: ${DURATION}.
    --bag PATH                      Existing rosbag2 directory to analyze
                                    (enables --skip-hardware semantics).
    --fake-bag-path PATH            Override path checked by --fake-bag.
                                    Default: ${DEFAULT_FAKE_BAG_PATH}.
    --front-ip IP                   Front Mid360 IP. Default: ${DEFAULT_FRONT_IP}.
    --back-ip IP                    Back  Mid360 IP. Default: ${DEFAULT_BACK_IP}.
    --namespace NS                  Navigation namespace override.
                                    Default: "" (unnamespaced).
    --world WORLD                   Override rm_navigation_reality_launch world.
    --slam True|False               Navigation slam arg. Default: ${SLAM}.
    --odometry-topic TOPIC          Odometry topic to record + analyze.
                                    Default: ${ODOMETRY_TOPIC}.
    --max-drift-cm-per-min N        PASS/FAIL threshold. Default: ${MAX_DRIFT_CM_PER_MIN}.
    --sync-sample-count N           T7 sync verifier sample count. Default: ${SYNC_SAMPLE_COUNT}.
    --sync-timeout-s SECS           T7 sync verifier wait timeout. Default: ${SYNC_TIMEOUT_S}.
    --sync-tolerance-ms MS          T7 sync verifier slop. Default: ${SYNC_TOLERANCE_MS}.
    --merger-diag-topic TOPIC       Optional diagnostics topic to echo while
                                    the stack is running. If empty, merger
                                    diagnostics evidence is reported BLOCKED.
    --nav-warmup SECS               Seconds to wait after launch before bag
                                    recording starts. Default: ${NAV_WARMUP}.
    --output-dir DIR                Evidence output directory.
                                    Default: <ws>/.sisyphus/evidence.
    --dry-run                       Print the planned sequence; do not launch.
    --verbose                       Extra logging.

SAFETY INVARIANTS (this script NEVER violates):
    * No Nav Goal is ever sent.
    * No /cmd_vel, /cmd_vel_nav, /cmd_vel_controller is ever published.
    * No chassis / gimbal motion is commanded.
    * No source file (launch, yaml, cpp) outside the evidence dir is modified.
    * If real hardware is unreachable, verdict is BLOCKED, NEVER a fake PASS.

EXIT CODES:
    0  PASS
    1  FAIL
    2  BLOCKED
    3  usage / argument error

REFERENCES:
    .sisyphus/plans/dual-mid360-fusion.md Task 16
    src/sentry_nav/sentry_dual_mid360/scripts/verify_dual_mid360_sync.py (T7)
    src/sentry_nav/sentry_dual_mid360/scripts/test_sim_dual_mid360.sh (T15)
    src/docs/QUICKSTART.md
EOF
}

# =============================================================================
# Argument parsing
# =============================================================================

parse_args() {
    local mode_count=0
    while [ $# -gt 0 ]; do
        case "$1" in
            --help|-h)
                print_usage
                exit 0
                ;;
            --version)
                echo "test_real_dual_mid360_static.sh v${SCRIPT_VERSION}"
                exit 0
                ;;
            --preflight)
                MODE_PREFLIGHT=true
                MODE_REAL=false
                mode_count=$((mode_count + 1))
                shift
                ;;
            --skip-hardware)
                MODE_SKIP_HARDWARE=true
                MODE_REAL=false
                mode_count=$((mode_count + 1))
                shift
                ;;
            --fake-bag)
                MODE_FAKE_BAG=true
                MODE_REAL=false
                mode_count=$((mode_count + 1))
                shift
                ;;
            --duration)
                [ $# -ge 2 ] || { log_error "--duration requires an argument"; exit 3; }
                DURATION="$2"
                shift 2
                ;;
            --bag)
                [ $# -ge 2 ] || { log_error "--bag requires an argument"; exit 3; }
                BAG_PATH="$2"
                shift 2
                ;;
            --fake-bag-path)
                [ $# -ge 2 ] || { log_error "--fake-bag-path requires an argument"; exit 3; }
                FAKE_BAG_PATH="$2"
                shift 2
                ;;
            --front-ip)
                [ $# -ge 2 ] || { log_error "--front-ip requires an argument"; exit 3; }
                FRONT_IP="$2"
                shift 2
                ;;
            --back-ip)
                [ $# -ge 2 ] || { log_error "--back-ip requires an argument"; exit 3; }
                BACK_IP="$2"
                shift 2
                ;;
            --namespace)
                [ $# -ge 2 ] || { log_error "--namespace requires an argument"; exit 3; }
                NAMESPACE="$2"
                shift 2
                ;;
            --world)
                [ $# -ge 2 ] || { log_error "--world requires an argument"; exit 3; }
                WORLD="$2"
                shift 2
                ;;
            --slam)
                [ $# -ge 2 ] || { log_error "--slam requires an argument"; exit 3; }
                SLAM="$2"
                shift 2
                ;;
            --odometry-topic)
                [ $# -ge 2 ] || { log_error "--odometry-topic requires an argument"; exit 3; }
                ODOMETRY_TOPIC="$2"
                shift 2
                ;;
            --max-drift-cm-per-min)
                [ $# -ge 2 ] || { log_error "--max-drift-cm-per-min requires an argument"; exit 3; }
                MAX_DRIFT_CM_PER_MIN="$2"
                shift 2
                ;;
            --sync-sample-count)
                [ $# -ge 2 ] || { log_error "--sync-sample-count requires an argument"; exit 3; }
                SYNC_SAMPLE_COUNT="$2"
                shift 2
                ;;
            --sync-timeout-s)
                [ $# -ge 2 ] || { log_error "--sync-timeout-s requires an argument"; exit 3; }
                SYNC_TIMEOUT_S="$2"
                shift 2
                ;;
            --sync-tolerance-ms)
                [ $# -ge 2 ] || { log_error "--sync-tolerance-ms requires an argument"; exit 3; }
                SYNC_TOLERANCE_MS="$2"
                shift 2
                ;;
            --merger-diag-topic)
                [ $# -ge 2 ] || { log_error "--merger-diag-topic requires an argument"; exit 3; }
                MERGER_DIAG_TOPIC="$2"
                shift 2
                ;;
            --nav-warmup)
                [ $# -ge 2 ] || { log_error "--nav-warmup requires an argument"; exit 3; }
                NAV_WARMUP="$2"
                shift 2
                ;;
            --output-dir)
                [ $# -ge 2 ] || { log_error "--output-dir requires an argument"; exit 3; }
                OUTPUT_DIR="$2"
                shift 2
                ;;
            --dry-run)
                DRY_RUN=true
                shift
                ;;
            --verbose)
                VERBOSE=true
                shift
                ;;
            *)
                log_error "Unknown option: $1"
                echo ""
                print_usage
                exit 3
                ;;
        esac
    done

    if [ "$mode_count" -gt 1 ]; then
        log_error "at most one of --preflight / --skip-hardware / --fake-bag may be supplied"
        exit 3
    fi

    if [ -n "$BAG_PATH" ] && $MODE_PREFLIGHT; then
        log_error "--preflight is preflight-only; it cannot be combined with --bag PATH"
        log_error "use --skip-hardware --bag PATH for bag analysis only"
        exit 3
    fi

    if [ -n "$BAG_PATH" ] && $MODE_FAKE_BAG; then
        log_error "--fake-bag and --bag PATH cannot be combined (pick one bag source)"
        exit 3
    fi

    if [ -n "$BAG_PATH" ] && ! $MODE_SKIP_HARDWARE; then
        MODE_SKIP_HARDWARE=true
        MODE_REAL=false
    fi
}

validate_args() {
    case "$SLAM" in
        True|False|true|false) : ;;
        *)
            log_error "--slam must be True or False (got: '${SLAM}')"
            exit 3
            ;;
    esac
    for var_name in DURATION SYNC_SAMPLE_COUNT SYNC_TIMEOUT_S SYNC_TOLERANCE_MS NAV_WARMUP; do
        local v="${!var_name}"
        if ! [[ "$v" =~ ^[0-9]+$ ]] || [ "$v" -lt 0 ]; then
            log_error "--${var_name,,} must be a non-negative integer (got: '${v}')"
            exit 3
        fi
    done
    if ! [[ "$MAX_DRIFT_CM_PER_MIN" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
        log_error "--max-drift-cm-per-min must be a non-negative number (got: '${MAX_DRIFT_CM_PER_MIN}')"
        exit 3
    fi
    if [ -z "$OUTPUT_DIR" ]; then
        OUTPUT_DIR="${WS_ROOT}/.sisyphus/evidence"
    fi
}

# =============================================================================
# Environment helpers
# =============================================================================

find_ros2_bin() {
    if command -v ros2 &>/dev/null; then
        command -v ros2
        return 0
    fi
    for candidate in /opt/ros/jazzy/bin/ros2 /opt/ros/humble/bin/ros2; do
        if [ -x "$candidate" ]; then
            echo "$candidate"
            return 0
        fi
    done
    return 1
}

source_ws_setup() {
    local candidates=(
        "${WS_ROOT}/install/setup.bash"
        "/home/pc/Documents/Sentry26/install/setup.bash"
    )
    for c in "${candidates[@]}"; do
        if [ -f "$c" ]; then
            set +u
            # shellcheck disable=SC1090
            source "$c"
            set -u
            return 0
        fi
    done
    return 1
}

# =============================================================================
# Preflight
# =============================================================================

PRE_ROS2_BIN=""
PRE_MISSING_PKGS=()
PRE_PRESENT_PKGS=()
PRE_PACKAGES_CHECKED=false
PRE_SYNC_SCRIPT_PATH=""
PRE_ANALYZER_PATH=""
PRE_FRONT_REACHABLE=false
PRE_BACK_REACHABLE=false
PRE_ROSBAG2_PY_AVAILABLE=false
PRE_BLOCKED_REASONS=()

check_ping() {
    local ip="$1"
    if ping -c 1 -W 2 "$ip" &>/dev/null; then
        return 0
    fi
    return 1
}

check_rosbag2_py() {
    if python3 -c "import rosbag2_py" &>/dev/null; then
        PRE_ROSBAG2_PY_AVAILABLE=true
        return 0
    fi
    PRE_ROSBAG2_PY_AVAILABLE=false
    return 1
}

run_preflight() {
    log_info "=== Preflight checks ==="

    if PRE_ROS2_BIN="$(find_ros2_bin)"; then
        log_ok "ros2 binary: ${PRE_ROS2_BIN}"
    else
        PRE_ROS2_BIN=""
        log_fail "ros2 binary not found on PATH or /opt/ros/*/bin/"
        PRE_BLOCKED_REASONS+=("ros2 binary missing (source /opt/ros/jazzy/setup.bash)")
    fi

    if source_ws_setup; then
        log_ok "workspace setup.bash sourced"
    else
        log_warn "no install/setup.bash found; package checks may be unreliable"
    fi

    if [ -n "$PRE_ROS2_BIN" ]; then
        PRE_PACKAGES_CHECKED=true
        for pkg in "${REQUIRED_PACKAGES[@]}"; do
            if "$PRE_ROS2_BIN" pkg prefix "$pkg" &>/dev/null; then
                log_ok "package present: ${pkg}"
                PRE_PRESENT_PKGS+=("$pkg")
            else
                log_fail "package MISSING: ${pkg}"
                PRE_MISSING_PKGS+=("$pkg")
            fi
        done
        if [ ${#PRE_MISSING_PKGS[@]} -gt 0 ] && $MODE_REAL; then
            PRE_BLOCKED_REASONS+=("missing ROS2 packages: ${PRE_MISSING_PKGS[*]}")
        fi
    else
        PRE_PACKAGES_CHECKED=false
        log_warn "package presence NOT CHECKED (ros2 binary missing)"
        if $MODE_REAL; then
            PRE_BLOCKED_REASONS+=("package presence not checked because ros2 binary is missing")
        fi
    fi

    PRE_SYNC_SCRIPT_PATH="${SCRIPT_DIR}/${SYNC_VERIFIER_NAME}"
    if [ -f "$PRE_SYNC_SCRIPT_PATH" ]; then
        log_ok "sync verifier found: ${PRE_SYNC_SCRIPT_PATH}"
    else
        log_fail "sync verifier missing: ${PRE_SYNC_SCRIPT_PATH}"
        if $MODE_REAL; then
            PRE_BLOCKED_REASONS+=("sync verifier missing: ${SYNC_VERIFIER_NAME}")
        fi
    fi

    PRE_ANALYZER_PATH="${SCRIPT_DIR}/${DRIFT_ANALYZER_NAME}"
    if [ -f "$PRE_ANALYZER_PATH" ]; then
        log_ok "drift analyzer found: ${PRE_ANALYZER_PATH}"
    else
        log_fail "drift analyzer missing: ${PRE_ANALYZER_PATH}"
        PRE_BLOCKED_REASONS+=("drift analyzer missing: ${DRIFT_ANALYZER_NAME}")
    fi

    if check_rosbag2_py; then
        log_ok "python3 rosbag2_py importable"
    else
        log_warn "python3 rosbag2_py NOT importable (analyzer will return BLOCKED)"
    fi

    if $MODE_REAL; then
        log_info "pinging front Mid360 @ ${FRONT_IP} (2s timeout)"
        if check_ping "$FRONT_IP"; then
            PRE_FRONT_REACHABLE=true
            log_ok "front Mid360 reachable: ${FRONT_IP}"
        else
            PRE_FRONT_REACHABLE=false
            log_fail "front Mid360 NOT reachable: ${FRONT_IP}"
            PRE_BLOCKED_REASONS+=("front Mid360 (${FRONT_IP}) not reachable via ICMP")
        fi

        log_info "pinging back Mid360 @ ${BACK_IP} (2s timeout)"
        if check_ping "$BACK_IP"; then
            PRE_BACK_REACHABLE=true
            log_ok "back Mid360 reachable: ${BACK_IP}"
        else
            PRE_BACK_REACHABLE=false
            log_fail "back Mid360 NOT reachable: ${BACK_IP}"
            PRE_BLOCKED_REASONS+=("back Mid360 (${BACK_IP}) not reachable via ICMP")
        fi
    else
        log_info "skipping hardware ping (non-real mode)"
    fi

    if $MODE_SKIP_HARDWARE; then
        if [ -z "$BAG_PATH" ]; then
            PRE_BLOCKED_REASONS+=("--skip-hardware / --bag mode requires --bag PATH")
        elif [ ! -d "$BAG_PATH" ]; then
            PRE_BLOCKED_REASONS+=("--bag path does not exist: ${BAG_PATH}")
        fi
    fi

    if $MODE_FAKE_BAG; then
        if [ ! -d "$FAKE_BAG_PATH" ]; then
            PRE_BLOCKED_REASONS+=(
                "--fake-bag path does not exist: ${FAKE_BAG_PATH} "
                "(produce one with: timeout --preserve-status ${DURATION}s "
                "ros2 bag record -o ${FAKE_BAG_PATH} ${ODOMETRY_TOPIC} "
                "while the robot is stationary)"
            )
        fi
    fi
}

preflight_is_blocked() {
    [ ${#PRE_BLOCKED_REASONS[@]} -gt 0 ]
}

# =============================================================================
# Evidence writers
# =============================================================================

write_preflight_evidence() {
    local out="${OUTPUT_DIR}/task-16-preflight.md"
    mkdir -p "$OUTPUT_DIR"
    {
        echo "# Task 16 Preflight -- test_real_dual_mid360_static.sh"
        echo ""
        echo "Generated: $(date -u +"%Y-%m-%dT%H:%M:%SZ")"
        echo "Workspace: ${WS_ROOT}"
        echo ""
        echo "## Mode"
        echo ""
        echo "| flag | value |"
        echo "|---|---|"
        echo "| MODE_REAL | ${MODE_REAL} |"
        echo "| MODE_PREFLIGHT | ${MODE_PREFLIGHT} |"
        echo "| MODE_SKIP_HARDWARE | ${MODE_SKIP_HARDWARE} |"
        echo "| MODE_FAKE_BAG | ${MODE_FAKE_BAG} |"
        echo "| BAG_PATH | ${BAG_PATH:-(none)} |"
        echo "| FAKE_BAG_PATH | ${FAKE_BAG_PATH} |"
        echo "| front_ip | ${FRONT_IP} |"
        echo "| back_ip | ${BACK_IP} |"
        echo "| duration_s | ${DURATION} |"
        echo "| odometry_topic | ${ODOMETRY_TOPIC} |"
        echo "| max_drift_cm_per_min | ${MAX_DRIFT_CM_PER_MIN} |"
        echo ""
        echo "## Environment"
        echo ""
        echo "| check | result |"
        echo "|---|---|"
        echo "| ros2 binary | ${PRE_ROS2_BIN:-MISSING} |"
        echo "| sync verifier (${SYNC_VERIFIER_NAME}) | ${PRE_SYNC_SCRIPT_PATH:-MISSING} |"
        echo "| drift analyzer (${DRIFT_ANALYZER_NAME}) | ${PRE_ANALYZER_PATH:-MISSING} |"
        echo "| python3 rosbag2_py | $($PRE_ROSBAG2_PY_AVAILABLE && echo YES || echo NO) |"
        echo ""
        echo "## ROS2 packages"
        echo ""
        echo "| package | status |"
        echo "|---|---|"
        for pkg in "${REQUIRED_PACKAGES[@]}"; do
            if ! $PRE_PACKAGES_CHECKED; then
                echo "| \`${pkg}\` | NOT_CHECKED |"
            elif printf '%s\n' "${PRE_MISSING_PKGS[@]:-}" | grep -qx "$pkg"; then
                echo "| \`${pkg}\` | MISSING |"
            else
                echo "| \`${pkg}\` | PRESENT |"
            fi
        done
        if ! $PRE_PACKAGES_CHECKED; then
            echo ""
            echo "> NOTE: package presence was NOT_CHECKED because \`ros2\` was not"
            echo "> found. Source a ROS2 Jazzy workspace to enable package checks."
        fi
        echo ""
        echo "## Hardware reachability"
        echo ""
        echo "| sensor | ip | reachable |"
        echo "|---|---|---|"
        echo "| front Mid360 | ${FRONT_IP} | $($PRE_FRONT_REACHABLE && echo YES || echo NO) |"
        echo "| back  Mid360 | ${BACK_IP} | $($PRE_BACK_REACHABLE && echo YES || echo NO) |"
        echo ""
        echo "## Verdict"
        echo ""
        if preflight_is_blocked; then
            echo "**BLOCKED** -- the following conditions prevent a valid runtime:"
            echo ""
            for r in "${PRE_BLOCKED_REASONS[@]}"; do
                echo "- ${r}"
            done
        else
            echo "**PASS** -- preflight satisfied for the selected mode."
        fi
        echo ""
        echo "## Intended real-hardware command sequence (reference)"
        echo ""
        echo '```bash'
        echo "# 1. Both Mid360 powered (front ${FRONT_IP}, back ${BACK_IP})."
        echo "# 2. Robot stationary on flat ground. No chassis/gimbal motion."
        echo "# 3. Source ROS2 workspace."
        echo "source ${WS_ROOT}/install/setup.bash"
        echo "# 4. Launch the navigation stack with dual enabled:"
        echo "ros2 launch ${NAV_LAUNCH_PKG} ${NAV_LAUNCH_FILE} \\"
        echo "  slam:=${SLAM} \\"
        echo "  use_dual_mid360:=True \\"
        echo "  use_robot_state_pub:=True${NAMESPACE:+ \\}${NAMESPACE:+ namespace:=${NAMESPACE}}${WORLD:+ \\}${WORLD:+ world:=${WORLD}}"
        echo "# 5. Verify sync then record /odometry:"
        echo "python3 ${SCRIPT_DIR}/${SYNC_VERIFIER_NAME} \\"
        echo "  --front-topic /livox/lidar_front --back-topic /livox/lidar_back \\"
        echo "  --sample-count ${SYNC_SAMPLE_COUNT} --timeout-s ${SYNC_TIMEOUT_S} \\"
        echo "  --sync-tolerance-ms ${SYNC_TOLERANCE_MS}"
        echo "ros2 bag record -o /tmp/task16_bag --duration ${DURATION} ${ODOMETRY_TOPIC}"
        echo "# (Internal runtime uses: timeout --signal=INT --preserve-status ${DURATION}s ros2 bag record -o <dir> ${ODOMETRY_TOPIC})"
        echo "# 6. Analyze:"
        echo "python3 ${SCRIPT_DIR}/${DRIFT_ANALYZER_NAME} \\"
        echo "  --bag /tmp/task16_bag --odometry-topic ${ODOMETRY_TOPIC} \\"
        echo "  --max-drift-cm-per-min ${MAX_DRIFT_CM_PER_MIN}"
        echo '```'
    } >"$out"
    log_info "preflight evidence written: ${out}"
}

write_blocker_evidence() {
    local out="${OUTPUT_DIR}/task-16-blocker.md"
    mkdir -p "$OUTPUT_DIR"
    {
        echo "# Task 16 BLOCKED -- static bench test did not produce a real verdict"
        echo ""
        echo "Generated: $(date -u +"%Y-%m-%dT%H:%M:%SZ")"
        echo "Workspace: ${WS_ROOT}"
        echo ""
        echo "## Blockers"
        echo ""
        for r in "${PRE_BLOCKED_REASONS[@]}"; do
            echo "- ${r}"
        done
        echo ""
        echo "## Why BLOCKED and not FAIL"
        echo ""
        echo "FAIL means the stack ran end-to-end but the measured drift"
        echo "exceeded the threshold. BLOCKED means the measurement could not"
        echo "be taken at all -- e.g. the Mid360 hardware is not reachable,"
        echo "\`rosbag2_py\` is not importable, or the user-supplied bag"
        echo "directory does not exist. Turning BLOCKED into a fake PASS"
        echo "would violate Task 16 honesty rules."
        echo ""
        echo "## Resolution paths"
        echo ""
        echo "- **Missing ROS2 packages / ros2 binary**: source the workspace"
        echo "  (\`source ${WS_ROOT}/install/setup.bash\`) or run"
        echo "  \`bash src/scripts/setup_env.sh\`."
        echo "- **Unreachable Mid360 IPs**: confirm PoE/power, network"
        echo "  configuration (\`192.168.1.0/24\`), and that front ="
        echo "  ${FRONT_IP}, back = ${BACK_IP} in"
        echo "  \`sentry_dual_mid360/config/mid360_user_config_dual.json\`."
        echo "- **Missing bag for --skip-hardware / --fake-bag**: produce one"
        echo "  while the robot is stationary:"
        echo ""
        echo '    ```bash'
        echo "    ros2 bag record -o ${FAKE_BAG_PATH} \\"
        echo "        ${ODOMETRY_TOPIC} &"
        echo "    sleep ${DURATION} && pkill -INT -f 'ros2 bag record'"
        echo '    ```'
        echo ""
        echo "  then re-run this script with \`--fake-bag\` or"
        echo "  \`--skip-hardware --bag ${FAKE_BAG_PATH}\`."
        echo "- **rosbag2_py ImportError**: source the ROS2 Jazzy workspace"
        echo "  before invoking the analyzer."
        echo ""
        echo "## Evidence reference"
        echo ""
        echo "See \`task-16-preflight.md\` for the exact environment snapshot"
        echo "that led to this BLOCKED verdict."
    } >"$out"
    log_info "blocker evidence written: ${out}"
}

write_summary() {
    local phase="$1"
    local verdict="$2"
    local bag_used="$3"
    local out="${OUTPUT_DIR}/task-16-summary.md"
    mkdir -p "$OUTPUT_DIR"
    {
        echo "# Task 16 Runtime Summary"
        echo ""
        echo "Generated: $(date -u +"%Y-%m-%dT%H:%M:%SZ")"
        echo "Workspace: ${WS_ROOT}"
        echo ""
        echo "| key | value |"
        echo "|---|---|"
        echo "| mode_real | ${MODE_REAL} |"
        echo "| mode_preflight | ${MODE_PREFLIGHT} |"
        echo "| mode_skip_hardware | ${MODE_SKIP_HARDWARE} |"
        echo "| mode_fake_bag | ${MODE_FAKE_BAG} |"
        echo "| duration_s | ${DURATION} |"
        echo "| front_ip | ${FRONT_IP} |"
        echo "| back_ip | ${BACK_IP} |"
        echo "| bag_path_used | ${bag_used:-(none)} |"
        echo "| odometry_topic | ${ODOMETRY_TOPIC} |"
        echo "| max_drift_cm_per_min | ${MAX_DRIFT_CM_PER_MIN} |"
        echo "| phase_reached | ${phase} |"
        echo "| final_verdict | ${verdict} |"
        echo ""
        echo "## Evidence files"
        echo ""
        for ev in preflight sync analyze analyze.json launch rosbag pointlio pointlio-errors merger merger-diag blocker; do
            local path="${OUTPUT_DIR}/task-16-${ev}"
            for ext in .md .txt .log .json; do
                if [ -f "${path}${ext}" ]; then
                    echo "- \`task-16-${ev}${ext}\`"
                fi
            done
        done
    } >"$out"
    log_info "summary written: ${out}"
}

write_blocked_marker() {
    local out_file="$1"
    local reason="$2"
    mkdir -p "$(dirname "$out_file")"
    {
        echo "# BLOCKED"
        echo ""
        echo "Generated: $(date -u +"%Y-%m-%dT%H:%M:%SZ")"
        echo ""
        echo "reason: ${reason}"
    } >"$out_file"
}

write_pointlio_errors_blocked() {
    write_blocked_marker "${OUTPUT_DIR}/task-16-pointlio-errors.txt" "$1"
}

# =============================================================================
# Dry run
# =============================================================================

do_dry_run() {
    local out="${OUTPUT_DIR}/task-16-dry-run.md"
    mkdir -p "$OUTPUT_DIR"
    {
        echo "# Task 16 Dry Run"
        echo ""
        echo "Generated: $(date -u +"%Y-%m-%dT%H:%M:%SZ")"
        echo ""
        echo "## Configuration"
        echo ""
        echo "| key | value |"
        echo "|---|---|"
        echo "| MODE_REAL | ${MODE_REAL} |"
        echo "| MODE_PREFLIGHT | ${MODE_PREFLIGHT} |"
        echo "| MODE_SKIP_HARDWARE | ${MODE_SKIP_HARDWARE} |"
        echo "| MODE_FAKE_BAG | ${MODE_FAKE_BAG} |"
        echo "| DURATION | ${DURATION}s |"
        echo "| BAG_PATH | ${BAG_PATH:-(none)} |"
        echo "| FAKE_BAG_PATH | ${FAKE_BAG_PATH} |"
        echo "| FRONT_IP / BACK_IP | ${FRONT_IP} / ${BACK_IP} |"
        echo "| odometry_topic | ${ODOMETRY_TOPIC} |"
        echo "| max_drift_cm_per_min | ${MAX_DRIFT_CM_PER_MIN} |"
        echo ""
        echo "## Planned phases"
        echo ""
        if $MODE_PREFLIGHT; then
            echo "1. Run preflight only; emit task-16-preflight.md; exit."
        elif $MODE_SKIP_HARDWARE; then
            echo "1. Verify BAG_PATH exists (${BAG_PATH:-(unset)})."
            echo "2. Run ${DRIFT_ANALYZER_NAME} against BAG_PATH."
            echo "3. Mark pointlio/merger evidence BLOCKED (no live runtime)."
        elif $MODE_FAKE_BAG; then
            echo "1. Verify FAKE_BAG_PATH exists (${FAKE_BAG_PATH})."
            echo "2. Run ${DRIFT_ANALYZER_NAME} against FAKE_BAG_PATH."
            echo "3. Mark pointlio/merger evidence BLOCKED (no live runtime)."
        else
            echo "1. Ping both Mid360 IPs; BLOCKED if either unreachable."
            echo "2. Source workspace; launch rm_navigation_reality_launch.py."
            echo "3. Wait \${NAV_WARMUP}s, then run sync verifier."
            echo "4. Record /odometry bag for \${DURATION}s."
            echo "5. Tear down stack."
            echo "6. Run ${DRIFT_ANALYZER_NAME} against recorded bag."
            echo "7. Emit PASS/FAIL/BLOCKED."
        fi
        echo ""
        echo "## Planned commands (for reference; not executed)"
        echo ""
        echo '```bash'
        echo "python3 ${SCRIPT_DIR}/${SYNC_VERIFIER_NAME} \\"
        echo "  --front-topic /livox/lidar_front --back-topic /livox/lidar_back \\"
        echo "  --sample-count ${SYNC_SAMPLE_COUNT} --timeout-s ${SYNC_TIMEOUT_S}"
        echo ""
        echo "python3 ${SCRIPT_DIR}/${DRIFT_ANALYZER_NAME} \\"
        echo "  --bag <bag-dir> --odometry-topic ${ODOMETRY_TOPIC} \\"
        echo "  --max-drift-cm-per-min ${MAX_DRIFT_CM_PER_MIN}"
        echo '```'
    } >"$out"
    log_info "dry-run plan written: ${out}"
}

# =============================================================================
# Sync verifier invocation
# =============================================================================

run_sync_verifier() {
    local out="${OUTPUT_DIR}/task-16-sync.log"
    log_info "invoking ${SYNC_VERIFIER_NAME}" >&2
    if [ ! -f "$PRE_SYNC_SCRIPT_PATH" ]; then
        write_blocked_marker "$out" "sync verifier script missing: ${PRE_SYNC_SCRIPT_PATH}"
        return 2
    fi
    local verifier_exit=0
    {
        echo "=== ${SYNC_VERIFIER_NAME} invocation $(date -u +"%Y-%m-%dT%H:%M:%SZ") ==="
        python3 "$PRE_SYNC_SCRIPT_PATH" \
            --front-topic /livox/lidar_front \
            --back-topic /livox/lidar_back \
            --sample-count "$SYNC_SAMPLE_COUNT" \
            --timeout-s "$SYNC_TIMEOUT_S" \
            --sync-tolerance-ms "$SYNC_TOLERANCE_MS" 2>&1
        verifier_exit=${PIPESTATUS[0]}
        echo ""
        echo "=== verifier_exit=${verifier_exit} ==="
    } >"$out" 2>&1
    log_info "sync verifier output -> ${out} (exit=${verifier_exit})" >&2
    return "$verifier_exit"
}

# =============================================================================
# Runtime orchestration (real mode)
# =============================================================================

NAV_PID=""
BAG_PID=""
MERGER_DIAG_PID=""
STACK_BAG_PATH=""

cleanup_children() {
    local rc=$?
    log_info "=== cleanup ==="
    if [ -n "$BAG_PID" ] && kill -0 "$BAG_PID" 2>/dev/null; then
        log_info "stopping ros2 bag PID ${BAG_PID}"
        kill -INT "$BAG_PID" 2>/dev/null || true
        sleep 2
        kill -9 "$BAG_PID" 2>/dev/null || true
    fi
    if [ -n "$MERGER_DIAG_PID" ] && kill -0 "$MERGER_DIAG_PID" 2>/dev/null; then
        kill -INT "$MERGER_DIAG_PID" 2>/dev/null || true
        sleep 1
        kill -9 "$MERGER_DIAG_PID" 2>/dev/null || true
    fi
    if [ -n "$NAV_PID" ] && kill -0 "$NAV_PID" 2>/dev/null; then
        log_info "stopping nav launch PID ${NAV_PID}"
        kill -INT "$NAV_PID" 2>/dev/null || true
        sleep 3
        kill -9 "$NAV_PID" 2>/dev/null || true
    fi
    pkill -9 -f '[r]m_navigation_reality_launch.py'  2>/dev/null || true
    pkill -9 -f '[l]ivox_ros_driver2_node'           2>/dev/null || true
    pkill -9 -f '[p]ointcloud_merger'                2>/dev/null || true
    pkill -9 -f '[m]erger_node'                      2>/dev/null || true
    pkill -9 -f '[p]oint_lio'                        2>/dev/null || true
    sleep 1
    exit "$rc"
}

run_real_stack() {
    trap cleanup_children EXIT INT TERM
    STACK_BAG_PATH=""

    local launch_log="${OUTPUT_DIR}/task-16-launch.log"
    local bag_log="${OUTPUT_DIR}/task-16-rosbag.log"
    local bag_dir="${OUTPUT_DIR}/task-16-odometry.bag"
    if [ -e "$bag_dir" ]; then
        rm -rf "$bag_dir"
    fi

    log_info "launching ${NAV_LAUNCH_FILE} (slam=${SLAM}, use_dual_mid360=True)"
    local launch_args=(
        "slam:=${SLAM}"
        "use_dual_mid360:=True"
        "use_robot_state_pub:=True"
    )
    [ -n "$NAMESPACE" ] && launch_args+=("namespace:=${NAMESPACE}")
    [ -n "$WORLD" ] && launch_args+=("world:=${WORLD}")

    "$PRE_ROS2_BIN" launch "$NAV_LAUNCH_PKG" "$NAV_LAUNCH_FILE" \
        "${launch_args[@]}" >"$launch_log" 2>&1 &
    NAV_PID=$!
    log_info "nav launch PID: ${NAV_PID} (log: ${launch_log})"

    log_info "sleeping ${NAV_WARMUP}s for stack warmup"
    sleep "$NAV_WARMUP"

    if ! kill -0 "$NAV_PID" 2>/dev/null; then
        log_fail "nav launch exited during warmup"
        PRE_BLOCKED_REASONS+=("nav launch exited during warmup; see task-16-launch.log")
        return 2
    fi

    local sync_rc=0
    run_sync_verifier || sync_rc=$?
    if [ "$sync_rc" -ne 0 ]; then
        log_fail "sync verifier failed (exit=${sync_rc}); refusing to record bag"
        PRE_BLOCKED_REASONS+=(
            "sync verifier failed (exit=${sync_rc}); see task-16-sync.log"
        )
        return 2
    fi

    if [ -n "$MERGER_DIAG_TOPIC" ]; then
        local diag_out="${OUTPUT_DIR}/task-16-merger-diag.txt"
        local diag_rc=0
        timeout --signal=INT --preserve-status 5s \
            "$PRE_ROS2_BIN" topic echo --once "$MERGER_DIAG_TOPIC" \
            >"$diag_out" 2>&1 || diag_rc=$?
        if [ "$diag_rc" -ne 0 ]; then
            write_blocked_marker "$diag_out" \
                "diagnostics topic ${MERGER_DIAG_TOPIC} unavailable or timed out after 5s (rc=${diag_rc})"
        fi
    else
        write_blocked_marker "${OUTPUT_DIR}/task-16-merger-diag.txt" \
            "no --merger-diag-topic supplied; live diagnostics not captured"
    fi

    log_info "recording ${ODOMETRY_TOPIC} for ~${DURATION}s -> ${bag_dir}"
    timeout --signal=INT --preserve-status "${DURATION}s" \
        "$PRE_ROS2_BIN" bag record \
        -o "$bag_dir" \
        "$ODOMETRY_TOPIC" >"$bag_log" 2>&1 &
    BAG_PID=$!
    local bag_exit=0
    wait "$BAG_PID" || bag_exit=$?
    BAG_PID=""

    if [ "$bag_exit" -ne 0 ] && [ "$bag_exit" -ne 143 ] && [ "$bag_exit" -ne 130 ]; then
        log_fail "bag record exited with code ${bag_exit}"
        PRE_BLOCKED_REASONS+=("bag record exited with code ${bag_exit}; see task-16-rosbag.log")
        return 2
    fi

    if [ ! -d "$bag_dir" ]; then
        log_fail "bag directory not created: ${bag_dir}"
        PRE_BLOCKED_REASONS+=("bag directory not created: ${bag_dir}")
        return 2
    fi

    local pl_out="${OUTPUT_DIR}/task-16-pointlio.log"
    local pl_errs="${OUTPUT_DIR}/task-16-pointlio-errors.txt"
    local mg_out="${OUTPUT_DIR}/task-16-merger.log"
    {
        echo "=== excerpt from task-16-launch.log grep point_lio ==="
        grep -Ei "point[_-]?lio|aft_mapped_to_init|lidar loop back" \
            "$launch_log" 2>/dev/null || echo "(no point_lio lines found in launch log)"
    } >"$pl_out"
    {
        echo "=== excerpt from task-16-launch.log grep merger ==="
        grep -Ei "merger|pointcloud_merger|sync_tolerance" \
            "$launch_log" 2>/dev/null || echo "(no merger lines found in launch log)"
    } >"$mg_out"

    local loop_back_count=0
    loop_back_count=$(grep -ci "lidar loop back" "$launch_log" 2>/dev/null || true)
    loop_back_count=${loop_back_count:-0}
    if ! [[ "$loop_back_count" =~ ^[0-9]+$ ]]; then
        loop_back_count=0
    fi
    {
        echo "# task-16-pointlio-errors.txt"
        echo ""
        echo "Generated: $(date -u +"%Y-%m-%dT%H:%M:%SZ")"
        echo "Source log: ${launch_log}"
        echo ""
        echo "loop_back_match_count: ${loop_back_count}"
        echo ""
        if [ "$loop_back_count" -gt 0 ]; then
            echo "## matching lines"
            echo ""
            grep -ni "lidar loop back" "$launch_log" 2>/dev/null || true
        else
            echo "## matching lines"
            echo ""
            echo "(no 'lidar loop back' occurrences found in launch log)"
        fi
    } >"$pl_errs"

    if [ "$loop_back_count" -gt 0 ]; then
        log_fail "point_lio reported 'lidar loop back' ${loop_back_count} time(s); see ${pl_errs}"
        PRE_BLOCKED_REASONS+=(
            "point_lio reported 'lidar loop back' ${loop_back_count} time(s); see task-16-pointlio-errors.txt"
        )
        return 1
    fi

    STACK_BAG_PATH="$bag_dir"
    return 0
}

# =============================================================================
# Analyzer invocation
# =============================================================================

run_analyzer() {
    local bag_path="$1"
    local out_md="${OUTPUT_DIR}/task-16-analyze.md"
    local out_json="${OUTPUT_DIR}/task-16-analyze.json"
    mkdir -p "$OUTPUT_DIR"

    if [ ! -f "$PRE_ANALYZER_PATH" ]; then
        write_blocked_marker "$out_md" \
            "drift analyzer missing: ${PRE_ANALYZER_PATH}"
        return 2
    fi

    local rc=0
    python3 "$PRE_ANALYZER_PATH" \
        --bag "$bag_path" \
        --odometry-topic "$ODOMETRY_TOPIC" \
        --max-drift-cm-per-min "$MAX_DRIFT_CM_PER_MIN" \
        --output "$out_md" \
        --json-output "$out_json" || rc=$?

    if [ "$rc" -eq 0 ]; then
        log_ok "analyzer verdict: PASS"
        return 0
    fi
    if [ "$rc" -eq 1 ]; then
        log_fail "analyzer verdict: FAIL"
        return 1
    fi
    log_warn "analyzer verdict: BLOCKED (rc=${rc})"
    return 2
}

# =============================================================================
# Main
# =============================================================================

main() {
    parse_args "$@"
    validate_args

    mkdir -p "$OUTPUT_DIR"
    log_info "test_real_dual_mid360_static.sh v${SCRIPT_VERSION}"
    log_info "workspace: ${WS_ROOT}"
    log_info "MODE_REAL=${MODE_REAL} MODE_PREFLIGHT=${MODE_PREFLIGHT} MODE_SKIP_HARDWARE=${MODE_SKIP_HARDWARE} MODE_FAKE_BAG=${MODE_FAKE_BAG} DRY_RUN=${DRY_RUN}"

    run_preflight
    write_preflight_evidence

    if $DRY_RUN; then
        do_dry_run
        if preflight_is_blocked; then
            write_blocker_evidence
            write_summary "dry-run" "BLOCKED" ""
            log_warn "dry-run complete; preflight is BLOCKED"
            exit 2
        fi
        write_summary "dry-run" "OK" ""
        log_ok "dry-run complete"
        exit 0
    fi

    if $MODE_PREFLIGHT; then
        if preflight_is_blocked; then
            write_blocker_evidence
            write_summary "preflight" "BLOCKED" ""
            log_fail "preflight BLOCKED"
            exit 2
        fi
        write_summary "preflight" "PASS" ""
        log_ok "preflight PASS"
        exit 0
    fi

    local bag_to_analyze=""

    if $MODE_SKIP_HARDWARE; then
        if preflight_is_blocked; then
            write_blocker_evidence
            write_blocked_marker "${OUTPUT_DIR}/task-16-pointlio.log" \
                "--skip-hardware mode: no live runtime; pointlio log not captured"
            write_blocked_marker "${OUTPUT_DIR}/task-16-merger.log" \
                "--skip-hardware mode: no live runtime; merger log not captured"
            write_blocked_marker "${OUTPUT_DIR}/task-16-merger-diag.txt" \
                "--skip-hardware mode: no live runtime; diagnostics not captured"
            write_blocked_marker "${OUTPUT_DIR}/task-16-sync.log" \
                "--skip-hardware mode: no live runtime; sync verifier not invoked"
            write_pointlio_errors_blocked \
                "--skip-hardware mode: preflight BLOCKED; no live runtime; point_lio 'lidar loop back' check not performed"
            write_summary "skip-hardware" "BLOCKED" "$BAG_PATH"
            log_fail "skip-hardware BLOCKED"
            exit 2
        fi
        bag_to_analyze="$BAG_PATH"
        write_blocked_marker "${OUTPUT_DIR}/task-16-pointlio.log" \
            "--skip-hardware mode: no live runtime; pointlio log not captured"
        write_blocked_marker "${OUTPUT_DIR}/task-16-merger.log" \
            "--skip-hardware mode: no live runtime; merger log not captured"
        write_blocked_marker "${OUTPUT_DIR}/task-16-merger-diag.txt" \
            "--skip-hardware mode: no live runtime; diagnostics not captured"
        write_blocked_marker "${OUTPUT_DIR}/task-16-sync.log" \
            "--skip-hardware mode: no live runtime; sync verifier not invoked"
        write_pointlio_errors_blocked \
            "--skip-hardware mode: no live runtime; point_lio 'lidar loop back' check not performed on user-supplied bag"
    elif $MODE_FAKE_BAG; then
        if preflight_is_blocked; then
            write_blocker_evidence
            write_blocked_marker "${OUTPUT_DIR}/task-16-pointlio.log" \
                "--fake-bag mode: no live runtime; pointlio log not captured"
            write_blocked_marker "${OUTPUT_DIR}/task-16-merger.log" \
                "--fake-bag mode: no live runtime; merger log not captured"
            write_blocked_marker "${OUTPUT_DIR}/task-16-merger-diag.txt" \
                "--fake-bag mode: no live runtime; diagnostics not captured"
            write_blocked_marker "${OUTPUT_DIR}/task-16-sync.log" \
                "--fake-bag mode: no live runtime; sync verifier not invoked"
            write_pointlio_errors_blocked \
                "--fake-bag mode: preflight BLOCKED; no live runtime; point_lio 'lidar loop back' check not performed"
            write_summary "fake-bag" "BLOCKED" "$FAKE_BAG_PATH"
            log_fail "fake-bag BLOCKED"
            exit 2
        fi
        bag_to_analyze="$FAKE_BAG_PATH"
        write_blocked_marker "${OUTPUT_DIR}/task-16-pointlio.log" \
            "--fake-bag mode: no live runtime; pointlio log not captured"
        write_blocked_marker "${OUTPUT_DIR}/task-16-merger.log" \
            "--fake-bag mode: no live runtime; merger log not captured"
        write_blocked_marker "${OUTPUT_DIR}/task-16-merger-diag.txt" \
            "--fake-bag mode: no live runtime; diagnostics not captured"
        write_blocked_marker "${OUTPUT_DIR}/task-16-sync.log" \
            "--fake-bag mode: no live runtime; sync verifier not invoked"
        write_pointlio_errors_blocked \
            "--fake-bag mode: no live runtime; point_lio 'lidar loop back' check not performed on fake bag"
    else
        if preflight_is_blocked; then
            write_blocker_evidence
            write_blocked_marker "${OUTPUT_DIR}/task-16-pointlio.log" \
                "real-mode preflight BLOCKED; live runtime not started"
            write_blocked_marker "${OUTPUT_DIR}/task-16-merger.log" \
                "real-mode preflight BLOCKED; live runtime not started"
            write_blocked_marker "${OUTPUT_DIR}/task-16-merger-diag.txt" \
                "real-mode preflight BLOCKED; live runtime not started"
            write_blocked_marker "${OUTPUT_DIR}/task-16-sync.log" \
                "real-mode preflight BLOCKED; sync verifier not invoked"
            write_blocked_marker "${OUTPUT_DIR}/task-16-analyze.md" \
                "real-mode preflight BLOCKED; no bag recorded"
            write_pointlio_errors_blocked \
                "real-mode preflight BLOCKED; live runtime not started; point_lio 'lidar loop back' check not performed"
            write_summary "real-preflight" "BLOCKED" ""
            log_fail "real-mode preflight BLOCKED -- refusing to launch stack"
            exit 2
        fi
        local stack_rc=0
        run_real_stack || stack_rc=$?
        if [ "$stack_rc" -eq 0 ] && [ -n "$STACK_BAG_PATH" ] && [ -d "$STACK_BAG_PATH" ]; then
            bag_to_analyze="$STACK_BAG_PATH"
        else
            write_blocker_evidence
            write_blocked_marker "${OUTPUT_DIR}/task-16-analyze.md" \
                "real-mode runtime did not produce a usable bag (stack_rc=${stack_rc})"
            if [ ! -f "${OUTPUT_DIR}/task-16-pointlio-errors.txt" ]; then
                write_pointlio_errors_blocked \
                    "real-mode runtime aborted before point_lio 'lidar loop back' check (stack_rc=${stack_rc}); see task-16-launch.log / task-16-sync.log / task-16-rosbag.log"
            fi
            write_summary "real-runtime" "BLOCKED" "${STACK_BAG_PATH:-}"
            log_fail "real-mode runtime BLOCKED -- no bag produced (stack_rc=${stack_rc})"
            exit 2
        fi
    fi

    local analyzer_rc=0
    run_analyzer "$bag_to_analyze" || analyzer_rc=$?

    case "$analyzer_rc" in
        0)
            write_summary "analyze" "PASS" "$bag_to_analyze"
            log_ok "Task 16 verdict: PASS"
            exit 0
            ;;
        1)
            write_summary "analyze" "FAIL" "$bag_to_analyze"
            log_fail "Task 16 verdict: FAIL"
            exit 1
            ;;
        *)
            PRE_BLOCKED_REASONS+=("analyzer returned BLOCKED; see task-16-analyze.md")
            write_blocker_evidence
            write_summary "analyze" "BLOCKED" "$bag_to_analyze"
            log_fail "Task 16 verdict: BLOCKED"
            exit 2
            ;;
    esac
}

main "$@"
