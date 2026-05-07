#!/bin/bash
# test_sim_dual_mid360.sh
# Dual Mid360 simulation full-chain smoke orchestrator for Sentry26.
#
# This script orchestrates the two-terminal Gazebo + Nav2 startup sequence
# documented in src/docs/QUICKSTART.md and AGENTS.md:
#
#   Terminal 1: ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
#               headless:=true
#   (manual) gz service set_performer + unpause
#   (manual) wait ~10s for sim clock to stabilise
#   Terminal 2: ros2 launch sentry_nav_bringup rm_navigation_simulation_launch.py
#               world:=rmuc_2026 slam:=True use_dual_mid360:=True
#
# What this script does:
#   * preflight: verify ROS2, required packages, bridge entries, launch args;
#     emit clear BLOCKED diagnostics if the current environment or simulator
#     configuration cannot support the requested mode.
#   * dry-run: print planned commands + preflight snapshot; create no processes.
#   * runtime: if preflight PASSES, actually orchestrate the two-terminal
#     sequence end-to-end in-script, capture topic/TF evidence, optionally
#     fire a NavigateToPose goal, then clean up all child processes.
#
# DEFAULT BEHAVIOUR:
#   * Modes tested: dual (use_dual_mid360:=True) by default; --mode can select
#     dual, single (use_dual_mid360:=False), or both (dual then single).
#   * --headless defaults to true so the script is safe to run on CI /
#     headless dev hosts.
#   * Evidence is written to .sisyphus/evidence/task-15-*.
#
# EXIT CODES:
#   0  smoke PASSED (or dry-run / preflight-only path completed cleanly)
#   1  smoke FAILED with a concrete runtime issue (topics missing, launch
#      crashed, goal never ACCEPTED, etc.)
#   2  BLOCKED — the environment or simulator configuration cannot support
#      the requested mode (e.g. ros_gz_bridge.yaml lacks back_mid360 entries
#      in dual mode, or Gazebo is not installed). Evidence file explains
#      exactly what is missing and how to unblock.
#   3  usage / argument error.
#
# IMPORTANT: This script does NOT modify any source file, launch, yaml, or
# package outside .sisyphus/evidence/. T15 is strictly a smoke orchestrator.
#
# Reference: .sisyphus/plans/dual-mid360-fusion.md Task 15 acceptance criteria.

set -euo pipefail

# =============================================================================
# Constants
# =============================================================================

SCRIPT_VERSION="0.1.0"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Resolve workspace root: scripts/ -> pkg/ -> sentry_nav/ -> src/ -> ws/
WS_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

# Launch entry points used by T15.
GAZEBO_LAUNCH_PKG="rmu_gazebo_simulator"
GAZEBO_LAUNCH_FILE="bringup_sim.launch.py"
NAV_LAUNCH_PKG="sentry_nav_bringup"
NAV_LAUNCH_FILE="rm_navigation_simulation_launch.py"

# Required ROS2 packages (colcon install / apt) for the full dual smoke chain.
REQUIRED_PACKAGES=(
    "sentry_nav_bringup"
    "sentry_dual_mid360"
    "rmu_gazebo_simulator"
    "sentry_robot_description"
    "livox_ros_driver2"
)

# Bridge yaml that decides which Gazebo LiDAR/IMU topics become ROS topics.
# T15 only inspects it; never modifies it.
BRIDGE_YAML_REL="src/simulator/rmu_gazebo_simulator/rmu_gazebo_simulator/config/ros_gz_bridge.yaml"

# Topics we sanity-check during runtime smoke. Each is prefixed with the
# namespace at runtime.
# NOTE: in single (use_dual_mid360:=False) mode, the T13 launch integration
# sets Point-LIO common.lid_topic=livox/lidar_front (no-merger front-lidar-
# direct), so we must check /livox/lidar_front here — NOT /livox/lidar. Dual
# mode still consumes the merger output on /livox/lidar.
DUAL_TOPICS=(
    "livox/lidar_front"
    "livox/lidar_back"
    "livox/lidar"
    "aft_mapped_to_init"
    "registered_scan"
    "odometry"
)
SINGLE_TOPICS=(
    "livox/lidar_front"
    "aft_mapped_to_init"
    "registered_scan"
    "odometry"
)

# TF frames we sanity-check via `tf2_echo` during runtime smoke.
TF_PAIRS=(
    "map:odom"
    "odom:base_footprint"
    "gimbal_pitch:front_mid360"
)
TF_PAIRS_DUAL_EXTRA=(
    "gimbal_pitch:back_mid360"
)

# =============================================================================
# Defaults (overridable via flags)
# =============================================================================

MODE="dual"                  # dual | single | both
WORLD="rmuc_2026"
NAMESPACE="red_standard_robot1"
SLAM="True"
HEADLESS="true"
DURATION=30                  # seconds to keep stack running before tearing down
STABILISE_WAIT=10            # seconds between unpause and nav launch
NAV_WARMUP=20                # seconds after nav launch before topic checks
DO_GOAL=true                 # whether to send a NavigateToPose goal
GOAL_X=2.0
GOAL_Y=0.0
GOAL_FRAME="map"
GOAL_WAIT=20                 # seconds to wait for goal ACCEPTED / feedback
OUTPUT_DIR=""
DRY_RUN=false
PREFLIGHT_ONLY=false
VERBOSE=false

# =============================================================================
# Logging helpers
# =============================================================================

log_info()  { echo "[INFO]  $*"; }
log_warn()  { echo "[WARN]  $*" >&2; }
log_error() { echo "[ERROR] $*" >&2; }
log_ok()    { echo "[OK]    $*"; }
log_fail()  { echo "[FAIL]  $*" >&2; }

# =============================================================================
# Help
# =============================================================================

print_usage() {
    cat <<EOF
test_sim_dual_mid360.sh — dual Mid360 simulation smoke orchestrator

USAGE:
    bash test_sim_dual_mid360.sh [OPTIONS]

OPTIONS:
    --help, -h              Show this help message and exit.
    --version               Print script version and exit.
    --dry-run               Print planned commands + preflight snapshot; do
                            NOT launch Gazebo or Nav2.
    --preflight             Run preflight checks only; do NOT launch Gazebo or
                            Nav2. Writes task-15-preflight.md evidence.
    --mode MODE             dual | single | both   [default: ${MODE}]
                            "dual"   -> use_dual_mid360:=True
                            "single" -> use_dual_mid360:=False
                            "both"   -> runs dual first, then single
    --world WORLD           Gazebo world name           [default: ${WORLD}]
    --namespace NS          Robot namespace             [default: ${NAMESPACE}]
    --slam True|False       Nav2 slam flag              [default: ${SLAM}]
    --headless true|false   Gazebo headless             [default: ${HEADLESS}]
    --duration SECS         Seconds to keep stack alive [default: ${DURATION}]
    --stabilise-wait SECS   Wait after unpause          [default: ${STABILISE_WAIT}]
    --nav-warmup SECS       Wait after Nav2 launch      [default: ${NAV_WARMUP}]
    --skip-goal             Skip NavigateToPose probe.
    --goal-x X              Goal x (map frame)          [default: ${GOAL_X}]
    --goal-y Y              Goal y (map frame)          [default: ${GOAL_Y}]
    --goal-wait SECS        NavigateToPose wait         [default: ${GOAL_WAIT}]
    --output-dir DIR        Evidence output directory
                            [default: <ws>/.sisyphus/evidence]
    --verbose               More verbose logging.

EXIT CODES:
    0  PASS  (or clean dry-run / preflight completion)
    1  FAIL  (runtime reached but topics/TF/goal failed)
    2  BLOCKED (preflight found missing packages / bridge entries / Gazebo)
    3  usage / argument error

EVIDENCE FILES:
    task-15-preflight.md            Preflight matrix (packages, bridge, launch).
    task-15-dry-run.md              Dry-run plan dump.
    task-15-runtime.md              Runtime orchestration summary (per mode).
    task-15-dual-run.log            Raw stdout/stderr of dual-mode runtime.
    task-15-single-run.log          Raw stdout/stderr of single-mode runtime.
    task-15-topics-<mode>.txt       \`ros2 topic hz\` per topic.
    task-15-tf-<mode>.txt           \`ros2 run tf2_ros tf2_echo\` per pair.
    task-15-goal-<mode>.log         \`ros2 action send_goal\` transcript.
    task-15-blocker.md              Present only when exit=2 (BLOCKED).

STARTUP SEQUENCE (reference, documented in src/docs/QUICKSTART.md):
    1. ros2 launch rmu_gazebo_simulator bringup_sim.launch.py headless:=true
    2. gz service -s /world/default/level/set_performer
         --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean
         --timeout 2000 --req 'data: "<namespace>"'
    3. gz service -s /world/default/control
         --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean
         --timeout 5000 --req 'pause: false'
    4. sleep ${STABILISE_WAIT}   # let sim clock + sensors settle
    5. ros2 launch sentry_nav_bringup rm_navigation_simulation_launch.py
         world:=<world> slam:=<slam> use_dual_mid360:=<True|False>
    6. sleep ${NAV_WARMUP}       # let Point-LIO + Nav2 spin up
    7. Topic / TF checks          (ros2 topic hz, tf2_echo)
    8. Optional NavigateToPose goal
    9. Teardown: kill Nav2 launch, then Gazebo launch, verify clean.

REFERENCES:
    src/docs/QUICKSTART.md
    AGENTS.md §4 (仿真启动顺序)
    .sisyphus/plans/dual-mid360-fusion.md Task 15
EOF
}

# =============================================================================
# Argument parsing
# =============================================================================

parse_args() {
    while [ $# -gt 0 ]; do
        case "$1" in
            --help|-h)
                print_usage
                exit 0
                ;;
            --version)
                echo "test_sim_dual_mid360.sh v${SCRIPT_VERSION}"
                exit 0
                ;;
            --dry-run)
                DRY_RUN=true
                shift
                ;;
            --preflight)
                PREFLIGHT_ONLY=true
                shift
                ;;
            --mode)
                [ $# -ge 2 ] || { log_error "--mode requires an argument"; exit 3; }
                MODE="$2"
                shift 2
                ;;
            --world)
                [ $# -ge 2 ] || { log_error "--world requires an argument"; exit 3; }
                WORLD="$2"
                shift 2
                ;;
            --namespace)
                [ $# -ge 2 ] || { log_error "--namespace requires an argument"; exit 3; }
                NAMESPACE="$2"
                shift 2
                ;;
            --slam)
                [ $# -ge 2 ] || { log_error "--slam requires an argument"; exit 3; }
                SLAM="$2"
                shift 2
                ;;
            --headless)
                [ $# -ge 2 ] || { log_error "--headless requires an argument"; exit 3; }
                HEADLESS="$2"
                shift 2
                ;;
            --duration)
                [ $# -ge 2 ] || { log_error "--duration requires an argument"; exit 3; }
                DURATION="$2"
                shift 2
                ;;
            --stabilise-wait)
                [ $# -ge 2 ] || { log_error "--stabilise-wait requires an argument"; exit 3; }
                STABILISE_WAIT="$2"
                shift 2
                ;;
            --nav-warmup)
                [ $# -ge 2 ] || { log_error "--nav-warmup requires an argument"; exit 3; }
                NAV_WARMUP="$2"
                shift 2
                ;;
            --skip-goal)
                DO_GOAL=false
                shift
                ;;
            --goal-x)
                [ $# -ge 2 ] || { log_error "--goal-x requires an argument"; exit 3; }
                GOAL_X="$2"
                shift 2
                ;;
            --goal-y)
                [ $# -ge 2 ] || { log_error "--goal-y requires an argument"; exit 3; }
                GOAL_Y="$2"
                shift 2
                ;;
            --goal-wait)
                [ $# -ge 2 ] || { log_error "--goal-wait requires an argument"; exit 3; }
                GOAL_WAIT="$2"
                shift 2
                ;;
            --output-dir)
                [ $# -ge 2 ] || { log_error "--output-dir requires an argument"; exit 3; }
                OUTPUT_DIR="$2"
                shift 2
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
}

validate_args() {
    case "$MODE" in
        dual|single|both) : ;;
        *)
            log_error "--mode must be 'dual', 'single', or 'both' (got: '${MODE}')"
            exit 3
            ;;
    esac
    case "$SLAM" in
        True|False|true|false) : ;;
        *)
            log_error "--slam must be True or False (got: '${SLAM}')"
            exit 3
            ;;
    esac
    case "$HEADLESS" in
        true|false) : ;;
        *)
            log_error "--headless must be 'true' or 'false' (got: '${HEADLESS}')"
            exit 3
            ;;
    esac
    for var_name in DURATION STABILISE_WAIT NAV_WARMUP GOAL_WAIT; do
        local v="${!var_name}"
        if ! [[ "$v" =~ ^[0-9]+$ ]] || [ "$v" -lt 0 ]; then
            log_error "--${var_name,,} must be a non-negative integer (got: '${v}')"
            exit 3
        fi
    done
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

find_gz_bin() {
    if command -v gz &>/dev/null; then
        command -v gz
        return 0
    fi
    return 1
}

source_ws_setup() {
    # Try to source install/setup.bash from the current worktree; fallback to
    # /home/pc/Documents/Sentry26/install/setup.bash if the current worktree
    # was not fully built (as is typical for feature worktrees).
    local candidates=(
        "${WS_ROOT}/install/setup.bash"
        "/home/pc/Documents/Sentry26/install/setup.bash"
    )
    for c in "${candidates[@]}"; do
        if [ -f "$c" ]; then
            # ROS2 setup.bash trips `set -u` on COLCON_TRACE; toggle -u around it.
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
# Preflight checks
# =============================================================================

# Populated by run_preflight().
PRE_ROS2_BIN=""
PRE_GZ_BIN=""
PRE_MISSING_PKGS=()
PRE_BRIDGE_HAS_STABLE_FRONT=false     # /<ns>/livox/lidar_front present
PRE_BRIDGE_HAS_STABLE_BACK=false      # /<ns>/livox/lidar_back present
PRE_BRIDGE_FRONT_TYPE=""              # ros message type for front stable topic (if any)
PRE_BRIDGE_BACK_TYPE=""               # ros message type for back stable topic (if any)
PRE_BRIDGE_HAS_LEGACY_LIDAR=false     # /<ns>/livox/lidar (single-lidar era) present
PRE_BRIDGE_LEGACY_TYPE=""             # ros message type for legacy topic (if any)
PRE_MSG_TYPE_MISMATCH=false           # bridge produces PointCloud2 while merger wants CustomMsg
PRE_LAUNCH_HAS_USE_DUAL=false
PRE_BLOCKED_REASONS=()

# Parse ros_gz_bridge.yaml and populate the PRE_BRIDGE_* variables.
# We look for the three topic names the rest of the stack relies on:
#   /<robot_name>/livox/lidar_front  — merger front input (expected CustomMsg)
#   /<robot_name>/livox/lidar_back   — merger back  input (expected CustomMsg)
#   /<robot_name>/livox/lidar        — legacy single-lidar topic (observed PointCloud2)
# and record their ros_type_name so we can flag PointCloud2-vs-CustomMsg mismatches.
parse_bridge_yaml() {
    local yaml="$1"
    [ -f "$yaml" ] || return 0
    python3 - "$yaml" <<'PY' 2>/dev/null || true
import sys, yaml, json
path = sys.argv[1]
with open(path) as fh:
    data = yaml.safe_load(fh) or []
rows = []
for entry in data:
    ros = (entry or {}).get("ros_topic_name", "") or ""
    typ = (entry or {}).get("ros_type_name", "") or ""
    rows.append((ros, typ))
out = {"front_type": "", "back_type": "", "legacy_type": ""}
for ros, typ in rows:
    base = ros.replace("/<robot_name>", "")
    if base == "/livox/lidar_front":
        out["front_type"] = typ
    elif base == "/livox/lidar_back":
        out["back_type"] = typ
    elif base == "/livox/lidar":
        out["legacy_type"] = typ
print(json.dumps(out))
PY
}

run_preflight() {
    log_info "=== Preflight checks ==="

    # 1. ros2 binary
    if PRE_ROS2_BIN="$(find_ros2_bin)"; then
        log_ok "ros2 binary: ${PRE_ROS2_BIN}"
    else
        PRE_ROS2_BIN=""
        log_fail "ros2 binary not found on PATH or /opt/ros/*/bin/"
        PRE_BLOCKED_REASONS+=("ros2 binary missing (source /opt/ros/jazzy/setup.bash)")
    fi

    # 2. Source workspace setup.bash (needed for ros2 pkg prefix / ros2 launch).
    if source_ws_setup; then
        log_ok "workspace setup.bash sourced"
    else
        log_warn "no workspace install/setup.bash found; package checks may be unreliable"
    fi

    # 3. Required ROS2 packages
    if [ -n "$PRE_ROS2_BIN" ]; then
        for pkg in "${REQUIRED_PACKAGES[@]}"; do
            if "$PRE_ROS2_BIN" pkg prefix "$pkg" &>/dev/null; then
                log_ok "package present: ${pkg}"
            else
                log_fail "package MISSING: ${pkg}"
                PRE_MISSING_PKGS+=("$pkg")
            fi
        done
        if [ ${#PRE_MISSING_PKGS[@]} -gt 0 ]; then
            PRE_BLOCKED_REASONS+=("missing ROS2 packages: ${PRE_MISSING_PKGS[*]}")
        fi
    fi

    # 4. Gazebo binary (gz). Needed for unpause + set_performer services.
    if PRE_GZ_BIN="$(find_gz_bin)"; then
        log_ok "gz binary: ${PRE_GZ_BIN}"
    else
        PRE_GZ_BIN=""
        log_fail "gz binary not found; Gazebo Harmonic (gz-sim) not installed"
        PRE_BLOCKED_REASONS+=("gz binary missing (install Gazebo Harmonic / gz-sim)")
    fi

    # 5. ros_gz_bridge.yaml: semantic check against the STABLE topic names the
    #    rest of the stack depends on. Simple sensor-name grep is not enough
    #    because T13 bound Point-LIO (single mode) to /livox/lidar_front and
    #    the merger (dual mode) to /livox/lidar_front + /livox/lidar_back.
    local bridge_yaml="${WS_ROOT}/${BRIDGE_YAML_REL}"
    if [ -f "$bridge_yaml" ]; then
        log_ok "bridge yaml found: ${bridge_yaml}"
        local bridge_json=""
        if command -v python3 &>/dev/null; then
            bridge_json="$(parse_bridge_yaml "$bridge_yaml")"
        fi
        if [ -n "$bridge_json" ]; then
            PRE_BRIDGE_FRONT_TYPE=$(printf '%s' "$bridge_json" | python3 -c "import json,sys;print(json.load(sys.stdin).get('front_type',''))")
            PRE_BRIDGE_BACK_TYPE=$(printf '%s' "$bridge_json" | python3 -c "import json,sys;print(json.load(sys.stdin).get('back_type',''))")
            PRE_BRIDGE_LEGACY_TYPE=$(printf '%s' "$bridge_json" | python3 -c "import json,sys;print(json.load(sys.stdin).get('legacy_type',''))")
            [ -n "$PRE_BRIDGE_FRONT_TYPE" ]  && PRE_BRIDGE_HAS_STABLE_FRONT=true
            [ -n "$PRE_BRIDGE_BACK_TYPE" ]   && PRE_BRIDGE_HAS_STABLE_BACK=true
            [ -n "$PRE_BRIDGE_LEGACY_TYPE" ] && PRE_BRIDGE_HAS_LEGACY_LIDAR=true
        else
            log_warn "python3/yaml unavailable; falling back to coarse grep on bridge yaml"
            if grep -q "livox/lidar_front" "$bridge_yaml"; then
                PRE_BRIDGE_HAS_STABLE_FRONT=true
                PRE_BRIDGE_FRONT_TYPE="unknown"
            fi
            if grep -q "livox/lidar_back" "$bridge_yaml"; then
                PRE_BRIDGE_HAS_STABLE_BACK=true
                PRE_BRIDGE_BACK_TYPE="unknown"
            fi
            if grep -q "livox/lidar\"" "$bridge_yaml"; then
                PRE_BRIDGE_HAS_LEGACY_LIDAR=true
                PRE_BRIDGE_LEGACY_TYPE="unknown"
            fi
        fi

        if $PRE_BRIDGE_HAS_STABLE_FRONT; then
            log_ok "bridge yaml exposes stable /<ns>/livox/lidar_front (${PRE_BRIDGE_FRONT_TYPE})"
        else
            log_warn "bridge yaml does NOT expose /<ns>/livox/lidar_front"
        fi
        if $PRE_BRIDGE_HAS_STABLE_BACK; then
            log_ok "bridge yaml exposes stable /<ns>/livox/lidar_back (${PRE_BRIDGE_BACK_TYPE})"
        else
            log_warn "bridge yaml does NOT expose /<ns>/livox/lidar_back"
        fi
        if $PRE_BRIDGE_HAS_LEGACY_LIDAR; then
            log_warn "bridge yaml still exposes legacy /<ns>/livox/lidar (${PRE_BRIDGE_LEGACY_TYPE})"
        fi

        # Message-type mismatch: merger + T13 false-branch Point-LIO both subscribe
        # to livox_ros_driver2/msg/CustomMsg. If the ONLY bridged LiDAR output is
        # sensor_msgs/msg/PointCloud2 (front/back stable or legacy), the merger
        # cannot consume it and no CustomMsg conversion path is wired today.
        local any_customs_msg=false
        for t in "$PRE_BRIDGE_FRONT_TYPE" "$PRE_BRIDGE_BACK_TYPE"; do
            case "$t" in *CustomMsg*) any_customs_msg=true ;; esac
        done
        for t in "$PRE_BRIDGE_FRONT_TYPE" "$PRE_BRIDGE_BACK_TYPE" "$PRE_BRIDGE_LEGACY_TYPE"; do
            case "$t" in
                *PointCloud2*)
                    if ! $any_customs_msg; then
                        PRE_MSG_TYPE_MISMATCH=true
                    fi
                    ;;
            esac
        done
        if $PRE_MSG_TYPE_MISMATCH; then
            log_warn "bridge emits sensor_msgs/PointCloud2 but merger expects livox_ros_driver2/CustomMsg"
        fi
    else
        log_fail "bridge yaml not found at ${bridge_yaml}"
        PRE_BLOCKED_REASONS+=("bridge yaml missing at ${BRIDGE_YAML_REL}")
    fi

    # 6. Navigation launch exposes use_dual_mid360.
    if [ -n "$PRE_ROS2_BIN" ] && "$PRE_ROS2_BIN" pkg prefix "$NAV_LAUNCH_PKG" &>/dev/null; then
        local nav_share
        nav_share="$("$PRE_ROS2_BIN" pkg prefix "$NAV_LAUNCH_PKG")/share/${NAV_LAUNCH_PKG}"
        local nav_launch_path="${nav_share}/launch/${NAV_LAUNCH_FILE}"
        if [ -f "$nav_launch_path" ]; then
            if grep -q "use_dual_mid360" "$nav_launch_path"; then
                PRE_LAUNCH_HAS_USE_DUAL=true
                log_ok "nav launch exposes use_dual_mid360"
            else
                log_fail "nav launch does NOT expose use_dual_mid360"
                PRE_BLOCKED_REASONS+=("${NAV_LAUNCH_FILE} missing use_dual_mid360 argument")
            fi
        else
            log_fail "nav launch file missing: ${nav_launch_path}"
            PRE_BLOCKED_REASONS+=("nav launch file missing: ${nav_launch_path}")
        fi
    fi

    # 7. Mode-specific verdicts.
    #    Single mode: T13 false branch uses /livox/lidar_front directly.
    #    Dual mode:   merger wants /livox/lidar_front + /livox/lidar_back,
    #                 both as livox_ros_driver2/msg/CustomMsg.
    if [ "$MODE" = "single" ] || [ "$MODE" = "both" ]; then
        if ! $PRE_BRIDGE_HAS_STABLE_FRONT; then
            PRE_BLOCKED_REASONS+=(
                "single mode requires stable /<ns>/livox/lidar_front in ros_gz_bridge.yaml (T13 false branch sets Point-LIO common.lid_topic=livox/lidar_front)"
            )
        fi
    fi
    if [ "$MODE" = "dual" ] || [ "$MODE" = "both" ]; then
        if ! $PRE_BRIDGE_HAS_STABLE_FRONT; then
            PRE_BLOCKED_REASONS+=(
                "dual mode requires stable /<ns>/livox/lidar_front bridged as livox_ros_driver2/msg/CustomMsg (merger front input)"
            )
        fi
        if ! $PRE_BRIDGE_HAS_STABLE_BACK; then
            PRE_BLOCKED_REASONS+=(
                "dual mode requires stable /<ns>/livox/lidar_back bridged as livox_ros_driver2/msg/CustomMsg (merger back input)"
            )
        fi
        if $PRE_MSG_TYPE_MISMATCH; then
            PRE_BLOCKED_REASONS+=(
                "dual merger requires livox_ros_driver2/msg/CustomMsg but current ros_gz_bridge produces sensor_msgs/msg/PointCloud2; no CustomMsg conversion path is wired today"
            )
        fi
        # Even if stable front/back entries existed, if their declared type
        # is PointCloud2 the dual merger still cannot consume them.
        for t in "$PRE_BRIDGE_FRONT_TYPE" "$PRE_BRIDGE_BACK_TYPE"; do
            case "$t" in
                *PointCloud2*)
                    PRE_BLOCKED_REASONS+=(
                        "dual mode: bridge entry declares ${t} — merger subscribes to livox_ros_driver2/msg/CustomMsg"
                    )
                    ;;
            esac
        done
    fi

    # Legacy-only bridge: the current committed ros_gz_bridge.yaml exposes
    # only /<ns>/livox/lidar as sensor_msgs/msg/PointCloud2. That satisfies
    # neither T13 single-mode (which needs /<ns>/livox/lidar_front) nor the
    # dual merger (which needs CustomMsg). Surface this explicitly so the
    # operator sees the exact state and doesn't mistake a legacy bridge for
    # a usable one.
    if $PRE_BRIDGE_HAS_LEGACY_LIDAR \
       && ! $PRE_BRIDGE_HAS_STABLE_FRONT \
       && ! $PRE_BRIDGE_HAS_STABLE_BACK; then
        PRE_BLOCKED_REASONS+=(
            "legacy-only bridge: ros_gz_bridge.yaml exposes only /<ns>/livox/lidar (${PRE_BRIDGE_LEGACY_TYPE:-PointCloud2}) — cannot feed pointcloud_merger (needs CustomMsg front+back) and cannot satisfy T13 false-branch Point-LIO on /<ns>/livox/lidar_front"
        )
    fi
}

preflight_is_blocked() {
    [ ${#PRE_BLOCKED_REASONS[@]} -gt 0 ]
}

# =============================================================================
# Evidence writers
# =============================================================================

write_preflight_evidence() {
    local out="${OUTPUT_DIR}/task-15-preflight.md"
    mkdir -p "$OUTPUT_DIR"
    {
        echo "# Task 15 Preflight — test_sim_dual_mid360.sh"
        echo ""
        echo "Generated: $(date -u +"%Y-%m-%dT%H:%M:%SZ")"
        echo "Workspace: ${WS_ROOT}"
        echo "Mode requested: ${MODE}"
        echo "World: ${WORLD}"
        echo "Namespace: ${NAMESPACE}"
        echo "SLAM: ${SLAM}"
        echo "Headless: ${HEADLESS}"
        echo ""
        echo "## Environment"
        echo ""
        echo "| check | result |"
        echo "|---|---|"
        echo "| ros2 binary | ${PRE_ROS2_BIN:-MISSING} |"
        echo "| gz binary | ${PRE_GZ_BIN:-MISSING} |"
        echo ""
        echo "## ROS2 packages"
        echo ""
        echo "| package | status |"
        echo "|---|---|"
        for pkg in "${REQUIRED_PACKAGES[@]}"; do
            if printf '%s\n' "${PRE_MISSING_PKGS[@]:-}" | grep -qx "$pkg"; then
                echo "| \`${pkg}\` | MISSING |"
            else
                echo "| \`${pkg}\` | PRESENT |"
            fi
        done
        echo ""
        echo "## ros_gz_bridge.yaml"
        echo ""
        echo "File: \`${BRIDGE_YAML_REL}\`"
        echo ""
        echo "| stable ROS topic (post-\`<robot_name>\` substitution) | bridged | ros_type_name |"
        echo "|---|---|---|"
        echo "| \`/<ns>/livox/lidar_front\` | $($PRE_BRIDGE_HAS_STABLE_FRONT && echo YES || echo NO) | ${PRE_BRIDGE_FRONT_TYPE:-(none)} |"
        echo "| \`/<ns>/livox/lidar_back\`  | $($PRE_BRIDGE_HAS_STABLE_BACK  && echo YES || echo NO) | ${PRE_BRIDGE_BACK_TYPE:-(none)} |"
        echo "| \`/<ns>/livox/lidar\` (legacy) | $($PRE_BRIDGE_HAS_LEGACY_LIDAR && echo YES || echo NO) | ${PRE_BRIDGE_LEGACY_TYPE:-(none)} |"
        echo ""
        echo "| mismatch check | value |"
        echo "|---|---|"
        echo "| merger expects | \`livox_ros_driver2/msg/CustomMsg\` |"
        echo "| bridge declares (front/back) | ${PRE_BRIDGE_FRONT_TYPE:-none} / ${PRE_BRIDGE_BACK_TYPE:-none} |"
        echo "| PointCloud2 ↔ CustomMsg mismatch | $($PRE_MSG_TYPE_MISMATCH && echo YES || echo NO) |"
        echo ""
        echo "NOTE: T13 bound the single (use_dual_mid360:=False) fallback to"
        echo "Point-LIO \`common.lid_topic=livox/lidar_front\`, so single mode also"
        echo "depends on a stable \`/<ns>/livox/lidar_front\` bridge entry — the"
        echo "legacy \`/<ns>/livox/lidar\` alone is NOT enough. Dual mode additionally"
        echo "needs \`/<ns>/livox/lidar_back\` AND both entries must be"
        echo "\`livox_ros_driver2/msg/CustomMsg\` so the merger (CustomMsg in,"
        echo "CustomMsg out) can consume them. Updating ros_gz_bridge.yaml (and"
        echo "the simulator CustomMsg path) is OUT OF T15 SCOPE — it will be"
        echo "tracked under a future simulator/bridge task."
        echo ""
        echo "## Navigation launch"
        echo ""
        echo "| check | result |"
        echo "|---|---|"
        echo "| ${NAV_LAUNCH_FILE} exposes use_dual_mid360 | $($PRE_LAUNCH_HAS_USE_DUAL && echo YES || echo NO) |"
        echo ""
        echo "## Verdict"
        echo ""
        if preflight_is_blocked; then
            echo "**BLOCKED** — the following conditions prevent runtime smoke:"
            echo ""
            for r in "${PRE_BLOCKED_REASONS[@]}"; do
                echo "- ${r}"
            done
        else
            echo "**PASS** — all preflight conditions satisfied; runtime smoke can proceed."
        fi
        echo ""
        echo "## Intended target-hardware command sequence"
        echo ""
        echo '```bash'
        echo "# Terminal 1: Gazebo"
        echo "QT_QPA_PLATFORM=xcb ros2 launch ${GAZEBO_LAUNCH_PKG} ${GAZEBO_LAUNCH_FILE} headless:=${HEADLESS}"
        echo ""
        echo "# After robot spawn completes:"
        echo "gz service -s /world/default/level/set_performer \\"
        echo "  --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \\"
        echo "  --timeout 2000 --req 'data: \"${NAMESPACE}\"'"
        echo ""
        echo "gz service -s /world/default/control \\"
        echo "  --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \\"
        echo "  --timeout 5000 --req 'pause: false'"
        echo ""
        echo "sleep ${STABILISE_WAIT}   # let sim clock + sensors settle"
        echo ""
        echo "# Terminal 2: Navigation"
        echo "ros2 launch ${NAV_LAUNCH_PKG} ${NAV_LAUNCH_FILE} \\"
        echo "  world:=${WORLD} slam:=${SLAM} use_dual_mid360:=True"
        echo '```'
    } >"$out"
    log_info "Preflight evidence written: ${out}"
}

write_blocker_evidence() {
    local out="${OUTPUT_DIR}/task-15-blocker.md"
    mkdir -p "$OUTPUT_DIR"
    {
        echo "# Task 15 BLOCKED — runtime smoke cannot proceed"
        echo ""
        echo "Generated: $(date -u +"%Y-%m-%dT%H:%M:%SZ")"
        echo "Workspace: ${WS_ROOT}"
        echo "Mode requested: ${MODE}"
        echo ""
        echo "## Blockers"
        echo ""
        for r in "${PRE_BLOCKED_REASONS[@]}"; do
            echo "- ${r}"
        done
        echo ""
        echo "## Why this is BLOCKED and not FAIL"
        echo ""
        echo "A FAIL would mean the simulator ran but produced wrong data. A"
        echo "BLOCKED means the current environment / simulator configuration"
        echo "cannot support the requested mode at all — e.g. Gazebo is not"
        echo "installed, ros_gz_bridge.yaml does not expose stable"
        echo "\`/<ns>/livox/lidar_front\` / \`/<ns>/livox/lidar_back\` topics, or"
        echo "those entries declare \`sensor_msgs/msg/PointCloud2\` while the"
        echo "merger (and T13 false branch Point-LIO) need"
        echo "\`livox_ros_driver2/msg/CustomMsg\`. Inventing topic Hz / Nav Goal"
        echo "success in these conditions would be a fake PASS."
        echo ""
        echo "## Resolution"
        echo ""
        echo "- Missing ROS2 packages: run \`bash src/scripts/setup_env.sh\` or"
        echo "  \`colcon build --symlink-install\` in the worktree."
        echo "- Missing \`gz\` binary: install Gazebo Harmonic following the"
        echo "  project QUICKSTART (Ubuntu 24.04 + gz-sim 8)."
        echo "- Missing stable \`/<ns>/livox/lidar_front\` and/or"
        echo "  \`/<ns>/livox/lidar_back\` bridge entries: single mode (T13 false"
        echo "  branch) reads \`/<ns>/livox/lidar_front\`, and the dual merger reads"
        echo "  both. The bridge entries must also declare"
        echo "  \`livox_ros_driver2/msg/CustomMsg\`, not \`sensor_msgs/msg/PointCloud2\`,"
        echo "  because the merger subscribes to CustomMsg (per"
        echo "  \`sentry_dual_mid360/config/pointcloud_merger_params.yaml\`)."
        echo "- PointCloud2 ↔ CustomMsg mismatch: the current Gazebo bridge"
        echo "  (\`ros_gz_bridge.yaml\`) maps \`ignition.msgs.PointCloudPacked\` to"
        echo "  \`sensor_msgs/msg/PointCloud2\`. There is no direct"
        echo "  \`ignition.msgs.* ↔ livox_ros_driver2/msg/CustomMsg\` mapping in"
        echo "  ros_gz_bridge. A dedicated PointCloud2→CustomMsg conversion node"
        echo "  (or a simulator-side CustomMsg publisher) is required before the"
        echo "  merger can consume simulator LiDAR data."
        echo ""
        echo "Example BLOCKED additions to \`${BRIDGE_YAML_REL}\` (still not"
        echo "sufficient on their own — see PointCloud2↔CustomMsg note above):"
        echo ""
        echo '    ```yaml'
        echo "    - ros_topic_name: \"/<robot_name>/livox/lidar_front\""
        echo "      gz_topic_name: \"/world/default/model/<robot_name>/link/front_mid360/sensor/front_mid360_lidar/scan/points\""
        echo "      ros_type_name: \"sensor_msgs/msg/PointCloud2\""
        echo "      gz_type_name: \"ignition.msgs.PointCloudPacked\""
        echo "      direction: \"GZ_TO_ROS\""
        echo "    - ros_topic_name: \"/<robot_name>/livox/lidar_back\""
        echo "      gz_topic_name: \"/world/default/model/<robot_name>/link/back_mid360/sensor/back_mid360_lidar/scan/points\""
        echo "      ros_type_name: \"sensor_msgs/msg/PointCloud2\""
        echo "      gz_type_name: \"ignition.msgs.PointCloudPacked\""
        echo "      direction: \"GZ_TO_ROS\""
        echo '    ```'
        echo ""
        echo "Touching \`ros_gz_bridge.yaml\` (and supplying the CustomMsg"
        echo "conversion path) is OUT OF T15 SCOPE; it belongs to a future"
        echo "simulator/bridge task."
        echo ""
        echo "## Intended runtime command sequence"
        echo ""
        echo "See \`task-15-preflight.md\` → 'Intended target-hardware command sequence'."
    } >"$out"
    log_info "Blocker evidence written: ${out}"
}

# =============================================================================
# Dry-run
# =============================================================================

do_dry_run() {
    log_info "=== DRY RUN ==="

    mkdir -p "$OUTPUT_DIR"
    local out="${OUTPUT_DIR}/task-15-dry-run.md"
    {
        echo "# Task 15 Dry Run — test_sim_dual_mid360.sh"
        echo ""
        echo "Generated: $(date -u +"%Y-%m-%dT%H:%M:%SZ")"
        echo ""
        echo "## Effective configuration"
        echo ""
        echo "| key | value |"
        echo "|---|---|"
        echo "| mode | ${MODE} |"
        echo "| world | ${WORLD} |"
        echo "| namespace | ${NAMESPACE} |"
        echo "| slam | ${SLAM} |"
        echo "| headless | ${HEADLESS} |"
        echo "| duration | ${DURATION}s |"
        echo "| stabilise_wait | ${STABILISE_WAIT}s |"
        echo "| nav_warmup | ${NAV_WARMUP}s |"
        echo "| do_goal | ${DO_GOAL} |"
        echo "| goal | x=${GOAL_X} y=${GOAL_Y} frame=${GOAL_FRAME} |"
        echo "| output_dir | ${OUTPUT_DIR} |"
        echo ""
        echo "## Preflight snapshot"
        echo ""
        echo "| check | value |"
        echo "|---|---|"
        echo "| stable /<ns>/livox/lidar_front bridged | $($PRE_BRIDGE_HAS_STABLE_FRONT && echo YES || echo NO) (${PRE_BRIDGE_FRONT_TYPE:-none}) |"
        echo "| stable /<ns>/livox/lidar_back  bridged | $($PRE_BRIDGE_HAS_STABLE_BACK  && echo YES || echo NO) (${PRE_BRIDGE_BACK_TYPE:-none}) |"
        echo "| legacy /<ns>/livox/lidar bridged | $($PRE_BRIDGE_HAS_LEGACY_LIDAR && echo YES || echo NO) (${PRE_BRIDGE_LEGACY_TYPE:-none}) |"
        echo "| PointCloud2 ↔ CustomMsg mismatch | $($PRE_MSG_TYPE_MISMATCH && echo YES || echo NO) |"
        echo ""
        if preflight_is_blocked; then
            echo "**Preflight: BLOCKED**"
            echo ""
            for r in "${PRE_BLOCKED_REASONS[@]}"; do
                echo "- ${r}"
            done
        else
            echo "**Preflight: PASS**"
        fi
        echo ""
        echo "## Planned commands (per mode)"
        for m in $(mode_list); do
            local use_dual
            use_dual="$(mode_use_dual "$m")"
            echo ""
            echo "### Mode: ${m} (use_dual_mid360:=${use_dual})"
            echo ""
            echo '```bash'
            echo "# 1. Gazebo"
            echo "ros2 launch ${GAZEBO_LAUNCH_PKG} ${GAZEBO_LAUNCH_FILE} headless:=${HEADLESS}"
            echo ""
            echo "# 2. set_performer (after spawn)"
            echo "gz service -s /world/default/level/set_performer \\"
            echo "  --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \\"
            echo "  --timeout 2000 --req 'data: \"${NAMESPACE}\"'"
            echo ""
            echo "# 3. unpause"
            echo "gz service -s /world/default/control \\"
            echo "  --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \\"
            echo "  --timeout 5000 --req 'pause: false'"
            echo ""
            echo "# 4. sleep ${STABILISE_WAIT}"
            echo ""
            echo "# 5. Navigation"
            echo "ros2 launch ${NAV_LAUNCH_PKG} ${NAV_LAUNCH_FILE} \\"
            echo "  world:=${WORLD} slam:=${SLAM} \\"
            echo "  use_dual_mid360:=${use_dual} \\"
            echo "  namespace:=${NAMESPACE}"
            echo ""
            echo "# 6. sleep ${NAV_WARMUP}"
            echo ""
            echo "# 7. Topic checks"
            for t in $(mode_topic_list "$m"); do
                echo "timeout 5 ros2 topic hz /${NAMESPACE}/${t}"
            done
            echo ""
            echo "# 8. TF checks"
            for pair in $(mode_tf_list "$m"); do
                local parent="${pair%:*}" child="${pair#*:}"
                echo "timeout 5 ros2 run tf2_ros tf2_echo ${parent} ${child}"
            done
            if $DO_GOAL; then
                echo ""
                echo "# 9. NavigateToPose goal"
                echo "ros2 action send_goal /${NAMESPACE}/navigate_to_pose \\"
                echo "  nav2_msgs/action/NavigateToPose \\"
                echo "  '{pose: {header: {frame_id: ${GOAL_FRAME}}, pose: {position: {x: ${GOAL_X}, y: ${GOAL_Y}}, orientation: {w: 1.0}}}}' \\"
                echo "  --feedback"
            fi
            echo '```'
        done
        echo ""
        echo "## Notes"
        echo ""
        echo "- This dry-run created no processes and no temporary files."
        echo "- Run without --dry-run to actually execute the smoke."
        echo "- Run --preflight for just the environment matrix."
    } >"$out"
    log_info "Dry-run plan written: ${out}"
}

# =============================================================================
# Mode helpers
# =============================================================================

mode_list() {
    if [ "$MODE" = "both" ]; then
        echo "dual single"
    else
        echo "$MODE"
    fi
}

mode_use_dual() {
    case "$1" in
        dual)   echo "True" ;;
        single) echo "False" ;;
        *)      echo "True" ;;
    esac
}

mode_topic_list() {
    case "$1" in
        dual)   printf '%s\n' "${DUAL_TOPICS[@]}" ;;
        single) printf '%s\n' "${SINGLE_TOPICS[@]}" ;;
        *)      printf '%s\n' "${SINGLE_TOPICS[@]}" ;;
    esac
}

mode_tf_list() {
    # Output one "parent:child" per line
    local pair
    for pair in "${TF_PAIRS[@]}"; do
        echo "$pair"
    done
    if [ "$1" = "dual" ]; then
        for pair in "${TF_PAIRS_DUAL_EXTRA[@]}"; do
            echo "$pair"
        done
    fi
}

# =============================================================================
# Runtime orchestration
# =============================================================================

# Child PIDs we need to clean up on EXIT.
GAZEBO_PID=""
NAV_PID=""

cleanup_children() {
    local rc=$?
    # shellcheck disable=SC2317  # trap invocation
    log_info "=== cleanup ==="
    if [ -n "$NAV_PID" ] && kill -0 "$NAV_PID" 2>/dev/null; then
        log_info "killing nav launch parent PID ${NAV_PID}"
        kill -INT "$NAV_PID" 2>/dev/null || true
        sleep 2
        kill -9 "$NAV_PID" 2>/dev/null || true
    fi
    if [ -n "$GAZEBO_PID" ] && kill -0 "$GAZEBO_PID" 2>/dev/null; then
        log_info "killing gazebo launch parent PID ${GAZEBO_PID}"
        kill -INT "$GAZEBO_PID" 2>/dev/null || true
        sleep 2
        kill -9 "$GAZEBO_PID" 2>/dev/null || true
    fi
    # Best-effort sweep of stragglers (only named patterns we expect).
    pkill -9 -f '[r]m_navigation_simulation_launch.py'  2>/dev/null || true
    pkill -9 -f '[b]ringup_sim.launch.py'               2>/dev/null || true
    pkill -9 -f '[p]ointcloud_merger'                   2>/dev/null || true
    pkill -9 -f '[m]erger_node'                         2>/dev/null || true
    pkill -9 -f '[g]z sim'                              2>/dev/null || true
    sleep 1
    exit "$rc"
}

start_gazebo_mode() {
    local mode_name="$1"
    local log_file="${OUTPUT_DIR}/task-15-${mode_name}-gazebo.log"
    log_info "launching Gazebo (mode=${mode_name}, headless=${HEADLESS})"
    # shellcheck disable=SC2086
    QT_QPA_PLATFORM=xcb "$PRE_ROS2_BIN" launch "$GAZEBO_LAUNCH_PKG" "$GAZEBO_LAUNCH_FILE" \
        "headless:=${HEADLESS}" \
        >"$log_file" 2>&1 &
    GAZEBO_PID=$!
    log_info "Gazebo launch PID: ${GAZEBO_PID} (log: ${log_file})"
}

gz_unpause_and_set_performer() {
    log_info "waiting for Gazebo to expose /world/default/control ..."
    local deadline=$((SECONDS + 60))
    while [ "$SECONDS" -lt "$deadline" ]; do
        if "$PRE_GZ_BIN" service -l 2>/dev/null | grep -q '^/world/default/control$'; then
            break
        fi
        sleep 2
    done
    if [ "$SECONDS" -ge "$deadline" ]; then
        log_fail "timed out waiting for Gazebo control service"
        return 1
    fi
    log_info "calling set_performer for '${NAMESPACE}'"
    "$PRE_GZ_BIN" service -s /world/default/level/set_performer \
        --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
        --timeout 2000 \
        --req "data: \"${NAMESPACE}\"" || log_warn "set_performer call returned non-zero"

    log_info "unpausing Gazebo"
    "$PRE_GZ_BIN" service -s /world/default/control \
        --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
        --timeout 5000 \
        --req "pause: false" || log_warn "unpause returned non-zero"

    log_info "sleeping ${STABILISE_WAIT}s for sim clock to settle"
    sleep "$STABILISE_WAIT"
}

start_nav_mode() {
    local mode_name="$1"
    local use_dual
    use_dual="$(mode_use_dual "$mode_name")"
    local log_file="${OUTPUT_DIR}/task-15-${mode_name}-run.log"
    log_info "launching Nav2 (mode=${mode_name}, use_dual_mid360=${use_dual})"
    # shellcheck disable=SC2086
    "$PRE_ROS2_BIN" launch "$NAV_LAUNCH_PKG" "$NAV_LAUNCH_FILE" \
        "world:=${WORLD}" \
        "slam:=${SLAM}" \
        "use_dual_mid360:=${use_dual}" \
        "namespace:=${NAMESPACE}" \
        "use_rviz:=False" \
        >"$log_file" 2>&1 &
    NAV_PID=$!
    log_info "Nav2 launch PID: ${NAV_PID} (log: ${log_file})"
    log_info "sleeping ${NAV_WARMUP}s for Point-LIO + Nav2 warmup"
    sleep "$NAV_WARMUP"
}

check_topics_mode() {
    local mode_name="$1"
    local out="${OUTPUT_DIR}/task-15-topics-${mode_name}.txt"
    log_info "checking topics for mode=${mode_name}"
    : >"$out"
    local all_ok=true
    local topic
    while read -r topic; do
        [ -z "$topic" ] && continue
        local full="/${NAMESPACE}/${topic}"
        {
            echo "=== ros2 topic hz ${full} ==="
            timeout 5 "$PRE_ROS2_BIN" topic hz "$full" 2>&1 || echo "(exit $?)"
            echo ""
        } >>"$out"
        # Verdict line
        if timeout 3 "$PRE_ROS2_BIN" topic info "$full" 2>&1 | grep -q "Subscription count: [1-9]\|Publisher count: [1-9]"; then
            log_ok "topic present: ${full}"
        else
            log_fail "topic missing: ${full}"
            all_ok=false
        fi
    done < <(mode_topic_list "$mode_name")
    $all_ok
}

check_tf_mode() {
    local mode_name="$1"
    local out="${OUTPUT_DIR}/task-15-tf-${mode_name}.txt"
    log_info "checking TF for mode=${mode_name}"
    : >"$out"
    local all_ok=true
    local pair
    while read -r pair; do
        [ -z "$pair" ] && continue
        local parent="${pair%:*}" child="${pair#*:}"
        {
            echo "=== tf2_echo ${parent} -> ${child} ==="
            timeout 5 "$PRE_ROS2_BIN" run tf2_ros tf2_echo "$parent" "$child" 2>&1 | head -30 || echo "(exit $?)"
            echo ""
        } >>"$out"
        if timeout 5 "$PRE_ROS2_BIN" run tf2_ros tf2_echo "$parent" "$child" 2>&1 | head -30 | grep -q "Translation:"; then
            log_ok "TF ok: ${parent} -> ${child}"
        else
            log_fail "TF missing: ${parent} -> ${child}"
            all_ok=false
        fi
    done < <(mode_tf_list "$mode_name")
    $all_ok
}

send_goal_mode() {
    local mode_name="$1"
    local out="${OUTPUT_DIR}/task-15-goal-${mode_name}.log"
    log_info "sending NavigateToPose goal (x=${GOAL_X}, y=${GOAL_Y})"
    local goal_yaml
    printf -v goal_yaml '{pose: {header: {frame_id: %s}, pose: {position: {x: %s, y: %s}, orientation: {w: 1.0}}}}' \
        "$GOAL_FRAME" "$GOAL_X" "$GOAL_Y"
    timeout "$GOAL_WAIT" "$PRE_ROS2_BIN" action send_goal \
        "/${NAMESPACE}/navigate_to_pose" \
        nav2_msgs/action/NavigateToPose \
        "$goal_yaml" \
        --feedback >"$out" 2>&1 || log_warn "send_goal returned non-zero (may be timeout / rejected)"
    if grep -q "Goal accepted" "$out"; then
        log_ok "NavigateToPose accepted"
        return 0
    fi
    log_fail "NavigateToPose never reported Goal accepted"
    return 1
}

run_one_mode() {
    local mode_name="$1"
    local summary_file="${OUTPUT_DIR}/task-15-runtime.md"
    log_info "############# running mode: ${mode_name} #############"

    start_gazebo_mode "$mode_name"
    if ! gz_unpause_and_set_performer; then
        log_fail "gz control setup failed; aborting mode=${mode_name}"
        return 1
    fi
    start_nav_mode "$mode_name"

    local topics_ok=true tf_ok=true goal_ok=true
    check_topics_mode "$mode_name" || topics_ok=false
    check_tf_mode     "$mode_name" || tf_ok=false
    if $DO_GOAL; then
        send_goal_mode "$mode_name" || goal_ok=false
    fi

    # Keep chain alive for a final settle window before cleanup (optional).
    if [ "$DURATION" -gt 0 ]; then
        log_info "holding chain for ${DURATION}s before teardown"
        sleep "$DURATION"
    fi

    {
        echo ""
        echo "## Mode: ${mode_name}"
        echo ""
        echo "- topics_ok: ${topics_ok}"
        echo "- tf_ok: ${tf_ok}"
        echo "- goal_ok: ${goal_ok}"
        echo "- gazebo_log: task-15-${mode_name}-gazebo.log"
        echo "- nav_log: task-15-${mode_name}-run.log"
        echo "- topics_evidence: task-15-topics-${mode_name}.txt"
        echo "- tf_evidence: task-15-tf-${mode_name}.txt"
        $DO_GOAL && echo "- goal_evidence: task-15-goal-${mode_name}.log"
    } >>"$summary_file"

    # Teardown this mode before starting the next. Reset PIDs so the outer
    # trap doesn't double-kill.
    log_info "tearing down mode=${mode_name}"
    if [ -n "$NAV_PID" ] && kill -0 "$NAV_PID" 2>/dev/null; then
        kill -INT "$NAV_PID" 2>/dev/null || true
        sleep 3
        kill -9 "$NAV_PID" 2>/dev/null || true
    fi
    NAV_PID=""
    if [ -n "$GAZEBO_PID" ] && kill -0 "$GAZEBO_PID" 2>/dev/null; then
        kill -INT "$GAZEBO_PID" 2>/dev/null || true
        sleep 3
        kill -9 "$GAZEBO_PID" 2>/dev/null || true
    fi
    GAZEBO_PID=""
    pkill -9 -f '[r]m_navigation_simulation_launch.py'  2>/dev/null || true
    pkill -9 -f '[b]ringup_sim.launch.py'               2>/dev/null || true
    pkill -9 -f '[p]ointcloud_merger'                   2>/dev/null || true
    pkill -9 -f '[m]erger_node'                         2>/dev/null || true
    pkill -9 -f '[g]z sim'                              2>/dev/null || true
    sleep 2

    if $topics_ok && $tf_ok && $goal_ok; then
        return 0
    fi
    return 1
}

run_runtime_smoke() {
    # Install cleanup trap.
    trap cleanup_children EXIT INT TERM

    mkdir -p "$OUTPUT_DIR"
    local summary_file="${OUTPUT_DIR}/task-15-runtime.md"
    {
        echo "# Task 15 Runtime — test_sim_dual_mid360.sh"
        echo ""
        echo "Generated: $(date -u +"%Y-%m-%dT%H:%M:%SZ")"
        echo "Workspace: ${WS_ROOT}"
        echo "World: ${WORLD}"
        echo "Namespace: ${NAMESPACE}"
        echo "SLAM: ${SLAM}"
        echo "Headless: ${HEADLESS}"
    } >"$summary_file"

    local overall_ok=true
    for m in $(mode_list); do
        if ! run_one_mode "$m"; then
            overall_ok=false
        fi
    done

    {
        echo ""
        echo "## Overall"
        echo ""
        if $overall_ok; then
            echo "**PASS** — all requested modes completed with topics/TF/goal OK."
        else
            echo "**FAIL** — at least one mode failed its topic/TF/goal checks."
        fi
    } >>"$summary_file"

    $overall_ok
}

# =============================================================================
# Main
# =============================================================================

main() {
    parse_args "$@"
    validate_args

    mkdir -p "$OUTPUT_DIR"
    log_info "test_sim_dual_mid360.sh v${SCRIPT_VERSION}"
    log_info "workspace: ${WS_ROOT}"
    log_info "mode=${MODE} world=${WORLD} ns=${NAMESPACE} slam=${SLAM} headless=${HEADLESS}"

    # Preflight runs unconditionally (cheap + drives dry-run / blocker paths).
    run_preflight
    write_preflight_evidence

    if $PREFLIGHT_ONLY; then
        if preflight_is_blocked; then
            write_blocker_evidence
            log_fail "preflight BLOCKED"
            exit 2
        fi
        log_ok "preflight PASS (no runtime executed)"
        exit 0
    fi

    if $DRY_RUN; then
        do_dry_run
        if preflight_is_blocked; then
            write_blocker_evidence
            log_warn "dry-run complete but preflight is BLOCKED; runtime would be skipped"
            exit 2
        fi
        log_ok "dry-run complete"
        exit 0
    fi

    if preflight_is_blocked; then
        write_blocker_evidence
        log_fail "preflight BLOCKED — refusing to launch Gazebo/Nav2"
        log_fail "see ${OUTPUT_DIR}/task-15-blocker.md for exact resolution steps"
        exit 2
    fi

    if run_runtime_smoke; then
        log_ok "runtime smoke PASSED"
        exit 0
    fi
    log_fail "runtime smoke FAILED"
    exit 1
}

main "$@"
