#!/bin/bash
# record_calib_bag.sh
# Records a static calibration rosbag2 for dual Mid360 extrinsic calibration.
#
# Captures raw LiDAR point clouds and TF data needed by Multi_LiCa (T11).
# NOTE: Multi_LiCa does NOT consume Livox CustomMsg directly; T11 must convert
#       /livox/lidar_front and /livox/lidar_back (CustomMsg) to PointCloud2/PCD
#       before feeding them to the calibration tool.
#
# Recorded topics:
#   REQUIRED (preflight aborts if missing):
#     /livox/lidar_front   — front Mid360 raw CustomMsg (IP: 192.168.1.144)
#     /livox/lidar_back    — back  Mid360 raw CustomMsg (IP: 192.168.1.145)
#   OPTIONAL (recorded when present, skipped with a warning when not):
#     /tf                  — dynamic transforms
#     /tf_static           — static transforms
#
# Multi_LiCa (T11) does not need /tf[_static] — external calibration reads
# the initial guess from xmacro via read_tf_from_table. tf topics are kept
# as optional because a standalone driver launch (dual_mid360_driver_launch.py)
# does not start robot_state_publisher and therefore does not publish them,
# but the rest of the stack — including full bringup — does.
#
# Usage:
#   bash record_calib_bag.sh [OPTIONS]
#
# Options:
#   --help                  Show this help message and exit
#   --env sim|real          Environment: sim (Gazebo) or real (hardware) [required]
#   --duration SECONDS      Recording duration in seconds [default: 60]
#   --output-dir DIR        Parent directory for bag output [default: logs/evidence/calib-bags]
#   --dry-run               Print planned command; create no bag
#
# Hardware assumptions (real mode):
#   Front Mid360: 192.168.1.144
#   Back  Mid360: 192.168.1.145
#
# Sim mode assumptions:
#   Gazebo Harmonic must already be running and unpaused.
#   Navigation stack (or at least livox_ros_driver2 + robot_state_publisher) must be up.
#   Do NOT launch Gazebo from within this script.

set -euo pipefail

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# REQUIRED_TOPICS must all be published; missing any aborts preflight.
REQUIRED_TOPICS=(
    "/livox/lidar_front"
    "/livox/lidar_back"
)
# OPTIONAL_TOPICS are recorded when present; missing ones emit a WARN but
# do not abort. Multi_LiCa reads the initial guess from xmacro directly,
# so tf is genuinely optional for T11.
OPTIONAL_TOPICS=(
    "/tf"
    "/tf_static"
)
# TOPICS_TO_RECORD is populated by check_topics() with only the required +
# available-optional subset. Used for both `ros2 bag record` and metadata.
TOPICS_TO_RECORD=()
FRONT_IP="192.168.1.144"
BACK_IP="192.168.1.145"

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
ENV=""
DURATION=60
OUTPUT_DIR=""
DRY_RUN=false

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
log_info()  { echo "[INFO]  $*"; }
log_warn()  { echo "[WARN]  $*" >&2; }
log_error() { echo "[ERROR] $*" >&2; }

print_usage() {
    cat <<EOF
record_calib_bag.sh — Static calibration rosbag2 recorder for dual Mid360

USAGE:
  bash record_calib_bag.sh --env <sim|real> [OPTIONS]

OPTIONS:
  --help                  Show this help message and exit
  --env sim|real          Environment: sim (Gazebo already running) or real (hardware)
  --duration SECONDS      Recording duration in seconds (default: 60)
  --output-dir DIR        Parent directory for bag output
                          (default: <workspace>/logs/evidence/calib-bags)
  --dry-run               Print planned command and metadata; create no bag

RECORDED TOPICS:
  REQUIRED (preflight aborts if missing):
    /livox/lidar_front    Front Mid360 raw CustomMsg  (real IP: $FRONT_IP)
    /livox/lidar_back     Back  Mid360 raw CustomMsg  (real IP: $BACK_IP)
  OPTIONAL (recorded when present, skipped with WARN otherwise):
    /tf                   Dynamic transforms
    /tf_static            Static transforms

  Why optional: T11 calibration reads the initial guess from xmacro via
  read_tf_from_table and does NOT consume bag tf. A standalone driver
  launch does not publish /tf[_static]; full bringup does.

OUTPUT LAYOUT:
  <output-dir>/<env>-<timestamp>/
    <env>-<timestamp>.bag/   — rosbag2 directory (mcap or sqlite3)
    metadata.yaml            — environment, topics, duration, IPs, command used

OPERATOR INSTRUCTIONS (real mode):
  1. Place robot on flat, open ground with clear 360-degree LiDAR view.
  2. Robot must be COMPLETELY STATIONARY during the entire recording.
  3. Ensure both Mid360 units are powered and publishing before running.
  4. Verify topics are live: ros2 topic hz /livox/lidar_front /livox/lidar_back
  5. Run this script; do NOT move the robot until recording completes.

OPERATOR INSTRUCTIONS (sim mode):
  1. Start Gazebo: ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
  2. Unpause and wait ~10s for sensor data to stabilize.
  3. Start navigation stack (or at minimum livox_ros_driver2 + robot_state_publisher).
  4. Run this script with --env sim.

NOTES:
  - Multi_LiCa (T11) requires PointCloud2 or PCD input, NOT Livox CustomMsg.
    T11 must convert /livox/lidar_front and /livox/lidar_back before calibration.
  - Do NOT record Point-LIO outputs (/cloud_registered, /aft_mapped_to_init, etc.).
  - Do NOT move the robot or command the chassis during recording.
  - Rosbag2 .db3/.mcap files are large; do NOT commit them to git.

EOF
}

# Resolve workspace root (5 levels up):
#   scripts/calib/ -> scripts/ -> pkg/ -> sentry_nav/ -> src/ -> ws/
resolve_workspace_root() {
    local candidate
    candidate="$(cd "$SCRIPT_DIR/../../../../.." && pwd)"
    echo "$candidate"
}

find_ros2() {
    if command -v ros2 &>/dev/null; then
        echo "ros2"
        return 0
    fi
    # Try common install paths
    for candidate in /opt/ros/jazzy/bin/ros2 /opt/ros/humble/bin/ros2; do
        if [ -x "$candidate" ]; then
            echo "$candidate"
            return 0
        fi
    done
    return 1
}

check_ros2() {
    local ros2_bin
    if ! ros2_bin="$(find_ros2)"; then
        log_error "ros2 not found. Source your ROS2 setup first:"
        log_error "  source /opt/ros/jazzy/setup.bash"
        return 1
    fi
    echo "$ros2_bin"
}

check_topics() {
    local ros2_bin="$1"
    local missing_required=()
    local missing_optional=()
    local topic_list

    # Reset the global pick list so repeated calls / re-runs start clean.
    TOPICS_TO_RECORD=()

    log_info "Checking topic availability..."
    if ! topic_list="$("$ros2_bin" topic list 2>/dev/null)"; then
        log_error "Failed to query ROS2 topic list. Is a ROS2 graph running?"
        return 1
    fi

    for topic in "${REQUIRED_TOPICS[@]}"; do
        if echo "$topic_list" | grep -qF "$topic"; then
            log_info "  FOUND (required): $topic"
            TOPICS_TO_RECORD+=("$topic")
        else
            log_warn "  MISSING (required): $topic"
            missing_required+=("$topic")
        fi
    done
    for topic in "${OPTIONAL_TOPICS[@]}"; do
        if echo "$topic_list" | grep -qF "$topic"; then
            log_info "  FOUND (optional): $topic"
            TOPICS_TO_RECORD+=("$topic")
        else
            # Optional-missing is expected for the standalone driver launch
            # (no robot_state_publisher → no tf). Record without it.
            log_warn "  SKIP  (optional, not published): $topic"
            missing_optional+=("$topic")
        fi
    done

    if [ ${#missing_required[@]} -gt 0 ]; then
        log_error "The following REQUIRED topics are not published:"
        for t in "${missing_required[@]}"; do
            log_error "  $t"
        done
        log_error ""
        if [ "$ENV" = "real" ]; then
            log_error "Real mode: ensure both Mid360 units are powered and livox_ros_driver2 is running."
            log_error "  Front Mid360 IP: $FRONT_IP"
            log_error "  Back  Mid360 IP: $BACK_IP"
            log_error "  Quickest bring-up: ros2 launch sentry_dual_mid360 dual_mid360_driver_launch.py"
        else
            log_error "Sim mode: ensure Gazebo is running, unpaused, and the navigation stack is up."
            log_error "  ros2 launch rmu_gazebo_simulator bringup_sim.launch.py"
            log_error "  (then unpause and wait ~10s before running this script)"
        fi
        return 1
    fi

    if [ ${#missing_optional[@]} -gt 0 ]; then
        log_info "Proceeding without optional topics (not required by T11 calibration):"
        for t in "${missing_optional[@]}"; do
            log_info "  - $t"
        done
    fi
    log_info "All required topics are available."
    return 0
}

write_metadata() {
    local meta_file="$1"
    local bag_dir="$2"
    local cmd_used="$3"
    local status="$4"  # "planned" or "recorded" or "failed"

    cat >"$meta_file" <<EOF
# Dual Mid360 Calibration Bag Metadata
# Generated by record_calib_bag.sh

env: $ENV
duration_seconds: $DURATION
status: $status
timestamp: $(date -u +"%Y-%m-%dT%H:%M:%SZ")
bag_directory: $bag_dir

topics_required:
$(for t in "${REQUIRED_TOPICS[@]}"; do echo "  - $t"; done)

topics_optional:
$(for t in "${OPTIONAL_TOPICS[@]}"; do echo "  - $t"; done)

topics_recorded:
$(if [ "${#TOPICS_TO_RECORD[@]}" -gt 0 ]; then
    for t in "${TOPICS_TO_RECORD[@]}"; do echo "  - $t"; done
else
    echo "  []  # preflight aborted before topic selection"
fi)

hardware_assumptions:
  front_mid360_ip: $FRONT_IP
  back_mid360_ip: $BACK_IP

notes:
  - Multi_LiCa (T11) requires PointCloud2/PCD, NOT Livox CustomMsg.
  - T11 must convert /livox/lidar_front and /livox/lidar_back before calibration.
  - Robot must be stationary during recording.
  - Do NOT commit .db3/.mcap bag files to git.

command_used: |
  $cmd_used

operator_instructions:
$(if [ "$ENV" = "real" ]; then
cat <<'REAL'
  - Place robot on flat, open ground with clear 360-degree LiDAR view.
  - Robot must be COMPLETELY STATIONARY during the entire recording.
  - Ensure both Mid360 units are powered and publishing before running.
  - Verify: ros2 topic hz /livox/lidar_front /livox/lidar_back
REAL
else
cat <<'SIM'
  - Gazebo must already be running and unpaused before this script is called.
  - Navigation stack (or livox_ros_driver2 + robot_state_publisher) must be up.
  - Wait ~10s after unpause for sensor data to stabilize.
SIM
fi)
EOF
}

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
parse_args() {
    while [ $# -gt 0 ]; do
        case "$1" in
            --help|-h)
                print_usage
                exit 0
                ;;
            --env)
                [ $# -ge 2 ] || { log_error "--env requires an argument (sim|real)"; exit 1; }
                ENV="$2"
                shift 2
                ;;
            --duration)
                [ $# -ge 2 ] || { log_error "--duration requires an argument"; exit 1; }
                DURATION="$2"
                shift 2
                ;;
            --output-dir)
                [ $# -ge 2 ] || { log_error "--output-dir requires an argument"; exit 1; }
                OUTPUT_DIR="$2"
                shift 2
                ;;
            --dry-run)
                DRY_RUN=true
                shift
                ;;
            *)
                log_error "Unknown option: $1"
                echo ""
                print_usage
                exit 1
                ;;
        esac
    done
}

validate_args() {
    if [ -z "$ENV" ]; then
        log_error "--env is required. Use --env sim or --env real."
        echo ""
        print_usage
        exit 1
    fi
    if [ "$ENV" != "sim" ] && [ "$ENV" != "real" ]; then
        log_error "--env must be 'sim' or 'real', got: '$ENV'"
        exit 1
    fi
    if ! [[ "$DURATION" =~ ^[0-9]+$ ]] || [ "$DURATION" -lt 1 ]; then
        log_error "--duration must be a positive integer, got: '$DURATION'"
        exit 1
    fi
    if [ -z "$OUTPUT_DIR" ]; then
        local ws_root
        ws_root="$(resolve_workspace_root)"
        OUTPUT_DIR="$ws_root/logs/evidence/calib-bags"
    fi
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
main() {
    parse_args "$@"
    validate_args

    local timestamp
    timestamp="$(date +"%Y%m%d_%H%M%S")"
    local run_name="${ENV}-${timestamp}"
    local run_dir="${OUTPUT_DIR}/${run_name}"
    local bag_dir="${run_dir}/${run_name}.bag"
    local meta_file="${run_dir}/metadata.yaml"

    # Build the ros2 bag record command
    local ros2_bin
    local record_cmd_display

    # Jazzy's `ros2 bag record` dropped the `--duration N` flag that Humble had
    # (only `-d/--max-bag-duration` for file splitting remains). Run recording
    # in the background and send SIGINT after $DURATION from a watcher, so
    # rosbag2 flushes metadata cleanly. `timeout` is unreliable here because
    # signals sent to the ros2cli entry-point do not always reach the rosbag2
    # writer thread in time.
    # Positional topic arguments are deprecated on Jazzy; use `--topics` form.
    # The final topic list is only known after check_topics runs; build the
    # display string lazily to keep it in sync with what actually gets recorded.

    # ---------------------------------------------------------------------------
    # Dry-run path
    # ---------------------------------------------------------------------------
    if $DRY_RUN; then
        log_info "=== DRY-RUN MODE ==="
        log_info "Environment  : $ENV"
        log_info "Duration     : ${DURATION}s"
        log_info "Output dir   : $run_dir"
        log_info "Bag dir      : $bag_dir"
        log_info "Metadata     : $meta_file"
        log_info ""
        log_info "Would preflight these topics:"
        for t in "${REQUIRED_TOPICS[@]}"; do
            log_info "  required: $t"
        done
        for t in "${OPTIONAL_TOPICS[@]}"; do
            log_info "  optional: $t"
        done
        # Represent the eventual record command with the union of required
        # and optional — actual selection happens in a live preflight.
        local preview_topics="${REQUIRED_TOPICS[*]} ${OPTIONAL_TOPICS[*]}"
        record_cmd_display="ros2 bag record -o ${bag_dir} --topics ${preview_topics}  # stopped via SIGINT after ${DURATION}s"
        log_info ""
        log_info "Planned command (best-case, assumes all optional topics are live):"
        log_info "  $record_cmd_display"
        log_info ""
        log_info "[DRY-RUN] No bag will be created. Exiting."

        # Create output dir and write metadata even in dry-run (for QA)
        mkdir -p "$run_dir"
        write_metadata "$meta_file" "$bag_dir" "$record_cmd_display" "planned-dry-run"
        log_info "Metadata written to: $meta_file"
        exit 0
    fi

    # ---------------------------------------------------------------------------
    # Live recording path
    # ---------------------------------------------------------------------------
    log_info "=== Dual Mid360 Calibration Bag Recorder ==="
    log_info "Environment  : $ENV"
    log_info "Duration     : ${DURATION}s"
    log_info "Output dir   : $run_dir"

    # Check ros2 availability
    if ! ros2_bin="$(check_ros2)"; then
        exit 1
    fi
    log_info "ros2 binary  : $ros2_bin"

    # Preflight: check topics (populates TOPICS_TO_RECORD).
    if ! check_topics "$ros2_bin"; then
        # Write metadata with failed status before exiting. No topics will
        # have been picked yet, so record_cmd_display is just an intent.
        mkdir -p "$run_dir"
        record_cmd_display="ros2 bag record -o ${bag_dir} --topics <preflight aborted>"
        write_metadata "$meta_file" "$bag_dir" "$record_cmd_display" "preflight-failed-missing-topics"
        log_error ""
        log_error "Preflight failed. Metadata written to: $meta_file"
        log_error "Fix the missing topics and re-run."
        exit 1
    fi

    # Now TOPICS_TO_RECORD is authoritative; freeze the display command.
    local topics_str
    topics_str="${TOPICS_TO_RECORD[*]}"
    record_cmd_display="ros2 bag record -o ${bag_dir} --topics ${topics_str}  # stopped via SIGINT after ${DURATION}s"

    # Create output directory
    mkdir -p "$run_dir"

    # Write metadata (pre-record)
    write_metadata "$meta_file" "$bag_dir" "$record_cmd_display" "recording"
    log_info "Metadata written to: $meta_file"

    # Operator reminder
    if [ "$ENV" = "real" ]; then
        log_warn "============================================================"
        log_warn "REAL MODE: Ensure robot is STATIONARY on flat, open ground."
        log_warn "Do NOT move the robot during recording."
        log_warn "Front Mid360 IP: $FRONT_IP"
        log_warn "Back  Mid360 IP: $BACK_IP"
        log_warn "============================================================"
        log_info "Starting in 3 seconds... (Ctrl-C to abort)"
        sleep 3
    fi

    log_info "Starting rosbag2 recording..."
    log_info "Command: $record_cmd_display"
    log_info "Recording for ${DURATION}s. Do NOT move the robot."

    # Run `ros2 bag record` in the background so we can deliver SIGINT to the
    # exact process group after $DURATION seconds. `setsid` puts it in its own
    # session, letting a single `kill -INT -<pgid>` reach both ros2cli and the
    # rosbag2 writer. Ctrl-C in the user terminal is trapped and forwarded so
    # partial recordings still flush metadata on manual abort.
    local record_exit=0
    local record_pid
    local record_pgid

    setsid "$ros2_bin" bag record \
        -o "$bag_dir" \
        --topics "${TOPICS_TO_RECORD[@]}" &
    record_pid=$!
    record_pgid="$record_pid"  # setsid makes pgid == pid of the new leader

    trap 'log_warn "Received interrupt, forwarding SIGINT to rosbag2..."; kill -INT "-${record_pgid}" 2>/dev/null || true' INT TERM

    # Wait up to $DURATION seconds, then send SIGINT for clean shutdown.
    local elapsed=0
    while [ $elapsed -lt "$DURATION" ]; do
        if ! kill -0 "$record_pid" 2>/dev/null; then
            log_warn "ros2 bag record exited early (after ${elapsed}s)."
            break
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done

    if kill -0 "$record_pid" 2>/dev/null; then
        log_info "Duration reached (${DURATION}s), stopping rosbag2..."
        kill -INT "-${record_pgid}" 2>/dev/null || true
    fi

    # Give rosbag2 up to 10s to flush metadata and exit; then SIGTERM as fallback.
    local wait_for_exit=0
    while [ $wait_for_exit -lt 10 ] && kill -0 "$record_pid" 2>/dev/null; do
        sleep 1
        wait_for_exit=$((wait_for_exit + 1))
    done
    if kill -0 "$record_pid" 2>/dev/null; then
        log_warn "rosbag2 did not exit within 10s of SIGINT, sending SIGTERM."
        kill -TERM "-${record_pgid}" 2>/dev/null || true
        sleep 2
    fi

    wait "$record_pid" 2>/dev/null || record_exit=$?
    trap - INT TERM

    # SIGINT gives 130 when bash-reported; treat as a clean stop.
    if [ $record_exit -eq 130 ]; then
        record_exit=0
    fi

    if [ $record_exit -eq 0 ]; then
        log_info "Recording complete."
        write_metadata "$meta_file" "$bag_dir" "$record_cmd_display" "recorded-ok"
        log_info ""
        log_info "Bag saved to  : $bag_dir"
        log_info "Metadata      : $meta_file"
        log_info ""
        log_info "Next steps (T11):"
        log_info "  1. Convert CustomMsg to PointCloud2/PCD:"
        log_info "     ros2 bag play $bag_dir  # replay while running converter node"
        log_info "  2. Run Multi_LiCa calibration:"
        log_info "     bash $(dirname "$0")/calibrate_dual_mid360.sh --check-deps"
        log_info "     ros2 launch multi_lidar_calibrator calibration.launch.py \\"
        log_info "       parameter_file:=<path>/params.yaml"
    else
        log_error "ros2 bag record exited with code $record_exit."
        write_metadata "$meta_file" "$bag_dir" "$record_cmd_display" "record-failed-exit-${record_exit}"
        log_error "Metadata updated: $meta_file"
        exit $record_exit
    fi
}

main "$@"
