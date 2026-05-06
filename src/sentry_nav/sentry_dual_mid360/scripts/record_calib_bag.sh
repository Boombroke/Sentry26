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
#   /livox/lidar_front   — front Mid360 raw CustomMsg (IP: 192.168.1.144)
#   /livox/lidar_back    — back  Mid360 raw CustomMsg (IP: 192.168.1.145)
#   /tf                  — dynamic transforms
#   /tf_static           — static transforms
#
# Usage:
#   bash record_calib_bag.sh [OPTIONS]
#
# Options:
#   --help                  Show this help message and exit
#   --env sim|real          Environment: sim (Gazebo) or real (hardware) [required]
#   --duration SECONDS      Recording duration in seconds [default: 60]
#   --output-dir DIR        Parent directory for bag output [default: .sisyphus/evidence/calib-bags]
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
RECORD_TOPICS=(
    "/livox/lidar_front"
    "/livox/lidar_back"
    "/tf"
    "/tf_static"
)
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
                          (default: <workspace>/.sisyphus/evidence/calib-bags)
  --dry-run               Print planned command and metadata; create no bag

RECORDED TOPICS:
  /livox/lidar_front      Front Mid360 raw CustomMsg  (real IP: $FRONT_IP)
  /livox/lidar_back       Back  Mid360 raw CustomMsg  (real IP: $BACK_IP)
  /tf                     Dynamic transforms
  /tf_static              Static transforms

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

# Resolve workspace root (4 levels up from scripts/): scripts/ -> pkg/ -> sentry_nav/ -> src/ -> ws/
resolve_workspace_root() {
    local candidate
    candidate="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
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
    local missing=()
    local topic_list

    log_info "Checking topic availability (timeout 5s per topic)..."
    # Get topic list once
    if ! topic_list="$("$ros2_bin" topic list 2>/dev/null)"; then
        log_error "Failed to query ROS2 topic list. Is a ROS2 graph running?"
        return 1
    fi

    for topic in "${RECORD_TOPICS[@]}"; do
        if echo "$topic_list" | grep -qF "$topic"; then
            log_info "  FOUND: $topic"
        else
            log_warn "  MISSING: $topic"
            missing+=("$topic")
        fi
    done

    if [ ${#missing[@]} -gt 0 ]; then
        log_error "The following required topics are not published:"
        for t in "${missing[@]}"; do
            log_error "  $t"
        done
        log_error ""
        if [ "$ENV" = "real" ]; then
            log_error "Real mode: ensure both Mid360 units are powered and livox_ros_driver2 is running."
            log_error "  Front Mid360 IP: $FRONT_IP"
            log_error "  Back  Mid360 IP: $BACK_IP"
            log_error "  Check: ros2 topic hz /livox/lidar_front /livox/lidar_back"
        else
            log_error "Sim mode: ensure Gazebo is running, unpaused, and the navigation stack is up."
            log_error "  ros2 launch rmu_gazebo_simulator bringup_sim.launch.py"
            log_error "  (then unpause and wait ~10s before running this script)"
        fi
        return 1
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

topics:
$(for t in "${RECORD_TOPICS[@]}"; do echo "  - $t"; done)

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
        OUTPUT_DIR="$ws_root/.sisyphus/evidence/calib-bags"
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

    # Build topic list for display
    local topics_str
    topics_str="${RECORD_TOPICS[*]}"

    record_cmd_display="ros2 bag record -o ${bag_dir} --duration ${DURATION} ${topics_str}"

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
        log_info "Planned command:"
        log_info "  $record_cmd_display"
        log_info ""
        log_info "Topics to record:"
        for t in "${RECORD_TOPICS[@]}"; do
            log_info "  $t"
        done
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

    # Preflight: check topics
    if ! check_topics "$ros2_bin"; then
        # Write metadata with failed status before exiting
        mkdir -p "$run_dir"
        write_metadata "$meta_file" "$bag_dir" "$record_cmd_display" "preflight-failed-missing-topics"
        log_error ""
        log_error "Preflight failed. Metadata written to: $meta_file"
        log_error "Fix the missing topics and re-run."
        exit 1
    fi

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

    # Execute recording
    local record_exit=0
    "$ros2_bin" bag record \
        -o "$bag_dir" \
        --duration "$DURATION" \
        "${RECORD_TOPICS[@]}" || record_exit=$?

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
