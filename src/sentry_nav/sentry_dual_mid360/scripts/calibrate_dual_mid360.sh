#!/bin/bash
# calibrate_dual_mid360.sh
#
# Dual Mid360 LiDAR-to-LiDAR extrinsic calibration workflow (Task 11).
#
# Wraps Multi_LiCa (TUMFTM) with an operator-ready preflight, BLOCKED
# evidence emission, and a safe xmacro writeback policy:
#
#   The script NEVER edits
#     src/sentry_robot_description/resource/xmacro/wheeled_biped_real.sdf.xmacro
#   unless ALL of the following are true:
#     1. A Multi_LiCa report was produced with numeric translation and rotation errors.
#     2. translation error < 2.0 cm AND rotation error < 0.5 deg (MH8 of the plan).
#     3. The operator passed --write-xmacro explicitly.
#   Any violation keeps the CAD default in place.
#
# Exit codes:
#   0 PASS     deps OK, calibration accepted, or clean dry-run / help
#   1 FAIL     live calibration produced numbers but violated thresholds
#   2 BLOCKED  inputs or runtime dependencies missing
#   3 USAGE    bad CLI arguments
#
# Hand-in contract: on every invocation the script writes / refreshes
#     <evidence-dir>/task-11-run.log
#     <evidence-dir>/task-11-report.md
#     <evidence-dir>/task-11-blocker.md      (when BLOCKED)
#     <evidence-dir>/task-11-precision-check.txt
#     <evidence-dir>/task-11-xmacro-update.txt

set -euo pipefail

SCRIPT_NAME="$(basename "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="$(cd "${PACKAGE_DIR}/../../.." && pwd)"
REAL_XMACRO_PATH="${WORKSPACE_ROOT}/src/sentry_robot_description/resource/xmacro/wheeled_biped_real.sdf.xmacro"

# Multi_LiCa vendored path (T6). Guard absence explicitly instead of relying
# on set -e with a fragile $(cd ...) fallback.
_multi_lica_candidate="${WORKSPACE_ROOT}/src/third_party/Multi_LiCa"
if [ -d "${_multi_lica_candidate}" ]; then
    MULTI_LICA_DIR="${_multi_lica_candidate}"
else
    MULTI_LICA_DIR="NOT_FOUND"
fi
unset _multi_lica_candidate

MULTI_LICA_COLCON_IGNORE="${MULTI_LICA_DIR}/COLCON_IGNORE"
TEASER_PLUSPLUS_DIR="${MULTI_LICA_DIR}/TEASER-plusplus"

# Calibration acceptance thresholds (plan MH8).
MAX_TRANSLATION_ERROR_CM="2.0"
MAX_ROTATION_ERROR_DEG="0.5"

# CAD default back_lidar_pose. Kept in sync with
# wheeled_biped_real.sdf.xmacro to detect drift in --check-deps output.
CAD_BACK_LIDAR_POSE="0.05 0 0.05 0.0 0.5235987755982988 3.141592653589793"

# CLI inputs populated by parse_args.
MODE="usage"
BAG_PATH=""
OUTPUT_REPORT=""
EVIDENCE_DIR=""
RUN_LOG_PATH=""
BLOCKER_PATH=""
PRECISION_CHECK_PATH=""
XMACRO_UPDATE_PATH=""
WRITE_XMACRO="no"
DRY_RUN="no"
LOG_LEVEL="info"

# Accumulators populated during dep / input checks.
BLOCKER_REASONS=()
MISSING_DEPS=()

log() {
    local level="$1"
    shift
    local timestamp
    timestamp="$(date '+%Y-%m-%dT%H:%M:%S%z')"
    echo "[${timestamp}] [${level}] $*"
}

info()  { log INFO "$@"; }
warn()  { log WARN "$@"; }
err()   { log ERROR "$@" 1>&2; }
debug() { [ "${LOG_LEVEL}" = "debug" ] && log DEBUG "$@"; return 0; }

print_usage() {
    cat <<EOF
${SCRIPT_NAME} - Dual Mid360 extrinsic calibration (Multi_LiCa + xmacro writeback)

USAGE:
  bash ${SCRIPT_NAME} --help
  bash ${SCRIPT_NAME} --check-deps [--evidence-dir <dir>]
  bash ${SCRIPT_NAME} --bag <rosbag2_dir> --output-report <path>
                     [--evidence-dir <dir>] [--write-xmacro]
                     [--dry-run] [--log-level info|debug]

REQUIRED INPUTS (calibrate mode)
  --bag <rosbag2_dir>
      Calibration bag produced by record_calib_bag.sh (T10). Must contain
      /livox/lidar_front and /livox/lidar_back (Livox CustomMsg) plus
      /tf and /tf_static. Multi_LiCa cannot ingest CustomMsg directly;
      this script extracts CustomMsg point data into deterministic front/back
      PCD files before invoking the calibrator, so the bag does NOT need to
      carry pre-converted point clouds.
  --output-report <path>
      Destination Markdown report path. Parent directory is created.
      Contains the CAD initial guess, Multi_LiCa output, optional
      refinement output, translation/rotation errors vs CAD, and a
      PASS / FAIL / BLOCKED verdict keyed on MH8 thresholds.

OPTIONAL
  --evidence-dir <dir>
      Evidence sink (default: <workspace>/.sisyphus/evidence). Always
      produces task-11-run.log, task-11-report.md, task-11-blocker.md,
      task-11-precision-check.txt, task-11-xmacro-update.txt.
  --write-xmacro
      Opt-in switch to update back_lidar_pose in wheeled_biped_real.sdf.xmacro
      AFTER a real accepted calibration report. Refused otherwise.
  --dry-run
      Run the preflight audit only; skip Multi_LiCa invocation. Still
      emits BLOCKED evidence when inputs are missing.
  --log-level info|debug
      Increase verbosity. Default info.

MODES
  --help        Print this usage and exit 0.
  --check-deps  Probe Multi_LiCa source, COLCON_IGNORE marker, runtime
                Python deps (teaserpp_python, open3d, pandas), ros2 CLI,
                xmacro source file, back_lidar_pose declaration, and
                evidence-dir writability. Exit 0 when every prerequisite
                is in place; exit 2 BLOCKED otherwise. Never invokes
                Multi_LiCa.

REQUIRED DEPENDENCIES (live calibration run)
  Tooling
    * Multi_LiCa source at ${MULTI_LICA_DIR}
    * A COLCON_IGNORE marker at ${MULTI_LICA_COLCON_IGNORE}
      excludes Multi_LiCa from the default workspace build. To build it
      explicitly, follow the T6 remove / build / restore workflow:
        rm ${MULTI_LICA_COLCON_IGNORE}
        source /opt/ros/jazzy/setup.bash
        colcon build --base-paths ${MULTI_LICA_DIR} \\
          --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
        source install/setup.bash
        touch ${MULTI_LICA_COLCON_IGNORE}
      Do NOT commit a repo state that leaves COLCON_IGNORE removed.
    * ros2 CLI on PATH (Jazzy).
    * multi_lidar_calibrator built and discoverable by ros2 pkg prefix.
  Python runtime (virtualenv or system site-packages)
    * teaserpp_python  - TEASER++ Python bindings. Build from
                         ${TEASER_PLUSPLUS_DIR}:
                           cd ${TEASER_PLUSPLUS_DIR} && mkdir -p build && cd build
                           cmake -DTEASERPP_PYTHON_VERSION=3.\$(python3 -c 'import sys; print(sys.version_info.minor)') ..
                           make teaserpp_python -j\$(nproc)
                           cd python && pip install . --break-system-packages
    * open3d           - pip install open3d --break-system-packages
    * pandas           - pip install pandas --break-system-packages
    * scipy, ros2_numpy - Multi_LiCa runtime helpers.
    * rosbag2_py, rclpy, rosidl_runtime_py - sourced ROS bag extraction helpers.
  Data
    * Calibration bag recorded by record_calib_bag.sh (static rig, flat
      ground, varied geometry). Multi_LiCa needs PointCloud2/PCD, not
      CustomMsg, so this script prepares PCD artifacts inside the pipeline.
    * wheeled_biped_real.sdf.xmacro with a back_lidar_pose parameter.

SAFETY RULES
  * wheeled_biped_real.sdf.xmacro is NEVER edited unless a Multi_LiCa
    report exists, translation_error_cm < ${MAX_TRANSLATION_ERROR_CM},
    rotation_error_deg < ${MAX_ROTATION_ERROR_DEG}, AND --write-xmacro was
    passed. Missing any condition keeps the CAD default in place:
      back_lidar_pose="${CAD_BACK_LIDAR_POSE}"
  * Calibration results live in xmacro only (Layer A installation pose).
    Do NOT write calibrated extrinsics into any JSON driver config or
    runtime YAML (MN16 / MN20 / MN22).
  * Multi_LiCa / TEASER++ upstream source is never modified by this
    script.

EVIDENCE CONTRACT
  Every invocation (including BLOCKED) writes or refreshes:
    <evidence-dir>/task-11-run.log
    <evidence-dir>/task-11-report.md
    <evidence-dir>/task-11-blocker.md        (BLOCKED path only)
    <evidence-dir>/task-11-precision-check.txt
                                            (BLOCKED unless real numbers)
    <evidence-dir>/task-11-xmacro-update.txt
                                            (action=UPDATED/NO_UPDATE)

EXAMPLES
  bash ${SCRIPT_NAME} --help
  bash ${SCRIPT_NAME} --check-deps
  bash ${SCRIPT_NAME} --bag .sisyphus/evidence/calib-bags/sim-20260601 \\
                      --output-report .sisyphus/evidence/task-11-report.md --dry-run
  bash ${SCRIPT_NAME} --bag .sisyphus/evidence/calib-bags/real-20260601 \\
                      --output-report logs/calib-report.md --write-xmacro

EXIT CODES
  0 PASS     (help, --check-deps OK, clean dry-run, or calibration accepted)
  1 FAIL     (live Multi_LiCa produced numbers but violated thresholds)
  2 BLOCKED  (inputs or runtime dependencies missing)
  3 USAGE    (bad CLI arguments)

REFERENCES
  Plan:       .sisyphus/plans/dual-mid360-fusion.md (Task 11)
  Multi_LiCa: https://github.com/TUMFTM/Multi_LiCa
  pixmoving:  https://github.com/pixmoving-moveit/multi_lidar_calibration_ros2

KNOWN UPSTREAM QUIRKS
  * TEASER++ v2.0 Python bindings may need a pip upgrade before building
    (see TUMFTM/Multi_LiCa#17).
  * scipy >= 1.6 renamed cKDTree.query() n_jobs to workers; if that trips
    during an attempted run, this script reports the exact Multi_LiCa log
    blocker instead of editing vendored upstream source.
EOF
}

parse_args() {
    if [ $# -eq 0 ]; then
        MODE="help"
        return 0
    fi

    while [ $# -gt 0 ]; do
        case "$1" in
            --help|-h)
                MODE="help"
                shift
                ;;
            --check-deps)
                MODE="check_deps"
                shift
                ;;
            --bag)
                [ $# -ge 2 ] || { err "--bag requires a directory argument"; return 3; }
                BAG_PATH="$2"
                MODE="calibrate"
                shift 2
                ;;
            --output-report)
                [ $# -ge 2 ] || { err "--output-report requires a path argument"; return 3; }
                OUTPUT_REPORT="$2"
                shift 2
                ;;
            --evidence-dir)
                [ $# -ge 2 ] || { err "--evidence-dir requires a path argument"; return 3; }
                EVIDENCE_DIR="$2"
                shift 2
                ;;
            --write-xmacro)
                WRITE_XMACRO="yes"
                shift
                ;;
            --dry-run)
                DRY_RUN="yes"
                shift
                ;;
            --log-level)
                [ $# -ge 2 ] || { err "--log-level requires info or debug"; return 3; }
                case "$2" in
                    info|debug) LOG_LEVEL="$2" ;;
                    *) err "unknown log level: $2"; return 3 ;;
                esac
                shift 2
                ;;
            --)
                shift
                break
                ;;
            -*)
                err "unknown option: $1"
                return 3
                ;;
            *)
                err "unexpected positional argument: $1"
                return 3
                ;;
        esac
    done
    return 0
}

resolve_evidence_paths() {
    if [ -z "${EVIDENCE_DIR}" ]; then
        EVIDENCE_DIR="${WORKSPACE_ROOT}/.sisyphus/evidence"
    fi
    mkdir -p "${EVIDENCE_DIR}"
    RUN_LOG_PATH="${EVIDENCE_DIR}/task-11-run.log"
    BLOCKER_PATH="${EVIDENCE_DIR}/task-11-blocker.md"
    PRECISION_CHECK_PATH="${EVIDENCE_DIR}/task-11-precision-check.txt"
    XMACRO_UPDATE_PATH="${EVIDENCE_DIR}/task-11-xmacro-update.txt"
    if [ -z "${OUTPUT_REPORT}" ]; then
        OUTPUT_REPORT="${EVIDENCE_DIR}/task-11-report.md"
    fi
    mkdir -p "$(dirname "${OUTPUT_REPORT}")"
}

record_run_log() {
    echo "$*" >> "${RUN_LOG_PATH}"
}

check_python_module() {
    local module="$1"
    python3 - "${module}" <<'PY'
import importlib.util
import sys
spec = importlib.util.find_spec(sys.argv[1])
sys.exit(0 if spec is not None else 1)
PY
}

check_cli() {
    command -v "$1" >/dev/null 2>&1
}

run_dependency_audit() {
    info "Probing Multi_LiCa vendor directory: ${MULTI_LICA_DIR}"
    if [ "${MULTI_LICA_DIR}" = "NOT_FOUND" ]; then
        err "  Multi_LiCa source tree NOT FOUND at ${WORKSPACE_ROOT}/src/third_party/Multi_LiCa"
        BLOCKER_REASONS+=("Multi_LiCa source tree not vendored at src/third_party/Multi_LiCa (clone TUMFTM/Multi_LiCa and remove nested .git)")
        MISSING_DEPS+=("Multi_LiCa vendored source")
    else
        info "  Multi_LiCa OK: ${MULTI_LICA_DIR}"
        if [ -f "${MULTI_LICA_COLCON_IGNORE}" ]; then
            info "  COLCON_IGNORE present (default workspace build skips Multi_LiCa, as expected)."
        else
            warn "  COLCON_IGNORE MISSING at ${MULTI_LICA_COLCON_IGNORE}"
            warn "  Default colcon builds may attempt to compile Multi_LiCa."
            BLOCKER_REASONS+=("COLCON_IGNORE missing at ${MULTI_LICA_COLCON_IGNORE}; restore before committing")
        fi
        if [ ! -d "${TEASER_PLUSPLUS_DIR}" ]; then
            warn "  TEASER-plusplus submodule missing at ${TEASER_PLUSPLUS_DIR}"
            BLOCKER_REASONS+=("TEASER-plusplus directory missing; re-vendor Multi_LiCa with submodules")
        fi
    fi

    info "Probing Python runtime modules (teaserpp_python, open3d, pandas, scipy, ros2_numpy, rosbag2_py, rclpy, rosidl_runtime_py)"
    local py_module
    for py_module in teaserpp_python open3d pandas scipy ros2_numpy rosbag2_py rclpy rosidl_runtime_py; do
        if check_python_module "${py_module}"; then
            info "  Python module '${py_module}' OK."
        else
            warn "  Python module '${py_module}' MISSING."
            MISSING_DEPS+=("python3-${py_module}")
            case "${py_module}" in
                teaserpp_python)
                    BLOCKER_REASONS+=("teaserpp_python not importable; build TEASER++ Python bindings from ${TEASER_PLUSPLUS_DIR}")
                    ;;
                open3d|pandas)
                    BLOCKER_REASONS+=("Python module '${py_module}' not importable; pip install ${py_module} --break-system-packages")
                    ;;
                scipy|ros2_numpy)
                    BLOCKER_REASONS+=("Python module '${py_module}' not importable; install Multi_LiCa requirements and source the ROS environment")
                    ;;
                rosbag2_py|rclpy|rosidl_runtime_py)
                    BLOCKER_REASONS+=("Python module '${py_module}' not importable; source /opt/ros/jazzy/setup.bash and this workspace before extracting bag data")
                    ;;
            esac
        fi
    done

    info "Probing ros2 CLI"
    if check_cli ros2; then
        info "  ros2 CLI OK: $(command -v ros2)"
        if ros2 pkg prefix multi_lidar_calibrator >/dev/null 2>&1; then
            info "  multi_lidar_calibrator package OK: $(ros2 pkg prefix multi_lidar_calibrator)"
        else
            warn "  multi_lidar_calibrator package NOT discoverable by ros2."
            MISSING_DEPS+=("multi_lidar_calibrator ROS2 package build")
            BLOCKER_REASONS+=("multi_lidar_calibrator is not discoverable; temporarily remove ${MULTI_LICA_COLCON_IGNORE}, build Multi_LiCa, source install/setup.bash, then restore COLCON_IGNORE")
        fi
    else
        err "  ros2 CLI NOT on PATH. source /opt/ros/jazzy/setup.bash before invoking calibration."
        MISSING_DEPS+=("ros2 CLI (Jazzy)")
        BLOCKER_REASONS+=("ros2 CLI not available; source /opt/ros/jazzy/setup.bash")
    fi

    info "Probing sentry_robot_description xmacro source"
    if [ -f "${REAL_XMACRO_PATH}" ]; then
        info "  Real-robot xmacro OK: ${REAL_XMACRO_PATH}"
        if grep -q "back_lidar_pose" "${REAL_XMACRO_PATH}"; then
            info "  back_lidar_pose parameter declared in xmacro (T4 complete)."
        else
            err "  back_lidar_pose parameter NOT declared. Writeback target missing."
            BLOCKER_REASONS+=("back_lidar_pose not declared in ${REAL_XMACRO_PATH}; complete T4 xmacro injection")
        fi
    else
        err "  xmacro source MISSING at ${REAL_XMACRO_PATH}"
        BLOCKER_REASONS+=("wheeled_biped_real.sdf.xmacro missing; cannot write back calibrated pose")
        MISSING_DEPS+=("${REAL_XMACRO_PATH}")
    fi

    info "Probing evidence directory writability: ${EVIDENCE_DIR}"
    if mkdir -p "${EVIDENCE_DIR}" 2>/dev/null && [ -w "${EVIDENCE_DIR}" ]; then
        info "  Evidence directory writable."
    else
        err "  Evidence directory NOT writable: ${EVIDENCE_DIR}"
        BLOCKER_REASONS+=("evidence directory ${EVIDENCE_DIR} is not writable by this user")
    fi
}

check_bag_path() {
    if [ -z "${BAG_PATH}" ]; then
        warn "No --bag argument provided for calibrate mode."
        BLOCKER_REASONS+=("missing --bag argument (calibrate mode requires a rosbag2 directory from record_calib_bag.sh)")
        MISSING_DEPS+=("calibration rosbag2 directory (T10 output)")
        return 1
    fi

    if [ ! -d "${BAG_PATH}" ]; then
        warn "Bag path missing or not a directory: ${BAG_PATH}"
        BLOCKER_REASONS+=("bag path not found: ${BAG_PATH}")
        MISSING_DEPS+=("calibration rosbag2 directory at ${BAG_PATH}")
        return 1
    fi

    if [ ! -f "${BAG_PATH}/metadata.yaml" ]; then
        warn "Bag ${BAG_PATH} lacks metadata.yaml; run 'ros2 bag reindex' before calibrating."
        BLOCKER_REASONS+=("bag ${BAG_PATH} lacks metadata.yaml; run 'ros2 bag reindex ${BAG_PATH}' first")
        return 1
    fi

    info "Bag structure looks valid: ${BAG_PATH}"
    return 0
}

format_reasons_block() {
    if [ ${#BLOCKER_REASONS[@]} -eq 0 ]; then
        printf -- '- (no specific reason recorded)\n'
    else
        local reason
        for reason in "${BLOCKER_REASONS[@]}"; do
            printf -- '- %s\n' "${reason}"
        done
    fi
}

format_missing_block() {
    if [ ${#MISSING_DEPS[@]} -eq 0 ]; then
        printf -- '- (none)\n'
    else
        local dep
        for dep in "${MISSING_DEPS[@]}"; do
            printf -- '- %s\n' "${dep}"
        done
    fi
}

write_precision_blocked() {
    local reason="$1"
    cat > "${PRECISION_CHECK_PATH}" <<EOF
verdict: BLOCKED
reason: ${reason}
rule: Only write PASS / FAIL here when a real Multi_LiCa report with
      numeric translation_error_cm and rotation_error_deg has been parsed.
      See ${SCRIPT_NAME} --help for acceptance thresholds
      (<${MAX_TRANSLATION_ERROR_CM} cm / <${MAX_ROTATION_ERROR_DEG} deg).
thresholds:
  max_translation_error_cm: ${MAX_TRANSLATION_ERROR_CM}
  max_rotation_error_deg:   ${MAX_ROTATION_ERROR_DEG}
next_steps:
  1. Restore prerequisites (see task-11-blocker.md).
  2. Re-run ${SCRIPT_NAME} --bag <dir> --output-report <path>.
  3. After a live run this file will contain PASS / FAIL numbers instead of BLOCKED.
EOF
}

write_xmacro_no_update_reason() {
    local reason="$1"
    cat > "${XMACRO_UPDATE_PATH}" <<EOF
xmacro_path: ${REAL_XMACRO_PATH}
action: NO_UPDATE
reason: ${reason}
safety_rule: |
  ${SCRIPT_NAME} refuses to edit back_lidar_pose unless ALL of the following hold:
    1. A Multi_LiCa calibration report exists and was parsed to numeric errors.
    2. translation_error_cm < ${MAX_TRANSLATION_ERROR_CM}
       AND rotation_error_deg < ${MAX_ROTATION_ERROR_DEG}
    3. The operator passed --write-xmacro explicitly.
  Any violation keeps the xmacro unchanged. The CAD default
      back_lidar_pose="${CAD_BACK_LIDAR_POSE}"
  remains in place until a real accepted calibration is produced.
EOF
}

write_blocked_report() {
    local reasons_block
    local missing_block
    reasons_block="$(format_reasons_block)"
    missing_block="$(format_missing_block)"
    local now
    now="$(date '+%Y-%m-%dT%H:%M:%S%z')"

    cat > "${OUTPUT_REPORT}" <<EOF
# Dual Mid360 Extrinsic Calibration Report

- verdict: **BLOCKED**
- generated_at: ${now}
- script: ${SCRIPT_NAME}
- workspace_root: ${WORKSPACE_ROOT}
- evidence_dir: ${EVIDENCE_DIR}
- run_log: ${RUN_LOG_PATH}

## Blocker Reasons

${reasons_block}
## Missing Dependencies

${missing_block}
## What A Live Run Would Produce

\`\`\`
translation error: <x.xx> cm
rotation error:    <y.yy> deg
back_lidar_pose:   <x y z roll pitch yaw>
\`\`\`

Acceptance thresholds (plan MH8):

- translation error < ${MAX_TRANSLATION_ERROR_CM} cm
- rotation error    < ${MAX_ROTATION_ERROR_DEG} deg

## xmacro Writeback Status

See task-11-xmacro-update.txt. This run performed NO writeback because
the calibration could not complete. CAD default remains in place:
\`back_lidar_pose="${CAD_BACK_LIDAR_POSE}"\`.

## Recovery Steps

1. Install runtime dependencies listed above. Multi_LiCa workflow:
   - Source already vendored at \`src/third_party/Multi_LiCa\`.
   - Remove COLCON_IGNORE, build, restore:
     \`\`\`
     rm ${MULTI_LICA_COLCON_IGNORE}
     source /opt/ros/jazzy/setup.bash
     colcon build --base-paths ${MULTI_LICA_DIR} \\
       --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
     source install/setup.bash
     touch ${MULTI_LICA_COLCON_IGNORE}
     \`\`\`
2. Build TEASER++ Python bindings:
   \`\`\`
   cd ${TEASER_PLUSPLUS_DIR} && mkdir -p build && cd build
   cmake -DTEASERPP_PYTHON_VERSION=3.\$(python3 -c 'import sys; print(sys.version_info.minor)') ..
   make teaserpp_python -j\$(nproc)
   cd python && pip install . --break-system-packages
   \`\`\`
3. Install Python deps:
   \`\`\`
   pip install open3d pandas --break-system-packages
   \`\`\`
4. Record a calibration bag (T10):
   \`\`\`
   bash src/sentry_nav/sentry_dual_mid360/scripts/record_calib_bag.sh --env real --duration 60
   \`\`\`
   Required topics: /livox/lidar_front, /livox/lidar_back, /tf, /tf_static.
5. Re-invoke:
   \`\`\`
   bash ${SCRIPT_NAME} --bag <bag_dir> --output-report <report.md>
   \`\`\`
6. Only after a PASS verdict (both thresholds met) should --write-xmacro
   be used to update back_lidar_pose.

## Notes

- Livox CustomMsg cannot be ingested by Multi_LiCa directly; this script
  extracts CustomMsg point data into PCD files inside the pipeline. That
  preprocessing relies on ros2 bag Python modules being importable.
- xmacro remains the sole truth source for the back Mid360 installation
  pose (plan MN16 / MN22). Do not write extrinsics into Livox JSON.
- Multi_LiCa and TEASER++ upstream source are never modified by this
  script.
EOF

    cat > "${BLOCKER_PATH}" <<EOF
# task-11 BLOCKED summary

- script: ${SCRIPT_NAME}
- workspace_root: ${WORKSPACE_ROOT}
- evidence_dir: ${EVIDENCE_DIR}
- generated_at: ${now}

## Blocker Reasons

${reasons_block}
## Missing Dependencies

${missing_block}
## Next Steps

See \`${OUTPUT_REPORT}\` section "Recovery Steps".
EOF
}

get_pose_attr() {
    local attr_name="$1"
    python3 - "${REAL_XMACRO_PATH}" "${attr_name}" <<'PY'
import re
import sys

path, attr = sys.argv[1:3]
text = open(path, encoding="utf-8").read()
match = re.search(rf'{re.escape(attr)}="([^"]+)"', text)
if not match:
    raise SystemExit(f"missing {attr} in {path}")
print(match.group(1))
PY
}

make_multi_lica_rel_path() {
    local target_path="$1"
    python3 - "${target_path}" <<'PY'
import os
import sys
import multi_lidar_calibrator

target = os.path.realpath(sys.argv[1])
module_dir = os.path.dirname(os.path.realpath(multi_lidar_calibrator.__file__))
print("/" + os.path.relpath(target, module_dir).rstrip("/") + "/")
PY
}

extract_custommsg_to_pcd() {
    local bag_dir="$1"
    local pcd_dir="$2"
    mkdir -p "${pcd_dir}"
    info "Extracting Livox CustomMsg points from bag into deterministic PCD files."
    python3 - "${bag_dir}" "${pcd_dir}" <<'PY'
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag_dir, pcd_dir = sys.argv[1:3]
topics = {
    "/livox/lidar_front": "front_mid360",
    "/livox/lidar_back": "back_mid360",
}
max_points_per_lidar = int(os.environ.get("SENTRY_DUAL_MID360_CALIB_MAX_POINTS", "300000"))

storage_id = "sqlite3"
if any(name.endswith(".mcap") for name in os.listdir(bag_dir)):
    storage_id = "mcap"

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage_id),
    rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
)
type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
missing = [topic for topic in topics if topic not in type_map]
if missing:
    raise SystemExit("missing required CustomMsg topics in bag: " + ", ".join(missing))

msg_types = {}
for topic in topics:
    if type_map[topic] != "livox_ros_driver2/msg/CustomMsg":
        raise SystemExit(f"topic {topic} has type {type_map[topic]}, expected livox_ros_driver2/msg/CustomMsg")
    msg_types[topic] = get_message(type_map[topic])

points = {topic: [] for topic in topics}
messages = {topic: 0 for topic in topics}
while reader.has_next():
    topic, data, _ = reader.read_next()
    if topic not in topics or len(points[topic]) >= max_points_per_lidar:
        continue
    msg = deserialize_message(data, msg_types[topic])
    messages[topic] += 1
    remaining = max_points_per_lidar - len(points[topic])
    for point in msg.points[:remaining]:
        points[topic].append((float(point.x), float(point.y), float(point.z)))

for topic, lidar_name in topics.items():
    if not points[topic]:
        raise SystemExit(f"topic {topic} produced zero points from {messages[topic]} messages")
    path = os.path.join(pcd_dir, f"{lidar_name}.pcd")
    with open(path, "w", encoding="utf-8") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(points[topic])}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points[topic])}\n")
        f.write("DATA ascii\n")
        for x, y, z in points[topic]:
            f.write(f"{x:.9g} {y:.9g} {z:.9g}\n")
    print(f"{topic}: messages={messages[topic]} points={len(points[topic])} pcd={path}")
PY
}

generate_multi_lica_params() {
    local params_path="$1"
    local output_params_path="$2"
    local pcd_rel="$3"
    local output_rel="$4"
    local results_file="$5"
    local front_pose
    local back_pose
    front_pose="$(get_pose_attr front_lidar_pose)"
    back_pose="$(get_pose_attr back_lidar_pose)"

    python3 - "${params_path}" "${output_params_path}" "${pcd_rel}" "${output_rel}" "${results_file}" "${front_pose}" "${back_pose}" <<'PY'
import math
import sys

params_path, output_params_path, pcd_rel, output_rel, results_file, front_pose, back_pose = sys.argv[1:]

def pose_to_table(pose):
    vals = [float(x) for x in pose.split()]
    if len(vals) != 6:
        raise SystemExit(f"expected 6 pose values, got {len(vals)} from {pose!r}")
    return vals[:3] + [math.degrees(v) for v in vals[3:]]

def fmt(values):
    return "[" + ", ".join(f"{v:.12g}" for v in values) + "]"

front = pose_to_table(front_pose)
back = pose_to_table(back_pose)
with open(params_path, "w", encoding="utf-8") as f:
    f.write("/**:\n")
    f.write("  ros__parameters:\n")
    f.write("    frame_count: 1\n")
    f.write("    runs_count: 1\n")
    f.write("    visualize: false\n")
    f.write("    use_fitness_based_calibration: false\n")
    f.write("    lidar_topics: [front_mid360, back_mid360]\n")
    f.write("    tf_topic: /tf_static\n")
    f.write("    read_tf_from_table: true\n")
    f.write("    table_degrees: true\n")
    f.write(f"    front_mid360: {fmt(front)}\n")
    f.write(f"    back_mid360: {fmt(back)}\n")
    f.write("    read_pcds_from_file: true\n")
    f.write(f"    pcd_directory: {pcd_rel}\n")
    f.write("    target_frame_id: front_mid360\n")
    f.write("    base_frame_id: gimbal_pitch\n")
    f.write("    calibrate_target: false\n")
    f.write("    calibrate_to_base: false\n")
    f.write(f"    output_dir: {output_rel}\n")
    f.write(f"    default_output_dir: {output_rel}\n")
    f.write(f"    results_file: {results_file}\n")
    f.write("    max_corresp_dist: 1.0\n")
    f.write("    rel_fitness: 0.0000001\n")
    f.write("    rel_rmse: 0.0000001\n")
    f.write("    max_iterations: 100\n")
    f.write("    epsilon: 0.005\n")
    f.write("    voxel_size: 0.05\n")
    f.write("    remove_ground_flag: false\n")
    f.write("    fitness_score_threshold: 0.2\n")
    f.write("    distance_threshold: 0.1\n")
    f.write("    ransac_n: 10\n")
    f.write("    num_iterations: 2000\n")
    f.write("    crop_cloud: 25\n")

with open(output_params_path, "w", encoding="utf-8") as f:
    f.write("/**:\n")
    f.write("  ros__parameters:\n")
    f.write(f"    output_dir: {output_rel}\n")
PY
}

parse_multi_lica_result() {
    local results_file_path="$1"
    local parsed_env_path="$2"
    python3 - "${results_file_path}" "${parsed_env_path}" "${PRECISION_CHECK_PATH}" "${REAL_XMACRO_PATH}" "${MAX_TRANSLATION_ERROR_CM}" "${MAX_ROTATION_ERROR_DEG}" <<'PY'
import math
import re
import shlex
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

results_path, env_path, precision_path, xmacro_path, max_t_cm, max_r_deg = sys.argv[1:]
max_t_cm = float(max_t_cm)
max_r_deg = float(max_r_deg)
text = open(results_path, encoding="utf-8", errors="replace").read()

def get_pose_attr(attr):
    src = open(xmacro_path, encoding="utf-8").read()
    m = re.search(rf'{re.escape(attr)}="([^"]+)"', src)
    if not m:
        raise RuntimeError(f"missing {attr} in {xmacro_path}")
    vals = [float(v) for v in m.group(1).split()]
    if len(vals) != 6:
        raise RuntimeError(f"{attr} should have 6 values, got {len(vals)}")
    return vals

block_re = re.compile(
    r"back_mid360\s+to\s+front_mid360\s+calibration(?P<body>.*?)(?:_{10,}|Complete calibration time|\Z)",
    re.S,
)
block_match = block_re.search(text)
search_text = block_match.group("body") if block_match else text
xyz_match = re.search(r"calibrated xyz\s*=\s*([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)", search_text)
rpy_match = re.search(r"calibrated rpy\s*=\s*([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)", search_text)
if not xyz_match or not rpy_match:
    reason = "Could not parse 'calibrated xyz' and 'calibrated rpy' for back_mid360 to front_mid360 from Multi_LiCa results"
    with open(precision_path, "w", encoding="utf-8") as f:
        f.write("verdict: BLOCKED\n")
        f.write(f"reason: {reason}\n")
        f.write(f"results_file: {results_path}\n")
    with open(env_path, "w", encoding="utf-8") as f:
        f.write("PARSE_VERDICT=BLOCKED\n")
        f.write("PARSE_REASON=" + shlex.quote(reason) + "\n")
    raise SystemExit(2)

candidate_xyz = np.array([float(xyz_match.group(i)) for i in range(1, 4)], dtype=float)
candidate_rpy_deg = np.array([float(rpy_match.group(i)) for i in range(1, 4)], dtype=float)

front_pose = get_pose_attr("front_lidar_pose")
back_pose = get_pose_attr("back_lidar_pose")

def matrix_from_pose(vals):
    mat = np.eye(4)
    mat[:3, 3] = vals[:3]
    mat[:3, :3] = R.from_euler("xyz", vals[3:], degrees=False).as_matrix()
    return mat

front_base = matrix_from_pose(front_pose)
back_base_cad = matrix_from_pose(back_pose)
cad_back_in_front = np.linalg.inv(front_base) @ back_base_cad
candidate_back_in_front = np.eye(4)
candidate_back_in_front[:3, 3] = candidate_xyz
candidate_back_in_front[:3, :3] = R.from_euler("xyz", candidate_rpy_deg, degrees=True).as_matrix()

translation_error_cm = float(np.linalg.norm(candidate_back_in_front[:3, 3] - cad_back_in_front[:3, 3]) * 100.0)
rot_delta = R.from_matrix(candidate_back_in_front[:3, :3] @ cad_back_in_front[:3, :3].T)
rotation_error_deg = float(np.linalg.norm(rot_delta.as_rotvec()) * 180.0 / math.pi)

candidate_back_base = front_base @ candidate_back_in_front
candidate_back_xyz = candidate_back_base[:3, 3]
candidate_back_rpy = R.from_matrix(candidate_back_base[:3, :3]).as_euler("xyz", degrees=False)
candidate_back_pose_vals = list(candidate_back_xyz) + list(candidate_back_rpy)
candidate_back_pose = " ".join(f"{v:.15g}" for v in candidate_back_pose_vals)
candidate_pair_pose = " ".join([*(f"{v:.15g}" for v in candidate_xyz), *(f"{v:.15g}" for v in np.deg2rad(candidate_rpy_deg))])

accepted = translation_error_cm < max_t_cm and rotation_error_deg < max_r_deg
verdict = "PASS" if accepted else "FAIL"
with open(precision_path, "w", encoding="utf-8") as f:
    f.write(f"verdict: {verdict}\n")
    f.write(f"translation_error_cm: {translation_error_cm:.6f}\n")
    f.write(f"rotation_error_deg: {rotation_error_deg:.6f}\n")
    f.write(f"max_translation_error_cm: {max_t_cm}\n")
    f.write(f"max_rotation_error_deg: {max_r_deg}\n")
    f.write(f"candidate_back_in_front_pose_xyz_rpy_rad: {candidate_pair_pose}\n")
    f.write(f"candidate_back_lidar_pose_xyz_rpy_rad: {candidate_back_pose}\n")
    f.write(f"results_file: {results_path}\n")

with open(env_path, "w", encoding="utf-8") as f:
    f.write(f"PARSE_VERDICT={verdict}\n")
    f.write(f"TRANSLATION_ERROR_CM={translation_error_cm:.6f}\n")
    f.write(f"ROTATION_ERROR_DEG={rotation_error_deg:.6f}\n")
    f.write("CANDIDATE_BACK_POSE=" + shlex.quote(candidate_back_pose) + "\n")
    f.write("CANDIDATE_PAIR_POSE=" + shlex.quote(candidate_pair_pose) + "\n")
    f.write("PARSE_REASON=''\n")
PY
}

write_live_report() {
    local verdict="$1"
    local work_dir="$2"
    local multi_lica_log="$3"
    local results_file_path="$4"
    local translation_error_cm="$5"
    local rotation_error_deg="$6"
    local candidate_back_pose="$7"
    local now
    now="$(date '+%Y-%m-%dT%H:%M:%S%z')"
    cat > "${OUTPUT_REPORT}" <<EOF
# Dual Mid360 Extrinsic Calibration - Live Run

- verdict: **${verdict}**
- generated_at: ${now}
- bag: ${BAG_PATH}
- work_dir: ${work_dir}
- multi_lica_log: ${multi_lica_log}
- multi_lica_results: ${results_file_path}
- run_log: ${RUN_LOG_PATH}

## Precision Gate

- translation_error_cm: ${translation_error_cm}
- rotation_error_deg: ${rotation_error_deg}
- max_translation_error_cm: ${MAX_TRANSLATION_ERROR_CM}
- max_rotation_error_deg: ${MAX_ROTATION_ERROR_DEG}

## Candidate xmacro Pose

\`\`\`
back_lidar_pose="${candidate_back_pose}"
\`\`\`

See task-11-precision-check.txt and task-11-xmacro-update.txt for the
machine-readable threshold and writeback status.
EOF
}

update_xmacro_pose() {
    local candidate_pose="$1"
    python3 - "${REAL_XMACRO_PATH}" "${candidate_pose}" <<'PY'
import re
import sys

path, pose = sys.argv[1:]
text = open(path, encoding="utf-8").read()
new_text, count = re.subn(r'back_lidar_pose="[^"]+"', f'back_lidar_pose="{pose}"', text, count=1)
if count != 1:
    raise SystemExit(f"expected exactly one back_lidar_pose in {path}, replaced {count}")
with open(path, "w", encoding="utf-8") as f:
    f.write(new_text)
PY
}

run_check_deps_mode() {
    resolve_evidence_paths
    : > "${RUN_LOG_PATH}"
    info "Running --check-deps (no calibration will be launched)."
    record_run_log "mode: check-deps"
    record_run_log "workspace_root: ${WORKSPACE_ROOT}"
    record_run_log "evidence_dir:   ${EVIDENCE_DIR}"

    run_dependency_audit

    if [ ${#BLOCKER_REASONS[@]} -eq 0 ]; then
        info "All prerequisites satisfied for a live calibration run."
        record_run_log "verdict: PASS (dependencies OK)"
        cat > "${OUTPUT_REPORT}" <<EOF
# Dual Mid360 Extrinsic Calibration - Dependency Audit

- verdict: **PASS (dependencies OK)**
- generated_at: $(date '+%Y-%m-%dT%H:%M:%S%z')
- workspace_root: ${WORKSPACE_ROOT}
- evidence_dir: ${EVIDENCE_DIR}
- run_log: ${RUN_LOG_PATH}

All runtime prerequisites for Multi_LiCa-based calibration are in
place. This report does NOT claim that a calibration has been
executed or accepted; re-run ${SCRIPT_NAME} with --bag to attempt an
actual live run.
EOF
        rm -f "${BLOCKER_PATH}"
        write_xmacro_no_update_reason "--check-deps mode only; no calibration executed, no writeback attempted"
        write_precision_blocked "--check-deps mode only; no Multi_LiCa report has been produced in this run"
        return 0
    fi

    warn "Dependency audit found ${#BLOCKER_REASONS[@]} blockers; writing BLOCKED evidence."
    record_run_log "verdict: BLOCKED"
    write_blocked_report
    write_xmacro_no_update_reason "dependency audit failed; cannot calibrate, therefore cannot update xmacro"
    write_precision_blocked "dependency audit failed; no calibration was executed, no numeric precision data exists"
    return 2
}

run_calibrate_mode() {
    resolve_evidence_paths
    : > "${RUN_LOG_PATH}"
    info "Running calibrate mode (dry-run=${DRY_RUN}, write-xmacro=${WRITE_XMACRO})."
    record_run_log "mode: calibrate"
    record_run_log "workspace_root: ${WORKSPACE_ROOT}"
    record_run_log "evidence_dir:   ${EVIDENCE_DIR}"
    record_run_log "bag_path:       ${BAG_PATH}"
    record_run_log "output_report:  ${OUTPUT_REPORT}"
    record_run_log "write_xmacro:   ${WRITE_XMACRO}"
    record_run_log "dry_run:        ${DRY_RUN}"

    run_dependency_audit
    # Bag check accumulates its own blocker reasons; we never short-circuit
    # the audit above so the final evidence lists all missing pieces.
    check_bag_path || true

    if [ ${#BLOCKER_REASONS[@]} -gt 0 ]; then
        warn "Calibrate mode cannot proceed; writing BLOCKED evidence."
        record_run_log "verdict: BLOCKED"
        write_blocked_report
        write_xmacro_no_update_reason "calibration did not run; prerequisites missing (see blocker reasons)"
        write_precision_blocked "calibration did not run; no Multi_LiCa output parsed"
        return 2
    fi

    if [ "${DRY_RUN}" = "yes" ]; then
        info "Dry-run requested; skipping Multi_LiCa invocation."
        record_run_log "dry_run_skip: Multi_LiCa invocation bypassed"
        cat > "${OUTPUT_REPORT}" <<EOF
# Dual Mid360 Extrinsic Calibration - Dry Run

- verdict: **PASS (dry-run)**
- generated_at: $(date '+%Y-%m-%dT%H:%M:%S%z')
- bag: ${BAG_PATH}
- run_log: ${RUN_LOG_PATH}

Dry-run completed: dependency audit passed, bag structure validated.
No Multi_LiCa process was launched, so no calibration numbers exist.
Re-run without --dry-run to produce an actual calibration report.
EOF
        rm -f "${BLOCKER_PATH}"
        write_xmacro_no_update_reason "dry-run only; no calibration output produced"
        write_precision_blocked "dry-run only; no Multi_LiCa report generated"
        return 0
    fi

    local work_dir pcd_dir multi_output_dir params_path output_params_path multi_lica_log results_file results_file_path parsed_env pcd_rel output_rel
    work_dir="$(mktemp -d "${EVIDENCE_DIR}/task-11-work.XXXXXX")"
    pcd_dir="${work_dir}/pcd"
    multi_output_dir="${work_dir}/multi_lica_output"
    params_path="${work_dir}/multi_lica_sentry_params.yaml"
    output_params_path="${work_dir}/multi_lica_output_params.yaml"
    multi_lica_log="${EVIDENCE_DIR}/task-11-multi-lica.log"
    results_file="sentry_dual_mid360_results.txt"
    results_file_path="${multi_output_dir}/${results_file}"
    parsed_env="${work_dir}/parsed.env"
    mkdir -p "${pcd_dir}" "${multi_output_dir}"

    info "Created calibration work directory: ${work_dir}"
    record_run_log "work_dir: ${work_dir}"
    record_run_log "stage: extract CustomMsg bag data to front/back PCD"
    if ! extract_custommsg_to_pcd "${BAG_PATH}" "${pcd_dir}" >> "${RUN_LOG_PATH}" 2>&1; then
        BLOCKER_REASONS+=("CustomMsg bag extraction failed; see ${RUN_LOG_PATH} for exact topic/type/point-count error")
        write_blocked_report
        write_xmacro_no_update_reason "bag extraction failed; no calibration output produced"
        write_precision_blocked "bag extraction failed before Multi_LiCa invocation"
        return 2
    fi

    pcd_rel="$(make_multi_lica_rel_path "${pcd_dir}")"
    output_rel="$(make_multi_lica_rel_path "${multi_output_dir}")"
    generate_multi_lica_params "${params_path}" "${output_params_path}" "${pcd_rel}" "${output_rel}" "${results_file}"
    record_run_log "stage: generated Multi_LiCa params ${params_path}"
    record_run_log "stage: invoking ros2 launch multi_lidar_calibrator calibration.launch.py"

    local launch_rc=0
    timeout --preserve-status 1800s ros2 launch multi_lidar_calibrator calibration.launch.py \
        parameter_file:="${params_path}" \
        output_dir:="${output_params_path}" \
        > "${multi_lica_log}" 2>&1 || launch_rc=$?
    if [ "${launch_rc}" -ne 0 ]; then
        BLOCKER_REASONS+=("Multi_LiCa invocation exited ${launch_rc}; see ${multi_lica_log}")
        write_blocked_report
        write_xmacro_no_update_reason "Multi_LiCa invocation failed; no accepted numeric calibration"
        write_precision_blocked "Multi_LiCa invocation failed before parseable result was produced"
        record_run_log "verdict: BLOCKED (Multi_LiCa exit ${launch_rc})"
        return 2
    fi

    if [ ! -s "${results_file_path}" ]; then
        BLOCKER_REASONS+=("Multi_LiCa exited 0 but results file is missing or empty: ${results_file_path}; see ${multi_lica_log}")
        write_blocked_report
        write_xmacro_no_update_reason "Multi_LiCa produced no results file; no accepted numeric calibration"
        write_precision_blocked "Multi_LiCa results file missing after successful process exit"
        record_run_log "verdict: BLOCKED (missing Multi_LiCa results file)"
        return 2
    fi

    local parse_rc=0
    parse_multi_lica_result "${results_file_path}" "${parsed_env}" || parse_rc=$?
    if [ "${parse_rc}" -ne 0 ]; then
        # shellcheck disable=SC1090
        [ -f "${parsed_env}" ] && source "${parsed_env}"
        BLOCKER_REASONS+=("${PARSE_REASON:-Multi_LiCa results could not be parsed}; see ${results_file_path}")
        write_blocked_report
        write_xmacro_no_update_reason "Multi_LiCa result parsing failed; no accepted numeric calibration"
        record_run_log "verdict: BLOCKED (parse failed)"
        return 2
    fi

    # shellcheck disable=SC1090
    source "${parsed_env}"
    if [ "${PARSE_VERDICT}" = "PASS" ]; then
        if [ "${WRITE_XMACRO}" = "yes" ]; then
            local before_pose
            before_pose="$(get_pose_attr back_lidar_pose)"
            update_xmacro_pose "${CANDIDATE_BACK_POSE}"
            cat > "${XMACRO_UPDATE_PATH}" <<EOF
xmacro_path: ${REAL_XMACRO_PATH}
action: UPDATED
before_back_lidar_pose: ${before_pose}
after_back_lidar_pose: ${CANDIDATE_BACK_POSE}
translation_error_cm: ${TRANSLATION_ERROR_CM}
rotation_error_deg: ${ROTATION_ERROR_DEG}
EOF
        else
            write_xmacro_no_update_reason "calibration accepted, but --write-xmacro was not supplied"
        fi
        write_live_report "PASS" "${work_dir}" "${multi_lica_log}" "${results_file_path}" "${TRANSLATION_ERROR_CM}" "${ROTATION_ERROR_DEG}" "${CANDIDATE_BACK_POSE}"
        rm -f "${BLOCKER_PATH}"
        record_run_log "verdict: PASS"
        return 0
    fi

    write_live_report "FAIL" "${work_dir}" "${multi_lica_log}" "${results_file_path}" "${TRANSLATION_ERROR_CM}" "${ROTATION_ERROR_DEG}" "${CANDIDATE_BACK_POSE}"
    write_xmacro_no_update_reason "calibration numeric result failed thresholds; xmacro unchanged"
    rm -f "${BLOCKER_PATH}"
    record_run_log "verdict: FAIL (thresholds not met)"
    return 1
}

main() {
    local parse_rc=0
    parse_args "$@" || parse_rc=$?
    if [ "${parse_rc}" -ne 0 ]; then
        print_usage
        return "${parse_rc}"
    fi

    case "${MODE}" in
        help|usage)
            print_usage
            return 0
            ;;
        check_deps)
            run_check_deps_mode
            return $?
            ;;
        calibrate)
            run_calibrate_mode
            return $?
            ;;
        *)
            err "internal error: unknown mode ${MODE}"
            return 1
            ;;
    esac
}

main "$@"
