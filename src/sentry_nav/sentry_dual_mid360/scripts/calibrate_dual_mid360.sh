#!/bin/bash
# calibrate_dual_mid360.sh
# Placeholder wrapper for dual Mid360 LiDAR-to-LiDAR extrinsic calibration.
# Actual calibration execution is deferred to T11.
#
# Tool: Multi_LiCa (TUMFTM/Multi_LiCa)
# Source: src/third_party/Multi_LiCa/
# Method: FPFH coarse alignment (TEASER++) + GICP fine registration
# Output: extrinsic transform front_mid360 -> back_mid360 (x, y, z, roll, pitch, yaw)
#
# Prerequisites (must be satisfied before running real calibration in T11):
#   1. Both Mid360 sensors powered and publishing PointCloud2 topics:
#        /front_mid360/livox/lidar  (or configured topic name)
#        /back_mid360/livox/lidar   (or configured topic name)
#   2. Multi_LiCa Python deps installed:
#        pip install open3d scipy ros2_numpy pandas --break-system-packages
#   3. TEASER++ Python bindings compiled from submodule:
#        cd src/third_party/Multi_LiCa/TEASER-plusplus
#        mkdir build && cd build
#        cmake -DTEASERPP_PYTHON_VERSION=3.$(python3 -c "import sys; print(sys.version_info.minor)") ..
#        make teaserpp_python -j$(nproc)
#        cd python && pip install . --break-system-packages
#   4. Multi_LiCa built with colcon (separate from main nav stack):
#        COLCON_IGNORE is present in src/third_party/Multi_LiCa/ to exclude it
#        from the default workspace build. To build it explicitly, temporarily
#        remove the marker, build, then restore it:
#          rm src/third_party/Multi_LiCa/COLCON_IGNORE
#          source /opt/ros/jazzy/setup.bash
#          colcon build --base-paths src/third_party/Multi_LiCa \
#            --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
#          source install/setup.bash
#          touch src/third_party/Multi_LiCa/COLCON_IGNORE
#   5. Robot stationary on flat ground during data capture.
#
# Usage (T11 — not yet implemented):
#   bash calibrate_dual_mid360.sh [--pcd-dir <dir>] [--params <yaml>] [--dry-run]
#
# Known issues (upstream TUMFTM/Multi_LiCa#17):
#   - TEASER++ v2.0 Python bindings may fail to compile; upgrade pip first.
#   - scipy >= 1.6 changed cKDTree.query() n_jobs -> workers parameter.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Path: scripts/ -> sentry_dual_mid360/ -> sentry_nav/ -> src/ -> third_party/Multi_LiCa
_multi_lica_candidate="$SCRIPT_DIR/../../../third_party/Multi_LiCa"
if [ -d "$_multi_lica_candidate" ]; then
    MULTI_LICA_DIR="$(cd "$_multi_lica_candidate" && pwd)"
else
    MULTI_LICA_DIR="NOT_FOUND"
fi
unset _multi_lica_candidate

print_usage() {
    cat <<EOF
calibrate_dual_mid360.sh — Dual Mid360 extrinsic calibration wrapper (T11 placeholder)

USAGE:
  bash calibrate_dual_mid360.sh [OPTIONS]

OPTIONS:
  --help          Show this help message and exit
  --check-deps    Check that Multi_LiCa dependencies are installed
  --dry-run       Print what would be executed without running calibration

DESCRIPTION:
  This script is a placeholder for T11 calibration execution.
  It wraps Multi_LiCa (src/third_party/Multi_LiCa/) to calibrate the
  extrinsic transform between front_mid360 and back_mid360.

  Real calibration is NOT implemented yet. This script currently only
  prints usage and dependency status.

MULTI_LICA SOURCE:
  $MULTI_LICA_DIR

CALIBRATION LAUNCH (T11 — manual until this script is fully implemented):
  source $MULTI_LICA_DIR/install/setup.bash
  ros2 launch multi_lidar_calibrator calibration.launch.py \\
    parameter_file:=$MULTI_LICA_DIR/config/params.yaml

CONFIGURATION:
  Edit $MULTI_LICA_DIR/config/params.yaml to set:
    - lidar_topics: ["/front_mid360/livox/lidar", "/back_mid360/livox/lidar"]
    - target_lidar: "/front_mid360/livox/lidar"
    - output_path: <path to write calibration result>

EOF
}

check_deps() {
    local ok=true

    echo "=== Multi_LiCa dependency check ==="
    echo ""

    if [ "$MULTI_LICA_DIR" = "NOT_FOUND" ]; then
        echo "FAIL: src/third_party/Multi_LiCa/ not found"
        echo "      Run: git clone --recurse-submodules https://github.com/TUMFTM/Multi_LiCa.git src/third_party/Multi_LiCa"
        ok=false
    else
        echo "PASS: Multi_LiCa source found at $MULTI_LICA_DIR"
    fi

    for pkg in open3d scipy pandas; do
        if python3 -c "import $pkg" 2>/dev/null; then
            echo "PASS: Python package '$pkg' available"
        else
            echo "WARN: Python package '$pkg' not found — install with:"
            echo "      pip install $pkg --break-system-packages"
            ok=false
        fi
    done

    if python3 -c "import teaserpp_python" 2>/dev/null; then
        echo "PASS: teaserpp_python available"
    else
        echo "WARN: teaserpp_python not found — build from submodule:"
        echo "      cd $MULTI_LICA_DIR/TEASER-plusplus && mkdir build && cd build"
        echo "      cmake -DTEASERPP_PYTHON_VERSION=3.x .. && make teaserpp_python -j\$(nproc)"
        echo "      cd python && pip install . --break-system-packages"
        ok=false
    fi

    if ros2 pkg list 2>/dev/null | grep -q "multi_lidar_calibrator"; then
        echo "PASS: multi_lidar_calibrator ROS2 package found"
    else
        echo "WARN: multi_lidar_calibrator not in ROS2 package list"
        echo "      COLCON_IGNORE is present in $MULTI_LICA_DIR — remove it to build:"
        echo "        rm $MULTI_LICA_DIR/COLCON_IGNORE"
        echo "        colcon build --base-paths $MULTI_LICA_DIR --symlink-install"
        echo "        source install/setup.bash"
        echo "        touch $MULTI_LICA_DIR/COLCON_IGNORE"
        ok=false
    fi

    echo ""
    if $ok; then
        echo "All dependencies satisfied. Ready for T11 calibration."
    else
        echo "Some dependencies missing. See WARN lines above."
        return 1
    fi
}

main() {
    if [ $# -eq 0 ]; then
        print_usage
        exit 0
    fi

    case "${1:-}" in
        --help|-h)
            print_usage
            exit 0
            ;;
        --check-deps)
            check_deps
            exit $?
            ;;
        --dry-run)
            echo "[DRY-RUN] calibrate_dual_mid360.sh: T11 calibration not yet implemented."
            echo "[DRY-RUN] Would launch: ros2 launch multi_lidar_calibrator calibration.launch.py"
            echo "[DRY-RUN] Multi_LiCa source: $MULTI_LICA_DIR"
            echo "[DRY-RUN] NOTE: COLCON_IGNORE present in Multi_LiCa/ — remove it before colcon build, restore after."
            exit 0
            ;;
        *)
            echo "ERROR: Unknown option: ${1}"
            echo ""
            print_usage
            exit 1
            ;;
    esac
}

main "$@"
