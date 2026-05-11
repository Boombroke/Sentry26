#!/bin/bash
# preview_real_xmacro.sh
#
# Quick 3D preview of wheeled_biped_real.sdf.xmacro in Gazebo Harmonic.
# Use this to eyeball sensor placement (front/back Mid360 poses, gimbal frame)
# after editing front_lidar_pose / back_lidar_pose in xmacro — no nav2 / slam /
# Point-LIO / rviz required.
#
# What it does:
#   1. Expands wheeled_biped_real.sdf.xmacro via xmacro4sdf into a tempfile.
#   2. Opens Gazebo with an empty world and spawns the expanded model at origin.
#
# What it does NOT do:
#   * start any of the navigation stack
#   * respect the real-robot Livox / IMU publishers (the real xmacro does not
#     include Gazebo sensor plugins; this is intentional — it is the *real*
#     TF profile, not a sim profile)
#   * validate the xmacro semantics beyond what xmacro4sdf checks
#
# Typical workflow:
#   edit src/sentry_robot_description/resource/xmacro/wheeled_biped_real.sdf.xmacro
#   bash src/sentry_nav/sentry_dual_mid360/scripts/preview_real_xmacro.sh
#   (inspect Gazebo GUI, close when done)
#
# Prerequisites:
#   source /opt/ros/jazzy/setup.bash
#   source install/setup.bash     # so GZ_SIM_RESOURCE_PATH picks up model://
#
# Flags:
#   --headless     Start Gazebo server only (no GUI). For CI / SDF sanity.
#   --keep-sdf     Do not delete the expanded SDF on exit (prints its path).
#   --help         This message.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
XMACRO_PATH="${WORKSPACE_ROOT}/src/sentry_robot_description/resource/xmacro/wheeled_biped_real.sdf.xmacro"

HEADLESS="no"
KEEP_SDF="no"

while [ $# -gt 0 ]; do
    case "$1" in
        --help|-h)
            sed -n '2,33p' "$0"   # print the comment block at the top
            exit 0
            ;;
        --headless) HEADLESS="yes"; shift ;;
        --keep-sdf) KEEP_SDF="yes"; shift ;;
        *) echo "[ERROR] unknown option: $1" >&2; exit 2 ;;
    esac
done

if [ ! -f "${XMACRO_PATH}" ]; then
    echo "[ERROR] xmacro not found: ${XMACRO_PATH}" >&2
    exit 1
fi

if ! command -v xmacro4sdf >/dev/null 2>&1; then
    echo "[ERROR] xmacro4sdf not on PATH. Source install/setup.bash first." >&2
    exit 1
fi

if ! command -v gz >/dev/null 2>&1; then
    echo "[ERROR] gz CLI not on PATH. Source /opt/ros/jazzy/setup.bash first." >&2
    exit 1
fi

# model:// resolution: Gazebo walks GZ_SIM_RESOURCE_PATH. Without sourcing the
# workspace install/ overlay it has no way to find mid360 / back_mid360 meshes.
if [ -z "${GZ_SIM_RESOURCE_PATH:-}" ]; then
    echo "[WARN] GZ_SIM_RESOURCE_PATH is empty; model:// URIs may fail to resolve."
    echo "[WARN] source install/setup.bash (or export GZ_SIM_RESOURCE_PATH) and retry."
fi

# xmacro4sdf resolves xmacro_include uri="file://..." relative to CWD, so we
# switch into the xmacro directory for the duration of expansion.
TMP_SDF="$(mktemp --suffix=.sdf /tmp/preview_real_xmacro.XXXXXX)"
cleanup() {
    if [ "${KEEP_SDF}" = "no" ]; then
        rm -f "${TMP_SDF}"
    else
        echo "[INFO] expanded SDF kept at: ${TMP_SDF}"
    fi
}
trap cleanup EXIT

pushd "$(dirname "${XMACRO_PATH}")" >/dev/null
echo "[INFO] expanding $(basename "${XMACRO_PATH}") -> ${TMP_SDF}"
xmacro4sdf "$(basename "${XMACRO_PATH}")" > "${TMP_SDF}"
popd >/dev/null

# Quick size sanity: a successful expansion for this model is normally >200
# lines. xmacro4sdf prints diagnostics to stdout on some failure paths, which
# would leave the file too small.
LINES=$(wc -l < "${TMP_SDF}")
if [ "${LINES}" -lt 50 ]; then
    echo "[ERROR] expanded SDF looks truncated (${LINES} lines). Contents:" >&2
    cat "${TMP_SDF}" >&2
    exit 1
fi
echo "[INFO] expansion OK (${LINES} lines)."

# Build an empty-world wrapper that spawns the expanded model as a single
# <include><uri>embedded</uri></include>. Easiest path: inline the expanded
# <model>…</model> block into an empty world.
WORLD_SDF="$(mktemp --suffix=.sdf /tmp/preview_real_xmacro_world.XXXXXX)"
trap 'rm -f "${WORLD_SDF}"; cleanup' EXIT

# Extract the <model>...</model> block from the expanded SDF. wheeled_biped
# xmacro outputs <sdf><model ...>...</model></sdf>; we strip the outer <sdf>
# wrapper and drop its contents into our world.
python3 - "${TMP_SDF}" "${WORLD_SDF}" <<'PY'
import re
import sys

src, dst = sys.argv[1:]
with open(src, encoding="utf-8") as f:
    text = f.read()

match = re.search(r"(<model\b.*?</model>)", text, flags=re.S)
if not match:
    sys.exit("could not locate <model>...</model> in expanded SDF")
model_block = match.group(1)

world = f"""<?xml version="1.0"?>
<sdf version="1.8">
  <world name="preview">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material>
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    {model_block}
  </world>
</sdf>
"""
with open(dst, "w", encoding="utf-8") as f:
    f.write(world)
PY

echo "[INFO] composed preview world: ${WORLD_SDF}"
echo "[INFO] launching Gazebo (close the window when done)..."

if [ "${HEADLESS}" = "yes" ]; then
    exec gz sim -s --verbose 3 -r "${WORLD_SDF}"
else
    exec gz sim --verbose 3 -r "${WORLD_SDF}"
fi
