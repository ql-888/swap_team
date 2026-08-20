#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd -- "${SCRIPT_DIR}/../.." && pwd)
source "${PROJECT_DIR}/scripts/source_ros_env.sh"

required_nodes=(
  /piper_x/agx_arm_ctrl_single_node
  /piper_x/robot_state_publisher
  /sensors/d435i
  /perception/d435i/d435i_drone_apriltag
)
nodes=$(ros2 node list 2>/dev/null || true)
for node in "${required_nodes[@]}"; do
  if ! grep -Fxq "${node}" <<<"${nodes}"; then
    echo "MISSING_NODE: ${node}" >&2
    exit 2
  fi
  echo "READY_NODE: ${node}"
done

speed=$(ros2 param get /piper_x/agx_arm_ctrl_single_node speed_percent | awk '{print $NF}')
control=$(ros2 param get /piper_x/agx_arm_ctrl_single_node control_enabled | awk '{print $NF}')
family=$(ros2 param get /perception/d435i/d435i_drone_apriltag family | awk '{print $NF}')
size=$(ros2 param get /perception/d435i/d435i_drone_apriltag size | awk '{print $NF}')

[[ "${speed}" == "5" ]] || { echo "BAD_SPEED_PERCENT: ${speed}" >&2; exit 3; }
[[ "${control}" == "True" ]] || { echo "CONTROL_DISABLED: ${control}" >&2; exit 3; }
[[ "${family}" == "36h11" ]] || { echo "BAD_TAG_FAMILY: ${family}" >&2; exit 3; }
[[ "${size}" == "0.029" ]] || { echo "BAD_TAG_SIZE: ${size}" >&2; exit 3; }

echo "speed_percent: ${speed}"
echo "control_enabled: ${control}"
echo "tag_family: ${family}"
echo "tag_size_m: ${size}"

POSE_FILE=$(mktemp /tmp/piper_eye_in_grasp_status.XXXXXX.yaml)
trap 'rm -f "${POSE_FILE}"' EXIT
"${PROJECT_DIR}/scripts/capture_d435i_apriltag_pose.py" \
  --samples 5 \
  --timeout-s 8 \
  --output "${POSE_FILE}"

"${PROJECT_DIR}/scripts/run_full_d435i_drone_grasp_retreat.sh" --preflight
echo "EYE_IN_GRASP_STATUS_READY"
