#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd -- "${SCRIPT_DIR}/../.." && pwd)
EYE_IN_DIR="${PROJECT_DIR}/scripts/eye_in _grasp"

[[ "${1:-}" == "I_CONFIRM_GLOBAL_RELATIVE_EYE_IN_GRASP" ]] || {
  echo "Required confirmation: I_CONFIRM_GLOBAL_RELATIVE_EYE_IN_GRASP" >&2; exit 2;
}
[[ "${2:-}" == "I_HAVE_CLEARED_THE_WORKSPACE" ]] || {
  echo "Required confirmation: I_HAVE_CLEARED_THE_WORKSPACE" >&2; exit 2;
}
[[ "${3:-}" == "I_CONFIRM_DRONE_AND_TAG_FIXED" ]] || {
  echo "Required confirmation: I_CONFIRM_DRONE_AND_TAG_FIXED" >&2; exit 2;
}

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
source "${PROJECT_DIR}/scripts/source_ros_env.sh"

node_list=$(ros2 node list 2>/dev/null || true)
grep -Fxq '/piper_x/agx_arm_ctrl_single_node' <<<"${node_list}" || {
  echo "Piper control node is not running in ROS_DOMAIN_ID=${ROS_DOMAIN_ID}." >&2; exit 3;
}
grep -Fxq '/perception/gemini336l/gemini336l_drone_apriltag' <<<"${node_list}" || {
  echo "Gemini detector is not running in ROS_DOMAIN_ID=${ROS_DOMAIN_ID}." >&2; exit 3;
}
grep -Fxq '/perception/d435i/d435i_drone_apriltag' <<<"${node_list}" || {
  echo "D435i detector is not running in ROS_DOMAIN_ID=${ROS_DOMAIN_ID}." >&2; exit 3;
}

echo "PIPELINE STAGE 1/2: Gemini Tag-relative pre-observation"
"${SCRIPT_DIR}/run_relative_observation.sh" \
  I_CONFIRM_EYE_TO_GRASP_RELATIVE \
  I_HAVE_CLEARED_THE_WORKSPACE \
  I_CONFIRM_DRONE_AND_TAG_FIXED

echo "PIPELINE STAGE 2/2: original eye-in-grasp local visual grasp"
"${PROJECT_DIR}/scripts/run_full_d435i_drone_grasp_retreat.sh" \
  I_CONFIRM_FULL_DRONE_GRASP_RETREAT \
  I_HAVE_CLEARED_THE_WORKSPACE \
  I_CONFIRM_DRONE_AND_TAG_FIXED

echo "GLOBAL_RELATIVE_EYE_IN_GRASP_COMPLETE"
