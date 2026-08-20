#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd -- "${SCRIPT_DIR}/../.." && pwd)
EYE_IN_DIR="${PROJECT_DIR}/scripts/eye_in _grasp"
DOMAIN="${ROS_DOMAIN_ID:-1}"
LOG_DIR="${PROJECT_DIR}/runtime/eye_to_grasp/logs"
mkdir -p "${LOG_DIR}"
export ROS_DOMAIN_ID="${DOMAIN}"

[[ "${1:-}" == "I_CONFIRM_BOOT_AND_GRASP" ]] || { echo "Required confirmation: I_CONFIRM_BOOT_AND_GRASP" >&2; exit 2; }
[[ "${2:-}" == "I_HAVE_CLEARED_THE_WORKSPACE" ]] || { echo "Required confirmation: I_HAVE_CLEARED_THE_WORKSPACE" >&2; exit 2; }
[[ "${3:-}" == "I_CONFIRM_DRONE_AND_TAG_FIXED" ]] || { echo "Required confirmation: I_CONFIRM_DRONE_AND_TAG_FIXED" >&2; exit 2; }

"${SCRIPT_DIR}/cleanup_eye_to_grasp.sh"

echo "EYE_TO_GRASP_CAN_SETUP"
"${PROJECT_DIR}/scripts/setup_can.sh" can0 1000000

cd "${PROJECT_DIR}"
"${SCRIPT_DIR}/start_robot_stack.sh" >"${LOG_DIR}/boot_robot.log" 2>&1 &
ROBOT_PID=$!
"${SCRIPT_DIR}/start_vision_stack.sh" >"${LOG_DIR}/boot_gemini.log" 2>&1 &
GEMINI_PID=$!
"${EYE_IN_DIR}/start_vision_stack.sh" >"${LOG_DIR}/boot_d435i.log" 2>&1 &
D435I_PID=$!

cleanup_children() {
  kill -TERM "${ROBOT_PID}" "${GEMINI_PID}" "${D435I_PID}" 2>/dev/null || true
}
trap cleanup_children INT TERM

for _ in $(seq 1 60); do
  nodes=$(ros2 node list 2>/dev/null || true)
  if grep -Fxq '/piper_x/agx_arm_ctrl_single_node' <<<"${nodes}" && \
     grep -Fxq '/piper_x/robot_state_publisher' <<<"${nodes}" && \
     grep -Fxq '/perception/gemini336l/gemini336l_drone_apriltag' <<<"${nodes}" && \
     grep -Fxq '/perception/d435i/d435i_drone_apriltag' <<<"${nodes}"; then
    echo "EYE_TO_GRASP_STACK_READY (ROS_DOMAIN_ID=${DOMAIN})"
    break
  fi
  if ! kill -0 "${ROBOT_PID}" 2>/dev/null; then
    echo "Robot stack failed during startup:" >&2
    sed -n '1,120p' "${LOG_DIR}/boot_robot.log" >&2 || true
    exit 4
  fi
  if ! kill -0 "${GEMINI_PID}" 2>/dev/null; then
    echo "Gemini stack failed during startup:" >&2
    sed -n '1,120p' "${LOG_DIR}/boot_gemini.log" >&2 || true
    exit 4
  fi
  if ! kill -0 "${D435I_PID}" 2>/dev/null; then
    echo "D435i stack failed during startup:" >&2
    sed -n '1,120p' "${LOG_DIR}/boot_d435i.log" >&2 || true
    exit 4
  fi
  sleep 1
done

nodes=$(ros2 node list 2>/dev/null || true)
for required in /piper_x/agx_arm_ctrl_single_node /piper_x/robot_state_publisher \
  /perception/gemini336l/gemini336l_drone_apriltag /perception/d435i/d435i_drone_apriltag; do
  grep -Fxq "${required}" <<<"${nodes}" || { echo "Missing node: ${required}" >&2; exit 4; }
done

"${SCRIPT_DIR}/run_global_relative_then_eye_in_grasp.sh" \
  I_CONFIRM_GLOBAL_RELATIVE_EYE_IN_GRASP \
  I_HAVE_CLEARED_THE_WORKSPACE \
  I_CONFIRM_DRONE_AND_TAG_FIXED

echo "BOOT_AND_GRASP_COMPLETE"
