#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
RUN_DIR="${PROJECT_DIR}/runtime/to_platform"
TARGET_POSE="${RUN_DIR}/standard52h13_platform_target.yaml"
REPORT="${RUN_DIR}/platform_above_report.yaml"

[[ "${1:-}" == "I_CONFIRM_PLATFORM_ABOVE_ONLY" ]] || {
  echo "Refusing motion. Required: I_CONFIRM_PLATFORM_ABOVE_ONLY" >&2
  exit 2
}
[[ "${2:-}" == "I_HAVE_CLEARED_THE_TRANSPORT_PATH" ]] || {
  echo "Refusing motion. Required: I_HAVE_CLEARED_THE_TRANSPORT_PATH" >&2
  exit 2
}

source "${PROJECT_DIR}/scripts/source_ros_env.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"

live_topic_info=$(ros2 topic info /perception/gemini336l_live/standard52h13/detections 2>/dev/null || true)
transport_topic_info=$(ros2 topic info /perception/gemini336l_transport/standard52h13/detections 2>/dev/null || true)
if ! grep -Fq 'Publisher count: 1' <<<"${live_topic_info}" && \
   ! grep -Fq 'Publisher count: 1' <<<"${transport_topic_info}"; then
  echo "Gemini Standard52h13 detector is not running." >&2
  echo "Start the existing 336L dual-tag viewer or detector first." >&2
  exit 3
fi

control_node=/piper_x/agx_arm_ctrl_single_node
ros2 node list 2>/dev/null | grep -Fxq "${control_node}" || {
  echo "Piper control node is not running: ${control_node}" >&2
  exit 3
}

mkdir -p "${RUN_DIR}"
"${PROJECT_DIR}/scripts/to_platform/capture_platform_target.py" \
  --output "${TARGET_POSE}"

mapfile -t Q < <("${PROJECT_DIR}/scripts/read_ros_joint_degrees.py")
[[ ${#Q[@]} -eq 6 ]] || { echo "Failed to read six joints." >&2; exit 4; }

env -u PYTHONPATH PYTHONNOUSERSITE=1 "${PROJECT_DIR}/.conda/bin/python" \
  "${PROJECT_DIR}/scripts/to_platform/plan_platform_above.py" \
  --target-pose "${TARGET_POSE}" \
  --current-q-deg "${Q[@]}" \
  --platform-height-m 0.170 \
  --object-height-above-tag-m 0.200 \
  --report "${REPORT}"

"${PROJECT_DIR}/scripts/execute_ros_pregrasp.py" \
  --report "${REPORT}" \
  --confirmation I_CONFIRM_TRANSPORT_WITH_GRIP_TO_STANDARD52 \
  --mode transport_hold \
  --gripper-open-m 0.085 \
  --gripper-closed-m 0.058 \
  --tolerance-deg 0.5 \
  --timeout-s 40

echo "DRONE_HELD_200MM_ABOVE_PLATFORM_TAG_COMPLETE"
echo "No descent or release was commanded."
