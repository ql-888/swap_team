#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
RUN_DIR="${PROJECT_DIR}/runtime/to_platform"
LOG_DIR="${RUN_DIR}/logs"
RELEASE_REPORT="${RUN_DIR}/platform_partial_release_debug_report.yaml"
RETREAT_REPORT="${RUN_DIR}/platform_open_retreat_debug_report.yaml"
DETECTOR_LOG="${LOG_DIR}/standard52h13_complete_pipeline.log"
DETECTOR_PID=""

[[ "${1:-}" == "I_CONFIRM_COMPLETE_PLATFORM_PLACEMENT" ]] || {
  echo "Refusing motion. Required: I_CONFIRM_COMPLETE_PLATFORM_PLACEMENT" >&2
  exit 2
}
[[ "${2:-}" == "I_HAVE_CLEARED_THE_WORKSPACE" ]] || {
  echo "Refusing motion. Required: I_HAVE_CLEARED_THE_WORKSPACE" >&2
  exit 2
}
[[ "${3:-}" == "I_CONFIRM_DRONE_AND_TAG_FIXED" ]] || {
  echo "Refusing motion. Required: I_CONFIRM_DRONE_AND_TAG_FIXED" >&2
  exit 2
}
[[ "${4:-}" == "I_HAVE_CLEARED_THE_TRANSPORT_PATH" ]] || {
  echo "Refusing motion. Required: I_HAVE_CLEARED_THE_TRANSPORT_PATH" >&2
  exit 2
}

checkpoint() {
  local required="$1"
  local description="$2"
  local answer=""
  echo >&2
  echo "CHECKPOINT: ${description}" >&2
  echo "Type exactly: ${required}" >&2
  if ! IFS= read -r answer || [[ "${answer}" != "${required}" ]]; then
    echo "Pipeline stopped. Required confirmation: ${required}" >&2
    exit 5
  fi
}

cleanup_detector() {
  if [[ -n "${DETECTOR_PID}" ]]; then
    kill "${DETECTOR_PID}" 2>/dev/null || true
    wait "${DETECTOR_PID}" 2>/dev/null || true
  fi
}
trap cleanup_detector EXIT INT TERM

source "${PROJECT_DIR}/scripts/source_ros_env.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
mkdir -p "${LOG_DIR}"

echo "PIPELINE 1/5: grasp the drone"
"${PROJECT_DIR}/scripts/eye_to_grasp/boot_and_grasp.sh" \
  I_CONFIRM_BOOT_AND_GRASP \
  I_HAVE_CLEARED_THE_WORKSPACE \
  I_CONFIRM_DRONE_AND_TAG_FIXED

checkpoint \
  I_CONFIRM_HELD_DRONE_TRANSPORT \
  "Grasp completed. Verify the drone is secure and the full transport path is clear."

held_width=$("${PROJECT_DIR}/scripts/read_ros_gripper_m.py")
HELD_WIDTH="${held_width}" /usr/bin/python3 -c \
  'import os; value=float(os.environ["HELD_WIDTH"]); assert 0.058 <= value <= 0.068, f"held-drone gripper feedback out of range: {value:.4f} m"'

echo "PIPELINE 2/5: detect the platform and reach the verified above-platform pose"
"${PROJECT_DIR}/scripts/run_apriltag_gemini336l_standard52h13.sh" \
  >"${DETECTOR_LOG}" 2>&1 &
DETECTOR_PID=$!

detector_ready=false
for _ in $(seq 1 30); do
  topic_info=$(ros2 topic info \
    /perception/gemini336l_transport/standard52h13/detections 2>/dev/null || true)
  if grep -Eq 'Publisher count: [1-9]' <<<"${topic_info}"; then
    detector_ready=true
    break
  fi
  if ! kill -0 "${DETECTOR_PID}" 2>/dev/null; then
    echo "Standard52h13 detector stopped during startup." >&2
    sed -n '1,120p' "${DETECTOR_LOG}" >&2 || true
    exit 3
  fi
  sleep 1
done
[[ "${detector_ready}" == true ]] || {
  echo "Standard52h13 detector did not become ready within 30 seconds." >&2
  exit 3
}

DEBUG_RUNNER="${PROJECT_DIR}/scripts/to_platform/run_platform_preplace_debug.sh"
"${DEBUG_RUNNER}" plan
"${DEBUG_RUNNER}" lift I_CONFIRM_PLATFORM_SAFE_LIFT_ONLY
"${DEBUG_RUNNER}" transport I_CONFIRM_PLATFORM_TRANSPORT_POSITION_ONLY
"${DEBUG_RUNNER}" align I_CONFIRM_PLATFORM_TILTED_ALIGN_ONLY

checkpoint \
  I_CONFIRM_PLATFORM_ABOVE_DESCENT \
  "The drone is 20 cm above the support plane at the verified 20-degree attitude. Verify alignment and descent clearance."

echo "PIPELINE 3/5: descend exactly 120 mm"
"${DEBUG_RUNNER}" plan-descend
"${DEBUG_RUNNER}" descend I_CONFIRM_PLATFORM_DESCEND_100MM_ONLY
"${DEBUG_RUNNER}" plan-descend-10mm
"${DEBUG_RUNNER}" descend-10mm I_CONFIRM_PLATFORM_DESCEND_10MM_ONLY
"${DEBUG_RUNNER}" plan-descend-10mm
"${DEBUG_RUNNER}" descend-10mm I_CONFIRM_PLATFORM_DESCEND_10MM_ONLY

checkpoint \
  I_CONFIRM_DRONE_SUPPORTED_RELEASE \
  "Descent completed. Verify the drone is stably supported by the platform before opening the gripper."

echo "PIPELINE 4/5: open the gripper exactly 10 mm"
held_width=$("${PROJECT_DIR}/scripts/read_ros_gripper_m.py")
env -u PYTHONPATH PYTHONNOUSERSITE=1 "${PROJECT_DIR}/.conda/bin/python" \
  "${PROJECT_DIR}/scripts/to_platform/plan_platform_release.py" \
  --current-width-m "${held_width}" \
  --delta-m 0.010 \
  --report "${RELEASE_REPORT}"
"${PROJECT_DIR}/scripts/to_platform/execute_platform_partial_release.py" \
  --report "${RELEASE_REPORT}" \
  --confirmation I_CONFIRM_PLATFORM_OPEN_GRIPPER_10MM_ONLY

echo "PIPELINE 5/5: retreat 120 mm along base +Z"
open_width=$("${PROJECT_DIR}/scripts/read_ros_gripper_m.py")
mapfile -t current_q < <("${PROJECT_DIR}/scripts/read_ros_joint_degrees.py")
[[ ${#current_q[@]} -eq 6 ]] || {
  echo "Failed to read six joints for retreat planning." >&2
  exit 4
}
env -u PYTHONPATH PYTHONNOUSERSITE=1 "${PROJECT_DIR}/.conda/bin/python" \
  "${PROJECT_DIR}/scripts/to_platform/plan_platform_open_retreat.py" \
  --current-q-deg "${current_q[@]}" \
  --retreat-m 0.120 \
  --open-width-m "${open_width}" \
  --report "${RETREAT_REPORT}"
"${PROJECT_DIR}/scripts/to_platform/execute_platform_open_retreat.py" \
  --report "${RETREAT_REPORT}" \
  --confirmation I_CONFIRM_PLATFORM_OPEN_RETREAT_120MM_ONLY

echo "COMPLETE_PLATFORM_PLACEMENT_FINISHED"
echo "The gripper remains open. No further robot motion is commanded."
