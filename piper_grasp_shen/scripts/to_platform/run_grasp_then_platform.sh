#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
RUN_DIR="${PROJECT_DIR}/runtime/to_platform"
LOG_DIR="${RUN_DIR}/logs"
STANDARD52_LOG="${LOG_DIR}/standard52h13_detector.log"
STANDARD52_PID=""

[[ "${1:-}" == "I_CONFIRM_GRASP_THEN_PLATFORM" ]] || {
  echo "Refusing motion. Required: I_CONFIRM_GRASP_THEN_PLATFORM" >&2
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

source "${PROJECT_DIR}/scripts/source_ros_env.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
mkdir -p "${LOG_DIR}"

cleanup_standard52() {
  if [[ -n "${STANDARD52_PID}" ]]; then
    kill "${STANDARD52_PID}" 2>/dev/null || true
    wait "${STANDARD52_PID}" 2>/dev/null || true
  fi
}
trap cleanup_standard52 EXIT INT TERM

echo "PIPELINE 1/2: run the existing eye-to-grasp workflow"
"${PROJECT_DIR}/scripts/eye_to_grasp/boot_and_grasp.sh" \
  I_CONFIRM_BOOT_AND_GRASP \
  I_HAVE_CLEARED_THE_WORKSPACE \
  I_CONFIRM_DRONE_AND_TAG_FIXED

echo "PIPELINE CHECK: verify the drone is still held"
gripper_feedback=$("${PROJECT_DIR}/scripts/read_ros_gripper_m.py")
env GRIPPER_FEEDBACK="${gripper_feedback}" /usr/bin/python3 - <<'PY'
import os

value = float(os.environ["GRIPPER_FEEDBACK"])
print(f"held_gripper_feedback_m: {value:.4f}")
if not 0.058 <= value <= 0.085:
    raise SystemExit("Refusing platform motion: gripper feedback does not indicate a held drone")
PY

echo "PIPELINE 2/2: start only the Gemini Standard52h13 platform detector"
"${PROJECT_DIR}/scripts/run_apriltag_gemini336l_standard52h13.sh" \
  >"${STANDARD52_LOG}" 2>&1 &
STANDARD52_PID=$!

ready=false
for _ in $(seq 1 30); do
  topic_info=$(ros2 topic info \
    /perception/gemini336l_transport/standard52h13/detections 2>/dev/null || true)
  if grep -Fq 'Publisher count: 1' <<<"${topic_info}"; then
    ready=true
    break
  fi
  if ! kill -0 "${STANDARD52_PID}" 2>/dev/null; then
    echo "Standard52h13 detector stopped during startup." >&2
    sed -n '1,120p' "${STANDARD52_LOG}" >&2 || true
    exit 3
  fi
  sleep 1
done
if [[ "${ready}" != true ]]; then
  echo "Standard52h13 detector did not become ready within 30 seconds." >&2
  exit 3
fi

"${PROJECT_DIR}/scripts/to_platform/run_platform_above.sh" \
  I_CONFIRM_PLATFORM_ABOVE_ONLY \
  I_HAVE_CLEARED_THE_TRANSPORT_PATH

echo "GRASP_THEN_PLATFORM_ABOVE_COMPLETE"
echo "The drone remains held above the platform; no descent or release was commanded."
