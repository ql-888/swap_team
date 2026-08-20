#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd -- "${SCRIPT_DIR}/../.." && pwd)

if [[ "${1:-}" == "--dry-run" ]]; then
  echo "mode: dry_run"
  echo "stack: vision"
  echo "component: d435i_camera"
  echo "component: d435i_handeye_tf"
  echo "component: d435i_drone_apriltag_29mm"
  exit 0
fi

source "${PROJECT_DIR}/scripts/source_ros_env.sh"

if ros2 node list 2>/dev/null | grep -Eq \
  '^/(sensors/d435i|perception/d435i/d435i_drone_apriltag)$'; then
  echo "Vision-stack nodes are already running; refusing to create duplicates." >&2
  echo "Use ./scripts/'eye_in _grasp'/status.sh to inspect them." >&2
  exit 3
fi

LOG_DIR="${PROJECT_DIR}/runtime/eye_in_grasp/logs"
mkdir -p "${LOG_DIR}"
STAMP=$(date +%Y%m%d_%H%M%S)
CAMERA_LOG="${LOG_DIR}/${STAMP}_d435i_camera.log"
HANDEYE_LOG="${LOG_DIR}/${STAMP}_d435i_handeye_tf.log"
TAG_LOG="${LOG_DIR}/${STAMP}_d435i_drone_apriltag.log"
PIDS=()

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  for pid in "${PIDS[@]}"; do
    kill -TERM "${pid}" 2>/dev/null || true
  done
  for pid in "${PIDS[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
  exit "${status}"
}
trap cleanup EXIT INT TERM

ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=sensors \
  camera_name:=d435i \
  enable_color:=true \
  enable_depth:=true \
  enable_infra1:=false \
  enable_infra2:=false \
  enable_gyro:=false \
  enable_accel:=false \
  >"${CAMERA_LOG}" 2>&1 &
PIDS+=("$!")

"${PROJECT_DIR}/scripts/publish_d435i_handeye_tf.sh" \
  >"${HANDEYE_LOG}" 2>&1 &
PIDS+=("$!")

"${PROJECT_DIR}/scripts/run_apriltag_d435i_drone.sh" \
  >"${TAG_LOG}" 2>&1 &
PIDS+=("$!")

ready=false
for _ in $(seq 1 40); do
  nodes=$(ros2 node list 2>/dev/null || true)
  if grep -Fxq '/sensors/d435i' <<<"${nodes}" && \
     grep -Fxq '/perception/d435i/d435i_drone_apriltag' <<<"${nodes}"; then
    ready=true
    break
  fi
  for pid in "${PIDS[@]}"; do
    kill -0 "${pid}" 2>/dev/null || {
      echo "A vision-stack process exited during startup." >&2
      tail -n 30 "${CAMERA_LOG}" "${HANDEYE_LOG}" "${TAG_LOG}" >&2 || true
      exit 4
    }
  done
  sleep 1
done

if [[ "${ready}" != "true" ]]; then
  echo "Vision stack did not become ready within 40 seconds." >&2
  tail -n 30 "${CAMERA_LOG}" "${HANDEYE_LOG}" "${TAG_LOG}" >&2 || true
  exit 5
fi

family=$(ros2 param get /perception/d435i/d435i_drone_apriltag family | awk '{print $NF}')
size=$(ros2 param get /perception/d435i/d435i_drone_apriltag size | awk '{print $NF}')
if [[ "${family}" != "36h11" || "${size}" != "0.029" ]]; then
  echo "Wrong AprilTag detector parameters: family=${family}, size=${size}." >&2
  exit 6
fi

echo "VISION_STACK_READY"
echo "camera_log: ${CAMERA_LOG}"
echo "handeye_log: ${HANDEYE_LOG}"
echo "apriltag_log: ${TAG_LOG}"
echo "Keep this terminal open."

set +e
wait -n "${PIDS[@]}"
child_status=$?
set -e
echo "A vision-stack process stopped (exit ${child_status}); stopping the stack." >&2
exit 7
