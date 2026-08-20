#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd -- "${SCRIPT_DIR}/../.." && pwd)
ORBBEC_ROS_SETUP="${ORBBEC_ROS_SETUP:-${HOME}/orbbec_humble_ws/install/setup.bash}"

# Keep eye-to-hand vision on the same DDS domain as the Piper control stack.
# Override explicitly when running an isolated test.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"

if [[ "${1:-}" == "--dry-run" ]]; then
  echo "mode: dry_run"
  echo "stack: vision"
  echo "component: Orbbec Gemini 336L"
  echo "component: Gemini eye-to-hand static TF"
  echo "component: tag36h11 ID 0, 29 mm"
  exit 0
fi

source "${PROJECT_DIR}/scripts/source_ros_env.sh"
[[ -f "${PROJECT_DIR}/calibration_data/gemini336l_eye_to_hand.json" ]] || {
  echo "Missing Gemini calibration JSON: calibration_data/gemini336l_eye_to_hand.json" >&2
  exit 2
}
[[ -f "${PROJECT_DIR}/config/handeye_orbbec_fixed.yaml" ]] || {
  echo "Missing normalized Gemini hand-eye calibration: config/handeye_orbbec_fixed.yaml" >&2
  exit 2
}
if [[ -f "${ORBBEC_ROS_SETUP}" ]]; then
  # Orbbec setup files reference optional variables under nounset.
  set +u
  source "${ORBBEC_ROS_SETUP}"
  set -u
else
  echo "Orbbec ROS setup not found: ${ORBBEC_ROS_SETUP}" >&2
  echo "Set ORBBEC_ROS_SETUP to the Orbbec workspace install/setup.bash." >&2
  exit 2
fi

ros2 node list 2>/dev/null | grep -Eq '^/perception/gemini336l/gemini336l_drone_apriltag$' && {
  echo "Gemini vision stack is already running; refusing duplicates." >&2; exit 3;
}
ros2 pkg prefix orbbec_camera >/dev/null 2>&1 || {
  echo "orbbec_camera is unavailable after sourcing ${ORBBEC_ROS_SETUP}." >&2; exit 4;
}

LOG_DIR="${PROJECT_DIR}/runtime/eye_to_grasp/logs"
mkdir -p "${LOG_DIR}"
STAMP=$(date +%Y%m%d_%H%M%S)
PIDS=()
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  for pid in "${PIDS[@]}"; do kill -TERM "${pid}" 2>/dev/null || true; done
  for pid in "${PIDS[@]}"; do wait "${pid}" 2>/dev/null || true; done
  exit "${status}"
}
trap cleanup EXIT INT TERM

ros2 launch orbbec_camera gemini_330_series.launch.py \
  camera_name:=gemini336l \
  enable_color:=true \
  enable_depth:=false \
  enable_left_ir:=false \
  enable_right_ir:=false \
  enable_point_cloud:=false \
  enable_colored_point_cloud:=false \
  enable_accel:=false \
  enable_gyro:=false \
  >"${LOG_DIR}/${STAMP}_gemini336l_camera.log" 2>&1 &
PIDS+=("$!")

"${PROJECT_DIR}/scripts/publish_gemini336l_handeye_tf.sh" \
  >"${LOG_DIR}/${STAMP}_gemini336l_handeye_tf.log" 2>&1 &
PIDS+=("$!")

ros2 run apriltag_ros apriltag_node --ros-args \
  -r __node:=gemini336l_drone_apriltag \
  -r __ns:=/perception/gemini336l \
  -r image_rect:=/gemini336l/color/image_raw \
  -r camera_info:=/gemini336l/color/camera_info \
  --params-file "${SCRIPT_DIR}/apriltag_gemini336l_drone.yaml" \
  >"${LOG_DIR}/${STAMP}_gemini336l_apriltag.log" 2>&1 &
PIDS+=("$!")

for _ in $(seq 1 50); do
  topic_info=$(ros2 topic info /gemini336l/color/image_raw 2>/dev/null || true)
  nodes=$(ros2 node list 2>/dev/null || true)
  if grep -Eq 'Publisher count: [1-9]' <<<"${topic_info}" && \
     grep -Fxq '/perception/gemini336l/gemini336l_drone_apriltag' <<<"${nodes}"; then
    echo "VISION_STACK_READY"
    echo "camera_topic: /gemini336l/color/image_raw"
    echo "tag_frame: gemini_apriltag_0"
    echo "Keep this terminal open."
    set +e; wait -n "${PIDS[@]}"; status=$?; set -e
    echo "A vision-stack process stopped (exit ${status})." >&2
    exit 7
  fi
  for pid in "${PIDS[@]}"; do
    kill -0 "${pid}" 2>/dev/null || { echo "Vision-stack startup failed." >&2; exit 5; }
  done
  sleep 1
done
echo "Vision stack did not become ready within 50 seconds." >&2
exit 6
