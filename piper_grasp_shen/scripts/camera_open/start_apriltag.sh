#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
CAMERA="${1:-}"
ID="${2:-}"
SIZE="${3:-}"

if [[ ! "${CAMERA}" =~ ^(d435i|gemini)$ || ! "${ID}" =~ ^[0-9]+$ || -z "${SIZE}" ]]; then
  echo "Usage: $0 d435i|gemini TAG_ID TAG_SIZE_METERS" >&2
  echo "Example: $0 d435i 0 0.018" >&2
  exit 2
fi

source "${PROJECT_DIR}/scripts/source_ros_env.sh"
if [[ "${CAMERA}" == gemini ]]; then
  set +u
  source "${ORBBEC_ROS_SETUP:-${HOME}/orbbec_humble_ws/install/setup.bash}"
  set -u
  IMAGE_TOPIC=/gemini336l/color/image_raw
  CAMERA_INFO_TOPIC=/gemini336l/color/camera_info
  NS=/perception/gemini336l
else
  IMAGE_TOPIC=/sensors/d435i/color/image_raw
  CAMERA_INFO_TOPIC=/sensors/d435i/color/camera_info
  NS=/perception/d435i
fi

DETECTIONS_TOPIC="${NS}/apriltag/detections"
RESULT_TOPIC="${NS}/apriltag/result"

ros2 run apriltag_ros apriltag_node --ros-args \
  -r __node:=${CAMERA}_apriltag \
  -r __ns:=${NS}/apriltag \
  -r image_rect:=${IMAGE_TOPIC} \
  -r camera_info:=${CAMERA_INFO_TOPIC} \
  -r detections:=${DETECTIONS_TOPIC} \
  -p family:=36h11 \
  -p size:=${SIZE} \
  -p tag.ids:="[${ID}]" \
  -p tag.sizes:="[${SIZE}]" \
  -p tag.frames:="[${CAMERA}_apriltag_${ID}]" &
TAG_PID=$!

python3 "${PROJECT_DIR}/scripts/camera_open/apriltag_overlay_view.py" \
  --image-topic "${IMAGE_TOPIC}" \
  --detections-topic "${DETECTIONS_TOPIC}" \
  --result-topic "${RESULT_TOPIC}" &
OVERLAY_PID=$!

cleanup() {
  kill -TERM "${TAG_PID}" "${OVERLAY_PID}" 2>/dev/null || true
  wait "${TAG_PID}" "${OVERLAY_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "AprilTag family: tag36h11, id=${ID}, size=${SIZE} m"
echo "Detections: ${DETECTIONS_TOPIC}"
echo "Result image: ${RESULT_TOPIC}"
echo "Select ${RESULT_TOPIC} in rqt_image_view."
wait "${TAG_PID}"
