#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/source_ros_env.sh"
set -u

PIDS=()
cleanup() {
  local pid
  for pid in "${PIDS[@]}"; do
    kill "${pid}" 2>/dev/null || true
  done
  for pid in "${PIDS[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

TOPIC_INFO=$(ros2 topic info /gemini336l/color/image_raw 2>/dev/null || true)
grep -Fq 'Publisher count: 1' <<< "${TOPIC_INFO}" || {
  echo "Gemini color stream is not running: /gemini336l/color/image_raw" >&2
  exit 2
}

ros2 run apriltag_ros apriltag_node --ros-args \
  -r __node:=gemini_tag36h11_live \
  -r __ns:=/perception/gemini336l_live \
  -r image_rect:=/gemini336l/color/image_raw \
  -r camera_info:=/gemini336l/color/camera_info \
  -r detections:=/perception/gemini336l_live/tag36h11/detections \
  --params-file "${PROJECT_DIR}/config/apriltag_gemini336l_live_36h11.yaml" &
PIDS+=("$!")

ros2 run apriltag_ros apriltag_node --ros-args \
  -r __node:=gemini_standard52h13_live \
  -r __ns:=/perception/gemini336l_live \
  -r image_rect:=/gemini336l/color/image_raw \
  -r camera_info:=/gemini336l/color/camera_info \
  -r detections:=/perception/gemini336l_live/standard52h13/detections \
  --params-file "${PROJECT_DIR}/config/apriltag_gemini336l_live_standard52h13.yaml" &
PIDS+=("$!")

sleep 1
/usr/bin/python3 "${PROJECT_DIR}/scripts/live_gemini336l_dual_tag_viewer.py"
