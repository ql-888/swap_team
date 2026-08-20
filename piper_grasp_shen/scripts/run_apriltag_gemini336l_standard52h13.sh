#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/source_ros_env.sh"
set -u

exec ros2 run apriltag_ros apriltag_node --ros-args \
  -r __node:=gemini_standard52h13_transport \
  -r __ns:=/perception/gemini336l_transport \
  -r image_rect:=/gemini336l/color/image_raw \
  -r camera_info:=/gemini336l/color/camera_info \
  -r detections:=/perception/gemini336l_transport/standard52h13/detections \
  --params-file "${PROJECT_DIR}/config/apriltag_gemini336l_live_standard52h13.yaml"
