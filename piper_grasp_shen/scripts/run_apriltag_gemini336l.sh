#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/source_ros_env.sh"
set -u

# This node only reads the Gemini color stream and publishes detections/TF.
# It does not open CAN or command the robot.
exec ros2 run apriltag_ros apriltag_node --ros-args \
  -r __node:=gemini336l_apriltag \
  -r __ns:=/perception/gemini336l \
  -r image_rect:=/gemini336l/color/image_raw \
  -r camera_info:=/gemini336l/color/camera_info \
  --params-file "${PROJECT_DIR}/config/apriltag_gemini336l.yaml"
