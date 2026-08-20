#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/source_ros_env.sh"
set -u

exec ros2 run apriltag_ros apriltag_node --ros-args \
  -r __node:=d435i_apriltag \
  -r __ns:=/perception/d435i \
  -r image_rect:=/sensors/d435i/color/image_raw \
  -r camera_info:=/sensors/d435i/color/camera_info \
  --params-file "${PROJECT_DIR}/config/apriltag_d435i.yaml"
