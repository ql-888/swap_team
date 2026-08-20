#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd -- "${SCRIPT_DIR}/../.." && pwd)
ORBBEC_ROS_SETUP="${ORBBEC_ROS_SETUP:-${HOME}/orbbec_humble_ws/install/setup.bash}"
CAMERA="both"
PIDS=()

usage() {
  echo "Usage: start_camera_drivers.sh [--camera d435i|gemini|both]"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --camera) CAMERA="${2:?missing camera name}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done
[[ "${CAMERA}" =~ ^(d435i|gemini|both)$ ]] || { usage >&2; exit 2; }

source "${PROJECT_DIR}/scripts/source_ros_env.sh"
pkill -TERM -f '[r]ealsense2_camera_node' 2>/dev/null || true
pkill -TERM -f '[c]omponent_container.*gemini336l' 2>/dev/null || true
sleep 1

if [[ "${CAMERA}" == gemini || "${CAMERA}" == both ]]; then
  [[ -f "${ORBBEC_ROS_SETUP}" ]] || { echo "Missing ${ORBBEC_ROS_SETUP}" >&2; exit 2; }
  set +u; source "${ORBBEC_ROS_SETUP}"; set -u
fi

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  for pid in "${PIDS[@]}"; do kill -TERM "${pid}" 2>/dev/null || true; done
  for pid in "${PIDS[@]}"; do wait "${pid}" 2>/dev/null || true; done
  exit "${status}"
}
trap cleanup EXIT INT TERM

if [[ "${CAMERA}" == d435i || "${CAMERA}" == both ]]; then
  ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=sensors camera_name:=d435i \
    enable_color:=true enable_depth:=true \
    enable_infra1:=false enable_infra2:=false \
    enable_gyro:=false enable_accel:=false initial_reset:=true \
    rgb_camera.color_profile:=640x480x30 \
    depth_module.depth_profile:=640x480x30 &
  PIDS+=("$!")
fi

if [[ "${CAMERA}" == gemini || "${CAMERA}" == both ]]; then
  ros2 launch orbbec_camera gemini_330_series.launch.py \
    camera_name:=gemini336l enable_color:=true enable_depth:=true \
    enable_left_ir:=false enable_right_ir:=false \
    enable_accel:=false enable_gyro:=false \
    color_width:=640 color_height:=480 color_fps:=30 \
    depth_width:=640 depth_height:=480 depth_fps:=30 \
    depth_registration:=false enable_point_cloud:=false \
    enable_colored_point_cloud:=false &
  PIDS+=("$!")
fi

echo "Camera drivers started. Keep this terminal open."
echo "D435i color: /sensors/d435i/color/image_raw"
echo "D435i depth: /sensors/d435i/depth/image_rect_raw"
echo "Gemini color: /gemini336l/color/image_raw"
echo "Gemini depth: /gemini336l/depth/image_raw"
wait -n "${PIDS[@]}"
