#!/usr/bin/env bash
# Start camera ROS 2 drivers and open rqt_image_view for topic selection.
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd -- "${SCRIPT_DIR}/../.." && pwd)
ORBBEC_ROS_SETUP="${ORBBEC_ROS_SETUP:-${HOME}/orbbec_humble_ws/install/setup.bash}"
CAMERA="both"
PIDS=()

stop_stale_local_processes() {
  # Only stop processes that can hold the camera or the viewer from this workflow.
  pkill -TERM -f '[r]ealsense2_camera_node' 2>/dev/null || true
  pkill -TERM -f '[c]omponent_container.*gemini336l' 2>/dev/null || true
  pkill -TERM -f '[r]qt.*image_view' 2>/dev/null || true
  sleep 1
}

usage() {
  cat <<'EOF'
Usage: start_ros_cameras_rqt.sh [--camera d435i|gemini|both]

Starts the selected ROS 2 camera drivers and then opens rqt_image_view.
Select a topic in rqt_image_view:
  D435i wrist color:      /sensors/d435i/color/image_raw
  D435i wrist depth:      /sensors/d435i/aligned_depth_to_color/image_raw
  Gemini 336L global:     /gemini336l/color/image_raw
  Gemini 336L depth:      /gemini336l/depth/image_raw

Close rqt_image_view or press Ctrl-C here to stop only drivers started by this script.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --camera)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      CAMERA="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
done

[[ "${CAMERA}" =~ ^(d435i|gemini|both)$ ]] || { usage >&2; exit 2; }

source "${PROJECT_DIR}/scripts/source_ros_env.sh"
stop_stale_local_processes
if [[ "${CAMERA}" == "gemini" || "${CAMERA}" == "both" ]]; then
  [[ -f "${ORBBEC_ROS_SETUP}" ]] || {
    echo "Orbbec ROS setup not found: ${ORBBEC_ROS_SETUP}" >&2
    exit 2
  }
  set +u
  source "${ORBBEC_ROS_SETUP}"
  set -u
  ros2 pkg prefix orbbec_camera >/dev/null || {
    echo "orbbec_camera is unavailable after sourcing ${ORBBEC_ROS_SETUP}" >&2
    exit 2
  }
fi

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

wait_for_topic() {
  local topic="$1"
  local label="$2"
  for _ in $(seq 1 60); do
    if ros2 topic info "${topic}" 2>/dev/null | grep -Eq 'Publisher count: [1-9]'; then
      echo "${label} ready: ${topic}"
      return 0
    fi
    for pid in "${PIDS[@]}"; do
      kill -0 "${pid}" 2>/dev/null || {
        echo "${label} driver stopped during startup." >&2
        return 1
      }
    done
    sleep 1
  done
  echo "Timed out waiting for ${label} topic: ${topic}" >&2
  return 1
}

if [[ "${CAMERA}" == "d435i" || "${CAMERA}" == "both" ]]; then
  if ros2 node list 2>/dev/null | grep -Fxq '/sensors/d435i'; then
    echo "D435i node is already running; reusing /sensors/d435i/color/image_raw"
  else
    ros2 launch realsense2_camera rs_launch.py \
      camera_namespace:=sensors \
      camera_name:=d435i \
      enable_color:=true \
      enable_depth:=true \
      enable_infra1:=false \
      enable_infra2:=false \
      enable_gyro:=false \
      enable_accel:=false \
      initial_reset:=true \
      rgb_camera.color_profile:=640x480x30 \
      depth_module.depth_profile:=640x480x30 &
    PIDS+=("$!")
  fi
fi

if [[ "${CAMERA}" == "gemini" || "${CAMERA}" == "both" ]]; then
  if ros2 node list 2>/dev/null | grep -Fxq '/gemini336l/camera_container'; then
    echo "Gemini node is already running; reusing /gemini336l/color/image_raw"
  else
    ros2 launch orbbec_camera gemini_330_series.launch.py \
      camera_name:=gemini336l \
      enable_color:=true \
      enable_depth:=true \
      enable_left_ir:=false \
      enable_right_ir:=false \
      enable_accel:=false \
      enable_gyro:=false \
      color_width:=640 \
      color_height:=480 \
      color_fps:=30 \
      depth_width:=640 \
      depth_height:=480 \
      depth_fps:=30 \
      depth_registration:=false \
      enable_point_cloud:=false \
      enable_colored_point_cloud:=false &
    PIDS+=("$!")
  fi
fi

if [[ "${CAMERA}" == "d435i" || "${CAMERA}" == "both" ]]; then
  wait_for_topic /sensors/d435i/color/image_raw "D435i"
fi
if [[ "${CAMERA}" == "gemini" || "${CAMERA}" == "both" ]]; then
  wait_for_topic /gemini336l/color/image_raw "Gemini 336L"
fi

echo "Opening rqt_image_view. Choose one of the camera topics from its drop-down list."
rqt --standalone rqt_image_view
