#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd -- "${SCRIPT_DIR}/../.." && pwd)
source "${PROJECT_DIR}/scripts/source_ros_env.sh"

for node in /piper_x/agx_arm_ctrl_single_node /piper_x/robot_state_publisher \
  /perception/gemini336l/gemini336l_drone_apriltag; do
  ros2 node list 2>/dev/null | grep -Fxq "${node}" || {
    echo "MISSING_NODE: ${node}" >&2; exit 2;
  }
  echo "READY_NODE: ${node}"
done

topic_ready=false
for _ in $(seq 1 10); do
  if ros2 topic info /gemini336l/color/image_raw 2>/dev/null | grep -Eq 'Publisher count: [1-9]'; then
    topic_ready=true
    break
  fi
  sleep 1
done
[[ "${topic_ready}" == "true" ]] || {
  echo "MISSING_TOPIC_PUBLISHER: /gemini336l/color/image_raw" >&2; exit 3;
}
[[ -f "${PROJECT_DIR}/calibration_data/gemini336l_eye_to_hand.json" ]] || {
  echo "MISSING_CALIBRATION: calibration_data/gemini336l_eye_to_hand.json" >&2; exit 4;
}
[[ -f "${PROJECT_DIR}/config/handeye_orbbec_fixed.yaml" ]] || {
  echo "MISSING_CALIBRATION: config/handeye_orbbec_fixed.yaml" >&2; exit 4;
}

speed=$(ros2 param get /piper_x/agx_arm_ctrl_single_node speed_percent | awk '{print $NF}')
control=$(ros2 param get /piper_x/agx_arm_ctrl_single_node control_enabled | awk '{print $NF}')
family=$(ros2 param get /perception/gemini336l/gemini336l_drone_apriltag family | awk '{print $NF}')
size=$(ros2 param get /perception/gemini336l/gemini336l_drone_apriltag size | awk '{print $NF}')
[[ "${speed}" == "5" ]] || { echo "BAD_SPEED_PERCENT: ${speed}" >&2; exit 5; }
[[ "${control}" == "True" ]] || { echo "CONTROL_DISABLED: ${control}" >&2; exit 5; }
[[ "${family}" == "36h11" && "${size}" == "0.029" ]] || {
  echo "BAD_TAG_CONFIG: family=${family} size=${size}" >&2; exit 5;
}
echo "speed_percent: ${speed}"
echo "control_enabled: ${control}"
echo "tag_family: ${family}"
echo "tag_size_m: ${size}"
echo "EYE_TO_GRASP_STATUS_READY"
