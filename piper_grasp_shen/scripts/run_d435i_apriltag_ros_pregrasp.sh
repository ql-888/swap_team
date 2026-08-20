#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
POSE_FILE="${PIPER_GRASP_POSE_FILE:-${PROJECT_DIR}/runtime/d435i_apriltag_object_pose.yaml}"
REPORT_FILE="${PIPER_GRASP_REPORT_FILE:-${PROJECT_DIR}/runtime/d435i_apriltag_ros_pregrasp_report.yaml}"
RECIPE_FILE="${PIPER_GRASP_RECIPE_FILE:-${PROJECT_DIR}/config/grasp_recipe_apriltag_top.yaml}"
GRIPPER_OPEN_M="${PIPER_GRASP_GRIPPER_OPEN_M:-0.070}"
GRIPPER_CLOSED_M="${PIPER_GRASP_GRIPPER_CLOSED_M:-0.048}"
mkdir -p "$(dirname -- "${POSE_FILE}")" "$(dirname -- "${REPORT_FILE}")"

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/source_ros_env.sh"
set -u

if [[ "${1:-}" != "I_CONFIRM_ROS_PREGRASP_ONLY" ]]; then
    echo "Refusing motion. Required confirmation: I_CONFIRM_ROS_PREGRASP_ONLY" >&2
    exit 2
fi

if ! ros2 node list | grep -Fxq '/piper_x/agx_arm_ctrl_single_node'; then
    echo "ROS Piper control node is not running." >&2
    exit 3
fi

speed=$(ros2 param get /piper_x/agx_arm_ctrl_single_node speed_percent | awk '{print $NF}')
if [[ "${speed}" != "5" ]]; then
    echo "Refusing motion: ROS Piper speed_percent must be 5, current value is ${speed}." >&2
    echo "Restart agx_arm_ctrl with speed_percent:=5." >&2
    exit 4
fi

control_enabled=$(
  ros2 param get /piper_x/agx_arm_ctrl_single_node control_enabled | awk '{print $NF}'
)
if [[ "${control_enabled}" != "True" ]]; then
    echo "Refusing motion: ROS external control is disabled." >&2
    echo "Restart with scripts/start_piper_x_ros_control_5pct.sh." >&2
    exit 5
fi

tcp_offset=$(
  ros2 param get /piper_x/agx_arm_ctrl_single_node tcp_offset | sed -n "s/.*\[\(.*\)\].*/\1/p" | tr -d ' '
)
if [[ "${tcp_offset}" != "0.0,0.0,0.198,0.0,0.0,0.8412486994612669" ]]; then
    echo "Refusing motion: ROS TCP offset does not match Z=198 mm / yaw=48.2 deg." >&2
    echo "Current tcp_offset: ${tcp_offset}" >&2
    exit 6
fi

"${PROJECT_DIR}/scripts/capture_d435i_apriltag_pose.py" --output "${POSE_FILE}"

mapfile -t CURRENT_Q_DEG < <(
  /usr/bin/python3 - "${POSE_FILE}" <<'PY'
import sys, yaml
with open(sys.argv[1], encoding="utf-8") as stream:
    values = yaml.safe_load(stream)["current_q_deg"]
for value in values: print(value)
PY
)

"${PROJECT_DIR}/scripts/run.sh" plan-grasp \
  --object-pose "${POSE_FILE}" \
  --recipe "${RECIPE_FILE}" \
  --current-q-deg "${CURRENT_Q_DEG[@]}" \
  --minimum-tcp-z-m 0.02 \
  --report "${REPORT_FILE}"

"${PROJECT_DIR}/scripts/execute_ros_pregrasp.py" \
  --report "${REPORT_FILE}" \
  --confirmation I_CONFIRM_ROS_PREGRASP_ONLY \
  --gripper-open-m "${GRIPPER_OPEN_M}" \
  --gripper-closed-m "${GRIPPER_CLOSED_M}" \
  --tolerance-deg 0.5 \
  --timeout-s 30
