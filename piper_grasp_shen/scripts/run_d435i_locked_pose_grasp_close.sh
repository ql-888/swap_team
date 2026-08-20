#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
POSE_FILE="${PIPER_GRASP_POSE_FILE:-${PROJECT_DIR}/runtime/d435i_apriltag_object_pose.yaml}"
APPROACH25_REPORT="${PIPER_GRASP_PREVIOUS_REPORT_FILE:-${PROJECT_DIR}/runtime/d435i_apriltag_ros_approach25_report.yaml}"
REPORT_FILE="${PIPER_GRASP_REPORT_FILE:-${PROJECT_DIR}/runtime/d435i_apriltag_ros_grasp_close_report.yaml}"
RECIPE_FILE="${PIPER_GRASP_RECIPE_FILE:-${PROJECT_DIR}/config/grasp_recipe_apriltag_top_approach25.yaml}"
GRIPPER_OPEN_M="${PIPER_GRASP_GRIPPER_OPEN_M:-0.070}"
GRIPPER_CLOSED_M="${PIPER_GRASP_GRIPPER_CLOSED_M:-0.048}"
mkdir -p "$(dirname -- "${REPORT_FILE}")"

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/source_ros_env.sh"
set -u

[[ "${1:-}" == "I_CONFIRM_FINAL_25MM_AND_CLOSE" ]] || {
  echo "Refusing motion. Required: I_CONFIRM_FINAL_25MM_AND_CLOSE" >&2; exit 2;
}
[[ "${2:-}" == "I_CONFIRM_OBJECT_AND_TAG_NOT_MOVED" ]] || {
  echo "Refusing motion. Required: I_CONFIRM_OBJECT_AND_TAG_NOT_MOVED" >&2; exit 2;
}
ros2 node list | grep -Fxq '/piper_x/agx_arm_ctrl_single_node' || {
  echo "ROS Piper control node is not running." >&2; exit 3;
}

speed=$(ros2 param get /piper_x/agx_arm_ctrl_single_node speed_percent | awk '{print $NF}')
control_enabled=$(ros2 param get /piper_x/agx_arm_ctrl_single_node control_enabled | awk '{print $NF}')
tcp_offset=$(ros2 param get /piper_x/agx_arm_ctrl_single_node tcp_offset | sed -n "s/.*\[\(.*\)\].*/\1/p" | tr -d ' ')
[[ "${speed}" == "5" ]] || { echo "Refusing motion: speed_percent must be 5." >&2; exit 4; }
[[ "${control_enabled}" == "True" ]] || { echo "Refusing motion: external control is disabled." >&2; exit 5; }
[[ "${tcp_offset}" == "0.0,0.0,0.198,0.0,0.0,0.8412486994612669" ]] || {
  echo "Refusing motion: ROS TCP mismatch." >&2; exit 6;
}

mapfile -t CURRENT_Q_DEG < <("${PROJECT_DIR}/scripts/read_ros_joint_degrees.py")
[[ ${#CURRENT_Q_DEG[@]} -eq 6 ]] || { echo "Failed to read six joints." >&2; exit 7; }

# Require the arm to still be at the successful 25 mm inspection target and
# validate that the locked pose belongs to the same continuation chain.
/usr/bin/python3 - "${POSE_FILE}" "${APPROACH25_REPORT}" "${CURRENT_Q_DEG[@]}" <<'PY'
import sys, yaml, numpy as np
pose_path, report_path = sys.argv[1], sys.argv[2]
current = np.asarray([float(x) for x in sys.argv[3:9]])
with open(pose_path, encoding="utf-8") as f: pose = yaml.safe_load(f)
with open(report_path, encoding="utf-8") as f: report = yaml.safe_load(f)
target = np.asarray(report["stages"]["pregrasp"]["q_deg"], dtype=float)
if report.get("inputs", {}).get("object_pose") != pose_path:
    raise SystemExit("Locked pose and approach25 report do not match")
error = float(np.max(np.abs(current - target)))
print(f"locked_object_pose_timestamp_s: {pose['timestamp_s']}")
print(f"start_max_joint_error_deg: {error:.3f}")
print("vision_policy: reuse locked pose; no new AprilTag read")
if error > 1.0:
    raise SystemExit("Arm is no longer at the verified 25 mm inspection position")
PY

"${PROJECT_DIR}/scripts/run.sh" plan-grasp \
  --object-pose "${POSE_FILE}" \
  --recipe "${RECIPE_FILE}" \
  --current-q-deg "${CURRENT_Q_DEG[@]}" \
  --minimum-tcp-z-m 0.02 \
  --report "${REPORT_FILE}"

"${PROJECT_DIR}/scripts/execute_ros_pregrasp.py" \
  --report "${REPORT_FILE}" \
  --confirmation I_CONFIRM_FINAL_25MM_AND_CLOSE \
  --mode grasp_close \
  --gripper-open-m "${GRIPPER_OPEN_M}" \
  --gripper-closed-m "${GRIPPER_CLOSED_M}" \
  --tolerance-deg 0.5 \
  --timeout-s 30
