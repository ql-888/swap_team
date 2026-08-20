#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
REPORT_FILE="${PIPER_GRASP_REPORT_FILE:-${PROJECT_DIR}/runtime/d435i_apriltag_ros_grasp_close_report.yaml}"
GRIPPER_OPEN_M="${PIPER_GRASP_GRIPPER_OPEN_M:-0.070}"
GRIPPER_CLOSED_M="${PIPER_GRASP_GRIPPER_CLOSED_M:-0.048}"
GRIPPER_FEEDBACK_MIN_M="${PIPER_GRASP_FEEDBACK_MIN_M:-0.050}"
GRIPPER_FEEDBACK_MAX_M="${PIPER_GRASP_FEEDBACK_MAX_M:-0.060}"

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/source_ros_env.sh"
set -u

[[ "${1:-}" == "I_CONFIRM_FULL_RETREAT_WITH_GRIP" ]] || {
  echo "Refusing motion. Required: I_CONFIRM_FULL_RETREAT_WITH_GRIP" >&2; exit 2;
}
[[ "${2:-}" == "I_CONFIRM_GRIP_IS_STABLE" ]] || {
  echo "Refusing motion. Required: I_CONFIRM_GRIP_IS_STABLE" >&2; exit 2;
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

# Read one full ROS feedback message so both arm pose and contact width can be
# checked before lifting the object.
FEEDBACK_FILE=$(mktemp /tmp/piper_retreat_feedback.XXXXXX.yaml)
trap 'rm -f "${FEEDBACK_FILE}"' EXIT
ros2 topic echo /piper_x/feedback/joint_states --once > "${FEEDBACK_FILE}"

/usr/bin/python3 - "${REPORT_FILE}" "${FEEDBACK_FILE}" "${GRIPPER_FEEDBACK_MIN_M}" "${GRIPPER_FEEDBACK_MAX_M}" <<'PY'
import sys, yaml, numpy as np
report_path, feedback_path = sys.argv[1:3]
feedback_min, feedback_max = map(float, sys.argv[3:5])
with open(report_path, encoding="utf-8") as f: report = yaml.safe_load(f)
with open(feedback_path, encoding="utf-8") as f:
    feedback = next(document for document in yaml.safe_load_all(f) if document)
positions = dict(zip(feedback["name"], feedback["position"]))
names = [f"joint{i}" for i in range(1, 7)]
current = np.rad2deg([positions[name] for name in names])
target = np.asarray(report["stages"]["grasp"]["q_deg"], dtype=float)
error = float(np.max(np.abs(current - target)))
gripper = float(positions["gripper"])
print(f"start_max_joint_error_deg: {error:.3f}")
print(f"start_gripper_feedback_m: {gripper:.4f}")
print("vision_policy: no AprilTag read; reuse validated grasp/retreat report")
if error > 1.0:
    raise SystemExit("Arm is no longer at the validated grasp position")
if not feedback_min <= gripper <= feedback_max:
    raise SystemExit("Gripper feedback no longer indicates the validated object contact")
PY

"${PROJECT_DIR}/scripts/execute_ros_pregrasp.py" \
  --report "${REPORT_FILE}" \
  --confirmation I_CONFIRM_FULL_RETREAT_WITH_GRIP \
  --mode retreat_hold \
  --gripper-open-m "${GRIPPER_OPEN_M}" \
  --gripper-closed-m "${GRIPPER_CLOSED_M}" \
  --tolerance-deg 0.5 \
  --timeout-s 30
