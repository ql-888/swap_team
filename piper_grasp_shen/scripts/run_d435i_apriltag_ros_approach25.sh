#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
POSE_FILE="${PIPER_GRASP_POSE_FILE:-${PROJECT_DIR}/runtime/d435i_apriltag_object_pose.yaml}"
REPORT_FILE="${PIPER_GRASP_REPORT_FILE:-${PROJECT_DIR}/runtime/d435i_apriltag_ros_approach25_report.yaml}"
APPROACH50_REPORT="${PIPER_GRASP_PREVIOUS_REPORT_FILE:-${PROJECT_DIR}/runtime/d435i_apriltag_ros_approach50_report.yaml}"
RECIPE_FILE="${PIPER_GRASP_RECIPE_FILE:-${PROJECT_DIR}/config/grasp_recipe_apriltag_top_approach25.yaml}"
GRIPPER_OPEN_M="${PIPER_GRASP_GRIPPER_OPEN_M:-0.070}"
GRIPPER_CLOSED_M="${PIPER_GRASP_GRIPPER_CLOSED_M:-0.048}"
mkdir -p "$(dirname -- "${POSE_FILE}")" "$(dirname -- "${REPORT_FILE}")"

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/source_ros_env.sh"
set -u

if [[ "${1:-}" != "I_CONFIRM_APPROACH_25MM_ONLY" ]]; then
    echo "Refusing motion. Required confirmation: I_CONFIRM_APPROACH_25MM_ONLY" >&2
    exit 2
fi
if [[ "${2:-}" != "I_CONFIRM_OBJECT_AND_TAG_NOT_MOVED" ]]; then
    echo "Refusing motion. The saved pose may only be reused after confirming:" >&2
    echo "I_CONFIRM_OBJECT_AND_TAG_NOT_MOVED" >&2
    exit 2
fi
if ! ros2 node list | grep -Fxq '/piper_x/agx_arm_ctrl_single_node'; then
    echo "ROS Piper control node is not running." >&2
    exit 3
fi

speed=$(ros2 param get /piper_x/agx_arm_ctrl_single_node speed_percent | awk '{print $NF}')
control_enabled=$(ros2 param get /piper_x/agx_arm_ctrl_single_node control_enabled | awk '{print $NF}')
tcp_offset=$(
  ros2 param get /piper_x/agx_arm_ctrl_single_node tcp_offset |
    sed -n "s/.*\[\(.*\)\].*/\1/p" | tr -d ' '
)
[[ "${speed}" == "5" ]] || { echo "Refusing motion: speed_percent must be 5." >&2; exit 4; }
[[ "${control_enabled}" == "True" ]] || { echo "Refusing motion: external control is disabled." >&2; exit 5; }
[[ "${tcp_offset}" == "0.0,0.0,0.198,0.0,0.0,0.8412486994612669" ]] || {
  echo "Refusing motion: ROS TCP does not match Z=198 mm / yaw=48.2 deg." >&2
  exit 6
}

# The wrist camera can no longer see the tag at this distance.  Deliberately
# reuse the last pose captured by the successful 50 mm inspection stage.  Do
# not call the AprilTag capture program here and do not overwrite POSE_FILE.
/usr/bin/python3 - "${POSE_FILE}" "${APPROACH50_REPORT}" <<'PY'
import sys, yaml
pose_path, report_path = sys.argv[1:]
with open(pose_path, encoding="utf-8") as stream:
    pose = yaml.safe_load(stream)
with open(report_path, encoding="utf-8") as stream:
    report = yaml.safe_load(stream)
capture = pose.get("capture", {})
if pose.get("object_id") != "apriltag_object" or capture.get("sample_count") != 30:
    raise SystemExit("Saved pose is not a validated 30-sample AprilTag object pose")
if float(capture.get("translation_rms_mm", 999)) > 10.0:
    raise SystemExit("Saved pose translation RMS exceeds the safety limit")
if float(capture.get("rotation_rms_deg", 999)) > 5.0:
    raise SystemExit("Saved pose rotation RMS exceeds the safety limit")
if report.get("inputs", {}).get("object_pose") != pose_path:
    raise SystemExit("The saved pose does not match the successful approach50 report")
print(f"locked_object_pose_timestamp_s: {pose['timestamp_s']}")
print(f"locked_translation_rms_mm: {capture['translation_rms_mm']:.3f}")
print(f"locked_rotation_rms_deg: {capture['rotation_rms_deg']:.3f}")
print("vision_policy: reuse locked approach50 pose; no new AprilTag read")
PY

mapfile -t CURRENT_Q_DEG < <("${PROJECT_DIR}/scripts/read_ros_joint_degrees.py")
if [[ ${#CURRENT_Q_DEG[@]} -ne 6 ]]; then
    echo "Failed to read all six current joint angles." >&2
    exit 7
fi

"${PROJECT_DIR}/scripts/run.sh" plan-grasp \
  --object-pose "${POSE_FILE}" \
  --recipe "${RECIPE_FILE}" \
  --current-q-deg "${CURRENT_Q_DEG[@]}" \
  --minimum-tcp-z-m 0.02 \
  --report "${REPORT_FILE}"

"${PROJECT_DIR}/scripts/execute_ros_pregrasp.py" \
  --report "${REPORT_FILE}" \
  --confirmation I_CONFIRM_APPROACH_25MM_ONLY \
  --mode approach25 \
  --gripper-open-m "${GRIPPER_OPEN_M}" \
  --gripper-closed-m "${GRIPPER_CLOSED_M}" \
  --tolerance-deg 0.5 \
  --timeout-s 30
