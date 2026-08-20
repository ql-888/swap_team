#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
POSE_FILE="${PIPER_GRASP_POSE_FILE:-${PROJECT_DIR}/runtime/d435i_apriltag_object_pose.yaml}"
REPORT_FILE="${PIPER_GRASP_REPORT_FILE:-${PROJECT_DIR}/runtime/d435i_apriltag_ros_approach50_report.yaml}"
RECIPE_FILE="${PIPER_GRASP_RECIPE_FILE:-${PROJECT_DIR}/config/grasp_recipe_apriltag_top_approach50.yaml}"
GRIPPER_OPEN_M="${PIPER_GRASP_GRIPPER_OPEN_M:-0.070}"
GRIPPER_CLOSED_M="${PIPER_GRASP_GRIPPER_CLOSED_M:-0.048}"
ALLOW_LOCKED_POSE_FALLBACK="${PIPER_GRASP_ALLOW_LOCKED_POSE_FALLBACK:-false}"
REUSE_LOCKED_POSE="${PIPER_GRASP_REUSE_LOCKED_POSE:-false}"
PREVIOUS_REPORT="${PIPER_GRASP_PREVIOUS_REPORT_FILE:-}"
mkdir -p "$(dirname -- "${POSE_FILE}")" "$(dirname -- "${REPORT_FILE}")"

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/source_ros_env.sh"
set -u

if [[ "${1:-}" != "I_CONFIRM_APPROACH_50MM_ONLY" ]]; then
    echo "Refusing motion. Required confirmation: I_CONFIRM_APPROACH_50MM_ONLY" >&2
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
if [[ "${speed}" != "5" ]]; then
    echo "Refusing motion: speed_percent must be 5, current value is ${speed}." >&2
    exit 4
fi
if [[ "${control_enabled}" != "True" ]]; then
    echo "Refusing motion: ROS external control is disabled." >&2
    exit 5
fi
if [[ "${tcp_offset}" != "0.0,0.0,0.198,0.0,0.0,0.8412486994612669" ]]; then
    echo "Refusing motion: ROS TCP does not match Z=198 mm / yaw=48.2 deg." >&2
    exit 6
fi

if [[ "${REUSE_LOCKED_POSE}" != "true" ]] && \
   "${PROJECT_DIR}/scripts/capture_d435i_apriltag_pose.py" --output "${POSE_FILE}"; then
  echo "vision_policy: fresh AprilTag pose accepted"
elif [[ "${ALLOW_LOCKED_POSE_FALLBACK}" == "true" ]]; then
  /usr/bin/python3 - "${POSE_FILE}" "${PREVIOUS_REPORT}" <<'PY'
import sys, yaml
pose_path, report_path = sys.argv[1:]
if not report_path:
    raise SystemExit("Locked-pose fallback requires the previous stage report")
with open(pose_path, encoding="utf-8") as stream:
    pose = yaml.safe_load(stream)
with open(report_path, encoding="utf-8") as stream:
    report = yaml.safe_load(stream)
capture = pose.get("capture", {})
if pose.get("object_id") != "apriltag_object" or capture.get("sample_count") != 30:
    raise SystemExit("Saved pose is not a validated 30-sample AprilTag pose")
if report.get("inputs", {}).get("object_pose") != pose_path:
    raise SystemExit("Saved pose does not match the successful previous-stage report")
if float(capture.get("translation_rms_mm", 999.0)) > 10.0:
    raise SystemExit("Saved pose translation RMS exceeds the safety limit")
if float(capture.get("rotation_rms_deg", 999.0)) > 5.0:
    raise SystemExit("Saved pose rotation RMS exceeds the safety limit")
print(f"locked_object_pose_timestamp_s: {pose['timestamp_s']}")
print(f"locked_translation_rms_mm: {capture['translation_rms_mm']:.3f}")
print(f"locked_rotation_rms_deg: {capture['rotation_rms_deg']:.3f}")
print("vision_policy: no new AprilTag read; reusing validated pregrasp pose")
PY
else
  exit 1
fi

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
  --confirmation I_CONFIRM_APPROACH_50MM_ONLY \
  --mode approach50 \
  --gripper-open-m "${GRIPPER_OPEN_M}" \
  --gripper-closed-m "${GRIPPER_CLOSED_M}" \
  --tolerance-deg 0.5 \
  --timeout-s 30
