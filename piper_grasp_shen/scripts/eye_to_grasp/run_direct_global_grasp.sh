#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd -- "${SCRIPT_DIR}/../.." && pwd)
RUN_DIR="${PROJECT_DIR}/runtime/eye_to_grasp"
POSE_FILE="${RUN_DIR}/gemini336l_drone_locked_pose.yaml"
REPORT_FILE="${RUN_DIR}/gemini336l_drone_direct_grasp_plan.yaml"
RECIPE_FILE="${PROJECT_DIR}/config/grasp_recipe_drone_apriltag_top.yaml"
CAPTURE_SCRIPT="${SCRIPT_DIR}/capture_global_apriltag_pose.py"

source "${PROJECT_DIR}/scripts/source_ros_env.sh"
set -u

require_ros_configuration() {
  ros2 node list | grep -Fxq '/piper_x/agx_arm_ctrl_single_node' || {
    echo "ROS Piper control node is not running." >&2; exit 3;
  }
  local speed control_enabled tcp_offset
  speed=$(ros2 param get /piper_x/agx_arm_ctrl_single_node speed_percent | awk '{print $NF}')
  control_enabled=$(ros2 param get /piper_x/agx_arm_ctrl_single_node control_enabled | awk '{print $NF}')
  tcp_offset=$(ros2 param get /piper_x/agx_arm_ctrl_single_node tcp_offset | sed -n 's/.*\[\(.*\)\].*/\1/p' | tr -d ' ')
  [[ "${speed}" == "5" ]] || { echo "Refusing motion: speed_percent must be 5 (got ${speed})." >&2; exit 4; }
  [[ "${control_enabled}" == "True" ]] || { echo "Refusing motion: external control is disabled." >&2; exit 5; }
  [[ "${tcp_offset}" == "0.0,0.0,0.198,0.0,0.0,0.8412486994612669" ]] || {
    echo "Refusing motion: ROS TCP mismatch: ${tcp_offset}" >&2; exit 6;
  }
}

require_vision_configuration() {
  ros2 node list 2>/dev/null | grep -Fxq '/gemini336l/gemini336l' || {
    echo "Gemini 336L camera node is not running." >&2; exit 7;
  }
  ros2 topic list 2>/dev/null | grep -Fxq '/gemini336l/color/image_raw' || {
    echo "Gemini 336L color topic is not advertised." >&2; exit 7;
  }
  ros2 node list | grep -Fxq '/perception/gemini336l/gemini336l_drone_apriltag' || {
    echo "Gemini drone AprilTag node is not running." >&2; exit 8;
  }
  [[ -f "${PROJECT_DIR}/calibration_data/gemini336l_eye_to_hand.json" ]] || {
    echo "Missing Gemini calibration JSON." >&2; exit 9;
  }
  [[ -f "${PROJECT_DIR}/config/handeye_orbbec_fixed.yaml" ]] || {
    echo "Missing normalized Gemini hand-eye calibration." >&2; exit 9;
  }
}

preflight() {
  require_ros_configuration
  require_vision_configuration
  [[ -f "${RECIPE_FILE}" ]] || { echo "Missing drone grasp recipe: ${RECIPE_FILE}" >&2; exit 10; }
  echo "EYE_TO_GRASP_PREFLIGHT_READY"
  echo "camera: Orbbec Gemini 336L"
  echo "tag: tag36h11 ID 0, 29 mm"
  echo "policy: one global capture, direct grasp, no wrist-camera correction"
  echo "No motion command was sent."
}

if [[ "${1:-}" == "--preflight" ]]; then
  preflight
  exit 0
fi

[[ "${1:-}" == "I_CONFIRM_EYE_TO_GRASP_DIRECT" ]] || {
  echo "Refusing motion. Required: I_CONFIRM_EYE_TO_GRASP_DIRECT" >&2; exit 2;
}
[[ "${2:-}" == "I_HAVE_CLEARED_THE_WORKSPACE" ]] || {
  echo "Refusing motion. Required: I_HAVE_CLEARED_THE_WORKSPACE" >&2; exit 2;
}
[[ "${3:-}" == "I_CONFIRM_DRONE_AND_TAG_FIXED" ]] || {
  echo "Refusing motion. Required: I_CONFIRM_DRONE_AND_TAG_FIXED" >&2; exit 2;
}

mkdir -p "${RUN_DIR}"
require_ros_configuration
require_vision_configuration

echo "STAGE 1/4: capture Gemini global pose once (10 stable samples)"
/usr/bin/python3 "${CAPTURE_SCRIPT}" \
  --base-frame piper_x/base_link \
  --tag-frame gemini_apriltag_0 \
  --samples 10 \
  --timeout-s 30 \
  --output "${POSE_FILE}"

echo "STAGE 2/4: plan pregrasp, grasp, and retreat from the locked pose"
mapfile -t CURRENT_Q_DEG < <(
  /usr/bin/python3 - "${POSE_FILE}" <<'PY'
import sys, yaml
with open(sys.argv[1], encoding="utf-8") as stream:
    values = yaml.safe_load(stream)["current_q_deg"]
for value in values:
    print(value)
PY
)
[[ ${#CURRENT_Q_DEG[@]} -eq 6 ]] || { echo "Captured pose has no six-joint seed." >&2; exit 11; }
"${PROJECT_DIR}/scripts/run.sh" plan-grasp \
  --object-pose "${POSE_FILE}" \
  --recipe "${RECIPE_FILE}" \
  --current-q-deg "${CURRENT_Q_DEG[@]}" \
  --minimum-tcp-z-m 0.02 \
  --report "${REPORT_FILE}"

echo "STAGE 3/4: move directly to pregrasp, then descend and close"
"${PROJECT_DIR}/scripts/execute_ros_pregrasp.py" \
  --report "${REPORT_FILE}" \
  --confirmation I_CONFIRM_ROS_PREGRASP_ONLY \
  --mode pregrasp \
  --gripper-open-m 0.085 \
  --gripper-closed-m 0.058 \
  --tolerance-deg 0.5 \
  --timeout-s 45
"${PROJECT_DIR}/scripts/execute_ros_pregrasp.py" \
  --report "${REPORT_FILE}" \
  --confirmation I_CONFIRM_DIRECT_100MM_AND_CLOSE \
  --mode direct_grasp_close \
  --gripper-open-m 0.085 \
  --gripper-closed-m 0.058 \
  --tolerance-deg 0.5 \
  --timeout-s 35

echo "Checking gripper contact before retreat"
GRIPPER_FEEDBACK=$("${PROJECT_DIR}/scripts/read_ros_gripper_m.py")
/usr/bin/python3 - "${GRIPPER_FEEDBACK}" <<'PY'
import sys
value = float(sys.argv[1])
print(f"verified_gripper_feedback_m: {value:.4f}")
if not 0.058 <= value <= 0.085:
    raise SystemExit("Gripper feedback is outside the expected drone-contact range (58--85 mm)")
PY

echo "STAGE 4/4: retreat while holding the closed command"
"${PROJECT_DIR}/scripts/execute_ros_pregrasp.py" \
  --report "${REPORT_FILE}" \
  --confirmation I_CONFIRM_FULL_RETREAT_WITH_GRIP \
  --mode retreat_hold \
  --gripper-open-m 0.085 \
  --gripper-closed-m 0.058 \
  --tolerance-deg 0.5 \
  --timeout-s 35

echo "EYE_TO_GRASP_DIRECT_GRASP_RETREAT_COMPLETE"
echo "The drone remains gripped. No release command was sent."
