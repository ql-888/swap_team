#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd -- "${SCRIPT_DIR}/../.." && pwd)
RUN_DIR="${PROJECT_DIR}/runtime/eye_to_grasp"
POSE_FILE="${RUN_DIR}/gemini336l_hover_locked_pose.yaml"
RECIPE_FILE="${RUN_DIR}/gemini336l_hover_15cm_recipe.yaml"
REPORT_FILE="${RUN_DIR}/gemini336l_hover_15cm_report.yaml"

[[ "${1:-}" == "I_CONFIRM_EYE_TO_GRASP_HOVER_15CM" ]] || {
  echo "Refusing motion. Required: I_CONFIRM_EYE_TO_GRASP_HOVER_15CM" >&2; exit 2;
}
[[ "${2:-}" == "I_HAVE_CLEARED_THE_WORKSPACE" ]] || {
  echo "Refusing motion. Required: I_HAVE_CLEARED_THE_WORKSPACE" >&2; exit 2;
}
[[ "${3:-}" == "I_CONFIRM_DRONE_AND_TAG_FIXED" ]] || {
  echo "Refusing motion. Required: I_CONFIRM_DRONE_AND_TAG_FIXED" >&2; exit 2;
}

source "${PROJECT_DIR}/scripts/source_ros_env.sh"
mkdir -p "${RUN_DIR}"

echo "STAGE 1/3: capture Gemini 336L tag pose"
python3 "${SCRIPT_DIR}/capture_global_apriltag_pose.py" \
  --base-frame piper_x/base_link --tag-frame gemini_apriltag_0 \
  --samples 10 --timeout-s 30 --output "${POSE_FILE}"

echo "STAGE 2/3: build eye-in-grasp posture at 150 mm above tag"
"${PROJECT_DIR}/.conda/bin/python" "${SCRIPT_DIR}/make_global_hover_recipe.py" \
  --pose "${POSE_FILE}" --height-m 0.15 --output "${RECIPE_FILE}"
mapfile -t CURRENT_Q_DEG < <("${PROJECT_DIR}/.conda/bin/python" - "${POSE_FILE}" <<'PY'
import sys, yaml
with open(sys.argv[1], encoding="utf-8") as stream:
    values = yaml.safe_load(stream)["current_q_deg"]
for value in values:
    print(value)
PY
)
"${PROJECT_DIR}/scripts/run.sh" plan-grasp \
  --object-pose "${POSE_FILE}" --recipe "${RECIPE_FILE}" \
  --current-q-deg "${CURRENT_Q_DEG[@]}" --minimum-tcp-z-m 0.02 \
  --report "${REPORT_FILE}"

echo "STAGE 3/3: execute pregrasp only with the eye-in-grasp ROS executor"
"${PROJECT_DIR}/scripts/execute_ros_pregrasp.py" \
  --report "${REPORT_FILE}" --confirmation I_CONFIRM_ROS_PREGRASP_ONLY \
  --mode pregrasp --gripper-open-m 0.085 --gripper-closed-m 0.058 \
  --tolerance-deg 0.5 --timeout-s 45
echo "GLOBAL_HOVER_15CM_COMPLETE"
