#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
POSE_FILE="${PIPER_GRASP_POSE_FILE:-${PROJECT_DIR}/runtime/d435i_apriltag_object_pose.yaml}"
REPORT_FILE="${PIPER_GRASP_REPORT_FILE:-${PROJECT_DIR}/runtime/d435i_apriltag_grasp_plan.yaml}"
RECIPE_FILE="${PIPER_GRASP_RECIPE_FILE:-${PROJECT_DIR}/config/grasp_recipe_apriltag_top.yaml}"
mkdir -p "$(dirname -- "${POSE_FILE}")" "$(dirname -- "${REPORT_FILE}")"

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/source_ros_env.sh"
set -u

"${PROJECT_DIR}/scripts/capture_d435i_apriltag_pose.py" --output "${POSE_FILE}"

mapfile -t CURRENT_Q_DEG < <(
  /usr/bin/python3 - "${POSE_FILE}" <<'PY'
import sys
import yaml
with open(sys.argv[1], encoding="utf-8") as stream:
    values = yaml.safe_load(stream)["current_q_deg"]
for value in values:
    print(value)
PY
)

"${PROJECT_DIR}/scripts/run.sh" plan-grasp \
  --object-pose "${POSE_FILE}" \
  --recipe "${RECIPE_FILE}" \
  --current-q-deg "${CURRENT_Q_DEG[@]}" \
  --minimum-tcp-z-m 0.02 \
  --report "${REPORT_FILE}"

echo "Live D435i pose was captured and an offline plan was validated."
echo "No CAN connection, gripper command, enable command, or arm motion was sent."
