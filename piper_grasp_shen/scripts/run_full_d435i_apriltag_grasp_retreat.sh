#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
RUN_DIR="${PROJECT_DIR}/runtime/full_grasp_cycle"
LOCKED_POSE="${RUN_DIR}/locked_object_pose.yaml"
CANDIDATE_POSE="${RUN_DIR}/candidate_object_pose.yaml"

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/source_ros_env.sh"
set -u

[[ "${1:-}" == "I_CONFIRM_FULL_VISION_GRASP_RETREAT" ]] || {
  echo "Refusing motion. Required: I_CONFIRM_FULL_VISION_GRASP_RETREAT" >&2; exit 2;
}
[[ "${2:-}" == "I_HAVE_CLEARED_THE_WORKSPACE" ]] || {
  echo "Refusing motion. Required: I_HAVE_CLEARED_THE_WORKSPACE" >&2; exit 2;
}

mkdir -p "${RUN_DIR}"

require_ros_configuration() {
  ros2 node list | grep -Fxq '/piper_x/agx_arm_ctrl_single_node' || {
    echo "ROS Piper control node is not running." >&2; exit 3;
  }
  local speed control_enabled tcp_offset
  speed=$(ros2 param get /piper_x/agx_arm_ctrl_single_node speed_percent | awk '{print $NF}')
  control_enabled=$(ros2 param get /piper_x/agx_arm_ctrl_single_node control_enabled | awk '{print $NF}')
  tcp_offset=$(ros2 param get /piper_x/agx_arm_ctrl_single_node tcp_offset | sed -n "s/.*\[\(.*\)\].*/\1/p" | tr -d ' ')
  [[ "${speed}" == "5" ]] || { echo "Refusing motion: speed_percent must be 5." >&2; exit 4; }
  [[ "${control_enabled}" == "True" ]] || { echo "Refusing motion: external control is disabled." >&2; exit 5; }
  [[ "${tcp_offset}" == "0.0,0.0,0.198,0.0,0.0,0.8412486994612669" ]] || {
    echo "Refusing motion: ROS TCP mismatch." >&2; exit 6;
  }
}

read_current_q() {
  mapfile -t CURRENT_Q_DEG < <("${PROJECT_DIR}/scripts/read_ros_joint_degrees.py")
  [[ ${#CURRENT_Q_DEG[@]} -eq 6 ]] || {
    echo "Failed to read all six current joint angles." >&2; exit 7;
  }
}

validate_candidate_jump() {
  /usr/bin/python3 - "${LOCKED_POSE}" "${CANDIDATE_POSE}" <<'PY'
import sys, yaml, numpy as np, math
with open(sys.argv[1], encoding="utf-8") as f: old = yaml.safe_load(f)
with open(sys.argv[2], encoding="utf-8") as f: new = yaml.safe_load(f)
A = np.asarray(old["frame_from_object"], dtype=float)
B = np.asarray(new["frame_from_object"], dtype=float)
translation_mm = float(np.linalg.norm(A[:3, 3] - B[:3, 3]) * 1000.0)
relative = A[:3, :3].T @ B[:3, :3]
rotation_deg = math.degrees(math.acos(float(np.clip((np.trace(relative)-1.0)/2.0, -1.0, 1.0))))
print(f"candidate_translation_jump_mm: {translation_mm:.3f}")
print(f"candidate_rotation_jump_deg: {rotation_deg:.3f}")
if translation_mm > 15.0 or rotation_deg > 5.0:
    raise SystemExit("Candidate pose jump exceeds 15 mm / 5 deg")
PY
}

try_refresh_pose() {
  local stage=$1
  echo "VISION_REFRESH_BEGIN: ${stage}"
  if "${PROJECT_DIR}/scripts/capture_d435i_apriltag_pose.py" \
      --timeout-s 5.0 --output "${CANDIDATE_POSE}"; then
    if validate_candidate_jump; then
      cp -- "${CANDIDATE_POSE}" "${LOCKED_POSE}"
      echo "VISION_REFRESH_ACCEPTED: ${stage}"
    else
      echo "VISION_REFRESH_REJECTED: ${stage}; reusing previous locked pose"
    fi
  else
    echo "VISION_REFRESH_UNAVAILABLE: ${stage}; reusing previous locked pose"
  fi
}

plan_stage() {
  local recipe=$1 report=$2
  read_current_q
  "${PROJECT_DIR}/scripts/run.sh" plan-grasp \
    --object-pose "${LOCKED_POSE}" \
    --recipe "${recipe}" \
    --current-q-deg "${CURRENT_Q_DEG[@]}" \
    --minimum-tcp-z-m 0.02 \
    --report "${report}"
}

execute_open_stage() {
  local report=$1 mode=$2 confirmation=$3
  "${PROJECT_DIR}/scripts/execute_ros_pregrasp.py" \
    --report "${report}" \
    --confirmation "${confirmation}" \
    --mode "${mode}" \
    --tolerance-deg 0.5 \
    --timeout-s 30
}

require_ros_configuration

echo "STAGE 1/8: mandatory initial AprilTag capture"
"${PROJECT_DIR}/scripts/capture_d435i_apriltag_pose.py" \
  --timeout-s 12.0 --output "${LOCKED_POSE}"

echo "STAGE 2/8: plan and move to 100 mm pregrasp"
REPORT_100="${RUN_DIR}/stage_100mm.yaml"
plan_stage "${PROJECT_DIR}/config/grasp_recipe_apriltag_top.yaml" "${REPORT_100}"
execute_open_stage "${REPORT_100}" pregrasp I_CONFIRM_ROS_PREGRASP_ONLY

echo "STAGE 3/8: refresh vision if visible, then move to 50 mm"
try_refresh_pose pregrasp_100mm
REPORT_50="${RUN_DIR}/stage_50mm.yaml"
plan_stage "${PROJECT_DIR}/config/grasp_recipe_apriltag_top_approach50.yaml" "${REPORT_50}"
execute_open_stage "${REPORT_50}" approach50 I_CONFIRM_APPROACH_50MM_ONLY

echo "STAGE 4/8: refresh vision if visible, then move to 25 mm"
try_refresh_pose approach_50mm
REPORT_25="${RUN_DIR}/stage_25mm.yaml"
plan_stage "${PROJECT_DIR}/config/grasp_recipe_apriltag_top_approach25.yaml" "${REPORT_25}"
execute_open_stage "${REPORT_25}" approach25 I_CONFIRM_APPROACH_25MM_ONLY

echo "STAGE 5/8: final vision attempt and final locked-pose plan"
try_refresh_pose approach_25mm
REPORT_FINAL="${RUN_DIR}/stage_final_grasp_retreat.yaml"
plan_stage "${PROJECT_DIR}/config/grasp_recipe_apriltag_top_approach25.yaml" "${REPORT_FINAL}"

echo "STAGE 6/8: final 25 mm approach and close to 48 mm"
"${PROJECT_DIR}/scripts/execute_ros_pregrasp.py" \
  --report "${REPORT_FINAL}" \
  --confirmation I_CONFIRM_FINAL_25MM_AND_CLOSE \
  --mode grasp_close \
  --tolerance-deg 0.5 \
  --timeout-s 30

echo "STAGE 7/8: verify physical contact feedback"
GRIPPER_FEEDBACK=$("${PROJECT_DIR}/scripts/read_ros_gripper_m.py")
/usr/bin/python3 - "${GRIPPER_FEEDBACK}" <<'PY'
import sys
value = float(sys.argv[1])
print(f"verified_gripper_feedback_m: {value:.4f}")
if not 0.050 <= value <= 0.060:
    raise SystemExit("Gripper feedback does not indicate the expected object contact; refusing retreat")
PY

echo "STAGE 8/8: hold grip and execute full 100 mm retreat"
"${PROJECT_DIR}/scripts/execute_ros_pregrasp.py" \
  --report "${REPORT_FINAL}" \
  --confirmation I_CONFIRM_FULL_RETREAT_WITH_GRIP \
  --mode retreat_hold \
  --tolerance-deg 0.5 \
  --timeout-s 30

echo "FULL_D435I_APRILTAG_GRASP_RETREAT_COMPLETE"
echo "The object remains gripped. No release command was sent."
