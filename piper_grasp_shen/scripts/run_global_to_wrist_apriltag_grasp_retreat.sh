#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
RUN_DIR="${PROJECT_DIR}/runtime/global_to_wrist_grasp_cycle"
GLOBAL_POSE="${RUN_DIR}/gemini_locked_object_pose.yaml"
WRIST_POSE="${RUN_DIR}/d435i_locked_object_pose.yaml"
CANDIDATE_POSE="${RUN_DIR}/d435i_candidate_object_pose.yaml"

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/source_ros_env.sh"
set -u

[[ "${1:-}" == "I_CONFIRM_GLOBAL_TO_WRIST_FULL_CYCLE" ]] || {
  echo "Refusing motion. Required: I_CONFIRM_GLOBAL_TO_WRIST_FULL_CYCLE" >&2; exit 2;
}
[[ "${2:-}" == "I_HAVE_CLEARED_THE_WORKSPACE" ]] || {
  echo "Refusing motion. Required: I_HAVE_CLEARED_THE_WORKSPACE" >&2; exit 2;
}
[[ "${3:-}" == "I_CONFIRM_OBJECT_STAYS_FIXED_DURING_RUN" ]] || {
  echo "Refusing motion. Required: I_CONFIRM_OBJECT_STAYS_FIXED_DURING_RUN" >&2; exit 2;
}

RESUME_FROM_WRIST=false
if [[ "${4:-}" == "I_CONFIRM_RESUME_FROM_D435I_OBSERVATION" ]]; then
  RESUME_FROM_WRIST=true
elif [[ -n "${4:-}" ]]; then
  echo "Refusing motion. Unknown fourth confirmation token." >&2
  exit 2
fi

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

plan_stage() {
  local pose=$1 recipe=$2 report=$3
  read_current_q
  "${PROJECT_DIR}/scripts/run.sh" plan-grasp \
    --object-pose "${pose}" \
    --recipe "${recipe}" \
    --current-q-deg "${CURRENT_Q_DEG[@]}" \
    --minimum-tcp-z-m 0.02 \
    --report "${report}"
}

validate_global_pregrasp_margin() {
  local report=$1
  /usr/bin/python3 - "${report}" <<'PY'
import sys, yaml
with open(sys.argv[1], encoding="utf-8") as stream:
    report = yaml.safe_load(stream)
j1 = float(report["stages"]["pregrasp"]["q_deg"][0])
print(f"global_pregrasp_j1_deg: {j1:.3f}")
print(f"global_pregrasp_j1_limit_margin_deg: {150.0 - abs(j1):.3f}")
if abs(j1) > 145.0:
    raise SystemExit(
        "Global pregrasp would place J1 within 5 deg of its limit; "
        "reposition the object or use a different observation pose"
    )
PY
}

validate_wrist_resume_position() {
  local report="${RUN_DIR}/stage_global_centered_camera_observation.yaml"
  [[ -f "${report}" ]] || {
    echo "Cannot resume: centered D435i observation report is missing." >&2; exit 8;
  }
  read_current_q
  /usr/bin/python3 - "${report}" "${CURRENT_Q_DEG[@]}" <<'PY'
import sys, yaml, numpy as np
report_path = sys.argv[1]
current = np.asarray([float(value) for value in sys.argv[2:8]], dtype=float)
with open(report_path, encoding="utf-8") as stream:
    report = yaml.safe_load(stream)
target = np.asarray(report["stages"]["pregrasp"]["q_deg"], dtype=float)
error = float(np.max(np.abs(current - target)))
print(f"resume_max_joint_error_deg: {error:.3f}")
if error > 1.0:
    raise SystemExit(
        "Cannot resume: arm is no longer at the validated D435i observation pose"
    )
PY
}

execute_open_stage() {
  local report=$1 mode=$2 confirmation=$3 timeout=$4
  "${PROJECT_DIR}/scripts/execute_ros_pregrasp.py" \
    --report "${report}" \
    --confirmation "${confirmation}" \
    --mode "${mode}" \
    --tolerance-deg 0.5 \
    --timeout-s "${timeout}"
}

validate_wrist_candidate_jump() {
  /usr/bin/python3 - "${WRIST_POSE}" "${CANDIDATE_POSE}" <<'PY'
import math, sys, yaml
import numpy as np
with open(sys.argv[1], encoding="utf-8") as stream:
    old = yaml.safe_load(stream)
with open(sys.argv[2], encoding="utf-8") as stream:
    new = yaml.safe_load(stream)
A = np.asarray(old["frame_from_object"], dtype=float)
B = np.asarray(new["frame_from_object"], dtype=float)
translation_mm = float(np.linalg.norm(A[:3, 3] - B[:3, 3]) * 1000.0)
relative = A[:3, :3].T @ B[:3, :3]
rotation_deg = math.degrees(math.acos(float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))))
print(f"candidate_translation_jump_mm: {translation_mm:.3f}")
print(f"candidate_rotation_jump_deg: {rotation_deg:.3f}")
if translation_mm > 15.0 or rotation_deg > 5.0:
    raise SystemExit("Candidate wrist pose jump exceeds 15 mm / 5 deg")
PY
}

try_refresh_wrist_pose() {
  local stage=$1
  echo "VISION_REFRESH_BEGIN: ${stage}"
  if "${PROJECT_DIR}/scripts/capture_d435i_apriltag_pose.py" \
      --timeout-s 5.0 --output "${CANDIDATE_POSE}"; then
    if validate_wrist_candidate_jump; then
      cp -- "${CANDIDATE_POSE}" "${WRIST_POSE}"
      echo "VISION_REFRESH_ACCEPTED: ${stage}"
    else
      echo "VISION_REFRESH_REJECTED: ${stage}; reusing previous wrist pose"
    fi
  else
    echo "VISION_REFRESH_UNAVAILABLE: ${stage}; reusing previous wrist pose"
  fi
}

require_ros_configuration

if [[ "${RESUME_FROM_WRIST}" == true ]]; then
  echo "STAGES 1-3/10: resume requested; verify existing D435i observation pose"
  validate_wrist_resume_position
else
  echo "STAGE 1/10: capture mandatory Gemini global pose (10 valid top-surface samples)"
  "${PROJECT_DIR}/scripts/capture_d435i_apriltag_pose.py" \
    --tag-frame gemini_apriltag_0 \
    --samples 10 \
    --timeout-s 30.0 \
    --output "${GLOBAL_POSE}"

  echo "STAGE 2/10: plan Gemini-guided centered D435i observation pose"
  REPORT_GLOBAL="${RUN_DIR}/stage_global_centered_camera_observation.yaml"
  plan_stage "${GLOBAL_POSE}" "${PROJECT_DIR}/config/grasp_recipe_apriltag_global_camera_observation.yaml" "${REPORT_GLOBAL}"
  validate_global_pregrasp_margin "${REPORT_GLOBAL}"

  echo "STAGE 3/10: move to Gemini-guided wrist-camera observation pose"
  execute_open_stage "${REPORT_GLOBAL}" pregrasp I_CONFIRM_ROS_PREGRASP_ONLY 45
fi

echo "STAGE 4/10: D435i must acquire the tag before descent"
"${PROJECT_DIR}/scripts/capture_d435i_apriltag_pose.py" \
  --timeout-s 12.0 \
  --output "${WRIST_POSE}"

echo "STAGE 5/10: plan and move directly to D435i-guided 50 mm approach"
REPORT_50="${RUN_DIR}/stage_wrist_50mm.yaml"
plan_stage "${WRIST_POSE}" "${PROJECT_DIR}/config/grasp_recipe_apriltag_top_approach50.yaml" "${REPORT_50}"
execute_open_stage "${REPORT_50}" approach50 I_CONFIRM_APPROACH_50MM_ONLY 30

echo "STAGE 6/10: refresh D435i if visible, then move to 25 mm"
try_refresh_wrist_pose approach_50mm
REPORT_25="${RUN_DIR}/stage_wrist_25mm.yaml"
plan_stage "${WRIST_POSE}" "${PROJECT_DIR}/config/grasp_recipe_apriltag_top_approach25.yaml" "${REPORT_25}"
execute_open_stage "${REPORT_25}" approach25 I_CONFIRM_APPROACH_25MM_ONLY 30

echo "STAGE 7/10: final D435i refresh attempt and locked-pose plan"
try_refresh_wrist_pose approach_25mm
REPORT_FINAL="${RUN_DIR}/stage_final_grasp_retreat.yaml"
plan_stage "${WRIST_POSE}" "${PROJECT_DIR}/config/grasp_recipe_apriltag_top_approach25.yaml" "${REPORT_FINAL}"

echo "STAGE 8/10: execute final 25 mm approach and close to 48 mm"
"${PROJECT_DIR}/scripts/execute_ros_pregrasp.py" \
  --report "${REPORT_FINAL}" \
  --confirmation I_CONFIRM_FINAL_25MM_AND_CLOSE \
  --mode grasp_close \
  --tolerance-deg 0.5 \
  --timeout-s 30

echo "STAGE 9/10: verify physical object contact"
GRIPPER_FEEDBACK=$("${PROJECT_DIR}/scripts/read_ros_gripper_m.py")
/usr/bin/python3 - "${GRIPPER_FEEDBACK}" <<'PY'
import sys
value = float(sys.argv[1])
print(f"verified_gripper_feedback_m: {value:.4f}")
if not 0.050 <= value <= 0.060:
    raise SystemExit("Gripper feedback does not indicate expected object contact; refusing retreat")
PY

echo "STAGE 10/10: hold grip and execute full retreat"
"${PROJECT_DIR}/scripts/execute_ros_pregrasp.py" \
  --report "${REPORT_FINAL}" \
  --confirmation I_CONFIRM_FULL_RETREAT_WITH_GRIP \
  --mode retreat_hold \
  --tolerance-deg 0.5 \
  --timeout-s 30

echo "GLOBAL_TO_WRIST_APRILTAG_GRASP_RETREAT_COMPLETE"
echo "The object remains gripped. No release command was sent."
