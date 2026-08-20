#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
RUN_DIR="${PROJECT_DIR}/runtime/to_platform"
TARGET_POSE="${RUN_DIR}/standard52h13_platform_target.yaml"
REPORT="${RUN_DIR}/platform_preplacement_debug_report.yaml"
DESCENT_REPORT="${RUN_DIR}/platform_descent_debug_report.yaml"
DESCENT_10MM_REPORT="${RUN_DIR}/platform_descent_10mm_debug_report.yaml"
ACTION="${1:-}"
CONFIRMATION="${2:-}"
EXEC_STAGE="${ACTION}"

usage() {
  echo "Usage: $0 plan|lift|transport|align|plan-descend|descend|plan-descend-10mm|descend-10mm [confirmation]" >&2
}

case "${ACTION}" in
  plan|plan-descend|plan-descend-10mm) ;;
  lift)
    [[ "${CONFIRMATION}" == "I_CONFIRM_PLATFORM_SAFE_LIFT_ONLY" ]] || {
      echo "Refusing motion. Required: I_CONFIRM_PLATFORM_SAFE_LIFT_ONLY" >&2
      exit 2
    }
    ;;
  transport)
    [[ "${CONFIRMATION}" == "I_CONFIRM_PLATFORM_TRANSPORT_POSITION_ONLY" ]] || {
      echo "Refusing motion. Required: I_CONFIRM_PLATFORM_TRANSPORT_POSITION_ONLY" >&2
      exit 2
    }
    ;;
  align)
    [[ "${CONFIRMATION}" == "I_CONFIRM_PLATFORM_TILTED_ALIGN_ONLY" ]] || {
      echo "Refusing motion. Required: I_CONFIRM_PLATFORM_TILTED_ALIGN_ONLY" >&2
      exit 2
    }
    ;;
  descend)
    [[ "${CONFIRMATION}" == "I_CONFIRM_PLATFORM_DESCEND_100MM_ONLY" ]] || {
      echo "Refusing motion. Required: I_CONFIRM_PLATFORM_DESCEND_100MM_ONLY" >&2
      exit 2
    }
    REPORT="${DESCENT_REPORT}"
    ;;
  descend-10mm)
    [[ "${CONFIRMATION}" == "I_CONFIRM_PLATFORM_DESCEND_10MM_ONLY" ]] || {
      echo "Refusing motion. Required: I_CONFIRM_PLATFORM_DESCEND_10MM_ONLY" >&2
      exit 2
    }
    REPORT="${DESCENT_10MM_REPORT}"
    EXEC_STAGE="descend10"
    ;;
  *) usage; exit 2 ;;
esac

source "${PROJECT_DIR}/scripts/source_ros_env.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
mkdir -p "${RUN_DIR}"

if [[ "${ACTION}" == "plan-descend" || "${ACTION}" == "plan-descend-10mm" ]]; then
  ros2 node list 2>/dev/null | grep -Fxq '/piper_x/agx_arm_ctrl_single_node' || {
    echo "Piper control node is not running." >&2
    exit 3
  }
  gripper_feedback=$("${PROJECT_DIR}/scripts/read_ros_gripper_m.py")
  GRIPPER_FEEDBACK="${gripper_feedback}" python3 -c \
    'import os; value=float(os.environ["GRIPPER_FEEDBACK"]); assert 0.058 <= value <= 0.085, f"held-drone gripper feedback out of range: {value:.4f} m"' || {
      echo "Refusing descent planning: gripper feedback does not indicate a held drone." >&2
      exit 4
    }
  mapfile -t Q < <("${PROJECT_DIR}/scripts/read_ros_joint_degrees.py")
  [[ ${#Q[@]} -eq 6 ]] || { echo "Failed to read six joints." >&2; exit 4; }
  descent_m=0.100
  descent_stage=descend
  descent_report="${DESCENT_REPORT}"
  if [[ "${ACTION}" == "plan-descend-10mm" ]]; then
    descent_m=0.010
    descent_stage=descend10
    descent_report="${DESCENT_10MM_REPORT}"
  fi
  env -u PYTHONPATH PYTHONNOUSERSITE=1 "${PROJECT_DIR}/.conda/bin/python" \
    "${PROJECT_DIR}/scripts/to_platform/plan_platform_descent.py" \
    --current-q-deg "${Q[@]}" \
    --descent-m "${descent_m}" \
    --stage-name "${descent_stage}" \
    --report "${descent_report}"
  echo "DESCENT_PLAN_ONLY_COMPLETE"
  echo "No robot or gripper command was published."
  exit 0
fi

if [[ "${ACTION}" == "plan" ]]; then
  topic_info=$(ros2 topic info \
    /perception/gemini336l_transport/standard52h13/detections 2>/dev/null || true)
  grep -Eq 'Publisher count: [1-9]' <<<"${topic_info}" || {
    echo "Gemini Standard52h13 platform detector is not running." >&2
    echo "Start scripts/run_apriltag_gemini336l_standard52h13.sh first." >&2
    exit 3
  }

  "${PROJECT_DIR}/scripts/to_platform/capture_platform_target.py" \
    --output "${TARGET_POSE}"
  mapfile -t Q < <("${PROJECT_DIR}/scripts/read_ros_joint_degrees.py")
  [[ ${#Q[@]} -eq 6 ]] || { echo "Failed to read six joints." >&2; exit 4; }

  env -u PYTHONPATH PYTHONNOUSERSITE=1 "${PROJECT_DIR}/.conda/bin/python" \
    "${PROJECT_DIR}/scripts/to_platform/plan_platform_above.py" \
    --target-pose "${TARGET_POSE}" \
    --current-q-deg "${Q[@]}" \
    --platform-height-m 0.170 \
    --object-height-above-tag-m 0.200 \
    --lift-waypoint-m 0.100 \
    --transport-tilt-deg 30.0 \
    --alignment-tilt-deg 20.0 \
    --report "${REPORT}"
  echo "PLAN_ONLY_COMPLETE"
  echo "No robot or gripper command was published."
  exit 0
fi

ros2 node list 2>/dev/null | grep -Fxq '/piper_x/agx_arm_ctrl_single_node' || {
  echo "Piper control node is not running." >&2
  exit 3
}
[[ -f "${REPORT}" ]] || {
  echo "Missing platform plan: ${REPORT}; run the plan action first." >&2
  exit 3
}

report_age_s=$(REPORT_PATH="${REPORT}" python3 -c \
  'import os,time,yaml; d=yaml.safe_load(open(os.environ["REPORT_PATH"], encoding="utf-8")); print(time.time()-float(d["created_at_unix_s"]))')
REPORT_AGE_S="${report_age_s}" python3 -c \
  'import os; age=float(os.environ["REPORT_AGE_S"]); assert 0.0 <= age <= 1800.0, f"platform plan is stale: {age:.1f} s"' || {
    echo "Refusing motion: platform plan is stale; run plan again." >&2
    exit 3
  }

gripper_feedback=$("${PROJECT_DIR}/scripts/read_ros_gripper_m.py")
GRIPPER_FEEDBACK="${gripper_feedback}" python3 -c \
  'import os; value=float(os.environ["GRIPPER_FEEDBACK"]); assert 0.058 <= value <= 0.085, f"held-drone gripper feedback out of range: {value:.4f} m"' || {
    echo "Refusing motion: gripper feedback does not indicate a held drone." >&2
    exit 4
  }
echo "held_gripper_feedback_m: ${gripper_feedback}"

"${PROJECT_DIR}/scripts/to_platform/execute_platform_stage.py" \
  --report "${REPORT}" \
  --stage "${EXEC_STAGE}" \
  --confirmation "${CONFIRMATION}"
