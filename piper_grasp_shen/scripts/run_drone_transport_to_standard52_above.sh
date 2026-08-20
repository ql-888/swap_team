#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
RUN_DIR="${PROJECT_DIR}/runtime/drone_grasp"
TARGET_POSE="${RUN_DIR}/standard52h13_id0_target.yaml"
LIFT_REPORT="${RUN_DIR}/standard52h13_transport_lift_report.yaml"
TARGET_REPORT="${RUN_DIR}/standard52h13_transport_target_report.yaml"
ROTATE_REPORT="${RUN_DIR}/standard52h13_transport_rotate_report.yaml"
VIA_POSE="${RUN_DIR}/transport_via_pose.yaml"

[[ "${1:-}" == "I_CONFIRM_TRANSPORT_WITH_GRIP_TO_STANDARD52" ]] || {
  echo "Refusing motion. Required: I_CONFIRM_TRANSPORT_WITH_GRIP_TO_STANDARD52" >&2; exit 2;
}
[[ "${2:-}" == "I_CONFIRM_TARGET_STANDARD52H13_ID0_FIXED" ]] || {
  echo "Refusing motion. Required: I_CONFIRM_TARGET_STANDARD52H13_ID0_FIXED" >&2; exit 2;
}
[[ "${3:-}" == "I_HAVE_CLEARED_THE_TRANSPORT_PATH" ]] || {
  echo "Refusing motion. Required: I_HAVE_CLEARED_THE_TRANSPORT_PATH" >&2; exit 2;
}

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/source_ros_env.sh"
set -u

speed=$(ros2 param get /piper_x/agx_arm_ctrl_single_node speed_percent | awk '{print $NF}')
control_enabled=$(ros2 param get /piper_x/agx_arm_ctrl_single_node control_enabled | awk '{print $NF}')
tcp_offset=$(ros2 param get /piper_x/agx_arm_ctrl_single_node tcp_offset | sed -n "s/.*\[\(.*\)\].*/\1/p" | tr -d ' ')
[[ "${speed}" == "5" && "${control_enabled}" == "True" ]] || {
  echo "Refusing motion: Piper ROS control must be enabled at 5 percent." >&2; exit 3;
}
[[ "${tcp_offset}" == "0.0,0.0,0.198,0.0,0.0,0.8412486994612669" ]] || {
  echo "Refusing motion: ROS TCP does not match Z=198 mm / yaw=48.2 deg." >&2; exit 3;
}

gripper=$("${PROJECT_DIR}/scripts/read_ros_gripper_m.py")
/usr/bin/python3 - "${gripper}" <<'PY'
import sys
value = float(sys.argv[1])
print(f"verified_gripper_feedback_m: {value:.4f}")
if not 0.058 <= value <= 0.085:
    raise SystemExit("Gripper feedback does not indicate a held drone")
PY

"${PROJECT_DIR}/scripts/capture_gemini_standard52_target.py" --output "${TARGET_POSE}"
mapfile -t Q < <("${PROJECT_DIR}/scripts/read_ros_joint_degrees.py")
[[ ${#Q[@]} -eq 6 ]] || { echo "Failed to read six joints." >&2; exit 4; }
"${PROJECT_DIR}/scripts/run.sh" --help >/dev/null
echo "STAGE 7/8: lift held drone vertically by 250 mm"
if [[ "${START_AT_STAGE8:-0}" != "1" ]]; then
plan_args=(
  --target-pose "${TARGET_POSE}" --current-q-deg "${Q[@]}" --height-m 0.200 --phase lift --report "${LIFT_REPORT}"
)
if [[ "${USE_SAVED_TRANSPORT_VIA_POSE:-0}" == "1" && -f "${VIA_POSE}" ]]; then
  plan_args+=(--via-pose "${VIA_POSE}")
  echo "via_policy: use saved transport waypoint ${VIA_POSE}"
else
  echo "via_policy: default 250 mm lift waypoint"
fi
env -u PYTHONPATH PYTHONNOUSERSITE=1 "${PROJECT_DIR}/.conda/bin/python" \
  "${PROJECT_DIR}/scripts/plan_standard52_transport.py" \
  "${plan_args[@]}"

"${PROJECT_DIR}/scripts/execute_ros_pregrasp.py" \
  --report "${LIFT_REPORT}" \
  --confirmation I_CONFIRM_TRANSPORT_WITH_GRIP_TO_STANDARD52 \
  --mode transport_hold \
  --gripper-open-m 0.085 \
  --gripper-closed-m 0.058 \
  --tolerance-deg 0.5 \
  --timeout-s 40
else
  echo "STAGE 7/8: already complete; reusing current lifted position"
fi
echo "STAGE 8/8: move from lifted waypoint to target tag above pose"
mapfile -t Q_AFTER_LIFT < <("${PROJECT_DIR}/scripts/read_ros_joint_degrees.py")
[[ ${#Q_AFTER_LIFT[@]} -eq 6 ]] || { echo "Failed to read six joints after lift." >&2; exit 4; }
env -u PYTHONPATH PYTHONNOUSITE=1 "${PROJECT_DIR}/.conda/bin/python" \
  "${PROJECT_DIR}/scripts/plan_standard52_transport.py" \
  --target-pose "${TARGET_POSE}" --current-q-deg "${Q_AFTER_LIFT[@]}" --height-m 0.200 \
  --phase target --report "${TARGET_REPORT}"
"${PROJECT_DIR}/scripts/execute_ros_pregrasp.py" \
  --report "${TARGET_REPORT}" \
  --confirmation I_CONFIRM_TRANSPORT_WITH_GRIP_TO_STANDARD52 \
  --mode transport_hold --gripper-open-m 0.085 --gripper-closed-m 0.058 \
  --tolerance-deg 0.5 --timeout-s 40
echo "DRONE_HELD_100MM_ABOVE_STANDARD52H13_ID0_COMPLETE"
echo "STAGE 9/9: rotate held gripper to align with target tag left-right axis"
mapfile -t Q_AFTER_TARGET < <("${PROJECT_DIR}/scripts/read_ros_joint_degrees.py")
[[ ${#Q_AFTER_TARGET[@]} -eq 6 ]] || { echo "Failed to read six joints before rotation." >&2; exit 4; }
env -u PYTHONPATH PYTHONNOUSITE=1 "${PROJECT_DIR}/.conda/bin/python" \
  "${PROJECT_DIR}/scripts/plan_standard52_transport.py" \
  --target-pose "${TARGET_POSE}" --current-q-deg "${Q_AFTER_TARGET[@]}" --height-m 0.200 \
  --phase rotate --report "${ROTATE_REPORT}"
"${PROJECT_DIR}/scripts/execute_ros_pregrasp.py" \
  --report "${ROTATE_REPORT}" \
  --confirmation I_CONFIRM_TRANSPORT_WITH_GRIP_TO_STANDARD52 \
  --mode transport_hold --gripper-open-m 0.085 --gripper-closed-m 0.058 \
  --tolerance-deg 0.5 --timeout-s 40
echo "DRONE_HELD_TARGET_TAG_ALIGNED_COMPLETE"
echo "The drone remains gripped. No descent or release was commanded."
