#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd -- "${SCRIPT_DIR}/../.." && pwd)

if [[ "${1:-}" == "--dry-run" ]]; then
  echo "mode: dry_run"
  echo "stack: robot"
  echo "component: piper_x_control_5pct"
  echo "component: piper_x_robot_state_publisher"
  exit 0
fi

source "${PROJECT_DIR}/scripts/source_ros_env.sh"

if ! ip link show can0 2>/dev/null | grep -q "UP"; then
  echo "can0 is missing or down. Run ./scripts/setup_can.sh can0 1000000 first." >&2
  exit 2
fi

if ros2 node list 2>/dev/null | grep -Eq \
  '^/piper_x/(agx_arm_ctrl_single_node|robot_state_publisher)$'; then
  echo "Robot stack nodes are already running; refusing to create duplicates." >&2
  echo "Use ./scripts/'eye_in _grasp'/status.sh to inspect them." >&2
  exit 3
fi

LOG_DIR="${PROJECT_DIR}/runtime/eye_in_grasp/logs"
mkdir -p "${LOG_DIR}"
STAMP=$(date +%Y%m%d_%H%M%S)
CONTROL_LOG="${LOG_DIR}/${STAMP}_robot_control.log"
RSP_LOG="${LOG_DIR}/${STAMP}_robot_state_publisher.log"
PIDS=()

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  for pid in "${PIDS[@]}"; do
    kill -TERM "${pid}" 2>/dev/null || true
  done
  for pid in "${PIDS[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
  exit "${status}"
}
trap cleanup EXIT INT TERM

"${PROJECT_DIR}/scripts/start_piper_x_ros_control_5pct.sh" \
  >"${CONTROL_LOG}" 2>&1 &
PIDS+=("$!")

ros2 run robot_state_publisher robot_state_publisher \
  "${PROJECT_DIR}/assets/piper_x_description/urdf/piper_x_with_gripper_description.urdf" \
  --ros-args \
  -r __ns:=/piper_x \
  -r joint_states:=feedback/joint_states \
  -p frame_prefix:=piper_x/ \
  >"${RSP_LOG}" 2>&1 &
PIDS+=("$!")

ready=false
for _ in $(seq 1 30); do
  nodes=$(ros2 node list 2>/dev/null || true)
  if grep -Fxq '/piper_x/agx_arm_ctrl_single_node' <<<"${nodes}" && \
     grep -Fxq '/piper_x/robot_state_publisher' <<<"${nodes}"; then
    ready=true
    break
  fi
  for pid in "${PIDS[@]}"; do
    kill -0 "${pid}" 2>/dev/null || {
      echo "A robot-stack process exited during startup." >&2
      tail -n 30 "${CONTROL_LOG}" "${RSP_LOG}" >&2 || true
      exit 4
    }
  done
  sleep 1
done

if [[ "${ready}" != "true" ]]; then
  echo "Robot stack did not become ready within 30 seconds." >&2
  tail -n 30 "${CONTROL_LOG}" "${RSP_LOG}" >&2 || true
  exit 5
fi

echo "ROBOT_STACK_READY"
echo "control_log: ${CONTROL_LOG}"
echo "robot_state_publisher_log: ${RSP_LOG}"
echo "Keep this terminal open. Do not press Ctrl-C while the drone is held."

set +e
wait -n "${PIDS[@]}"
child_status=$?
set -e
echo "A robot-stack process stopped (exit ${child_status}); stopping the stack." >&2
exit 6
