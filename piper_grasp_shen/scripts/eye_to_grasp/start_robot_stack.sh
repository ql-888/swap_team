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
  echo "can0 is missing or down. Run scripts/setup_can.sh can0 1000000 first." >&2
  exit 2
fi

if ros2 node list 2>/dev/null | grep -Eq '^/piper_x/(agx_arm_ctrl_single_node|robot_state_publisher)$'; then
  echo "Robot stack nodes are already running; refusing duplicates." >&2
  exit 3
fi

LOG_DIR="${PROJECT_DIR}/runtime/eye_to_grasp/logs"
mkdir -p "${LOG_DIR}"
STAMP=$(date +%Y%m%d_%H%M%S)
PIDS=()
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  for pid in "${PIDS[@]}"; do kill -TERM "${pid}" 2>/dev/null || true; done
  for pid in "${PIDS[@]}"; do wait "${pid}" 2>/dev/null || true; done
  exit "${status}"
}
trap cleanup EXIT INT TERM

"${PROJECT_DIR}/scripts/start_piper_x_ros_control_5pct.sh" \
  >"${LOG_DIR}/${STAMP}_robot_control.log" 2>&1 &
PIDS+=("$!")
ros2 run robot_state_publisher robot_state_publisher \
  "${PROJECT_DIR}/assets/piper_x_description/urdf/piper_x_with_gripper_description.urdf" \
  --ros-args -r __ns:=/piper_x -r joint_states:=feedback/joint_states \
  -p frame_prefix:=piper_x/ \
  >"${LOG_DIR}/${STAMP}_robot_state_publisher.log" 2>&1 &
PIDS+=("$!")

for _ in $(seq 1 40); do
  nodes=$(ros2 node list 2>/dev/null || true)
  if grep -Fxq '/piper_x/agx_arm_ctrl_single_node' <<<"${nodes}" && \
     grep -Fxq '/piper_x/robot_state_publisher' <<<"${nodes}"; then
    echo "ROBOT_STACK_READY"
    echo "Keep this terminal open while the drone is held."
    set +e; wait -n "${PIDS[@]}"; status=$?; set -e
    echo "A robot-stack process stopped (exit ${status})." >&2
    exit 6
  fi
  for pid in "${PIDS[@]}"; do
    kill -0 "${pid}" 2>/dev/null || { echo "Robot-stack startup failed." >&2; exit 4; }
  done
  sleep 1
done
echo "Robot stack did not become ready within 40 seconds." >&2
exit 5
