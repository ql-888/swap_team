#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/source_ros_env.sh"
set -u

if ! ros2 node list | grep -Fxq '/piper_x/agx_arm_ctrl_single_node'; then
    echo "ROS Piper control node is not running." >&2
    echo "Start agx_arm_ctrl first; this check itself will not enable or move the arm." >&2
    exit 2
fi

mapfile -t CURRENT_Q_DEG < <("${PROJECT_DIR}/scripts/read_ros_joint_degrees.py")
if [[ ${#CURRENT_Q_DEG[@]} -ne 6 ]]; then
    echo "Failed to read all six Piper joint angles." >&2
    exit 3
fi

"${PROJECT_DIR}/scripts/run.sh" gripper-axis \
  --current-q-deg "${CURRENT_Q_DEG[@]}"
