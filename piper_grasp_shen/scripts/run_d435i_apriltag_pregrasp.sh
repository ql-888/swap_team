#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
POSE_FILE="${PROJECT_DIR}/runtime/d435i_apriltag_object_pose.yaml"
REPORT_FILE="${PROJECT_DIR}/runtime/d435i_apriltag_pregrasp_report.yaml"

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/source_ros_env.sh"
set -u

if [[ "${1:-}" != "I_CONFIRM_PREGRASP_ONLY" ]]; then
    echo "Refusing motion." >&2
    echo "After clearing the workspace, run:" >&2
    echo "  $0 I_CONFIRM_PREGRASP_ONLY" >&2
    exit 2
fi

# Never let this legacy direct-SDK executor compete with the ROS driver for
# the same SocketCAN interface. The ROS node must be the sole CAN owner.
if ros2 node list 2>/dev/null | grep -Fxq '/piper_x/agx_arm_ctrl_single_node'; then
    echo "Refusing direct-SDK motion: /piper_x/agx_arm_ctrl_single_node already owns can0." >&2
    echo "Use the ROS pregrasp executor after it is configured; do not run this command again." >&2
    exit 3
fi

# Capture a fresh, averaged base->tag pose immediately before planning.
"${PROJECT_DIR}/scripts/capture_d435i_apriltag_pose.py" \
  --output "${POSE_FILE}"

# This command opens the gripper and moves only to the pregrasp waypoint.
# It never commands the descent, gripper closing, grasp, or retreat stages.
# The measured offline IK/collision plan takes about 6-8 seconds on this PC.
# This pregrasp-only check therefore allows 15 seconds from capture to motion;
# the tag and object must remain rigidly fixed for the whole operation.
"${PROJECT_DIR}/scripts/run.sh" pregrasp \
  --object-pose "${POSE_FILE}" \
  --recipe "${PROJECT_DIR}/config/grasp_recipe_apriltag_top.yaml" \
  --can can0 \
  --speed 5 \
  --max-step-deg 0.25 \
  --minimum-tcp-z-m 0.02 \
  --maximum-pose-age-s 15.0 \
  --joint-goal-tolerance-deg 0.5 \
  --joint-goal-timeout-s 25.0 \
  --maximum-final-position-error-m 0.002 \
  --maximum-final-orientation-error-deg 1.0 \
  --report "${REPORT_FILE}" \
  --execute \
  --confirmation I_CONFIRM_PREGRASP_ONLY
