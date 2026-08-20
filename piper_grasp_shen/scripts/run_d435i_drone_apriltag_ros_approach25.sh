#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export PIPER_GRASP_POSE_FILE="${PROJECT_DIR}/runtime/drone_grasp/d435i_apriltag_object_pose.yaml"
export PIPER_GRASP_PREVIOUS_REPORT_FILE="${PROJECT_DIR}/runtime/drone_grasp/approach50_report.yaml"
export PIPER_GRASP_REPORT_FILE="${PROJECT_DIR}/runtime/drone_grasp/approach25_report.yaml"
export PIPER_GRASP_RECIPE_FILE="${PROJECT_DIR}/config/grasp_recipe_drone_apriltag_top_approach25.yaml"
export PIPER_GRASP_GRIPPER_OPEN_M=0.085
export PIPER_GRASP_GRIPPER_CLOSED_M=0.058
exec "${PROJECT_DIR}/scripts/run_d435i_apriltag_ros_approach25.sh" "$@"
