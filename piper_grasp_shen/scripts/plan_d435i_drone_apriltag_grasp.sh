#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export PIPER_GRASP_POSE_FILE="${PROJECT_DIR}/runtime/drone_grasp/d435i_apriltag_object_pose.yaml"
export PIPER_GRASP_REPORT_FILE="${PROJECT_DIR}/runtime/drone_grasp/plan.yaml"
export PIPER_GRASP_RECIPE_FILE="${PROJECT_DIR}/config/grasp_recipe_drone_apriltag_top.yaml"
exec "${PROJECT_DIR}/scripts/plan_d435i_apriltag_grasp.sh" "$@"
