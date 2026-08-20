#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export PIPER_GRASP_REPORT_FILE="${PROJECT_DIR}/runtime/drone_grasp/grasp_close_report.yaml"
export PIPER_GRASP_GRIPPER_OPEN_M=0.085
export PIPER_GRASP_GRIPPER_CLOSED_M=0.058
export PIPER_GRASP_FEEDBACK_MIN_M=0.058
export PIPER_GRASP_FEEDBACK_MAX_M=0.085
exec "${PROJECT_DIR}/scripts/run_locked_pose_full_retreat.sh" "$@"
