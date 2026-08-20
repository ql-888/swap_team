#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
PYTHON_BIN="${PROJECT_DIR}/.conda/bin/python"

preflight() {
  local stage_scripts=(
    plan_d435i_drone_apriltag_grasp.sh
    run_d435i_drone_apriltag_ros_pregrasp.sh
    run_d435i_drone_apriltag_ros_approach50.sh
    run_d435i_drone_apriltag_ros_approach25.sh
    run_d435i_drone_locked_pose_grasp_close.sh
    run_drone_locked_pose_full_retreat.sh
  )
  local stage_script

  [[ -x "${PYTHON_BIN}" ]] || {
    echo "Project Python environment is missing: ${PYTHON_BIN}" >&2
    return 3
  }
  for stage_script in "${stage_scripts[@]}"; do
    [[ -x "${PROJECT_DIR}/scripts/${stage_script}" ]] || {
      echo "Required stage is missing or not executable: ${stage_script}" >&2
      return 3
    }
  done

  "${PYTHON_BIN}" - "${PROJECT_DIR}" <<'PY'
from pathlib import Path
import sys

import yaml

project = Path(sys.argv[1])
with (project / "config/apriltag_d435i_drone.yaml").open(encoding="utf-8") as stream:
    detector = yaml.safe_load(stream)["/**"]["ros__parameters"]

expected_distances = (0.10, 0.05, 0.025)
recipe_names = (
    "grasp_recipe_drone_apriltag_top.yaml",
    "grasp_recipe_drone_apriltag_top_approach50.yaml",
    "grasp_recipe_drone_apriltag_top_approach25.yaml",
)
recipes = []
for name, expected_distance in zip(recipe_names, expected_distances):
    with (project / "config" / name).open(encoding="utf-8") as stream:
        recipe = yaml.safe_load(stream)
    if float(recipe["pregrasp_distance_m"]) != expected_distance:
        raise SystemExit(f"Unexpected pregrasp distance in {name}")
    recipes.append(recipe)

if detector["family"] != "36h11" or float(detector["size"]) != 0.029:
    raise SystemExit("D435i drone detector must use tag36h11 at 0.029 m")
if detector["tag"]["ids"] != [0] or detector["tag"]["sizes"] != [0.029]:
    raise SystemExit("D435i drone detector must use tag ID 0 at 0.029 m")
for name, recipe in zip(recipe_names, recipes):
    if float(recipe["object_from_tcp"][2][3]) != -0.045:
        raise SystemExit(f"Unexpected TCP depth in {name}")
    if float(recipe["gripper_opening_m"]) != 0.085:
        raise SystemExit(f"Unexpected gripper opening in {name}")
    if float(recipe["gripper_closed_m"]) != 0.058:
        raise SystemExit(f"Unexpected gripper closing in {name}")

print("mode: preflight_only")
print("tag_size_m: 0.029")
print("tcp_below_tag_m: 0.045")
print("gripper_open_m: 0.085")
print("gripper_closed_m: 0.058")
print(
    "stages: capture_plan,pregrasp_100mm,approach_50mm,"
    "approach_25mm,grasp_close,retreat_hold"
)
PY
}

if [[ "${1:-}" == "--preflight" ]]; then
  preflight
  exit 0
fi

[[ "${1:-}" == "I_CONFIRM_FULL_DRONE_GRASP_RETREAT" ]] || {
  echo "Refusing motion. Required: I_CONFIRM_FULL_DRONE_GRASP_RETREAT" >&2
  exit 2
}
[[ "${2:-}" == "I_HAVE_CLEARED_THE_WORKSPACE" ]] || {
  echo "Refusing motion. Required: I_HAVE_CLEARED_THE_WORKSPACE" >&2
  exit 2
}
[[ "${3:-}" == "I_CONFIRM_DRONE_AND_TAG_FIXED" ]] || {
  echo "Refusing motion. Required: I_CONFIRM_DRONE_AND_TAG_FIXED" >&2
  exit 2
}

preflight
source "${PROJECT_DIR}/scripts/source_ros_env.sh"
set -u

DETECTOR_NODE=/perception/d435i/d435i_drone_apriltag
ros2 node list | grep -Fxq "${DETECTOR_NODE}" || {
  echo "D435i drone AprilTag detector is not running." >&2
  echo "Start scripts/run_apriltag_d435i_drone.sh first." >&2
  exit 3
}
detector_family=$(ros2 param get "${DETECTOR_NODE}" family | awk '{print $NF}')
detector_size=$(ros2 param get "${DETECTOR_NODE}" size | awk '{print $NF}')
[[ "${detector_family}" == "36h11" && "${detector_size}" == "0.029" ]] || {
  echo "Refusing motion: running detector is not tag36h11 at 0.029 m." >&2
  exit 4
}

echo "STAGE 1/6: capture 30 D435i samples and validate the drone grasp plan"
"${PROJECT_DIR}/scripts/plan_d435i_drone_apriltag_grasp.sh"

echo "STAGE 2/6: recapture and move to the 100 mm pregrasp"
"${PROJECT_DIR}/scripts/run_d435i_drone_apriltag_ros_pregrasp.sh" \
  I_CONFIRM_ROS_PREGRASP_ONLY

echo "STAGE 3/6: move to the locked-pose 50 mm inspection position"
"${PROJECT_DIR}/scripts/run_d435i_drone_apriltag_ros_approach50.sh" \
  I_CONFIRM_APPROACH_50MM_ONLY

echo "STAGE 4/6: move to the locked-pose 25 mm inspection position"
"${PROJECT_DIR}/scripts/run_d435i_drone_apriltag_ros_approach25.sh" \
  I_CONFIRM_APPROACH_25MM_ONLY I_CONFIRM_OBJECT_AND_TAG_NOT_MOVED

echo "STAGE 5/6: descend to 45 mm below the tag and close to 58 mm"
"${PROJECT_DIR}/scripts/run_d435i_drone_locked_pose_grasp_close.sh" \
  I_CONFIRM_FINAL_25MM_AND_CLOSE I_CONFIRM_OBJECT_AND_TAG_NOT_MOVED

echo "STAGE 6/6: retreat 100 mm while holding the 58 mm command"
"${PROJECT_DIR}/scripts/run_drone_locked_pose_full_retreat.sh" \
  I_CONFIRM_FULL_RETREAT_WITH_GRIP I_CONFIRM_GRIP_IS_STABLE

echo "FULL_D435I_DRONE_GRASP_RETREAT_COMPLETE"
echo "The drone remains held after retreat; no release was commanded."
