#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)

[[ "${1:-}" == "I_CONFIRM_FULL_DRONE_GRASP_TRANSPORT" ]] || {
  echo "Refusing motion. Required: I_CONFIRM_FULL_DRONE_GRASP_TRANSPORT" >&2; exit 2;
}
[[ "${2:-}" == "I_CONFIRM_OBJECT_AND_TARGET_TAGS_FIXED" ]] || {
  echo "Refusing motion. Required: I_CONFIRM_OBJECT_AND_TARGET_TAGS_FIXED" >&2; exit 2;
}
[[ "${3:-}" == "I_HAVE_CLEARED_GRASP_AND_TRANSPORT_PATH" ]] || {
  echo "Refusing motion. Required: I_HAVE_CLEARED_GRASP_AND_TRANSPORT_PATH" >&2; exit 2;
}

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/source_ros_env.sh"
set -u

topic_info=$(ros2 topic info /perception/gemini336l_transport/standard52h13/detections 2>/dev/null || true)
grep -Fq 'Publisher count: 1' <<< "${topic_info}" || {
  echo "Gemini Standard52h13 transport detector is not running." >&2
  echo "Start scripts/run_apriltag_gemini336l_standard52h13.sh first." >&2
  exit 3
}

echo "STAGE 1/7: capture D435i drone tag and validate complete grasp plan"
"${PROJECT_DIR}/scripts/plan_d435i_drone_apriltag_grasp.sh"

echo "STAGE 2/7: fresh D435i capture and move to 100 mm pregrasp"
"${PROJECT_DIR}/scripts/run_d435i_drone_apriltag_ros_pregrasp.sh" \
  I_CONFIRM_ROS_PREGRASP_ONLY

echo "STAGE 3/7: move to 50 mm inspection position using locked pose"
"${PROJECT_DIR}/scripts/run_d435i_drone_apriltag_ros_approach50.sh" \
  I_CONFIRM_APPROACH_50MM_ONLY

echo "STAGE 4/7: move to 25 mm inspection position using locked pose"
"${PROJECT_DIR}/scripts/run_d435i_drone_apriltag_ros_approach25.sh" \
  I_CONFIRM_APPROACH_25MM_ONLY I_CONFIRM_OBJECT_AND_TAG_NOT_MOVED

echo "STAGE 5/7: final approach and close gripper to 58 mm"
"${PROJECT_DIR}/scripts/run_d435i_drone_locked_pose_grasp_close.sh" \
  I_CONFIRM_FINAL_25MM_AND_CLOSE I_CONFIRM_OBJECT_AND_TAG_NOT_MOVED

echo "STAGE 6/7: retreat while continuously holding 58 mm"
"${PROJECT_DIR}/scripts/run_drone_locked_pose_full_retreat.sh" \
  I_CONFIRM_FULL_RETREAT_WITH_GRIP I_CONFIRM_GRIP_IS_STABLE

echo "STAGE 7-8/8: detect Standard52h13 ID 0, lift 250 mm, then transport above it"
"${PROJECT_DIR}/scripts/run_drone_transport_to_standard52_above.sh" \
  I_CONFIRM_TRANSPORT_WITH_GRIP_TO_STANDARD52 \
  I_CONFIRM_TARGET_STANDARD52H13_ID0_FIXED \
  I_HAVE_CLEARED_THE_TRANSPORT_PATH

echo "FULL_DRONE_GRASP_TRANSPORT_STANDARD52_COMPLETE"
echo "The drone remains held at the target-above pose; no release was commanded."
