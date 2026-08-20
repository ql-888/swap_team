#!/usr/bin/env bash
set -euo pipefail

# Only terminate processes belonging to this project's robot/vision launchers.
patterns=(
  'start_single_agx_arm.launch.py.*namespace:=piper_x'
  'agx_arm_ctrl_single.*__ns:=/piper_x'
  'robot_state_publisher.*piper_x_with_gripper_description.urdf'
  'orbbec_camera gemini_330_series.launch.py.*camera_name:=gemini336l'
  'gemini_330_series.launch.py'
  'component_container.*__ns:=/gemini336l'
  'realsense2_camera rs_launch.py.*camera_name:=d435i'
  'realsense2_camera_node.*__ns:=/sensors'
  'run_apriltag_d435i_drone.sh'
  'apriltag_node.*gemini336l_drone_apriltag'
  'apriltag_node.*d435i_drone_apriltag'
  'static_transform_publisher.*gemini336l_link'
  'static_transform_publisher.*d435i_link'
)

for pattern in "${patterns[@]}"; do
  pkill -TERM -f "${pattern}" 2>/dev/null || true
done

sleep 2
for pattern in "${patterns[@]}"; do
  pkill -KILL -f "${pattern}" 2>/dev/null || true
done

# Fast DDS shared-memory port locks can survive a crashed camera container.
rm -f /dev/shm/fastrtps_port* /dev/shm/sem.fastrtps_port* 2>/dev/null || true

# Force the local ROS CLI discovery cache to restart after stale participants.
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}" ros2 daemon stop >/dev/null 2>&1 || true
sleep 5

echo "EYE_TO_GRASP_CLEANUP_COMPLETE"
