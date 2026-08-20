#!/usr/bin/env bash
set -eo pipefail

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/source_ros_env.sh"
set -u

# The measured eye-to-hand calibration is
# piper_x/base_link_T_gemini336l_color_optical_frame.  The Orbbec driver owns
# gemini336l_link -> gemini336l_color_frame -> gemini336l_color_optical_frame,
# so publish the equivalent transform to the camera root instead of assigning
# a second parent directly to its optical frame.
# This static publisher does not open CAN or command the robot.
exec ros2 run tf2_ros static_transform_publisher \
  --x 0.6589229214393953 \
  --y 0.02259948292410663 \
  --z 0.7379360892425801 \
  --qx -0.557640889086826 \
  --qy -0.007091021236534 \
  --qz 0.830020993242771 \
  --qw 0.007176838618831 \
  --frame-id piper_x/base_link \
  --child-frame-id gemini336l_link
