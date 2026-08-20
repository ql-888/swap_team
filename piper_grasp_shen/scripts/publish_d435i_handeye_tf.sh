#!/usr/bin/env bash
set -eo pipefail

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/source_ros_env.sh"
set -u

# The measured calibration is flange_link_T_d435i_color_optical_frame.  The
# RealSense driver already owns the tree
# d435i_link -> d435i_color_frame -> d435i_color_optical_frame, so this script
# publishes the equivalent flange_link_T_d435i_link transform.  Publishing a
# second parent directly to d435i_color_optical_frame would split the TF tree.
# This node only publishes a static TF; it does not open CAN or command the arm.
exec ros2 run tf2_ros static_transform_publisher \
  --x 0.03418093323949302 \
  --y -0.05293696381041548 \
  --z 0.11837043884252953 \
  --qx 0.6652212848717037 \
  --qy -0.25298183558053433 \
  --qz 0.6541502585702431 \
  --qw 0.256063023946112 \
  --frame-id piper_x/flange_link \
  --child-frame-id d435i_link
