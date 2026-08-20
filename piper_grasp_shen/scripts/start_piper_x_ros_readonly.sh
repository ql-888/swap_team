#!/usr/bin/env bash
set -eo pipefail

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/source_ros_env.sh"
set -u

exec ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  namespace:=piper_x \
  can_port:=can0 \
  arm_type:=piper_x \
  effector_type:=agx_gripper \
  auto_enable:=false \
  control_enabled:=false \
  tcp_offset:='[0.0,0.0,0.198,0.0,0.0,0.8412486994612669]'
