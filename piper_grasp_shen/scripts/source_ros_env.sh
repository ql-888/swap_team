#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "source_ros_env.sh must be sourced, not executed." >&2
  exit 2
fi

# ROS-generated setup files are not safe to source while Bash nounset is
# enabled.  Preserve the caller's setting, temporarily disable it while the
# overlays load, and restore it before returning to the caller.
PIPER_GRASP_NOUNSET_WAS_ENABLED=false
case $- in
  *u*)
    PIPER_GRASP_NOUNSET_WAS_ENABLED=true
    set +u
    ;;
esac

piper_grasp_restore_nounset() {
  if [[ "${PIPER_GRASP_NOUNSET_WAS_ENABLED}" == "true" ]]; then
    set -u
  fi
  unset PIPER_GRASP_NOUNSET_WAS_ENABLED
  unset -f piper_grasp_restore_nounset
}

ROS_SETUP="/opt/ros/humble/setup.bash"
PIPER_ROS_SETUP="${PIPER_ROS_SETUP:-${HOME}/gy_ws/handeye/install/setup.bash}"
AGX_ARM_ROS_SETUP="${AGX_ARM_ROS_SETUP:-${HOME}/agx_arm_ws/install/setup.bash}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS 2 Humble setup is missing: ${ROS_SETUP}" >&2
  piper_grasp_restore_nounset
  return 1
fi
if [[ ! -f "${PIPER_ROS_SETUP}" ]]; then
  echo "Piper ROS workspace setup is missing: ${PIPER_ROS_SETUP}" >&2
  echo "Set PIPER_ROS_SETUP to the correct install/setup.bash path." >&2
  piper_grasp_restore_nounset
  return 1
fi
if [[ ! -f "${AGX_ARM_ROS_SETUP}" ]]; then
  echo "AgileX arm ROS workspace setup is missing: ${AGX_ARM_ROS_SETUP}" >&2
  echo "Set AGX_ARM_ROS_SETUP to the agx_arm_ws install/setup.bash path." >&2
  piper_grasp_restore_nounset
  return 1
fi

source "${ROS_SETUP}"
source "${PIPER_ROS_SETUP}"
source "${AGX_ARM_ROS_SETUP}"
piper_grasp_restore_nounset
