#!/usr/bin/env bash
set -euo pipefail

CAN_NAME=${1:-can0}
BITRATE=${2:-1000000}

if [[ ! "${CAN_NAME}" =~ ^[a-zA-Z0-9_.-]+$ ]]; then
    echo "Invalid CAN interface name: ${CAN_NAME}" >&2
    exit 2
fi
if [[ ! "${BITRATE}" =~ ^[0-9]+$ ]] || (( BITRATE < 10000 || BITRATE > 1000000 )); then
    echo "Bitrate must be an integer between 10000 and 1000000." >&2
    exit 2
fi

sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev
sudo modprobe gs_usb

if [[ ! -e "/sys/class/net/${CAN_NAME}" ]]; then
    echo "${CAN_NAME} does not exist. The USB adapter has not registered as SocketCAN." >&2
    echo "Check lsusb and dmesg; Arduino/CH341 serial devices are not gs_usb CAN adapters." >&2
    exit 1
fi

sudo ip link set "${CAN_NAME}" down
# Some gs_usb adapters reject restart-ms with "Device doesn't support restart
# from Bus Off". Configure only the universally supported bitrate; recovery is
# performed by bringing the interface down and up when this script is rerun.
sudo ip link set "${CAN_NAME}" type can bitrate "${BITRATE}"
sudo ip link set "${CAN_NAME}" txqueuelen 1000
sudo ip link set "${CAN_NAME}" up
ip -details -statistics link show "${CAN_NAME}"
