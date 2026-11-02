#!/usr/bin/env bash
set -euo pipefail

CAN_INTERFACE="${1:-can0}"

sudo ip link set "${CAN_INTERFACE}" down 2>/dev/null || true
sudo ip link set "${CAN_INTERFACE}" type can bitrate 500000 restart-ms 100
sudo ip link set "${CAN_INTERFACE}" up

echo "${CAN_INTERFACE} configured at 500 kbit/s"
ip -details -statistics link show "${CAN_INTERFACE}"

