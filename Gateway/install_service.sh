#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${PROJECT_DIR}/build"

cmake -S "${PROJECT_DIR}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release
cmake --build "${BUILD_DIR}" -j2
ctest --test-dir "${BUILD_DIR}" --output-on-failure

sudo install -m 0755 "${BUILD_DIR}/can_gateway" /usr/local/bin/can_gateway
sudo install -m 0644 "${PROJECT_DIR}/can-gateway.service" \
  /etc/systemd/system/can-gateway.service
sudo systemctl daemon-reload
sudo systemctl enable --now can-gateway.service

echo "CAN-Ethernet Gateway service installed."
sudo systemctl --no-pager --full status can-gateway.service

