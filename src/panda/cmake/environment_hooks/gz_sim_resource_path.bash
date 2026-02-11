#!/usr/bin/env bash

if [ -n "${AMENT_CURRENT_PREFIX:-}" ]; then
  PKG_SHARE_DIR="${AMENT_CURRENT_PREFIX}/share/panda"
else
  PKG_SHARE_DIR="$(cd "$(dirname "$(dirname "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
fi
ament_prepend_unique_value GZ_SIM_RESOURCE_PATH "${PKG_SHARE_DIR}"
ament_prepend_unique_value GZ_SIM_RESOURCE_PATH "${PKG_SHARE_DIR}/models"
# Backward-compatible export for older tooling that still reads IGN path.
ament_prepend_unique_value IGN_GAZEBO_RESOURCE_PATH "${PKG_SHARE_DIR}"
ament_prepend_unique_value IGN_GAZEBO_RESOURCE_PATH "${PKG_SHARE_DIR}/models"
