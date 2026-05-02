#!/usr/bin/env bash

if [ -n "${AMENT_CURRENT_PREFIX:-}" ]; then
  PKG_SHARE_DIR="${AMENT_CURRENT_PREFIX}/share/robotic_arm_sim"
else
  PKG_SHARE_DIR="$(cd "$(dirname "$(dirname "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
fi
ament_prepend_unique_value SDF_PATH "${PKG_SHARE_DIR}"
