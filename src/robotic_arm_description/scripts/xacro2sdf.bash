#!/usr/bin/env bash
# This script converts the Panda xacro into the SDF asset stored by `robotic_arm_sim`.

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
PACKAGE_DIR="$(dirname "${SCRIPT_DIR}")"
XACRO_PATH="${PACKAGE_DIR}/urdf/panda/panda.xacro.urdf"
SDF_PATH="${PACKAGE_DIR}/../robotic_arm_sim/panda/model.sdf"
TMP_URDF_PATH="/tmp/panda_tmp.urdf"

# Arguments for xacro
XACRO_ARGS=(
    name:=panda
    gripper:=true
    collision_arm:=true
    collision_gripper:=true
    ros2_control:=true
    ros2_control_plugin:=gz
    ros2_control_command_interface:=effort
    gazebo_preserve_fixed_joint:=false
)

# Remove old SDF file
rm "${SDF_PATH}" 2>/dev/null

# Process xacro into URDF, then convert URDF to SDF and edit the SDF to use relative paths for meshes
xacro "${XACRO_PATH}" "${XACRO_ARGS[@]}" -o "${TMP_URDF_PATH}" &&
gz sdf -p "${TMP_URDF_PATH}" | sed "s/model:\/\/panda\///g" >"${SDF_PATH}" &&
echo "Created new ${SDF_PATH}"

# Remove temporary URDF file
rm "${TMP_URDF_PATH}" 2>/dev/null
