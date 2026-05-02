# robotic_arm_sim

Owns Gazebo resources only: worlds, object models, tables, camera scenes, and
the generated Panda SDF model asset.

The package exports `GZ_SIM_RESOURCE_PATH`, `IGN_GAZEBO_RESOURCE_PATH`, and
`SDF_PATH` through environment hooks after the workspace is sourced.
