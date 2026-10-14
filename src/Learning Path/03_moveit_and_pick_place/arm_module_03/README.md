# arm_module_03 — Module 3

This is an `ament_cmake` package containing the C++ implementation for [Module 3](../README.md). Build it with the dependencies using [the course build instructions](../../README.md#setup-and-build); do not build it as an isolated source folder.

Follow these project READMEs in order. Each has complete setup, launch, run, expected-result and shutdown instructions:

- [MoveIt recipes in mock controllers](../projects/01_moveit_recipe/README.md)
- [Save poses and run a taught pick/place sequence](../projects/02_taught_poses/README.md)
- [MoveIt recipes in Gazebo](../projects/03_gazebo_recipe/README.md)

The package's `src/` directory holds node responsibilities described in the guides; `launch/` holds Python orchestration. Application logic is C++17. The project baselines use simulation. Consult the module notes for attaching clients to the existing real stack with its existing hard limits and command-derived state.

[All projects and automated README runner](../../PROJECTS.md)

