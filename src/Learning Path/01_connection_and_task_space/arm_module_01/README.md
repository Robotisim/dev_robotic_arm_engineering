# arm_module_01 — Module 1

This is an `ament_cmake` package containing the C++ implementation for [Module 1](../README.md). Build it with the dependencies using [the course build instructions](../../README.md#setup-and-build); do not build it as an isolated source folder.

Follow these project READMEs in order. Each has complete setup, launch, run, expected-result and shutdown instructions:

- [Display joint angles and sample the workspace](../projects/01_display_workspace/README.md)
- [Mock controller angle commands](../projects/02_mock_jog/README.md)
- [Gazebo angle commands](../projects/03_gazebo_jog/README.md)

The package's `src/` directory holds node responsibilities described in the guides; `launch/` holds Python orchestration. Application logic is C++17. The project baselines use simulation. Consult the module notes for attaching clients to the existing real stack with its existing hard limits and command-derived state.

[All projects and automated README runner](../../PROJECTS.md)

