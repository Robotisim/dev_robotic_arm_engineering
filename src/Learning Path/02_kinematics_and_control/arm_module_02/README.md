# arm_module_02 — Module 2

This is an `ament_cmake` package containing the C++ implementation for [Module 2](../README.md). Build it with the dependencies using [the course build instructions](../../README.md#setup-and-build); do not build it as an isolated source folder.

Follow these project READMEs in order. Each has complete setup, launch, run, expected-result and shutdown instructions:

- [DH, FK and XYZ motion without MoveIt](../projects/01_fk_dh_xyz/README.md)

The package's `src/` directory holds node responsibilities described in the guides; `launch/` holds Python orchestration. Application logic is C++17. The project baselines use simulation. Consult the module notes for attaching clients to the existing real stack with its existing hard limits and command-derived state.

[All projects and automated README runner](../../PROJECTS.md)

