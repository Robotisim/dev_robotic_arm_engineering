# arm_module_07 — Module 7

This is an `ament_cmake` package containing the C++ implementation for [Module 7](../README.md). Build it with the dependencies using [the course build instructions](../../README.md#setup-and-build); do not build it as an isolated source folder.

Follow these project READMEs in order. Each has complete setup, launch, run, expected-result and shutdown instructions:

- [Record two successful expert demonstrations](../projects/01_record_demonstrations/README.md)
- [Train and evaluate the C++ policy](../projects/02_native_training/README.md)
- [Preview and dispatch a bounded learned policy](../projects/03_policy_rollout/README.md)

The package's `src/` directory holds node responsibilities described in the guides; `launch/` holds Python orchestration. Application logic is C++17. All projects in this package use Panda simulation; they do not start the real robot.

[All projects and automated README runner](../../PROJECTS.md)

