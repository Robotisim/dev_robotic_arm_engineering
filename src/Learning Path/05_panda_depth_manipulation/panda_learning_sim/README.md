# panda_learning_sim — Module 5

This is an `ament_cmake` package containing the C++ implementation for [Module 5](../README.md). Build it with the dependencies using [the course build instructions](../../README.md#setup-and-build); do not build it as an isolated source folder.

Follow these project READMEs in order. Each has complete setup, launch, run, expected-result and shutdown instructions:

- [Sort three coloured cubes](../projects/01_sort/README.md)
- [Stack three cubes](../projects/02_stack/README.md)
- [Compare 0°, 45° and 90° grasps](../projects/03_grasp_yaw/README.md)

The package's `src/` directory holds node responsibilities described in the guides; `launch/` holds Python orchestration for the bare workcell. The complete perception launch lives in `arm_module_04` to avoid a package dependency cycle. Application logic is C++17. All projects in this package use Panda simulation; they do not start the real robot.

[All projects and automated README runner](../../PROJECTS.md)

