# arm_module_06 — Module 6

This is an `ament_cmake` package containing the C++ implementation for [Module 6](../README.md). Build it with the dependencies using [the course build instructions](../../README.md#setup-and-build); do not build it as an isolated source folder.

Follow these project READMEs in order. Each has complete setup, launch, run, expected-result and shutdown instructions:

- [Ground a language command using rules](../projects/01_rules/README.md)
- [Use a small local LLM for task intent](../projects/02_local_llm/README.md)
- [Use the simulator image with a local VLM](../projects/03_image_vlm/README.md)

The package's `src/` directory holds node responsibilities described in the guides; `launch/` holds Python orchestration. Application logic is C++17. All projects in this package use Panda simulation; they do not start the real robot.

[All projects and automated README runner](../../PROJECTS.md)

