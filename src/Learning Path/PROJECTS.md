# Runnable projects in teaching order

Build once using the [root instructions](README.md#setup-and-build), then follow each project guide from top to bottom. Each guide contains its own terminal setup, backend launch, C++ client commands, assertions, expected output and shutdown procedure. Application code is C++; Python is used only for launch files.

| Order | Project | Runtime |
|---|---|---|
| 1 | [Module 1 — Display joint angles and sample the workspace](01_connection_and_task_space/projects/01_display_workspace/README.md) | RoboFlex display |
| 2 | [Module 1 — Mock controller angle commands](01_connection_and_task_space/projects/02_mock_jog/README.md) | RoboFlex mock |
| 3 | [Module 1 — Gazebo angle commands](01_connection_and_task_space/projects/03_gazebo_jog/README.md) | RoboFlex Gazebo |
| 4 | [Module 2 — DH, FK and XYZ motion without MoveIt](02_kinematics_and_control/projects/01_fk_dh_xyz/README.md) | RoboFlex mock |
| 5 | [Module 3 — MoveIt recipes in mock controllers](03_moveit_and_pick_place/projects/01_moveit_recipe/README.md) | RoboFlex mock |
| 6 | [Module 3 — Save poses and run a taught pick/place sequence](03_moveit_and_pick_place/projects/02_taught_poses/README.md) | RoboFlex mock |
| 7 | [Module 3 — MoveIt recipes in Gazebo](03_moveit_and_pick_place/projects/03_gazebo_recipe/README.md) | RoboFlex Gazebo |
| 8 | [Module 4 — Find a coloured cube with depth and pick it](04_roboflex_depth_perception/projects/01_depth_pick/README.md) | Panda Gazebo |
| 9 | [Module 5 — Sort three coloured cubes](05_panda_depth_manipulation/projects/01_sort/README.md) | Panda Gazebo |
| 10 | [Module 5 — Stack three cubes](05_panda_depth_manipulation/projects/02_stack/README.md) | Panda Gazebo |
| 11 | [Module 5 — Compare 0°, 45° and 90° grasps](05_panda_depth_manipulation/projects/03_grasp_yaw/README.md) | Panda Gazebo |
| 12 | [Module 6 — Ground a language command using rules](06_language_guided_tasks/projects/01_rules/README.md) | Panda Gazebo |
| 13 | [Module 6 — Use a small local LLM for task intent](06_language_guided_tasks/projects/02_local_llm/README.md) | Panda Gazebo |
| 14 | [Module 6 — Use the simulator image with a local VLM](06_language_guided_tasks/projects/03_image_vlm/README.md) | Panda Gazebo |
| 15 | [Module 7 — Record two successful expert demonstrations](07_demonstrations_and_vla/projects/01_record_demonstrations/README.md) | Panda Gazebo |
| 16 | [Module 7 — Train and evaluate the C++ policy](07_demonstrations_and_vla/projects/02_native_training/README.md) | Offline native C++ |
| 17 | [Module 7 — Preview and dispatch a bounded learned policy](07_demonstrations_and_vla/projects/03_policy_rollout/README.md) | Panda Gazebo |

## Run one guide or the entire sequence

Stop existing course launches first. The runner executes the marked Bash blocks from each README itself, so documentation and the verification command sequence are the same.

```bash
cd /home/luqman/repos/RoboFlexv2.1
bash 'Learning Path/tools/run_project.sh' \
  'Learning Path/01_connection_and_task_space/projects/01_display_workspace/README.md'
```

For the complete simulation sequence:

```bash
bash 'Learning Path/tools/run_all_projects.sh'
```

Each project starts its own backend; allow time for Gazebo/MoveIt startup and several physical transfers. Run sequentially because the guides reuse ROS domains 81/82. Panda gets a fresh Gazebo transport partition on each launch. A full run also needs Ollama serving locally with `qwen2.5:0.5b` and vision-capable `gemma4:12b` installed. Missing models are failures with an actionable prerequisite, not skipped passes. The VLM project is optional for readers with limited hardware, but included in the complete verification sequence.

The suite creates a unique `artifacts/readme-run-*` directory with per-project `backend.log`, `run.log`, `commands.bash`, task outputs and `result.tsv`. The top-level `results.tsv` records every attempted project and exit status. A nonzero exit makes the suite fail even if later projects pass.

## Scope of the checks

- Module 1 checks displayed angles, invalid-angle rejection, 2,000 workspace samples and controller-backed motion.
- Module 2 checks DH/core math, live FK, XYZ execution and unreachable-goal rejection.
- Module 3 checks recipe preview/execution, persistent taught poses and invalid requests in mock/Gazebo.
- Modules 4–6 check actual simulated lift/place results, sorting, stacking, grasp yaw and grounded language tasks.
- Module 7 records two successful episodes, trains natively, previews 50 predictions and dispatches five bounded steps. The short rollout is expected to report no completed manipulation.
- Real hardware attachment, interactive RViz inspection, randomized robustness trials and full learned picking are extensions. They are not counted as successful automated baseline runs.

[Latest README execution report](resources/README_RUN_REPORT.md) · [Earlier implementation validation](resources/VALIDATION.md)
