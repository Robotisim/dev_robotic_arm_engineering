# README execution report — 2026-10-14

This report records execution of the [17 standalone project guides](../PROJECTS.md), including problems found while following their commands. The older [implementation validation](VALIDATION.md) remains a separate record.

## Accepted results

**All 17 project guides passed their documented baseline checks after repairs.** Final verification used isolated RoboFlex/Panda groups and targeted reruns, not one uninterrupted clean run of the full suite. Twelve accepted runs are in `artifacts/verified-readmes`; five final reruns are in `artifacts/verified-final`. Earlier failed attempts remain part of the record below.

The [portable evidence index](validation/readme/README.md) contains the exact extracted commands, logs and result files; the [machine-readable results](validation/readme/results.tsv) identify the source of each accepted run.

| Order | Project | Result | Verified outcome |
|---|---|---|---|
| 1 | [Module 1 · Project 01 — Display joint angles and sample the workspace](../01_connection_and_task_space/projects/01_display_workspace/README.md) | [PASS](validation/readme/01_connection_and_task_space_01_display_workspace/run.log) | Joint display, invalid-angle rejection and 2,000 workspace samples |
| 2 | [Module 1 · Project 02 — Mock controller angle commands](../01_connection_and_task_space/projects/02_mock_jog/README.md) | [PASS](validation/readme/01_connection_and_task_space_02_mock_jog/run.log) | Mock joint command reached 0.05 rad within 0.005 rad |
| 3 | [Module 1 · Project 03 — Gazebo angle commands](../01_connection_and_task_space/projects/03_gazebo_jog/README.md) | [PASS](validation/readme/01_connection_and_task_space_03_gazebo_jog/run.log) | Gazebo clock advanced and joint command reached tolerance |
| 4 | [Module 2 · Project 01 — DH, FK and XYZ motion without MoveIt](../02_kinematics_and_control/projects/01_fk_dh_xyz/README.md) | [PASS](validation/readme/02_kinematics_and_control_01_fk_dh_xyz/run.log) | DH/FK checks, XYZ execution (0.000079 m residual), unreachable goal rejected |
| 5 | [Module 3 · Project 01 — MoveIt recipes in mock controllers](../03_moveit_and_pick_place/projects/01_moveit_recipe/README.md) | [PASS](validation/readme/03_moveit_and_pick_place_01_moveit_recipe/run.log) | Three-step MoveIt recipe preview/execution; out-of-range goal rejected |
| 6 | [Module 3 · Project 02 — Save poses and run a taught pick/place sequence](../03_moveit_and_pick_place/projects/02_taught_poses/README.md) | [PASS](validation/readme/03_moveit_and_pick_place_02_taught_poses/run.log) | Four persisted poses; 11-step taught recipe; unknown pose rejected |
| 7 | [Module 3 · Project 03 — MoveIt recipes in Gazebo](../03_moveit_and_pick_place/projects/03_gazebo_recipe/README.md) | [PASS](validation/readme/03_moveit_and_pick_place_03_gazebo_recipe/run.log) | MoveIt recipe executed with Gazebo controllers |
| 8 | [Module 4 · Project 01 — Find a coloured cube with depth and pick it](../04_roboflex_depth_perception/projects/01_depth_pick/README.md) | [PASS](validation/readme/04_roboflex_depth_perception_01_depth_pick/run.log) | Depth-derived pregrasp preview and one verified physical lift/place |
| 9 | [Module 5 · Project 01 — Sort three coloured cubes](../05_panda_depth_manipulation/projects/01_sort/README.md) | [PASS](validation/readme/05_panda_depth_manipulation_01_sort/run.log) | Three of three colour-sorting transfers verified |
| 10 | [Module 5 · Project 02 — Stack three cubes](../05_panda_depth_manipulation/projects/02_stack/README.md) | [PASS](validation/readme/05_panda_depth_manipulation_02_stack/run.log) | Three of three stacking transfers verified |
| 11 | [Module 5 · Project 03 — Compare 0°, 45° and 90° grasps](../05_panda_depth_manipulation/projects/03_grasp_yaw/README.md) | [PASS](validation/readme/05_panda_depth_manipulation_03_grasp_yaw/run.log) | Three of three transfers at 0°, 45° and 90° verified |
| 12 | [Module 6 · Project 01 — Ground a language command using rules](../06_language_guided_tasks/projects/01_rules/README.md) | [PASS](validation/readme/06_language_guided_tasks_01_rules/run.log) | Red/left task verified; ambiguous and unsupported colours rejected |
| 13 | [Module 6 · Project 02 — Use a small local LLM for task intent](../06_language_guided_tasks/projects/02_local_llm/README.md) | [PASS](validation/readme/06_language_guided_tasks_02_local_llm/run.log) | Local LLM interpreted blue/right and completed physical transfer |
| 14 | [Module 6 · Project 03 — Use the simulator image with a local VLM](../06_language_guided_tasks/projects/03_image_vlm/README.md) | [PASS](validation/readme/06_language_guided_tasks_03_image_vlm/run.log) | Image VLM interpreted green/right and completed physical transfer |
| 15 | [Module 7 · Project 01 — Record two successful expert demonstrations](../07_demonstrations_and_vla/projects/01_record_demonstrations/README.md) | [PASS](validation/readme/07_demonstrations_and_vla_01_record_demonstrations/run.log) | Two successful expert episodes (142 and 137 samples), then native training |
| 16 | [Module 7 · Project 02 — Train and evaluate the C++ policy](../07_demonstrations_and_vla/projects/02_native_training/README.md) | [PASS](validation/readme/07_demonstrations_and_vla_02_native_training/run.log) | 100-epoch native training, held-out evaluation and checkpoint self-test |
| 17 | [Module 7 · Project 03 — Preview and dispatch a bounded learned policy](../07_demonstrations_and_vla/projects/03_policy_rollout/README.md) | [PASS](validation/readme/07_demonstrations_and_vla_03_policy_rollout/run.log) | 50 preview predictions and five dispatched steps; no completed pick expected |

The two newly recorded expert episodes contain **279 labeled samples** in total. Every sample has 206 finite input features, eight finite action values and an existing image; measured future-action horizons were 0.200–0.348 seconds. Training on these episodes produced normalized MSE 0.0131877 on training data and 0.0434965 on the held-out episode. The separate bundled-data training project produced 0.0105605 / 0.048335. Two fixed-scene episodes establish a working data/training pipeline, not generalization.

The policy rollout reported mean native inference time of 0.181443 ms for 50 preview steps and 0.185486 ms for five dispatched steps. Both CSVs correctly report `manipulation_success=0`: the cube was not picked in this short rollout. This project passes its bounded inference/dispatch checks, not a full learned-manipulation benchmark.

## Method and environment

`tools/run_project.sh` extracts each guide's marked setup, launch and run Bash blocks. It starts that project's backend, executes the README's commands and assertions, then stops the process group it created. `tools/run_all_projects.sh` runs the guides in teaching order and retains every exit status. The commands are read from the documentation; there is no second copy of the exercise implementation in the runner.

Environment: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, C++17, native Eigen/OpenCV/PCL libraries. RoboFlex tests use domain 81; Panda tests use domain 82; both use localhost discovery. The existing real hardware process was left running in its own domain and was not commanded by these tests.

The nine course packages and three existing robot dependencies built successfully. `rosdep check` reports all declared system dependencies satisfied. All three CTests pass: core geometry, 500 executor startup/unwind cycles, and native-policy learning/checkpoint round-trip. All 109 Bash fences parse, project links resolve, and launch syntax plus package/world XML checks pass. Python is used for ROS launch orchestration; application nodes, recording, training and inference remain C++.

Language tests use the installed Ollama 0.30.7 service with `qwen2.5:0.5b` (model ID `a8b0c5157701`) and vision-capable `gemma4:12b` (`4eb23ef187e2`). These model weights are external prerequisites, not bundled course files.

## Problems found and changes made

- Initial topic queries could precede ROS discovery. Readiness checks now retry, use reliable transient-local subscriptions for retained messages, and disable CLI array truncation when verifying all 2,000 workspace points.
- Background shells could inherit an ignored SIGINT. The runner and background labs now restore that signal and track only their own process groups. Cleanup allows 20 seconds after SIGINT and five after SIGTERM, then kills remaining members of that owned group so a simulator cannot survive its launch parent.
- ROS log messages were absent from files produced by stdout-only `tee`. Project commands now capture stderr as well, so interpretation assertions inspect the actual logged JSON.
- An unknown taught pose produced YAML's generic `bad conversion`. The C++ server now explicitly rejects an unknown name or a saved value that is not a joint-angle list.
- A recorder readiness pipeline closed `ros2 service list` early with `grep -q`, causing a broken pipe under `pipefail`. The guide now saves the complete service list before checking it and waits for the new recorder's startup message.
- A recording attempt produced images but zero labeled samples while ROS simulation time was not advancing. Bridges now map the world-qualified Gazebo clock to ROS `/clock`; helpers require advancing time, and the C++ recorder rejects sensor/controller stamps ahead of its clock. Gazebo may omit its global clock when another publisher is detected, as described in the [official bridge documentation](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md).
- Repeated cold starts exposed motion requests arriving before controller activation and stale node-name discovery during parameter setting. The helpers now wait for active arm/hand controllers and address the scene manager's parameter service directly. `ros2controlcli` is declared as a dependency.
- Policy launch now accepts an explicit CSV report path. The policy guide clears its previous reports before each attempt so a failed child launch cannot pass by reusing old results.
- Stalls recurred during manipulation even after clock checks passed. The shared helper now uses one executor thread for short, mutually exclusive callbacks; long operations remain on worker/main threads. The motion server retains its planning-scene query client and removes timed-out pending requests. This reduces callback scheduling and entity-lifetime complexity; the exact underlying cause of the earlier stalls was not conclusively established.
- A rapid restart acknowledged entity creation but never started a new controller manager. Panda now uses a unique Gazebo transport partition per launch, preventing requests from reaching a retiring world's services. Controller readiness remains mandatory; startup failures are not counted as successful project runs.
- An invalid instruction could unwind before its executor thread started. An early `cancel()` could then be overwritten by `spin()` starting, leaving the destructor's join blocked. The spinner now waits for the executor's running flag before returning from construction. A dedicated C++ test covers 500 immediate start/unwind cycles under a 15-second deadline.

## Retained failure evidence and limits

Earlier grasp-yaw and stacking attempts stalled between completed controller actions. The execution code was revised as described above and the project guides were rerun. Those failed attempts must remain in reliability statistics; later passing baselines do not prove that every race or randomized-scene failure has been eliminated.

Exploratory runs and failures remain under `Learning Path/artifacts/readme-run-ECy3Vy`, `readme-run-lJLuYU`, `readme-run-z7UHjx`, `readme-reruns`, `clock-verified` and the failed depth/rules attempts in `verified-readmes`. These directories are ignored by Git. Failed recording directories were not silently reused as successful training data.

Hardware attachment, firmware uploading/arming, interactive RViz assessment and randomized robustness trials are not part of these simulation baselines. RoboFlex's real motors still have no measured joint feedback. The saved-pose mock recipe tests sequencing, not a physical RoboFlex grasp.

The learning baseline covers dataset collection, native training, 50 preview predictions and five bounded dispatched actions. A successful short policy loop does not establish a successful pick. The native educational MLP is not pretrained SmolVLA/OpenVLA; the CSV's physical manipulation result must be read separately from process completion.
