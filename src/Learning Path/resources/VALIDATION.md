# Validation record — 2026-10-14

These checks ran in this workspace on Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic. Simulations used isolated ROS domains 81/82; language-only tests used 83. No real motor commands were sent and existing hardware hard limits were not changed by this work.

Installed versions:

- `ros-jazzy-moveit-ros-planning-interface`: 2.12.4-1noble.20260903.100014
- `ros-jazzy-gz-ros2-control`: 1.2.20-1noble.20260905.083607
- `ros-jazzy-ros-gz-sim`: 1.0.24-1noble.20260905.085151

## Completed checks

| Area | Evidence / result |
|---|---|
| Build | All nine course packages built with `ament_cmake`, C++17 and Release. `tools/build.sh` also rebuilt the three existing robot dependencies into a separate prefix. |
| Dependencies | `rosdep check` satisfied declared course/robot dependencies with the unused warehouse database skipped. |
| Kinematics | 100 DH/KDL comparisons, 100 nearby IK targets, finite-difference Jacobian, timed-trajectory bounds, unreachable/NaN rejection. Worst DH matrix discrepancy 1.17845e-6. |
| XYZ control | C++ solver executed `(0.048,0.01,0.308)` in mock; position residual 0.000079 m. |
| MoveIt | C++ small-motion recipe executed in mock and in the RoboFlex Gazebo teaching model. Saved-pose/load preview succeeded; out-of-range joint_1 goal rejected. |
| Camera | Three cube top surfaces detected; initial red/blue XY centre error approximately 2 mm relative to simulator truth. |
| Physical pick/place | Two successful red/left demonstration episodes verified lift and placement; 145 and 146 synchronized samples recorded. |
| Sorting | Three of three colour transfers physically verified. See [sort.csv](validation/sort.csv). |
| Stacking | Three of three cube placements physically verified after support-contact and object-pose fixes. See [stack.csv](validation/stack.csv). |
| Grasp comparison | Three of three transfers verified at 0°, 45° and 90°. See [grasps.csv](validation/grasps.csv). |
| Language | A full LLM → depth → MoveIt blue/right pick physically completed. Rules rejected two-colour ambiguity. Local `qwen2.5:0.5b` returned valid structured intent. Image-conditioned `gemma4:12b` returned green/right intent. |
| Native learning | 145 training samples, 146 held-out samples from separate episodes; normalized MSE 0.0105605 train, 0.048335 held out. |
| Native inference | 50 live preview steps; mean model inference approximately 0.124 ms, excluding ROS/controller overhead. |
| Policy dispatch | Five bounded simulated action steps completed with sampled collision checks and command ownership; mean native inference approximately 0.095 ms. No manipulation completion was claimed. |
| Offline checks | Two CTest cases passed: geometry/IK/limits and native learning/serialization. Python launch syntax and package/world XML parsed successfully. |

## Artifacts

- [Numeric DH report](roboflex_dh.txt).
- [Sample demonstration archive](sample_demonstrations.tar.gz): RGB PNGs + JSONL for two successful red/left episodes.
- [Sample C++ checkpoint](sample_policy.json): trained on the first sample episode, held out on the second.
- [Native training output](validation/native_training.txt), [XYZ result](validation/xyz_execution.txt), [RoboFlex Gazebo recipe](validation/roboflex_gazebo_recipe.txt), [five-step policy metrics](validation/policy_five_steps.csv).
- [LLM interpretation](validation/llm.txt), [VLM interpretation](validation/vlm.txt), [language-driven pick](validation/language_pick.txt), and [offline checks](validation/offline_checks.txt).

## Practical limits

- Real hardware attachment/scripts are implemented against the repository's existing interfaces. Physical execution was not tested here. Its motor positions remain unmeasured.
- The early RoboFlex Gazebo model disables gravity because the real servo dynamics have not been identified. Panda manipulation uses gravity, friction and physical contacts.
- Cube geometry, colour vocabulary and camera placement are constrained for the course. General object grasping is not implemented.
- A randomized-scene grasp failed the physical lift check and was excluded from the supplied successful demonstration dataset. Single baseline trials do not establish a robust success rate across random scenes.
- Camera occlusions retain collision geometry for up to 20 idle frames; task-time scene geometry is frozen. This is a teaching approximation, not moving-obstacle tracking.
- The native MLP was trained on only red/left demonstrations. It has not demonstrated a complete learned pick/place; five successful dispatch steps establish interface integration only. It is not pretrained SmolVLA/OpenVLA inference.
- Policy collision checks sample a segment and do not enable learned target-contact behavior. A collision rejection is an expected bounded outcome, not a successful grasp.
- MoveIt may log that no OctoMap sensor plugin is configured. This course deliberately uses explicit segmented collision boxes and a workbench, not an OctoMap updater.

Use `tools/checks.sh` for offline checks and the module READMEs to reproduce runtime projects. Preserve failed trials when calculating success rates; do not report only successful demonstrations as an evaluation set.
