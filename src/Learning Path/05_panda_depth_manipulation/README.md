# Module 5 — Panda sorting, stacking and grasp experiments

## Runnable projects

Start with these standalone guides; the sections below explain concepts and additional exercises.

- [Sort three coloured cubes](projects/01_sort/README.md)
- [Stack three cubes](projects/02_stack/README.md)
- [Compare 0°, 45° and 90° grasps](projects/03_grasp_yaw/README.md)

Reuse the depth pipeline to implement multi-step projects. Compare physical outcomes across repeated scenes and grasp orientations.

## Features and topics

Seven-axis Panda redundancy, TCP versus hand frame, scene geometry, planning around clutter, top-down grasp yaw, gripper opening, task state, repeated experiments, failure reports and resettable scenes.

## Read in sequence

1. `panda_learning_sim/worlds/course.sdf` — table, camera, cubes, masses, friction and lighting.
2. `panda_learning_sim/launch/workcell.launch.py` — upstream Panda geometry, fixed base, TCP, controllers and MoveIt.
3. `panda_learning_sim/config/controllers.yaml` and `bridge.yaml` — ROS/Gazebo interface contracts.
4. `panda_learning_sim/src/scene_manager.cpp` — native Gazebo pose subscription, ROS evaluator output and reset service.
5. `panda_learning_sim/src/project_runner.cpp` — observe → select → pick → place → evaluate loop.

Panda finger targets use a small margin inside their 0–0.04 m hard limits (0.001–0.038 m) to avoid sticking at exact DART joint stops. The model limits themselves remain unchanged.

The evaluator subscribes directly to Gazebo transport because the standard Pose_V→TFMessage bridge does not preserve the object names in this world. `/learning/sim_poses` remains a diagnostic bridge; task outcomes use `/learning/ground_truth` from the C++ scene manager.

## Lab 1 — Reuse or start the stack

If Module 4 is still running, keep it. Otherwise:

```bash
ros2 launch arm_module_04 module_05.launch.py headless:=true rviz:=false
```

The bare `workcell.launch.py` starts simulation without perception; `module_05.launch.py` includes the complete Module 4 stack. A project runs as a separate C++ client.

Each Panda launch assigns a fresh Gazebo transport partition and prints its name. This keeps a new world from contacting a previous world's entity-creation service during shutdown. ROS topics remain in the selected ROS domain. For external Gazebo CLI inspection, use the printed value as `GZ_PARTITION`; the bare workcell launch also accepts an explicit `partition` argument.

## Project 1 — Sort three colours

```bash
ros2 run panda_learning_sim project_runner --ros-args \
  -p use_sim_time:=true -p project:=sort -p execute:=true \
  -p trials:=1 -p report:=/tmp/panda_sort.csv
```

Pick red, green and blue in order and place them at separate positions along the sorting row. The perception target is refreshed before each pick. Read the CSV when a task fails; the process returns a nonzero status if all three transfers did not succeed.

## Project 2 — Stack three cubes

Reset when idle, then:

```bash
ros2 service call /learning/reset_scene std_srvs/srv/Trigger '{}'
ros2 run panda_learning_sim project_runner --ros-args \
  -p use_sim_time:=true -p project:=stack -p execute:=true \
  -p report:=/tmp/panda_stack.csv
```

The destination is `(0.60,0.25)` with heights increasing by 4 cm. This project adds support contact and placement-height reasoning. The existing stack remains a collision object for transfer planning; only the final contact is permitted during placement. Grasping uses the top surface and known cube assumptions, so stack stability and scene updates should be inspected.

## Project 3 — Compare grasp yaw

```bash
ros2 service call /learning/reset_scene std_srvs/srv/Trigger '{}'
ros2 run panda_learning_sim project_runner --ros-args \
  -p use_sim_time:=true -p project:=grasps -p execute:=true \
  -p trials:=1 -p report:=/tmp/panda_grasps.csv
```

Compare 0°, 45° and 90° top grasps across the three colours. For an axis-aligned cube, the jaw span is `side × (|cos(yaw)| + |sin(yaw)|)`; the C++ server uses this to set the closing target and rejects a grasp wider than the usable opening. The standalone project guide uses one fixed-scene trial. For a separate robustness experiment, set `randomize true` on the scene manager and increase `trials` to 3; later trials reset through a seeded random sequence. A 45° grasp on a cube may fail because jaw width and corner contact differ; a failed result is useful experiment data. This is a comparison of top-down grasp yaw, not an implementation of arbitrary side grasps or a learned grasp proposal network.

## Completion evidence

Produce the three project CSV files. Report task completions separately from full-project success, identify the stage of failures, and compare the same scene/seed where possible. A planner success, gripper completion and physically stable placed cube are three different measurements.

For a repeatable baseline, restore the original cube positions instead of randomizing:

```bash
ros2 param set /scene_manager randomize false
ros2 service call /learning/reset_scene std_srvs/srv/Trigger '{}'
```

Set `randomize true` to resume the seeded random sequence. Return the arm to `ready` with the Module 3 client before resetting if it is near an object.
