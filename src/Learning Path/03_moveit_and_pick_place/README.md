# Module 3 — MoveIt, saved poses and taught pick/place

## Runnable projects

Start with these standalone guides; the sections below explain concepts and additional exercises.

- [MoveIt recipes in mock controllers](projects/01_moveit_recipe/README.md)
- [Save poses and run a taught pick/place sequence](projects/02_taught_poses/README.md)
- [MoveIt recipes in Gazebo](projects/03_gazebo_recipe/README.md)

Use the repository's working MoveIt configuration from C++. Finish with saved poses and a reusable pick/place recipe for the real robot. This is the last module that targets real hardware.

## Features and concepts

MoveGroupInterface, planning groups, SRDF named states, planning versus execution, hard limits, trajectories, scene collisions, gripper action, pose persistence, task actions, sequential recipes and cancellation.

## Nodes and launch sequence

| Component | Responsibility |
|---|---|
| `arm_module_03/src/motion_server.cpp` | Single task owner; MoveIt plans; checks trajectories; executes arm/gripper; saves/loads poses and recipes |
| `arm_module_03/src/task_client.cpp` | C++ command-line action client with progress and timeout |
| `arm_module_03/launch/module_03.launch.py` | Start model backend + MoveIt + task server, or attach to an existing stack |
| `/learning/task` | Typed action: named, joints, pose, cartesian, gripper, save, load, recipe; Panda adds pick later |
| `/learning/stage` | Current task stage for observation and later demonstration recording |

## Lab 1 — Plan and execute in mock

Stop Module 2 first:

```bash
ros2 launch arm_module_03 module_03.launch.py backend:=mock rviz:=false allow_execution:=true
```

Second terminal:

```bash
ros2 run arm_module_03 task_client --ros-args -p operation:=named -p target:='Arm Up'
ros2 run arm_module_03 task_client --ros-args -p operation:=named -p target:='Arm Up' -p execute:=true
ros2 run arm_module_03 task_client --ros-args -p operation:=joints \
  -p joints:='[0.07,-0.3839724,-0.9250245,-0.3665191]' -p execute:=true
```

Server execution is disabled by default. A motion requires both server `allow_execution:=true` and client `execute:=true`. This makes the same teaching commands usable for plan inspection and deliberate execution. Existing URDF limits and MoveIt controller mappings remain the source of truth.

## Lab 2 — Small scripted motion

```bash
ros2 run arm_module_03 task_client --ros-args -p operation:=recipe \
  -p target:="$COURSE_ROOT/Learning Path/03_moveit_and_pick_place/arm_module_03/config/real_motion_recipe.yaml"
ros2 run arm_module_03 task_client --ros-args -p operation:=recipe \
  -p target:="$COURSE_ROOT/Learning Path/03_moveit_and_pick_place/arm_module_03/config/real_motion_recipe.yaml" \
  -p execute:=true
```

Preview plans each step from the previous planned endpoint. Execution replans each step from the latest state. The example moves around the repository's Arm Up state. Test an out-of-range goal and confirm it fails.

## Lab 3 — Save and load poses

```bash
ros2 run arm_module_03 task_client --ros-args -p operation:=save -p target:=my_pose
ros2 run arm_module_03 task_client --ros-args -p operation:=load -p target:=my_pose
ros2 run arm_module_03 task_client --ros-args -p operation:=load -p target:=my_pose -p execute:=true
```

The server's `pose_store` defaults to `/tmp/roboflex_course_poses.yaml`. Set a persistent absolute path in the launch for teaching work. Saved YAML records robot ID, joint order and named angle vectors. On the real robot those are command-derived angles, not measured teaching-by-hand poses. Move with existing controls, then save the commanded pose.

## Lab 4 — Gripper and a complete taught pick/place recipe

```bash
ros2 run arm_module_03 task_client --ros-args -p operation:=gripper -p joints:='[0.3]'
```

Add `-p execute:=true` to send the gripper trajectory. The canonical allowed range is read from the model. Check which end corresponds to open/closed on your existing mechanical mapping; the supplied recipe uses 0.523333 for open and 0.0 for closed.

Teach four positions with your existing control workflow: `pre_pick`, `pick`, `pre_place`, `place`. At each commanded position run `operation:=save -p target:=NAME`, substituting that name. Keep approach and retreat clear of the workbench. The file `config/taught_pick_place.yaml` implements open → approach → close → wait → lift → transfer → lower → release → retreat using those four stored positions.

```bash
ros2 run arm_module_03 task_client --ros-args -p operation:=recipe \
  -p target:="$COURSE_ROOT/Learning Path/03_moveit_and_pick_place/arm_module_03/config/taught_pick_place.yaml"
```

After teaching and inspecting the preview, add `-p execute:=true` to run it. Unknown pose names are rejected; no physical pick coordinates are invented in the source. Recipe steps support `named`, `saved_pose`, `joints`, `gripper`, and bounded `wait` seconds.

## Lab 5 — Attach to the working real robot

Keep the existing firmware and MoveIt running with their current hard limits. In the same ROS domain, launch only this server:

```bash
ros2 launch arm_module_03 module_03.launch.py backend:=hardware \
  start_backend:=false start_moveit:=false rviz:=false \
  allow_execution:=true pose_store:=/tmp/roboflex_course_poses.yaml
```

Use the C++ client commands above. Do not start duplicate controllers or MoveIt instances. Existing ARM/HOLD/health behavior still belongs to the repository's hardware system. A successful ROS action means the command trajectory completed; use visual observation to assess actual pick/place. No feedback-based grasp verification is claimed for this arm.

## Completion evidence

Show a sequential preview, executed mock recipe, persistent pose store, invalid-angle rejection and a taught recipe with four meaningful poses. Hardware execution is optional for advancing. All subsequent modules use Panda simulation.
