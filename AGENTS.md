# AGENTS.md

This repository is a ROS 2 Jazzy teaching workspace for robotic arm engineering.
Use it as a course project first and a software project second. The goal is to
help students understand how robotic arm systems work through runnable code,
launch files, RViz, Gazebo, MoveIt, perception, manipulation, multi-robot task
execution, and LLM/VLM-based tasking.

## Teaching Source Of Truth

There are two documentation layers. Use them in this order.

1. Module scope docs in `docs/`

   These explain what each module is supposed to teach, what is in scope, what is
   out of scope, the success criteria, and what code demonstrations are expected.
   Read these first when deciding what a module should contain.

   Examples:

   ```text
   docs/Module 1 _ ROS2 Workspace, Simulation, and Arm Model.md
   docs/Module 2 _ Joint Control, Calibration, and Forward Kinematics.md
   docs/Module 3 _ Inverse Kinematics, Trajectories, and Dynamics Basics.md
   docs/Module 4 _ MoveIt 2 Planning and Collision Avoidance.md
   docs/Module 5 _ Pick and Place with Fixed Targets.md
   docs/Module 6 _ RGB-D Perception, Calibration, and Scene Mapping.md
   docs/Module 7 _ Multiple robot task execution.md
   docs/Module 8 _ LLM-VLM-Based Robot Action Using Vision.md
   ```

2. Lesson plans in `docs/Lesson Plans/`

   These are for lecture production. Use them after understanding the module
   scope. They break each module into video segments, story questions, screen
   recording demos, animation ideas, and expected student outcomes.

   Examples:

   ```text
   docs/Lesson Plans/M1.md
   docs/Lesson Plans/M2.md
   docs/Lesson Plans/M3.md
   docs/Lesson Plans/M4.md
   docs/Lesson Plans/M5.md
   docs/Lesson Plans/M6.md
   docs/Lesson Plans/M7.md
   docs/Lesson Plans/M8.md
   ```

When the scope doc and lesson plan disagree, treat the scope doc as the
curriculum authority and update the lesson plan to match it.

## Course Flow

Teach the course as a progressive robot capability stack:

```text
Module 1: The robot exists as a model.
Module 2: The robot joints can be controlled safely.
Module 3: The robot can move through IK and trajectories.
Module 4: The robot can plan safe motions with MoveIt.
Module 5: The robot can pick known objects using fixed targets.
Module 6: The robot can detect objects and pick using perception.
Module 7: Multiple robots can share a scene and coordinate tasks.
Module 8: LLM/VLM systems can use vision to request visual pick-and-place actions.
```

Avoid turning early modules into advanced implementation projects. Each module
should answer one clear question and show that answer through code.

## Package Map

Use the current `robotic_arm_*` packages as the preferred teaching path.
Deprecated wrapper packages exist only for compatibility and should not be used
as the main teaching names.

```text
robotic_arm_description      URDF/Xacro, meshes, robot model definitions
robotic_arm_sim              Gazebo worlds, SDF objects, simulation resources
robotic_arm_control          controller YAML and controller examples
robotic_arm_hardware         placeholder for later real hardware work
robotic_arm_bringup          top-level launch files students run
robotic_arm_moveit_config    MoveIt config for Panda
robotic_arm_motion           IK and trajectory demos
robotic_arm_vision           RGB-D perception, segmentation, calibration checks
robotic_arm_manipulation     pick/place, multi-object execution, VLM scripts
```

Preferred public launch commands:

```bash
ros2 launch robotic_arm_bringup view_robot.launch.py robot:=stick_arm
ros2 launch robotic_arm_bringup view_robot.launch.py robot:=panda
ros2 launch robotic_arm_bringup sim_robot.launch.py robot:=stick_arm
ros2 launch robotic_arm_bringup sim_robot.launch.py robot:=panda
ros2 launch robotic_arm_bringup moveit_robot.launch.py robot:=panda
ros2 launch robotic_arm_bringup pick_place_world.launch.py world:=cubes
ros2 launch robotic_arm_bringup multi_object_world.launch.py
ros2 launch robotic_arm_bringup vla_world.launch.py
ros2 launch robotic_arm_vision segmentation.launch.py
ros2 launch robotic_arm_manipulation pick_place_multi_object.launch.py
```

## How To Teach From This Repo

For each module:

1. Read the module scope doc.
2. Identify the main student question.
3. Identify the package and launch files that demonstrate the answer.
4. Use the lesson plan to decide lecture/video order.
5. Run the demo command before recording.
6. Explain the code path from launch file to node/config/topic.
7. Keep out-of-scope topics as previews only.
8. End with the success criteria from the scope doc.

The teaching rhythm should be:

```text
concept -> file -> launch command -> observed behavior -> debugging idea
```

Do not teach a command as magic. Always connect it to the file that starts it
and the ROS concepts it demonstrates.

## Module Guidance

Module 1 should focus on robot description:

```text
URDF/Xacro -> robot_description -> TF -> RViz -> Gazebo bringup
```

Module 2 should focus on joint control:

```text
controller YAML -> controller manager -> joint command -> joint_states -> TF
```

Module 3 should focus on IK and trajectories:

```text
Cartesian target -> workspace check -> IK -> waypoints -> marker preview -> JointTrajectory
```

Module 4 should focus on MoveIt planning:

```text
SRDF -> planning group -> kinematics plugin -> planning scene -> collision-free trajectory
```

Module 5 should focus on structured manipulation with known targets:

```text
fixed target -> pre-grasp -> grasp -> lift -> place -> retreat
```

Module 6 should focus on perception-driven manipulation:

```text
RGB-D -> segmentation -> depth point -> camera frame -> robot base frame -> pick/place
```

Module 7 should focus on multi-robot task execution:

```text
namespaces -> separate TF trees -> shared world -> task assignment -> coordination
```

Module 8 should focus on LLM/VLM-based visual robot action:

```text
language/image input -> visual target -> RGB-D pose -> pick/place procedure
```

## Lecture Production Notes

Use lesson plans as production scripts, not as curriculum definitions.

Each lesson-plan video should have:

```text
purpose
story question
files to open
command to run
concepts to explain
expected student output
```

Use the naming convention already present in lesson plans:

```text
M<N>_V<NN>_<Title>_[F|S|A]
```

Where:

```text
F = face recording
S = screen recording
A = animation / visual explanation
```

When creating or updating lesson plans, keep them aligned with the repo. If a
demo does not exist in code yet, mark it clearly as missing or create the code
before treating it as a production-ready lecture.

## Code And Documentation Rules

- Prefer `robotic_arm_*` package names in all new docs and commands.
- Keep deprecated package names only in wrapper or migration notes.
- Keep Module 7 as multi-robot task execution, not failure-recovery/evaluation.
- Keep Module 8 as LLM/VLM-based robot action using vision and visual pick/place.
- Do not reintroduce `robotic_arm_evaluation` unless the curriculum explicitly
  returns to a logging/evaluation module.
- Keep docs practical: every important concept should point to a file, launch
  command, topic, or RViz/Gazebo observation.
- If a module deliverable is not implemented, say so directly in the module doc
  or lesson plan.

## Verification Commands

Use these when checking whether the workspace is still coherent:

```bash
source /opt/ros/jazzy/setup.bash
colcon list --base-paths src
colcon build --symlink-install
source install/setup.bash
```

Useful smoke checks:

```bash
ros2 launch robotic_arm_bringup view_robot.launch.py robot:=stick_arm --show-args
ros2 launch robotic_arm_bringup view_robot.launch.py robot:=panda --show-args
ros2 launch robotic_arm_bringup sim_robot.launch.py robot:=stick_arm --show-args
ros2 launch robotic_arm_bringup sim_robot.launch.py robot:=panda --show-args
ros2 launch robotic_arm_bringup moveit_robot.launch.py robot:=panda --show-args
ros2 launch robotic_arm_vision segmentation.launch.py --show-args
ros2 launch robotic_arm_manipulation pick_place_multi_object.launch.py --show-args
```
