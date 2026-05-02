# Module 7 Scope

## Module 7 title

**Multiple Robot Task Execution**

## Instructor Flow Design Answers

**Questions Based Instructor Flow Design?**

* Start with the question: what changes when two robots share one ROS 2 scene?
* Teach in the rhythm `namespaces -> separate TF trees -> shared world -> task assignment -> coordination`.
* Keep the module about multi-robot task execution, not evaluation, fleet optimization, or language control.

**What is this module about?**

* This module is about scaling the single-robot stack into a shared scene with multiple robot instances.
* Students learn namespaces, separated topics, separated TF trees, controller isolation, and shared world frames.
* The teaching goal is simple coordination and task assignment, not production fleet management.

**What final project outcome should students achieve?**

* Students should demonstrate two robots in one shared Gazebo scene with no topic or TF collisions.
* Students should assign left-side objects to one robot and right-side objects to the other.
* The current repo has multi-object task assets; a true two-robot namespace launch should be created or marked missing before recording.

**What concepts must the instructor explain?**

* Explain ROS 2 namespaces, per-robot `robot_description`, per-robot controllers, and per-robot `/joint_states`.
* Explain separate TF trees, shared world frames, workspace zones, and topic conflict debugging.
* Explain simple task assignment and why single-robot launch files need careful scaling.

**What exact code/package/files should be shown?**

* Show `robotic_arm_bringup/launch/multi_object_world.launch.py` as the current shared-scene entry point.
* Show `robotic_arm_bringup/launch/sim_robot.launch.py` and `moveit_robot.launch.py` where namespace arguments are reserved.
* Show `robotic_arm_manipulation/src/pick_place_multi_executor.cpp` and `robotic_arm_sim/worlds/multi_object_sequential_env.sdf`.

**What commands should be run?**

* Run `ros2 launch robotic_arm_bringup multi_object_world.launch.py` for the current multi-object scene.
* Run `ros2 launch robotic_arm_manipulation pick_place_multi_object.launch.py` for the current multi-object executor.
* For the intended two-robot lesson, add and run a namespaced multi-robot launch, then inspect `ros2 node list` and `ros2 topic list`.

**What visual/demo result should appear?**

* The current demo should show one Panda working in a shared multi-object scene.
* The intended final demo should show two Panda robots in one Gazebo world with separated TF and topics.
* Objects should be assigned by workspace side so students can see simple coordination rules.

**What mistakes should the instructor avoid?**

* Do not reframe Module 7 as failure recovery, logging evaluation, or warehouse optimization.
* Do not pretend a multi-object single-robot demo is the same as true multi-robot execution.
* Do not skip namespace and TF isolation, because those are the core multi-robot lessons.

**What should the student understand by the end of each lesson?**

* Students should know which namespace, topic, or TF tree belongs to which robot.
* Students should connect shared-world behavior to launch structure and topic names.
* Students should be able to rerun the demo and debug topic collisions or mixed TF frames.

**Package Names And Responsibilities?**

* `robotic_arm_bringup` should own the public multi-robot/shared-scene launch files.
* `robotic_arm_sim` owns the shared Gazebo world and multi-object scene assets.
* `robotic_arm_manipulation` owns task assignment and multi-object or multi-robot executors.

## Core module description

Module 7 teaches students how robotic systems change when more than one robot shares the same simulated world.

The goal is not industrial fleet management. The goal is to demonstrate core ROS 2 multi-robot ideas using clear, visual examples.

## Main story of Module 7

By Module 6, students understand one complete robot pipeline:

```text
description
→ control
→ motion
→ planning
→ manipulation
→ perception
```

Now the question becomes:

> What changes when two robots need to exist in the same ROS 2 system and share a task scene?

## Module 7 in scope

Module 7 should cover:

* launching more than one robot instance
* ROS 2 namespaces
* separate robot descriptions
* separate controller managers
* separate joint states
* separate TF trees
* shared world frame
* shared Gazebo scene
* per-robot RViz visualization
* simple task assignment
* workspace zones
* avoiding topic name conflicts
* how single-robot launch files scale into multi-robot launch files

## Module 7 out of scope

Do not make these the core of Module 7:

* production fleet scheduling
* industrial reliability reporting
* warehouse task optimization
* advanced multi-agent planning
* cloud orchestration
* real robot networking
* distributed SLAM
* model-based language control

These may be future advanced topics.

## Recommended final demo

The strongest teaching demo is:

```text
two Panda robots
one shared Gazebo world
separate namespaces
separate TF trees
one shared table scene
simple object assignment by workspace side
```

Example:

```text
left-side object  -> robot_1
right-side object -> robot_2
```

## Required deliverables

By the end of Module 7, students should have:

* a multi-robot launch file
* two robot instances visible in Gazebo
* namespaced topics for each robot
* separated TF trees
* separated controllers
* a shared world frame
* a simple task assignment demo
* a short explanation of what breaks without namespaces

## Success criteria

Students should be able to:

* explain why namespaces are needed
* explain how two identical robots can coexist
* inspect per-robot topics
* inspect per-robot TF trees
* understand shared world vs robot-local frames
* describe how a task can be assigned to one robot or another
* explain how Module 7 prepares the system for model-assisted visual robot actions in Module 8

## Final scope decision

Keep Module 7 focused on:

```text
multi-robot launch
→ namespaces
→ TF separation
→ shared world
→ simple task assignment
```

Do not turn Module 7 into reliability, logging, or production evaluation.
