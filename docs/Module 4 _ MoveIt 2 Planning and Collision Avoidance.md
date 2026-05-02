# Module 4 Scope

## Module 4 title

**MoveIt 2 Planning, Collision Avoidance, and Planning Scene**

## Core module description

Module 4 teaches students how to use **MoveIt 2** to plan and execute safe robot motions. In Module 3, students manually understood IK and trajectories. In Module 4, they learn how MoveIt automates many of those steps using planning groups, SRDF, kinematics plugins, OMPL planners, planning scenes, collision checking, and controller execution.

The main robot for implementation should be **Panda**, because your current MoveIt teaching launch supports Panda. The toy/stick arm can still be used for simplified explanation, but the full MoveIt workflow should be Panda-first. Your current teaching command is:

```bash
ros2 launch robotic_arm_bringup moveit_robot.launch.py robot:=panda
```

This starts Panda in Gazebo, starts controllers, starts `move_group`, and opens RViz with MoveIt planning.

## Main story of Module 4

In Module 3, we asked:

> If I want the gripper to reach a point, how do I calculate joint angles?

In Module 4, the question becomes:

> If there are obstacles, joint limits, planning constraints, and execution controllers, how do we plan a safe path from start to goal?

So the core flow is:

```text
Goal pose → MoveIt planning request → IK → collision checking → planner → trajectory → controller execution
```

## Module 4 in scope

Module 4 should cover:

* What MoveIt 2 provides
* Why MoveIt is needed after custom IK/trajectory demos
* URDF vs SRDF
* Planning groups
* End-effector groups
* Named robot states
* Disabled collision pairs
* Kinematics plugin
* OMPL planner configuration
* Planning scene
* Collision objects
* Collision checking
* Safety padding
* Start state validity
* Goal pose planning
* Joint-space planning
* Planning constraints
* Controller execution through MoveIt
* Planning benchmark: success rate and planning time
* Common planning failure debugging

Your plan already defines the Module 4 goal as planning and executing collision-free motions with MoveIt 2, using Panda as the primary reference arm and the toy arm only for simplified explanation.

## Module 4 out of scope

Do not make these the focus of Module 4:

* Full pick and place
* Gripper timing
* Object attachment/detachment
* RGB-D perception
* Runtime object detection
* Camera calibration
* Multi-object manipulation
* Deep dynamics
* Advanced industrial planner tuning
* Real hardware execution as a required student task

These can be previewed, but Module 4 should stay focused on **planning and collision avoidance**.

## Module 4 required deliverables

By the end of Module 4, students should have:

* A working `robotic_arm_moveit_config` package
* Panda MoveIt launch working in RViz
* Planning groups loading correctly
* Kinematics plugin configured
* OMPL planner config understood
* Controller manager mapping understood
* A demo that adds at least 2 collision objects
* A demo that plans to at least 3 target poses
* A planning benchmark CSV for 10 trials
* A short report with success rate, mean planning time, and failure reasons

Your current plan asks for a generated MoveIt config, a collision-object demo with at least 2 objects and 3 targets, and a benchmark CSV for 10 trials.

## Module 4 cleanup before recording

Before recording Module 4, fix or clearly mark these issues:

* Docs should say `src/robotic_arm_moveit_config`, not `src/arm_moveit_config`.
* Add a dedicated collision-object demo node/script.
* Add a benchmark script that creates the 10-trial CSV.
* Clean the OMPL group key from `arm:` to the active Panda group names such as `panda_arm` / `panda_arm_hand`.

These cleanup items are already identified in your teaching map.

## Module 4 success criteria

Students should be able to:

* Explain why MoveIt is needed after IK.
* Explain the difference between URDF and SRDF.
* Identify Panda planning groups.
* Launch MoveIt with Panda.
* Plan a motion in RViz.
* Add collision objects.
* Explain why a direct path may fail.
* Explain how collision checking changes planning.
* Execute a planned trajectory.
* Record planning success rate and planning time.
* Debug common planning failures.

## Module 4 final scope decision

Keep Module 4 focused on:

```text
MoveIt config → planning groups → planning scene → collision objects → collision-free planning → execution → benchmark
```

Do not include pick/place as a core part of Module 4. Pick/place belongs in Module 5.
